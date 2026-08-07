from pathlib import Path
import pickle
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from src.common.coordinates import Pose3D
from src.reconstruction import (
    CameraObservation,
    MeshAligner,
    ReconstructionCoordinator,
    ReconstructionJob,
    ReconstructionStatus,
    SAM3DObjectReconstructor,
    UpstreamSAM3DBackend,
    VehicleDimensions,
    project_vehicle_box,
)
from src.data import NuScenesFrame, VehicleBox
from src.reconstruction.nuscenes_pipeline import (
    NuScenesCarReconstructionPipeline,
    PreparedAssetReconstructionPipeline,
)
from src.reconstruction.sam3d_upstream import _SameFrameDepthCache


def _job(tmp_path: Path, request_id: str = "request-1") -> ReconstructionJob:
    observation = CameraObservation(
        image=np.zeros((20, 30, 3), dtype=np.uint8),
        camera_intrinsic=np.eye(3),
        world_T_camera=Pose3D.identity(),
        timestamp_us=123,
    )
    return ReconstructionJob(
        request_id=request_id,
        track_id="car-1",
        observation=observation,
        mask=np.ones((20, 30), dtype=bool),
        dimensions=VehicleDimensions(4.0, 2.0, 1.5),
        world_T_object=Pose3D.identity(),
        output_path=tmp_path / "raw.glb",
    )


def test_projects_metric_box_and_clips_to_image():
    world_T_object = np.eye(4)
    world_T_object[:3, 3] = (0.0, 0.0, 10.0)
    box = project_vehicle_box(
        world_T_object=world_T_object,
        world_T_camera=np.eye(4),
        intrinsics=np.asarray([[100.0, 0, 50.0], [0, 100.0, 40.0], [0, 0, 1.0]]),
        dimensions=VehicleDimensions(4.0, 2.0, 2.0),
        image_size=(80, 100),
        minimum_area_px=1.0,
    )
    assert box is not None
    np.testing.assert_allclose(box.xyxy, [27.7777778, 28.8888889, 72.2222222, 51.1111111])
    assert box.visible_fraction == pytest.approx(1.0)


def test_projection_rejects_box_behind_or_crossing_camera():
    world_T_object = np.eye(4)
    world_T_object[2, 3] = 0.5
    assert (
        project_vehicle_box(
            world_T_object=world_T_object,
            world_T_camera=np.eye(4),
            intrinsics=np.eye(3),
            dimensions=VehicleDimensions(4.0, 2.0, 2.0),
            image_size=(100, 100),
            minimum_area_px=0,
        )
        is None
    )


def test_sam3d_adapter_loads_backend_once(tmp_path):
    loads = []

    class Backend:
        def reconstruct(self, *, output_path, **_):
            output_path.write_bytes(b"mesh")

    def factory():
        loads.append(True)
        return Backend()

    adapter = SAM3DObjectReconstructor(factory=factory)
    first = adapter.reconstruct(_job(tmp_path))
    second = adapter.reconstruct(_job(tmp_path, "request-2"))
    assert first == second == tmp_path / "raw.glb"
    assert loads == [True]


def test_upstream_sam3d_validates_step_overrides(tmp_path):
    with pytest.raises(ValueError, match="stage1_inference_steps"):
        UpstreamSAM3DBackend(
            repository=tmp_path,
            config_path="pipeline.yaml",
            stage1_inference_steps=0,
        )
    backend = UpstreamSAM3DBackend(
        repository=tmp_path,
        config_path="pipeline.yaml",
        stage1_inference_steps=8,
        stage2_inference_steps=12,
    )
    assert backend.stage1_inference_steps == 8
    assert backend.stage2_inference_steps == 12
    with pytest.raises(ValueError, match="pointmap_cache_size"):
        UpstreamSAM3DBackend(
            repository=tmp_path,
            config_path="pipeline.yaml",
            pointmap_cache_size=-1,
        )


def test_same_frame_depth_cache_reuses_rgb_and_evicts_lru():
    calls = []

    class DepthModel:
        def __call__(self, image):
            calls.append(image.clone())
            return {
                "pointmaps": torch.ones((*image.shape[1:], 3)),
                "intrinsics": torch.eye(3),
                "unused": torch.zeros(1),
            }

    cache = _SameFrameDepthCache(DepthModel(), capacity=1)
    first = torch.zeros((3, 4, 5))
    second = torch.ones((3, 4, 5))

    assert "unused" in cache(first)
    cached = cache(first.clone())
    assert set(cached) == {"pointmaps", "intrinsics"}
    assert cache.last_call_metrics["pointmap_cache_hit"] is True
    cache(second)
    cache(first)

    assert len(calls) == 3
    assert cache.last_call_metrics["pointmap_cache_hits"] == 1
    assert cache.last_call_metrics["pointmap_cache_misses"] == 3
    assert cache.last_call_metrics["pointmap_cache_evictions"] == 2


def test_reconstruction_job_is_process_serializable(tmp_path):
    restored = pickle.loads(pickle.dumps(_job(tmp_path)))
    assert restored.track_id == "car-1"
    assert restored.observation.timestamp_us == 123


def test_coordinator_suppresses_duplicate_and_publishes_ready(tmp_path):
    class Reconstructor:
        def reconstruct(self, job):
            job.output_path.write_bytes(b"raw")
            return job.output_path

    class Aligner:
        def align(self, raw_path, aligned_path, measured):
            aligned_path.write_bytes(b"aligned")
            return SimpleNamespace(
                output_path=aligned_path,
                object_T_raw_mesh=np.eye(4),
                uniform_scale=2.0,
                aligned_dimensions=measured.as_array,
                dimension_residual=np.zeros(3),
                ground_offset=0.25,
                center_error=0.0,
                ground_error=0.0,
            )

    coordinator = ReconstructionCoordinator(Reconstructor(), Aligner())
    job = _job(tmp_path)
    assert coordinator.submit(job).status is ReconstructionStatus.QUEUED
    assert coordinator.submit(_job(tmp_path, "duplicate")).request_id == "request-1"
    assert coordinator.pending_count == 1

    result = coordinator.run_next()
    assert result is not None
    assert result.status is ReconstructionStatus.READY
    assert result.aligned_mesh_path == tmp_path / "raw.aligned.glb"
    assert result.metadata["ground_offset_m"] == 0.25
    assert coordinator.pending_count == 0


def test_coordinator_isolates_failure_and_allows_explicit_retry(tmp_path):
    class BrokenReconstructor:
        def reconstruct(self, job):
            raise RuntimeError("CUDA out of memory")

    coordinator = ReconstructionCoordinator(BrokenReconstructor(), SimpleNamespace())
    failed = coordinator.submit(_job(tmp_path))
    assert failed.status is ReconstructionStatus.QUEUED
    failed = coordinator.run_next()
    assert failed is not None
    assert failed.status is ReconstructionStatus.FAILED
    assert "CUDA out of memory" in (failed.error or "")

    assert coordinator.submit(_job(tmp_path, "ignored")).request_id == "request-1"
    retry = coordinator.submit(_job(tmp_path, "retry"), retry_failed=True)
    assert retry.status is ReconstructionStatus.QUEUED
    assert retry.request_id == "retry"


def test_mesh_alignment_uses_median_scale_and_grounds_mesh(tmp_path):
    trimesh = pytest.importorskip("trimesh")
    raw_path = tmp_path / "car.glb"
    output_path = tmp_path / "car.aligned.glb"
    # Deliberately offset: alignment must not assume the raw origin is useful.
    mesh = trimesh.creation.box(extents=(2.0, 2.0, 1.0))
    mesh.apply_translation((10.0, -4.0, 7.0))
    mesh.export(raw_path)

    result = MeshAligner().align(
        raw_path,
        output_path,
        VehicleDimensions(length=6.0, width=4.0, height=4.0),
    )
    # ratios = (3, 2, 4), so the robust isotropic scale is 3.
    assert result.uniform_scale == pytest.approx(3.0)
    np.testing.assert_allclose(result.aligned_dimensions, [6.0, 6.0, 3.0])
    aligned = trimesh.load(output_path, force="scene")
    np.testing.assert_allclose(aligned.bounds[:, 2], [-2.0, 1.0], atol=1e-6)
    np.testing.assert_allclose(aligned.bounds[:, :2].mean(axis=0), [0.0, 0.0], atol=1e-6)


def test_nuscenes_vertical_slice_prepares_mask_and_manifest(tmp_path):
    world_T_object = np.eye(4)
    world_T_object[2, 3] = 10.0
    frame = NuScenesFrame(
        scene_token="scene-token",
        sample_token="sample-token",
        timestamp_us=100,
        image=np.zeros((80, 100, 3), dtype=np.uint8),
        image_path=tmp_path / "image.jpg",
        lidar_points=np.zeros((1, 5), dtype=np.float32),
        lidar_path=tmp_path / "lidar.bin",
        camera_intrinsic=np.asarray([[100.0, 0, 50.0], [0, 100.0, 40.0], [0, 0, 1.0]]),
        lidar_to_camera=np.eye(4),
        vehicle_boxes=(
            VehicleBox(
                "ann",
                "track",
                "vehicle.car",
                np.asarray([0, 0, 10], dtype=np.float32),
                np.asarray([2, 4, 2], dtype=np.float32),
                0.0,
                world_T_object,
            ),
        ),
        trajectory=np.zeros((1, 3)),
        trajectory_timestamps_us=np.asarray([100]),
        world_T_camera=np.eye(4),
        camera_timestamp_us=101,
    )

    class Sequence:
        name = "scene-001"

        def __getitem__(self, index):
            assert index == 0
            return frame

    class Dataset:
        def __getitem__(self, index):
            assert index == 0
            return Sequence()

    pipeline = NuScenesCarReconstructionPipeline(
        dataset=Dataset(), reconstructor=None, minimum_area_px=1
    )
    mask_path = tmp_path / "offline-mask.png"
    offline_mask = np.zeros(frame.image.shape[:2], dtype=np.uint8)
    offline_mask[20:60, 25:75] = 255
    assert pytest.importorskip("cv2").imwrite(str(mask_path), offline_mask)
    _, vehicle, _, mask, asset = pipeline.run(
        scene="0",
        frame_index=0,
        output_root=tmp_path / "out",
        mask_path=mask_path,
        prepare_only=True,
    )

    assert vehicle.instance_token == "track"
    assert mask.any()
    assert asset is None
    manifest = (
        tmp_path / "out" / "scene-001" / "000" / "track" / "manifest.json"
    ).read_text()
    assert '"dimensions_lwh_m": [' in manifest
    assert '"camera_intrinsic": [' in manifest
    assert '"status": "prepared"' in manifest

    class Reconstructor:
        def reconstruct(self, job):
            job.output_path.write_bytes(b"raw")
            return job.output_path

    class Aligner:
        def align(self, raw_path, aligned_path, measured):
            aligned_path.write_bytes(b"aligned")
            return SimpleNamespace(
                output_path=aligned_path,
                object_T_raw_mesh=np.eye(4),
                uniform_scale=1.0,
                aligned_dimensions=measured.as_array,
                dimension_residual=np.zeros(3),
                ground_offset=0.0,
                center_error=0.0,
                ground_error=0.0,
            )

    prepared_dir = tmp_path / "out" / "scene-001" / "000" / "track"
    result = PreparedAssetReconstructionPipeline(
        reconstructor=Reconstructor(), aligner=Aligner()
    ).run(prepared_dir)
    assert result.status is ReconstructionStatus.READY
    assert '"status": "ready"' in (prepared_dir / "manifest.json").read_text()

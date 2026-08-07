"""One-car nuScenes to metric SAM3D asset vertical slice."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any
from uuid import uuid4

import cv2
import numpy as np

from src.common import CameraObservation, Pose3D
from src.data import NuScenesFrame, NuScenesSequence, NuScenesSequenceDataset, VehicleBox
from src.reconstruction.coordinator import ReconstructionCoordinator
from src.reconstruction.mesh_alignment import MeshAligner
from src.reconstruction.models import (
    ReconstructedAsset,
    ReconstructionJob,
    VehicleDimensions,
)
from src.reconstruction.projection import ProjectedBox, project_vehicle_box
from src.reconstruction.sam3d import SAM3DObjectReconstructor
from src.reconstruction.sam3d_upstream import UpstreamSAM3DBackend


class NuScenesCarReconstructionPipeline:
    """Prepare and optionally reconstruct one authoritative nuScenes car track."""

    def __init__(
        self,
        *,
        dataset: NuScenesSequenceDataset,
        reconstructor: SAM3DObjectReconstructor | None,
        minimum_area_px: float = 1024.0,
    ) -> None:
        self.dataset = dataset
        self.reconstructor = reconstructor
        self.minimum_area_px = minimum_area_px

    def run(
        self,
        *,
        scene: str,
        frame_index: int,
        output_root: Path,
        track_id: str | None = None,
        mask_path: Path | None = None,
        seed: int = 42,
        prepare_only: bool = False,
    ) -> tuple[NuScenesFrame, VehicleBox, ProjectedBox, np.ndarray, ReconstructedAsset | None]:
        sequence = self._sequence(scene)
        frame = sequence[frame_index]
        vehicle, projected = self._select_vehicle(frame, track_id)
        mask = self._mask(frame, mask_path)

        asset_dir = output_root / sequence.name / f"{frame_index:03d}" / vehicle.instance_token
        asset_dir.mkdir(parents=True, exist_ok=True)
        _write_rgb(asset_dir / "image.jpg", frame.image)
        _write_mask(asset_dir / "mask.png", mask)

        dimensions = VehicleDimensions(*vehicle.dimensions_lwh)
        observation = CameraObservation(
            image=frame.image,
            mask=mask,
            camera_intrinsic=frame.camera_intrinsic,
            world_T_camera=Pose3D.from_matrix(frame.world_T_camera),
            timestamp_us=frame.camera_timestamp_us,
            crop_xyxy=tuple(int(round(value)) for value in projected.xyxy),
            metadata={"scene": sequence.name, "sample_token": frame.sample_token},
        )
        job = ReconstructionJob(
            request_id=str(uuid4()),
            track_id=vehicle.instance_token,
            observation=observation,
            mask=mask,
            dimensions=dimensions,
            world_T_object=Pose3D.from_matrix(vehicle.world_T_object),
            output_path=asset_dir / "raw.glb",
            seed=seed,
            metadata={
                "scene": sequence.name,
                "frame_index": frame_index,
                "sample_token": frame.sample_token,
                "annotation_token": vehicle.annotation_token,
                "projected_box_xyxy": projected.xyxy.tolist(),
                "visible_fraction": projected.visible_fraction,
                "mask_source": "offline",
                "reconstructor": _component_metadata(self.reconstructor),
            },
        )
        self._write_manifest(asset_dir / "manifest.json", frame, vehicle, job, None)
        if prepare_only:
            return frame, vehicle, projected, mask, None
        if self.reconstructor is None:
            raise RuntimeError("a reconstructor is required unless --prepare-only is used")

        coordinator = ReconstructionCoordinator(self.reconstructor, MeshAligner())
        coordinator.submit(job)
        result = coordinator.run_next()
        assert result is not None
        self._write_manifest(asset_dir / "manifest.json", frame, vehicle, job, result)
        if result.error:
            raise RuntimeError(result.error)
        return frame, vehicle, projected, mask, result

    def _sequence(self, scene: str) -> NuScenesSequence:
        try:
            return self.dataset[int(scene)]
        except ValueError:
            for sequence in self.dataset:
                if sequence.name == scene:
                    return sequence
        raise ValueError(f"nuScenes scene not found: {scene}")

    def _select_vehicle(
        self, frame: NuScenesFrame, track_id: str | None
    ) -> tuple[VehicleBox, ProjectedBox]:
        candidates: list[tuple[float, VehicleBox, ProjectedBox]] = []
        for vehicle in frame.vehicle_boxes:
            if vehicle.category != "vehicle.car" or (
                track_id is not None and vehicle.instance_token != track_id
            ):
                continue
            dimensions = VehicleDimensions(*vehicle.dimensions_lwh)
            projected = project_vehicle_box(
                world_T_object=vehicle.world_T_object,
                world_T_camera=frame.world_T_camera,
                intrinsics=frame.camera_intrinsic,
                dimensions=dimensions,
                image_size=frame.image.shape[:2],
                minimum_area_px=self.minimum_area_px,
            )
            if projected is not None:
                candidates.append(
                    (projected.width * projected.height * projected.visible_fraction, vehicle, projected)
                )
        if not candidates:
            suffix = f" for track {track_id}" if track_id else ""
            raise ValueError(f"no visible vehicle.car candidate{suffix}")
        _, vehicle, projected = max(candidates, key=lambda item: item[0])
        return vehicle, projected

    def _mask(
        self, frame: NuScenesFrame, mask_path: Path | None
    ) -> np.ndarray:
        if mask_path is None:
            raise RuntimeError(
                "offline reconstruction requires an explicit --mask-path; "
                "use src.realtime.mask_process for online SAM 3.1 masks"
            )
        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if mask is None:
            raise FileNotFoundError(f"could not read mask: {mask_path}")
        if mask.shape != frame.image.shape[:2]:
            raise ValueError("prepared mask dimensions must match the source image")
        return mask > 0

    @staticmethod
    def _write_manifest(
        path: Path,
        frame: NuScenesFrame,
        vehicle: VehicleBox,
        job: ReconstructionJob,
        result: ReconstructedAsset | None,
    ) -> None:
        ego_T_object = np.linalg.inv(frame.world_T_ego) @ vehicle.world_T_object
        payload: dict[str, Any] = {
            "request_id": job.request_id,
            "track_id": job.track_id,
            "timestamp_us": frame.timestamp_us,
            "camera_timestamp_us": frame.camera_timestamp_us,
            "coordinate_convention": "right-handed, x-forward, y-left, z-up, meters",
            "quaternion_order": "xyzw",
            "dimensions_lwh_m": job.dimensions.as_array.tolist(),
            "world_T_ego": frame.world_T_ego.tolist(),
            "world_T_lidar": frame.world_T_lidar.tolist(),
            "world_T_camera": frame.world_T_camera.tolist(),
            "camera_intrinsic": frame.camera_intrinsic.tolist(),
            "world_T_object": vehicle.world_T_object.tolist(),
            "ego_T_object": ego_T_object.tolist(),
            "seed": job.seed,
            "metadata": dict(job.metadata),
            "status": "prepared" if result is None else result.status.value,
        }
        if result is not None:
            payload.update(_result_payload(result))
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


class PreparedAssetReconstructionPipeline:
    """Run reconstruction from a persisted vehicle-side request bundle."""

    def __init__(
        self,
        *,
        reconstructor: SAM3DObjectReconstructor,
        aligner: MeshAligner | None = None,
    ) -> None:
        self.reconstructor = reconstructor
        self.aligner = aligner or MeshAligner()

    def run(self, asset_dir: Path) -> ReconstructedAsset:
        manifest_path = asset_dir / "manifest.json"
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
        image_bgr = cv2.imread(str(asset_dir / "image.jpg"), cv2.IMREAD_COLOR)
        mask_image = cv2.imread(str(asset_dir / "mask.png"), cv2.IMREAD_GRAYSCALE)
        if image_bgr is None or mask_image is None:
            raise FileNotFoundError(f"prepared RGB/mask is incomplete: {asset_dir}")
        image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        mask = mask_image > 0
        metadata = dict(payload.get("metadata", {}))
        projected_box = metadata.get("projected_box_xyxy")
        crop = tuple(int(round(value)) for value in projected_box) if projected_box else None
        observation = CameraObservation(
            image=image,
            mask=mask,
            camera_intrinsic=np.asarray(payload.get("camera_intrinsic", np.eye(3))),
            world_T_camera=Pose3D.from_matrix(np.asarray(payload["world_T_camera"])),
            timestamp_us=int(payload.get("camera_timestamp_us", payload["timestamp_us"])),
            crop_xyxy=crop,
            metadata={"prepared_manifest": str(manifest_path)},
        )
        dimensions = VehicleDimensions(*payload["dimensions_lwh_m"])
        job = ReconstructionJob(
            request_id=str(payload["request_id"]),
            track_id=str(payload["track_id"]),
            observation=observation,
            mask=mask,
            dimensions=dimensions,
            world_T_object=Pose3D.from_matrix(np.asarray(payload["world_T_object"])),
            output_path=asset_dir / "raw.glb",
            seed=int(payload.get("seed", 42)),
            metadata={**metadata, "resumed_from_prepared_bundle": True},
        )
        coordinator = ReconstructionCoordinator(self.reconstructor, self.aligner)
        coordinator.submit(job)
        result = coordinator.run_next()
        assert result is not None
        payload.update({"status": result.status.value, **_result_payload(result)})
        manifest_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        if result.error:
            raise RuntimeError(result.error)
        return result


def _result_payload(result: ReconstructedAsset) -> dict[str, Any]:
    return {
        "raw_mesh_path": str(result.raw_mesh_path),
        "aligned_mesh_path": str(result.aligned_mesh_path),
        "object_T_raw_mesh": (
            result.object_T_raw_mesh.tolist() if result.object_T_raw_mesh is not None else None
        ),
        "uniform_scale": result.uniform_scale,
        "aligned_dimensions_m": (
            result.aligned_dimensions.tolist() if result.aligned_dimensions is not None else None
        ),
        "result_metadata": dict(result.metadata),
        "error": result.error,
    }


def _write_rgb(path: Path, image: np.ndarray) -> None:
    if not cv2.imwrite(str(path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR)):
        raise RuntimeError(f"failed to write {path}")


def _write_mask(path: Path, mask: np.ndarray) -> None:
    if not cv2.imwrite(str(path), mask.astype(np.uint8) * 255):
        raise RuntimeError(f"failed to write {path}")


def _component_metadata(component: object | None) -> dict[str, Any] | None:
    if component is None:
        return None
    metadata: dict[str, Any] = {"class": f"{type(component).__module__}.{type(component).__name__}"}
    for name in ("checkpoint", "model_type", "device", "_factory_path", "_factory_kwargs"):
        value = getattr(component, name, None)
        if value is not None:
            metadata[name.lstrip("_")] = _jsonable(value)
    return metadata


def _jsonable(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataroot", nargs="?")
    parser.add_argument("--prepared-dir", type=Path)
    parser.add_argument("--scene", default="scene-0061")
    parser.add_argument("--frame", type=int, default=35)
    parser.add_argument("--track-id")
    parser.add_argument("--version", default="v1.0-mini")
    parser.add_argument("--output-root", type=Path, default=Path("artifacts/sam3d"))
    parser.add_argument("--mask-path", type=Path)
    parser.add_argument("--sam3d-repository", type=Path, default=Path("thirdparty/sam-3d-objects"))
    parser.add_argument("--sam3d-config", type=Path)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--rerun", action="store_true")
    args = parser.parse_args()

    if args.prepared_dir is None and args.mask_path is None:
        parser.error("--mask-path is required unless --prepared-dir is used")
    reconstructor = None
    if not args.prepare_only:
        if args.sam3d_config is None:
            parser.error("--sam3d-config is required unless --prepare-only is used")
        reconstructor = SAM3DObjectReconstructor(
            factory=UpstreamSAM3DBackend,
            factory_kwargs={
                "repository": args.sam3d_repository,
                "config_path": args.sam3d_config,
            },
        )
    if args.prepared_dir is not None:
        if args.prepare_only:
            parser.error("--prepared-dir cannot be combined with --prepare-only")
        assert reconstructor is not None
        asset = PreparedAssetReconstructionPipeline(reconstructor=reconstructor).run(
            args.prepared_dir
        )
        print(f"ready: {asset.aligned_mesh_path}")
        return
    if args.dataroot is None:
        parser.error("dataroot is required unless --prepared-dir is used")
    pipeline = NuScenesCarReconstructionPipeline(
        dataset=NuScenesSequenceDataset(args.dataroot, version=args.version, future_steps=1),
        reconstructor=reconstructor,
    )
    result = pipeline.run(
        scene=args.scene,
        frame_index=args.frame,
        output_root=args.output_root,
        track_id=args.track_id,
        mask_path=args.mask_path,
        seed=args.seed,
        prepare_only=args.prepare_only,
    )
    asset = result[-1]
    if args.rerun and asset is not None and asset.aligned_mesh_path is not None:
        from src.viz.reconstruction_visualizer import ReconstructionRerunVisualizer

        ReconstructionRerunVisualizer().show(
            frame=result[0],
            vehicle=result[1],
            mask=result[3],
            aligned_mesh=asset.aligned_mesh_path,
            projected_box=result[2],
            raw_mesh=asset.raw_mesh_path,
            object_T_raw_mesh=asset.object_T_raw_mesh,
        )
    print("prepared inputs" if asset is None else f"ready: {asset.aligned_mesh_path}")


if __name__ == "__main__":
    main()

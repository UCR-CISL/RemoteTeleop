from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from src.realtime.protocol import (
    AssetState,
    BoxPrompt,
    FrameDetections,
    ReconstructionRequest,
)
from src.realtime.sam3d_worker import SAM3DReconstructionService, SAM3DWorkerProcess
from src.realtime.gpu_residency import ResidencyAcquisition


IDENTITY_3 = tuple(np.eye(3).reshape(-1))
IDENTITY_4 = tuple(np.eye(4).reshape(-1))


def _request(
    track_id: str,
    *,
    generation: str = "scene-1:generation-1",
    quality: float = 0.5,
) -> ReconstructionRequest:
    image = np.full((12, 16, 3), 127, dtype=np.uint8)
    mask = np.zeros((12, 16), dtype=np.uint8)
    mask[2:10, 3:13] = 255
    image_ok, image_jpeg = cv2.imencode(".jpg", image)
    mask_ok, mask_png = cv2.imencode(".png", mask)
    assert image_ok and mask_ok
    return ReconstructionRequest(
        request_id=f"request-{generation}-{track_id}",
        scene_generation=generation,
        track_id=track_id,
        frame_id="frame-1",
        timestamp_us=123,
        image_jpeg=image_jpeg.tobytes(),
        mask_png=mask_png.tobytes(),
        camera_intrinsic=IDENTITY_3,
        dimensions_lwh=(4.2, 1.9, 1.5),
        world_T_object=IDENTITY_4,
        world_T_camera=IDENTITY_4,
        quality_score=quality,
        seed=7,
    )


def _frame(
    *track_ids: str,
    generation: str = "scene-1:generation-1",
    timestamp_us: int = 123,
    end_of_scene: bool = False,
) -> FrameDetections:
    ok, jpeg = cv2.imencode(".jpg", np.zeros((12, 16, 3), dtype=np.uint8))
    assert ok
    return FrameDetections(
        frame_id=f"frame-{timestamp_us}",
        request_id=f"frame-request-{timestamp_us}",
        scene_generation=generation,
        timestamp_us=timestamp_us,
        image_jpeg=jpeg.tobytes(),
        camera_intrinsic=IDENTITY_3,
        world_T_camera=IDENTITY_4,
        world_T_ego=IDENTITY_4,
        boxes=tuple(
            BoxPrompt(
                track_id=track_id,
                xyxy=(1.0, 1.0, 10.0, 10.0),
                dimensions_lwh=(4.2, 1.9, 1.5),
                world_T_object=IDENTITY_4,
            )
            for track_id in track_ids
        ),
        end_of_scene=end_of_scene,
    )


class FakeReconstructor:
    def __init__(self) -> None:
        self.loads = 0
        self.jobs = []
        self.unloads = 0

    def load(self) -> None:
        self.loads += 1

    def reconstruct(self, job):
        self.jobs.append(job)
        job.output_path.parent.mkdir(parents=True, exist_ok=True)
        job.output_path.write_bytes(b"raw mesh")
        return job.output_path

    def unload(self) -> None:
        self.unloads += 1


class FakeAligner:
    def align(self, raw_path, aligned_path, measured):
        assert raw_path.is_file()
        aligned_path.write_bytes(b"aligned mesh")
        return SimpleNamespace(
            output_path=aligned_path,
            object_T_raw_mesh=np.eye(4),
            uniform_scale=2.5,
            aligned_dimensions=measured.as_array,
            dimension_residual=np.zeros(3),
            ground_offset=0.1,
            center_error=0.0,
            ground_error=0.0,
        )


def test_service_loads_once_and_queues_by_quality(tmp_path):
    reconstructor = FakeReconstructor()
    service = SAM3DReconstructionService(
        reconstructor=reconstructor,
        aligner=FakeAligner(),
        output_root=tmp_path,
    )
    service.load()
    low = _request("low", quality=0.1)
    high = _request("high", quality=0.9)
    assert service.enqueue(low)[1].state is AssetState.QUEUED
    assert service.enqueue(high)[1].state is AssetState.QUEUED
    service.update_visibility(_frame("low", "high"))

    assert service.pop_next().request == high
    assert service.pop_next().request == low
    assert service.pop_next() is None
    assert reconstructor.loads == 1


def test_service_unloads_model_without_discarding_queued_work(tmp_path):
    reconstructor = FakeReconstructor()
    service = SAM3DReconstructionService(
        reconstructor=reconstructor,
        aligner=FakeAligner(),
        output_root=tmp_path,
    )
    request = _request("waiting")
    service.enqueue(request)

    service.load()
    service.unload()

    assert reconstructor.loads == 1
    assert reconstructor.unloads == 1
    assert service.queue_depth == 1


def test_worker_model_handoff_logs_switch_latency(tmp_path):
    class Service:
        queue_depth = 0

        def __init__(self):
            self.loads = 0
            self.unloads = 0

        def load(self):
            self.loads += 1
            return {
                "residency_action": "cuda_resume",
                "resume_transfer_seconds": 0.2,
            }

        def unload(self):
            self.unloads += 1
            return {
                "residency_action": "cpu_offload",
                "offload_transfer_seconds": 0.3,
            }

    class Lease:
        def __init__(self):
            self.releases = 0

        def release(self):
            self.releases += 1

    service = Service()
    lease = Lease()
    metrics_path = tmp_path / "sam3d.jsonl"
    process = SAM3DWorkerProcess(
        service=service,
        router=None,
        assets=None,
        health=None,
        metrics_path=metrics_path,
        residency_lease=lease,
    )

    process._load_model(wait_seconds=0.75)
    process._unload_model()

    records = [json.loads(line) for line in metrics_path.read_text().splitlines()]
    assert service.loads == service.unloads == lease.releases == 1
    assert [record["event"] for record in records] == [
        "model_loaded",
        "model_released",
    ]
    assert records[0]["residency_wait_seconds"] == 0.75
    assert records[0]["resume_transfer_seconds"] == 0.2
    assert records[1]["offload_transfer_seconds"] == 0.3
    assert all(record["runtime_mode"] == "take_turns" for record in records)


def test_worker_prewarm_offloads_retained_model_and_writes_ready_marker(tmp_path):
    class Service:
        def __init__(self):
            self.loads = 0
            self.unloads = 0
            self.queue_depth = 0

        def load(self):
            self.loads += 1
            return {"residency_action": "initial_load"}

        def unload(self):
            self.unloads += 1
            return {"residency_action": "cpu_offload"}

    class Lease:
        def __init__(self):
            self.releases = 0

        def try_acquire(self):
            return ResidencyAcquisition("sam3d-object", 0.0)

        def release(self):
            self.releases += 1

    service = Service()
    lease = Lease()
    ready = tmp_path / "sam3d-ready"
    process = SAM3DWorkerProcess(
        service=service,
        router=None,
        assets=None,
        health=SimpleNamespace(send=lambda _: None),
        metrics_path=tmp_path / "sam3d.jsonl",
        residency_lease=lease,
        ready_path=ready,
    )

    process._prewarm_model()
    process._write_ready()

    records = [json.loads(line) for line in (tmp_path / "sam3d.jsonl").read_text().splitlines()]
    assert service.loads == service.unloads == lease.releases == 1
    assert ready.read_text() == "ready\n"
    assert [record["event"] for record in records] == [
        "prewarm_residency_acquired",
        "model_loaded",
        "model_released",
        "model_prewarmed",
    ]


def test_worker_resident_prewarm_keeps_model_on_cuda_and_releases_lease(tmp_path):
    class Service:
        queue_depth = 0

        def __init__(self):
            self.loads = 0
            self.unloads = 0

        def load(self):
            self.loads += 1
            return {"residency_action": "initial_load"}

        def unload(self):
            self.unloads += 1

    class Lease:
        def __init__(self):
            self.releases = 0

        def try_acquire(self):
            return ResidencyAcquisition("sam3d-object", 0.0)

        def release(self):
            self.releases += 1

    service, lease = Service(), Lease()
    process = SAM3DWorkerProcess(
        service=service, router=None, assets=None,
        health=SimpleNamespace(send=lambda _: None),
        metrics_path=tmp_path / "sam3d.jsonl", residency_lease=lease,
        keep_cuda_resident=True,
    )
    process._prewarm_model()

    assert service.loads == 1
    assert service.unloads == 0
    assert lease.releases == 1
    assert process._model_loaded is True


def test_worker_rejects_prewarm_without_residency_lease(tmp_path):
    with pytest.raises(ValueError, match="requires a GPU residency lease"):
        SAM3DWorkerProcess(
            service=SimpleNamespace(),
            router=None,
            assets=None,
            health=SimpleNamespace(send=lambda _: None),
            metrics_path=tmp_path / "sam3d.jsonl",
            prewarm=True,
        )


def test_service_deduplicates_track_within_generation(tmp_path):
    service = SAM3DReconstructionService(
        reconstructor=FakeReconstructor(),
        aligner=FakeAligner(),
        output_root=tmp_path,
    )
    first = _request("car-1")
    duplicate = _request("car-1", quality=0.9)
    another_generation = _request(
        "car-1", generation="scene-1:generation-2", quality=0.8
    )

    assert service.enqueue(first)[1] is not None
    acknowledgement, event = service.enqueue(duplicate)
    assert acknowledgement.accepted
    assert acknowledgement.detail == "track already queued or reconstructed"
    assert event is None
    assert service.enqueue(another_generation)[1] is not None
    assert service.queue_depth == 2


def test_service_decodes_request_and_publishes_metric_aligned_asset(tmp_path):
    reconstructor = FakeReconstructor()
    service = SAM3DReconstructionService(
        reconstructor=reconstructor,
        aligner=FakeAligner(),
        output_root=tmp_path,
    )
    request = _request("car/unsafe", quality=0.7)
    service.enqueue(request)
    service.update_visibility(_frame("car/unsafe"))

    event = service.reconstruct(service.pop_next())

    assert event.state is AssetState.READY
    assert event.aligned_mesh_path.is_file()
    assert event.aligned_mesh_path.is_relative_to(tmp_path)
    assert event.metrics["uniform_scale"] == pytest.approx(2.5)
    assert event.metrics["measured_dimensions_lwh_m"] == [4.2, 1.9, 1.5]
    assert event.metrics["dimension_residual_m"] == [0.0, 0.0, 0.0]
    job = reconstructor.jobs[0]
    assert job.mask.sum() == 80
    np.testing.assert_allclose(job.dimensions.as_array, [4.2, 1.9, 1.5])
    np.testing.assert_allclose(job.world_T_object.matrix, np.eye(4))


def test_service_converts_reconstruction_failures_to_asset_events(tmp_path):
    class FailingReconstructor(FakeReconstructor):
        def reconstruct(self, job):
            raise RuntimeError("out of memory")

    service = SAM3DReconstructionService(
        reconstructor=FailingReconstructor(),
        aligner=FakeAligner(),
        output_root=tmp_path,
    )
    request = _request("car-1")
    service.enqueue(request)
    service.update_visibility(_frame("car-1"))

    event = service.reconstruct(service.pop_next())

    assert event.state is AssetState.FAILED
    assert event.error == "RuntimeError: out of memory"
    assert event.metrics["failed_after_ms"] >= 0.0


def test_service_cancels_queued_track_after_it_exits(tmp_path):
    now = [1.0]
    service = SAM3DReconstructionService(
        reconstructor=FakeReconstructor(),
        aligner=FakeAligner(),
        output_root=tmp_path,
        clock=lambda: now[0],
    )
    live = _request("live")
    exited = _request("exited")
    service.enqueue(live)
    service.enqueue(exited)

    now[0] = 1.5
    events = service.update_visibility(_frame("live", timestamp_us=623))

    assert service.queue_depth == 1
    assert service.pop_next().request == live
    assert len(events) == 1
    assert events[0].track_id == "exited"
    assert events[0].state is AssetState.CANCELLED
    assert events[0].metrics["visible_lifetime_ms"] == pytest.approx(0.5)
    assert events[0].metrics["queue_latency_ms"] == pytest.approx(500.0)


def test_service_rejects_request_that_arrives_after_track_exit(tmp_path):
    service = SAM3DReconstructionService(
        reconstructor=FakeReconstructor(),
        aligner=FakeAligner(),
        output_root=tmp_path,
    )
    service.update_visibility(_frame(timestamp_us=500))

    acknowledgement, event = service.enqueue(_request("late"))

    assert acknowledgement.accepted
    assert "expired" in acknowledgement.detail
    assert event is not None
    assert event.state is AssetState.CANCELLED
    assert event.metrics["cancelled_before_enqueue"] is True
    assert service.queue_depth == 0


def test_service_retires_queued_and_inflight_work_from_previous_generation(tmp_path):
    service = SAM3DReconstructionService(
        reconstructor=FakeReconstructor(),
        aligner=FakeAligner(),
        output_root=tmp_path,
    )
    generation = "scene-1:generation-1"
    service.update_visibility(_frame("running", "queued", generation=generation))
    running = _request("running", generation=generation)
    queued = _request("queued", generation=generation)
    service.enqueue(running)
    service.enqueue(queued)
    running_item = service.pop_next()
    ready = service.reconstruct(running_item)

    cancelled = service.update_visibility(
        _frame("new", generation="scene-1:generation-2", timestamp_us=1_000)
    )

    assert len(cancelled) == 1
    assert cancelled[0].track_id == "queued"
    assert cancelled[0].metrics["scene_generation_retired"] is True
    stale = service.discard_stale_result(ready)
    assert stale.state is AssetState.CANCELLED
    assert stale.metrics["result_discarded_after_inference"] is True
    assert stale.aligned_mesh_path is None


def test_service_ignores_out_of_order_and_retired_generation_frames(tmp_path):
    service = SAM3DReconstructionService(
        reconstructor=FakeReconstructor(),
        aligner=FakeAligner(),
        output_root=tmp_path,
    )
    old_generation = "scene-1:generation-1"
    new_generation = "scene-1:generation-2"
    service.update_visibility(
        _frame("current", generation=old_generation, timestamp_us=500)
    )
    assert service.update_visibility(
        _frame(generation=old_generation, timestamp_us=400)
    ) == ()
    assert service.is_active(old_generation, "current")

    service.update_visibility(
        _frame("new", generation=new_generation, timestamp_us=100)
    )
    assert service.update_visibility(
        _frame("current", generation=old_generation, timestamp_us=600)
    ) == ()
    assert service.is_active(new_generation, "new")
    assert not service.is_active(old_generation, "current")


def test_scene_end_cancels_final_visible_queue_and_stales_running_result(tmp_path):
    service = SAM3DReconstructionService(
        reconstructor=FakeReconstructor(),
        aligner=FakeAligner(),
        output_root=tmp_path,
    )
    service.update_visibility(_frame("running", "queued", timestamp_us=123))
    running = _request("running")
    queued = _request("queued")
    service.enqueue(running)
    service.enqueue(queued)
    running_item = service.pop_next()
    ready = service.reconstruct(running_item)

    end = _frame(timestamp_us=1_000, end_of_scene=True)
    cancelled = service.update_visibility(end)

    assert len(cancelled) == 1
    assert cancelled[0].track_id == "queued"
    assert cancelled[0].metrics["scene_generation_retired"] is True
    assert service.discard_stale_result(ready).state is AssetState.CANCELLED


def test_service_coalesces_before_dispatch(tmp_path):
    now = [0.0]
    service = SAM3DReconstructionService(
        reconstructor=FakeReconstructor(),
        aligner=FakeAligner(),
        output_root=tmp_path,
        clock=lambda: now[0],
        coalesce_seconds=0.5,
    )
    service.update_visibility(_frame("car"))
    service.enqueue(_request("car"))

    now[0] = 0.49
    assert service.pop_next() is None
    now[0] = 0.5
    assert service.pop_next().request.track_id == "car"


def test_deadline_policy_prioritizes_stable_track_over_higher_quality_exit(tmp_path):
    service = SAM3DReconstructionService(
        reconstructor=FakeReconstructor(),
        aligner=FakeAligner(),
        output_root=tmp_path,
        priority_policy="deadline",
    )
    camera = (10.0, 0.0, 50.0, 0.0, 10.0, 50.0, 0.0, 0.0, 1.0)
    first = replace(
        _frame(timestamp_us=123),
        camera_intrinsic=camera,
        boxes=(
            BoxPrompt("outgoing", (20.0, 20.0, 40.0, 40.0), (4, 2, 1.5), IDENTITY_4),
            BoxPrompt("stable", (20.0, 20.0, 40.0, 40.0), (4, 2, 1.5), IDENTITY_4),
        ),
    )
    second = replace(
        first,
        frame_id="frame-623",
        request_id="frame-request-623",
        timestamp_us=623,
        boxes=(
            BoxPrompt("outgoing", (1.0, 20.0, 21.0, 40.0), (4, 2, 1.5), IDENTITY_4),
            BoxPrompt("stable", (20.0, 20.0, 40.0, 40.0), (4, 2, 1.5), IDENTITY_4),
        ),
    )
    service.update_visibility(first)
    service.update_visibility(second)
    service.enqueue(_request("outgoing", quality=0.99))
    service.enqueue(_request("stable", quality=0.1))

    assert service.pop_next().request.track_id == "stable"

from __future__ import annotations

import json
from dataclasses import replace
from types import SimpleNamespace

import cv2
import numpy as np

from src.data.nuscenes_loader import VehicleBox
from src.realtime.mask_process import (
    MaskProcessService,
    ReliableReconstructionClient,
    StableTrackAdmission,
    StableTrackAdmissionConfig,
    build_reconstruction_requests,
    protocol_frame_to_prompt,
    segmentation_batch_to_protocol,
)
from src.realtime.nuscenes_replay import (
    NuScenesReplayPublisher,
    ReplayConfig,
    frame_to_detections,
)
from src.realtime.protocol import (
    Acknowledgement,
    BoxPrompt,
    FrameDetections,
    ReconstructionRequest,
)
from src.realtime.runtime_support import JsonlMetricsWriter
from src.segmentation.models import (
    MaskBatch as SegmentationMaskBatch,
    MaskResult as SegmentationMaskResult,
    MaskWorkerMetrics,
)


def _vehicle(track_id: str, category: str = "vehicle.car") -> VehicleBox:
    pose = np.eye(4)
    pose[2, 3] = 10.0
    return VehicleBox(
        annotation_token=f"annotation-{track_id}",
        instance_token=track_id,
        category=category,
        center=np.asarray([0.0, 0.0, 10.0]),
        size_wlh=np.asarray([2.0, 4.0, 2.0]),
        yaw=0.0,
        world_T_object=pose,
    )


def _frame(timestamp_us: int = 1_000_000, token: str = "sample-0"):
    return SimpleNamespace(
        sample_token=token,
        timestamp_us=timestamp_us,
        image=np.zeros((100, 200, 3), dtype=np.uint8),
        camera_intrinsic=np.asarray(
            [[100.0, 0.0, 100.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]]
        ),
        world_T_camera=np.eye(4),
        world_T_ego=np.eye(4),
        vehicle_boxes=(_vehicle("car-a"), _vehicle("truck", "vehicle.truck")),
    )


def test_replay_projects_only_exact_car_category_and_keeps_metric_state():
    message = frame_to_detections(
        _frame(), ReplayConfig(scene_generation="scene:generation")
    )

    assert message.frame_id == "sample-0"
    assert len(message.boxes) == 1
    box = message.boxes[0]
    assert box.track_id == "car-a"
    assert box.dimensions_lwh == (4.0, 2.0, 2.0)
    np.testing.assert_allclose(np.asarray(box.world_T_object).reshape(4, 4)[2, 3], 10.0)
    assert 0.0 < box.visibility <= 1.0
    decoded = cv2.imdecode(np.frombuffer(message.image_jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (100, 200, 3)


def test_replay_uses_recorded_timestamp_cadence():
    now = [10.0]
    sleeps: list[float] = []
    published: list[FrameDetections] = []

    class Publisher:
        def send(self, message, *, topic=None):
            published.append(message)

    def sleep(seconds: float) -> None:
        sleeps.append(seconds)
        now[0] += seconds

    replay = NuScenesReplayPublisher(
        Publisher(),
        ReplayConfig(scene_generation="scene:generation", rate=2.0),
        monotonic=lambda: now[0],
        sleep=sleep,
    )
    count = replay.run(
        [_frame(1_000_000, "sample-0"), _frame(2_000_000, "sample-1")]
    )

    assert count == 2
    np.testing.assert_allclose(sleeps, [0.5, 0.05, 0.05])
    assert len(published) == 3
    assert published[-1].end_of_scene
    assert published[-1].boxes == ()


def _wire_frame() -> FrameDetections:
    image = np.zeros((10, 20, 3), dtype=np.uint8)
    ok, jpeg = cv2.imencode(".jpg", image)
    assert ok
    identity = tuple(float(value) for value in np.eye(4).reshape(-1))
    return FrameDetections(
        frame_id="frame",
        request_id="frame-request",
        scene_generation="generation",
        timestamp_us=123,
        image_jpeg=jpeg.tobytes(),
        camera_intrinsic=(10.0, 0.0, 10.0, 0.0, 10.0, 5.0, 0.0, 0.0, 1.0),
        world_T_camera=identity,
        world_T_ego=identity,
        boxes=(
            BoxPrompt(
                track_id="car",
                xyxy=(2.0, 2.0, 10.0, 8.0),
                dimensions_lwh=(4.0, 2.0, 1.5),
                world_T_object=identity,
                visibility=0.8,
            ),
        ),
    )


def _segmentation_batch() -> SegmentationMaskBatch:
    mask = np.zeros((10, 20), dtype=bool)
    mask[2:8, 2:10] = True
    return SegmentationMaskBatch(
        frame_id="frame",
        timestamp_us=123,
        masks={
            "car": SegmentationMaskResult(
                track_id="car",
                mask=mask,
                confidence=0.9,
                in_box_fraction=1.0,
                box_coverage=1.0,
            )
        },
        rejected_tracks={},
        metrics=MaskWorkerMetrics(
            backend="fake",
            model_load_seconds=1.0,
            warmup_seconds=2.0,
            image_encode_seconds=0.01,
            prompt_decode_seconds=0.02,
            total_seconds=0.03,
            prompts=1,
            accepted_masks=1,
        ),
    )


def test_mask_adapters_preserve_frame_and_generate_png():
    source = _wire_frame()
    prompt = protocol_frame_to_prompt(source)
    batch = segmentation_batch_to_protocol(source, _segmentation_batch())

    assert prompt.image.shape == (10, 20, 3)
    assert prompt.boxes[0].track_id == "car"
    assert batch.request_id == source.request_id
    assert batch.batch_latency_ms == 30.0
    decoded = cv2.imdecode(
        np.frombuffer(batch.masks[0].mask_png, np.uint8), cv2.IMREAD_GRAYSCALE
    )
    assert decoded.dtype == np.uint8
    assert int((decoded > 0).sum()) == 48


def test_protocol_frame_to_prompt_filters_tracks_and_preserves_order():
    source = _wire_frame()
    second = replace(source.boxes[0], track_id="car-2", xyxy=(11.0, 2.0, 18.0, 8.0))
    source = replace(source, boxes=(source.boxes[0], second))

    prompt = protocol_frame_to_prompt(source, track_ids={"car-2"})

    assert [box.track_id for box in prompt.boxes] == ["car-2"]


class _FakeSocket:
    def __init__(self, incoming):
        self.incoming = incoming

    def poll(self, timeout=0):
        return bool(self.incoming)


class _FakeDealer:
    def __init__(self):
        self.incoming: list[Acknowledgement] = []
        self.socket = _FakeSocket(self.incoming)
        self.sent: list[ReconstructionRequest] = []

    def send(self, message, *, topic=None):
        self.sent.append(message)

    def receive(self, *, flags=0):
        return "acknowledgement", self.incoming.pop(0)


class _Collector:
    def __init__(self):
        self.items = []

    def send(self, message, *, topic=None):
        self.items.append(message)

    def append(self, record):
        self.items.append(record)


class _Heartbeat:
    def __init__(self):
        self.events = []

    def update(self, state, **values):
        self.events.append((state, values))


class _MaskWorker:
    def __init__(self):
        self.prompts = []
        self.starts = 0
        self.unloads = 0

    def start(self, *, warmup_iterations=3):
        self.starts += 1

    def unload(self):
        self.unloads += 1

    def process(self, prompt):
        self.prompts.append(prompt)
        mask = np.ones(prompt.image.shape[:2], dtype=bool)
        return SegmentationMaskBatch(
            frame_id=prompt.frame_id,
            timestamp_us=prompt.timestamp_us,
            masks={
                box.track_id: SegmentationMaskResult(
                    track_id=box.track_id,
                    mask=mask,
                    confidence=0.9,
                    in_box_fraction=1.0,
                    box_coverage=1.0,
                )
                for box in prompt.boxes
            },
            rejected_tracks={},
            metrics=MaskWorkerMetrics(
                backend="fake",
                model_load_seconds=0.0,
                warmup_seconds=0.0,
                image_encode_seconds=0.01,
                prompt_decode_seconds=0.02,
                total_seconds=0.03,
                prompts=len(prompt.boxes),
                accepted_masks=len(prompt.boxes),
            ),
        )


def _mask_service(client, worker):
    masks = _Collector()
    metrics = _Collector()
    return (
        MaskProcessService(
            worker=worker,
            frames=None,
            masks=masks,
            reconstructions=client,
            heartbeat=_Heartbeat(),
            metrics=metrics,
            admission_config=StableTrackAdmissionConfig(
                stable_seconds=0.0,
                minimum_projected_area_px=0.0,
            ),
        ),
        masks,
        metrics,
    )


def test_mask_service_skips_inference_when_all_tracks_are_handled():
    dealer = _FakeDealer()
    client = ReliableReconstructionClient(dealer)
    frame = _wire_frame()
    request = build_reconstruction_requests(
        frame,
        segmentation_batch_to_protocol(frame, _segmentation_batch()),
        client,
    )[0]
    assert client.enqueue(request)
    worker = _MaskWorker()
    service, masks, metrics = _mask_service(client, worker)

    service.process_frame(frame)

    assert worker.prompts == []
    assert masks.items[0].masks == ()
    assert metrics.items[0]["inference_skipped"] is True
    assert metrics.items[0]["candidate_prompts"] == 0
    assert metrics.items[0]["inference_ms"] == 0.0


def test_mask_service_only_prompts_unhandled_tracks():
    dealer = _FakeDealer()
    client = ReliableReconstructionClient(dealer)
    frame = _wire_frame()
    second = replace(frame.boxes[0], track_id="car-2", xyxy=(11.0, 2.0, 18.0, 8.0))
    frame = replace(frame, boxes=(frame.boxes[0], second))
    first_only = replace(frame, boxes=(frame.boxes[0],))
    request = build_reconstruction_requests(
        first_only,
        segmentation_batch_to_protocol(first_only, _segmentation_batch()),
        client,
    )[0]
    assert client.enqueue(request)
    worker = _MaskWorker()
    service, masks, metrics = _mask_service(client, worker)

    service.process_frame(frame)

    assert [[box.track_id for box in prompt.boxes] for prompt in worker.prompts] == [
        ["car-2"]
    ]
    assert [mask.track_id for mask in masks.items[0].masks] == ["car-2"]
    assert metrics.items[0]["candidate_prompts"] == 1
    assert metrics.items[0]["inference_skipped"] is False


def test_take_turn_mask_service_retains_wave_until_gpu_lease_is_available():
    class Lease:
        def __init__(self):
            self.available = False
            self.releases = 0

        def try_acquire(self):
            if not self.available:
                return None
            return SimpleNamespace(wait_seconds=0.25)

        def release(self):
            self.releases += 1

    dealer = _FakeDealer()
    client = ReliableReconstructionClient(dealer)
    worker = _MaskWorker()
    lease = Lease()
    masks = _Collector()
    metrics = _Collector()
    service = MaskProcessService(
        worker=worker,
        frames=None,
        masks=masks,
        reconstructions=client,
        heartbeat=_Heartbeat(),
        metrics=metrics,
        admission_config=StableTrackAdmissionConfig(
            stable_seconds=0.0,
            minimum_projected_area_px=0.0,
        ),
        residency_lease=lease,
        warmup_iterations=1,
    )
    frame = _wire_frame()

    service.process_frame(frame)
    assert worker.prompts == []
    assert metrics.items[-1]["skip_reason"] == "waiting_for_gpu_residency"

    lease.available = True
    processed = service._process_queued_take_turn_wave()

    assert processed is not None
    assert len(worker.prompts) == 1
    assert worker.starts == worker.unloads == lease.releases == 1
    assert len(dealer.sent) == 1
    assert any(item.get("event") == "model_loaded" for item in metrics.items)
    assert any(item.get("event") == "model_released" for item in metrics.items)


def test_take_turn_wave_remains_drainable_after_end_of_scene():
    class Lease:
        available = False

        def try_acquire(self):
            return SimpleNamespace(wait_seconds=0.1) if self.available else None

        def release(self):
            pass

    dealer = _FakeDealer()
    client = ReliableReconstructionClient(dealer)
    worker = _MaskWorker()
    lease = Lease()
    service = MaskProcessService(
        worker=worker,
        frames=None,
        masks=_Collector(),
        reconstructions=client,
        heartbeat=_Heartbeat(),
        metrics=_Collector(),
        admission_config=StableTrackAdmissionConfig(
            stable_seconds=0.0,
            minimum_projected_area_px=0.0,
        ),
        residency_lease=lease,
    )
    frame = _wire_frame()
    service.process_frame(frame)
    assert service._work_queue_depth() == 1

    service.process_frame(replace(frame, end_of_scene=True, boxes=()))
    assert service._work_queue_depth() == 1

    lease.available = True
    assert service._process_queued_take_turn_wave() is not None
    assert service._work_queue_depth() == 1  # awaiting reconstruction ACK


def test_reconstruction_requests_retry_until_ack_then_dedupe_track():
    now = [0.0]
    dealer = _FakeDealer()
    client = ReliableReconstructionClient(
        dealer, retry_seconds=1.0, monotonic=lambda: now[0]
    )
    frame = _wire_frame()
    masks = segmentation_batch_to_protocol(frame, _segmentation_batch())
    requests = build_reconstruction_requests(frame, masks, client, seed=7)

    assert len(requests) == 1
    assert client.enqueue(requests[0])
    assert dealer.sent[0].camera_intrinsic == frame.camera_intrinsic
    assert dealer.sent[0].seed == 7
    assert client.pending_count == 1

    now[0] = 1.1
    client.service()
    assert len(dealer.sent) == 2

    dealer.incoming.append(
        Acknowledgement(request_id=requests[0].request_id, accepted=True)
    )
    acknowledgements = client.service()
    assert acknowledgements[0].accepted
    assert client.pending_count == 0
    assert not build_reconstruction_requests(frame, masks, client)


def _admission_frame(
    timestamp_us: int,
    track_ids: tuple[str, ...],
    *,
    area: float = 1600.0,
    visibility: float = 0.8,
) -> FrameDetections:
    source = _wire_frame()
    side = area**0.5
    boxes = tuple(
        replace(
            source.boxes[0],
            track_id=track_id,
            xyxy=(50.0, 20.0, 50.0 + side, 20.0 + side),
            visibility=visibility,
        )
        for track_id in track_ids
    )
    return replace(
        source,
        frame_id=f"frame-{timestamp_us}",
        request_id=f"request-{timestamp_us}",
        timestamp_us=timestamp_us,
        boxes=boxes,
    )


def test_stable_admission_requires_continuous_two_second_visibility():
    admission = StableTrackAdmission(
        StableTrackAdmissionConfig(maximum_gap_seconds=1.0)
    )

    assert not admission.observe(_admission_frame(0, ("car",))).frames
    assert not admission.observe(_admission_frame(1_000_000, ("car",))).frames
    wave = admission.observe(_admission_frame(2_000_000, ("car",)))

    assert wave.admitted_track_ids == ("car",)
    assert wave.frames[0].timestamp_us == 0


def test_stable_admission_resets_on_missing_or_low_quality_observation():
    admission = StableTrackAdmission(
        StableTrackAdmissionConfig(stable_seconds=0.4, maximum_gap_seconds=0.25)
    )
    admission.observe(_admission_frame(0, ("car",)))
    missing = admission.observe(_admission_frame(100_000, ()))
    assert missing.reset_track_ids == ("car",)
    admission.observe(_admission_frame(200_000, ("car",)))
    low_visibility = admission.observe(
        _admission_frame(300_000, ("car",), visibility=0.4)
    )
    assert low_visibility.reset_track_ids == ("car",)
    admission.observe(_admission_frame(400_000, ("car",)))
    gap = admission.observe(_admission_frame(700_000, ("car",)))
    assert gap.reset_track_ids == ("car",)
    assert not admission.observe(_admission_frame(900_000, ("car",))).frames


def test_stable_admission_caps_and_deterministically_spills_next_wave():
    admission = StableTrackAdmission(
        StableTrackAdmissionConfig(
            stable_seconds=0.1,
            maximum_gap_seconds=1.0,
            maximum_admission_batch=5,
        )
    )
    tracks = tuple(f"car-{index}" for index in range(6))
    admission.observe(_admission_frame(0, tracks))
    first = admission.observe(_admission_frame(100_000, tracks))
    second = admission.observe(_admission_frame(200_000, tracks))

    assert first.admitted_track_ids == tracks[:5]
    assert first.pending_count == 1
    assert second.admitted_track_ids == tracks[5:]
    assert second.pending_count == 0


def test_pending_admission_is_dropped_when_track_leaves_view():
    admission = StableTrackAdmission(
        StableTrackAdmissionConfig(
            stable_seconds=0.1,
            maximum_gap_seconds=1.0,
            maximum_admission_batch=1,
        )
    )
    admission.observe(_admission_frame(0, ("car-a", "car-b")))
    first = admission.observe(_admission_frame(100_000, ("car-a", "car-b")))
    second = admission.observe(_admission_frame(200_000, ("car-a",)))

    assert first.admitted_track_ids == ("car-a",)
    assert second.admitted_track_ids == ()
    assert second.reset_track_ids == ("car-b",)


def test_stable_admission_groups_tracks_that_share_best_keyframe():
    admission = StableTrackAdmission(
        StableTrackAdmissionConfig(stable_seconds=0.2, maximum_gap_seconds=1.0)
    )
    admission.observe(_admission_frame(0, ("car-a", "car-b"), area=1600.0))
    admission.observe(_admission_frame(100_000, ("car-a", "car-b"), area=2500.0))
    wave = admission.observe(
        _admission_frame(200_000, ("car-a", "car-b"), area=1200.0)
    )

    assert len(wave.frames) == 1
    assert wave.frames[0].timestamp_us == 100_000
    assert tuple(box.track_id for box in wave.frames[0].boxes) == ("car-a", "car-b")


def test_metrics_writer_emits_valid_json_line(tmp_path):
    path = tmp_path / "metrics.jsonl"
    JsonlMetricsWriter(path).append({"event": "frame", "fps": 2.0})

    line = path.read_text(encoding="utf-8")
    assert json.loads(line)["event"] == "frame"
    assert line.endswith("\n")

"""Resident SAM 3.1 mask process for real-time box-prompted segmentation."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path
import signal
import time
from typing import Collection, Protocol

import cv2
import numpy as np
import zmq

from src.realtime.protocol import (
    Acknowledgement,
    BoxPrompt,
    FrameDetections,
    MaskBatch,
    MaskResult,
    ReconstructionRequest,
    WorkerState,
)
from src.realtime.runtime_support import HealthHeartbeat, JsonlMetricsWriter
from src.realtime.gpu_residency import GpuResidencyLease
from src.realtime.transport import (
    DealerTransport,
    LatestValueSubscriber,
    PublisherTransport,
)
from src.segmentation import (
    BoxPrompt as SegmentationBoxPrompt,
    FramePrompt,
    MaskWorker,
    OfflineMaskBackend,
    SAM3MaskBackend,
    SAM3MaskBackendConfig,
)
from src.segmentation.models import MaskBatch as SegmentationMaskBatch


class DealerLike(Protocol):
    socket: object

    def send(
        self, message: ReconstructionRequest, *, topic: str | None = None
    ) -> None: ...

    def receive(self, *, flags: int = 0) -> tuple[str, object]: ...


@dataclass
class _PendingRequest:
    request: ReconstructionRequest
    last_sent: float
    attempts: int


@dataclass(frozen=True)
class StableTrackAdmissionConfig:
    """Quality and dwell-time gates applied before expensive reconstruction."""

    stable_seconds: float = 2.0
    maximum_gap_seconds: float = 0.25
    maximum_admission_batch: int = 5
    minimum_projected_area_px: float = 1024.0
    minimum_visible_fraction: float = 0.5

    def __post_init__(self) -> None:
        if self.stable_seconds < 0:
            raise ValueError("stable_seconds must be non-negative")
        if self.maximum_gap_seconds <= 0:
            raise ValueError("maximum_gap_seconds must be positive")
        if self.maximum_admission_batch <= 0:
            raise ValueError("maximum_admission_batch must be positive")
        if self.minimum_projected_area_px < 0:
            raise ValueError("minimum_projected_area_px must be non-negative")
        if not 0.0 <= self.minimum_visible_fraction <= 1.0:
            raise ValueError("minimum_visible_fraction must be between zero and one")


@dataclass
class _StableTrack:
    first_timestamp_us: int
    last_timestamp_us: int
    best_score: float
    best_frame: FrameDetections
    pending: bool = False


@dataclass(frozen=True)
class AdmissionWave:
    """A bounded wave of keyframes grouped for shared image-mask inference."""

    frames: tuple[FrameDetections, ...]
    admitted_track_ids: tuple[str, ...]
    reset_track_ids: tuple[str, ...]
    pending_count: int


@dataclass(frozen=True)
class _QueuedAdmissionWave:
    enqueued_monotonic: float
    frames: tuple[FrameDetections, ...]
    track_ids: tuple[str, ...]


class StableTrackAdmission:
    """Admit continuously useful tracks while retaining their best keyframe."""

    def __init__(self, config: StableTrackAdmissionConfig | None = None) -> None:
        self.config = config or StableTrackAdmissionConfig()
        self._tracks: dict[tuple[str, str], _StableTrack] = {}

    def observe(
        self,
        frame: FrameDetections,
        *,
        handled_tracks: Collection[tuple[str, str]] = (),
    ) -> AdmissionWave:
        handled = set(handled_tracks)
        generation = frame.scene_generation
        visible = {box.track_id: box for box in frame.boxes}
        resets: list[str] = []

        if frame.end_of_scene:
            self._tracks = {
                key: state for key, state in self._tracks.items() if key[0] != generation
            }
            return AdmissionWave((), (), (), 0)

        # A missing observation breaks continuous visibility, including for an
        # eligible track waiting behind the per-frame admission cap. Rebuilding
        # a mesh for an object that has already left cannot improve the live view.
        for key, state in tuple(self._tracks.items()):
            if key[0] != generation or key[1] in visible:
                continue
            del self._tracks[key]
            resets.append(key[1])

        for track_id, box in visible.items():
            key = (generation, track_id)
            if key in handled:
                self._tracks.pop(key, None)
                continue
            area = _box_area(box)
            qualifies = (
                area >= self.config.minimum_projected_area_px
                and box.visibility >= self.config.minimum_visible_fraction
            )
            state = self._tracks.get(key)
            if not qualifies:
                if state is not None and not state.pending:
                    resets.append(track_id)
                    del self._tracks[key]
                continue

            score = _keyframe_quality(frame, box)
            gap_us = (
                frame.timestamp_us - state.last_timestamp_us if state is not None else 0
            )
            if state is None or (
                not state.pending
                and (gap_us < 0 or gap_us > self.config.maximum_gap_seconds * 1_000_000)
            ):
                if state is not None:
                    resets.append(track_id)
                state = _StableTrack(
                    first_timestamp_us=frame.timestamp_us,
                    last_timestamp_us=frame.timestamp_us,
                    best_score=score,
                    best_frame=replace(frame, boxes=(box,)),
                )
                self._tracks[key] = state
            elif not state.pending:
                state.last_timestamp_us = frame.timestamp_us
                if score > state.best_score:
                    state.best_score = score
                    state.best_frame = replace(frame, boxes=(box,))

            if (
                not state.pending
                and frame.timestamp_us - state.first_timestamp_us
                >= self.config.stable_seconds * 1_000_000
            ):
                state.pending = True

        candidates = [
            (key, state)
            for key, state in self._tracks.items()
            if key[0] == generation and state.pending and key not in handled
        ]
        candidates.sort(key=lambda item: (-item[1].best_score, item[0][1]))
        selected = candidates[: self.config.maximum_admission_batch]

        grouped: dict[tuple[str, int], list[tuple[str, _StableTrack]]] = {}
        for key, state in selected:
            group_key = (state.best_frame.frame_id, state.best_frame.timestamp_us)
            grouped.setdefault(group_key, []).append((key[1], state))

        admitted: list[str] = []
        frames: list[FrameDetections] = []
        for items in grouped.values():
            source = items[0][1].best_frame
            boxes = tuple(item.best_frame.boxes[0] for _, item in items)
            frames.append(replace(source, boxes=boxes))
            admitted.extend(track_id for track_id, _ in items)
        for key, _ in selected:
            del self._tracks[key]

        return AdmissionWave(
            frames=tuple(frames),
            admitted_track_ids=tuple(admitted),
            reset_track_ids=tuple(sorted(set(resets))),
            pending_count=len(candidates) - len(selected),
        )


def _box_area(box: BoxPrompt) -> float:
    x0, y0, x1, y1 = box.xyxy
    return (x1 - x0) * (y1 - y0)


def _keyframe_quality(frame: FrameDetections, box: BoxPrompt) -> float:
    """Prefer large, visible, confident boxes away from image boundaries."""

    x0, y0, x1, y1 = box.xyxy
    intrinsic = frame.camera_intrinsic
    image_width = max(1.0, 2.0 * intrinsic[2])
    image_height = max(1.0, 2.0 * intrinsic[5])
    clearance = max(0.0, min(x0, y0, image_width - x1, image_height - y1))
    border_scale = max(1.0, min(image_width, image_height) * 0.25)
    border_score = min(1.0, clearance / border_scale)
    return (
        _box_area(box)
        * box.visibility
        * box.detection_confidence
        * (0.5 + 0.5 * border_score)
    )


class ReliableReconstructionClient:
    """Retain new-track requests until the reconstruction worker acknowledges."""

    def __init__(
        self,
        transport: DealerLike,
        *,
        retry_seconds: float = 1.0,
        monotonic=time.monotonic,
    ) -> None:
        if retry_seconds <= 0:
            raise ValueError("retry_seconds must be positive")
        self.transport = transport
        self.retry_seconds = retry_seconds
        self._monotonic = monotonic
        self._pending: dict[str, _PendingRequest] = {}
        self._terminal_tracks: set[tuple[str, str]] = set()

    @property
    def pending_count(self) -> int:
        return len(self._pending)

    def has_track(self, scene_generation: str, track_id: str) -> bool:
        key = (scene_generation, track_id)
        return key in self._terminal_tracks or any(
            (item.request.scene_generation, item.request.track_id) == key
            for item in self._pending.values()
        )

    def enqueue(self, request: ReconstructionRequest) -> bool:
        if self.has_track(request.scene_generation, request.track_id):
            return False
        now = self._monotonic()
        self.transport.send(request)
        self._pending[request.request_id] = _PendingRequest(request, now, 1)
        return True

    def service(self) -> list[Acknowledgement]:
        acknowledgements: list[Acknowledgement] = []
        socket = self.transport.socket
        while bool(socket.poll(timeout=0)):
            _, message = self.transport.receive(flags=zmq.NOBLOCK)
            if not isinstance(message, Acknowledgement):
                continue
            pending = self._pending.pop(message.request_id, None)
            if pending is None:
                continue
            self._terminal_tracks.add(
                (pending.request.scene_generation, pending.request.track_id)
            )
            acknowledgements.append(message)
        now = self._monotonic()
        for request_id, item in tuple(self._pending.items()):
            if now - item.last_sent < self.retry_seconds:
                continue
            self.transport.send(item.request)
            self._pending[request_id] = _PendingRequest(
                item.request, now, item.attempts + 1
            )
        return acknowledgements


class OfflineMaskDirectory:
    """Load explicit fallback masks from ``ROOT/FRAME_ID/TRACK_ID.png``."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)

    def __call__(
        self, frame: FramePrompt, box: SegmentationBoxPrompt
    ) -> np.ndarray | None:
        frame_id = _safe_component(frame.frame_id)
        track_id = _safe_component(box.track_id)
        mask = cv2.imread(
            str(self.root / frame_id / f"{track_id}.png"),
            cv2.IMREAD_GRAYSCALE,
        )
        return None if mask is None else mask > 0


def protocol_frame_to_prompt(
    frame: FrameDetections,
    *,
    track_ids: Collection[str] | None = None,
) -> FramePrompt:
    encoded = np.frombuffer(frame.image_jpeg, dtype=np.uint8)
    image_bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise ValueError(f"invalid JPEG for frame {frame.frame_id}")
    return FramePrompt(
        frame_id=frame.frame_id,
        timestamp_us=frame.timestamp_us,
        image=cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB),
        boxes=tuple(
            SegmentationBoxPrompt(box.track_id, box.xyxy)
            for box in frame.boxes
            if track_ids is None or box.track_id in track_ids
        ),
    )


def segmentation_batch_to_protocol(
    source: FrameDetections,
    batch: SegmentationMaskBatch,
) -> MaskBatch:
    latency_ms = (
        batch.metrics.prompt_decode_seconds * 1_000.0 / max(1, batch.metrics.prompts)
    )
    masks: list[MaskResult] = []
    for track_id, result in batch.masks.items():
        success, encoded = cv2.imencode(
            ".png", np.asarray(result.mask, dtype=np.uint8) * 255
        )
        if not success:
            raise RuntimeError(f"failed to encode mask for track {track_id}")
        masks.append(
            MaskResult(
                track_id=track_id,
                confidence=result.confidence,
                coverage=float(result.mask.mean()),
                box_overlap=result.in_box_fraction,
                latency_ms=latency_ms,
                mask_png=encoded.tobytes(),
            )
        )
    return MaskBatch(
        frame_id=source.frame_id,
        request_id=source.request_id,
        scene_generation=source.scene_generation,
        timestamp_us=source.timestamp_us,
        masks=tuple(masks),
        batch_latency_ms=batch.metrics.total_seconds * 1_000.0,
    )


def build_reconstruction_requests(
    frame: FrameDetections,
    masks: MaskBatch,
    client: ReliableReconstructionClient,
    *,
    seed: int = 0,
) -> tuple[ReconstructionRequest, ...]:
    boxes = {box.track_id: box for box in frame.boxes}
    requests: list[ReconstructionRequest] = []
    for mask in masks.masks:
        box = boxes.get(mask.track_id)
        if box is None or client.has_track(frame.scene_generation, mask.track_id):
            continue
        width = box.xyxy[2] - box.xyxy[0]
        height = box.xyxy[3] - box.xyxy[1]
        quality_score = (
            width
            * height
            * box.visibility
            * box.detection_confidence
            * mask.confidence
            * mask.box_overlap
        )
        requests.append(
            ReconstructionRequest(
                request_id=f"{frame.scene_generation}:{mask.track_id}",
                scene_generation=frame.scene_generation,
                track_id=mask.track_id,
                frame_id=frame.frame_id,
                timestamp_us=frame.timestamp_us,
                image_jpeg=frame.image_jpeg,
                mask_png=mask.mask_png,
                camera_intrinsic=frame.camera_intrinsic,
                dimensions_lwh=box.dimensions_lwh,
                world_T_object=box.world_T_object,
                world_T_camera=frame.world_T_camera,
                quality_score=quality_score,
                seed=seed,
            )
        )
    return tuple(requests)


class MaskProcessService:
    """Adapt the transport-neutral mask worker to real-time ZMQ contracts."""

    def __init__(
        self,
        *,
        worker: MaskWorker,
        frames: LatestValueSubscriber,
        masks: PublisherTransport,
        reconstructions: ReliableReconstructionClient,
        heartbeat: HealthHeartbeat,
        metrics: JsonlMetricsWriter,
        seed: int = 0,
        mask_policy: str = "new_tracks",
        admission_config: StableTrackAdmissionConfig | None = None,
        residency_lease: GpuResidencyLease | None = None,
        warmup_iterations: int = 3,
    ) -> None:
        if mask_policy not in {"new_tracks", "all_visible"}:
            raise ValueError("mask_policy must be new_tracks or all_visible")
        self.worker = worker
        self.frames = frames
        self.masks = masks
        self.reconstructions = reconstructions
        self.heartbeat = heartbeat
        self.metrics = metrics
        self.seed = seed
        self.mask_policy = mask_policy
        self.admission = StableTrackAdmission(admission_config)
        self.residency_lease = residency_lease
        self.warmup_iterations = warmup_iterations
        self._queued_waves: deque[_QueuedAdmissionWave] = deque()
        self._stop = False
        self._received_frames = 0
        self._inference_frames = 0
        self._run_started = 0.0

    def request_stop(self, *_: object) -> None:
        self._stop = True

    def start(self, *, warmup_iterations: int = 3) -> None:
        self.warmup_iterations = warmup_iterations
        if self.residency_lease is not None:
            self.metrics.append(
                {
                    "event": "model_idle",
                    "runtime_mode": "take_turns",
                    "stable_seconds": self.admission.config.stable_seconds,
                    "maximum_admission_batch": self.admission.config.maximum_admission_batch,
                }
            )
            self.heartbeat.update(WorkerState.READY)
            return
        self.heartbeat.update(WorkerState.LOADING)
        started = time.perf_counter()
        self.worker.start(warmup_iterations=warmup_iterations)
        self.metrics.append(
            {
                "event": "model_ready",
                "startup_seconds": time.perf_counter() - started,
                "warmup_iterations": warmup_iterations,
                "mask_policy": self.mask_policy,
                "stable_seconds": self.admission.config.stable_seconds,
                "maximum_gap_seconds": self.admission.config.maximum_gap_seconds,
                "maximum_admission_batch": (
                    self.admission.config.maximum_admission_batch
                ),
                "minimum_projected_area_px": (
                    self.admission.config.minimum_projected_area_px
                ),
                "minimum_visible_fraction": (
                    self.admission.config.minimum_visible_fraction
                ),
                **getattr(
                    getattr(self.worker, "backend", None),
                    "model_metrics",
                    {},
                ),
            }
        )
        self.heartbeat.update(WorkerState.READY)

    def run(self) -> None:
        self._run_started = time.perf_counter()
        while not self._stop:
            acknowledgements = self.reconstructions.service()
            for acknowledgement in acknowledgements:
                self.metrics.append(
                    {
                        "event": "reconstruction_acknowledgement",
                        "request_id": acknowledgement.request_id,
                        "accepted": acknowledgement.accepted,
                        "detail": acknowledgement.detail,
                    }
                )
            if not self.frames.socket.poll(timeout=100):
                processed = self._process_queued_take_turn_wave()
                if processed is not None:
                    _batches, segmentations, submitted, wait_seconds, queue_seconds = processed
                    self.metrics.append(
                        {
                            "event": "admission_wave_processed",
                            "runtime_mode": "take_turns",
                            "prompts": sum(item.metrics.prompts for item in segmentations),
                            "reconstruction_requests_submitted": submitted,
                            "residency_wait_seconds": wait_seconds,
                            "admission_queue_seconds": queue_seconds,
                            "admission_waves_queued": len(self._queued_waves),
                        }
                    )
                self.heartbeat.update(
                    WorkerState.READY,
                    queue_depth=self._work_queue_depth(),
                )
                continue
            _, message, discarded = self.frames.receive_latest()
            if not isinstance(message, FrameDetections):
                continue
            self.process_frame(message, discarded_input_frames=discarded)

    def process_frame(
        self, message: FrameDetections, *, discarded_input_frames: int = 0
    ) -> None:
        """Mask only tracks that can still generate a new reconstruction request."""

        if self._run_started == 0.0:
            self._run_started = time.perf_counter()
        frame_started = time.perf_counter()
        self.heartbeat.update(
            WorkerState.BUSY,
            queue_depth=self.reconstructions.pending_count,
            detail=message.frame_id,
        )
        handled = {
            (message.scene_generation, box.track_id)
            for box in message.boxes
            if self.reconstructions.has_track(message.scene_generation, box.track_id)
        }
        wave = self.admission.observe(message, handled_tracks=handled)
        self._prune_queued_waves(message)
        if wave.frames and self.residency_lease is not None:
            self._queued_waves.append(
                _QueuedAdmissionWave(
                    enqueued_monotonic=time.monotonic(),
                    frames=wave.frames,
                    track_ids=wave.admitted_track_ids,
                )
            )
        selected_frames = wave.frames
        residency_wait_seconds: float | None = None
        queued_wait_seconds: float | None = None
        if self.residency_lease is not None:
            selected_frames = ()
            claimed = self._claim_queued_wave()
            if claimed is not None:
                selected_frames, residency_wait_seconds, queued_wait_seconds = claimed
        batches, segmentations, submitted = self._run_selected_frames(
            selected_frames,
            residency_wait_seconds=residency_wait_seconds,
        )
        if not batches:
            batch = MaskBatch(
                frame_id=message.frame_id,
                request_id=message.request_id,
                scene_generation=message.scene_generation,
                timestamp_us=message.timestamp_us,
                masks=(),
                batch_latency_ms=0.0,
            )
            batches.append(batch)
            self.masks.send(batch)
        self._received_frames += 1
        elapsed = time.perf_counter() - frame_started
        inference_metrics = [item.metrics for item in segmentations]
        self.metrics.append(
            {
                "event": "mask_batch",
                "frame_id": message.frame_id,
                "scene_generation": message.scene_generation,
                "end_of_scene": message.end_of_scene,
                "discarded_input_frames": discarded_input_frames,
                "input_boxes": len(message.boxes),
                "candidate_prompts": len(wave.admitted_track_ids),
                "admission_waves_queued": len(self._queued_waves),
                "residency_wait_seconds": residency_wait_seconds,
                "admission_queue_seconds": queued_wait_seconds,
                "prompts": sum(item.prompts for item in inference_metrics),
                "accepted_masks": sum(item.accepted_masks for item in inference_metrics),
                "rejected_masks": sum(
                    len(item.rejected_tracks) for item in segmentations
                ),
                "image_encode_ms": sum(
                    item.image_encode_seconds * 1_000.0 for item in inference_metrics
                ),
                "prompt_decode_ms": sum(
                    item.prompt_decode_seconds * 1_000.0 for item in inference_metrics
                ),
                "inference_ms": sum(
                    item.total_seconds * 1_000.0 for item in inference_metrics
                ),
                "inference_skipped": not segmentations,
                "skip_reason": (
                    "waiting_for_gpu_residency"
                    if not segmentations and self._queued_waves
                    else "awaiting_stability" if not segmentations else None
                ),
                "stable_tracks_admitted": list(wave.admitted_track_ids),
                "stability_resets": list(wave.reset_track_ids),
                "admission_pending": wave.pending_count,
                "admission_groups": len(wave.frames),
                "process_end_to_end_ms": elapsed * 1_000.0,
                "effective_fps": (
                    1.0 / elapsed
                    if segmentations and elapsed > 0
                    else 0.0
                ),
                "cumulative_fps": self._inference_frames
                / max(time.perf_counter() - self._run_started, 1e-9),
                "cumulative_input_fps": self._received_frames
                / max(time.perf_counter() - self._run_started, 1e-9),
                "gpu_peak_allocated_mib": (
                    max(
                        (
                            item.gpu_peak_allocated_mib
                            for item in inference_metrics
                            if item.gpu_peak_allocated_mib is not None
                        ),
                        default=None,
                    )
                ),
                "gpu_peak_reserved_mib": (
                    max(
                        (
                            item.gpu_peak_reserved_mib
                            for item in inference_metrics
                            if item.gpu_peak_reserved_mib is not None
                        ),
                        default=None,
                    )
                ),
                "reconstruction_requests_submitted": submitted,
                "reconstruction_requests_pending": self.reconstructions.pending_count,
            }
        )
        self.heartbeat.update(
            WorkerState.READY,
            queue_depth=self._work_queue_depth(),
        )

    def _work_queue_depth(self) -> int:
        return self.reconstructions.pending_count + sum(
            len(wave.track_ids) for wave in self._queued_waves
        )

    def _claim_queued_wave(
        self,
    ) -> tuple[tuple[FrameDetections, ...], float, float] | None:
        if self.residency_lease is None or not self._queued_waves:
            return None
        acquisition = self.residency_lease.try_acquire()
        if acquisition is None:
            return None
        queued = self._queued_waves.popleft()
        queue_seconds = time.monotonic() - queued.enqueued_monotonic
        self.metrics.append(
            {
                "event": "residency_acquired",
                "worker_id": "sam3-mask",
                "runtime_mode": "take_turns",
                "wait_seconds": acquisition.wait_seconds,
                "admission_queue_seconds": queue_seconds,
                "tracks": list(queued.track_ids),
            }
        )
        return queued.frames, acquisition.wait_seconds, queue_seconds

    def _process_queued_take_turn_wave(
        self,
    ) -> tuple[
        list[MaskBatch], list[SegmentationMaskBatch], int, float, float
    ] | None:
        claimed = self._claim_queued_wave()
        if claimed is None:
            return None
        frames, wait_seconds, queue_seconds = claimed
        batches, segmentations, submitted = self._run_selected_frames(
            frames, residency_wait_seconds=wait_seconds
        )
        return batches, segmentations, submitted, wait_seconds, queue_seconds

    def _run_selected_frames(
        self,
        frames: tuple[FrameDetections, ...],
        *,
        residency_wait_seconds: float | None,
    ) -> tuple[list[MaskBatch], list[SegmentationMaskBatch], int]:
        batches: list[MaskBatch] = []
        segmentations: list[SegmentationMaskBatch] = []
        submitted = 0
        try:
            if frames and self.residency_lease is not None:
                load_started = time.perf_counter()
                self.heartbeat.update(WorkerState.LOADING)
                transfer_metrics = self.worker.start(
                    warmup_iterations=self.warmup_iterations
                )
                self.metrics.append(
                    {
                        "event": "model_loaded",
                        "worker_id": "sam3-mask",
                        "runtime_mode": "take_turns",
                        "load_seconds": time.perf_counter() - load_started,
                        "residency_wait_seconds": residency_wait_seconds,
                        **(transfer_metrics or {}),
                        **getattr(
                            getattr(self.worker, "backend", None),
                            "model_metrics",
                            {},
                        ),
                    }
                )
            for source in frames:
                prompt = protocol_frame_to_prompt(source)
                segmentation = self.worker.process(prompt)
                batch = segmentation_batch_to_protocol(source, segmentation)
                segmentations.append(segmentation)
                batches.append(batch)
                self.masks.send(batch)
                submitted += sum(
                    self.reconstructions.enqueue(request)
                    for request in build_reconstruction_requests(
                        source, batch, self.reconstructions, seed=self.seed
                    )
                )
                self._inference_frames += 1
        finally:
            if frames and self.residency_lease is not None:
                unload_started = time.perf_counter()
                transfer_metrics = self.worker.unload()
                self.metrics.append(
                    {
                        "event": "model_released",
                        "worker_id": "sam3-mask",
                        "runtime_mode": "take_turns",
                        "unload_seconds": time.perf_counter() - unload_started,
                        **(transfer_metrics or {}),
                    }
                )
                self.residency_lease.release()
        return batches, segmentations, submitted

    def _prune_queued_waves(self, frame: FrameDetections) -> None:
        """Do not spend a later GPU turn on tracks that have left the live view."""

        if (
            self.residency_lease is None
            or not self._queued_waves
            or frame.end_of_scene
        ):
            return
        visible = {box.track_id for box in frame.boxes}
        retained: deque[_QueuedAdmissionWave] = deque()
        for wave in self._queued_waves:
            frames: list[FrameDetections] = []
            track_ids: list[str] = []
            for source in wave.frames:
                boxes = tuple(box for box in source.boxes if box.track_id in visible)
                if boxes and not frame.end_of_scene:
                    frames.append(replace(source, boxes=boxes))
                    track_ids.extend(box.track_id for box in boxes)
            if frames:
                retained.append(replace(wave, frames=tuple(frames), track_ids=tuple(track_ids)))
            else:
                self.metrics.append(
                    {
                        "event": "admission_wave_dropped",
                        "reason": "tracks_left_view",
                        "tracks": list(wave.track_ids),
                    }
                )
        self._queued_waves = retained


def _safe_component(value: str) -> str:
    if not value or Path(value).name != value or value in {".", ".."}:
        raise ValueError("frame and track IDs must be single safe path components")
    return value


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames-endpoint", default="tcp://127.0.0.1:5555")
    parser.add_argument("--masks-endpoint", default="tcp://127.0.0.1:5556")
    parser.add_argument("--reconstruction-endpoint", default="tcp://127.0.0.1:5557")
    parser.add_argument("--health-endpoint", default="tcp://127.0.0.1:5559")
    parser.add_argument(
        "--metrics-path",
        type=Path,
        default=Path("artifacts/realtime/metrics/sam3-mask.jsonl"),
    )
    parser.add_argument("--backend", choices=("sam3", "offline"), default="sam3")
    parser.add_argument("--offline-mask-root", type=Path)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--compile", action="store_true")
    parser.add_argument(
        "--precision", choices=("default", "fp16", "nf4"), default="default"
    )
    parser.add_argument(
        "--prompt-mode",
        choices=("serial_grounding", "interactive_batch"),
        default="serial_grounding",
    )
    parser.add_argument(
        "--mask-policy",
        choices=("new_tracks", "all_visible"),
        default="new_tracks",
    )
    parser.add_argument("--confidence-threshold", type=float, default=0.5)
    parser.add_argument("--minimum-mask-in-box-fraction", type=float, default=0.5)
    parser.add_argument("--minimum-box-coverage", type=float, default=0.05)
    parser.add_argument("--warmup-iterations", type=int, default=3)
    parser.add_argument("--retry-seconds", type=float, default=1.0)
    parser.add_argument("--stable-seconds", type=float, default=2.0)
    parser.add_argument("--maximum-gap-seconds", type=float, default=0.25)
    parser.add_argument("--maximum-admission-batch", type=int, default=5)
    parser.add_argument("--minimum-projected-area-px", type=float, default=1024.0)
    parser.add_argument("--minimum-visible-fraction", type=float, default=0.5)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--gpu-residency-lock",
        type=Path,
        help="enable lazy take-turn residency using this cross-process lock",
    )
    return parser


def _build_worker(args: argparse.Namespace) -> MaskWorker:
    if args.backend == "offline":
        if args.offline_mask_root is None:
            raise ValueError("--offline-mask-root is required for the offline backend")
        return MaskWorker(OfflineMaskBackend(OfflineMaskDirectory(args.offline_mask_root)))
    return MaskWorker(
        SAM3MaskBackend(
            SAM3MaskBackendConfig(
                checkpoint=args.checkpoint,
                device=args.device,
                confidence_threshold=args.confidence_threshold,
                minimum_mask_in_box_fraction=args.minimum_mask_in_box_fraction,
                minimum_box_coverage=args.minimum_box_coverage,
                compile=args.compile,
                precision=args.precision,
                prompt_mode=args.prompt_mode,
            )
        )
    )


def main() -> None:
    args = _parser().parse_args()
    context = zmq.Context()
    frames = LatestValueSubscriber.open(
        context,
        args.frames_endpoint,
        bind=False,
        topics=("frame_detections",),
        high_water_mark=2,
    )
    masks = PublisherTransport.open(
        context, args.masks_endpoint, bind=True, high_water_mark=10
    )
    dealer = DealerTransport.open(
        context,
        args.reconstruction_endpoint,
        identity="sam3-mask",
        high_water_mark=100,
    )
    client = ReliableReconstructionClient(
        dealer, retry_seconds=args.retry_seconds
    )
    heartbeat = HealthHeartbeat(
        endpoint=args.health_endpoint,
        worker_id="sam3-mask",
        device=args.device if args.backend == "sam3" else "cpu",
    )
    service = MaskProcessService(
        worker=_build_worker(args),
        frames=frames,
        masks=masks,
        reconstructions=client,
        heartbeat=heartbeat,
        metrics=JsonlMetricsWriter(args.metrics_path),
        seed=args.seed,
        mask_policy=args.mask_policy,
        admission_config=StableTrackAdmissionConfig(
            stable_seconds=args.stable_seconds,
            maximum_gap_seconds=args.maximum_gap_seconds,
            maximum_admission_batch=args.maximum_admission_batch,
            minimum_projected_area_px=args.minimum_projected_area_px,
            minimum_visible_fraction=args.minimum_visible_fraction,
        ),
        residency_lease=(
            GpuResidencyLease(args.gpu_residency_lock, owner="sam3-mask")
            if args.gpu_residency_lock is not None and args.backend == "sam3"
            else None
        ),
        warmup_iterations=args.warmup_iterations,
    )
    signal.signal(signal.SIGINT, service.request_stop)
    signal.signal(signal.SIGTERM, service.request_stop)
    heartbeat.start()
    try:
        service.start(warmup_iterations=args.warmup_iterations)
        service.run()
    except Exception as exc:
        heartbeat.update(WorkerState.FAILED, detail=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        if service.residency_lease is not None:
            service.residency_lease.close()
        heartbeat.stop()
        frames.close()
        masks.close()
        dealer.close()
        context.term()


if __name__ == "__main__":
    main()

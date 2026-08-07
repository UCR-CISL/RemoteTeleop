"""Resident SAM 3.1 mask process for real-time box-prompted segmentation."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
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
        self._stop = False
        self._received_frames = 0
        self._inference_frames = 0
        self._run_started = 0.0

    def request_stop(self, *_: object) -> None:
        self._stop = True

    def start(self, *, warmup_iterations: int = 3) -> None:
        self.heartbeat.update(WorkerState.LOADING)
        started = time.perf_counter()
        self.worker.start(warmup_iterations=warmup_iterations)
        self.metrics.append(
            {
                "event": "model_ready",
                "startup_seconds": time.perf_counter() - started,
                "warmup_iterations": warmup_iterations,
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
        if self.mask_policy == "all_visible":
            candidate_track_ids = {box.track_id for box in message.boxes}
        else:
            candidate_track_ids = {
                box.track_id
                for box in message.boxes
                if not self.reconstructions.has_track(
                    message.scene_generation, box.track_id
                )
            }
        if message.end_of_scene:
            candidate_track_ids.clear()

        segmentation: SegmentationMaskBatch | None = None
        if candidate_track_ids:
            prompt = protocol_frame_to_prompt(message, track_ids=candidate_track_ids)
            segmentation = self.worker.process(prompt)
            batch = segmentation_batch_to_protocol(message, segmentation)
            self._inference_frames += 1
        else:
            batch = MaskBatch(
                frame_id=message.frame_id,
                request_id=message.request_id,
                scene_generation=message.scene_generation,
                timestamp_us=message.timestamp_us,
                masks=(),
                batch_latency_ms=0.0,
            )
        self.masks.send(batch)
        new_requests = build_reconstruction_requests(
            message, batch, self.reconstructions, seed=self.seed
        )
        submitted = sum(
            self.reconstructions.enqueue(request) for request in new_requests
        )
        self._received_frames += 1
        elapsed = time.perf_counter() - frame_started
        metrics = segmentation.metrics if segmentation is not None else None
        self.metrics.append(
            {
                "event": "mask_batch",
                "frame_id": message.frame_id,
                "scene_generation": message.scene_generation,
                "end_of_scene": message.end_of_scene,
                "discarded_input_frames": discarded_input_frames,
                "input_boxes": len(message.boxes),
                "candidate_prompts": len(candidate_track_ids),
                "prompts": metrics.prompts if metrics is not None else 0,
                "accepted_masks": metrics.accepted_masks if metrics is not None else 0,
                "rejected_masks": (
                    len(segmentation.rejected_tracks) if segmentation is not None else 0
                ),
                "image_encode_ms": (
                    metrics.image_encode_seconds * 1_000.0 if metrics is not None else 0.0
                ),
                "prompt_decode_ms": (
                    metrics.prompt_decode_seconds * 1_000.0 if metrics is not None else 0.0
                ),
                "inference_ms": (
                    metrics.total_seconds * 1_000.0 if metrics is not None else 0.0
                ),
                "inference_skipped": segmentation is None,
                "skip_reason": "no_new_tracks" if segmentation is None else None,
                "process_end_to_end_ms": elapsed * 1_000.0,
                "effective_fps": (
                    1.0 / elapsed
                    if segmentation is not None and elapsed > 0
                    else 0.0
                ),
                "cumulative_fps": self._inference_frames
                / max(time.perf_counter() - self._run_started, 1e-9),
                "cumulative_input_fps": self._received_frames
                / max(time.perf_counter() - self._run_started, 1e-9),
                "gpu_peak_allocated_mib": (
                    metrics.gpu_peak_allocated_mib if metrics is not None else None
                ),
                "gpu_peak_reserved_mib": (
                    metrics.gpu_peak_reserved_mib if metrics is not None else None
                ),
                "reconstruction_requests_submitted": submitted,
                "reconstruction_requests_pending": self.reconstructions.pending_count,
            }
        )
        self.heartbeat.update(
            WorkerState.READY,
            queue_depth=self.reconstructions.pending_count,
        )


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
    parser.add_argument("--precision", choices=("default", "fp16"), default="default")
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
    parser.add_argument("--seed", type=int, default=0)
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
        heartbeat.stop()
        frames.close()
        masks.close()
        dealer.close()
        context.term()


if __name__ == "__main__":
    main()

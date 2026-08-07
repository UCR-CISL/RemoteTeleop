"""Reliable, process-isolated SAM 3D Objects reconstruction worker."""

from __future__ import annotations

import argparse
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
import heapq
import json
import os
from pathlib import Path
import re
import time
from typing import Callable

import cv2
import numpy as np
import zmq

from src.common.coordinates import Pose3D
from src.common.models import CameraObservation
from src.realtime.protocol import (
    Acknowledgement,
    AssetEvent,
    AssetState,
    FrameDetections,
    ReconstructionRequest,
    WorkerHealth,
    WorkerState,
)
from src.realtime.transport import (
    LatestValueSubscriber,
    PublisherTransport,
    RouterTransport,
)
from src.reconstruction.mesh_alignment import MeshAligner
from src.reconstruction.models import ReconstructionJob, VehicleDimensions
from src.reconstruction.sam3d import ObjectReconstructor, SAM3DObjectReconstructor
from src.reconstruction.sam3d_upstream import UpstreamSAM3DBackend


_SAFE_COMPONENT = re.compile(r"[^A-Za-z0-9_.-]+")


@dataclass(frozen=True)
class QueuedRequest:
    request: ReconstructionRequest
    enqueued_monotonic: float


@dataclass
class TrackVisibility:
    """Compact image-space history used for deadline-aware queue ordering."""

    first_timestamp_us: int
    latest_timestamp_us: int
    margin: float
    first_seen_monotonic: float
    previous_timestamp_us: int | None = None
    previous_margin: float | None = None

    def update(self, timestamp_us: int, margin: float) -> None:
        if timestamp_us <= self.latest_timestamp_us:
            return
        self.previous_timestamp_us = self.latest_timestamp_us
        self.previous_margin = self.margin
        self.latest_timestamp_us = timestamp_us
        self.margin = margin

    @property
    def observed_seconds(self) -> float:
        return (self.latest_timestamp_us - self.first_timestamp_us) / 1_000_000.0

    @property
    def estimated_remaining_seconds(self) -> float:
        if self.previous_timestamp_us is None or self.previous_margin is None:
            return self.margin * 10.0
        elapsed = (self.latest_timestamp_us - self.previous_timestamp_us) / 1_000_000.0
        if elapsed <= 0:
            return self.margin * 10.0
        margin_rate = (self.margin - self.previous_margin) / elapsed
        if margin_rate >= -1e-6:
            return 30.0
        return min(30.0, max(0.0, self.margin / -margin_rate))


class SAM3DReconstructionService:
    """Own one resident model, priority queue, and metric alignment stage."""

    def __init__(
        self,
        *,
        reconstructor: ObjectReconstructor,
        output_root: Path,
        aligner: MeshAligner | None = None,
        clock: Callable[[], float] = time.perf_counter,
        priority_policy: str = "quality",
        coalesce_seconds: float = 0.0,
    ) -> None:
        if priority_policy not in {"quality", "deadline"}:
            raise ValueError("priority_policy must be quality or deadline")
        if coalesce_seconds < 0:
            raise ValueError("coalesce_seconds must be non-negative")
        self._reconstructor = reconstructor
        self._output_root = Path(output_root)
        self._aligner = aligner or MeshAligner()
        self._clock = clock
        self._priority_policy = priority_policy
        self._coalesce_seconds = coalesce_seconds
        self._queue: list[tuple[float, int, QueuedRequest]] = []
        self._known_tracks: set[tuple[str, str]] = set()
        self._active_tracks: dict[str, set[str]] = {}
        self._latest_visibility_us: dict[str, int] = {}
        self._current_generation: str | None = None
        self._retired_generations: set[str] = set()
        self._visibility: dict[tuple[str, str], TrackVisibility] = {}
        self._sequence = 0

    def load(self) -> None:
        """Load model state once, before the transport accepts requests."""

        load = getattr(self._reconstructor, "load", None)
        if load is not None:
            load()

    @property
    def queue_depth(self) -> int:
        return len(self._queue)

    def enqueue(
        self, request: ReconstructionRequest
    ) -> tuple[Acknowledgement, AssetEvent | None]:
        """Idempotently accept a request and queue higher quality first."""

        key = (request.scene_generation, request.track_id)
        if key in self._known_tracks:
            return (
                Acknowledgement(
                    request_id=request.request_id,
                    accepted=True,
                    detail="track already queued or reconstructed",
                ),
                None,
            )
        self._known_tracks.add(key)
        if self._request_status(request) == "stale":
            return (
                Acknowledgement(
                    request_id=request.request_id,
                    accepted=True,
                    detail="track already expired before reconstruction admission",
                ),
                _asset_event(
                    request,
                    AssetState.CANCELLED,
                    error="track exited before reconstruction admission",
                    metrics={
                        "deadline_hit": False,
                        "stale_job_cancelled": True,
                        "cancelled_before_enqueue": True,
                        "track_exit_timestamp_us": self._latest_visibility_us.get(
                            request.scene_generation
                        ),
                    },
                ),
            )
        queued = QueuedRequest(request=request, enqueued_monotonic=self._clock())
        heapq.heappush(self._queue, (-request.quality_score, self._sequence, queued))
        self._sequence += 1
        return (
            Acknowledgement(request_id=request.request_id, accepted=True),
            _asset_event(request, AssetState.QUEUED),
        )

    def pop_next(self) -> QueuedRequest | None:
        now = self._clock()
        candidates = [
            (index, item)
            for index, item in enumerate(self._queue)
            if self._request_status(item[2].request) == "current"
            and now - item[2].enqueued_monotonic >= self._coalesce_seconds
        ]
        if not candidates:
            return None
        selected_index, selected_item = max(
            candidates,
            key=lambda indexed: self._dispatch_priority(indexed[1]),
        )
        self._queue.pop(selected_index)
        heapq.heapify(self._queue)
        return selected_item[2]

    def update_visibility(self, frame: FrameDetections) -> tuple[AssetEvent, ...]:
        """Cancel queued work immediately when its track leaves the live snapshot."""

        if frame.scene_generation in self._retired_generations:
            return ()
        latest_timestamp_us = self._latest_visibility_us.get(frame.scene_generation)
        if latest_timestamp_us is not None and frame.timestamp_us < latest_timestamp_us:
            return ()
        previous_generation = self._current_generation
        self._current_generation = frame.scene_generation
        if (
            previous_generation is not None
            and previous_generation != frame.scene_generation
        ):
            self._active_tracks[previous_generation] = set()
            self._retired_generations.add(previous_generation)
        active = {box.track_id for box in frame.boxes}
        if frame.end_of_scene:
            active.clear()
            self._retired_generations.add(frame.scene_generation)
        self._active_tracks[frame.scene_generation] = active
        self._latest_visibility_us[frame.scene_generation] = frame.timestamp_us
        image_width = max(1.0, frame.camera_intrinsic[2] * 2.0)
        image_height = max(1.0, frame.camera_intrinsic[5] * 2.0)
        for box in frame.boxes:
            key = (frame.scene_generation, box.track_id)
            x0, y0, x1, y1 = box.xyxy
            margin = max(
                0.0,
                min(
                    x0 / image_width,
                    y0 / image_height,
                    (image_width - x1) / image_width,
                    (image_height - y1) / image_height,
                ),
            )
            history = self._visibility.get(key)
            if history is None:
                self._visibility[key] = TrackVisibility(
                    first_timestamp_us=frame.timestamp_us,
                    latest_timestamp_us=frame.timestamp_us,
                    margin=margin,
                    first_seen_monotonic=self._clock(),
                )
            else:
                history.update(frame.timestamp_us, margin)
        retained: list[tuple[float, int, QueuedRequest]] = []
        cancelled: list[AssetEvent] = []
        while self._queue:
            priority, sequence, queued = heapq.heappop(self._queue)
            request = queued.request
            if (
                request.scene_generation == frame.scene_generation
                and request.track_id in active
            ):
                retained.append((priority, sequence, queued))
                continue
            retired_generation = (
                request.scene_generation != frame.scene_generation or frame.end_of_scene
            )
            exit_timestamp_us = (
                self._latest_visibility_us.get(request.scene_generation)
                if retired_generation
                else frame.timestamp_us
            )
            cancelled.append(
                _asset_event(
                    request,
                    AssetState.CANCELLED,
                    error=(
                        "scene generation retired before reconstruction started"
                        if retired_generation
                        else "track exited before reconstruction started"
                    ),
                    metrics={
                        "deadline_hit": False,
                        "stale_job_cancelled": True,
                        "scene_generation_retired": retired_generation,
                        "track_exit_timestamp_us": exit_timestamp_us,
                        "visible_lifetime_ms": (
                            max(0, exit_timestamp_us - request.timestamp_us) / 1_000.0
                            if exit_timestamp_us is not None
                            else None
                        ),
                        "queue_latency_ms": (
                            self._clock() - queued.enqueued_monotonic
                        )
                        * 1_000.0,
                    },
                )
            )
        self._queue = retained
        heapq.heapify(self._queue)
        return tuple(cancelled)

    def is_active(self, scene_generation: str, track_id: str) -> bool:
        """Treat unseen generations as active until their first snapshot arrives."""

        if (
            scene_generation in self._retired_generations
            or (
                self._current_generation is not None
                and scene_generation != self._current_generation
            )
        ):
            return False
        active = self._active_tracks.get(scene_generation)
        return active is None or track_id in active

    def _request_status(self, request: ReconstructionRequest) -> str:
        if request.scene_generation in self._retired_generations:
            return "stale"
        if self._current_generation is None:
            return "pending"
        if request.scene_generation != self._current_generation:
            return "stale"
        latest_timestamp_us = self._latest_visibility_us.get(request.scene_generation)
        if latest_timestamp_us is None or request.timestamp_us > latest_timestamp_us:
            return "pending"
        if request.track_id in self._active_tracks[request.scene_generation]:
            return "current"
        return "stale"

    def _dispatch_priority(
        self, item: tuple[float, int, QueuedRequest]
    ) -> tuple[float, ...]:
        _negative_quality, sequence, queued = item
        request = queued.request
        if self._priority_policy == "quality":
            return request.quality_score, -float(sequence)
        history = self._visibility.get(
            (request.scene_generation, request.track_id)
        )
        if history is None:
            return 0.0, 0.0, request.quality_score, -float(sequence)
        return (
            history.estimated_remaining_seconds,
            history.observed_seconds,
            request.quality_score,
            -float(sequence),
        )

    def discard_stale_result(self, event: AssetEvent) -> AssetEvent:
        """Convert a completed result into a cancellation after track expiry."""

        if event.state is not AssetState.READY or self.is_active(
            event.scene_generation, event.track_id
        ):
            return event
        metrics = dict(event.metrics)
        exit_timestamp_us = self._latest_visibility_us.get(event.scene_generation)
        metrics.update(
            {
                "deadline_hit": False,
                "stale_job_cancelled": True,
                "result_discarded_after_inference": True,
                "track_exit_timestamp_us": exit_timestamp_us,
            }
        )
        return AssetEvent(
            request_id=event.request_id,
            scene_generation=event.scene_generation,
            track_id=event.track_id,
            state=AssetState.CANCELLED,
            error="track exited while reconstruction was running",
            metrics=metrics,
        )

    def reconstruct(self, queued: QueuedRequest) -> AssetEvent:
        """Decode, reconstruct, and align one mesh to the authoritative metric box."""

        request = queued.request
        started = self._clock()
        queue_latency_ms = (started - queued.enqueued_monotonic) * 1_000.0
        try:
            job = self._to_job(request)
            raw_path = self._reconstructor.reconstruct(job)
            aligned_path = raw_path.with_name(f"{raw_path.stem}.aligned{raw_path.suffix}")
            alignment = self._aligner.align(raw_path, aligned_path, job.dimensions)
            elapsed = self._clock() - started
            metrics = {
                "queue_latency_ms": queue_latency_ms,
                "reconstruction_latency_ms": elapsed * 1_000.0,
                "request_to_mesh_ms": queue_latency_ms + elapsed * 1_000.0,
                "deadline_hit": True,
                "stale_job_cancelled": False,
                "priority_policy": self._priority_policy,
                "coalesce_seconds": self._coalesce_seconds,
                "first_detection_to_mesh_ms": self._first_detection_to_mesh_ms(
                    request
                ),
                "objects_per_second": 1.0 / elapsed if elapsed > 0 else 0.0,
                "uniform_scale": alignment.uniform_scale,
                "object_T_raw_mesh": alignment.object_T_raw_mesh.tolist(),
                "world_T_object": job.world_T_object.matrix.tolist(),
                "measured_dimensions_lwh_m": job.dimensions.as_array.tolist(),
                "aligned_dimensions_lwh_m": alignment.aligned_dimensions.tolist(),
                "dimension_residual_m": alignment.dimension_residual.tolist(),
                "ground_offset_m": alignment.ground_offset,
                "center_error_m": alignment.center_error,
                "ground_error_m": alignment.ground_error,
                **_backend_metrics(raw_path),
                **_cuda_metrics(),
            }
            return _asset_event(
                request,
                AssetState.READY,
                aligned_mesh_path=alignment.output_path,
                metrics=metrics,
            )
        except Exception as error:
            return _asset_event(
                request,
                AssetState.FAILED,
                error=f"{type(error).__name__}: {error}",
                metrics={
                    "queue_latency_ms": queue_latency_ms,
                    "failed_after_ms": (self._clock() - started) * 1_000.0,
                    **_cuda_metrics(),
                },
            )

    def _first_detection_to_mesh_ms(
        self, request: ReconstructionRequest
    ) -> float | None:
        history = self._visibility.get(
            (request.scene_generation, request.track_id)
        )
        if history is None:
            return None
        return (self._clock() - history.first_seen_monotonic) * 1_000.0

    def _to_job(self, request: ReconstructionRequest) -> ReconstructionJob:
        image_buffer = np.frombuffer(request.image_jpeg, dtype=np.uint8)
        bgr = cv2.imdecode(image_buffer, cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError("image_jpeg could not be decoded")
        image = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        mask_buffer = np.frombuffer(request.mask_png, dtype=np.uint8)
        mask = cv2.imdecode(mask_buffer, cv2.IMREAD_GRAYSCALE)
        if mask is None:
            raise ValueError("mask_png could not be decoded")
        mask_bool = mask > 0
        observation = CameraObservation(
            image=image,
            camera_intrinsic=np.asarray(request.camera_intrinsic, dtype=np.float64).reshape(3, 3),
            world_T_camera=_pose(request.world_T_camera),
            timestamp_us=request.timestamp_us,
            mask=mask_bool,
            metadata={
                "frame_id": request.frame_id,
                "scene_generation": request.scene_generation,
            },
        )
        length, width, height = request.dimensions_lwh
        raw_path = (
            self._output_root
            / _safe_component(request.scene_generation)
            / _safe_component(request.track_id)
            / f"{_safe_component(request.request_id)}.glb"
        )
        return ReconstructionJob(
            request_id=request.request_id,
            track_id=request.track_id,
            observation=observation,
            mask=mask_bool,
            dimensions=VehicleDimensions(length, width, height),
            world_T_object=_pose(request.world_T_object),
            output_path=raw_path,
            seed=request.seed,
            metadata={
                "frame_id": request.frame_id,
                "scene_generation": request.scene_generation,
                "quality_score": request.quality_score,
            },
        )


class SAM3DWorkerProcess:
    """Keep the ROUTER responsive while a single inference thread uses CUDA."""

    def __init__(
        self,
        *,
        service: SAM3DReconstructionService,
        router: RouterTransport,
        assets: PublisherTransport,
        health: PublisherTransport,
        metrics_path: Path,
        frames: LatestValueSubscriber | None = None,
        worker_id: str = "sam3d-object",
        device: str = "cuda:0",
        heartbeat_seconds: float = 1.0,
    ) -> None:
        self._service = service
        self._router = router
        self._assets = assets
        self._health = health
        self._metrics_path = Path(metrics_path)
        self._frames = frames
        self._worker_id = worker_id
        self._device = device
        self._heartbeat_seconds = heartbeat_seconds

    def run(self) -> None:
        self._publish_health(WorkerState.LOADING)
        try:
            self._service.load()
        except Exception as error:
            self._publish_health(WorkerState.FAILED, detail=f"{type(error).__name__}: {error}")
            raise
        self._publish_health(WorkerState.READY)
        future: Future[AssetEvent] | None = None
        next_heartbeat = time.monotonic() + self._heartbeat_seconds
        poller = zmq.Poller()
        poller.register(self._router.socket, zmq.POLLIN)
        if self._frames is not None:
            poller.register(self._frames.socket, zmq.POLLIN)
        with ThreadPoolExecutor(max_workers=1, thread_name_prefix="sam3d-inference") as executor:
            try:
                while True:
                    ready = dict(poller.poll(timeout=50))
                    if self._frames is not None and self._frames.socket in ready:
                        _, message, _discarded = self._frames.receive_latest()
                        if isinstance(message, FrameDetections):
                            for cancelled in self._service.update_visibility(message):
                                self._assets.send(cancelled)
                                self._append_metrics(cancelled)

                    if self._router.socket in ready:
                        identity, _topic, message = self._router.receive()
                        if not isinstance(message, ReconstructionRequest):
                            request_id = getattr(message, "request_id", "invalid")
                            self._router.send(
                                identity,
                                Acknowledgement(
                                    request_id=request_id,
                                    accepted=False,
                                    detail="expected reconstruction_request",
                                ),
                            )
                            continue
                        acknowledgement, queued_event = self._service.enqueue(message)
                        self._router.send(identity, acknowledgement)
                        if queued_event is not None:
                            self._assets.send(queued_event)
                            if queued_event.state is AssetState.CANCELLED:
                                self._append_metrics(queued_event)

                    if future is not None and future.done():
                        terminal = self._service.discard_stale_result(future.result())
                        self._assets.send(terminal)
                        self._append_metrics(terminal)
                        future = None
                        if terminal.state is AssetState.FAILED and _is_out_of_memory(
                            terminal.error
                        ):
                            self._publish_health(
                                WorkerState.FAILED,
                                detail=terminal.error,
                            )
                            return

                    if future is None:
                        queued = self._service.pop_next()
                        if queued is not None:
                            self._assets.send(_asset_event(queued.request, AssetState.RUNNING))
                            future = executor.submit(self._service.reconstruct, queued)

                    now = time.monotonic()
                    if now >= next_heartbeat:
                        state = WorkerState.BUSY if future is not None else WorkerState.READY
                        self._publish_health(state)
                        next_heartbeat = now + self._heartbeat_seconds
            except KeyboardInterrupt:
                self._publish_health(WorkerState.STOPPING)

    def _publish_health(self, state: WorkerState, *, detail: str | None = None) -> None:
        cuda = _cuda_metrics()
        self._health.send(
            WorkerHealth(
                worker_id=self._worker_id,
                state=state,
                timestamp_us=time.time_ns() // 1_000,
                pid=os.getpid(),
                device=self._device,
                queue_depth=self._service.queue_depth,
                cuda_allocated_mib=cuda.get("cuda_allocated_mib", 0.0),
                cuda_reserved_mib=cuda.get("cuda_reserved_mib", 0.0),
                detail=detail,
            )
        )

    def _append_metrics(self, event: AssetEvent) -> None:
        record = {
            "timestamp_us": time.time_ns() // 1_000,
            "request_id": event.request_id,
            "scene_generation": event.scene_generation,
            "track_id": event.track_id,
            "state": event.state,
            "error": event.error,
            **event.metrics,
        }
        self._metrics_path.parent.mkdir(parents=True, exist_ok=True)
        with self._metrics_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, allow_nan=False) + "\n")


def _asset_event(
    request: ReconstructionRequest,
    state: AssetState,
    *,
    aligned_mesh_path: Path | None = None,
    error: str | None = None,
    metrics: dict[str, object] | None = None,
) -> AssetEvent:
    return AssetEvent(
        request_id=request.request_id,
        scene_generation=request.scene_generation,
        track_id=request.track_id,
        state=state,
        aligned_mesh_path=aligned_mesh_path,
        error=error,
        metrics=metrics or {},
    )


def _pose(flattened: tuple[float, ...]) -> Pose3D:
    return Pose3D.from_matrix(np.asarray(flattened, dtype=np.float64).reshape(4, 4))


def _safe_component(value: str) -> str:
    component = _SAFE_COMPONENT.sub("_", value).strip("._")
    return component[:120] or "unnamed"


def _cuda_metrics() -> dict[str, float]:
    try:
        import torch

        if torch.cuda.is_available():
            mib = 1024.0 * 1024.0
            return {
                "cuda_allocated_mib": torch.cuda.memory_allocated() / mib,
                "cuda_reserved_mib": torch.cuda.memory_reserved() / mib,
                "cuda_peak_allocated_mib": torch.cuda.max_memory_allocated() / mib,
                "cuda_peak_reserved_mib": torch.cuda.max_memory_reserved() / mib,
            }
    except (ImportError, RuntimeError):
        pass
    return {}


def _backend_metrics(raw_path: Path) -> dict[str, object]:
    sidecar = raw_path.with_suffix(".sam3d.json")
    if not sidecar.is_file():
        return {}
    try:
        return {"sam3d_backend": json.loads(sidecar.read_text(encoding="utf-8"))}
    except (OSError, json.JSONDecodeError):
        return {"sam3d_backend_metrics_error": f"could not read {sidecar}"}


def _is_out_of_memory(error: str | None) -> bool:
    if error is None:
        return False
    normalized = error.casefold()
    return "out of memory" in normalized or "cuda error: memory allocation" in normalized


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames-endpoint", default="tcp://127.0.0.1:5555")
    parser.add_argument("--reconstruction-endpoint", default="tcp://127.0.0.1:5557")
    parser.add_argument("--assets-endpoint", default="tcp://127.0.0.1:5558")
    parser.add_argument("--health-endpoint", default="tcp://127.0.0.1:5559")
    parser.add_argument("--output-root", type=Path, default=Path("artifacts/realtime/sam3d"))
    parser.add_argument(
        "--metrics-path",
        type=Path,
        default=Path("artifacts/realtime/metrics/sam3d.jsonl"),
    )
    parser.add_argument(
        "--sam3d-repository", type=Path, default=Path("thirdparty/sam-3d-objects")
    )
    parser.add_argument(
        "--sam3d-config",
        type=Path,
        default=Path("checkpoints/hf/pipeline.yaml"),
    )
    parser.add_argument("--compile-model", action="store_true")
    parser.add_argument("--precision", choices=("default", "fp16"), default="default")
    parser.add_argument(
        "--priority-policy", choices=("quality", "deadline"), default="quality"
    )
    parser.add_argument("--coalesce-ms", type=float, default=0.0)
    parser.add_argument("--stage1-inference-steps", type=int)
    parser.add_argument("--stage2-inference-steps", type=int)
    parser.add_argument("--pointmap-cache-size", type=int, default=0)
    args = parser.parse_args()

    reconstructor = SAM3DObjectReconstructor(
        factory=UpstreamSAM3DBackend,
        factory_kwargs={
            "repository": args.sam3d_repository,
            "config_path": args.sam3d_config,
            "compile_model": args.compile_model,
            "precision": args.precision,
            "stage1_inference_steps": args.stage1_inference_steps,
            "stage2_inference_steps": args.stage2_inference_steps,
            "pointmap_cache_size": args.pointmap_cache_size,
        },
    )
    service = SAM3DReconstructionService(
        reconstructor=reconstructor,
        output_root=args.output_root,
        priority_policy=args.priority_policy,
        coalesce_seconds=args.coalesce_ms / 1_000.0,
    )
    context = zmq.Context()
    frames = LatestValueSubscriber.open(
        context,
        args.frames_endpoint,
        bind=False,
        topics=("frame_detections",),
        high_water_mark=2,
    )
    router = RouterTransport.open(context, args.reconstruction_endpoint, bind=True)
    assets = PublisherTransport.open(context, args.assets_endpoint, bind=True)
    health = PublisherTransport.open(context, args.health_endpoint, bind=False)
    worker = SAM3DWorkerProcess(
        service=service,
        router=router,
        assets=assets,
        health=health,
        metrics_path=args.metrics_path,
        frames=frames,
    )
    try:
        worker.run()
    finally:
        frames.close()
        router.close()
        assets.close()
        health.close()
        context.term()


if __name__ == "__main__":
    main()

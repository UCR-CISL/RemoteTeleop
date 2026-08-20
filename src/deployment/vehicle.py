"""Vehicle-side admission, SAM3 prompting, and bounded mesh publication."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import time
from typing import Callable, Collection, Protocol

import zmq

from src.realtime.mask_process import (
    StableTrackAdmission,
    StableTrackAdmissionConfig,
    protocol_frame_to_prompt,
    segmentation_batch_to_protocol,
)
from src.realtime.protocol import (
    Acknowledgement,
    AssetChunk,
    AssetCommitAck,
    AssetFetch,
    AssetManifest,
    AssetSync,
    AssetEvent,
    AssetState,
    BoxPrompt,
    FrameDetections,
    FrameAck,
    FrameHello,
    FrameEnd,
    FramePending,
    FrameSnapshot,
    ReconstructionRequest,
)
from src.realtime.transport import DealerTransport, RouterTransport
from src.segmentation.worker import MaskWorker


class MeshEventPublisher(Protocol):
    def send(self, message: AssetEvent, *, topic: str | None = None) -> None: ...


class MeshDeliveryError(RuntimeError):
    """Raised when the remote host does not durably acknowledge a mesh asset."""


class AssetStorePublisher:
    """SAM3D asset sink that makes READY meshes durable without network PUB."""

    def __init__(self, store: "VehicleAssetStore") -> None:
        self.store = store
        self.lifecycle: list[AssetEvent] = []

    def send(self, event: AssetEvent, *, topic: str | None = None) -> None:
        del topic
        if event.state is AssetState.READY:
            self.store.publish(event)
        else:
            self.lifecycle.append(event)


class VehicleFrameSpool:
    """Crash-safe, append-only frame source for receiver-driven replay."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)

    def append(self, snapshot: FrameSnapshot) -> None:
        directory = self.root / _safe_component(snapshot.scene_generation)
        directory.mkdir(parents=True, exist_ok=True)
        target = directory / f"{snapshot.sequence:020d}.json"
        if target.exists():
            existing = self.get(snapshot.scene_generation, snapshot.sequence)
            if existing != snapshot:
                raise MeshDeliveryError(f"conflicting durable frame at sequence {snapshot.sequence}")
            return
        temporary = target.with_suffix(".tmp")
        temporary.write_text(json.dumps(snapshot._metadata(), separators=(",", ":")), encoding="utf-8")
        _fsync_file(temporary)
        os.replace(temporary, target)
        _fsync_directory(directory)

    def get(self, scene_generation: str, sequence: int) -> FrameSnapshot | None:
        path = self.root / _safe_component(scene_generation) / f"{sequence:020d}.json"
        if not path.is_file():
            return None
        values = json.loads(path.read_text(encoding="utf-8"))
        values["boxes"] = tuple(BoxPrompt.from_dict(value) for value in values.get("boxes", ()))
        return FrameSnapshot(**values)

    def finalize(self, end: FrameEnd) -> None:
        if self.get(end.scene_generation, end.final_sequence) is None:
            raise MeshDeliveryError("cannot finalize a spool without its final snapshot")
        directory = self.root / _safe_component(end.scene_generation)
        path = directory / "end.json"
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(end._metadata(), separators=(",", ":")), encoding="utf-8")
        _fsync_file(temporary); os.replace(temporary, path); _fsync_directory(directory)

    def end(self, scene_generation: str) -> FrameEnd | None:
        path = self.root / _safe_component(scene_generation) / "end.json"
        return None if not path.is_file() else FrameEnd(**json.loads(path.read_text(encoding="utf-8")))


class VehicleAnalysisSpool:
    """Crash-safe local spool for image-bearing SAM analysis frames.

    This spool is never exposed to the remote host.  A JSON commit marker is
    installed only after its JPEG is durable, so readers cannot observe a
    partially written frame.
    """

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)

    def append(self, frame: FrameDetections) -> None:
        sequence = _analysis_sequence(frame)
        directory = self.root / _safe_component(frame.scene_generation)
        directory.mkdir(parents=True, exist_ok=True)
        metadata_path = directory / f"{sequence:020d}.json"
        if metadata_path.exists():
            existing = self.get(frame.scene_generation, sequence)
            if existing != frame:
                raise MeshDeliveryError(
                    f"conflicting durable analysis frame at sequence {sequence}"
                )
            return

        image_path = directory / f"{sequence:020d}.jpg"
        image_temporary = image_path.with_suffix(".jpg.tmp")
        image_temporary.write_bytes(frame.image_jpeg)
        _fsync_file(image_temporary)
        os.replace(image_temporary, image_path)

        values = frame._metadata()
        values["frame_sha256"] = frame.sha256
        metadata_temporary = metadata_path.with_suffix(".json.tmp")
        metadata_temporary.write_text(
            json.dumps(values, separators=(",", ":")), encoding="utf-8"
        )
        _fsync_file(metadata_temporary)
        os.replace(metadata_temporary, metadata_path)
        _fsync_directory(directory)

    def get(self, scene_generation: str, sequence: int) -> FrameDetections | None:
        directory = self.root / _safe_component(scene_generation)
        metadata_path = directory / f"{sequence:020d}.json"
        image_path = directory / f"{sequence:020d}.jpg"
        if not metadata_path.is_file():
            return None
        if not image_path.is_file():
            raise MeshDeliveryError(
                f"durable analysis metadata has no JPEG at sequence {sequence}"
            )
        values = json.loads(metadata_path.read_text(encoding="utf-8"))
        expected_sha256 = values.pop("frame_sha256")
        values["boxes"] = tuple(
            BoxPrompt.from_dict(value) for value in values.get("boxes", ())
        )
        frame = FrameDetections(image_jpeg=image_path.read_bytes(), **values)
        if frame.sha256 != expected_sha256:
            raise MeshDeliveryError(
                f"durable analysis frame hash mismatch at sequence {sequence}"
            )
        return frame

    def finalize(self, end: FrameEnd) -> None:
        if self.get(end.scene_generation, end.final_sequence) is None:
            raise MeshDeliveryError("cannot finalize analysis spool without its final frame")
        directory = self.root / _safe_component(end.scene_generation)
        path = directory / "end.json"
        temporary = path.with_suffix(".tmp")
        temporary.write_text(
            json.dumps(end._metadata(), separators=(",", ":")), encoding="utf-8"
        )
        _fsync_file(temporary)
        os.replace(temporary, path)
        _fsync_directory(directory)

    def end(self, scene_generation: str) -> FrameEnd | None:
        path = self.root / _safe_component(scene_generation) / "end.json"
        return (
            None
            if not path.is_file()
            else FrameEnd(**json.loads(path.read_text(encoding="utf-8")))
        )


class VehicleFrameSession:
    """Receiver-pulled, window-one ROUTER session over the durable spool."""

    def __init__(
        self, router: RouterTransport, spool: VehicleFrameSpool | VehicleAnalysisSpool
    ) -> None:
        self.router = router
        self.spool = spool
        self._in_flight: dict[bytes, FrameSnapshot | FrameDetections] = {}

    def serve_once(self) -> None:
        identity, _, message = self.router.receive()
        if isinstance(message, FrameHello):
            self._send_requested(identity, message.scene_generation, message.next_sequence)
            return
        if not isinstance(message, FrameAck):
            raise MeshDeliveryError("frame session accepts only FrameHello and FrameAck")
        current = self._in_flight.get(identity)
        if current is None:
            raise MeshDeliveryError("received FrameAck without an in-flight snapshot")
        if (message.scene_generation, message.sequence, message.frame_id) != (
            current.scene_generation, _durable_sequence(current), current.frame_id,
        ):
            raise MeshDeliveryError("frame acknowledgement does not match the in-flight snapshot")
        if message.snapshot_sha256 != current.sha256:
            raise MeshDeliveryError("frame acknowledgement snapshot hash does not match in-flight snapshot")
        self._in_flight.pop(identity)

    def _send_requested(self, identity: bytes, generation: str, sequence: int) -> None:
        current = self._in_flight.get(identity)
        if current is not None and (
            current.scene_generation, _durable_sequence(current)
        ) != (generation, sequence):
            # A reconnecting receiver supplies its durable cursor. It may have
            # committed the prior frame just before its ACK was lost, so replace
            # stale in-flight state only on this explicit pull request.
            self._in_flight.pop(identity)
        snapshot = self.spool.get(generation, sequence)
        if snapshot is None:
            end = self.spool.end(generation)
            if end is not None and sequence == end.final_sequence + 1:
                _send_if_connected(self.router, identity, end)
                return
            _send_if_connected(self.router, identity, FramePending(generation, sequence))
            return
        if not _send_if_connected(self.router, identity, snapshot):
            self._in_flight.pop(identity, None)
            return
        self._in_flight[identity] = snapshot


def _analysis_sequence(frame: FrameDetections) -> int:
    if frame.source_sequence is None:
        raise ValueError("durable analysis frames require source_sequence")
    return frame.source_sequence


def _durable_sequence(frame: FrameSnapshot | FrameDetections) -> int:
    return frame.sequence if isinstance(frame, FrameSnapshot) else _analysis_sequence(frame)


class VehicleAssetStore:
    """Content-addressed mesh store that exposes bounded resumable chunks."""

    def __init__(self, root: str | Path, *, chunk_bytes: int = 1024 * 1024) -> None:
        if not 1 <= chunk_bytes <= AssetChunk.MAX_PAYLOAD_BYTES:
            raise ValueError("chunk_bytes must be between 1 byte and 4 MiB")
        self.root = Path(root)
        self.chunk_bytes = chunk_bytes

    def publish(self, event: AssetEvent) -> AssetManifest:
        if event.state is not AssetState.READY or event.aligned_mesh_path is None:
            raise ValueError("only ready local mesh assets can be published")
        source = event.aligned_mesh_path
        digest, byte_length = _copy_and_hash(source, self.root / ".staging")
        asset_id = event.asset_id or f"{event.scene_generation}:{event.track_id}"
        target = self._path(event.scene_generation, asset_id, event.asset_version, digest, source.suffix or ".glb")
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.is_file():
            staged = self.root / ".staging" / digest
            os.replace(staged, target)
            _fsync_directory(target.parent)
        else:
            staged = self.root / ".staging" / digest
            if staged.exists():
                staged.unlink()
        if event.source_timestamp_us is None or event.source_frame_id is None:
            raise ValueError("ready mesh assets require source timestamp and frame provenance")
        manifest = AssetManifest(
            scene_generation=event.scene_generation, track_id=event.track_id, request_id=event.request_id,
            source_sequence=event.source_sequence, source_timestamp_us=event.source_timestamp_us,
            source_frame_id=event.source_frame_id, asset_id=asset_id, asset_version=event.asset_version,
            mesh_suffix=target.suffix, byte_length=byte_length, content_sha256=digest,
        )
        journal = target.with_suffix(target.suffix + ".manifest.json")
        journal.write_text(json.dumps(manifest._metadata(), separators=(",", ":")), encoding="utf-8")
        _fsync_file(journal)
        return manifest

    def fetch(self, manifest: AssetManifest, request: AssetFetch) -> AssetChunk | None:
        if (request.asset_id, request.asset_version, request.content_sha256) != (manifest.asset_id, manifest.asset_version, manifest.content_sha256):
            raise MeshDeliveryError("asset fetch does not match manifest")
        if request.offset > manifest.byte_length:
            raise MeshDeliveryError("asset fetch offset exceeds manifest length")
        if request.offset == manifest.byte_length:
            return None
        path = self._path(manifest.scene_generation, manifest.asset_id, manifest.asset_version, manifest.content_sha256, manifest.mesh_suffix)
        with path.open("rb") as stream:
            stream.seek(request.offset)
            payload = stream.read(min(self.chunk_bytes, manifest.byte_length - request.offset))
        return AssetChunk(manifest.asset_id, manifest.asset_version, manifest.content_sha256, request.offset, payload)

    def _path(self, generation: str, asset_id: str, version: int, digest: str, suffix: str) -> Path:
        return self.root / _safe_component(generation) / f"{_safe_component(asset_id)}-v{version}-{digest}{suffix}"

    def manifests(self, scene_generation: str | None = None) -> tuple[AssetManifest, ...]:
        """Reload only fully durable journal entries after a vehicle restart."""
        result: list[AssetManifest] = []
        for journal in self.root.rglob("*.manifest.json"):
            values = json.loads(journal.read_text(encoding="utf-8"))
            manifest = AssetManifest(**values)
            if scene_generation is not None and manifest.scene_generation != scene_generation:
                continue
            path = self._path(
                manifest.scene_generation, manifest.asset_id, manifest.asset_version,
                manifest.content_sha256, manifest.mesh_suffix,
            )
            if not path.is_file() or path.stat().st_size != manifest.byte_length:
                continue
            digest, byte_length = _hash_file(path)
            if digest == manifest.content_sha256 and byte_length == manifest.byte_length:
                result.append(manifest)
        return tuple(result)


class VehicleAssetSession:
    """Serve durable manifests and one explicitly requested mesh chunk at a time."""

    def __init__(self, router: RouterTransport, store: VehicleAssetStore) -> None:
        self.router = router
        self.store = store
        self._manifests: dict[tuple[str, int, str], AssetManifest] = {}
        self.committed: dict[bytes, set[tuple[str, int, str]]] = {}

    def publish(self, event: AssetEvent) -> AssetManifest:
        manifest = self.store.publish(event)
        self._manifests[(manifest.asset_id, manifest.asset_version, manifest.content_sha256)] = manifest
        return manifest

    def serve_once(self) -> None:
        identity, _, message = self.router.receive()
        if isinstance(message, AssetSync):
            for manifest in self.store.manifests(message.scene_generation):
                self._manifests[(manifest.asset_id, manifest.asset_version, manifest.content_sha256)] = manifest
            for manifest in self._manifests.values():
                if manifest.scene_generation == message.scene_generation:
                    if not _send_if_connected(self.router, identity, manifest):
                        return
            return
        if isinstance(message, AssetFetch):
            manifest = self._manifests.get((message.asset_id, message.asset_version, message.content_sha256))
            if manifest is None:
                raise MeshDeliveryError("remote requested an unknown mesh asset")
            chunk = self.store.fetch(manifest, message)
            if chunk is not None:
                _send_if_connected(self.router, identity, chunk)
            return
        if isinstance(message, AssetCommitAck):
            key = (message.asset_id, message.asset_version, message.content_sha256)
            if key not in self._manifests:
                raise MeshDeliveryError("remote committed an unknown mesh asset")
            self.committed.setdefault(identity, set()).add(key)
            return
        raise MeshDeliveryError("asset session received an unsupported message")


class AcknowledgedMeshPublisher:
    """Deliver mesh events over a request/reply channel with explicit remote ACKs.

    Unlike PUB/SUB, this path blocks until the remote cache acknowledges the
    event.  A timeout is an error for the supervisor to retry or stop on; it is
    never interpreted as permission to discard an asset.
    """

    def __init__(self, transport: DealerTransport) -> None:
        self._transport = transport

    def send_confirmed(self, event: AssetEvent, *, timeout_ms: int | None = None) -> None:
        if timeout_ms is not None and timeout_ms <= 0:
            raise ValueError("timeout_ms must be positive when provided")
        self._transport.send(event)
        poller = zmq.Poller()
        poller.register(self._transport.socket, zmq.POLLIN)
        deadline = None if timeout_ms is None else time.monotonic() + timeout_ms / 1_000.0
        while True:
            wait_ms = -1
            if deadline is not None:
                remaining_seconds = deadline - time.monotonic()
                if remaining_seconds <= 0:
                    raise MeshDeliveryError(
                        f"timed out waiting for remote acknowledgement of {event.request_id}"
                    )
                wait_ms = max(1, int(remaining_seconds * 1_000.0))
            if self._transport.socket not in dict(poller.poll(wait_ms)):
                raise MeshDeliveryError(
                    f"timed out waiting for remote acknowledgement of {event.request_id}"
                )
            _, reply = self._transport.receive()
            if not isinstance(reply, Acknowledgement):
                raise MeshDeliveryError("mesh endpoint returned a non-acknowledgement message")
            if reply.request_id != event.request_id:
                continue
            if not reply.accepted:
                raise MeshDeliveryError(reply.detail or f"remote rejected mesh asset {event.request_id}")
            return


class VehicleProcessGraph:
    """Bounded, Torch-spawned handoff between camera, SAM3, and sequential SAM3D.

    The vehicle camera loop calls :meth:`submit_frame` only after publishing
    its frame stream to the remote host. A full input queue applies
    backpressure: it never silently discards an analysis frame.
    """

    def __init__(
        self,
        *,
        sam3_target: Callable[[object, object], None],
        sam3d_target: Callable[[object, object], None],
        queue_size: int = 4,
    ) -> None:
        if queue_size <= 0:
            raise ValueError("queue_size must be positive")
        self._sam3_target = sam3_target
        self._sam3d_target = sam3d_target
        self._queue_size = queue_size
        self._frames: object | None = None
        self._requests: object | None = None
        self._assets: object | None = None
        self._processes: dict[str, object] = {}

    def start(self) -> None:
        if self._processes:
            return
        import torch.multiprocessing as multiprocessing

        context = multiprocessing.get_context("spawn")
        self._frames = context.Queue(maxsize=self._queue_size)
        self._requests = context.Queue(maxsize=self._queue_size)
        self._assets = context.Queue(maxsize=self._queue_size)
        self._processes = {
            "sam3": context.Process(
                target=self._sam3_target,
                args=(self._frames, self._requests),
                name="vehicle-sam3",
            ),
            "sam3d": context.Process(
                target=self._sam3d_target,
                args=(self._requests, self._assets),
                name="vehicle-sam3d",
            ),
        }
        for process in self._processes.values():
            process.start()

    def join(self) -> None:
        for process in self._processes.values():
            process.join()

    def submit_frame(self, frame: FrameDetections) -> bool:
        """Block until the local SAM3 worker accepts this frame."""

        if self._frames is None:
            raise RuntimeError("vehicle process graph has not started")
        self._frames.put(frame)  # type: ignore[union-attr]
        return True

    def next_asset(self) -> AssetEvent | None:
        if self._assets is None:
            raise RuntimeError("vehicle process graph has not started")
        import queue

        try:
            event = self._assets.get_nowait()  # type: ignore[union-attr]
        except queue.Empty:
            return None
        if not isinstance(event, AssetEvent):
            raise TypeError("SAM3D process produced a non-asset message")
        return event


@dataclass(frozen=True)
class VehicleMeshRuntimeConfig:
    """The only deployment policy knobs for asynchronous mesh production."""

    maximum_sam3d_queue: int = 4
    admission: StableTrackAdmissionConfig = StableTrackAdmissionConfig()
    seed: int = 0

    def __post_init__(self) -> None:
        if self.maximum_sam3d_queue <= 0:
            raise ValueError("maximum_sam3d_queue must be positive")


class VehicleMeshRuntime:
    """Keep camera publication independent from resident SAM3/SAM3D work.

    The caller publishes every :class:`FrameDetections` immediately.  This
    class only turns stable tracks into labelled SAM3 prompts and retains a
    FIFO of reconstruction requests for one sequential SAM3D process.
    """

    def __init__(self, worker: MaskWorker, config: VehicleMeshRuntimeConfig | None = None) -> None:
        self.worker = worker
        self.config = config or VehicleMeshRuntimeConfig()
        self._admission = StableTrackAdmission(self.config.admission)
        self._pending: deque[ReconstructionRequest] = deque()
        self._terminal_tracks: set[tuple[str, str]] = set()

    @property
    def queue_depth(self) -> int:
        return len(self._pending)

    def observe(self, frame: FrameDetections) -> tuple[AssetEvent, ...]:
        """Run SAM3 in its worker process without waiting for SAM3D."""

        live = {(frame.scene_generation, box.track_id) for box in frame.boxes}
        events = list(self._discard_expired(frame.scene_generation, live))
        handled = self._terminal_tracks | {
            (request.scene_generation, request.track_id) for request in self._pending
        }
        wave = self._admission.observe(frame, handled_tracks=handled)
        for source in wave.frames:
            segmentation = self.worker.process(protocol_frame_to_prompt(source))
            masks = segmentation_batch_to_protocol(source, segmentation)
            for request in _requests_from_masks(source, masks, self.config.seed):
                events.extend(self._enqueue(request))
        return tuple(events)

    def pop_next(self) -> ReconstructionRequest | None:
        """Return at most one request for the single SAM3D worker."""

        return self._pending.popleft() if self._pending else None

    def mark_terminal(self, event: AssetEvent) -> None:
        if event.state in {AssetState.READY, AssetState.FAILED, AssetState.CANCELLED}:
            self._terminal_tracks.add((event.scene_generation, event.track_id))

    def _enqueue(self, request: ReconstructionRequest) -> tuple[AssetEvent, ...]:
        key = (request.scene_generation, request.track_id)
        if key in self._terminal_tracks or any(
            (item.scene_generation, item.track_id) == key for item in self._pending
        ):
            return ()
        if len(self._pending) >= self.config.maximum_sam3d_queue:
            raise MeshDeliveryError(
                "SAM3D request queue is full; drain it before accepting another mesh request"
            )
        self._pending.append(request)
        return (_event(request, AssetState.QUEUED),)

    def _discard_expired(
        self, generation: str, live: Collection[tuple[str, str]]
    ) -> tuple[AssetEvent, ...]:
        kept: deque[ReconstructionRequest] = deque()
        cancelled: list[AssetEvent] = []
        while self._pending:
            request = self._pending.popleft()
            if (request.scene_generation, request.track_id) in live:
                kept.append(request)
            else:
                self._terminal_tracks.add((request.scene_generation, request.track_id))
                cancelled.append(_event(request, AssetState.CANCELLED, error="track is no longer live"))
        self._pending = kept
        return tuple(cancelled)


def publish_mesh_asset(event: AssetEvent, publisher: MeshEventPublisher) -> AssetEvent:
    """Publish a ready local mesh as a self-verifying cross-host asset event."""

    if event.state is not AssetState.READY or event.aligned_mesh_path is None:
        publisher.send(event)
        return event
    payload = event.aligned_mesh_path.read_bytes()
    published = AssetEvent(
        request_id=event.request_id,
        scene_generation=event.scene_generation,
        track_id=event.track_id,
        state=event.state,
        asset_id=event.asset_id or f"{event.scene_generation}:{event.track_id}",
        asset_version=event.asset_version,
        content_sha256=hashlib.sha256(payload).hexdigest(),
        mesh_payload=payload,
        mesh_suffix=event.aligned_mesh_path.suffix or ".glb",
        metrics=event.metrics,
        source_sequence=event.source_sequence,
        source_timestamp_us=event.source_timestamp_us,
        source_frame_id=event.source_frame_id,
    )
    confirmed = getattr(publisher, "send_confirmed", None)
    if confirmed is not None:
        confirmed(published)
    else:
        publisher.send(published)
    return published


def _requests_from_masks(frame, masks, seed: int) -> tuple[ReconstructionRequest, ...]:
    boxes = {box.track_id: box for box in frame.boxes}
    return tuple(
        ReconstructionRequest(
            request_id=f"{frame.scene_generation}:{mask.track_id}",
            scene_generation=frame.scene_generation,
            track_id=mask.track_id,
            frame_id=frame.frame_id,
            timestamp_us=frame.timestamp_us,
            image_jpeg=frame.image_jpeg,
            mask_png=mask.mask_png,
            camera_intrinsic=frame.camera_intrinsic,
            dimensions_lwh=boxes[mask.track_id].dimensions_lwh,
            world_T_object=boxes[mask.track_id].world_T_object,
            world_T_camera=frame.world_T_camera,
            quality_score=mask.confidence * mask.coverage * mask.box_overlap,
            seed=seed,
            source_sequence=frame.source_sequence,
        )
        for mask in masks.masks if mask.track_id in boxes
    )


def _event(request: ReconstructionRequest, state: AssetState, *, error: str | None = None) -> AssetEvent:
    return AssetEvent(
        request_id=request.request_id,
        scene_generation=request.scene_generation,
        track_id=request.track_id,
        state=state,
        error=error,
        source_sequence=request.source_sequence,
        source_timestamp_us=request.timestamp_us,
        source_frame_id=request.frame_id,
    )


def _safe_component(value: str) -> str:
    return "".join(character if character.isalnum() or character in "._-" else "_" for character in value)[:120] or "asset"


def _fsync_file(path: Path) -> None:
    with path.open("rb") as stream:
        os.fsync(stream.fileno())


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _copy_and_hash(source: Path, staging_root: Path) -> tuple[str, int]:
    """Copy a mesh once while calculating its identity; avoid whole-file RAM use."""
    staging_root.mkdir(parents=True, exist_ok=True)
    temporary = staging_root / f"{source.name}.{os.getpid()}.partial"
    digest = hashlib.sha256()
    byte_length = 0
    with source.open("rb") as input_stream, temporary.open("wb") as output_stream:
        while chunk := input_stream.read(1024 * 1024):
            digest.update(chunk)
            output_stream.write(chunk)
            byte_length += len(chunk)
        output_stream.flush()
        os.fsync(output_stream.fileno())
    target = staging_root / digest.hexdigest()
    if target.is_file():
        temporary.unlink()
    else:
        os.replace(temporary, target)
        _fsync_directory(staging_root)
    return digest.hexdigest(), byte_length


def _hash_file(path: Path) -> tuple[str, int]:
    digest = hashlib.sha256()
    byte_length = 0
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
            byte_length += len(chunk)
    return digest.hexdigest(), byte_length


def _send_if_connected(router: RouterTransport, identity: bytes, message: object) -> bool:
    """Treat a ROUTER peer disappearing after its request as resumable."""
    try:
        router.send(identity, message)  # type: ignore[arg-type]
    except zmq.ZMQError as error:
        if error.errno == zmq.EHOSTUNREACH:
            return False
        raise
    return True

"""Render localized GSplat frames from the ordered pose-only ZMQ stream."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import signal
import threading
import time
from queue import Empty, SimpleQueue

import cv2
import numpy as np
import zmq

from src.localization.gaussian_map import load_spz_v3
from src.deployment.remote import ResumableRemoteAssetCache
from src.realtime.protocol import (
    AssetChunk, AssetCommitAck, AssetFetch, AssetManifest, AssetSync,
    EgoPoseSample, FrameAck, FrameEnd, FrameHello, FrameObjectDetections, FramePending, FrameSnapshot, WorkerState,
)
from src.realtime.runtime_support import HealthHeartbeat, JsonlMetricsWriter
from src.realtime.transport import DealerTransport
from src.viz.composited_camera import (
    CudaGaussianBufferRenderer,
    PyTorch3DObjectMeshRenderer,
    PoseOnlyCompositedCameraBackend,
    RemoteCameraConfig,
)


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


class CompositedFrameWriter:
    """Persist numbered frames and a realtime-cadence MP4."""

    def __init__(self, output_dir: Path, frames_per_second: float) -> None:
        if frames_per_second <= 0:
            raise ValueError("frames_per_second must be positive")
        self.output_dir = Path(output_dir)
        self.frames_dir = self.output_dir / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.frames_per_second = frames_per_second
        self._size: tuple[int, int] | None = None
        existing = self._existing_frames()
        self.count = len(existing)
        if existing:
            image = cv2.imread(str(existing[0]))
            if image is None:
                raise RuntimeError(f"failed to read existing composited frame {existing[0]}")
            self._size = (image.shape[1], image.shape[0])

    def _existing_frames(self) -> list[Path]:
        indexed: dict[int, Path] = {}
        for path in self.frames_dir.glob("*.png"):
            try:
                index = int(path.name.split("_", 1)[0])
            except (ValueError, IndexError):
                continue
            if index in indexed:
                raise ValueError(f"duplicate composited frame index {index}")
            indexed[index] = path
        if sorted(indexed) != list(range(len(indexed))):
            raise ValueError("existing composited frames are not contiguous")
        return [indexed[index] for index in range(len(indexed))]

    def write(self, frame_id: str, rgb: np.ndarray) -> Path:
        image = np.clip(np.asarray(rgb) * 255.0, 0, 255).astype(np.uint8)
        height, width = image.shape[:2]
        size = (width, height)
        if self._size is None:
            self._size = size
        if size != self._size:
            raise ValueError(f"composited frame size changed from {self._size} to {size}")
        bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        path = self.frames_dir / f"{self.count:06d}_{frame_id}.png"
        temporary = self.frames_dir / f".{self.count:06d}_{frame_id}.partial.png"
        if not cv2.imwrite(str(temporary), bgr):
            raise RuntimeError(f"failed to write composited frame {path}")
        with temporary.open("rb") as stream:
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        _fsync_directory(self.frames_dir)
        self.count += 1
        return path

    def reconcile(self, committed_count: int) -> None:
        """Discard only PNGs written before a crash but not cursor-committed."""

        frames = self._existing_frames()
        if len(frames) < committed_count:
            raise ValueError(
                f"only {len(frames)} durable frames exist for {committed_count} committed cursor entries"
            )
        for path in frames[committed_count:]:
            path.unlink()
        if len(frames) > committed_count:
            _fsync_directory(self.frames_dir)
        remaining = self._existing_frames()
        self.count = committed_count
        if remaining:
            image = cv2.imread(str(remaining[0]))
            if image is None:
                raise RuntimeError(f"failed to read existing composited frame {remaining[0]}")
            self._size = (image.shape[1], image.shape[0])
        else:
            self._size = None

    def close(self) -> None:
        frames = self._existing_frames()
        if not frames:
            return
        assert self._size is not None
        temporary = self.output_dir / ".composited.partial.mp4"
        if temporary.exists():
            temporary.unlink()
        video = cv2.VideoWriter(
            str(temporary), cv2.VideoWriter_fourcc(*"mp4v"), self.frames_per_second, self._size
        )
        if not video.isOpened():
            raise RuntimeError("failed to open composited MP4 writer")
        try:
            for path in frames:
                image = cv2.imread(str(path))
                if image is None or (image.shape[1], image.shape[0]) != self._size:
                    raise RuntimeError(f"failed to read consistent composited frame {path}")
                video.write(image)
        finally:
            video.release()
        with temporary.open("rb") as stream:
            os.fsync(stream.fileno())
        os.replace(temporary, self.output_dir / "composited.mp4")
        _fsync_directory(self.output_dir)


@dataclass
class FrameCursor:
    """Durably remember the last rendered snapshot before acknowledging it."""

    def __init__(self, path: Path, scene_generation: str) -> None:
        self.path = Path(path)
        self.scene_generation = scene_generation
        self._sequence = -1
        self._frame_id: str | None = None
        self._timestamp_us: int | None = None
        self._snapshot_sha256: str | None = None
        if self.path.is_file():
            data = json.loads(self.path.read_text())
            if data["scene_generation"] != scene_generation:
                raise ValueError("cursor scene generation does not match receiver configuration")
            self._sequence = int(data["sequence"])
            self._frame_id = str(data["frame_id"])
            self._timestamp_us = int(data["timestamp_us"])
            self._snapshot_sha256 = str(data["snapshot_sha256"])

    @property
    def next_sequence(self) -> int:
        return self._sequence + 1

    def validate(self, snapshot: FrameSnapshot) -> dict[str, int]:
        if snapshot.scene_generation != self.scene_generation:
            raise ValueError("snapshot scene generation does not match cursor")
        if snapshot.sequence != self.next_sequence:
            raise ValueError(f"snapshot sequence {snapshot.sequence} does not follow cursor {self._sequence}")
        if self._timestamp_us is not None and snapshot.timestamp_us != self._timestamp_us + 100_000:
            raise ValueError(
                f"snapshot timestamp {snapshot.timestamp_us} does not follow 100000us cadence "
                f"from {self._timestamp_us}"
            )
        return {
            "sequence_gap": 0,
            "timestamp_gap_us": 0,
            "timestamp_delta_us": 0 if self._timestamp_us is None else snapshot.timestamp_us - self._timestamp_us,
        }

    def commit(self, snapshot: FrameSnapshot) -> None:
        self.validate(snapshot)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".partial")
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump({"scene_generation": snapshot.scene_generation, "sequence": snapshot.sequence,
                       "frame_id": snapshot.frame_id, "timestamp_us": snapshot.timestamp_us,
                       "snapshot_sha256": snapshot.sha256}, stream, sort_keys=True)
            stream.flush()
            __import__("os").fsync(stream.fileno())
        temporary.replace(self.path)
        _fsync_directory(self.path.parent)
        self._sequence = snapshot.sequence
        self._frame_id = snapshot.frame_id
        self._timestamp_us = snapshot.timestamp_us
        self._snapshot_sha256 = snapshot.sha256

    def is_committed(self, snapshot: FrameSnapshot) -> bool:
        return (
            snapshot.sequence == self._sequence
            and snapshot.frame_id == self._frame_id
            and snapshot.sha256 == self._snapshot_sha256
        )

    def complete(self, end: FrameEnd) -> None:
        if (
            end.scene_generation != self.scene_generation
            or end.final_sequence != self._sequence
            or end.final_timestamp_us != self._timestamp_us
        ):
            raise ValueError("frame end does not match the committed cursor")
        path = self.path.with_name("frame-complete.json")
        temporary = path.with_suffix(".partial")
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(end._metadata(), stream, sort_keys=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        _fsync_directory(path.parent)


@dataclass
class OrderedPoseMetrics:
    """Legacy validation helper retained for pose-only diagnostic callers."""

    expected_sequence: int = 0
    expected_timestamp_us: int = 0
    previous_timestamp_us: int | None = None
    scene_generation: str | None = None
    expected_cadence_us: int = 100_000

    def observe(self, sample: EgoPoseSample) -> dict[str, int]:
        if self.scene_generation is None:
            self.scene_generation = sample.scene_generation
        elif sample.scene_generation != self.scene_generation:
            raise ValueError("mismatched scene generation")
        if sample.sequence < self.expected_sequence:
            raise ValueError("out-of-order pose sequence")
        if self.previous_timestamp_us is not None and sample.timestamp_us <= self.previous_timestamp_us:
            raise ValueError("pose source timestamps must be strictly increasing")
        result = {
            "sequence_gap": sample.sequence - self.expected_sequence,
            "timestamp_gap_us": sample.timestamp_us - self.expected_timestamp_us,
            "timestamp_delta_us": 0 if self.previous_timestamp_us is None else sample.timestamp_us - self.previous_timestamp_us,
        }
        self.expected_sequence = sample.sequence + 1
        self.expected_timestamp_us = sample.timestamp_us + self.expected_cadence_us
        self.previous_timestamp_us = sample.timestamp_us
        return result


class OrderedPoseDetectionJoin:
    """Legacy strict join helper; reliable snapshots replace it in production."""

    def __init__(self) -> None:
        self._pending: FrameObjectDetections | None = None

    def observe_detection(self, detections: FrameObjectDetections) -> None:
        if self._pending is not None:
            raise ValueError("received detections before their matching pose")
        self._pending = detections

    def take_for_pose(self, pose: EgoPoseSample) -> FrameObjectDetections:
        if self._pending is None:
            raise ValueError("pose must be preceded by matching frame detections")
        detections, self._pending = self._pending, None
        for field in ("sequence", "timestamp_us", "scene_generation", "frame_id"):
            if getattr(detections, field) != getattr(pose, field):
                raise ValueError(f"detection/pose {field} mismatch")
        return detections


class RemoteAssetReceiver:
    """Receive resumable mesh chunks on an independent DEALER socket."""

    def __init__(self, endpoint: str, receiver_id: str, scene_generation: str, cache_dir: Path) -> None:
        self._endpoint = endpoint
        self._receiver_id = receiver_id
        self._scene_generation = scene_generation
        self._cache = ResumableRemoteAssetCache(cache_dir)
        self.events: SimpleQueue = SimpleQueue()
        self.errors: SimpleQueue = SimpleQueue()
        self._stop = threading.Event()
        self._activated: set[tuple[str, int, str]] = set()
        self._active: set[tuple[str, int, str]] = set()
        self._thread = threading.Thread(target=self._run, name="remote-asset-receiver", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        context = zmq.Context()
        assets = DealerTransport.open(context, self._endpoint, identity=f"{self._receiver_id}-assets")
        poller = zmq.Poller()
        poller.register(assets.socket, zmq.POLLIN)
        try:
            next_sync = 0.0
            while not self._stop.is_set():
                now = time.monotonic()
                if now >= next_sync:
                    assets.send(AssetSync(
                        self._receiver_id, self._scene_generation, self._cache.known_hashes()
                    ))
                    next_sync = now + 1.0
                if assets.socket not in dict(poller.poll(100)):
                    continue
                _, message = assets.receive()
                if isinstance(message, AssetManifest):
                    if message.scene_generation != self._scene_generation:
                        continue
                    key = (message.asset_id, message.asset_version, message.content_sha256)
                    if key in self._active:
                        continue
                    offset = self._cache.begin(message)
                    event = self._cache.committed(message)
                    if event is not None:
                        if key not in self._activated:
                            self.events.put(event)
                            self._activated.add(key)
                        assets.send(AssetCommitAck(message.asset_id, message.asset_version, message.content_sha256))
                    else:
                        self._active.add(key)
                        assets.send(AssetFetch(message.asset_id, message.asset_version, message.content_sha256, offset))
                elif isinstance(message, AssetChunk):
                    key = (message.asset_id, message.asset_version, message.content_sha256)
                    if key not in self._active:
                        raise ValueError("asset chunk arrived without an active fetch")
                    event = self._cache.append(message)
                    if event is not None:
                        self._active.remove(key)
                        if key not in self._activated:
                            self.events.put(event)
                            self._activated.add(key)
                        assets.send(AssetCommitAck(message.asset_id, message.asset_version, message.content_sha256))
                        next_sync = 0.0
                    else:
                        assets.send(AssetFetch(
                            message.asset_id, message.asset_version, message.content_sha256,
                            self._cache.next_offset(
                                message.asset_id, message.asset_version, message.content_sha256
                            ),
                        ))
                else:
                    raise TypeError("asset endpoint produced an unsupported message")
        except Exception as error:
            self.errors.put(error)
        finally:
            assets.close()
            context.term()


class CompositedCameraProcess:
    """Receive every pose sample and render using only remote-local calibration."""

    def __init__(
        self,
        *,
        frames_endpoint: str,
        assets_endpoint: str | None = None,
        scene_generation: str,
        receiver_id: str,
        health_endpoint: str,
        splat_path: Path,
        output_dir: Path,
        metrics_path: Path,
        camera: RemoteCameraConfig,
        frames_per_second: float = 10.0,
        device: str = "cuda",
        cursor_path: Path | None = None,
        mesh_cache_dir: Path | None = None,
    ) -> None:
        self._context = zmq.Context()
        self._frames = DealerTransport.open(self._context, frames_endpoint, identity=receiver_id)
        self._scene_generation = scene_generation
        self._receiver_id = receiver_id
        self._heartbeat = HealthHeartbeat(endpoint=health_endpoint, worker_id="viewer", device=device)
        self._metrics = JsonlMetricsWriter(metrics_path)
        self._writer = CompositedFrameWriter(output_dir, frames_per_second)
        gaussian_map = load_spz_v3(splat_path)
        self._backend = PoseOnlyCompositedCameraBackend(
            CudaGaussianBufferRenderer(gaussian_map, device=device, downsample=1),
            camera,
            PyTorch3DObjectMeshRenderer(device=device),
        )
        self._cursor = FrameCursor(cursor_path or output_dir / "frame-cursor.json", scene_generation)
        self._writer.reconcile(self._cursor.next_sequence)
        self._assets = (
            RemoteAssetReceiver(
                assets_endpoint, receiver_id, scene_generation,
                mesh_cache_dir or output_dir / "mesh-cache",
            ) if assets_endpoint is not None else None
        )
        self._stop = False

    def request_stop(self, *_: object) -> None:
        self._stop = True

    def run(self) -> None:
        poller = zmq.Poller()
        poller.register(self._frames.socket, zmq.POLLIN)
        self._heartbeat.start()
        if self._assets is not None:
            self._assets.start()
        self._heartbeat.update(WorkerState.READY)
        try:
            hello_needed = True
            scene_complete = False
            while not self._stop:
                if self._assets is not None:
                    try:
                        raise self._assets.errors.get_nowait()
                    except Empty:
                        pass
                    while True:
                        try:
                            self._backend.handle_asset_event(self._assets.events.get_nowait())
                        except Empty:
                            break
                if scene_complete:
                    time.sleep(0.05)
                    continue
                if hello_needed:
                    self._frames.send(FrameHello(
                        self._scene_generation, self._receiver_id, self._cursor.next_sequence
                    ))
                    hello_needed = False
                if self._frames.socket not in dict(poller.poll(1_000)):
                    hello_needed = True
                    continue
                _, snapshot = self._frames.receive()
                if isinstance(snapshot, FrameEnd):
                    self._cursor.complete(snapshot)
                    self._metrics.append({
                        "event": "compositor_scene_end",
                        "scene_generation": snapshot.scene_generation,
                        "final_sequence": snapshot.final_sequence,
                        "final_timestamp_us": snapshot.final_timestamp_us,
                    })
                    scene_complete = True
                    continue
                if isinstance(snapshot, FramePending):
                    if (
                        snapshot.scene_generation != self._scene_generation
                        or snapshot.next_sequence != self._cursor.next_sequence
                    ):
                        raise ValueError("frame pending response does not match the outstanding hello")
                    time.sleep(0.05)
                    hello_needed = True
                    continue
                if not isinstance(snapshot, FrameSnapshot):
                    raise TypeError("frame endpoint produced an unsupported message")
                if snapshot.scene_generation != self._scene_generation:
                    raise ValueError("snapshot scene generation does not match receiver configuration")
                if snapshot.sequence < self._cursor.next_sequence:
                    if not self._cursor.is_committed(snapshot):
                        raise ValueError("duplicate snapshot conflicts with committed output")
                    self._frames.send(FrameAck(snapshot.scene_generation, snapshot.sequence, snapshot.frame_id, snapshot.sha256))
                    hello_needed = True
                    continue
                if snapshot.sequence != self._cursor.next_sequence:
                    raise ValueError(f"frame sequence gap: expected {self._cursor.next_sequence}, got {snapshot.sequence}")
                ordering = self._cursor.validate(snapshot)
                started = time.perf_counter()
                rendered = self._backend.render_pose(snapshot, snapshot.boxes)
                path = self._writer.write(snapshot.frame_id, rendered.rgb)
                self._cursor.commit(snapshot)
                rendered_sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
                self._frames.send(FrameAck(snapshot.scene_generation, snapshot.sequence, snapshot.frame_id, snapshot.sha256, rendered_sha256))
                hello_needed = True
                elapsed = time.perf_counter() - started
                self._metrics.append({
                    "event": "compositor_frame",
                    "sequence": rendered.sequence,
                    "scene_generation": rendered.scene_generation,
                    "source_timestamp_us": rendered.timestamp_us,
                    **ordering,
                    "box_count": len(snapshot.boxes),
                    "proxy_count": sum(state == "proxy" for state in rendered.track_states.values()),
                    "mesh_count": sum(state == "mesh" for state in rendered.track_states.values()),
                    "track_states": dict(rendered.track_states),
                    "proxy_track_ids": sorted(track_id for track_id, state in rendered.track_states.items() if state == "proxy"),
                    "mesh_track_ids": sorted(track_id for track_id, state in rendered.track_states.items() if state == "mesh"),
                    "output_path": str(path),
                    "render_ms": elapsed * 1_000.0,
                    "effective_fps": 1.0 / elapsed if elapsed > 0 else 0.0,
                })
                self._heartbeat.update(WorkerState.READY, detail=str(rendered.sequence))
        except Exception as error:
            self._heartbeat.update(WorkerState.FAILED, detail=f"{type(error).__name__}: {error}")
            raise
        finally:
            self.close()

    def close(self) -> None:
        self._writer.close()
        self._heartbeat.stop()
        self._frames.close()
        if self._assets is not None:
            self._assets.close()
        self._context.term()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames-endpoint", required=True)
    parser.add_argument("--assets-endpoint")
    parser.add_argument("--mesh-cache-dir", type=Path)
    parser.add_argument("--scene-generation", required=True)
    parser.add_argument("--receiver-id", required=True)
    parser.add_argument("--cursor-path", type=Path)
    parser.add_argument("--health-endpoint", required=True)
    parser.add_argument("--splat", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--metrics-path", type=Path, required=True)
    parser.add_argument("--agent", default="1")
    parser.add_argument("--render-width", type=int, default=480)
    parser.add_argument("--render-height", type=int, default=300)
    parser.add_argument("--frames-per-second", type=float, default=10.0)
    parser.add_argument("--device", default="cuda")
    args = parser.parse_args()
    process = CompositedCameraProcess(
        frames_endpoint=args.frames_endpoint, assets_endpoint=args.assets_endpoint,
        scene_generation=args.scene_generation,
        receiver_id=args.receiver_id,
        health_endpoint=args.health_endpoint,
        splat_path=args.splat, output_dir=args.output_dir, metrics_path=args.metrics_path,
        camera=RemoteCameraConfig.for_cooperscene_agent(
            args.agent, image_size=(args.render_width, args.render_height)
        ),
        frames_per_second=args.frames_per_second, device=args.device,
        cursor_path=args.cursor_path, mesh_cache_dir=args.mesh_cache_dir,
    )
    signal.signal(signal.SIGINT, process.request_stop)
    signal.signal(signal.SIGTERM, process.request_stop)
    process.run()


if __name__ == "__main__":
    main()

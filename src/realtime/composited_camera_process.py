"""Render localized GSplat frames with proxy boxes and ready SAM3D meshes."""

from __future__ import annotations

import argparse
from pathlib import Path
import signal
import time

import cv2
import numpy as np
import zmq

from src.localization.gaussian_map import load_spz_v3
from src.realtime.protocol import AssetEvent, FrameDetections, WorkerState
from src.realtime.runtime_support import HealthHeartbeat, JsonlMetricsWriter
from src.realtime.transport import LatestValueSubscriber, SubscriberTransport
from src.viz.composited_camera import (
    CompositedCameraBackend,
    CudaGaussianBufferRenderer,
    PyTorch3DObjectMeshRenderer,
)


class CompositedFrameWriter:
    """Persist numbered frames and a realtime-cadence MP4."""

    def __init__(self, output_dir: Path, frames_per_second: float) -> None:
        if frames_per_second <= 0:
            raise ValueError("frames_per_second must be positive")
        self.output_dir = Path(output_dir)
        self.frames_dir = self.output_dir / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.frames_per_second = frames_per_second
        self._video: cv2.VideoWriter | None = None
        self._size: tuple[int, int] | None = None
        self.count = 0

    def write(self, frame_id: str, rgb: np.ndarray) -> Path:
        image = np.clip(np.asarray(rgb) * 255.0, 0, 255).astype(np.uint8)
        height, width = image.shape[:2]
        size = (width, height)
        if self._video is None:
            self._size = size
            self._video = cv2.VideoWriter(
                str(self.output_dir / "composited.mp4"),
                cv2.VideoWriter_fourcc(*"mp4v"),
                self.frames_per_second,
                size,
            )
            if not self._video.isOpened():
                raise RuntimeError("failed to open composited MP4 writer")
        if size != self._size:
            raise ValueError(f"composited frame size changed from {self._size} to {size}")
        bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        path = self.frames_dir / f"{self.count:06d}_{frame_id}.png"
        if not cv2.imwrite(str(path), bgr):
            raise RuntimeError(f"failed to write composited frame {path}")
        self._video.write(bgr)
        self.count += 1
        return path

    def close(self) -> None:
        if self._video is not None:
            self._video.release()
            self._video = None


class CompositedCameraProcess:
    """Subscribe to realtime state and assets without blocking reconstruction."""

    def __init__(
        self,
        *,
        frames_endpoint: str,
        assets_endpoint: str,
        health_endpoint: str,
        splat_path: Path,
        output_dir: Path,
        metrics_path: Path,
        frames_per_second: float = 10.0,
        render_downsample: int = 2,
        device: str = "cuda",
    ) -> None:
        self._context = zmq.Context()
        self._frames = LatestValueSubscriber.open(
            self._context, frames_endpoint, bind=False, topics=("frame_detections",), high_water_mark=2
        )
        self._assets = SubscriberTransport.open(
            self._context, assets_endpoint, bind=False, topics=("asset_event",), high_water_mark=100
        )
        self._heartbeat = HealthHeartbeat(
            endpoint=health_endpoint, worker_id="viewer", device=device
        )
        self._metrics = JsonlMetricsWriter(metrics_path)
        self._writer = CompositedFrameWriter(output_dir, frames_per_second)
        gaussian_map = load_spz_v3(splat_path)
        self._backend = CompositedCameraBackend(
            CudaGaussianBufferRenderer(
                gaussian_map, device=device, downsample=render_downsample
            ),
            PyTorch3DObjectMeshRenderer(device=device),
        )
        self._stop = False

    def request_stop(self, *_: object) -> None:
        self._stop = True

    def run(self) -> None:
        poller = zmq.Poller()
        poller.register(self._frames.socket, zmq.POLLIN)
        poller.register(self._assets.socket, zmq.POLLIN)
        self._heartbeat.start()
        self._heartbeat.update(WorkerState.READY)
        try:
            while not self._stop:
                ready = dict(poller.poll(100))
                while self._assets.socket in ready:
                    _, event = self._assets.receive()
                    if not isinstance(event, AssetEvent):
                        raise TypeError("asset endpoint produced a non-asset message")
                    self._backend.handle_asset_event(event)
                    self._metrics.append({
                        "event": "compositor_asset",
                        "track_id": event.track_id,
                        "state": event.state.value,
                    })
                    ready = dict(poller.poll(0))
                if self._frames.socket not in ready:
                    continue
                _, frame, discarded = self._frames.receive_latest()
                if not isinstance(frame, FrameDetections):
                    raise TypeError("frame endpoint produced a non-frame message")
                if frame.end_of_scene:
                    self._metrics.append({"event": "compositor_scene_end", "frame_id": frame.frame_id})
                    continue
                started = time.perf_counter()
                rendered = self._backend.render_frame(frame)
                path = self._writer.write(rendered.frame_id, rendered.rgb)
                elapsed = time.perf_counter() - started
                self._metrics.append({
                    "event": "compositor_frame",
                    "frame_id": frame.frame_id,
                    "timestamp_us": frame.timestamp_us,
                    "discarded_frames": discarded,
                    "track_states": dict(rendered.track_states),
                    "output_path": str(path),
                    "render_ms": elapsed * 1_000.0,
                    "effective_fps": 1.0 / elapsed if elapsed > 0 else 0.0,
                })
                self._heartbeat.update(WorkerState.READY, detail=frame.frame_id)
        except Exception as error:
            self._heartbeat.update(WorkerState.FAILED, detail=f"{type(error).__name__}: {error}")
            raise
        finally:
            self.close()

    def close(self) -> None:
        self._writer.close()
        self._heartbeat.stop()
        self._frames.close()
        self._assets.close()
        self._context.term()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames-endpoint", required=True)
    parser.add_argument("--assets-endpoint", required=True)
    parser.add_argument("--health-endpoint", required=True)
    parser.add_argument("--splat", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--metrics-path", type=Path, required=True)
    parser.add_argument("--frames-per-second", type=float, default=10.0)
    parser.add_argument("--render-downsample", type=int, default=2)
    parser.add_argument("--device", default="cuda")
    args = parser.parse_args()
    process = CompositedCameraProcess(
        frames_endpoint=args.frames_endpoint,
        assets_endpoint=args.assets_endpoint,
        health_endpoint=args.health_endpoint,
        splat_path=args.splat,
        output_dir=args.output_dir,
        metrics_path=args.metrics_path,
        frames_per_second=args.frames_per_second,
        render_downsample=args.render_downsample,
        device=args.device,
    )
    signal.signal(signal.SIGINT, process.request_stop)
    signal.signal(signal.SIGTERM, process.request_stop)
    process.run()


if __name__ == "__main__":
    main()

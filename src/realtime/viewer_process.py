"""Rerun process that continuously replaces box proxies with ready meshes."""

from __future__ import annotations

import argparse
import signal
from time import monotonic

import zmq

from src.realtime.process_support import HealthReporter, JsonlMetrics
from src.realtime.protocol import AssetEvent, FrameDetections, WorkerState
from src.realtime.transport import (
    LatestValueSubscriber,
    PublisherTransport,
    SubscriberTransport,
)
from src.viz.realtime_reconstruction_viewer import (
    RealtimeReconstructionViewer,
    RerunReconstructionBackend,
)


class RealtimeViewerProcess:
    """Own the Rerun SDK and both of its read-only subscriptions."""

    def __init__(
        self,
        *,
        frames_endpoint: str,
        assets_endpoint: str,
        health_endpoint: str,
        metrics_path: str | None = None,
        spawn_viewer: bool = True,
    ) -> None:
        self._context = zmq.Context()
        self._frames = LatestValueSubscriber.open(
            self._context, frames_endpoint, bind=False, topics=("frame_detections",)
        )
        self._assets = SubscriberTransport.open(
            self._context, assets_endpoint, bind=False, topics=("asset_event",)
        )
        health = PublisherTransport.open(
            self._context, health_endpoint, bind=False, high_water_mark=10
        )
        self._health = health
        self._reporter = HealthReporter(health, worker_id="viewer", device="cpu")
        self._metrics = JsonlMetrics(metrics_path)
        self._viewer = RealtimeReconstructionViewer(
            RerunReconstructionBackend(spawn=spawn_viewer)
        )
        self._stopping = False

    def stop(self, *_: object) -> None:
        self._stopping = True

    def run(self) -> None:
        poller = zmq.Poller()
        poller.register(self._frames.socket, zmq.POLLIN)
        poller.register(self._assets.socket, zmq.POLLIN)
        next_health = 0.0
        self._reporter.publish(WorkerState.READY)
        try:
            while not self._stopping:
                ready = dict(poller.poll(100))
                if self._frames.socket in ready:
                    _, message, discarded = self._frames.receive_latest()
                    if not isinstance(message, FrameDetections):
                        raise TypeError("frame endpoint produced a non-frame message")
                    self._viewer.handle_world_state(message)
                    self._metrics.write(
                        "viewer_frame",
                        {
                            "frame_id": message.frame_id,
                            "tracks": len(message.boxes),
                            "discarded_frames": discarded,
                            "end_of_scene": message.end_of_scene,
                        },
                    )
                if self._assets.socket in ready:
                    _, message = self._assets.receive()
                    if not isinstance(message, AssetEvent):
                        raise TypeError("asset endpoint produced a non-asset message")
                    self._viewer.handle_asset_event(message)
                    self._metrics.write(
                        "viewer_asset",
                        {
                            "request_id": message.request_id,
                            "track_id": message.track_id,
                            "state": message.state.value,
                        },
                    )
                now = monotonic()
                if now >= next_health:
                    self._reporter.publish(WorkerState.READY)
                    next_health = now + 1.0
        except Exception as error:
            self._reporter.publish(WorkerState.FAILED, detail=str(error))
            raise
        finally:
            self._reporter.publish(WorkerState.STOPPING)
            self.close()

    def close(self) -> None:
        self._frames.close()
        self._assets.close()
        self._health.close()
        self._context.term()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames-endpoint", required=True)
    parser.add_argument("--assets-endpoint", required=True)
    parser.add_argument("--health-endpoint", required=True)
    parser.add_argument("--metrics-path")
    parser.add_argument("--no-spawn", action="store_true")
    args = parser.parse_args()
    process = RealtimeViewerProcess(
        frames_endpoint=args.frames_endpoint,
        assets_endpoint=args.assets_endpoint,
        health_endpoint=args.health_endpoint,
        metrics_path=args.metrics_path,
        spawn_viewer=not args.no_spawn,
    )
    signal.signal(signal.SIGINT, process.stop)
    signal.signal(signal.SIGTERM, process.stop)
    process.run()


if __name__ == "__main__":
    main()

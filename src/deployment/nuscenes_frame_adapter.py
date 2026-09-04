"""Direct durable adapter for Bilateral Driving processed nuScenes scenes."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import signal
from threading import Event
import time
from typing import Callable

import numpy as np
import zmq

from src.data.processed_nuscenes import ProcessedNuScenesFrame, ProcessedNuScenesSequence
from src.deployment.ros_frame_adapter import FrameRouterService
from src.deployment.vehicle import VehicleFrameSession, VehicleFrameSpool
from src.realtime.process_support import JsonlMetrics
from src.realtime.protocol import BoxPrompt, FrameEnd, FrameSnapshot
from src.realtime.transport import RouterTransport
from src.reconstruction.models import VehicleDimensions
from src.reconstruction.projection import project_vehicle_box


@dataclass(frozen=True)
class NuScenesFrameAdapterConfig:
    scene_generation: str
    frames_per_second: float = 10.0
    rate: float = 1.0
    minimum_area_px: float = 64.0

    def __post_init__(self) -> None:
        if not self.scene_generation:
            raise ValueError("scene_generation must be non-empty")
        if self.frames_per_second <= 0 or self.rate <= 0:
            raise ValueError("frames_per_second and rate must be positive")
        if self.minimum_area_px < 0:
            raise ValueError("minimum_area_px must be non-negative")


class NuScenesFrameAdapter:
    """Pace processed frames into a crash-safe spool, independent of ROS/MCAP."""

    def __init__(
        self,
        spool: VehicleFrameSpool,
        config: NuScenesFrameAdapterConfig,
        *,
        metrics: JsonlMetrics | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.spool = spool
        self.config = config
        self.metrics = metrics or JsonlMetrics(None)
        self._monotonic = monotonic
        self._sleep = sleep

    def process(self, sequence: ProcessedNuScenesSequence) -> int:
        """Append and finalize the sequence at its configured nominal cadence."""

        if not sequence:
            return 0
        started = self._monotonic()
        previous_append = started
        for sequence_index, frame in enumerate(sequence):
            target = started + sequence_index / self.config.frames_per_second / self.config.rate
            remaining = target - self._monotonic()
            if remaining > 0:
                self._sleep(remaining)
            build_started = self._monotonic()
            snapshot = self.snapshot(frame, sequence_index)
            self.spool.append(snapshot)
            appended_at = self._monotonic()
            interval = appended_at - previous_append
            previous_append = appended_at
            self.metrics.write("frame_spooled", {
                "scene_generation": self.config.scene_generation,
                "sequence": sequence_index,
                "frame_id": frame.frame_id,
                "source_timestamp_us": frame.timestamp_us,
                "box_count": len(snapshot.boxes),
                "build_and_fsync_ms": (appended_at - build_started) * 1_000.0,
                "append_interval_ms": interval * 1_000.0,
                "nominal_fps": self.config.frames_per_second * self.config.rate,
            })
        end = FrameEnd(
            scene_generation=self.config.scene_generation,
            final_sequence=len(sequence) - 1,
            final_timestamp_us=sequence[-1].timestamp_us,
        )
        self.spool.finalize(end)
        self.metrics.write("scene_spooled", {
            "scene_generation": self.config.scene_generation,
            "frame_count": len(sequence),
            "final_timestamp_us": end.final_timestamp_us,
        })
        return len(sequence)

    def snapshot(self, frame: ProcessedNuScenesFrame, sequence: int) -> FrameSnapshot:
        boxes = []
        for vehicle in frame.vehicles:
            if vehicle.category != "vehicle.car":
                continue
            projected = project_vehicle_box(
                world_T_object=vehicle.map_T_object,
                world_T_camera=frame.map_T_camera,
                intrinsics=frame.camera_intrinsic,
                dimensions=VehicleDimensions(*vehicle.dimensions_lwh),
                image_size=frame.image.shape[:2],
                minimum_area_px=self.config.minimum_area_px,
            )
            if projected is None:
                continue
            boxes.append(BoxPrompt(
                track_id=vehicle.track_id,
                xyxy=tuple(float(value) for value in projected.xyxy),
                dimensions_lwh=vehicle.dimensions_lwh,
                world_T_object=_matrix4_tuple(vehicle.map_T_object),
                text_label="car",
                visibility=projected.visible_fraction,
                detection_confidence=1.0,
            ))
        return FrameSnapshot(
            sequence=sequence,
            timestamp_us=frame.timestamp_us,
            scene_generation=self.config.scene_generation,
            frame_id=frame.frame_id,
            world_T_ego=_matrix4_tuple(frame.map_T_lidar),
            world_T_camera=_matrix4_tuple(frame.map_T_camera),
            boxes=tuple(boxes),
        )

    def router_session(self, router: RouterTransport) -> VehicleFrameSession:
        """Expose the populated spool through the existing receiver-pulled contract."""

        return VehicleFrameSession(router, self.spool)


def _matrix4_tuple(matrix: np.ndarray) -> tuple[float, ...]:
    values = np.asarray(matrix, dtype=np.float64)
    if values.shape != (4, 4):
        raise ValueError("transform must have shape (4, 4)")
    return tuple(float(value) for value in values.reshape(-1))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-root", type=Path, required=True)
    parser.add_argument("--frames-endpoint", default="tcp://0.0.0.0:5555")
    parser.add_argument("--expected-samples", type=int, default=191)
    parser.add_argument("--scene-generation", required=True)
    parser.add_argument("--metrics-path", type=Path)
    parser.add_argument(
        "--spool-root", type=Path, default=Path("artifacts/vehicle_frame_spool")
    )
    parser.add_argument("--frames-per-second", type=float, default=10.0)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--minimum-area-px", type=float, default=64.0)
    parser.add_argument("--high-water-mark", type=int, default=2_048)
    return parser


def serve_processed_scene(args: argparse.Namespace, stop: Event) -> int:
    """Spool one scene while serving requests, then wait for shutdown."""

    if args.expected_samples <= 0:
        raise ValueError("expected_samples must be positive")
    if args.high_water_mark <= 0:
        raise ValueError("high_water_mark must be positive")
    sequence = ProcessedNuScenesSequence(
        args.scene_root, expected_frame_count=args.expected_samples
    )
    context = zmq.Context()
    router = RouterTransport.open(
        context,
        args.frames_endpoint,
        bind=True,
        high_water_mark=args.high_water_mark,
    )
    spool = VehicleFrameSpool(args.spool_root)
    adapter = NuScenesFrameAdapter(
        spool,
        NuScenesFrameAdapterConfig(
            scene_generation=args.scene_generation,
            frames_per_second=args.frames_per_second,
            rate=args.rate,
            minimum_area_px=args.minimum_area_px,
        ),
        metrics=JsonlMetrics(args.metrics_path),
    )
    service = FrameRouterService(adapter.router_session(router))
    try:
        service.start()
        frame_count = adapter.process(sequence)
        while not stop.wait(0.25):
            service.raise_if_failed()
        service.raise_if_failed()
        return frame_count
    finally:
        service.stop()
        router.close()
        context.term()


def main() -> None:
    parser = _parser()
    args = parser.parse_args()
    if args.expected_samples <= 0:
        parser.error("--expected-samples must be positive")
    if args.frames_per_second <= 0:
        parser.error("--frames-per-second must be positive")
    if args.rate <= 0:
        parser.error("--rate must be positive")
    if args.minimum_area_px < 0:
        parser.error("--minimum-area-px must be non-negative")
    if args.high_water_mark <= 0:
        parser.error("--high-water-mark must be positive")

    stop = Event()
    previous_handlers = {}

    def request_stop(_signum: int, _frame: object) -> None:
        stop.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, request_stop)
    try:
        serve_processed_scene(args, stop)
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    main()

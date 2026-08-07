"""Publish nuScenes front-camera keyframes and projected GT car tracks."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
from pathlib import Path
import time
from typing import Callable, Protocol, Sequence

import cv2
import numpy as np
import zmq

from src.data.nuscenes_loader import (
    NuScenesFrame,
    NuScenesSequence,
    NuScenesSequenceDataset,
)
from src.realtime.protocol import BoxPrompt, FrameDetections, WorkerState
from src.realtime.runtime_support import HealthHeartbeat, JsonlMetricsWriter
from src.realtime.transport import PublisherTransport
from src.reconstruction.models import VehicleDimensions
from src.reconstruction.projection import project_vehicle_box


class FramePublisher(Protocol):
    def send(self, message: FrameDetections, *, topic: str | None = None) -> None: ...


@dataclass(frozen=True)
class ReplayConfig:
    scene_generation: str
    rate: float = 1.0
    minimum_area_px: float = 64.0
    jpeg_quality: int = 90
    scene_end_grace_seconds: float = 0.05

    def __post_init__(self) -> None:
        if not self.scene_generation:
            raise ValueError("scene_generation must be non-empty")
        if self.rate <= 0:
            raise ValueError("rate must be positive")
        if self.minimum_area_px < 0:
            raise ValueError("minimum_area_px must be non-negative")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")
        if self.scene_end_grace_seconds < 0:
            raise ValueError("scene_end_grace_seconds must be non-negative")


class NuScenesReplayPublisher:
    """Replay one sequence at its recorded timestamp cadence."""

    def __init__(
        self,
        publisher: FramePublisher,
        config: ReplayConfig,
        *,
        metrics: JsonlMetricsWriter | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.publisher = publisher
        self.config = config
        self.metrics = metrics or JsonlMetricsWriter(None)
        self._monotonic = monotonic
        self._sleep = sleep

    def run(self, sequence: Sequence[NuScenesFrame]) -> int:
        if not sequence:
            return 0
        first_timestamp_us = sequence[0].timestamp_us
        replay_started = self._monotonic()
        previous_send = replay_started
        last_message: FrameDetections | None = None
        for frame_index, frame in enumerate(sequence):
            target = replay_started + (
                (frame.timestamp_us - first_timestamp_us) / 1_000_000.0 / self.config.rate
            )
            remaining = target - self._monotonic()
            if remaining > 0:
                self._sleep(remaining)
            build_started = self._monotonic()
            message = frame_to_detections(frame, self.config)
            self.publisher.send(message)
            last_message = message
            sent_at = self._monotonic()
            interval = sent_at - previous_send
            previous_send = sent_at
            elapsed = sent_at - replay_started
            self.metrics.append(
                {
                    "event": "frame_published",
                    "frame_index": frame_index,
                    "frame_id": message.frame_id,
                    "scene_generation": message.scene_generation,
                    "source_timestamp_us": message.timestamp_us,
                    "box_count": len(message.boxes),
                    "track_ids": [box.track_id for box in message.boxes],
                    "jpeg_bytes": len(message.image_jpeg),
                    "build_latency_ms": (sent_at - build_started) * 1_000.0,
                    "publish_interval_ms": interval * 1_000.0,
                    "instantaneous_fps": 1.0 / interval if interval > 0 else 0.0,
                    "cumulative_fps": (frame_index + 1) / elapsed if elapsed > 0 else 0.0,
                }
            )
        if self.config.scene_end_grace_seconds > 0:
            self._sleep(self.config.scene_end_grace_seconds)
        assert last_message is not None
        self.publisher.send(
            replace(
                last_message,
                frame_id=f"{last_message.frame_id}:scene-end",
                request_id=f"{last_message.request_id}:scene-end",
                boxes=(),
                end_of_scene=True,
            )
        )
        self.metrics.append(
            {
                "event": "scene_end_published",
                "scene_generation": self.config.scene_generation,
                "source_timestamp_us": last_message.timestamp_us,
            }
        )
        if self.config.scene_end_grace_seconds > 0:
            self._sleep(self.config.scene_end_grace_seconds)
        return len(sequence)


def frame_to_detections(
    frame: NuScenesFrame,
    config: ReplayConfig,
) -> FrameDetections:
    """Convert one keyframe into the strict real-time wire contract."""

    boxes: list[BoxPrompt] = []
    for vehicle in frame.vehicle_boxes:
        if vehicle.category != "vehicle.car":
            continue
        dimensions = VehicleDimensions(*vehicle.dimensions_lwh)
        projected = project_vehicle_box(
            world_T_object=vehicle.world_T_object,
            world_T_camera=frame.world_T_camera,
            intrinsics=frame.camera_intrinsic,
            dimensions=dimensions,
            image_size=frame.image.shape[:2],
            minimum_area_px=config.minimum_area_px,
        )
        if projected is None:
            continue
        boxes.append(
            BoxPrompt(
                track_id=vehicle.instance_token,
                xyxy=tuple(float(value) for value in projected.xyxy),
                dimensions_lwh=tuple(float(value) for value in vehicle.dimensions_lwh),
                world_T_object=_matrix4_tuple(vehicle.world_T_object),
                visibility=projected.visible_fraction,
                detection_confidence=1.0,
            )
        )
    success, jpeg = cv2.imencode(
        ".jpg",
        cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR),
        [cv2.IMWRITE_JPEG_QUALITY, config.jpeg_quality],
    )
    if not success:
        raise RuntimeError(f"failed to JPEG-encode frame {frame.sample_token}")
    return FrameDetections(
        frame_id=frame.sample_token,
        request_id=f"{config.scene_generation}:{frame.sample_token}",
        scene_generation=config.scene_generation,
        timestamp_us=frame.timestamp_us,
        image_jpeg=jpeg.tobytes(),
        camera_intrinsic=tuple(float(value) for value in frame.camera_intrinsic.reshape(-1)),
        world_T_camera=_matrix4_tuple(frame.world_T_camera),
        world_T_ego=_matrix4_tuple(frame.world_T_ego),
        boxes=tuple(boxes),
    )


def _matrix4_tuple(matrix: np.ndarray) -> tuple[float, ...]:
    values = np.asarray(matrix, dtype=np.float64)
    if values.shape != (4, 4):
        raise ValueError("transform must have shape (4, 4)")
    return tuple(float(value) for value in values.reshape(-1))


def _select_sequence(
    dataset: NuScenesSequenceDataset, scene: str
) -> NuScenesSequence:
    for sequence in dataset:
        if sequence.name == scene or sequence.scene_token == scene:
            return sequence
    try:
        return dataset[int(scene)]
    except (ValueError, IndexError):
        raise ValueError(f"nuScenes scene not found: {scene}") from None


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataroot", type=Path, required=True)
    parser.add_argument("--version", default="v1.0-mini")
    parser.add_argument("--scene", default="scene-0061")
    parser.add_argument("--frames-endpoint", default="tcp://127.0.0.1:5555")
    parser.add_argument("--health-endpoint", default="tcp://127.0.0.1:5559")
    parser.add_argument(
        "--metrics-path",
        type=Path,
        default=Path("artifacts/realtime/metrics/replay.jsonl"),
    )
    parser.add_argument("--scene-generation")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--minimum-area-px", type=float, default=64.0)
    parser.add_argument("--jpeg-quality", type=int, default=90)
    parser.add_argument("--startup-delay-seconds", type=float, default=0.5)
    parser.add_argument("--scene-end-grace-seconds", type=float, default=0.05)
    return parser


def main() -> None:
    args = _parser().parse_args()
    dataset = NuScenesSequenceDataset(args.dataroot, version=args.version)
    sequence = _select_sequence(dataset, args.scene)
    scene_generation = args.scene_generation or f"{sequence.name}:{time.time_ns()}"
    config = ReplayConfig(
        scene_generation=scene_generation,
        rate=args.rate,
        minimum_area_px=args.minimum_area_px,
        jpeg_quality=args.jpeg_quality,
        scene_end_grace_seconds=args.scene_end_grace_seconds,
    )
    metrics = JsonlMetricsWriter(args.metrics_path)
    context = zmq.Context()
    publisher = PublisherTransport.open(
        context, args.frames_endpoint, bind=True, high_water_mark=2
    )
    heartbeat = HealthHeartbeat(
        endpoint=args.health_endpoint,
        worker_id="nuscenes-replay",
        device="cpu",
    )
    heartbeat.start()
    try:
        heartbeat.update(WorkerState.READY, detail=scene_generation)
        if args.startup_delay_seconds > 0:
            time.sleep(args.startup_delay_seconds)
        published = NuScenesReplayPublisher(
            publisher, config, metrics=metrics
        ).run(sequence)
        metrics.append(
            {
                "event": "replay_complete",
                "scene": sequence.name,
                "scene_generation": scene_generation,
                "published_frames": published,
            }
        )
    except Exception as exc:
        heartbeat.update(WorkerState.FAILED, detail=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        heartbeat.stop()
        publisher.close(linger_ms=1_000)
        context.term()


if __name__ == "__main__":
    main()

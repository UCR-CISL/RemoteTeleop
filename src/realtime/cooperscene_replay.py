"""Convert and replay one CooperScene vehicle's camera frames."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import json
from pathlib import Path
import signal
import time
from typing import Callable, Protocol, Sequence

import cv2
import numpy as np
import zmq

from src.localization.cooperscene import CooperSceneFrame, CooperSceneSequenceDataset
from src.localization.cooperscene_calibration import (
    camera_T_lidar,
    camera_intrinsic,
    compose_map_T_camera,
)
from src.realtime.protocol import BoxPrompt, FrameDetections
from src.localization.cooperscene_localization import _as_transform
from src.realtime.protocol import WorkerState
from src.realtime.runtime_support import HealthHeartbeat, JsonlMetricsWriter
from src.realtime.transport import PublisherTransport
from src.reconstruction.models import VehicleDimensions
from src.reconstruction.projection import project_vehicle_box


class FramePublisher(Protocol):
    def send(self, message: FrameDetections, *, topic: str | None = None) -> None: ...


@dataclass(frozen=True)
class CooperSceneReplayConfig:
    scene_generation: str
    agent: str = "1"
    frames_per_second: float = 10.0
    rate: float = 1.0
    minimum_area_px: float = 64.0
    jpeg_quality: int = 90
    scene_end_grace_seconds: float = 0.05
    stop_at_overlap: bool = False

    def __post_init__(self) -> None:
        if not self.scene_generation:
            raise ValueError("scene_generation must be non-empty")
        if self.frames_per_second <= 0 or self.rate <= 0:
            raise ValueError("frames_per_second and rate must be positive")
        if self.minimum_area_px < 0:
            raise ValueError("minimum_area_px must be non-negative")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")
        if self.scene_end_grace_seconds < 0:
            raise ValueError("scene_end_grace_seconds must be non-negative")


class CooperSceneReplayPublisher:
    """Replay selected-agent frames at the dataset's fixed recorded cadence."""

    def __init__(
        self,
        publisher: FramePublisher,
        config: CooperSceneReplayConfig,
        gaussian_T_cooperscene: np.ndarray,
        *,
        metrics: JsonlMetricsWriter | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        transform = np.asarray(gaussian_T_cooperscene, dtype=np.float64)
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise ValueError("gaussian_T_cooperscene must be a finite 4x4 transform")
        self.publisher = publisher
        self.config = config
        self.gaussian_T_cooperscene = transform.copy()
        self.metrics = metrics or JsonlMetricsWriter(None)
        self._monotonic = monotonic
        self._sleep = sleep

    def run(
        self,
        sequence: Sequence[CooperSceneFrame],
        *,
        overlap_allows: Callable[[CooperSceneFrame], bool] | None = None,
    ) -> int:
        """Publish all frames unless explicitly configured to apply an overlap gate."""

        started = self._monotonic()
        previous_send = started
        last_message: FrameDetections | None = None
        published = 0
        for sequence_index in range(len(sequence)):
            frame_load_started = self._monotonic()
            frame = sequence[sequence_index]
            frame_load_ms = (self._monotonic() - frame_load_started) * 1_000.0
            if (
                self.config.stop_at_overlap
                and overlap_allows is not None
                and not overlap_allows(frame)
            ):
                break
            target = started + published / self.config.frames_per_second / self.config.rate
            remaining = target - self._monotonic()
            if remaining > 0:
                self._sleep(remaining)
            build_started = self._monotonic()
            conversion_timings: dict[str, float] = {}
            message = cooper_scene_frame_to_detections(
                frame,
                self.config,
                self.gaussian_T_cooperscene,
                timestamp_us=round(published * 1_000_000 / self.config.frames_per_second),
                timings=conversion_timings,
            )
            self.publisher.send(message)
            last_message = message
            sent_at = self._monotonic()
            self.metrics.append({
                "event": "frame_published",
                "frame_index": published,
                "frame_id": message.frame_id,
                "scene_generation": message.scene_generation,
                "source_timestamp_us": message.timestamp_us,
                "box_count": len(message.boxes),
                "track_ids": [box.track_id for box in message.boxes],
                "jpeg_bytes": len(message.image_jpeg),
                "frame_metadata_load_ms": frame_load_ms,
                "pcd_read_ms": 0.0,
                "lidar_points_loaded": frame.has_lidar_points,
                **conversion_timings,
                "build_latency_ms": (sent_at - build_started) * 1_000.0,
                "publish_interval_ms": (sent_at - previous_send) * 1_000.0,
            })
            previous_send = sent_at
            published += 1

        if last_message is None:
            return 0
        if self.config.scene_end_grace_seconds > 0:
            self._sleep(self.config.scene_end_grace_seconds)
        self.publisher.send(replace(
            last_message,
            frame_id=f"{last_message.frame_id}:scene-end",
            request_id=f"{last_message.request_id}:scene-end",
            boxes=(),
            end_of_scene=True,
        ))
        return published


def cooper_scene_frame_to_detections(
    frame: CooperSceneFrame,
    config: CooperSceneReplayConfig,
    gaussian_T_cooperscene: np.ndarray,
    *,
    timestamp_us: int,
    timings: dict[str, float] | None = None,
) -> FrameDetections:
    """Convert selected-agent GT boxes and its original camera0 image."""

    if frame.agent != str(config.agent):
        raise ValueError(f"frame agent {frame.agent} does not match configured agent {config.agent}")
    if frame.camera_path is None:
        raise FileNotFoundError(f"frame {frame.frame_id} has no camera0 image")
    image_started = time.perf_counter()
    image = cv2.imread(str(frame.camera_path), cv2.IMREAD_COLOR)
    image_read_ms = (time.perf_counter() - image_started) * 1_000.0
    if image is None:
        raise ValueError(f"failed to decode camera image: {frame.camera_path}")

    projection_started = time.perf_counter()
    gaussian_T_cooperscene = np.asarray(gaussian_T_cooperscene, dtype=np.float64)
    if gaussian_T_cooperscene.shape != (4, 4):
        raise ValueError("gaussian_T_cooperscene must have shape (4, 4)")
    intrinsic = camera_intrinsic(config.agent)
    world_T_ego = gaussian_T_cooperscene @ frame.map_T_lidar
    world_T_camera = compose_map_T_camera(world_T_ego, camera_T_lidar(config.agent))
    boxes = []
    for vehicle in frame.vehicle_boxes:
        world_T_object = gaussian_T_cooperscene @ vehicle.map_T_object
        projected = project_vehicle_box(
            world_T_object=world_T_object,
            world_T_camera=world_T_camera,
            intrinsics=intrinsic,
            dimensions=VehicleDimensions(*vehicle.dimensions_lwh),
            image_size=image.shape[:2],
            minimum_area_px=config.minimum_area_px,
        )
        if projected is None:
            continue
        boxes.append(BoxPrompt(
            track_id=vehicle.track_id,
            xyxy=tuple(float(value) for value in projected.xyxy),
            dimensions_lwh=vehicle.dimensions_lwh,
            world_T_object=_matrix4_tuple(world_T_object),
            visibility=projected.visible_fraction,
            detection_confidence=1.0,
        ))
    projection_ms = (time.perf_counter() - projection_started) * 1_000.0
    jpeg_started = time.perf_counter()
    success, jpeg = cv2.imencode(
        ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, config.jpeg_quality]
    )
    if not success:
        raise RuntimeError(f"failed to JPEG-encode frame {frame.frame_id}")
    jpeg_encode_ms = (time.perf_counter() - jpeg_started) * 1_000.0
    if timings is not None:
        timings.update({
            "camera_read_decode_ms": image_read_ms,
            "pose_box_projection_ms": projection_ms,
            "jpeg_encode_ms": jpeg_encode_ms,
        })
    return FrameDetections(
        frame_id=frame.frame_id,
        request_id=f"{config.scene_generation}:{frame.frame_id}",
        scene_generation=config.scene_generation,
        timestamp_us=timestamp_us,
        image_jpeg=jpeg.tobytes(),
        camera_intrinsic=tuple(float(value) for value in intrinsic.reshape(-1)),
        world_T_camera=_matrix4_tuple(world_T_camera),
        world_T_ego=_matrix4_tuple(world_T_ego),
        boxes=tuple(boxes),
    )


def _matrix4_tuple(matrix: np.ndarray) -> tuple[float, ...]:
    return tuple(float(value) for value in np.asarray(matrix, dtype=np.float64).reshape(16))


def _load_transform(path: Path) -> np.ndarray:
    if path.suffix == ".npz":
        with np.load(path) as archive:
            for key in ("gaussian_T_cooperscene", "map_T_source", "transform"):
                if key in archive:
                    return _as_transform(archive[key], path)
    elif path.suffix == ".json":
        payload = json.loads(path.read_text(encoding="utf-8"))
        for key in ("gaussian_T_cooperscene", "map_T_source", "transform"):
            if key in payload:
                return _as_transform(payload[key], path)
    else:
        raise ValueError("transform must end in .json or .npz")
    raise ValueError(f"no Gaussian-to-CooperScene transform found in {path}")


def _overlap_gate(path: Path | None) -> Callable[[CooperSceneFrame], bool] | None:
    if path is None:
        return None
    payload = json.loads(path.read_text(encoding="utf-8"))
    first_low = payload.get("first_low_frame_id")
    if first_low is None:
        return lambda _frame: True
    first_low = str(first_low)
    return lambda frame: frame.frame_id != first_low


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--split", default="train")
    parser.add_argument("--scenario", default="1")
    parser.add_argument("--agent", choices=("1", "2", "3"), default="1")
    parser.add_argument("--transform", type=Path, required=True)
    parser.add_argument("--overlap-manifest", type=Path)
    parser.add_argument(
        "--stop-at-overlap",
        action="store_true",
        help="Stop before the overlap manifest's first low-overlap frame.",
    )
    parser.add_argument("--frames-endpoint", default="tcp://127.0.0.1:5555")
    parser.add_argument("--health-endpoint", default="tcp://127.0.0.1:5559")
    parser.add_argument("--metrics-path", type=Path, default=Path("artifacts/realtime/metrics/replay.jsonl"))
    parser.add_argument("--scene-generation")
    parser.add_argument("--frames-per-second", type=float, default=10.0)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--minimum-area-px", type=float, default=64.0)
    parser.add_argument("--jpeg-quality", type=int, default=90)
    parser.add_argument("--startup-delay-seconds", type=float, default=0.5)
    args = parser.parse_args()
    if args.stop_at_overlap and args.overlap_manifest is None:
        parser.error("--stop-at-overlap requires --overlap-manifest")

    generation = args.scene_generation or (
        f"cooperscene:{args.split}:{args.scenario}:{args.agent}:{time.time_ns()}"
    )
    context = zmq.Context()
    publisher = PublisherTransport.open(
        context, args.frames_endpoint, bind=True, high_water_mark=2
    )
    heartbeat = HealthHeartbeat(
        endpoint=args.health_endpoint, worker_id="cooperscene-replay", device="cpu"
    )
    heartbeat.start()
    try:
        sequence = CooperSceneSequenceDataset.metadata_only(
            args.data_root, cache_frames=False
        ).sequence(args.split, args.scenario, args.agent)
        replay = CooperSceneReplayPublisher(
            publisher,
            CooperSceneReplayConfig(
                scene_generation=generation,
                agent=args.agent,
                frames_per_second=args.frames_per_second,
                rate=args.rate,
                minimum_area_px=args.minimum_area_px,
                jpeg_quality=args.jpeg_quality,
                stop_at_overlap=args.stop_at_overlap,
            ),
            _load_transform(args.transform),
            metrics=JsonlMetricsWriter(args.metrics_path),
        )
        heartbeat.update(WorkerState.READY, detail=generation)
        if args.startup_delay_seconds > 0:
            time.sleep(args.startup_delay_seconds)
        replay.run(
            sequence,
            overlap_allows=(
                _overlap_gate(args.overlap_manifest) if args.stop_at_overlap else None
            ),
        )
    except Exception as error:
        heartbeat.update(WorkerState.FAILED, detail=f"{type(error).__name__}: {error}")
        raise
    finally:
        heartbeat.stop()
        publisher.close()
        context.term()


if __name__ == "__main__":
    main()

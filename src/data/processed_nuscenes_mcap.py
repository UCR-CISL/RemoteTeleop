"""Convert all Bilateral Driving processed nuScenes scenes to ROS 2 MCAP."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import struct
from typing import BinaryIO, Callable

import numpy as np

from src.data.cooperscene_mcap import (
    McapWriter,
    ROS2_IMAGE_SCHEMA,
    ROS2_STRING_SCHEMA,
    _mcap_writer,
    _serialize_image,
    _serialize_metadata,
)
from src.data.data_processor import DataProcessor
from src.data.processed_nuscenes import ProcessedNuScenesFrame, ProcessedNuScenesSequence
from src.realtime.protocol import BoxPrompt, FrameDetections
from src.reconstruction.models import VehicleDimensions
from src.reconstruction.projection import project_vehicle_box


ROS2_POINT_CLOUD2_SCHEMA = b"""std_msgs/Header header
uint32 height
uint32 width
sensor_msgs/PointField[] fields
bool is_bigendian
uint32 point_step
uint32 row_step
uint8[] data
bool is_dense

================================================================================
MSG: sensor_msgs/PointField
uint8 INT8=1
uint8 UINT8=2
uint8 INT16=3
uint8 UINT16=4
uint8 INT32=5
uint8 UINT32=6
uint8 FLOAT32=7
uint8 FLOAT64=8
string name
uint32 offset
uint8 datatype
uint32 count

================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id

================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""


@dataclass(frozen=True)
class ProcessedNuScenesMcapConfig:
    processed_root: Path
    output_root: Path
    scenes: tuple[str, ...] | None = None
    expected_samples: int | None = None
    frames_per_second: float = 10.0
    minimum_area_px: float = 64.0
    scene_generation_prefix: str = "nuscenes"
    image_topic: str = "/camera/image_raw"
    metadata_topic: str = "/camera/frame_detections"
    lidar_topic: str = "/lidar/points"
    camera_frame_id: str = "CAM_FRONT"

    def __post_init__(self) -> None:
        object.__setattr__(self, "processed_root", Path(self.processed_root))
        object.__setattr__(self, "output_root", Path(self.output_root))
        if self.scenes is not None:
            scenes = tuple(str(scene) for scene in self.scenes)
            if not scenes or any(not scene for scene in scenes):
                raise ValueError("scenes must contain non-empty scene names")
            if len(scenes) != len(set(scenes)):
                raise ValueError("scenes must not contain duplicates")
            object.__setattr__(self, "scenes", scenes)
        if self.expected_samples is not None and self.expected_samples <= 0:
            raise ValueError("expected_samples must be positive when provided")
        if self.frames_per_second <= 0:
            raise ValueError("frames_per_second must be positive")
        if self.minimum_area_px < 0:
            raise ValueError("minimum_area_px must be non-negative")
        if not self.scene_generation_prefix or not self.camera_frame_id:
            raise ValueError("scene_generation_prefix and camera_frame_id must be non-empty")
        for name in ("image_topic", "metadata_topic", "lidar_topic"):
            if not getattr(self, name).startswith("/"):
                raise ValueError(f"{name} must be an absolute ROS topic")
        if len({self.image_topic, self.metadata_topic, self.lidar_topic}) != 3:
            raise ValueError("image, metadata and LiDAR topics must differ")


class ProcessedNuScenesMcapProcessor(DataProcessor):
    """Write one independently replayable MCAP per processed scene."""

    def __init__(
        self,
        config: ProcessedNuScenesMcapConfig,
        *,
        sequence_factory: Callable[..., ProcessedNuScenesSequence] = ProcessedNuScenesSequence,
        writer_factory: Callable[[BinaryIO], McapWriter] | None = None,
    ) -> None:
        self.config = config
        self._sequence_factory = sequence_factory
        self._writer_factory = writer_factory or _mcap_writer
        self.outputs: dict[str, Path] = {}

    def process(self) -> int:
        """Convert selected scenes and return the total number of frame pairs."""

        scenes = self.config.scenes or self._discover_scenes()
        if not scenes:
            raise FileNotFoundError(
                f"no processed nuScenes scene directories under {self.config.processed_root}"
            )
        self.config.output_root.mkdir(parents=True, exist_ok=True)
        total = 0
        for scene in scenes:
            scene_root = self.config.processed_root / scene
            sequence = self._sequence_factory(
                scene_root,
                expected_frame_count=self.config.expected_samples,
                timestamp_step_us=round(1_000_000 / self.config.frames_per_second),
            )
            output_path = self.config.output_root / f"{scene}.mcap"
            self._write_scene(scene, sequence, output_path)
            self.outputs[scene] = output_path
            total += len(sequence)
        return total

    def _discover_scenes(self) -> tuple[str, ...]:
        return tuple(
            path.name
            for path in sorted(
                (path for path in self.config.processed_root.iterdir() if path.is_dir()),
                key=lambda path: (not path.name.isdigit(), int(path.name) if path.name.isdigit() else path.name),
            )
            if (path / "images").is_dir() and (path / "instances").is_dir()
        )

    def _write_scene(
        self,
        scene: str,
        sequence: ProcessedNuScenesSequence,
        output_path: Path,
    ) -> None:
        with output_path.open("wb") as output:
            writer = self._writer_factory(output)
            writer.start(profile="ros2")
            try:
                image_schema = writer.register_schema(
                    name="sensor_msgs/msg/Image", encoding="ros2msg", data=ROS2_IMAGE_SCHEMA
                )
                image_channel = writer.register_channel(
                    topic=self.config.image_topic,
                    message_encoding="cdr",
                    schema_id=image_schema,
                )
                metadata_schema = writer.register_schema(
                    name="std_msgs/msg/String", encoding="ros2msg", data=ROS2_STRING_SCHEMA
                )
                metadata_channel = writer.register_channel(
                    topic=self.config.metadata_topic,
                    message_encoding="cdr",
                    schema_id=metadata_schema,
                )
                lidar_schema = writer.register_schema(
                    name="sensor_msgs/msg/PointCloud2",
                    encoding="ros2msg",
                    data=ROS2_POINT_CLOUD2_SCHEMA,
                )
                lidar_channel = writer.register_channel(
                    topic=self.config.lidar_topic,
                    message_encoding="cdr",
                    schema_id=lidar_schema,
                )
                for index, frame in enumerate(sequence):
                    timestamp_ns = frame.timestamp_us * 1_000
                    detections = _frame_to_detections(
                        frame,
                        scene_generation=f"{self.config.scene_generation_prefix}:{scene}",
                        source_sequence=index,
                        minimum_area_px=self.config.minimum_area_px,
                    )
                    height, width = frame.image.shape[:2]
                    writer.add_message(
                        channel_id=image_channel,
                        log_time=timestamp_ns,
                        publish_time=timestamp_ns,
                        sequence=index,
                        data=_serialize_image(
                            timestamp_ns=timestamp_ns,
                            frame_id=self.config.camera_frame_id,
                            height=height,
                            width=width,
                            data=np.ascontiguousarray(frame.image).tobytes(),
                        ),
                    )
                    writer.add_message(
                        channel_id=metadata_channel,
                        log_time=timestamp_ns,
                        publish_time=timestamp_ns,
                        sequence=index,
                        data=_serialize_metadata(
                            detections,
                            camera_distortion=tuple(float(value) for value in frame.distortion),
                            map_T_lidar=_matrix4_tuple(frame.map_T_lidar),
                            map_T_nuscenes_global=_matrix4_tuple(sequence.map_T_global),
                        ),
                    )
                    writer.add_message(
                        channel_id=lidar_channel,
                        log_time=timestamp_ns,
                        publish_time=timestamp_ns,
                        sequence=index,
                        data=_serialize_point_cloud2(
                            timestamp_ns=timestamp_ns,
                            frame_id="LIDAR_TOP",
                            points=frame.lidar_points,
                        ),
                    )
            finally:
                writer.finish()


def _frame_to_detections(
    frame: ProcessedNuScenesFrame,
    *,
    scene_generation: str,
    source_sequence: int,
    minimum_area_px: float,
) -> FrameDetections:
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
            minimum_area_px=minimum_area_px,
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
    return FrameDetections(
        frame_id=frame.frame_id,
        request_id=f"{scene_generation}:{frame.frame_id}",
        scene_generation=scene_generation,
        timestamp_us=frame.timestamp_us,
        image_jpeg=b"image-is-stored-in-the-mcap-image-channel",
        camera_intrinsic=tuple(float(value) for value in frame.camera_intrinsic.reshape(-1)),
        world_T_camera=_matrix4_tuple(frame.map_T_camera),
        world_T_ego=_matrix4_tuple(frame.map_T_lidar),
        boxes=tuple(boxes),
        source_sequence=source_sequence,
    )


def _matrix4_tuple(matrix: np.ndarray) -> tuple[float, ...]:
    return tuple(float(value) for value in np.asarray(matrix, dtype=np.float64).reshape(16))


def _serialize_point_cloud2(
    *, timestamp_ns: int, frame_id: str, points: np.ndarray
) -> bytes:
    """Serialize processed x/y/z/intensity points as ROS 2 PointCloud2 CDR."""

    values = np.asarray(points, dtype="<f4")
    if values.ndim != 2 or values.shape[1] != 4:
        raise ValueError("processed point cloud must have shape (N, 4)")
    values = np.ascontiguousarray(values)
    cdr = bytearray(b"\x00\x01\x00\x00")
    seconds, nanoseconds = divmod(timestamp_ns, 1_000_000_000)
    _append_i32(cdr, seconds)
    _append_u32(cdr, nanoseconds)
    _append_string(cdr, frame_id)
    _append_u32(cdr, 1)
    _append_u32(cdr, len(values))
    _append_u32(cdr, 4)
    for offset, name in enumerate(("x", "y", "z", "intensity")):
        _append_string(cdr, name)
        _append_u32(cdr, offset * 4)
        cdr.append(7)  # sensor_msgs/PointField.FLOAT32
        _append_u32(cdr, 1)
    cdr.append(0)
    _append_u32(cdr, 16)
    _append_u32(cdr, len(values) * 16)
    raw = values.tobytes()
    _append_u32(cdr, len(raw))
    cdr.extend(raw)
    cdr.append(0)
    return bytes(cdr)


def _align(buffer: bytearray, alignment: int) -> None:
    buffer.extend(b"\x00" * (-(len(buffer) - 4) % alignment))


def _append_u32(buffer: bytearray, value: int) -> None:
    _align(buffer, 4)
    buffer.extend(struct.pack("<I", value))


def _append_i32(buffer: bytearray, value: int) -> None:
    _align(buffer, 4)
    buffer.extend(struct.pack("<i", value))


def _append_string(buffer: bytearray, value: str) -> None:
    encoded = value.encode("utf-8") + b"\x00"
    _append_u32(buffer, len(encoded))
    buffer.extend(encoded)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--processed-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--scene", action="append", dest="scenes")
    parser.add_argument("--expected-samples", type=int)
    parser.add_argument("--frames-per-second", type=float, default=10.0)
    parser.add_argument("--minimum-area-px", type=float, default=64.0)
    parser.add_argument("--scene-generation-prefix", default="nuscenes")
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--metadata-topic", default="/camera/frame_detections")
    parser.add_argument("--lidar-topic", default="/lidar/points")
    return parser


def main() -> None:
    args = _parser().parse_args()
    config = ProcessedNuScenesMcapConfig(
        processed_root=args.processed_root,
        output_root=args.output_root,
        scenes=None if args.scenes is None else tuple(args.scenes),
        expected_samples=args.expected_samples,
        frames_per_second=args.frames_per_second,
        minimum_area_px=args.minimum_area_px,
        scene_generation_prefix=args.scene_generation_prefix,
        image_topic=args.image_topic,
        metadata_topic=args.metadata_topic,
        lidar_topic=args.lidar_topic,
    )
    ProcessedNuScenesMcapProcessor(config).process()


if __name__ == "__main__":
    main()

"""Write a selected CooperScene take as a ROS 2 MCAP recording."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import struct
from typing import Any, BinaryIO, Callable, Protocol

import cv2
import numpy as np

from src.data.data_processor import DataProcessor
from src.localization.cooperscene import CooperSceneFrame, CooperSceneSequenceDataset
from src.realtime.cooperscene_replay import CooperSceneReplayConfig, cooper_scene_frame_to_detections


ROS2_IMAGE_SCHEMA = b"""std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data

================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id

================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""
ROS2_STRING_SCHEMA = b"string data\n"


class McapWriter(Protocol):
    def start(self, *, profile: str = "") -> None: ...
    def finish(self) -> None: ...
    def register_schema(self, *, name: str, encoding: str, data: bytes) -> int: ...
    def register_channel(
        self, *, topic: str, message_encoding: str, schema_id: int
    ) -> int: ...
    def add_message(
        self,
        *,
        channel_id: int,
        log_time: int,
        publish_time: int,
        data: bytes,
        sequence: int,
    ) -> None: ...


@dataclass(frozen=True)
class CooperSceneMcapConfig:
    """The dataset selection and ROS contracts for one MCAP output."""

    data_root: Path
    output_path: Path
    split: str = "train"
    scenario: str = "1"
    agent: str = "1"
    frames_per_second: float = 10.0
    scene_generation: str = "cooperscene"
    gaussian_T_cooperscene: np.ndarray | None = None
    image_topic: str = "/camera/image_raw"
    metadata_topic: str = "/camera/frame_detections"
    camera_frame_id: str = "camera0"

    def __post_init__(self) -> None:
        if not self.split or not self.scenario or not self.agent:
            raise ValueError("split, scenario, and agent must be non-empty")
        if self.frames_per_second <= 0:
            raise ValueError("frames_per_second must be positive")
        if not self.scene_generation:
            raise ValueError("scene_generation must be non-empty")
        for name in ("image_topic", "metadata_topic"):
            if not getattr(self, name).startswith("/"):
                raise ValueError(f"{name} must be an absolute ROS topic")
        if not self.camera_frame_id:
            raise ValueError("camera_frame_id must be non-empty")
        transform = np.eye(4) if self.gaussian_T_cooperscene is None else np.asarray(
            self.gaussian_T_cooperscene, dtype=np.float64
        )
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise ValueError("gaussian_T_cooperscene must be a finite 4x4 transform")
        object.__setattr__(self, "gaussian_T_cooperscene", transform.copy())


class CooperSceneMcapProcessor(DataProcessor):
    """Convert one metadata-only CooperScene agent sequence to ROS 2 Image MCAP."""

    def __init__(
        self,
        config: CooperSceneMcapConfig,
        *,
        dataset_factory: Callable[[Path], CooperSceneSequenceDataset] = (
            lambda root: CooperSceneSequenceDataset.metadata_only(root, cache_frames=False)
        ),
        writer_factory: Callable[[BinaryIO], McapWriter] | None = None,
    ) -> None:
        self.config = config
        self._dataset_factory = dataset_factory
        self._writer_factory = writer_factory or _mcap_writer

    def process(self) -> int:
        """Write the selected sequence, using dataset cadence as ROS header time."""

        sequence = self._dataset_factory(self.config.data_root).sequence(
            self.config.split, self.config.scenario, self.config.agent
        )
        self.config.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.config.output_path.open("wb") as output:
            writer = self._writer_factory(output)
            writer.start(profile="ros2")
            try:
                schema_id = writer.register_schema(
                    name="sensor_msgs/msg/Image",
                    encoding="ros2msg",
                    data=ROS2_IMAGE_SCHEMA,
                )
                image_channel = writer.register_channel(
                    topic=self.config.image_topic,
                    message_encoding="cdr",
                    schema_id=schema_id,
                )
                metadata_schema = writer.register_schema(
                    name="std_msgs/msg/String", encoding="ros2msg", data=ROS2_STRING_SCHEMA
                )
                metadata_channel = writer.register_channel(
                    topic=self.config.metadata_topic,
                    message_encoding="cdr",
                    schema_id=metadata_schema,
                )
                for index, frame in enumerate(sequence):
                    timestamp_ns = round(index * 1_000_000_000 / self.config.frames_per_second)
                    detections = cooper_scene_frame_to_detections(
                        frame,
                        CooperSceneReplayConfig(
                            scene_generation=self.config.scene_generation,
                            agent=self.config.agent,
                        ),
                        self.config.gaussian_T_cooperscene,
                        timestamp_us=timestamp_ns // 1_000,
                        source_sequence=index,
                    )
                    writer.add_message(
                        channel_id=image_channel,
                        log_time=timestamp_ns,
                        publish_time=timestamp_ns,
                        sequence=index,
                        data=self._serialize_image(frame, timestamp_ns),
                    )
                    writer.add_message(
                        channel_id=metadata_channel,
                        log_time=timestamp_ns,
                        publish_time=timestamp_ns,
                        sequence=index,
                        data=_serialize_metadata(detections),
                    )
            finally:
                writer.finish()
        return len(sequence)

    def _serialize_image(self, frame: CooperSceneFrame, timestamp_ns: int) -> bytes:
        if frame.agent != self.config.agent:
            raise ValueError(
                f"frame agent {frame.agent} does not match configured agent {self.config.agent}"
            )
        if frame.camera_path is None:
            raise FileNotFoundError(f"frame {frame.frame_id} has no camera0 image")
        image = cv2.imread(str(frame.camera_path), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"failed to decode camera image: {frame.camera_path}")
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(f"camera image must be a three-channel BGR image: {frame.camera_path}")
        image = np.ascontiguousarray(image)
        height, width = image.shape[:2]
        return _serialize_image(
            timestamp_ns=timestamp_ns,
            frame_id=self.config.camera_frame_id,
            height=height,
            width=width,
            data=image.tobytes(),
        )


def _mcap_writer(output: BinaryIO) -> McapWriter:
    """Load MCAP lazily so importing this adapter does not require it."""

    try:
        from mcap.writer import CompressionType, Writer
    except ImportError as error:  # pragma: no cover - environment dependent
        raise RuntimeError(
            "MCAP output requires the optional 'mcap' package; install the project "
            "dependencies before processing a CooperScene take."
        ) from error
    return Writer(output, compression=CompressionType.NONE)


def _serialize_image(
    *, timestamp_ns: int, frame_id: str, height: int, width: int, data: bytes
) -> bytes:
    """Serialize ``sensor_msgs/msg/Image`` as little-endian ROS 2 CDR.

    This avoids requiring a ROS installation on the offline dataset-conversion host.
    ``ros2 bag play`` resolves the ROS type from the ``ros2msg`` MCAP schema.
    """

    cdr = bytearray(b"\x00\x01\x00\x00")  # CDR_LE encapsulation.
    _append_header(cdr, timestamp_ns, frame_id)
    _append_u32(cdr, height)
    _append_u32(cdr, width)
    _append_string(cdr, "bgr8")
    cdr.append(0)
    _align(cdr, 4)
    _append_u32(cdr, width * 3)
    _append_u32(cdr, len(data))
    cdr.extend(data)
    return bytes(cdr)


def _serialize_metadata(detections: Any) -> bytes:
    """Serialize the non-image fields of ``FrameDetections`` as ROS String JSON."""

    payload = {
        "version": 1,
        "frame_id": detections.frame_id,
        "request_id": detections.request_id,
        "scene_generation": detections.scene_generation,
        "timestamp_us": detections.timestamp_us,
        "camera_intrinsic": detections.camera_intrinsic,
        "world_T_camera": detections.world_T_camera,
        "world_T_ego": detections.world_T_ego,
        "boxes": [dict(box.to_dict(), text_label="car") for box in detections.boxes],
        "end_of_scene": detections.end_of_scene,
    }
    cdr = bytearray(b"\x00\x01\x00\x00")
    _append_string(cdr, json.dumps(payload, separators=(",", ":"), allow_nan=False))
    return bytes(cdr)


def _append_header(buffer: bytearray, timestamp_ns: int, frame_id: str) -> None:
    seconds, nanoseconds = divmod(timestamp_ns, 1_000_000_000)
    _append_i32(buffer, seconds)
    _append_u32(buffer, nanoseconds)
    _append_string(buffer, frame_id)


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

from __future__ import annotations

import struct
import json

import cv2
import numpy as np

from src.data.cooperscene_mcap import (
    ROS2_IMAGE_SCHEMA,
    ROS2_STRING_SCHEMA,
    CooperSceneMcapConfig,
    CooperSceneMcapProcessor,
    _serialize_metadata,
)
from src.localization.cooperscene import CooperSceneFrame
from src.realtime.protocol import BoxPrompt, FrameDetections


class _Writer:
    def __init__(self) -> None:
        self.started = False
        self.finished = False
        self.profile = None
        self.schemas = []
        self.channels = []
        self.messages = []

    def start(self, *, profile=""):
        self.started = True
        self.profile = profile

    def finish(self):
        self.finished = True

    def register_schema(self, **kwargs):
        self.schemas.append(kwargs)
        return len(self.schemas)

    def register_channel(self, **kwargs):
        self.channels.append(kwargs)
        return len(self.channels)

    def add_message(self, **kwargs):
        self.messages.append(kwargs)


class _Dataset:
    def __init__(self, frames):
        self.frames = frames
        self.selection = None

    def sequence(self, split, scenario, agent):
        self.selection = (split, scenario, agent)
        return self.frames


def _frame(tmp_path, frame_id="1"):
    path = tmp_path / f"{frame_id}_camera0.png"
    image = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
    assert cv2.imwrite(str(path), image)
    return CooperSceneFrame(
        split="train", scenario="1", agent="1", frame_id=frame_id,
        lidar_path=None, annotation_path=None, lidar_points=None,
        map_T_lidar=np.eye(4), camera_path=path,
    ), image


def test_processor_writes_ros2_image_messages_at_dataset_cadence(tmp_path):
    first, image = _frame(tmp_path, "100")
    second, _ = _frame(tmp_path, "101")
    dataset = _Dataset((first, second))
    writer = _Writer()
    processor = CooperSceneMcapProcessor(
        CooperSceneMcapConfig(tmp_path, tmp_path / "take.mcap", camera_frame_id="front_camera"),
        dataset_factory=lambda _root: dataset,
        writer_factory=lambda _output: writer,
    )

    assert processor.process() == 2

    assert dataset.selection == ("train", "1", "1")
    assert writer.started and writer.finished
    assert writer.profile == "ros2"
    assert writer.schemas == [
        {"name": "sensor_msgs/msg/Image", "encoding": "ros2msg", "data": ROS2_IMAGE_SCHEMA},
        {"name": "std_msgs/msg/String", "encoding": "ros2msg", "data": ROS2_STRING_SCHEMA},
    ]
    assert writer.channels == [
        {"topic": "/camera/image_raw", "message_encoding": "cdr", "schema_id": 1},
        {"topic": "/camera/frame_detections", "message_encoding": "cdr", "schema_id": 2},
    ]
    assert [message["log_time"] for message in writer.messages] == [0, 0, 100_000_000, 100_000_000]
    assert [message["sequence"] for message in writer.messages] == [0, 0, 1, 1]
    assert _decode_image(writer.messages[0]["data"]) == (2, 3, "bgr8", "front_camera", 9, image.tobytes())
    assert struct.unpack_from("<iI", writer.messages[2]["data"], 4) == (0, 100_000_000)
    metadata = _decode_string(writer.messages[3]["data"])
    assert metadata["frame_id"] == "101"
    assert metadata["scene_generation"] == "cooperscene"
    assert metadata["timestamp_us"] == 100_000
    assert set(metadata) == {
        "version", "frame_id", "request_id", "scene_generation", "timestamp_us",
        "camera_intrinsic", "world_T_camera", "world_T_ego", "boxes", "end_of_scene",
    }
    assert metadata["boxes"] == []


def test_processor_rejects_non_absolute_ros_topic(tmp_path):
    try:
        CooperSceneMcapConfig(tmp_path, tmp_path / "take.mcap", image_topic="camera/raw")
    except ValueError as error:
        assert "absolute ROS topic" in str(error)
    else:
        raise AssertionError("expected topic validation to fail")


def test_metadata_preserves_projected_box_contract_and_uses_car_label():
    metadata = _decode_string(_serialize_metadata(FrameDetections(
        frame_id="42",
        request_id="take:42",
        scene_generation="take",
        timestamp_us=12,
        image_jpeg=b"jpeg",
        camera_intrinsic=(1.0,) * 9,
        world_T_camera=(1.0,) * 16,
        world_T_ego=(2.0,) * 16,
        boxes=(BoxPrompt(
            track_id="vehicle-7", xyxy=(1, 2, 3, 4), dimensions_lwh=(4, 2, 1.5),
            world_T_object=(3.0,) * 16,
        ),),
    )))

    assert metadata["boxes"] == [{
        "track_id": "vehicle-7", "xyxy": [1.0, 2.0, 3.0, 4.0],
        "dimensions_lwh": [4.0, 2.0, 1.5], "world_T_object": [3.0] * 16,
        "text_label": "car", "visibility": 1.0, "detection_confidence": 1.0,
    }]


def _decode_image(data):
    assert data[:4] == b"\x00\x01\x00\x00"
    offset = 4

    def u32():
        nonlocal offset
        offset = (offset + 3) & ~3
        value = struct.unpack_from("<I", data, offset)[0]
        offset += 4
        return value

    def string():
        nonlocal offset
        length = u32()
        value = data[offset:offset + length - 1].decode()
        offset += length
        return value

    assert u32() == 0
    assert u32() == 0
    frame_id = string()
    height, width = u32(), u32()
    encoding = string()
    assert data[offset] == 0
    offset += 1
    offset = (offset + 3) & ~3
    step, length = u32(), u32()
    return height, width, encoding, frame_id, step, data[offset:offset + length]


def _decode_string(data):
    assert data[:4] == b"\x00\x01\x00\x00"
    length = struct.unpack_from("<I", data, 4)[0]
    return json.loads(data[8:8 + length - 1])

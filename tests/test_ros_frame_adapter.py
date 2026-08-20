from __future__ import annotations

import json
from types import SimpleNamespace

import cv2
import numpy as np

import pytest

from src.deployment.ros_frame_adapter import RosAnalysisAssembler, RosFrameAdapter, RosFrameAdapterConfig
from src.deployment.vehicle import VehicleAnalysisSpool
from src.realtime.process_support import JsonlMetrics
from src.realtime.protocol import ProtocolCodec
from src.realtime import EgoPoseSample, FrameObjectDetections


class _Publisher:
    def __init__(self) -> None:
        self.messages = []

    def send(self, message, *, topic=None):
        self.messages.append((topic, message))


def _metadata(timestamp_us=100_000, frame_id="100", generation="cooperscene:train:1:1"):
    # These legacy MCAP fields demonstrate that the adapter does not pass them
    # through to its focused pose-only contract.
    return {
        "version": 1,
        "frame_id": frame_id,
        "request_id": f"take:{frame_id}",
        "scene_generation": generation,
        "timestamp_us": timestamp_us,
        "camera_intrinsic": [1.0] * 9,
        "world_T_camera": [1.0] * 16,
        "world_T_ego": [
            1.0, 0.0, 0.0, 2.0,
            0.0, 1.0, 0.0, 3.0,
            0.0, 0.0, 1.0, 4.0,
            0.0, 0.0, 0.0, 1.0,
        ],
        "boxes": [{
            "track_id": "car-1",
            "xyxy": [10.0, 20.0, 110.0, 80.0],
            "dimensions_lwh": [4.5, 1.8, 1.5],
            "world_T_object": [
                1.0, 0.0, 0.0, 2.0,
                0.0, 1.0, 0.0, 3.0,
                0.0, 0.0, 1.0, 4.0,
                0.0, 0.0, 0.0, 1.0,
            ],
            "text_label": "car",
        }],
        "end_of_scene": False,
    }


def test_adapter_publishes_ordered_pose_only_samples_and_metrics(tmp_path):
    publisher = _Publisher()
    metrics_path = tmp_path / "adapter.jsonl"
    adapter = RosFrameAdapter(publisher, metrics=JsonlMetrics(metrics_path))

    first = adapter.receive_metadata(json.dumps(_metadata(100_000, "100")), arrival_ns=20)
    second = adapter.receive_metadata(_metadata(200_000, "101"), arrival_ns=30)

    assert [type(message) for _, message in publisher.messages] == [
        FrameObjectDetections, EgoPoseSample, FrameObjectDetections, EgoPoseSample
    ]
    assert [message.sequence for _, message in publisher.messages] == [0, 0, 1, 1]
    detections, first_pose, second_detections, second_pose = [
        message for _, message in publisher.messages
    ]
    assert detections.sequence == first_pose.sequence == 0
    assert detections.frame_id == first_pose.frame_id == "100"
    assert second_detections.timestamp_us == second_pose.timestamp_us == 200_000
    assert len(detections.boxes) == 1
    assert first.frame_id == "100"
    assert second.timestamp_us == 200_000
    assert set(first._metadata()) == {
        "sequence", "timestamp_us", "scene_generation", "frame_id", "world_T_ego"
    }
    events = [json.loads(line) for line in metrics_path.read_text().splitlines()]
    assert [event["event"] for event in events] == ["ros_frame_snapshot_spooled"] * 2
    assert [event["sequence"] for event in events] == [0, 1]
    assert [event["metadata_callback_arrival_ns"] for event in events] == [20, 30]
    assert all(event["publication_monotonic_ns"] >= event["callback_completed_monotonic_ns"] for event in events)
    assert [event["object_detection_count"] for event in events] == [1, 1]
    assert all(event["transport_contract"] == "durable_frame_snapshot" for event in events)


def test_adapter_output_round_trips_without_binary_image_parts():
    adapter = RosFrameAdapter(_Publisher())
    pose = adapter.receive_metadata(_metadata())

    encoded = ProtocolCodec().encode(pose)
    assert len(encoded) == 2
    topic, decoded = ProtocolCodec().decode(encoded)
    assert topic == "ego_pose_sample"
    assert decoded == pose


def test_adapter_rejects_malformed_or_duplicate_detection_prompts():
    duplicate = _metadata()
    duplicate["boxes"] *= 2
    with pytest.raises(ValueError, match="one prompt per track_id"):
        RosFrameAdapter(_Publisher()).receive_metadata(duplicate)
    malformed = _metadata()
    malformed["boxes"] = {"not": "an array"}
    with pytest.raises(ValueError, match="boxes must be an array"):
        RosFrameAdapter(_Publisher()).receive_metadata(malformed)


def test_adapter_latches_and_rejects_mismatched_scene_generation():
    adapter = RosFrameAdapter(_Publisher(), RosFrameAdapterConfig(expected_scene_generation="scene-a"))

    with pytest.raises(ValueError, match="does not match"):
        adapter.receive_metadata(_metadata(generation="scene-b"))
    assert adapter.scene_generation == "scene-a"
    assert adapter.receive_metadata(_metadata(generation="scene-a")).sequence == 0


@pytest.mark.parametrize("payload, error", [
    ({"version": 2}, "version must be 1"),
    (_metadata(timestamp_us=-1), "timestamp_us must be a non-negative integer"),
    (_metadata(generation=""), "scene_generation must be a non-empty string"),
])
def test_adapter_rejects_invalid_metadata(payload, error):
    with pytest.raises(ValueError, match=error):
        RosFrameAdapter(_Publisher()).receive_metadata(payload)


def _image(timestamp_us, *, encoding="bgr8", padding=0):
    bgr = np.zeros((4, 6, 3), dtype=np.uint8)
    bgr[:, :] = (10, 30, 220)
    pixels = bgr if encoding == "bgr8" else cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    packed = pixels.reshape(4, 18)
    rows = np.pad(packed, ((0, 0), (0, padding)))
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=timestamp_us // 1_000_000, nanosec=(timestamp_us % 1_000_000) * 1_000)
        ),
        height=4, width=6, step=18 + padding, encoding=encoding, data=rows.tobytes(),
    )


@pytest.mark.parametrize("image_first", [False, True])
def test_analysis_assembler_durably_joins_image_and_metadata_in_either_order(tmp_path, image_first):
    spool = VehicleAnalysisSpool(tmp_path / "analysis")
    assembler = RosAnalysisAssembler(spool, expected_samples=1)
    metadata = _metadata(timestamp_us=400_000)
    image = _image(400_000, encoding="rgb8", padding=5)

    if image_first:
        assembler.receive_image(image)
        assembler.receive_metadata(metadata, 0)
    else:
        assembler.receive_metadata(metadata, 0)
        assembler.receive_image(image)
    assembler.close()

    frame = spool.get(metadata["scene_generation"], 0)
    assert frame is not None
    assert frame.source_sequence == 0
    assert frame.boxes[0].text_label == "car"
    decoded = cv2.imdecode(np.frombuffer(frame.image_jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (4, 6, 3)
    assert decoded[0, 0, 2] > decoded[0, 0, 0]
    assert spool.end(metadata["scene_generation"]) is not None

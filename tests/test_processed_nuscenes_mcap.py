import json
from pathlib import Path
import struct

import cv2
import numpy as np

from src.data.processed_nuscenes_mcap import (
    ROS2_POINT_CLOUD2_SCHEMA,
    ProcessedNuScenesMcapConfig,
    ProcessedNuScenesMcapProcessor,
    _serialize_point_cloud2,
)
from src.deployment.ros_frame_adapter import _CdrReader


def _write_scene(root, frame_count):
    for name in ("images", "intrinsics", "extrinsics", "lidar", "lidar_pose", "instances"):
        (root / name).mkdir(parents=True)
    np.savetxt(root / "intrinsics" / "0.txt", [50, 50, 50, 50, 0, 0, 0, 0, 0])
    anchor = np.eye(4)
    anchor[0, 3] = 10
    for index in range(frame_count):
        frame_id = f"{index:03d}"
        assert cv2.imwrite(
            str(root / "images" / f"{frame_id}_0.jpg"),
            np.zeros((100, 100, 3), dtype=np.uint8),
        )
        np.savetxt(root / "extrinsics" / f"{frame_id}_0.txt", anchor)
        lidar_pose = anchor.copy()
        lidar_pose[0, 3] += index + 1
        np.savetxt(root / "lidar_pose" / f"{frame_id}.txt", lidar_pose)
        np.asarray([[1, 2, 3, 0.5]], dtype=np.float32).tofile(
            root / "lidar" / f"{frame_id}.bin"
        )
    object_pose = anchor.copy()
    object_pose[2, 3] = 10
    annotations = {
        "frame_idx": list(range(frame_count)),
        "obj_to_world": [object_pose.tolist()] * frame_count,
        "box_size": [[4.0, 2.0, 2.0]] * frame_count,
    }
    instances = {
        "0": {
            "id": "stable-car",
            "class_name": "vehicle.car",
            "frame_annotations": annotations,
        }
    }
    frames = {str(index): [0] for index in range(frame_count)}
    (root / "instances" / "instances_info.json").write_text(json.dumps(instances))
    (root / "instances" / "frame_instances.json").write_text(json.dumps(frames))


class _Writer:
    def __init__(self):
        self.schemas = []
        self.channels = []
        self.messages = []
        self.started = False
        self.finished = False

    def start(self, *, profile=""):
        self.started = profile == "ros2"

    def finish(self):
        self.finished = True

    def register_schema(self, **values):
        self.schemas.append(values)
        return len(self.schemas)

    def register_channel(self, **values):
        self.channels.append(values)
        return len(self.channels)

    def add_message(self, **values):
        self.messages.append(values)


def _decode_json_string(data):
    assert data[:4] == b"\x00\x01\x00\x00"
    length = struct.unpack_from("<I", data, 4)[0]
    return json.loads(data[8:8 + length - 1])


def test_all_scene_converter_discovers_variable_lengths_and_preserves_geometry(tmp_path):
    processed = tmp_path / "processed"
    _write_scene(processed / "152", frame_count=2)
    _write_scene(processed / "171", frame_count=3)
    writers = {}

    def writer_factory(output):
        writer = _Writer()
        writers[Path(output.name).name] = writer
        return writer

    processor = ProcessedNuScenesMcapProcessor(
        ProcessedNuScenesMcapConfig(
            processed_root=processed,
            output_root=tmp_path / "mcap",
            expected_samples=None,
            minimum_area_px=1,
        ),
        writer_factory=writer_factory,
    )

    assert processor.process() == 5
    assert list(processor.outputs) == ["152", "171"]
    assert set(writers) == {"152.mcap", "171.mcap"}
    first_writer = writers["152.mcap"]
    assert first_writer.started and first_writer.finished
    assert [schema["name"] for schema in first_writer.schemas] == [
        "sensor_msgs/msg/Image", "std_msgs/msg/String", "sensor_msgs/msg/PointCloud2"
    ]
    assert first_writer.schemas[2]["data"] == ROS2_POINT_CLOUD2_SCHEMA
    assert [channel["topic"] for channel in first_writer.channels] == [
        "/camera/image_raw", "/camera/frame_detections", "/lidar/points"
    ]
    assert len(first_writer.messages) == 6
    assert [message["log_time"] for message in first_writer.messages] == [
        0, 0, 0, 100_000_000, 100_000_000, 100_000_000
    ]
    metadata = _decode_json_string(first_writer.messages[1]["data"])
    assert metadata["scene_generation"] == "nuscenes:152"
    assert metadata["timestamp_us"] == 0
    assert metadata["camera_intrinsic"] == [50.0, 0.0, 50.0, 0.0, 50.0, 50.0, 0.0, 0.0, 1.0]
    assert metadata["camera_distortion"] == [0.0] * 5
    assert len(metadata["world_T_camera"]) == 16
    assert len(metadata["world_T_ego"]) == 16
    assert metadata["map_T_lidar"] == metadata["world_T_ego"]
    assert len(metadata["map_T_nuscenes_global"]) == 16
    assert metadata["gaussian_T_nuscenes_global"] == metadata["map_T_nuscenes_global"]
    assert [box["track_id"] for box in metadata["boxes"]] == ["stable-car"]


def test_lidar_is_standard_xyzi_pointcloud2():
    points = np.asarray([[1, 2, 3, 0.5], [4, 5, 6, 0.75]], dtype=np.float32)
    reader = _CdrReader(
        _serialize_point_cloud2(timestamp_ns=1_200_000_003, frame_id="LIDAR_TOP", points=points)
    )

    assert (reader.i32(), reader.u32(), reader.string()) == (1, 200_000_003, "LIDAR_TOP")
    assert (reader.u32(), reader.u32(), reader.u32()) == (1, 2, 4)
    fields = []
    for _ in range(4):
        fields.append((reader.string(), reader.u32(), reader.u8(), reader.u32()))
    assert fields == [
        ("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1), ("intensity", 12, 7, 1)
    ]
    assert reader.u8() == 0
    reader.align(4)
    assert (reader.u32(), reader.u32()) == (16, 32)
    np.testing.assert_array_equal(np.frombuffer(reader.bytes(), dtype="<f4").reshape(-1, 4), points)
    assert reader.u8() == 0
    assert reader.remaining == 0


def test_converter_can_select_one_scene_and_validate_expected_count(tmp_path):
    processed = tmp_path / "processed"
    _write_scene(processed / "152", frame_count=2)
    _write_scene(processed / "171", frame_count=3)
    writers = []

    processor = ProcessedNuScenesMcapProcessor(
        ProcessedNuScenesMcapConfig(
            processed_root=processed,
            output_root=tmp_path / "mcap",
            scenes=("171",),
            expected_samples=3,
        ),
        writer_factory=lambda _output: writers.append(_Writer()) or writers[-1],
    )

    assert processor.process() == 3
    assert list(processor.outputs) == ["171"]


def test_converter_uses_localized_gaussian_global_alignment(tmp_path):
    processed = tmp_path / "processed"
    _write_scene(processed / "152", frame_count=2)
    transform = np.eye(4)
    transform[:3, 3] = [3, 4, 5]
    localization = tmp_path / "localization.json"
    localization.write_text(json.dumps({"gaussian_T_nuscenes_global": transform.tolist()}))
    writers = []

    processor = ProcessedNuScenesMcapProcessor(
        ProcessedNuScenesMcapConfig(
            processed_root=processed, output_root=tmp_path / "mcap", scenes=("152",),
            expected_samples=2, localization_transform=localization,
        ), writer_factory=lambda _output: writers.append(_Writer()) or writers[-1]
    )

    assert processor.process() == 2
    metadata = _decode_json_string(writers[0].messages[1]["data"])
    assert np.asarray(metadata["gaussian_T_nuscenes_global"]).reshape(4, 4)[0:3, 3].tolist() == [3, 4, 5]
    assert np.asarray(metadata["world_T_camera"]).reshape(4, 4)[0:3, 3].tolist() == [13, 4, 5]


def test_generated_mcap_is_readable_with_expected_realtime_topics(tmp_path):
    from mcap.reader import make_reader

    processed = tmp_path / "processed"
    _write_scene(processed / "152", frame_count=2)
    processor = ProcessedNuScenesMcapProcessor(ProcessedNuScenesMcapConfig(
        processed_root=processed,
        output_root=tmp_path / "mcap",
        scenes=("152",),
        expected_samples=2,
        minimum_area_px=1,
    ))

    assert processor.process() == 2
    with processor.outputs["152"].open("rb") as stream:
        records = list(make_reader(stream).iter_messages(log_time_order=False))

    assert [channel.topic for _, channel, _ in records] == [
        "/camera/image_raw",
        "/camera/frame_detections",
        "/lidar/points",
        "/camera/image_raw",
        "/camera/frame_detections",
        "/lidar/points",
    ]
    assert [message.log_time for _, _, message in records] == [
        0, 0, 0, 100_000_000, 100_000_000, 100_000_000
    ]

from __future__ import annotations

import cv2
import json
import numpy as np

import src.localization.cooperscene as cooperscene

from src.localization.cooperscene import (
    CooperSceneFrame, CooperSceneSequenceDataset, CooperSceneVehicleBox,
)
from src.localization.cooperscene_calibration import camera_T_lidar, compose_map_T_camera
from src.realtime.cooperscene_replay import (
    CooperSceneReplayConfig,
    CooperSceneReplayPublisher,
    cooper_scene_frame_to_detections,
)
from src.realtime.protocol import FrameDetections
from src.realtime.runtime_support import JsonlMetricsWriter


def _frame(tmp_path, frame_id: str = "100") -> CooperSceneFrame:
    image_path = tmp_path / f"{frame_id}_camera0.png"
    assert cv2.imwrite(str(image_path), np.zeros((1200, 1920, 3), dtype=np.uint8))
    map_T_lidar = np.eye(4)
    map_T_camera = compose_map_T_camera(map_T_lidar, camera_T_lidar("1"))
    map_T_object = map_T_camera.copy()
    map_T_object[:3, 3] += map_T_camera[:3, :3] @ np.asarray([0.0, 0.0, 15.0])
    return CooperSceneFrame(
        split="train",
        scenario="1",
        agent="1",
        frame_id=frame_id,
        lidar_path=tmp_path / f"{frame_id}.pcd",
        annotation_path=tmp_path / f"{frame_id}.yaml",
        lidar_points=np.zeros((1, 4), dtype=np.float32),
        map_T_lidar=map_T_lidar,
        camera_path=image_path,
        vehicle_boxes=(CooperSceneVehicleBox("car-7", (4.0, 2.0, 2.0), map_T_object),),
    )


def test_frame_conversion_uses_gaussian_poses_and_projects_gt_box(tmp_path):
    frame = _frame(tmp_path)
    gaussian_T_map = np.eye(4)
    gaussian_T_map[:3, 3] = [10.0, -2.0, 3.0]

    message = cooper_scene_frame_to_detections(
        frame,
        CooperSceneReplayConfig(scene_generation="take-1"),
        gaussian_T_map,
        timestamp_us=200_000,
    )

    assert message.frame_id == "100"
    assert message.timestamp_us == 200_000
    assert len(message.boxes) == 1
    assert message.boxes[0].track_id == "car-7"
    assert message.boxes[0].dimensions_lwh == (4.0, 2.0, 2.0)
    world_T_ego = np.asarray(message.world_T_ego).reshape(4, 4)
    np.testing.assert_allclose(world_T_ego[:3, 3], [10.0, -2.0, 3.0])
    expected_object = gaussian_T_map @ frame.vehicle_boxes[0].map_T_object
    np.testing.assert_allclose(
        np.asarray(message.boxes[0].world_T_object).reshape(4, 4), expected_object
    )
    decoded = cv2.imdecode(np.frombuffer(message.image_jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (1200, 1920, 3)


def test_replay_uses_ten_hz_and_stops_before_overlap_failure_when_enabled(tmp_path):
    now = [5.0]
    sleeps: list[float] = []
    published: list[FrameDetections] = []

    class Publisher:
        def send(self, message, *, topic=None):
            published.append(message)

    def sleep(seconds: float) -> None:
        sleeps.append(seconds)
        now[0] += seconds

    frames = [_frame(tmp_path, str(frame_id)) for frame_id in (100, 101, 102)]
    replay = CooperSceneReplayPublisher(
        Publisher(),
        CooperSceneReplayConfig(
            scene_generation="take-1", scene_end_grace_seconds=0, stop_at_overlap=True
        ),
        np.eye(4),
        monotonic=lambda: now[0],
        sleep=sleep,
    )

    count = replay.run(frames, overlap_allows=lambda frame: frame.frame_id != "102")

    assert count == 2
    np.testing.assert_allclose(sleeps, [0.1])
    assert [message.timestamp_us for message in published[:-1]] == [0, 100_000]
    assert published[-1].end_of_scene
    assert published[-1].frame_id == "101:scene-end"


def test_replay_ignores_overlap_gate_by_default(tmp_path):
    published: list[FrameDetections] = []

    class Publisher:
        def send(self, message, *, topic=None):
            published.append(message)

    frames = [_frame(tmp_path, str(frame_id)) for frame_id in (100, 101, 102)]
    replay = CooperSceneReplayPublisher(
        Publisher(),
        CooperSceneReplayConfig(scene_generation="take-1", scene_end_grace_seconds=0),
        np.eye(4),
        monotonic=lambda: 0.0,
        sleep=lambda _seconds: None,
    )

    count = replay.run(frames, overlap_allows=lambda frame: frame.frame_id != "102")

    assert count == 3
    assert [message.frame_id for message in published[:-1]] == ["100", "101", "102"]


def test_metadata_replay_reads_no_pcd_and_publishes_all_501_with_timers(
    tmp_path, monkeypatch
):
    sequence_root = tmp_path / "train" / "1" / "1"
    sequence_root.mkdir(parents=True)
    image = np.zeros((16, 16, 3), dtype=np.uint8)
    for frame_id in range(481260, 481761):
        (sequence_root / f"{frame_id}.yaml").write_text(
            f"lidar_pose: [{frame_id - 481260}, 2, 3, 0, 0, 0]\nvehicles: {{}}\n",
            encoding="utf-8",
        )
        assert cv2.imwrite(str(sequence_root / f"{frame_id}_camera0.png"), image)
    monkeypatch.setattr(
        cooperscene, "read_ascii_xyzi_pcd",
        lambda _path: (_ for _ in ()).throw(AssertionError("PCD read")),
    )
    sequence = CooperSceneSequenceDataset.metadata_only(
        tmp_path, cache_frames=False
    ).sequence("train", 1, 1)
    published = []

    class Publisher:
        def send(self, message, *, topic=None):
            published.append(message)

    metrics_path = tmp_path / "replay.jsonl"
    replay = CooperSceneReplayPublisher(
        Publisher(),
        CooperSceneReplayConfig(
            scene_generation="take-1", scene_end_grace_seconds=0,
            minimum_area_px=0,
        ),
        np.eye(4),
        metrics=JsonlMetricsWriter(metrics_path),
        monotonic=lambda: 0.0,
        sleep=lambda _seconds: None,
    )

    assert replay.run(sequence) == 501
    assert [row.frame_id for row in published[:-1]] == [
        str(value) for value in range(481260, 481761)
    ]
    np.testing.assert_allclose(
        np.asarray(published[0].world_T_ego).reshape(4, 4)[:3, 3], [0, 2, 3]
    )
    np.testing.assert_allclose(
        np.asarray(published[-2].world_T_ego).reshape(4, 4)[:3, 3], [500, 2, 3]
    )
    metrics = [json.loads(line) for line in metrics_path.read_text().splitlines()]
    assert len(metrics) == 501
    assert all(row["lidar_points_loaded"] is False for row in metrics)
    assert all(row["pcd_read_ms"] == 0 for row in metrics)
    assert {
        "frame_metadata_load_ms", "camera_read_decode_ms", "pose_box_projection_ms",
        "jpeg_encode_ms",
    } <= metrics[0].keys()

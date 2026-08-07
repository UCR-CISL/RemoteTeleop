import json

import cv2
import numpy as np

from src.data.nuscenes_loader import NuScenesSequenceDataset, VehicleBox


def test_vehicle_box_keeps_legacy_constructor_compatible():
    box = VehicleBox(
        "annotation",
        "instance",
        "vehicle.car",
        np.asarray([1, 2, 3]),
        np.asarray([2, 4, 1.5]),
        0.0,
    )

    np.testing.assert_allclose(box.world_T_object, np.eye(4))
    np.testing.assert_allclose(box.dimensions_lwh, [4, 2, 1.5])


def test_loader_keeps_scene_separate_and_uses_lidar_coordinates(tmp_path):
    metadata = tmp_path / "v1.0-mini"
    metadata.mkdir()
    samples = tmp_path / "samples"
    samples.mkdir()
    for name in ("camera0.jpg", "camera1.jpg"):
        assert cv2.imwrite(str(samples / name), np.zeros((4, 6, 3), dtype=np.uint8))
    for name in ("lidar0.bin", "lidar1.bin"):
        np.asarray([[1, 2, 3, 0.5, 7]], dtype=np.float32).tofile(samples / name)

    tables = {
        "scene": [{"token": "scene", "name": "scene-001", "first_sample_token": "sample0"}],
        "sample": [
            {"token": "sample0", "timestamp": 100, "next": "sample1", "data": {"CAM_FRONT": "cam0", "LIDAR_TOP": "lidar0"}, "anns": ["car", "pedestrian"]},
            {"token": "sample1", "timestamp": 200, "next": "", "data": {"CAM_FRONT": "cam1", "LIDAR_TOP": "lidar1"}, "anns": []},
        ],
        "sample_data": [
            {"token": "cam0", "filename": "samples/camera0.jpg", "calibrated_sensor_token": "camera_cal", "ego_pose_token": "pose0"},
            {"token": "lidar0", "filename": "samples/lidar0.bin", "calibrated_sensor_token": "lidar_cal", "ego_pose_token": "pose0"},
            {"token": "cam1", "filename": "samples/camera1.jpg", "calibrated_sensor_token": "camera_cal", "ego_pose_token": "pose1"},
            {"token": "lidar1", "filename": "samples/lidar1.bin", "calibrated_sensor_token": "lidar_cal", "ego_pose_token": "pose1"},
        ],
        "calibrated_sensor": [
            {"token": "camera_cal", "translation": [0, 0, 0], "rotation": [1, 0, 0, 0], "camera_intrinsic": [[2, 0, 3], [0, 2, 2], [0, 0, 1]]},
            {"token": "lidar_cal", "translation": [0, 0, 0], "rotation": [1, 0, 0, 0]},
        ],
        "ego_pose": [
            {"token": "pose0", "translation": [0, 0, 0], "rotation": [1, 0, 0, 0]},
            {"token": "pose1", "translation": [5, 0, 0], "rotation": [1, 0, 0, 0]},
        ],
        "sample_annotation": [
            {"token": "car", "instance_token": "car-instance", "category_token": "vehicle", "translation": [10, 1, 0], "rotation": [1, 0, 0, 0], "size": [2, 4, 1]},
            {"token": "pedestrian", "instance_token": "pedestrian-instance", "category_token": "pedestrian", "translation": [2, 0, 0], "rotation": [1, 0, 0, 0], "size": [1, 1, 2]},
        ],
        "category": [{"token": "vehicle", "name": "vehicle.truck"}, {"token": "pedestrian", "name": "human.pedestrian.adult"}],
    }
    for table, records in tables.items():
        (metadata / f"{table}.json").write_text(json.dumps(records), encoding="utf-8")

    dataset = NuScenesSequenceDataset(tmp_path, future_steps=None)

    assert len(dataset) == 1
    sequence = dataset[0]
    assert sequence.name == "scene-001"
    assert len(sequence) == 2
    frame = sequence[0]
    assert frame.image.shape == (4, 6, 3)
    np.testing.assert_array_equal(frame.lidar_points, [[1, 2, 3, 0.5, 7]])
    np.testing.assert_array_equal(frame.camera_intrinsic, [[2, 0, 3], [0, 2, 2], [0, 0, 1]])
    np.testing.assert_allclose(frame.lidar_to_camera, np.eye(4))
    np.testing.assert_allclose(frame.world_T_ego, np.eye(4))
    np.testing.assert_allclose(frame.world_T_lidar, np.eye(4))
    np.testing.assert_allclose(frame.world_T_camera, np.eye(4))
    assert frame.camera_timestamp_us == 100
    assert frame.lidar_timestamp_us == 100
    assert len(frame.vehicle_boxes) == 1
    assert frame.vehicle_boxes[0].category == "vehicle.truck"
    np.testing.assert_allclose(frame.vehicle_boxes[0].center, [10, 1, 0])
    np.testing.assert_allclose(frame.vehicle_boxes[0].dimensions_lwh, [4, 2, 1])
    np.testing.assert_allclose(
        frame.vehicle_boxes[0].world_T_object[:3, 3], [10, 1, 0]
    )
    np.testing.assert_allclose(frame.ego_T_object[0][:3, 3], [10, 1, 0])
    np.testing.assert_allclose(frame.trajectory, [[0, 0, 0], [5, 0, 0]])
    np.testing.assert_array_equal(frame.trajectory_timestamps_us, [100, 200])
    np.testing.assert_allclose(sequence[1].trajectory, [[0, 0, 0]])

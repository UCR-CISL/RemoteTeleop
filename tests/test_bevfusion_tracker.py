from __future__ import annotations

import math

import numpy as np
import pytest

from src.realtime.bevfusion_tracker import BevFusionKalmanTracker, WorldDetection


def _detection(x: float, y: float = 0.0, *, label: str = "car") -> WorldDetection:
    transform = np.eye(4)
    transform[:3, 3] = (x, y, 1.0)
    return WorldDetection(transform, (4.5, 1.8, 1.5), label, 0.9)


def test_kalman_tracker_keeps_identity_and_predicts_motion():
    tracker = BevFusionKalmanTracker()
    tracker.update((_detection(0.0),), timestamp_us=0)
    first = tracker.boxes_at(0)[0]
    tracker.update((_detection(1.0),), timestamp_us=100_000)
    second = tracker.boxes_at(300_000)[0]

    assert second.track_id == first.track_id
    assert second.world_T_object[0, 3] > 1.0


def test_kalman_tracker_association_is_independent_of_detection_order():
    tracker = BevFusionKalmanTracker()
    tracker.update((_detection(-2.0), _detection(2.0)), timestamp_us=0)
    initial = {box.track_id: box.world_T_object[0, 3] for box in tracker.boxes_at(0)}
    tracker.update((_detection(1.0), _detection(-1.0)), timestamp_us=100_000)
    updated = {box.track_id: box.world_T_object[0, 3] for box in tracker.boxes_at(100_000)}

    left_id = min(initial, key=initial.get)
    right_id = max(initial, key=initial.get)
    assert updated[left_id] < updated[right_id]


def test_kalman_tracker_is_class_aware_and_expires_coasted_tracks():
    tracker = BevFusionKalmanTracker(maximum_coast_seconds=0.5)
    tracker.update((_detection(0.0, label="car"),), timestamp_us=0)
    car_id = tracker.boxes_at(0)[0].track_id
    tracker.update((_detection(0.0, label="truck"),), timestamp_us=100_000)
    assert {box.label for box in tracker.boxes_at(100_000)} == {"car", "truck"}
    assert car_id in {box.track_id for box in tracker.boxes_at(500_000)}

    tracker.update((), timestamp_us=600_000)
    assert car_id not in {box.track_id for box in tracker.boxes_at(600_000)}


def test_kalman_tracker_preserves_heading_in_a_non_z_up_map():
    # This axis permutation mirrors the NuScenes-to-Gaussian-map transform:
    # physical LiDAR Z is approximately world -Y rather than world Z.
    world_T_lidar = np.asarray([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
    yaw = math.radians(87.0)
    lidar_T_object = np.eye(4)
    lidar_T_object[:3, :3] = (
        (math.cos(yaw), -math.sin(yaw), 0.0),
        (math.sin(yaw), math.cos(yaw), 0.0),
        (0.0, 0.0, 1.0),
    )
    lidar_T_object[:3, 3] = (12.0, 25.0, 0.0)
    tracker = BevFusionKalmanTracker()
    tracker.update(
        (WorldDetection(world_T_lidar @ lidar_T_object, (4.2, 1.8, 1.5), "car", 0.9),),
        timestamp_us=0,
    )

    tracked = tracker.boxes_at(0)[0]
    recovered = np.linalg.inv(world_T_lidar) @ tracked.world_T_object
    recovered_yaw = math.atan2(recovered[1, 0], recovered[0, 0])
    assert math.degrees(recovered_yaw) == pytest.approx(87.0)

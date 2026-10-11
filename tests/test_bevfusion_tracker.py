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


def test_detection_cannot_resurrect_an_expired_identity():
    tracker = BevFusionKalmanTracker(maximum_coast_seconds=0.5)
    tracker.update((_detection(0.0),), timestamp_us=0)
    original_id = tracker.boxes_at(0)[0].track_id
    assert tracker.boxes_at(600_000) == ()
    tracker.update((_detection(0.0),), timestamp_us=600_000)
    assert tracker.boxes_at(600_000)[0].track_id != original_id


def test_detection_at_coast_boundary_keeps_identity():
    tracker = BevFusionKalmanTracker(maximum_coast_seconds=0.5)
    tracker.update((_detection(0.0),), timestamp_us=0)
    original_id = tracker.boxes_at(0)[0].track_id
    tracker.update((_detection(0.0),), timestamp_us=500_000)
    assert tracker.boxes_at(500_000)[0].track_id == original_id


def test_single_frame_ghost_is_never_emitted_with_confirmation():
    tracker = BevFusionKalmanTracker(minimum_confirmation_hits=2)
    tracker.update((_detection(0.0),), timestamp_us=0)
    assert tracker.boxes_at(0) == ()
    assert tracker.boxes_at(100_000) == ()  # Polling does not confirm a birth.
    tracker.update((), timestamp_us=100_000)
    assert tracker.boxes_at(100_000) == ()
    tracker.update((), timestamp_us=600_000)
    assert tracker.boxes_at(600_000) == ()


def test_birth_confirms_on_second_consecutive_observation_then_coasts():
    tracker = BevFusionKalmanTracker(minimum_confirmation_hits=2)
    tracker.update((_detection(0.0),), timestamp_us=0)
    assert tracker.boxes_at(0) == ()
    tracker.update((_detection(1.0),), timestamp_us=100_000)
    observed = tracker.boxes_at(100_000)
    assert len(observed) == 1
    tracker.update((), timestamp_us=200_000)
    coasted = tracker.boxes_at(300_000)
    assert coasted[0].track_id == observed[0].track_id
    assert coasted[0].world_T_object[0, 3] > observed[0].world_T_object[0, 3]
    assert tracker.boxes_at(600_000)[0].track_id == observed[0].track_id
    assert tracker.boxes_at(600_001) == ()


def test_unmatched_tentative_update_resets_consecutive_hit_streak():
    tracker = BevFusionKalmanTracker(minimum_confirmation_hits=2)
    tracker.update((_detection(0.0),), timestamp_us=0)
    tracker.update((), timestamp_us=100_000)
    tracker.update((_detection(0.0),), timestamp_us=200_000)
    assert tracker.boxes_at(200_000) == ()
    tracker.update((_detection(0.0),), timestamp_us=300_000)
    assert len(tracker.boxes_at(300_000)) == 1


def test_duplicate_capture_does_not_confirm_a_birth():
    tracker = BevFusionKalmanTracker(minimum_confirmation_hits=2)
    tracker.update((_detection(0.0),), timestamp_us=0)
    tracker.update((_detection(0.0),), timestamp_us=0)
    assert tracker.boxes_at(0) == ()
    tracker.update((_detection(0.0),), timestamp_us=100_000)
    assert len(tracker.boxes_at(100_000)) == 1


@pytest.mark.parametrize('minimum_hits', [0, -1, True, 1.5])
def test_confirmation_requires_positive_integer(minimum_hits):
    with pytest.raises(ValueError, match='minimum_confirmation_hits'):
        BevFusionKalmanTracker(minimum_confirmation_hits=minimum_hits)


def test_coarse_vehicle_association_preserves_native_truck_kind_during_coast():
    from dataclasses import replace

    tracker = BevFusionKalmanTracker()
    tracker.update((replace(_detection(0.0, label='vehicle'), detector_class_name='truck'),), timestamp_us=0)
    initial = tracker.boxes_at(0)[0]
    assert initial.label == 'vehicle'
    assert initial.detector_class_name == 'truck'
    tracker.update((), timestamp_us=100_000)
    coasted = tracker.boxes_at(200_000)[0]
    assert coasted.track_id == initial.track_id
    assert coasted.detector_class_name == 'truck'
    # An update without optional rendering metadata preserves the last supplied class.
    tracker.update((_detection(0.0, label='vehicle'),), timestamp_us=300_000)
    assert tracker.boxes_at(300_000)[0].detector_class_name == 'truck'


def test_low_confidence_observation_continues_confirmed_track_but_cannot_birth():
    from dataclasses import replace

    tracker = BevFusionKalmanTracker(birth_score_threshold=.6, continuation_score_threshold=.1)
    tracker.update((replace(_detection(20.0), score=.2),), timestamp_us=0)
    assert tracker.boxes_at(0) == ()
    tracker.update((_detection(0.0),), timestamp_us=100_000)
    original_id = tracker.boxes_at(100_000)[0].track_id
    tracker.update((replace(_detection(1.0), score=.2),), timestamp_us=200_000)
    continued = tracker.boxes_at(200_000)
    assert len(continued) == 1
    assert continued[0].track_id == original_id
    assert continued[0].score == .2
    assert continued[0].world_T_object[0, 3] > .5


def test_low_confidence_cannot_confirm_tentative_birth():
    from dataclasses import replace

    tracker = BevFusionKalmanTracker(minimum_confirmation_hits=2, birth_score_threshold=.6,
                                    continuation_score_threshold=.1)
    tracker.update((_detection(0.0),), timestamp_us=0)
    tracker.update((replace(_detection(0.0), score=.2),), timestamp_us=100_000)
    assert tracker.boxes_at(100_000) == ()
    tracker.update((_detection(0.0),), timestamp_us=200_000)
    assert tracker.boxes_at(200_000) == ()
    tracker.update((_detection(0.0),), timestamp_us=300_000)
    assert len(tracker.boxes_at(300_000)) == 1


def test_high_confidence_association_precedes_better_geometry_low_candidate():
    from dataclasses import replace

    tracker = BevFusionKalmanTracker(birth_score_threshold=.6, continuation_score_threshold=.1)
    tracker.update((_detection(0.0),), timestamp_us=0)
    tracker.update((_detection(1.0), replace(_detection(0.0), score=.2)), timestamp_us=100_000)
    boxes = tracker.boxes_at(100_000)
    assert len(boxes) == 1
    assert boxes[0].world_T_object[0, 3] > .5
    assert boxes[0].score == .9


def test_low_stage_only_fills_unmatched_confirmed_tracks_and_high_can_birth():
    from dataclasses import replace

    tracker = BevFusionKalmanTracker(birth_score_threshold=.6, continuation_score_threshold=.1)
    tracker.update((_detection(0.0), _detection(20.0)), timestamp_us=0)
    original_ids = [box.track_id for box in tracker.boxes_at(0)]
    tracker.update((_detection(1.0), replace(_detection(21.0), score=.2), _detection(40.0)),
                   timestamp_us=100_000)
    boxes = tracker.boxes_at(100_000)
    assert len(boxes) == 3
    assert [box.track_id for box in boxes[:2]] == original_ids
    assert [box.score for box in boxes] == [.9, .2, .9]
    assert boxes[1].world_T_object[0, 3] > 20.5
    assert boxes[2].world_T_object[0, 3] == 40.0


@pytest.mark.parametrize('x,label,score', [(100., 'car', .2), (0., 'truck', .2), (0., 'car', .05)])
def test_low_confidence_continuation_keeps_class_distance_and_score_gates(x, label, score):
    from dataclasses import replace

    tracker = BevFusionKalmanTracker(birth_score_threshold=.6, continuation_score_threshold=.1)
    tracker.update((_detection(0.0),), timestamp_us=0)
    tracker.update((replace(_detection(x, label=label), score=score),), timestamp_us=100_000)
    boxes = tracker.boxes_at(100_000)
    assert len(boxes) == 1
    assert boxes[0].world_T_object[0, 3] == 0
    assert boxes[0].score == .9  # Existing track coasted without a low-score correction.


@pytest.mark.parametrize('birth,continuation', [(.1, .2), (-.1, 0), (1.1, 0), (.2, -.1), (float('nan'), 0), (.2, float('inf'))])
def test_invalid_confidence_stage_thresholds_are_rejected(birth, continuation):
    with pytest.raises(ValueError, match='score thresholds'):
        BevFusionKalmanTracker(birth_score_threshold=birth, continuation_score_threshold=continuation)

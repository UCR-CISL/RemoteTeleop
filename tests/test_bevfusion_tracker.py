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


def test_duplicate_birth_guard_preserves_assigned_track_and_detector_inputs():
    from dataclasses import replace

    original = _detection(0.)
    continued = _detection(.1)
    duplicate = replace(_detection(.8), score=.6)
    saved = duplicate.world_T_object.copy()
    baseline = BevFusionKalmanTracker()
    guarded = BevFusionKalmanTracker(suppress_duplicate_births=True)
    for tracker in (baseline, guarded):
        tracker.update((original,), timestamp_us=0)
        tracker.update((continued, duplicate), timestamp_us=100_000)
    before, after = baseline.boxes_at(100_000), guarded.boxes_at(100_000)
    assert len(before) == 2 and len(after) == 1  # Default remains unchanged.
    assert after[0].track_id == before[0].track_id
    assert after[0].score == continued.score
    np.testing.assert_array_equal(after[0].world_T_object, before[0].world_T_object)
    np.testing.assert_array_equal(duplicate.world_T_object, saved)


@pytest.mark.parametrize('kind', ['adjacent', 'other_class', 'other_height', 'other_heading', 'tilted'])
def test_duplicate_guard_allows_distinct_boxes_and_nonparallel_normals(kind):
    from dataclasses import replace
    from scipy.spatial.transform import Rotation

    candidate = _detection(.8)
    pose = candidate.world_T_object.copy()
    if kind == 'adjacent':
        pose[:3, 3] = (0, 2., 1.)
    elif kind == 'other_class':
        candidate = replace(candidate, label='truck')
    elif kind == 'other_height':
        pose[2, 3] = 4.
    elif kind == 'other_heading':
        pose[:3, :3] = Rotation.from_euler('z', 45, degrees=True).as_matrix()
    else:
        pose[:3, :3] = Rotation.from_euler('x', 10, degrees=True).as_matrix()
        assert BevFusionKalmanTracker._parallel_box_iou(replace(candidate, world_T_object=pose), _detection(0.)) is None
    candidate = replace(candidate, world_T_object=pose)
    tracker = BevFusionKalmanTracker(suppress_duplicate_births=True)
    tracker.update((_detection(0.),), timestamp_us=0)
    tracker.update((_detection(0.), candidate), timestamp_us=100_000)
    assert len(tracker.boxes_at(100_000)) == 2


def test_duplicate_overlap_is_exact_for_parallel_boxes_and_modulo_pi():
    from dataclasses import replace
    from scipy.spatial.transform import Rotation

    original = _detection(0.)
    shifted = _detection(1.)
    # Parallel boxes with identical heights/widths reduce to their interval IoU.
    assert BevFusionKalmanTracker._parallel_box_iou(shifted, original) == pytest.approx(3.5 / 5.5)
    pose = shifted.world_T_object.copy()
    pose[:3, :3] = Rotation.from_euler('z', 180, degrees=True).as_matrix()
    reversed_heading = replace(shifted, world_T_object=pose)
    assert BevFusionKalmanTracker._duplicate_birth(reversed_heading, original)
    pose[:3, :3] = Rotation.from_euler('x', 180, degrees=True).as_matrix()
    assert BevFusionKalmanTracker._parallel_box_iou(replace(shifted, world_T_object=pose), original) == pytest.approx(3.5 / 5.5)


def test_duplicate_guard_is_invariant_to_arbitrary_world_rotation_and_translation():
    from dataclasses import replace
    from scipy.spatial.transform import Rotation

    world = np.eye(4)
    world[:3, :3] = Rotation.from_euler('xyz', [63, -45, 32], degrees=True).as_matrix()
    world[:3, 3] = (10., -20., 30.)
    boxes = (_detection(0.), _detection(.1), _detection(.8))
    transformed = tuple(replace(box, world_T_object=world @ box.world_T_object) for box in boxes)
    assert BevFusionKalmanTracker._parallel_box_iou(boxes[2], boxes[1]) == pytest.approx(
        BevFusionKalmanTracker._parallel_box_iou(transformed[2], transformed[1]))
    output = []
    for collection in (boxes, transformed):
        tracker = BevFusionKalmanTracker(suppress_duplicate_births=True)
        tracker.update(collection[:1], timestamp_us=0)
        tracker.update(collection[1:], timestamp_us=100_000)
        output.append(tracker.boxes_at(100_000))
    assert len(output[0]) == len(output[1]) == 1
    assert output[0][0].track_id == output[1][0].track_id
    np.testing.assert_allclose(world @ output[0][0].world_T_object, output[1][0].world_T_object, atol=1e-12)


def test_duplicate_guard_does_not_use_coasted_unmatched_ghost_as_birth_reference():
    from dataclasses import replace

    ghost = replace(_detection(0.), dimensions_lwh=(20., 2., 2.))
    candidate = replace(_detection(8.), dimensions_lwh=(20., 2., 2.))
    assert BevFusionKalmanTracker._duplicate_birth(candidate, ghost)
    tracker = BevFusionKalmanTracker(suppress_duplicate_births=True)
    tracker.update((ghost,), timestamp_us=0)
    # The 8m innovation fails the association gate, while the long boxes overlap.
    tracker.update((candidate,), timestamp_us=100_000)
    assert len(tracker.boxes_at(100_000)) == 2


def test_newly_confirmed_tentative_track_cannot_suppress_another_birth():
    tracker = BevFusionKalmanTracker(suppress_duplicate_births=True, minimum_confirmation_hits=2)
    tracker.update((_detection(0.),), timestamp_us=0)
    tracker.update((_detection(0.), _detection(.8)), timestamp_us=100_000)
    assert len(tracker.boxes_at(100_000)) == 1
    # The second box still starts its own tentative identity at this frame.
    tracker.update((_detection(.8),), timestamp_us=200_000)
    assert len(tracker.boxes_at(200_000)) == 2


def test_parallel_box_iou_clips_rotated_footprint_and_partial_height():
    from dataclasses import replace
    from scipy.spatial.transform import Rotation

    square = replace(_detection(0.), dimensions_lwh=(2., 2., 2.))
    pose = square.world_T_object.copy()
    pose[:3, :3] = Rotation.from_euler('z', 45, degrees=True).as_matrix()
    rotated = replace(square, world_T_object=pose)
    assert BevFusionKalmanTracker._parallel_box_iou(rotated, square) == pytest.approx(1 / math.sqrt(2))
    pose = square.world_T_object.copy()
    pose[2, 3] += 1.
    assert BevFusionKalmanTracker._parallel_box_iou(replace(square, world_T_object=pose), square) == pytest.approx(1 / 3)


def test_suppression_provenance_is_read_only_and_resets_each_update():
    tracker = BevFusionKalmanTracker(suppress_duplicate_births=True)
    assert tracker.suppressed_birth_pairs == ()
    tracker.update((_detection(0.),), timestamp_us=0)
    tracker.update((_detection(0.), _detection(.8)), timestamp_us=100_000)
    assert tracker.suppressed_birth_pairs == ((1, 0),)
    with pytest.raises(AttributeError):
        tracker.suppressed_birth_pairs = ()
    tracker.update((), timestamp_us=200_000)
    assert tracker.suppressed_birth_pairs == ()
    default = BevFusionKalmanTracker()
    default.update((_detection(0.),), timestamp_us=0)
    default.update((_detection(0.), _detection(.8)), timestamp_us=100_000)
    assert default.suppressed_birth_pairs == ()


def test_duplicate_guard_ignores_low_stage_reference_and_same_update_births():
    from dataclasses import replace

    ghost = replace(_detection(0.), dimensions_lwh=(20., 2., 2.))
    candidate = replace(_detection(8.), dimensions_lwh=(20., 2., 2.))
    tracker = BevFusionKalmanTracker(suppress_duplicate_births=True,
                                    birth_score_threshold=.6, continuation_score_threshold=.1)
    tracker.update((ghost,), timestamp_us=0)
    tracker.update((candidate, replace(ghost, score=.2)), timestamp_us=100_000)
    assert len(tracker.boxes_at(100_000)) == 2
    assert tracker.boxes_at(100_000)[0].score == .2  # Low continuation remains untouched.
    assert tracker.suppressed_birth_pairs == ()
    newborns = BevFusionKalmanTracker(suppress_duplicate_births=True)
    newborns.update((_detection(0.), _detection(.8)), timestamp_us=0)
    assert len(newborns.boxes_at(0)) == 2
    assert newborns.suppressed_birth_pairs == ()


@pytest.mark.parametrize("seconds", [-1.0, float('nan'), float('inf')])
def test_identity_reacquisition_rejects_invalid_window(seconds):
    with pytest.raises(ValueError, match='maximum_reacquisition_seconds'):
        BevFusionKalmanTracker(maximum_reacquisition_seconds=seconds)


def test_optional_reacquisition_changes_only_identity_of_fresh_filter():
    original = BevFusionKalmanTracker()
    opted = BevFusionKalmanTracker(maximum_reacquisition_seconds=2.0)
    for tracker in (original, opted):
        tracker.update((_detection(0),), timestamp_us=0)
        tracker.update((), timestamp_us=600_000)
        tracker.update((_detection(1),), timestamp_us=700_000)
    assert original.boxes_at(700_000)[0].track_id == 'bevfusion:car:1'
    assert opted.boxes_at(700_000)[0].track_id == 'bevfusion:car:0'
    assert opted._tracks[0].track_id == 'bevfusion:car:1'
    assert np.array_equal(opted._tracks[0].state, original._tracks[0].state)
    assert np.array_equal(opted._tracks[0].covariance, original._tracks[0].covariance)
    assert np.array_equal(opted.boxes_at(800_000)[0].world_T_object,
                          original.boxes_at(800_000)[0].world_T_object)


@pytest.mark.parametrize('ambiguous_old', [True, False])
def test_reacquisition_requires_mutually_unique_pairs(ambiguous_old):
    tracker = BevFusionKalmanTracker(maximum_reacquisition_seconds=2.0)
    first = (_detection(-1), _detection(1)) if ambiguous_old else (_detection(0),)
    tracker.update(first, timestamp_us=0)
    tracker.update((), timestamp_us=600_000)
    new = (_detection(0),) if ambiguous_old else (_detection(-1), _detection(1))
    tracker.update(new, timestamp_us=700_000)
    assert all(box.track_id == track.track_id for box, track in
               zip(tracker.boxes_at(700_000), tracker._tracks))
    assert len({b.track_id for b in tracker.boxes_at(700_000)}) == len(new)


@pytest.mark.parametrize('gap,label,yaw', [(2_000_001, 'car', 0),
                                           (700_000, 'truck', 0),
                                           (700_000, 'car', math.pi / 2)])
def test_reacquisition_rejects_age_class_and_heading(gap, label, yaw):
    tracker = BevFusionKalmanTracker(maximum_reacquisition_seconds=2.0)
    tracker.update((_detection(0),), timestamp_us=0)
    tracker.update((), timestamp_us=600_000)
    detection = _detection(0, label=label)
    c, s = math.cos(yaw), math.sin(yaw)
    detection.world_T_object[:3, :3] = ((c, -s, 0), (s, c, 0), (0, 0, 1))
    tracker.update((detection,), timestamp_us=gap)
    assert tracker.boxes_at(gap)[0].track_id == tracker._tracks[0].track_id


def test_reacquisition_prunes_retired_states_and_inactive_mappings():
    tracker = BevFusionKalmanTracker(maximum_reacquisition_seconds=2.0)
    tracker.update((_detection(0),), timestamp_us=0)
    tracker.update((), timestamp_us=600_000)
    assert len(tracker._identity_registry.retired) == 1
    assert not tracker._identity_registry.identities
    tracker.update((), timestamp_us=2_000_001)
    assert not tracker._identity_registry.retired


def test_reacquisition_does_not_assign_an_active_identity_twice():
    tracker = BevFusionKalmanTracker(maximum_reacquisition_seconds=2.0)
    tracker.update((_detection(0),), timestamp_us=0)
    tracker.update((), timestamp_us=600_000)
    tracker.update((_detection(0),), timestamp_us=700_000)
    tracker.update((_detection(0), _detection(2)), timestamp_us=800_000)
    boxes = tracker.boxes_at(800_000)
    assert len({b.track_id for b in boxes}) == len(boxes) == 2


def test_reacquisition_is_equivariant_under_rigid_world_rotation():
    from scipy.spatial.transform import Rotation
    Q = Rotation.from_euler('xyz', [.4, -.7, .3]).as_matrix()
    identities = []
    for rotation in (np.eye(3), Q):
        tracker = BevFusionKalmanTracker(maximum_reacquisition_seconds=2.0)
        for stamp, x in ((0, 0), (700_000, 1)):
            detection = _detection(x)
            detection.world_T_object[:3, 3] = rotation @ detection.world_T_object[:3, 3]
            detection.world_T_object[:3, :3] = rotation
            tracker.update((detection,), timestamp_us=stamp)
        identities.append(tracker.boxes_at(700_000)[0].track_id)
    assert identities == ['bevfusion:car:0', 'bevfusion:car:0']


def test_last_matches_report_confirmed_assignments_at_the_scan_and_exclude_coasting():
    tracker = BevFusionKalmanTracker(minimum_confirmation_hits=2, maximum_reacquisition_seconds=2.0)
    tracker.update((_detection(0.0), _detection(20.0)), timestamp_us=0)
    assert tracker.last_matches == ()  # tentative births are not confirmed yet
    tracker.update((_detection(30.0), _detection(0.5)), timestamp_us=100_000)
    (match,) = tracker.last_matches  # x=30 is a tentative birth; x=20 coasts unconfirmed
    assert match.detection_index == 1 and match.filter_id == match.track.track_id
    assert match.track.track_id == tracker.boxes_at(100_000)[0].track_id
    np.testing.assert_array_equal(match.track.world_T_object, tracker.boxes_at(100_000)[0].world_T_object)
    tracker.update((), timestamp_us=200_000)
    assert tracker.last_matches == () and len(tracker.boxes_at(200_000)) == 1  # coasting is never a match
    identity = match.track.track_id
    tracker.update((_detection(0.7),), timestamp_us=700_000)
    tracker.update((_detection(0.8),), timestamp_us=800_000)
    (reacquired,) = tracker.last_matches
    assert reacquired.track.track_id == identity and reacquired.filter_id != identity

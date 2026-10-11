from __future__ import annotations

import numpy as np
import pytest

from src.evaluation.cooperscene_tracking import FrontCameraCoverageEvaluator, GeometricTrackingEvaluator, GroundTruthContinuity


def box(identity, x=0.0, *, label="car", yaw=0.0):
    pose = np.eye(4)
    pose[0, 3] = x
    pose[:2, :2] = ((np.cos(yaw), -np.sin(yaw)), (np.sin(yaw), np.cos(yaw)))
    return dict(track_id=identity, world_T_object=pose.tolist(), dimensions_lwh=[4, 2, 1.5], label=label)


def update(evaluator, time, gt, predictions, sequence="take1/v1"):
    return evaluator.update(sequence_id=sequence, timestamp_us=time, gt=gt, predictions=predictions)


def test_gate_before_assignment_preserves_maximum_cardinality():
    evaluator = GeometricTrackingEvaluator()
    result = update(evaluator, 0, [box("a", 0), box("b", 2)], [box("x", .5), box("y", -1.5)])
    assert [(m["gt_id"], m["pred_id"]) for m in result["matches"]] == [("a", "y"), ("b", "x")]
    assert evaluator.summary()["tp"] == 2


def test_switch_fragmentation_and_global_identity_assignment():
    evaluator = GeometricTrackingEvaluator()
    update(evaluator, 0, [box("a")], [box("x")])
    update(evaluator, 1, [box("a")], [])
    result = update(evaluator, 2, [box("a")], [box("y")])
    update(evaluator, 3, [box("a")], [box("y")])
    assert result["id_switches"] == [dict(gt_id="a", previous_pred_id="x", pred_id="y")]
    summary = evaluator.summary()
    assert (summary["tp"], summary["fp"], summary["fn"]) == (3, 0, 1)
    assert (summary["id_switches"], summary["fragmentation"]) == (1, 1)
    assert (summary["idtp"], summary["idfp"], summary["idfn"]) == (2, 1, 2)
    assert summary["geometric_idf1"] == pytest.approx(4 / 7)


def test_global_identity_uses_all_eligible_pairs_not_only_local_matches():
    evaluator = GeometricTrackingEvaluator()
    update(evaluator, 0, [box("a", 0), box("b", 1)], [box("x", 0), box("y", 1)])
    update(evaluator, 1, [box("a", 0), box("b", 1)], [box("x", 1), box("y", 0)])
    assert evaluator.summary()["id_switches"] == 2
    assert evaluator.summary()["geometric_idf1"] == 1.0


def test_class_gate_duplicate_fp_and_sequence_identity_namespace():
    evaluator = GeometricTrackingEvaluator()
    result = update(evaluator, 0, [box("a")], [box("x"), box("y"), box("z", label="truck")])
    assert len(result["false_positive_indices"]) == 2
    update(evaluator, 0, [box("a")], [box("different")], sequence="take2/v1")
    summary = evaluator.summary()
    assert summary["id_switches"] == 0
    assert summary["idtp"] == 2


def test_inclusive_threshold_rotation_and_unannotated_gap():
    evaluator = GeometricTrackingEvaluator()
    result = update(evaluator, 0, [box("a")], [box("x", 2, yaw=np.pi)])
    assert result["matches"][0]["heading_error_deg"] == pytest.approx(0)
    update(evaluator, 1, [], [])
    update(evaluator, 2, [box("a")], [box("x")])
    assert evaluator.summary()["fragmentation"] == 0
    with pytest.raises(ValueError, match="strictly increase"):
        update(evaluator, 2, [], [])


def test_empty_sequence_and_invalid_input():
    evaluator = GeometricTrackingEvaluator()
    update(evaluator, 0, [], [])
    assert evaluator.summary()["geometric_idf1"] is None
    with pytest.raises(ValueError, match="unique"):
        update(evaluator, 1, [box("a"), box("a")], [])


def test_gt_continuity_segments_are_explicit_and_do_not_modify_input():
    continuity = GroundTruthContinuity()
    original = [box("a")]
    first = continuity.segment(sequence_id="take1", frame_id="0", timestamp_us=0, gt=original)
    second = continuity.segment(sequence_id="take1", frame_id="1", timestamp_us=100_000, gt=[box("a", 49)])
    assert original[0]["track_id"] == "a"
    assert first[0]["track_id"] == "a#segment:0"
    assert second[0]["track_id"] == "a#segment:1"
    assert continuity.discontinuities[0]["threshold_m"] == 10
    continuity.segment(sequence_id="take1", frame_id="2", timestamp_us=200_000, gt=[])
    after_gap = continuity.segment(sequence_id="take1", frame_id="3", timestamp_us=300_000, gt=[box("a", 0)])
    assert after_gap[0]["track_id"] == "a#segment:1"
    assert len(continuity.discontinuities) == 1


def test_front_camera_coverage_filters_symmetrically_and_exposes_never_detected():
    evaluator = FrontCameraCoverageEvaluator()
    visible, behind, unseen = box("a"), box("behind"), box("unseen", 3)
    visible["world_T_object"][2][3] = unseen["world_T_object"][2][3] = 10
    behind["world_T_object"][2][3] = -10
    prediction = dict(visible, track_id="p", score=.9)
    for index in range(5):
        frame = dict(sequence_id="take1", frame_id=str(index), timestamp_us=index*100000,
            image_width=100, image_height=100, world_T_camera=np.eye(4).tolist(),
            camera_intrinsic=[[100,0,50],[0,100,50],[0,0,1]],
            gt=[visible, behind, unseen], detections=[prediction, dict(behind, track_id="fp-behind")], tracks=[prediction])
        result = evaluator.update(frame)
        assert result["visible_gt"] == 2
        assert result["visible_detections"] == 1
    summary = evaluator.summary()
    assert summary["detections"]["tp"] == 5
    assert summary["detections"]["fn"] == 5
    assert summary["acceptance"]["never_detected_gt_ids"] == ["unseen"]
    assert summary["per_gt_id"][1]["longest_track_miss_frames"] == 5
    assert summary["acceptance"]["per_id_recall_pass"] is False


def test_front_camera_near_plane_rejection_and_precision_target():
    evaluator = FrontCameraCoverageEvaluator(precision_target=.9)
    visible, crossing = box("a"), box("crossing")
    visible["world_T_object"][2][3] = 10
    crossing["world_T_object"][2][3] = .5
    for index in range(5):
        predictions = [dict(visible, track_id="p"), dict(visible, track_id="duplicate")]
        evaluator.update(dict(sequence_id="take1", frame_id=str(index), timestamp_us=index*100000,
            image_width=100, image_height=100, world_T_camera=np.eye(4).tolist(),
            camera_intrinsic=[[100,0,50],[0,100,50],[0,0,1]],
            gt=[visible, crossing], detections=predictions, tracks=predictions))
    summary = evaluator.summary()
    assert len(summary["per_gt_id"]) == 1
    assert summary["acceptance"]["per_id_recall_pass"]
    assert not summary["acceptance"]["optional_precision_target_pass"]
    assert summary["detections"]["precision"] == .5

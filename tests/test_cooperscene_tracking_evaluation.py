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


def _camera_frame(index, *, image_timestamp_ns=1, prediction_id="p"):
    visible = box("a")
    visible["world_T_object"][2][3] = 10
    return dict(sequence_id="take1", frame_id=str(index), timestamp_us=index * 100000,
                image_timestamp_ns=image_timestamp_ns, image_width=100, image_height=100,
                world_T_camera=np.eye(4).tolist(), camera_intrinsic=[[100,0,50],[0,100,50],[0,0,1]],
                gt=[visible], detections=[] if prediction_id is None else [dict(visible, track_id=prediction_id)],
                tracks=[] if prediction_id is None else [dict(visible, track_id=prediction_id)])


def test_strict_camera_timestamp_skip_preserves_identity_and_coverage():
    evaluator = FrontCameraCoverageEvaluator(require_valid_image_timestamp=True)
    evaluator.update(_camera_frame(0, image_timestamp_ns=1_000_000_001))
    skipped = evaluator.update(_camera_frame(1, image_timestamp_ns=0, prediction_id=None))
    assert skipped["camera_evaluated"] is False
    assert skipped["visible_gt_ids"] == []
    assert skipped["skipped_gt_ids"] == ["a"]
    assert skipped["tracks"] == dict(matches=[], missed_gt_indices=[], false_positive_indices=[], id_switches=[])
    after = evaluator.update(_camera_frame(2, image_timestamp_ns=1_200_000_003, prediction_id="new"))
    assert after["tracks"]["id_switches"] == [dict(gt_id="a", previous_pred_id="p", pred_id="new")]
    summary = evaluator.summary()
    assert summary["tracks"]["frames"] == 2
    assert summary["tracks"]["fragmentation"] == 0
    assert summary["per_gt_id"][0]["visible_frames"] == 2
    assert summary["per_gt_id"][0]["longest_track_miss_frames"] == 0
    validity = summary["image_timestamp_validity"]
    assert (validity["input_frames"], validity["evaluated_frames"], validity["skipped_frames"]) == (3, 2, 1)
    assert validity["skipped_supplied_gt_ids"] == [dict(sequence_id="take1", gt_id="a", skipped_frames=1)]
    with pytest.raises(ValueError, match="strictly increase"):
        evaluator.update(_camera_frame(1, image_timestamp_ns=0))


@pytest.mark.parametrize("timestamp", [None, 1.0, "1", True])
def test_strict_camera_requires_exact_source_timestamp_metadata(timestamp):
    evaluator = FrontCameraCoverageEvaluator(require_valid_image_timestamp=True)
    frame = _camera_frame(0, image_timestamp_ns=timestamp)
    if timestamp is None:
        del frame["image_timestamp_ns"]
    frame["image_minus_lidar_ms"] = 0.0  # Cannot substitute or infer an exact source timestamp.
    with pytest.raises(ValueError, match="exact integer image_timestamp_ns"):
        evaluator.update(frame)
    assert evaluator.summary()["image_timestamp_validity"]["input_frames"] == 0


def test_default_camera_policy_still_evaluates_unknown_and_invalid_captures():
    default = FrontCameraCoverageEvaluator()
    explicit = FrontCameraCoverageEvaluator(require_valid_image_timestamp=False)
    for index, stamp in enumerate((1, 0, None)):
        frame = _camera_frame(index, image_timestamp_ns=stamp)
        if stamp is None:
            del frame["image_timestamp_ns"]
        assert default.update(frame) == explicit.update(frame)
    assert default.summary() == explicit.summary()
    assert default.summary()["tracks"]["frames"] == 3
    assert "image_timestamp_validity" not in default.summary()


def test_camera_timestamp_cli_sensitivity_preserves_global_lidar_metrics(tmp_path):
    import json
    import subprocess
    import sys

    frames = [_camera_frame(0), _camera_frame(1, image_timestamp_ns=0, prediction_id=None), _camera_frame(2)]
    source = tmp_path / "frames.jsonl"
    source.write_text("".join(json.dumps(frame) + "\n" for frame in frames))
    summaries, assignments = [], []
    for strict in (False, True):
        output = tmp_path / str(strict)
        command = [sys.executable, "-m", "scripts.investigation.cooperscene_tracking.evaluate", str(source),
                   "--output-dir", str(output), "--front-camera-only", "--split-discontinuous-gt"]
        if strict:
            command.append("--require-valid-image-timestamp")
        subprocess.run(command, check=True, capture_output=True, text=True)
        summaries.append(json.loads((output / "metrics.json").read_text()))
        assignments.append([json.loads(line) for line in (output / "assignments.jsonl").read_text().splitlines()])
    for key in ("detections", "tracks", "tracks_split_gt_sensitivity"):
        assert summaries[0][key] == summaries[1][key]
        assert all(a[key] == b[key] for a, b in zip(*assignments, strict=True))
    assert summaries[0]["tracks"]["frames"] == 3
    assert summaries[0]["front_camera"]["tracks"]["fn"] == 1
    assert summaries[1]["front_camera"]["tracks"]["fn"] == 0
    assert summaries[1]["front_camera"]["tracks"]["fragmentation"] == 0
    assert assignments[1][1]["front_camera"]["camera_evaluated"] is False


def test_skipped_camera_capture_neither_adds_nor_resets_valid_miss_run():
    evaluator = FrontCameraCoverageEvaluator(require_valid_image_timestamp=True)
    for index, (stamp, identity) in enumerate(((1, "p"), (2, None), (0, None), (4, None), (5, "p"))):
        evaluator.update(_camera_frame(index, image_timestamp_ns=stamp, prediction_id=identity))
    summary = evaluator.summary()
    assert summary["tracks"]["fn"] == 2
    assert summary["tracks"]["fragmentation"] == 1
    assert summary["tracks"]["id_switches"] == 0
    assert summary["per_gt_id"][0]["longest_track_miss_frames"] == 2

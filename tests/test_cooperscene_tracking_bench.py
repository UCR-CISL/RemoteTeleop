from __future__ import annotations

import numpy as np
import pytest

from src.evaluation.cooperscene_tracking import GeometricTrackingEvaluator
from src.evaluation.cooperscene_tracking_bench import (
    BevHota, FrameOutput, Region, SequenceData, SequenceScorer, SequenceTrackerRun, TrackerConfig,
    box_matrices, ego_mask, matrices_to_boxes, summarize)
from src.localization.cooperscene import pose_to_matrix


def pose(x=0.0, z=0.0, pitch_deg=0.0):
    matrix = np.array(pose_to_matrix([x, 0, z, 0, 0, pitch_deg]))
    return matrix.tolist()


def test_bev_only_evaluator_ignores_height_size_and_pitch():
    gt = [dict(track_id="g", world_T_object=pose(), dimensions_lwh=[4, 2, 1.5])]
    pred = [dict(track_id="p", world_T_object=pose(1.0, z=3.0, pitch_deg=20), dimensions_lwh=[8, 3, 4])]
    full, bev = GeometricTrackingEvaluator(), GeometricTrackingEvaluator(bev_only=True)
    full.update(sequence_id="s", timestamp_us=0, gt=gt, predictions=pred)
    result = bev.update(sequence_id="s", timestamp_us=0, gt=gt, predictions=pred)
    assert full.summary()["tp"] == 0
    assert bev.summary()["tp"] == 1
    assert result["matches"][0]["center_error_m"] == pytest.approx(1.0)
    assert result["matches"][0]["heading_error_deg"] == pytest.approx(0.0, abs=1e-6)


def test_box_matrix_round_trip():
    boxes = np.array([[1.0, -2.0, 0.5, 4.0, 2.0, 1.5, 2.5], [0, 0, 0, 1, 1, 1, -3.0]])
    assert np.allclose(matrices_to_boxes(box_matrices(boxes), boxes[:, 3:6]), boxes)


def test_ego_mask_flags_box_containing_lidar_origin():
    boxes = np.array([[0.5, -0.3, -1, 5, 2, 1.8, 0.1], [6, 0, -1, 5, 2, 1.8, 0.0]])
    assert ego_mask(boxes).tolist() == [True, False]


def _hota(frames):
    hota = BevHota()
    for gt_ids, tr_ids, gt_xy, tr_xy in frames:
        hota.add(gt_ids, tr_ids, np.asarray(gt_xy, float).reshape(-1, 2), np.asarray(tr_xy, float).reshape(-1, 2))
    return BevHota.combine([hota.counts()])


def test_hota_perfect_split_identity_and_misses():
    perfect = _hota([(["a"], ["x"], [[0, 0]], [[0, 0]])] * 4)
    assert perfect["HOTA"] == pytest.approx(1.0) and perfect["LocA"] == pytest.approx(1.0)
    split = _hota([(["a"], ["x"], [[0, 0]], [[0, 0]])] * 2 + [(["a"], ["y"], [[0, 0]], [[0, 0]])] * 2)
    assert split["DetA"] == pytest.approx(1.0)
    assert split["AssA"] == pytest.approx(0.5)
    assert split["HOTA"] == pytest.approx(np.sqrt(0.5))
    missed = _hota([(["a"], [], [[0, 0]], [])] * 3)
    assert missed["HOTA"] == 0.0
    # sim = 1 - d/2: a 1 m offset matches only for alpha <= 0.5 (10 of 19 alphas).
    offset = _hota([(["a"], ["x"], [[0, 0]], [[1, 0]])])
    assert offset["DetA"] == pytest.approx(10 / 19)


def _sequence(gt_lidar, det_lidar, scores, gt_ids, pose6=(10.0, 5.0, 0.0, 0.0, 30.0, 0.0)):
    frames = len(gt_lidar)
    map_T_lidar = np.repeat(np.asarray(pose_to_matrix(pose6))[None], frames, 0)
    gt_world = [matrices_to_boxes(map_T_lidar[0] @ box_matrices(g), g[:, 3:6]) for g in gt_lidar]
    return SequenceData("val", "13", "1", np.arange(100, 100 + frames), map_T_lidar,
                        gt_ids=[np.asarray(i) for i in gt_ids], gt_world=gt_world, gt_lidar=gt_lidar,
                        gt_points=[np.full(len(g), 50) for g in gt_lidar], det_lidar=det_lidar,
                        det_scores=[np.asarray(s, float) for s in scores])


def test_tracker_run_maps_detections_to_world_and_keeps_one_identity():
    boxes = [np.array([[10 + 0.5 * t, 3, -1, 4.5, 2, 1.6, 0.2]]) for t in range(20)]
    sequence = _sequence(boxes, boxes, [[0.6]] * 20, [[7]] * 20)
    outputs = SequenceTrackerRun(sequence, TrackerConfig()).run()
    assert {str(i) for out in outputs for i in out.ids} == {"bevfusion:car:0"}
    assert np.allclose(outputs[0].world[0, :2], sequence.gt_world[0][0, :2])
    assert np.allclose(outputs[0].lidar[0], boxes[0][0])
    metrics = summarize([SequenceScorer(Region("all_r50")).score(sequence, outputs)])
    assert (metrics["IDF1"], metrics["MOTA"], metrics["id_switches"]) == (1.0, 1.0, 0)
    assert metrics["MT"] == 1 and metrics["bev_center_rmse_m"] < 0.2


def test_scorer_ignores_outputs_on_ego_and_out_of_region_gt_but_counts_region_false_positives():
    gt = [np.array([[0.2, 0, -1, 5, 2, 1.8, 0], [60, 0, -1, 5, 2, 1.8, 0], [20, 0, -1, 5, 2, 1.8, 0]])]
    sequence = _sequence(gt, [np.zeros((0, 7))], [[]], [[1, 2, 3]])
    lidar = np.array([[0.2, 0.5, -1, 5, 2, 1.8, 0], [60.5, 0, -1, 5, 2, 1.8, 0], [61, 20, -1, 4, 2, 1.5, 0],
                      [30, 10, -1, 4, 2, 1.5, 0]])
    world = matrices_to_boxes(sequence.map_T_lidar[0] @ box_matrices(lidar), lidar[:, 3:6])
    out = FrameOutput(np.array(["e", "far", "far_fp", "fp"]), world, lidar, np.zeros(4, bool), np.ones(4), np.zeros(4))
    metrics = summarize([SequenceScorer(Region("all_r50")).score(sequence, [out])])
    # Only GT 3 is scored; only the in-region unmatched output counts as FP.
    assert (metrics["tp"], metrics["fn"], metrics["fp"]) == (0, 1, 1)


def test_raw_detections_have_frame_local_identities():
    boxes = [np.array([[10, 0, -1, 4, 2, 1.5, 0], [20, 0, -1, 4, 2, 1.5, 0]])] * 2
    sequence = _sequence(boxes, boxes, [[0.9, 0.1]] * 2, [[1, 2]] * 2)
    outputs = SequenceTrackerRun.raw_detections(sequence, 0.5)
    assert [list(o.ids) for o in outputs] == [["100:0"], ["101:0"]]


def test_front_fov_region_uses_benchmark_calibration():
    sequence = _sequence([np.zeros((0, 7))], [np.zeros((0, 7))], [[]], [[]])
    boxes = np.array([[20, 0, -1, 4, 2, 1.5, 0], [-20, 0, -1, 4, 2, 1.5, 0], [55, 0, -1, 4, 2, 1.5, 0]])
    assert Region("fov_r50")(sequence, boxes).tolist() == [True, False, False]

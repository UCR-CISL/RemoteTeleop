import numpy as np

from src.evaluation.cooperscene_detection import DetectionBenchmark, DetectionFrame, iou_matrices, voc_ap


def _box(x, y, yaw=0.0, l=4.0, w=2.0):
    return [x, y, 0.0, l, w, 1.5, yaw]


def test_rotated_bev_iou_matches_closed_forms():
    gt = np.array([_box(0, 0), _box(10, 0)])
    pred = np.array([_box(0, 0, np.pi), _box(1, 0), _box(0, 0, np.pi / 2), _box(30, 0)])
    bev, iou3d = iou_matrices(pred, gt)
    np.testing.assert_allclose(bev[:, 0], [1.0, 6 / 10, 4 / 12, 0.0], atol=1e-9)
    np.testing.assert_allclose(bev[:, 1], 0.0)
    np.testing.assert_allclose(iou3d[:, 0], bev[:, 0], atol=1e-9)  # identical z extent


def test_voc_ap_perfect_and_half():
    assert voc_ap(np.array([.9, .8]), np.array([True, True]), 2) == 1.0
    assert voc_ap(np.array([.9]), np.array([True]), 2) == 0.5


def test_ignore_semantics_for_unobserved_gt():
    frame = DetectionFrame("2", "1", 0.0, gt=np.array([_box(0, 0), _box(20, 0)]), gt_points=np.array([50, 0]),
                           pred=np.array([_box(0.2, 0), _box(20, 0)]), scores=np.array([.9, .8]), ego_pose=np.zeros(6))
    bench = DetectionBenchmark(lambda f, b: np.ones(len(b), bool))
    bench.add(frame)
    summary = bench.summarize()
    assert summary["all/allgt"]["AP_BEV@0.5"] == 1.0
    observed = summary["all/gt5pts"]
    assert observed["n_gt"] == 1 and observed["AP_BEV@0.5"] == 1.0
    assert observed["center2m"]["0.5"]["fp"] == 0  # prediction on the unobserved GT is ignored

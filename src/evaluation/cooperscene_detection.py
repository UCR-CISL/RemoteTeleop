"""Single-frame BEV detection benchmark for cached CooperScene predictions.

Boxes are [x, y, z_gravity_center, l, w, h, yaw] in each sample's LiDAR frame
(mmdet3d LiDAR convention, l along the heading). BEV quantities are primary:
rotated BEV IoU AP (OpenCOOD/CooperScene protocol: per-frame greedy by score
against the best remaining GT, global score sort, VOC2010 AP) and inclusive
BEV center-distance Hungarian matching. 3D IoU AP, z and size errors are
secondary diagnostics and never used for matching.

Subsets (FOV, range, LiDAR-observed GT) use "ignore" semantics: matching always
runs over every in-range GT, then predictions matched to a non-subset GT are
dropped and unmatched predictions count as FP only when they lie in the subset.

``DetectionFrame`` keeps ``ego_pose`` so a later tracking evaluation can map
cached detections to the dataset world frame per (scenario, agent) sequence.
"""
from __future__ import annotations

from dataclasses import dataclass
import pickle
from typing import Callable, Iterator

import numpy as np
import shapely
from scipy.optimize import linear_sum_assignment

PCD_LIMIT = np.array([-72, -72, -5, 72, 72, 3], dtype=float)  # CooperSceneDataset GT filter


@dataclass
class DetectionFrame:
    scenario: str
    agent_id: str
    timestamp: float
    gt: np.ndarray            # (G,7) in-range GT
    gt_points: np.ndarray     # (G,) LiDAR points inside each GT box
    pred: np.ndarray          # (P,7)
    scores: np.ndarray        # (P,)
    ego_pose: np.ndarray      # CARLA [x,y,z,roll,yaw,pitch] deg (for future world-frame tracking)

    @property
    def sequence(self) -> tuple[str, str]:
        return self.scenario, self.agent_id


def load_frames(predictions_npz: str, info_pkl: str) -> list[DetectionFrame]:
    """Join a batch_detect.py NPZ with the info pkl; frames sorted by sequence/time."""
    pred = np.load(predictions_npz)
    infos = pickle.load(open(info_pkl, "rb"))["data_list"]
    offsets = np.concatenate([[0], np.cumsum(pred["counts"])])
    gt_offsets = np.concatenate([[0], np.cumsum(pred["gt_counts"])])
    frames = []
    for i, idx in enumerate(pred["sample_idx"]):
        info = infos[int(idx)]
        assert str(info["scenario"]) == pred["scenario"][i] and float(info["timestamp"]) == pred["timestamp"][i]
        gt = np.asarray([x["bbox_3d"] for x in info["instances"] if x["bbox_label_3d"] == 0], float).reshape(-1, 7)
        gt_pts = pred["gt_num_points"][gt_offsets[i]:gt_offsets[i + 1]]
        assert len(gt_pts) == len(info["instances"]) == len(gt), "non-vehicle GT labels are not expected"
        keep = np.all((gt[:, :3] >= PCD_LIMIT[:3]) & (gt[:, :3] <= PCD_LIMIT[3:]), axis=1)
        sl = slice(offsets[i], offsets[i + 1])
        vehicle = pred["labels"][sl] == 0
        frames.append(DetectionFrame(str(info["scenario"]), str(info["agent_id"]), float(info["timestamp"]),
                                     gt[keep], gt_pts[keep], pred["boxes"][sl][vehicle].astype(float),
                                     pred["scores"][sl][vehicle].astype(float), np.asarray(info["ego_pose"], float)))
    frames.sort(key=lambda f: (int(f.scenario), int(f.agent_id), f.timestamp))
    return frames


def bev_corners(boxes: np.ndarray) -> np.ndarray:
    c, s = np.cos(boxes[:, 6]), np.sin(boxes[:, 6])
    local = np.array([[.5, .5], [-.5, .5], [-.5, -.5], [.5, -.5]])
    dx, dy = local[None, :, 0] * boxes[:, 3:4], local[None, :, 1] * boxes[:, 4:5]
    return np.stack([boxes[:, :1] + c[:, None] * dx - s[:, None] * dy,
                     boxes[:, 1:2] + s[:, None] * dx + c[:, None] * dy], axis=-1)


def corners_3d(boxes: np.ndarray) -> np.ndarray:
    xy = bev_corners(boxes)
    z0, z1 = boxes[:, 2] - boxes[:, 5] / 2, boxes[:, 2] + boxes[:, 5] / 2
    return np.concatenate([np.concatenate([xy, np.repeat(z[:, None, None], 4, 1)], -1) for z in (z0, z1)], 1)


def iou_matrices(pred: np.ndarray, gt: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rotated BEV IoU and upright 3D IoU, (P,G); only nearby pairs are intersected."""
    bev = np.zeros((len(pred), len(gt)))
    if not len(pred) or not len(gt):
        return bev, bev.copy()
    radius = lambda b: 0.5 * np.hypot(b[:, 3], b[:, 4])
    near = np.linalg.norm(pred[:, None, :2] - gt[None, :, :2], axis=-1) < radius(pred)[:, None] + radius(gt)[None]
    i, j = np.nonzero(near)
    area_p, area_g = pred[:, 3] * pred[:, 4], gt[:, 3] * gt[:, 4]
    inter = np.zeros_like(bev)
    if len(i):
        polys_p, polys_g = shapely.polygons(bev_corners(pred)), shapely.polygons(bev_corners(gt))
        inter[i, j] = shapely.area(shapely.intersection(polys_p[i], polys_g[j]))
    bev = inter / (area_p[:, None] + area_g[None] - inter)
    zp0, zp1 = pred[:, 2] - pred[:, 5] / 2, pred[:, 2] + pred[:, 5] / 2
    zg0, zg1 = gt[:, 2] - gt[:, 5] / 2, gt[:, 2] + gt[:, 5] / 2
    dz = np.clip(np.minimum(zp1[:, None], zg1[None]) - np.maximum(zp0[:, None], zg0[None]), 0, None)
    vol = inter * dz
    iou3d = vol / (area_p[:, None] * pred[:, 5:6] + (area_g * gt[:, 5])[None] - vol)
    return bev, iou3d


def greedy_iou_match(iou: np.ndarray, scores: np.ndarray, threshold: float) -> np.ndarray:
    """OpenCOOD protocol: each prediction (by score) takes its best remaining GT if IoU >= thr."""
    match = np.full(len(scores), -1)
    free = np.ones(iou.shape[1], bool)
    for p in np.argsort(-scores, kind="stable"):
        if not free.any():
            break
        row = np.where(free, iou[p], -1)
        g = int(np.argmax(row))
        if row[g] >= threshold:
            match[p], free[g] = g, False
    return match


def center_match(pred: np.ndarray, gt: np.ndarray, gate: float) -> np.ndarray:
    """Max-cardinality then min-distance Hungarian on BEV center distance (inclusive gate)."""
    match = np.full(len(pred), -1)
    if not len(pred) or not len(gt):
        return match
    d = np.linalg.norm(pred[:, None, :2] - gt[None, :, :2], axis=-1)
    cost = np.where(d <= gate, d, (min(d.shape) + 1) * (gate + 1))
    rows, cols = linear_sum_assignment(cost)
    ok = d[rows, cols] <= gate
    match[rows[ok]] = cols[ok]
    return match


def voc_ap(scores: np.ndarray, tp: np.ndarray, n_gt: int) -> float:
    if n_gt == 0 or not len(scores):
        return float("nan") if n_gt == 0 else 0.0
    order = np.argsort(-scores, kind="stable")
    tps = np.cumsum(tp[order]); fps = np.cumsum(~tp[order])
    rec = np.concatenate([[0], tps / n_gt, [1]]); prec = np.concatenate([[0], tps / (tps + fps), [0]])
    prec = np.maximum.accumulate(prec[::-1])[::-1]
    i = np.nonzero(rec[1:] != rec[:-1])[0] + 1
    return float(np.sum((rec[i] - rec[i - 1]) * prec[i]))


class FrontCameraFov:
    """Box at least partially in the 1920x1200 front image (any corner or the center, depth > 0.1)."""

    def __init__(self, calibration: dict, width: int = 1920, height: int = 1200) -> None:
        self.calibration, self.size = calibration, (width, height)

    def __call__(self, frame: DetectionFrame, boxes: np.ndarray) -> np.ndarray:
        if not len(boxes):
            return np.zeros(0, bool)
        entry = self.calibration[f"scene{frame.scenario}_agent{frame.agent_id}"]
        T, K = np.asarray(entry["T_lidar_to_cam"]), np.asarray(entry["intrinsic"])
        pts = np.concatenate([corners_3d(boxes), boxes[:, None, :3]], 1)
        cam = pts @ T[:3, :3].T + T[:3, 3]
        uv = cam @ K.T
        depth = cam[..., 2]
        u, v = uv[..., 0] / np.maximum(depth, 1e-6), uv[..., 1] / np.maximum(depth, 1e-6)
        inside = (depth > 0.1) & (u >= 0) & (u < self.size[0]) & (v >= 0) & (v < self.size[1])
        return inside.any(1)


class DetectionBenchmark:
    """Accumulate per-frame match results once; summarize any frame group and subset."""

    REGIONS = ("all", "fov", "r0-30", "r30-50", "r50-72")

    def __init__(self, fov: Callable[[DetectionFrame, np.ndarray], np.ndarray], *,
                 iou_thresholds=(0.3, 0.5, 0.7), score_thresholds=(0.1, 0.2, 0.3, 0.4, 0.5),
                 center_gate_m: float = 2.0, min_points: int = 5) -> None:
        self.fov, self.iou_thresholds, self.score_thresholds = fov, iou_thresholds, score_thresholds
        self.gate, self.min_points = center_gate_m, min_points
        self.records: list[dict] = []

    def _regions(self, frame: DetectionFrame, boxes: np.ndarray) -> dict[str, np.ndarray]:
        r = np.hypot(boxes[:, 0], boxes[:, 1]) if len(boxes) else np.zeros(0)
        return {"all": np.ones(len(boxes), bool), "fov": self.fov(frame, boxes),
                "r0-30": r < 30, "r30-50": (r >= 30) & (r < 50), "r50-72": (r >= 50) & (r < 72)}

    def add(self, frame: DetectionFrame) -> None:
        bev, iou3d = iou_matrices(frame.pred, frame.gt)
        rec = dict(frame=frame, gt_regions=self._regions(frame, frame.gt),
                   pred_regions=self._regions(frame, frame.pred),
                   observed=frame.gt_points >= self.min_points,
                   bev={t: greedy_iou_match(bev, frame.scores, t) for t in self.iou_thresholds},
                   iou3d={t: greedy_iou_match(iou3d, frame.scores, t) for t in self.iou_thresholds},
                   center={})
        for s in self.score_thresholds:
            keep = np.nonzero(frame.scores >= s)[0]
            m = np.full(len(frame.scores), -1)
            m[keep] = center_match(frame.pred[keep], frame.gt, self.gate)
            rec["center"][s] = m
        self.records.append(rec)

    @staticmethod
    def _classify(match, care, pred_in):
        """-> (counted prediction mask, TP flags among counted, n_care)."""
        matched = match >= 0
        to_care = np.zeros(len(match), bool)
        to_care[matched] = care[match[matched]]
        counted = to_care | (~matched & pred_in)
        return counted, to_care[counted], int(care.sum())

    def summarize(self, select: Callable[[DetectionFrame], bool] = lambda f: True) -> dict:
        recs = [r for r in self.records if select(r["frame"])]
        out = {"frames": len(recs)}
        for region in self.REGIONS:
            for gt_set in ("allgt", f"gt{self.min_points}pts"):
                care_of = lambda r: r["gt_regions"][region] & (r["observed"] if gt_set != "allgt" else True)
                res = {"n_gt": int(sum(care_of(r).sum() for r in recs))}
                for kind in ("bev", "iou3d"):
                    for t in self.iou_thresholds:
                        sc, tp, n = [], [], 0
                        for r in recs:
                            counted, flags, k = self._classify(r[kind][t], care_of(r), r["pred_regions"][region])
                            sc.append(r["frame"].scores[counted]); tp.append(flags); n += k
                        res[f"AP_{'BEV' if kind == 'bev' else '3D'}@{t}"] = voc_ap(np.concatenate(sc), np.concatenate(tp), n) if recs else None
                prf = {}
                for s in self.score_thresholds:
                    tp = fp = 0; errs = []
                    for r in recs:
                        f, m = r["frame"], r["center"][s]
                        counted, flags, k = self._classify(m, care_of(r), r["pred_regions"][region] & (f.scores >= s))
                        tp += int(flags.sum()); fp += int((~flags).sum())
                        idx = np.nonzero(counted)[0][flags]
                        if len(idx):
                            errs.append(np.concatenate([f.pred[idx], f.gt[m[idx]]], 1))
                    n = res["n_gt"]
                    p, rc = tp / max(tp + fp, 1), tp / max(n, 1)
                    prf[s] = dict(tp=tp, fp=fp, fn=n - tp, precision=p, recall=rc,
                                  f1=2 * p * rc / max(p + rc, 1e-12), errors=self._errors(np.concatenate(errs) if errs else np.zeros((0, 14))))
                best = max(prf, key=lambda s: prf[s]["f1"])
                res["center2m"] = {str(s): v for s, v in prf.items()}
                res["best_f1_threshold"] = best
                out[f"{region}/{gt_set}"] = res
        return out

    @staticmethod
    def _errors(pairs: np.ndarray) -> dict:
        """Matched-pair errors. Primary: BEV center, yaw. Secondary: z/bottom/size."""
        if not len(pairs):
            return {}
        p, g = pairs[:, :7], pairs[:, 7:]
        rmse = lambda x: float(np.sqrt(np.mean(x ** 2)))
        dyaw = np.abs((p[:, 6] - g[:, 6] + np.pi) % (2 * np.pi) - np.pi)
        dz = p[:, 2] - g[:, 2]
        dbottom = (p[:, 2] - p[:, 5] / 2) - (g[:, 2] - g[:, 5] / 2)
        return dict(n=len(p), bev_center_rmse=rmse(np.hypot(*(p[:, :2] - g[:, :2]).T)),
                    yaw_err_deg_mean=float(np.degrees(dyaw).mean()),
                    yaw_err_mod180_deg_mean=float(np.degrees(np.minimum(dyaw, np.pi - dyaw)).mean()),
                    z_center_err_mean=float(dz.mean()), z_center_err_rmse=rmse(dz),
                    z_bottom_err_mean=float(dbottom.mean()), z_bottom_err_rmse=rmse(dbottom),
                    lwh_err_mean=(p[:, 3:6] - g[:, 3:6]).mean(0).tolist(),
                    lwh_abs_err_mean=np.abs(p[:, 3:6] - g[:, 3:6]).mean(0).tolist())


def iter_sequences(frames: list[DetectionFrame]) -> Iterator[tuple[tuple[str, str], list[DetectionFrame]]]:
    """Time-ordered frames per (scenario, agent): the unit for a future tracking evaluation."""
    groups: dict = {}
    for f in frames:
        groups.setdefault(f.sequence, []).append(f)
    yield from groups.items()

"""BEV tracking benchmark for cached CooperScene detections.

Cached LiDAR-frame detections (``batch_detect.py``) are mapped to the CARLA
world with each frame's ``pose_to_matrix(ego_pose)`` and replayed, per
(take, agent) sequence in time order, through the production
``BevFusionKalmanTracker``. Scoring is horizontal only: world X/Y center
distance (2 m inclusive gate, class-agnostic vehicles) and length-axis heading;
height, z and box size never enter matching or metrics.

Ground-truth identities come from the native YAML ``vehicles`` IDs, cached
locally by :class:`GroundTruthTrackCache` and verified against the info-pkl
LiDAR-frame boxes. Regions (front-camera FOV, range) are evaluated in each
frame's LiDAR frame, with the same ignore semantics as
:mod:`src.evaluation.cooperscene_detection`: tracks matched (BEV, 2 m) to a GT
outside the scored set are dropped, unmatched tracks count only inside the
region. The tracker always sees every detection; filtering happens at scoring.
"""
from __future__ import annotations

from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
import multiprocessing
import pickle
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np
from scipy.optimize import linear_sum_assignment

from src.evaluation.cooperscene_detection import PCD_LIMIT, center_match, in_camera_fov
from src.evaluation.cooperscene_tracking import GeometricTrackingEvaluator
from src.localization.cooperscene import CooperSceneSequenceDataset, pose_to_matrix
from src.localization.cooperscene_calibration import camera_intrinsic, camera_T_lidar
from src.realtime.bevfusion_tracker import BevFusionKalmanTracker, WorldDetection

FRAME_US = 100_000  # CooperScene timestamps are 10 Hz frame numbers
SPLIT_DIRS = {"test": "test", "val": "validate"}
LABEL = "car"


def box_matrices(boxes: np.ndarray) -> np.ndarray:
    """[x, y, z, l, w, h, yaw] rows -> (N, 4, 4) object poses (yaw about local Z)."""
    poses = np.tile(np.eye(4), (len(boxes), 1, 1))
    c, s = np.cos(boxes[:, 6]), np.sin(boxes[:, 6])
    poses[:, 0, 0], poses[:, 0, 1], poses[:, 1, 0], poses[:, 1, 1] = c, -s, s, c
    poses[:, :3, 3] = boxes[:, :3]
    return poses


def matrices_to_boxes(poses: np.ndarray, dimensions: np.ndarray) -> np.ndarray:
    """(N, 4, 4) poses + (N, 3) LWH -> [x, y, z, l, w, h, yaw] with yaw of the projected length axis."""
    yaw = np.arctan2(poses[:, 1, 0], poses[:, 0, 0])
    return np.concatenate([poses[:, :3, 3], dimensions, yaw[:, None]], 1) if len(poses) else np.zeros((0, 7))


class GroundTruthTrackCache:
    """Per-frame native-ID GT boxes (world and LiDAR frame) cached under one directory.

    One ``<split>.npz`` holds frame arrays (take, agent, frame, map_T_lidar,
    gt_offsets) and box arrays (gt_id, world/lidar ``[x,y,z,l,w,h,yaw]``,
    LiDAR point count or -1, in_pkl). YAMLs are read once over the network
    mount; ``verification`` in the npz records the agreement with the pkl.
    """

    def __init__(self, directory: str | Path) -> None:
        self.directory = Path(directory)

    def path(self, split: str) -> Path:
        return self.directory / f"{split}.npz"

    def build(self, split: str, *, data_root: str | Path, info_pkl: str | Path,
              predictions_npz: str | Path, workers: int = 16) -> dict:
        with open(info_pkl, "rb") as stream:
            infos = pickle.load(stream)["data_list"]
        with np.load(predictions_npz) as pred:
            gt_counts, gt_num_points, sample_idx = pred["gt_counts"], pred["gt_num_points"], pred["sample_idx"]
        gt_offsets = np.concatenate([[0], np.cumsum(gt_counts)])
        points = {int(idx): gt_num_points[gt_offsets[i]:gt_offsets[i + 1]] for i, idx in enumerate(sample_idx)}
        by_key = {(str(info["scenario"]), str(info["agent_id"]), int(info["timestamp"])): (i, info)
                  for i, info in enumerate(infos)}
        dataset = CooperSceneSequenceDataset.metadata_only(data_root, cache_frames=False)
        frames_out, boxes_out = [], []
        stats = Counter()
        max_err = dict(pose=0.0, center=0.0, dims=0.0, yaw=0.0)
        for take, agent in sorted({key[:2] for key in by_key}, key=lambda k: (int(k[0]), int(k[1]))):
            sequence = dataset.sequence(SPLIT_DIRS[split], take, agent)
            with ThreadPoolExecutor(workers) as pool:
                frames = list(pool.map(sequence.__getitem__, range(len(sequence))))
            for frame in frames:
                key = (take, agent, int(frame.frame_id))
                if key not in by_key:
                    stats["yaml_frames_without_pkl_sample"] += 1
                    continue
                index, info = by_key.pop(key)
                stats["frames"] += 1
                pkl_pose = pose_to_matrix(info["ego_pose"])
                max_err["pose"] = max(max_err["pose"], float(np.abs(pkl_pose - frame.map_T_lidar).max()))
                lidar_T_map = np.linalg.inv(frame.map_T_lidar)
                ids = np.array([int(box.track_id) for box in frame.vehicle_boxes], int)
                world_poses = np.array([box.map_T_object for box in frame.vehicle_boxes]).reshape(-1, 4, 4)
                dims = np.array([box.dimensions_lwh for box in frame.vehicle_boxes], float).reshape(-1, 3)
                world = matrices_to_boxes(world_poses, dims)
                lidar = matrices_to_boxes(lidar_T_map @ world_poses, dims)
                gt = np.asarray([x["bbox_3d"] for x in info["instances"]], float).reshape(-1, 7)
                count = np.full(len(ids), -1, int)
                in_pkl = np.zeros(len(ids), bool)
                match = center_match(lidar, gt, 0.05) if len(gt) else np.full(len(ids), -1)
                for i, j in enumerate(match):
                    if j < 0:
                        continue
                    in_pkl[i], count[i] = True, points[index][j]
                    max_err["center"] = max(max_err["center"], float(np.abs(lidar[i, :3] - gt[j, :3]).max()))
                    max_err["dims"] = max(max_err["dims"], float(np.abs(lidar[i, 3:6] - gt[j, 3:6]).max()))
                    dyaw = abs((lidar[i, 6] - gt[j, 6] + np.pi) % (2 * np.pi) - np.pi)
                    max_err["yaw"] = max(max_err["yaw"], float(dyaw))
                    lidar[i] = gt[j]  # the pkl box is the detection benchmark's reference
                stats["pkl_gt"] += len(gt)
                stats["pkl_gt_matched"] += int(in_pkl.sum())
                stats["yaml_gt_not_in_pkl"] += int((~in_pkl).sum())
                stats["pkl_gt_same_order"] += int(np.array_equal(match, np.arange(len(gt))))
                frames_out.append((int(take), int(agent), int(frame.frame_id), np.asarray(frame.map_T_lidar), len(ids)))
                boxes_out.append((ids, world, lidar, count, in_pkl))
        stats["pkl_samples_without_yaml"] = len(by_key)
        verification = dict(stats, **{f"max_abs_{k}_error": v for k, v in max_err.items()})
        self.directory.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            self.path(split),
            take=np.array([f[0] for f in frames_out]), agent=np.array([f[1] for f in frames_out]),
            frame=np.array([f[2] for f in frames_out]), map_T_lidar=np.array([f[3] for f in frames_out]),
            gt_offsets=np.concatenate([[0], np.cumsum([f[4] for f in frames_out])]),
            gt_id=np.concatenate([b[0] for b in boxes_out]), gt_world=np.concatenate([b[1] for b in boxes_out]),
            gt_lidar=np.concatenate([b[2] for b in boxes_out]), gt_points=np.concatenate([b[3] for b in boxes_out]),
            gt_in_pkl=np.concatenate([b[4] for b in boxes_out]),
            verification=np.array(repr(verification)))
        return verification

    def load(self, split: str) -> dict[str, np.ndarray]:
        with np.load(self.path(split)) as data:
            return {key: data[key] for key in data.files}


@dataclass
class SequenceData:
    """One (take, agent) sequence: per-frame GT and cached detections."""

    split: str
    take: str
    agent: str
    frames: np.ndarray                      # (F,) frame numbers
    map_T_lidar: np.ndarray                 # (F, 4, 4)
    gt_ids: list[np.ndarray] = field(default_factory=list)
    gt_world: list[np.ndarray] = field(default_factory=list)   # [x,y,z,l,w,h,yaw] world
    gt_lidar: list[np.ndarray] = field(default_factory=list)   # same box, frame LiDAR
    gt_points: list[np.ndarray] = field(default_factory=list)  # -1: not in pkl
    det_lidar: list[np.ndarray] = field(default_factory=list)
    det_scores: list[np.ndarray] = field(default_factory=list)

    @property
    def name(self) -> str:
        return f"take{self.take}_agent{self.agent}"

    @classmethod
    def load_split(cls, split: str, gt_cache: GroundTruthTrackCache,
                   predictions_npz: str | Path) -> list["SequenceData"]:
        gt = gt_cache.load(split)
        with np.load(predictions_npz) as data:
            pred = {key: data[key] for key in data.files}
        offsets = np.concatenate([[0], np.cumsum(pred["counts"])])
        samples = {(str(s), str(a), int(t)): i for i, (s, a, t)
                   in enumerate(zip(pred["scenario"], pred["agent_id"], pred["timestamp"]))}
        sequences: dict[tuple[str, str], SequenceData] = {}
        go = gt["gt_offsets"]
        for f in range(len(gt["frame"])):
            take, agent, frame = str(gt["take"][f]), str(gt["agent"][f]), int(gt["frame"][f])
            seq = sequences.get((take, agent))
            if seq is None:
                seq = sequences[(take, agent)] = cls(split, take, agent, [], [])
            seq.frames.append(frame)
            seq.map_T_lidar.append(gt["map_T_lidar"][f])
            sl = slice(go[f], go[f + 1])
            seq.gt_ids.append(gt["gt_id"][sl])
            seq.gt_world.append(gt["gt_world"][sl])
            seq.gt_lidar.append(gt["gt_lidar"][sl])
            seq.gt_points.append(gt["gt_points"][sl])
            i = samples[(take, agent, frame)]
            ps = slice(offsets[i], offsets[i + 1])
            vehicle = pred["labels"][ps] == 0
            seq.det_lidar.append(pred["boxes"][ps][vehicle].astype(float))
            seq.det_scores.append(pred["scores"][ps][vehicle].astype(float))
        result = []
        for seq in sequences.values():
            order = np.argsort(seq.frames)
            seq.frames = np.asarray(seq.frames)[order]
            seq.map_T_lidar = np.asarray(seq.map_T_lidar)[order]
            for name in ("gt_ids", "gt_world", "gt_lidar", "gt_points", "det_lidar", "det_scores"):
                values = getattr(seq, name)
                setattr(seq, name, [values[i] for i in order])
            result.append(seq)
        return sorted(result, key=lambda s: (int(s.take), int(s.agent)))


@dataclass(frozen=True)
class TrackerConfig:
    """Knobs of the production tracker (``ros_frame_adapter`` passes score_threshold as birth)."""

    birth: float = 0.2
    continuation: float = 0.2
    confirmation_hits: int = 1
    coast_s: float = 0.5
    reacquisition_s: float = 0.0
    duplicate_guard: bool = False

    @property
    def label(self) -> str:
        return (f"b{self.birth:g}/c{self.continuation:g}/h{self.confirmation_hits}/coast{self.coast_s:g}"
                f"/reacq{self.reacquisition_s:g}/{'guard' if self.duplicate_guard else 'noguard'}")

    def build(self) -> BevFusionKalmanTracker:
        return BevFusionKalmanTracker(
            maximum_coast_seconds=self.coast_s, minimum_confirmation_hits=self.confirmation_hits,
            birth_score_threshold=self.birth, continuation_score_threshold=self.continuation,
            suppress_duplicate_births=self.duplicate_guard, maximum_reacquisition_seconds=self.reacquisition_s)


@dataclass
class FrameOutput:
    """Boxes to score for one frame (tracks, or raw detections with frame-local IDs)."""

    ids: np.ndarray            # (N,) str
    world: np.ndarray          # (N, 7)
    lidar: np.ndarray          # (N, 7)
    coasted: np.ndarray        # (N,) bool: no detection associated at this timestamp
    scores: np.ndarray         # (N,) last correcting detection score
    speed: np.ndarray          # (N,) filter horizontal speed m/s (nan for raw detections)


class SequenceTrackerRun:
    """Replay one sequence through the production tracker, collecting per-frame outputs."""

    def __init__(self, sequence: SequenceData, config: TrackerConfig) -> None:
        self.sequence, self.config = sequence, config

    def detections(self, f: int) -> list[WorldDetection]:
        seq = self.sequence
        boxes, scores = seq.det_lidar[f], seq.det_scores[f]
        poses = seq.map_T_lidar[f] @ box_matrices(boxes)
        return [WorldDetection(pose, tuple(box[3:6]), LABEL, float(score))
                for pose, box, score in zip(poses, boxes, scores)]

    def run(self) -> list[FrameOutput]:
        seq, tracker, outputs = self.sequence, self.config.build(), []
        for f, frame in enumerate(seq.frames):
            timestamp = int(frame) * FRAME_US
            tracker.update(self.detections(f), timestamp_us=timestamp)
            boxes = tracker.boxes_at(timestamp)
            # Read-only diagnostics: boxes_at() yields this same filtered order.
            live = [t for t in tracker._tracks
                    if t.confirmed and timestamp - t.last_observed_us <= tracker.maximum_coast_us]
            assert len(live) == len(boxes)
            poses = np.array([b.world_T_object for b in boxes]).reshape(-1, 4, 4)
            dims = np.array([b.dimensions_lwh for b in boxes], float).reshape(-1, 3)
            outputs.append(FrameOutput(
                ids=np.array([b.track_id for b in boxes], str),
                world=matrices_to_boxes(poses, dims),
                lidar=matrices_to_boxes(np.linalg.inv(seq.map_T_lidar[f]) @ poses, dims),
                coasted=np.array([t.last_observed_us < timestamp for t in live], bool),
                scores=np.array([b.score for b in boxes], float),
                speed=np.array([np.hypot(*t.state[3:5]) for t in live], float)))
        return outputs

    @staticmethod
    def raw_detections(sequence: SequenceData, threshold: float) -> list[FrameOutput]:
        """Per-frame detections above ``threshold`` with frame-local IDs (no tracker)."""
        outputs = []
        for f, frame in enumerate(sequence.frames):
            keep = sequence.det_scores[f] >= threshold
            lidar = sequence.det_lidar[f][keep]
            world = matrices_to_boxes(sequence.map_T_lidar[f] @ box_matrices(lidar), lidar[:, 3:6])
            outputs.append(FrameOutput(np.array([f"{frame}:{i}" for i in range(len(lidar))], str), world, lidar,
                                       np.zeros(len(lidar), bool), sequence.det_scores[f][keep],
                                       np.full(len(lidar), np.nan)))
        return outputs


class Region:
    """Scored region in the frame's LiDAR frame; ``fov`` uses the benchmark front-camera calibration."""

    NAMES = ("fov_r50", "all_r50", "all")

    def __init__(self, name: str, calibration: str = "benchmark") -> None:
        if name not in self.NAMES:
            raise ValueError(f"unknown region {name!r}")
        self.name = name
        self._camera = {agent: (camera_intrinsic(agent, calibration), camera_T_lidar(agent, calibration))
                        for agent in ("1", "2", "3")}

    @staticmethod
    def in_limit(lidar: np.ndarray) -> np.ndarray:
        return np.all((lidar[:, :3] >= PCD_LIMIT[:3]) & (lidar[:, :3] <= PCD_LIMIT[3:]), axis=1)

    def __call__(self, sequence: SequenceData, lidar: np.ndarray) -> np.ndarray:
        inside = self.in_limit(lidar)
        if self.name in ("fov_r50", "all_r50"):
            inside &= np.hypot(lidar[:, 0], lidar[:, 1]) <= 50
        if self.name == "fov_r50" and len(lidar):
            inside &= in_camera_fov(lidar, *self._camera[sequence.agent])
        return inside


class BevHota:
    """HOTA (Luiten et al. 2021, TrackEval algorithm) with BEV similarity max(0, 1 - d / gate)."""

    ALPHAS = np.arange(0.05, 0.99, 0.05)

    def __init__(self, gate_m: float = 2.0) -> None:
        self.gate = gate_m
        self._frames: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
        self._gt_index: dict[str, int] = {}
        self._tr_index: dict[str, int] = {}

    def add(self, gt_ids: Sequence[str], tr_ids: Sequence[str], gt_xy: np.ndarray, tr_xy: np.ndarray) -> None:
        g = np.array([self._gt_index.setdefault(str(i), len(self._gt_index)) for i in gt_ids], int)
        t = np.array([self._tr_index.setdefault(str(i), len(self._tr_index)) for i in tr_ids], int)
        d = np.linalg.norm(gt_xy[:, None, :2] - tr_xy[None, :, :2], axis=-1) if len(g) and len(t) \
            else np.zeros((len(g), len(t)))
        self._frames.append((g, t, np.clip(1 - d / self.gate, 0, None)))

    def counts(self) -> dict[str, np.ndarray]:
        """Summable per-alpha counts (TrackEval combine_sequences weights Ass*/LocA by TP)."""
        n_alpha, n_g, n_t = len(self.ALPHAS), len(self._gt_index), len(self._tr_index)
        potential = np.zeros((n_g, n_t))
        gt_count, tr_count = np.zeros((n_g, 1)), np.zeros((1, n_t))
        for g, t, sim in self._frames:
            if len(g) and len(t):
                denom = sim.sum(0, keepdims=True) + sim.sum(1, keepdims=True) - sim
                potential[g[:, None], t[None]] += np.divide(sim, denom, out=np.zeros_like(sim), where=denom > 1e-10)
            gt_count[g, 0] += 1
            tr_count[0, t] += 1
        alignment = potential / np.maximum(1e-10, gt_count + tr_count - potential)
        tp, fn, fp, loc = (np.zeros(n_alpha) for _ in range(4))
        matches = np.zeros((n_alpha, n_g, n_t))
        for g, t, sim in self._frames:
            if not len(g) or not len(t):
                fn += len(g); fp += len(t)
                continue
            score = alignment[g[:, None], t[None]] * sim
            rows, cols = linear_sum_assignment(-score)
            for a, alpha in enumerate(self.ALPHAS):
                ok = sim[rows, cols] >= alpha - np.finfo(float).eps
                r, c = rows[ok], cols[ok]
                tp[a] += len(r); fn[a] += len(g) - len(r); fp[a] += len(t) - len(r)
                loc[a] += sim[r, c].sum()
                matches[a, g[r], t[c]] += 1
        ass_a, ass_re, ass_pr = np.zeros(n_alpha), np.zeros(n_alpha), np.zeros(n_alpha)
        for a in range(n_alpha):
            m = matches[a]
            ass_a[a] = (m * m / np.maximum(1, gt_count + tr_count - m)).sum()
            ass_re[a] = (m * m / np.maximum(1, gt_count)).sum()
            ass_pr[a] = (m * m / np.maximum(1, tr_count)).sum()
        return dict(tp=tp, fn=fn, fp=fp, loc_sum=loc, ass_a_tp=ass_a, ass_re_tp=ass_re, ass_pr_tp=ass_pr)

    @staticmethod
    def combine(counts: Iterable[dict[str, np.ndarray]]) -> dict[str, float]:
        total = {key: sum(c[key] for c in counts) for key in ("tp", "fn", "fp", "loc_sum", "ass_a_tp",
                                                               "ass_re_tp", "ass_pr_tp")} if counts else {}
        if not total:
            return {}
        tp = np.maximum(1, total["tp"])
        det_a = total["tp"] / np.maximum(1, total["tp"] + total["fn"] + total["fp"])
        ass_a = total["ass_a_tp"] / tp
        hota = np.sqrt(det_a * ass_a)
        return dict(HOTA=float(hota.mean()), DetA=float(det_a.mean()), AssA=float(ass_a.mean()),
                    AssRe=float((total["ass_re_tp"] / tp).mean()), AssPr=float((total["ass_pr_tp"] / tp).mean()),
                    LocA=float((np.maximum(1e-10, total["loc_sum"]) / np.maximum(1e-10, total["tp"])).mean()), HOTA_alpha50=float(hota[9]))


def ego_mask(lidar: np.ndarray) -> np.ndarray:
    """GT boxes containing the LiDAR origin in BEV: the recording vehicle itself (never observable)."""
    c, s = np.cos(lidar[:, 6]), np.sin(lidar[:, 6])
    local_x, local_y = -c * lidar[:, 0] - s * lidar[:, 1], s * lidar[:, 0] - c * lidar[:, 1]
    return (np.abs(local_x) < lidar[:, 3] / 2) & (np.abs(local_y) < lidar[:, 4] / 2)


class SequenceScorer:
    """BEV CLEAR/IDF1 (GeometricTrackingEvaluator), HOTA and MT/PT/ML for one sequence.

    Scored GT ("care"): inside the PCD limit and the region, not the ego box,
    and for ``gt5pts`` with >= 5 LiDAR points. Outputs are pre-matched (BEV
    Hungarian, gate) to every GT; those matched to a non-care GT are ignored,
    unmatched outputs count only when inside the region.
    """

    GT_SETS = ("allgt", "gt5pts")

    def __init__(self, region: Region, gt_set: str = "allgt", *, gate_m: float = 2.0, min_points: int = 5,
                 identity: bool = True, diagnose: bool = False, continuation: float = 0.0,
                 birth: float = 0.0) -> None:
        if gt_set not in self.GT_SETS:
            raise ValueError(f"unknown gt set {gt_set!r}")
        self.region, self.gt_set, self.gate, self.min_points = region, gt_set, gate_m, min_points
        self.identity, self.diagnose = identity, diagnose
        self.continuation, self.birth = continuation, birth

    def care(self, sequence: SequenceData, f: int) -> np.ndarray:
        lidar = sequence.gt_lidar[f]
        care = self.region(sequence, lidar) & ~ego_mask(lidar)
        if self.gt_set == "gt5pts":
            care &= sequence.gt_points[f] >= self.min_points
        return care

    def score(self, sequence: SequenceData, outputs: Sequence[FrameOutput]) -> dict[str, Any]:
        evaluator = GeometricTrackingEvaluator(center_threshold_m=self.gate, identity_metrics=self.identity,
                                               bev_only=True)
        hota = BevHota(self.gate)
        coverage: dict[int, list[int]] = {}
        sq_err = heading = 0.0
        failures: Counter[str] = Counter()
        last_match: dict[str, int] = {}
        first_seen: dict[str, int] = {}
        coasted_err, observed_err, speed_err = [], [], []
        previous_gt: dict[int, np.ndarray] = {}
        for f, out in enumerate(outputs):
            for identity in out.ids:
                first_seen.setdefault(str(identity), int(sequence.frames[f]))
            care = self.care(sequence, f)
            gt_world = sequence.gt_world[f]
            match = center_match(out.world, gt_world, self.gate)
            matched = match >= 0
            keep = self.region(sequence, out.lidar)
            keep[matched] = care[match[matched]]
            gt_ids = sequence.gt_ids[f][care]
            gt_boxes = [dict(track_id=str(i), world_T_object=p, dimensions_lwh=b[3:6], label=LABEL)
                        for i, p, b in zip(gt_ids, box_matrices(gt_world[care]), gt_world[care])]
            kept = np.nonzero(keep)[0]
            tr_boxes = [dict(track_id=str(out.ids[k]), world_T_object=p, dimensions_lwh=out.world[k, 3:6], label=LABEL)
                        for k, p in zip(kept, box_matrices(out.world[kept]))]
            result = evaluator.update(sequence_id=sequence.name, timestamp_us=int(sequence.frames[f]) * FRAME_US,
                                      gt=gt_boxes, predictions=tr_boxes)
            hota.add(gt_ids.astype(str), out.ids[kept], gt_world[care], out.world[kept])
            matched_gt = set()
            for m in result["matches"]:
                matched_gt.add(m["gt_index"])
                sq_err += m["center_error_m"] ** 2
                heading += m["heading_error_deg"]
            for i, identity in enumerate(gt_ids):
                cover = coverage.setdefault(int(identity), [0, 0])
                cover[0] += 1
                cover[1] += i in matched_gt
            if self.diagnose:
                self._diagnose(sequence, f, out, kept, care, result, failures, last_match, first_seen,
                               coasted_err, observed_err, speed_err, previous_gt)
        summary = evaluator.summary()
        counts = {key: summary.get(key, 0) for key in ("frames", "tp", "fp", "fn", "id_switches", "fragmentation",
                                                     "idtp", "idfp", "idfn")}
        ratios = [matched / visible for visible, matched in coverage.values()]
        counts.update(sq_err=sq_err, heading_sum=heading, gt_ids=len(ratios),
                      mt=sum(r >= 0.8 for r in ratios), ml=sum(r < 0.2 for r in ratios),
                      hota=hota.counts() if self.identity else None)
        counts["pt"] = counts["gt_ids"] - counts["mt"] - counts["ml"]
        if self.diagnose:
            counts["failures"] = dict(failures)
            counts["error_samples"] = dict(coasted=coasted_err, observed=observed_err, speed=speed_err)
        return counts

    def _diagnose(self, sequence, f, out, kept, care, result, failures, last_match, first_seen,
                  coasted_err, observed_err, speed_err, previous_gt) -> None:
        """Bucket each FN, FP and ID switch into a likely cause (diagnostic, not a metric)."""
        gt_ids, gt_world = sequence.gt_ids[f][care], sequence.gt_world[f][care]
        live = set(map(str, out.ids))
        frame = int(sequence.frames[f])  # frame numbers: the tracker's clock (FRAME_US each)
        for switch in result["id_switches"]:
            gap = frame - last_match.get(switch["gt_id"], frame)
            if switch["previous_pred_id"] in live:
                failures["idsw/previous_track_still_live(swap_or_duplicate)"] += 1
            else:
                fresh = frame - first_seen[switch["pred_id"]] <= gap
                failures[f"idsw/previous_track_ended/{'new_track' if fresh else 'old_track'}/"
                         f"gap{'<=5' if gap <= 5 else '6-20' if gap <= 20 else '>20'}"] += 1
        for m in result["matches"]:
            last_match[m["gt_id"]] = frame
            k = kept[m["pred_index"]]
            (coasted_err if out.coasted[k] else observed_err).append(m["center_error_m"])
            identity = int(gt_ids[m["gt_index"]])
            if identity in previous_gt and np.isfinite(out.speed[k]):
                dt_s = (frame - int(sequence.frames[f - 1])) * FRAME_US / 1e6  # previous_gt is frame f - 1
                gt_speed = np.linalg.norm(gt_world[m["gt_index"], :2] - previous_gt[identity]) / dt_s
                speed_err.append((float(out.speed[k] - gt_speed), float(out.scores[k]), bool(out.coasted[k])))
        dets, scores = sequence.det_lidar[f], sequence.det_scores[f]
        det_world = matrices_to_boxes(sequence.map_T_lidar[f] @ box_matrices(dets), dets[:, 3:6])
        for i in result["missed_gt_indices"]:
            d = np.linalg.norm(det_world[:, :2] - gt_world[i, :2], axis=1)
            best = scores[d <= self.gate].max(initial=0.0)
            t = np.linalg.norm(out.world[:, :2] - gt_world[i, :2], axis=1) if len(out.ids) else np.zeros(0)
            if best == 0:
                failures["fn/no_detection_within_gate"] += 1
            elif best < self.continuation:
                failures["fn/detection_below_continuation"] += 1
            elif np.any(t <= 2 * self.gate):
                failures["fn/track_offset_gate_to_2x_gate"] += 1
            elif best < self.birth:
                failures["fn/detection_between_continuation_and_birth_no_track"] += 1
            else:
                failures["fn/birth_score_detection_but_no_track(unconfirmed_or_suppressed)"] += 1
        matched_xy = gt_world[[m["gt_index"] for m in result["matches"]], :2]
        for j in result["false_positive_indices"]:
            k = kept[j]
            if out.coasted[k]:
                failures["fp/coasted_track_without_detection"] += 1
            elif len(matched_xy) and np.linalg.norm(matched_xy - out.world[k, :2], axis=1).min() <= 2 * self.gate:
                failures["fp/observed_near_tracked_gt(duplicate_or_offset)"] += 1
            else:
                failures["fp/observed_isolated(detector_fp)"] += 1
        previous_gt.clear()
        previous_gt.update({int(i): b[:2] for i, b in zip(sequence.gt_ids[f], sequence.gt_world[f])})


def summarize(counts: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Combine summable per-sequence counts into metrics."""
    total = {key: sum(c[key] for c in counts) for key in ("frames", "tp", "fp", "fn", "id_switches", "fragmentation",
                                                       "idtp", "idfp", "idfn", "sq_err", "heading_sum",
                                                       "gt_ids", "mt", "pt", "ml")}
    tp, fp, fn = total["tp"], total["fp"], total["fn"]
    n_gt = tp + fn
    result = dict(n_gt=n_gt, tp=tp, fp=fp, fn=fn,
                  precision=tp / (tp + fp) if tp + fp else None, recall=tp / n_gt if n_gt else None,
                  MODA=1 - (fn + fp) / n_gt if n_gt else None,
                  bev_center_rmse_m=float(np.sqrt(total["sq_err"] / tp)) if tp else None,
                  heading_err_mod180_deg=total["heading_sum"] / tp if tp else None)
    if counts and counts[0]["hota"] is not None:
        idtp, idfp, idfn = total["idtp"], total["idfp"], total["idfn"]
        result.update(MOTA=1 - (fn + fp + total["id_switches"]) / n_gt if n_gt else None,
                      IDF1=2 * idtp / (2 * idtp + idfp + idfn) if idtp + idfp + idfn else None,
                      IDP=idtp / (idtp + idfp) if idtp + idfp else None, IDR=idtp / (idtp + idfn) if idtp + idfn else None,
                      id_switches=total["id_switches"], fragmentation=total["fragmentation"],
                      gt_ids=total["gt_ids"], MT=total["mt"], PT=total["pt"], ML=total["ml"],
                      **BevHota.combine([c["hota"] for c in counts]))
    if "failures" in counts[0]:
        failures = Counter()
        for c in counts:
            failures.update(c["failures"])
        result["failures"] = dict(sorted(failures.items()))
        samples = {k: [x for c in counts for x in c["error_samples"][k]] for k in ("coasted", "observed", "speed")}
        rmse = lambda x: float(np.sqrt(np.mean(np.square(x)))) if len(x) else None
        speed = np.array(samples["speed"], float).reshape(-1, 3)
        big = np.abs(speed[:, 0]) > 3
        result["localization"] = dict(
            observed_matches=len(samples["observed"]), observed_rmse_m=rmse(samples["observed"]),
            coasted_matches=len(samples["coasted"]), coasted_rmse_m=rmse(samples["coasted"]),
            speed_pairs=len(speed), speed_abs_err_median_mps=float(np.median(np.abs(speed[:, 0]))) if len(speed) else None,
            speed_abs_err_p95_mps=float(np.percentile(np.abs(speed[:, 0]), 95)) if len(speed) else None,
            speed_err_gt3mps=int(big.sum()),
            speed_err_gt3mps_last_score_lt_0_3=int((big & (speed[:, 1] < 0.3)).sum()),
            speed_err_gt3mps_coasted=int((big & (speed[:, 2] > 0)).sum()))
    return result


# ---------------------------------------------------------------------------
# Multiprocessing harness. Each pool's initializer points this worker-local
# global at its benchmark's sequences (inherited via fork, not pickled), so
# only (key, job) tuples cross process boundaries.
_SEQUENCES: dict[tuple[str, str], list[SequenceData]] = {}


def _init_worker(sequences: dict[tuple[str, str], list[SequenceData]]) -> None:
    global _SEQUENCES
    _SEQUENCES = sequences


def _score_job(job: tuple) -> tuple:
    key, index, spec, regions, gt_sets, diagnose = job
    sequence = _SEQUENCES[key][index]
    if isinstance(spec, TrackerConfig):
        outputs = SequenceTrackerRun(sequence, spec).run()
        identity, continuation, birth = True, spec.continuation, spec.birth
    else:
        outputs = SequenceTrackerRun.raw_detections(sequence, float(spec))
        identity, continuation, birth = False, float(spec), float(spec)
    result = {}
    for region in regions:
        reg = Region(region)
        for gt_set in gt_sets:
            scorer = SequenceScorer(reg, gt_set, identity=identity, diagnose=diagnose and identity,
                                    continuation=continuation, birth=birth)
            result[f"{region}/{gt_set}"] = scorer.score(sequence, outputs)
    return job, sequence.take, sequence.agent, result


class TrackingBenchmark:
    """Run tracker configs (or raw-detection thresholds) over loaded splits in parallel."""

    def __init__(self, sequences: dict[tuple[str, str], list[SequenceData]], *, processes: int | None = None) -> None:
        self.sequences = sequences
        self.processes = processes or min(32, multiprocessing.cpu_count())

    def evaluate(self, specs: Sequence[TrackerConfig | float], keys: Sequence[tuple[str, str]], *,
                 regions: Sequence[str] = Region.NAMES, gt_sets: Sequence[str] = ("allgt",),
                 diagnose: bool = False) -> list[dict[tuple[str, str], dict[str, Any]]]:
        """For each spec: {(split, variant): {"overall"|"take<N>": {region/gt_set: metrics}}}."""
        jobs = [(key, i, spec_index, tuple(regions), tuple(gt_sets), diagnose)
                for spec_index in range(len(specs)) for key in keys for i in range(len(self.sequences[key]))]
        jobs.sort(key=lambda j: -len(self.sequences[j[0]][j[1]].frames))
        resolved = [(key, i, specs[s], r, g, d) for key, i, s, r, g, d in jobs]
        context = multiprocessing.get_context("fork")
        with context.Pool(self.processes, initializer=_init_worker, initargs=(self.sequences,)) as pool:
            done = pool.map(_score_job, resolved, chunksize=1)
        grouped: list[dict] = [dict() for _ in specs]
        for (key, i, s, *_), (_, take, agent, result) in zip(jobs, done):
            grouped[s].setdefault(key, {}).setdefault(take, []).append(result)
        out = []
        for per_key in grouped:
            out.append({key: self._combine(per_take) for key, per_take in per_key.items()})
        return out

    @staticmethod
    def _combine(per_take: dict[str, list[dict]]) -> dict[str, dict]:
        everything = [r for results in per_take.values() for r in results]
        names = everything[0].keys()
        combined = {"overall": {n: summarize([r[n] for r in everything]) for n in names}}
        for take, results in sorted(per_take.items(), key=lambda kv: int(kv[0])):
            combined[f"take{take}"] = {n: summarize([r[n] for r in results]) for n in names}
        return combined

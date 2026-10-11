"""Class-aware XYZ center matching and global geometric IDF1.

Every box must already be in the same frame; this module never transforms GT
into tracker inputs. Matching maximizes the number of pairs within the inclusive
distance gate, then minimizes their distance. IDF1 uses a separate sequence-wide
one-to-one identity assignment over all geometrically eligible frame pairs.
This is center-distance evaluation, not official IoU MOTChallenge or HOTA.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import math
from typing import Any, Mapping, Sequence

import numpy as np
from scipy.optimize import linear_sum_assignment


@dataclass(frozen=True)
class EvaluationBox:
    track_id: str
    world_T_object: np.ndarray
    dimensions_lwh: np.ndarray
    label: str = "car"

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "EvaluationBox":
        pose = np.asarray(value["world_T_object"], dtype=float).reshape(4, 4)
        dimensions = np.asarray(value["dimensions_lwh"], dtype=float)
        identity = str(value["track_id"])
        label = str(value.get("label", value.get("text_label", "car")))
        if not identity or not label or dimensions.shape != (3,):
            raise ValueError("evaluation boxes require identity, label and LWH dimensions")
        if not np.all(np.isfinite(pose)) or not np.all(np.isfinite(dimensions)) or min(dimensions) <= 0:
            raise ValueError("evaluation box geometry must be finite with positive dimensions")
        if not np.allclose(pose[3], (0, 0, 0, 1)) or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(pose[:3, :3]), 1, atol=1e-5):
            raise ValueError("evaluation poses must be rigid transforms")
        return cls(identity, pose, dimensions, label)


class GeometricTrackingEvaluator:
    """Accumulate one or more independent sequences without mixing identities."""

    def __init__(self, *, center_threshold_m: float = 2.0, identity_metrics: bool = True) -> None:
        if not math.isfinite(center_threshold_m) or center_threshold_m <= 0:
            raise ValueError("center_threshold_m must be positive and finite")
        self.threshold = center_threshold_m
        self.identity_metrics = identity_metrics
        self.frames = self.tp = self.fp = self.fn = self.switches = self.fragments = 0
        self._gt_counts: Counter[tuple[str, str]] = Counter()
        self._pred_counts: Counter[tuple[str, str]] = Counter()
        self._eligible: Counter[tuple[tuple[str, str], tuple[str, str]]] = Counter()
        self._last_match: dict[tuple[str, str], str] = {}
        self._interrupted: set[tuple[str, str]] = set()
        self._last_time: dict[str, int] = {}
        self._errors: list[dict[str, Any]] = []

    def update(self, *, sequence_id: str, timestamp_us: int, predictions: Sequence[Mapping[str, Any]], gt: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
        if not sequence_id or isinstance(timestamp_us, bool) or not isinstance(timestamp_us, int) or timestamp_us < 0:
            raise ValueError("sequence and nonnegative integer timestamp are required")
        if sequence_id in self._last_time and timestamp_us <= self._last_time[sequence_id]:
            raise ValueError("evaluation timestamps must strictly increase within each sequence")
        self._last_time[sequence_id] = timestamp_us
        truth = [EvaluationBox.from_dict(box) for box in gt]
        predicted = [EvaluationBox.from_dict(box) for box in predictions]
        for boxes in (truth, predicted):
            if len({box.track_id for box in boxes}) != len(boxes):
                raise ValueError("box identities must be unique in each frame")
        distances = np.full((len(truth), len(predicted)), np.inf)
        for i, actual in enumerate(truth):
            for j, estimate in enumerate(predicted):
                if actual.label == estimate.label:
                    distance = float(np.linalg.norm(actual.world_T_object[:3, 3] - estimate.world_T_object[:3, 3]))
                    if distance <= self.threshold:
                        distances[i, j] = distance
                        self._eligible[((sequence_id, actual.track_id), (sequence_id, estimate.track_id))] += 1
        # Each forbidden pair costs more than the total cost of every feasible
        # pair. Thus Hungarian assignment prioritizes feasible cardinality.
        pairs = []
        if distances.size:
            forbidden = (min(distances.shape) + 1) * (self.threshold + 1)
            rows, columns = linear_sum_assignment(np.where(np.isfinite(distances), distances, forbidden))
            pairs = [(int(i), int(j)) for i, j in zip(rows, columns) if np.isfinite(distances[i, j])]
        matched_gt = {i for i, _ in pairs}
        matched_pred = {j for _, j in pairs}
        switches, matches = [], []
        for i, j in pairs:
            actual, estimate = truth[i], predicted[j]
            key = (sequence_id, actual.track_id)
            previous = self._last_match.get(key)
            if previous is not None and previous != estimate.track_id:
                switches.append({"gt_id": actual.track_id, "previous_pred_id": previous, "pred_id": estimate.track_id})
            if key in self._interrupted:
                self.fragments += 1
                self._interrupted.remove(key)
            self._last_match[key] = estimate.track_id
            heading = math.degrees(math.acos(float(np.clip(abs(actual.world_T_object[:3, 0] @ estimate.world_T_object[:3, 0]), 0, 1))))
            match = {"gt_id": actual.track_id, "pred_id": estimate.track_id, "gt_index": i, "pred_index": j,
                     "center_error_m": float(distances[i, j]),
                     "dimension_abs_error_lwh_m": np.abs(actual.dimensions_lwh - estimate.dimensions_lwh).tolist(),
                     "heading_error_deg": heading}
            matches.append(match)
            self._errors.append(match)
        for i, actual in enumerate(truth):
            key = (sequence_id, actual.track_id)
            self._gt_counts[key] += 1
            if i not in matched_gt and key in self._last_match:
                self._interrupted.add(key)
        self._pred_counts.update((sequence_id, box.track_id) for box in predicted)
        self.frames += 1
        self.tp += len(pairs)
        self.fp += len(predicted) - len(pairs)
        self.fn += len(truth) - len(pairs)
        self.switches += len(switches)
        return {"matches": matches, "missed_gt_indices": [i for i in range(len(truth)) if i not in matched_gt],
                "false_positive_indices": [j for j in range(len(predicted)) if j not in matched_pred], "id_switches": switches if self.identity_metrics else []}

    def summary(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "protocol": {"geometry": "class-aware 3D center distance", "center_threshold_m": self.threshold,
                         "gate_inclusive": True, "assignment": "maximum cardinality then minimum distance Hungarian",
                         "heading": "length-axis angular error modulo 180 degrees",
                         "id_switch": "change from previous matched identity including across unmatched/absent gaps",
                         "fragmentation": "recovery after an unmatched annotated frame; absent GT frames alone do not interrupt"},
            "frames": self.frames, "sequences": len(self._last_time), "tp": self.tp, "fp": self.fp, "fn": self.fn,
            "precision": self.tp / (self.tp + self.fp) if self.tp + self.fp else None,
            "recall": self.tp / (self.tp + self.fn) if self.tp + self.fn else None,
            "matched_center_error_mean_m": float(np.mean([m["center_error_m"] for m in self._errors])) if self._errors else None,
            "matched_center_error_rmse_m": float(np.sqrt(np.mean([m["center_error_m"] ** 2 for m in self._errors]))) if self._errors else None,
            "matched_dimensions_mae_lwh_m": np.mean([m["dimension_abs_error_lwh_m"] for m in self._errors], axis=0).tolist() if self._errors else None,
            "matched_heading_error_mean_deg": float(np.mean([m["heading_error_deg"] for m in self._errors])) if self._errors else None,
        }
        if self.identity_metrics:
            idtp = 0
            for sequence in self._last_time:
                actual = [key for key in self._gt_counts if key[0] == sequence]
                predicted = [key for key in self._pred_counts if key[0] == sequence]
                counts = np.asarray([[self._eligible[(g, p)] for p in predicted] for g in actual], dtype=int)
                if counts.size:
                    rows, columns = linear_sum_assignment(-counts)
                    idtp += int(counts[rows, columns].sum())
            idfn, idfp = sum(self._gt_counts.values()) - idtp, sum(self._pred_counts.values()) - idtp
            result.update(id_switches=self.switches, fragmentation=self.fragments, idtp=idtp, idfp=idfp, idfn=idfn,
                          geometric_idf1=2 * idtp / (2 * idtp + idfp + idfn) if 2 * idtp + idfp + idfn else None)
        return result


class GroundTruthContinuity:
    """Produce optional annotation-ID sensitivity segments without changing GT."""

    def __init__(self, *, minimum_jump_m: float = 10.0, maximum_speed_mps: float = 50.0, tolerance_m: float = 2.0) -> None:
        self.minimum_jump_m = minimum_jump_m
        self.maximum_speed_mps = maximum_speed_mps
        self.tolerance_m = tolerance_m
        self._previous: dict[str, tuple[int, dict[str, EvaluationBox]]] = {}
        self._segments: Counter[tuple[str, str]] = Counter()
        self.discontinuities: list[dict[str, Any]] = []

    def segment(self, *, sequence_id: str, frame_id: str, timestamp_us: int, gt: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
        parsed = {box.track_id: box for box in map(EvaluationBox.from_dict, gt)}
        previous_time, previous = self._previous.get(sequence_id, (timestamp_us, {}))
        dt = (timestamp_us - previous_time) / 1e6
        if dt < 0:
            raise ValueError("continuity timestamps must increase")
        threshold = max(self.minimum_jump_m, self.maximum_speed_mps * dt + self.tolerance_m)
        for identity in parsed.keys() & previous.keys():
            distance = float(np.linalg.norm(parsed[identity].world_T_object[:3, 3] - previous[identity].world_T_object[:3, 3]))
            if distance > threshold:
                self._segments[(sequence_id, identity)] += 1
                self.discontinuities.append(dict(sequence_id=sequence_id, frame_id=frame_id, timestamp_us=timestamp_us,
                    gt_id=identity, displacement_m=distance, dt_seconds=dt, threshold_m=threshold,
                    segment=int(self._segments[(sequence_id, identity)])))
        self._previous[sequence_id] = (timestamp_us, parsed)
        return [dict(box, track_id=f"{box['track_id']}#segment:{self._segments[(sequence_id, str(box['track_id']))]}") for box in gt]

    def provenance(self) -> dict[str, Any]:
        return dict(policy="split only same ID present in consecutive evaluated frames", minimum_jump_m=self.minimum_jump_m,
                    maximum_speed_mps=self.maximum_speed_mps, tolerance_m=self.tolerance_m,
                    threshold="max(minimum_jump_m, maximum_speed_mps * dt + tolerance_m)",
                    discontinuities=self.discontinuities, canonical_gt_modified=False)


class FrontCameraCoverageEvaluator:
    """Geometric image visibility and per-annotation identity coverage.

    Visibility means projected cuboid intersection, not actual occlusion or
    annotation completeness. Filtering is symmetric for prediction and GT.
    """

    def __init__(self, *, center_threshold_m: float = 2.0, minimum_area_px: float = 16.0, precision_target: float | None = None) -> None:
        if precision_target is not None and not 0 < precision_target <= 1:
            raise ValueError("precision_target must be in (0, 1]")
        self.minimum_area_px = minimum_area_px
        self.precision_target = precision_target
        self.detection = GeometricTrackingEvaluator(center_threshold_m=center_threshold_m, identity_metrics=False)
        self.tracking = GeometricTrackingEvaluator(center_threshold_m=center_threshold_m)
        self._coverage: dict[tuple[str, str], dict[str, Any]] = {}

    def update(self, frame: Mapping[str, Any]) -> dict[str, Any]:
        from src.reconstruction.models import VehicleDimensions
        from src.reconstruction.projection import project_vehicle_box

        camera = np.asarray(frame["world_T_camera"], dtype=float).reshape(4, 4)
        intrinsic = np.asarray(frame["camera_intrinsic"], dtype=float).reshape(3, 3)
        def visible(boxes):
            return [box for box in boxes if project_vehicle_box(
                world_T_object=np.asarray(box["world_T_object"], dtype=float).reshape(4, 4),
                world_T_camera=camera, intrinsics=intrinsic, dimensions=VehicleDimensions(*box["dimensions_lwh"]),
                image_size=(int(frame["image_height"]), int(frame["image_width"])),
                minimum_area_px=self.minimum_area_px) is not None]
        gt, detections, tracks = (visible(frame[key]) for key in ("gt", "detections", "tracks"))
        common = dict(sequence_id=frame["sequence_id"], timestamp_us=frame["timestamp_us"], gt=gt)
        detection = self.detection.update(predictions=detections, **common)
        tracking = self.tracking.update(predictions=tracks, **common)
        detected = {match["gt_id"] for match in detection["matches"]}
        traced = {match["gt_id"] for match in tracking["matches"]}
        switched = Counter(match["gt_id"] for match in tracking["id_switches"])
        current = {(frame["sequence_id"], str(box["track_id"])) for box in gt}
        for key, coverage in self._coverage.items():
            if key not in current and key[0] == frame["sequence_id"]:
                coverage["_miss_run"] = 0
        for box in gt:
            identity = str(box["track_id"])
            key = (frame["sequence_id"], identity)
            coverage = self._coverage.setdefault(key, dict(sequence_id=key[0], gt_id=identity,
                visible_frames=0, detected_frames=0, traced_frames=0, longest_track_miss_frames=0,
                id_switches=0, _miss_run=0))
            coverage["visible_frames"] += 1
            coverage["detected_frames"] += int(identity in detected)
            coverage["traced_frames"] += int(identity in traced)
            coverage["id_switches"] += switched[identity]
            coverage["_miss_run"] = 0 if identity in traced else coverage["_miss_run"] + 1
            coverage["longest_track_miss_frames"] = max(coverage["longest_track_miss_frames"], coverage["_miss_run"])
        return dict(detections=detection, tracks=tracking, visible_gt_ids=[box["track_id"] for box in gt],
                    visible_gt=len(gt), visible_detections=len(detections), visible_tracks=len(tracks))

    def summary(self) -> dict[str, Any]:
        coverage = []
        for record in self._coverage.values():
            item = {key: value for key, value in record.items() if not key.startswith("_")}
            item.update(detection_recall=item["detected_frames"] / item["visible_frames"],
                        track_recall=item["traced_frames"] / item["visible_frames"],
                        ever_detected=item["detected_frames"] > 0, eligible=item["visible_frames"] >= 5)
            coverage.append(item)
        eligible = [item for item in coverage if item["eligible"]]
        observations = sum(item["visible_frames"] for item in coverage)
        switches = sum(item["id_switches"] for item in coverage)
        detection_summary, tracking_summary = self.detection.summary(), self.tracking.summary()
        precision_pass = None if self.precision_target is None else all(
            result["precision"] is not None and result["precision"] >= self.precision_target
            for result in (detection_summary, tracking_summary))
        return dict(visibility_policy=dict(type="geometric projected cuboid intersection", minimum_area_px=self.minimum_area_px,
                    near_plane_policy="reject entire box if any corner has camera depth <= 0.1 m; no near-plane clipping",
                    actual_occlusion_tested=False, symmetric_prediction_filter=True), detections=detection_summary,
                    tracks=tracking_summary, per_gt_id=coverage,
                    acceptance=dict(minimum_visible_frames=5, minimum_per_id_detection_recall=.9, minimum_per_id_track_recall=.9,
                        eligible_gt_ids=len(eligible), never_detected_gt_ids=[item["gt_id"] for item in coverage if not item["ever_detected"]],
                        per_id_recall_pass=bool(eligible) and all(item["detection_recall"] >= .9 and item["track_recall"] >= .9 for item in eligible),
                        zero_never_detected_pass=bool(coverage) and all(item["ever_detected"] for item in coverage),
                        id_switches_per_100_visible_observations=100 * switches / observations if observations else None,
                        id_switch_rate_pass=bool(observations) and 100 * switches / observations <= 1,
                        optional_detection_and_track_precision_target=self.precision_target,
                        optional_precision_target_pass=precision_pass,
                        zero_id_switches=switches == 0))

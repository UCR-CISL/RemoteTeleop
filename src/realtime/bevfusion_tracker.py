"""Lightweight 3D tracking for BEVFusion detections without native track IDs."""

from __future__ import annotations

from dataclasses import dataclass, replace
import copy
import math
from typing import Sequence

import numpy as np
from scipy.optimize import linear_sum_assignment


@dataclass(frozen=True)
class WorldDetection:
    """One detector box expressed in the playback world frame."""

    world_T_object: np.ndarray
    dimensions_lwh: tuple[float, float, float]
    label: str
    score: float
    detector_class_name: str | None = None


@dataclass(frozen=True)
class TrackedWorldBox:
    """A current or briefly coasted estimate with a persistent identity."""

    track_id: str
    world_T_object: np.ndarray
    dimensions_lwh: tuple[float, float, float]
    label: str
    score: float
    detector_class_name: str | None = None


class _KalmanTrack:
    """Constant-velocity center filter plus smoothed box shape and rotation."""

    _measurement = np.column_stack((np.eye(3), np.zeros((3, 3))))
    # The playback map may permute the physical ground and vertical axes, so
    # all filter covariance must remain independent of a presumed world Z-up.
    _measurement_noise = np.eye(3, dtype=np.float64)

    def __init__(self, track_id: str, detection: WorldDetection, timestamp_us: int,
                 minimum_confirmation_hits: int) -> None:
        center = np.asarray(detection.world_T_object, dtype=np.float64)[:3, 3]
        self.track_id = track_id
        self.label = detection.label
        self.detector_class_name = detection.detector_class_name
        self.state = np.concatenate((center, np.zeros(3, dtype=np.float64)))
        self.covariance = np.diag((4.0, 4.0, 4.0, 100.0, 100.0, 100.0))
        self.dimensions = np.asarray(detection.dimensions_lwh, dtype=np.float64)
        self.rotation = np.asarray(
            detection.world_T_object, dtype=np.float64
        )[:3, :3].copy()
        self.score = detection.score
        self.timestamp_us = timestamp_us
        self.last_observed_us = timestamp_us
        self.minimum_confirmation_hits = minimum_confirmation_hits
        self.hit_streak = 1
        self.confirmed = minimum_confirmation_hits == 1

    def predict_to(self, timestamp_us: int) -> None:
        if timestamp_us < self.timestamp_us:
            raise ValueError("tracker timestamps must be monotonic")
        dt = (timestamp_us - self.timestamp_us) / 1_000_000.0
        transition = np.eye(6, dtype=np.float64)
        transition[:3, 3:] = np.eye(3) * dt
        process_noise = np.diag(
            (0.5 * dt * dt,) * 3 + (4.0 * max(dt, 1e-3),) * 3
        )
        self.state = transition @ self.state
        self.covariance = transition @ self.covariance @ transition.T + process_noise
        self.timestamp_us = timestamp_us

    def association_cost(self, detection: WorldDetection, timestamp_us: int) -> float:
        center = np.asarray(detection.world_T_object, dtype=np.float64)[:3, 3]
        innovation = center - self.state[:3]
        innovation_covariance = (
            self._measurement @ self.covariance @ self._measurement.T
            + self._measurement_noise
        )
        mahalanobis = float(innovation @ np.linalg.solve(innovation_covariance, innovation))
        observation_gap = (timestamp_us - self.last_observed_us) / 1_000_000.0
        distance_gate = min(8.0, 3.0 + 10.0 * observation_gap)
        if mahalanobis > 11.345 or np.linalg.norm(innovation) > distance_gate:
            return math.inf
        dimensions = np.asarray(detection.dimensions_lwh, dtype=np.float64)
        dimension_cost = float(np.abs(np.log(dimensions / self.dimensions)).sum())
        rotation = np.asarray(detection.world_T_object, dtype=np.float64)[:3, :3]
        heading_similarity = float(
            np.clip(abs(rotation[:, 0] @ self.rotation[:, 0]), 0.0, 1.0)
        )
        heading_cost = math.acos(heading_similarity)
        return mahalanobis + dimension_cost + 0.5 * heading_cost

    def correct(self, detection: WorldDetection, timestamp_us: int) -> None:
        center = np.asarray(detection.world_T_object, dtype=np.float64)[:3, 3]
        innovation = center - self._measurement @ self.state
        innovation_covariance = (
            self._measurement @ self.covariance @ self._measurement.T
            + self._measurement_noise
        )
        gain = (
            self.covariance
            @ self._measurement.T
            @ np.linalg.inv(innovation_covariance)
        )
        self.state += gain @ innovation
        identity = np.eye(6, dtype=np.float64)
        # Joseph form preserves a positive semi-definite covariance under
        # floating-point roundoff.
        residual = identity - gain @ self._measurement
        self.covariance = (
            residual @ self.covariance @ residual.T
            + gain @ self._measurement_noise @ gain.T
        )
        self.dimensions = 0.8 * self.dimensions + 0.2 * np.asarray(
            detection.dimensions_lwh, dtype=np.float64
        )
        rotation = np.asarray(detection.world_T_object, dtype=np.float64)[:3, :3]
        # Cuboid headings are equivalent modulo 180 degrees.  Align the
        # measured x/y axes before averaging, then project the blend back onto
        # SO(3) so arbitrary map-frame rotations remain intact.
        if rotation[:, 0] @ self.rotation[:, 0] < 0:
            rotation = rotation.copy()
            rotation[:, :2] *= -1
        left, _, right = np.linalg.svd(0.8 * self.rotation + 0.2 * rotation)
        self.rotation = left @ right
        if np.linalg.det(self.rotation) < 0:
            left[:, -1] *= -1
            self.rotation = left @ right
        self.score = detection.score
        if timestamp_us > self.last_observed_us:
            self.hit_streak += 1
        self.last_observed_us = timestamp_us
        if detection.detector_class_name is not None:
            self.detector_class_name = detection.detector_class_name
        self.confirmed |= self.hit_streak >= self.minimum_confirmation_hits

    def snapshot(self, timestamp_us: int) -> TrackedWorldBox:
        dt = max(0.0, (timestamp_us - self.timestamp_us) / 1_000_000.0)
        center = self.state[:3] + self.state[3:] * dt
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = self.rotation
        transform[:3, 3] = center
        return TrackedWorldBox(
            track_id=self.track_id,
            world_T_object=transform,
            dimensions_lwh=tuple(float(value) for value in self.dimensions),
            label=self.label,
            score=self.score,
            detector_class_name=self.detector_class_name,
        )


class _PersistentIdentityRegistry:
    """Retired identity candidates, separate from live filter state."""

    def __init__(self, maximum_age_us: int) -> None:
        self.maximum_age_us = maximum_age_us
        self.identities: dict[str, str] = {}
        self.retired: dict[str, _KalmanTrack] = {}

    def update(self, previous: Sequence[_KalmanTrack], current: Sequence[_KalmanTrack],
               timestamp_us: int) -> None:
        previous_ids = {track.track_id for track in previous}
        current_ids = {track.track_id for track in current}
        for track in previous:
            if track.track_id not in current_ids and track.confirmed:
                self.retired[self.identities[track.track_id]] = copy.deepcopy(track)
        self.retired = {identity: track for identity, track in self.retired.items()
                        if timestamp_us - track.last_observed_us <= self.maximum_age_us}
        active = {self.identities[track.track_id] for track in current
                  if track.track_id in self.identities}
        births = [track for track in current if track.track_id not in previous_ids]
        pairs = []
        for track in births:
            pose = np.eye(4)
            pose[:3, :3], pose[:3, 3] = track.rotation, track.state[:3]
            detection = WorldDetection(pose, tuple(track.dimensions), track.label, track.score)
            for identity, retired in self.retired.items():
                if identity in active or retired.label != track.label:
                    continue
                predicted = copy.deepcopy(retired)
                predicted.predict_to(timestamp_us)
                if not math.isfinite(predicted.association_cost(detection, timestamp_us)):
                    continue
                if np.linalg.norm(predicted.state[:3] - track.state[:3]) > 3.0:
                    continue
                if abs(predicted.rotation[:, 0] @ track.rotation[:, 0]) < math.cos(math.pi / 6):
                    continue
                ratio = track.dimensions / predicted.dimensions
                if np.any(ratio < 0.8) or np.any(ratio > 1.25):
                    continue
                pairs.append((track.track_id, identity))
        for track in births:
            candidates = [identity for internal, identity in pairs if internal == track.track_id]
            identity = track.track_id
            if len(candidates) == 1 and sum(old == candidates[0] for _, old in pairs) == 1:
                identity = candidates[0]
                del self.retired[identity]
            self.identities[track.track_id] = identity
        self.identities = {internal: identity for internal, identity in self.identities.items()
                           if internal in current_ids}

    def remap(self, box: TrackedWorldBox) -> TrackedWorldBox:
        return replace(box, track_id=self.identities[box.track_id])


class BevFusionKalmanTracker:
    """Class-aware Hungarian assignment over world-frame Kalman estimates."""

    def __init__(self, *, maximum_coast_seconds: float = 0.5,
                 minimum_confirmation_hits: int = 1,
                 birth_score_threshold: float = 0.0,
                 continuation_score_threshold: float = 0.0,
                 suppress_duplicate_births: bool = False,
                 maximum_reacquisition_seconds: float = 0.0) -> None:
        if not math.isfinite(maximum_reacquisition_seconds) or maximum_reacquisition_seconds < 0:
            raise ValueError("maximum_reacquisition_seconds must be finite and non-negative")
        self._identity_registry = (_PersistentIdentityRegistry(int(maximum_reacquisition_seconds * 1_000_000))
                                   if maximum_reacquisition_seconds > 0 else None)
        if maximum_coast_seconds <= 0:
            raise ValueError("maximum_coast_seconds must be positive")
        if (isinstance(minimum_confirmation_hits, bool)
                or not isinstance(minimum_confirmation_hits, int)
                or minimum_confirmation_hits < 1):
            raise ValueError("minimum_confirmation_hits must be a positive integer")
        if (not math.isfinite(birth_score_threshold)
                or not math.isfinite(continuation_score_threshold)
                or not 0 <= continuation_score_threshold <= birth_score_threshold <= 1):
            raise ValueError("score thresholds must satisfy 0 <= continuation <= birth <= 1")
        self.maximum_coast_us = int(maximum_coast_seconds * 1_000_000)
        self.minimum_confirmation_hits = minimum_confirmation_hits
        self.birth_score_threshold = float(birth_score_threshold)
        self.continuation_score_threshold = float(continuation_score_threshold)
        self.suppress_duplicate_births = suppress_duplicate_births
        self._tracks: list[_KalmanTrack] = []
        self._next_track_id = 0
        self._timestamp_us: int | None = None
        self._suppressed_birth_pairs: tuple[tuple[int, int], ...] = ()

    @property
    def suppressed_birth_pairs(self) -> tuple[tuple[int, int], ...]:
        """Last update's (unmatched input index, matched reference index) pairs."""
        return self._suppressed_birth_pairs

    def update(
        self, detections: Sequence[WorldDetection], *, timestamp_us: int
    ) -> None:
        if self._timestamp_us is not None and timestamp_us < self._timestamp_us:
            raise ValueError("tracker timestamps must be monotonic")
        previous = tuple(self._tracks) if self._identity_registry is not None else ()
        self._timestamp_us = timestamp_us
        self._suppressed_birth_pairs = ()
        # Expiry is decided before association: a new detection must not revive
        # an expired filter whose observation gap exceeded the coast limit.
        self._tracks = [
            track
            for track in self._tracks
            if timestamp_us - track.last_observed_us <= self.maximum_coast_us
        ]
        for track in self._tracks:
            track.predict_to(timestamp_us)

        high = [i for i, detection in enumerate(detections)
                if detection.score >= self.birth_score_threshold]
        low = [i for i, detection in enumerate(detections)
               if self.continuation_score_threshold <= detection.score < self.birth_score_threshold]
        assignments = self._associate(range(len(self._tracks)), high, detections, timestamp_us)
        # Protect only current high detections associated to identities that
        # were already confirmed. Coasting ghosts, tentative confirmations and
        # new births must not prevent another object from starting a track.
        birth_references = [column for row, column in assignments
                            if self._tracks[row].confirmed]
        high_assigned_tracks = {row for row, _ in assignments}
        continuing = [i for i, track in enumerate(self._tracks)
                      if track.confirmed and i not in high_assigned_tracks]
        assignments += self._associate(continuing, low, detections, timestamp_us)
        assigned_detections = {column for _, column in assignments}
        for track_index, detection_index in assignments:
            self._tracks[track_index].correct(detections[detection_index], timestamp_us)
        assigned_tracks = {row for row, _ in assignments}
        for track_index, track in enumerate(self._tracks):
            if not track.confirmed and track_index not in assigned_tracks:
                track.hit_streak = 0

        suppressed = []
        for detection_index in high:
            if detection_index in assigned_detections:
                continue
            detection = detections[detection_index]
            if self.suppress_duplicate_births:
                reference = next((column for column in birth_references
                                  if self._duplicate_birth(detection, detections[column])), None)
                if reference is not None:
                    suppressed.append((detection_index, reference))
                    continue
            label = detection.label.replace(".", "_").replace("/", "_")
            track_id = f"bevfusion:{label}:{self._next_track_id}"
            self._next_track_id += 1
            self._tracks.append(_KalmanTrack(
                track_id, detection, timestamp_us, self.minimum_confirmation_hits,
            ))
        self._suppressed_birth_pairs = tuple(suppressed)

        self._tracks = [
            track
            for track in self._tracks
            if timestamp_us - track.last_observed_us <= self.maximum_coast_us
        ]
        if self._identity_registry is not None:
            self._identity_registry.update(previous, self._tracks, timestamp_us)

    @staticmethod
    def _duplicate_birth(candidate: WorldDetection, reference: WorldDetection) -> bool:
        """Frozen exploratory birth guard; never removes a detector box."""
        if candidate.label != reference.label:
            return False
        candidate_pose = np.asarray(candidate.world_T_object, dtype=float)
        reference_pose = np.asarray(reference.world_T_object, dtype=float)
        if abs(candidate_pose[:3, 0] @ reference_pose[:3, 0]) < math.cos(math.radians(30)):
            return False
        overlap = BevFusionKalmanTracker._parallel_box_iou(candidate, reference)
        return overlap is not None and overlap >= 0.3

    @staticmethod
    def _parallel_box_iou(candidate: WorldDetection, reference: WorldDetection) -> float | None:
        """Exact cuboid IoU for parallel vertical axes, in reference box axes.

        Current detector boxes share a LiDAR vertical axis. Arbitrary world
        rotations preserve this test; nonparallel normals are skipped rather
        than approximating their 3D intersection.
        """
        pose = np.asarray(candidate.world_T_object, dtype=float)
        other = np.asarray(reference.world_T_object, dtype=float)
        if np.linalg.norm(np.cross(pose[:3, 2], other[:3, 2])) > 1e-6:
            return None
        rotation = other[:3, :3].T @ pose[:3, :3]
        center = other[:3, :3].T @ (pose[:3, 3] - other[:3, 3])
        length, width, height = candidate.dimensions_lwh
        ref_length, ref_width, ref_height = reference.dimensions_lwh
        vertical = max(0.0, min(center[2] + height / 2, ref_height / 2)
                       - max(center[2] - height / 2, -ref_height / 2))
        if vertical == 0:
            return 0.0
        corners = np.array(((-1, -1), (1, -1), (1, 1), (-1, 1)))
        polygon = (corners * (length / 2, width / 2)) @ rotation[:2, :2].T + center[:2]
        # Clip the candidate footprint against the reference rectangle.
        for axis, bound, direction in ((0, ref_length / 2, 1), (0, -ref_length / 2, -1),
                                       (1, ref_width / 2, 1), (1, -ref_width / 2, -1)):
            if len(polygon) == 0:
                return 0.0
            clipped = []
            previous = polygon[-1]
            previous_inside = direction * (previous[axis] - bound) <= 0
            for point in polygon:
                inside = direction * (point[axis] - bound) <= 0
                if inside != previous_inside:
                    fraction = (bound - previous[axis]) / (point[axis] - previous[axis])
                    clipped.append(previous + fraction * (point - previous))
                if inside:
                    clipped.append(point)
                previous, previous_inside = point, inside
            polygon = np.asarray(clipped)
        if len(polygon) < 3:
            return 0.0
        area = abs(float(polygon[:, 0] @ np.roll(polygon[:, 1], 1)
                         - polygon[:, 1] @ np.roll(polygon[:, 0], 1))) / 2
        intersection = area * vertical
        union = length * width * height + ref_length * ref_width * ref_height - intersection
        return min(1.0, max(0.0, intersection / union))

    def _associate(self, track_indices: Sequence[int], detection_indices: Sequence[int],
                   detections: Sequence[WorldDetection], timestamp_us: int) -> list[tuple[int, int]]:
        """Use the same class and geometric gates in both confidence stages."""
        costs = np.full((len(track_indices), len(detection_indices)), math.inf)
        for row, track_index in enumerate(track_indices):
            track = self._tracks[track_index]
            for column, detection_index in enumerate(detection_indices):
                detection = detections[detection_index]
                if detection.label == track.label:
                    costs[row, column] = track.association_cost(detection, timestamp_us)
        if not costs.size:
            return []
        rows, columns = linear_sum_assignment(np.where(np.isfinite(costs), costs, 1e9))
        return [(track_indices[row], detection_indices[column])
                for row, column in zip(rows, columns, strict=True)
                if math.isfinite(costs[row, column])]

    def boxes_at(self, timestamp_us: int) -> tuple[TrackedWorldBox, ...]:
        if self._timestamp_us is None:
            return ()
        if timestamp_us < self._timestamp_us:
            raise ValueError("cannot project tracker state into the past")
        return tuple(
            (self._identity_registry.remap(track.snapshot(timestamp_us))
             if self._identity_registry is not None else track.snapshot(timestamp_us))
            for track in self._tracks
            if track.confirmed and timestamp_us - track.last_observed_us <= self.maximum_coast_us
        )

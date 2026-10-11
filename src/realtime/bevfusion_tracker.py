"""Lightweight 3D tracking for BEVFusion detections without native track IDs."""

from __future__ import annotations

from dataclasses import dataclass
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


class BevFusionKalmanTracker:
    """Class-aware Hungarian assignment over world-frame Kalman estimates."""

    def __init__(self, *, maximum_coast_seconds: float = 0.5,
                 minimum_confirmation_hits: int = 1,
                 birth_score_threshold: float = 0.0,
                 continuation_score_threshold: float = 0.0) -> None:
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
        self._tracks: list[_KalmanTrack] = []
        self._next_track_id = 0
        self._timestamp_us: int | None = None

    def update(
        self, detections: Sequence[WorldDetection], *, timestamp_us: int
    ) -> None:
        if self._timestamp_us is not None and timestamp_us < self._timestamp_us:
            raise ValueError("tracker timestamps must be monotonic")
        self._timestamp_us = timestamp_us
        # Expiry is decided before association: a new detection must not revive
        # an identity whose observation gap already exceeded the coast limit.
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

        for detection_index in high:
            if detection_index in assigned_detections:
                continue
            detection = detections[detection_index]
            label = detection.label.replace(".", "_").replace("/", "_")
            track_id = f"bevfusion:{label}:{self._next_track_id}"
            self._next_track_id += 1
            self._tracks.append(_KalmanTrack(
                track_id, detection, timestamp_us, self.minimum_confirmation_hits,
            ))

        self._tracks = [
            track
            for track in self._tracks
            if timestamp_us - track.last_observed_us <= self.maximum_coast_us
        ]

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
            track.snapshot(timestamp_us)
            for track in self._tracks
            if track.confirmed and timestamp_us - track.last_observed_us <= self.maximum_coast_us
        )

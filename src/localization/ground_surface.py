"""Bounded, map-derived road support for rendering without plane extrapolation."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree

from src.localization.ground_plane import GroundPlane


@dataclass
class GroundSurface:
    """Sparse road samples in native map axes, with explicit unsupported fallback.

    Samples are road candidates connected to a map-ground seed by bounded grade.
    Placement uses local support only. Unsupported boxes retain their incoming
    pose; their detector/localization height is not treated as map ground truth.
    """

    origin: np.ndarray
    basis: np.ndarray
    samples: np.ndarray
    max_distance_m: float = 4.0
    neighborhood_m: float = 6.0
    last_placement: dict = field(default_factory=dict, init=False)

    def __post_init__(self):
        self.origin = np.asarray(self.origin, dtype=float)
        self.basis = np.asarray(self.basis, dtype=float)
        self.samples = np.asarray(self.samples, dtype=float).reshape(-1, 3)
        if self.origin.shape != (3,) or self.basis.shape != (3, 3) or not all(np.isfinite(x).all() for x in (self.origin, self.basis, self.samples)):
            raise ValueError("ground surface requires finite origin, basis and Nx3 samples")
        if not np.allclose(self.basis.T @ self.basis, np.eye(3), atol=1e-6) or np.linalg.det(self.basis) < .999:
            raise ValueError("ground surface basis must be right-handed orthonormal")
        if not 0 < self.max_distance_m <= self.neighborhood_m:
            raise ValueError("invalid ground support radii")
        self._tree = cKDTree(self.samples[:, :2])

    @classmethod
    def from_dict(cls, value):
        return cls(value["origin"], value["basis"], value["samples"],
                   value.get("max_distance_m", 4.), value.get("neighborhood_m", 6.))

    def to_dict(self):
        return dict(type="road_surface", origin=self.origin.tolist(), basis=self.basis.tolist(),
                    samples=self.samples.tolist(), max_distance_m=self.max_distance_m,
                    neighborhood_m=self.neighborhood_m,
                    unsupported_policy="preserve_input_pose")

    def place_box(self, world_T_object, dimensions_lwh):
        pose = np.asarray(world_T_object, dtype=float).reshape(4, 4).copy()
        query = (pose[:3, 3] - self.origin) @ self.basis
        distance, _ = self._tree.query(query[:2])
        indices = self._tree.query_ball_point(query[:2], self.neighborhood_m)
        self.last_placement = dict(status="unsupported_preserve_input_pose",
                                   nearest_support_m=float(distance) if np.isfinite(distance) else None,
                                   support_count=len(indices), height_shift_m=0.)
        if distance > self.max_distance_m or len(indices) < 3:
            return pose
        neighbors = self.samples[indices]
        # A few surviving floaters must not tilt an otherwise coherent road.
        centered = neighbors - np.array([query[0], query[1], 0.])
        try:
            fitted, quality = GroundPlane.fit(centered, up_hint=[0, 0, 1],
                                             distance_threshold_m=.12, min_inlier_ratio=.5,
                                             min_inliers=5, max_tilt_deg=11.3, iterations=96)
        except ValueError:
            self.last_placement["reason"] = "local_support_failed_grade_or_residual"
            return pose
        local_normal = np.asarray(fitted.normal)
        height = -fitted.offset_m / local_normal[2]
        normal = self.basis @ local_normal
        road_point = self.origin + self.basis @ np.array([query[0], query[1], height])
        plane = GroundPlane(tuple(normal), -float(normal @ road_point))
        result = plane.place_box(pose, dimensions_lwh)
        self.last_placement.update(status="local_map_road", road_height_m=float(height),
                                   median_residual_m=quality["median_residual_m"],
                                   inlier_count=quality["inlier_count"], inlier_ratio=quality["inlier_ratio"],
                                   height_shift_m=float((result[:3, 3] - pose[:3, 3]) @ self.basis[:, 2]))
        return result

    @classmethod
    def from_map_candidates(cls, points, *, origin, basis, seed_height_m):
        """Select locally dense low surface modes and seed-connected road cells.

        Input points must already be thin, upward-facing Gaussian surfels.
        Heights and connectivity come solely from map geometry, not object boxes.
        """
        basis = np.asarray(basis, dtype=float)
        origin = np.asarray(origin, dtype=float)
        local = (np.asarray(points, dtype=float) - origin) @ basis
        local = local[np.isfinite(local).all(axis=1) & (local[:, 2] > seed_height_m - 2.5) & (local[:, 2] < seed_height_m + 1.)]
        cells = np.floor(local[:, :2] / 2.).astype(int)
        _, inverse = np.unique(cells, axis=0, return_inverse=True)
        order = np.argsort(inverse)
        groups = np.split(order, np.flatnonzero(np.diff(inverse[order])) + 1)
        samples = []
        for group in groups:
            cluster = local[group]
            if len(cluster) < 3:
                continue
            heights = np.sort(cluster[:, 2])
            end = np.searchsorted(heights, heights + .20, side="right")
            counts = end - np.arange(len(heights))
            strongest = counts.max()
            if strongest < 3:
                continue
            # Lowest maximally supported mode avoids bias toward sparse low floaters.
            start = np.flatnonzero(counts == strongest)[0]
            mode = cluster[(cluster[:, 2] >= heights[start]) & (cluster[:, 2] <= heights[start] + .20)]
            samples.append(np.median(mode, axis=0))
        samples = np.asarray(samples, dtype=float).reshape(-1, 3)
        tree = cKDTree(samples[:, :2])
        seeds = np.flatnonzero((np.linalg.norm(samples[:, :2], axis=1) < 12) & (np.abs(samples[:, 2] - seed_height_m) < .4))
        visited = set(map(int, seeds)); pending = deque(visited)
        while pending:
            index = pending.popleft()
            for other in tree.query_ball_point(samples[index, :2], 6.):
                distance = np.linalg.norm(samples[index, :2] - samples[other, :2])
                if other not in visited and abs(samples[index, 2] - samples[other, 2]) <= .15 + .12 * distance:
                    visited.add(other); pending.append(other)
        retained = samples[sorted(visited)]
        return cls(origin, basis, retained), dict(candidate_points=len(local), candidate_cells=len(samples),
                                               seed_cells=len(seeds), connected_road_cells=len(retained),
                                               grid_m=2., mode_width_m=.2, connection_radius_m=6.,
                                               max_connection_grade=.12, seed_height_m=float(seed_height_m))


def load_ground_placement(value):
    """Load an explicit plane or a bounded map-derived road surface."""
    if value.get("type", "plane") == "road_surface":
        return GroundSurface.from_dict(value)
    if value.get("type", "plane") != "plane":
        raise ValueError("unknown ground placement type")
    return GroundPlane.from_dict(value)

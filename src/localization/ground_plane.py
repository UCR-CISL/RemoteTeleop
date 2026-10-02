"""Explicit map-frame road planes for rendering, independent of localization."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import numpy as np


@dataclass(frozen=True)
class GroundPlane:
    """Upward-facing plane ``normal @ map_point + offset_m == 0``.

    The normal is expressed in the actual map axes, which need not be Z-up.
    Ground placement changes rendering only; it must not update estimated poses.
    """

    normal: tuple[float, float, float]
    offset_m: float
    align_up: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.align_up, bool):
            raise ValueError("align_up must be a boolean")
        normal = np.asarray(self.normal, dtype=np.float64)
        if normal.shape != (3,) or not np.isfinite(normal).all():
            raise ValueError("ground-plane normal must be a finite 3-vector")
        norm = np.linalg.norm(normal)
        if norm < 1e-12 or not np.isfinite(self.offset_m):
            raise ValueError("ground-plane normal must be nonzero and offset finite")
        object.__setattr__(self, "normal", tuple(normal / norm))
        object.__setattr__(self, "offset_m", float(self.offset_m) / norm)

    @classmethod
    def from_dict(cls, value: Mapping) -> "GroundPlane":
        return cls(tuple(value["normal"]), float(value["offset_m"]), value.get("align_up", True))

    def to_dict(self) -> dict:
        return {"normal": list(self.normal), "offset_m": self.offset_m, "align_up": self.align_up}

    def place_box(self, world_T_object: np.ndarray, dimensions_lwh) -> np.ndarray:
        """Preserve projected heading and put the bbox bottom on the plane.

        Assets use centered metric bounding boxes with local X forward, Y left,
        and Z up. Translation changes only along the map-frame plane normal.
        With ``align_up=False``, retain the entire input orientation and anchor
        its lowest oriented corner, matching the legacy flat-ground renderer.
        """
        pose = np.asarray(world_T_object, dtype=np.float64).reshape(4, 4).copy()
        dimensions = np.asarray(dimensions_lwh, dtype=np.float64)
        if not np.isfinite(pose).all() or dimensions.shape != (3,) or not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
            raise ValueError("box pose and positive dimensions must be finite")
        normal = np.asarray(self.normal)
        if not self.align_up:
            lowest_corner = np.dot(normal, pose[:3, 3]) + self.offset_m - np.sum(
                np.abs(normal @ pose[:3, :3]) * dimensions / 2)
            pose[:3, 3] -= normal * lowest_corner
            return pose
        forward = pose[:3, 0] - normal * np.dot(normal, pose[:3, 0])
        length = np.linalg.norm(forward)
        if length < 1e-8:
            raise ValueError("vehicle heading is perpendicular to ground plane")
        forward /= length
        pose[:3, :3] = np.column_stack((forward, np.cross(normal, forward), normal))
        signed_height = np.dot(normal, pose[:3, 3]) + self.offset_m
        pose[:3, 3] += normal * (dimensions[2] / 2 - signed_height)
        return pose

    @classmethod
    def fit(
        cls, points: np.ndarray, *, up_hint, max_tilt_deg: float = 15.0,
        distance_threshold_m: float = 0.10, min_inlier_ratio: float = 0.35,
        min_inliers: int = 100, iterations: int = 256,
    ) -> tuple["GroundPlane", dict]:
        """Fit preselected road candidates; never infer ground from all map points.

        Callers must select a plausible road height/spatial region first. An
        upward-facing roof can otherwise be as planar as the road. Diagnostics
        describe fit quality, not semantic correctness or independent accuracy.
        """
        points = np.asarray(points, dtype=np.float64)
        up = np.array(up_hint, dtype=np.float64, copy=True)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError("ground candidates must be finite Nx3 points")
        if up.shape != (3,) or not np.isfinite(up).all() or np.linalg.norm(up) < 1e-12:
            raise ValueError("up_hint must be a finite nonzero map-frame vector")
        if not (0 <= max_tilt_deg < 90 and distance_threshold_m > 0 and 0 < min_inlier_ratio <= 1 and min_inliers >= 3 and iterations > 0):
            raise ValueError("invalid ground fit settings")
        if len(points) < min_inliers:
            raise ValueError("insufficient ground candidates")
        up /= np.linalg.norm(up)
        cosine = np.cos(np.deg2rad(max_tilt_deg))
        rng = np.random.default_rng(0)
        best = None
        best_count = 0
        for _ in range(iterations):
            a, b, c = points[rng.choice(len(points), 3, replace=False)]
            normal = np.cross(b - a, c - a)
            norm = np.linalg.norm(normal)
            if norm < 1e-12:
                continue
            normal /= norm
            if normal @ up < 0:
                normal *= -1
            if normal @ up < cosine:
                continue
            mask = np.abs(points @ normal - a @ normal) <= distance_threshold_m
            count = int(mask.sum())
            if count > best_count:
                best, best_count = mask, count
        if best is None or best_count < min_inliers or best_count / len(points) < min_inlier_ratio:
            raise ValueError("ground-plane fit has insufficient support")
        for _ in range(3):
            selected = points[best]
            center = selected.mean(axis=0)
            _, singular, vectors = np.linalg.svd(selected - center, full_matrices=False)
            if singular[1] < 1e-6:
                raise ValueError("ground-plane support is collinear")
            normal = vectors[-1]
            if normal @ up < 0:
                normal *= -1
            offset = -float(normal @ center)
            residuals = np.abs(points @ normal + offset)
            best = residuals <= distance_threshold_m
            if best.sum() < min_inliers or best.mean() < min_inlier_ratio or normal @ up < cosine:
                raise ValueError("refined ground plane failed support/tilt gates")
        plane = cls(tuple(normal), offset)
        return plane, dict(candidate_count=len(points), inlier_count=int(best.sum()),
                           inlier_ratio=float(best.mean()),
                           median_residual_m=float(np.median(residuals[best])),
                           p95_residual_m=float(np.percentile(residuals[best], 95)),
                           tilt_deg=float(np.rad2deg(np.arccos(np.clip(normal @ up, -1, 1)))))

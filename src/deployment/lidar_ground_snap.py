"""Snap detected boxes onto the local LiDAR ground (vehicle side, LiDAR frame)."""

from __future__ import annotations

import numpy as np


class LidarGroundSnapper:
    """Estimate ground height in a ring around each box footprint and return z corrections.

    Ground is the median of ring points within ``ground_band_m`` of the ring's
    ``ground_percentile``-th height.  Boxes keep their height (zero correction) when the
    ring has fewer than ``minimum_points`` points or the correction exceeds
    ``maximum_correction_m``.  Points are bucketed once per scan into a coarse BEV grid and
    dense crops near the ego are strided down to ``maximum_crop_points``.
    """

    def __init__(self, *, ring_inner_m: float = 0.3, ring_outer_m: float = 1.5, ground_percentile: float = 10.0,
                 ground_band_m: float = 0.08, minimum_points: int = 20, maximum_correction_m: float = 0.6,
                 cell_m: float = 1.5, maximum_crop_points: int = 8192) -> None:
        if not 0 <= ring_inner_m < ring_outer_m or minimum_points < 1 or maximum_correction_m <= 0 or cell_m <= 0 \
                or maximum_crop_points < 1:
            raise ValueError("invalid LiDAR ground snapping parameters")
        self.ring_inner_m, self.ring_outer_m = ring_inner_m, ring_outer_m
        self.ground_percentile, self.ground_band_m = ground_percentile, ground_band_m
        self.minimum_points, self.maximum_correction_m, self.cell_m = minimum_points, maximum_correction_m, cell_m
        self.maximum_crop_points = maximum_crop_points
        self.snapped_count = 0
        self.skipped_count = 0

    def offsets(self, points: np.ndarray, centers: np.ndarray, yaws: np.ndarray, dimensions: np.ndarray) -> np.ndarray:
        """Return per-box z shifts placing each box bottom (center z - h/2) on the local ground."""
        centers = np.asarray(centers, dtype=np.float64).reshape(-1, 3)
        dimensions = np.asarray(dimensions, dtype=np.float64).reshape(-1, 3)
        yaws = np.asarray(yaws, dtype=np.float64).reshape(-1)
        shifts = np.zeros(len(centers))
        xyz = np.asarray(points, dtype=np.float32).reshape(len(points), -1)[:, :3] if len(points) else np.empty((0, 3), np.float32)
        grid = _BevGrid(xyz[:, 0], xyz[:, 1], self.cell_m) if len(centers) else None
        if grid is None or grid.empty:
            self.skipped_count += len(centers)
            return shifts
        for index, (center, yaw, (length, width, height)) in enumerate(zip(centers, yaws, dimensions)):
            reach = float(np.hypot(length / 2, width / 2)) + self.ring_outer_m
            nearby = grid.near(center[:2], reach)
            nearby = nearby[::-(-len(nearby) // self.maximum_crop_points)] if len(nearby) else nearby  # bounded cost
            local = xyz[nearby] - center.astype(np.float32)
            local = local[np.abs(local[:, 2]) < height / 2 + self.ring_outer_m]  # also drops non-finite heights
            cos, sin = np.float32(np.cos(yaw)), np.float32(np.sin(yaw))
            along, across = local[:, 0] * cos + local[:, 1] * sin, local[:, 1] * cos - local[:, 0] * sin
            outside = np.maximum(np.abs(along) - length / 2, np.abs(across) - width / 2)
            ring_z = local[(outside > self.ring_inner_m) & (outside < self.ring_outer_m), 2].astype(np.float64)
            if len(ring_z) < self.minimum_points:
                self.skipped_count += 1
                continue
            low = np.percentile(ring_z, self.ground_percentile)
            shift = float(np.median(ring_z[ring_z < low + self.ground_band_m])) + height / 2
            if abs(shift) > self.maximum_correction_m:
                self.skipped_count += 1
                continue
            shifts[index] = shift
            self.snapped_count += 1
        return shifts


class _BevGrid:
    """Coarse BEV bucketing of points so per-box crops touch only nearby cells."""

    def __init__(self, x: np.ndarray, y: np.ndarray, cell_m: float) -> None:
        self.cell_m = cell_m
        finite = np.flatnonzero(np.isfinite(x) & np.isfinite(y))
        self.empty = len(finite) == 0
        if self.empty:
            return
        column, row = np.floor(x[finite] / cell_m).astype(np.int64), np.floor(y[finite] / cell_m).astype(np.int64)
        self.origin = np.array([column.min(), row.min()])
        column -= self.origin[0]
        row -= self.origin[1]
        self.shape = (int(column.max()) + 1, int(row.max()) + 1)
        self.rows = self.shape[1]
        keys = column * self.rows + row
        if self.shape[0] * self.rows <= np.iinfo(np.int16).max:
            keys = keys.astype(np.int16)  # small keys sort with radix sort
        order = np.argsort(keys, kind="stable")
        self.order, self.sorted_keys = finite[order], keys[order].astype(np.int64)

    def near(self, center_xy: np.ndarray, reach: float) -> np.ndarray:
        """Indices of points in cells overlapping the square of half-size ``reach`` around ``center_xy``."""
        low = np.floor((np.asarray(center_xy) - reach) / self.cell_m).astype(np.int64) - self.origin
        high = np.floor((np.asarray(center_xy) + reach) / self.cell_m).astype(np.int64) - self.origin
        low, high = np.maximum(low, 0), np.minimum(high, np.asarray(self.shape) - 1)
        if (low > high).any():
            return np.empty(0, dtype=np.int64)
        starts = np.arange(low[0], high[0] + 1) * self.rows
        bounds = np.searchsorted(self.sorted_keys, np.stack([starts + low[1], starts + high[1] + 1]).ravel())
        bounds = bounds.reshape(2, -1)
        return np.concatenate([self.order[a:b] for a, b in zip(*bounds)])

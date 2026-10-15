"""Fit compact car shape parameters to accumulated LiDAR returns (vehicle side).

Points live in the tracked box frame: centered on the bbox, +X forward, +Y left,
+Z up. Only NumPy/SciPy are imported; the car profile comes from the same
trimesh-free section tables that build the rendered proxy mesh.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np
from scipy.optimize import least_squares

from src.viz.primitive_vehicle import CarShapeParams, PrimitiveVehicleAsset, car_section_tables

# Right half of a chamfered loft ring (see section_rings), bottom center to top
# center: Y as a fraction of half width, Z as bottom/top plus bevel steps.
_HALF_RING_Y = np.array([0., .78, .95, 1., 1., .95, .78, 0.])
_HALF_RING_BOTTOM = np.array([1., 1., 1., 1., 0., 0., 0., 0.])
_HALF_RING_BEVEL = np.array([0., 0., .4, 1., -1., -.4, 0., 0.])


def _box_mask(points: np.ndarray, dimensions: np.ndarray, *, margin_m: float,
              ground_clearance_m: float) -> np.ndarray:
    """Finite points inside the inflated box, excluding returns near its floor."""
    half = dimensions / 2
    with np.errstate(invalid="ignore"):
        return (np.isfinite(points).all(axis=1) & np.all(np.abs(points[:, :2]) <= half[:2] + margin_m, axis=1)
                & (points[:, 2] >= -half[2] + ground_clearance_m) & (points[:, 2] <= half[2] + margin_m))


def _voxel_keys(points: np.ndarray, voxel_m: float) -> np.ndarray:
    index = np.floor(points / voxel_m).astype(np.int64) + (1 << 20)
    return (index[:, 0] << 42) | (index[:, 1] << 21) | index[:, 2]


def _validated_dimensions(dimensions_lwh) -> np.ndarray:
    dimensions = np.asarray(dimensions_lwh, dtype=float)
    if dimensions.shape != (3,) or not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
        raise ValueError("Vehicle dimensions must be three finite positive LWH values")
    return dimensions


@dataclass
class _TrackVoxels:
    keys: np.ndarray
    sums: np.ndarray
    hits: np.ndarray
    last_timestamp_us: int


class ObjectPointAccumulator:
    """Per-track voxel hash of returns in dims-normalized object coordinates.

    Each scan adds at most one hit per voxel (its mean point), so a voxel's
    mean is averaged over scans and ``min(hits, max_hits)`` weights repeated
    observations without letting one panel dominate. Normalized storage keeps
    history valid as the tracked dimensions are refined. The caller crops points
    near other tracks and supplies coordinates from the track pose at the scan
    timestamp.
    """

    def __init__(self, *, voxel_m: float = .08, max_voxels: int = 4000, max_hits: int = 3,
                 margin_m: float = .15, ground_clearance_m: float = .15) -> None:
        if voxel_m <= 0 or max_voxels < 1 or max_hits < 1 or margin_m < 0 or ground_clearance_m < 0:
            raise ValueError("invalid object point accumulator parameters")
        self.voxel_m, self.max_voxels, self.max_hits = voxel_m, max_voxels, max_hits
        self.margin_m, self.ground_clearance_m = margin_m, ground_clearance_m
        self._tracks: dict[str, _TrackVoxels] = {}

    def add(self, track_id: str, object_points_normalized, dims_lwh, timestamp_us: int) -> int:
        """Merge one scan; stale or repeated timestamps are ignored. Returns evidence."""
        dimensions = _validated_dimensions(dims_lwh)
        state = self._tracks.get(track_id)
        if state is not None and timestamp_us <= state.last_timestamp_us:
            return len(state.keys)
        points = np.asarray(object_points_normalized, dtype=float).reshape(-1, 3) * dimensions
        points = points[_box_mask(points, dimensions, margin_m=self.margin_m,
                                  ground_clearance_m=self.ground_clearance_m)]
        keys, inverse = np.unique(_voxel_keys(points, self.voxel_m), return_inverse=True)
        means = np.zeros((len(keys), 3))
        np.add.at(means, inverse, points / dimensions)
        means /= np.bincount(inverse, minlength=len(keys))[:, None]
        if state is None:
            state = _TrackVoxels(np.empty(0, np.int64), np.empty((0, 3)), np.empty(0, np.int64), timestamp_us)
        merged, inverse = np.unique(np.concatenate([state.keys, keys]), return_inverse=True)
        sums, hits = np.zeros((len(merged), 3)), np.zeros(len(merged), np.int64)
        np.add.at(sums, inverse, np.concatenate([state.sums, means]))
        np.add.at(hits, inverse, np.concatenate([state.hits, np.ones(len(keys), np.int64)]))
        if len(merged) > self.max_voxels:
            # Evict the least-observed voxels first.
            keep = np.sort(np.argsort(-hits, kind="stable")[:self.max_voxels])
            merged, sums, hits = merged[keep], sums[keep], hits[keep]
        self._tracks[track_id] = _TrackVoxels(merged, sums, hits, timestamp_us)
        return len(merged)

    def points(self, track_id: str) -> tuple[np.ndarray, np.ndarray]:
        """Return (normalized voxel means, weights); empty arrays for unknown tracks."""
        state = self._tracks.get(track_id)
        if state is None:
            return np.empty((0, 3)), np.empty(0)
        return state.sums / state.hits[:, None], np.minimum(state.hits, self.max_hits).astype(float)

    def evidence(self, track_id: str) -> int:
        state = self._tracks.get(track_id)
        return 0 if state is None else len(state.keys)

    def drop(self, track_id: str) -> None:
        self._tracks.pop(track_id, None)

    @property
    def track_ids(self) -> tuple[str, ...]:
        return tuple(self._tracks)


@dataclass(frozen=True)
class VehicleShapeFit:
    """``cost`` is the weighted RMS point-to-surface distance (m); NaN when not fitted."""

    params: CarShapeParams
    cost: float
    evidence_voxels: int
    accepted: bool


class CarShapeFitter:
    """Bounded robust fit of CarShapeParams to one-sided surface evidence.

    Residuals run only from observed points to the model surface, so unseen
    panels cost nothing and keep their preset priors. Y is folded onto the
    symmetric profile. Each body preset and both forward signs (tracker yaw is
    ambiguous modulo 180 degrees) are fitted; the lowest robust cost wins, and
    a previous body type or sign is kept unless beaten by ``switch_margin``.
    The data term scales with occupied voxels against a Gaussian preset prior,
    so sparse evidence stays near the preset.
    """

    #: Optimization half-range around each preset value; keeps X anchors ordered.
    FIT_SPANS = {
        "windshield_base_x": .08, "roof_front_x": .06, "roof_rear_x": .06, "rear_window_base_x": .05,
        "hood_z": .08, "deck_z": .08, "nose_half_width": .06, "tail_half_width": .06, "cabin_half_width": .05,
    }

    def __init__(self, *, body_types: tuple[str, ...] = CarShapeParams.BODY_TYPES, huber_m: float = .08,
                 min_evidence_voxels: int = 35, evidence_voxel_m: float = .10, prior_weight: float = 1.,
                 switch_margin: float = .2, max_fit_points: int = 400,
                 max_rms_m: float = .25, margin_m: float = .15, ground_clearance_m: float = .15,
                 max_nfev: int = 40, tolerance: float = 1e-2) -> None:
        if not body_types or any(body_type not in CarShapeParams.BODY_TYPES for body_type in body_types):
            raise ValueError(f"body_types must be drawn from {CarShapeParams.BODY_TYPES}")
        if (huber_m <= 0 or min_evidence_voxels < 1 or evidence_voxel_m <= 0 or prior_weight < 0
                or not 0 <= switch_margin < 1 or max_fit_points < 1 or max_rms_m <= 0):
            raise ValueError("invalid car shape fitter parameters")
        self.body_types, self.huber_m = tuple(body_types), huber_m
        self.min_evidence_voxels, self.evidence_voxel_m = min_evidence_voxels, evidence_voxel_m
        self.prior_weight, self.switch_margin = prior_weight, switch_margin
        self.max_fit_points, self.max_rms_m = max_fit_points, max_rms_m
        self.margin_m, self.ground_clearance_m = margin_m, ground_clearance_m
        self.max_nfev, self.tolerance = max_nfev, tolerance

    def fit(self, points_object_frame_metric, dims_lwh, previous: CarShapeParams | None = None,
            *, weights=None) -> VehicleShapeFit:
        dimensions = _validated_dimensions(dims_lwh)
        points = np.asarray(points_object_frame_metric, dtype=float).reshape(-1, 3)
        weights = np.ones(len(points)) if weights is None else np.asarray(weights, dtype=float).reshape(-1)
        if len(weights) != len(points):
            raise ValueError("weights must match points")
        keep = _box_mask(points, dimensions, margin_m=self.margin_m, ground_clearance_m=self.ground_clearance_m)
        keep &= np.isfinite(weights) & (weights > 0)
        points, weights = points[keep], weights[keep]
        evidence = len(np.unique(_voxel_keys(points, self.evidence_voxel_m)))
        fallback = previous if previous is not None else CarShapeParams.default(self.body_types[0])
        if evidence < self.min_evidence_voxels:
            return VehicleShapeFit(fallback, float("nan"), evidence, False)
        if len(points) > self.max_fit_points:
            stride = np.linspace(0, len(points) - 1, self.max_fit_points).astype(int)
            points, weights = points[stride], weights[stride]
        # Unique voxels, not raw returns, set the data's strength against the prior.
        weights = weights * (evidence / weights.sum())
        candidates = {}
        for body_type in self.body_types:
            for sign in (1., -1.):
                candidates[body_type, sign] = self._fit_candidate(points, weights, dimensions, body_type, sign,
                                                                  previous)
        best = min(candidates, key=lambda key: candidates[key][1])
        if previous is not None and best != (previous.body_type, previous.forward_sign) \
                and (previous.body_type, previous.forward_sign) in candidates:
            incumbent = (previous.body_type, previous.forward_sign)
            if candidates[best][1] >= (1 - self.switch_margin) * candidates[incumbent][1]:
                best = incumbent
        params, _, rms = candidates[best]
        accepted = bool(rms <= self.max_rms_m)
        return VehicleShapeFit(params if accepted else fallback, rms, evidence, accepted)

    def _fit_candidate(self, points, weights, dimensions, body_type, sign, previous):
        prior = CarShapeParams.default(body_type)
        names = CarShapeParams.CONTINUOUS
        theta0 = np.array([getattr(prior, name) for name in names])
        span = np.array([self.FIT_SPANS[name] for name in names])
        bounds = np.array([CarShapeParams.BOUNDS[name] for name in names]).T
        lower, upper = np.maximum(theta0 - span, bounds[0]), np.minimum(theta0 + span, bounds[1])
        start = theta0
        if previous is not None and previous.body_type == body_type and previous.forward_sign == sign:
            start = np.clip([getattr(previous, name) for name in names], lower, upper)
        # A reversed car is the template turned 180 degrees; Y is folded anyway.
        local = np.column_stack([sign * points[:, 0], np.abs(points[:, 1]), points[:, 2] + dimensions[2] / 2])
        root_weights = np.sqrt(weights)
        prior_scale = np.sqrt(self.prior_weight) / (span / 2)

        def shape(theta):
            return replace(prior, forward_sign=sign, **dict(zip(names, theta)))

        def residuals(theta):
            scaled = surface_distance(local, dimensions, shape(theta)) / self.huber_m
            magnitude = np.abs(scaled)
            robust = np.where(magnitude <= 1, scaled, np.sign(scaled) * np.sqrt(np.maximum(2 * magnitude - 1, 1.)))
            return np.concatenate([root_weights * robust, prior_scale * (theta - theta0)])

        result = least_squares(residuals, start, bounds=(lower, upper), x_scale=span, diff_step=1e-3,
                               ftol=self.tolerance, xtol=self.tolerance, gtol=1e-6, max_nfev=self.max_nfev)
        fitted = shape(result.x)
        rms = float(np.sqrt(weights @ surface_distance(local, dimensions, fitted) ** 2 / weights.sum()))
        return fitted, float(result.cost), rms


def surface_distance(points, dimensions_lwh, shape: CarShapeParams) -> np.ndarray:
    """Signed distance (m, negative inside) from folded points to the car profile.

    ``points`` hold (X along the car's own forward axis, |Y|, Z above the bbox
    floor). Each loft is the convex chamfered section interpolated at the
    point's X, intersected with its X extent; body and cabin are unioned.
    Distances in the section plane are vertical/lateral, not surface-normal.
    """
    length, width, _ = dimensions_lwh
    x, y, z = points[:, 0] / length, points[:, 1], points[:, 2]
    tables = car_section_tables(shape)
    # Rows: body then cabin; columns: points.
    half_width, bottom, top = (np.stack([np.interp(x, sections[:, 0], sections[:, column]) for sections in tables])
                               for column in (1, 2, 3))
    ring_y = half_width[..., None] * _HALF_RING_Y * width
    ring_z = PrimitiveVehicleAsset.car_profile_z(
        _HALF_RING_BOTTOM * bottom[..., None] + (1 - _HALF_RING_BOTTOM) * top[..., None]
        + _HALF_RING_BEVEL * ((top - bottom) * .16)[..., None], dimensions_lwh)
    edge_y, edge_z = np.diff(ring_y, axis=-1), np.diff(ring_z, axis=-1)
    offset_y, offset_z = y[:, None] - ring_y[..., :-1], z[:, None] - ring_z[..., :-1]
    along = np.clip((offset_y * edge_y + offset_z * edge_z) / np.maximum(edge_y ** 2 + edge_z ** 2, 1e-12), 0., 1.)
    planar = np.sqrt(((offset_y - along * edge_y) ** 2 + (offset_z - along * edge_z) ** 2).min(axis=-1))
    # Counter-clockwise ring: the interior lies left of every edge.
    planar = np.where(np.all(edge_y * offset_z - edge_z * offset_y >= 0, axis=-1), -planar, planar)
    starts, stops = (np.array([[sections[index, 0]] for sections in tables]) for index in (0, -1))
    extent = np.maximum(starts - x, x - stops) * length
    return np.maximum(planar, extent).min(axis=0)

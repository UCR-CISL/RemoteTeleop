"""Fit universal vehicle shapes to accumulated LiDAR returns (vehicle side).

Points live in the tracked box frame: centered on the bbox, +X forward, +Y left,
+Z up. Only NumPy is imported; the body profile comes from the same NumPy
universal template that builds the rendered proxy mesh.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from src.viz.universal_vehicle import (BOUNDS, PARAMETERS, TYPE_NAMES, VEHICLE_TYPES, UniversalBodyProfile, UniversalVehicleShape,
                                       prior_moments, sanitize)

_SIZE_PARAMETERS = np.isin(PARAMETERS, ("ground_offset_m", "clearance_m", "x_tail", "x_nose",
                                        "half_width_tail", "half_width_mid", "half_width_nose"))

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
    """``cost`` is the weighted RMS point-to-surface distance (m); NaN when not fitted.

    ``robust_cost`` is the winning candidate's weighted mean robust data loss,
    the reference that lets a later refit skip the type sweep. ``prior_only``
    marks ``params`` that are the dimension-prior type mean (no usable fit).
    """

    params: UniversalVehicleShape
    cost: float
    evidence_voxels: int
    accepted: bool
    robust_cost: float = float("nan")
    prior_only: bool = False


class VehicleShapeFitter:
    """Robust MAP fit of a UniversalVehicleShape to one-sided surface evidence.

    Residuals run only from observed points to the model surface, so unseen
    panels cost nothing and stay at the type prior; parameters the points do
    not constrain (data information below ``freeze_information`` x prior
    information) are frozen. Y is folded onto the symmetric profile and the box
    heading is trusted. The residual is the ring-plane distance of
    RingSurfaceDistance (ends included), Huber-weighted at ``huber_m``.

    Types: each body type has a metric archetype prior normalized to the box
    and a dimension likelihood of the box given the type, with detector boxes
    modeled as vehicle + ``inflation_lwh_m`` (CooperScene-style boxes run about
    0.3 m larger in every axis). A sweep fits every type for ``screen_iterations``
    and refines the best ``alternatives`` plus the incumbent; the lowest objective
    (``type_evidence_scale`` x robust data + parameter prior + ``dimension_weight``
    x dimension NLL) wins, and an incumbent type is kept unless beaten by
    ``switch_margin`` (relative). Each type is fit inside ``trust_sigmas`` of its
    prior so types stay distinct. The loss is clamped: returns deeper than
    ``inside_cap_m`` inside the body (interior surfaces seen through glass) or
    farther than ``outside_cap_m`` outside it (motion smear, clutter) stop
    pulling. The data strength saturates at ``max_effective_evidence`` voxels.
    Given ``incumbent_cost`` (the previous fit's ``robust_cost``) only the
    incumbent is refit unless its robust cost grew by more than
    ``sweep_cost_growth`` (plus 0.05, about 1.8 cm of added noise at 8 cm Huber). Sparse evidence returns the previous shape, or the
    dimension-prior type mean (``prior_only``).
    """

    def __init__(self, *, body_types: tuple[str, ...] = TYPE_NAMES, huber_m: float = .08,
                 min_evidence_voxels: int = 35, evidence_voxel_m: float = .10, prior_weight: float = 1.,
                 switch_margin: float = .15, max_fit_points: int = 400, max_rms_m: float = .25,
                 margin_m: float = .15, ground_clearance_m: float = .15, max_iterations: int = 8,
                 tolerance: float = 1e-3, sweep_cost_growth: float = .25, alternatives: int = 3,
                 inflation_lwh_m=(.3, .36, .32), inflation_std_m=(.15, .06, .1), dimension_weight: float = 1.,
                 freeze_information: float = .1, trust_sigmas: float = 2.,
                 max_effective_evidence: float = 300., inside_cap_m: float = .25,
                 type_evidence_scale: float = .25, screen_iterations: int = 2,
                 screen_points: int = 150, outside_cap_m: float = .3) -> None:
        if not body_types or any(body_type not in VEHICLE_TYPES for body_type in body_types):
            raise ValueError(f"body_types must be drawn from {TYPE_NAMES}")
        if (huber_m <= 0 or min_evidence_voxels < 1 or evidence_voxel_m <= 0 or prior_weight <= 0
                or not 0 <= switch_margin < 1 or max_fit_points < 1 or max_rms_m <= 0 or sweep_cost_growth < 0
                or max_iterations < 1 or alternatives < 0 or dimension_weight < 0 or freeze_information < 0
                or len(inflation_lwh_m) != 3 or len(inflation_std_m) != 3 or min(inflation_std_m) <= 0
                or trust_sigmas <= 0 or max_effective_evidence <= 0
                or inside_cap_m <= 0 or type_evidence_scale <= 0 or not 0 <= screen_iterations < max_iterations):
            raise ValueError("invalid vehicle shape fitter parameters")
        self.body_types, self.huber_m = tuple(body_types), huber_m
        self.min_evidence_voxels, self.evidence_voxel_m = min_evidence_voxels, evidence_voxel_m
        self.prior_weight, self.switch_margin = prior_weight, switch_margin
        self.max_fit_points, self.max_rms_m = max_fit_points, max_rms_m
        self.margin_m, self.ground_clearance_m = margin_m, ground_clearance_m
        self.max_iterations, self.tolerance, self.sweep_cost_growth = max_iterations, tolerance, sweep_cost_growth
        self.alternatives, self.dimension_weight = alternatives, dimension_weight
        self.inflation_lwh_m = tuple(float(value) for value in inflation_lwh_m)
        self.inflation_std_m = tuple(float(value) for value in inflation_std_m)
        self.freeze_information = freeze_information
        self.trust_sigmas = trust_sigmas
        self.max_effective_evidence = max_effective_evidence
        self.inside_cap_m, self.type_evidence_scale = inside_cap_m, type_evidence_scale
        self.screen_iterations, self.screen_points = screen_iterations, max(1, screen_points)
        if outside_cap_m <= 0:
            raise ValueError("outside_cap_m must be positive")
        self.outside_cap_m = outside_cap_m

    # ---- priors ----------------------------------------------------------
    def dimension_nll(self, dims_lwh) -> dict[str, float]:
        """Negative log likelihood of the box per type (plus type log prior), min-shifted to 0."""
        dimensions = _validated_dimensions(dims_lwh)
        nll = {}
        for body_type in self.body_types:
            prior = VEHICLE_TYPES[body_type]
            median, spread = prior.dimension_stats()
            sigma = np.sqrt(spread ** 2 + np.square(self.inflation_std_m))
            residual = (dimensions - median - self.inflation_lwh_m) / sigma
            nll[body_type] = float(.5 * residual @ residual + np.log(sigma).sum() + prior.log_prior)
        low = min(nll.values())
        return {body_type: value - low for body_type, value in nll.items()}

    def prior_moments(self, body_type: str, dims_lwh) -> tuple[np.ndarray, np.ndarray]:
        return prior_moments(body_type, dims_lwh, inflation_lwh_m=self.inflation_lwh_m,
                             inflation_std_m=self.inflation_std_m)

    def prior_shape(self, dims_lwh) -> UniversalVehicleShape:
        """Mean of the most likely type for this box alone, normalized to the box."""
        nll = self.dimension_nll(dims_lwh)
        body_type = min(nll, key=nll.get)
        return UniversalVehicleShape(body_type, tuple(sanitize(self.prior_moments(body_type, dims_lwh)[0])[0]))

    # ---- fitting ---------------------------------------------------------
    def fit(self, points_object_frame_metric, dims_lwh, previous: UniversalVehicleShape | None = None,
            *, weights=None, incumbent_cost: float | None = None) -> VehicleShapeFit:
        dimensions = _validated_dimensions(dims_lwh)
        points = np.asarray(points_object_frame_metric, dtype=float).reshape(-1, 3)
        weights = np.ones(len(points)) if weights is None else np.asarray(weights, dtype=float).reshape(-1)
        if len(weights) != len(points):
            raise ValueError("weights must match points")
        keep = _box_mask(points, dimensions, margin_m=self.margin_m, ground_clearance_m=self.ground_clearance_m)
        keep &= np.isfinite(weights) & (weights > 0)
        points, weights = points[keep], weights[keep]
        evidence = len(np.unique(_voxel_keys(points, self.evidence_voxel_m)))
        if previous is not None and previous.body_type not in self.body_types:
            previous = None
        fallback = previous if previous is not None else self.prior_shape(dimensions)
        if evidence < self.min_evidence_voxels:
            return VehicleShapeFit(fallback, float("nan"), evidence, False, prior_only=previous is None)
        if len(points) > self.max_fit_points:
            stride = np.linspace(0, len(points) - 1, self.max_fit_points).astype(int)
            points, weights = points[stride], weights[stride]
        # Unique voxels, not raw returns, set the data's strength against the prior; accumulated
        # scans share pose errors, so the strength saturates at max_effective_evidence.
        weights = weights * (min(evidence, self.max_effective_evidence) / weights.sum())
        local = np.column_stack([points[:, 0], np.abs(points[:, 1]), points[:, 2] + dimensions[2] / 2])
        nll = self.dimension_nll(dimensions)
        incumbent = None if previous is None else previous.body_type
        candidates: dict[str, _Candidate] = {}
        if incumbent is not None and incumbent_cost is not None:
            candidates[incumbent] = self._fit_candidate(local, weights, dimensions, incumbent, previous)
            if candidates[incumbent].robust_cost <= (1 + self.sweep_cost_growth) * incumbent_cost + .05:
                return self._result(candidates[incumbent], evidence, fallback)
        # Sweep: a short fit of every type ranks them; the best alternatives and the incumbent are refined.
        # Screening runs on a subsample; refinements and the reported costs use every point.
        stride = max(1, len(local) // self.screen_points)
        quick = {body_type: self._fit_candidate(local[::stride], weights[::stride] * stride, dimensions, body_type,
                                                previous, max_iterations=self.screen_iterations)
                 for body_type in self.body_types if body_type not in candidates}
        ranked = sorted(quick, key=lambda body_type: self._type_score(
            (quick[body_type].data_objective, quick[body_type].prior_objective), nll[body_type]))
        refine = [body_type for body_type in ranked if body_type != incumbent][:self.alternatives]
        for body_type in refine + ([incumbent] if incumbent in quick else []):
            candidates[body_type] = self._fit_candidate(local, weights, dimensions, body_type, quick[body_type].shape,
                                                        max_iterations=self.max_iterations - self.screen_iterations)
        score = {body_type: self._type_score((candidate.data_objective, candidate.prior_objective), nll[body_type])
                 for body_type, candidate in candidates.items()}
        best = min(score, key=score.get)
        if incumbent is not None and best != incumbent and score[best] >= (1 - self.switch_margin) * score[incumbent]:
            best = incumbent
        return self._result(candidates[best], evidence, fallback)

    def _type_score(self, parts, nll: float) -> float:
        """Type objective: data (scaled by type_evidence_scale) + parameter prior + dimension NLL (>= 0)."""
        data, prior = parts
        return self.type_evidence_scale * data + prior + self.dimension_weight * nll

    def _result(self, candidate: _Candidate, evidence: int, fallback: UniversalVehicleShape) -> VehicleShapeFit:
        accepted = bool(candidate.rms <= self.max_rms_m)
        return VehicleShapeFit(candidate.shape if accepted else fallback, candidate.rms, evidence, accepted,
                               candidate.robust_cost)

    def _fit_candidate(self, local, weights, dimensions, body_type, previous, *,
                       max_iterations: int | None = None) -> _Candidate:
        mean, std = self.prior_moments(body_type, dimensions)
        residuals = _Residuals(local, weights, dimensions, body_type, mean, std, self)
        # A trust region on the silhouette keeps each type recognizable, so type objectives stay
        # comparable; size (ground, clearance, ends, half widths) only has the prior.
        trust = np.where(_SIZE_PARAMETERS, np.inf, self.trust_sigmas * std)
        lower = np.maximum(BOUNDS[:, 0], mean - trust)
        upper = np.minimum(BOUNDS[:, 1], mean + trust)
        start = previous.array if previous is not None and previous.body_type == body_type else mean
        theta = sanitize(np.clip(start, lower, upper))[0]
        value, jacobian = residuals.evaluate(theta, jacobian=True)
        # Parameters the points barely constrain keep their start value (prior or incumbent).
        free = np.flatnonzero((jacobian[:len(local)] ** 2).sum(0)
                              >= self.freeze_information * self.prior_weight / std ** 2)
        jacobian = jacobian[:, free]
        cost, damping = value @ value, 1e-3
        for _ in range((self.max_iterations if max_iterations is None else max_iterations) if len(free) else 0):
            # Elementwise reductions: multithreaded BLAS is slower than useful on these tiny products.
            normal = np.einsum("ni,nj->ij", jacobian, jacobian, optimize=False)
            gradient = np.einsum("ni,n->i", jacobian, value, optimize=False)
            improved = False
            for _ in range(4):
                step = np.linalg.solve(normal + damping * np.diag(np.diag(normal) + 1e-9), -gradient)
                trial = theta.copy()
                trial[free] = np.clip(theta[free] + step, lower[free], upper[free])
                trial = sanitize(trial)[0]
                trial_value = residuals.evaluate(trial)
                trial_cost = trial_value @ trial_value
                if trial_cost < cost:
                    improved = True
                    break
                damping *= 4
            if not improved:
                break
            relative = (cost - trial_cost) / max(cost, 1e-12)
            theta, cost, damping = trial, trial_cost, max(damping / 3, 1e-7)
            if relative < self.tolerance:
                break
            value, jacobian = residuals.evaluate(theta, jacobian=True, columns=free)
        return residuals.candidate(theta)


@dataclass(frozen=True)
class _Candidate:
    shape: UniversalVehicleShape
    data_objective: float  # 0.5 * robust data sum of squares
    prior_objective: float  # 0.5 * standardized parameter prior sum of squares
    rms: float
    robust_cost: float


class _Residuals:
    """Robust data residuals and standardized prior residuals of one body type."""

    STEP = 1e-5

    def __init__(self, local, weights, dimensions, body_type, mean, std, fitter: VehicleShapeFitter) -> None:
        self.local, self.weights, self.dimensions = local, weights, dimensions
        self.root_weights = np.sqrt(weights)
        self.mean, self.prior_scale = mean, np.sqrt(fitter.prior_weight) / std
        self.huber_m, self.body_type = fitter.huber_m, body_type
        self.inside_cap = fitter.inside_cap_m
        self.outside_cap = fitter.outside_cap_m
        self.prior = VEHICLE_TYPES[body_type]

    def distance(self, theta, *, jacobian: bool = False, columns=None):
        """Distances and, with ``jacobian``, their (len(columns), N) parameter derivatives."""
        columns = np.arange(len(theta)) if columns is None else np.asarray(columns)
        rows = theta[None]
        if jacobian:
            stepped = np.repeat(theta[None], len(columns), axis=0)
            stepped[np.arange(len(columns)), columns] += self.STEP
            rows = np.vstack([rows, stepped])
        rows = sanitize(rows)
        stations, rings, _ = UniversalBodyProfile.rings(rows, self.dimensions, self.prior)
        surface = RingSurfaceDistance(self.local, stations[0], rings[0])
        if not jacobian:
            return surface.distance, None
        step = rows[1:, columns].diagonal() - rows[0, columns]
        step = np.where(np.abs(step) > 0, step, np.inf)  # a sanitize-pinned parameter has no effect
        d_stations = (stations[1:] - stations[0]) / step[:, None]
        d_rings = (rings[1:] - rings[0]) / step[:, None, None, None]
        return surface.distance, surface.jacobian(d_stations, d_rings)

    def evaluate(self, theta, *, jacobian: bool = False, columns=None):
        """Stacked robust data and prior residuals (and their Jacobian over ``columns``)."""
        distance, d_distance = self.distance(theta, jacobian=jacobian, columns=columns)
        # Deep inside (interior returns, motion smear) or far outside (smear, clutter): no pull.
        deep = (distance < -self.inside_cap) | (distance > self.outside_cap)
        distance = np.clip(distance, -self.inside_cap, self.outside_cap)
        if jacobian:
            d_distance = np.where(deep, 0., d_distance)
        scaled = distance / self.huber_m
        magnitude = np.abs(scaled)
        robust = np.where(magnitude <= 1, scaled, np.sign(scaled) * np.sqrt(np.maximum(2 * magnitude - 1, 1.)))
        root_weights = self.root_weights
        value = np.concatenate([root_weights * robust, self.prior_scale * (theta - self.mean)])
        if not jacobian:
            return value
        columns = np.arange(len(theta)) if columns is None else np.asarray(columns)
        slope = root_weights / self.huber_m / np.sqrt(np.maximum(2 * magnitude - 1, 1.))
        prior = np.zeros((len(theta), len(columns)))
        prior[columns, np.arange(len(columns))] = self.prior_scale[columns]
        return value, np.vstack([slope[:, None] * d_distance.T, prior])

    def candidate(self, theta) -> _Candidate:
        distance, _ = self.distance(theta)
        value = self.evaluate(theta)
        data = value[:len(distance)]
        total = self.weights.sum()
        return _Candidate(UniversalVehicleShape(self.body_type, tuple(theta)), float(.5 * data @ data),
                          float(.5 * (value @ value - data @ data)),
                          float(np.sqrt(self.weights @ distance ** 2 / total)), float(data @ data / total))


class RingSurfaceDistance:
    """Signed distance (m, negative inside) from folded points to one body profile.

    Points are (X from the box center along +X, |Y|, Z above the box floor).
    Within the station range the body ring is interpolated at the point's X and
    the distance runs to that half-ring polyline in the (Y, Z) plane, divided by
    sqrt(1 + slope^2) with the slope of the nearest ring point along X (a
    first-order correction on windshields and cab walls). Planar end caps bound
    the body: inside points may be nearer a cap, and points beyond an end
    measure to its cap (or its rim). ``jacobian`` chains station and ring
    derivatives through the interpolation and the nearest ring edge.
    """

    def __init__(self, points, stations, rings) -> None:
        x, y, z = np.asarray(points, dtype=float).T
        count = len(x)
        self.stations, self.rings = stations, rings
        span = np.clip(np.searchsorted(stations, x, side="right") - 1, 0, len(stations) - 2)
        width = np.maximum(stations[span + 1] - stations[span], 1e-9)
        self.before, self.after = x < stations[0], x > stations[-1]
        within = ~(self.before | self.after)
        along = np.clip((x - stations[span]) / width, 0., 1.)
        ring = (1 - along)[:, None, None] * rings[span] + along[:, None, None] * rings[span + 1]
        start, edge = ring[:, :-1], np.diff(ring, axis=1)
        offset = np.stack([y, z], axis=-1)[:, None] - start
        t = np.clip((offset * edge).sum(-1) / np.maximum((edge * edge).sum(-1), 1e-12), 0., 1.)
        gap = offset - t[..., None] * edge
        nearest = (gap ** 2).sum(-1).argmin(axis=1)
        rows = np.arange(count)
        t, gap = t[rows, nearest], gap[rows, nearest]
        planar = np.linalg.norm(gap, axis=1)
        # Crossing number of a +Y ray (the y = 0 closure lies behind every point).
        low, high = start[..., 1], ring[:, 1:, 1]
        straddle = (low > z[:, None]) != (high > z[:, None])
        crossing = start[..., 0] + (z[:, None] - low) * edge[..., 0] / np.where(straddle, edge[..., 1], 1.)
        inside = (straddle & (crossing > y[:, None])).sum(axis=1) % 2 == 1
        unit = gap / np.maximum(planar, 1e-9)[:, None]
        slope_ring = (rings[span + 1] - rings[span]) / width[:, None, None]
        slope = -(unit * ((1 - t)[:, None] * slope_ring[rows, nearest]
                          + t[:, None] * slope_ring[rows, nearest + 1])).sum(-1)
        factor = np.where(within, 1 / np.sqrt(1 + slope ** 2), 1.)
        ring_distance = planar * factor
        tail_gap, nose_gap = x - stations[0], stations[-1] - x
        cap_gap = np.minimum(tail_gap, nose_gap)
        outside_gap = np.where(self.before, -tail_gap, -nose_gap)  # positive beyond an end
        corner = np.hypot(outside_gap, planar)
        # Modes: ring (sign +-1), tail/nose cap (inside or beyond with in-plane containment), end corner.
        self.ring_mode = within & (~inside | (ring_distance <= cap_gap))
        self.cap_mode = ~self.ring_mode & (within | inside)
        self.corner_mode = ~(within | inside)
        self.tail_side = np.where(within, tail_gap <= nose_gap, self.before)
        self.distance = np.select(
            [self.ring_mode, self.cap_mode],
            [np.where(inside, -ring_distance, ring_distance), np.where(within, -cap_gap, outside_gap)], corner)
        self._sign = np.where(inside, -1., 1.)
        self._state = (span, width, along, nearest, t, unit, factor, planar, outside_gap, corner, within)

    def jacobian(self, d_stations, d_rings) -> np.ndarray:
        """(P, N) distance derivatives from (P, S) station and (P, S, K, 2) ring derivatives."""
        span, width, along, nearest, t, unit, factor, planar, outside_gap, corner, within = self._state
        rows = np.arange(len(span))
        d_along = np.where(within, -((1 - along) * d_stations[:, span] + along * d_stations[:, span + 1]) / width, 0.)

        def vertex(k):
            moved = (1 - along)[:, None] * d_rings[:, span, k] + along[:, None] * d_rings[:, span + 1, k]
            return moved + (self.rings[span + 1, k] - self.rings[span, k]) * d_along[..., None]

        d_nearest = (1 - t)[:, None] * vertex(nearest) + t[:, None] * vertex(nearest + 1)
        d_planar = -(unit * d_nearest).sum(-1)
        # d(gap beyond the end) / dtheta and d(cap gap inside) / dtheta, by end.
        d_tail, d_nose = d_stations[:, 0, None], d_stations[:, -1, None]
        d_end = np.where(self.tail_side, d_tail, -d_nose)
        d_cap = np.where(within, d_end, d_end)
        jacobian = np.where(self.ring_mode, self._sign * factor * d_planar, 0.)
        jacobian += np.where(self.cap_mode, d_cap, 0.)
        jacobian += np.where(self.corner_mode, (outside_gap * d_end + planar * d_planar) / np.maximum(corner, 1e-9), 0.)
        return jacobian


def surface_distance(points, dimensions_lwh, shape: UniversalVehicleShape) -> np.ndarray:
    """RingSurfaceDistance of folded points (X, |Y|, Z above the box floor) to ``shape`` in a box."""
    stations, rings, _ = UniversalBodyProfile.rings(shape.array, dimensions_lwh, shape.prior)
    return RingSurfaceDistance(points, stations[0], rings[0]).distance


def triangle_distance(points, triangles, *, chunk: int = 256) -> np.ndarray:
    """Unsigned closest-point distance from (N, 3) points to (T, 3, 3) triangles (evaluation only)."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    a, b, c = (np.asarray(triangles, dtype=float)[:, index] for index in range(3))
    ab, ac = b - a, c - a
    result = np.empty(len(points))
    for begin in range(0, len(points), chunk):
        p = points[begin:begin + chunk, None]
        ap = p - a
        d1, d2 = (ab * ap).sum(-1), (ac * ap).sum(-1)
        bp, cp = p - b, p - c
        d3, d4 = (ab * bp).sum(-1), (ac * bp).sum(-1)
        d5, d6 = (ab * cp).sum(-1), (ac * cp).sum(-1)
        va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2
        with np.errstate(divide="ignore", invalid="ignore"):
            denominator = va + vb + vc
            v, w = vb / denominator, vc / denominator
            closest = a + v[..., None] * ab + w[..., None] * ac  # face region
            edge_ab = a + (d1 / (d1 - d3))[..., None] * ab
            edge_ac = a + (d2 / (d2 - d6))[..., None] * ac
            edge_bc = b + ((d4 - d3) / ((d4 - d3) + (d5 - d6)))[..., None] * (c - b)
        regions = [
            ((d1 <= 0) & (d2 <= 0), np.broadcast_to(a, closest.shape)),
            ((d3 >= 0) & (d4 <= d3), np.broadcast_to(b, closest.shape)),
            ((d6 >= 0) & (d5 <= d6), np.broadcast_to(c, closest.shape)),
            ((vc <= 0) & (d1 >= 0) & (d3 <= 0), edge_ab),
            ((vb <= 0) & (d2 >= 0) & (d6 <= 0), edge_ac),
            ((va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0), edge_bc),
        ]
        for mask, value in reversed(regions):
            closest = np.where(mask[..., None], value, closest)
        result[begin:begin + chunk] = np.sqrt(((p - closest) ** 2).sum(-1).min(axis=1))
    return result

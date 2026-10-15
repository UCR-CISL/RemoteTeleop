"""Fit universal vehicle shapes to accumulated LiDAR returns (vehicle side).

Points live in the tracked box frame: centered on the bbox, +X forward, +Y left,
+Z up. Only NumPy is imported; the body profile comes from the same NumPy
universal template that builds the rendered proxy mesh.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace

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


def above_ground_layer(points_m: np.ndarray, dimensions, *, max_height_m: float = .25, band_m: float = .03,
                       min_fraction: float = .45, min_points: int = 15, clearance_m: float = .08) -> np.ndarray:
    """Mask of box-frame returns not on a road layer that the box floor left in view.

    When the box floor sits below the road, road returns survive the floor cut as a
    thin dense layer. A layer (2 cm histogram mode) within ``max_height_m`` of the
    floor holding ``min_fraction`` of the returns below 0.45 m (within +-``band_m``)
    is removed up to ``clearance_m`` above it; bumpers, rockers and tires are
    vertical surfaces and never hold such a layer.
    """
    z = np.asarray(points_m, dtype=float)[:, 2] + dimensions[2] / 2
    low = z[z < .45]
    if len(low) < min_points:
        return np.ones(len(z), bool)
    counts, edges = np.histogram(low, bins=np.arange(0., .47, .02))
    mode = edges[counts.argmax()] + .01
    if mode > max_height_m or (np.abs(low - mode) <= band_m).mean() < min_fraction:
        return np.ones(len(z), bool)
    return z >= mode + clearance_m


def coverage_cells(points_normalized, *, minimum_points: int) -> np.ndarray:
    """Occupied coarse surface cells (8 along X, center/side by |Y|, 4 in Z) of box-normalized points."""
    points = np.asarray(points_normalized, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    x = np.clip(((points[:, 0] + .5) * 8).astype(int), 0, 7)
    side = (np.abs(points[:, 1]) > .3).astype(int)
    z = np.clip(((points[:, 2] + .5) * 4).astype(int), 0, 3)
    return np.bincount((x * 2 + side) * 4 + z, minlength=64) >= minimum_points


def _validated_dimensions(dimensions_lwh) -> np.ndarray:
    dimensions = np.asarray(dimensions_lwh, dtype=float)
    if dimensions.shape != (3,) or not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
        raise ValueError("Vehicle dimensions must be three finite positive LWH values")
    return dimensions


@dataclass(frozen=True)
class _Voxels:
    """Per-voxel sums of normalized scan means and the number of scans that hit each voxel."""

    keys: np.ndarray
    sums: np.ndarray
    hits: np.ndarray

    @classmethod
    def empty(cls) -> _Voxels:
        return cls(np.empty(0, np.int64), np.empty((0, 3)), np.empty(0, np.int64))

    def merged(self, *others: _Voxels, max_voxels: int) -> _Voxels:
        keys, inverse = np.unique(np.concatenate([self.keys, *(other.keys for other in others)]), return_inverse=True)
        sums, hits = np.zeros((len(keys), 3)), np.zeros(len(keys), np.int64)
        np.add.at(sums, inverse, np.concatenate([self.sums, *(other.sums for other in others)]))
        np.add.at(hits, inverse, np.concatenate([self.hits, *(other.hits for other in others)]))
        if len(keys) > max_voxels:
            # Evict the least-observed voxels first.
            keep = np.sort(np.argsort(-hits, kind="stable")[:max_voxels])
            keys, sums, hits = keys[keep], sums[keep], hits[keep]
        return _Voxels(keys, sums, hits)


@dataclass
class _Scan:
    """One buffered scan: box-frame metric returns (unshifted), its box and end-face position."""

    points: np.ndarray
    dimensions: np.ndarray
    face_m: float | None
    end: int
    shift_m: float = 0.
    voxels: _Voxels | None = None


@dataclass
class _TrackVoxels:
    base: _Voxels  # scans that left the registration buffer
    merged: _Voxels  # base + buffered scans at their current shifts
    scans: list
    last_timestamp_us: int
    registration: EndFaceRegistration
    sensor_origin_m: np.ndarray | None = None

    @property
    def keys(self) -> np.ndarray:
        return self.merged.keys


class EndFaceRegistration:
    """1-DoF along-track registration of one track's scans to a robust end-face reference.

    Tracked boxes jitter along their heading (up to ~0.35 m per scan), which
    smears the visible end of a moving car into a thick slab. Each scan's
    visible end (the end facing the sensor) is located in its belt band
    (``band`` x H above the box floor, |y| < ``half_width`` x W, within
    ``depth_m`` of the box end) as the 10th (rear) / 90th (front) x percentile.
    The reference of each end is the median of its last ``window`` faces, so
    scans are never chained to the previous scan and a stationary car does not
    drift. A scan with fewer than ``min_points`` face returns or a correction
    beyond ``max_shift_m`` is left unshifted.
    """

    def __init__(self, *, min_points: int = 12, max_shift_m: float = .5, band=(.25, .6), half_width: float = .4,
                 depth_m: float = .9, window: int = 10, percentile: float = 10.) -> None:
        if min_points < 1 or max_shift_m < 0 or depth_m <= 0 or window < 1:
            raise ValueError("invalid end-face registration parameters")
        self.min_points, self.max_shift_m, self.band, self.half_width = min_points, max_shift_m, band, half_width
        self.depth_m, self.window, self.percentile = depth_m, window, percentile
        self._faces: dict[int, list[float]] = {}
        self._reference: dict[int, float] = {}

    @staticmethod
    def visible_end(sensor_origin_m, dimensions) -> int:
        """+1 when the sensor sees the front cap, -1 the rear, 0 when it is beside the car."""
        if sensor_origin_m is None:
            return 0
        x = float(np.asarray(sensor_origin_m, dtype=float)[0])
        return 0 if abs(x) <= dimensions[0] / 2 else int(np.sign(x))

    def face(self, points_m: np.ndarray, dimensions: np.ndarray, end: int) -> float | None:
        """Visible end-face x (m) of one scan's box-frame returns, or None when ill-observed."""
        if end == 0:
            return None
        length, width, height = dimensions
        z = points_m[:, 2] + height / 2
        band = ((z >= self.band[0] * height) & (z <= self.band[1] * height)
                & (np.abs(points_m[:, 1]) < self.half_width * width) & (end * points_m[:, 0] >= length / 2 - self.depth_m))
        if band.sum() < self.min_points:
            return None
        return float(np.percentile(points_m[band, 0], 50 + end * (50 - self.percentile)))

    def observe(self, face_m: float | None, end: int) -> None:
        """Add a face position to the reference of ``end``."""
        if face_m is None:
            return
        faces = self._faces.setdefault(end, [])
        faces.append(face_m)
        del faces[:-self.window]
        self._reference[end] = float(np.median(faces))

    def shift(self, face_m: float | None, end: int) -> float:
        """Correction (m along +x) aligning a face to its end's reference; 0 when gated."""
        reference = self._reference.get(end)
        if face_m is None or reference is None or abs(reference - face_m) > self.max_shift_m:
            return 0.
        return reference - face_m


class ObjectPointAccumulator:
    """Per-track voxel hash of returns in dims-normalized object coordinates.

    Each scan adds at most one hit per voxel (its mean point), so a voxel's
    mean is averaged over scans and ``min(hits, max_hits)`` weights repeated
    observations without letting one panel dominate. Normalized storage keeps
    history valid as the tracked dimensions are refined. The caller crops points
    near other tracks and supplies coordinates from the track pose at the scan
    timestamp. Given the sensor position in the object frame, scans are shifted
    along X onto an EndFaceRegistration reference (``register``); the last
    ``buffer_scans`` scans stay raw, so they are re-voxelized when the reference
    moves by more than ``rebuild_m``.
    """

    def __init__(self, *, voxel_m: float = .08, max_voxels: int = 4000, max_hits: int = 3,
                 margin_m: float = .15, ground_clearance_m: float = .15, register: bool = True,
                 buffer_scans: int = 10, rebuild_m: float = .01) -> None:
        if (voxel_m <= 0 or max_voxels < 1 or max_hits < 1 or margin_m < 0 or ground_clearance_m < 0
                or buffer_scans < 1 or rebuild_m < 0):
            raise ValueError("invalid object point accumulator parameters")
        self.voxel_m, self.max_voxels, self.max_hits = voxel_m, max_voxels, max_hits
        self.margin_m, self.ground_clearance_m = margin_m, ground_clearance_m
        self.register, self.buffer_scans, self.rebuild_m = register, buffer_scans, rebuild_m
        self._tracks: dict[str, _TrackVoxels] = {}

    def add(self, track_id: str, object_points_normalized, dims_lwh, timestamp_us: int,
            sensor_origin_object=None) -> int:
        """Merge one scan; stale or repeated timestamps are ignored. Returns evidence.

        ``sensor_origin_object`` is the sensor position (m) in the box frame of this scan.
        """
        dimensions = _validated_dimensions(dims_lwh)
        state = self._tracks.get(track_id)
        if state is not None and timestamp_us <= state.last_timestamp_us:
            return len(state.keys)
        if state is None:
            state = self._tracks[track_id] = _TrackVoxels(_Voxels.empty(), _Voxels.empty(), [], timestamp_us,
                                                          EndFaceRegistration())
        state.last_timestamp_us = timestamp_us
        points = np.asarray(object_points_normalized, dtype=float).reshape(-1, 3) * dimensions
        points = points[np.isfinite(points).all(axis=1)]
        registration = state.registration
        end = registration.visible_end(sensor_origin_object, dimensions) if self.register else 0
        if sensor_origin_object is not None:
            state.sensor_origin_m = np.asarray(sensor_origin_object, dtype=float).reshape(3)
        face = registration.face(points, dimensions, end)
        registration.observe(face, end)
        scan = _Scan(points, dimensions, face, end)
        state.scans.append(scan)
        rebuild = False
        for buffered in state.scans:  # (re)align buffered scans to the updated reference
            shift = registration.shift(buffered.face_m, buffered.end)
            if buffered.voxels is not None and abs(shift - buffered.shift_m) > self.rebuild_m:
                buffered.voxels, rebuild = None, True
            if buffered.voxels is None:
                buffered.shift_m = shift
                buffered.voxels = self._voxelize(buffered)
        if rebuild:
            state.merged = state.base.merged(*(buffered.voxels for buffered in state.scans), max_voxels=self.max_voxels)
        else:
            state.merged = state.merged.merged(scan.voxels, max_voxels=self.max_voxels)
        if len(state.scans) > self.buffer_scans:
            state.base = state.base.merged(state.scans.pop(0).voxels, max_voxels=self.max_voxels)
        return len(state.keys)

    def _voxelize(self, scan: _Scan) -> _Voxels:
        points = scan.points + (scan.shift_m, 0., 0.)
        points = points[_box_mask(points, scan.dimensions, margin_m=self.margin_m,
                                  ground_clearance_m=self.ground_clearance_m)]
        keys, inverse = np.unique(_voxel_keys(points, self.voxel_m), return_inverse=True)
        means = np.zeros((len(keys), 3))
        np.add.at(means, inverse, points / scan.dimensions)
        means /= np.bincount(inverse, minlength=len(keys))[:, None]
        return _Voxels(keys, means, np.ones(len(keys), np.int64))

    def points(self, track_id: str) -> tuple[np.ndarray, np.ndarray]:
        """Return (normalized voxel means, weights); empty arrays for unknown tracks."""
        state = self._tracks.get(track_id)
        if state is None:
            return np.empty((0, 3)), np.empty(0)
        merged = state.merged
        return merged.sums / merged.hits[:, None], np.minimum(merged.hits, self.max_hits).astype(float)

    def sensor_origin(self, track_id: str) -> np.ndarray | None:
        """Sensor position (m, box frame) of the track's latest scan that supplied one."""
        state = self._tracks.get(track_id)
        return None if state is None else state.sensor_origin_m

    def alignment(self, track_id: str, object_points_m, dims_lwh, sensor_origin_object) -> float:
        """Correction (m along +x) that would register a scan to the track's reference, without adding it."""
        state = self._tracks.get(track_id)
        if state is None or not self.register:
            return 0.
        dimensions = _validated_dimensions(dims_lwh)
        end = state.registration.visible_end(sensor_origin_object, dimensions)
        points = np.asarray(object_points_m, dtype=float).reshape(-1, 3)
        return state.registration.shift(state.registration.face(points, dimensions, end), end)

    def shifts(self, track_id: str) -> list[float]:
        """Current along-track corrections (m) of the buffered scans, oldest first (diagnostics)."""
        state = self._tracks.get(track_id)
        return [] if state is None else [scan.shift_m for scan in state.scans]

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
    ``type_nats`` holds the relative negative log posterior (0 = best) of every
    type this fit refined, ``candidates`` those types' fits (see ``for_type``), and
    ``coverage_cells`` the occupied coarse surface cells that set the data's
    strength in the type posterior.
    """

    params: UniversalVehicleShape
    cost: float
    evidence_voxels: int
    accepted: bool
    robust_cost: float = float("nan")
    prior_only: bool = False
    type_nats: dict = field(default_factory=dict)
    candidates: dict = field(default_factory=dict, repr=False)
    coverage_cells: int = 0
    rise_x_m: float = float("nan")
    max_rms_m: float = float("inf")  # acceptance bound of the fitter that made this fit

    def for_type(self, body_type: str) -> VehicleShapeFit:
        """This fit with ``body_type``'s refined candidate as the result (same acceptance bound)."""
        candidate = self.candidates[body_type]
        return replace(self, params=candidate.shape, cost=candidate.rms, robust_cost=candidate.robust_cost,
                       accepted=bool(candidate.rms <= self.max_rms_m), prior_only=False)


class TypePosterior:
    """Per-track body-type decision from a decayed log posterior over sweeps.

    Each informative sweep (evidence grew by ``min_growth`` or new coverage
    cells; refits of the same accumulation are not independent votes) adds its
    ``type_nats`` to the log posterior after decaying it by ``decay``. The first
    sweep adopts the fitter's choice. Below ``min_type_evidence`` voxels the
    type never changes. A tentative incumbent yields to the argmax once it leads
    by ``tentative_margin_nats``; a confident one (posterior >= ``confidence``
    after ``min_sweeps`` sweeps) only after the argmax leads by
    ``switch_margin_nats`` on ``confirmations`` consecutive informative sweeps.
    """

    def __init__(self, *, decay: float = .7, switch_margin_nats: float = 3., tentative_margin_nats: float = 1.,
                 confirmations: int = 2, confidence: float = .8, min_sweeps: int = 2, min_growth: float = .1,
                 min_type_evidence: int = 150) -> None:
        if not 0 <= decay <= 1 or switch_margin_nats < 0 or tentative_margin_nats < 0 or confirmations < 1:
            raise ValueError("invalid type posterior parameters")
        self.decay, self.switch_margin_nats, self.tentative_margin_nats = decay, switch_margin_nats, tentative_margin_nats
        self.confirmations, self.confidence, self.min_sweeps = confirmations, confidence, min_sweeps
        self.min_growth, self.min_type_evidence = min_growth, min_type_evidence
        self.log_posterior: dict[str, float] = {}
        self.incumbent: str | None = None
        self.sweeps = self._streak = 0
        self._evidence = self._cells = 0

    @property
    def probability(self) -> float:
        """Posterior probability of the incumbent type (0 before the first sweep)."""
        if self.incumbent is None:
            return 0.
        values = np.array(list(self.log_posterior.values()))
        return float(1 / np.exp(values - self.log_posterior[self.incumbent]).sum())

    @property
    def confident(self) -> bool:
        return self.sweeps >= self.min_sweeps and self.probability >= self.confidence

    def update(self, fit: VehicleShapeFit) -> VehicleShapeFit:
        """Fold one fit in and return it re-targeted to the decided type (when that type was refined)."""
        if not fit.accepted and not fit.candidates:
            return fit
        if len(fit.type_nats) > 1 and self._informative(fit):
            confident = self.confident  # judged before this sweep's vote
            self.sweeps += 1
            worst = max(fit.type_nats.values())
            for body_type in set(self.log_posterior) | set(fit.type_nats):
                self.log_posterior[body_type] = (self.decay * self.log_posterior.get(body_type, 0.)
                                                 - fit.type_nats.get(body_type, worst))
            self._decide(fit, confident)
        elif self.incumbent is None:
            self.incumbent = fit.params.body_type
        if self.incumbent in fit.candidates and fit.params.body_type != self.incumbent:
            return fit.for_type(self.incumbent)
        return fit

    def _informative(self, fit: VehicleShapeFit) -> bool:
        informative = (self.sweeps == 0 or fit.evidence_voxels >= (1 + self.min_growth) * self._evidence
                       or fit.coverage_cells > self._cells)
        if informative:
            self._evidence, self._cells = fit.evidence_voxels, fit.coverage_cells
        return informative

    def _decide(self, fit: VehicleShapeFit, confident: bool) -> None:
        if self.incumbent is None or self.incumbent not in self.log_posterior:
            self.incumbent, self._streak = fit.params.body_type, 0
            return
        best = max(self.log_posterior, key=self.log_posterior.get)
        lead = self.log_posterior[best] - self.log_posterior[self.incumbent]
        if best == self.incumbent or best not in fit.candidates or fit.evidence_voxels < self.min_type_evidence:
            self._streak = 0
            return
        if not confident:
            if lead >= self.tentative_margin_nats:
                self.incumbent, self._streak = best, 0
            return
        self._streak = self._streak + 1 if lead >= self.switch_margin_nats else 0
        if self._streak >= self.confirmations:
            self.incumbent, self._streak = best, 0


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
    and refines the best ``alternatives``, every type within ``dimension_window_nats``
    of the most likely box size, and the incumbent. Types are ranked by a
    calibrated negative log posterior: the data term is a profile likelihood,
    0.5 x N x ln(robust cost / lowest robust cost), with N the occupied coarse
    surface cells (at most ``max_coverage_cells``; accumulated voxels share pose
    errors and are not independent), plus ``dimension_weight`` x the box
    dimension NLL (with the type log prior) and ``rank_prior_weight`` x the
    parameter prior. An incumbent type is kept unless beaten by
    ``switch_margin_nats``; below ``min_type_evidence`` voxels the incumbent is
    never switched and a new track takes the box-dimension type unless another
    type beats it by that margin; below ``min_shape_evidence`` only size
    parameters are fit. Given the sensor position, the visible end adds a
    TopProfile rise likelihood per type, and model center-line tops above the
    observed upper envelope where the roof would have been in view cost
    ``envelope_weight`` of the data weight (beyond ``envelope_slack_m``). A road
    layer left above the box floor is dropped first (above_ground_layer). Each
    type is fit inside ``trust_sigmas`` of its prior so types stay distinct. The loss is clamped: returns deeper than
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
                 switch_margin_nats: float = 3., max_fit_points: int = 400, max_rms_m: float = .25,
                 margin_m: float = .15, ground_clearance_m: float = .15, max_iterations: int = 8,
                 tolerance: float = 1e-3, sweep_cost_growth: float = .25, alternatives: int = 3,
                 inflation_lwh_m=(.3, .36, .32), inflation_std_m=(.15, .06, .1), dimension_weight: float = 1.,
                 freeze_information: float = .1, trust_sigmas: float = 2.,
                 max_effective_evidence: float = 300., inside_cap_m: float = .25,
                 screen_iterations: int = 2, screen_points: int = 150, outside_cap_m: float = .3,
                 max_coverage_cells: int = 48, coverage_points: int = 3, rank_prior_weight: float = .1,
                 dimension_window_nats: float = 3., min_type_evidence: int = 150,
                 min_shape_evidence: int = 150, envelope_weight: float = .3, envelope_slack_m: float = .1,
                 envelope_view_margin_m: float = .2) -> None:
        if not body_types or any(body_type not in VEHICLE_TYPES for body_type in body_types):
            raise ValueError(f"body_types must be drawn from {TYPE_NAMES}")
        if (huber_m <= 0 or min_evidence_voxels < 1 or evidence_voxel_m <= 0 or prior_weight <= 0
                or switch_margin_nats < 0 or max_fit_points < 1 or max_rms_m <= 0 or sweep_cost_growth < 0
                or max_iterations < 1 or alternatives < 0 or dimension_weight < 0 or freeze_information < 0
                or len(inflation_lwh_m) != 3 or len(inflation_std_m) != 3 or min(inflation_std_m) <= 0
                or trust_sigmas <= 0 or max_effective_evidence <= 0 or inside_cap_m <= 0
                or not 0 <= screen_iterations < max_iterations or outside_cap_m <= 0 or max_coverage_cells < 1
                or coverage_points < 1 or rank_prior_weight < 0 or dimension_window_nats < 0 or min_type_evidence < 0
                or min_shape_evidence < 0 or envelope_weight < 0 or envelope_slack_m < 0):
            raise ValueError("invalid vehicle shape fitter parameters")
        self.body_types, self.huber_m = tuple(body_types), huber_m
        self.min_evidence_voxels, self.evidence_voxel_m = min_evidence_voxels, evidence_voxel_m
        self.prior_weight, self.switch_margin_nats = prior_weight, switch_margin_nats
        self.max_fit_points, self.max_rms_m = max_fit_points, max_rms_m
        self.margin_m, self.ground_clearance_m = margin_m, ground_clearance_m
        self.max_iterations, self.tolerance, self.sweep_cost_growth = max_iterations, tolerance, sweep_cost_growth
        self.alternatives, self.dimension_weight = alternatives, dimension_weight
        self.inflation_lwh_m = tuple(float(value) for value in inflation_lwh_m)
        self.inflation_std_m = tuple(float(value) for value in inflation_std_m)
        self.freeze_information = freeze_information
        self.trust_sigmas = trust_sigmas
        self.max_effective_evidence = max_effective_evidence
        self.inside_cap_m = inside_cap_m
        self.screen_iterations, self.screen_points = screen_iterations, max(1, screen_points)
        self.outside_cap_m = outside_cap_m
        self.max_coverage_cells, self.coverage_points = max_coverage_cells, coverage_points
        self.rank_prior_weight, self.dimension_window_nats = rank_prior_weight, dimension_window_nats
        self.min_type_evidence = min_type_evidence
        self.min_shape_evidence = min_shape_evidence
        self.envelope_weight, self.envelope_slack_m = envelope_weight, envelope_slack_m
        self.envelope_view_margin_m = envelope_view_margin_m

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
            *, weights=None, incumbent_cost: float | None = None, sensor_origin=None) -> VehicleShapeFit:
        """``sensor_origin`` (m, box frame) enables the visible-end features (TopProfile)."""
        dimensions = _validated_dimensions(dims_lwh)
        points = np.asarray(points_object_frame_metric, dtype=float).reshape(-1, 3)
        weights = np.ones(len(points)) if weights is None else np.asarray(weights, dtype=float).reshape(-1)
        if len(weights) != len(points):
            raise ValueError("weights must match points")
        keep = _box_mask(points, dimensions, margin_m=self.margin_m, ground_clearance_m=self.ground_clearance_m)
        keep &= np.isfinite(weights) & (weights > 0)
        points, weights = points[keep], weights[keep]
        keep = above_ground_layer(points, dimensions)
        points, weights = points[keep], weights[keep]
        evidence = len(np.unique(_voxel_keys(points, self.evidence_voxel_m)))
        if previous is not None and previous.body_type not in self.body_types:
            previous = None
        fallback = previous if previous is not None else self.prior_shape(dimensions)
        if evidence < self.min_evidence_voxels:
            return VehicleShapeFit(fallback, float("nan"), evidence, False, prior_only=previous is None)
        coverage = int(min(coverage_cells(points / dimensions, minimum_points=self.coverage_points).sum(),
                           self.max_coverage_cells))
        if len(points) > self.max_fit_points:
            stride = np.linspace(0, len(points) - 1, self.max_fit_points).astype(int)
            points, weights = points[stride], weights[stride]
        # Unique voxels, not raw returns, set the data's strength against the prior; accumulated
        # scans share pose errors, so the strength saturates at max_effective_evidence.
        weights = weights * (min(evidence, self.max_effective_evidence) / weights.sum())
        local = np.column_stack([points[:, 0], np.abs(points[:, 1]), points[:, 2] + dimensions[2] / 2])
        nll = self.dimension_nll(dimensions)
        end = EndFaceRegistration.visible_end(sensor_origin, dimensions)
        profile = TopProfile(local, dimensions, end, sensor_origin)
        options = dict(sparse=evidence < self.min_shape_evidence, envelope=profile if self.envelope_weight > 0 else None)
        # Box size and visible-end shape features, per type (nats).
        context = _LazyTypeValues(lambda body_type: self.dimension_weight * nll[body_type] + (
            profile.nats(self._type_rise(body_type, dimensions, profile)) if profile.valid else 0.))
        incumbent = None if previous is None else previous.body_type
        candidates: dict[str, _Candidate] = {}
        if incumbent is not None and (incumbent_cost is not None or evidence < self.min_type_evidence):
            candidates[incumbent] = self._fit_candidate(local, weights, dimensions, incumbent, previous, **options)
            # Sparse evidence never switches an incumbent type; a trusted incumbent skips the sweep.
            if incumbent_cost is None or \
                    candidates[incumbent].robust_cost <= (1 + self.sweep_cost_growth) * incumbent_cost + .05:
                return self._result(candidates, incumbent, evidence, fallback, context, coverage, profile)
        # Sweep: a short fit of every type ranks them; the best alternatives, the types the box size
        # supports and the incumbent are refined. Screening runs on a subsample; refinements and the
        # reported costs use every point.
        stride = max(1, len(local) // self.screen_points)
        quick = {body_type: self._fit_candidate(local[::stride], weights[::stride] * stride, dimensions, body_type,
                                                previous, max_iterations=self.screen_iterations, **options)
                 for body_type in self.body_types if body_type not in candidates}
        screen = self._type_nats(quick, context, coverage)
        refine = sorted(screen, key=screen.get)[:self.alternatives]
        # The box-size type is always refined; others it supports unless their screen is hopeless.
        refine += [body_type for body_type in quick if nll[body_type] == 0. or (
            nll[body_type] <= self.dimension_window_nats and screen[body_type] <= 2 * self.switch_margin_nats)]
        refine += [incumbent] if incumbent in quick else []
        for body_type in dict.fromkeys(refine):
            candidates[body_type] = self._fit_candidate(local, weights, dimensions, body_type, quick[body_type].shape,
                                                        max_iterations=self.max_iterations - self.screen_iterations,
                                                        **options)
        nats = self._type_nats(candidates, context, coverage)
        best = min(nats, key=nats.get)
        if incumbent is not None and best != incumbent and nats[incumbent] - nats[best] < self.switch_margin_nats:
            best = incumbent
        if incumbent is None and evidence < self.min_type_evidence:
            # Sparse: the box size decides unless the shape evidence is decisive.
            box_type = min(nll, key=nll.get)
            if box_type in nats and nats[box_type] - nats[best] < self.switch_margin_nats:
                best = box_type
        return self._result(candidates, best, evidence, fallback, context, coverage, profile)

    def _type_rise(self, body_type: str, dimensions, profile: TopProfile) -> tuple[float, float]:
        """(rear, front) rise distances (m) of a type's mean in this box."""
        mean = sanitize(self.prior_moments(body_type, dimensions)[0])
        stations, rings, _ = UniversalBodyProfile.rings(mean, dimensions, VEHICLE_TYPES[body_type])
        return profile.model_rise(stations[0], rings[0, :, 7, 1], -1), profile.model_rise(stations[0], rings[0, :, 7, 1], 1)

    def _type_nats(self, candidates: dict, context: dict, coverage: int) -> dict[str, float]:
        """Relative negative log posterior of each candidate type (0 = best), in nats."""
        floor = max(min(candidate.robust_cost for candidate in candidates.values()), 1e-9)
        score = {body_type: .5 * coverage * np.log(max(candidate.robust_cost, floor) / floor)
                 + context[body_type] + self.rank_prior_weight * candidate.prior_objective
                 for body_type, candidate in candidates.items()}
        low = min(score.values())
        return {body_type: float(value - low) for body_type, value in score.items()}

    def _result(self, candidates: dict, best: str, evidence: int, fallback: UniversalVehicleShape, context: dict,
                coverage: int, profile: TopProfile) -> VehicleShapeFit:
        candidate = candidates[best]
        accepted = bool(candidate.rms <= self.max_rms_m)
        return VehicleShapeFit(candidate.shape if accepted else fallback, candidate.rms, evidence, accepted,
                               candidate.robust_cost, type_nats=self._type_nats(candidates, context, coverage),
                               candidates=dict(candidates),
                               coverage_cells=coverage, rise_x_m=profile.rise_x_m, max_rms_m=self.max_rms_m)

    def _fit_candidate(self, local, weights, dimensions, body_type, previous, *,
                       max_iterations: int | None = None, sparse: bool = False,
                       envelope: TopProfile | None = None) -> _Candidate:
        mean, std = self.prior_moments(body_type, dimensions)
        residuals = _Residuals(local, weights, dimensions, body_type, mean, std, self, envelope=envelope)
        # A trust region on the silhouette keeps each type recognizable, so type objectives stay
        # comparable; size (ground, clearance, ends, half widths) only has the prior. Sparse evidence
        # fits size only.
        trust = np.where(_SIZE_PARAMETERS, np.inf, 0. if sparse else self.trust_sigmas * std)
        lower = np.maximum(BOUNDS[:, 0], mean - trust)
        upper = np.minimum(BOUNDS[:, 1], mean + trust)
        start = previous.array if previous is not None and previous.body_type == body_type else mean
        theta = sanitize(np.clip(start, lower, upper))[0]
        value, jacobian = residuals.evaluate(theta, jacobian=True)
        # Parameters the points barely constrain keep their start value (prior or incumbent).
        free = np.flatnonzero(((jacobian[:len(local)] ** 2).sum(0)
                               >= self.freeze_information * self.prior_weight / std ** 2) & (upper > lower))
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


class TopProfile:
    """Observed upper envelope of a track's returns seen from its visible end, and its rise distance.

    Points are folded (X, |Y|, Z above the box floor). Returns are binned every
    ``bin_m`` along X; a bin with at least ``min_points`` returns has a top
    (95th Z percentile). The end top is the highest top within ``end_m`` of
    the end face; walking inward, the rise is where the top first climbs
    ``rise_fraction`` of the way from the end top to the highest top. It is
    0 for a step under ``min_step_m`` (SUV, minivan and van tails), about 1 m
    behind a sedan trunk and ~2 m behind a pickup tailgate (the cab).
    ``rise_x_m`` is the distance from the end face to that rise; NaN unless the
    end face lies within ``end_reach_m`` of the box end and returns continue
    ``beyond_m`` past the rise (the roof was seen, not just the end face). Only
    bins contiguous with the end (gaps up to ``max_gap_m``) and returns below
    the box top count, so clutter above or beyond the car does not fake a roof.
    ``nats`` scores a type's own rise (``model_rise``, the same rule on its
    profile) against it.
    """

    def __init__(self, local, dimensions, end: int, sensor_origin=None, *, bin_m: float = .25, min_points: int = 3,
                 rise_fraction: float = .5, end_m: float = .5, min_step_m: float = .15, end_reach_m: float = .75,
                 beyond_m: float = .5, max_gap_m: float = .75, sigma_m: float = .3, max_nats: float = 8.) -> None:
        self.end, self.sigma_m, self.max_nats = end, sigma_m, max_nats
        self.rise_fraction, self.end_m, self.min_step_m = rise_fraction, end_m, min_step_m
        self.rise_x_m = float("nan")
        self.envelope_x, self.envelope_top, self.envelope_shadow, self.bins = np.empty(0), np.empty(0), np.empty(0), 0
        self.sensor_height_m = None if sensor_origin is None else float(sensor_origin[2]) + dimensions[2] / 2
        if end == 0 or len(local) < min_points:
            return
        local = local[local[:, 2] <= dimensions[2]]  # clutter above the (inflated) box top
        x, z = local[:, 0], local[:, 2]
        bins = np.floor(x / bin_m).astype(int)
        order, inverse, counts = np.unique(bins, return_inverse=True, return_counts=True)
        inverse = inverse.reshape(-1)
        observed = np.flatnonzero(counts >= min_points)[::-end]  # from the visible end inward
        if not len(observed):
            return
        # The body is contiguous from its visible end: stop at the first gap wider than max_gap_m.
        gaps = np.flatnonzero(np.abs(np.diff(order[observed])) * bin_m > max_gap_m + 1e-9)
        observed = observed[:gaps[0] + 1] if len(gaps) else observed
        body = np.isin(inverse, observed)
        top = np.array([np.percentile(z[inverse == index], 95) for index in observed])
        end_x = float(np.percentile(x[body], 50 + end * 49))
        if end * end_x < dimensions[0] / 2 - end_reach_m:
            return
        near = -end * ((order[observed] + .5) * bin_m - end_x) <= end_m
        level = self._level(top[near].max() if near.any() else top[0], top.max())
        if level is None:
            rise, rise_x = 0, end_x
        else:
            rise = int(np.argmax(top >= level))
            high = (inverse == observed[rise]) & (z >= level)
            rise_x = float(np.percentile(x[high], 50 + end * 40))
        if (-end * (x[body] - rise_x)).max() < beyond_m:
            return
        self.rise_x_m = max(-end * (rise_x - end_x), 0.)
        # Bins between the end and the rise bound the body from above where the roof would be in view.
        self.envelope_x, self.envelope_top = (order[observed[:rise]] + .5) * bin_m, top[:rise]
        self.bins = len(observed)
        if self.sensor_height_m is not None:
            # Height below which each bin is shadowed by nearer tops (rays from the sensor in the X-Z plane).
            sensor = np.asarray(sensor_origin, dtype=float)
            reach = np.hypot(self.envelope_x - sensor[0], sensor[1])
            drop = (self.sensor_height_m - self.envelope_top) / reach  # ray descent over each bin's top
            # The most grazing ray over any nearer top bounds what a bin could show.
            self.envelope_shadow = self.sensor_height_m - reach * np.minimum.accumulate(np.r_[np.inf, drop[:-1]])

    def _level(self, end_top: float, max_top: float) -> float | None:
        """Top height that marks the rise, or None when the profile has no step."""
        if max_top - end_top < self.min_step_m:
            return None
        return end_top + self.rise_fraction * (max_top - end_top)

    @property
    def valid(self) -> bool:
        return bool(np.isfinite(self.rise_x_m))

    def nats(self, model_rise: tuple[float, float]) -> float:
        """Gaussian NLL (capped) of the observed rise given a type's (rear, front) rise; 0 when unobserved."""
        if not self.valid:
            return 0.
        expected = model_rise[0] if self.end < 0 else model_rise[1]
        return float(min(.5 * ((self.rise_x_m - expected) / self.sigma_m) ** 2, self.max_nats))

    def model_rise(self, stations_m, top_m, end: int) -> float:
        """The rise rule on a model profile (piecewise linear top over station X), from its ``end``."""
        x, top = np.asarray(stations_m, dtype=float), np.asarray(top_m, dtype=float)
        if end > 0:
            x, top = x[::-1], top[::-1]
        distance = np.abs(x - x[0])
        fine = np.arange(0., distance[-1], .02)
        profile = np.interp(fine, distance, top)
        level = self._level(profile[fine <= self.end_m].max(), profile.max())
        return 0. if level is None else float(fine[np.argmax(profile >= level)])


class _LazyTypeValues(dict):
    """Per-type values computed on first access (only the types a fit actually ranks)."""

    def __init__(self, compute) -> None:
        super().__init__()
        self._compute = compute

    def __missing__(self, body_type: str) -> float:
        value = self[body_type] = self._compute(body_type)
        return value


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

    def __init__(self, local, weights, dimensions, body_type, mean, std, fitter: VehicleShapeFitter, *,
                 envelope: TopProfile | None = None) -> None:
        self.local, self.weights, self.dimensions = local, weights, dimensions
        self.root_weights = np.sqrt(weights)
        self.mean, self.prior_scale = mean, np.sqrt(fitter.prior_weight) / std
        self.huber_m, self.body_type = fitter.huber_m, body_type
        self.inside_cap = fitter.inside_cap_m
        self.outside_cap = fitter.outside_cap_m
        self.prior = VEHICLE_TYPES[body_type]
        # Upper envelope: where the roof would be in view, a model top above the observed top costs.
        self.envelope = None
        if envelope is not None and len(envelope.envelope_x) and envelope.sensor_height_m is not None:
            weight = fitter.envelope_weight * weights.sum() / envelope.bins
            self.envelope = (envelope.envelope_x, envelope.envelope_top, envelope.sensor_height_m,
                             envelope.envelope_shadow, np.sqrt(weight))
        self.envelope_slack_m, self.view_margin_m = fitter.envelope_slack_m, fitter.envelope_view_margin_m
        self._top = None

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
        if self.envelope is not None:
            # Center-line top (roof center, or a pickup's bed floor): thin bed rails may go unseen.
            top = np.stack([np.interp(self.envelope[0], station, ring[:, 10, 1]) for station, ring in zip(stations, rings)])
        if not jacobian:
            self._top = None if self.envelope is None else (top[0], None)
            return surface.distance, None
        step = rows[1:, columns].diagonal() - rows[0, columns]
        step = np.where(np.abs(step) > 0, step, np.inf)  # a sanitize-pinned parameter has no effect
        self._top = None if self.envelope is None else (top[0], (top[1:] - top[0]) / step[:, None])
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
        envelope, d_envelope = self._envelope_rows()
        value = np.concatenate([root_weights * robust, envelope, self.prior_scale * (theta - self.mean)])
        if not jacobian:
            return value
        columns = np.arange(len(theta)) if columns is None else np.asarray(columns)
        slope = root_weights / self.huber_m / np.sqrt(np.maximum(2 * magnitude - 1, 1.))
        prior = np.zeros((len(theta), len(columns)))
        prior[columns, np.arange(len(columns))] = self.prior_scale[columns]
        if d_envelope is None or not len(envelope):
            d_envelope = np.empty((0, len(columns)))
        return value, np.vstack([slope[:, None] * d_distance.T, d_envelope, prior])

    def _envelope_rows(self):
        """Upper-envelope residuals (Huber units) and their (bins, columns) Jacobian from the last distance()."""
        if self._top is None:
            return np.empty(0), np.empty((0, 0))
        _, observed, sensor, shadow, root = self.envelope
        top, d_top = self._top
        excess = top - observed - self.envelope_slack_m
        # Only model tops the sensor would have seen: above the sensor-height margin and nearer shadows.
        active = (excess > 0) & (sensor > top + self.view_margin_m) & (top > shadow)
        rows = root * np.where(active, excess, 0.) / self.huber_m
        if d_top is None:
            return rows, None
        return rows, (root * np.where(active, 1., 0.) / self.huber_m)[:, None] * d_top.T

    def candidate(self, theta) -> _Candidate:
        distance, _ = self.distance(theta)
        value = self.evaluate(theta)
        data = value[:len(distance) + len(self._envelope_rows()[0])]  # surface and envelope rows
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

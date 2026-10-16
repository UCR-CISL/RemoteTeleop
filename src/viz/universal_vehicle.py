"""Universal deformable car body: one fixed-topology, left/right-symmetric template.

Every car type (sedan ... pickup) is the same mesh with different profile
values, so a shape hint is a short float vector and instancing only replaces
vertices and vertex colors. NumPy only: the vehicle-side shape fitter evaluates
the same profile as the rendered mesh.

Coordinates follow the proxy box: +X forward, +Y left, +Z up. A profile is
normalized to the detection box (L, W, H): X as a fraction of L about the box
center, half widths as fractions of W/2, heights as fractions of H above the
ground floor (box bottom + ``ground_offset_m``). Nothing is pinned to the box:
detector boxes are usually larger than the vehicle in every axis, so type
priors are authored in metres and normalized per box.

The side silhouette is piecewise linear between semantic landmarks so corners
survive: tail end, rear-window base, roof rear, roof front, windshield base and
nose end. Belt = top - greenhouse and roof half width = body half width - taper
(both differences non-negative). Ends are planar caps rounded by explicit bevel
lengths; an open pickup bed is a cavity in every ring whose depth is zero for
enclosed bodies.
"""
from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace
from typing import ClassVar

import numpy as np

SCHEMA_VERSION = 2.0

#: Continuous parameters, transported after (schema, type index). Units: metres for
#: the first two; x/L for ends, landmarks and bevels; /H above the ground floor for
#: heights, greenhouse and bed depth; /(W/2) for half widths and tapers.
PARAMETERS = (
    "ground_offset_m", "clearance_m", "x_tail", "x_nose",
    "x_rear_window", "x_roof_rear", "x_roof_front", "x_windshield",
    "top_tail", "top_rear_window", "top_roof_rear", "top_roof_front", "top_windshield", "top_nose",
    "greenhouse_rear_window", "greenhouse_roof_rear", "greenhouse_roof_front", "greenhouse_windshield",
    "half_width_tail", "half_width_mid", "half_width_nose",
    "taper_tail", "taper_mid", "taper_nose",
    "bevel_rear", "bevel_front", "bed_depth", "bed_width_ratio",
)
_P = {name: index for index, name in enumerate(PARAMETERS)}
BOUNDS = np.array([
    (-.3, .3), (.05, .5), (-.5, -.3), (.3, .5),
    (-.5, .2), (-.5, .3), (-.4, .45), (-.3, .48),
    *[(.15, 1.1)] * 6,
    *[(0., .7)] * 4,
    *[(.5, 1.)] * 3,
    *[(.005, .5)] * 3,
    (.003, .08), (.003, .08), (0., .45), (.6, .97),
])

# Fixed station layout (x/L): close pairs at tailgate and bulkhead carry the bed cavity.
_TAILGATE, _D_PILLAR, _B_PILLAR = .004, .02, .012
_OVERHANG_LIFT_M = .2  # underside rise from an axle to the bumper end
# Minimum landmark spacing (x/L), enforced by sanitize() and validation.
_MIN_REAR_GLASS, _MIN_ROOF, _MIN_WINDSHIELD, _MIN_HOOD = .008, .06, .02, .02
STATIONS = 18
HALF_RING = 11  # bottom center ... roof/bed center (see UniversalBodyProfile.rings)
RING = 2 * HALF_RING - 2


@dataclass(frozen=True)
class VehicleTypePrior:
    """One body type: a metric archetype profile, its spread and fixed details.

    ``profile`` is normalized to the archetype's own dimensions (ends at
    x = +-0.5); ``exemplars`` are manufacturer (L, W, H) in metres for the
    dimension likelihood. Wheels and lights are type properties, not fitted.
    """

    name: str
    archetype_lwh: tuple[float, float, float]
    exemplars: tuple[tuple[float, float, float], ...]
    profile: dict
    std: dict
    wheel_radius_m: float
    tire_width_m: float
    overhang_front: float  # axle distance from the nose end, fraction of body length
    overhang_rear: float
    b_pillar_offset: float  # B-pillar behind the windshield base, x/L
    rear_side_glass: bool = True
    tall_lights: bool = False  # vertical corner tail lights and a large grille
    log_prior: float = 0.  # -log relative frequency, nats

    def dimension_stats(self) -> tuple[np.ndarray, np.ndarray]:
        """Exemplar median and robust spread (1.4826 MAD, floored) of (L, W, H)."""
        exemplars = np.asarray(self.exemplars, dtype=float)
        median = np.median(exemplars, axis=0)
        spread = 1.4826 * np.median(np.abs(exemplars - median), axis=0)
        return median, np.maximum(spread, (.12, .04, .04))


def _profile(x, top, greenhouse, half_width, taper, bevel, *, clearance, bed=(0., .9)):
    """Archetype profile dict from compact tuples (all normalized to the archetype)."""
    return dict(ground_offset_m=0., clearance_m=clearance, x_tail=-.5, x_nose=.5,
                **dict(zip(PARAMETERS[4:8], x)), **dict(zip(PARAMETERS[8:14], top)),
                **dict(zip(PARAMETERS[14:18], greenhouse)), **dict(zip(PARAMETERS[18:21], half_width)),
                **dict(zip(PARAMETERS[21:24], taper)), bevel_rear=bevel[0], bevel_front=bevel[1],
                bed_depth=bed[0], bed_width_ratio=bed[1])


_STD = dict(ground_offset_m=.05, clearance_m=.04, x_tail=.04, x_nose=.04,
            x_rear_window=.03, x_roof_rear=.03, x_roof_front=.03, x_windshield=.03,
            top_tail=.08, top_rear_window=.08, top_roof_rear=.05, top_roof_front=.05, top_windshield=.08,
            top_nose=.08, **{name: .04 for name in PARAMETERS[14:18]},
            **{name: .025 for name in PARAMETERS[18:21]}, **{name: .03 for name in PARAMETERS[21:24]},
            bevel_rear=.01, bevel_front=.01, bed_depth=.01, bed_width_ratio=.03)

VEHICLE_TYPES: dict[str, VehicleTypePrior] = {prior.name: prior for prior in (
    VehicleTypePrior(  # Camry
        "sedan", (4.88, 1.84, 1.45),
        ((4.885, 1.840, 1.445), (4.971, 1.862, 1.450), (4.720, 1.849, 1.441), (4.674, 1.802, 1.415),
         (4.900, 1.850, 1.445), (4.630, 1.780, 1.435), (4.710, 1.825, 1.415), (4.641, 1.816, 1.447)),
        _profile((-.30, -.17, .03, .22), (.66, .70, .95, 1., .68, .53), (.02, .29, .36, .02),
                 (.90, 1., .86), (.09, .26, .10), (.025, .05), clearance=.14),
        _STD, .33, .23, .195, .22, .24),
    VehicleTypePrior(  # Golf / Civic hatch
        "hatchback", (4.40, 1.79, 1.44),
        ((4.284, 1.789, 1.456), (4.548, 1.802, 1.415), (4.375, 1.790, 1.435), (4.460, 1.795, 1.435),
         (4.600, 1.780, 1.430)),
        _profile((-.47, -.405, .03, .23), (.62, .65, .95, 1., .67, .53), (.02, .31, .36, .02),
                 (.92, 1., .86), (.16, .25, .10), (.03, .05), clearance=.14),
        _STD, .32, .22, .19, .17, .23, log_prior=.7),
    VehicleTypePrior(  # RAV4
        "crossover", (4.60, 1.86, 1.69),
        ((4.600, 1.855, 1.685), (4.703, 1.866, 1.681), (4.630, 1.865, 1.665), (4.648, 1.839, 1.699),
         (4.751, 1.921, 1.624)),
        _profile((-.46, -.405, .04, .22), (.60, .62, .95, 1., .62, .53), (.02, .36, .40, .03),
                 (.92, 1., .87), (.17, .22, .11), (.03, .05), clearance=.20),
        _STD, .36, .23, .20, .21, .23),
    VehicleTypePrior(  # Tahoe
        "full_suv", (5.35, 2.06, 1.93),
        ((5.352, 2.058, 1.927), (5.334, 2.029, 1.946), (4.830, 1.925, 1.816), (5.050, 2.004, 1.775),
         (5.204, 1.979, 1.816), (4.882, 1.894, 1.848)),
        _profile((-.465, -.445, .09, .22), (.57, .58, .97, 1., .63, .58), (.02, .40, .42, .03),
                 (.96, 1., .94), (.10, .15, .07), (.015, .03), clearance=.22),
        _STD, .40, .26, .18, .23, .22, tall_lights=True, log_prior=.7),
    VehicleTypePrior(  # Odyssey
        "minivan", (5.16, 1.99, 1.74),
        ((5.217, 1.994, 1.765), (5.175, 1.994, 1.775), (5.176, 2.022, 1.777), (5.155, 1.995, 1.775)),
        _profile((-.465, -.445, .13, .32), (.55, .56, .95, 1., .56, .47), (.02, .42, .46, .04),
                 (.94, 1., .88), (.15, .16, .12), (.02, .05), clearance=.14),
        _STD, .36, .24, .18, .22, .21, tall_lights=True, log_prior=1.2),
    VehicleTypePrior(  # Transit (low roof)
        "van", (5.58, 2.06, 2.10),
        ((5.531, 2.032, 2.083), (5.910, 2.020, 2.430), (5.410, 2.050, 2.300), (5.690, 2.012, 2.083),
         (5.981, 2.032, 2.537)),
        _profile((-.47, -.455, .24, .36), (.55, .56, .98, 1., .48, .40), (.02, .44, .48, .02),
                 (.98, 1., .92), (.07, .08, .09), (.01, .04), clearance=.17),
        _STD, .40, .24, .15, .21, .16, rear_side_glass=False, tall_lights=True, log_prior=1.8),
    VehicleTypePrior(  # F-150 crew cab, 5.5 ft bed
        "pickup", (5.89, 2.03, 1.92),
        ((5.885, 2.029, 1.961), (5.885, 2.063, 1.918), (5.916, 2.084, 1.971), (5.392, 1.910, 1.793),
         (5.931, 2.037, 1.981)),
        _profile((-.195, -.183, .115, .235), (.70, .70, .97, 1., .68, .64), (0., .30, .33, .02),
                 (.98, 1., .95), (.04, .14, .07), (.01, .03), clearance=.24, bed=(.25, .88)),
        {**_STD, "bed_depth": .05}, .40, .27, .16, .23, .22, tall_lights=True, log_prior=1.2),
)}
TYPE_NAMES = tuple(VEHICLE_TYPES)


def sanitize(theta) -> np.ndarray:
    """Project (B, P) parameter rows onto bounds, landmark order and difference limits.

    Idempotent; the result always passes UniversalVehicleShape validation.
    """
    theta = np.clip(np.array(theta, dtype=float, ndmin=2), BOUNDS[:, 0], BOUNDS[:, 1])
    tail, nose, bevel_rear, bevel_front = (theta[:, _P[name]] for name in
                                           ("x_tail", "x_nose", "bevel_rear", "bevel_front"))
    low = tail + bevel_rear + 2 * _TAILGATE + .004
    high = nose - bevel_front - _MIN_HOOD
    gaps = (_MIN_REAR_GLASS, _MIN_ROOF, _MIN_WINDSHIELD)
    landmarks = theta[:, 4:8]
    landmarks[:, 0] = np.maximum(landmarks[:, 0], low)
    for index, gap in enumerate(gaps, 1):  # forward: keep each landmark behind the next
        landmarks[:, index] = np.maximum(landmarks[:, index], landmarks[:, index - 1] + gap)
    landmarks[:, 3] = np.minimum(landmarks[:, 3], high)
    for index in range(2, -1, -1):  # backward: stay ahead of the nose limit
        landmarks[:, index] = np.minimum(landmarks[:, index], landmarks[:, index + 1] - gaps[index])
    theta[:, 4:8] = landmarks  # (box limits keep landmarks feasible for any bounded ends)
    half_width, taper = theta[:, 18:21], theta[:, 21:24]
    theta[:, 21:24] = np.clip(taper, .005, half_width - .2)
    return theta


@dataclass(frozen=True)
class UniversalVehicleShape:
    """A transported universal shape: body type plus ``PARAMETERS`` values.

    The type selects prior, wheels, lights and glass layout; the geometry is the
    profile. ``to_vector`` is ``(SCHEMA_VERSION, type index, *values)``.
    """

    body_type: str
    values: tuple[float, ...]

    TYPES: ClassVar[tuple[str, ...]] = TYPE_NAMES
    VECTOR_LENGTH: ClassVar[int] = 2 + len(PARAMETERS)

    def __post_init__(self) -> None:
        if self.body_type not in VEHICLE_TYPES:
            raise ValueError(f"Unsupported vehicle body type: {self.body_type!r}")
        values = tuple(float(value) for value in self.values)
        if len(values) != len(PARAMETERS) or not np.isfinite(values).all():
            raise ValueError(f"Universal shapes hold {len(PARAMETERS)} finite values")
        array = np.asarray(values)
        outside = (array < BOUNDS[:, 0] - 1e-9) | (array > BOUNDS[:, 1] + 1e-9)
        if outside.any():
            name = PARAMETERS[int(np.argmax(outside))]
            raise ValueError(f"{name}={array[_P[name]]!r} is outside {tuple(BOUNDS[_P[name]])}")
        if np.max(np.abs(sanitize(array)[0] - array)) > 1e-6:
            raise ValueError("Universal shape landmarks or tapers violate ordering/difference limits")
        object.__setattr__(self, "values", values)

    @property
    def array(self) -> np.ndarray:
        return np.asarray(self.values)

    @property
    def prior(self) -> VehicleTypePrior:
        return VEHICLE_TYPES[self.body_type]

    def __getitem__(self, name: str) -> float:
        return self.values[_P[name]]

    def to_vector(self) -> tuple[float, ...]:
        return (SCHEMA_VERSION, float(self.TYPES.index(self.body_type)), *self.values)

    @classmethod
    def from_vector(cls, values) -> UniversalVehicleShape:
        values = [float(value) for value in values]
        if len(values) != cls.VECTOR_LENGTH or values[0] != SCHEMA_VERSION:
            raise ValueError(f"Universal shape vectors hold {cls.VECTOR_LENGTH} values with schema {SCHEMA_VERSION}")
        index = values[1]
        if not np.isfinite(index) or index != int(index) or not 0 <= index < len(cls.TYPES):
            raise ValueError(f"Unknown vehicle body type index: {index!r}")
        return cls(cls.TYPES[int(index)], tuple(values[2:]))

    @classmethod
    def mean(cls, body_type: str, dimensions_lwh=None, **kwargs) -> UniversalVehicleShape:
        """Type mean normalized to a box (default: the type archetype); see prior_moments."""
        mean, _ = prior_moments(body_type, dimensions_lwh, **kwargs)
        return cls(body_type, tuple(sanitize(mean)[0]))


def physical_dimensions(body_type: str, dimensions_lwh, inflation_lwh_m=(0., 0., 0.),
                        inflation_std_m=(.15, .06, .1)) -> np.ndarray:
    """Vehicle size estimate inside a box: (box - inflation) blended with the type exemplars."""
    median, spread = VEHICLE_TYPES[body_type].dimension_stats()
    observed = np.asarray(dimensions_lwh, dtype=float) - np.asarray(inflation_lwh_m, dtype=float)
    observed_precision = 1 / np.square(inflation_std_m)
    type_precision = 1 / np.square(3 * spread)  # a weak pull: the box is the main evidence
    estimate = (observed * observed_precision + median * type_precision) / (observed_precision + type_precision)
    return np.clip(estimate, .5 * np.asarray(dimensions_lwh, dtype=float), np.asarray(dimensions_lwh, dtype=float))


def prior_moments(body_type: str, dimensions_lwh=None, *, inflation_lwh_m=(0., 0., 0.),
                  inflation_std_m=(.15, .06, .1)) -> tuple[np.ndarray, np.ndarray]:
    """(mean, std) of the type's parameters in this box's normalized units."""
    prior = VEHICLE_TYPES[body_type]
    box = np.asarray(prior.archetype_lwh if dimensions_lwh is None else dimensions_lwh, dtype=float)
    vehicle = (box if dimensions_lwh is None
               else physical_dimensions(body_type, box, inflation_lwh_m, inflation_std_m))
    scale = vehicle / box
    archetype = np.asarray(prior.archetype_lwh, dtype=float)
    unit = np.ones(len(PARAMETERS))
    unit[2:8] = unit[24:26] = scale[0]
    unit[8:18] = unit[26] = scale[2]
    unit[18:24] = scale[1]
    unit[1] = vehicle[2] / archetype[2]  # clearance in metres follows the vehicle's height
    mean = np.array([prior.profile[name] for name in PARAMETERS]) * unit
    std = np.array([prior.std[name] for name in PARAMETERS]) * np.where(np.arange(len(PARAMETERS)) < 2, 1., unit)
    return mean, std


def _interp_rows(x, xp, fp):
    """Row-wise np.interp: x (B, S), xp/fp (B, M) with increasing xp."""
    index = np.clip((x[..., None] >= xp[:, None, :]).sum(-1) - 1, 0, xp.shape[1] - 2)
    low, high = np.take_along_axis(xp, index, 1), np.take_along_axis(xp, index + 1, 1)
    along = np.clip((x - low) / np.maximum(high - low, 1e-12), 0., 1.)
    value = np.take_along_axis(fp, index, 1)
    return value + along * (np.take_along_axis(fp, index + 1, 1) - value)


class UniversalBodyProfile:
    """Stations and half rings of the body for batches of parameter rows.

    Half ring vertices (Y >= 0, Z above the box floor), bottom center to top:
    0 bottom center, 1 rocker inner, 2 rocker, 3 upper side, 4 belt shoulder,
    5 glass sill, 6 glass head, 7 roof edge, 8 rail lip, 9 bed floor edge,
    10 bed floor / roof center. Every row stays ordered for any parameters, so
    triangles never degenerate.
    """

    # Span roles between consecutive stations (index = span).
    SPANS = ("rear_cap", "rear_bevel", "tailgate", "bed", "bulkhead", "c_pillar", "rear_quarter",
             "d_pillar", "rear_door", "b_pillar", "front_door", "windshield", "windshield",
             "hood", "hood", "front_bevel", "front_cap")

    @staticmethod
    def stations(theta, b_pillar_offset=.23) -> np.ndarray:
        """(B, STATIONS) station x/L for sanitized rows."""
        theta = np.atleast_2d(theta)
        tail, nose, rear_window, roof_rear, roof_front, windshield, bevel_rear, bevel_front = (
            theta[:, _P[name]] for name in ("x_tail", "x_nose", "x_rear_window", "x_roof_rear", "x_roof_front",
                                            "x_windshield", "bevel_rear", "bevel_front"))
        pillar = np.clip(windshield - b_pillar_offset, roof_rear + _D_PILLAR + _B_PILLAR + .005,
                         roof_front - _B_PILLAR - .005)
        hood = nose - bevel_front
        return np.stack([
            tail, tail + bevel_rear / 2, tail + bevel_rear, tail + bevel_rear + _TAILGATE,
            rear_window - _TAILGATE, rear_window, (rear_window + roof_rear) / 2, roof_rear,
            roof_rear + _D_PILLAR, pillar - _B_PILLAR, pillar + _B_PILLAR, roof_front,
            (roof_front + windshield) / 2, windshield, (windshield + hood) / 2, hood, nose - bevel_front / 2,
            nose], axis=1)

    @classmethod
    def rings(cls, theta, dimensions_lwh, prior: VehicleTypePrior | None = None):
        """Metric profile of sanitized rows for a body type (default sedan).

        Returns station X (B, S) in metres from the box center, half rings
        (B, S, HALF_RING, 2) as (Y, Z above the box floor) in metres, and the
        per-station greenhouse height (B, S) in metres. The type places the
        B-pillar and the axles: the underside rises over each overhang (bumpers
        ride higher than the rockers), so wheels show from behind and ahead.
        """
        prior = VEHICLE_TYPES["sedan"] if prior is None else prior
        theta = np.atleast_2d(theta)
        length, width, height = np.asarray(dimensions_lwh, dtype=float)
        stations = cls.stations(theta, prior.b_pillar_offset)
        P = _P
        tail, nose = theta[:, P["x_tail"]], theta[:, P["x_nose"]]
        knots = np.column_stack([tail, theta[:, 4:8], nose])
        top = _interp_rows(stations, knots, theta[:, 8:14]) * height
        zeros = np.zeros_like(tail)
        greenhouse = _interp_rows(stations, knots, np.column_stack([zeros, theta[:, 14:18], zeros])) * height
        thirds = np.column_stack([tail, (tail + nose) / 2, nose])
        half_body = _interp_rows(stations, thirds, theta[:, 18:21]) * width / 2
        half_top = half_body - _interp_rows(stations, thirds, theta[:, 21:24]) * width / 2
        ground = theta[:, P["ground_offset_m"], None]
        bottom = ground + theta[:, P["clearance_m"], None]
        top = np.maximum(ground + top, bottom + .25)
        span = nose - tail
        rear_axle, front_axle = tail + prior.overhang_rear * span, nose - prior.overhang_front * span
        overhang = np.clip(np.maximum((rear_axle[:, None] - stations) / (prior.overhang_rear * span[:, None]),
                                      (stations - front_axle[:, None]) / (prior.overhang_front * span[:, None])), 0., 1.)
        bottom = bottom + _OVERHANG_LIFT_M * overhang
        bottom = np.minimum(bottom, top - .25)
        belt = np.clip(top - greenhouse, bottom + .1, top)
        greenhouse = top - belt
        depth = np.zeros_like(top)
        depth[:, 3:5] = theta[:, P["bed_depth"], None] * height
        depth = np.minimum(depth, top - bottom - .1)
        lower = belt - bottom
        taper = half_body - half_top
        roof_edge = half_top - np.minimum(.04, .08 * half_top)
        lip = theta[:, P["bed_width_ratio"], None] * roof_edge
        y = np.stack([np.zeros_like(top), .9 * half_body, half_body, half_body,
                      half_body - .1 * taper, half_body - .2 * taper - .001, half_top + .03 * taper,
                      roof_edge, lip, lip - .015, np.zeros_like(top)], axis=-1)
        z = np.stack([bottom, bottom, bottom + np.minimum(.06, .3 * lower), belt - np.minimum(.04, .2 * lower),
                      belt, belt + np.minimum(.03, .15 * greenhouse), top - np.minimum(.06, .2 * greenhouse),
                      top, top, top - depth, top - depth], axis=-1)
        # Bevels: the cap ring and its neighbour shrink toward the section center (quarter-round).
        inset = np.zeros_like(top)
        inset[:, 0], inset[:, 1] = theta[:, P["bevel_rear"]] * length, theta[:, P["bevel_rear"]] * length * .134
        inset[:, -1], inset[:, -2] = theta[:, P["bevel_front"]] * length, theta[:, P["bevel_front"]] * length * .134
        center = (bottom + top) / 2
        y = y * np.maximum(1 - inset / half_body, .5)[..., None]
        z = center[..., None] + (z - center[..., None]) * np.maximum(1 - 2 * inset / (top - bottom), .5)[..., None]
        return stations * length, np.stack([y, z], axis=-1), greenhouse


# Material ids and their template colors (RGBA).
BODY, GLASS, TRIM, TIRE, RIM, TAIL_LIGHT, HEAD_LIGHT, GRILLE, UNDERBODY = range(9)
MATERIAL_COLORS = np.array([
    (232, 234, 236, 255), (86, 94, 104, 255), (58, 61, 65, 255), (52, 55, 59, 255), (128, 133, 139, 255),
    (167, 52, 50, 255), (238, 240, 242, 255), (46, 49, 53, 255), (40, 42, 45, 255)], dtype=np.uint8)
_LIGHT = np.array([.25, -.35, .90]) / np.linalg.norm([.25, -.35, .90])
_WHEEL_SEGMENTS = 20


def _full_ring_index() -> np.ndarray:
    """Half-ring vertex index and Y sign for each full-ring position (counter-clockwise)."""
    half = np.arange(HALF_RING)
    mirrored = half[-2:0:-1]
    return np.concatenate([half, mirrored]), np.concatenate([np.ones(HALF_RING), -np.ones(len(mirrored))])


class UniversalVehicleMesh:
    """Fixed-topology universal car: body, lights and four wheels.

    Triangles never share vertices, so materials can follow the profile (glass,
    bed liner) and flat shading is recomputed from the deformed face normals
    per instance. ``instance`` returns metric vertices centered on the box
    and RGBA vertex colors; the topology (``faces``) never changes.
    """

    def __init__(self) -> None:
        ring, self._ring_sign = _full_ring_index()
        self._ring_half = ring
        grid = np.arange(STATIONS * RING).reshape(STATIONS, RING)
        triangles, span, edge = [], [], []
        for station in range(STATIONS - 1):
            for corner in range(RING):
                a, b = grid[station, corner], grid[station, (corner + 1) % RING]
                c, d = grid[station + 1, (corner + 1) % RING], grid[station + 1, corner]
                triangles += [(a, c, b), (a, d, c)]
                span += [station] * 2
                edge += [corner] * 2
        for station, flip in ((0, False), (STATIONS - 1, True)):
            for corner in range(1, RING - 1):
                fan = (grid[station, 0], grid[station, corner + 1], grid[station, corner])
                triangles.append(fan[::-1] if flip else fan)
                span.append(-1 if station == 0 else -2)
                edge.append(corner)
        self._body = np.asarray(triangles)
        self._span, self._edge = np.asarray(span), np.asarray(edge)
        # Mirror ring edges onto right-half edge numbers: 0 underside ... 9 bed floor / roof center.
        self._half_edge = np.where(self._edge < HALF_RING - 1, self._edge, RING - 1 - self._edge)
        self._wheel = self._wheel_template()
        self._lights = 5  # two tail lights, two head lights, one grille: one quad each
        body_corners = 3 * len(self._body)
        light_corners = 6 * self._lights
        wheel_corners = 3 * len(self._wheel[0])
        self.vertex_count = body_corners + light_corners + 4 * wheel_corners
        self.faces = np.arange(self.vertex_count).reshape(-1, 3)
        self._orientation_checked = False
        self._ensure_outward()

    # ---- template --------------------------------------------------------
    @staticmethod
    def _wheel_template():
        """Unit tire (radius 1, width 1, outer face at +Y) and outer rim disc triangles."""
        profile = [(.5, .72), (.5, .9), (.32, 1.), (-.32, 1.), (-.5, .9), (-.5, .72)]
        angles = np.arange(_WHEEL_SEGMENTS) * 2 * np.pi / _WHEEL_SEGMENTS
        points = np.array([(r * np.cos(a), y, r * np.sin(a)) for y, r in profile for a in angles])
        triangles = []
        for ring in range(len(profile) - 1):
            for index in range(_WHEEL_SEGMENTS):
                a, b = ring * _WHEEL_SEGMENTS + index, ring * _WHEEL_SEGMENTS + (index + 1) % _WHEEL_SEGMENTS
                c, d = a + _WHEEL_SEGMENTS, b + _WHEEL_SEGMENTS
                triangles += [(a, d, b), (a, c, d)]
        tire = points[np.asarray(triangles)]
        rim_center = np.array([0., .48, 0.])
        rim_ring = np.array([(.74 * np.cos(a), .48, .74 * np.sin(a)) for a in angles])
        rim = np.array([(rim_center, rim_ring[(index + 1) % _WHEEL_SEGMENTS], rim_ring[index])
                        for index in range(_WHEEL_SEGMENTS)])
        triangles = np.concatenate([tire, rim])
        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        centroid = triangles.mean(axis=1)
        outward = np.where(np.arange(len(triangles)) < len(tire), 1, 0)
        # Tire faces point away from the axle; rim faces point to +Y.
        direction = np.where(outward[:, None] == 1, centroid * (1, 2, 1), (0, 1, 0))
        flip = np.einsum("ij,ij->i", normals, direction) < 0
        triangles[flip] = triangles[flip][:, ::-1]
        materials = np.where(outward == 1, TIRE, RIM)
        return triangles, materials

    def _ensure_outward(self) -> None:
        """Orient side faces and both caps outward on the sedan archetype (winding is shape-invariant)."""
        shape = UniversalVehicleShape.mean("sedan")
        x, rings, _ = UniversalBodyProfile.rings(shape.array, shape.prior.archetype_lwh)
        corners = self._body_corners(x[0], rings[0])
        normals = np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0])
        center = corners.reshape(-1, 3).mean(axis=0)
        sides = self._span >= 0
        groups = ((sides, np.einsum("ij,ij->i", normals, corners.mean(axis=1) - center)),
                  (self._span == -1, -normals[:, 0]), (self._span == -2, normals[:, 0]))
        for group, outward in groups:
            if outward[group].sum() < 0:
                self._body[group] = self._body[group][:, ::-1]

    def _body_corners(self, stations_m, half_rings) -> np.ndarray:
        """(F, 3, 3) metric body triangles for one profile (Z above the box floor)."""
        ring_y = half_rings[:, self._ring_half, 0] * self._ring_sign
        ring_z = half_rings[:, self._ring_half, 1]
        vertices = np.stack([np.repeat(stations_m[:, None], RING, 1), ring_y, ring_z], axis=-1).reshape(-1, 3)
        return vertices[self._body]

    def template_geometry(self) -> SimpleNamespace:
        """Archetype sedan geometry for PyTorch3D ``load_geometry(preserve_topology=True)``."""
        shape = UniversalVehicleShape.mean("sedan")
        vertices, colors = self.instance(shape, shape.prior.archetype_lwh)
        return SimpleNamespace(vertices=vertices, faces=self.faces, visual=SimpleNamespace(vertex_colors=colors))

    # ---- instances -------------------------------------------------------
    def instance(self, shape: UniversalVehicleShape, dimensions_lwh, body_rgb=None):
        """Metric vertices (centered on the box) and RGBA colors for ``shape`` in a box.

        ``body_rgb`` (0-1 sRGB) repaints body panels with the legacy paint lift.
        """
        dimensions = np.asarray(dimensions_lwh, dtype=float)
        triangles, materials = self._floor_surface(shape, dimensions)
        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
        shade = .78 + .22 * np.maximum(normals @ _LIGHT, 0.)
        colors = MATERIAL_COLORS[materials].astype(float)
        if body_rgb is not None:
            from src.viz.primitive_vehicle import PrimitiveVehicleAsset

            colors[materials == BODY, :3] = np.rint(PrimitiveVehicleAsset.paint_rgb(body_rgb) * 255)
        colors[:, :3] = np.rint(colors[:, :3] * shade[:, None])
        vertices = triangles.reshape(-1, 3)
        vertices[:, 2] -= dimensions[2] / 2
        return vertices, np.repeat(colors.astype(np.uint8), 3, axis=0)

    def surface(self, shape: UniversalVehicleShape, dimensions_lwh) -> tuple[np.ndarray, np.ndarray]:
        """Metric (F, 3, 3) triangles centered on the box and their (F,) material ids (``BODY`` ...)."""
        triangles, materials = self._floor_surface(shape, np.asarray(dimensions_lwh, dtype=float))
        triangles[..., 2] -= float(dimensions_lwh[2]) / 2
        return triangles, materials

    def _floor_surface(self, shape: UniversalVehicleShape, dimensions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Triangles with Z above the box floor, and material ids."""
        if dimensions.shape != (3,) or not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
            raise ValueError("Vehicle dimensions must be three finite positive LWH values")
        prior = shape.prior
        stations, rings, greenhouse = UniversalBodyProfile.rings(shape.array, dimensions, prior)
        stations, rings, greenhouse = stations[0], rings[0], greenhouse[0]
        body = self._body_corners(stations, rings)
        body_materials = self._body_materials(shape, rings, greenhouse, dimensions)
        lights, light_materials = self._light_quads(prior, stations, rings)
        wheels, wheel_materials = self._wheels(shape, stations, rings, dimensions)
        return (np.concatenate([body, lights, wheels]),
                np.concatenate([body_materials, light_materials, wheel_materials]))

    def _body_materials(self, shape, rings, greenhouse, dimensions) -> np.ndarray:
        """Semantic face regions: windows by span role and ring edge, bed liner, underbody."""
        prior = shape.prior
        span, edge = self._span, self._half_edge
        role = np.array(UniversalBodyProfile.SPANS + ("cap", "cap"))[span]
        materials = np.full(len(span), BODY)
        materials[edge == 0] = UNDERBODY
        side_band = edge == 5
        face_greenhouse = np.where(span >= 0, (greenhouse[np.maximum(span, 0)]
                                               + greenhouse[np.minimum(np.maximum(span, 0) + 1, STATIONS - 1)]) / 2, 0.)
        windows = {"front_door", "windshield"} | ({"rear_door", "rear_quarter"} if prior.rear_side_glass else set())
        materials[side_band & np.isin(role, list(windows)) & (face_greenhouse >= .12)] = GLASS
        inner = (edge == 8) | (edge == 9)
        materials[inner & (role == "windshield")] = GLASS
        top = rings[:, 10, 1]
        if top[7] - top[5] > .1 * dimensions[2]:  # the roof falls to the rear-window base
            materials[inner & np.isin(role, ["c_pillar", "rear_quarter"])] = GLASS
        if shape["bed_depth"] * dimensions[2] > .05:
            materials[inner & np.isin(role, ["tailgate", "bed", "bulkhead"])] = TRIM
        return materials

    @staticmethod
    def _cap_quads(x, ring, bands, outward) -> np.ndarray:
        """Quads on a cap plane: bands of (y0, y1, z0, z1) metres -> (2 * bands, 3, 3) triangles."""
        quads = []
        for y0, y1, z0, z1 in bands:
            corners = np.array([(x, y0, z0), (x, y1, z0), (x, y1, z1), (x, y0, z1)])
            first, second = (0, 1, 2), (0, 2, 3)
            if outward < 0:
                first, second = first[::-1], second[::-1]
            quads += [corners[list(first)], corners[list(second)]]
        return np.asarray(quads)

    def _light_quads(self, prior, stations, rings):
        tail, nose = rings[0], rings[-1]
        triangles = []
        for ring, x, sign in ((tail, stations[0] - .004, -1.), (nose, stations[-1] + .004, 1.)):
            side, low, high = ring[3, 0], ring[2, 1], ring[4, 1]
            if sign < 0:
                height, (inner, outer) = (.3, (.76, .95)) if prior.tall_lights else (.12, (.52, .93))
                z1 = high - .05
                bands = [(-outer * side, -inner * side, z1 - height, z1), (inner * side, outer * side, z1 - height, z1)]
            else:
                z1 = high - .04
                grille = .3 if prior.tall_lights else .14
                bands = [(-.9 * side, -.5 * side, z1 - .1, z1), (.5 * side, .9 * side, z1 - .1, z1),
                         (-.42 * side, .42 * side, z1 - .02 - grille, z1 - .02)]
            bands = [(y0, y1, max(z0, low + .02), max(z1, low + .04)) for y0, y1, z0, z1 in bands]
            # Quads are wound in +Y then +Z order; a -X face (tail) needs the reverse.
            triangles.append(self._cap_quads(x, ring, bands, sign))
        materials = np.repeat([TAIL_LIGHT, TAIL_LIGHT, HEAD_LIGHT, HEAD_LIGHT, GRILLE], 2)
        return np.concatenate(triangles), materials

    def _wheels(self, shape, stations, rings, dimensions):
        prior = shape.prior
        length, width, height = dimensions
        body = stations[-1] - stations[0]
        radius, tire = prior.wheel_radius_m, prior.tire_width_m
        factor = min(1., body / (radius * 4.05), height / (2.4 * radius), width / 2 / tire)
        radius, tire = radius * factor, tire * factor
        axles = (stations[0] + prior.overhang_rear * body, stations[-1] - prior.overhang_front * body)
        unit, materials = self._wheel
        triangles = []
        for x in axles:
            half_body = np.interp(x, stations, rings[:, 3, 0])
            outer = min(half_body + .025, width / 2 + .02)  # sidewall just proud of the body side
            for side in (1., -1.):
                center = np.array([x, side * (outer - tire / 2), shape["ground_offset_m"] + radius])
                wheel = unit * (radius, tire, radius)
                if side < 0:
                    wheel = wheel * (1, -1, 1)
                    wheel = wheel[:, ::-1]  # mirroring flips the winding
                triangles.append(wheel + center)
        return np.concatenate(triangles), np.tile(materials, 4)

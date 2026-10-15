"""Vehicle-side universal shape accumulation and fitting from sparse one-sided returns."""
import subprocess
import sys
import time

import numpy as np
import pytest

from src.deployment.vehicle_shape import (EndFaceRegistration, ObjectPointAccumulator, TopProfile, TypePosterior,
                                          VehicleShapeFit, VehicleShapeFitter, _Residuals, above_ground_layer,
                                          surface_distance, triangle_distance)
from src.viz.universal_vehicle import (PARAMETERS, TYPE_NAMES, VEHICLE_TYPES, UniversalVehicleMesh,
                                       UniversalVehicleShape, sanitize)

DIMENSIONS = np.array([4.7, 1.85, 1.5])
INFLATION = np.array([.3, .36, .32])
MESH = UniversalVehicleMesh()


def _body_triangles(shape, dimensions):
    vertices, _ = MESH.instance(shape, dimensions)
    return vertices[:3 * len(MESH._body)].reshape(-1, 3, 3)


def _sample(shape, dimensions, count, sensor, *, seed=0, noise=.02, where=None):
    """Area-weighted returns on body faces facing a sensor position (object frame, box centered)."""
    triangles = _body_triangles(shape, dimensions)
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    centers = triangles.mean(axis=1)
    weight = np.linalg.norm(normals, axis=1) * (np.einsum("ij,ij->i", normals, np.asarray(sensor) - centers) > 0)
    if where is not None:
        weight = weight * where(centers)
    rng = np.random.default_rng(seed)
    faces = rng.choice(len(triangles), count, p=weight / weight.sum())
    a, b = rng.random((2, count))
    flip = a + b > 1
    a[flip], b[flip] = 1 - a[flip], 1 - b[flip]
    t = triangles[faces]
    return t[:, 0] + a[:, None] * (t[:, 1] - t[:, 0]) + b[:, None] * (t[:, 2] - t[:, 0]) + rng.normal(
        0, noise, (count, 3))


def _truth(body_type, *, seed=3, scale=.5):
    """A plausible member of a type in a detector-inflated box, and that box."""
    box = VEHICLE_TYPES[body_type].dimension_stats()[0] + INFLATION
    fitter = VehicleShapeFitter()
    mean, std = fitter.prior_moments(body_type, box)
    theta = sanitize(mean + np.random.default_rng(seed).normal(0, scale, len(mean)) * std)[0]
    return UniversalVehicleShape(body_type, tuple(theta)), box


def _error(shape, truth, box):
    _, std = VehicleShapeFitter().prior_moments(truth.body_type, box)
    return float(np.sqrt(np.mean(((shape.array - truth.array) / std) ** 2)))


def _local(points, dimensions):
    return np.column_stack([points[:, 0], np.abs(points[:, 1]), points[:, 2] + dimensions[2] / 2])


def test_module_imports_without_rendering_dependencies():
    code = ("import sys, src.deployment.vehicle_shape; "
            "print([name for name in ('trimesh', 'torch', 'open3d', 'pytorch3d') if name in sys.modules])")
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True)
    assert result.stdout.strip() == "[]"


@pytest.mark.parametrize("body_type", TYPE_NAMES)
def test_surface_distance_matches_the_emitted_triangles(body_type):
    shape, box = _truth(body_type)
    clean = _sample(shape, box, 400, (-9., 5., 2.), noise=0.)
    assert np.percentile(np.abs(surface_distance(_local(clean, box), box, shape)), 95) < 5e-3
    noisy = clean + np.random.default_rng(1).normal(0, .1, clean.shape)
    ring = surface_distance(_local(noisy, box), box, shape)
    exact = triangle_distance(noisy, _body_triangles(shape, box))
    assert np.percentile(np.abs(np.abs(ring) - exact), 90) < .02
    center = np.array([[0., 0., box[2] * .4]])
    assert surface_distance(center, box, shape)[0] < -.3  # inside is negative


def test_accumulator_merges_scans_per_voxel_and_crops():
    accumulator = ObjectPointAccumulator(voxel_m=.1, max_hits=3)
    point = np.array([[.4, .49, .1]])  # Normalized, near the left rear quarter.
    ground = [[0., .49, -.49]]  # 1.5 cm above the box floor.
    outside = [[.9, 0., 0.]]
    for timestamp in range(5):
        accumulator.add("car", np.vstack([point, point + 1e-3, ground, outside]), DIMENSIONS, timestamp)
    assert accumulator.add("car", [[0., 0., .4]], DIMENSIONS, 4) == 1  # repeated timestamp ignored
    points, weights = accumulator.points("car")
    assert accumulator.evidence("car") == 1
    np.testing.assert_allclose(points, point + 5e-4)
    np.testing.assert_array_equal(weights, [3.])
    accumulator.add("car", [[0., 0., .4]], DIMENSIONS, 9)
    assert accumulator.evidence("car") == 2
    assert accumulator.track_ids == ("car",)
    accumulator.drop("car")
    assert accumulator.evidence("car") == 0 and len(accumulator.points("car")[0]) == 0


def test_accumulator_caps_voxels_by_evicting_least_observed():
    accumulator = ObjectPointAccumulator(voxel_m=.1, max_voxels=10)
    seen = np.column_stack([np.linspace(-.45, .45, 5), np.zeros(5), np.zeros(5)])
    accumulator.add("car", seen, DIMENSIONS, 0)
    accumulator.add("car", seen, DIMENSIONS, 1)
    once = np.column_stack([np.zeros(20), np.linspace(-.45, .45, 20), np.full(20, .3)])
    assert accumulator.add("car", once, DIMENSIONS, 2) == 10
    points, weights = accumulator.points("car")
    assert (weights == 2).sum() == 5
    assert all(np.isclose(points, row).all(axis=1).any() for row in seen)


def _face_slab(accumulator, track_id, box):
    """X thickness (p90 - p10) of the accumulated rear face's belt band."""
    points = accumulator.points(track_id)[0] * box
    z = points[:, 2] + box[2] / 2
    band = (z > .3 * box[2]) & (z < .55 * box[2]) & (np.abs(points[:, 1]) < .3 * box[1]) & (points[:, 0] < -box[0] / 2 + .9)
    return np.subtract(*np.percentile(points[band, 0], [90, 10]))


def test_end_face_registration_removes_along_track_jitter_without_drift():
    truth, box = _truth("sedan")
    sensor = np.array([-12., 2., 1.9 - box[2] / 2])  # behind the car, box frame
    jitter = np.random.default_rng(2).uniform(-.2, .2, 10)
    smeared, registered, still = (ObjectPointAccumulator(register=False), ObjectPointAccumulator(),
                                  ObjectPointAccumulator())
    for scan, offset in enumerate(jitter):
        points = _sample(truth, box, 600, sensor, seed=scan, noise=.01)
        for accumulator, shift in ((smeared, offset), (registered, offset), (still, 0.)):
            accumulator.add("car", (points + (shift, 0., 0.)) / box, box, scan, sensor_origin_object=sensor - (shift, 0, 0))
    assert _face_slab(smeared, "car", box) > .25
    assert _face_slab(registered, "car", box) < .12
    # The reference is the median face, so registered scans sit near the jitter-free face, ...
    assert np.median(registered.shifts("car") + jitter) == pytest.approx(np.median(jitter), abs=.03)
    # ... and a stationary car is not dragged anywhere.
    assert np.abs(still.shifts("car")).max() < .02
    assert _face_slab(still, "car", box) < .12


def _end_slab(accumulator, track_id, box, end):
    """Median over 0.2 m (Y, Z) cells of the X thickness (p90 - p10) of one end's accumulated belt band."""
    points = accumulator.points(track_id)[0] * box
    z = points[:, 2] + box[2] / 2
    points = points[(z > .25 * box[2]) & (z < .6 * box[2]) & (np.abs(points[:, 1]) < .4 * box[1])
                    & (end * points[:, 0] > box[0] / 2 - .9)]
    _, cell, counts = np.unique(np.floor(points[:, 1:] / .2).astype(int), axis=0, return_inverse=True,
                                return_counts=True)
    return np.median([np.subtract(*np.percentile(points[cell.reshape(-1) == index, 0], [90, 10]))
                      for index in np.flatnonzero(counts >= 4)])


def test_registration_keeps_one_frame_through_a_box_bias_change_and_an_end_transition():
    truth, box = _truth("sedan")
    rng = np.random.default_rng(4)
    biased, clean = ObjectPointAccumulator(), ObjectPointAccumulator(register=False)
    for scan in range(60):
        # Sensor behind the car, then ahead of it; the box bias jumps by 0.3 m mid-way (inward,
        # so the box crop does not clip the front face).
        sensor = np.array([-12. if scan < 30 else 12., 2., 1.9 - box[2] / 2])
        offset = (-.3 if scan >= 20 else 0.) + rng.uniform(-.08, .08)
        points = _sample(truth, box, 600, sensor, seed=scan, noise=.01)
        biased.add("car", (points + (offset, 0., 0.)) / box, box, scan, sensor_origin_object=sensor - (offset, 0, 0))
        steady = 0. if scan < 30 else -.3  # one consistent box per visible end
        clean.add("car", (points + (steady, 0., 0.)) / box, box, scan, sensor_origin_object=sensor - (steady, 0, 0))
    for end in (-1, 1):  # the slab a perfectly registered track would show, plus 5 cm
        assert _end_slab(biased, "car", box, end) < _end_slab(clean, "car", box, end) + .05


def test_end_face_registration_gates_unobserved_or_implausible_faces():
    registration = EndFaceRegistration()
    box = np.array([4.8, 2.1, 1.8])
    assert registration.visible_end([-10., 0., 1.], box) == -1 and registration.visible_end([1., 8., 1.], box) == 0
    assert registration.face(np.zeros((5, 3)), box, -1) is None  # too few face returns
    face = np.column_stack([np.full(40, -2.2), np.linspace(-.5, .5, 40), np.full(40, -.3)])
    assert registration.face(face, box, -1) == pytest.approx(-2.2)
    registration.observe(-2.2, -1)
    assert registration.shift(-2.0, -1) == pytest.approx(-.2) and registration.shift(-1.5, -1) == 0.  # > 0.5 m
    assert registration.shift(-2.0, 1) == 0.  # no reference for the front yet
    for _ in range(registration.window):
        registration.observe(-2.0, -1)
    assert registration.locked(-1) and not registration.locked(1)
    reference = registration.shift(-2.0, -1)
    registration.observe(-1.7, -1)  # a locked reference no longer follows the box
    assert registration.shift(-2.0, -1) == reference


def test_road_layer_left_above_the_box_floor_is_removed():
    truth, box = _truth("sedan")
    car = _sample(truth, box, 600, (-9., 5., 1.9))
    rng = np.random.default_rng(0)
    road = np.column_stack([rng.uniform(-3, 3, 300), rng.uniform(-1, 1, 300), np.full(300, .18 - box[2] / 2)])
    keep = above_ground_layer(np.vstack([car, road]), box)
    assert not keep[len(car):].any() and keep[:len(car)].mean() > .95
    assert above_ground_layer(car, box).all()  # no layer: nothing removed


@pytest.mark.parametrize("body_type", ["sedan", "crossover", "pickup"])
def test_rise_from_the_visible_end_tells_trunks_and_beds_from_tailgates(body_type):
    truth, box = _truth(body_type, scale=0.)
    sensor = np.array([-12., 1., 2.3 - box[2] / 2])
    points = _sample(truth, box, 1500, sensor, seed=1, noise=.01)
    profile = TopProfile(_local(points, box), box, -1, sensor)
    fitter = VehicleShapeFitter()
    expected = fitter._type_rise(body_type, box, profile)[0]
    assert profile.valid and profile.rise_x_m == pytest.approx(expected, abs=.3)
    rises = {other: fitter._type_rise(other, box, profile)[0] for other in TYPE_NAMES}
    assert rises["crossover"] < rises["sedan"] - .4 and rises["sedan"] < rises["pickup"] - .2
    assert profile.nats(fitter._type_rise(body_type, box, profile)) < 1.
    assert not TopProfile(_local(points, box), box, 0).valid  # no visible end, no feature


def test_rise_is_unknown_without_upper_body_returns():
    truth, box = _truth("sedan", scale=0.)
    sensor = np.array([-12., 1., 2.3 - box[2] / 2])
    # A low side/rear panel only (no roof): the flat top must not read as an SUV tail (rise 0).
    low = _sample(truth, box, 300, sensor, seed=2, noise=.01, where=lambda centers: centers[:, 2] < 0)
    profile = TopProfile(_local(low, box), box, -1, sensor)
    assert not profile.valid and profile.nats((0., 0.)) == 0.


def test_pickup_seen_from_behind_beats_a_minivan_of_the_same_box():
    truth, box = _truth("pickup", scale=.3)
    sensor = np.array([-12., 2., 2.3 - box[2] / 2])
    points = _sample(truth, box, 1500, sensor, seed=6)
    fit = VehicleShapeFitter().fit(points, box, sensor_origin=sensor)
    assert fit.params.body_type == "pickup" and fit.type_nats["pickup"] == 0.
    assert fit.type_nats.get("minivan", np.inf) > 3. and np.isfinite(fit.rise_x_m)
    # Every type is ranked: refined types by their fits, screened-out ones by their screens.
    assert set(fit.candidates) < set(fit.type_nats) == set(TYPE_NAMES)
    assert fit.for_type(next(iter(fit.candidates))).accepted


def test_sparse_evidence_fits_size_only_and_takes_the_box_type():
    truth, box = _truth("crossover")
    points = _sample(truth, box, 120, (-9., 5., 1.9), seed=2)
    fitter = VehicleShapeFitter()
    fit = TypePosterior().update(fitter.fit(points, box))
    assert fitter.min_evidence_voxels <= fit.evidence_voxels < fitter.min_shape_evidence and fit.accepted
    assert fit.params.body_type == fit.box_type == fitter.prior_shape(box).body_type
    mean = sanitize(fitter.prior_moments(fit.params.body_type, box)[0])[0]
    for name in ("x_rear_window", "top_roof_rear", "greenhouse_roof_front", "bed_depth"):
        assert fit.params[name] == pytest.approx(mean[PARAMETERS.index(name)], abs=5e-3)  # (sanitize ordering only)


def _sweep(nats, evidence, cells=20, refined=None):
    """A fit result carrying only what TypePosterior reads (``refined``: the types with candidates)."""
    shapes = {body_type: UniversalVehicleShape.mean(body_type) for body_type in nats}
    candidates = {body_type: type("C", (), dict(shape=shape, rms=.05, robust_cost=1.))()
                  for body_type, shape in shapes.items() if refined is None or body_type in refined}
    best = min(nats, key=nats.get)
    return VehicleShapeFit(shapes[best], .05, evidence, True, 1., type_nats=nats, candidates=candidates,
                           coverage_cells=cells, max_rms_m=.25, box_type=next(iter(nats)))


def test_type_posterior_needs_confirmation_to_leave_a_confident_type():
    posterior = TypePosterior()
    assert posterior.update(_sweep({"sedan": 0., "hatchback": 4.}, 200)).params.body_type == "sedan"
    assert posterior.update(_sweep({"sedan": 0., "hatchback": 4.}, 300)).params.body_type == "sedan"
    assert posterior.confident
    # A repeated refit of the same accumulation is not a new vote.
    assert posterior.update(_sweep({"sedan": 9., "hatchback": 0.}, 300)).params.body_type == "sedan"
    assert posterior.sweeps == 2
    # One decisive informative sweep is not enough for a confident type; a second consecutive one is.
    assert posterior.update(_sweep({"sedan": 9., "hatchback": 0.}, 400)).params.body_type == "sedan"
    assert posterior.update(_sweep({"sedan": 9., "hatchback": 0.}, 500)).params.body_type == "hatchback"


def test_type_posterior_lets_a_tentative_type_follow_the_evidence_but_not_when_sparse():
    posterior = TypePosterior()
    # Sparse or narrowly covered sweeps do not vote; the box-size type (minivan here) is kept meanwhile.
    assert posterior.update(_sweep({"minivan": 5., "pickup": 0.}, 100)).params.body_type == "minivan"
    assert posterior.update(_sweep({"minivan": 5., "pickup": 0.}, 140)).params.body_type == "minivan"  # < 150 voxels
    assert posterior.update(_sweep({"minivan": 5., "pickup": 0.}, 300, cells=8)).params.body_type == "minivan"
    assert posterior.sweeps == 0 and not posterior.log_posterior
    assert posterior.update(_sweep({"minivan": 5., "pickup": 0.}, 400)).params.body_type == "pickup"
    assert not posterior.confident or posterior.sweeps >= 2


def test_screened_out_types_lose_posterior_mass():
    absent, screened = TypePosterior(), TypePosterior()
    for posterior in (absent, screened):
        posterior.update(_sweep({"sedan": 0., "hatchback": 1., "pickup": 1., "van": 1.}, 200))
    # Only sedan and hatchback were refined; a type missing from type_nats counts as the worst refined one.
    absent.update(_sweep({"sedan": 0., "hatchback": 1.}, 300))
    screened.update(_sweep({"sedan": 0., "hatchback": 1., "pickup": 5., "van": 5.}, 300, refined=("sedan", "hatchback")))
    assert screened.confident and not absent.confident


def test_established_type_needs_the_same_challenger_twice():
    posterior = TypePosterior()
    for evidence in (200, 300):
        posterior.update(_sweep({"sedan": 0., "hatchback": 6., "pickup": 6.}, evidence))
    assert posterior.confident and posterior.incumbent == "sedan"
    # Alternating challengers, each far ahead once: the established type stays.
    assert posterior.update(_sweep({"sedan": 12., "hatchback": 0., "pickup": 12.}, 400)).params.body_type == "sedan"
    assert not posterior.confident  # contradicted, but still established
    assert posterior.update(_sweep({"sedan": 30., "hatchback": 30., "pickup": 0.}, 500)).params.body_type == "sedan"
    assert posterior.update(_sweep({"sedan": 30., "hatchback": 30., "pickup": 0.}, 600)).params.body_type == "pickup"


@pytest.mark.parametrize("body_type", ["sedan", "pickup"])
def test_analytic_jacobian_matches_finite_differences(body_type):
    shape, box = _truth(body_type)
    points = _sample(shape, box, 300, (-9., 5., 2.), seed=8)
    fitter = VehicleShapeFitter()
    residuals = _Residuals(_local(points, box), np.ones(len(points)), box, body_type,
                           *fitter.prior_moments(body_type, box), fitter)
    theta = sanitize(shape.array * (1 + .02 * np.sin(np.arange(len(PARAMETERS)))))[0]
    _, analytic = residuals.distance(theta, jacobian=True)
    numeric = np.array([(residuals.distance(theta + step)[0] - residuals.distance(theta - step)[0]) / 2e-6
                        for step in np.eye(len(theta)) * 1e-6])
    close = np.abs(analytic - numeric) <= 1e-3 + .05 * np.abs(numeric)
    # Only the ignored slope-factor derivative on steep walls (cab back, hatch) may differ.
    assert close.mean() > .97


@pytest.mark.parametrize("body_type", TYPE_NAMES)
@pytest.mark.parametrize("sensor", [(-9., 5., 1.9), (0., -8., 1.9)])
def test_one_sided_points_recover_type_and_move_toward_truth(body_type, sensor):
    truth, box = _truth(body_type)
    points = _sample(truth, box, 400, sensor, seed=7)
    fitter = VehicleShapeFitter()
    fit = fitter.fit(points, box)
    assert fit.accepted and not fit.prior_only and fit.cost < .05
    assert fit.params.body_type == body_type
    mean = UniversalVehicleShape(body_type, tuple(sanitize(fitter.prior_moments(body_type, box)[0])[0]))
    assert _error(fit.params, truth, box) < _error(mean, truth, box)


def test_sparse_evidence_falls_back_to_the_dimension_prior_type():
    truth, box = _truth("pickup")
    points = _sample(truth, box, 15, (-9., 5., 1.9))
    fitter = VehicleShapeFitter()
    fit = fitter.fit(points, box)
    assert not fit.accepted and fit.prior_only and fit.evidence_voxels < 35 and np.isnan(fit.cost)
    assert fit.params == fitter.prior_shape(box) and fit.params.body_type == "pickup"
    assert fitter.prior_shape(VEHICLE_TYPES["sedan"].dimension_stats()[0] + INFLATION).body_type == "sedan"
    previous = UniversalVehicleShape.mean("van", box)
    kept = fitter.fit(points, box, previous)
    assert kept.params is previous and not kept.prior_only


def test_unseen_front_keeps_the_prior():
    truth, box = _truth("crossover")
    points = _sample(truth, box, 400, (-9., 3., 1.9), where=lambda centers: centers[:, 0] < -.5)
    fitter = VehicleShapeFitter()
    fit = fitter.fit(points, box, UniversalVehicleShape.mean("crossover", box, inflation_lwh_m=INFLATION),
                     incumbent_cost=1e9)
    mean = sanitize(fitter.prior_moments("crossover", box)[0])[0]
    for name in ("x_windshield", "top_nose", "half_width_nose", "bevel_front"):
        assert fit.params[name] == pytest.approx(mean[PARAMETERS.index(name)], abs=1e-9)


def test_incumbent_only_refits_skip_the_sweep_until_the_cost_grows():
    truth, box = _truth("full_suv")
    points = _sample(truth, box, 400, (-9., 5., 1.9), seed=4)
    previous = UniversalVehicleShape.mean("minivan", box, inflation_lwh_m=INFLATION)
    fitter = VehicleShapeFitter()
    swept = fitter.fit(points, box, previous)
    assert swept.params.body_type == "full_suv" and np.isfinite(swept.robust_cost)
    # A trusted incumbent is refit alone, so the better type is not even tried ...
    kept = fitter.fit(points, box, previous, incumbent_cost=1e9)
    assert kept.params.body_type == "minivan" and kept.robust_cost > swept.robust_cost
    # ... until its cost grows past sweep_cost_growth.
    assert fitter.fit(points, box, previous, incumbent_cost=0.).params.body_type == "full_suv"


def test_fit_latency_at_four_hundred_points():
    truth, box = _truth("pickup")
    points = _sample(truth, box, 2000, (-9., 5., 1.9), seed=5)
    fitter = VehicleShapeFitter()
    first = fitter.fit(points, box)
    sweeps, refits = [], []
    for seed in range(5):
        jittered = points + np.random.default_rng(seed).normal(0, .005, points.shape)
        start = time.perf_counter()
        fitter.fit(jittered, box)
        sweeps.append(time.perf_counter() - start)
        start = time.perf_counter()
        fitter.fit(jittered, box, first.params, incumbent_cost=first.robust_cost)
        refits.append(time.perf_counter() - start)
    # Targets are ~10 ms / ~60 ms; generous bounds keep loaded CI machines green.
    assert np.median(refits) < .04 and np.median(sweeps) < .2

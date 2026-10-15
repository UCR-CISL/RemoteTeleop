"""Vehicle-side universal shape accumulation and fitting from sparse one-sided returns."""
import subprocess
import sys
import time

import numpy as np
import pytest

from src.deployment.vehicle_shape import (ObjectPointAccumulator, VehicleShapeFitter, _Residuals, surface_distance,
                                          triangle_distance)
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


def test_incumbent_type_is_kept_without_a_clear_margin_and_refits_skip_the_sweep():
    truth, box = _truth("full_suv")
    points = _sample(truth, box, 400, (-9., 5., 1.9), seed=4)
    previous = UniversalVehicleShape.mean("minivan", box, inflation_lwh_m=INFLATION)
    assert VehicleShapeFitter(switch_margin=.99).fit(points, box, previous).params.body_type == "minivan"
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

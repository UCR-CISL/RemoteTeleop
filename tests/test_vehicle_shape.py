"""Vehicle-side car shape accumulation and fitting from sparse one-sided returns."""
import subprocess
import sys
import time
from dataclasses import replace

import numpy as np
import pytest
from scipy.optimize import least_squares
from scipy.optimize._numdiff import approx_derivative

from src.deployment.vehicle_shape import CarShapeFitter, ObjectPointAccumulator, _CandidateResiduals, surface_distance
from src.viz.primitive_vehicle import CarShapeParams

DIMENSIONS = np.array([4.7, 1.85, 1.5])
TRUTH = CarShapeParams(windshield_base_x=.35, roof_front_x=.06, roof_rear_x=-.19, rear_window_base_x=-.39,
                       hood_z=.61, deck_z=.55, nose_half_width=.31, tail_half_width=.39, cabin_half_width=.35)


def _sample(shape, count, sensor, *, seed=0, noise=.02, where=None):
    """Area-weighted returns from mesh faces facing a sensor direction on its side."""
    from src.viz.primitive_car import PrimitiveCarAsset

    asset = PrimitiveCarAsset()
    triangles = asset.vertices_for_dimensions(DIMENSIONS, shape)[asset.mesh.faces]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    areas = np.linalg.norm(normals, axis=1)
    sensor = np.asarray(sensor, dtype=float) / np.linalg.norm(sensor)
    centers = triangles.mean(axis=1)
    # Crude occlusion: far-side faces (e.g. inner tire walls) are not seen.
    weight = areas * ((normals / np.maximum(areas, 1e-12)[:, None]) @ sensor > .2) * (
        (centers[:, 1] * sensor[1] >= 0) | (np.abs(centers[:, 1]) < .25 * DIMENSIONS[1]))
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


def _error(shape, truth):
    return np.linalg.norm([(getattr(shape, name) - getattr(truth, name)) / CarShapeFitter.FIT_SPANS[name]
                           for name in CarShapeParams.CONTINUOUS])


def test_module_imports_without_rendering_dependencies():
    code = ("import sys, src.deployment.vehicle_shape; "
            "print([name for name in ('trimesh', 'torch', 'open3d', 'pytorch3d') if name in sys.modules])")
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True)
    assert result.stdout.strip() == "[]"


def test_surface_distance_is_zero_on_profile_and_signed():
    points = np.array([[0., .495 * DIMENSIONS[1], .3 * DIMENSIONS[2]],    # body side
                       [-.06 * DIMENSIONS[0], 0., DIMENSIONS[2]],          # roof
                       [0., .2, .3 * DIMENSIONS[2]],                       # inside
                       [0., .495 * DIMENSIONS[1] + .1, .3 * DIMENSIONS[2]]])
    distance = surface_distance(points, DIMENSIONS, CarShapeParams())
    np.testing.assert_allclose(distance[[0, 1, 3]], [0., 0., .1], atol=1e-3)
    assert distance[2] < -.2


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


@pytest.mark.parametrize("count", [60, 200])
def test_sparse_one_sided_points_move_fit_toward_truth(count):
    pytest.importorskip("trimesh")
    points = _sample(TRUTH, count, (.5, -1., .6))
    fit = CarShapeFitter().fit(points, DIMENSIONS)
    assert fit.accepted and fit.params.body_type == "sedan" and fit.params.forward_sign == 1
    assert fit.cost < .06
    assert _error(fit.params, TRUTH) < .85 * _error(CarShapeParams(), TRUTH)


def test_insufficient_evidence_keeps_prior_or_previous():
    pytest.importorskip("trimesh")
    points = _sample(TRUTH, 20, (.5, -1., .6))
    fit = CarShapeFitter().fit(points, DIMENSIONS)
    assert not fit.accepted and fit.evidence_voxels < 35 and np.isnan(fit.cost)
    assert fit.params == CarShapeParams()
    previous = CarShapeParams.default("hatch_suv")
    assert CarShapeFitter().fit(points, DIMENSIONS, previous).params is previous


def test_unseen_front_is_not_penalized():
    pytest.importorskip("trimesh")
    points = _sample(TRUTH, 300, (-.3, -1., .5), where=lambda centers: centers[:, 0] < 0)
    fit = CarShapeFitter().fit(points, DIMENSIONS)
    assert fit.accepted and fit.cost < .06
    prior = CarShapeParams()
    # Front-only parameters have no data and stay at the preset.
    assert abs(fit.params.windshield_base_x - prior.windshield_base_x) < 5e-3
    assert abs(fit.params.nose_half_width - prior.nose_half_width) < 5e-3
    assert abs(fit.params.rear_window_base_x - TRUTH.rear_window_base_x) < abs(
        prior.rear_window_base_x - TRUTH.rear_window_base_x)


@pytest.mark.parametrize("truth", [CarShapeParams(), replace(CarShapeParams(), forward_sign=-1.),
                                   CarShapeParams.default("hatch_suv"),
                                   replace(CarShapeParams.default("hatch_suv"), forward_sign=-1.)])
def test_body_type_and_forward_sign_recovered(truth):
    pytest.importorskip("trimesh")
    fit = CarShapeFitter().fit(_sample(truth, 400, (0., -1., .4), seed=3), DIMENSIONS)
    assert fit.accepted
    assert (fit.params.body_type, fit.params.forward_sign) == (truth.body_type, truth.forward_sign)


def test_previous_body_type_is_kept_without_a_clear_margin():
    pytest.importorskip("trimesh")
    points = _sample(CarShapeParams(), 400, (0., -1., .4), seed=4)
    previous = replace(CarShapeParams(), forward_sign=-1.)
    assert CarShapeFitter(switch_margin=.99).fit(points, DIMENSIONS, previous).params.forward_sign == -1
    assert CarShapeFitter().fit(points, DIMENSIONS, previous).params.forward_sign == 1


def test_fit_latency_for_two_thousand_points():
    pytest.importorskip("trimesh")
    points = _sample(TRUTH, 2000, (.5, -1., .6), seed=5)
    fitter = CarShapeFitter()
    fitter.fit(points, DIMENSIONS)
    start = time.perf_counter()
    fit = fitter.fit(points, DIMENSIONS)
    assert time.perf_counter() - start < .5
    assert fit.accepted and _error(fit.params, TRUTH) < .5 * _error(CarShapeParams(), TRUTH)


class _ReferenceFitter(CarShapeFitter):
    """Candidates fitted as before vectorization: SciPy's 2-point Jacobian over surface_distance."""

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
        local = np.column_stack([sign * points[:, 0], np.abs(points[:, 1]), points[:, 2] + dimensions[2] / 2])

        def shape(theta):
            return replace(prior, forward_sign=sign, **dict(zip(names, theta)))

        def residuals(theta):
            scaled = surface_distance(local, dimensions, shape(theta)) / self.huber_m
            magnitude = np.abs(scaled)
            robust = np.where(magnitude <= 1, scaled, np.sign(scaled) * np.sqrt(np.maximum(2 * magnitude - 1, 1.)))
            prior_residuals = np.sqrt(self.prior_weight) / (span / 2) * (theta - theta0)
            return np.concatenate([np.sqrt(weights) * robust, prior_residuals])

        result = least_squares(residuals, start, bounds=(lower, upper), x_scale=span, diff_step=1e-3,
                               ftol=self.tolerance, xtol=self.tolerance, gtol=1e-6, max_nfev=self.max_nfev)
        rms = float(np.sqrt(weights @ surface_distance(local, dimensions, shape(result.x)) ** 2 / weights.sum()))
        return shape(result.x), float(result.cost), rms, float("nan")


@pytest.mark.parametrize("truth", [TRUTH, replace(TRUTH, forward_sign=-1.), CarShapeParams.default("hatch_suv"),
                                   replace(CarShapeParams.default("hatch_suv"), forward_sign=-1., hood_z=.6)])
@pytest.mark.parametrize("sensor", [(.5, -1., .6), (0., -1., .4)])
def test_fit_matches_the_finite_difference_reference(truth, sensor):
    pytest.importorskip("trimesh")
    points = _sample(truth, 150, sensor, seed=7)
    fit, reference = CarShapeFitter().fit(points, DIMENSIONS), _ReferenceFitter().fit(points, DIMENSIONS)
    assert fit.accepted and reference.accepted
    assert fit.params.to_vector()[:2] == reference.params.to_vector()[:2]  # body type and forward sign
    assert abs(fit.cost - reference.cost) < 2e-3
    assert max(abs(getattr(fit.params, name) - getattr(reference.params, name)) / CarShapeFitter.FIT_SPANS[name]
               for name in CarShapeParams.CONTINUOUS) < .1


def test_analytic_jacobian_matches_fine_finite_differences():
    pytest.importorskip("trimesh")
    points = _sample(TRUTH, 300, (.5, -1., .6), seed=8)
    local = np.column_stack([points[:, 0], np.abs(points[:, 1]), points[:, 2] + DIMENSIONS[2] / 2])
    span = np.array([CarShapeFitter.FIT_SPANS[name] for name in CarShapeParams.CONTINUOUS])
    for body_type in CarShapeParams.BODY_TYPES:
        theta0 = np.array([getattr(CarShapeParams.default(body_type), name) for name in CarShapeParams.CONTINUOUS])
        problem = _CandidateResiduals(local, np.ones(len(local)), DIMENSIONS, theta0, 2 / span, .08)
        theta = theta0 + np.random.default_rng(1).uniform(-.4, .4, len(theta0)) * span
        expected = approx_derivative(problem.residuals, theta, abs_step=1e-7)
        np.testing.assert_allclose(problem.jacobian(theta), expected, atol=1e-4 * np.abs(expected).max())


def test_incumbent_refit_skips_the_sweep_unless_its_cost_grows():
    pytest.importorskip("trimesh")
    points = _sample(CarShapeParams(), 400, (0., -1., .4), seed=4)
    previous = replace(CarShapeParams(), forward_sign=-1.)
    fitter = CarShapeFitter()
    swept = fitter.fit(points, DIMENSIONS, previous)
    assert swept.params.forward_sign == 1 and np.isfinite(swept.robust_cost)
    # A trusted incumbent is refit alone, so the better reversed hypothesis is not even tried.
    kept = fitter.fit(points, DIMENSIONS, previous, incumbent_cost=1e9)
    assert kept.params.forward_sign == -1 and kept.robust_cost > swept.robust_cost
    # Its cost grew past sweep_cost_growth: the sweep runs and the switch margin applies as usual.
    assert fitter.fit(points, DIMENSIONS, previous, incumbent_cost=kept.robust_cost / 2).params.forward_sign == 1
    incumbent = fitter.fit(points, DIMENSIONS, swept.params, incumbent_cost=swept.robust_cost)
    assert incumbent.params.forward_sign == 1 and abs(incumbent.cost - swept.cost) < 2e-3

import numpy as np
import pytest

pytest.importorskip("open3d")

from src.localization.registration import PointCloudRegistrar, RegistrationConfig


def _transform(yaw_degrees: float, translation: tuple[float, float, float]) -> np.ndarray:
    yaw = np.radians(yaw_degrees)
    transform = np.eye(4)
    transform[:3, :3] = [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
    transform[:3, 3] = translation
    return transform


def _apply(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


def test_refines_initial_pose_and_reports_residual_diagnostics():
    rng = np.random.default_rng(7)
    source = rng.uniform([-4, -3, -1], [4, 3, 2], size=(1500, 3))
    truth = _transform(9.0, (1.2, -0.8, 0.35))
    target = _apply(source, truth) + rng.normal(scale=0.004, size=source.shape)
    initial = _transform(7.0, (1.0, -0.7, 0.3))
    config = RegistrationConfig(
        coarse_voxel_size=0.4,
        refinement_voxel_sizes=(0.4, 0.2),
        max_p95_residual=0.08,
    )

    result = PointCloudRegistrar(config).register(source, target, initial)

    np.testing.assert_allclose(result.map_T_source, truth, atol=0.03)
    assert result.accepted
    assert result.inlier_ratio > 0.95
    assert result.median_residual < 0.03
    assert result.p95_residual < 0.05
    assert result.all_point_median_residual < 0.03
    assert result.all_point_p95_residual < 0.05
    assert 0 < result.correction_translation < 1.0
    assert 0 < result.correction_rotation_degrees < 5.0
    assert result.runtime_seconds >= 0


def test_rejects_large_correction_against_prediction():
    rng = np.random.default_rng(8)
    source = rng.uniform([-3, -3, -1], [3, 3, 2], size=(1000, 3))
    truth = _transform(0.0, (4.0, 0.0, 0.0))
    target = _apply(source, truth)
    config = RegistrationConfig(
        coarse_voxel_size=0.4,
        refinement_voxel_sizes=(0.4, 0.2),
        max_correction_translation=0.5,
    )

    result = PointCloudRegistrar(config).register(source, target, np.eye(4))

    assert not result.accepted
    assert "correction_translation" in result.rejection_reasons


def test_global_fpfh_ransac_produces_a_coarse_candidate():
    rng = np.random.default_rng(9)
    source = rng.uniform([-3, -2, -1], [3, 2, 2], size=(1400, 3))
    truth = _transform(18.0, (0.9, -0.6, 0.2))
    target = _apply(source, truth)
    config = RegistrationConfig(
        coarse_voxel_size=0.4,
        refinement_voxel_sizes=(0.4, 0.2),
        ransac_max_iterations=50_000,
        max_correction_translation=10.0,
        max_correction_rotation_degrees=180.0,
    )

    result = PointCloudRegistrar(config).register(source, target)

    assert result.coarse_inlier_ratio is not None
    assert result.correction_translation == 0.0
    assert result.correction_rotation_degrees == 0.0
    np.testing.assert_allclose(result.map_T_source, truth, atol=0.08)


def test_zero_inliers_reports_infinite_accepted_residuals_and_finite_all_point_metrics():
    rng = np.random.default_rng(10)
    source = rng.uniform([-1, -1, -1], [1, 1, 1], size=(500, 3))
    target = source + np.array([100.0, 0.0, 0.0])
    config = RegistrationConfig(
        refinement_voxel_sizes=(0.4, 0.2),
        max_median_residual=1.0,
        max_p95_residual=1.0,
    )

    result = PointCloudRegistrar(config).register(source, target, np.eye(4))

    assert result.inlier_ratio == 0.0
    assert np.isinf(result.median_residual)
    assert np.isinf(result.p90_residual)
    assert np.isinf(result.p95_residual)
    assert np.isfinite(result.all_point_median_residual)
    assert np.isfinite(result.all_point_p90_residual)
    assert np.isfinite(result.all_point_p95_residual)
    assert not result.accepted
    assert {"inlier_ratio", "median_residual", "p95_residual"} <= set(result.rejection_reasons)

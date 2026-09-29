import numpy as np

from src.localization.enu_prior_fusion import EnuPriorFuser, EnuPriorFusionConfig


def _yaw_transform(yaw_degrees, translation):
    yaw = np.radians(yaw_degrees)
    transform = np.eye(4)
    transform[:3, :3] = [
        [np.cos(yaw), -np.sin(yaw), 0.0],
        [np.sin(yaw), np.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ]
    transform[:3, 3] = translation
    return transform


def test_fusion_pulls_scan_alignment_toward_enu_pose_prior():
    rng = np.random.default_rng(4)
    source = rng.uniform([-4, -3, -1], [4, 3, 2], size=(1000, 3))
    scan = _yaw_transform(2.0, (0.3, -0.2, 0.1))
    target = source @ scan[:3, :3].T + scan[:3, 3]
    prior = np.eye(4)
    source_T_lidar = np.eye(4)
    source_T_lidar[0, 3] = -10.0
    scan_prior_error = np.linalg.norm(
        (scan @ source_T_lidar)[:2, 3] - (prior @ source_T_lidar)[:2, 3]
    )

    result = EnuPriorFuser(EnuPriorFusionConfig(
        prior_weight=0.5,
        heading_lever_arm_m=10.0,
        max_correspondence_distance_m=2.0,
    )).fuse(source, target, scan, prior, source_T_lidar)

    assert result.converged
    assert result.correspondence_count > 200
    assert result.prior_bev_error_m < scan_prior_error
    assert abs(result.prior_heading_error_deg) < 2.0
    np.testing.assert_allclose(result.map_T_source[2, 3], scan[2, 3])
    np.testing.assert_allclose(result.map_T_source[2, :3], scan[2, :3])
    np.testing.assert_allclose(result.map_T_source, [
        [0.9999923619358674, 0.0039084613245014, 0.0, 0.1866709488824557],
        [-0.0039084613245014, 0.9999923619358674, 0.0, -0.1256868376250560],
        [0.0, 0.0, 1.0, 0.1],
        [0.0, 0.0, 0.0, 1.0],
    ], atol=1e-12)
    assert result.weighting_mode == "fixed"
    assert result.estimated_scan_residual_std_m is None
    assert result.effective_prior_translation_weight == 0.5


def test_uncertainty_weighting_uses_scan_residual_and_distinct_pose_uncertainties():
    rng = np.random.default_rng(7)
    source = rng.uniform([-20, -20, -1], [20, 20, 2], size=(6000, 3))
    scan = _yaw_transform(1.0, (0.2, -0.1, 0.0))
    target = source @ scan[:3, :3].T + scan[:3, 3]
    target += rng.normal(0.0, 0.08, target.shape)

    result = EnuPriorFuser(EnuPriorFusionConfig(
        weighting_mode="uncertainty",
        prior_translation_std_m=0.10,
        prior_heading_std_deg=2.0,
        heading_lever_arm_m=10.0,
        max_correspondence_distance_m=2.0,
        source_stride=1,
        uncertainty_block_size_m=4.0,
        uncertainty_min_block_points=10,
    )).fuse(source, target, scan, np.eye(4), np.eye(4))

    assert result.converged
    assert result.weighting_mode == "uncertainty"
    assert result.estimated_scan_residual_std_m > 0.05
    assert result.uncertainty_total_block_count >= result.uncertainty_eligible_block_count >= 6
    assert result.data_hessian_eigenvalues.shape == (3,)
    assert result.data_hessian_condition_number >= 1.0
    assert result.scan_transform_covariance.shape == (3, 3)
    assert result.prior_lidar_pose_covariance.shape == (3, 3)
    assert result.posterior_transform_covariance.shape == (3, 3)
    assert np.linalg.eigvalsh(result.scan_transform_covariance).min() > 0
    assert result.innovation_nis >= 0
    assert isinstance(result.innovation_exceeds_nominal_chi2_95pct, bool)
    assert result.effective_prior_translation_weight is None

    weaker_prior = EnuPriorFuser(EnuPriorFusionConfig(
        weighting_mode="uncertainty",
        prior_translation_std_m=0.50,
        prior_heading_std_deg=5.0,
        heading_lever_arm_m=10.0,
        max_correspondence_distance_m=2.0,
        source_stride=1,
        uncertainty_block_size_m=4.0,
        uncertainty_min_block_points=10,
    )).fuse(source, target, scan, np.eye(4), np.eye(4))

    assert result.prior_bev_error_m < weaker_prior.prior_bev_error_m


def test_fixed_xyz_mode_fuses_height_while_bev_mode_preserves_scan_height():
    rng = np.random.default_rng(12)
    source = rng.uniform([-4, -3, -1], [4, 3, 2], size=(1200, 3))
    scan = _yaw_transform(1.0, (0.2, -0.1, 0.4))
    target = source @ scan[:3, :3].T + scan[:3, 3]

    bev = EnuPriorFuser(EnuPriorFusionConfig(
        prior_weight=0.5,
        max_correspondence_distance_m=2.0,
    )).fuse(source, target, scan, np.eye(4), np.eye(4))
    xyz = EnuPriorFuser(EnuPriorFusionConfig(
        prior_weight=0.5,
        position_mode="xyz",
        max_correspondence_distance_m=2.0,
    )).fuse(source, target, scan, np.eye(4), np.eye(4))

    np.testing.assert_allclose(bev.map_T_source[2, 3], 0.4)
    assert abs(xyz.map_T_source[2, 3]) < abs(bev.map_T_source[2, 3])
    assert xyz.prior_vertical_error_m == abs(xyz.map_T_source[2, 3])
    assert xyz.prior_translation_error_m > xyz.prior_bev_error_m
    assert xyz.position_mode == "xyz"


def test_uncertainty_xyz_mode_reports_four_dimensional_covariance():
    rng = np.random.default_rng(13)
    source = rng.uniform([-20, -20, -2], [20, 20, 3], size=(6000, 3))
    scan = _yaw_transform(1.0, (0.2, -0.1, 0.3))
    target = source @ scan[:3, :3].T + scan[:3, 3]
    target += rng.normal(0.0, 0.08, target.shape)

    result = EnuPriorFuser(EnuPriorFusionConfig(
        weighting_mode="uncertainty",
        position_mode="xyz",
        prior_translation_std_m=0.10,
        prior_vertical_std_m=0.15,
        prior_heading_std_deg=2.0,
        max_correspondence_distance_m=2.0,
        source_stride=1,
        uncertainty_block_size_m=4.0,
        uncertainty_min_block_points=10,
    )).fuse(source, target, scan, np.eye(4), np.eye(4))

    assert result.converged
    assert result.position_mode == "xyz"
    assert result.prior_vertical_error_m is not None
    assert result.prior_translation_error_m is not None
    assert result.scan_transform_covariance.shape == (4, 4)
    assert result.prior_lidar_pose_covariance.shape == (4, 4)
    assert result.posterior_transform_covariance.shape == (4, 4)

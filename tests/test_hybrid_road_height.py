import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from src.localization.hybrid_road_height import HybridRoadHeightEstimator


def road_points(slope=.04):
    x, y = np.meshgrid(np.linspace(-14, 14, 45), np.linspace(-14, 14, 45))
    return np.column_stack((x.ravel(), y.ravel(), -2 + slope*x.ravel()))


def camera_extrinsic():
    transform = np.eye(4)
    transform[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    transform[:3, 3] = [.8, .3, -.25]
    return transform


@pytest.mark.parametrize('angles', [[0, 0, 0], [90, 3, 27]])
@pytest.mark.parametrize('height_error', [-8., 1.25, 8.])
def test_corrects_only_frozen_up_with_native_map_axes_and_large_initial_height(angles, height_error):
    source = road_points()
    rotation = Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
    up = rotation[:, 2]
    true_pose = np.eye(4); true_pose[:3, :3] = rotation; true_pose[:3, 3] = [3, -4, 10]
    initial = true_pose.copy(); initial[:3, 3] += up * height_error
    target = source @ rotation.T + true_pose[:3, 3]
    normal = rotation @ np.array([-.04, 0, 1.]); normal /= np.linalg.norm(normal)
    result = HybridRoadHeightEstimator().estimate(source, map_T_lidar=initial, lidar_T_camera=camera_extrinsic(), map_up=up,
                                                 map_points=target, map_normals=np.tile(normal, (len(target), 1)))
    assert result.accepted, result.diagnostics
    np.testing.assert_allclose(result.map_T_lidar, true_pose, atol=1e-10)
    np.testing.assert_array_equal(result.map_T_lidar[:3, :3], initial[:3, :3])
    assert result.diagnostics['height_shift_m'] == pytest.approx(-height_error, abs=1e-10)
    assert result.diagnostics['transverse_translation_error_m'] < 1e-10
    assert result.diagnostics['original_registration_gates_applied'] is False
    # Includes the horizontal camera lever arm on sloped ground, not just dz.
    expected = (2 - .04*.8 - .25) / np.sqrt(1 + .04**2)
    assert result.diagnostics['camera_normal_clearance_m'] == pytest.approx(expected, abs=1e-10)
    assert result.diagnostics['camera_vertical_clearance_m'] == pytest.approx(2 - .04*.8 - .25, abs=1e-10)


def test_competing_map_layers_are_rejected_without_selecting_nearest_height():
    source = road_points(0)
    target = np.vstack([source, source + [0, 0, 2]])
    result = HybridRoadHeightEstimator().estimate(source, map_T_lidar=np.eye(4), lidar_T_camera=camera_extrinsic(), map_up=[0, 0, 1], map_points=target)
    assert not result.accepted
    assert result.reason == 'ambiguous_map_ground_layers'
    assert result.candidate_map_T_lidar is None
    np.testing.assert_array_equal(result.map_T_lidar, np.eye(4))


def test_dynamic_mask_excludes_large_flat_dynamic_surface():
    road = road_points()
    dynamic = np.tile(road + [0, 0, 1.2], (2, 1))
    source = np.vstack([road, dynamic])
    mask = np.r_[np.zeros(len(road), dtype=bool), np.ones(len(dynamic), dtype=bool)]
    result = HybridRoadHeightEstimator().estimate(source, map_T_lidar=np.eye(4), lidar_T_camera=camera_extrinsic(), map_up=[0, 0, 1], map_points=road, dynamic_mask=mask)
    assert result.accepted
    assert result.diagnostics['source_after_dynamic_finite_count'] == len(road)
    assert result.diagnostics['dynamic_mask_used'] is True
    assert result.diagnostics['height_shift_m'] == pytest.approx(0, abs=1e-10)


def test_normal_disagreement_is_not_hidden_by_changing_roll_pitch():
    source = road_points(0)
    target = source.copy(); target[:, 2] += np.tan(np.deg2rad(8))*target[:, 0]
    result = HybridRoadHeightEstimator().estimate(source, map_T_lidar=np.eye(4), lidar_T_camera=camera_extrinsic(), map_up=[0, 0, 1], map_points=target)
    assert not result.accepted
    assert result.reason == 'insufficient_or_incompatible_map_ground'
    assert result.diagnostics['map_layers'][0]['normal_disagreement_deg'] == pytest.approx(8)
    np.testing.assert_array_equal(result.map_T_lidar, np.eye(4))


def test_insufficient_lidar_or_narrow_map_support_retains_initial_pose():
    estimator = HybridRoadHeightEstimator()
    source = road_points()
    for scan, target, reason in [(source[:2], source, 'insufficient_lidar_ground'),
                                 (source, source[np.abs(source[:, 1]) < 1], 'insufficient_or_incompatible_map_ground')]:
        result = estimator.estimate(scan, map_T_lidar=np.eye(4), lidar_T_camera=camera_extrinsic(), map_up=[0, 0, 1], map_points=target)
        assert not result.accepted
        assert result.reason == reason
        np.testing.assert_array_equal(result.map_T_lidar, np.eye(4))


def test_narrow_layer_does_not_hide_a_later_ambiguous_broad_layer():
    source = road_points(0)
    x, y = np.meshgrid(np.linspace(-14, 14, 100), np.linspace(-1, 1, 10))
    narrow = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    x, y = np.meshgrid(np.linspace(-14, 14, 20), np.linspace(-14, 14, 20))
    upper = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, 2.)))
    result = HybridRoadHeightEstimator().estimate(source, map_T_lidar=np.eye(4), lidar_T_camera=camera_extrinsic(), map_up=[0, 0, 1], map_points=np.vstack([source, narrow, upper]))
    assert not result.accepted
    assert result.reason == 'ambiguous_map_ground_layers'
    assert any(not row['fit']['extent_accepted'] for row in result.diagnostics['map_layers'])
    assert sum(row['plausible'] for row in result.diagnostics['map_layers']) == 2


def test_layer_search_cap_cannot_silently_accept_first_plane():
    from src.localization.hybrid_road_height import HybridRoadHeightConfig
    source = road_points(0)
    estimator = HybridRoadHeightEstimator(HybridRoadHeightConfig(max_layers=1))
    result = estimator.estimate(source, map_T_lidar=np.eye(4), lidar_T_camera=camera_extrinsic(), map_up=[0, 0, 1], map_points=np.vstack([source, source + [0, 0, 2]]))
    assert not result.accepted
    assert result.reason == 'unresolved_map_ground_layers'
    assert result.diagnostics['map_layer_search_truncated'] is True

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from src.localization.bev_lidar_height import BevLidarHeightConfig, BevLidarHeightEstimator
from src.localization.ground_plane import GroundPlane


def estimator(mode='measured'):
    return BevLidarHeightEstimator(BevLidarHeightConfig(ground_plane_policy='synthetic known physical road', clearance_mode=mode))


@pytest.mark.parametrize('angles', [[0, 0, 0], [90, 3, 27]])
def test_height_only_proposal_in_native_map_axes_and_full_camera_extrinsic(angles):
    rotation = Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
    up = rotation[:, 2]
    plane = GroundPlane(tuple(up), 2.8)
    pose = np.eye(4); pose[:3, :3] = rotation; pose[:3, 3] = [8, 4, 3]
    extrinsic = np.eye(4)
    extrinsic[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    extrinsic[:3, 3] = [.9, .2, -.3]
    result = estimator().estimate(map_T_lidar=pose, map_up=up, ground_plane=plane,
                                  lidar_normal_clearance_m=2., source_fit_accepted=True, lidar_T_camera=extrinsic)
    assert result.proposal_valid and not result.applied and not result.accepted
    np.testing.assert_array_equal(result.applied_map_T_lidar, pose)
    np.testing.assert_array_equal(result.proposed_map_T_lidar[:3, :3], pose[:3, :3])
    delta = result.proposed_map_T_lidar[:3, 3] - pose[:3, 3]
    np.testing.assert_allclose((np.eye(3)-np.outer(up, up)) @ delta, 0, atol=1e-12)
    assert up @ result.proposed_map_T_lidar[:3, 3] + 2.8 == pytest.approx(2)
    assert result.diagnostics['camera_normal_clearance_m'] == pytest.approx(1.7)
    np.testing.assert_allclose(result.proposed_map_T_camera[:3, 3]-(pose@extrinsic)[:3, 3], delta, atol=1e-12)
    np.testing.assert_array_equal(result.proposed_map_T_camera[:3, :3], (pose@extrinsic)[:3, :3])


def test_sloped_plane_uses_normal_clearance_divided_by_frozen_up_projection():
    normal = np.array([.1, 0, 1]); normal /= np.linalg.norm(normal)
    pose = np.eye(4); pose[:3, 3] = [5, 0, 3]
    result = estimator().estimate(map_T_lidar=pose, map_up=[0, 0, 1], ground_plane=GroundPlane(tuple(normal), 2),
                                  lidar_normal_clearance_m=1.8, source_fit_accepted=True)
    expected_z = (1.8 - 2 - 5*normal[0])/normal[2]
    assert result.proposed_map_T_lidar[2, 3] == pytest.approx(expected_z)
    np.testing.assert_array_equal(result.proposed_map_T_lidar[:2, 3], pose[:2, 3])


def test_each_original_gate_is_required_and_rejection_resets_camera_after_success():
    pose = np.eye(4); camera = np.eye(4); camera[:3, 3] = [.8, .2, -.3]
    result = estimator().estimate(map_T_lidar=pose, map_up=[0, 0, 1], ground_plane=GroundPlane((0, 0, 1), 2.8),
                                  lidar_normal_clearance_m=2, source_fit_accepted=True, lidar_T_camera=camera)
    accepted = result.with_gate_results(upstream_accepted=True, registration_accepted=True)
    assert accepted.applied and accepted.accepted
    for upstream, registration in [(False, True), (True, False), (False, False)]:
        rejected = accepted.with_gate_results(upstream_accepted=upstream, registration_accepted=registration)
        assert not rejected.applied and not rejected.accepted
        np.testing.assert_array_equal(rejected.applied_map_T_lidar, pose)
        np.testing.assert_array_equal(rejected.applied_map_T_camera, camera)


def test_rejected_measurement_only_uses_explicitly_accepted_nominal_when_requested():
    args = dict(map_T_lidar=np.eye(4), map_up=[0, 0, 1], ground_plane=GroundPlane((0, 0, 1), 2.8),
                lidar_normal_clearance_m=None, source_fit_accepted=False, nominal_normal_clearance_m=1.9)
    assert not estimator().estimate(**args, nominal_fit_accepted=True).proposal_valid
    assert not estimator('measured_or_nominal').estimate(**args).proposal_valid
    result = estimator('measured_or_nominal').estimate(**args, nominal_fit_accepted=True)
    assert result.proposal_valid
    assert result.diagnostics['clearance_source'] == 'nominal'
    assert result.proposed_map_T_lidar[2, 3] == pytest.approx(-.9)
    assert not result.with_gate_results(upstream_accepted=False, registration_accepted=True).applied


@pytest.mark.parametrize('normal', [(1, 0, 0), (0, 0, -1)])
def test_bad_or_reversed_plane_conditioning_keeps_pose(normal):
    result = estimator().estimate(map_T_lidar=np.eye(4), map_up=[0, 0, 1], ground_plane=GroundPlane(normal, 2.8),
                                  lidar_normal_clearance_m=2, source_fit_accepted=True)
    assert not result.proposal_valid
    assert result.reason == 'ground_normal_up_misaligned_or_ill_conditioned'
    np.testing.assert_array_equal(result.applied_map_T_lidar, np.eye(4))


def test_proposed_height_does_not_inherit_initial_height_or_any_reference_target():
    proposals = []
    for initial_height in [-8., 8.]:
        pose = np.eye(4); pose[:3, 3] = [3, 7, initial_height]
        result = estimator().estimate(map_T_lidar=pose, map_up=[0, 0, 1], ground_plane=GroundPlane((0, 0, 1), 2.8),
                                      lidar_normal_clearance_m=1.9, source_fit_accepted=True)
        proposals.append(result.proposed_map_T_lidar)
    np.testing.assert_allclose(proposals[0], proposals[1], atol=1e-12)
    assert proposals[0][2, 3] == pytest.approx(-.9)

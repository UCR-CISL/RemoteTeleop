import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from src.localization.cooperscene_reference import CooperSceneReference


def test_sensor_extrinsic_composes_in_body_frame_with_rotated_lever_arm():
    extrinsic = np.eye(4)
    extrinsic[:3, :3] = Rotation.from_euler('x', 25, degrees=True).as_matrix()
    extrinsic[:3, 3] = [1, 0, 0]
    map_T_enu = np.eye(4)
    map_T_enu[:3, 3] = [100, 200, 300]
    reference = CooperSceneReference((0, 0, 0), map_T_enu, extrinsic)
    q = Rotation.from_euler('z', 90, degrees=True).as_quat()
    metadata = dict(Latitude=0, Longitude=0, Altitude=0,
                    Quat_qx=q[0], Quat_qy=q[1], Quat_qz=q[2], Quat_qw=q[3])
    pose = reference.map_T_lidar(metadata)
    np.testing.assert_allclose(pose[:3, 3], [100, 199, 300])
    # A point transformed LiDAR -> Xsens -> map must agree with the pose.
    point_lidar = np.array([2, 3, 4, 1])
    point_xsens = np.linalg.solve(extrinsic, point_lidar)
    expected = map_T_enu @ np.r_[Rotation.from_quat(q).apply(point_xsens[:3]), 1]
    np.testing.assert_allclose(pose @ point_lidar, expected)
    assert not np.allclose(pose[:3, :3], extrinsic[:3, :3].T @ Rotation.from_quat(q).as_matrix())


def test_raw_lla_has_priority_over_altered_enu_fields():
    reference = CooperSceneReference((0, 0, 0), np.eye(4), np.eye(4))
    metadata = dict(Latitude=0, Longitude=0, Altitude=10, enu_e=100, enu_n=200, enu_u=300)
    np.testing.assert_allclose(reference.enu_position(metadata), [0, 0, 10], atol=1e-8)


def test_reflection_is_not_a_valid_sensor_extrinsic():
    with pytest.raises(ValueError, match='rigid'):
        CooperSceneReference((0, 0, 0), np.eye(4), np.diag([-1, 1, 1, 1]))


def test_release_attitude_preprocessing_is_removed_before_body_composition():
    extrinsic = np.eye(4)
    extrinsic[:3, :3] = Rotation.from_euler('xyz', [7, -3, 11], degrees=True).as_matrix()
    extrinsic[:3, 3] = [.2, -.1, .3]
    raw_rotation = Rotation.from_euler('xyz', [2, 5, 80], degrees=True).as_matrix()
    # Raw metadata below is already normalized to the release's device axes.
    raw_quat = Rotation.from_matrix(raw_rotation).as_quat()
    stored_quat = Rotation.from_matrix(extrinsic[:3,:3] @ raw_rotation).as_quat()
    raw = dict(Latitude=0, Longitude=0, Altitude=10)
    raw.update(zip(('Quat_qx','Quat_qy','Quat_qz','Quat_qw'), raw_quat))
    stored = dict(raw)
    stored.update(zip(('Quat_qx','Quat_qy','Quat_qz','Quat_qw'), stored_quat))
    unchanged = dict(stored)
    reference = CooperSceneReference((0,0,0), np.eye(4), extrinsic)
    np.testing.assert_allclose(reference.map_T_lidar_from_release_metadata(stored),
                               reference.map_T_lidar(raw), atol=1e-12)
    assert stored == unchanged
    assert not np.allclose(reference.map_T_lidar(stored), reference.map_T_lidar(raw))

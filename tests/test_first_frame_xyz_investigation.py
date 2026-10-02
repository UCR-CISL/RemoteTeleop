"""Coordinate checks for offline investigation metrics and annotation masks."""
import numpy as np
from scipy.spatial.transform import Rotation

from scripts.investigation.first_frame_xyz import pose_error, source_points


def test_rotated_dynamic_box_mask_uses_dataset_coordinates_and_preserves_static_classes():
    box = np.eye(4)
    box[:3, :3] = Rotation.from_euler("z", 45, degrees=True).as_matrix()
    box[:3, 3] = (3, 4, 1)
    local_points = np.array([[0.4, 0, 0], [1.15, 0, 0], [1.3, 0, 0]])
    world_points = local_points @ box[:3, :3].T + box[:3, 3]
    barrier = np.eye(4)
    barrier[:3, 3] = (-3, -4, 1)
    world_points = np.vstack((world_points, barrier[:3, 3]))
    lidar_pose = np.eye(4)
    lidar_pose[:3, :3] = Rotation.from_euler("z", 90, degrees=True).as_matrix()
    lidar_pose[:3, 3] = (10, 20, 3)
    frame = dict(
        dataset_T_lidar=lidar_pose,
        points_lidar=(world_points - lidar_pose[:3, 3]) @ lidar_pose[:3, :3],
        boxes_T_dataset_object=np.stack((box, barrier)),
        boxes_dimensions_lwh=np.array([[2, 2, 2], [2, 2, 2]]),
        box_labels=np.array(["vehicle.car", "movable_object.barrier"]),
    )
    np.testing.assert_allclose(source_points(frame, True), world_points[[2, 3]])
    np.testing.assert_allclose(source_points(frame, False), world_points)


def test_pose_errors_resolve_translation_and_rotation_in_dataset_z_up_axes():
    reference = np.eye(4)
    reference[:3, :3] = Rotation.from_euler("xyz", [90, 0, 40], degrees=True).as_matrix()
    reference[:3, 3] = (10, -5, 7)
    lidar_pose = np.eye(4)
    lidar_pose[:3, :3] = Rotation.from_euler("xyz", [10, -2, 90], degrees=True).as_matrix()
    lidar_pose[:3, 3] = (50, 20, 2)
    estimated_pose = reference @ lidar_pose
    estimated_pose[:3, 3] += reference[:3, :3] @ np.array([0.1, -0.2, 0.3])
    estimated_pose[:3, :3] = (reference[:3, :3]
        @ Rotation.from_euler("z", 7, degrees=True).as_matrix() @ lidar_pose[:3, :3])
    error = pose_error(estimated_pose @ np.linalg.inv(lidar_pose), reference, lidar_pose)
    np.testing.assert_allclose([error[k] for k in ("dx_m", "dy_m", "dz_m")], [.1, -.2, .3], atol=1e-12)
    np.testing.assert_allclose([error[k] for k in ("roll_deg", "pitch_deg", "yaw_deg")], [0, 0, 7], atol=1e-12)
    np.testing.assert_allclose([error["xy_m"], error["xyz_m"]], [np.sqrt(.05), np.sqrt(.14)], atol=1e-12)
    assert error["independent_ground_truth"] is False

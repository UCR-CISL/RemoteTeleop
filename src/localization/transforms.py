"""Frame-composition helpers used by localization consumers."""

from __future__ import annotations

import numpy as np


def compose_map_T_camera(
    map_T_lidar: np.ndarray, lidar_T_camera: np.ndarray
) -> np.ndarray:
    """Compose a localized LiDAR pose and rigid LiDAR-to-camera calibration.

    Transform names use the project's ``target_T_source`` convention.  Thus
    ``lidar_T_camera`` maps camera coordinates into the LiDAR frame and the
    result maps camera coordinates into the Gaussian-map frame.
    """

    localized_pose = _rigid_transform(map_T_lidar, "map_T_lidar")
    calibration = _rigid_transform(lidar_T_camera, "lidar_T_camera")
    return localized_pose @ calibration


def _rigid_transform(values: np.ndarray, name: str) -> np.ndarray:
    transform = np.asarray(values, dtype=np.float64)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError(f"{name} must be a finite 4x4 homogeneous transform")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError(f"{name} must be a homogeneous rigid transform")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not np.isclose(
        np.linalg.det(rotation), 1.0, atol=1e-6
    ):
        raise ValueError(f"{name} rotation must be orthonormal and right-handed")
    return transform

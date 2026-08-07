"""Documented front-camera calibration for CooperScene vehicle agents."""

from __future__ import annotations

import numpy as np

# Copied from CooperScene's dataset converter (SensorPlatform calibration).
# The extrinsics are camera_T_lidar in CV camera coordinates (right, down,
# forward), not CARLA sensor coordinates.
CAMERA_CALIBRATION = {
    "1": ([[1810.6983377214349, 0.0, 970.0037900558976], [0.0, 1808.4746114412712, 526.7243033929532], [0.0, 0.0, 1.0]], [[1.97675856e-02, -9.98998871e-01, -4.01310140e-02, -1.14236290e-04], [-4.47204289e-02, 3.92152089e-02, -9.98229558e-01, -1.75248879e-01], [9.98803948e-01, 2.15272644e-02, -4.39004680e-02, -5.25987245e-02], [0.0, 0.0, 0.0, 1.0]]),
    "2": ([[1807.294188309111, 0.0, 959.1274184107824], [0.0, 1805.1344921703185, 562.0235332672737], [0.0, 0.0, 1.0]], [[0.00383388, -0.99998521, -0.00385735, -0.0013977], [-0.00271154, 0.00384697, -0.99998892, -0.23015363], [0.99998897, 0.0038443, -0.00269675, -0.07767752], [0.0, 0.0, 0.0, 1.0]]),
    "3": ([[1814.1641900346326, 0.0, 951.718698093622], [0.0, 1812.1352460466578, 568.1954332460546], [0.0, 0.0, 1.0]], [[0.064023, -0.99792386, -0.00700149, -0.00332843], [-0.01444825, 0.00608825, -0.99987708, -0.26612881], [0.99784383, 0.06411629, -0.01402846, -0.0864176], [0.0, 0.0, 0.0, 1.0]]),
}


def camera_intrinsic(agent: str | int) -> np.ndarray:
    """Return the documented camera0 intrinsic matrix for vehicle ``agent``."""

    try:
        intrinsic, _ = CAMERA_CALIBRATION[str(agent)]
    except KeyError as error:
        raise ValueError(f"CooperScene agent {agent} has no front camera calibration") from error
    return np.asarray(intrinsic, dtype=np.float64)


def camera_T_lidar(agent: str | int) -> np.ndarray:
    """Return the documented CV-camera-from-LiDAR transform for ``agent``."""

    try:
        _, transform = CAMERA_CALIBRATION[str(agent)]
    except KeyError as error:
        raise ValueError(f"CooperScene agent {agent} has no front camera calibration") from error
    return np.asarray(transform, dtype=np.float64)


def compose_map_T_camera(map_T_lidar: np.ndarray, lidar_to_camera: np.ndarray) -> np.ndarray:
    """Compose camera pose from ``map_T_lidar`` and ``camera_T_lidar``."""

    map_T_lidar = np.asarray(map_T_lidar, dtype=np.float64)
    lidar_to_camera = np.asarray(lidar_to_camera, dtype=np.float64)
    if map_T_lidar.shape != (4, 4) or lidar_to_camera.shape != (4, 4):
        raise ValueError("map_T_lidar and lidar_to_camera must both be 4x4 transforms")
    return map_T_lidar @ np.linalg.inv(lidar_to_camera)

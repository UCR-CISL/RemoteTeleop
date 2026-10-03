"""Independent CooperScene pose reconstruction from raw INS/GNSS metadata.

This is an explicit offline reference utility. Calibration and map origins must
be supplied; it does not alter the native dataset loader or runtime poses.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class CooperSceneReference:
    """Compose ``map_T_xsens @ xsens_T_lidar`` at the raw GNSS timestamp.

    The caller must establish the altitude datum and whether the recorded GNSS
    position denotes the Xsens origin. No capture-time interpolation is implied.
    """

    origin_lla: tuple[float, float, float]
    map_T_enu: np.ndarray
    lidar_T_xsens: np.ndarray

    def __post_init__(self) -> None:
        origin = tuple(float(value) for value in self.origin_lla)
        if len(origin) != 3 or not np.all(np.isfinite(origin)):
            raise ValueError('origin_lla must contain three finite values')
        object.__setattr__(self, 'origin_lla', origin)
        for name in ('map_T_enu', 'lidar_T_xsens'):
            matrix = np.array(getattr(self, name), dtype=float, copy=True)
            if (matrix.shape != (4, 4) or not np.all(np.isfinite(matrix))
                    or not np.allclose(matrix[3], [0, 0, 0, 1])
                    or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6)
                    or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-6)):
                raise ValueError(f'{name} must be a rigid homogeneous transform')
            matrix.flags.writeable = False
            object.__setattr__(self, name, matrix)

    @staticmethod
    def _ecef(lla: tuple[float, float, float]) -> np.ndarray:
        latitude, longitude = np.radians(lla[:2])
        height = lla[2]
        radius, eccentricity_squared = 6378137., 6.6943799901413165e-3
        normal = radius / np.sqrt(1 - eccentricity_squared * np.sin(latitude)**2)
        return np.array([(normal + height) * np.cos(latitude) * np.cos(longitude),
                         (normal + height) * np.cos(latitude) * np.sin(longitude),
                         (normal * (1 - eccentricity_squared) + height) * np.sin(latitude)])

    def enu_position(self, metadata: Mapping[str, float]) -> np.ndarray:
        """Recompute ENU from raw LLA; ignore the archive's altered enu_* fields."""
        lla = tuple(float(metadata[key]) for key in ('Latitude', 'Longitude', 'Altitude'))
        if not np.all(np.isfinite(lla)):
            raise ValueError('raw LLA must be finite')
        latitude, longitude = np.radians(self.origin_lla[:2])
        basis = np.array([[-np.sin(longitude), np.cos(longitude), 0],
                          [-np.sin(latitude)*np.cos(longitude), -np.sin(latitude)*np.sin(longitude), np.cos(latitude)],
                          [np.cos(latitude)*np.cos(longitude), np.cos(latitude)*np.sin(longitude), np.sin(latitude)]])
        return basis @ (self._ecef(lla) - self._ecef(self.origin_lla))

    def map_T_xsens(self, metadata: Mapping[str, float]) -> np.ndarray:
        pose = np.eye(4)
        pose[:3, 3] = self.enu_position(metadata)
        pose[:3, :3] = Rotation.from_quat(
            [metadata[key] for key in ('Quat_qx', 'Quat_qy', 'Quat_qz', 'Quat_qw')]
        ).as_matrix()
        return self.map_T_enu @ pose

    def map_T_lidar(self, metadata: Mapping[str, float]) -> np.ndarray:
        """Apply sensor calibration on the right, including the rotated lever arm."""
        return self.map_T_xsens(metadata) @ np.linalg.inv(self.lidar_T_xsens)

"""Coordinate conventions and rigid transforms used across the MVP.

Transforms are named ``target_T_source`` and map source-frame homogeneous
coordinates into the target frame.  Project/world/object frames are
right-handed, use x-forward, y-left, z-up, and measure distance in meters.
Quaternions at public boundaries are ordered ``[x, y, z, w]``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


HANDEDNESS = "right"
FORWARD_AXIS = "+x"
LEFT_AXIS = "+y"
UP_AXIS = "+z"
LENGTH_UNIT = "meter"
QUATERNION_ORDER = "xyzw"


def _immutable_vector(values: Sequence[float], *, length: int, name: str) -> np.ndarray:
    vector = np.array(values, dtype=np.float64, copy=True)
    if vector.shape != (length,):
        raise ValueError(f"{name} must have shape ({length},), got {vector.shape}")
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain only finite values")
    vector.flags.writeable = False
    return vector


@dataclass(frozen=True, eq=False)
class Pose3D:
    """A metric rigid pose with an xyzw quaternion."""

    translation: np.ndarray
    quaternion_xyzw: np.ndarray

    def __post_init__(self) -> None:
        translation = _immutable_vector(self.translation, length=3, name="translation")
        quaternion = _immutable_vector(self.quaternion_xyzw, length=4, name="quaternion_xyzw")
        norm = float(np.linalg.norm(quaternion))
        if norm <= np.finfo(np.float64).eps:
            raise ValueError("quaternion_xyzw must not be zero")
        quaternion = quaternion / norm
        quaternion.flags.writeable = False
        object.__setattr__(self, "translation", translation)
        object.__setattr__(self, "quaternion_xyzw", quaternion)

    @classmethod
    def identity(cls) -> "Pose3D":
        return cls(np.zeros(3), np.asarray([0.0, 0.0, 0.0, 1.0]))

    @classmethod
    def from_matrix(cls, target_T_source: np.ndarray) -> "Pose3D":
        """Construct a pose from a homogeneous rigid transform."""

        matrix = np.asarray(target_T_source, dtype=np.float64)
        if matrix.shape != (4, 4):
            raise ValueError(f"target_T_source must have shape (4, 4), got {matrix.shape}")
        if not np.all(np.isfinite(matrix)):
            raise ValueError("target_T_source must contain only finite values")
        if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
            raise ValueError("target_T_source must be a homogeneous rigid transform")
        rotation = matrix[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not np.isclose(
            np.linalg.det(rotation), 1.0, atol=1e-6
        ):
            raise ValueError("target_T_source rotation must be orthonormal and right-handed")
        return cls(matrix[:3, 3], _rotation_matrix_to_quaternion_xyzw(rotation))

    @property
    def matrix(self) -> np.ndarray:
        """Return a new 4x4 homogeneous transform."""

        x, y, z, w = self.quaternion_xyzw
        rotation = np.asarray(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rotation
        transform[:3, 3] = self.translation
        return transform

    @property
    def quaternion(self) -> np.ndarray:
        """The normalized quaternion in the public xyzw ordering."""

        return self.quaternion_xyzw

    def as_matrix(self) -> np.ndarray:
        """Alias for callers that prefer an explicit conversion method."""

        return self.matrix

    def inverse(self) -> "Pose3D":
        return Pose3D.from_matrix(np.linalg.inv(self.matrix))

    def compose(self, source_T_child: "Pose3D") -> "Pose3D":
        """Compose ``self`` (target_T_source) with ``source_T_child``."""

        return Pose3D.from_matrix(self.matrix @ source_T_child.matrix)


def _rotation_matrix_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    """Convert a rotation matrix to a stable, normalized xyzw quaternion."""

    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        quaternion = np.asarray(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            scale = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
            quaternion = np.asarray(
                [
                    0.25 * scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[2, 1] - rotation[1, 2]) / scale,
                ]
            )
        elif axis == 1:
            scale = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
            quaternion = np.asarray(
                [
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    0.25 * scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    (rotation[0, 2] - rotation[2, 0]) / scale,
                ]
            )
        else:
            scale = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
            quaternion = np.asarray(
                [
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    0.25 * scale,
                    (rotation[1, 0] - rotation[0, 1]) / scale,
                ]
            )
    if quaternion[3] < 0:
        quaternion = -quaternion
    return quaternion / np.linalg.norm(quaternion)

"""Immutable state passed between dataset, reconstruction, and visualization."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterator, Mapping, TypeVar

import numpy as np

from src.common.coordinates import Pose3D


K = TypeVar("K")
V = TypeVar("V")


class FrozenMapping(Mapping[K, V]):
    """Small immutable mapping that remains pickle/process-queue safe."""

    def __init__(self, values: Mapping[K, V] | None = None) -> None:
        self._values = dict(values or {})

    def __getitem__(self, key: K) -> V:
        return self._values[key]

    def __iter__(self) -> Iterator[K]:
        return iter(self._values)

    def __len__(self) -> int:
        return len(self._values)

    def __reduce__(self) -> tuple[type["FrozenMapping"], tuple[dict[K, V]]]:
        return type(self), (self._values,)


def _immutable_array(value: np.ndarray, *, shape: tuple[int, ...] | None, name: str) -> np.ndarray:
    array = np.array(value, copy=True)
    if shape is not None and array.shape != shape:
        raise ValueError(f"{name} must have shape {shape}, got {array.shape}")
    is_numeric = np.issubdtype(array.dtype, np.number) or np.issubdtype(array.dtype, np.bool_)
    if not is_numeric or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be a finite numeric array")
    array.flags.writeable = False
    return array


@dataclass(frozen=True, eq=False)
class VehicleState:
    """Authoritative metric state for a tracked vehicle."""

    track_id: str
    timestamp_us: int
    world_T_object: Pose3D
    dimensions_lwh: np.ndarray
    category: str = "vehicle.car"
    confidence: float = 1.0
    source: str = "unknown"

    def __post_init__(self) -> None:
        dimensions = _immutable_array(
            self.dimensions_lwh, shape=(3,), name="dimensions_lwh"
        ).astype(np.float64)
        if np.any(dimensions <= 0):
            raise ValueError("dimensions_lwh must be positive")
        dimensions.flags.writeable = False
        if not np.isfinite(self.confidence) or not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be between 0 and 1")
        object.__setattr__(self, "dimensions_lwh", dimensions)

    def ego_T_object(self, world_T_ego: Pose3D) -> Pose3D:
        """Return the object pose relative to the supplied ego pose."""

        return world_T_ego.inverse().compose(self.world_T_object)

    @property
    def dimensions(self) -> np.ndarray:
        """Metric length, width, height dimensions."""

        return self.dimensions_lwh


@dataclass(frozen=True, eq=False)
class CameraObservation:
    """A camera exposure and optional object mask used for reconstruction."""

    image: np.ndarray
    camera_intrinsic: np.ndarray
    world_T_camera: Pose3D
    timestamp_us: int
    mask: np.ndarray | None = None
    crop_xyxy: tuple[int, int, int, int] | None = None
    metadata: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        image = _immutable_array(self.image, shape=None, name="image")
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(f"image must have shape (H, W, 3), got {image.shape}")
        intrinsic = _immutable_array(
            self.camera_intrinsic, shape=(3, 3), name="camera_intrinsic"
        ).astype(np.float64)
        intrinsic.flags.writeable = False
        mask = None
        if self.mask is not None:
            mask = _immutable_array(self.mask, shape=image.shape[:2], name="mask").astype(bool)
            mask.flags.writeable = False
        if self.crop_xyxy is not None:
            x_min, y_min, x_max, y_max = self.crop_xyxy
            if not (0 <= x_min < x_max <= image.shape[1] and 0 <= y_min < y_max <= image.shape[0]):
                raise ValueError("crop_xyxy must be a non-empty region within the image")
        object.__setattr__(self, "image", image)
        object.__setattr__(self, "camera_intrinsic", intrinsic)
        object.__setattr__(self, "mask", mask)
        object.__setattr__(self, "metadata", FrozenMapping(self.metadata))

    @property
    def intrinsics(self) -> np.ndarray:
        return self.camera_intrinsic

"""Projection helpers for prompting instance segmentation from a 3D box."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import product
from typing import Any

import numpy as np
from numpy.typing import NDArray

from .models import VehicleDimensions


@dataclass(frozen=True)
class ProjectedBox:
    """An image-space ``(x_min, y_min, x_max, y_max)`` prompt."""

    xyxy: NDArray[np.float64]
    visible_fraction: float

    def __post_init__(self) -> None:
        xyxy = np.array(self.xyxy, dtype=np.float64, copy=True)
        if xyxy.shape != (4,) or not np.all(np.isfinite(xyxy)):
            raise ValueError("xyxy must be a finite array with shape (4,)")
        if xyxy[2] <= xyxy[0] or xyxy[3] <= xyxy[1]:
            raise ValueError("xyxy must describe a non-empty box")
        if not np.isfinite(self.visible_fraction) or not 0 <= self.visible_fraction <= 1:
            raise ValueError("visible_fraction must be between zero and one")
        xyxy.flags.writeable = False
        object.__setattr__(self, "xyxy", xyxy)

    @property
    def width(self) -> float:
        return float(self.xyxy[2] - self.xyxy[0])

    @property
    def height(self) -> float:
        return float(self.xyxy[3] - self.xyxy[1])


def project_vehicle_box(
    *,
    world_T_object: NDArray[np.floating[Any]],
    world_T_camera: NDArray[np.floating[Any]],
    intrinsics: NDArray[np.floating[Any]],
    dimensions: VehicleDimensions,
    image_size: tuple[int, int],
    minimum_depth: float = 0.1,
    minimum_area_px: float = 64.0,
) -> ProjectedBox | None:
    """Project an object box into a camera whose optical axis is positive Z.

    Returns ``None`` for boxes that cross the camera plane, do not intersect
    the image, or are too small to produce a useful segmentation prompt.
    """

    world_T_object = np.asarray(world_T_object, dtype=np.float64)
    world_T_camera = np.asarray(world_T_camera, dtype=np.float64)
    intrinsics = np.asarray(intrinsics, dtype=np.float64)
    if world_T_object.shape != (4, 4) or world_T_camera.shape != (4, 4):
        raise ValueError("poses must have shape (4, 4)")
    if intrinsics.shape != (3, 3):
        raise ValueError("intrinsics must have shape (3, 3)")
    height, width = image_size
    if height <= 0 or width <= 0:
        raise ValueError("image_size must be positive")

    half = dimensions.as_array / 2.0
    corners = np.asarray(list(product((-half[0], half[0]), (-half[1], half[1]), (-half[2], half[2]))))
    homogeneous = np.column_stack((corners, np.ones(8)))
    camera_T_object = np.linalg.inv(world_T_camera) @ world_T_object
    camera_corners = (camera_T_object @ homogeneous.T).T[:, :3]
    depths = camera_corners[:, 2]
    # A box intersecting the camera plane has an unbounded/unstable projection.
    if np.any(depths <= minimum_depth):
        return None

    pixels_h = (intrinsics @ camera_corners.T).T
    pixels = pixels_h[:, :2] / pixels_h[:, 2:3]
    raw_min = pixels.min(axis=0)
    raw_max = pixels.max(axis=0)
    raw_area = float(np.prod(np.maximum(raw_max - raw_min, 0.0)))
    if raw_area <= 0:
        return None

    clipped_min = np.maximum(raw_min, (0.0, 0.0))
    clipped_max = np.minimum(raw_max, (float(width - 1), float(height - 1)))
    clipped_size = np.maximum(clipped_max - clipped_min, 0.0)
    clipped_area = float(np.prod(clipped_size))
    if clipped_area < minimum_area_px:
        return None
    visible_fraction = clipped_area / raw_area
    return ProjectedBox(
        xyxy=np.asarray((*clipped_min, *clipped_max), dtype=np.float64),
        visible_fraction=float(np.clip(visible_fraction, 0.0, 1.0)),
    )

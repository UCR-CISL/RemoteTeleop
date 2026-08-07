"""Data passed across the object-reconstruction boundary.

These models intentionally do not depend on the replay or viewer models.  The
reconstruction worker is expected to run in a separate process, so its inputs
should remain small, explicit, and straightforward to serialize.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from src.common.coordinates import Pose3D
from src.common.models import CameraObservation, FrozenMapping


FloatArray = NDArray[np.floating[Any]]
BoolArray = NDArray[np.bool_]


@dataclass(frozen=True)
class VehicleDimensions:
    """Metric dimensions in project order: length, width, height."""

    length: float
    width: float
    height: float

    def __post_init__(self) -> None:
        if not np.all(np.isfinite(self.as_array)) or np.any(self.as_array <= 0):
            raise ValueError("vehicle dimensions must be finite and positive")

    @property
    def as_array(self) -> NDArray[np.float64]:
        return np.asarray((self.length, self.width, self.height), dtype=np.float64)


@dataclass(frozen=True)
class ReconstructionJob:
    request_id: str
    track_id: str
    observation: CameraObservation
    mask: BoolArray
    dimensions: VehicleDimensions
    world_T_object: Pose3D
    output_path: Path
    seed: int = 0
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.request_id or not self.track_id:
            raise ValueError("request_id and track_id must be non-empty")
        if self.mask.shape != self.observation.image.shape[:2]:
            raise ValueError("mask dimensions must match the image")
        mask = np.array(self.mask, dtype=np.bool_, copy=True)
        if not np.any(mask):
            raise ValueError("mask must contain at least one foreground pixel")
        mask.flags.writeable = False
        if not isinstance(self.world_T_object, Pose3D):
            raise TypeError("world_T_object must be a Pose3D")
        object.__setattr__(self, "mask", mask)
        object.__setattr__(self, "output_path", Path(self.output_path))
        object.__setattr__(self, "metadata", FrozenMapping(self.metadata))


class ReconstructionStatus(StrEnum):
    QUEUED = "queued"
    RUNNING = "running"
    READY = "ready"
    FAILED = "failed"


@dataclass(frozen=True)
class ReconstructedAsset:
    request_id: str
    track_id: str
    status: ReconstructionStatus
    raw_mesh_path: Path | None = None
    aligned_mesh_path: Path | None = None
    object_T_raw_mesh: FloatArray | None = None
    uniform_scale: float | None = None
    measured_dimensions: VehicleDimensions | None = None
    aligned_dimensions: FloatArray | None = None
    world_T_object: Pose3D | None = None
    error: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

"""Process-safe domain objects for frame-level box-prompted segmentation."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Mapping

import numpy as np
from numpy.typing import NDArray


class WorkerState(str, Enum):
    CREATED = "created"
    LOADING = "loading"
    LOADED = "loaded"
    WARMING = "warming"
    READY = "ready"
    BUSY = "busy"
    FAILED = "failed"


@dataclass(frozen=True)
class BoxPrompt:
    """A positive image-space box assigned to one authoritative track."""

    track_id: str
    xyxy: tuple[float, float, float, float]
    text_label: str = "object"

    def __post_init__(self) -> None:
        if not self.track_id:
            raise ValueError("track_id must not be empty")
        if not self.text_label:
            raise ValueError("text_label must not be empty")
        values = np.asarray(self.xyxy, dtype=np.float64)
        if values.shape != (4,) or not np.all(np.isfinite(values)):
            raise ValueError("xyxy must contain four finite values")
        if values[2] <= values[0] or values[3] <= values[1]:
            raise ValueError("xyxy must describe a non-empty box")


@dataclass(frozen=True, eq=False)
class FramePrompt:
    """One RGB frame and every projected detection that should be segmented."""

    frame_id: str
    timestamp_us: int
    image: NDArray[np.uint8]
    boxes: tuple[BoxPrompt, ...]

    def __post_init__(self) -> None:
        if not self.frame_id:
            raise ValueError("frame_id must not be empty")
        image = np.array(self.image, dtype=np.uint8, copy=True)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("image must have shape (height, width, 3)")
        if len({box.track_id for box in self.boxes}) != len(self.boxes):
            raise ValueError("track IDs must be unique within a frame")
        image.flags.writeable = False
        object.__setattr__(self, "image", image)
        object.__setattr__(self, "boxes", tuple(self.boxes))


@dataclass(frozen=True, eq=False)
class MaskResult:
    track_id: str
    mask: NDArray[np.bool_]
    confidence: float
    in_box_fraction: float
    box_coverage: float

    def __post_init__(self) -> None:
        mask = np.array(self.mask, dtype=np.bool_, copy=True)
        if mask.ndim != 2 or not np.any(mask):
            raise ValueError("mask must be a non-empty 2D array")
        for name in ("confidence", "in_box_fraction", "box_coverage"):
            value = getattr(self, name)
            if not np.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be between zero and one")
        mask.flags.writeable = False
        object.__setattr__(self, "mask", mask)


@dataclass(frozen=True)
class MaskWorkerMetrics:
    backend: str
    model_load_seconds: float
    warmup_seconds: float
    image_encode_seconds: float
    prompt_decode_seconds: float
    total_seconds: float
    prompts: int
    accepted_masks: int
    gpu_peak_allocated_mib: float | None = None
    gpu_peak_reserved_mib: float | None = None

    @property
    def masks_per_second(self) -> float:
        return self.accepted_masks / self.total_seconds if self.total_seconds > 0 else 0.0


@dataclass(frozen=True)
class MaskBatch:
    frame_id: str
    timestamp_us: int
    masks: Mapping[str, MaskResult]
    rejected_tracks: Mapping[str, str]
    metrics: MaskWorkerMetrics

    def __post_init__(self) -> None:
        # Plain dictionaries keep the batch compatible with multiprocessing
        # serialization; the frozen dataclass prevents attribute replacement.
        object.__setattr__(self, "masks", dict(self.masks))
        object.__setattr__(self, "rejected_tracks", dict(self.rejected_tracks))

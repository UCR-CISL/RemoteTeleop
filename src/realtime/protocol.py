"""Versioned, language-neutral contracts for real-time worker processes.

Each message is encoded as a topic, a compact JSON metadata document, and zero
or more opaque binary parts.  Large images and masks never pass through JSON.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any, ClassVar, Mapping, Sequence, TypeVar

try:
    from enum import StrEnum
except ImportError:  # Python 3.10

    class StrEnum(str, Enum):
        """Python 3.10-compatible subset of :class:`enum.StrEnum`."""


PROTOCOL_VERSION = 2
Matrix3 = tuple[float, float, float, float, float, float, float, float, float]
Matrix4 = tuple[
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
]


class ProtocolError(ValueError):
    """Raised when a multipart message violates the wire contract."""


class MessageType(StrEnum):
    FRAME_SNAPSHOT = "frame_snapshot"
    FRAME_HELLO = "frame_hello"
    FRAME_ACK = "frame_ack"
    FRAME_PENDING = "frame_pending"
    FRAME_END = "frame_end"
    EGO_POSE_SAMPLE = "ego_pose_sample"
    FRAME_OBJECT_DETECTIONS = "frame_object_detections"
    FRAME_DETECTIONS = "frame_detections"
    MASK_BATCH = "mask_batch"
    RECONSTRUCTION_REQUEST = "reconstruction_request"
    RECONSTRUCTION_END = "reconstruction_end"
    ASSET_EVENT = "asset_event"
    WORKER_HEALTH = "worker_health"
    ACKNOWLEDGEMENT = "acknowledgement"
    ASSET_SYNC = "asset_sync"
    ASSET_MANIFEST = "asset_manifest"
    ASSET_FETCH = "asset_fetch"
    ASSET_CHUNK = "asset_chunk"
    ASSET_COMMIT_ACK = "asset_commit_ack"


class AssetState(StrEnum):
    QUEUED = "queued"
    RUNNING = "running"
    READY = "ready"
    FAILED = "failed"
    CANCELLED = "cancelled"


class WorkerState(StrEnum):
    LOADING = "loading"
    WARMING = "warming"
    READY = "ready"
    BUSY = "busy"
    FAILED = "failed"
    STOPPING = "stopping"


def _non_empty(value: str, name: str) -> str:
    if not value:
        raise ValueError(f"{name} must be non-empty")
    return value


def _finite(value: float, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def _fixed_floats(
    values: Sequence[float], length: int, name: str
) -> tuple[float, ...]:
    result = tuple(_finite(value, name) for value in values)
    if len(result) != length:
        raise ValueError(f"{name} must contain {length} values")
    return result


def _rigid_transform(values: Sequence[float], name: str) -> Matrix4:
    """Validate a finite row-major homogeneous rigid transform."""
    transform = _fixed_floats(values, 16, name)
    tolerance = 1e-6
    if any(
        abs(transform[index] - expected) > tolerance
        for index, expected in zip((12, 13, 14, 15), (0.0, 0.0, 0.0, 1.0))
    ):
        raise ValueError(f"{name} must have a homogeneous final row")
    rotation = (
        transform[0:3],
        transform[4:7],
        transform[8:11],
    )
    for row in rotation:
        if abs(sum(value * value for value in row) - 1.0) > tolerance:
            raise ValueError(f"{name} rotation must be orthonormal")
    for first, second in ((0, 1), (0, 2), (1, 2)):
        if abs(sum(a * b for a, b in zip(rotation[first], rotation[second]))) > tolerance:
            raise ValueError(f"{name} rotation must be orthonormal")
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    if abs(determinant - 1.0) > tolerance:
        raise ValueError(f"{name} rotation must have determinant one")
    return transform  # type: ignore[return-value]


def _unit_interval(value: float, name: str) -> float:
    result = _finite(value, name)
    if not 0.0 <= result <= 1.0:
        raise ValueError(f"{name} must be between 0 and 1")
    return result


def _json_mapping(values: Mapping[str, Any], name: str) -> dict[str, Any]:
    result = dict(values)
    try:
        json.dumps(result, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be JSON serializable") from error
    return result


@dataclass(frozen=True)
class BoxPrompt:
    """One authoritative track and its geometric image prompt."""

    track_id: str
    xyxy: tuple[float, float, float, float]
    dimensions_lwh: tuple[float, float, float]
    world_T_object: Matrix4
    text_label: str = "object"
    visibility: float = 1.0
    detection_confidence: float = 1.0

    def __post_init__(self) -> None:
        _non_empty(self.track_id, "track_id")
        _non_empty(self.text_label, "text_label")
        xyxy = _fixed_floats(self.xyxy, 4, "xyxy")
        if xyxy[0] >= xyxy[2] or xyxy[1] >= xyxy[3]:
            raise ValueError("xyxy must describe a non-empty box")
        dimensions = _fixed_floats(self.dimensions_lwh, 3, "dimensions_lwh")
        if any(value <= 0.0 for value in dimensions):
            raise ValueError("dimensions_lwh must be positive")
        object.__setattr__(self, "xyxy", xyxy)
        object.__setattr__(self, "dimensions_lwh", dimensions)
        object.__setattr__(
            self, "world_T_object", _fixed_floats(self.world_T_object, 16, "world_T_object")
        )
        object.__setattr__(self, "visibility", _unit_interval(self.visibility, "visibility"))
        object.__setattr__(
            self,
            "detection_confidence",
            _unit_interval(self.detection_confidence, "detection_confidence"),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "track_id": self.track_id,
            "xyxy": self.xyxy,
            "dimensions_lwh": self.dimensions_lwh,
            "world_T_object": self.world_T_object,
            "text_label": self.text_label,
            "visibility": self.visibility,
            "detection_confidence": self.detection_confidence,
        }

    @classmethod
    def from_dict(cls, values: Mapping[str, Any]) -> "BoxPrompt":
        return cls(**values)


@dataclass(frozen=True)
class MaskResult:
    track_id: str
    confidence: float
    coverage: float
    box_overlap: float
    latency_ms: float
    mask_png: bytes = field(repr=False)

    def __post_init__(self) -> None:
        _non_empty(self.track_id, "track_id")
        object.__setattr__(self, "confidence", _unit_interval(self.confidence, "confidence"))
        object.__setattr__(self, "coverage", _unit_interval(self.coverage, "coverage"))
        object.__setattr__(self, "box_overlap", _unit_interval(self.box_overlap, "box_overlap"))
        latency = _finite(self.latency_ms, "latency_ms")
        if latency < 0:
            raise ValueError("latency_ms must be non-negative")
        object.__setattr__(self, "latency_ms", latency)
        if not self.mask_png:
            raise ValueError("mask_png must be non-empty")
        object.__setattr__(self, "mask_png", bytes(self.mask_png))


MessageT = TypeVar("MessageT", bound="WireMessage")


class WireMessage(ABC):
    """Base class implemented by every protocol message."""

    message_type: ClassVar[MessageType]

    @abstractmethod
    def _metadata(self) -> dict[str, Any]:
        raise NotImplementedError

    def _binary_parts(self) -> tuple[bytes, ...]:
        return ()

    @classmethod
    @abstractmethod
    def _from_wire(
        cls: type[MessageT], metadata: Mapping[str, Any], binary_parts: Sequence[bytes]
    ) -> MessageT:
        raise NotImplementedError


@dataclass(frozen=True)
class EgoPoseSample(WireMessage):
    """Pose-only camera-rendering input sent from vehicle to remote teleop."""

    message_type: ClassVar[MessageType] = MessageType.EGO_POSE_SAMPLE

    sequence: int
    timestamp_us: int
    scene_generation: str
    frame_id: str
    world_T_ego: Matrix4

    def __post_init__(self) -> None:
        if isinstance(self.sequence, bool) or not isinstance(self.sequence, int):
            raise ValueError("sequence must be an integer")
        if self.sequence < 0:
            raise ValueError("sequence must be non-negative")
        if isinstance(self.timestamp_us, bool) or not isinstance(self.timestamp_us, int):
            raise ValueError("timestamp_us must be an integer")
        if self.timestamp_us < 0:
            raise ValueError("timestamp_us must be non-negative")
        for name in ("scene_generation", "frame_id"):
            value = getattr(self, name)
            if not isinstance(value, str):
                raise ValueError(f"{name} must be a string")
            _non_empty(value, name)
        object.__setattr__(
            self, "world_T_ego", _rigid_transform(self.world_T_ego, "world_T_ego")
        )

    def _metadata(self) -> dict[str, Any]:
        return {
            "sequence": self.sequence,
            "timestamp_us": self.timestamp_us,
            "scene_generation": self.scene_generation,
            "frame_id": self.frame_id,
            "world_T_ego": self.world_T_ego,
        }

    @classmethod
    def _from_wire(
        cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]
    ) -> "EgoPoseSample":
        _expect_binary_parts(binary_parts, 0, cls.message_type)
        return cls(**metadata)


@dataclass(frozen=True)
class FrameObjectDetections(WireMessage):
    """Object prompts paired with one pose sample, without camera/image data."""

    message_type: ClassVar[MessageType] = MessageType.FRAME_OBJECT_DETECTIONS

    sequence: int
    timestamp_us: int
    scene_generation: str
    frame_id: str
    boxes: tuple[BoxPrompt, ...] = ()

    def __post_init__(self) -> None:
        if isinstance(self.sequence, bool) or not isinstance(self.sequence, int):
            raise ValueError("sequence must be an integer")
        if self.sequence < 0:
            raise ValueError("sequence must be non-negative")
        if isinstance(self.timestamp_us, bool) or not isinstance(self.timestamp_us, int):
            raise ValueError("timestamp_us must be an integer")
        if self.timestamp_us < 0:
            raise ValueError("timestamp_us must be non-negative")
        for name in ("scene_generation", "frame_id"):
            value = getattr(self, name)
            if not isinstance(value, str):
                raise ValueError(f"{name} must be a string")
            _non_empty(value, name)
        boxes = tuple(self.boxes)
        if not all(isinstance(box, BoxPrompt) for box in boxes):
            raise ValueError("boxes must contain BoxPrompt values")
        track_ids = [box.track_id for box in boxes]
        if len(track_ids) != len(set(track_ids)):
            raise ValueError("boxes must contain at most one prompt per track_id")
        object.__setattr__(self, "boxes", boxes)

    def _metadata(self) -> dict[str, Any]:
        return {
            "sequence": self.sequence,
            "timestamp_us": self.timestamp_us,
            "scene_generation": self.scene_generation,
            "frame_id": self.frame_id,
            "boxes": [box.to_dict() for box in self.boxes],
        }

    @classmethod
    def _from_wire(
        cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]
    ) -> "FrameObjectDetections":
        _expect_binary_parts(binary_parts, 0, cls.message_type)
        values = dict(metadata)
        values["boxes"] = tuple(BoxPrompt.from_dict(value) for value in values.pop("boxes", ()))
        return cls(**values)


@dataclass(frozen=True)
class FrameDetections(WireMessage):
    message_type: ClassVar[MessageType] = MessageType.FRAME_DETECTIONS

    frame_id: str
    request_id: str
    scene_generation: str
    timestamp_us: int
    image_jpeg: bytes = field(repr=False)
    camera_intrinsic: Matrix3
    world_T_camera: Matrix4
    world_T_ego: Matrix4
    boxes: tuple[BoxPrompt, ...] = ()
    end_of_scene: bool = False
    source_sequence: int | None = None

    def __post_init__(self) -> None:
        for name in ("frame_id", "request_id", "scene_generation"):
            _non_empty(getattr(self, name), name)
        if self.timestamp_us < 0:
            raise ValueError("timestamp_us must be non-negative")
        if not self.image_jpeg:
            raise ValueError("image_jpeg must be non-empty")
        object.__setattr__(self, "image_jpeg", bytes(self.image_jpeg))
        object.__setattr__(
            self,
            "camera_intrinsic",
            _fixed_floats(self.camera_intrinsic, 9, "camera_intrinsic"),
        )
        object.__setattr__(
            self, "world_T_camera", _fixed_floats(self.world_T_camera, 16, "world_T_camera")
        )
        object.__setattr__(
            self, "world_T_ego", _fixed_floats(self.world_T_ego, 16, "world_T_ego")
        )
        object.__setattr__(self, "boxes", tuple(self.boxes))
        if not isinstance(self.end_of_scene, bool):
            raise ValueError("end_of_scene must be a boolean")
        if self.source_sequence is not None:
            if isinstance(self.source_sequence, bool) or not isinstance(self.source_sequence, int):
                raise ValueError("source_sequence must be an integer when provided")
            if self.source_sequence < 0:
                raise ValueError("source_sequence must be non-negative")
        track_ids = [box.track_id for box in self.boxes]
        if len(track_ids) != len(set(track_ids)):
            raise ValueError("boxes must contain at most one prompt per track_id")

    def _metadata(self) -> dict[str, Any]:
        return {
            "frame_id": self.frame_id,
            "request_id": self.request_id,
            "scene_generation": self.scene_generation,
            "timestamp_us": self.timestamp_us,
            "camera_intrinsic": self.camera_intrinsic,
            "world_T_camera": self.world_T_camera,
            "world_T_ego": self.world_T_ego,
            "boxes": [box.to_dict() for box in self.boxes],
            "end_of_scene": self.end_of_scene,
            "source_sequence": self.source_sequence,
        }

    def _binary_parts(self) -> tuple[bytes, ...]:
        return (self.image_jpeg,)

    @property
    def sha256(self) -> str:
        """Stable identity for durable, receiver-pulled local analysis frames."""

        canonical = json.dumps(
            self._metadata(), sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")
        digest = hashlib.sha256()
        digest.update(canonical)
        digest.update(b"\0")
        digest.update(self.image_jpeg)
        return digest.hexdigest()

    @classmethod
    def _from_wire(
        cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]
    ) -> "FrameDetections":
        _expect_binary_parts(binary_parts, 1, cls.message_type)
        values = dict(metadata)
        values["boxes"] = tuple(BoxPrompt.from_dict(value) for value in values.pop("boxes", ()))
        return cls(image_jpeg=binary_parts[0], **values)


@dataclass(frozen=True)
class FrameSnapshot(WireMessage):
    """One atomic, pose-only render input; never carries image/calibration bytes."""

    message_type: ClassVar[MessageType] = MessageType.FRAME_SNAPSHOT
    sequence: int
    timestamp_us: int
    scene_generation: str
    frame_id: str
    world_T_ego: Matrix4
    boxes: tuple[BoxPrompt, ...] = ()

    def __post_init__(self) -> None:
        if isinstance(self.sequence, bool) or not isinstance(self.sequence, int) or self.sequence < 0:
            raise ValueError("sequence must be a non-negative integer")
        if isinstance(self.timestamp_us, bool) or not isinstance(self.timestamp_us, int) or self.timestamp_us < 0:
            raise ValueError("timestamp_us must be a non-negative integer")
        for name in ("scene_generation", "frame_id"):
            value = getattr(self, name)
            if not isinstance(value, str):
                raise ValueError(f"{name} must be a string")
            _non_empty(value, name)
        object.__setattr__(self, "world_T_ego", _rigid_transform(self.world_T_ego, "world_T_ego"))
        boxes = tuple(self.boxes)
        if not all(isinstance(box, BoxPrompt) for box in boxes):
            raise ValueError("boxes must contain BoxPrompt values")
        if len({box.track_id for box in boxes}) != len(boxes):
            raise ValueError("boxes must contain at most one prompt per track_id")
        object.__setattr__(self, "boxes", boxes)

    def _metadata(self) -> dict[str, Any]:
        return {"sequence": self.sequence, "timestamp_us": self.timestamp_us,
                "scene_generation": self.scene_generation, "frame_id": self.frame_id,
                "world_T_ego": self.world_T_ego, "boxes": [box.to_dict() for box in self.boxes]}

    @property
    def sha256(self) -> str:
        canonical = json.dumps(self._metadata(), sort_keys=True, separators=(",", ":"), allow_nan=False)
        return hashlib.sha256(canonical.encode("utf-8")).hexdigest()

    @classmethod
    def _from_wire(cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]) -> "FrameSnapshot":
        _expect_binary_parts(binary_parts, 0, cls.message_type)
        values = dict(metadata)
        values["boxes"] = tuple(BoxPrompt.from_dict(value) for value in values.pop("boxes", ()))
        return cls(**values)


@dataclass(frozen=True)
class FrameHello(WireMessage):
    message_type: ClassVar[MessageType] = MessageType.FRAME_HELLO
    scene_generation: str
    receiver_id: str
    next_sequence: int

    def __post_init__(self) -> None:
        _non_empty(self.scene_generation, "scene_generation"); _non_empty(self.receiver_id, "receiver_id")
        if isinstance(self.next_sequence, bool) or not isinstance(self.next_sequence, int) or self.next_sequence < 0:
            raise ValueError("next_sequence must be a non-negative integer")
    def _metadata(self) -> dict[str, Any]: return {"scene_generation": self.scene_generation, "receiver_id": self.receiver_id, "next_sequence": self.next_sequence}
    @classmethod
    def _from_wire(cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]) -> "FrameHello":
        _expect_binary_parts(binary_parts, 0, cls.message_type); return cls(**metadata)


@dataclass(frozen=True)
class FrameAck(WireMessage):
    message_type: ClassVar[MessageType] = MessageType.FRAME_ACK
    scene_generation: str
    sequence: int
    frame_id: str
    snapshot_sha256: str
    rendered_sha256: str | None = None

    def __post_init__(self) -> None:
        _non_empty(self.scene_generation, "scene_generation"); _non_empty(self.frame_id, "frame_id")
        if isinstance(self.sequence, bool) or not isinstance(self.sequence, int) or self.sequence < 0:
            raise ValueError("sequence must be a non-negative integer")
        object.__setattr__(self, "snapshot_sha256", _sha256(self.snapshot_sha256, "snapshot_sha256"))
        if self.rendered_sha256 is not None: _sha256(self.rendered_sha256, "rendered_sha256")
    def _metadata(self) -> dict[str, Any]: return {"scene_generation": self.scene_generation, "sequence": self.sequence, "frame_id": self.frame_id, "snapshot_sha256": self.snapshot_sha256, "rendered_sha256": self.rendered_sha256}
    @classmethod
    def _from_wire(cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]) -> "FrameAck":
        _expect_binary_parts(binary_parts, 0, cls.message_type); return cls(**metadata)


@dataclass(frozen=True)
class FramePending(WireMessage):
    """The requested snapshot is not spooled yet; retry the same cursor later."""

    message_type: ClassVar[MessageType] = MessageType.FRAME_PENDING
    scene_generation: str
    next_sequence: int

    def __post_init__(self) -> None:
        _non_empty(self.scene_generation, "scene_generation")
        if isinstance(self.next_sequence, bool) or not isinstance(self.next_sequence, int) or self.next_sequence < 0:
            raise ValueError("next_sequence must be a non-negative integer")

    def _metadata(self) -> dict[str, Any]:
        return {"scene_generation": self.scene_generation, "next_sequence": self.next_sequence}

    @classmethod
    def _from_wire(cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]) -> "FramePending":
        _expect_binary_parts(binary_parts, 0, cls.message_type)
        return cls(**metadata)


@dataclass(frozen=True)
class FrameEnd(WireMessage):
    message_type: ClassVar[MessageType] = MessageType.FRAME_END
    scene_generation: str
    final_sequence: int
    final_timestamp_us: int
    def __post_init__(self) -> None:
        _non_empty(self.scene_generation, "scene_generation")
        for name in ("final_sequence", "final_timestamp_us"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
    def _metadata(self) -> dict[str, Any]: return {"scene_generation": self.scene_generation, "final_sequence": self.final_sequence, "final_timestamp_us": self.final_timestamp_us}
    @classmethod
    def _from_wire(cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]) -> "FrameEnd":
        _expect_binary_parts(binary_parts, 0, cls.message_type); return cls(**metadata)


@dataclass(frozen=True)
class MaskBatch(WireMessage):
    message_type: ClassVar[MessageType] = MessageType.MASK_BATCH

    frame_id: str
    request_id: str
    scene_generation: str
    timestamp_us: int
    masks: tuple[MaskResult, ...]
    batch_latency_ms: float

    def __post_init__(self) -> None:
        for name in ("frame_id", "request_id", "scene_generation"):
            _non_empty(getattr(self, name), name)
        object.__setattr__(self, "masks", tuple(self.masks))
        latency = _finite(self.batch_latency_ms, "batch_latency_ms")
        if latency < 0:
            raise ValueError("batch_latency_ms must be non-negative")
        object.__setattr__(self, "batch_latency_ms", latency)
        track_ids = [mask.track_id for mask in self.masks]
        if len(track_ids) != len(set(track_ids)):
            raise ValueError("masks must contain at most one result per track_id")

    def _metadata(self) -> dict[str, Any]:
        return {
            "frame_id": self.frame_id,
            "request_id": self.request_id,
            "scene_generation": self.scene_generation,
            "timestamp_us": self.timestamp_us,
            "batch_latency_ms": self.batch_latency_ms,
            "masks": [
                {
                    "track_id": mask.track_id,
                    "confidence": mask.confidence,
                    "coverage": mask.coverage,
                    "box_overlap": mask.box_overlap,
                    "latency_ms": mask.latency_ms,
                    "binary_index": index,
                }
                for index, mask in enumerate(self.masks)
            ],
        }

    def _binary_parts(self) -> tuple[bytes, ...]:
        return tuple(mask.mask_png for mask in self.masks)

    @classmethod
    def _from_wire(
        cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]
    ) -> "MaskBatch":
        values = dict(metadata)
        mask_values = values.pop("masks", ())
        _expect_binary_parts(binary_parts, len(mask_values), cls.message_type)
        masks = []
        for expected_index, mask_metadata in enumerate(mask_values):
            item = dict(mask_metadata)
            binary_index = item.pop("binary_index")
            if binary_index != expected_index:
                raise ProtocolError("mask binary indices must be contiguous and ordered")
            masks.append(MaskResult(mask_png=binary_parts[binary_index], **item))
        return cls(masks=tuple(masks), **values)


@dataclass(frozen=True)
class ReconstructionRequest(WireMessage):
    message_type: ClassVar[MessageType] = MessageType.RECONSTRUCTION_REQUEST

    request_id: str
    scene_generation: str
    track_id: str
    frame_id: str
    timestamp_us: int
    image_jpeg: bytes = field(repr=False)
    mask_png: bytes = field(repr=False)
    camera_intrinsic: Matrix3
    dimensions_lwh: tuple[float, float, float]
    world_T_object: Matrix4
    world_T_camera: Matrix4
    quality_score: float
    seed: int = 0
    source_sequence: int | None = None

    def __post_init__(self) -> None:
        for name in ("request_id", "scene_generation", "track_id", "frame_id"):
            _non_empty(getattr(self, name), name)
        if not self.image_jpeg or not self.mask_png:
            raise ValueError("image_jpeg and mask_png must be non-empty")
        object.__setattr__(self, "image_jpeg", bytes(self.image_jpeg))
        object.__setattr__(self, "mask_png", bytes(self.mask_png))
        object.__setattr__(
            self,
            "camera_intrinsic",
            _fixed_floats(self.camera_intrinsic, 9, "camera_intrinsic"),
        )
        dimensions = _fixed_floats(self.dimensions_lwh, 3, "dimensions_lwh")
        if any(value <= 0 for value in dimensions):
            raise ValueError("dimensions_lwh must be positive")
        object.__setattr__(self, "dimensions_lwh", dimensions)
        object.__setattr__(
            self, "world_T_object", _fixed_floats(self.world_T_object, 16, "world_T_object")
        )
        object.__setattr__(
            self, "world_T_camera", _fixed_floats(self.world_T_camera, 16, "world_T_camera")
        )
        object.__setattr__(self, "quality_score", _finite(self.quality_score, "quality_score"))
        if self.source_sequence is not None:
            if isinstance(self.source_sequence, bool) or not isinstance(self.source_sequence, int):
                raise ValueError("source_sequence must be an integer when provided")
            if self.source_sequence < 0:
                raise ValueError("source_sequence must be non-negative")

    def _metadata(self) -> dict[str, Any]:
        return {
            "request_id": self.request_id,
            "scene_generation": self.scene_generation,
            "track_id": self.track_id,
            "frame_id": self.frame_id,
            "timestamp_us": self.timestamp_us,
            "camera_intrinsic": self.camera_intrinsic,
            "dimensions_lwh": self.dimensions_lwh,
            "world_T_object": self.world_T_object,
            "world_T_camera": self.world_T_camera,
            "quality_score": self.quality_score,
            "seed": self.seed,
            "source_sequence": self.source_sequence,
        }

    def _binary_parts(self) -> tuple[bytes, ...]:
        return self.image_jpeg, self.mask_png

    @classmethod
    def _from_wire(
        cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]
    ) -> "ReconstructionRequest":
        _expect_binary_parts(binary_parts, 2, cls.message_type)
        return cls(image_jpeg=binary_parts[0], mask_png=binary_parts[1], **metadata)


@dataclass(frozen=True)
class ReconstructionEnd(WireMessage):
    """SAM3 has submitted every reconstruction request for one replay."""

    message_type: ClassVar[MessageType] = MessageType.RECONSTRUCTION_END
    request_id: str
    scene_generation: str
    final_sequence: int

    def __post_init__(self) -> None:
        _non_empty(self.request_id, "request_id")
        _non_empty(self.scene_generation, "scene_generation")
        if (
            isinstance(self.final_sequence, bool)
            or not isinstance(self.final_sequence, int)
            or self.final_sequence < 0
        ):
            raise ValueError("final_sequence must be a non-negative integer")

    def _metadata(self) -> dict[str, Any]:
        return {
            "request_id": self.request_id,
            "scene_generation": self.scene_generation,
            "final_sequence": self.final_sequence,
        }

    @classmethod
    def _from_wire(
        cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]
    ) -> "ReconstructionEnd":
        _expect_binary_parts(binary_parts, 0, cls.message_type)
        return cls(**metadata)


@dataclass(frozen=True)
class AssetEvent(WireMessage):
    message_type: ClassVar[MessageType] = MessageType.ASSET_EVENT

    request_id: str
    scene_generation: str
    track_id: str
    state: AssetState
    aligned_mesh_path: Path | None = None
    asset_id: str | None = None
    asset_version: int = 1
    content_sha256: str | None = None
    mesh_payload: bytes | None = field(default=None, repr=False)
    mesh_suffix: str = ".glb"
    error: str | None = None
    metrics: Mapping[str, Any] = field(default_factory=dict)
    source_sequence: int | None = None
    source_timestamp_us: int | None = None
    source_frame_id: str | None = None

    def __post_init__(self) -> None:
        for name in ("request_id", "scene_generation", "track_id"):
            _non_empty(getattr(self, name), name)
        object.__setattr__(self, "state", AssetState(self.state))
        path = Path(self.aligned_mesh_path) if self.aligned_mesh_path is not None else None
        payload = None if self.mesh_payload is None else bytes(self.mesh_payload)
        if self.state is AssetState.READY and path is None and not payload:
            raise ValueError("ready assets require aligned_mesh_path or mesh_payload")
        if self.state is AssetState.FAILED and not self.error:
            raise ValueError("failed assets require error")
        if self.asset_id is not None:
            _non_empty(self.asset_id, "asset_id")
        if self.asset_version <= 0:
            raise ValueError("asset_version must be positive")
        if re.fullmatch(r"\.[A-Za-z0-9]{1,15}", self.mesh_suffix) is None:
            raise ValueError("mesh_suffix must be a simple filename suffix")
        if self.content_sha256 is not None:
            digest = self.content_sha256.casefold()
            if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
                raise ValueError("content_sha256 must be a SHA-256 hex digest")
            object.__setattr__(self, "content_sha256", digest)
        if payload is not None and self.content_sha256 is None:
            raise ValueError("mesh_payload requires content_sha256")
        if self.source_sequence is not None:
            if isinstance(self.source_sequence, bool) or not isinstance(self.source_sequence, int):
                raise ValueError("source_sequence must be an integer when provided")
            if self.source_sequence < 0:
                raise ValueError("source_sequence must be non-negative")
        if self.source_timestamp_us is not None:
            if isinstance(self.source_timestamp_us, bool) or not isinstance(self.source_timestamp_us, int):
                raise ValueError("source_timestamp_us must be an integer when provided")
            if self.source_timestamp_us < 0:
                raise ValueError("source_timestamp_us must be non-negative")
        if self.source_frame_id is not None:
            _non_empty(self.source_frame_id, "source_frame_id")
        object.__setattr__(self, "aligned_mesh_path", path)
        object.__setattr__(self, "mesh_payload", payload)
        object.__setattr__(self, "metrics", _json_mapping(self.metrics, "metrics"))

    def _metadata(self) -> dict[str, Any]:
        return {
            "request_id": self.request_id,
            "scene_generation": self.scene_generation,
            "track_id": self.track_id,
            "state": self.state,
            "aligned_mesh_path": (
                str(self.aligned_mesh_path) if self.aligned_mesh_path is not None else None
            ),
            "asset_id": self.asset_id,
            "asset_version": self.asset_version,
            "content_sha256": self.content_sha256,
            "mesh_suffix": self.mesh_suffix,
            "error": self.error,
            "metrics": self.metrics,
            "source_sequence": self.source_sequence,
            "source_timestamp_us": self.source_timestamp_us,
            "source_frame_id": self.source_frame_id,
        }

    @classmethod
    def _from_wire(
        cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]
    ) -> "AssetEvent":
        if len(binary_parts) > 1:
            raise ProtocolError("asset_event expects at most one binary mesh payload")
        values = dict(metadata)
        return cls(mesh_payload=binary_parts[0] if binary_parts else None, **values)

    def _binary_parts(self) -> tuple[bytes, ...]:
        return () if self.mesh_payload is None else (self.mesh_payload,)


def _sha256(value: str, name: str) -> str:
    digest = value.casefold()
    if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
        raise ValueError(f"{name} must be a SHA-256 hex digest")
    return digest


@dataclass(frozen=True)
class AssetSync(WireMessage):
    message_type: ClassVar[MessageType] = MessageType.ASSET_SYNC
    receiver_id: str
    scene_generation: str
    known_content_sha256: tuple[str, ...] = ()
    def __post_init__(self) -> None:
        _non_empty(self.receiver_id, "receiver_id"); _non_empty(self.scene_generation, "scene_generation")
        hashes = tuple(_sha256(value, "known_content_sha256") for value in self.known_content_sha256)
        object.__setattr__(self, "known_content_sha256", hashes)
    def _metadata(self) -> dict[str, Any]: return {"receiver_id": self.receiver_id, "scene_generation": self.scene_generation, "known_content_sha256": self.known_content_sha256}
    @classmethod
    def _from_wire(cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]) -> "AssetSync":
        _expect_binary_parts(binary_parts, 0, cls.message_type); return cls(**metadata)


@dataclass(frozen=True)
class AssetManifest(WireMessage):
    message_type: ClassVar[MessageType] = MessageType.ASSET_MANIFEST
    scene_generation: str
    track_id: str
    request_id: str
    source_sequence: int | None
    source_timestamp_us: int
    source_frame_id: str
    asset_id: str
    asset_version: int
    mesh_suffix: str
    byte_length: int
    content_sha256: str
    alignment_convention_version: int = 1
    def __post_init__(self) -> None:
        for name in ("scene_generation", "track_id", "request_id", "source_frame_id", "asset_id"):
            _non_empty(getattr(self, name), name)
        if self.source_sequence is not None and (isinstance(self.source_sequence, bool) or not isinstance(self.source_sequence, int) or self.source_sequence < 0): raise ValueError("source_sequence must be a non-negative integer when provided")
        for name in ("source_timestamp_us", "asset_version", "byte_length", "alignment_convention_version"):
            value = getattr(self, name)
            minimum = 0 if name == "source_timestamp_us" else 1
            if isinstance(value, bool) or not isinstance(value, int) or value < minimum: raise ValueError(f"{name} is out of range")
        if self.byte_length == 0:
            raise ValueError("byte_length must be positive")
        if re.fullmatch(r"\.[A-Za-z0-9]{1,15}", self.mesh_suffix) is None: raise ValueError("mesh_suffix must be a simple filename suffix")
        object.__setattr__(self, "content_sha256", _sha256(self.content_sha256, "content_sha256"))
    def _metadata(self) -> dict[str, Any]: return {name: getattr(self, name) for name in ("scene_generation", "track_id", "request_id", "source_sequence", "source_timestamp_us", "source_frame_id", "asset_id", "asset_version", "mesh_suffix", "byte_length", "content_sha256", "alignment_convention_version")}
    @classmethod
    def _from_wire(cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]) -> "AssetManifest":
        _expect_binary_parts(binary_parts, 0, cls.message_type); return cls(**metadata)


@dataclass(frozen=True)
class AssetFetch(WireMessage):
    message_type: ClassVar[MessageType] = MessageType.ASSET_FETCH
    asset_id: str; asset_version: int; content_sha256: str; offset: int = 0
    def __post_init__(self) -> None:
        _non_empty(self.asset_id, "asset_id"); object.__setattr__(self, "content_sha256", _sha256(self.content_sha256, "content_sha256"))
        if isinstance(self.asset_version, bool) or not isinstance(self.asset_version, int) or self.asset_version < 1: raise ValueError("asset_version must be positive")
        if isinstance(self.offset, bool) or not isinstance(self.offset, int) or self.offset < 0: raise ValueError("offset must be non-negative")
    def _metadata(self) -> dict[str, Any]: return {"asset_id": self.asset_id, "asset_version": self.asset_version, "content_sha256": self.content_sha256, "offset": self.offset}
    @classmethod
    def _from_wire(cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]) -> "AssetFetch": _expect_binary_parts(binary_parts, 0, cls.message_type); return cls(**metadata)


@dataclass(frozen=True)
class AssetChunk(WireMessage):
    message_type: ClassVar[MessageType] = MessageType.ASSET_CHUNK
    asset_id: str; asset_version: int; content_sha256: str; offset: int; payload: bytes = field(repr=False)
    MAX_PAYLOAD_BYTES: ClassVar[int] = 4 * 1024 * 1024
    def __post_init__(self) -> None:
        _non_empty(self.asset_id, "asset_id"); object.__setattr__(self, "content_sha256", _sha256(self.content_sha256, "content_sha256"))
        if isinstance(self.asset_version, bool) or not isinstance(self.asset_version, int) or self.asset_version < 1: raise ValueError("asset_version must be positive")
        if isinstance(self.offset, bool) or not isinstance(self.offset, int) or self.offset < 0: raise ValueError("offset must be non-negative")
        payload = bytes(self.payload)
        if not payload or len(payload) > self.MAX_PAYLOAD_BYTES: raise ValueError("payload must be between 1 byte and 4 MiB")
        object.__setattr__(self, "payload", payload)
    def _metadata(self) -> dict[str, Any]: return {"asset_id": self.asset_id, "asset_version": self.asset_version, "content_sha256": self.content_sha256, "offset": self.offset}
    def _binary_parts(self) -> tuple[bytes, ...]: return (self.payload,)
    @classmethod
    def _from_wire(cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]) -> "AssetChunk": _expect_binary_parts(binary_parts, 1, cls.message_type); return cls(payload=binary_parts[0], **metadata)


@dataclass(frozen=True)
class AssetCommitAck(WireMessage):
    message_type: ClassVar[MessageType] = MessageType.ASSET_COMMIT_ACK
    asset_id: str; asset_version: int; content_sha256: str
    def __post_init__(self) -> None:
        _non_empty(self.asset_id, "asset_id"); object.__setattr__(self, "content_sha256", _sha256(self.content_sha256, "content_sha256"))
        if isinstance(self.asset_version, bool) or not isinstance(self.asset_version, int) or self.asset_version < 1: raise ValueError("asset_version must be positive")
    def _metadata(self) -> dict[str, Any]: return {"asset_id": self.asset_id, "asset_version": self.asset_version, "content_sha256": self.content_sha256}
    @classmethod
    def _from_wire(cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]) -> "AssetCommitAck": _expect_binary_parts(binary_parts, 0, cls.message_type); return cls(**metadata)


@dataclass(frozen=True)
class WorkerHealth(WireMessage):
    message_type: ClassVar[MessageType] = MessageType.WORKER_HEALTH

    worker_id: str
    state: WorkerState
    timestamp_us: int
    pid: int
    device: str
    queue_depth: int = 0
    cuda_allocated_mib: float = 0.0
    cuda_reserved_mib: float = 0.0
    detail: str | None = None

    def __post_init__(self) -> None:
        _non_empty(self.worker_id, "worker_id")
        object.__setattr__(self, "state", WorkerState(self.state))
        if self.timestamp_us < 0 or self.pid <= 0 or self.queue_depth < 0:
            raise ValueError("timestamp_us, pid, and queue_depth are out of range")
        for name in ("cuda_allocated_mib", "cuda_reserved_mib"):
            value = _finite(getattr(self, name), name)
            if value < 0:
                raise ValueError(f"{name} must be non-negative")
            object.__setattr__(self, name, value)

    def _metadata(self) -> dict[str, Any]:
        return {
            "worker_id": self.worker_id,
            "state": self.state,
            "timestamp_us": self.timestamp_us,
            "pid": self.pid,
            "device": self.device,
            "queue_depth": self.queue_depth,
            "cuda_allocated_mib": self.cuda_allocated_mib,
            "cuda_reserved_mib": self.cuda_reserved_mib,
            "detail": self.detail,
        }

    @classmethod
    def _from_wire(
        cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]
    ) -> "WorkerHealth":
        _expect_binary_parts(binary_parts, 0, cls.message_type)
        return cls(**metadata)


@dataclass(frozen=True)
class Acknowledgement(WireMessage):
    message_type: ClassVar[MessageType] = MessageType.ACKNOWLEDGEMENT

    request_id: str
    accepted: bool
    detail: str | None = None

    def __post_init__(self) -> None:
        _non_empty(self.request_id, "request_id")

    def _metadata(self) -> dict[str, Any]:
        return {
            "request_id": self.request_id,
            "accepted": self.accepted,
            "detail": self.detail,
        }

    @classmethod
    def _from_wire(
        cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]
    ) -> "Acknowledgement":
        _expect_binary_parts(binary_parts, 0, cls.message_type)
        return cls(**metadata)


_MESSAGE_CLASSES: dict[MessageType, type[WireMessage]] = {
    message_class.message_type: message_class
    for message_class in (
        EgoPoseSample,
        FrameObjectDetections,
        FrameDetections,
        FrameSnapshot,
        FrameHello,
        FrameAck,
        FramePending,
        FrameEnd,
        MaskBatch,
        ReconstructionRequest,
        ReconstructionEnd,
        AssetEvent,
        AssetSync,
        AssetManifest,
        AssetFetch,
        AssetChunk,
        AssetCommitAck,
        WorkerHealth,
        Acknowledgement,
    )
}


def _expect_binary_parts(
    binary_parts: Sequence[bytes], expected: int, message_type: MessageType
) -> None:
    if len(binary_parts) != expected:
        raise ProtocolError(
            f"{message_type} expects {expected} binary parts, got {len(binary_parts)}"
        )


class ProtocolCodec:
    """Encode and strictly validate version-one multipart messages."""

    def encode(self, message: WireMessage, *, topic: str | None = None) -> list[bytes]:
        message_topic = topic or message.message_type.value
        _non_empty(message_topic, "topic")
        envelope = {
            "protocol_version": PROTOCOL_VERSION,
            "message_type": message.message_type,
            "payload": message._metadata(),
        }
        try:
            metadata = json.dumps(
                envelope, separators=(",", ":"), allow_nan=False
            ).encode("utf-8")
        except (TypeError, ValueError) as error:
            raise ProtocolError("message metadata must be JSON serializable") from error
        return [message_topic.encode("utf-8"), metadata, *message._binary_parts()]

    def decode(self, parts: Sequence[bytes]) -> tuple[str, WireMessage]:
        if len(parts) < 2:
            raise ProtocolError("a message requires topic and metadata parts")
        try:
            topic = bytes(parts[0]).decode("utf-8")
            envelope = json.loads(bytes(parts[1]).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise ProtocolError("invalid UTF-8 or JSON envelope") from error
        if not isinstance(envelope, dict):
            raise ProtocolError("metadata envelope must be an object")
        version = envelope.get("protocol_version")
        if version != PROTOCOL_VERSION:
            raise ProtocolError(
                f"unsupported protocol version {version!r}; expected {PROTOCOL_VERSION}"
            )
        try:
            message_type = MessageType(envelope["message_type"])
            payload = envelope["payload"]
        except (KeyError, ValueError) as error:
            raise ProtocolError("unknown or missing message_type") from error
        if not isinstance(payload, dict):
            raise ProtocolError("payload must be an object")
        message_class = _MESSAGE_CLASSES[message_type]
        try:
            message = message_class._from_wire(payload, [bytes(part) for part in parts[2:]])
        except ProtocolError:
            raise
        except (KeyError, TypeError, ValueError) as error:
            raise ProtocolError(f"invalid {message_type} payload: {error}") from error
        return topic, message

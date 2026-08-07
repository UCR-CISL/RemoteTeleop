"""Versioned, language-neutral contracts for real-time worker processes.

Each message is encoded as a topic, a compact JSON metadata document, and zero
or more opaque binary parts.  Large images and masks never pass through JSON.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import StrEnum
import json
import math
from pathlib import Path
from typing import Any, ClassVar, Mapping, Sequence, TypeVar


PROTOCOL_VERSION = 1
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
    FRAME_DETECTIONS = "frame_detections"
    MASK_BATCH = "mask_batch"
    RECONSTRUCTION_REQUEST = "reconstruction_request"
    ASSET_EVENT = "asset_event"
    WORKER_HEALTH = "worker_health"
    ACKNOWLEDGEMENT = "acknowledgement"


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
    visibility: float = 1.0
    detection_confidence: float = 1.0

    def __post_init__(self) -> None:
        _non_empty(self.track_id, "track_id")
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
        }

    def _binary_parts(self) -> tuple[bytes, ...]:
        return (self.image_jpeg,)

    @classmethod
    def _from_wire(
        cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]
    ) -> "FrameDetections":
        _expect_binary_parts(binary_parts, 1, cls.message_type)
        values = dict(metadata)
        values["boxes"] = tuple(BoxPrompt.from_dict(value) for value in values.pop("boxes", ()))
        return cls(image_jpeg=binary_parts[0], **values)


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
class AssetEvent(WireMessage):
    message_type: ClassVar[MessageType] = MessageType.ASSET_EVENT

    request_id: str
    scene_generation: str
    track_id: str
    state: AssetState
    aligned_mesh_path: Path | None = None
    error: str | None = None
    metrics: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for name in ("request_id", "scene_generation", "track_id"):
            _non_empty(getattr(self, name), name)
        object.__setattr__(self, "state", AssetState(self.state))
        path = Path(self.aligned_mesh_path) if self.aligned_mesh_path is not None else None
        if self.state is AssetState.READY and path is None:
            raise ValueError("ready assets require aligned_mesh_path")
        if self.state is AssetState.FAILED and not self.error:
            raise ValueError("failed assets require error")
        object.__setattr__(self, "aligned_mesh_path", path)
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
            "error": self.error,
            "metrics": self.metrics,
        }

    @classmethod
    def _from_wire(
        cls, metadata: Mapping[str, Any], binary_parts: Sequence[bytes]
    ) -> "AssetEvent":
        _expect_binary_parts(binary_parts, 0, cls.message_type)
        return cls(**metadata)


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
        FrameDetections,
        MaskBatch,
        ReconstructionRequest,
        AssetEvent,
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

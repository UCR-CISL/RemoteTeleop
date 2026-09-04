"""Small, standalone wire contract for the BEVFusion Docker sidecar.

The general replay protocol deliberately never sends images or point clouds to
the remote host.  This contract is local to the vehicle and sends one raw BGR
front image plus an ``Nx4`` little-endian float32 LiDAR cloud to the detector.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Mapping, Sequence

import zmq


BEVFUSION_PROTOCOL_VERSION = 1
REQUEST_TOPIC = "bevfusion_request"
RESULT_TOPIC = "bevfusion_result"


class BevFusionProtocolError(ValueError):
    """Raised when a BEVFusion sidecar message is malformed."""


def _non_empty(value: str, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be a non-empty string")
    return value


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _fixed(values: Sequence[float], length: int, name: str) -> tuple[float, ...]:
    result = tuple(_finite(value, name) for value in values)
    if len(result) != length:
        raise ValueError(f"{name} must contain {length} values")
    return result


def _non_negative_int(value: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


@dataclass(frozen=True)
class BevFusionRequest:
    """A single front-camera / LiDAR inference request.

    ``image_bgr`` is raw row-major BGR uint8 data.  ``lidar_f32`` is a
    contiguous little-endian float32 matrix shaped ``(lidar_point_count, 4)``.
    Poses map their named sensor coordinates into the playback world frame.
    """

    request_id: str
    scene_generation: str
    sequence: int
    frame_id: str
    timestamp_us: int
    config_id: str
    camera_intrinsic: tuple[float, ...]
    world_T_camera: tuple[float, ...]
    world_T_lidar: tuple[float, ...]
    image_width: int
    image_height: int
    image_bgr: bytes
    lidar_point_count: int
    lidar_f32: bytes

    def __post_init__(self) -> None:
        for name in ("request_id", "scene_generation", "frame_id", "config_id"):
            _non_empty(getattr(self, name), name)
        _non_negative_int(self.sequence, "sequence")
        _non_negative_int(self.timestamp_us, "timestamp_us")
        object.__setattr__(self, "camera_intrinsic", _fixed(self.camera_intrinsic, 9, "camera_intrinsic"))
        object.__setattr__(self, "world_T_camera", _fixed(self.world_T_camera, 16, "world_T_camera"))
        object.__setattr__(self, "world_T_lidar", _fixed(self.world_T_lidar, 16, "world_T_lidar"))
        if isinstance(self.image_width, bool) or not isinstance(self.image_width, int) or self.image_width <= 0:
            raise ValueError("image_width must be a positive integer")
        if isinstance(self.image_height, bool) or not isinstance(self.image_height, int) or self.image_height <= 0:
            raise ValueError("image_height must be a positive integer")
        image = bytes(self.image_bgr)
        if len(image) != self.image_width * self.image_height * 3:
            raise ValueError("image_bgr must contain exactly width * height * 3 bytes")
        _non_negative_int(self.lidar_point_count, "lidar_point_count")
        lidar = bytes(self.lidar_f32)
        if len(lidar) != self.lidar_point_count * 16:
            raise ValueError("lidar_f32 must contain exactly lidar_point_count * 16 bytes")
        object.__setattr__(self, "image_bgr", image)
        object.__setattr__(self, "lidar_f32", lidar)

    def metadata(self) -> dict[str, Any]:
        return {
            "request_id": self.request_id, "scene_generation": self.scene_generation,
            "sequence": self.sequence, "frame_id": self.frame_id,
            "timestamp_us": self.timestamp_us, "config_id": self.config_id,
            "camera_intrinsic": self.camera_intrinsic,
            "world_T_camera": self.world_T_camera, "world_T_lidar": self.world_T_lidar,
            "image_width": self.image_width, "image_height": self.image_height,
            "lidar_point_count": self.lidar_point_count,
        }


@dataclass(frozen=True)
class BevFusionBox:
    """A car detection expressed in the request LiDAR coordinate frame."""

    center_xyz: tuple[float, float, float]
    dimensions_lwh: tuple[float, float, float]
    yaw_rad: float
    score: float
    class_id: int = 0
    class_name: str = "car"

    def __post_init__(self) -> None:
        object.__setattr__(self, "center_xyz", _fixed(self.center_xyz, 3, "center_xyz"))
        dimensions = _fixed(self.dimensions_lwh, 3, "dimensions_lwh")
        if min(dimensions) <= 0:
            raise ValueError("dimensions_lwh must be positive")
        object.__setattr__(self, "dimensions_lwh", dimensions)
        object.__setattr__(self, "yaw_rad", _finite(self.yaw_rad, "yaw_rad"))
        score = _finite(self.score, "score")
        if not 0 <= score <= 1:
            raise ValueError("score must be between 0 and 1")
        object.__setattr__(self, "score", score)
        if isinstance(self.class_id, bool) or not isinstance(self.class_id, int):
            raise ValueError("class_id must be an integer")
        _non_empty(self.class_name, "class_name")

    def to_dict(self) -> dict[str, Any]:
        return {"center_xyz": self.center_xyz, "dimensions_lwh": self.dimensions_lwh,
                "yaw_rad": self.yaw_rad, "score": self.score, "class_id": self.class_id,
                "class_name": self.class_name}

    @classmethod
    def from_dict(cls, values: Mapping[str, Any]) -> "BevFusionBox":
        return cls(**values)


@dataclass(frozen=True)
class BevFusionResult:
    """Detector response with provenance matching exactly one request."""

    request_id: str
    scene_generation: str
    sequence: int
    frame_id: str
    timestamp_us: int
    config_id: str
    boxes: tuple[BevFusionBox, ...] = ()
    timings_ms: Mapping[str, float] | None = None
    error: str | None = None

    def __post_init__(self) -> None:
        for name in ("request_id", "scene_generation", "frame_id", "config_id"):
            _non_empty(getattr(self, name), name)
        _non_negative_int(self.sequence, "sequence")
        _non_negative_int(self.timestamp_us, "timestamp_us")
        boxes = tuple(self.boxes)
        if not all(isinstance(box, BevFusionBox) for box in boxes):
            raise ValueError("boxes must contain BevFusionBox values")
        object.__setattr__(self, "boxes", boxes)
        timings = {} if self.timings_ms is None else {str(k): _finite(v, "timings_ms") for k, v in self.timings_ms.items()}
        if any(value < 0 for value in timings.values()):
            raise ValueError("timings_ms values must be non-negative")
        object.__setattr__(self, "timings_ms", timings)
        if self.error is not None:
            _non_empty(self.error, "error")
            if boxes:
                raise ValueError("an error result cannot contain boxes")

    def metadata(self) -> dict[str, Any]:
        return {"request_id": self.request_id, "scene_generation": self.scene_generation,
                "sequence": self.sequence, "frame_id": self.frame_id,
                "timestamp_us": self.timestamp_us, "config_id": self.config_id,
                "boxes": [box.to_dict() for box in self.boxes],
                "timings_ms": dict(self.timings_ms or {}), "error": self.error}


class BevFusionCodec:
    """Versioned multipart codec independent from the remote replay protocol."""

    def encode_request(self, request: BevFusionRequest) -> list[bytes]:
        return self._encode(REQUEST_TOPIC, request.metadata(), (request.image_bgr, request.lidar_f32))

    def decode_request(self, parts: Sequence[bytes]) -> BevFusionRequest:
        payload, binary = self._decode(parts, REQUEST_TOPIC, 2)
        return BevFusionRequest(image_bgr=binary[0], lidar_f32=binary[1], **payload)

    def encode_result(self, result: BevFusionResult) -> list[bytes]:
        return self._encode(RESULT_TOPIC, result.metadata(), ())

    def decode_result(self, parts: Sequence[bytes]) -> BevFusionResult:
        payload, _ = self._decode(parts, RESULT_TOPIC, 0)
        values = dict(payload)
        values["boxes"] = tuple(BevFusionBox.from_dict(box) for box in values.get("boxes", ()))
        return BevFusionResult(**values)

    @staticmethod
    def _encode(topic: str, payload: Mapping[str, Any], binary: Sequence[bytes]) -> list[bytes]:
        envelope = {"bevfusion_protocol_version": BEVFUSION_PROTOCOL_VERSION, "payload": payload}
        return [topic.encode("utf-8"), json.dumps(envelope, separators=(",", ":"), allow_nan=False).encode("utf-8"), *binary]

    @staticmethod
    def _decode(parts: Sequence[bytes], expected_topic: str, binary_count: int) -> tuple[Mapping[str, Any], tuple[bytes, ...]]:
        if len(parts) != 2 + binary_count:
            raise BevFusionProtocolError(f"{expected_topic} expects {binary_count} binary parts")
        try:
            topic = bytes(parts[0]).decode("utf-8")
            envelope = json.loads(bytes(parts[1]).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise BevFusionProtocolError("invalid UTF-8 or JSON envelope") from error
        if topic != expected_topic or envelope.get("bevfusion_protocol_version") != BEVFUSION_PROTOCOL_VERSION:
            raise BevFusionProtocolError("unexpected BEVFusion message topic or protocol version")
        payload = envelope.get("payload")
        if not isinstance(payload, Mapping):
            raise BevFusionProtocolError("BEVFusion payload must be an object")
        return payload, tuple(bytes(part) for part in parts[2:])


class BevFusionClient:
    """Synchronous, window-one client for a local persistent sidecar."""

    def __init__(self, socket: zmq.Socket[Any], *, codec: BevFusionCodec | None = None, owns_socket: bool = False, timeout_ms: int = 100) -> None:
        self.socket = socket
        self.codec = codec or BevFusionCodec()
        self._owns_socket = owns_socket
        self._timeout_ms = timeout_ms
        self._in_flight: BevFusionRequest | None = None

    @classmethod
    def open(cls, context: zmq.Context[Any], endpoint: str, *, identity: str, timeout_ms: int = 100) -> "BevFusionClient":
        if timeout_ms <= 0:
            raise ValueError("timeout_ms must be positive")
        socket = context.socket(zmq.DEALER)
        socket.setsockopt(zmq.IDENTITY, identity.encode("utf-8"))
        socket.setsockopt(zmq.SNDHWM, 1); socket.setsockopt(zmq.RCVHWM, 1)
        socket.setsockopt(zmq.LINGER, 0); socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        socket.connect(endpoint)
        return cls(socket, owns_socket=True, timeout_ms=timeout_ms)

    def infer(self, request: BevFusionRequest) -> BevFusionResult:
        self.submit(request)
        result = self.poll(timeout_ms=self._timeout_ms)
        if result is None:
            raise TimeoutError(f"BEVFusion result timed out for {request.request_id}")
        if (result.request_id, result.scene_generation, result.sequence, result.frame_id, result.timestamp_us, result.config_id) != (request.request_id, request.scene_generation, request.sequence, request.frame_id, request.timestamp_us, request.config_id):
            raise BevFusionProtocolError("BEVFusion result provenance does not match request")
        return result

    def submit(self, request: BevFusionRequest) -> bool:
        """Send one request; callers must poll it before sending another."""
        if self._in_flight is not None:
            raise RuntimeError("BEVFusion client permits only one in-flight request")
        try:
            self.socket.send_multipart(self.codec.encode_request(request), flags=zmq.NOBLOCK)
        except zmq.Again as error:
            raise RuntimeError("BEVFusion sidecar request queue is full") from error
        self._in_flight = request
        return True

    def poll(self, *, timeout_ms: int | None = None) -> BevFusionResult | None:
        """Return the outstanding result, or ``None`` when a zero-time poll has none."""
        request = self._in_flight
        if request is None:
            return None
        wait_ms = 0 if timeout_ms is None else timeout_ms
        if wait_ms < 0:
            raise ValueError("timeout_ms must be non-negative")
        if not self.socket.poll(timeout=wait_ms):
            return None
        try:
            result = self.codec.decode_result(self.socket.recv_multipart())
        except zmq.Again as error:
            raise TimeoutError(f"BEVFusion result timed out for {request.request_id}") from error
        self._in_flight = None
        if (result.request_id, result.scene_generation, result.sequence, result.frame_id, result.timestamp_us, result.config_id) != (request.request_id, request.scene_generation, request.sequence, request.frame_id, request.timestamp_us, request.config_id):
            raise BevFusionProtocolError("BEVFusion result provenance does not match request")
        return result

    def close(self) -> None:
        if self._owns_socket:
            self.socket.close(linger=0)

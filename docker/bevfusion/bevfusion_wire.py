"""Docker-local decoder for the host's BEVFusion multipart protocol.

It is intentionally dependency-free because `/workspace` is the only project
mount provided to the Docker container.
"""
from dataclasses import dataclass
import json
from typing import Mapping, Sequence

BEVFUSION_PROTOCOL_VERSION = 1
REQUEST_TOPIC = "bevfusion_request"
RESULT_TOPIC = "bevfusion_result"


@dataclass(frozen=True)
class BevFusionRequest:
    request_id: str; scene_generation: str; sequence: int; frame_id: str
    timestamp_us: int; config_id: str; camera_intrinsic: tuple[float, ...]
    world_T_camera: tuple[float, ...]; world_T_lidar: tuple[float, ...]
    image_width: int; image_height: int; image_bgr: bytes
    lidar_point_count: int; lidar_f32: bytes
    lidar_feature_count: int = 4


@dataclass(frozen=True)
class BevFusionBox:
    center_xyz: tuple[float, float, float]
    dimensions_lwh: tuple[float, float, float]
    yaw_rad: float; score: float; class_id: int = 0; class_name: str = "car"

    def to_dict(self):
        return {"center_xyz": self.center_xyz, "dimensions_lwh": self.dimensions_lwh,
                "yaw_rad": self.yaw_rad, "score": self.score, "class_id": self.class_id,
                "class_name": self.class_name}


@dataclass(frozen=True)
class BevFusionResult:
    request_id: str; scene_generation: str; sequence: int; frame_id: str
    timestamp_us: int; config_id: str; boxes: tuple[BevFusionBox, ...] = ()
    timings_ms: Mapping[str, float] | None = None; error: str | None = None

    def metadata(self):
        return {"request_id": self.request_id, "scene_generation": self.scene_generation,
                "sequence": self.sequence, "frame_id": self.frame_id,
                "timestamp_us": self.timestamp_us, "config_id": self.config_id,
                "boxes": [box.to_dict() for box in self.boxes],
                "timings_ms": dict(self.timings_ms or {}), "error": self.error}


class BevFusionCodec:
    def decode_request(self, parts: Sequence[bytes]) -> BevFusionRequest:
        if len(parts) != 4 or parts[0] != REQUEST_TOPIC.encode():
            raise ValueError("invalid BEVFusion request multipart message")
        envelope = json.loads(parts[1].decode())
        if envelope.get("bevfusion_protocol_version") != BEVFUSION_PROTOCOL_VERSION:
            raise ValueError("unsupported BEVFusion protocol version")
        payload = envelope["payload"]
        request = BevFusionRequest(
            request_id=payload["request_id"], scene_generation=payload["scene_generation"],
            sequence=payload["sequence"], frame_id=payload["frame_id"],
            timestamp_us=payload["timestamp_us"], config_id=payload["config_id"],
            camera_intrinsic=tuple(payload["camera_intrinsic"]),
            world_T_camera=tuple(payload["world_T_camera"]), world_T_lidar=tuple(payload["world_T_lidar"]),
            image_width=payload["image_width"], image_height=payload["image_height"],
            image_bgr=bytes(parts[2]), lidar_point_count=payload["lidar_point_count"], lidar_f32=bytes(parts[3]),
            lidar_feature_count=payload.get("lidar_feature_count", 4))
        if request.image_width <= 0 or request.image_height <= 0:
            raise ValueError("BEVFusion image dimensions must be positive")
        if len(request.image_bgr) != request.image_width * request.image_height * 3:
            raise ValueError("invalid BEVFusion BGR byte count")
        if isinstance(request.lidar_feature_count, bool) or not isinstance(request.lidar_feature_count, int) or request.lidar_feature_count not in (4, 5):
            raise ValueError("invalid BEVFusion LiDAR feature count")
        if request.lidar_point_count < 0 or len(request.lidar_f32) != request.lidar_point_count * request.lidar_feature_count * 4:
            raise ValueError("invalid BEVFusion LiDAR byte count")
        if len(request.camera_intrinsic) != 9:
            raise ValueError("invalid BEVFusion camera intrinsic")
        if len(request.world_T_camera) != 16 or len(request.world_T_lidar) != 16:
            raise ValueError("invalid BEVFusion sensor transform")
        return request

    def encode_result(self, result: BevFusionResult) -> list[bytes]:
        envelope = {"bevfusion_protocol_version": BEVFUSION_PROTOCOL_VERSION, "payload": result.metadata()}
        return [RESULT_TOPIC.encode(), json.dumps(envelope, separators=(",", ":"), allow_nan=False).encode()]

"""Read CooperScene-style Foxglove MCAP recordings without modifying source data.

The source exporter writes complete, ordered sensor streams with asynchronous
capture timestamps. Join by ordinal frame, retaining source labels and checking
record-time skew; never join on exact sensor timestamps or duplicate local boxes.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Iterator

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from src.localization.cooperscene_calibration import DEFAULT_CALIBRATION, camera_intrinsic, camera_T_lidar
from src.realtime.protocol import BoxPrompt
from src.reconstruction.models import VehicleDimensions
from src.reconstruction.projection import project_vehicle_box


@dataclass(frozen=True)
class FoxgloveVehicle:
    track_id: str
    dimensions_lwh: tuple[float, float, float]
    world_T_object: np.ndarray
    text_label: str = "car"


@dataclass(frozen=True)
class FoxgloveFrame:
    sequence: int
    timestamp_ns: int
    image_bgr: np.ndarray
    metadata: dict[str, Any]
    lidar_points: np.ndarray | None
    vehicles: tuple[FoxgloveVehicle, ...]


class _ProtobufDecoder:
    """Build message types from MCAP schemas, including their dependencies."""

    def __init__(self) -> None:
        try:
            from google.protobuf.descriptor_pool import DescriptorPool
        except ImportError as exc:
            raise RuntimeError("Foxglove MCAP reading requires protobuf") from exc
        self.pool = DescriptorPool()
        self.classes: dict[tuple[str, bytes], Any] = {}

    def decode(self, schema: Any, channel: Any, data: bytes) -> Any:
        from google.protobuf import descriptor_pb2, message_factory
        if schema is None or channel.message_encoding != "protobuf" or schema.encoding != "protobuf":
            raise ValueError(f"{channel.topic} must use a protobuf schema and encoding")
        key = (schema.name, schema.data)
        if key not in self.classes:
            pending = list(descriptor_pb2.FileDescriptorSet.FromString(schema.data).file)
            while pending:
                remaining = []
                for descriptor in pending:
                    try:
                        self.pool.Add(descriptor)
                    except Exception:
                        remaining.append(descriptor)
                if len(remaining) == len(pending):
                    raise ValueError(f"unresolvable protobuf schema {schema.name}")
                pending = remaining
            descriptor = self.pool.FindMessageTypeByName(schema.name)
            self.classes[key] = message_factory.GetMessageClass(descriptor)
        return self.classes[key].FromString(data)


def _transform(position: Any, orientation: Any) -> np.ndarray:
    values = np.asarray([orientation.x, orientation.y, orientation.z, orientation.w], dtype=float)
    if not np.all(np.isfinite(values)) or np.linalg.norm(values) < 1e-12:
        raise ValueError("invalid Foxglove pose quaternion")
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(values).as_matrix()
    result[:3, 3] = [position.x, position.y, position.z]
    if not np.all(np.isfinite(result)):
        raise ValueError("non-finite Foxglove pose")
    return result


class FoxgloveMcapReader:
    """Yield camera frames and normalized replay metadata from source MCAPs.

    Use the global annotation union, not the overlapping per-agent display
    layers. NuScenes exports must embed ``teleop`` MCAP metadata and provide
    ``map -> <agent>_cam`` transforms per frame. Ego defaults to the
    LiDAR frame, matching the existing replay convention. CooperScene camera
    calibration is not baked into source MCAPs; ``calibration`` names the
    documented per-agent table used to derive it.
    """

    def __init__(self, path: str | Path, *, agent: str = "1", scene_generation: str | None = None,
                 include_lidar: bool = False, calibration: str = DEFAULT_CALIBRATION) -> None:
        self.path = Path(path)
        self.agent = str(agent)
        self.calibration = calibration
        self.scene_generation = scene_generation
        self.include_lidar = include_lidar
        if not self.agent or not self.path.is_file():
            raise ValueError("reader requires an existing MCAP and non-empty agent")

    @staticmethod
    def _vehicles(entity: Any) -> tuple[FoxgloveVehicle, ...]:
        if entity.frame_id != "map":
            raise ValueError("global boxes must be in map coordinates")
        ids = [text.text for text in entity.texts]
        if len(set(ids)) != len(ids) or any(not value for value in ids):
            raise ValueError("global vehicle IDs must be nonempty and unique")
        if len(entity.cubes) not in (len(ids), 2 * len(ids)):
            raise ValueError("global boxes must have one body or body/heading pair per ID")
        stride = 2 if ids and len(entity.cubes) == 2 * len(ids) else 1
        labels = {item.key: item.value for item in entity.metadata}
        vehicles = []
        for track_id, cube in zip(ids, entity.cubes[::stride]):
            dimensions = (cube.size.x, cube.size.y, cube.size.z)
            VehicleDimensions(*dimensions)
            vehicles.append(FoxgloveVehicle(track_id, dimensions,
                _transform(cube.pose.position, cube.pose.orientation), labels.get(f"label:{track_id}", "car")))
        return tuple(vehicles)

    @staticmethod
    def _point_cloud(message: Any) -> np.ndarray:
        fields = {field.name: field for field in message.fields}
        if message.point_stride <= 0 or len(message.data) % message.point_stride:
            raise ValueError("invalid point cloud stride")
        columns = []
        count = len(message.data) // message.point_stride
        for name in ("x", "y", "z", "intensity"):
            field = fields.get(name)
            if field is None and name == "intensity":
                columns.append(np.zeros(count, dtype=np.float32))
                continue
            if field is None or field.type != 7 or field.offset + 4 > message.point_stride:
                raise ValueError(f"point field {name} must be FLOAT32")
            columns.append(np.ndarray((count,), dtype="<f4", buffer=message.data,
                                      offset=field.offset, strides=(message.point_stride,)))
        points = np.column_stack(columns).astype(np.float32)
        if not np.all(np.isfinite(points)):
            raise ValueError("non-finite point cloud")
        return points

    def __iter__(self) -> Iterator[FoxgloveFrame]:
        from mcap.reader import make_reader
        with self.path.open("rb") as stream:
            reader = make_reader(stream)
            options: dict[str, str] = {}
            for metadata in reader.iter_metadata():
                if metadata.name == "teleop":
                    options.update(metadata.metadata)
            dataset = options.get("dataset", "cooperscene")
            if dataset not in ("cooperscene", "nuscenes"):
                raise ValueError(f"unsupported Foxglove dataset {dataset}")
            dynamic_camera = dataset == "nuscenes"
            fps = float(options.get("frames_per_second", "10"))
            if not np.isfinite(fps) or fps <= 0:
                raise ValueError("frames_per_second must be positive and finite")
            generation = self.scene_generation or options.get("scene_generation", dataset)
            if dynamic_camera:
                intrinsic = np.asarray(json.loads(options["camera_intrinsic"]), dtype=float).reshape(3, 3)
            else:
                intrinsic = camera_intrinsic(self.agent, self.calibration)
                lidar_T_camera = np.linalg.inv(camera_T_lidar(self.agent, self.calibration))
            if not np.all(np.isfinite(intrinsic)):
                raise ValueError("non-finite camera intrinsics")
            image_topic, lidar_topic = f"/{self.agent}_Camera", f"/{self.agent}_LIDAR"
            topics = [image_topic, "/bbox_global", "/tf"]
            if self.include_lidar:
                topics.append(lidar_topic)
            summary = reader.get_summary()
            if summary is None or summary.statistics is None:
                raise ValueError("Foxglove replay requires indexed MCAP summary statistics")
            counts = {c.topic: summary.statistics.channel_message_counts.get(i, 0)
                      for i, c in summary.channels.items()}
            expected = counts.get(image_topic, 0)
            if not expected or counts.get("/bbox_global", 0) not in (expected, 2 * expected):
                raise ValueError("missing or inconsistent camera/global-box frame counts")
            if self.include_lidar and counts.get(lidar_topic) != expected:
                raise ValueError("camera and LiDAR sample counts differ")
            required = ["image", "boxes", "frame_id", "lidar_pose"]
            if dynamic_camera:
                required += ["camera_pose"]
            if self.include_lidar:
                required.append("lidar")
            queues: dict[str, deque] = {name: deque() for name in required}
            decoder = _ProtobufDecoder()
            sequence = 0
            previous_time: int | None = None
            previous_frame: str | None = None
            for schema, channel, record in reader.iter_messages(topics=topics, log_time_order=True):
                msg = decoder.decode(schema, channel, record.data)
                if channel.topic == image_topic:
                    if schema.name != "foxglove.CompressedImage" or msg.format not in ("png", "jpeg", "jpg"):
                        raise ValueError("unsupported compressed camera image")
                    image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                    if image is None:
                        raise ValueError("failed to decode Foxglove camera image")
                    queues["image"].append((record.log_time, image))
                elif channel.topic == "/bbox_global":
                    if schema.name != "foxglove.SceneUpdate":
                        raise ValueError("global boxes require foxglove.SceneUpdate")
                    for entity in msg.entities:
                        if entity.id == "global_bboxes":
                            queues["boxes"].append((record.log_time, self._vehicles(entity)))
                        elif entity.id.endswith("_frame_label"):
                            if len(entity.texts) != 1 or not entity.texts[0].text:
                                raise ValueError("invalid source frame label")
                            queues["frame_id"].append((record.log_time, entity.texts[0].text))
                elif channel.topic == "/tf":
                    if schema.name != "foxglove.FrameTransform":
                        raise ValueError("transforms require foxglove.FrameTransform")
                    key = {f"{self.agent}_base_link": "lidar_pose", f"{self.agent}_cam": "camera_pose",
                           f"{self.agent}_ego": "ego_pose"}.get(msg.child_frame_id)
                    if key in queues:
                        if msg.parent_frame_id != "map":
                            raise ValueError("agent transforms must be relative to map")
                        queues[key].append((record.log_time, _transform(msg.translation, msg.rotation)))
                elif channel.topic == lidar_topic:
                    if schema.name != "foxglove.PointCloud":
                        raise ValueError("LiDAR requires foxglove.PointCloud")
                    queues["lidar"].append((record.log_time, self._point_cloud(msg)))
                while all(queues[name] for name in required):
                    values = {name: queues[name].popleft() for name in required}
                    timestamp, image = values["image"]
                    if previous_time is not None and timestamp <= previous_time:
                        raise ValueError("camera timestamps must strictly increase")
                    if max(abs(time - timestamp) for time, _ in values.values()) > 1e9 / fps:
                        raise ValueError("Foxglove frame streams are out of sync")
                    frame_id = values["frame_id"][1]
                    if frame_id == previous_frame:
                        raise ValueError("duplicate source frame label")
                    previous_time, previous_frame = timestamp, frame_id
                    lidar_pose = values["lidar_pose"][1]
                    camera_pose = values["camera_pose"][1] if dynamic_camera else lidar_pose @ lidar_T_camera
                    ego_pose = lidar_pose
                    vehicles = values["boxes"][1]
                    boxes = []
                    for vehicle in vehicles:
                        projected = project_vehicle_box(world_T_object=vehicle.world_T_object,
                            world_T_camera=camera_pose, intrinsics=intrinsic,
                            dimensions=VehicleDimensions(*vehicle.dimensions_lwh), image_size=image.shape[:2])
                        if projected is not None:
                            boxes.append(BoxPrompt(track_id=vehicle.track_id, xyxy=tuple(projected.xyxy),
                                dimensions_lwh=vehicle.dimensions_lwh,
                                world_T_object=tuple(vehicle.world_T_object.reshape(-1)),
                                text_label=vehicle.text_label, visibility=projected.visible_fraction,
                                detection_confidence=1.0).to_dict())
                    payload = {"version": 1, "frame_id": frame_id, "source_sequence": sequence,
                        "request_id": f"{generation}:{frame_id}", "scene_generation": generation,
                        "timestamp_us": round(sequence * 1_000_000 / fps), "source_timestamp_ns": timestamp,
                        "camera_intrinsic": intrinsic.reshape(-1).tolist(), "world_T_camera": camera_pose.reshape(-1).tolist(),
                        "world_T_ego": ego_pose.reshape(-1).tolist(), "map_T_lidar": lidar_pose.reshape(-1).tolist(),
                        "boxes": boxes, "end_of_scene": sequence == expected - 1}
                    if not dynamic_camera:
                        payload.update(camera_calibration=self.calibration, camera_agent=self.agent)
                    for key in ("camera_distortion", "map_T_nuscenes_global", "gaussian_T_nuscenes_global"):
                        if key in options:
                            payload[key] = json.loads(options[key])
                    yield FoxgloveFrame(sequence, timestamp, image, payload,
                                        values["lidar"][1] if self.include_lidar else None, vehicles)
                    sequence += 1
                if any(len(queue) > 8 for queue in queues.values()):
                    raise ValueError("missing or unbalanced Foxglove frame stream")
            if sequence != expected or any(queues.values()):
                raise ValueError(f"incomplete Foxglove frame join: {sequence}/{expected} frames")

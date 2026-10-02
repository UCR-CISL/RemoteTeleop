"""Stream processed nuScenes into the public CooperScene Foxglove topic layout."""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from src.common.vehicle_categories import vehicle_proxy_kind
from src.data.processed_nuscenes import ProcessedNuScenesSequence


class NuScenesFoxgloveWriter:
    """Write camera, local LiDAR, local/global boxes and synchronized transforms.

    Boxes are never filtered by the camera view. All supported vehicle tracks
    survive export, including vehicles behind the front camera.
    """

    def __init__(self, *, agent: str = "1") -> None:
        self.agent = agent
        self.agent_frame = f"{agent}_base_link"
        self.camera_frame = f"{agent}_cam"

    def write(self, path: Path, sequence: ProcessedNuScenesSequence, *, scene_generation: str) -> None:
        from mcap.writer import Writer
        from mcap_protobuf.schema import register_schema
        from foxglove_schemas_protobuf.CompressedImage_pb2 import CompressedImage
        from foxglove_schemas_protobuf.PointCloud_pb2 import PointCloud
        from foxglove_schemas_protobuf.PackedElementField_pb2 import PackedElementField

        # A partial file never masquerades as a replayable completed scene.
        temporary = path.with_suffix(path.suffix + ".partial")
        first = sequence[0]
        with temporary.open("wb") as stream:
            writer = Writer(stream)
            writer.start()
            channels = {}
            try:
                writer.add_metadata("teleop", {
                    "dataset": "nuscenes", "agent": self.agent,
                    "scene_generation": scene_generation,
                    "frames_per_second": str(1_000_000 / sequence.timestamp_step_us),
                    "camera_frame_id": self.camera_frame,
                    "camera_intrinsic": json.dumps(first.camera_intrinsic.reshape(-1).tolist()),
                    "camera_distortion": json.dumps(first.distortion.tolist()),
                    "camera_T_lidar": json.dumps((np.linalg.inv(first.map_T_camera) @ first.map_T_lidar).reshape(-1).tolist()),
                    "map_T_nuscenes_global": json.dumps(sequence.map_T_global.reshape(-1).tolist()),
                    "gaussian_T_nuscenes_global": json.dumps(sequence.gaussian_T_nuscenes_global.reshape(-1).tolist()),
                })
                for index, frame in enumerate(sequence):
                    timestamp_ns = frame.timestamp_us * 1000
                    def emit(topic, message):
                        if topic not in channels:
                            schema_id = register_schema(writer, type(message))
                            channels[topic] = writer.register_channel(topic=topic, message_encoding="protobuf", schema_id=schema_id)
                        writer.add_message(channels[topic], log_time=timestamp_ns,
                                           data=message.SerializeToString(), publish_time=timestamp_ns, sequence=index)

                    emit("/tf", self._transform(frame.map_T_lidar, "map", self.agent_frame, timestamp_ns))
                    emit("/tf", self._transform(frame.map_T_lidar, "map", f"{self.agent}_ego", timestamp_ns))
                    emit("/tf", self._transform(frame.map_T_camera, "map", self.camera_frame, timestamp_ns))
                    emit(f"/{self.agent}_Bbox", self._boxes(frame, np.eye(4), "map", timestamp_ns, f"{self.agent}_bboxes"))
                    emit("/bbox_global", self._boxes(frame, np.eye(4), "map", timestamp_ns, "global_bboxes"))
                    points = np.ascontiguousarray(frame.lidar_points, dtype="<f4")
                    if points.ndim != 2 or points.shape[1] != 4:
                        raise ValueError("processed LiDAR must have shape (N, 4)")
                    cloud = PointCloud(frame_id=self.agent_frame, point_stride=16, data=points.tobytes())
                    cloud.timestamp.FromNanoseconds(timestamp_ns)
                    cloud.pose.orientation.w = 1
                    for offset, name in enumerate(("x", "y", "z", "intensity")):
                        cloud.fields.add(name=name, offset=offset * 4, type=PackedElementField.FLOAT32)
                    emit(f"/{self.agent}_LIDAR", cloud)
                    ok, encoded = cv2.imencode(".png", frame.image)
                    if not ok:
                        raise ValueError(f"failed to encode image {frame.frame_id}")
                    image = CompressedImage(frame_id=self.camera_frame, format="png", data=encoded.tobytes())
                    image.timestamp.FromNanoseconds(timestamp_ns)
                    emit(f"/{self.agent}_Camera", image)
            finally:
                writer.finish()
        temporary.replace(path)

    @staticmethod
    def _pose(message, matrix):
        message.position.x, message.position.y, message.position.z = matrix[:3, 3]
        q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
        message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = q

    @staticmethod
    def _transform(matrix, parent, child, timestamp_ns):
        from foxglove_schemas_protobuf.FrameTransform_pb2 import FrameTransform
        message = FrameTransform(parent_frame_id=parent, child_frame_id=child)
        message.timestamp.FromNanoseconds(timestamp_ns)
        message.translation.x, message.translation.y, message.translation.z = matrix[:3, 3]
        q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
        message.rotation.x, message.rotation.y, message.rotation.z, message.rotation.w = q
        return message

    def _boxes(self, frame, target_T_map, frame_id, timestamp_ns, entity_id):
        from foxglove_schemas_protobuf.SceneUpdate_pb2 import SceneUpdate
        from foxglove_schemas_protobuf.SceneEntityDeletion_pb2 import SceneEntityDeletion
        update = SceneUpdate()
        deletion = update.deletions.add(type=SceneEntityDeletion.ALL)
        deletion.timestamp.FromNanoseconds(timestamp_ns)
        entity = update.entities.add(id=entity_id, frame_id=frame_id)
        entity.timestamp.FromNanoseconds(timestamp_ns)
        frame_label = update.entities.add(id=entity_id.replace("bboxes", "frame_label"), frame_id=frame_id)
        frame_label.timestamp.FromNanoseconds(timestamp_ns)
        frame_label.texts.add(text=frame.frame_id)
        for vehicle in frame.vehicles:
            if vehicle_proxy_kind(vehicle.category) is None:
                continue
            entity.metadata.add(key=f"label:{vehicle.track_id}", value=vehicle.category)
            cube = entity.cubes.add()
            self._pose(cube.pose, target_T_map @ vehicle.map_T_object)
            cube.size.x, cube.size.y, cube.size.z = vehicle.dimensions_lwh
            cube.color.r, cube.color.g, cube.color.b, cube.color.a = (0.2, 0.7, 1.0, 0.5)
            text = entity.texts.add(text=vehicle.track_id, font_size=0.5, billboard=True)
            text.pose.CopyFrom(cube.pose)
            text.pose.position.z += vehicle.dimensions_lwh[2] / 2 + 0.5
            text.color.r = text.color.g = text.color.b = text.color.a = 1
        return update

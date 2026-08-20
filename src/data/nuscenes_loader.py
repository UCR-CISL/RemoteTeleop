"""Sequence-preserving NuScenes loader for offline teleoperation experiments.

The dataset is indexed by scene rather than by individual frame, so samples from
different drives are never silently joined together.  A frame uses NuScenes
keyframes: ``CAM_FRONT`` and ``LIDAR_TOP`` are associated through the same
sample record.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterator, Sequence

import cv2
import numpy as np


CAMERA_CHANNEL = "CAM_FRONT"
LIDAR_CHANNEL = "LIDAR_TOP"
VEHICLE_CATEGORY_PREFIX = "vehicle."


@dataclass(frozen=True)
class VehicleBox:
    """A vehicle annotation expressed in the current LiDAR coordinate frame.

    ``size_wlh`` is ordered width, length, height, following NuScenes labels.
    ``yaw`` is the box heading about the LiDAR z axis in radians.
    """

    annotation_token: str
    instance_token: str
    category: str
    center: np.ndarray
    size_wlh: np.ndarray
    yaw: float
    world_T_object: np.ndarray = field(default_factory=lambda: np.eye(4, dtype=np.float64))

    @property
    def dimensions_lwh(self) -> np.ndarray:
        """Metric dimensions ordered length, width, height."""

        return self.size_wlh[[1, 0, 2]]

    @property
    def dimensions(self) -> np.ndarray:
        """Alias for the public length, width, height convention."""

        return self.dimensions_lwh


@dataclass(frozen=True)
class NuScenesFrame:
    """All offline inputs and labels for one NuScenes keyframe.

    Absolute transforms use nuScenes global as ``world`` and follow the
    ``target_T_source`` naming rule. ``world_T_ego`` is evaluated at the LiDAR
    exposure; camera and LiDAR transforms each retain their own exposure pose.
    """

    scene_token: str
    sample_token: str
    timestamp_us: int
    image: np.ndarray
    image_path: Path
    lidar_points: np.ndarray
    lidar_path: Path
    camera_intrinsic: np.ndarray
    lidar_to_camera: np.ndarray
    vehicle_boxes: tuple[VehicleBox, ...]
    trajectory: np.ndarray
    trajectory_timestamps_us: np.ndarray
    world_T_ego: np.ndarray = field(default_factory=lambda: np.eye(4, dtype=np.float64))
    world_T_lidar: np.ndarray = field(default_factory=lambda: np.eye(4, dtype=np.float64))
    world_T_camera: np.ndarray = field(default_factory=lambda: np.eye(4, dtype=np.float64))
    camera_timestamp_us: int = 0
    lidar_timestamp_us: int = 0

    @property
    def ego_T_object(self) -> tuple[np.ndarray, ...]:
        """Vehicle poses relative to ego at the LiDAR exposure."""

        ego_T_world = np.linalg.inv(self.world_T_ego)
        return tuple(ego_T_world @ box.world_T_object for box in self.vehicle_boxes)


class NuScenesSequence(Sequence[NuScenesFrame]):
    """One scene, retaining its original chronological keyframe boundaries."""

    def __init__(self, dataset: "NuScenesSequenceDataset", scene: dict) -> None:
        self._dataset = dataset
        self.scene_token = scene["token"]
        self.name = scene["name"]
        self._sample_tokens = tuple(dataset._scene_sample_tokens(scene))

    def __len__(self) -> int:
        return len(self._sample_tokens)

    def __getitem__(self, index: int | slice) -> NuScenesFrame | list[NuScenesFrame]:
        if isinstance(index, slice):
            return [self._dataset._frame_for_sample(token, self.scene_token) for token in self._sample_tokens[index]]
        return self._dataset._frame_for_sample(self._sample_tokens[index], self.scene_token)

    def __iter__(self) -> Iterator[NuScenesFrame]:
        for sample_token in self._sample_tokens:
            yield self._dataset._frame_for_sample(sample_token, self.scene_token)


class NuScenesSequenceDataset(Sequence[NuScenesSequence]):
    """NuScenes scenes exposed as separate, chronologically ordered sequences.

    Args:
        dataroot: Parent NuScenes directory containing ``samples/``, ``sweeps/``
            and the selected ``v1.0-*`` metadata directory.
        version: NuScenes metadata version. Use ``"v1.0-mini"`` for nuScenes-mini.
        camera_channel: Front-facing camera channel; defaults to ``CAM_FRONT``.
        lidar_channel: LiDAR channel; defaults to ``LIDAR_TOP``.
        future_steps: Number of trajectory poses (including the current pose).
            ``None`` keeps every remaining keyframe in that scene.
        nusc: Optional object exposing the NuScenes ``scene`` table and ``get``
            method, useful for dependency injection in tests. Normal callers
            should omit it.
    """

    def __init__(
        self,
        dataroot: str | Path,
        *,
        version: str = "v1.0-mini",
        camera_channel: str = CAMERA_CHANNEL,
        lidar_channel: str = LIDAR_CHANNEL,
        future_steps: int | None = None,
        nusc: object | None = None,
    ) -> None:
        if future_steps is not None and future_steps < 1:
            raise ValueError("future_steps must be at least 1 or None")

        self.dataroot = Path(dataroot)
        self.version = version
        self.camera_channel = camera_channel
        self.lidar_channel = lidar_channel
        self.future_steps = future_steps
        self._nusc = nusc if nusc is not None else _NuScenesMetadata(self.dataroot, self.version)
        self._scenes = tuple(self._nusc.scene)
        self._frame_cache: dict[str, NuScenesFrame] = {}
        self._trajectory_cache: dict[str, tuple[np.ndarray, np.ndarray]] = {}

    def __len__(self) -> int:
        return len(self._scenes)

    def __getitem__(self, index: int | slice) -> NuScenesSequence | list[NuScenesSequence]:
        if isinstance(index, slice):
            return [NuScenesSequence(self, scene) for scene in self._scenes[index]]
        return NuScenesSequence(self, self._scenes[index])

    def __iter__(self) -> Iterator[NuScenesSequence]:
        for scene in self._scenes:
            yield NuScenesSequence(self, scene)

    def _scene_sample_tokens(self, scene: dict) -> Iterator[str]:
        sample_token = scene["first_sample_token"]
        while sample_token:
            sample = self._nusc.get("sample", sample_token)
            yield sample_token
            sample_token = sample["next"]

    def _frame_for_sample(self, sample_token: str, scene_token: str) -> NuScenesFrame:
        if sample_token in self._frame_cache:
            return self._frame_cache[sample_token]

        sample = self._nusc.get("sample", sample_token)
        camera = self._nusc.get("sample_data", sample["data"][self.camera_channel])
        lidar = self._nusc.get("sample_data", sample["data"][self.lidar_channel])
        camera_calibration = self._nusc.get("calibrated_sensor", camera["calibrated_sensor_token"])
        lidar_calibration = self._nusc.get("calibrated_sensor", lidar["calibrated_sensor_token"])
        world_T_ego = _transform(self._nusc.get("ego_pose", lidar["ego_pose_token"]))
        world_T_lidar = self._global_from_sensor(lidar, lidar_calibration)
        world_T_camera = self._global_from_sensor(camera, camera_calibration)

        frame = NuScenesFrame(
            scene_token=scene_token,
            sample_token=sample_token,
            timestamp_us=int(sample["timestamp"]),
            image=self._read_rgb(self._path_for(camera)),
            image_path=self._path_for(camera),
            lidar_points=self._read_lidar(self._path_for(lidar)),
            lidar_path=self._path_for(lidar),
            camera_intrinsic=np.asarray(camera_calibration["camera_intrinsic"], dtype=np.float32),
            lidar_to_camera=self._lidar_to_camera(lidar, camera, lidar_calibration, camera_calibration),
            vehicle_boxes=tuple(self._vehicle_boxes(sample, lidar)),
            trajectory=self._trajectory_for_sample(sample_token)[0],
            trajectory_timestamps_us=self._trajectory_for_sample(sample_token)[1],
            world_T_ego=world_T_ego,
            world_T_lidar=world_T_lidar,
            world_T_camera=world_T_camera,
            camera_timestamp_us=int(camera.get("timestamp", sample["timestamp"])),
            lidar_timestamp_us=int(lidar.get("timestamp", sample["timestamp"])),
        )
        self._frame_cache[sample_token] = frame
        return frame

    def _path_for(self, sample_data: dict) -> Path:
        return self.dataroot / sample_data["filename"]

    @staticmethod
    def _read_rgb(path: Path) -> np.ndarray:
        image_bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image_bgr is None:
            raise FileNotFoundError(f"Could not read NuScenes image: {path}")
        return cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

    @staticmethod
    def _read_lidar(path: Path) -> np.ndarray:
        points = np.fromfile(path, dtype=np.float32)
        if points.size % 5:
            raise ValueError(f"NuScenes LiDAR file does not contain 5-float points: {path}")
        return points.reshape(-1, 5)

    def _lidar_to_camera(
        self, lidar: dict, camera: dict, lidar_calibration: dict, camera_calibration: dict
    ) -> np.ndarray:
        global_from_lidar = self._global_from_sensor(lidar, lidar_calibration)
        global_from_camera = self._global_from_sensor(camera, camera_calibration)
        return (np.linalg.inv(global_from_camera) @ global_from_lidar).astype(np.float32)

    def _global_from_sensor(self, sample_data: dict, calibration: dict) -> np.ndarray:
        global_from_ego = _transform(self._nusc.get("ego_pose", sample_data["ego_pose_token"]))
        ego_from_sensor = _transform(calibration)
        return global_from_ego @ ego_from_sensor

    def _vehicle_boxes(self, sample: dict, lidar: dict) -> Iterator[VehicleBox]:
        global_from_lidar = self._global_from_sensor(
            lidar, self._nusc.get("calibrated_sensor", lidar["calibrated_sensor_token"])
        )
        lidar_from_global = np.linalg.inv(global_from_lidar)
        for annotation_token in sample["anns"]:
            annotation = self._nusc.get("sample_annotation", annotation_token)
            category = self._nusc.get("category", annotation["category_token"])["name"]
            if not category.startswith(VEHICLE_CATEGORY_PREFIX):
                continue
            global_from_box = _transform(annotation)
            lidar_from_box = lidar_from_global @ global_from_box
            yield VehicleBox(
                annotation_token=annotation_token,
                instance_token=annotation["instance_token"],
                category=category,
                center=lidar_from_box[:3, 3].astype(np.float32),
                size_wlh=np.asarray(annotation["size"], dtype=np.float32),
                yaw=float(np.arctan2(lidar_from_box[1, 0], lidar_from_box[0, 0])),
                world_T_object=global_from_box,
            )

    def _trajectory_for_sample(self, sample_token: str) -> tuple[np.ndarray, np.ndarray]:
        cached = self._trajectory_cache.get(sample_token)
        if cached is not None:
            return cached

        sample = self._nusc.get("sample", sample_token)
        lidar = self._nusc.get("sample_data", sample["data"][self.lidar_channel])
        lidar_calibration = self._nusc.get("calibrated_sensor", lidar["calibrated_sensor_token"])
        lidar_from_global = np.linalg.inv(self._global_from_sensor(lidar, lidar_calibration))

        positions: list[np.ndarray] = []
        timestamps: list[int] = []
        next_token = sample_token
        while next_token and (self.future_steps is None or len(positions) < self.future_steps):
            future_sample = self._nusc.get("sample", next_token)
            future_lidar = self._nusc.get("sample_data", future_sample["data"][self.lidar_channel])
            future_pose = self._nusc.get("ego_pose", future_lidar["ego_pose_token"])
            ego_origin_global = np.asarray([*future_pose["translation"], 1.0], dtype=np.float64)
            positions.append((lidar_from_global @ ego_origin_global)[:3].astype(np.float32))
            timestamps.append(int(future_sample["timestamp"]))
            next_token = future_sample["next"]

        trajectory = np.stack(positions) if positions else np.empty((0, 3), dtype=np.float32)
        result = trajectory, np.asarray(timestamps, dtype=np.int64)
        self._trajectory_cache[sample_token] = result
        return result


def _transform(record: dict) -> np.ndarray:
    """Return the homogeneous transform represented by NuScenes pose fields."""

    rotation = _quaternion_to_rotation_matrix(record["rotation"])
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(record["translation"], dtype=np.float64)
    return transform


def _quaternion_to_rotation_matrix(quaternion: Sequence[float]) -> np.ndarray:
    """Convert a NuScenes ``[w, x, y, z]`` quaternion to a rotation matrix."""

    w, x, y, z = np.asarray(quaternion, dtype=np.float64)
    norm = np.linalg.norm((w, x, y, z))
    if norm == 0:
        raise ValueError("Quaternion must not be zero")
    w, x, y, z = (w / norm, x / norm, y / norm, z / norm)
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


class _NuScenesMetadata:
    """Small read-only adapter for NuScenes JSON metadata.

    Keeping this local avoids making the heavyweight devkit a runtime dependency;
    the NuScenes table format is identical for mini and full releases.
    """

    _TABLES = (
        "scene",
        "sample",
        "sample_data",
        "calibrated_sensor",
        "ego_pose",
        "sample_annotation",
        "category",
    )
    _OPTIONAL_TABLES = ("sensor", "instance")

    def __init__(self, dataroot: Path, version: str) -> None:
        import json

        metadata_root = dataroot / version
        if not metadata_root.is_dir():
            raise FileNotFoundError(
                f"NuScenes metadata directory not found: {metadata_root}. "
                "Pass the parent directory containing samples/, sweeps/, and v1.0-mini/."
            )
        self._tables: dict[str, dict[str, dict]] = {}
        for table in self._TABLES:
            path = metadata_root / f"{table}.json"
            if not path.is_file():
                raise FileNotFoundError(f"NuScenes metadata table not found: {path}")
            with path.open(encoding="utf-8") as file:
                records = json.load(file)
            self._tables[table] = {record["token"]: record for record in records}
        for table in self._OPTIONAL_TABLES:
            path = metadata_root / f"{table}.json"
            if path.is_file():
                with path.open(encoding="utf-8") as file:
                    records = json.load(file)
                self._tables[table] = {record["token"]: record for record in records}
        self._build_reverse_indices()
        self.scene = list(self._tables["scene"].values())

    def get(self, table: str, token: str) -> dict:
        try:
            return self._tables[table][token]
        except KeyError as error:
            raise KeyError(f"NuScenes {table} token not found: {token}") from error

    def _build_reverse_indices(self) -> None:
        """Populate the convenience links normally added by nuscenes-devkit."""

        samples = self._tables["sample"]
        for sample in samples.values():
            sample.setdefault("data", {})
            sample.setdefault("anns", [])

        sensors = self._tables.get("sensor", {})
        for sample_data in self._tables["sample_data"].values():
            sample_token = sample_data.get("sample_token")
            if not sample_token or sample_token not in samples:
                continue
            calibration = self._tables["calibrated_sensor"][sample_data["calibrated_sensor_token"]]
            sensor = sensors.get(calibration.get("sensor_token"))
            if sensor is not None:
                samples[sample_token]["data"].setdefault(sensor["channel"], sample_data["token"])

        instances = self._tables.get("instance", {})
        for annotation in self._tables["sample_annotation"].values():
            sample_token = annotation.get("sample_token")
            if sample_token in samples:
                samples[sample_token]["anns"].append(annotation["token"])
            if "category_token" not in annotation:
                instance = instances.get(annotation.get("instance_token"))
                if instance is not None:
                    annotation["category_token"] = instance["category_token"]

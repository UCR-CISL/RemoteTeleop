"""Loader for Bilateral Driving's processed 10 Hz nuScenes scenes."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Iterator, Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class ProcessedNuScenesVehicle:
    """One stable object annotation expressed in the Gaussian map frame."""

    track_id: str
    category: str
    dimensions_lwh: tuple[float, float, float]
    map_T_object: np.ndarray


@dataclass(frozen=True)
class ProcessedNuScenesFrame:
    """Front-camera and LiDAR data for one interpolated 10 Hz frame."""

    index: int
    frame_id: str
    timestamp_us: int
    image: np.ndarray
    image_path: Path
    lidar_points: np.ndarray
    lidar_path: Path
    camera_intrinsic: np.ndarray
    distortion: np.ndarray
    map_T_camera: np.ndarray
    map_T_lidar: np.ndarray
    vehicles: tuple[ProcessedNuScenesVehicle, ...]


class ProcessedNuScenesSequence(Sequence[ProcessedNuScenesFrame]):
    """A contiguous scene expressed in its Gaussian-splat map frame.

    Without ``localization_transform``, this is the legacy Bilateral Driving
    replay: the map is anchored at the first front camera.  Supplying a
    localization artifact instead applies its ``gaussian_T_nuscenes_global``
    to the original NuScenes-global camera, LiDAR, and annotation poses.
    """

    def __init__(
        self,
        root: str | Path,
        *,
        expected_frame_count: int | None = 191,
        timestamp_step_us: int = 100_000,
        localization_transform: str | Path | None = None,
    ) -> None:
        self.root = Path(root)
        if expected_frame_count is not None and expected_frame_count <= 0:
            raise ValueError("expected_frame_count must be positive or None")
        if timestamp_step_us <= 0:
            raise ValueError("timestamp_step_us must be positive")
        self.timestamp_step_us = timestamp_step_us

        image_indices = _numbered_indices(self.root / "images", "_0.jpg")
        if not image_indices:
            raise FileNotFoundError(f"no front-camera images found under {self.root / 'images'}")
        expected_indices = tuple(range(len(image_indices)))
        if image_indices != expected_indices:
            raise ValueError("front-camera frame indices must be contiguous and start at zero")
        if expected_frame_count is not None and len(image_indices) != expected_frame_count:
            raise ValueError(
                f"expected {expected_frame_count} frames, found {len(image_indices)}"
            )
        self._indices = image_indices
        self._require_frame_files()

        intrinsic_values = np.loadtxt(self.root / "intrinsics" / "0.txt", dtype=np.float64).reshape(-1)
        if intrinsic_values.size < 4:
            raise ValueError("front-camera intrinsics must contain at least fx, fy, cx, cy")
        fx, fy, cx, cy = intrinsic_values[:4]
        self.camera_intrinsic = np.asarray(
            ((fx, 0.0, cx), (0.0, fy, cy), (0.0, 0.0, 1.0)), dtype=np.float64
        )
        self.distortion = np.pad(intrinsic_values[4:9], (0, max(0, 5 - intrinsic_values[4:9].size)))
        self.localization_transform = (
            None if localization_transform is None else Path(localization_transform)
        )
        self.gaussian_T_nuscenes_global = (
            np.linalg.inv(self._load_transform("extrinsics", "000_0.txt"))
            if self.localization_transform is None
            else _load_gaussian_T_nuscenes_global(
                self.localization_transform, expected_scene_id=self.root.name
            )
        )
        # Compatibility alias for pre-localization MCAP metadata consumers.
        self.map_T_global = self.gaussian_T_nuscenes_global
        self._instances, self._frame_instances = self._load_instances()

    def __len__(self) -> int:
        return len(self._indices)

    def __getitem__(self, index: int | slice) -> ProcessedNuScenesFrame | list[ProcessedNuScenesFrame]:
        if isinstance(index, slice):
            return [self._frame(frame_index) for frame_index in self._indices[index]]
        frame_index = self._indices[index]
        return self._frame(frame_index)

    def __iter__(self) -> Iterator[ProcessedNuScenesFrame]:
        for frame_index in self._indices:
            yield self._frame(frame_index)

    def _frame(self, frame_index: int) -> ProcessedNuScenesFrame:
        frame_id = f"{frame_index:03d}"
        image_path = self.root / "images" / f"{frame_id}_0.jpg"
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"failed to decode front-camera image: {image_path}")
        lidar_path = self.root / "lidar" / f"{frame_id}.bin"
        lidar_values = np.fromfile(lidar_path, dtype=np.float32)
        if lidar_values.size % 4:
            raise ValueError(f"processed LiDAR must contain four-float points: {lidar_path}")
        return ProcessedNuScenesFrame(
            index=frame_index,
            frame_id=frame_id,
            timestamp_us=frame_index * self.timestamp_step_us,
            image=image,
            image_path=image_path,
            lidar_points=lidar_values.reshape(-1, 4),
            lidar_path=lidar_path,
            camera_intrinsic=self.camera_intrinsic.copy(),
            distortion=self.distortion.copy(),
            map_T_camera=self.gaussian_T_nuscenes_global @ self._load_transform("extrinsics", f"{frame_id}_0.txt"),
            map_T_lidar=self.gaussian_T_nuscenes_global @ self._load_transform("lidar_pose", f"{frame_id}.txt"),
            vehicles=self._vehicles_for_frame(frame_index),
        )

    def _require_frame_files(self) -> None:
        for frame_index in self._indices:
            frame_id = f"{frame_index:03d}"
            for path in (
                self.root / "extrinsics" / f"{frame_id}_0.txt",
                self.root / "lidar" / f"{frame_id}.bin",
                self.root / "lidar_pose" / f"{frame_id}.txt",
            ):
                if not path.is_file():
                    raise FileNotFoundError(f"processed scene is missing {path}")

    def _load_instances(self) -> tuple[dict[str, dict], dict[str, list]]:
        directory = self.root / "instances"
        with (directory / "instances_info.json").open(encoding="utf-8") as stream:
            instances = json.load(stream)
        with (directory / "frame_instances.json").open(encoding="utf-8") as stream:
            frames = json.load(stream)
        return instances, frames

    def _vehicles_for_frame(self, frame_index: int) -> tuple[ProcessedNuScenesVehicle, ...]:
        vehicles = []
        for simplified_id in self._frame_instances.get(str(frame_index), ()):
            key = str(simplified_id)
            info = self._instances[key]
            annotations = info["frame_annotations"]
            try:
                annotation_index = annotations["frame_idx"].index(frame_index)
            except ValueError as error:
                raise ValueError(f"instance {key} is listed in frame {frame_index} without an annotation") from error
            dimensions = tuple(float(value) for value in annotations["box_size"][annotation_index])
            if len(dimensions) != 3 or any(value <= 0 for value in dimensions):
                raise ValueError(f"instance {key} has invalid length/width/height dimensions")
            vehicles.append(ProcessedNuScenesVehicle(
                track_id=str(info["id"]),
                category=str(info["class_name"]),
                dimensions_lwh=dimensions,
                map_T_object=self.gaussian_T_nuscenes_global @ _as_transform(
                    annotations["obj_to_world"][annotation_index], f"instance {key} pose"
                ),
            ))
        return tuple(vehicles)

    def _load_transform(self, directory: str, filename: str) -> np.ndarray:
        return _as_transform(np.loadtxt(self.root / directory / filename), filename)


def _numbered_indices(directory: Path, suffix: str) -> tuple[int, ...]:
    indices = []
    for path in directory.glob(f"*{suffix}"):
        prefix = path.name.removesuffix(suffix)
        if prefix.isdigit():
            indices.append(int(prefix))
    return tuple(sorted(indices))


def _as_transform(value: object, label: str) -> np.ndarray:
    transform = np.asarray(value, dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError(f"{label} must be a finite 4x4 transform")
    if not np.allclose(transform[3], (0.0, 0.0, 0.0, 1.0), atol=1e-7):
        raise ValueError(f"{label} must be homogeneous")
    return transform


def _load_gaussian_T_nuscenes_global(
    path: Path, *, expected_scene_id: str | None = None
) -> np.ndarray:
    """Read the fixed NuScenes-global to Gaussian-map localization handoff."""

    if path.suffix == ".json":
        payload = json.loads(path.read_text(encoding="utf-8"))
        value = payload.get("gaussian_T_nuscenes_global")
        scene_id = payload.get("scene_id")
    elif path.suffix == ".npz":
        with np.load(path) as archive:
            value = archive["gaussian_T_nuscenes_global"] if "gaussian_T_nuscenes_global" in archive else None
            scene_id = str(archive["scene_id"]) if "scene_id" in archive else None
    else:
        raise ValueError("localization_transform must end in .json or .npz")
    if value is None:
        raise ValueError(
            f"localization transform {path} must contain gaussian_T_nuscenes_global"
        )
    if scene_id and expected_scene_id is not None and scene_id != expected_scene_id:
        raise ValueError(
            f"localization transform is for scene {scene_id}, not {expected_scene_id}"
        )
    return _as_transform(value, f"gaussian_T_nuscenes_global in {path}")

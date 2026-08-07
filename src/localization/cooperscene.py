"""Small dependency-free reader for CooperScene LiDAR sequences.

CooperScene's ``lidar_pose`` is the LiDAR sensor pose in its global map frame.
This module calls that frame ``map`` and uses ``target_T_source`` transform
naming, so :attr:`CooperSceneFrame.map_T_lidar` maps LiDAR points into the
CooperScene map.  The Euler convention deliberately matches the project's
OpenCOOD/CARLA converter; it is not a conventional right-handed Rz @ Ry @ Rx.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Sequence

import numpy as np
import yaml


def _readonly_array(values: np.ndarray, *, dtype: np.dtype | type) -> np.ndarray:
    array = np.array(values, dtype=dtype, copy=True)
    array.flags.writeable = False
    return array


def pose_to_matrix(pose: Sequence[float] | np.ndarray) -> np.ndarray:
    """Return CooperScene/CARLA ``map_T_lidar`` for a pose.

    ``pose`` is either a 4x4 matrix or ``[x, y, z, roll_deg, yaw_deg,
    pitch_deg]``.  This is intentionally the same formula as CooperScene's
    ``coop_dataset.pose_to_matrix`` without importing its detector devkit.
    """

    pose_array = np.asarray(pose, dtype=np.float64)
    if pose_array.ndim == 2:
        if pose_array.shape != (4, 4):
            raise ValueError(f"CooperScene pose matrix must have shape (4, 4), got {pose_array.shape}")
        if not np.all(np.isfinite(pose_array)):
            raise ValueError("CooperScene pose must contain only finite values")
        return _readonly_array(pose_array, dtype=np.float64)
    if pose_array.shape != (6,):
        raise ValueError(
            "CooperScene pose must be [x, y, z, roll_deg, yaw_deg, pitch_deg] "
            f"or a 4x4 matrix, got {pose_array.shape}"
        )
    if not np.all(np.isfinite(pose_array)):
        raise ValueError("CooperScene pose must contain only finite values")

    x, y, z, roll_deg, yaw_deg, pitch_deg = pose_array
    roll, yaw, pitch = np.radians([roll_deg, yaw_deg, pitch_deg])
    c_y, s_y = np.cos(yaw), np.sin(yaw)
    c_r, s_r = np.cos(roll), np.sin(roll)
    c_p, s_p = np.cos(pitch), np.sin(pitch)
    rotation = np.array(
        [
            [c_p * c_y, c_y * s_p * s_r - s_y * c_r, -c_y * s_p * c_r - s_y * s_r],
            [s_y * c_p, s_y * s_p * s_r + c_y * c_r, -s_y * s_p * c_r + c_y * s_r],
            [s_p, -c_p * s_r, c_p * c_r],
        ],
        dtype=np.float64,
    )
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = [x, y, z]
    transform.flags.writeable = False
    return transform


def read_ascii_xyzi_pcd(path: str | Path) -> np.ndarray:
    """Read an ASCII PCD with scalar ``x y z intensity`` fields as float32.

    Binary PCD and converted ``.bin`` files are deliberately unsupported: the
    native CooperScene records used for localization are ASCII ``.pcd`` files.
    """

    path = Path(path)
    with path.open("r", encoding="ascii") as stream:
        header: dict[str, list[str]] = {}
        for line in stream:
            words = line.strip().split()
            if not words or words[0].startswith("#"):
                continue
            key, values = words[0].upper(), words[1:]
            header[key] = values
            if key == "DATA":
                break
        else:
            raise ValueError(f"PCD header has no DATA entry: {path}")

        if header.get("DATA", [None])[0].lower() != "ascii":
            raise ValueError(f"Only ASCII PCD is supported: {path}")
        fields = header.get("FIELDS")
        counts = header.get("COUNT", ["1"] * (len(fields) if fields else 0))
        if fields is None or set(fields) != {"x", "y", "z", "intensity"} or len(fields) != 4:
            raise ValueError(f"PCD must contain exactly x y z intensity fields: {path}")
        if len(counts) != 4 or any(count != "1" for count in counts):
            raise ValueError(f"PCD x y z intensity fields must be scalar: {path}")

        data = np.loadtxt(stream, dtype=np.float32, ndmin=2)

    if data.size == 0:
        data = np.empty((0, 4), dtype=np.float32)
    if data.ndim != 2 or data.shape[1] != 4:
        raise ValueError(f"PCD data must have four columns: {path}")
    points = data[:, [fields.index(name) for name in ("x", "y", "z", "intensity")]]
    expected = header.get("POINTS")
    if expected is not None and len(expected) == 1 and points.shape[0] != int(expected[0]):
        raise ValueError(f"PCD POINTS count does not match data rows: {path}")
    return _readonly_array(points, dtype=np.float32)


@dataclass(frozen=True)
class CooperSceneFrame:
    """One native CooperScene LiDAR frame, with immutable numeric payloads."""

    split: str
    scenario: str
    agent: str
    frame_id: str
    lidar_path: Path
    annotation_path: Path
    lidar_points: np.ndarray
    map_T_lidar: np.ndarray

    def __post_init__(self) -> None:
        points = _readonly_array(self.lidar_points, dtype=np.float32)
        transform = _readonly_array(self.map_T_lidar, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 4:
            raise ValueError(f"lidar_points must have shape (N, 4), got {points.shape}")
        if transform.shape != (4, 4):
            raise ValueError(f"map_T_lidar must have shape (4, 4), got {transform.shape}")
        object.__setattr__(self, "lidar_points", points)
        object.__setattr__(self, "map_T_lidar", transform)

    @property
    def timestamp(self) -> int | str:
        """Numeric frame IDs are returned as integers; otherwise retain the ID."""

        return int(self.frame_id) if self.frame_id.isdigit() else self.frame_id


class CooperSceneSequence(Sequence[CooperSceneFrame]):
    """Chronologically ordered frames for one ``split/scenario/agent`` path."""

    def __init__(self, dataset: "CooperSceneSequenceDataset", split: str, scenario: str, agent: str) -> None:
        self._dataset = dataset
        self.split, self.scenario, self.agent = split, scenario, agent
        self.path = dataset.root / split / scenario / agent
        if not self.path.is_dir():
            raise FileNotFoundError(f"CooperScene sequence does not exist: {self.path}")
        self._frame_ids = tuple(sorted(
            (path.stem for path in self.path.glob("*.yaml") if not path.stem.endswith("_additional")
             and not path.stem.endswith("_separate")),
            key=_frame_sort_key,
        ))

    def __len__(self) -> int:
        return len(self._frame_ids)

    def __getitem__(self, index: int | slice) -> CooperSceneFrame | list[CooperSceneFrame]:
        if isinstance(index, slice):
            return [self._dataset._frame(self, frame_id) for frame_id in self._frame_ids[index]]
        return self._dataset._frame(self, self._frame_ids[index])

    def __iter__(self) -> Iterator[CooperSceneFrame]:
        for frame_id in self._frame_ids:
            yield self._dataset._frame(self, frame_id)


class CooperSceneSequenceDataset:
    """Access native CooperScene sequences without the detector devkit."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)
        self._frame_cache: dict[tuple[str, str, str, str], CooperSceneFrame] = {}

    def sequence(self, split: str, scenario: str | int, agent: str | int) -> CooperSceneSequence:
        return CooperSceneSequence(self, str(split), str(scenario), str(agent))

    def _frame(self, sequence: CooperSceneSequence, frame_id: str) -> CooperSceneFrame:
        key = (sequence.split, sequence.scenario, sequence.agent, frame_id)
        cached = self._frame_cache.get(key)
        if cached is not None:
            return cached
        annotation_path = sequence.path / f"{frame_id}.yaml"
        lidar_path = sequence.path / f"{frame_id}.pcd"
        if not lidar_path.is_file():
            raise FileNotFoundError(f"Missing native CooperScene PCD for {annotation_path}: {lidar_path}")
        with annotation_path.open("r", encoding="utf-8") as stream:
            annotation = yaml.safe_load(stream)
        if not isinstance(annotation, dict):
            raise ValueError(f"CooperScene annotation is not a mapping: {annotation_path}")
        pose = annotation.get("lidar_pose", annotation.get("true_ego_pos"))
        if pose is None:
            raise ValueError(f"CooperScene annotation has no lidar_pose: {annotation_path}")
        frame = CooperSceneFrame(
            split=sequence.split,
            scenario=sequence.scenario,
            agent=sequence.agent,
            frame_id=frame_id,
            lidar_path=lidar_path,
            annotation_path=annotation_path,
            lidar_points=read_ascii_xyzi_pcd(lidar_path),
            map_T_lidar=pose_to_matrix(pose),
        )
        self._frame_cache[key] = frame
        return frame


def _frame_sort_key(frame_id: str) -> tuple[int, int | str]:
    return (0, int(frame_id)) if frame_id.isdigit() else (1, frame_id)

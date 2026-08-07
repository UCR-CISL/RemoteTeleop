"""Continuous proxy-to-mesh visualization for asynchronous reconstruction."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path
from typing import Any, Iterable, Mapping, Protocol
from urllib.parse import quote

import numpy as np
import rerun as rr

from src.common.coordinates import Pose3D


class DisplayState(StrEnum):
    """Geometry currently shown for a live track."""

    PROXY = "proxy"
    QUEUED = "queued"
    RUNNING = "running"
    MESH = "mesh"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class TrackKey:
    """A track identity that remains unique when a scene is replayed."""

    scene_generation: str
    track_id: str


@dataclass
class TrackView:
    """Viewer-owned state for one currently live track."""

    key: TrackKey
    world_T_object: np.ndarray
    dimensions_lwh: np.ndarray
    timestamp_us: int
    state: DisplayState = DisplayState.PROXY
    mesh_path: Path | None = None
    request_id: str | None = None


class ReconstructionViewBackend(Protocol):
    """Rendering operations needed by :class:`RealtimeReconstructionViewer`."""

    def set_time(self, timestamp_us: int) -> None: ...

    def show_ego(
        self,
        scene_generation: str,
        world_T_ego: np.ndarray,
        trajectory: np.ndarray,
    ) -> None: ...

    def clear_ego(self, scene_generation: str) -> None: ...

    def show_proxy(self, track: TrackView) -> None: ...

    def update_transform(self, track: TrackView) -> None: ...

    def show_mesh(self, track: TrackView) -> None: ...

    def clear_track(self, key: TrackKey) -> None: ...


class RerunReconstructionBackend:
    """Rerun implementation that keeps each asset in its object-local frame."""

    _PROXY_COLORS = {
        DisplayState.PROXY: [0, 180, 255],
        DisplayState.QUEUED: [255, 210, 0],
        DisplayState.RUNNING: [255, 140, 0],
        DisplayState.FAILED: [255, 40, 40],
        DisplayState.CANCELLED: [140, 140, 140],
    }

    def __init__(
        self, *, application_id: str = "teleop_realtime_reconstruction", spawn: bool = True
    ) -> None:
        rr.init(application_id, spawn=spawn)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    def set_time(self, timestamp_us: int) -> None:
        rr.set_time_nanos("timestamp", timestamp_us * 1_000)

    def show_ego(
        self,
        scene_generation: str,
        world_T_ego: np.ndarray,
        trajectory: np.ndarray,
    ) -> None:
        entity = self._ego_entity(scene_generation)
        position = world_T_ego[:3, 3]
        forward = world_T_ego[:3, 0] * 3.0
        rr.log(
            f"{entity}/trajectory",
            rr.LineStrips3D([trajectory], colors=[40, 120, 255], radii=0.12),
        )
        rr.log(
            f"{entity}/current",
            rr.Points3D(
                [position],
                colors=[40, 120, 255],
                radii=0.45,
                labels=["ego"],
            ),
        )
        rr.log(
            f"{entity}/heading",
            rr.Arrows3D(
                origins=[position],
                vectors=[forward],
                colors=[40, 120, 255],
                radii=0.08,
            ),
        )

    def clear_ego(self, scene_generation: str) -> None:
        rr.log(self._ego_entity(scene_generation), rr.Clear(recursive=True))

    def show_proxy(self, track: TrackView) -> None:
        self.update_transform(track)
        rr.log(
            f"{self._entity(track.key)}/proxy",
            rr.Boxes3D(
                half_sizes=[track.dimensions_lwh / 2.0],
                colors=self._PROXY_COLORS[track.state],
                labels=[f"{track.key.track_id} ({track.state})"],
            ),
        )

    def update_transform(self, track: TrackView) -> None:
        rr.log(
            self._entity(track.key),
            rr.Transform3D(
                translation=track.world_T_object[:3, 3],
                mat3x3=track.world_T_object[:3, :3],
                relation=rr.TransformRelation.ParentFromChild,
            ),
        )

    def show_mesh(self, track: TrackView) -> None:
        if track.mesh_path is None:
            raise ValueError("mesh_path is required when showing a mesh")
        self.update_transform(track)
        entity = self._entity(track.key)
        # Publish the replacement before clearing the proxy so the track never
        # disappears between two Rerun log operations at the same timestamp.
        rr.log(f"{entity}/mesh", rr.Asset3D(path=track.mesh_path))
        rr.log(f"{entity}/proxy", rr.Clear(recursive=True))

    def clear_track(self, key: TrackKey) -> None:
        rr.log(self._entity(key), rr.Clear(recursive=True))

    @staticmethod
    def _entity(key: TrackKey) -> str:
        generation = quote(key.scene_generation, safe="")
        track_id = quote(key.track_id, safe="")
        return f"world/tracks/{generation}/{track_id}"

    @staticmethod
    def _ego_entity(scene_generation: str) -> str:
        generation = quote(scene_generation, safe="")
        return f"world/ego/{generation}"


class RealtimeReconstructionViewer:
    """Maintain live track geometry while reconstruction finishes asynchronously.

    Inputs may be decoded dictionaries or dataclass-like objects. World-state
    messages contain ``scene_generation``, ``timestamp_us`` and ``boxes``.
    Each box contains ``track_id``, ``world_T_object`` and
    ``dimensions_lwh``. Asset events contain ``scene_generation``,
    ``track_id``, ``state`` and, when ready, ``aligned_mesh_path``.
    """

    def __init__(self, backend: ReconstructionViewBackend | None = None) -> None:
        self._backend = backend or RerunReconstructionBackend()
        self._tracks: dict[TrackKey, TrackView] = {}
        self._last_snapshot_us: dict[str, int] = {}
        self._active_generation: str | None = None
        self._retired_generations: set[str] = set()
        self._pending_assets: dict[TrackKey, object] = {}
        self._ego_trajectories: dict[str, list[np.ndarray]] = {}

    @property
    def tracks(self) -> Mapping[TrackKey, TrackView]:
        return dict(self._tracks)

    def handle_world_state(self, message: object) -> None:
        """Apply one full world snapshot and expire tracks absent from it."""

        generation = _required_text(message, "scene_generation")
        timestamp_us = int(_field(message, "timestamp_us"))
        if generation in self._retired_generations:
            return
        if timestamp_us < self._last_snapshot_us.get(generation, -1):
            return

        if generation != self._active_generation:
            if self._active_generation is not None:
                self._retired_generations.add(self._active_generation)
                self._backend.clear_ego(self._active_generation)
            for key in tuple(self._tracks):
                self._backend.clear_track(key)
            self._tracks.clear()
            self._ego_trajectories = {generation: []}
            self._pending_assets = {
                key: event
                for key, event in self._pending_assets.items()
                if key.scene_generation == generation
            }
            self._active_generation = generation

        tracks = _field(message, "boxes")
        if not isinstance(tracks, Iterable):
            raise TypeError("boxes must be iterable")

        self._backend.set_time(timestamp_us)
        world_T_ego = _pose_matrix(_field(message, "world_T_ego"))
        ego_trajectory = self._ego_trajectories.setdefault(generation, [])
        ego_position = world_T_ego[:3, 3].copy()
        if not ego_trajectory or not np.array_equal(ego_trajectory[-1], ego_position):
            ego_trajectory.append(ego_position)
        self._backend.show_ego(
            generation,
            world_T_ego,
            np.asarray(ego_trajectory, dtype=np.float64),
        )
        live_keys: set[TrackKey] = set()
        for payload in tracks:
            key = TrackKey(generation, _required_text(payload, "track_id"))
            live_keys.add(key)
            pose = _pose_matrix(_field(payload, "world_T_object"))
            dimensions = _dimensions(_field(payload, "dimensions_lwh"))
            track = self._tracks.get(key)
            if track is None:
                track = TrackView(key, pose, dimensions, timestamp_us)
                self._tracks[key] = track
                self._backend.show_proxy(track)
                pending = self._pending_assets.pop(key, None)
                if pending is not None:
                    self.handle_asset_event(pending)
                continue

            track.world_T_object = pose
            track.dimensions_lwh = dimensions
            track.timestamp_us = timestamp_us
            if track.state is DisplayState.MESH:
                self._backend.update_transform(track)
            else:
                self._backend.show_proxy(track)

        for key in tuple(self._tracks):
            if key.scene_generation == generation and key not in live_keys:
                self._backend.clear_track(key)
                del self._tracks[key]
        self._last_snapshot_us[generation] = timestamp_us

    def handle_asset_event(self, message: object) -> None:
        """Update reconstruction state without blocking world-state updates."""

        key = TrackKey(
            _required_text(message, "scene_generation"),
            _required_text(message, "track_id"),
        )
        track = self._tracks.get(key)
        if track is None:
            if key.scene_generation not in self._retired_generations:
                self._pending_assets[key] = message
            return

        request_id = _optional_text(message, "request_id")
        if track.request_id is not None and request_id is not None and request_id != track.request_id:
            return
        if request_id is not None:
            track.request_id = request_id

        status_value = str(_field(message, "state")).lower()
        allowed_statuses = {"queued", "running", "ready", "failed", "cancelled"}
        if status_value not in allowed_statuses:
            raise ValueError(
                "asset status must be queued, running, ready, failed, or cancelled"
            )
        if track.state in {
            DisplayState.MESH,
            DisplayState.FAILED,
            DisplayState.CANCELLED,
        }:
            return
        if track.state is DisplayState.RUNNING and status_value == "queued":
            return

        self._backend.set_time(track.timestamp_us)
        if status_value == "ready":
            mesh_path = Path(_field(message, "aligned_mesh_path"))
            if not mesh_path.is_file():
                track.state = DisplayState.FAILED
                self._backend.show_proxy(track)
                return
            track.state = DisplayState.MESH
            track.mesh_path = mesh_path
            self._backend.show_mesh(track)
            return

        status = DisplayState(status_value)
        track.state = status
        self._backend.show_proxy(track)


def _field(value: object, name: str) -> Any:
    if isinstance(value, Mapping):
        if name not in value:
            raise ValueError(f"missing required field {name!r}")
        return value[name]
    try:
        return getattr(value, name)
    except AttributeError as error:
        raise ValueError(f"missing required field {name!r}") from error


def _required_text(value: object, name: str) -> str:
    text = str(_field(value, name))
    if not text:
        raise ValueError(f"{name} must be non-empty")
    return text


def _optional_text(value: object, name: str) -> str | None:
    if isinstance(value, Mapping) and name not in value:
        return None
    if not isinstance(value, Mapping) and not hasattr(value, name):
        return None
    result = _field(value, name)
    return None if result is None else str(result)


def _pose_matrix(value: object) -> np.ndarray:
    matrix = value.matrix if isinstance(value, Pose3D) else np.asarray(value, dtype=np.float64)
    if matrix.shape == (16,):
        matrix = matrix.reshape(4, 4)
    return Pose3D.from_matrix(matrix).matrix


def _dimensions(value: object) -> np.ndarray:
    dimensions = np.asarray(value, dtype=np.float64)
    if (
        dimensions.shape != (3,)
        or not np.all(np.isfinite(dimensions))
        or np.any(dimensions <= 0)
    ):
        raise ValueError("dimensions_lwh must contain three finite positive values")
    return dimensions.copy()

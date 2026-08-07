from dataclasses import dataclass
from pathlib import Path

import numpy as np

from src.common import Pose3D
from src.realtime.protocol import AssetEvent, AssetState, BoxPrompt, FrameDetections
from src.viz.realtime_reconstruction_viewer import (
    DisplayState,
    RealtimeReconstructionViewer,
    RerunReconstructionBackend,
    TrackKey,
    TrackView,
)


class RecordingBackend:
    def __init__(self) -> None:
        self.events: list[tuple] = []

    def set_time(self, timestamp_us: int) -> None:
        self.events.append(("time", timestamp_us))

    def show_ego(self, scene_generation, world_T_ego, trajectory) -> None:
        self.events.append(
            (
                "ego",
                scene_generation,
                tuple(world_T_ego[:3, 3]),
                tuple(map(tuple, trajectory)),
            )
        )

    def clear_ego(self, scene_generation: str) -> None:
        self.events.append(("clear_ego", scene_generation))

    def show_proxy(self, track: TrackView) -> None:
        self.events.append(("proxy", track.key, track.state))

    def update_transform(self, track: TrackView) -> None:
        self.events.append(
            ("transform", track.key, tuple(track.world_T_object[:3, 3]))
        )

    def show_mesh(self, track: TrackView) -> None:
        self.events.append(("mesh", track.key, track.mesh_path))

    def clear_track(self, key: TrackKey) -> None:
        self.events.append(("clear", key))


def test_rerun_backend_sets_source_timestamp_in_nanoseconds(monkeypatch):
    calls = []
    monkeypatch.setattr(
        "src.viz.realtime_reconstruction_viewer.rr.init",
        lambda *_args, **_kwargs: None,
    )
    monkeypatch.setattr(
        "src.viz.realtime_reconstruction_viewer.rr.log",
        lambda *_args, **_kwargs: None,
    )
    monkeypatch.setattr(
        "src.viz.realtime_reconstruction_viewer.rr.set_time_nanos",
        lambda timeline, nanos: calls.append((timeline, nanos)),
    )

    backend = RerunReconstructionBackend(spawn=False)
    backend.set_time(1_532_402_927_647_951)

    assert calls == [("timestamp", 1_532_402_927_647_951_000)]


def _track(track_id: str, x: float = 0.0) -> dict:
    pose = np.eye(4)
    pose[0, 3] = x
    return {
        "track_id": track_id,
        "world_T_object": pose,
        "dimensions_lwh": [4.5, 1.8, 1.5],
    }


def _world(
    generation: str,
    timestamp_us: int,
    *tracks: dict,
    ego_x: float = 0.0,
) -> dict:
    world_T_ego = np.eye(4)
    world_T_ego[0, 3] = ego_x
    return {
        "scene_generation": generation,
        "timestamp_us": timestamp_us,
        "world_T_ego": world_T_ego,
        "boxes": list(tracks),
    }


def test_proxy_is_immediate_and_ready_mesh_replaces_it(tmp_path: Path):
    backend = RecordingBackend()
    viewer = RealtimeReconstructionViewer(backend)
    key = TrackKey("scene-1", "car-1")

    viewer.handle_world_state(_world("scene-1", 100, _track("car-1")))
    viewer.handle_asset_event(
        {
            "scene_generation": "scene-1",
            "track_id": "car-1",
            "request_id": "request-1",
            "state": "queued",
        }
    )
    mesh = tmp_path / "car.glb"
    mesh.write_bytes(b"glTF")
    viewer.handle_asset_event(
        {
            "scene_generation": "scene-1",
            "track_id": "car-1",
            "request_id": "request-1",
            "state": "ready",
            "aligned_mesh_path": mesh,
        }
    )

    assert ("proxy", key, DisplayState.PROXY) in backend.events
    assert ("proxy", key, DisplayState.QUEUED) in backend.events
    assert ("mesh", key, mesh) in backend.events
    assert viewer.tracks[key].state is DisplayState.MESH

    backend.events.clear()
    viewer.handle_world_state(_world("scene-1", 200, _track("car-1", x=3.0)))
    assert backend.events == [
        ("time", 200),
        ("ego", "scene-1", (0.0, 0.0, 0.0), ((0.0, 0.0, 0.0),)),
        ("transform", key, (3.0, 0.0, 0.0)),
    ]


def test_ego_current_location_and_trajectory_follow_replay_generation():
    backend = RecordingBackend()
    viewer = RealtimeReconstructionViewer(backend)

    viewer.handle_world_state(_world("scene-1", 100, ego_x=1.0))
    viewer.handle_world_state(_world("scene-1", 200, ego_x=2.0))
    viewer.handle_world_state(_world("scene-2", 300, ego_x=10.0))

    assert (
        "ego",
        "scene-1",
        (2.0, 0.0, 0.0),
        ((1.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
    ) in backend.events
    assert ("clear_ego", "scene-1") in backend.events
    assert (
        "ego",
        "scene-2",
        (10.0, 0.0, 0.0),
        ((10.0, 0.0, 0.0),),
    ) in backend.events


def test_ready_mesh_arriving_before_visible_track_is_applied(tmp_path: Path):
    backend = RecordingBackend()
    viewer = RealtimeReconstructionViewer(backend)
    key = TrackKey("scene-1", "car-1")
    mesh = tmp_path / "early.glb"
    mesh.write_bytes(b"glTF")

    viewer.handle_asset_event(
        {
            "scene_generation": "scene-1",
            "track_id": "car-1",
            "request_id": "request-1",
            "state": "ready",
            "aligned_mesh_path": mesh,
        }
    )
    viewer.handle_world_state(_world("scene-1", 100, _track("car-1")))

    assert viewer.tracks[key].state is DisplayState.MESH
    assert ("mesh", key, mesh) in backend.events


def test_failed_or_missing_asset_keeps_proxy_visible(tmp_path: Path):
    backend = RecordingBackend()
    viewer = RealtimeReconstructionViewer(backend)
    key = TrackKey("scene-1", "car-1")
    viewer.handle_world_state(_world("scene-1", 100, _track("car-1")))

    viewer.handle_asset_event(
        {
            "scene_generation": "scene-1",
            "track_id": "car-1",
            "state": "ready",
            "aligned_mesh_path": tmp_path / "missing.glb",
        }
    )

    assert viewer.tracks[key].state is DisplayState.FAILED
    assert backend.events[-1] == ("proxy", key, DisplayState.FAILED)
    assert not any(event[0] == "mesh" for event in backend.events)


def test_snapshot_expiry_generation_change_and_late_assets_are_ignored(tmp_path: Path):
    backend = RecordingBackend()
    viewer = RealtimeReconstructionViewer(backend)
    old_key = TrackKey("generation-1", "reused-id")
    new_key = TrackKey("generation-2", "reused-id")

    viewer.handle_world_state(_world("generation-1", 100, _track("reused-id")))
    viewer.handle_world_state(_world("generation-1", 200))
    assert ("clear", old_key) in backend.events

    viewer.handle_world_state(_world("generation-2", 300, _track("reused-id")))
    mesh = tmp_path / "late.glb"
    mesh.write_bytes(b"glTF")
    viewer.handle_asset_event(
        {
            "scene_generation": "generation-1",
            "track_id": "reused-id",
            "state": "ready",
            "aligned_mesh_path": mesh,
        }
    )

    assert new_key in viewer.tracks
    assert viewer.tracks[new_key].state is DisplayState.PROXY
    assert not any(event[:2] == ("mesh", old_key) for event in backend.events)

    backend.events.clear()
    viewer.handle_world_state(_world("generation-1", 400, _track("reused-id")))
    assert new_key in viewer.tracks
    assert old_key not in viewer.tracks
    assert backend.events == []


def test_stale_world_snapshot_cannot_rewind_or_expire_tracks():
    backend = RecordingBackend()
    viewer = RealtimeReconstructionViewer(backend)
    key = TrackKey("scene-1", "car-1")
    viewer.handle_world_state(_world("scene-1", 200, _track("car-1", x=2.0)))

    backend.events.clear()
    viewer.handle_world_state(_world("scene-1", 100))

    assert key in viewer.tracks
    np.testing.assert_allclose(viewer.tracks[key].world_T_object[:3, 3], [2, 0, 0])
    assert backend.events == []


def test_terminal_asset_state_ignores_delayed_events(tmp_path: Path):
    backend = RecordingBackend()
    viewer = RealtimeReconstructionViewer(backend)
    key = TrackKey("scene-1", "car-1")
    viewer.handle_world_state(_world("scene-1", 100, _track("car-1")))
    mesh = tmp_path / "car.glb"
    mesh.write_bytes(b"glTF")
    ready = {
        "scene_generation": "scene-1",
        "track_id": "car-1",
        "request_id": "request-1",
        "state": "ready",
        "aligned_mesh_path": mesh,
    }
    viewer.handle_asset_event(ready)

    backend.events.clear()
    viewer.handle_asset_event(ready)
    viewer.handle_asset_event(
        {
            "scene_generation": "scene-1",
            "track_id": "car-1",
            "request_id": "request-1",
            "state": "running",
        }
    )

    assert viewer.tracks[key].state is DisplayState.MESH
    assert backend.events == []


@dataclass
class DataclassTrack:
    track_id: str
    world_T_object: Pose3D
    dimensions_lwh: tuple[float, float, float]


@dataclass
class DataclassWorld:
    scene_generation: int
    timestamp_us: int
    world_T_ego: Pose3D
    boxes: list[DataclassTrack]


def test_dataclass_messages_and_pose_objects_are_supported():
    backend = RecordingBackend()
    viewer = RealtimeReconstructionViewer(backend)
    viewer.handle_world_state(
        DataclassWorld(
            scene_generation=7,
            timestamp_us=42,
            world_T_ego=Pose3D.identity(),
            boxes=[
                DataclassTrack(
                    "car/with/slash",
                    Pose3D([1, 2, 3], [0, 0, 0, 1]),
                    (4.0, 2.0, 1.5),
                )
            ],
        )
    )

    key = TrackKey("7", "car/with/slash")
    assert key in viewer.tracks
    np.testing.assert_allclose(viewer.tracks[key].world_T_object[:3, 3], [1, 2, 3])


def test_current_wire_protocol_messages_are_consumed_directly(tmp_path: Path):
    backend = RecordingBackend()
    viewer = RealtimeReconstructionViewer(backend)
    identity = tuple(np.eye(4).reshape(-1))
    viewer.handle_world_state(
        FrameDetections(
            frame_id="frame-1",
            request_id="mask-1",
            scene_generation="scene-1",
            timestamp_us=100,
            image_jpeg=b"jpeg",
            camera_intrinsic=tuple(np.eye(3).reshape(-1)),
            world_T_camera=identity,
            world_T_ego=identity,
            boxes=(
                BoxPrompt(
                    track_id="car-1",
                    xyxy=(10, 10, 100, 100),
                    dimensions_lwh=(4.0, 2.0, 1.5),
                    world_T_object=identity,
                ),
            ),
        )
    )
    mesh = tmp_path / "car.glb"
    mesh.write_bytes(b"glTF")
    viewer.handle_asset_event(
        AssetEvent(
            request_id="reconstruction-1",
            scene_generation="scene-1",
            track_id="car-1",
            state=AssetState.READY,
            aligned_mesh_path=mesh,
        )
    )

    assert viewer.tracks[TrackKey("scene-1", "car-1")].state is DisplayState.MESH

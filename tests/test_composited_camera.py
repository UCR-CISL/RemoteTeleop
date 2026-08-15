from pathlib import Path

import cv2
import numpy as np

from src.realtime.protocol import AssetEvent, AssetState, BoxPrompt, FrameDetections
from src.viz.composited_camera import CompositedCameraBackend, MeshRenderBuffers
from src.viz.localization_visualizer import GaussianRenderBuffers


def _jpeg(width=32, height=24):
    ok, encoded = cv2.imencode(".jpg", np.zeros((height, width, 3), dtype=np.uint8))
    assert ok
    return encoded.tobytes()


def _box(track_id="car", z=4.0):
    pose = np.eye(4)
    pose[2, 3] = z
    return BoxPrompt(
        track_id=track_id,
        xyxy=(8, 6, 24, 18),
        dimensions_lwh=(2.0, 2.0, 2.0),
        world_T_object=tuple(pose.reshape(-1)),
    )


def _frame(generation="scene-1", boxes=(), frame_id="1"):
    intrinsic = np.array([[20.0, 0.0, 16.0], [0.0, 20.0, 12.0], [0.0, 0.0, 1.0]])
    identity = tuple(np.eye(4).reshape(-1))
    return FrameDetections(
        frame_id=frame_id,
        request_id=f"frame-{frame_id}",
        scene_generation=generation,
        timestamp_us=int(frame_id),
        image_jpeg=_jpeg(),
        camera_intrinsic=tuple(intrinsic.reshape(-1)),
        world_T_camera=identity,
        world_T_ego=identity,
        boxes=tuple(boxes),
    )


class FakeGaussianRenderer:
    def __init__(self, depth=5.0):
        self.depth = depth

    def render(self, _pose, _intrinsic, image_size):
        width, height = image_size
        return GaussianRenderBuffers(
            np.zeros((height, width, 3), dtype=np.float32),
            np.full((height, width), self.depth, dtype=np.float32),
            np.ones((height, width), dtype=np.float32),
        )


class FakeMeshRenderer:
    def __init__(self):
        self.loads = []
        self.poses = []
        self.fail_render = False

    def load(self, path: Path):
        self.loads.append(path)
        return path.name

    def render(self, handle, pose, _camera_pose, _intrinsic, image_size):
        if self.fail_render:
            raise RuntimeError("render failed")
        self.poses.append((handle, pose.copy()))
        width, height = image_size
        rgb = np.zeros((height, width, 3), dtype=np.float32)
        rgb[..., 0] = 1.0
        alpha = np.zeros((height, width), dtype=np.float32)
        alpha[10:14, 14:18] = 1.0
        return MeshRenderBuffers(
            rgb,
            np.full((height, width), 4.0, dtype=np.float32),
            alpha,
        )


def test_proxy_is_immediate_and_depth_occluded():
    mesh_renderer = FakeMeshRenderer()
    visible = CompositedCameraBackend(FakeGaussianRenderer(5.0), mesh_renderer)
    visible_frame = visible.render_frame(_frame(boxes=(_box(z=4.0),)))
    assert visible_frame.track_states == {"car": "proxy"}
    assert np.any(visible_frame.rgb[..., 1:] > 0)

    occluded = CompositedCameraBackend(FakeGaussianRenderer(2.0), mesh_renderer)
    occluded_frame = occluded.render_frame(_frame(boxes=(_box(z=4.0),)))
    assert not np.any(occluded_frame.rgb > 0)


def test_ready_mesh_atomically_replaces_proxy_and_tracks_gt_pose(tmp_path):
    meshes = FakeMeshRenderer()
    backend = CompositedCameraBackend(FakeGaussianRenderer(), meshes)
    mesh_path = tmp_path / "car.glb"
    mesh_path.write_bytes(b"glTF")
    backend.handle_asset_event(AssetEvent(
        request_id="request-1", scene_generation="scene-1", track_id="car",
        state=AssetState.READY, aligned_mesh_path=mesh_path,
    ))

    result = backend.render_frame(_frame(boxes=(_box(z=4.0),)))
    assert result.track_states == {"car": "mesh"}
    assert np.all(result.rgb[10:14, 14:18, 0] == 1.0)
    assert not np.any(result.rgb[..., 1] > 0)
    assert meshes.loads == [mesh_path]

    backend.render_frame(_frame(boxes=(), frame_id="2"))
    backend.render_frame(_frame(boxes=(_box(z=8.0),), frame_id="3"))
    assert meshes.loads == [mesh_path]
    np.testing.assert_allclose(meshes.poses[-1][1][:3, 3], [0, 0, 8])


def test_failure_retains_proxy_and_scene_reset_rejects_late_asset(tmp_path):
    meshes = FakeMeshRenderer()
    meshes.fail_render = True
    backend = CompositedCameraBackend(FakeGaussianRenderer(), meshes)
    mesh_path = tmp_path / "car.glb"
    mesh_path.write_bytes(b"glTF")
    ready = AssetEvent(
        request_id="request-1", scene_generation="scene-1", track_id="car",
        state=AssetState.READY, aligned_mesh_path=mesh_path,
    )
    backend.handle_asset_event(ready)
    result = backend.render_frame(_frame(boxes=(_box(),)))
    assert result.track_states == {"car": "proxy"}
    assert np.any(result.rgb[..., 1:] > 0)

    backend.render_frame(_frame(generation="scene-2", boxes=()))
    backend.handle_asset_event(ready)
    result = backend.render_frame(_frame(generation="scene-2", boxes=(_box(),)))
    assert result.track_states == {"car": "proxy"}
    assert meshes.loads == [mesh_path]


def test_mesh_behind_gaussian_is_hidden(tmp_path):
    meshes = FakeMeshRenderer()
    backend = CompositedCameraBackend(FakeGaussianRenderer(depth=3.0), meshes)
    mesh_path = tmp_path / "car.glb"
    mesh_path.write_bytes(b"glTF")
    backend.handle_asset_event(AssetEvent(
        request_id="request-1", scene_generation="scene-1", track_id="car",
        state=AssetState.READY, aligned_mesh_path=mesh_path,
    ))
    result = backend.render_frame(_frame(boxes=(_box(),)))
    assert result.track_states == {"car": "mesh"}
    assert not np.any(result.rgb > 0)

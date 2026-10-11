from pathlib import Path

import cv2
import numpy as np
import pytest

from src.realtime.protocol import (
    AssetEvent, AssetState, BoxPrompt, EgoPoseSample, FrameDetections, FrameSnapshot,
)
from src.viz.composited_camera import (
    CompositedCameraBackend,
    MeshRenderBuffers,
    PoseOnlyCompositedCameraBackend,
    RemoteCameraConfig,
    _decimate_mesh_for_render,
    _flat_ground_box_pose,
)
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
    world_T_camera = np.eye(4)
    world_T_camera[2, 3] = -3.0
    identity = tuple(np.eye(4).reshape(-1))
    return FrameDetections(
        frame_id=frame_id,
        request_id=f"frame-{frame_id}",
        scene_generation=generation,
        timestamp_us=int(frame_id),
        image_jpeg=_jpeg(),
        camera_intrinsic=tuple(intrinsic.reshape(-1)),
        world_T_camera=tuple(world_T_camera.reshape(-1)),
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
        self.fail_load = False
        self.fail_render = False

    def load(self, path: Path):
        self.loads.append(path)
        if self.fail_load:
            raise ModuleNotFoundError("No module named 'pytorch3d'")
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


def test_cached_mesh_decimation_reduces_faces_while_preserving_surface_extent():
    coordinates = np.linspace(-2.0, 2.0, 41, dtype=np.float32)
    x, y = np.meshgrid(coordinates, coordinates)
    vertices = np.column_stack((x.reshape(-1), y.reshape(-1), np.zeros(x.size, dtype=np.float32)))
    cells = np.arange(40 * 40, dtype=np.int64).reshape(40, 40)
    top_left = cells + np.arange(40, dtype=np.int64)[:, None]
    faces = np.concatenate((
        np.column_stack((top_left.reshape(-1), (top_left + 1).reshape(-1), (top_left + 41).reshape(-1))),
        np.column_stack(((top_left + 1).reshape(-1), (top_left + 42).reshape(-1), (top_left + 41).reshape(-1))),
    ))
    colors = np.tile(np.array([[0.2, 0.4, 0.6]], dtype=np.float32), (len(vertices), 1))

    reduced_vertices, reduced_faces, reduced_colors = _decimate_mesh_for_render(
        vertices, faces, colors, max_faces=200
    )

    assert len(reduced_faces) <= 200
    assert len(reduced_faces) < len(faces)
    np.testing.assert_allclose(reduced_vertices[:, :2].min(axis=0), [-2.0, -2.0], atol=0.3)
    np.testing.assert_allclose(reduced_vertices[:, :2].max(axis=0), [2.0, 2.0], atol=0.3)
    np.testing.assert_allclose(reduced_colors, np.tile([0.2, 0.4, 0.6], (len(reduced_colors), 1)))


def _pose(sequence=0, timestamp_us=0, generation="scene-1", translation=(0, 0, 0)):
    transform = np.eye(4)
    transform[:3, 3] = translation
    return EgoPoseSample(
        sequence=sequence,
        timestamp_us=timestamp_us,
        scene_generation=generation,
        frame_id=f"frame-{sequence}",
        world_T_ego=tuple(transform.reshape(-1)),
    )


def test_pose_only_renderer_derives_camera_pose_from_remote_config():
    ego_T_camera = np.eye(4)
    ego_T_camera[0, 3] = 2.0
    camera = RemoteCameraConfig(
        ego_T_camera=ego_T_camera,
        camera_intrinsic=np.eye(3),
        image_size=(20, 12),
    )
    gaussian = FakeGaussianRenderer()
    backend = PoseOnlyCompositedCameraBackend(gaussian, camera)

    rendered = backend.render_pose(_pose(translation=(3, 0, 0)))

    np.testing.assert_allclose(rendered.world_T_camera[:3, 3], [5, 0, 0])
    assert rendered.rgb.shape == (12, 20, 3)


def test_local_cooperscene_config_scales_intrinsics_and_uses_inverse_extrinsic():
    camera = RemoteCameraConfig.for_cooperscene_agent("1", image_size=(480, 300))
    from src.localization.cooperscene_calibration import camera_T_lidar, camera_intrinsic

    np.testing.assert_allclose(camera.ego_T_camera, np.linalg.inv(camera_T_lidar("1")))
    np.testing.assert_allclose(camera.camera_intrinsic[0], camera_intrinsic("1")[0] * 0.25)
    np.testing.assert_allclose(camera.camera_intrinsic[1], camera_intrinsic("1")[1] * 0.25)
    assert camera.image_size == (480, 300)


def test_generic_camera_config_scales_source_intrinsics_independently():
    camera = RemoteCameraConfig.from_source_calibration(
        ego_T_camera=np.eye(4),
        camera_intrinsic=np.array([
            [1266.0, 0.0, 816.0], [0.0, 1266.0, 491.0], [0.0, 0.0, 1.0]
        ]),
        source_image_size=(1600, 900),
        image_size=(480, 270),
    )

    np.testing.assert_allclose(
        camera.camera_intrinsic,
        [[379.8, 0.0, 244.8], [0.0, 379.8, 147.3], [0.0, 0.0, 1.0]],
    )


def test_pose_only_renderer_prefers_snapshot_exact_camera_pose():
    exact_pose = np.eye(4)
    exact_pose[:3, 3] = [7.0, 8.0, 9.0]
    snapshot = FrameSnapshot(
        0, 0, "scene-1", "frame-0", tuple(np.eye(4).reshape(-1)),
        world_T_camera=tuple(exact_pose.reshape(-1)),
    )
    backend = PoseOnlyCompositedCameraBackend(
        FakeGaussianRenderer(),
        RemoteCameraConfig(np.eye(4), np.eye(3), (20, 12)),
    )

    rendered = backend.render_pose(snapshot)

    np.testing.assert_allclose(rendered.world_T_camera, exact_pose)


def test_pose_only_renderer_rejects_invalid_pose_or_scene_generation():
    backend = PoseOnlyCompositedCameraBackend(
        FakeGaussianRenderer(),
        RemoteCameraConfig(np.eye(4), np.eye(3), (20, 12)),
    )
    backend.render_pose(_pose())
    with np.testing.assert_raises_regex(ValueError, "mismatched scene generation"):
        backend.render_pose(_pose(sequence=1, timestamp_us=1, generation="scene-2"))
    invalid = _pose(sequence=2, timestamp_us=2)
    object.__setattr__(invalid, "world_T_ego", tuple(np.zeros(16)))
    with np.testing.assert_raises_regex(ValueError, "homogeneous transform"):
        backend.render_pose(invalid)


def test_pose_only_renderer_draws_depth_tested_detection_proxies():
    ego_T_camera = np.eye(4)
    ego_T_camera[2, 3] = -3.0
    camera = RemoteCameraConfig(
        ego_T_camera=ego_T_camera,
        camera_intrinsic=np.array([[20.0, 0.0, 16.0], [0.0, 20.0, 12.0], [0.0, 0.0, 1.0]]),
        image_size=(32, 24),
    )
    backend = PoseOnlyCompositedCameraBackend(FakeGaussianRenderer(depth=5.0), camera, ground_z=0.0)

    visible = backend.render_pose(_pose(), (_box(z=4.0),))
    assert visible.track_states == {"car": "proxy"}
    assert np.any(visible.rgb[..., 1:] > 0)

    hidden = PoseOnlyCompositedCameraBackend(FakeGaussianRenderer(depth=2.0), camera, ground_z=0.0).render_pose(
        _pose(), (_box(z=4.0),)
    )
    assert hidden.track_states == {"car": "proxy"}
    assert not np.any(hidden.rgb > 0)


def test_pose_only_mesh_waits_for_its_source_frame_and_uses_snapshot_box_pose(tmp_path):
    camera = RemoteCameraConfig(
        ego_T_camera=np.eye(4),
        camera_intrinsic=np.array([[20.0, 0.0, 16.0], [0.0, 20.0, 12.0], [0.0, 0.0, 1.0]]),
        image_size=(32, 24),
    )
    meshes = FakeMeshRenderer()
    backend = PoseOnlyCompositedCameraBackend(FakeGaussianRenderer(), camera, meshes)
    mesh_path = tmp_path / "car.glb"
    mesh_path.write_bytes(b"glTF")
    backend.handle_asset_event(AssetEvent(
        request_id="request-1", scene_generation="scene-1", track_id="car",
        state=AssetState.READY, aligned_mesh_path=mesh_path, source_sequence=2,
    ))
    assert backend.drain_mesh_statuses()[0].state == "loaded"

    assert backend.render_pose(_pose(sequence=1), (_box(z=4.0),)).track_states == {"car": "proxy"}
    rendered = backend.render_pose(_pose(sequence=2), (_box(z=8.0),))

    assert rendered.track_states == {"car": "mesh"}
    np.testing.assert_allclose(meshes.poses[-1][1][:3, 3], [0, 0, 8])


def test_flat_ground_anchor_removes_vertical_jitter_without_changing_xy_or_rotation():
    pose = np.eye(4)
    pose[:3, 3] = [4.0, -3.0, 12.0]
    angle = np.deg2rad(20.0)
    pose[:3, :3] = [[np.cos(angle), 0.0, np.sin(angle)], [0.0, 1.0, 0.0], [-np.sin(angle), 0.0, np.cos(angle)]]
    box = BoxPrompt(
        track_id="car", xyxy=(0, 0, 1, 1), dimensions_lwh=(4.0, 2.0, 1.5),
        world_T_object=tuple(pose.reshape(-1)),
    )

    anchored = _flat_ground_box_pose(box, ground_z=0.0)
    corners = np.array([
        [-2.0, -1.0, -0.75], [-2.0, -1.0, 0.75], [-2.0, 1.0, -0.75], [-2.0, 1.0, 0.75],
        [2.0, -1.0, -0.75], [2.0, -1.0, 0.75], [2.0, 1.0, -0.75], [2.0, 1.0, 0.75],
    ])
    world = corners @ anchored[:3, :3].T + anchored[:3, 3]

    assert world[:, 2].min() == pytest.approx(0.0)
    np.testing.assert_allclose(anchored[:2, 3], pose[:2, 3])
    np.testing.assert_allclose(anchored[:3, :3], pose[:3, :3])


def test_disabled_flat_ground_anchor_preserves_model_frame_object_pose():
    box = _box(z=12.0)

    preserved = _flat_ground_box_pose(box, ground_z=None)

    np.testing.assert_allclose(preserved, np.asarray(box.world_T_object).reshape(4, 4))


def test_pose_only_mesh_load_failure_is_visible_and_keeps_the_proxy(tmp_path):
    camera = RemoteCameraConfig(np.eye(4), np.eye(3), (32, 24))
    meshes = FakeMeshRenderer()
    meshes.fail_load = True
    backend = PoseOnlyCompositedCameraBackend(FakeGaussianRenderer(), camera, meshes)
    mesh_path = tmp_path / "car.glb"
    mesh_path.write_bytes(b"glTF")

    backend.handle_asset_event(AssetEvent(
        request_id="request-1", scene_generation="scene-1", track_id="car",
        state=AssetState.READY, aligned_mesh_path=mesh_path, source_sequence=0,
    ))

    status = backend.drain_mesh_statuses()
    assert len(status) == 1
    assert status[0].state == "load_failed"
    assert "ModuleNotFoundError" in status[0].error
    assert backend.render_pose(_pose(), (_box(),)).track_states == {"car": "proxy"}


def test_pose_only_mesh_render_failure_is_visible_and_keeps_the_proxy(tmp_path):
    camera = RemoteCameraConfig(np.eye(4), np.eye(3), (32, 24))
    meshes = FakeMeshRenderer()
    meshes.fail_render = True
    backend = PoseOnlyCompositedCameraBackend(FakeGaussianRenderer(), camera, meshes)
    mesh_path = tmp_path / "car.glb"
    mesh_path.write_bytes(b"glTF")
    backend.handle_asset_event(AssetEvent(
        request_id="request-1", scene_generation="scene-1", track_id="car",
        state=AssetState.READY, aligned_mesh_path=mesh_path, source_sequence=0,
    ))
    backend.drain_mesh_statuses()

    assert backend.render_pose(_pose(), (_box(),)).track_states == {"car": "proxy"}
    status = backend.drain_mesh_statuses()
    assert len(status) == 1
    assert status[0].state == "render_failed"
    assert "RuntimeError: render failed" == status[0].error


def test_pose_only_mesh_without_source_sequence_waits_for_source_timestamp(tmp_path):
    camera = RemoteCameraConfig(
        ego_T_camera=np.eye(4),
        camera_intrinsic=np.array([[20.0, 0.0, 16.0], [0.0, 20.0, 12.0], [0.0, 0.0, 1.0]]),
        image_size=(32, 24),
    )
    meshes = FakeMeshRenderer()
    backend = PoseOnlyCompositedCameraBackend(FakeGaussianRenderer(), camera, meshes)
    mesh_path = tmp_path / "legacy.glb"
    mesh_path.write_bytes(b"glTF")
    backend.handle_asset_event(AssetEvent(
        request_id="request-1", scene_generation="scene-1", track_id="car",
        state=AssetState.READY, aligned_mesh_path=mesh_path,
        source_timestamp_us=200_000,
    ))

    assert backend.render_pose(_pose(sequence=10, timestamp_us=100_000), (_box(),)).track_states == {"car": "proxy"}
    assert backend.render_pose(_pose(sequence=11, timestamp_us=200_000), (_box(),)).track_states == {"car": "mesh"}


def test_proxy_is_immediate_and_depth_occluded():
    mesh_renderer = FakeMeshRenderer()
    visible = CompositedCameraBackend(FakeGaussianRenderer(5.0), mesh_renderer, ground_z=0.0)
    visible_frame = visible.render_frame(_frame(boxes=(_box(z=4.0),)))
    assert visible_frame.track_states == {"car": "proxy"}
    assert np.any(visible_frame.rgb[..., 1:] > 0)

    occluded = CompositedCameraBackend(FakeGaussianRenderer(2.0), mesh_renderer, ground_z=0.0)
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
    backend = CompositedCameraBackend(FakeGaussianRenderer(), meshes, ground_z=0.0)
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


class FakeVehicleProxies:
    def __init__(self, renderer):
        self.renderer = renderer
        self.calls = []

    def render(self, kind, dimensions, pose, camera, intrinsic, image_size):
        self.calls.append((kind, dimensions, pose.copy()))
        return self.renderer.render(kind, pose, camera, intrinsic, image_size)


def test_vehicle_proxy_receives_each_current_bbox_and_rigid_pose():
    from dataclasses import replace

    renderer = FakeMeshRenderer()
    proxies = FakeVehicleProxies(renderer)
    camera = RemoteCameraConfig(np.eye(4), np.eye(3), (32, 24))
    backend = PoseOnlyCompositedCameraBackend(
        FakeGaussianRenderer(), camera, renderer, vehicle_proxies=proxies,
    )
    pose = np.eye(4)
    pose[:3, :3] = [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]
    pose[:3, 3] = [2., 3., 4.]
    car = replace(_box(), text_label="car", dimensions_lwh=(4.6, 1.85, 1.5),
                  world_T_object=tuple(pose.reshape(-1)))
    bus = replace(car, track_id="bus", text_label="vehicle.bus.bendy", dimensions_lwh=(18., 2.5, 3.2))
    result = backend.render_pose(_pose(), (car, bus))
    assert result.track_states == {"car": "mesh", "bus": "mesh"}
    assert [call[0] for call in proxies.calls] == ["car", "articulated_bus"]
    for (_, dimensions, model), box in zip(proxies.calls, (car, bus)):
        assert dimensions == box.dimensions_lwh
        np.testing.assert_allclose(model, pose)
    assert result.ground_placements['car']['status'] == 'preserve_input_pose'
    assert backend.render_pose(_pose(1, 100_000), ()).track_states == {}
    tall = replace(car, dimensions_lwh=(4.6, 1.85, 3.))
    backend.render_pose(_pose(2, 200_000), (tall,))
    assert proxies.calls[-1][1] == tall.dimensions_lwh


def test_vehicle_proxy_preserves_ground_occlusion_and_nonvehicle_boxes():
    from dataclasses import replace

    renderer = FakeMeshRenderer()
    proxies = FakeVehicleProxies(renderer)
    backend = PoseOnlyCompositedCameraBackend(
        FakeGaussianRenderer(depth=3.), RemoteCameraConfig(np.eye(4), np.eye(3), (32, 24)),
        renderer, ground_z=0., vehicle_proxies=proxies,
    )
    car = replace(_box(), text_label="vehicle.truck", dimensions_lwh=(8., 2.5, 3.))
    result = backend.render_pose(_pose(), (car,))
    assert result.track_states == {"car": "mesh"}
    assert not result.rgb.any()  # Mesh depth 4 is behind opaque scene depth 3.
    np.testing.assert_allclose(renderer.poses[-1][1][2, 3], 1.5)
    pedestrian = replace(car, track_id="person", text_label="pedestrian")
    assert backend.render_pose(_pose(1, 100_000), (pedestrian,)).track_states == {"person": "proxy"}
    assert len(renderer.poses) == 1


def test_vehicle_proxy_errors_are_explicit():
    from dataclasses import replace

    renderer = FakeMeshRenderer()
    backend = PoseOnlyCompositedCameraBackend(
        FakeGaussianRenderer(), RemoteCameraConfig(np.eye(4), np.eye(3), (32, 24)),
        renderer, vehicle_proxies=FakeVehicleProxies(renderer),
    )
    renderer.fail_render = True
    with pytest.raises(RuntimeError, match="render failed"):
        backend.render_pose(_pose(), (replace(_box(), text_label="car"),))


@pytest.mark.parametrize('backend_kind', ['frame', 'pose', 'vehicle_proxy'])
@pytest.mark.parametrize('reverse', [False, True])
def test_overlapping_meshes_share_per_pixel_depth_in_either_track_order(tmp_path, backend_kind, reverse):
    """Intersecting depth fields defeat whole-object sorting; 1cm defeats tolerance reuse."""
    from dataclasses import replace

    class OverlapRenderer(FakeMeshRenderer):
        def render(self, handle, pose, camera, intrinsic, image_size):
            width, height = image_size
            red = pose[0, 3] < 0
            rgb = np.empty((height, width, 3), dtype=np.float32)
            rgb[:] = [1, 0, 0] if red else [0, 0, 1]
            depth = np.full((height, width), 4.01 if red else 4.0, dtype=np.float32)
            depth[:, :width // 2] = 4.0 if red else 4.01
            return MeshRenderBuffers(rgb, depth, np.ones_like(depth))

    renderer = OverlapRenderer()
    gaussian = FakeGaussianRenderer(depth=10)
    if backend_kind == 'frame':
        backend = CompositedCameraBackend(gaussian, renderer)
    else:
        backend = PoseOnlyCompositedCameraBackend(
            gaussian, RemoteCameraConfig(np.eye(4), np.eye(3), (32, 24)), renderer,
            ground_z=None, vehicle_proxies=FakeVehicleProxies(renderer) if backend_kind == 'vehicle_proxy' else None,
        )
    boxes = []
    for track, x in [('red', -1), ('blue', 1)]:
        pose = np.eye(4)
        pose[:3, 3] = [x, 0, 4]
        boxes.append(replace(_box(track), world_T_object=tuple(pose.reshape(-1)), text_label='car'))
        if backend_kind != 'vehicle_proxy':
            path = tmp_path / f'{track}.glb'
            path.write_bytes(b'glTF')
            backend.handle_asset_event(AssetEvent(request_id=track, scene_generation='scene-1', track_id=track,
                                                state=AssetState.READY, aligned_mesh_path=path, source_sequence=0))
    if reverse:
        boxes.reverse()
    result = backend.render_frame(_frame(boxes=boxes)) if backend_kind == 'frame' else backend.render_pose(_pose(sequence=1), tuple(boxes))
    np.testing.assert_array_equal(result.rgb[:, :16], np.broadcast_to([1, 0, 0], (24, 16, 3)))
    np.testing.assert_array_equal(result.rgb[:, 16:], np.broadcast_to([0, 0, 1], (24, 16, 3)))
    # A later frame starts with a fresh object depth buffer.
    empty = backend.render_frame(_frame(boxes=())) if backend_kind == 'frame' else backend.render_pose(_pose(sequence=2), ())
    assert not np.any(empty.rgb)


def test_far_wireframe_edge_cannot_overwrite_nearer_mesh():
    from src.viz.composited_camera import _draw_depth_tested_edge

    rgb = np.zeros((8, 8, 3), dtype=np.float32)
    rgb[:] = [1, 0, 0]
    object_depth = np.full((8, 8), 4.0)
    _draw_depth_tested_edge(rgb, np.full((8, 8), 10), np.ones((8, 8)),
                           np.array([1, 4]), np.array([6, 4]), 4.01, 4.01,
                           np.array([0, 1, 0]), 1, .01, .03, shared_depth=object_depth)
    np.testing.assert_array_equal(rgb, np.broadcast_to([1, 0, 0], rgb.shape))
    _draw_depth_tested_edge(rgb, np.full((8, 8), 10), np.ones((8, 8)),
                           np.array([1, 4]), np.array([6, 4]), 3.0, 3.0,
                           np.array([0, 1, 0]), 1, .01, .03, shared_depth=object_depth)
    np.testing.assert_array_equal(rgb[4, 1:7], np.broadcast_to([0, 1, 0], (6, 3)))
    np.testing.assert_allclose(object_depth[4, 1:7], np.full(6, 3.0))


def test_bounded_surface_reports_each_track_and_preserves_unsupported_pose():
    from dataclasses import replace
    from src.localization.ground_surface import load_ground_placement

    surface = load_ground_placement(dict(type='road_surface', origin=[0, 0, 0],
        basis=np.eye(3).tolist(), samples=[[x, y, 0] for x in (-2, 0, 2) for y in (-2, 0, 2)]))
    renderer = FakeMeshRenderer()
    backend = PoseOnlyCompositedCameraBackend(
        FakeGaussianRenderer(), RemoteCameraConfig(np.eye(4), np.eye(3), (32, 24)), renderer,
        ground_z=None, ground_plane=surface, vehicle_proxies=FakeVehicleProxies(renderer),
    )
    supported = replace(_box('supported', z=4), text_label='car')
    unsupported_pose = np.eye(4)
    unsupported_pose[:3, 3] = [50, 0, 8]
    unsupported = replace(_box('unsupported'), world_T_object=tuple(unsupported_pose.reshape(-1)), text_label='car')
    frame = backend.render_pose(_pose(sequence=0), (supported, unsupported))
    assert frame.ground_placements['supported']['status'] == 'local_map_road'
    assert frame.ground_placements['unsupported']['status'] == 'unsupported_preserve_input_pose'
    assert frame.ground_placements['supported']['support_count'] == 9
    np.testing.assert_allclose(renderer.poses[0][1][:3, 3], [0, 0, 1])
    np.testing.assert_allclose(renderer.poses[1][1], unsupported_pose)
    # Per-frame maps cannot retain absent tracks or mutate earlier diagnostics.
    assert backend.render_pose(_pose(sequence=1), ()).ground_placements == {}
    assert frame.ground_placements['supported']['status'] == 'local_map_road'
    with pytest.raises(ValueError, match='mismatched scene generation'):
        backend.render_pose(_pose(sequence=2, generation='new-scene'), (supported,))
    replacement = PoseOnlyCompositedCameraBackend(
        FakeGaussianRenderer(), RemoteCameraConfig(np.eye(4), np.eye(3), (32, 24)),
        ground_z=None, ground_plane=surface,
    )
    assert replacement.render_pose(_pose(generation='new-scene'), ()).ground_placements == {}


def test_explicit_plane_retains_height_and_reports_placement_status():
    from dataclasses import replace
    from src.localization.ground_surface import load_ground_placement

    plane = load_ground_placement({'normal': [0, 0, 1], 'offset_m': 2.8})
    renderer = FakeMeshRenderer()
    backend = PoseOnlyCompositedCameraBackend(
        FakeGaussianRenderer(), RemoteCameraConfig(np.eye(4), np.eye(3), (32, 24)), renderer,
        ground_z=None, ground_plane=plane, vehicle_proxies=FakeVehicleProxies(renderer),
    )
    frame = backend.render_pose(_pose(), (replace(_box(), text_label='car'),))
    assert frame.ground_placements['car'] == {'status': 'explicit_plane'}
    assert renderer.poses[0][1][2, 3] == pytest.approx(-1.8)

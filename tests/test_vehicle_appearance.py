"""Bounded per-track vehicle shape/color hints from synthetic scans of moving cars."""
from __future__ import annotations

import json
import subprocess
import sys

import cv2
import numpy as np
import pytest

from src.deployment.lidar_ground_snap import _BevGrid
from src.deployment.vehicle_appearance import AppearanceCamera, VehicleAppearanceConfig, VehicleAppearanceEstimator
from src.realtime.bevfusion_tracker import DetectionMatch, TrackedWorldBox
from src.realtime.protocol import BoxPrompt
from src.viz.primitive_vehicle import CarShapeParams

DIMENSIONS = (4.5, 1.8, 1.5)
K = np.array([[500., 0, 320], [0, 500., 240], [0, 0, 1]])
SIZE = (640, 480)
# CV camera at the LiDAR origin looking along LiDAR +X (x right, y down, z forward).
CAMERA_T_LIDAR = np.array([[0., -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
# World differs from the LiDAR frame so frame conversions are exercised.
WORLD_T_LIDAR = np.array([[0., -1, 0, 100], [1, 0, 0, -20], [0, 0, 1, 2], [0, 0, 0, 1]])
PAINT_BGR = (40, 50, 190)


def _car_triangles():
    from src.viz.primitive_car import PrimitiveCarAsset

    asset = PrimitiveCarAsset()
    return asset.vertices_for_dimensions(np.asarray(DIMENSIONS))[asset.mesh.faces]


TRIANGLES = _car_triangles()


def _lidar_T_object(x, y=-3., yaw=0.):
    pose = np.eye(4)
    pose[:3, :3] = ((np.cos(yaw), -np.sin(yaw), 0), (np.sin(yaw), np.cos(yaw), 0), (0, 0, 1))
    pose[:3, 3] = (x, y, -1.)  # LiDAR 1.75 m above the ground
    return pose


def _scan(lidar_T_objects, *, count=400, seed=0):
    """LiDAR-frame returns on sensor-facing car faces of each pose plus a ground plane."""
    rng = np.random.default_rng(seed)
    clouds = []
    for pose in lidar_T_objects:
        sensor = -pose[:3, :3].T @ pose[:3, 3]  # sensor position in the object frame
        normals = np.cross(TRIANGLES[:, 1] - TRIANGLES[:, 0], TRIANGLES[:, 2] - TRIANGLES[:, 0])
        areas = np.linalg.norm(normals, axis=1)
        toward = sensor - TRIANGLES.mean(axis=1)
        facing = np.einsum("ij,ij->i", normals / np.maximum(areas, 1e-12)[:, None],
                           toward / np.linalg.norm(toward, axis=1, keepdims=True)) > .2
        weight = areas * facing
        faces = rng.choice(len(TRIANGLES), count, p=weight / weight.sum())
        a, b = rng.random((2, count))
        flip = a + b > 1
        a[flip], b[flip] = 1 - a[flip], 1 - b[flip]
        t = TRIANGLES[faces]
        points = t[:, 0] + a[:, None] * (t[:, 1] - t[:, 0]) + b[:, None] * (t[:, 2] - t[:, 0])
        points += rng.normal(0, .015, points.shape)
        clouds.append(points @ pose[:3, :3].T + pose[:3, 3])
    ground = np.c_[rng.uniform(3, 30, 3000), rng.uniform(-10, 10, 3000), np.full(3000, -1.75)]
    xyz = np.vstack([*clouds, ground])
    return np.c_[xyz, np.zeros(len(xyz))].astype(np.float32)


def _box(track_id, lidar_T_object, dimensions=DIMENSIONS, label="car"):
    return TrackedWorldBox(track_id, WORLD_T_LIDAR @ lidar_T_object, tuple(dimensions), label, .9)


def _match(box, filter_id=None, index=0):
    return DetectionMatch(index, box, box.track_id if filter_id is None else filter_id)


def _image(lidar_T_objects):
    image = np.full((SIZE[1], SIZE[0], 3), (90, 120, 80), np.uint8)
    signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], dtype=float)
    for pose in lidar_T_objects:
        camera = (CAMERA_T_LIDAR @ pose)
        corners = signs * np.asarray(DIMENSIONS) / 2 @ camera[:3, :3].T + camera[:3, 3]
        uv = corners @ K.T
        uv = uv[:, :2] / uv[:, 2:]
        cv2.fillConvexPoly(image, cv2.convexHull(np.rint(uv).astype(np.int32)), PAINT_BGR)
    noise = np.random.default_rng(1).normal(0, 2, image.shape)
    return np.clip(image + noise, 0, 255).astype(np.uint8)


def _drive(estimator, frames, *, track_id="bevfusion:car:0", filter_id=None, camera=True, start=0,
           dimensions=DIMENSIONS):
    for frame in range(start, start + frames):
        pose = _lidar_T_object(12. + .5 * frame)
        box = _box(track_id, pose, dimensions)
        estimator.update([_match(box, filter_id)], [box], timestamp_us=frame * 100_000, world_T_lidar=WORLD_T_LIDAR,
                         lidar_points=_scan([pose], seed=frame),
                         camera=AppearanceCamera(_image([pose]), K, CAMERA_T_LIDAR) if camera else None)


def test_module_imports_without_rendering_dependencies():
    code = ("import sys, src.deployment.vehicle_appearance; "
            "print([name for name in ('trimesh', 'torch', 'open3d', 'pytorch3d') if name in sys.modules])")
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True)
    assert result.stdout.strip() == "[]"


def test_moving_car_gets_accepted_shape_and_published_paint():
    estimator = VehicleAppearanceEstimator()
    _drive(estimator, 12)

    fields = estimator.prompt_fields("bevfusion:car:0")
    shape = CarShapeParams.from_vector(fields["vehicle_shape"])
    assert shape.body_type in CarShapeParams.BODY_TYPES
    assert 1 <= estimator.fit_count <= 12 * estimator.config.max_fits_per_update
    rgb = np.float32([[fields["body_color_rgb"], np.array(PAINT_BGR[::-1]) / 255]])
    lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2Lab)[0]
    assert np.linalg.norm(lab[0] - lab[1]) < 6
    assert .5 <= fields["color_confidence"] <= 1
    prompt = BoxPrompt("bevfusion:car:0", (1, 2, 30, 40), DIMENSIONS, tuple(np.eye(4).reshape(-1)), **fields)
    assert BoxPrompt.from_dict(json.loads(json.dumps(prompt.to_dict()))) == prompt


def test_color_waits_for_several_observations_and_needs_an_image():
    estimator = VehicleAppearanceEstimator(VehicleAppearanceConfig(shape=False))
    _drive(estimator, 2)
    assert estimator.prompt_fields("bevfusion:car:0") == {}  # fewer than min_observations
    _drive(estimator, 2, start=2)
    assert set(estimator.prompt_fields("bevfusion:car:0")) == {"body_color_rgb", "color_confidence"}
    no_camera = VehicleAppearanceEstimator(VehicleAppearanceConfig(shape=False))
    _drive(no_camera, 6, camera=False)
    assert no_camera.prompt_fields("bevfusion:car:0") == {}


def test_crop_excludes_neighbor_returns_and_ground():
    estimator = VehicleAppearanceEstimator(VehicleAppearanceConfig(color=False))
    pose = _lidar_T_object(12.)
    neighbor = _lidar_T_object(16.6)  # queued 0.1 m ahead: its rear face lies inside the padded crop
    box, other = _box("a", pose), _box("b", neighbor)
    points = _scan([pose, neighbor])
    grid = _BevGrid(points[:, 0], points[:, 1], 2.)
    dimensions = np.asarray(DIMENSIONS)
    alone = estimator._crop(points[:, :3], grid, pose, dimensions, "a", [])
    cropped = estimator._crop(points[:, :3], grid, pose, dimensions, "a",
                              [(neighbor, dimensions, "b"), (pose, dimensions, "a")])
    assert alone[:, 0].max() > dimensions[0] / 2  # neighbor returns leak in without exclusion
    assert len(cropped) > 100 and cropped[:, 0].max() < 4.6 - dimensions[0] / 2 - .3
    assert np.all(np.abs(cropped[:, :2]) <= dimensions[:2] / 2 + .15)
    estimator.update([_match(box), _match(other, index=1)], [box, other], timestamp_us=0,
                     world_T_lidar=WORLD_T_LIDAR, lidar_points=points)
    stored, _ = estimator.accumulator.points("a")
    assert len(stored) and stored[:, 2].min() * DIMENSIONS[2] >= -DIMENSIONS[2] / 2 + .15 - 1e-9  # no ground


def test_coasted_tracks_are_not_fed_and_dead_tracks_are_dropped():
    estimator = VehicleAppearanceEstimator()
    _drive(estimator, 6)
    evidence = estimator.accumulator.evidence("bevfusion:car:0")
    assert evidence and estimator.prompt_fields("bevfusion:car:0")
    coasting = _box("bevfusion:car:0", _lidar_T_object(20.))
    estimator.update([], [coasting], timestamp_us=700_000, world_T_lidar=WORLD_T_LIDAR,
                     lidar_points=_scan([_lidar_T_object(20.)]))
    assert estimator.accumulator.evidence("bevfusion:car:0") == evidence
    estimator.update([], [], timestamp_us=800_000, world_T_lidar=WORLD_T_LIDAR, lidar_points=_scan([]))
    assert estimator.prompt_fields("bevfusion:car:0") == {}
    assert estimator.accumulator.evidence("bevfusion:car:0") == 0
    assert estimator.color_estimator.color("bevfusion:car:0") is None


def test_reacquired_identity_with_a_new_filter_starts_over():
    estimator = VehicleAppearanceEstimator()
    _drive(estimator, 6, filter_id="bevfusion:car:0")
    assert "body_color_rgb" in estimator.prompt_fields("bevfusion:car:0")
    evidence = estimator.accumulator.evidence("bevfusion:car:0")
    _drive(estimator, 1, filter_id="bevfusion:car:7", start=6)
    # Paint needs fresh observations; shape evidence restarts from this one scan.
    assert "body_color_rgb" not in estimator.prompt_fields("bevfusion:car:0")
    assert 0 < estimator.accumulator.evidence("bevfusion:car:0") < evidence


def test_refits_are_bounded_per_update_round_robin():
    estimator = VehicleAppearanceEstimator(VehicleAppearanceConfig(color=False, max_fits_per_update=1))
    poses = [_lidar_T_object(12. + 8 * index, y=-3. - 6 * (index % 2)) for index in range(3)]
    fitted = []
    for frame in range(4):
        boxes = [_box(f"car{index}", pose) for index, pose in enumerate(poses)]
        before = estimator.fit_count
        estimator.update([_match(box, index=i) for i, box in enumerate(boxes)], boxes, timestamp_us=frame,
                         world_T_lidar=WORLD_T_LIDAR, lidar_points=_scan(poses, seed=frame))
        assert estimator.fit_count - before <= 1
        fitted.append(sorted(track_id for track_id, state in estimator._shapes.items() if state.last_fit_tick > 0))
    assert fitted[2] == ["car0", "car1", "car2"]  # every track got a turn


def test_stable_shape_freezes_until_tracked_dimensions_change():
    estimator = VehicleAppearanceEstimator(VehicleAppearanceConfig(
        color=False, refit_every_scans=1, min_param_change=.5, freeze_after_stable_fits=2))
    _drive(estimator, 4, camera=False)
    state = estimator._shapes["bevfusion:car:0"]
    assert state.frozen_dimensions is not None
    published, fits = state.published, estimator.fit_count
    _drive(estimator, 3, camera=False, start=4)
    assert estimator.fit_count == fits and state.published == published
    _drive(estimator, 1, camera=False, start=7, dimensions=(5.2, 1.8, 1.5))  # > 10% longer
    assert state.frozen_dimensions is None and estimator.fit_count == fits + 1


def test_small_shape_changes_keep_the_published_params():
    estimator = VehicleAppearanceEstimator(VehicleAppearanceConfig(color=False))
    published = CarShapeParams()
    assert not estimator._changed(published, CarShapeParams(hood_z=published.hood_z + .004))
    assert estimator._changed(published, CarShapeParams(hood_z=published.hood_z + .01))
    assert estimator._changed(published, CarShapeParams(forward_sign=-1.))


@pytest.mark.parametrize("label", ["pedestrian", "bicycle"])
def test_non_car_labels_get_no_shape_and_riders_no_paint(label):
    estimator = VehicleAppearanceEstimator()
    pose = _lidar_T_object(12.)
    box = _box("x", pose, label=label)
    for frame in range(5):
        estimator.update([_match(box)], [box], timestamp_us=frame, world_T_lidar=WORLD_T_LIDAR,
                         lidar_points=_scan([pose], seed=frame), camera=AppearanceCamera(_image([pose]), K, CAMERA_T_LIDAR))
    assert estimator.prompt_fields("x") == {} and estimator.accumulator.evidence("x") == 0

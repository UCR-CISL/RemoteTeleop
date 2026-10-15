"""Bounded per-track vehicle shape/color hints from synthetic scans of moving cars."""
from __future__ import annotations

import json
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
import pytest

from src.deployment.lidar_ground_snap import _BevGrid
from src.deployment.vehicle_appearance import AppearanceCamera, VehicleAppearanceConfig, VehicleAppearanceEstimator
from src.deployment.vehicle_shape import VehicleShapeFitter
from src.realtime.bevfusion_tracker import DetectionMatch, TrackedWorldBox
from src.realtime.protocol import BoxPrompt
from src.viz.universal_vehicle import BOUNDS, PARAMETERS, UniversalVehicleShape

DIMENSIONS = (4.5, 1.8, 1.5)
K = np.array([[500., 0, 320], [0, 500., 240], [0, 0, 1]])
SIZE = (640, 480)
# CV camera at the LiDAR origin looking along LiDAR +X (x right, y down, z forward).
CAMERA_T_LIDAR = np.array([[0., -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
# World differs from the LiDAR frame so frame conversions are exercised.
WORLD_T_LIDAR = np.array([[0., -1, 0, 100], [1, 0, 0, -20], [0, 0, 1, 2], [0, 0, 0, 1]])
PAINT_BGR = (40, 50, 190)
SYNC = VehicleAppearanceConfig(fit_in_background=False)  # deterministic: fits run inside update


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
    estimator = VehicleAppearanceEstimator(SYNC)
    _drive(estimator, 12)

    fields = estimator.prompt_fields("bevfusion:car:0")
    shape = UniversalVehicleShape.from_vector(fields["vehicle_shape"])
    assert shape.body_type in UniversalVehicleShape.TYPES
    assert estimator._shapes["bevfusion:car:0"].fitted.body_type == shape.body_type
    assert 1 <= estimator.fit_count <= 12 * estimator.config.max_fits_per_update
    rgb = np.float32([[fields["body_color_rgb"], np.array(PAINT_BGR[::-1]) / 255]])
    lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2Lab)[0]
    assert np.linalg.norm(lab[0] - lab[1]) < 6
    assert .5 <= fields["color_confidence"] <= 1
    prompt = BoxPrompt("bevfusion:car:0", (1, 2, 30, 40), DIMENSIONS, tuple(np.eye(4).reshape(-1)), **fields)
    assert BoxPrompt.from_dict(json.loads(json.dumps(prompt.to_dict()))) == prompt


def test_color_waits_for_several_observations_and_needs_an_image():
    estimator = VehicleAppearanceEstimator(VehicleAppearanceConfig(shape=False, fit_in_background=False))
    _drive(estimator, 2)
    assert estimator.prompt_fields("bevfusion:car:0") == {}  # fewer than min_observations
    _drive(estimator, 2, start=2)
    assert set(estimator.prompt_fields("bevfusion:car:0")) == {"body_color_rgb", "color_confidence"}
    no_camera = VehicleAppearanceEstimator(VehicleAppearanceConfig(shape=False, fit_in_background=False))
    _drive(no_camera, 6, camera=False)
    assert no_camera.prompt_fields("bevfusion:car:0") == {}


def test_crop_excludes_neighbor_returns_and_ground():
    estimator = VehicleAppearanceEstimator(VehicleAppearanceConfig(color=False, fit_in_background=False))
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
    estimator = VehicleAppearanceEstimator(SYNC)
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
    estimator = VehicleAppearanceEstimator(SYNC)
    _drive(estimator, 6, filter_id="bevfusion:car:0")
    assert "body_color_rgb" in estimator.prompt_fields("bevfusion:car:0")
    evidence = estimator.accumulator.evidence("bevfusion:car:0")
    _drive(estimator, 1, filter_id="bevfusion:car:7", start=6)
    # Paint needs fresh observations; shape evidence restarts from this one scan.
    assert "body_color_rgb" not in estimator.prompt_fields("bevfusion:car:0")
    assert 0 < estimator.accumulator.evidence("bevfusion:car:0") < evidence


def test_refits_are_bounded_per_update_round_robin():
    estimator = VehicleAppearanceEstimator(VehicleAppearanceConfig(color=False, max_fits_per_update=1,
                                                                   fit_in_background=False))
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
        color=False, refit_every_scans=1, min_param_change=.5, freeze_after_stable_fits=2, fit_in_background=False,
        unfreeze_new_cells=64, sweep_every_refits=1))
    _drive(estimator, 4, camera=False)
    assert estimator._shapes["bevfusion:car:0"].types.confident  # freezing also needs a settled body type
    state = estimator._shapes["bevfusion:car:0"]
    assert state.frozen_dimensions is not None
    published, fits = state.published, estimator.fit_count
    _drive(estimator, 3, camera=False, start=4)
    assert estimator.fit_count == fits and state.published == published
    _drive(estimator, 1, camera=False, start=7, dimensions=(5.2, 1.8, 1.5))  # > 10% longer
    assert state.frozen_dimensions is None and estimator.fit_count == fits + 1


def test_frozen_shape_reopens_when_a_scan_covers_new_surface():
    estimator = VehicleAppearanceEstimator(VehicleAppearanceConfig(
        color=False, refit_every_scans=1, min_param_change=.5, freeze_after_stable_fits=2, fit_in_background=False,
        sweep_every_refits=1))
    state = None
    for frame in range(6):  # a parked car seen from one side only
        pose = _lidar_T_object(12.)
        box = _box("car", pose)
        estimator.update([_match(box)], [box], timestamp_us=frame, world_T_lidar=WORLD_T_LIDAR,
                         lidar_points=_scan([pose], seed=frame))
        state = estimator._shapes["car"]
    assert state.frozen_dimensions is not None and state.coverage is not None
    fits = estimator.fit_count
    turned = _lidar_T_object(12., yaw=np.pi)  # same box, now its other side and other end face the sensor
    box = _box("car", turned)
    estimator.update([_match(box)], [box], timestamp_us=10, world_T_lidar=WORLD_T_LIDAR,
                     lidar_points=_scan([turned], seed=10))
    assert state.frozen_dimensions is None and estimator.fit_count == fits + 1


def test_small_shape_changes_keep_the_published_params():
    estimator = VehicleAppearanceEstimator(VehicleAppearanceConfig(color=False, fit_in_background=False))
    published = UniversalVehicleShape.mean("sedan")
    index = PARAMETERS.index("top_roof_front")
    span = BOUNDS[index, 1] - BOUNDS[index, 0]

    def moved(delta):
        values = list(published.values)
        values[index] -= delta * span
        return UniversalVehicleShape("sedan", tuple(values))

    assert not estimator._changed(published, moved(.01))
    assert estimator._changed(published, moved(.03))
    assert estimator._changed(published, UniversalVehicleShape("hatchback", published.values))


def test_prior_shape_is_published_until_the_first_accepted_fit():
    estimator = VehicleAppearanceEstimator(VehicleAppearanceConfig(color=False, fit_in_background=False))
    pose = _lidar_T_object(40.)  # far: a handful of returns, no fit yet
    box = _box("far", pose, dimensions=(6.2, 2.45, 2.15))
    estimator.update([_match(box)], [box], timestamp_us=0, world_T_lidar=WORLD_T_LIDAR,
                     lidar_points=_scan([pose], count=8))
    shape = UniversalVehicleShape.from_vector(estimator.prompt_fields("far")["vehicle_shape"])
    assert estimator.fit_count == 0 and shape == estimator.fitter.prior_shape((6.2, 2.45, 2.15))
    assert shape.body_type == "pickup"
    quiet = VehicleAppearanceEstimator(VehicleAppearanceConfig(color=False, fit_in_background=False,
                                                               publish_prior_shape=False))
    quiet.update([_match(box)], [box], timestamp_us=0, world_T_lidar=WORLD_T_LIDAR,
                 lidar_points=_scan([pose], count=8))
    assert quiet.prompt_fields("far") == {}


@pytest.mark.parametrize("label", ["pedestrian", "bicycle"])
def test_non_car_labels_get_no_shape_and_riders_no_paint(label):
    estimator = VehicleAppearanceEstimator(SYNC)
    pose = _lidar_T_object(12.)
    box = _box("x", pose, label=label)
    for frame in range(5):
        estimator.update([_match(box)], [box], timestamp_us=frame, world_T_lidar=WORLD_T_LIDAR,
                         lidar_points=_scan([pose], seed=frame), camera=AppearanceCamera(_image([pose]), K, CAMERA_T_LIDAR))
    assert estimator.prompt_fields("x") == {} and estimator.accumulator.evidence("x") == 0


class _GatedFitter(VehicleShapeFitter):
    """Records each fit's thread and incumbent cost; fits wait until ``gate`` opens."""

    def __init__(self, *, open_gate=False):
        super().__init__()
        self.gate, self.calls = threading.Event(), []
        if open_gate:
            self.gate.set()

    def fit(self, *args, incumbent_cost=None, **kwargs):
        self.calls.append((threading.current_thread().name, incumbent_cost))
        assert self.gate.wait(10)
        return super().fit(*args, incumbent_cost=incumbent_cost, **kwargs)


def _finished_count(estimator, count):
    deadline = time.monotonic() + 10
    while len(estimator._worker._finished) < count and time.monotonic() < deadline:
        time.sleep(.005)
    return len(estimator._worker._finished)


def test_background_fits_run_off_the_update_thread_and_apply_on_a_later_update():
    fitter = _GatedFitter()
    estimator = VehicleAppearanceEstimator(VehicleAppearanceConfig(color=False), fitter=fitter)
    _drive(estimator, 3, camera=False)
    state = estimator._shapes["bevfusion:car:0"]
    # One fit in flight: the track is not resubmitted and nothing is applied yet.
    assert len(fitter.calls) == 1 and state.fitting and estimator.fit_count == 0 and state.fitted is None
    fitter.gate.set()
    assert _finished_count(estimator, 1) == 1
    _drive(estimator, 1, camera=False, start=3)
    assert estimator.fit_count == 1 and state.fitted is not None and state.published is state.fitted
    assert fitter.calls[0] == ("vehicle-shape-fit", None)
    thread = estimator._worker._thread
    estimator.close()
    assert not thread.is_alive()
    _drive(estimator, 3, camera=False, start=4)  # closed: no further fits start
    assert estimator.fit_count == 1


def test_background_fit_of_a_dropped_or_reset_track_is_discarded():
    fitter = _GatedFitter()
    estimator = VehicleAppearanceEstimator(VehicleAppearanceConfig(color=False), fitter=fitter)
    _drive(estimator, 1, camera=False, filter_id="f0")
    _drive(estimator, 1, camera=False, filter_id="f1", start=1)  # reacquired by a new filter: fresh state
    estimator.update([], [], timestamp_us=200_000, world_T_lidar=WORLD_T_LIDAR, lidar_points=_scan([]))  # died
    fitter.gate.set()
    assert _finished_count(estimator, 2) == 2
    estimator.update([], [], timestamp_us=300_000, world_T_lidar=WORLD_T_LIDAR, lidar_points=_scan([]))
    assert estimator.fit_count == 0 and estimator._shapes == {}
    estimator.close()


def test_background_fit_error_is_counted_and_the_track_refits_later():
    class _FailOnce(VehicleShapeFitter):
        failed = False

        def fit(self, *args, **kwargs):
            if not self.failed:
                self.failed = True
                raise RuntimeError("fit failed")
            return super().fit(*args, **kwargs)

    estimator = VehicleAppearanceEstimator(VehicleAppearanceConfig(color=False, refit_every_scans=1),
                                           fitter=_FailOnce())
    _drive(estimator, 1, camera=False)
    assert _finished_count(estimator, 1) == 1
    _drive(estimator, 1, camera=False, start=1)  # error consumed without raising; fitting flag cleared
    assert estimator.fit_error_count == 1
    assert _finished_count(estimator, 1) == 1
    _drive(estimator, 1, camera=False, start=2)
    assert estimator.fit_count == 1
    estimator.close()


def test_refits_sweep_all_candidates_only_periodically():
    fitter = _GatedFitter(open_gate=True)
    estimator = VehicleAppearanceEstimator(VehicleAppearanceConfig(
        color=False, refit_every_scans=1, freeze_after_stable_fits=99, sweep_every_refits=3,
        fit_in_background=False), fitter=fitter)
    _drive(estimator, 7, camera=False)
    assert all(thread == threading.current_thread().name for thread, _ in fitter.calls)
    assert [cost is None for _, cost in fitter.calls] == [True, False, False, True, False, False, True]

from __future__ import annotations

import cv2
import numpy as np
import pytest

from src.deployment.vehicle_color import (SparseDepthBuffer, VehicleColorConfig, VehicleColorEstimator,
                                          project_points_to_image)

K = np.array([[500.0, 0, 320], [0, 500.0, 240], [0, 0, 1]])
SIZE = (640, 480)
DIMS = (4.5, 1.9, 1.5)
# Object +x (forward) -> camera right, +z (up) -> camera up; the object's right side (y = -W/2) faces the camera.
CAMERA_R_OBJECT = np.array([[1.0, 0, 0], [0, 0, -1], [0, 1, 0]])


def _pose(depth=10.0, lateral=0.0):
    pose = np.eye(4)
    pose[:3, :3] = CAMERA_R_OBJECT
    pose[:3, 3] = (lateral, 0.85, depth)
    return pose


def _side_points(step=0.08):
    """Metric object-frame LiDAR returns on the camera-facing side of the box."""
    length, width, height = DIMS
    x, z = np.meshgrid(np.arange(-length / 2, length / 2, step), np.arange(-height / 2, height / 2, step))
    return np.c_[x.ravel(), np.full(x.size, -width / 2), z.ravel()]


def _to_camera(points, pose):
    return points @ pose[:3, :3].T + pose[:3, 3]


def _fill_box(image, pose, bgr, dims=DIMS):
    signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], dtype=float)
    uv, _, _ = project_points_to_image(_to_camera(signs * np.asarray(dims) / 2, pose), K, SIZE)
    cv2.fillConvexPoly(image, cv2.convexHull(np.rint(uv).astype(np.int32)), bgr)


def _scene(bgr, pose=None, seed=0):
    pose = _pose() if pose is None else pose
    rng = np.random.default_rng(seed)
    image = np.full((SIZE[1], SIZE[0], 3), (60, 110, 70), np.uint8)
    _fill_box(image, pose, bgr)
    noisy = image.astype(float) + rng.normal(0, 3, image.shape)
    return np.clip(noisy, 0, 255).astype(np.uint8), pose


def _delta_e(rgb_a, rgb_b):
    lab = cv2.cvtColor(np.float32([[rgb_a, rgb_b]]), cv2.COLOR_RGB2Lab)[0]
    return float(np.linalg.norm(lab[0] - lab[1]))


def _rgb(bgr):
    return tuple(value / 255 for value in bgr[::-1])


def test_projection_matches_hand_computed_pinhole():
    points = np.array([[1.0, 2.0, 10.0], [0.0, 0.0, -5.0], [10.0, 0.0, 1.0], [0.0, 0.0, 2.0]])
    intrinsic = np.array([[100.0, 0, 50], [0, 100.0, 40], [0, 0, 1]])

    uv, depth, valid = project_points_to_image(points, intrinsic, (101, 81))

    np.testing.assert_allclose(uv[0], (60.0, 60.0))
    np.testing.assert_allclose(uv[3], (50.0, 40.0))
    np.testing.assert_allclose(depth, points[:, 2])
    assert valid.tolist() == [True, False, False, True]  # behind camera, outside image


@pytest.mark.parametrize("bgr", [(30, 30, 200), (190, 70, 20), (22, 20, 20), (235, 236, 238), (175, 172, 168)],
                         ids=["red", "blue", "black", "white", "silver"])
def test_recovers_uniform_paint_in_rgb_order(bgr):
    image, pose = _scene(bgr)

    observation = VehicleColorEstimator().observe("car", image, K, pose, _side_points(), DIMS)

    assert observation is not None
    assert _delta_e(observation.rgb, _rgb(bgr)) < 3
    assert observation.dominant_fraction > 0.95 and observation.confidence > 0.9


def test_highlight_and_shadow_outliers_do_not_move_paint_color():
    bgr = (40, 40, 180)
    image, pose = _scene(bgr)
    uv, _, _ = project_points_to_image(_to_camera(_side_points(), pose), K, SIZE)
    left, right = uv[:, 0].min(), uv[:, 0].max()
    first, second = int(left + 0.2 * (right - left)), int(left + 0.45 * (right - left))
    image[:, first:int(left + 0.3 * (right - left))] = 255  # clipped specular stripe
    image[:, second:int(left + 0.65 * (right - left))] = (image[:, second:int(left + 0.65 * (right - left))]
                                                          * 0.35).astype(np.uint8)  # cast shadow

    observation = VehicleColorEstimator().observe("car", image, K, pose, _side_points(), DIMS)

    assert observation is not None
    assert _delta_e(observation.rgb, _rgb(bgr)) < 4
    assert 0.6 < observation.dominant_fraction < 1.0


def test_depth_buffer_rejects_samples_hidden_behind_nearer_object():
    bgr = (30, 30, 200)
    image, pose = _scene(bgr)
    occluder_pose = _pose(depth=6.0, lateral=-1.4)
    occluder_dims = (1.0, 0.5, 2.0)
    _fill_box(image, occluder_pose, (40, 200, 40), occluder_dims)
    y, z = np.meshgrid(np.arange(-0.5, 0.5, 0.03), np.arange(-1.0, 1.0, 0.03))
    occluder_points = _to_camera(np.c_[y.ravel(), np.full(y.size, -0.25), z.ravel()], occluder_pose)
    car_points = _side_points()
    scan = np.r_[_to_camera(car_points, pose), occluder_points]
    estimator = VehicleColorEstimator()

    naive = estimator.observe("a", image, K, pose, car_points, DIMS)
    tested = estimator.observe("b", image, K, pose, car_points, DIMS, scan_points_camera=scan)
    shared = estimator.observe("c", image, K, pose, car_points, DIMS,
                               depth_buffer=SparseDepthBuffer(scan, K, SIZE))

    assert naive is not None and naive.dominant_fraction < 0.9
    assert tested is not None and tested.dominant_fraction > 0.95
    assert _delta_e(tested.rgb, _rgb(bgr)) < 3
    assert shared == tested


def test_nearer_track_boxes_remove_samples_or_skip_heavily_covered_track():
    bgr = (190, 70, 20)
    image, pose = _scene(bgr)
    uv, _, _ = project_points_to_image(_to_camera(_side_points(), pose), K, SIZE)
    left, right, bottom = uv[:, 0].min(), uv[:, 0].max(), uv[:, 1].max()
    image[:, : int(left + 0.2 * (right - left))] = (20, 200, 240)
    estimator = VehicleColorEstimator()

    partial = estimator.observe("car", image, K, pose, _side_points(), DIMS,
                                occluder_xyxy=[(0, 0, left + 0.22 * (right - left), SIZE[1])])
    covered = estimator.observe("car", image, K, pose, _side_points(), DIMS,
                                occluder_xyxy=[(0, 0, left + 0.6 * (right - left), bottom + 50)])

    assert partial is not None and partial.dominant_fraction > 0.95
    assert _delta_e(partial.rgb, _rgb(bgr)) < 3
    assert covered is None


def test_too_few_samples_or_tiny_box_yield_no_observation():
    image, pose = _scene((30, 30, 200))
    estimator = VehicleColorEstimator()
    belt = _side_points()[np.abs(_side_points()[:, 2]) < 0.1]

    assert estimator.observe("car", image, K, pose, belt[:5], DIMS) is None
    glass_only = _side_points()[_side_points()[:, 2] > 0.4]
    assert estimator.observe("car", image, K, pose, glass_only, DIMS) is None
    far_image, far_pose = _scene((30, 30, 200), _pose(depth=150.0))
    assert estimator.observe("car", far_image, K, far_pose, _side_points(), DIMS) is None
    assert estimator.color("car") is None


def test_temporal_fusion_publishes_after_min_observations_converges_and_is_stable():
    bgr = (30, 30, 200)
    estimator = VehicleColorEstimator(VehicleColorConfig(min_observations=3))
    rng = np.random.default_rng(1)
    estimates, observations = [], []
    for frame in range(25):
        image, pose = _scene(bgr, seed=frame)
        image = np.clip(image * rng.uniform(0.9, 1.1), 0, 255).astype(np.uint8)  # exposure flicker
        observations.append(estimator.observe("car", image, K, pose, _side_points(), DIMS).rgb)
        estimate = estimator.color("car")
        assert (estimate is None) == (frame < 2)
        if estimate is not None:
            estimates.append(estimate)

    assert estimates[-1].observation_count == 25 and estimates[-1].confidence > 0.9
    assert _delta_e(estimates[-1].rgb, _rgb(bgr)) < 4
    fused_spread = max(_delta_e(estimate.rgb, estimates[-1].rgb) for estimate in estimates[-10:])
    raw_spread = max(_delta_e(rgb, estimates[-1].rgb) for rgb in observations[-10:])
    assert fused_spread < 4 and fused_spread < 0.5 * raw_spread

    estimator.drop("car")
    assert estimator.color("car") is None


def test_fusion_moves_luminance_slower_than_chroma_and_skips_low_confidence():
    estimator = VehicleColorEstimator(VehicleColorConfig(min_observations=1))
    red, pose = _scene((30, 30, 200))
    for _ in range(10):
        estimator.observe("car", red, K, pose, _side_points(), DIMS)
    settled = cv2.cvtColor(np.float32([[estimator.color("car").rgb]]), cv2.COLOR_RGB2Lab)[0, 0]
    sparse = _side_points()[np.abs(_side_points()[:, 2]) < 0.1][::6]  # few samples -> low confidence
    blue, _ = _scene((190, 70, 20))
    low = estimator.observe("car", blue, K, pose, sparse, DIMS)
    assert low is not None and low.confidence < 0.5
    assert estimator.color("car").observation_count == 10

    observed = estimator.observe("car", blue, K, pose, _side_points(), DIMS)
    fused = cv2.cvtColor(np.float32([[estimator.color("car").rgb]]), cv2.COLOR_RGB2Lab)[0, 0]
    target = np.asarray(observed.lab)
    luminance_step = abs(fused[0] - settled[0]) / abs(target[0] - settled[0])
    chroma_step = np.linalg.norm(fused[1:] - settled[1:]) / np.linalg.norm(target[1:] - settled[1:])
    assert luminance_step < chroma_step < 0.5


def test_config_validation_rejects_bad_band():
    with pytest.raises(ValueError):
        VehicleColorConfig(belt_band=(0.7, 0.3))


def test_sunlit_white_paint_is_not_trimmed_as_highlight():
    image, pose = _scene((255, 255, 255))
    uv, _, _ = project_points_to_image(_to_camera(_side_points(), pose), K, SIZE)
    left, right = uv[:, 0].min(), uv[:, 0].max()
    image[:, int(left + 0.4 * (right - left)):int(left + 0.65 * (right - left))] = (25, 25, 30)  # dark trim

    observation = VehicleColorEstimator().observe("car", image, K, pose, _side_points(), DIMS)

    assert observation is not None and min(observation.rgb) > 0.95


def test_colored_minority_cluster_wins_over_achromatic_trim():
    bgr = (35, 35, 170)
    image, pose = _scene(bgr)
    uv, _, _ = project_points_to_image(_to_camera(_side_points(), pose), K, SIZE)
    left, right = uv[:, 0].min(), uv[:, 0].max()
    image[:, int(left):int(left + 0.4 * (right - left))] = (18, 18, 18)  # black trim / spare tire
    image[:, int(left + 0.4 * (right - left)):int(left + 0.7 * (right - left))] = (45, 45, 48)  # shadow

    observation = VehicleColorEstimator().observe("car", image, K, pose, _side_points(), DIMS)

    assert observation is not None and 0.2 < observation.dominant_fraction < 0.4
    assert _delta_e(observation.rgb, _rgb(bgr)) < 4
    assert observation.confidence >= VehicleColorConfig().min_confidence


def test_reported_luminance_favors_lit_part_of_smoothly_shaded_paint():
    image, pose = _scene((230, 230, 230))
    uv, _, _ = project_points_to_image(_to_camera(_side_points(), pose), K, SIZE)
    left, right = int(uv[:, 0].min()) - 2, int(uv[:, 0].max()) + 3
    image[:, left:right] = (image[:, left:right] * np.linspace(0.7, 1.0, right - left)[None, :, None]).astype(np.uint8)

    observation = VehicleColorEstimator().observe("car", image, K, pose, _side_points(), DIMS)
    median_gray = cv2.cvtColor(np.float32([[[0.85 * 230 / 255] * 3]]), cv2.COLOR_RGB2Lab)[0, 0, 0]

    assert observation is not None and observation.lab[0] > median_gray + 2

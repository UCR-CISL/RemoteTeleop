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


def test_fusion_rejects_outliers_moves_luminance_slower_than_chroma_and_skips_low_confidence():
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

    switched = {}
    for count in range(1, 12):
        target = np.asarray(estimator.observe("car", blue, K, pose, _side_points(), DIMS).lab)
        fused = cv2.cvtColor(np.float32([[estimator.color("car").rgb]]), cv2.COLOR_RGB2Lab)[0, 0]
        if count == 1:  # one incompatible reading is rejected, not averaged in
            assert _delta_e(estimator.color("car").rgb, cv2.cvtColor(np.float32([[settled]]), cv2.COLOR_Lab2RGB)[0, 0]) < 1
        if np.linalg.norm(fused[1:] - target[1:]) < np.linalg.norm(fused[1:] - settled[1:]):
            switched.setdefault("chroma", count)
        if abs(fused[0] - target[0]) < abs(fused[0] - settled[0]):
            switched.setdefault("luminance", count)
    assert 1 < switched["chroma"] < switched["luminance"]


def test_neutral_paint_snaps_to_gray_but_saturated_dark_paint_keeps_its_hue():
    estimator = VehicleColorEstimator()
    for bgr in ((168, 172, 178), (40, 8, 4)):  # silver with a slight warm cast; deep navy in shade
        image, pose = _scene(bgr)
        observation = estimator.observe(str(bgr), image, K, pose, _side_points(), DIMS)
        chroma = np.hypot(*observation.lab[1:])
        assert (chroma == 0) == (bgr[0] > 100)


def test_road_ring_cast_explains_warm_tint_on_silver_but_not_red_paint():
    warm_road = (70, 105, 140)  # asphalt under low sun, BGR
    for paint, neutral in (((150, 160, 175), True), ((40, 40, 150), False)):
        pose = _pose()
        image = np.full((SIZE[1], SIZE[0], 3), warm_road, np.uint8)
        _fill_box(image, pose, paint)
        scan = np.r_[_to_camera(_side_points(), pose)]
        observation = VehicleColorEstimator().observe("car", image, K, pose, _side_points(), DIMS,
                                                      scan_points_camera=scan)
        assert observation is not None
        assert (np.hypot(*observation.lab[1:]) == 0) == neutral
        plain = VehicleColorEstimator().observe("car", image, K, pose, _side_points(), DIMS)
        assert np.hypot(*plain.lab[1:]) > 8  # without a road reference the tint is kept


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


def test_colored_minority_cluster_does_not_win_over_bright_white_paint():
    image, pose = _scene((240, 240, 240))
    uv, _, _ = project_points_to_image(_to_camera(_side_points(), pose), K, SIZE)
    left, right = uv[:, 0].min(), uv[:, 0].max()
    image[:, int(left):int(left + 0.3 * (right - left))] = (40, 40, 200)  # tail lights / reflectors

    observation = VehicleColorEstimator().observe("car", image, K, pose, _side_points(), DIMS)

    assert observation is not None and observation.dominant_fraction > 0.6
    assert observation.lab[0] > 90 and np.hypot(*observation.lab[1:]) < 3


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


def _belt_rows(pose):
    """Image row range of the belt band on the camera-facing side."""
    points = _side_points()
    low, high = (-DIMS[2] / 2 + f * DIMS[2] for f in VehicleColorConfig().belt_band)
    uv, _, _ = project_points_to_image(_to_camera(points[(points[:, 2] >= low) & (points[:, 2] <= high)], pose), K, SIZE)
    return int(uv[:, 1].min()), int(uv[:, 1].max()) + 1, uv[:, 0].min(), uv[:, 0].max()


def _two_tone(lower_bgr, upper_bgr, upper_fraction=0.45, columns=(0.0, 1.0), seed=0):
    """Paint ``lower_bgr`` with the upper belt rows (between ``columns`` fractions of the side) ``upper_bgr``."""
    image, pose = _scene(lower_bgr, seed=seed)
    top, bottom, left, right = _belt_rows(pose)
    split = int(top + upper_fraction * (bottom - top))
    first, last = (int(left + f * (right - left)) for f in columns)
    image[:split, first:last + 1] = np.clip(np.asarray(upper_bgr) + np.random.default_rng(seed).normal(
        0, 3, image[:split, first:last + 1].shape), 0, 255).astype(np.uint8)
    return image, pose


def _fused_lab(lower_bgr, upper_bgr, frames=4, **kwargs):
    estimator = VehicleColorEstimator()
    for frame in range(frames):
        image, pose = _two_tone(lower_bgr, upper_bgr, seed=frame, **kwargs)
        estimator.observe("car", image, K, pose, _side_points(), DIMS)
    estimate = estimator.color("car")
    return None if estimate is None else cv2.cvtColor(np.float32([[estimate.rgb]]), cv2.COLOR_RGB2Lab)[0, 0]


def test_shaded_silver_publishes_its_persistent_bright_neutral_part():
    shaded, lit = (70, 70, 72), (175, 176, 178)
    image, pose = _two_tone(shaded, lit)
    observation = VehicleColorEstimator().observe("car", image, K, pose, _side_points(), DIMS)
    assert observation.lab[0] < 40 and observation.bright_lab is not None and observation.bright_lab[0] > 65

    two_frames = _fused_lab(shaded, lit, frames=2)
    assert two_frames is None  # min_observations
    estimator = VehicleColorEstimator(VehicleColorConfig(min_observations=1))
    for frame in range(3):
        image, pose = _two_tone(shaded, lit, seed=frame)
        estimator.observe("car", image, K, pose, _side_points(), DIMS)
        lightness = cv2.cvtColor(np.float32([[estimator.color("car").rgb]]), cv2.COLOR_RGB2Lab)[0, 0, 0]
        assert (lightness > 65) == (frame == 2)  # published only once it persists over 3 observations
    assert np.hypot(*_fused_lab(shaded, lit)[1:]) < 1


def test_localized_bright_patch_or_colored_paint_keeps_the_dark_color():
    shaded, lit = (70, 70, 72), (175, 176, 178)
    localized = _fused_lab(shaded, lit, upper_fraction=1.0, columns=(0.0, 0.3))  # one lit panel / reflection
    assert localized[0] < 40
    dark_red = _fused_lab((30, 30, 120), lit)  # colored paint is never replaced by a neutral group
    assert dark_red[0] < 40 and np.hypot(*dark_red[1:]) > 20
    image, pose = _two_tone((22, 20, 20), (60, 60, 62))  # a dim group is not white/silver paint
    assert VehicleColorEstimator().observe("car", image, K, pose, _side_points(), DIMS).bright_lab is None


def _sedan():
    from src.viz.universal_vehicle import UniversalVehicleShape

    return UniversalVehicleShape.mean("sedan", DIMS)


def test_body_panel_crop_samples_visible_paint_panels_by_group():
    from src.deployment.vehicle_color import BodyPanelSampler

    bgr = (40, 40, 180)
    image, pose = _scene(bgr)
    panels = BodyPanelSampler().sample(image, K, pose, _sedan(), DIMS)

    assert panels is not None and len(panels.uv) > 100
    groups = set(np.unique(panels.group).tolist())
    assert {BodyPanelSampler.TOP, BodyPanelSampler.SIDE} <= groups  # the camera is above the hood line
    lab = cv2.cvtColor(np.float32(panels.rgb).reshape(-1, 1, 3), cv2.COLOR_RGB2Lab).reshape(-1, 3)
    paint = cv2.cvtColor(np.float32([[_rgb(bgr)]]), cv2.COLOR_RGB2Lab)[0, 0]
    assert np.percentile(np.linalg.norm(lab - paint, axis=1), 95) < 5  # inside the silhouette only
    camera = np.c_[panels.uv, np.ones(len(panels.uv))] @ np.linalg.inv(K).T * panels.depth[:, None]
    local = (camera - pose[:3, 3]) @ pose[:3, :3]
    assert np.all(np.abs(local) <= np.asarray(DIMS) / 2 + 0.05)  # depths lie on the body surface
    assert local[:, 2].min() >= -DIMS[2] / 2 + 0.3 * DIMS[2] - 0.05  # wheels and rockers excluded

    observation = VehicleColorEstimator().observe("car", image, K, pose, np.empty((0, 3)), DIMS, shape=_sedan())
    assert observation is not None and _delta_e(observation.rgb, _rgb(bgr)) < 3


def test_body_panel_crop_sees_lit_trunk_lid_of_white_car_with_shaded_rear_face():
    # Camera behind the car (object +x away from the camera), 0.85 m above the box center.
    pose = np.eye(4)
    pose[:3, :3] = [[0.0, -1, 0], [0, 0, -1], [1, 0, 0]]
    pose[:3, 3] = (0.0, 0.85, 8.0)
    image = np.full((SIZE[1], SIZE[0], 3), (60, 110, 70), np.uint8)
    _fill_box(image, pose, (55, 55, 55))  # backlit white car: shaded rear face
    length, width, height = DIMS
    deck = np.array([[x, y, z] for x in (-length / 2, -0.2 * length) for y in (-width / 2, width / 2)
                     for z in (-height / 2 + 0.6 * height, -height / 2 + 0.8 * height)])
    uv, _, _ = project_points_to_image(_to_camera(deck, pose), K, SIZE)
    cv2.fillConvexPoly(image, cv2.convexHull(np.rint(uv).astype(np.int32)), (232, 234, 236))  # lit trunk lid
    rear = np.array([[-length / 2, y, z] for y in np.linspace(-width / 2, width / 2, 30)
                     for z in np.linspace(-height / 2, height / 2, 20)])

    belt, crop = VehicleColorEstimator(), VehicleColorEstimator()
    for _ in range(3):
        belt.observe("car", image, K, pose, rear, DIMS)
        crop.observe("car", image, K, pose, rear, DIMS, shape=_sedan())

    assert np.mean(belt.color("car").rgb) < 0.3  # belt LiDAR only sees the shaded rear face
    assert np.mean(crop.color("car").rgb) > 0.8


def test_fused_estimate_carries_the_camera_road_reference():
    road_bgr, paint = (70, 105, 140), (40, 40, 150)
    pose = _pose()
    image = np.full((SIZE[1], SIZE[0], 3), road_bgr, np.uint8)
    _fill_box(image, pose, paint)
    scan = _to_camera(_side_points(), pose)
    estimator, no_road = VehicleColorEstimator(), VehicleColorEstimator()
    for _ in range(3):
        estimator.observe("car", image, K, pose, _side_points(), DIMS, scan_points_camera=scan, shape=_sedan())
        no_road.observe("car", image, K, pose, _side_points(), DIMS, shape=_sedan())

    np.testing.assert_allclose(estimator.color("car").road_rgb, _rgb(road_bgr), atol=1e-6)
    assert no_road.color("car").road_rgb is None  # no depth buffer, no visible-road reference

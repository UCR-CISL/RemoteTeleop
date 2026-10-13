from __future__ import annotations

import time

import numpy as np
import pytest

from src.deployment.lidar_ground_snap import LidarGroundSnapper


def _ground(height_at, extent=30., step=.25):
    xy = np.stack(np.meshgrid(np.arange(-extent, extent, step), np.arange(-extent, extent, step)), -1).reshape(-1, 2)
    return np.c_[xy, height_at(xy), np.ones(len(xy))].astype(np.float32)


def test_flat_ground_places_box_bottom_on_ground_and_ignores_points_inside_footprint():
    points = _ground(lambda xy: np.full(len(xy), -1.8))
    roof = np.c_[np.random.default_rng(0).uniform(-1.5, 1.5, (500, 2)) + [10, 5], np.full(500, -.3), np.ones(500)]
    snapper = LidarGroundSnapper()
    shifts = snapper.offsets(np.r_[points, roof.astype(np.float32)], [[10, 5, -1.0], [-12, 3, -1.5]], [.4, -1.2],
                             [[4.5, 1.9, 1.6], [4, 2, 1.4]])
    # bottoms: -1.8 and -2.2 -> ground -1.8
    np.testing.assert_allclose(shifts, [-.0, .4], atol=1e-6)
    assert snapper.snapped_count == 2 and snapper.skipped_count == 0


def test_sloped_ground_uses_local_height_around_footprint():
    slope = np.tan(np.radians(4))
    points = _ground(lambda xy: -1.8 + slope * xy[:, 0])
    (shift,) = LidarGroundSnapper().offsets(points, [[20, 0, -1.8 + slope * 20 + .8 - .1]], [0.], [[4, 2, 1.6]])
    # Low-percentile ring ground sits slightly downhill of the centre height (bounded by the ring extent).
    assert -.25 < shift - .1 < 0.


@pytest.mark.parametrize("points, center", [
    (np.empty((0, 4), np.float32), [10, 0, -1]),                                  # no scan
    (_ground(lambda xy: np.full(len(xy), -1.8), extent=5.), [20, 0, -1]),         # no ring points
    (_ground(lambda xy: np.full(len(xy), -1.8)), [10, 0, 1.0]),                   # implausible |dz| = 2 m
])
def test_insufficient_support_or_implausible_correction_leaves_box_unchanged(points, center):
    snapper = LidarGroundSnapper()
    assert snapper.offsets(points, [center], [0.], [[4, 2, 1.6]]).tolist() == [0.]
    assert snapper.skipped_count == 1


def test_frame_cost_is_bounded_for_twenty_boxes_and_100k_points():
    rng = np.random.default_rng(1)
    points = np.c_[rng.uniform(-60, 60, (100_000, 2)), rng.normal(-1.8, .05, 100_000), np.ones(100_000)].astype(np.float32)
    centers = np.c_[rng.uniform(-40, 40, (20, 2)), np.full(20, -1.0)]
    snapper = LidarGroundSnapper()
    snapper.offsets(points, centers, rng.uniform(-3, 3, 20), np.tile([4.5, 1.9, 1.6], (20, 1)))
    start = time.perf_counter()
    for _ in range(5):
        snapper.offsets(points, centers, rng.uniform(-3, 3, 20), np.tile([4.5, 1.9, 1.6], (20, 1)))
    assert (time.perf_counter() - start) / 5 < .02  # loose CI bound; ~2-3 ms measured locally

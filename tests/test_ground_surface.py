import numpy as np
from scipy.spatial.transform import Rotation

from src.localization.ground_surface import GroundSurface, load_ground_placement


def test_local_road_placement_is_independent_of_incoming_height_and_map_axes():
    basis = Rotation.from_euler('xyz', [90, 5, 12], degrees=True).as_matrix()
    samples = np.array([[x, y, -2 + .04*x] for x in range(-4, 5, 2) for y in range(-4, 5, 2)])
    surface = GroundSurface([2, 3, 4], basis, samples)
    for height in [-10, 10]:
        pose = np.eye(4)
        pose[:3, :3] = basis
        pose[:3, 3] = surface.origin + basis @ [0, 0, height]
        placed = surface.place_box(pose, [4, 2, 2])
        assert surface.last_placement['status'] == 'local_map_road'
        bottom = placed[:3, 3] - placed[:3, 2]
        local = (bottom - surface.origin) @ basis
        assert abs(local[2] - (-2 + .04*local[0])) < 1e-10


def test_outside_support_never_extrapolates_plane():
    surface = GroundSurface([0, 0, 0], np.eye(3), [[0, 0, -2], [1, 0, -2], [0, 1, -2]])
    pose = np.eye(4); pose[:3, 3] = [50, 2, 8]
    np.testing.assert_array_equal(surface.place_box(pose, [4, 2, 2]), pose)
    assert surface.last_placement['status'] == 'unsupported_preserve_input_pose'
    assert surface.last_placement['nearest_support_m'] > 40


def test_seed_connected_ground_excludes_roofs_and_isolated_ground_patches():
    rng = np.random.default_rng(1)
    road = np.array([[x, y, -2] for x in np.arange(-8, 9, .3) for y in np.arange(-4, 5, .3)])
    road[:, 2] += rng.normal(0, .01, len(road))
    roof = road.copy(); roof[:, 2] += 1.5
    isolated = road.copy(); isolated[:, 0] += 100
    surface, stats = GroundSurface.from_map_candidates(np.vstack([road, roof, isolated]), origin=[0, 0, 0], basis=np.eye(3), seed_height_m=-2)
    assert stats['connected_road_cells'] > 10
    assert surface.samples[:, 0].max() < 10
    np.testing.assert_allclose(surface.samples[:, 2], -2, atol=.03)
    loaded = load_ground_placement(surface.to_dict())
    np.testing.assert_array_equal(loaded.samples, surface.samples)


def test_empty_support_has_explicit_fallback():
    surface, stats = GroundSurface.from_map_candidates(np.empty((0, 3)), origin=[0, 0, 0], basis=np.eye(3), seed_height_m=-2)
    np.testing.assert_array_equal(surface.place_box(np.eye(4), [4, 2, 2]), np.eye(4))
    assert surface.last_placement['nearest_support_m'] is None

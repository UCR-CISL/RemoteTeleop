"""Per-splat occluder masks: LiDAR free-space carving and their use in the proxy occluder pass."""
import numpy as np
import pytest

torch = pytest.importorskip("torch")

from scripts.carve_occluder_mask import FreeSpaceCarver  # noqa: E402
from src.viz.composited_camera import OpacityThresholdDepth  # noqa: E402


def _splats(n):
    return {
        "means": torch.arange(3 * n, dtype=torch.float32).reshape(n, 3),
        "quats": torch.tensor([[1.0, 0.0, 0.0, 0.0]] * n),
        "scales": torch.full((n, 3), 0.1),
        "opacities": torch.full((n,), 0.9),
    }


def test_occluder_mask_restricts_only_the_occluder_splats():
    splats = _splats(3)
    keep = np.array([True, False, True])
    kept = OpacityThresholdDepth(ground_max_tilt_deg=None, occluder_mask=keep)._occluder_splats(splats)
    assert torch.equal(kept["means"], splats["means"][[0, 2]])
    assert len(OpacityThresholdDepth(ground_max_tilt_deg=None)._occluder_splats(splats)["means"]) == 3
    with pytest.raises(ValueError, match="occluder_mask has 2 entries"):
        OpacityThresholdDepth(occluder_mask=keep[:2])._occluder_splats(splats)


def test_occluder_mask_combines_with_ground_removal():
    splats = _splats(3)
    splats["scales"][0] = torch.tensor([1.0, 1.0, 0.01])  # flat along z: road-like, dropped by ground removal
    kept = OpacityThresholdDepth(ground_band_m=None, near_ground_m=None,
                                 occluder_mask=np.array([True, True, False]))._occluder_splats(splats)
    assert torch.equal(kept["means"], splats["means"][[1]])


def test_load_occluder_mask_checks_the_splat_map(tmp_path):
    ply, other = tmp_path / "map.ply", tmp_path / "other.ply"
    ply.write_bytes(b"map"); other.write_bytes(b"other")
    path = tmp_path / "map.occluder_mask.npz"
    np.savez(path, occluder_mask=np.array([True, False]), num_splats=2,
             ply_sha256=OpacityThresholdDepth.splat_map_sha256(ply))
    assert OpacityThresholdDepth.load_occluder_mask(path, ply).tolist() == [True, False]
    with pytest.raises(ValueError, match="not made for splat map"):
        OpacityThresholdDepth.load_occluder_mask(path, other)


def test_free_space_carving_marks_only_traversed_empty_space():
    # Sensor at the origin, a wall of returns at x = 10 m; splats: a floater at 5 m on the rays,
    # one on the wall, one behind the wall (never traversed) and one off to the side of every ray.
    means = np.array([[5.0, 0.0, 0.0], [10.0, 0.0, 0.0], [12.0, 0.0, 0.0], [5.0, 4.0, 0.0]])
    carver = FreeSpaceCarver(means, voxel_m=0.2, free_margin_m=0.4, max_range_m=20.0, groups=2, device="cpu")
    y, z = np.meshgrid(np.linspace(-0.3, 0.3, 7), np.linspace(-0.3, 0.3, 7))
    wall = np.stack([np.full(y.size, 10.0), y.ravel(), z.ravel()], axis=1)
    for scan in range(10):
        carver.add_scan(wall, np.zeros(3), group=scan % 2)
    free, hit = carver.splat_counts()
    assert free[0] > 0 and hit[0] == 0 and free[1] == 0 and hit[1] > 0
    assert free[2] == free[3] == hit[2] == hit[3] == 0
    assert carver.splat_counts(groups=[0])[0][0] == free[0] // 2
    mask = FreeSpaceCarver.occluder_mask(free, hit, min_free=8, max_hit_fraction=0.05)
    assert mask.tolist() == [False, True, True, True]
    # A voxel that rays also hit often enough (e.g. a moving vehicle stopping there) stays an occluder.
    assert FreeSpaceCarver.occluder_mask(np.array([100]), np.array([10]), max_hit_fraction=0.05).tolist() == [True]


def test_free_space_carving_subsamples_rays_and_skips_masked_hits():
    means = np.array([[5.0, 0.0, 0.0], [10.0, 0.0, 0.0]])
    carver = FreeSpaceCarver(means, max_range_m=20.0, device="cpu")
    wall = np.array([[10.0, 0.0, 0.0]] * 6 + [[0.5, 0.0, 0.0]])  # last return is inside min_range_m
    carver.add_scan(wall, np.zeros(3), hit_mask=np.array([False] * 6 + [True]), max_rays=3,
                    rng=np.random.default_rng(0))
    free, hit = carver.splat_counts()
    assert free.tolist() == [3, 0] and hit.tolist() == [0, 0]  # moving-vehicle returns end rays, never hit

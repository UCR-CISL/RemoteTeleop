import numpy as np
import pytest
from matplotlib import image as mpimg

from src.localization.gaussian_map import GaussianMap
from src.localization.transforms import compose_map_T_camera
from src.viz.localization_visualizer import (
    plot_localization_trajectory,
    prepare_camera_render,
    render_gaussian_camera_view,
    render_localized_camera_view,
)


def _map() -> GaussianMap:
    return GaussianMap(
        means=np.array([[0.0, 0.0, 5.0]], dtype=np.float32),
        quats=np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32),
        scales=np.ones((1, 3), dtype=np.float32),
        opacities=np.ones(1, dtype=np.float32),
        sh_coeffs=np.zeros((1, 1, 3), dtype=np.float32),
        sh_degree=0,
        antialiased=False,
    )


def test_stationary_trajectory_plot_is_saved(tmp_path):
    poses = np.repeat(np.eye(4)[None], 3, axis=0)
    poses[:, :3, 3] = [12.0, -4.0, 2.0]
    output = plot_localization_trajectory(poses, tmp_path / "stationary.png")
    assert output.exists()
    assert output.stat().st_size > 0


def test_prepare_camera_render_inverts_pose_and_scales_intrinsics():
    map_T_camera = np.eye(4)
    map_T_camera[:3, 3] = [1.0, 2.0, 3.0]
    viewmat, intrinsic, width, height = prepare_camera_render(
        map_T_camera, np.array([[100.0, 0.0, 80.0], [0.0, 120.0, 60.0], [0.0, 0.0, 1.0]]), (160, 120), downsample=2
    )
    np.testing.assert_allclose(viewmat[:3, 3], [-1.0, -2.0, -3.0])
    np.testing.assert_allclose(intrinsic, [[50.0, 0.0, 40.0], [0.0, 60.0, 30.0], [0.0, 0.0, 1.0]])
    assert (width, height) == (80, 60)


def test_compose_camera_pose_uses_explicit_lidar_calibration():
    map_T_lidar = np.eye(4)
    map_T_lidar[0, 3] = 4.0
    lidar_T_camera = np.eye(4)
    lidar_T_camera[1, 3] = 1.5
    np.testing.assert_allclose(compose_map_T_camera(map_T_lidar, lidar_T_camera)[:3, 3], [4.0, 1.5, 0.0])


def test_renderer_can_be_mocked_without_cuda(monkeypatch, tmp_path):
    pose = np.eye(4)
    pose[0, 3] = 2.0
    result = render_gaussian_camera_view(
        _map(), pose, np.eye(3), (8, 6), tmp_path / "render.png", downsample=2, device="cpu"
    )
    assert result.path.exists()
    assert result.backend == "cpu-projective-splats"
    image = mpimg.imread(result.path)
    assert image.shape[:2] == (3, 4)
    assert np.any(image[..., :3] > 0)


def test_cuda_render_failure_falls_back_to_cpu(monkeypatch, tmp_path):
    monkeypatch.setattr(
        "src.viz.localization_visualizer._render_gsplat",
        lambda *args: (_ for _ in ()).throw(RuntimeError("CUDA driver unavailable")),
    )
    result = render_gaussian_camera_view(
        _map(), np.eye(4), np.eye(3), (8, 6), tmp_path / "fallback.png", device="cuda"
    )
    assert result.backend == "cpu-projective-splats"
    image = mpimg.imread(result.path)
    assert image.shape[:2] == (6, 8)
    assert np.any(image[..., :3] > 0)


def test_renderer_rejects_non_png_output(tmp_path):
    with pytest.raises(ValueError, match=".png"):
        render_localized_camera_view(_map(), np.eye(4), np.eye(4), np.eye(3), (4, 4), tmp_path / "bad.jpg")

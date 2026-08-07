"""Offline plots and Gaussian-splat renders for localization results."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from src.localization.gaussian_map import GaussianMap
from src.localization.transforms import compose_map_T_camera


@dataclass(frozen=True)
class CameraRender:
    """A saved camera render and the backend that produced it."""

    path: Path
    backend: str


def plot_localization_trajectory(
    map_T_lidar: Sequence[np.ndarray] | np.ndarray,
    output_path: str | Path,
    *,
    title: str = "Localized LiDAR trajectory",
) -> Path:
    """Save an XY trajectory plot in Gaussian-map coordinates.

    A fixed minimum extent keeps stationary trajectories visible and avoids
    singular-axis warnings.
    """

    poses = _poses(map_T_lidar, "map_T_lidar")
    positions = poses[:, :3, 3]
    output = _output_path(output_path)
    figure, axis = plt.subplots(figsize=(7, 7), constrained_layout=True)
    axis.plot(positions[:, 0], positions[:, 1], "-o", color="tab:blue", ms=3, label="LiDAR")
    axis.scatter(*positions[0, :2], color="tab:green", s=55, label="start", zorder=3)
    axis.scatter(*positions[-1, :2], color="tab:red", marker="x", s=65, label="end", zorder=3)
    forward = poses[-1, :2, 0]
    if (norm := np.linalg.norm(forward)) > np.finfo(float).eps:
        axis.arrow(*positions[-1, :2], *(forward / norm * 0.5), width=0.02, head_width=0.16,
                  color="tab:red", length_includes_head=True, zorder=3)
    _set_trajectory_limits(axis, positions[:, :2])
    axis.set_aspect("equal", adjustable="box")
    axis.set(xlabel="Gaussian-map X (m)", ylabel="Gaussian-map Y (m)", title=title)
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.savefig(output, dpi=160)
    plt.close(figure)
    return output


def render_localized_camera_view(
    gaussian_map: GaussianMap,
    map_T_lidar: np.ndarray,
    lidar_T_camera: np.ndarray,
    camera_intrinsic: np.ndarray,
    image_size: tuple[int, int],
    output_path: str | Path,
    *,
    downsample: int = 1,
    device: str | None = None,
    max_splats: int = 150_000,
) -> CameraRender:
    """Render a camera calibrated to a localized LiDAR pose as a PNG."""

    return render_gaussian_camera_view(
        gaussian_map, compose_map_T_camera(map_T_lidar, lidar_T_camera), camera_intrinsic,
        image_size, output_path, downsample=downsample, device=device, max_splats=max_splats,
    )


def render_gaussian_camera_view(
    gaussian_map: GaussianMap,
    map_T_camera: np.ndarray,
    camera_intrinsic: np.ndarray,
    image_size: tuple[int, int],
    output_path: str | Path,
    *,
    downsample: int = 1,
    device: str | None = None,
    max_splats: int = 150_000,
) -> CameraRender:
    """Render an already composed map-frame camera pose.

    CUDA uses gsplat.  CPU uses a depth-sorted Gaussian-center projection for
    useful localization inspection when the NVIDIA driver is unavailable.
    """

    viewmat, intrinsic, width, height = prepare_camera_render(
        map_T_camera, camera_intrinsic, image_size, downsample=downsample
    )
    if max_splats <= 0:
        raise ValueError("max_splats must be positive")
    try:
        import torch
    except ImportError:
        return _render_cpu(gaussian_map, viewmat, intrinsic, width, height, output_path, max_splats)
    try:
        target_device = device or ("cuda" if torch.cuda.is_available() else "cpu")
    except Exception:  # NVML/driver initialization can fail after importing torch.
        target_device = "cpu"
    if not target_device.startswith("cuda"):
        return _render_cpu(gaussian_map, viewmat, intrinsic, width, height, output_path, max_splats)
    try:
        colors = _render_gsplat(torch, gaussian_map, viewmat, intrinsic, width, height, target_device)
    except Exception:
        # A host can report CUDA available but fail tensor allocation or gsplat
        # initialization (for example when NVML cannot reach the driver).
        return _render_cpu(gaussian_map, viewmat, intrinsic, width, height, output_path, max_splats)
    output = _output_path(output_path)
    _save_colors(output, colors)
    return CameraRender(output, "gsplat-cuda")


def _render_gsplat(torch, gaussian_map: GaussianMap, viewmat: np.ndarray,
                   intrinsic: np.ndarray, width: int, height: int, device: str):
    """Return gsplat's batched ``(camera, height, width, color)`` tensor."""

    tensors = {
        "means": torch.as_tensor(gaussian_map.means, dtype=torch.float32, device=device),
        "quats": torch.as_tensor(gaussian_map.quats, dtype=torch.float32, device=device),
        "scales": torch.as_tensor(gaussian_map.scales, dtype=torch.float32, device=device),
        "opacities": torch.as_tensor(gaussian_map.opacities, dtype=torch.float32, device=device),
        "colors": torch.as_tensor(gaussian_map.sh_coeffs, dtype=torch.float32, device=device),
        "viewmats": torch.as_tensor(viewmat[None], dtype=torch.float32, device=device),
        "Ks": torch.as_tensor(intrinsic[None], dtype=torch.float32, device=device),
    }
    colors, _, _ = _rasterize(tensors, gaussian_map, width, height)
    return colors


def render_gaussian_camera(
    splats: GaussianMap, map_T_camera: np.ndarray, intrinsic: np.ndarray, output_path: str | Path,
    *, width: int = 1920, height: int = 1080, downsample: int = 1, device: str | None = None,
) -> CameraRender:
    """Compatibility wrapper for a map-frame Gaussian camera render."""

    return render_gaussian_camera_view(
        splats, map_T_camera, intrinsic, (width, height), output_path, downsample=downsample, device=device
    )


def prepare_camera_render(
    map_T_camera: np.ndarray, camera_intrinsic: np.ndarray, image_size: tuple[int, int], *, downsample: int = 1,
) -> tuple[np.ndarray, np.ndarray, int, int]:
    """Return gsplat world-to-camera matrix, scaled intrinsics, and dimensions."""

    pose = _poses(map_T_camera, "map_T_camera")[0]
    intrinsic = np.asarray(camera_intrinsic, dtype=np.float64)
    if intrinsic.shape != (3, 3) or not np.isfinite(intrinsic).all():
        raise ValueError("camera_intrinsic must be a finite 3x3 matrix")
    width, height = image_size
    if not isinstance(width, int) or not isinstance(height, int) or width <= 0 or height <= 0:
        raise ValueError("image_size must be positive integer (width, height)")
    if not isinstance(downsample, int) or downsample <= 0:
        raise ValueError("downsample must be a positive integer")
    scaled = intrinsic.copy()
    scaled[:2] /= downsample
    return np.linalg.inv(pose), scaled, max(1, width // downsample), max(1, height // downsample)


def _render_cpu(map_: GaussianMap, world_to_camera: np.ndarray, intrinsic: np.ndarray,
                width: int, height: int, output_path: str | Path, max_splats: int) -> CameraRender:
    """Draw visible centers as radius-scaled, depth-sorted alpha splats."""

    camera_points = np.asarray(map_.means, dtype=np.float64) @ world_to_camera[:3, :3].T + world_to_camera[:3, 3]
    depth = camera_points[:, 2]
    projected = camera_points @ intrinsic.T
    pixels = projected[:, :2] / np.maximum(depth[:, None], 1e-4)
    indices = np.flatnonzero((depth > 1e-4) & (pixels[:, 0] >= -24) & (pixels[:, 0] < width + 24)
                         & (pixels[:, 1] >= -24) & (pixels[:, 1] < height + 24))
    if len(indices) > max_splats:
        indices = indices[np.argpartition(map_.opacities[indices], -max_splats)[-max_splats:]]
    image = np.zeros((height, width, 3), dtype=np.float32)
    for index in indices[np.argsort(depth[indices])[::-1]]:  # far to near
        x, y = pixels[index]
        radius = int(np.clip(np.mean(map_.scales[index]) * intrinsic[0, 0] / depth[index], 1, 24))
        x0, x1 = max(0, int(x) - radius), min(width, int(x) + radius + 1)
        y0, y1 = max(0, int(y) - radius), min(height, int(y) + radius + 1)
        if x0 >= x1 or y0 >= y1:
            continue
        yy, xx = np.ogrid[y0:y1, x0:x1]
        mask = (xx - x) ** 2 + (yy - y) ** 2 <= radius * radius
        alpha = float(np.clip(map_.opacities[index], 0.0, 1.0))
        color = np.clip(0.5 + 0.28209479177387814 * map_.sh_coeffs[index, 0], 0.0, 1.0)
        patch = image[y0:y1, x0:x1]
        patch[mask] = color * alpha + patch[mask] * (1.0 - alpha)
    output = _output_path(output_path)
    _save_colors(output, image)
    return CameraRender(output, "cpu-projective-splats")


def _save_colors(output: Path, colors) -> None:
    """Save one RGB image, rejecting accidental batch/tuple-shaped outputs."""

    image = colors.detach().float().cpu().numpy() if hasattr(colors, "detach") else np.asarray(colors)
    if image.ndim == 4:
        if image.shape[0] != 1:
            raise ValueError(f"expected one rendered camera, got color tensor shape {image.shape}")
        image = image[0]
    if image.ndim != 3 or image.shape[2] not in (3, 4):
        raise ValueError(f"rendered colors must have shape (height, width, 3|4), got {image.shape}")
    plt.imsave(output, np.clip(image, 0.0, 1.0))


def _rasterize(tensors, gaussian_map: GaussianMap, width: int, height: int):
    from gsplat import rasterization
    return rasterization(tensors["means"], tensors["quats"], tensors["scales"], tensors["opacities"],
                         tensors["colors"], tensors["viewmats"], tensors["Ks"], width, height,
                         sh_degree=gaussian_map.sh_degree, packed=False,
                         rasterize_mode="antialiased" if gaussian_map.antialiased else "classic")


def _poses(values: Sequence[np.ndarray] | np.ndarray, name: str) -> np.ndarray:
    poses = np.asarray(values, dtype=np.float64)
    if poses.shape == (4, 4):
        poses = poses[None]
    if poses.ndim != 3 or poses.shape[1:] != (4, 4) or not np.isfinite(poses).all() or len(poses) == 0:
        raise ValueError(f"{name} must have shape (N, 4, 4) or (4, 4) and finite values")
    if not np.allclose(poses[:, 3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError(f"{name} must contain homogeneous transforms")
    return poses


def _set_trajectory_limits(axis, xy: np.ndarray) -> None:
    center = (xy.min(axis=0) + xy.max(axis=0)) / 2
    extent = np.maximum(xy.max(axis=0) - xy.min(axis=0), 2.0)
    padding = np.maximum(0.15 * extent, 0.5)
    axis.set_xlim(center[0] - extent[0] / 2 - padding[0], center[0] + extent[0] / 2 + padding[0])
    axis.set_ylim(center[1] - extent[1] / 2 - padding[1], center[1] + extent[1] / 2 + padding[1])


def _output_path(path: str | Path) -> Path:
    output = Path(path)
    if output.suffix.lower() != ".png":
        raise ValueError("output_path must end in .png")
    output.parent.mkdir(parents=True, exist_ok=True)
    return output

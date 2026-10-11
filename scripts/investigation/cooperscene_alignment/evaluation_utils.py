"""Unchanged map extraction and registration settings from the prior evaluation."""
from pathlib import Path
import numpy as np
from src.localization.gaussian_map import load_gaussian_map
from src.localization.registration import RegistrationConfig


def map_proxy(
    map_path: str | Path, *, min_opacity: float = 0.0, stride: int = 1
) -> np.ndarray:
    gaussian_map = load_gaussian_map(map_path)
    means, opacities = gaussian_map.means, gaussian_map.opacities
    keep = (
        np.isfinite(means).all(axis=1)
        & np.isfinite(opacities)
        & (opacities > min_opacity)
    )
    keep &= np.abs(means).max(axis=1) <= 10000
    return means[keep][::stride].astype(np.float64)


def registration_config(dataset: str) -> RegistrationConfig:
    if dataset == "cooperscene":
        return RegistrationConfig(
            coarse_voxel_size=0.25,
            refinement_voxel_sizes=(0.25, 0.10),
            icp_distance_factor=2.0,
            min_inlier_ratio=0.35,
            max_median_residual=0.20,
            max_p95_residual=0.60,
            max_all_point_median_residual=0.25,
            max_all_point_p95_residual=0.75,
            max_correction_translation=1.0,
            max_correction_rotation_degrees=3.0,
        )
    return RegistrationConfig()


def jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {key: jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [jsonable(item) for item in value]
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value

#!/usr/bin/env python3
"""Visualize CooperScene initial-pose error and Gaussian-map registration."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.localization.cooperscene import CooperSceneSequenceDataset
from src.localization.gaussian_map import load_gaussian_map


_WGS84_A_M = 6_378_137.0
_WGS84_F = 1.0 / 298.257223563
_COOPERSCENE_ORIGIN_LLA = (33.975676, -117.339873, 273.0)
_INTERSECTION_LAT_LON = (33.975676, -117.339892)


@dataclass(frozen=True)
class DiagnosticConfig:
    localization: Path
    georeference: Path
    output_dir: Path
    data_root: Path | None = None
    splat_path: Path | None = None
    max_map_points: int = 100_000
    max_lidar_points: int = 50_000
    roi_margin_m: float = 5.0
    random_seed: int = 0


class CooperSceneLocalizationDiagnostic:
    """Reconstruct and plot the inputs and result of initial localization."""

    def __init__(self, config: DiagnosticConfig) -> None:
        if min(config.max_map_points, config.max_lidar_points) < 1:
            raise ValueError("point display limits must be positive")
        if config.roi_margin_m <= 0:
            raise ValueError("roi_margin_m must be positive")
        self.config = config

    def run(self) -> dict:
        payload = json.loads(self.config.localization.read_text(encoding="utf-8"))
        run_config = payload["config"]
        data_root = self.config.data_root or Path(run_config["data_root"])
        splat_path = self.config.splat_path or Path(run_config["splat_path"])
        split, scenario, agent = (str(run_config[key]) for key in ("split", "scenario", "agent"))
        frame_id = str(payload["synchronized_frame_id"])

        frame = next(
            frame
            for frame in CooperSceneSequenceDataset(data_root).sequence(split, scenario, agent)
            if frame.frame_id == frame_id
        )
        recorded_T_lidar = np.asarray(frame.map_T_lidar, dtype=np.float64)
        lidar_points = _transform_points(
            np.asarray(frame.require_lidar_points()[:, :3], dtype=np.float64),
            recorded_T_lidar,
        )
        gaussian_map = load_gaussian_map(splat_path)
        map_points = _finite_gaussian_centers(gaussian_map.means, gaussian_map.opacities)
        estimated_T = np.asarray(payload["gaussian_T_cooperscene"], dtype=np.float64)
        initial_T = _load_transform(run_config.get("initial_transform_path"))
        if initial_T is None:
            initial_T = np.eye(4, dtype=np.float64)

        georeference = json.loads(self.config.georeference.read_text(encoding="utf-8"))
        gsplat_origin = georeference["enu_origin"]
        z_reference = georeference.get("z_reference", {}).get("z_ref_m", 0.0)
        gsplat_lla = (
            float(gsplat_origin["latitude_deg"]),
            float(gsplat_origin["longitude_deg"]),
            float(z_reference),
        )
        cooper_intersection = _enu_from_lla(
            (*_INTERSECTION_LAT_LON, _COOPERSCENE_ORIGIN_LLA[2]),
            _COOPERSCENE_ORIGIN_LLA,
        )
        gsplat_intersection = _enu_from_lla(
            (*_INTERSECTION_LAT_LON, gsplat_lla[2]),
            gsplat_lla,
        )
        geodetic_origin_offset = _enu_from_lla(_COOPERSCENE_ORIGIN_LLA, gsplat_lla)
        reconciled_origin_offset = geodetic_origin_offset.copy()
        reconciled_origin_offset[:2] = gsplat_intersection[:2] - cooper_intersection[:2]
        expected_T = np.eye(4, dtype=np.float64)
        expected_T[:3, 3] = reconciled_origin_offset

        reference_pose = expected_T @ recorded_T_lidar
        estimated_pose = estimated_T @ recorded_T_lidar
        position_delta = estimated_pose[:3, 3] - reference_pose[:3, 3]
        horizontal_error = float(np.linalg.norm(position_delta[:2]))
        orientation_error = _rotation_degrees(
            estimated_pose[:3, :3] @ reference_pose[:3, :3].T
        )
        heading_error = _heading_error_degrees(estimated_pose, reference_pose)
        reference_intersection_xy = recorded_T_lidar[:2, 3] - cooper_intersection[:2]
        estimated_intersection_xy = estimated_pose[:2, 3] - gsplat_intersection[:2]
        reconciled_delta = estimated_intersection_xy - reference_intersection_xy
        reconciled_error = float(np.linalg.norm(reconciled_delta))

        self.config.output_dir.mkdir(parents=True, exist_ok=True)
        error_path = self.config.output_dir / "initial_localization_error.png"
        alignment_path = self.config.output_dir / "point_cloud_alignment.png"
        reconciliation_path = self.config.output_dir / "origin_reconciliation.png"
        self._plot_error(
            expected_T, estimated_T, reference_pose, estimated_pose,
            position_delta, horizontal_error, heading_error, orientation_error, error_path,
        )
        self._plot_origin_reconciliation(
            recorded_T_lidar[:2, 3], estimated_pose[:2, 3],
            cooper_intersection[:2], gsplat_intersection[:2],
            reference_intersection_xy, estimated_intersection_xy,
            reconciled_delta, reconciled_error, reconciliation_path,
        )
        alignment = self._plot_alignment(
            lidar_points, map_points, initial_T, estimated_T, payload,
            alignment_path,
        )
        summary = {
            "frame_id": frame_id,
            "horizontal_error_m": horizontal_error,
            "east_error_m": float(position_delta[0]),
            "north_error_m": float(position_delta[1]),
            "orientation_error_deg": orientation_error,
            "heading_error_deg": heading_error,
            "intersection_lat_lon": list(_INTERSECTION_LAT_LON),
            "intersection_in_cooperscene_enu_m": cooper_intersection[:2].tolist(),
            "intersection_in_gsplat_enu_m": gsplat_intersection[:2].tolist(),
            "intersection_centered_reference_xy_m": reference_intersection_xy.tolist(),
            "intersection_centered_estimated_xy_m": estimated_intersection_xy.tolist(),
            "intersection_centered_delta_xy_m": reconciled_delta.tolist(),
            "intersection_centered_horizontal_discrepancy_m": reconciled_error,
            "geodetic_cooperscene_origin_in_gsplat_enu_m": geodetic_origin_offset.tolist(),
            "intersection_reconciled_cooperscene_origin_in_gsplat_enu_m": reconciled_origin_offset.tolist(),
            "intersection_reconciliation_horizontal_closure_m": float(np.linalg.norm(
                cooper_intersection[:2] + reconciled_origin_offset[:2] - gsplat_intersection[:2]
            )),
            "expected_gaussian_T_cooperscene": expected_T.tolist(),
            "estimated_gaussian_T_cooperscene": estimated_T.tolist(),
            "error_figure": error_path.name,
            "origin_reconciliation_figure": reconciliation_path.name,
            "alignment_figure": alignment_path.name,
            **alignment,
        }
        (self.config.output_dir / "diagnostic.json").write_text(
            json.dumps(summary, indent=2), encoding="utf-8"
        )
        return summary

    @staticmethod
    def _plot_error(
        expected_T: np.ndarray,
        estimated_T: np.ndarray,
        reference_pose: np.ndarray,
        estimated_pose: np.ndarray,
        delta: np.ndarray,
        horizontal_error: float,
        heading_error: float,
        orientation_error: float,
        output_path: Path,
    ) -> None:
        figure, (site, origins, zoom) = plt.subplots(
            1, 3, figsize=(19, 6), constrained_layout=True
        )
        expected_origin = expected_T[:2, 3]
        estimated_origin = estimated_T[:2, 3]
        reference_xy = reference_pose[:2, 3]
        estimated_xy = estimated_pose[:2, 3]

        site.scatter(*[0.0, 0.0], marker="*", s=180, color="black", label="GSplat ENU origin")
        site.scatter(*expected_origin, marker="s", s=70, color="tab:blue", label="CooperScene origin (geodetic)")
        site.scatter(*estimated_origin, marker="x", s=90, color="tab:red", label="CooperScene origin (ICP estimate)")
        site.scatter(*reference_xy, marker="o", s=70, color="tab:green", label="Initial LiDAR pose (ENU reference)")
        site.scatter(*estimated_xy, marker="x", s=90, color="tab:orange", label="Initial LiDAR pose (localized)")
        site.plot([expected_origin[0], reference_xy[0]], [expected_origin[1], reference_xy[1]],
                  linestyle="--", color="tab:blue", alpha=0.6)
        site.set_title("Common GSplat ENU frame")
        site.set_xlabel("East [m]")
        site.set_ylabel("North [m]")
        site.axis("equal")
        site.grid(alpha=0.25)
        site.legend(fontsize=8, loc="best")

        origins.scatter(*expected_origin, marker="s", s=90, color="tab:blue",
                        label="Geodetic origin")
        origins.scatter(*estimated_origin, marker="x", s=110, linewidth=2.5,
                        color="tab:red", label="ICP-estimated origin")
        origins.plot([expected_origin[0], estimated_origin[0]],
                     [expected_origin[1], estimated_origin[1]], color="tab:red", linewidth=2)
        origin_delta = estimated_origin - expected_origin
        origins.plot([expected_origin[0], estimated_origin[0]],
                     [expected_origin[1], expected_origin[1]], linestyle=":", color="tab:purple")
        origins.plot([estimated_origin[0], estimated_origin[0]],
                     [expected_origin[1], estimated_origin[1]], linestyle=":", color="tab:brown")
        origins.set_title(
            f"Frame-origin translation residual: {np.linalg.norm(origin_delta):.3f} m\n"
            f"ΔE {origin_delta[0]:+.3f} m, ΔN {origin_delta[1]:+.3f} m"
        )
        origins.set_xlabel("East [m]")
        origins.set_ylabel("North [m]")
        origins.axis("equal")
        origins.grid(alpha=0.25)
        origins.legend(fontsize=8, loc="best")

        translation_only_xy = reference_xy + origin_delta
        zoom.scatter(*reference_xy, s=110, color="tab:green", label="Geodetic ENU reference")
        zoom.scatter(*translation_only_xy, marker="D", s=70, color="tab:blue",
                     label="After origin-translation residual")
        zoom.scatter(*estimated_xy, marker="x", s=130, linewidth=2.5,
                     color="tab:orange", label="ICP-localized pose")
        zoom.annotate("", xy=translation_only_xy, xytext=reference_xy,
                      arrowprops={"arrowstyle": "->", "color": "tab:blue", "lw": 2})
        zoom.annotate("", xy=estimated_xy, xytext=translation_only_xy,
                      arrowprops={"arrowstyle": "->", "color": "tab:orange", "lw": 2})
        zoom.plot([reference_xy[0], estimated_xy[0]], [reference_xy[1], estimated_xy[1]],
                  color="tab:red", linewidth=1.5, alpha=0.7)
        zoom.plot([reference_xy[0], estimated_xy[0]], [reference_xy[1], reference_xy[1]],
                  linestyle=":", color="tab:purple", label=f"East error {delta[0]:+.3f} m")
        zoom.plot([estimated_xy[0], estimated_xy[0]], [reference_xy[1], estimated_xy[1]],
                  linestyle=":", color="tab:brown", label=f"North error {delta[1]:+.3f} m")
        for pose, color in ((reference_pose, "tab:green"), (estimated_pose, "tab:orange")):
            start = pose[:2, 3]
            end = start + pose[:2, 0] * 0.45
            zoom.annotate("", xy=end, xytext=start,
                          arrowprops={"arrowstyle": "-|>", "color": color, "lw": 2})
        zoom.set_title(
            f"Initial pose: {horizontal_error:.3f} m BEV, heading {abs(heading_error):.3f}°\n"
            f"Full 3D rotation difference: {orientation_error:.3f}°"
        )
        zoom.set_xlabel("East [m]")
        zoom.set_ylabel("North [m]")
        zoom.axis("equal")
        zoom.grid(alpha=0.25)
        zoom.legend(fontsize=8, loc="best")
        lever_delta = estimated_xy - translation_only_xy
        zoom.text(
            0.02, 0.02,
            f"Origin translation: ({origin_delta[0]:+.3f}, {origin_delta[1]:+.3f}) m\n"
            f"Rotation lever arm: ({lever_delta[0]:+.3f}, {lever_delta[1]:+.3f}) m\n"
            f"Combined: ({delta[0]:+.3f}, {delta[1]:+.3f}) m",
            transform=zoom.transAxes, fontsize=9, va="bottom",
            bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "0.8"},
        )
        figure.suptitle("CooperScene initial localization error decomposition", fontsize=15)
        figure.savefig(output_path, dpi=180)
        plt.close(figure)

    @staticmethod
    def _plot_origin_reconciliation(
        recorded_xy: np.ndarray,
        estimated_xy: np.ndarray,
        cooper_intersection: np.ndarray,
        gsplat_intersection: np.ndarray,
        reference_common: np.ndarray,
        estimated_common: np.ndarray,
        delta: np.ndarray,
        error: float,
        output_path: Path,
    ) -> None:
        figure, axes = plt.subplots(1, 3, figsize=(19, 6), constrained_layout=True)
        cooper = axes[0]
        cooper.scatter(0.0, 0.0, marker="*", s=180, color="black",
                       label="CooperScene ENU origin")
        cooper.scatter(*cooper_intersection, marker="s", s=90, color="tab:purple",
                       label="Shared physical intersection")
        cooper.scatter(*recorded_xy, marker="o", s=90, color="tab:orange",
                       label="CooperScene-reported LiDAR pose")
        cooper.plot([cooper_intersection[0], recorded_xy[0]],
                    [cooper_intersection[1], recorded_xy[1]], linestyle="--", color="0.5")
        cooper.set_title(
            "CooperScene reported ENU\n"
            f"intersection = ({cooper_intersection[0]:+.3f}, "
            f"{cooper_intersection[1]:+.3f}) m"
        )

        gsplat = axes[1]
        expected_gsplat_xy = reference_common + gsplat_intersection
        gsplat.scatter(0.0, 0.0, marker="*", s=180, color="black",
                       label="GSplat metadata origin")
        gsplat.scatter(*gsplat_intersection, marker="s", s=90, color="tab:purple",
                       label="Shared physical intersection")
        gsplat.scatter(*expected_gsplat_xy, marker="o", s=90, color="tab:green",
                       label="CooperScene pose via origin reconciliation")
        gsplat.scatter(*estimated_xy, marker="x", s=110, linewidth=2.5,
                       color="tab:orange", label="LiDAR-to-GSplat ICP pose")
        gsplat.plot([gsplat_intersection[0], expected_gsplat_xy[0]],
                    [gsplat_intersection[1], expected_gsplat_xy[1]],
                    linestyle="--", color="tab:green", alpha=0.7)
        gsplat.plot([gsplat_intersection[0], estimated_xy[0]],
                    [gsplat_intersection[1], estimated_xy[1]],
                    linestyle="--", color="tab:orange", alpha=0.7)
        gsplat.set_title(
            "GSplat georeferenced map coordinates\n"
            f"intersection = ({gsplat_intersection[0]:+.3f}, "
            f"{gsplat_intersection[1]:+.3f}) m"
        )

        for axis in axes[:2]:
            axis.set_xlabel("East [m]")
            axis.set_ylabel("North [m]")
            axis.axis("equal")
            axis.grid(alpha=0.25)
            axis.legend(fontsize=8)

        common = axes[2]
        common.scatter(*reference_common, s=110, color="tab:green",
                       label="CooperScene ENU reference")
        common.scatter(*estimated_common, marker="x", s=130, linewidth=2.5,
                       color="tab:orange", label="ICP-localized pose")
        common.plot([reference_common[0], estimated_common[0]],
                    [reference_common[1], estimated_common[1]], color="tab:red", linewidth=2)
        common.plot([reference_common[0], estimated_common[0]],
                    [reference_common[1], reference_common[1]], linestyle=":",
                    color="tab:purple", label=f"ΔE {delta[0]:+.3f} m")
        common.plot([estimated_common[0], estimated_common[0]],
                    [reference_common[1], estimated_common[1]], linestyle=":",
                    color="tab:brown", label=f"ΔN {delta[1]:+.3f} m")
        common.set_title(
            "Both frames re-originated at the same intersection\n"
            f"vehicle-pose zoom: remaining BEV discrepancy = {error:.3f} m"
        )
        common.set_xlabel("East from intersection [m]")
        common.set_ylabel("North from intersection [m]")
        common.axis("equal")
        common.grid(alpha=0.25)
        common.legend(fontsize=8, loc="best")
        figure.suptitle("CooperScene and GSplat origin reconciliation", fontsize=15)
        figure.savefig(output_path, dpi=180)
        plt.close(figure)

    def _plot_alignment(
        self,
        lidar_points: np.ndarray,
        map_points: np.ndarray,
        initial_T: np.ndarray,
        estimated_T: np.ndarray,
        payload: dict,
        output_path: Path,
    ) -> dict:
        initial_lidar = _transform_points(lidar_points, initial_T)
        final_lidar = _transform_points(lidar_points, estimated_T)
        roi_map = _crop_to_cloud(map_points, final_lidar, self.config.roi_margin_m)
        residuals = _nearest_residuals(final_lidar, roi_map)
        rng = np.random.default_rng(self.config.random_seed)
        displayed_map = _sample(roi_map, self.config.max_map_points, rng)
        displayed_initial = _sample(initial_lidar, self.config.max_lidar_points, rng)
        displayed_final = _sample(final_lidar, self.config.max_lidar_points, rng)

        figure, axes = plt.subplots(2, 2, figsize=(15, 12))
        sensor_xy = np.asarray(payload["selected_map_T_lidar"], dtype=np.float64)[:2, 3]
        view_radius = 45.0
        self._cloud_panel(axes[0, 0], displayed_map, displayed_initial, (0, 1), "Initial-prior overlay", "East", "North")
        self._cloud_panel(axes[0, 1], displayed_map, displayed_final, (0, 1), "Final ICP overlay", "East", "North")
        self._cloud_panel(axes[1, 0], displayed_map, displayed_final, (0, 2), "Final ICP side view", "East", "Up")
        for axis in axes[0]:
            axis.set_xlim(sensor_xy[0] - view_radius, sensor_xy[0] + view_radius)
            axis.set_ylim(sensor_xy[1] - view_radius, sensor_xy[1] + view_radius)
        axes[1, 0].set_xlim(sensor_xy[0] - view_radius, sensor_xy[0] + view_radius)
        histogram_limit = float(np.percentile(residuals, 99.5))
        axes[1, 1].hist(residuals[residuals <= histogram_limit], bins=80,
                        color="tab:orange", alpha=0.85)
        final = payload["final_registration"]
        axes[1, 1].axvline(0.20, color="tab:blue", linestyle="-.",
                           label="Final ICP inlier threshold 0.200 m")
        axes[1, 1].axvline(final["median_residual"], color="black", linestyle="--",
                           label=f"Inlier median {final['median_residual']:.3f} m")
        axes[1, 1].axvline(final["p95_residual"], color="tab:red", linestyle=":",
                           label=f"Inlier p95 {final['p95_residual']:.3f} m")
        axes[1, 1].set_title("LiDAR-to-proxy nearest-neighbor residuals")
        axes[1, 1].set_xlabel("Residual [m]")
        axes[1, 1].set_ylabel("LiDAR point count")
        axes[1, 1].set_xlim(0.0, histogram_limit)
        axes[1, 1].grid(alpha=0.25)
        axes[1, 1].legend(fontsize=9)
        clipped_count = int(np.count_nonzero(residuals > histogram_limit))
        axes[1, 1].text(
            0.98, 0.80, f"{clipped_count:,} points above {histogram_limit:.2f} m omitted",
            transform=axes[1, 1].transAxes, ha="right", fontsize=9,
        )
        correction = payload["final_registration"]
        figure.suptitle(
            "Registration inputs and result\n"
            "Gaussian centers → 0.25 m ICP → 0.10 m ICP → final transform "
            f"(Δ {correction['correction_translation']:.3f} m, "
            f"{correction['correction_rotation_degrees']:.3f}° from prior)",
            fontsize=15,
        )
        figure.text(
            0.5, 0.015,
            f"Displayed: {len(displayed_map):,}/{len(roi_map):,} ROI Gaussian centers and "
            f"{len(displayed_final):,}/{len(final_lidar):,} LiDAR points. "
            "Display sampling does not affect registration.",
            ha="center", fontsize=9,
        )
        figure.subplots_adjust(left=0.07, right=0.98, bottom=0.08, top=0.88,
                               hspace=0.28, wspace=0.20)
        figure.savefig(output_path, dpi=180)
        plt.close(figure)
        return {
            "lidar_point_count": len(final_lidar),
            "roi_gaussian_center_count": len(roi_map),
            "displayed_lidar_point_count": len(displayed_final),
            "displayed_gaussian_center_count": len(displayed_map),
            "all_point_median_residual_m_recomputed": float(np.median(residuals)),
            "all_point_p95_residual_m_recomputed": float(np.percentile(residuals, 95)),
        }

    @staticmethod
    def _cloud_panel(axis, map_points, lidar_points, dimensions, title, x_name, y_name) -> None:
        x, y = dimensions
        axis.scatter(map_points[:, x], map_points[:, y], s=0.35, alpha=0.22,
                     color="tab:blue", rasterized=True, label="Gaussian-center proxy")
        axis.scatter(lidar_points[:, x], lidar_points[:, y], s=1.2, alpha=0.55,
                     color="tab:orange", rasterized=True, label="CooperScene LiDAR")
        axis.set_title(title)
        axis.set_xlabel(f"{x_name} [m]")
        axis.set_ylabel(f"{y_name} [m]")
        axis.set_aspect("equal", adjustable="box")
        axis.grid(alpha=0.2)
        axis.legend(markerscale=5, fontsize=8)


def _finite_gaussian_centers(means: np.ndarray, opacities: np.ndarray) -> np.ndarray:
    means = np.asarray(means, dtype=np.float64)
    opacities = np.asarray(opacities, dtype=np.float64)
    keep = np.isfinite(means).all(axis=1) & np.isfinite(opacities) & (opacities > 0.0)
    keep &= np.abs(means).max(axis=1) <= 10_000.0
    result = means[keep]
    if not len(result):
        raise ValueError("Gaussian map contains no valid centers")
    return result


def _transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


def _load_transform(path_value: str | None) -> np.ndarray | None:
    if not path_value:
        return None
    path = Path(path_value)
    if path.suffix == ".npz":
        payload = np.load(path)
        for key in ("gaussian_T_cooperscene", "map_T_source", "transform"):
            if key in payload:
                return np.asarray(payload[key], dtype=np.float64)
        raise ValueError(f"no transform found in {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    for key in ("gaussian_T_cooperscene", "map_T_source", "transform"):
        if key in payload:
            return np.asarray(payload[key], dtype=np.float64)
    raise ValueError(f"no transform found in {path}")


def _crop_to_cloud(map_points: np.ndarray, cloud: np.ndarray, margin: float) -> np.ndarray:
    lower = cloud.min(axis=0) - margin
    upper = cloud.max(axis=0) + margin
    keep = np.all((map_points >= lower) & (map_points <= upper), axis=1)
    result = map_points[keep]
    if not len(result):
        raise ValueError("no Gaussian centers overlap the localized LiDAR extent")
    return result


def _sample(points: np.ndarray, limit: int, rng: np.random.Generator) -> np.ndarray:
    if len(points) <= limit:
        return points
    return points[rng.choice(len(points), size=limit, replace=False)]


def _nearest_residuals(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    try:
        from scipy.spatial import cKDTree
    except ImportError as error:  # pragma: no cover - installation dependent
        raise ImportError("SciPy is required to calculate diagnostic residuals") from error
    return cKDTree(target).query(source, k=1, workers=-1)[0]


def _rotation_degrees(rotation: np.ndarray) -> float:
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def _heading_error_degrees(estimated_pose: np.ndarray, reference_pose: np.ndarray) -> float:
    estimated = math.atan2(estimated_pose[1, 0], estimated_pose[0, 0])
    reference = math.atan2(reference_pose[1, 0], reference_pose[0, 0])
    return math.degrees(math.atan2(math.sin(estimated - reference), math.cos(estimated - reference)))


def _enu_from_lla(lla: tuple[float, float, float], origin: tuple[float, float, float]) -> np.ndarray:
    delta = _ecef(*lla) - _ecef(*origin)
    latitude, longitude = map(math.radians, origin[:2])
    rotation = np.array([
        [-math.sin(longitude), math.cos(longitude), 0.0],
        [-math.sin(latitude) * math.cos(longitude), -math.sin(latitude) * math.sin(longitude), math.cos(latitude)],
        [math.cos(latitude) * math.cos(longitude), math.cos(latitude) * math.sin(longitude), math.sin(latitude)],
    ])
    return rotation @ delta


def _ecef(latitude_deg: float, longitude_deg: float, altitude_m: float) -> np.ndarray:
    latitude, longitude = math.radians(latitude_deg), math.radians(longitude_deg)
    eccentricity_sq = _WGS84_F * (2.0 - _WGS84_F)
    radius = _WGS84_A_M / math.sqrt(1.0 - eccentricity_sq * math.sin(latitude) ** 2)
    return np.array([
        (radius + altitude_m) * math.cos(latitude) * math.cos(longitude),
        (radius + altitude_m) * math.cos(latitude) * math.sin(longitude),
        (radius * (1.0 - eccentricity_sq) + altitude_m) * math.sin(latitude),
    ])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--localization", type=Path, required=True)
    parser.add_argument("--georeference", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--data-root", type=Path)
    parser.add_argument("--splat", dest="splat_path", type=Path)
    parser.add_argument("--max-map-points", type=int, default=100_000)
    parser.add_argument("--max-lidar-points", type=int, default=50_000)
    parser.add_argument("--roi-margin-m", type=float, default=5.0)
    args = parser.parse_args()
    result = CooperSceneLocalizationDiagnostic(DiagnosticConfig(**vars(args))).run()
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()

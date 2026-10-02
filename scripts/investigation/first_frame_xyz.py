"""Offline first-frame XYZ investigation; retain rejected candidates and original gates.

Run with the repository environment, capped BLAS/OMP threads, and --inputs pointing
at the shared first-frame cache. Reference agreement is not independent accuracy.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from src.localization.gaussian_map import load_gaussian_map
from src.localization.registration import PointCloudRegistrar, RegistrationConfig
from src.localization.enu_prior_fusion import EnuPriorFuser, EnuPriorFusionConfig
import src.localization.enu_prior_fusion as fusion_module


class LimitedKDTree(cKDTree):
    """Bound SciPy worker creation; nearest-neighbor results are unchanged."""

    def query(self, *args, **kwargs):
        kwargs["workers"] = 2
        return super().query(*args, **kwargs)


def source_points(frame, mask_dynamic: bool) -> np.ndarray:
    pose = frame["dataset_T_lidar"]
    points = frame["points_lidar"][:, :3].astype(np.float64) @ pose[:3, :3].T + pose[:3, 3]
    if mask_dynamic:
        keep = np.ones(len(points), dtype=bool)
        labels = frame["box_labels"] if "box_labels" in frame else ["car"] * len(frame["boxes_T_dataset_object"])
        for transform, dimensions, label in zip(frame["boxes_T_dataset_object"], frame["boxes_dimensions_lwh"], labels):
            # CooperScene's vehicle boxes use "car"; nuScenes contains other
            # annotation classes too, including static barriers and cones.
            if not (label == "car" or label == "animal" or str(label).startswith(("vehicle.", "human.pedestrian."))):
                continue
            local = (points - transform[:3, 3]) @ transform[:3, :3]
            keep &= ~np.all(np.abs(local) <= dimensions / 2 + 0.2, axis=1)
        points = points[keep]
    return points


def map_proxy(map_path: str | Path, *, min_opacity: float = 0.0, stride: int = 1) -> np.ndarray:
    gaussian_map = load_gaussian_map(map_path)
    means, opacities = gaussian_map.means, gaussian_map.opacities
    keep = np.isfinite(means).all(axis=1) & np.isfinite(opacities) & (opacities > min_opacity)
    keep &= np.abs(means).max(axis=1) <= 10000
    return means[keep][::stride].astype(np.float64)


def registration_config(dataset: str) -> RegistrationConfig:
    if dataset == "cooperscene":
        return RegistrationConfig(
            coarse_voxel_size=0.25, refinement_voxel_sizes=(0.25, 0.10),
            icp_distance_factor=2.0, min_inlier_ratio=0.35,
            max_median_residual=0.20, max_p95_residual=0.60,
            max_all_point_median_residual=0.25, max_all_point_p95_residual=0.75,
            max_correction_translation=1.0, max_correction_rotation_degrees=3.0,
        )
    return RegistrationConfig()


def pose_error(transform: np.ndarray, reference: np.ndarray, dataset_T_lidar: np.ndarray) -> dict:
    estimated_pose, reference_pose = transform @ dataset_T_lidar, reference @ dataset_T_lidar
    axes = reference[:3, :3]
    delta = axes.T @ (estimated_pose[:3, 3] - reference_pose[:3, 3])
    delta_rotation = axes.T @ (estimated_pose[:3, :3] @ reference_pose[:3, :3].T) @ axes
    rpy = Rotation.from_matrix(delta_rotation).as_euler("xyz", degrees=True)
    return dict(dx_m=float(delta[0]), dy_m=float(delta[1]), dz_m=float(delta[2]),
                xy_m=float(np.linalg.norm(delta[:2])), xyz_m=float(np.linalg.norm(delta)),
                roll_deg=float(rpy[0]), pitch_deg=float(rpy[1]), yaw_deg=float(rpy[2]),
                orientation_deg=float(np.degrees(Rotation.from_matrix(delta_rotation).magnitude())),
                error_axes="dataset_z_up", independent_ground_truth=False,
                estimated_map_T_lidar=estimated_pose.tolist(), reference_map_T_lidar=reference_pose.tolist())


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


class FirstFrameXYZInvestigation:
    def __init__(self, inputs: Path, output: Path):
        self.inputs, self.output = inputs, output
        self._map_path = None
        self._target = None
        self._coarse_target = None
        self._opacity_target = None
        fusion_module.cKDTree = LimitedKDTree

    def _map(self, path):
        if path != self._map_path:
            self._target = None
            self._target = map_proxy(path)
            self._coarse_target = None
            self._opacity_target = None
            self._map_path = path
        return self._target

    def _write(self, path: Path, record: dict):
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(jsonable(record), indent=2, allow_nan=False) + "\n")
        temporary.replace(path)

    def run_one(self, metadata: dict, variant: str) -> dict:
        run_id = metadata["run_id"]
        frame = np.load(self.inputs / run_id / "frame.npz")
        dataset = metadata["dataset"]
        masked = dataset == "cooperscene" or variant != "baseline"
        source = source_points(frame, mask_dynamic=masked)
        target = self._map(metadata["map_path"])
        original_target_count = len(target)
        if variant == "static_opacity05":
            if self._opacity_target is None:
                self._opacity_target = map_proxy(metadata["map_path"], min_opacity=0.05)
            target = self._opacity_target
        initial = frame["pipeline_initial_map_T_dataset"]
        reference = frame["reference_map_T_dataset"]
        config = registration_config(dataset)
        registrar = PointCloudRegistrar(config)
        record = dict(run_id=run_id, dataset=dataset, scene=metadata["scene"],
                      agent=metadata.get("agent"), frame_id=metadata["frame_id"], variant=variant,
                      first_frame_only=True, reference_kind=metadata.get("reference_kind"),
                      inherited_fit_frames=metadata.get("inherited_fit_frames"),
                      dynamic_mask_applied=masked, dynamic_box_padding_m=0.2,
                      dynamic_mask_classes=["car (CooperScene)", "vehicle.*", "human.pedestrian.*", "animal"],
                      input_points=len(frame["points_lidar"]), source_points=len(source), target_points=len(target),
                      min_map_opacity=0.05 if variant == "static_opacity05" else 0.0,
                      map_points_removed_by_opacity=original_target_count - len(target),
                      reference_description=metadata.get("reference_description", "comparison reference; not independent ground truth"),
                      registration_config=asdict(config),
                      initial_map_T_dataset=initial, reference_map_T_dataset=reference,
                      inherited_error=pose_error(frame["inherited_map_T_dataset"], reference, frame["dataset_T_lidar"]),
                      initial_error=pose_error(initial, reference, frame["dataset_T_lidar"]))
        started = time.monotonic()
        if variant == "baseline":
            record["inherited_registration"] = asdict(registrar.evaluate(
                source, target, frame["inherited_map_T_dataset"], initial))
            record["reference_registration"] = asdict(registrar.evaluate(source, target, reference, initial))
        fit_config = config
        if variant == "static_fine":
            fit_config = replace(config, refinement_voxel_sizes=(0.5, 0.25, 0.10), icp_max_iterations=100)
        record["fit_config"] = asdict(fit_config)
        try:
            if variant == "static_global":
                if self._coarse_target is None:
                    self._coarse_target = map_proxy(metadata["map_path"], min_opacity=0.05, stride=8)
                coarse_config = RegistrationConfig(
                    coarse_voxel_size=1.0, refinement_voxel_sizes=(0.5, 0.25),
                    min_inlier_ratio=0.05, max_median_residual=1.0, max_p95_residual=3.0,
                    max_correction_translation=float("inf"), max_correction_rotation_degrees=180.0)
                coarse = PointCloudRegistrar(coarse_config).register(source[::8], self._coarse_target)
                record["coarse_registration"] = asdict(coarse)
                record["coarse_config"] = asdict(coarse_config)
                record["initialization_method"] = "existing_single_agent_global_fpfh_ransac_then_strict_refinement"
                initial = coarse.map_T_source
            record["refinement_initial_map_T_dataset"] = initial
            scan = PointCloudRegistrar(fit_config).register(source, target, initial)
            # Always assess candidates with the existing registrar and original initializer.
            if variant == "static_fine":
                scan = registrar.evaluate(source, target, scan.map_T_source, initial)
            record["scan_registration"] = asdict(scan)
            record["scan_error"] = pose_error(scan.map_T_source, reference, frame["dataset_T_lidar"])
            final = scan
            fusion = None
            if dataset == "cooperscene" and scan.accepted:
                fusion_config = EnuPriorFusionConfig(position_mode="xyz")
                fusion = EnuPriorFuser(fusion_config).fuse(source, target, scan.map_T_source,
                                                        reference, frame["dataset_T_lidar"])
                record["fusion_config"] = asdict(fusion_config)
                record["fusion"] = asdict(fusion)
                final = registrar.evaluate(source, target, fusion.map_T_source, initial)
            record["final_registration"] = asdict(final)
            record["map_T_dataset"] = final.map_T_source
            record["fresh_error"] = pose_error(final.map_T_source, reference, frame["dataset_T_lidar"])
            record["accepted"] = bool(final.accepted and (fusion is None or fusion.converged))
            record["rejection_reasons"] = list(final.rejection_reasons)
            if fusion is not None and not fusion.converged:
                record["rejection_reasons"].append("fusion_not_converged")
            record["fusion_skipped_scan_rejected"] = dataset == "cooperscene" and not scan.accepted
        except Exception as error:
            record["exception"] = f"{type(error).__name__}: {error}"
            record["accepted"] = False
        record["runtime_seconds"] = time.monotonic() - started
        self._write(self.output / variant / f"{run_id}.json", record)
        print(run_id, variant, "accepted", record["accepted"], "error", record.get("fresh_error", {}).get("xyz_m"),
              "reasons", record.get("rejection_reasons", record.get("exception")), flush=True)
        return record

    def run(self, run_ids: list[str] | None, variants: list[str]):
        manifest = json.loads((self.inputs / "manifest.json").read_text())
        entries = manifest if isinstance(manifest, list) else manifest.get("runs", manifest.get("entries"))
        if isinstance(entries, dict):
            entries = [dict(value, run_id=key) for key, value in entries.items()]
        for item in entries:
            if run_ids and item["run_id"] not in run_ids:
                continue
            for variant in variants:
                if item["dataset"] == "cooperscene" and variant == "static":
                    continue  # Baseline already applies the same annotated-box mask.
                path = self.output / variant / f"{item['run_id']}.json"
                if not path.exists():
                    self.run_one(item, variant)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, default=Path("artifacts/first-frame-localization/inputs"))
    parser.add_argument("--output", type=Path, default=Path("artifacts/first-frame-localization/xyz"))
    parser.add_argument("--run-id", action="append")
    parser.add_argument("--variant", choices=("baseline", "static", "static_fine", "static_global", "static_opacity05"), action="append")
    args = parser.parse_args()
    FirstFrameXYZInvestigation(args.inputs, args.output).run(args.run_id, args.variant or ["baseline", "static", "static_fine"])

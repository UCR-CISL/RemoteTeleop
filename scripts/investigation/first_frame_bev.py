"""Offline BEV comparison: existing Cooper fusion and experimental nuScenes ICP.

Only cached first frames are consumed. nuScenes uses 3D point-to-plane matches,
Tukey weights and the pipeline's original scales/gates; updates are constrained
to dataset-horizontal translation and yaw. Its vertical/roll/pitch constraints
are imposed initializer assumptions, not independently measured accuracy.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, replace
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from src.localization.registration import PointCloudRegistrar, _estimate_normals
from src.localization.enu_prior_fusion import EnuPriorFuser, EnuPriorFusionConfig
import src.localization.enu_prior_fusion as fusion_module


class TwoWorkerTree(cKDTree):
    """Keep the existing fuser's nearest-neighbor queries within the CPU budget."""

    def query(self, *args, **kwargs):
        kwargs["workers"] = 2
        return super().query(*args, **kwargs)


class GravityConstrainedRegistrar(PointCloudRegistrar):
    """Research comparison retaining the initializer's gravity and vertical pose."""

    def __init__(self, config, dataset_T_lidar):
        super().__init__(config)
        self.dataset_T_lidar = dataset_T_lidar
        self.level_diagnostics = []

    def _refine(self, o3d, source_cloud, target_cloud, initial):
        transform = initial.copy()
        basis = initial[:3, :3]
        axis = basis[:, 2]
        for voxel_size in self.config.refinement_voxel_sizes:
            source = np.asarray(source_cloud.voxel_down_sample(voxel_size).points)
            target_cloud_level = target_cloud.voxel_down_sample(voxel_size)
            _estimate_normals(o3d, target_cloud_level, voxel_size, self.config.normal_radius_factor)
            target = np.asarray(target_cloud_level.points)
            normals = np.asarray(target_cloud_level.normals)
            tree = cKDTree(target)
            threshold = self.config.icp_distance_factor * voxel_size
            kernel = self.config.robust_kernel_scale_factor * threshold
            previous_fitness = previous_rmse = None
            for iteration in range(self.config.icp_max_iterations):
                points = source @ transform[:3, :3].T + transform[:3, 3]
                distances, indices = tree.query(points, workers=2)
                keep = distances <= threshold
                if np.count_nonzero(keep) < 3:
                    raise RuntimeError("constrained ICP has fewer than three correspondences")
                points, neighbors, n = points[keep], target[indices[keep]], normals[indices[keep]]
                residual = np.einsum("ij,ij->i", points - neighbors, n)
                weights = np.maximum(0, 1 - (residual / kernel) ** 2) ** 2
                weights[np.abs(residual) >= kernel] = 0
                anchor = (transform @ self.dataset_T_lidar)[:3, 3]
                yaw_derivative = np.cross(axis, points - anchor)
                jacobian = np.column_stack((n @ basis[:, :2], np.einsum("ij,ij->i", n, yaw_derivative)))
                weighted = jacobian * np.sqrt(weights[:, None])
                step, _, rank, _ = np.linalg.lstsq(weighted, -residual * np.sqrt(weights), rcond=None)
                if rank < 3:
                    raise RuntimeError("constrained ICP horizontal/yaw system is rank deficient")
                rotation = Rotation.from_rotvec(axis * step[2]).as_matrix()
                transform[:3, 3] = rotation @ (transform[:3, 3] - anchor) + anchor + basis[:, :2] @ step[:2]
                transform[:3, :3] = rotation @ transform[:3, :3]
                fitness = float(np.mean(keep))
                rmse = float(np.sqrt(np.mean(distances[keep] ** 2)))
                # Open3D's standard relative fitness/RMSE stopping thresholds.
                if previous_fitness is not None and abs(fitness - previous_fitness) < 1e-6 and abs(rmse - previous_rmse) < 1e-6:
                    break
                previous_fitness, previous_rmse = fitness, rmse
            self.level_diagnostics.append(dict(voxel_size_m=voxel_size, iterations=iteration + 1, fitness=fitness, rmse_m=rmse))
        return transform


def json_safe(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, default=ROOT / "artifacts/first-frame-localization/inputs")
    parser.add_argument("--xyz-dir", type=Path, default=ROOT / "artifacts/first-frame-localization/xyz")
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/first-frame-localization/bev")
    parser.add_argument("--run-id", action="append")
    parser.add_argument("--variant", choices=("baseline", "static", "static_fine", "static_global"), action="append")
    parser.add_argument("--refresh-plots", action="store_true")
    args = parser.parse_args()
    runner = FirstFrameBevComparison(args)
    fusion_module.cKDTree = TwoWorkerTree
    runner.run()


class FirstFrameBevComparison:
    def __init__(self, args):
        self.args = args
        self.maps = {}

    def run(self):
        from scripts.investigation.first_frame_xyz import map_proxy, source_points, registration_config, pose_error
        self.args.output.mkdir(parents=True, exist_ok=True)
        rows = []
        for directory in sorted(path for path in self.args.inputs.iterdir() if path.is_dir()):
            run_id = directory.name
            if self.args.run_id and run_id not in self.args.run_id:
                continue
            manifest = json.loads((directory / "metadata.json").read_text())
            dataset = manifest["dataset"]
            if self.args.variant == ["static_global"] and dataset != "cooperscene":
                continue
            frame = dict(np.load(directory / "frame.npz", allow_pickle=False))
            map_path = manifest["map_path"]
            if map_path not in self.maps:
                self.maps.clear()  # Each scene has its own large map.
                self.maps[map_path] = map_proxy(map_path)
            target = self.maps[map_path]
            initial = frame["pipeline_initial_map_T_dataset"]
            variants = (("baseline", True), ("static_fine", True), ("static_global", True)) if dataset == "cooperscene" else (("baseline", False), ("static", True), ("static_fine", True))
            for variant, mask in variants:
                if self.args.variant and variant not in self.args.variant:
                    continue
                path = self.args.output / f"{run_id}_{variant}.json"
                if path.exists():
                    existing = json.loads(path.read_text())
                    rows.append(existing)
                    if self.args.refresh_plots and "registration" in existing:
                        self._plot(source_points(frame, mask_dynamic=mask), target,
                                   np.asarray(existing["registration"]["map_T_source"]), frame,
                                   path.with_suffix(".png"), run_id, variant, existing["accepted"])
                    continue
                if dataset == "cooperscene" and not (self.args.xyz_dir / variant / f"{run_id}.json").exists():
                    print(f"{run_id}: awaiting fresh XYZ scan cache", flush=True)
                    continue
                source = source_points(frame, mask_dynamic=mask)
                result = dict(run_id=run_id, dataset=dataset, variant=variant,
                              frame_id=manifest["frame_id"], source_count=len(source), dynamic_mask_applied=mask,
                              reference_kind="initial pipeline reference; not independent ground truth")
                registrar = PointCloudRegistrar(registration_config(dataset))
                result["registration_config"] = asdict(registrar.config)
                inherited = registrar.evaluate(source, target, frame["inherited_map_T_dataset"], initial)
                result["inherited_registration"] = asdict(inherited)
                result["inherited_error"] = pose_error(frame["inherited_map_T_dataset"], frame["reference_map_T_dataset"], frame["dataset_T_lidar"])
                try:
                    if dataset.lower().startswith("cooper"):
                        scan_data = json.loads((self.args.xyz_dir / variant / f"{run_id}.json").read_text())
                        result["scan_registration"] = scan_data.get("scan_registration")
                        result["scan_error"] = scan_data.get("scan_error")
                        result.update(self._cooper(run_id, variant, frame, source, target, registrar))
                    else:
                        fit_config = replace(registrar.config, refinement_voxel_sizes=(0.5, 0.25, 0.10), icp_max_iterations=100) if variant == "static_fine" else registrar.config
                        constrained = GravityConstrainedRegistrar(fit_config, frame["dataset_T_lidar"])
                        registration = constrained.register(source, target, initial)
                        if variant == "static_fine":
                            registration = registrar.evaluate(source, target, registration.map_T_source, initial)
                        result.update(method="experimental gravity-constrained point-to-plane ICP; XYZ associations", registration=asdict(registration),
                                      fit_config=asdict(fit_config),
                                      level_diagnostics=constrained.level_diagnostics,
                                      constraints="Initial lidar vertical position and gravity/roll/pitch imposed; no ENU reference penalty")
                    transform = np.asarray(result["registration"]["map_T_source"])
                    result["error"] = pose_error(transform, frame["reference_map_T_dataset"], frame["dataset_T_lidar"])
                    result["accepted"] = result["registration"]["accepted"] and result.get("upstream_scan_accepted", True) and result.get("fusion_converged", True)
                    self._plot(source, target, transform, frame, self.args.output / f"{run_id}_{variant}.png", run_id, variant, result["accepted"])
                except Exception as error:
                    result.update(accepted=False, refusal=str(error))
                path.write_text(json.dumps(result, indent=2, default=json_safe) + "\n")
                rows.append(result)
                print(json.dumps(dict(run_id=run_id, variant=variant, accepted=result["accepted"], xyz_m=result.get("error", {}).get("xyz_m"), refusal=result.get("refusal")), default=json_safe), flush=True)
        rows = [json.loads(path.read_text()) for path in sorted(self.args.output.glob("*_*.json"))]
        (self.args.output / "results.json").write_text(json.dumps(rows, indent=2, default=json_safe) + "\n")
        self._summarize(rows)

    def _summarize(self, rows):
        fields = ("run_id", "dataset", "variant", "accepted", "dynamic_mask_applied", "dx_m", "dy_m", "dz_m", "xy_m", "xyz_m", "roll_deg", "pitch_deg", "yaw_deg", "inlier_ratio", "median_residual", "p95_residual", "all_point_median_residual", "all_point_p95_residual", "rejection_reasons")
        with (self.args.output / "summary.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for row in rows:
                merged = {**row, **row.get("error", {}), **row.get("registration", {})}
                merged["accepted"] = row["accepted"]
                reasons = list(row.get("registration", {}).get("rejection_reasons", []))
                if not row.get("upstream_scan_accepted", True):
                    reasons.append("upstream_scan_rejected")
                if not row.get("fusion_converged", True):
                    reasons.append("fusion_not_converged")
                if row.get("refusal"):
                    reasons.append(row["refusal"])
                merged["rejection_reasons"] = ";".join(reasons)
                writer.writerow({key: merged.get(key) for key in fields})
        counts = []
        for dataset, variant in sorted(set((row["dataset"], row["variant"]) for row in rows)):
            group = [row for row in rows if row["dataset"] == dataset and row["variant"] == variant]
            accepted = [row for row in group if row["accepted"]]
            counts.append(dict(dataset=dataset, variant=variant, attempted=len(group), accepted=len(accepted),
                               accepted_xyz_under_0_1m=sum(row["error"]["xyz_m"] < .1 for row in accepted),
                               accepted_xy_under_0_1m=sum(row["error"]["xy_m"] < .1 for row in accepted)))
        (self.args.output / "counts.json").write_text(json.dumps(counts, indent=2) + "\n")

    def _cooper(self, run_id, variant, frame, source, target, registrar):
        path = self.args.xyz_dir / variant / f"{run_id}.json"
        data = json.loads(path.read_text())
        scan = data["scan_registration"]
        fusion = EnuPriorFuser(EnuPriorFusionConfig(position_mode="bev")).fuse(
            source, target, np.asarray(scan["map_T_source"]), frame["reference_map_T_dataset"], frame["dataset_T_lidar"])
        initial = np.asarray(data.get("refinement_initial_map_T_dataset", frame["pipeline_initial_map_T_dataset"]))
        registration = registrar.evaluate(source, target, fusion.map_T_source, initial)
        return dict(method="existing CooperScene BEV ENU fusion (weight 0.14)",
                    registration=asdict(registration), fusion=asdict(fusion),
                    scan_registration=scan, upstream_scan_accepted=scan["accepted"],
                    fusion_converged=fusion.converged,
                    refinement_initial_map_T_dataset=initial,
                    diagnostic_only=not scan["accepted"],
                    constraints="Scan transform Z/roll/pitch fixed; 3D nearest neighbors; XY residual/prior and yaw optimized")

    @staticmethod
    def _plot(source, target, transform, frame, path, run_id, variant, accepted):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        # Display in reference dataset axes so the camera-anchored nuScenes map
        # has an actual top view, and show vertical alignment separately.
        dataset_T_map = np.linalg.inv(frame["reference_map_T_dataset"])
        pose = frame["dataset_T_lidar"][:3, 3]
        scan_map = source @ transform[:3, :3].T + transform[:3, 3]
        scan = scan_map @ dataset_T_map[:3, :3].T + dataset_T_map[:3, 3] - pose
        reference = source - pose
        mapped = target @ dataset_T_map[:3, :3].T + dataset_T_map[:3, 3] - pose
        keep = (np.linalg.norm(mapped[:, :2], axis=1) < 60) & (np.abs(mapped[:, 2]) < 12)
        mapped = mapped[keep][::max(1, int(np.count_nonzero(keep) / 70000))]
        scan = scan[::max(1, len(scan) // 15000)]
        reference = reference[::max(1, len(reference) // 15000)]
        figure, axes = plt.subplots(1, 2, figsize=(12, 5))
        for ax, dims in zip(axes, ((0, 1), (0, 2))):
            ax.scatter(mapped[:, dims[0]], mapped[:, dims[1]], s=.2, color="0.6", alpha=.5, label="Gaussian centers")
            ax.scatter(reference[:, dims[0]], reference[:, dims[1]], s=.3, color="#2aa34a", alpha=.4, label="Reference pose LiDAR")
            ax.scatter(scan[:, dims[0]], scan[:, dims[1]], s=.3, color="#d83e25", alpha=.5, label="Fresh BEV LiDAR")
            ax.set(xlabel="Dataset X from first LiDAR (m)", ylabel=f"Dataset {'Y' if dims[1] == 1 else 'Z'} (m)", xlim=(-55,55), ylim=(-55,55) if dims[1] == 1 else (-8,12))
            ax.set_aspect("equal")
            ax.grid(alpha=.2)
        axes[0].legend(markerscale=8, fontsize=7)
        status = "ACCEPTED" if accepted else "REJECTED — diagnostic only"
        figure.suptitle(f"{run_id} — {variant}; {status}\nBEV first-frame fit (reference is not independent GT)")
        figure.tight_layout()
        figure.savefig(path, dpi=140)
        plt.close(figure)


if __name__ == "__main__":
    main()

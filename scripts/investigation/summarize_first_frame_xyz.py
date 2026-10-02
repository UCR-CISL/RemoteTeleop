"""Select first-frame XYZ fits without using reference error; render cloud diagnostics."""
from __future__ import annotations

import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.investigation.first_frame_xyz import (
    FirstFrameXYZInvestigation, PointCloudRegistrar, asdict, jsonable,
    registration_config, source_points,
)


class FirstFrameXYZSummary(FirstFrameXYZInvestigation):
    """Rank accepted fits by common static-cloud median, then p95 residual."""

    def summarize(self):
        rows = []
        for metadata in json.loads((self.inputs / "manifest.json").read_text()):
            run_id = metadata["run_id"]
            frame = np.load(self.inputs / run_id / "frame.npz")
            source = source_points(frame, mask_dynamic=True)
            target = self._map(metadata["map_path"])
            registrar = PointCloudRegistrar(registration_config(metadata["dataset"]))
            candidates = []
            for variant in ("baseline", "static", "static_fine", "static_global", "static_opacity05"):
                path = self.output / variant / f"{run_id}.json"
                if not path.exists():
                    continue
                candidate = json.loads(path.read_text())
                if "map_T_dataset" not in candidate:
                    continue
                if (metadata["dataset"] == "nuscenes" and variant == "baseline") or variant == "static_opacity05":
                    candidate["selection_registration"] = asdict(registrar.evaluate(
                        source, target, candidate["map_T_dataset"], candidate["initial_map_T_dataset"]))
                else:
                    candidate["selection_registration"] = candidate["final_registration"]
                candidates.append(candidate)
            if not candidates:
                continue
            accepted = [item for item in candidates if item["accepted"] and item["selection_registration"]["accepted"]]
            best = min(accepted or candidates, key=lambda item: (
                item["selection_registration"]["all_point_median_residual"],
                item["selection_registration"]["all_point_p95_residual"]))
            best["selection_policy"] = "accepted candidates first; common static-cloud all-point median then p95; never reference error"
            best["first_frame_only"] = True
            best["reference_kind"] = metadata.get("reference_kind")
            best["inherited_fit_frames"] = metadata.get("inherited_fit_frames")
            best["masking_limitation"] = ("Potentially dynamic annotation classes are removed, including parked vehicles; "
                                          "remaining scan points and Gaussian centers are not proven static.")
            best["accepted"] = bool(accepted)
            best["selection_is_rejected_diagnostic"] = not bool(accepted)
            best["available_variants"] = [item["variant"] for item in candidates]
            reference = frame["reference_map_T_dataset"]
            center = frame["dataset_T_lidar"][:3, 3]
            dataset_target = (target - reference[:3, 3]) @ reference[:3, :3]
            bounds = np.percentile(dataset_target, (1, 99), axis=0)
            sensor_outside = np.maximum(bounds[0] - center, 0) + np.maximum(center - bounds[1], 0)
            best["coverage"] = dict(
                robust_map_bounds_dataset_xyz_m=bounds.tolist(),
                dataset_lidar_xyz_m=center.tolist(),
                sensor_xy_distance_outside_robust_bounds_m=float(np.linalg.norm(sensor_outside[:2])),
                robust_bounds_percentiles=[1, 99],
                reference_nearest_map_center_to_sensor_m=float(cKDTree(dataset_target).query(center)[0]),
                no_points_cropped_for_registration=True,
            )
            for label, transform in (("reference", reference), ("inherited", frame["inherited_map_T_dataset"]),
                                     ("fresh", np.asarray(best["map_T_dataset"]))):
                mapped = source @ transform[:3, :3].T + transform[:3, 3]
                mapped_dataset = (mapped - reference[:3, 3]) @ reference[:3, :3]
                in_bounds = (mapped_dataset >= bounds[0]) & (mapped_dataset <= bounds[1])
                best["coverage"][f"{label}_source_fraction_in_robust_map_xyz_bounds"] = float(in_bounds.all(axis=1).mean())
                best["coverage"][f"{label}_source_fraction_in_robust_map_xy_bounds"] = float(in_bounds[:, :2].all(axis=1).mean())
            self._write(self.output / "best" / f"{run_id}.json", best)
            self._plot(metadata, frame, source, dataset_target, best)
            row = dict(run_id=run_id, variant=best["variant"], accepted=best["accepted"],
                       inherited_error=best["inherited_error"], fresh_error=best["fresh_error"],
                       final_registration=best["final_registration"], coverage=best["coverage"])
            rows.append(row)
            print(run_id, best["variant"], best["accepted"], best["fresh_error"]["xyz_m"], flush=True)
        self._write(self.output / "summary.json", dict(
            selection_policy="accepted first; common static-cloud all-point median, then p95; no reference error ranking",
            independent_ground_truth=False, first_frame_only=True, results=rows))

    def _plot(self, metadata, frame, source, dataset_target, best):
        reference = frame["reference_map_T_dataset"]
        center = frame["dataset_T_lidar"][:3, 3]
        target = dataset_target - center
        # Display crop only. Every fit and gate above used the complete static scan.
        target = target[(np.abs(target[:, :2]) <= 55).all(axis=1) & (np.abs(target[:, 2]) <= 20)]
        target = target[::max(1, len(target) // 100000)]
        figure, axes = plt.subplots(2, 2, figsize=(12, 9))
        for column, (label, transform) in enumerate((
            ("Inherited", frame["inherited_map_T_dataset"]),
            (f"Fresh {best['variant']} ({'accepted' if best['accepted'] else 'REJECTED diagnostic'})", np.asarray(best["map_T_dataset"]))
        )):
            mapped = source @ transform[:3, :3].T + transform[:3, 3]
            points = (mapped - reference[:3, 3]) @ reference[:3, :3] - center
            points = points[::max(1, len(points) // 30000)]
            for row, dimensions in enumerate(((0, 1), (0, 2))):
                ax = axes[row, column]
                ax.scatter(target[:, dimensions[0]], target[:, dimensions[1]], s=.25, c="#777777", alpha=.35, rasterized=True)
                ax.scatter(points[:, dimensions[0]], points[:, dimensions[1]], s=.35, c="#d34f35", alpha=.5, rasterized=True)
                ax.plot(0, 0, "+", color="#2266dd", markersize=10)
                ax.set(xlim=(-50, 50), ylim=(-50, 50) if row == 0 else (-10, 15),
                       xlabel="Dataset X relative to reference LiDAR (m)", ylabel="Dataset " + ("Y" if row == 0 else "Z") + " (m)",
                       title=label if row == 0 else "Side view")
                ax.grid(alpha=.15)
                ax.set_aspect("equal")
        figure.suptitle(f"{metadata['run_id']} first frame {metadata['frame_id']} | gray: Gaussian centers; orange: annotation-masked scan\n"
                       f"Reference-relative XYZ={best['fresh_error']['xyz_m']:.3f} m (not independent ground truth); display crop only")
        figure.tight_layout()
        output = self.output / "cloud_views" / f"{metadata['run_id']}.png"
        output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(output, dpi=130)
        plt.close(figure)


if __name__ == "__main__":
    root = Path("artifacts/first-frame-localization")
    FirstFrameXYZSummary(root / "inputs", root / "xyz").summarize()

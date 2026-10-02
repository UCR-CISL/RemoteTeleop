#!/usr/bin/env python3
"""Fit auditable rendering-only road planes in each scene's actual map axes."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from src.localization.gaussian_map import load_gaussian_map
from src.localization.ground_plane import GroundPlane


def fit_scene(inputs: Path, run_id: str, output: Path) -> dict:
    metadata = json.loads((inputs / run_id / "metadata.json").read_text())
    frame = np.load(inputs / run_id / "frame.npz")
    transform = frame["reference_map_T_dataset"]
    lidar = frame["dataset_T_lidar"]
    center = (transform @ lidar)[:3, 3]
    basis = transform[:3, :3]
    gaussian_map = load_gaussian_map(metadata["map_path"])
    points = gaussian_map.means
    keep = np.isfinite(points).all(axis=1) & (gaussian_map.opacities > 0)
    local = (points - center) @ basis
    nearby = keep & (np.linalg.norm(local[:, :2], axis=1) < 25)
    candidates = nearby & (local[:, 2] >= -3.0) & (local[:, 2] <= -1.0)
    # Annotation boxes are a conservative exclusion for this offline map audit.
    for box, dimensions in zip(frame["boxes_T_dataset_object"], frame["boxes_dimensions_lwh"]):
        indices = np.flatnonzero(candidates)
        map_T_box = transform @ box
        box_points = (points[indices] - map_T_box[:3, 3]) @ map_T_box[:3, :3]
        candidates[indices[np.all(np.abs(box_points) <= dimensions / 2 + .25, axis=1)]] = False
    indices = np.flatnonzero(candidates)
    scales = gaussian_map.scales[indices]
    rotations = Rotation.from_quat(gaussian_map.quats[indices][:, [1, 2, 3, 0]]).as_matrix()
    normals = rotations[np.arange(len(indices)), :, np.argmin(scales, axis=1)]
    surface = (scales.min(axis=1) < .02) & (np.abs(normals @ basis[:, 2]) > .98)
    selected = points[indices[surface]].astype(np.float64)
    # Equalize spatial support instead of weighting opaque/dense map patches more.
    _, unique = np.unique(np.floor(selected / .10).astype(np.int64), axis=0, return_index=True)
    selected = selected[unique]
    if len(selected) > 20000:
        selected = selected[np.random.default_rng(0).choice(len(selected), 20000, replace=False)]
    record = dict(run_id=run_id, map_path=metadata["map_path"],
                  reference_input=str(inputs / run_id / "frame.npz"),
                  coordinate_frame="native_gsplat_map", purpose="rendering_only",
                  plane_fit_gates=dict(min_inlier_ratio=.25, min_inliers=100, distance_threshold_m=.10,
                                       min_xy_span_m=8, max_tilt_deg=15),
                  candidate_region=dict(radius_m=25, relative_height_band_m=[-3, -1],
                                        annotation_padding_m=.25, voxel_m=.10,
                                        splat_min_scale_max_m=.02, splat_normal_alignment_min=.98),
                  reference_lidar_map_position=center.tolist(), up_hint=basis[:, 2].tolist(),
                  warning="A local road-plane approximation; fit quality is not semantic or global ground-truth validation.")
    try:
        plane, stats = GroundPlane.fit(selected, up_hint=basis[:, 2], min_inlier_ratio=.25, iterations=384)
        stats["reference_lidar_clearance_m"] = float(np.dot(plane.normal, center) + plane.offset_m)
        record.update(plane.to_dict(), fit=stats, status="fit_passed")
        support = selected[np.abs(selected @ plane.normal + plane.offset_m) <= .10]
        projected = (support - center) @ basis
        spans = np.ptp(projected[:, :2], axis=0)
        record["fit"]["support_span_xy_m"] = spans.tolist()
        if np.min(spans) < 8:
            raise ValueError("ground plane lacks eight metres support along both dataset axes")
    except ValueError as error:
        record.update(status="rejected", error=str(error))
    output.mkdir(parents=True, exist_ok=True)
    (output / f"{run_id}.json").write_text(json.dumps(record, indent=2) + "\n")
    display = local[nearby]
    display = display[::max(1, len(display) // 40000)]
    selected_local = (selected - center) @ basis
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    for ax, horizontal in zip(axes[:2], [0, 1]):
        ax.scatter(display[:, horizontal], display[:, 2], s=.2, c="lightgray", label="nearby map")
        ax.scatter(selected_local[:, horizontal], selected_local[:, 2], s=.4, c="tab:blue", label="road candidates")
        if record["status"] == "fit_passed":
            normal_local = basis.T @ np.asarray(plane.normal)
            x = np.linspace(-25, 25)
            z = -(normal_local[horizontal] * x + np.dot(plane.normal, center) + plane.offset_m) / normal_local[2]
            ax.plot(x, z, c="red", label="fitted plane, cross-section at 0")
        ax.set(xlabel=f"dataset {'XY'[horizontal]} relative to LiDAR (m)", ylabel="dataset up relative to LiDAR (m)", ylim=(-5, 5))
    axes[0].legend(fontsize=7)
    if record["status"] == "fit_passed":
        residual = selected @ plane.normal + plane.offset_m
        im = axes[2].scatter(selected_local[:, 0], selected_local[:, 1], c=residual, cmap="coolwarm", vmin=-.3, vmax=.3, s=2)
        fig.colorbar(im, ax=axes[2], label="signed plane distance (m)")
    axes[2].set(xlabel="dataset X relative to LiDAR (m)", ylabel="dataset Y relative to LiDAR (m)", aspect="equal")
    fig.suptitle(f"{run_id}: {record['status']} — actual GSplat geometry, reference-guided candidate region")
    fig.tight_layout()
    fig.savefig(output / f"{run_id}.png", dpi=140)
    plt.close(fig)
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, default=Path("artifacts/first-frame-localization/inputs"))
    parser.add_argument("--output", type=Path, default=Path("artifacts/bev-replay-ground-planes-final"))
    args = parser.parse_args()
    rows = []
    for run_id in [f"nuscenes-{scene}" for scene in (152, 164, 171, 200, 209, 359, 529, 916)] + ["cooperscene-1-1"]:
        row = fit_scene(args.inputs, run_id, args.output)
        rows.append(row)
        print(run_id, row["status"], row.get("fit", row.get("error")), flush=True)
    (args.output / "summary.json").write_text(json.dumps(rows, indent=2) + "\n")


if __name__ == "__main__":
    main()

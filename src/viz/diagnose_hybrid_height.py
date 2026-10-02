"""Render predeclared hybrid-height pilot frames without rerunning localization.

Images show conditional registration proposals, including rejected proposals.
Reference errors are labels only; the plotting basis uses the reported map up.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from scripts.investigation.first_frame_xyz import source_points
from src.localization.gaussian_map import load_gaussian_map
from src.viz.composited_camera import CudaGaussianBufferRenderer


class HybridHeightVisuals:
    """Keep each map resident while comparing identical fixed camera frames."""

    def __init__(self, output: Path, width=640):
        self.output = output
        self.width = width
        self.output.mkdir(parents=True, exist_ok=True)
        self.map_path = None
        self.gaussian = None
        self.renderer = None

    @staticmethod
    def label(image, title, subtitle=""):
        panel = cv2.copyMakeBorder(image, 58, 0, 0, 0, cv2.BORDER_CONSTANT, value=(245, 245, 245))
        cv2.putText(panel, title, (9, 23), cv2.FONT_HERSHEY_SIMPLEX, .50, (20, 20, 20), 1, cv2.LINE_AA)
        cv2.putText(panel, subtitle, (9, 46), cv2.FONT_HERSHEY_SIMPLEX, .44, (45, 45, 45), 1, cv2.LINE_AA)
        return panel

    def render(self, result_path):
        result = json.loads(result_path.read_text())
        metadata = result["metadata"]
        frame = dict(np.load(metadata["npz_path"], allow_pickle=False))
        if self.map_path != metadata["map_path"]:
            self.gaussian = load_gaussian_map(metadata["map_path"])
            self.renderer = CudaGaussianBufferRenderer(self.gaussian)
            self.map_path = metadata["map_path"]
        camera = cv2.imread(metadata["camera_path"])
        if camera is None:
            raise ValueError(f"cannot decode camera: {metadata['camera_path']}")
        intrinsic = frame["camera_intrinsic"].copy()
        distortion = frame.get("camera_distortion", np.array([]))
        if distortion.size and np.any(distortion):
            camera = cv2.undistort(camera, intrinsic, distortion)
        h, w = camera.shape[:2]
        size = (self.width, round(h * self.width / w))
        intrinsic[0] *= size[0] / w
        intrinsic[1] *= size[1] / h
        camera = cv2.resize(camera, size)
        methods = result["methods"]
        rendered = {}
        stem = result_path.stem
        for method in ("xyz", "bev", "hybrid_candidate", "hybrid"):
            buffers = self.renderer.render(np.asarray(methods[method]["map_T_camera"]), intrinsic, size)
            rendered[method] = np.clip(buffers.rgb[..., ::-1] * 255, 0, 255).astype(np.uint8)
            cv2.imwrite(str(self.output / f"{stem}-{method}-gsplat.png"), rendered[method])

        def subtitle(method):
            item = methods[method]
            return f"reference XY {item['error']['xy_m']:.3f}m; Z {item['error']['dz_m']:+.3f}m; reg gate {'PASS' if item['accepted'] else 'FAIL'}"

        applied = methods["hybrid"]["accepted"]
        candidate_title = "Height proposal: APPLIED" if applied else "Height proposal: NOT APPLIED"
        panels = [self.label(camera, f"{result['run_id']} | source frame {metadata['frame_id']}", "Recorded camera; no render-only object grounding"),
                  self.label(rendered["bev"], "Frozen BEV pose", subtitle("bev")),
                  self.label(rendered["hybrid_candidate"], candidate_title, subtitle("hybrid_candidate")),
                  self.label(cv2.addWeighted(camera, .5, rendered["xyz"], .5, 0), "XYZ: 50% camera / 50% GSplat", subtitle("xyz")),
                  self.label(cv2.addWeighted(camera, .5, rendered["bev"], .5, 0), "BEV: 50% camera / 50% GSplat", subtitle("bev")),
                  self.label(cv2.addWeighted(camera, .5, rendered["hybrid_candidate"], .5, 0), "Height proposal: 50% overlay", subtitle("hybrid_candidate"))]
        cv2.imwrite(str(self.output / f"{stem}-camera.png"), np.vstack((np.hstack(panels[:3]), np.hstack(panels[3:]))))
        self.cloud(result, frame, stem)

        # Check clearance transport with the *estimated* plane and full relative
        # extrinsic. No recorded/reference height enters this identity.
        diagnostics = result["road_height"]["diagnostics"]
        clearance_error = None
        if "lidar_plane_local" in diagnostics:
            basis = self.basis(result["map_up"])
            pose = np.asarray(methods["bev"]["map_T_lidar"])
            normal_local = np.asarray(diagnostics["lidar_plane_local"]["normal"])
            normal_sensor = pose[:3, :3].T @ basis @ normal_local
            relative = np.asarray(result["lidar_T_camera"])
            clearance = float(diagnostics["lidar_plane_local"]["offset_m"] + normal_sensor @ relative[:3, 3])
            clearance_error = abs(clearance - diagnostics["camera_normal_clearance_m"])
            if clearance_error > 1e-7:
                raise ValueError(f"camera/LiDAR clearance transport mismatch: {clearance_error}")
        audit = dict(result_path=str(result_path), raw_frame_sha256=result["raw_frame_sha256"],
                     render_backend="gsplat CUDA RGB+ED", map_path=self.map_path,
                     road_height_reason=result["road_height"]["reason"], hybrid_applied=applied,
                     hybrid_selection_reason=methods["hybrid"]["reason"],
                     camera_clearance_transport_abs_error_m=clearance_error,
                     visualization_basis="reported map-up tangent basis centered on BEV LiDAR; no reference pose used",
                     source_mask=result["static_mask"])
        (self.output / f"{stem}.json").write_text(json.dumps(audit, indent=2) + "\n")
        print(json.dumps(audit), flush=True)
        return audit

    @staticmethod
    def basis(map_up):
        up = np.asarray(map_up, dtype=float)
        up /= np.linalg.norm(up)
        tangent = np.eye(3)[np.argmin(np.abs(up))]
        tangent -= up * (tangent @ up)
        tangent /= np.linalg.norm(tangent)
        return np.column_stack((tangent, np.cross(up, tangent), up))

    def cloud(self, result, frame, stem):
        methods = result["methods"]
        center = np.asarray(methods["bev"]["map_T_lidar"])[:3, 3]
        basis = self.basis(result["map_up"])
        target = (self.gaussian.means - center) @ basis
        target = target[np.isfinite(target).all(axis=1) & (np.linalg.norm(target[:, :2], axis=1) < 20)]
        target = target[::max(1, len(target) // 50000)]
        static = source_points(frame, mask_dynamic=True)
        static = static[::max(1, len(static) // 30000)]
        diag = result["road_height"]["diagnostics"]
        levels = [layer["height_at_lidar_xy_m"] for layer in diag.get("map_layers", [])]
        lidar_plane = diag.get("lidar_plane_local")
        shift = diag.get("height_shift_m", 0.)
        if lidar_plane:
            height = -lidar_plane["offset_m"] / lidar_plane["normal"][2]
            levels += [height, height + shift]
        limits = (min([-5.] + levels) - 1., max([3.] + levels) + 1.)
        fig, axes = plt.subplots(2, 2, figsize=(13, 8), layout="constrained", sharex=True, sharey=True)
        for col, method in enumerate(("bev", "hybrid_candidate")):
            transform = np.asarray(methods[method]["map_T_dataset"])
            scan = (static @ transform[:3, :3].T + transform[:3, 3] - center) @ basis
            scan = scan[np.linalg.norm(scan[:, :2], axis=1) < 20]
            for row, axis in enumerate(axes[:, col]):
                axis.scatter(target[:, row], target[:, 2], s=.4, color="gray", alpha=.35, label="Map centers")
                axis.scatter(scan[:, row], scan[:, 2], s=.6, color="#df6633", alpha=.65, label="Static LiDAR")
                x = np.linspace(-18, 18, 100)
                for layer in diag.get("map_layers", []):
                    if layer["plausible"]:
                        normal = np.asarray(layer["plane"]["normal"])
                        z = -(normal[row] * x + layer["plane"]["offset_m"]) / normal[2]
                        axis.plot(x, z, "--", color="#2474b5", lw=1, alpha=.8)
                if lidar_plane:
                    normal = np.asarray(lidar_plane["normal"])
                    z = -(normal[row] * x + lidar_plane["offset_m"]) / normal[2] + (shift if col else 0)
                    axis.plot(x, z, color="#bc3f13", lw=1.5, label="Fitted scan road")
                axis.set(xlim=(-18, 18), ylim=limits, xlabel=f"Map-up tangent {'XY'[row]} from BEV LiDAR (m)",
                         ylabel="Map up from BEV LiDAR (m)", title="Frozen BEV" if col == 0 else "Height proposal (diagnostic if rejected)")
                axis.grid(alpha=.2)
        axes[0, 0].legend(fontsize=8)
        fig.suptitle(f"{result['run_id']} offset {result['frame_offset']} | {result['road_height']['reason']}\nBlue dashed: plausible map layers; camera/scan poses remain unchanged when gates reject")
        fig.savefig(self.output / f"{stem}-cloud.png", dpi=130)
        plt.close(fig)


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, default=root / "artifacts/hybrid-road-height/pilot-v3/results")
    parser.add_argument("--output", type=Path, default=root / "artifacts/hybrid-road-height/pilot-v3/visuals")
    parser.add_argument("--case", action="append", help="Predeclared run_id:frame_offset")
    args = parser.parse_args()
    cases = args.case or ["nuscenes-152:36", "nuscenes-152:95", "nuscenes-152:190", "cooperscene-1-1:1", "cooperscene-1-3:0", "cooperscene-3-2:0", "cooperscene-4-3:85"]
    visualizer = HybridHeightVisuals(args.output)
    summaries = []
    for case in cases:
        run_id, index = case.rsplit(":", 1)
        summaries.append(visualizer.render(args.results / f"{run_id}-f{int(index):06d}.json"))
    (args.output / "summary.json").write_text(json.dumps(summaries, indent=2) + "\n")


if __name__ == "__main__":
    main()

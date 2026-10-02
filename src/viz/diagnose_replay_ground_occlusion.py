"""Render cached replay frames with preserved and corrected ground/composition.

Uses the production CUDA GSplat and PyTorch3D rasterizers, without replaying
detectors or localization. Input poses/boxes are read by VehicleAnalysisSpool.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

import cv2
import numpy as np

from src.common.vehicle_categories import vehicle_proxy_kind
from src.deployment.vehicle import VehicleAnalysisSpool
from src.localization.gaussian_map import load_gaussian_map
from src.localization.ground_plane import GroundPlane
from src.viz import composited_camera as current
from src.viz.primitive_vehicle import PrimitiveVehicleAsset
from src.viz.vehicle_proxy import VehicleProxyRenderer


class CachedProxyOverlays:
    """Reuse actual rasterized overlays across composition/order comparisons."""

    def __init__(self, renderer):
        self.renderer = renderer
        self.overlays = {}

    def render(self, kind, dimensions, object_pose, camera_pose, intrinsic, size):
        key = (kind, tuple(dimensions), np.asarray(object_pose).tobytes())
        if key not in self.overlays:
            self.overlays[key] = self.renderer.render(kind, dimensions, object_pose, camera_pose, intrinsic, size)
        return self.overlays[key]


class ReplayGroundOcclusionDiagnostics:
    def __init__(self, args):
        self.args = args
        self.root = Path(__file__).resolve().parents[2]
        args.output.mkdir(parents=True, exist_ok=True)
        source = subprocess.check_output(["git", "show", f"{args.before_revision}:src/viz/composited_camera.py"], cwd=self.root)
        snapshot = args.output / "before_composited_camera.py"
        snapshot.write_bytes(source)
        spec = importlib.util.spec_from_file_location("replay_before_compositor", snapshot)
        self.before = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = self.before
        spec.loader.exec_module(self.before)
        self.meshes = VehicleProxyRenderer(current.PyTorch3DObjectMeshRenderer(device="cuda"))

    @staticmethod
    def save(path, rgb):
        cv2.imwrite(str(path), np.clip(rgb[..., ::-1] * 255, 0, 255).astype(np.uint8))

    @staticmethod
    def label(rgb, text):
        panel = np.clip(rgb[..., ::-1] * 255, 0, 255).astype(np.uint8)
        panel = cv2.copyMakeBorder(panel, 42, 0, 0, 0, cv2.BORDER_CONSTANT, value=(245, 245, 245))
        cv2.putText(panel, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .62, (25, 25, 25), 1, cv2.LINE_AA)
        return panel

    def render(self, run_id, sequence):
        attempt = self.args.results / run_id / "attempt-001"
        spool_root = attempt / "lambda/analysis-spool"
        metadata_path = next(spool_root.glob(f"*/{sequence:020d}.json"))
        metadata = json.loads(metadata_path.read_text())
        frame = VehicleAnalysisSpool(spool_root).get(metadata["scene_generation"], sequence)
        image = cv2.imdecode(np.frombuffer(frame.image_jpeg, np.uint8), cv2.IMREAD_COLOR)[..., ::-1] / 255.
        old_plane_path = self.args.planes / ("cooperscene-1-1.json" if run_id.startswith("cooper") else f"{run_id}.json")
        plane_record = json.loads(old_plane_path.read_text())
        old_plane = GroundPlane.from_dict(plane_record)
        corrected_plane_path = self.args.corrected_planes / old_plane_path.name
        new_plane = GroundPlane.from_dict(json.loads(corrected_plane_path.read_text()))
        map_path = plane_record["map_path"]
        camera_pose = np.asarray(frame.world_T_camera).reshape(4, 4)
        ego_pose = np.asarray(frame.world_T_ego).reshape(4, 4)
        size = (self.args.width, round(image.shape[0] * self.args.width / image.shape[1]))
        camera = current.RemoteCameraConfig.from_source_calibration(
            ego_T_camera=np.linalg.inv(ego_pose) @ camera_pose,
            camera_intrinsic=np.asarray(frame.camera_intrinsic).reshape(3, 3),
            source_image_size=(image.shape[1], image.shape[0]), image_size=size)
        gaussian = current.CudaGaussianBufferRenderer(load_gaussian_map(map_path))
        background = gaussian.render(camera_pose, camera.camera_intrinsic, size)
        constant_background = SimpleNamespace(render=lambda *args: background)
        proxies = CachedProxyOverlays(self.meshes)
        pose = SimpleNamespace(sequence=sequence, timestamp_us=frame.timestamp_us, scene_generation=frame.scene_generation,
                               world_T_ego=frame.world_T_ego, world_T_camera=frame.world_T_camera)

        def composed(module, plane, boxes):
            backend = module.PoseOnlyCompositedCameraBackend(constant_background, camera,
                vehicle_proxies=proxies, ground_z=None, ground_plane=plane)
            return backend.render_pose(pose, boxes).rgb

        old = composed(self.before, old_plane, frame.boxes)
        ground_only = composed(self.before, new_plane, frame.boxes)
        fixed = composed(current, new_plane, frame.boxes)
        reversed_fixed = composed(current, new_plane, tuple(reversed(frame.boxes)))
        reversed_old = composed(self.before, new_plane, tuple(reversed(frame.boxes)))
        stem = f"{run_id}-f{frame.frame_id}"
        for name, rgb in (("before", old), ("ground-only", ground_only), ("after", fixed), ("background", background.rgb)):
            self.save(self.args.output / f"{stem}-{name}.png", rgb)
        panels = [self.label(cv2.resize(image, size), f"Recorded camera | {run_id} frame {frame.frame_id}"),
                  self.label(old, "Before: replay ground plane + original composition"),
                  self.label(ground_only, "Fixed ground only; original composition"),
                  self.label(fixed, "Fixed ground + shared vehicle depth")]
        cv2.imwrite(str(self.args.output / f"{stem}-comparison.png"), np.vstack((np.hstack(panels[:2]), np.hstack(panels[2:]))))
        original = next((attempt / "composited/frames").glob(f"{sequence:06d}_*.png"), None)
        if original:
            shutil.copy2(original, self.args.output / f"{stem}-original-replay.png")

        contacts, overlays = [], []
        for box in frame.boxes:
            kind = vehicle_proxy_kind(box.text_label)
            if kind is None:
                continue
            before_pose = old_plane.place_box(np.asarray(box.world_T_object).reshape(4, 4), box.dimensions_lwh)
            placed = new_plane.place_box(np.asarray(box.world_T_object).reshape(4, 4), box.dimensions_lwh)
            vertices = PrimitiveVehicleAsset(kind).vertices_for_dimensions(box.dimensions_lwh)
            world_vertices = vertices @ placed[:3, :3].T + placed[:3, 3]
            contacts.append(dict(track_id=box.track_id, old_center_z_m=float(before_pose[2, 3]),
                new_center_z_m=float(placed[2, 3]), center_z_change_m=float(placed[2, 3] - before_pose[2, 3]),
                minimum_mesh_plane_distance_m=float(np.min(world_vertices @ np.asarray(new_plane.normal) + new_plane.offset_m))))
            overlays.append(proxies.render(kind, box.dimensions_lwh, placed, camera_pose, camera.camera_intrinsic, size))
        gaussian_valid = np.isfinite(background.depth) & (background.depth > 0) & (background.alpha > .01)
        depths, colors = [], []
        for overlay in overlays:
            valid = ((overlay.alpha > .999) & np.isfinite(overlay.depth) & (overlay.depth > 0)
                     & (~gaussian_valid | (overlay.depth <= background.depth + .03)))
            depths.append(np.where(valid, overlay.depth, np.inf))
            colors.append(overlay.rgb)
        depth_stack = np.stack(depths)
        nearest_depth = depth_stack.min(axis=0)
        last_depth = np.full(nearest_depth.shape, np.inf)
        count = np.sum(np.isfinite(depth_stack), axis=0)
        for depth in depth_stack:
            last_depth = np.where(np.isfinite(depth), depth, last_depth)
        wrong = np.isfinite(nearest_depth) & (last_depth > nearest_depth + 1e-4)
        nearest_colors = np.take_along_axis(np.stack(colors), depth_stack.argmin(axis=0)[None, ..., None], axis=0)[0]
        opaque_overlap = count >= 2
        nearest_color_error = np.max(np.abs(fixed - nearest_colors), axis=2)
        heat = fixed.copy()
        heat[wrong] = .25 * heat[wrong] + .75 * np.array([1., 0., .1])
        self.save(self.args.output / f"{stem}-occlusion-pixels.png", heat)
        diff = np.max(np.abs(ground_only - fixed), axis=2)
        ys, xs = np.nonzero(diff > 1 / 255)
        if len(xs):
            x0, x1 = max(0, xs.min() - 30), min(size[0], xs.max() + 31)
            y0, y1 = max(0, ys.min() - 30), min(size[1], ys.max() + 31)
            crop_panels = [self.label(cv2.resize(rgb[y0:y1, x0:x1], (size[0], max(120, round((y1-y0)*size[0]/(x1-x0))))), text)
                           for rgb, text in ((ground_only, "Original composition: overlap crop"), (fixed, "Shared depth: same crop"))]
            cv2.imwrite(str(self.args.output / f"{stem}-occlusion-crop.png"), np.hstack(crop_panels))
        result = dict(run_id=run_id, source_sequence=sequence, frame_id=frame.frame_id, source_metadata=str(metadata_path),
                      before_revision=self.args.before_revision, map_path=map_path, old_ground_plane=old_plane.to_dict(),
                      new_ground_plane=new_plane.to_dict(), corrected_plane_path=str(corrected_plane_path), image_size=size,
                      rasterizers=["gsplat CUDA RGB+ED", "PyTorch3D CUDA"],
                      original_order_wrong_depth_pixels=int(wrong.sum()), opaque_overlap_pixels=int(opaque_overlap.sum()),
                      corrected_nearest_color_mismatch_pixels=int(np.sum(opaque_overlap & (nearest_color_error > 1e-5))),
                      old_order_change_pixels=int(np.sum(np.max(np.abs(ground_only - reversed_old), axis=2) > 1e-6)),
                      corrected_order_change_pixels=int(np.sum(np.max(np.abs(fixed - reversed_fixed), axis=2) > 1e-6)),
                      corrected_order_max_rgb_difference=float(np.max(np.abs(fixed - reversed_fixed))), contacts=contacts)
        (self.args.output / f"{stem}.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps({key: value for key, value in result.items() if key != "contacts"}), flush=True)
        del gaussian
        return result


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, default=root / "artifacts/localization-evaluation/set2-bev-grounded/results")
    parser.add_argument("--planes", type=Path, default=root / "artifacts/bev-replay-ground-planes-final")
    parser.add_argument("--corrected-planes", type=Path, default=root / "artifacts/bev-replay-ground-planes-riverside-restored")
    parser.add_argument("--output", type=Path, default=root / "artifacts/ground-occlusion-diagnostics")
    parser.add_argument("--before-revision", default="3d0906bd44d4c6717fef2935584a821c826b674d")
    parser.add_argument("--frame", action="append", help="run_id:replay_sequence")
    parser.add_argument("--width", type=int, default=960)
    args = parser.parse_args()
    renderer = ReplayGroundOcclusionDiagnostics(args)
    results = [renderer.render(run_id, int(sequence)) for run_id, sequence in
               (value.rsplit(":", 1) for value in (args.frame or ["cooperscene-1-1:64", "cooperscene-1-1:153", "nuscenes-164:93", "nuscenes-529:77"]))]
    (args.output / "summary.json").write_text(json.dumps(results, indent=2) + "\n")


if __name__ == "__main__":
    main()

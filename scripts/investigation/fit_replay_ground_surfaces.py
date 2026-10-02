#!/usr/bin/env python3
"""Export bounded map-road support; unsupported locations keep input poses."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from src.localization.gaussian_map import load_gaussian_map
from src.localization.ground_surface import GroundSurface


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planes", type=Path, default=Path("artifacts/bev-replay-ground-planes-riverside-restored"))
    parser.add_argument("--inputs", type=Path, default=Path("artifacts/first-frame-localization/inputs"))
    parser.add_argument("--output", type=Path, default=Path("artifacts/bev-replay-ground-surfaces"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    index = {}
    for scene in (152, 164, 171, 200, 209, 359, 529, 916):
        run_id = f"nuscenes-{scene}"
        prior = json.loads((args.planes / f"{run_id}.json").read_text())
        frame = np.load(args.inputs / run_id / "frame.npz")
        transform = frame["reference_map_T_dataset"]
        basis = transform[:3, :3]
        origin = (transform @ frame["dataset_T_lidar"])[:3, 3]
        normal = np.asarray(prior["normal"])
        seed = -(normal @ origin + prior["offset_m"]) / (normal @ basis[:, 2])
        gaussian = load_gaussian_map(prior["map_path"])
        scales = np.sort(gaussian.scales, axis=1)
        indices = np.flatnonzero(np.isfinite(gaussian.means).all(axis=1) & (gaussian.opacities > 0)
                                 & (scales[:, 0] < .08) & (scales[:, 0] < .4 * scales[:, 1]))
        rotations = Rotation.from_quat(gaussian.quats[indices][:, [1, 2, 3, 0]]).as_matrix()
        normals = rotations[np.arange(len(indices)), :, gaussian.scales[indices].argmin(axis=1)]
        indices = indices[np.abs(normals @ basis[:, 2]) > .98]
        surface, diagnostics = GroundSurface.from_map_candidates(gaussian.means[indices], origin=origin, basis=basis, seed_height_m=seed)
        output = dict(surface.to_dict(), run_id=run_id, map_path=prior["map_path"],
                      method="Seed-connected low surface modes from thin GSplat geometry; bounded local fits.",
                      gaussian_surface_filter=dict(max_thickness_m=.08, max_min_mid_scale_ratio=.4, min_normal_up_alignment=.98),
                      diagnostics=diagnostics, original_seed_plane=str(args.planes / f"{run_id}.json"),
                      seed_usage="Existing plane only initializes road region near origin; never extrapolated for placement.",
                      unsupported_policy="preserve_input_pose", object_height_used_for_ground_estimation=False)
        path = args.output / f"{run_id}.json"
        path.write_text(json.dumps(output, indent=2) + "\n")
        index[run_id] = str(path.resolve())
        print(run_id, diagnostics, flush=True)
    coop = args.output / "cooperscene-1-1.json"
    coop.write_bytes((args.planes / coop.name).read_bytes())
    for take in range(1, 6):
        for agent in range(1, 4):
            index[f"cooperscene-{take}-{agent}"] = str(coop.resolve())
    (args.output / "local-index.json").write_text(json.dumps(index, indent=2) + "\n")


if __name__ == "__main__":
    main()

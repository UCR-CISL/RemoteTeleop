#!/usr/bin/env python3
"""Render a CooperScene vehicle trajectory until LiDAR leaves the Gaussian map."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.localization.cooperscene_sequence_render import (
    CooperSceneSequenceRenderConfig,
    CooperSceneSequenceRenderer,
)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, default=Path("/mnt/bcc-data/data/CooperScene"))
    parser.add_argument("--splat", type=Path, default=Path("data/riverside_r3.spz"))
    parser.add_argument("--transform", type=Path, required=True,
                        help="Prior localization JSON/NPZ containing gaussian_T_cooperscene.")
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/cooperscene_train1_render"))
    parser.add_argument("--split", default="train")
    parser.add_argument("--scenario", default="1")
    parser.add_argument("--agent", choices=("1", "2", "3"), default="1")
    parser.add_argument("--frame-stride", type=int, default=1)
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--overlap-point-stride", type=int, default=4)
    parser.add_argument("--overlap-map-stride", type=int, default=8)
    parser.add_argument("--baseline-frames", type=int, default=30)
    parser.add_argument("--fine-distance", type=float, default=0.20)
    parser.add_argument("--broad-distance", type=float, default=0.50)
    parser.add_argument("--minimum-frustum-points", type=int, default=1000)
    parser.add_argument("--minimum-fine-ratio", type=float, default=0.25)
    parser.add_argument("--fine-baseline-fraction", type=float, default=0.50)
    parser.add_argument("--maximum-median", type=float, default=0.35)
    parser.add_argument("--median-baseline-factor", type=float, default=1.75)
    parser.add_argument("--stop-after-low-overlap", type=int, default=5)
    parser.add_argument("--probe-only", action="store_true")
    parser.add_argument("--render-device", choices=("auto", "cpu", "cuda"), default="cuda")
    parser.add_argument("--render-downsample", type=int, default=2)
    args = parser.parse_args(argv)
    result = CooperSceneSequenceRenderer(CooperSceneSequenceRenderConfig(
        data_root=args.data_root,
        splat_path=args.splat,
        transform_path=args.transform,
        output_dir=args.output_dir,
        split=args.split,
        scenario=args.scenario,
        agent=args.agent,
        frame_stride=args.frame_stride,
        max_frames=args.max_frames,
        overlap_point_stride=args.overlap_point_stride,
        overlap_map_stride=args.overlap_map_stride,
        baseline_frames=args.baseline_frames,
        fine_distance_m=args.fine_distance,
        broad_distance_m=args.broad_distance,
        minimum_frustum_points=args.minimum_frustum_points,
        minimum_fine_ratio=args.minimum_fine_ratio,
        fine_baseline_fraction=args.fine_baseline_fraction,
        maximum_median_m=args.maximum_median,
        median_baseline_factor=args.median_baseline_factor,
        stop_after_low_overlap=args.stop_after_low_overlap,
        probe_only=args.probe_only,
        render_device=None if args.render_device == "auto" else args.render_device,
        render_downsample=args.render_downsample,
    )).run()
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()

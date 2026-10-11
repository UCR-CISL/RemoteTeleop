"""Evaluate fixed offline frame JSONL; never feed ground truth to a tracker."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from src.evaluation.cooperscene_tracking import FrontCameraCoverageEvaluator, GeometricTrackingEvaluator, GroundTruthContinuity


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frames", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--center-threshold-m", type=float, default=2.0)
    parser.add_argument("--split-discontinuous-gt", action="store_true", help="add separate identity sensitivity metrics; preserve canonical metrics")
    parser.add_argument("--front-camera-only", action="store_true", help="also report symmetric geometric front-camera visibility and per-GT-ID coverage")
    parser.add_argument("--require-valid-image-timestamp", action="store_true", help="opt-in front-camera sensitivity: skip nonpositive exact image_timestamp_ns; absent metadata is an error; all LiDAR scoring remains unchanged")
    parser.add_argument("--precision-target", type=float, help="optional declared minimum for both front-camera detection and tracking precision (e.g. 0.9)")
    args = parser.parse_args()
    if args.require_valid_image_timestamp and not args.front_camera_only:
        parser.error("--require-valid-image-timestamp requires --front-camera-only")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    detection = GeometricTrackingEvaluator(center_threshold_m=args.center_threshold_m, identity_metrics=False)
    tracking = GeometricTrackingEvaluator(center_threshold_m=args.center_threshold_m)
    continuity = GroundTruthContinuity()
    sensitivity = GeometricTrackingEvaluator(center_threshold_m=args.center_threshold_m)
    front = FrontCameraCoverageEvaluator(center_threshold_m=args.center_threshold_m, precision_target=args.precision_target,
                                         require_valid_image_timestamp=args.require_valid_image_timestamp)
    with args.frames.open() as source, (args.output_dir / "assignments.jsonl").open("w") as output:
        for line in source:
            frame = json.loads(line)
            provenance = {key: frame[key] for key in ("sequence_id", "frame_id", "timestamp_us")}
            common = {key: frame[key] for key in ("sequence_id", "timestamp_us", "gt")}
            assigned = dict(provenance, detections=detection.update(predictions=frame["detections"], **common),
                            tracks=tracking.update(predictions=frame["tracks"], **common))
            segmented = continuity.segment(gt=frame["gt"], **provenance)
            if args.split_discontinuous_gt:
                assigned["tracks_split_gt_sensitivity"] = sensitivity.update(predictions=frame["tracks"], gt=segmented,
                    sequence_id=frame["sequence_id"], timestamp_us=frame["timestamp_us"])
            if args.front_camera_only:
                assigned["front_camera"] = front.update(frame)
            output.write(json.dumps(assigned) + "\n")
    summary = {"input": str(args.frames.resolve()), "detections": detection.summary(), "tracks": tracking.summary()}
    if args.split_discontinuous_gt:
        summary["tracks_split_gt_sensitivity"] = sensitivity.summary()
    if args.front_camera_only:
        summary["front_camera"] = front.summary()
    (args.output_dir / "gt_continuity.json").write_text(json.dumps(continuity.provenance(), indent=2) + "\n")
    (args.output_dir / "metrics.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()

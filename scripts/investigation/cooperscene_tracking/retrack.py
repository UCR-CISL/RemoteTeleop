"""Re-run a tracker on cached detections, preserving frames and scored GT."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from src.realtime.bevfusion_tracker import BevFusionKalmanTracker, WorldDetection


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frames", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--maximum-coast-seconds", type=float, default=.5)
    parser.add_argument("--minimum-confirmation-hits", type=int, default=1,
                        help="Consecutive distinct detector observations required before emitting a track (default: 1).")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    trackers: dict[str, BevFusionKalmanTracker] = {}
    timestamps: dict[str, int] = {}
    count = 0
    with args.frames.open() as source, (args.output_dir / "frames.jsonl").open("w") as output:
        for line in source:
            frame = json.loads(line)
            sequence, timestamp = frame["sequence_id"], frame["timestamp_us"]
            if sequence in timestamps and timestamp <= timestamps[sequence]:
                raise ValueError("cached frame timestamps must strictly increase")
            timestamps[sequence] = timestamp
            tracker = trackers.setdefault(sequence, BevFusionKalmanTracker(
                maximum_coast_seconds=args.maximum_coast_seconds,
                minimum_confirmation_hits=args.minimum_confirmation_hits,
            ))
            # Only detector geometry is consumed by the tracker. GT is carried
            # through unchanged so subsequent evaluations use identical inputs.
            detections = [WorldDetection(np.asarray(box["world_T_object"], dtype=float).reshape(4, 4),
                                         tuple(box["dimensions_lwh"]), box.get("label", "car"), float(box["score"]),
                                         box.get("detector_class_name"))
                          for box in frame["detections"]]
            tracker.update(detections, timestamp_us=timestamp)
            frame["tracks"] = [dict(track_id=box.track_id, world_T_object=box.world_T_object.tolist(),
                                    dimensions_lwh=box.dimensions_lwh, label=box.label, score=box.score,
                                    detector_class_name=box.detector_class_name)
                               for box in tracker.boxes_at(timestamp)]
            output.write(json.dumps(frame) + "\n")
            count += 1
    (args.output_dir / "retrack_manifest.json").write_text(json.dumps({"input": str(args.frames.resolve()), "frames": count,
        "tracker": "BevFusionKalmanTracker", "maximum_coast_seconds": args.maximum_coast_seconds,
        "minimum_confirmation_hits": args.minimum_confirmation_hits,
        "confirmation": "consecutive matched detector updates at distinct capture times; unmatched tentative streak resets; only confirmed tracks are emitted",
        "expiry": "before association", "gt_used_as_tracker_input": False}, indent=2) + "\n")


if __name__ == "__main__":
    main()

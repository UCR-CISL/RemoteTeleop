"""Controlled class/confidence trials from immutable raw detector outputs."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from src.realtime.bevfusion_tracker import BevFusionKalmanTracker, WorldDetection


class CachedVehicleClassTrial:
    """Group declared motor-vehicle classes without consuming GT in tracking."""

    motor_vehicle_classes = {"car", "truck", "bus", "trailer", "construction_vehicle"}

    def __init__(self, *, classes, score_threshold=.2):
        self.classes = set(classes)
        if not self.classes or not self.classes <= self.motor_vehicle_classes:
            raise ValueError("classes must be a nonempty subset of declared motor vehicle classes")
        if not math.isfinite(score_threshold) or not 0 <= score_threshold <= 1:
            raise ValueError("score_threshold must be in [0, 1]")
        self.threshold = score_threshold

    def run(self, frames_path: Path, raw_directory: Path, output_dir: Path) -> dict:
        raw = {}
        for path in raw_directory.glob("*.json"):
            record = json.loads(path.read_text())
            if record["scene_generation"] == "warmup":
                continue
            key = (record["scene_generation"], record["frame_id"], record["timestamp_us"])
            if key in raw:
                raise ValueError(f"duplicate raw provenance: {key}")
            raw[key] = record
        frames = [json.loads(line) for line in frames_path.read_text().splitlines()]
        counts, trackers, last_times = {}, {}, {}
        paired = []
        for frame in frames:
            sequence = frame["sequence_id"]
            index = counts.get(sequence, 0)
            counts[sequence] = index + 1
            key = (sequence, frame["frame_id"], frame["timestamp_us"])
            if key not in raw:
                raise ValueError(f"missing exact raw frame provenance: {key}")
            record = raw[key]
            if record["sequence"] != index or record["request_id"] != f"{sequence}:{index}":
                raise ValueError("raw request identity or ordinal does not match cached frame")
            for field, shape in (("world_T_lidar", (4, 4)), ("world_T_camera", (4, 4)), ("camera_intrinsic", (3, 3))):
                if not np.allclose(np.asarray(record[field]).reshape(shape), np.asarray(frame[field]).reshape(shape), atol=1e-9, rtol=0):
                    raise ValueError(f"raw calibration mismatch: {field}")
            if any(record[field] != frame[field] for field in ("image_width", "image_height")):
                raise ValueError("raw image dimensions mismatch")
            if sequence in last_times and frame["timestamp_us"] <= last_times[sequence]:
                raise ValueError("cached timestamps must strictly increase")
            last_times[sequence] = frame["timestamp_us"]
            paired.append((frame, record))
        output_dir.mkdir(parents=True, exist_ok=False)
        with (output_dir / "frames.jsonl").open("w") as output:
            for original, record in paired:
                frame = dict(original)
                sequence = frame["sequence_id"]
                tracker = trackers.setdefault(sequence, BevFusionKalmanTracker())
                world = np.asarray(frame["world_T_lidar"]).reshape(4, 4)
                detections, world_detections = [], []
                for raw_index, box in enumerate(record["predictions"]):
                    if box["class_name"] not in self.classes or box["score"] < self.threshold:
                        continue
                    tensor = np.asarray(box["box_tensor"], dtype=float)
                    x_size, y_size, height = tensor[3:6]
                    yaw = float(tensor[6])
                    if y_size > x_size:
                        length, width = y_size, x_size
                        yaw += math.pi / 2
                    else:
                        length, width = x_size, y_size
                    transform = np.eye(4)
                    c, s = math.cos(yaw), math.sin(yaw)
                    transform[:3, :3] = ((c, -s, 0), (s, c, 0), (0, 0, 1))
                    transform[:3, 3] = box["gravity_center"]
                    pose, dimensions = world @ transform, (float(length), float(width), float(height))
                    detections.append(dict(track_id=f"raw:{record['sequence']}:{raw_index}", world_T_object=pose.tolist(),
                        dimensions_lwh=dimensions, label="vehicle", detector_class_name=box["class_name"],
                        detector_class_id=box["class_id"], score=box["score"]))
                    world_detections.append(WorldDetection(pose, dimensions, "vehicle", box["score"]))
                # Only raw detector boxes and original timestamps enter tracking.
                tracker.update(world_detections, timestamp_us=frame["timestamp_us"])
                frame["detections"] = detections
                frame["tracks"] = [dict(track_id=box.track_id, world_T_object=box.world_T_object.tolist(),
                    dimensions_lwh=box.dimensions_lwh, label="vehicle", score=box.score)
                    for box in tracker.boxes_at(frame["timestamp_us"])]
                frame["gt"] = [dict(box, label="vehicle", annotation_class_name=box.get("label", "car")) for box in original["gt"]]
                output.write(json.dumps(frame) + "\n")
        manifest = dict(input_frames=str(frames_path.resolve()), raw_directory=str(raw_directory.resolve()),
            frames=len(frames), sequence_frame_counts=counts, score_threshold=self.threshold,
            detector_classes=sorted(self.classes), evaluated_class="vehicle",
            gt_label_policy="native YAML vehicles has no detailed category; preserve annotation_class_name and group only evaluation label",
            detector_label_policy="preserve detector_class_name/class_id; group declared motor vehicle classes for tracking and evaluation",
            tracker="BevFusionKalmanTracker; 0.5 seconds coast; expiry before association",
            gt_used_as_tracker_input=False, gt_geometry_and_timestamps_unchanged=True,
            raw_provenance="exact generation/frame/timestamp/request identity/ordinal; matching calibration and image dimensions")
        (output_dir / "refilter_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frames", type=Path)
    parser.add_argument("--raw-directory", type=Path, required=True)
    parser.add_argument("--classes", nargs="+", default=["car"])
    parser.add_argument("--score-threshold", type=float, default=.2)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(CachedVehicleClassTrial(classes=args.classes, score_threshold=args.score_threshold).run(
        args.frames, args.raw_directory, args.output_dir), indent=2))


if __name__ == "__main__":
    main()

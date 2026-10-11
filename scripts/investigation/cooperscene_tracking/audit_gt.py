"""Inspect native annotation identity continuity without altering GT IDs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from src.localization.cooperscene import CooperSceneSequenceDataset


class GroundTruthIdentityAudit:
    """Flag implausible annotation jumps for declared identity sensitivity checks."""

    def __init__(self, *, adjacent_jump_threshold_m: float = 10.0) -> None:
        if adjacent_jump_threshold_m <= 0 or not np.isfinite(adjacent_jump_threshold_m):
            raise ValueError("jump threshold must be positive and finite")
        self.threshold = adjacent_jump_threshold_m

    def run(self, frames) -> dict:
        previous = {}
        identities = set()
        max_step = 0.0
        max_dimension_change = 0.0
        flags = []
        count = 0
        for frame in frames:
            current = {box.track_id: box for box in frame.vehicle_boxes}
            identities.update(current)
            for identity in current.keys() & previous.keys():
                before, after = previous[identity], current[identity]
                distance = float(np.linalg.norm(after.map_T_object[:3, 3] - before.map_T_object[:3, 3]))
                dimension_delta = np.abs(np.asarray(after.dimensions_lwh) - before.dimensions_lwh)
                max_step = max(max_step, distance)
                max_dimension_change = max(max_dimension_change, float(dimension_delta.max()))
                if distance > self.threshold:
                    flags.append(dict(track_id=identity, previous_frame_id=previous_frame, frame_id=frame.frame_id,
                                      adjacent_displacement_m=distance, dimension_abs_change_lwh_m=dimension_delta.tolist()))
            previous, previous_frame = current, frame.frame_id
            count += 1
        return dict(frames=count, unique_ids=len(identities), adjacent_jump_threshold_m=self.threshold,
                    max_adjacent_displacement_m=max_step, max_adjacent_dimension_delta_m=max_dimension_change,
                    flags=flags, gt_modified=False)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--split", default="train")
    parser.add_argument("--take", required=True)
    parser.add_argument("--agent", required=True)
    parser.add_argument("--start-frame")
    parser.add_argument("--max-frames", type=int, default=100)
    parser.add_argument("--adjacent-jump-threshold-m", type=float, default=10.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.max_frames <= 0:
        parser.error("max-frames must be positive")
    dataset = CooperSceneSequenceDataset(args.data_root, load_lidar=False)
    frames = []
    for frame in dataset.sequence(args.split, args.take, args.agent):
        if args.start_frame and int(frame.frame_id) < int(args.start_frame):
            continue
        frames.append(frame)
        if len(frames) == args.max_frames:
            break
    result = GroundTruthIdentityAudit(adjacent_jump_threshold_m=args.adjacent_jump_threshold_m).run(frames)
    result.update(split=args.split, take=args.take, agent=args.agent, first_frame=frames[0].frame_id if frames else None,
                  last_frame=frames[-1].frame_id if frames else None)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as output:
        json.dump(result, output, indent=2)
        output.write("\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()

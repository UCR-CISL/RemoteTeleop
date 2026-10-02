"""Search replay time with fixed CooperScene XYZ localization, never tune the fit."""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import os
from pathlib import Path
import shutil
import sys

for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[name] = "2"
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import cv2
import numpy as np
from mcap.reader import make_reader
from scipy.spatial import cKDTree

from scripts.investigation.cache_localization_first_frames import save_frame
from scripts.investigation.first_frame_xyz import FirstFrameXYZInvestigation, map_proxy, source_points, registration_config
from src.data.foxglove_mcap import FoxgloveMcapReader, _ProtobufDecoder, _transform


class IndexedCooperFrameReader:
    """Cross-check indexed sensor reads against the original ordinal replay poses."""

    def __init__(self, first_root: Path, original_id: str, trajectory_root: Path):
        self.original_id = original_id
        directory = first_root / original_id
        self.first = dict(np.load(directory / "frame.npz"))
        self.metadata = json.loads((directory / "metadata.json").read_text())
        trajectory = trajectory_root / original_id / "attempt-001/lambda/localization-trajectory.jsonl"
        self.rows = [json.loads(line) for line in trajectory.read_text().splitlines()]
        self.agent = self.metadata["agent"]
        self.first_time = self.metadata["source_timestamp_ns"]
        self.decoder = _ProtobufDecoder()
        self.stream = Path(self.metadata["source_path"]).open("rb")
        self.reader = make_reader(self.stream)
        self.dataset_T_reference = np.linalg.inv(self.first["reference_map_T_dataset"])
        self.lidar_T_camera = np.linalg.inv(self.first["dataset_T_lidar"]) @ self.first["dataset_T_camera"]
        summary = self.reader.get_summary()
        counts = {c.topic: summary.statistics.channel_message_counts.get(i, 0)
                  for i, c in summary.channels.items()}
        assert counts[f"/{self.agent}_Camera"] == len(self.rows) == counts[f"/{self.agent}_LIDAR"]

    def close(self):
        self.stream.close()

    def extract(self, index: int, root: Path) -> dict:
        run_id = f"{self.original_id}-f{index:06d}"
        existing = root / run_id / "metadata.json"
        if existing.exists():
            return json.loads(existing.read_text())
        row = self.rows[index]
        expected_pose = self.dataset_T_reference @ np.asarray(row["reference_map_T_lidar"])
        approximate_time = self.first_time + int(row["timestamp_us"]) * 1000
        image_topic, lidar_topic = f"/{self.agent}_Camera", f"/{self.agent}_LIDAR"
        images, lidars, poses, boxes, labels = {}, {}, {}, {}, {}
        for schema, channel, message in self.reader.iter_messages(
                topics=[image_topic, lidar_topic, "/tf", "/bbox_global"],
                start_time=approximate_time - 160_000_000, end_time=approximate_time + 160_000_000):
            decoded = self.decoder.decode(schema, channel, message.data)
            timestamp = message.log_time
            if channel.topic == image_topic:
                images[timestamp] = decoded
            elif channel.topic == lidar_topic:
                lidars[timestamp] = decoded
            elif channel.topic == "/tf" and decoded.child_frame_id == f"{self.agent}_base_link":
                assert decoded.parent_frame_id == "map"
                poses[timestamp] = _transform(decoded.translation, decoded.rotation)
            elif channel.topic == "/bbox_global":
                for entity in decoded.entities:
                    if entity.id == "global_bboxes":
                        boxes[timestamp] = entity
                    elif entity.id.endswith("_frame_label"):
                        labels[timestamp] = entity.texts[0].text
        matching = [timestamp for timestamp, pose in poses.items()
                    if np.allclose(pose, expected_pose, atol=1e-9, rtol=0)]
        if not matching:
            raise ValueError(f"{run_id}: indexed TF does not match ordinal replay pose")
        timestamp = min(matching, key=lambda time: abs(time - approximate_time))
        if abs(timestamp - approximate_time) > 30_000_000:
            raise ValueError(f"{run_id}: sensor clock drift exceeds unambiguous-frame tolerance")
        label_times = [time for time, label in labels.items() if label == row["frame_id"]]
        if len(label_times) != 1 or label_times[0] not in boxes:
            raise ValueError(f"{run_id}: cannot identify expected ordinal box/frame label")
        label_time = label_times[0]
        if abs(timestamp - label_time) > 100_000_000:
            raise ValueError(f"{run_id}: camera/annotation skew exceeds existing reader limit")
        image = cv2.imdecode(np.frombuffer(images[timestamp].data, np.uint8), cv2.IMREAD_COLOR)
        points = FoxgloveMcapReader._point_cloud(lidars[timestamp])
        vehicles = FoxgloveMcapReader._vehicles(boxes[label_time])
        arrays = dict(self.first)
        arrays.update(points_lidar=points, dataset_T_lidar=poses[timestamp],
                      dataset_T_camera=poses[timestamp] @ self.lidar_T_camera,
                      boxes_T_dataset_object=np.asarray([v.world_T_object for v in vehicles]).reshape(-1, 4, 4),
                      boxes_dimensions_lwh=np.asarray([v.dimensions_lwh for v in vehicles]).reshape(-1, 3))
        record = dict(self.metadata, run_id=run_id, original_run_id=self.original_id,
                      frame_id=row["frame_id"], sequence=index, frame_offset=index,
                      timestamp_us=row["timestamp_us"], elapsed_seconds=row["timestamp_us"] / 1e6,
                      sequence_frame_count=len(self.rows), sequence_fraction=index / (len(self.rows)-1),
                      source_timestamp_ns=timestamp, first_frame_only=index == 0,
                      source_pose_max_abs_error=float(np.max(np.abs(poses[timestamp] - expected_pose))),
                      sensor_clock_prediction_error_ns=timestamp-approximate_time,
                      source_join="exact ordinal frame label and pose checked; camera/LiDAR timestamps equal TF")
        return save_frame(root, record, image, arrays, [v.text_label for v in vehicles])


_worker = None


def strict_rejection_certificate(initial_points, initial_distances, config):
    """Conservative necessary residual bounds under the unchanged correction gates.

    For C = candidate @ inverse(initial), ||C p-p|| <= t_max +
    2 sin(theta_max/2)||p||. Distance to a fixed point set is 1-Lipschitz.
    Every input point must be a full-resolution retained registration point.
    """
    displacement = (config.max_correction_translation +
                    2 * np.sin(np.radians(config.max_correction_rotation_degrees) / 2)
                    * np.linalg.norm(initial_points, axis=1))
    lower = np.maximum(0, initial_distances - displacement - 1e-9)
    median, p95 = np.percentile(lower, [50, 95])
    threshold = config.icp_distance_factor * config.refinement_voxel_sizes[-1]
    ratio_upper = float(np.mean(lower <= threshold))
    reasons = []
    if median > config.max_all_point_median_residual:
        reasons.append("all_point_median_residual")
    if p95 > config.max_all_point_p95_residual:
        reasons.append("all_point_p95_residual")
    if ratio_upper < config.min_inlier_ratio:
        reasons.append("inlier_ratio")
    return dict(certified_impossible=bool(reasons), rejection_reasons=reasons,
                full_retained_point_count=len(lower), residual_median_lower_bound_m=float(median),
                residual_p95_lower_bound_m=float(p95), inlier_ratio_upper_bound=ratio_upper,
                max_correction_translation_m=config.max_correction_translation,
                max_correction_rotation_deg=config.max_correction_rotation_degrees,
                inlier_radius_m=threshold, numerical_margin_m=1e-9,
                mathematical_basis="distance-to-map Lipschitz bound with original map-frame relative-correction gates")


def initialize_worker(inputs, output):
    global _worker
    _worker = FirstFrameXYZInvestigation(Path(inputs), Path(output))


def register_frame(record):
    result = _worker.run_one(record, "baseline")
    result.update(original_run_id=record["original_run_id"], frame_offset=record["frame_offset"],
                  elapsed_seconds=record["elapsed_seconds"], sequence_fraction=record["sequence_fraction"],
                  sequence_percentage=100*record["sequence_fraction"],
                  sequence_frame_count=record["sequence_frame_count"], first_frame_only=record["frame_offset"] == 0,
                  frame_advance_only=True, optimizer_settings_changed=False,
                  cheap_overlap=record.get("cheap_overlap"), sampling_phase=record["sampling_phase"])
    target_hit = result.get("accepted", False) and result.get("fresh_error", {}).get("xyz_m", 1e9) < .1
    result.update(target_eligible=target_hit and record["sequence_fraction"] <= .25,
                  target_preferred=target_hit and record["sequence_fraction"] < .15)
    _worker._write(_worker.output / "baseline" / f"{record['run_id']}.json", result)
    return result


class CooperFrameAdvanceSearch:
    def __init__(self, args):
        self.args = args
        self.inputs = args.output / "inputs"
        self.results = args.output / "results"
        self.readers = {f"cooperscene-{take}-{agent}": IndexedCooperFrameReader(args.first_inputs,
                        f"cooperscene-{take}-{agent}", args.trajectory_root)
                        for take in range(1, 6) for agent in (1, 2, 3)}
        self.target = map_proxy(next(iter(self.readers.values())).metadata["map_path"])
        self.tree = cKDTree(self.target)
        self.records = {run_id: {} for run_id in self.readers}
        self.certificates = {run_id: {} for run_id in self.readers}
        self.refined_brackets = {run_id: {} for run_id in self.readers}
        summary_path = args.output / "summary.json"
        if summary_path.exists():
            for row in json.loads(summary_path.read_text())["results"]:
                self.refined_brackets[row["run_id"]] = row.get("refinement_brackets", {})
        self.inputs.mkdir(parents=True, exist_ok=True)
        self.results.mkdir(parents=True, exist_ok=True)
        for path in (self.results / "baseline").glob("*.json"):
            record = json.loads(path.read_text())
            if "original_run_id" in record:
                target = record.get("accepted", False) and record.get("fresh_error", {}).get("xyz_m", 1e9) < .1
                record.update(sequence_percentage=100*record["sequence_fraction"],
                              target_eligible=target and record["sequence_fraction"] <= .25,
                              target_preferred=target and record["sequence_fraction"] < .15)
                temporary = path.with_suffix(".tmp")
                temporary.write_text(json.dumps(record, indent=2) + "\n")
                temporary.replace(path)
                self.records[record["original_run_id"]][record["frame_offset"]] = record
        for path in (args.output / "certified_exclusions").glob("*.json"):
            record = json.loads(path.read_text())
            self.certificates[record["original_run_id"]][record["frame_offset"]] = record

    @staticmethod
    def hits(record):
        target = record.get("accepted", False) and record.get("fresh_error", {}).get("xyz_m", 1e9) < .1
        return dict(scan_accepted=record.get("scan_registration", {}).get("accepted", False),
                    xyz_accepted=record.get("accepted", False),
                    target=target and record["sequence_fraction"] <= .25,
                    target_eligible=target and record["sequence_fraction"] <= .25,
                    target_preferred=target and record["sequence_fraction"] < .15)

    def prepare(self, run_id, index, phase):
        record = self.readers[run_id].extract(index, self.inputs)
        frame = dict(np.load(record["npz_path"]))
        source = source_points(frame, True)
        initial = frame["pipeline_initial_map_T_dataset"]
        predicted = source @ initial[:3, :3].T + initial[:3, 3]
        distances, _ = self.tree.query(predicted, workers=2)
        record["rejection_certificate"] = strict_rejection_certificate(
            predicted, distances, registration_config("cooperscene"))
        sampled = distances[::16]
        record["cheap_overlap"] = dict(source_stride=16, initializer_only=True,
             fraction_within_02m=float(np.mean(sampled <= .2)), fraction_within_1m=float(np.mean(sampled <= 1)),
             median_m=float(np.median(sampled)), p95_m=float(np.percentile(sampled, 95)),
             heuristic_used_to_skip_registration=False)
        record["sampling_phase"] = phase
        Path(record["npz_path"]).with_name("metadata.json").write_text(json.dumps(record, indent=2) + "\n")
        return record

    def batch(self, pool, requests):
        def priority(request):
            previous = max(self.records[request[0]].values(), key=lambda row: row["frame_offset"])
            return -previous.get("cheap_overlap", {}).get("fraction_within_1m",
                       previous.get("scan_registration", {}).get("inlier_ratio", 0))
        futures = {}
        # Previous cheap overlap prioritizes scheduling. Extraction overlaps CPU
        # fitting; every requested frame is still fitted, including poor coverage.
        for run_id, index, phase in sorted(requests, key=priority):
            if index not in self.records[run_id] and index not in self.certificates[run_id]:
                record = self.prepare(run_id, index, phase)
                if record["rejection_certificate"]["certified_impossible"]:
                    record.update(registration_executed=False, accepted_possible=False,
                                  optimizer_settings_changed=False, frame_advance_only=True)
                    destination = self.args.output / "certified_exclusions" / f"{record['run_id']}.json"
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    destination.write_text(json.dumps(record, indent=2) + "\n")
                    self.certificates[run_id][index] = record
                    print("CERTIFIED_EXCLUSION", run_id, index, record["rejection_certificate"]["rejection_reasons"], flush=True)
                    self.write_summary()
                else:
                    futures[pool.submit(register_frame, record)] = record
        for future in as_completed(futures):
            record = future.result()
            self.records[record["original_run_id"]][record["frame_offset"]] = record
            print("RESULT", record["original_run_id"], record["frame_offset"], self.hits(record),
                  record.get("fresh_error", {}).get("xyz_m"), flush=True)
            self.write_summary()

    def seed_frame_zero(self):
        for run_id, reader in self.readers.items():
            if 0 in self.records[run_id]:
                continue
            # Reuse the already independently validated exact baseline computation.
            old = json.loads((self.args.first_results / f"{run_id}.json").read_text())
            new_id = f"{run_id}-f000000"
            shutil.copytree(self.args.first_inputs / run_id, self.inputs / new_id, dirs_exist_ok=True)
            metadata = dict(reader.metadata, run_id=new_id, original_run_id=run_id, frame_offset=0,
                            elapsed_seconds=0.0, sequence_fraction=0.0, sequence_frame_count=len(reader.rows),
                            npz_path=str((self.inputs/new_id/"frame.npz").resolve()),
                            camera_path=str((self.inputs/new_id/"camera.png").resolve()))
            (self.inputs/new_id/"metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
            old.update(run_id=new_id, original_run_id=run_id, frame_offset=0, elapsed_seconds=0.0,
                       sequence_fraction=0.0, sequence_frame_count=len(reader.rows),
                       frame_advance_only=True, optimizer_settings_changed=False, sampling_phase="reused_frame_zero")
            destination = self.results / "baseline" / f"{new_id}.json"
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text(json.dumps(old, indent=2) + "\n")
            self.records[run_id][0] = old

    def write_summary(self):
        summaries = []
        for run_id, records in self.records.items():
            ordered = [records[index] for index in sorted(records) if records[index]["sequence_fraction"] <= .25]
            examined = sorted(set(records) | set(self.certificates[run_id]))
            cutoff = int(.25*(len(self.readers[run_id].rows)-1))
            eligible_examined = [index for index in examined if index <= cutoff]
            grid = set(range(0, cutoff+1, 10)) | {cutoff}
            first = {}
            for milestone in ("scan_accepted", "xyz_accepted", "target"):
                hits = [record for record in ordered if self.hits(record)[milestone]]
                first[milestone] = None if not hits else {key: hits[0][key] for key in (
                    "run_id", "frame_id", "frame_offset", "elapsed_seconds", "sequence_fraction", "fresh_error")}
                if first[milestone] is not None:
                    first[milestone]["sequence_percentage"] = 100*first[milestone]["sequence_fraction"]
            summaries.append(dict(run_id=run_id, sequence_frame_count=len(self.readers[run_id].rows),
                 hard_cutoff_frame_offset=int(.25*(len(self.readers[run_id].rows)-1)),
                 preferred_maximum_fraction=.15, hard_maximum_fraction=.25,
                 tested_indices=sorted(records), first_observed=first,
                 certified_excluded_indices=sorted(self.certificates[run_id]),
                 examined_indices=examined,
                 end_of_sequence_tested=len(self.readers[run_id].rows)-1 in records,
                 end_of_sequence_examined=len(self.readers[run_id].rows)-1 in examined,
                 target_found=first["target"] is not None,
                 target_eligible=first["target"] is not None,
                 target_preferred=bool(first["target"] and first["target"]["sequence_fraction"] < .15),
                 eligible_grid_search_complete=(grid.issubset(examined) or first["target"] is not None),
                 eligible_window_fully_examined=eligible_examined == list(range(cutoff+1)),
                 eligible_examined_frame_count=len(eligible_examined), eligible_window_frame_count=cutoff+1,
                 refinement_brackets=self.refined_brackets[run_id],
                 globally_earliest_proven={key: bool(value and list(range(value["frame_offset"]+1)) ==
                    [i for i in examined if i <= value["frame_offset"]]) for key, value in first.items()},
                 maximum_tested_fraction=max((r["sequence_fraction"] for r in ordered), default=0),
                 maximum_examined_fraction=max(examined, default=0)/(len(self.readers[run_id].rows)-1)))
        payload = dict(settings="Exact existing baseline; no optimizer, prior, initializer, mask or gate changes",
                       search_selection="Later frames searched for acceptance and <0.1m reference agreement; not optimizer improvement",
                       independent_ground_truth=False, grid_spacing_frames=10,
                       preferred_maximum_fraction=.15, hard_maximum_fraction=.25,
                       grid_completion_definition="All planned grid points/cutoff examined or target found; does not imply every eligible frame examined",
                       sampling="1-second grid; each requested frame fitted or exactly certified unable to pass original gates; milestone brackets refined per frame",
                       results=summaries)
        destination = self.args.output / "summary.json"
        temporary = destination.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload, indent=2) + "\n")
        temporary.replace(destination)

    def run(self):
        self.seed_frame_zero()
        self.write_summary()
        with ProcessPoolExecutor(max_workers=self.args.workers, initializer=initialize_worker,
                                 initargs=(str(self.inputs), str(self.results))) as pool:
            maximum = max(int(.25*(len(reader.rows)-1)) for reader in self.readers.values())
            for index in range(10, maximum+9, 10):
                requests = []
                for run_id, reader in self.readers.items():
                    if any(self.hits(row)["target"] for row in self.records[run_id].values()):
                        continue
                    selected = min(index, int(.25*(len(reader.rows)-1)))
                    phase = "early_quarter_grid" if selected <= (len(reader.rows)-1)/4 else "later_grid"
                    requests.append((run_id, selected, phase))
                if not requests:
                    break
                self.batch(pool, requests)
                refine = set()
                for run_id, records in self.records.items():
                    for milestone in ("scan_accepted", "xyz_accepted", "target"):
                        if milestone in self.refined_brackets[run_id]:
                            continue
                        hits = [i for i, record in records.items() if record["sequence_fraction"] <= .25
                                and self.hits(record)[milestone]]
                        if not hits:
                            continue
                        upper = min(hits)
                        lower = max(0, ((upper-1)//10)*10)
                        self.refined_brackets[run_id][milestone] = [lower, upper]
                        refine.update((run_id, frame, "milestone_bracket_refinement") for frame in range(lower, upper))
                self.batch(pool, sorted(refine))
                # A positive target merits exhaustive earlier-frame verification,
                # including gaps before the first successful one-second bracket.
                prefix = []
                for run_id, records in self.records.items():
                    hits = [i for i, record in records.items() if self.hits(record)["target"]]
                    if hits:
                        upper = min(hits)
                        self.refined_brackets[run_id]["target_prefix"] = [0, upper]
                        prefix.extend((run_id, frame, "target_prefix_verification") for frame in range(upper))
                self.batch(pool, prefix)
                self.write_summary()
            self.dense_near_misses(pool)
        for reader in self.readers.values():
            reader.close()

    def dense_near_misses(self, pool):
        """Check chronological gaps for accepted <0.15 m near misses only."""
        for run_id, reader in self.readers.items():
            records = self.records[run_id]
            if any(self.hits(record)["target"] for record in records.values()):
                continue
            if not any(record.get("accepted", False) and record["sequence_fraction"] <= .25
                       and record.get("fresh_error", {}).get("xyz_m", 1e9) < .15 for record in records.values()):
                continue
            cutoff = int(.25*(len(reader.rows)-1))
            self.refined_brackets[run_id]["dense_near_miss_window"] = [0, cutoff]
            missing = [index for index in range(cutoff+1)
                       if index not in records and index not in self.certificates[run_id]]
            for start in range(0, len(missing), self.args.workers):
                if any(self.hits(record)["target"] for record in records.values()):
                    break
                self.batch(pool, [(run_id, index, "dense_near_miss_chronological")
                                  for index in missing[start:start+self.args.workers]])
                self.write_summary()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("artifacts/frame-advance-localization/cooperscene"))
    parser.add_argument("--first-inputs", type=Path, default=Path("artifacts/first-frame-localization/inputs"))
    parser.add_argument("--first-results", type=Path, default=Path("artifacts/first-frame-localization/xyz/baseline"))
    parser.add_argument("--trajectory-root", type=Path, default=Path("/tmp/RemoteTeleop-mcap-replay/artifacts/proxy-batch/xyz-results"))
    parser.add_argument("--workers", type=int, default=3)
    args = parser.parse_args()
    CooperFrameAdvanceSearch(args).run()

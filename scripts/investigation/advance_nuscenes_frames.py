"""Fixed-method later-frame reference-agreement search; no localization tuning.

Frame zero is reused from the completed first-frame investigation. A fixed 1 s
grid, including the exact 25% cutoff, is followed by native 0.1 s checks from
zero until the first hit or cutoff. This establishes the earliest eligible hit.
Frames strictly before 15% are preferred. Earlier unrestricted attempts remain
immutable historical records, excluded from eligible results.
All fits retain the original scene initializer, reference, and acceptance gates.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
from dataclasses import asdict, replace
import io
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tarfile

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.investigation.first_frame_xyz import FirstFrameXYZInvestigation, jsonable, source_points, registration_config, pose_error

REMOTE_ROOT = "/home/justin/bilateral-driving-repro/data/nuscenes/processed_10Hz/trainval"


class NuScenesFrameAdvance:
    """Stage selected recorded frames and reuse the established XYZ fitter."""

    def __init__(self, output: Path, scenes: list[str], variants: list[str], native_workers: int = 1):
        self.output, self.scenes, self.variants = output, scenes, variants
        self.inputs = output / "inputs"
        self.original = ROOT / "artifacts/first-frame-localization"
        self.runner = FirstFrameXYZInvestigation(self.inputs, output / "attempts")
        self.scene_metadata = {}
        self.native_workers = native_workers

    @staticmethod
    def _write(path, payload):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(jsonable(payload), indent=2, allow_nan=False) + "\n")

    def _fetch(self, scene: str, index: int | None, limit: int = 0):
        # Remote code only reads recorded files and emits a tar stream.
        code = """import io,json,pathlib,sys,tarfile
root=pathlib.Path(sys.argv[1]); index=int(sys.argv[2])
with tarfile.open(fileobj=sys.stdout.buffer, mode='w|') as archive:
 if index == -1:
  paths=['instances/instances_info.json','instances/frame_instances.json','intrinsics/0.txt']
  indices=sorted(int(p.name.split('_')[0]) for p in (root/'images').glob('*_0.jpg'))
  payload=json.dumps({'indices':indices,'frame_count':len(indices),'timestamp_step_us':100000}).encode()
  item=tarfile.TarInfo('scene_info.json'); item.size=len(payload); archive.addfile(item,io.BytesIO(payload))
 else:
  paths=[]
  indices=range(int(sys.argv[3])+1) if index == -2 else [index]
  for frame in indices:
   stem=f'{frame:03d}'
   paths.extend([f'lidar/{stem}.bin',f'lidar_pose/{stem}.txt',f'extrinsics/{stem}_0.txt',f'images/{stem}_0.jpg'])
 for name in paths: archive.add(root/name,arcname=name,recursive=False)
"""
        command = " ".join(shlex.quote(value) for value in ("python3", "-c", code, f"{REMOTE_ROOT}/{scene}", str(-1 if index is None else index), str(limit)))
        completed = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=15", "lambda-justin", command], check=True, capture_output=True)
        destination = self.output / "recorded" / scene
        destination.mkdir(parents=True, exist_ok=True)
        with tarfile.open(fileobj=io.BytesIO(completed.stdout)) as archive:
            for member in archive.getmembers():
                target = destination / member.name
                if not target.resolve().is_relative_to(destination.resolve()):
                    raise ValueError("remote archive has an unexpected path")
                target.parent.mkdir(parents=True, exist_ok=True)
                with archive.extractfile(member) as source, target.open("wb") as sink:
                    shutil.copyfileobj(source, sink)

    def _scene(self, scene):
        if scene not in self.scene_metadata:
            directory = self.output / "recorded" / scene
            if not (directory / "scene_info.json").exists():
                self._fetch(scene, None)
            info = json.loads((directory / "scene_info.json").read_text())
            if info["indices"] != list(range(info["frame_count"])):
                raise ValueError(f"scene {scene} frame indices are not contiguous")
            self.scene_metadata[scene] = info
        return self.scene_metadata[scene]

    def _stage(self, scene, index):
        run_id = f"nuscenes-{scene}-f{index:03d}"
        destination = self.inputs / run_id
        metadata_path = destination / "metadata.json"
        if metadata_path.exists():
            return json.loads(metadata_path.read_text())
        original_directory = self.original / "inputs" / f"nuscenes-{scene}"
        metadata = json.loads((original_directory / "metadata.json").read_text())
        frame = dict(np.load(original_directory / "frame.npz"))
        destination.mkdir(parents=True, exist_ok=True)
        info = self._scene(scene)
        if index == 0:
            shutil.copy2(original_directory / "camera.png", destination / "camera.png")
            camera_path = destination / "camera.png"
        else:
            recorded = self.output / "recorded" / scene
            stem = f"{index:03d}"
            if not (recorded / "lidar" / f"{stem}.bin").exists():
                self._fetch(scene, index)
            frame["points_lidar"] = np.fromfile(recorded / "lidar" / f"{stem}.bin", dtype=np.float32).reshape(-1, 4)
            frame["dataset_T_lidar"] = np.loadtxt(recorded / "lidar_pose" / f"{stem}.txt")
            frame["dataset_T_camera"] = np.loadtxt(recorded / "extrinsics" / f"{stem}_0.txt")
            instances = json.loads((recorded / "instances/instances_info.json").read_text())
            frame_instances = json.loads((recorded / "instances/frame_instances.json").read_text())
            transforms, dimensions, labels = [], [], []
            for identifier in frame_instances.get(str(index), []):
                instance = instances[str(identifier)]
                annotation = instance["frame_annotations"]
                at = annotation["frame_idx"].index(index)
                transforms.append(annotation["obj_to_world"][at])
                dimensions.append(annotation["box_size"][at])
                labels.append(instance["class_name"])
            frame["boxes_T_dataset_object"] = np.asarray(transforms).reshape(-1, 4, 4)
            frame["boxes_dimensions_lwh"] = np.asarray(dimensions).reshape(-1, 3)
            frame["box_labels"] = np.asarray(labels, dtype=str)
            camera_path = recorded / "images" / f"{stem}_0.jpg"
        np.savez_compressed(destination / "frame.npz", **frame)
        metadata.update(run_id=run_id, frame_id=f"{index:03d}", sequence=index, timestamp_us=index * 100000,
                        first_frame_only=False, offset_seconds=index / 10, sequence_fraction=index / (info["frame_count"] - 1),
                        frame_count=info["frame_count"], npz_path=str(destination / "frame.npz"), camera_path=str(camera_path),
                        point_count=len(frame["points_lidar"]), box_count=len(frame["boxes_dimensions_lwh"]),
                        box_labels=frame["box_labels"].tolist(), initializer_policy="original scene first-camera anchor retained")
        self._write(metadata_path, metadata)
        return metadata

    @staticmethod
    def _eligibility(result):
        fraction = result["sequence_index"] / (result["frame_count"] - 1)
        reference_hit = result["accepted"] and result.get("fresh_error", {}).get("xyz_m", float("inf")) < .1
        result.update(sequence_fraction=fraction, sequence_percentage=100 * fraction,
                      preferred=fraction < .15, eligible=fraction <= .25,
                      hard_cutoff_index=(result["frame_count"] - 1) // 4,
                      reference_target_hit=reference_hit, target_hit=reference_hit and fraction <= .25)
        return result

    def _attempt(self, scene, index, variant, sampling_label):
        assert index <= (self._scene(scene)["frame_count"] - 1) // 4, "fit exceeds hard 25% cutoff"
        metadata = self._stage(scene, index)
        path = self.output / "attempts" / variant / f"{metadata['run_id']}.json"
        if path.exists():
            result = json.loads(path.read_text())
            labels = result.setdefault("sampling_labels", [])
            if sampling_label not in labels:
                labels.append(sampling_label)
            return self._eligibility(result)  # Existing fit artifacts stay immutable.
        if index == 0:
            original_path = self.original / "xyz" / variant / f"nuscenes-{scene}.json"
            result = json.loads(original_path.read_text())
            result["reused_first_frame_record"] = str(original_path)
        else:
            result = self.runner.run_one(metadata, variant)
        result.update(run_id=metadata["run_id"], frame_id=metadata["frame_id"], first_frame_only=False,
                      offset_seconds=metadata["offset_seconds"], sequence_fraction=metadata["sequence_fraction"],
                      sequence_index=index, frame_count=metadata["frame_count"],
                      initialization_policy="original scene first-camera anchor; independent per-frame fits; no warm start")
        result["scan_overlap_accepted"] = bool(result.get("scan_registration", {}).get("accepted", False))
        result["final_accepted"] = bool(result["accepted"])
        result["target_hit"] = bool(result["accepted"] and result.get("fresh_error", {}).get("xyz_m", float("inf")) < .1)
        result["sampling_labels"] = [sampling_label]
        self._eligibility(result)
        self._write(path, result)
        return result

    def search(self, scene, variant):
        count = self._scene(scene)["frame_count"]
        cutoff = (count - 1) // 4
        first = self._attempt(scene, 0, variant, "frame_zero")
        attempts = [first]
        grid_hit = 0 if first["target_hit"] else None
        if grid_hit is None:
            recorded = self.output / "recorded" / scene
            if not all((recorded / "lidar" / f"{index:03d}.bin").exists() for index in range(cutoff + 1)):
                self._fetch(scene, -2, cutoff)
            grid = list(range(10, cutoff + 1, 10))
            if not grid or grid[-1] != cutoff:
                grid.append(cutoff)
            for index in grid:
                attempt = self._attempt(scene, index, variant, "one_second_grid")
                attempts.append(attempt)
                if attempt["target_hit"]:
                    grid_hit = index
                    break
        if grid_hit != 0:
            by_index = {row["sequence_index"]: row for row in attempts}
            limit = grid_hit if grid_hit is not None else cutoff
            # Preload the immutable target before sharing this one-scene runner.
            # Each task constructs its own registrar and writes a distinct frame.
            if self.native_workers > 1:
                self.runner._map(self._stage(scene, 0)["map_path"])
            with ThreadPoolExecutor(max_workers=self.native_workers) as pool:
                for start in range(1, limit + 1, self.native_workers):
                    indices = list(range(start, min(limit + 1, start + self.native_workers)))
                    futures = {index: pool.submit(self._attempt, scene, index, variant, "bounded_native_dense_search") for index in indices if index not in by_index}
                    for index, future in futures.items():
                        by_index[index] = future.result()
                        attempts.append(by_index[index])
                    if any(by_index[index]["target_hit"] for index in indices):
                        break
        seen = {row["sequence_index"] for row in attempts}
        for path in (self.output / "attempts" / variant).glob(f"nuscenes-{scene}-f*.json"):
            prior = self._eligibility(json.loads(path.read_text()))
            if prior["eligible"] and prior["sequence_index"] not in seen:
                prior.setdefault("sampling_labels", ["previously_computed_eligible_frame"])
                attempts.append(prior)
        ordered = sorted(attempts, key=lambda row: row["sequence_index"])
        def earliest(key):
            matches = [row for row in ordered if row[key]]
            if not matches:
                return None
            row = matches[0]
            return {key: row[key] for key in ("frame_id", "sequence_index", "offset_seconds", "sequence_fraction", "sequence_percentage", "preferred", "eligible", "scan_overlap_accepted", "final_accepted", "target_hit", "fresh_error")}
        summary = dict(scene=scene, dataset="nuscenes", variant=variant, frame_count=count,
                       search_description="Later-frame reference-agreement test; frame zero already has accepted scan overlap",
                       initialization_policy="original scene initializer fixed across every attempted frame",
                       target="accepted XYZ reference-relative error < 0.1 m; not independent ground truth",
                       first_scan_overlap_acceptance=earliest("scan_overlap_accepted"),
                       first_final_acceptance=earliest("final_accepted"), earliest_tested_target_hit=earliest("target_hit"),
                       target_found=any(row["target_hit"] for row in ordered), first_successful_grid_frame=grid_hit,
                       hard_cutoff_index=cutoff, hard_cutoff_seconds=cutoff / 10,
                       preference="strictly before 15%; hard cutoff at or before 25% of native sequence span",
                       refinement_policy="every native frame from zero to first eligible hit or cutoff; settings unchanged",
                       attempted_indices=[row["sequence_index"] for row in ordered],
                       eligible_untested_indices=[index for index in range(cutoff + 1) if index not in {row["sequence_index"] for row in ordered}],
                       untested_indices=[index for index in range(count) if index not in {row["sequence_index"] for row in ordered}],
                       earliest_claim="earliest native-frame eligible success; every earlier native frame checked",
                       attempts=[{key: row.get(key) for key in ("frame_id", "sequence_index", "offset_seconds", "sequence_fraction", "sequence_percentage", "preferred", "eligible", "sampling_labels", "scan_overlap_accepted", "final_accepted", "target_hit", "fresh_error", "rejection_reasons", "exception")} for row in ordered])
        accepted = [row for row in ordered if row["final_accepted"]]
        best = min(accepted, key=lambda row: row["fresh_error"]["xyz_m"]) if accepted else None
        summary["best_tested_accepted"] = None if best is None else {key: best[key] for key in ("frame_id", "offset_seconds", "sequence_fraction", "sequence_percentage", "preferred", "eligible", "fresh_error")}
        historical = [self._eligibility(json.loads(path.read_text())) for path in (self.output / "attempts" / variant).glob(f"nuscenes-{scene}-f*.json")]
        summary["historical_ineligible_indices"] = sorted(row["sequence_index"] for row in historical if not row["eligible"])
        hit_index = summary["earliest_tested_target_hit"]["sequence_index"] if summary["target_found"] else cutoff
        assert set(range(hit_index + 1)).issubset({row["sequence_index"] for row in ordered})
        assert all(row["eligible"] for row in ordered)
        summary["all_native_frames_before_selected_hit_tested"] = True
        summary["eligible_window_fully_tested"] = not summary["eligible_untested_indices"]
        summary["earliest_eligible_hit_proven"] = summary["target_found"]
        summary["no_eligible_hit_proven"] = not summary["target_found"] and summary["eligible_window_fully_tested"]
        self._write(self.output / "summaries" / f"nuscenes-{scene}_{variant}.json", summary)
        hit = summary["earliest_tested_target_hit"]
        print("SEARCH", scene, variant, "earliest tested", None if hit is None else hit["frame_id"],
              "XYZ", None if hit is None else hit["fresh_error"]["xyz_m"], flush=True)
        return summary

    def verify(self):
        checked = 0
        index_rows = []
        for path in sorted((self.output / "attempts").glob("*/*.json")):
            row = json.loads(path.read_text())
            original = json.loads((self.original / "xyz" / row["variant"] / f"nuscenes-{row['scene']}.json").read_text())
            assert not row["first_frame_only"], path
            assert row["registration_config"] == original["registration_config"], path
            assert row["fit_config"] == original["fit_config"], path
            assert np.array_equal(row["initial_map_T_dataset"], original["initial_map_T_dataset"]), path
            assert np.array_equal(row["reference_map_T_dataset"], original["reference_map_T_dataset"]), path
            assert row["target_hit"] == (row["accepted"] and row.get("fresh_error", {}).get("xyz_m", float("inf")) < .1), path
            if "hard_cutoff_index" in row:
                assert row["sequence_index"] <= row["hard_cutoff_index"], path
            annotated = self._eligibility(row.copy())
            index_rows.append({**{key: annotated.get(key) for key in ("scene", "variant", "frame_id", "sequence_index", "offset_seconds", "sequence_percentage", "preferred", "eligible", "target_hit", "reference_target_hit", "accepted", "sampling_labels")},
                               "xyz_m": annotated.get("fresh_error", {}).get("xyz_m"), "artifact": str(path)})
            checked += 1
        self._write(self.output / "attempt_index.json", index_rows)
        self._write(self.output / "verification.json", dict(attempts_checked=checked,
                    original_initializers_preserved=True, original_references_preserved=True,
                    original_fit_settings_and_acceptance_gates_preserved=True,
                    target_hit_requires_final_acceptance=True, first_frame_only_false_on_new_records=True,
                    new_fit_hard_cutoff_assertion_passed=True,
                    historical_ineligible_artifacts_retained_unchanged=True))

    def secondary_bev(self, summaries):
        from scripts.investigation.first_frame_bev import GravityConstrainedRegistrar
        from src.localization.registration import PointCloudRegistrar
        for summary in summaries:
            selected = summary["earliest_tested_target_hit"]
            if selected is None or selected["sequence_index"] == 0:
                continue
            scene, variant = summary["scene"], summary["variant"]
            index = selected["sequence_index"]
            assert selected["eligible"] and index <= summary["hard_cutoff_index"]
            path = self.output / "bev_selected" / f"nuscenes-{scene}_{variant}_f{index:03d}.json"
            if path.exists():
                continue
            metadata = self._stage(scene, index)
            frame = dict(np.load(metadata["npz_path"]))
            source = source_points(frame, mask_dynamic=variant != "baseline")
            target = self.runner._map(metadata["map_path"])
            config = registration_config("nuscenes")
            fit_config = replace(config, refinement_voxel_sizes=(0.5, 0.25, 0.10), icp_max_iterations=100) if variant == "static_fine" else config
            fitted = GravityConstrainedRegistrar(fit_config, frame["dataset_T_lidar"]).register(source, target, frame["pipeline_initial_map_T_dataset"])
            assessed = PointCloudRegistrar(config).evaluate(source, target, fitted.map_T_source, frame["pipeline_initial_map_T_dataset"])
            error = pose_error(assessed.map_T_source, frame["reference_map_T_dataset"], frame["dataset_T_lidar"])
            self._write(path, dict(scene=scene, variant=variant, frame_id=metadata["frame_id"],
                                   offset_seconds=metadata["offset_seconds"], first_frame_only=False,
                                   sequence_percentage=selected["sequence_percentage"], preferred=selected["preferred"], eligible=True,
                                   selection="same frame selected by XYZ search; no independent BEV search",
                                   constraints="initializer vertical position/gravity imposed; agreement is not independent accuracy",
                                   registration_config=asdict(config), fit_config=asdict(fit_config),
                                   registration=asdict(assessed), error=error, accepted=assessed.accepted,
                                   target_hit=assessed.accepted and error["xyz_m"] < .1))

    def run(self, *, defer_finalization=False):
        summaries = []
        for scene in self.scenes:
            for variant in self.variants:
                summaries.append(self.search(scene, variant))
        if defer_finalization:
            return
        self._write(self.output / "summary.json", summaries)
        with (self.output / "summary.csv").open("w", newline="") as stream:
            fields = ["scene", "variant", "scan_overlap_offset_seconds", "final_acceptance_offset_seconds", "target_found", "target_frame", "target_offset_seconds", "target_sequence_fraction", "target_sequence_percentage", "target_preferred", "target_eligible", "target_xyz_m", "attempt_count", "best_tested_accepted_frame", "best_tested_accepted_xyz_m"]
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for row in summaries:
                hit = row["earliest_tested_target_hit"] or {}
                writer.writerow(dict(scene=row["scene"], variant=row["variant"],
                                     scan_overlap_offset_seconds=(row["first_scan_overlap_acceptance"] or {}).get("offset_seconds"),
                                     final_acceptance_offset_seconds=(row["first_final_acceptance"] or {}).get("offset_seconds"),
                                     target_found=row["target_found"], target_frame=hit.get("frame_id"), target_offset_seconds=hit.get("offset_seconds"),
                                     target_sequence_fraction=hit.get("sequence_fraction"), target_xyz_m=hit.get("fresh_error", {}).get("xyz_m"),
                                     target_sequence_percentage=hit.get("sequence_percentage"), target_preferred=hit.get("preferred"), target_eligible=hit.get("eligible"),
                                     attempt_count=len(row["attempted_indices"]),
                                     best_tested_accepted_frame=(row["best_tested_accepted"] or {}).get("frame_id"),
                                     best_tested_accepted_xyz_m=(row["best_tested_accepted"] or {}).get("fresh_error", {}).get("xyz_m")))
        self.verify()
        self.secondary_bev(summaries)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/frame-advance-localization/nuscenes")
    parser.add_argument("--scene", action="append")
    parser.add_argument("--variant", choices=("baseline", "static_fine"), action="append")
    parser.add_argument("--defer-finalization", action="store_true", help="Allow independent scene workers; aggregate after all workers finish")
    parser.add_argument("--native-workers", type=int, choices=(1, 2, 3, 4), default=1, help="Scheduling only; each fit retains its existing two-thread configuration")
    args = parser.parse_args()
    NuScenesFrameAdvance(args.output, args.scene or ["152", "164", "171", "200", "209", "359", "529", "916"], args.variant or ["baseline", "static_fine"], args.native_workers).run(defer_finalization=args.defer_finalization)

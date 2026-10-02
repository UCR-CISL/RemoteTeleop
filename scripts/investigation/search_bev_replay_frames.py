"""Resume an offline BEV replay-start search using unchanged registration gates.

Selection uses recorded-reference horizontal error, not an online confidence
signal. Every native frame through the 25% cutoff is checked; the smallest
accepted XY error below 0.1 m is selected. If none qualifies, replay uses frame
zero with the previous replay alignment. XYZ remains diagnostic.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import shutil
import sys
import time

for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[name] = "2"

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import numpy as np

from scripts.investigation.advance_cooperscene_frames import IndexedCooperFrameReader
from scripts.investigation.advance_nuscenes_frames import NuScenesFrameAdvance
from scripts.investigation.first_frame_bev import GravityConstrainedRegistrar, TwoWorkerTree
from scripts.investigation.first_frame_xyz import jsonable, map_proxy, pose_error, registration_config, source_points
from src.localization.enu_prior_fusion import EnuPriorFuser, EnuPriorFusionConfig
import src.localization.enu_prior_fusion as fusion_module
from src.localization.registration import PointCloudRegistrar


class BevReplayFrameSearch:
    """Own the resumable search, immutable source caches and replay manifests."""

    def __init__(self, args):
        self.args = args
        self.first = ROOT / "artifacts/first-frame-localization"
        self.previous = ROOT / "artifacts/frame-advance-localization"
        self.output = args.output.resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.entries = json.loads((self.first / "inputs/manifest.json").read_text())
        self.maps = {}
        self.nuscenes = NuScenesFrameAdvance(self.output / "nuscenes-cache", [], ["baseline"])
        fusion_module.cKDTree = TwoWorkerTree

    @staticmethod
    def write(path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(jsonable(value), indent=2, allow_nan=False) + "\n")
        temporary.replace(path)

    def target(self, map_path):
        if map_path not in self.maps:
            self.maps.clear()
            self.maps[map_path] = map_proxy(map_path)
        return self.maps[map_path]

    def metadata(self, entry, index, reader):
        run_id = entry["run_id"]
        if index == 0:
            return dict(entry)
        if entry["dataset"] == "nuscenes":
            directory = self.previous / "nuscenes/inputs" / f"{run_id}-f{index:03d}"
            if (directory / "metadata.json").exists():
                return json.loads((directory / "metadata.json").read_text())
            return self.nuscenes._stage(entry["scene"], index)
        directory = self.previous / "cooperscene/inputs" / f"{run_id}-f{index:06d}"
        if (directory / "metadata.json").exists():
            return json.loads((directory / "metadata.json").read_text())
        return reader.extract(index, self.output / "cooperscene-cache/inputs")

    def fit(self, entry, metadata, index):
        run_id, dataset = entry["run_id"], entry["dataset"]
        prior = self.first / "bev" / f"{run_id}_baseline.json"
        if index == 0 and prior.exists():
            result = json.loads(prior.read_text())
            result["reused_result_path"] = str(prior)
            return result
        frame = dict(np.load(metadata["npz_path"], allow_pickle=False))
        source = source_points(frame, mask_dynamic=dataset == "cooperscene")
        target = self.target(metadata["map_path"])
        initial = frame["pipeline_initial_map_T_dataset"]
        registrar = PointCloudRegistrar(registration_config(dataset))
        result = dict(run_id=metadata["run_id"], dataset=dataset, variant="baseline",
                      registration_config=asdict(registrar.config), source_count=len(source),
                      dynamic_mask_applied=dataset == "cooperscene", accepted=False)
        started = time.monotonic()
        try:
            if dataset == "cooperscene":
                prior = self.previous / "cooperscene/results/baseline" / f"{metadata['run_id']}.json"
                cached = json.loads(prior.read_text()) if prior.exists() else {}
                if "scan_registration" in cached:
                    scan = cached["scan_registration"]
                    result["reused_scan_path"] = str(prior)
                else:
                    scan = asdict(registrar.register(source, target, initial))
                fusion_config = EnuPriorFusionConfig(position_mode="bev")
                fusion = EnuPriorFuser(fusion_config).fuse(source, target,
                    np.asarray(scan["map_T_source"]), frame["reference_map_T_dataset"], frame["dataset_T_lidar"])
                registration = registrar.evaluate(source, target, fusion.map_T_source, initial)
                result.update(method="existing CooperScene BEV ENU fusion (weight 0.14)",
                              scan_registration=scan, upstream_scan_accepted=scan["accepted"],
                              fusion=asdict(fusion), fusion_config=asdict(fusion_config),
                              fusion_converged=fusion.converged,
                              constraints="Scan Z/roll/pitch fixed; 3D nearest neighbors; optimize XY and yaw")
            else:
                constrained = GravityConstrainedRegistrar(registrar.config, frame["dataset_T_lidar"])
                registration = constrained.register(source, target, initial)
                result.update(method="experimental gravity-constrained point-to-plane ICP; XYZ associations",
                              level_diagnostics=constrained.level_diagnostics,
                              constraints="Initial vertical pose and roll/pitch imposed; optimize dataset XY and yaw")
            result["registration"] = asdict(registration)
            result["error"] = pose_error(registration.map_T_source, frame["reference_map_T_dataset"], frame["dataset_T_lidar"])
            result["accepted"] = bool(registration.accepted and result.get("upstream_scan_accepted", True)
                                      and result.get("fusion_converged", True))
        except Exception as error:
            result["exception"] = f"{type(error).__name__}: {error}"
        result["runtime_seconds"] = time.monotonic() - started
        return result

    def attempt(self, entry, index, count, reader):
        run_id = entry["run_id"]
        path = self.output / "attempts" / f"{run_id}-f{index:06d}.json"
        if path.exists():
            return json.loads(path.read_text())
        metadata = self.metadata(entry, index, reader)
        result = self.fit(entry, metadata, index)
        result.update(original_run_id=run_id, frame_offset=index, frame_count=count,
                      frame_id=metadata["frame_id"], elapsed_seconds=float(metadata["timestamp_us"]) / 1e6,
                      sequence_percent=100 * index / (count - 1), preferred=100 * index < 15 * (count - 1),
                      eligible=4 * index <= count - 1, input_npz=metadata["npz_path"], map_path=metadata["map_path"],
                      optimizer_settings_changed=False, initialization_policy="original initializer retained; independent per-frame fits",
                      selection_metric="accepted recorded-reference XY error < 0.1 m; not independent ground truth")
        result["target_hit"] = bool(result["accepted"] and result.get("error", {}).get("xy_m", float("inf")) < .1 and result["eligible"])
        self.write(path, result)
        progress = {key: result.get(key) for key in ("original_run_id", "frame_offset", "sequence_percent", "accepted", "target_hit", "exception")}
        progress.update({key: result.get("error", {}).get(key) for key in ("xy_m", "xyz_m", "dz_m")})
        print(json.dumps(progress), flush=True)
        return result

    def summarize(self, entry, rows, count, complete):
        accepted = [row for row in rows if row["accepted"] and "error" in row]
        best = min(accepted, key=lambda row: row["error"]["xy_m"]) if accepted else None
        qualified = best is not None and best["target_hit"]
        selected = best if complete and qualified else None
        if complete and not qualified:
            frame = dict(np.load(entry["npz_path"], allow_pickle=False))
            original = next(row for row in rows if row["frame_offset"] == 0)
            inherited = frame["inherited_map_T_dataset"]
            selected = dict(original, fallback_existing_alignment=True, target_hit=False,
                            accepted=original.get("inherited_registration", {}).get("accepted"),
                            registration=dict(original.get("inherited_registration", {}), map_T_source=inherited),
                            error=pose_error(inherited, frame["reference_map_T_dataset"], frame["dataset_T_lidar"]),
                            transform_source="frame-zero inherited_map_T_dataset from previous replay; not a newly accepted BEV fit")
        preferred = [row for row in accepted if row["preferred"]]
        summary = dict(run_id=entry["run_id"], dataset=entry["dataset"], variant="baseline", localization_mode="bev",
                       frame_count=count, hard_cutoff_frame=(count - 1) // 4,
                       status=("qualified" if qualified else "fallback_frame_zero") if complete else "searching",
                       selection_metric="minimum accepted XY < 0.1 m within 25%; otherwise previous frame-zero replay alignment",
                       fallback_reason=None if qualified or not complete else "no_accepted_fit_below_0.1m" if best else "no_accepted_fit",
                       reference_privileged_selection=True, preferred_percent=15, hard_cutoff_percent=25,
                       attempted_frame_offsets=[row["frame_offset"] for row in rows],
                       eligible_window_fully_tested=complete, search_complete=complete,
                       selected=selected, best_accepted=best,
                       best_accepted_before_15_percent=min(preferred, key=lambda row: row["error"]["xy_m"]) if preferred else None,
                       render_ground_anchoring="required downstream; localization transform is not height-corrected")
        if selected:
            transform_path = self.output / "transforms" / f"{entry['run_id']}.json"
            self.write(transform_path, dict(map_T_dataset=selected["registration"]["map_T_source"],
                       run_id=entry["run_id"], frame_offset=selected["frame_offset"], frame_id=selected["frame_id"],
                       localization_mode="bev", error=selected["error"],
                       fallback_existing_alignment=selected.get("fallback_existing_alignment", False),
                       transform_source=selected.get("transform_source", "accepted BEV registration"),
                       source_result=str(self.output / "attempts" / f"{entry['run_id']}-f{selected['frame_offset']:06d}.json")))
            summary["replay_transform_path"] = str(transform_path)
            summary["replay_start_frame"] = selected["frame_offset"]
        self.write(self.output / "sequences" / f"{entry['run_id']}.json", summary)
        sequences = [json.loads(path.read_text()) for path in sorted((self.output / "sequences").glob("*.json"))]
        self.write(self.output / "selections.json", dict(schema_version=1, localization_mode="bev",
                   policy="minimum accepted XY <0.1 m within 25%; record <15% comparison; no qualifying fit falls back to frame0 inherited alignment",
                   planned_run_ids=[entry["run_id"] for entry in self.entries],
                   sequences=sequences, complete=len(sequences) == len(self.entries) and all(row["search_complete"] for row in sequences)))

    def run(self):
        for entry in self.entries:
            if self.args.run_id and entry["run_id"] not in self.args.run_id:
                continue
            reader = None
            if entry["dataset"] == "cooperscene":
                reader = IndexedCooperFrameReader(self.first / "inputs", entry["run_id"], self.args.trajectory_root)
                count = len(reader.rows)
            else:
                # Copy recorded cache; staging never writes into prior evaluations.
                old = self.previous / "nuscenes/recorded" / entry["scene"]
                new = self.nuscenes.output / "recorded" / entry["scene"]
                if not new.exists():
                    shutil.copytree(old, new)
                count = self.nuscenes._scene(entry["scene"])["frame_count"]
            rows = []
            try:
                for index in range((count - 1) // 4 + 1):
                    if self.args.max_frames is not None and index >= self.args.max_frames:
                        break
                    row = self.attempt(entry, index, count, reader)
                    rows.append(row)
                    complete = index == (count - 1) // 4
                    self.summarize(entry, rows, count, complete)
                    if complete:
                        break
            finally:
                if reader:
                    reader.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/bev-frame-search")
    parser.add_argument("--trajectory-root", type=Path, default=Path("/tmp/RemoteTeleop-mcap-replay/artifacts/proxy-batch/xyz-results"))
    parser.add_argument("--run-id", action="append")
    parser.add_argument("--max-frames", type=int, help="Optional smoke-test bound; unfinished sequences remain searching")
    BevReplayFrameSearch(parser.parse_args()).run()

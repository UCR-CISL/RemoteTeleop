#!/usr/bin/env python3
"""Run a persistent, sequential queue of offline MCAP replays with proxy assets."""
from __future__ import annotations

import argparse
from dataclasses import fields
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shlex
import subprocess
import sys
import time
from typing import Callable

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.deployment.run_remote_teleop_sam_simulation import (
    SamLaunchOptions, SamRemoteTeleopSimulationLauncher,
)
from scripts.deployment.run_remote_teleop_simulation import LaunchError, LaunchOptions
from src.common.config import RemoteTeleopConfig


class ReplayInputReadiness:
    """Check completed MCAPs and referenced files without starting replay services."""

    MCAP_PROBE = """
import json, sys
from pathlib import Path
from mcap.reader import make_reader
spec = json.loads(sys.argv[1])
p = Path(spec['mcap'])
before = p.stat()
assert before.st_size > 16, 'MCAP empty'
with p.open('rb') as stream:
    stream.seek(-8, 2)
    assert stream.read() == b'\\x89MCAP0\\r\\n', 'MCAP footer incomplete'
    stream.seek(0)
    summary = make_reader(stream).get_summary()
assert summary is not None and summary.statistics is not None, 'MCAP summary missing'
counts = {}
for channel_id, count in summary.statistics.channel_message_counts.items():
    topic = summary.channels[channel_id].topic
    counts[topic] = counts.get(topic, 0) + count
for agent in spec['agents']:
    assert counts.get('/' + agent + '_Camera') == spec['expected'], 'camera count mismatch: ' + agent
assert counts.get('/bbox_global') in (spec['expected'], 2 * spec['expected']), 'global bbox count mismatch'
after = p.stat()
assert (before.st_size, before.st_mtime_ns) == (after.st_size, after.st_mtime_ns), 'MCAP changing'
for reference in spec['references']:
    assert Path(reference).is_file() and Path(reference).stat().st_size > 0, 'missing reference: ' + reference
"""

    def __init__(self, *, runner: Callable = subprocess.run):
        self.runner = runner
        self.last_error = ""

    def ready(self, options: LaunchOptions, sam: SamLaunchOptions) -> bool:
        config = RemoteTeleopConfig.from_yaml(options.config_path)
        vehicle, remote = config.vehicle, config.remote
        references = [str(vehicle.repo_path / value) for value in (
            options.nuscenes_localization_transform, options.cooperscene_localization_transform,
            options.cooperscene_stored_localization_transform, options.localization_reference_transform,
        ) if value is not None]
        spec = dict(mcap=str(vehicle.repo_path / options.mcap_path), expected=options.expected_samples + options.mcap_start_sequence,
                    agents=["1", "2", "3"] if options.source_mode == "mcap" else [sam.mcap_agent],
                    references=references)
        python = shlex.split(sam.vehicle_python)
        if not python:
            raise ValueError("vehicle_python must not be empty")
        if not PurePosixPath(python[0]).is_absolute():
            python[0] = str(vehicle.repo_path / python[0])
        probes = [(vehicle, shlex.join([*python, "-c", self.MCAP_PROBE, json.dumps(spec)])),
                  (remote, "test -s " + shlex.quote(str(remote.repo_path / options.splat_path)))]
        if options.ground_plane is not None:
            probes.append((remote, "test -s " + shlex.quote(str(remote.repo_path / options.ground_plane))))
        for host, probe in probes:
            command = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10",
                       "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=2",
                       f"{host.user}@{host.address}", probe]
            try:
                result = self.runner(command, capture_output=True, text=True, check=False, timeout=30)
            except (OSError, subprocess.TimeoutExpired) as error:
                self.last_error = str(error)
                return False
            if result.returncode:
                self.last_error = (result.stderr or result.stdout or "input file unavailable").strip()[-600:]
                return False
        self.last_error = ""
        return True


class ReplayBatch:
    """Own queue validation, exclusive execution, and durable attempt history.

    Successful runs are skipped on resume. Failed/interrupted runs get fresh local
    and remote attempt paths. Existing launcher cleanup handles normal exceptions
    and interrupts; after an unclean process kill, clean remote services first.
    """

    def __init__(self, manifest: Path, output_root: Path, *, launcher_factory: Callable = SamRemoteTeleopSimulationLauncher, output: Callable[[str], None] = print, readiness: ReplayInputReadiness | None = None, sleep: Callable = time.sleep, clock: Callable = time.monotonic):
        self.manifest_path = manifest
        self.manifest = json.loads(manifest.read_text())
        self.output_root = output_root.resolve()
        self.launcher_factory = launcher_factory
        self.output = output
        self.readiness = readiness or ReplayInputReadiness()
        self.sleep, self.clock = sleep, clock
        self.fingerprint = hashlib.sha256(json.dumps(self.manifest, sort_keys=True).encode()).hexdigest()
        if self.manifest.get("version") != 1:
            raise ValueError("batch manifest version must be 1")
        self.runs = self.manifest.get("runs", [])
        if not self.runs:
            raise ValueError("batch manifest must contain at least one run")
        ids = [run["id"] for run in self.runs]
        if any(not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", value) for value in ids):
            raise ValueError("run IDs must be safe path components")
        if len(set(ids)) != len(ids):
            raise ValueError("run IDs must be unique")
        # Validate every run before any network or state mutation.
        for run in self.runs:
            self._options(run, 1)

    def _options(self, run: dict, attempt: int) -> tuple[LaunchOptions, SamLaunchOptions]:
        values = {**self.manifest.get("defaults", {}), **run}
        run_id = values.pop("id")
        dataset = values.pop("dataset")
        scene = str(values.pop("scene"))
        agent = str(values.pop("agent", "1"))
        if dataset not in {"nuscenes", "cooperscene"}:
            raise ValueError(f"unsupported dataset: {dataset}")
        if agent not in {"1", "2", "3"}:
            raise ValueError("camera agent must be 1, 2, or 3")
        namespace = hashlib.sha256(str(self.output_root).encode()).hexdigest()[:10]
        remote_root = PurePosixPath(values.pop("remote_output_root", f"artifacts/teleop-batch/{self.output_root.name}-{namespace}"))
        if ".." in remote_root.parts:
            raise ValueError("remote_output_root cannot contain parent traversal")
        suffix = PurePosixPath(run_id) / f"attempt-{attempt:03d}"
        values["mcap_path"] = PurePosixPath(values.pop("mcap"))
        values["splat_path"] = PurePosixPath(values.pop("splat"))
        values["output_root"] = remote_root / suffix
        values["local_artifacts"] = self.output_root / suffix
        values.setdefault("config_path", "cfg/alien3_lambda.yaml")
        values["config_path"] = Path(values["config_path"])
        values.setdefault("ros_sidecar", "remote_teleop_ros_mcap")
        values.setdefault("remote_python", ".venv/bin/python")
        source_mode = "nuscenes_mcap" if dataset == "nuscenes" else "mcap"
        values.setdefault("source_mode", source_mode)
        if values["source_mode"] != source_mode:
            raise ValueError(f"{dataset} queue runs require source_mode={source_mode!r}")
        values.setdefault("scene_generation", f"nuscenes:{scene}" if dataset == "nuscenes" else f"cooperscene:train:{scene}:{agent}")
        if "expected_samples" not in values:
            raise ValueError(f"{run_id}: expected_samples is required")
        allowed = {field.name for field in fields(LaunchOptions)}
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"{run_id}: unknown options: {sorted(unknown)}")
        for name in ("ground_plane", "localization_reference_transform", "nuscenes_scene_root", "nuscenes_localization_transform", "cooperscene_localization_transform", "cooperscene_stored_localization_transform", "bevfusion_runtime_dir", "bevfusion_checkpoint", "bevfusion_config", "bevfusion_native_dir"):
            if values.get(name) is not None:
                values[name] = PurePosixPath(values[name])
        if "camera_source_size" in values:
            values["camera_source_size"] = tuple(values["camera_source_size"])
        return LaunchOptions(**values), SamLaunchOptions(asset_proxy=True, mcap_agent=agent, vehicle_python=values.get("vehicle_python", ".venv/bin/python"))

    def _load_state(self) -> dict:
        path = self.output_root / "state.json"
        if path.exists():
            state = json.loads(path.read_text())
            if state.get("manifest_sha256") != self.fingerprint:
                raise ValueError("manifest changed; use a new output root")
            return state
        return {"version": 1, "manifest_sha256": self.fingerprint, "runs": {run["id"]: {"status": "queued", "attempts": []} for run in self.runs}}

    def _save_state(self, state: dict) -> None:
        temp = self.output_root / "state.json.tmp"
        with temp.open("w") as stream:
            json.dump(state, stream, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        temp.replace(self.output_root / "state.json")

    def run(self, *, dry_run: bool = False, continue_on_error: bool = False, retry_interrupted: bool = False, max_runs: int | None = None, wait_for_inputs: bool = False, input_timeout_seconds: float = 21600) -> int:
        if input_timeout_seconds <= 0:
            raise ValueError("input_timeout_seconds must be positive")
        if max_runs is not None and max_runs <= 0:
            raise ValueError("max_runs must be positive")
        if dry_run:
            state = self._load_state()
            selected = 0
            for run in self.runs:
                entry = state["runs"][run["id"]]
                if entry["status"] == "succeeded":
                    continue
                if max_runs is not None and selected >= max_runs:
                    break
                selected += 1
                options, sam = self._options(run, len(entry["attempts"]) + 1)
                self.output(f"Run {run['id']}: {options.local_artifacts}")
                self.launcher_factory(options, sam, dry_run=True, output=self.output).run()
            return 0
        self.output_root.mkdir(parents=True, exist_ok=True)
        with (self.output_root / ".queue.lock").open("a") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise LaunchError("another process owns this queue") from error
            return self._run_locked(continue_on_error, retry_interrupted, max_runs, wait_for_inputs, input_timeout_seconds)

    def _run_locked(self, continue_on_error: bool, retry_interrupted: bool, max_runs: int | None = None, wait_for_inputs: bool = False, input_timeout_seconds: float = 21600) -> int:
        state = self._load_state()
        interrupted = [run_id for run_id, entry in state["runs"].items() if entry["status"] == "running"]
        if interrupted and not retry_interrupted:
            raise LaunchError(
                "interrupted runs may still own remote services: " + ", ".join(interrupted)
                + "; clean up their Lambda/Alien3 processes, then resume with --retry-interrupted"
            )
        self._save_state(state)
        failed = False
        selected = 0
        for run in self.runs:
            entry = state["runs"][run["id"]]
            if entry["status"] == "succeeded":
                self.output(f"Skipping successful run {run['id']}")
                continue
            if max_runs is not None and selected >= max_runs:
                break
            selected += 1
            if entry["status"] == "running":
                entry["attempts"][-1].update(status="failed", error="previous queue process interrupted")
            attempt_number = len(entry["attempts"]) + 1
            options, sam = self._options(run, attempt_number)
            if wait_for_inputs:
                entry["status"] = "queued"
                self._save_state(state)
                deadline = self.clock() + input_timeout_seconds
                while not self.readiness.ready(options, sam):
                    remaining = deadline - self.clock()
                    if remaining <= 0:
                        raise LaunchError(f"timed out waiting for inputs for {run['id']}: {self.readiness.last_error}")
                    self.output(f"Waiting for inputs for {run['id']}: {self.readiness.last_error}")
                    self.sleep(min(30, remaining))
            attempt = {"number": attempt_number, "status": "running", "started_at": datetime.now(timezone.utc).isoformat(), "local_artifacts": str(options.local_artifacts), "remote_output_root": str(options.output_root)}
            entry["attempts"].append(attempt)
            entry["status"] = "running"
            self._save_state(state)
            self.output(f"Running {run['id']} (attempt {attempt_number})")
            try:
                result = self.launcher_factory(options, sam, output=self.output).run()
            except BaseException as error:
                attempt.update(status="failed", error=f"{type(error).__name__}: {error}")
                entry["status"] = "failed"
                failed = True
                self.output(f"Failed {run['id']}: {error}")
                if not isinstance(error, Exception):
                    raise
            else:
                attempt.update(status="succeeded", result=str(result))
                entry["status"] = "succeeded"
            finally:
                attempt["finished_at"] = datetime.now(timezone.utc).isoformat()
                self._save_state(state)
            if entry["status"] == "failed" and not continue_on_error:
                break
        return 1 if failed else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--wait-for-inputs", action="store_true", help="wait for complete remote MCAPs and referenced files before starting each run")
    parser.add_argument("--input-timeout-seconds", type=float, default=21600, help="maximum input wait per run (default: six hours)")
    parser.add_argument("--max-runs", type=int, help="Run at most this many pending entries, then leave the rest queued")
    parser.add_argument("--continue-on-error", action="store_true")
    parser.add_argument("--retry-interrupted", action="store_true", help="retry stale running entries after manually cleaning up their remote services")
    args = parser.parse_args(argv)
    try:
        return ReplayBatch(args.manifest, args.output_root).run(dry_run=args.dry_run, continue_on_error=args.continue_on_error, retry_interrupted=args.retry_interrupted, max_runs=args.max_runs, wait_for_inputs=args.wait_for_inputs, input_timeout_seconds=args.input_timeout_seconds)
    except (ValueError, KeyError, OSError, LaunchError) as error:
        parser.exit(1, f"batch error: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Run reproducible scene-0061 policy comparisons sequentially on one GPU."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time


@dataclass(frozen=True)
class Policy:
    prompt_mode: str
    mask_policy: str
    priority_policy: str
    coalesce_ms: float
    stage1_steps: int | None = None
    stage2_steps: int | None = None
    pointmap_cache_size: int = 0


POLICIES = {
    "serial_baseline": Policy("serial_grounding", "all_visible", "quality", 0.0),
    "batch_only": Policy("interactive_batch", "all_visible", "quality", 0.0),
    "batch_new_tracks": Policy("interactive_batch", "new_tracks", "quality", 0.0),
    "deadline_750": Policy("interactive_batch", "new_tracks", "deadline", 750.0),
    "deadline_1000": Policy("interactive_batch", "new_tracks", "deadline", 1_000.0),
    "deadline_750_steps12": Policy(
        "interactive_batch", "new_tracks", "deadline", 750.0, 12, 12
    ),
    "deadline_750_steps8_12": Policy(
        "interactive_batch", "new_tracks", "deadline", 750.0, 8, 12
    ),
    "deadline_750_steps8": Policy(
        "interactive_batch", "new_tracks", "deadline", 750.0, 8, 8
    ),
    "deadline_750_steps8_cache": Policy(
        "interactive_batch", "new_tracks", "deadline", 750.0, 8, 8, 2
    ),
}


def _gpu_used_mib() -> float | None:
    result = subprocess.run(
        [
            "nvidia-smi",
            "--query-gpu=memory.used",
            "--format=csv,noheader,nounits",
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if result.returncode != 0:
        return None
    try:
        return float(result.stdout.strip().splitlines()[0])
    except (IndexError, ValueError):
        return None


def _wait_for_gpu(max_used_mib: float, timeout_seconds: float) -> None:
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        used = _gpu_used_mib()
        if used is not None and used <= max_used_mib:
            return
        print(
            f"waiting for GPU: used={used!r} MiB, threshold={max_used_mib:.0f} MiB",
            flush=True,
        )
        time.sleep(30.0)
    raise TimeoutError("GPU did not become available before experiment timeout")


def _run_policy(
    *,
    policy_name: str,
    policy: Policy,
    dataroot: Path,
    version: str,
    scene: str,
    output_root: Path,
    python: Path,
) -> int:
    run_root = output_root / policy_name
    run_root.mkdir(parents=True, exist_ok=False)
    command = [
        str(python),
        "-m",
        "src.realtime.supervisor",
        "--dataroot",
        str(dataroot),
        "--version",
        version,
        "--scene",
        scene,
        "--output-root",
        str(run_root),
        "--python",
        str(python),
        "--fp16",
        "--no-viewer",
        "--sam3-prompt-mode",
        policy.prompt_mode,
        "--sam3-mask-policy",
        policy.mask_policy,
        "--sam3d-priority-policy",
        policy.priority_policy,
        "--sam3d-coalesce-ms",
        str(policy.coalesce_ms),
        "--sam3d-pointmap-cache-size",
        str(policy.pointmap_cache_size),
    ]
    if policy.stage1_steps is not None:
        command.extend(["--sam3d-stage1-inference-steps", str(policy.stage1_steps)])
    if policy.stage2_steps is not None:
        command.extend(["--sam3d-stage2-inference-steps", str(policy.stage2_steps)])
    environment = dict(os.environ)
    environment["MAX_JOBS"] = "2"
    environment["PYTORCH_CUDA_ALLOC_CONF"] = "expandable_segments:True"
    print(f"running {policy_name}: {' '.join(command)}", flush=True)
    with (run_root / "supervisor.log").open("w", encoding="utf-8") as stream:
        result = subprocess.run(
            command,
            env=environment,
            stdout=stream,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
    if result.returncode == 0:
        summary_path = run_root / "summary.json"
        subprocess.run(
            [
                str(python),
                "scripts/analyze_realtime_run.py",
                str(run_root),
                "--json-output",
                str(summary_path),
            ],
            check=True,
        )
    return result.returncode


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataroot", type=Path, required=True)
    parser.add_argument("--version", default="v1.0-mini")
    parser.add_argument("--scene", default="scene-0061")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("artifacts/realtime-policy-experiments")
        / datetime.now().strftime("%Y%m%d-%H%M%S"),
    )
    parser.add_argument("--python", type=Path, default=Path(".venv/bin/python"))
    parser.add_argument(
        "--policies",
        nargs="+",
        choices=tuple(POLICIES),
        default=tuple(POLICIES),
    )
    parser.add_argument("--max-gpu-used-mib", type=float, default=4_096.0)
    parser.add_argument("--gpu-wait-timeout-seconds", type=float, default=21_600.0)
    args = parser.parse_args()
    args.output_root.mkdir(parents=True, exist_ok=False)
    results = {}
    for policy_name in args.policies:
        _wait_for_gpu(args.max_gpu_used_mib, args.gpu_wait_timeout_seconds)
        try:
            return_code = _run_policy(
                policy_name=policy_name,
                policy=POLICIES[policy_name],
                dataroot=args.dataroot,
                version=args.version,
                scene=args.scene,
                output_root=args.output_root,
                python=args.python.resolve(),
            )
        except Exception as error:
            results[policy_name] = {"status": "error", "error": str(error)}
            print(f"{policy_name} failed: {error}", file=sys.stderr, flush=True)
            continue
        results[policy_name] = {
            "status": "complete" if return_code == 0 else "failed",
            "return_code": return_code,
        }
    (args.output_root / "experiment_status.json").write_text(
        json.dumps(results, indent=2) + "\n", encoding="utf-8"
    )
    summaries = {}
    for policy_name, result in results.items():
        summary_path = args.output_root / policy_name / "summary.json"
        if result.get("status") == "complete" and summary_path.is_file():
            summaries[policy_name] = json.loads(summary_path.read_text(encoding="utf-8"))
    if not summaries:
        raise SystemExit(1)
    recommended = max(
        summaries,
        key=lambda name: (
            int(summaries[name].get("mesh_ready_before_track_exit", 0)),
            float(summaries[name].get("deadline_hit_rate", 0.0)),
            -float(summaries[name].get("first_detection_to_mesh_ms_mean") or 1e12),
            -float(summaries[name].get("sam3", {}).get("inference_ms_mean") or 1e12),
        ),
    )
    (args.output_root / "comparison.json").write_text(
        json.dumps(
            {"recommended_policy": recommended, "runs": summaries},
            indent=2,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()

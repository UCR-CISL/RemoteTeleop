#!/usr/bin/env python3
"""Run the CooperScene model-residency/precision matrix sequentially."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.realtime.benchmark_summary import summarize_experiment


@dataclass(frozen=True)
class Variant:
    name: str
    arguments: tuple[str, ...]


VARIANTS = (
    Variant("take_turns_fp16", ("--runtime-mode", "take_turns", "--fp16")),
    Variant(
        "sam3_cpu_sam3d_gpu",
        ("--sam3-device", "cpu", "--sam3d-device", "cuda:0", "--fp16"),
    ),
    Variant(
        "sam3_gpu_sam3d_cpu",
        ("--sam3-device", "cuda", "--sam3d-device", "cpu", "--fp16"),
    ),
    Variant(
        "both_gpu_nf4",
        ("--sam3-precision", "nf4", "--sam3d-precision", "nf4"),
    ),
    Variant(
        "sam3_fp16_sam3d_nf4",
        ("--sam3-precision", "fp16", "--sam3d-precision", "nf4"),
    ),
    Variant(
        "both_gpu_nf4_steps12",
        (
            "--sam3-precision", "nf4", "--sam3d-precision", "nf4",
            "--sam3d-stage1-inference-steps", "12",
            "--sam3d-stage2-inference-steps", "12",
        ),
    ),
    Variant(
        "both_gpu_nf4_render4",
        (
            "--sam3-precision", "nf4", "--sam3d-precision", "nf4",
            "--render-downsample", "4",
        ),
    ),
    Variant(
        "both_gpu_nf4_steps12_render4",
        (
            "--sam3-precision", "nf4", "--sam3d-precision", "nf4",
            "--sam3d-stage1-inference-steps", "12",
            "--sam3d-stage2-inference-steps", "12",
            "--render-downsample", "4",
        ),
    ),
    Variant(
        "sam3_fp16_sam3d_nf4_steps12_render4",
        (
            "--sam3-precision", "fp16", "--sam3d-precision", "nf4",
            "--sam3d-stage1-inference-steps", "12",
            "--sam3d-stage2-inference-steps", "12",
            "--render-downsample", "4",
        ),
    ),
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--python", type=Path, default=Path(".venv/bin/python"))
    parser.add_argument("--timeout-seconds", type=float, default=3600.0)
    parser.add_argument("--variants", nargs="*", choices=tuple(item.name for item in VARIANTS))
    args = parser.parse_args()
    selected = set(args.variants or (item.name for item in VARIANTS))
    root = args.output_root.resolve()
    root.mkdir(parents=True, exist_ok=True)
    matrix_path = root / "matrix.json"
    records: list[dict[str, object]] = []
    if matrix_path.exists():
        records = json.loads(matrix_path.read_text(encoding="utf-8"))
    for variant in VARIANTS:
        if variant.name not in selected:
            continue
        output = root / variant.name
        command = _base_command(args.python, output) + list(variant.arguments)
        started = time.time()
        record: dict[str, object] = {
            "variant": variant.name,
            "command": command,
            "started_at": datetime.now(timezone.utc).isoformat(),
        }
        log_path = root / f"{variant.name}.log"
        environment = dict(os.environ)
        environment.setdefault("MAX_JOBS", "2")
        environment.setdefault("MPLCONFIGDIR", "/tmp/remote-teleop-mpl")
        try:
            with log_path.open("w", encoding="utf-8") as log:
                result = subprocess.run(
                    command,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    text=True,
                    env=environment,
                    timeout=args.timeout_seconds,
                    check=False,
                )
            record["returncode"] = result.returncode
        except subprocess.TimeoutExpired:
            record["returncode"] = None
            record["controller_error"] = "timeout"
        except Exception as error:
            record["returncode"] = None
            record["controller_error"] = f"{type(error).__name__}: {error}"
        record["wall_seconds"] = time.time() - started
        record["finished_at"] = datetime.now(timezone.utc).isoformat()
        record["summary"] = summarize_experiment(output, variant=variant.name).to_dict()
        records.append(record)
        matrix_path.write_text(json.dumps(records, indent=2), encoding="utf-8")
    failed = sum(record.get("returncode") != 0 for record in records)
    print(f"completed {len(records)} variants ({failed} nonzero); {matrix_path}")


def _base_command(python: Path, output: Path) -> list[str]:
    return [
        str(python), "-m", "src.realtime.supervisor",
        "--source", "cooperscene",
        "--dataroot", "/mnt/bcc-data/data/CooperScene",
        "--split", "train", "--scene", "1", "--agent", "1",
        "--splat", "data/riverside_r3.spz",
        "--localization-transform",
        "artifacts/cooperscene_single_agent_coarse_cuda/localization.json",
        "--overlap-manifest",
        "artifacts/cooperscene_train1_agent1_probe/render_manifest.json",
        "--output-root", str(output),
        "--sam3d-config", "checkpoints/hf/pipeline.yaml",
    ]


if __name__ == "__main__":
    main()

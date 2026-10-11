"""Run independent take searches concurrently and refresh one combined report."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = Path(__file__).resolve().parent


def collect(output):
    summaries = {}
    for path in sorted(output.glob("take*/take*-vehicle*/summary.json")):
        summaries[path.parent.name] = json.loads(path.read_text())
    if summaries:
        temp = output / "summary.json.tmp"
        temp.write_text(json.dumps(summaries, indent=2) + "\n")
        temp.replace(output / "summary.json")
        subprocess.run(
            [sys.executable, str(SCRIPTS / "search_report.py"), str(output)], check=True
        )
    return summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    if args.workers < 1:
        parser.error("--workers must be positive")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if not args.resume and any(output.iterdir()):
        parser.error("Use a new output directory or explicit --resume")
    env = dict(
        os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1"
    )

    def run(take):
        command = [
            sys.executable,
            str(SCRIPTS / "frame_search.py"),
            "--output",
            str(output / f"take{take:02d}"),
            "--takes",
            str(take),
        ]
        if args.resume:
            command.append("--resume")
        with (output / f"take{take:02d}.log").open("a") as log:
            result = subprocess.run(
                command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT
            )
        return result.returncode

    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {take: pool.submit(run, take) for take in range(1, 15)}
        while True:
            summary = collect(output)
            codes = {str(t): f.result() for t, f in futures.items() if f.done()}
            status = dict(
                completed_take_processes=codes,
                requested_takes=14,
                observed_sequences=len(summary),
                completed_sequences=sum(s["complete"] for s in summary.values()),
                examined_frames=sum(s["examined_frames"] for s in summary.values()),
                expected_frames=18492,
                refreshed_at_unix=time.time(),
            )
            temp = output / "status.json.tmp"
            temp.write_text(json.dumps(status, indent=2) + "\n")
            temp.replace(output / "status.json")
            print(json.dumps(status), flush=True)
            if len(codes) == len(futures):
                if any(codes.values()):
                    raise SystemExit("Some take searches failed; inspect per-take logs")
                break
            time.sleep(30)


if __name__ == "__main__":
    main()

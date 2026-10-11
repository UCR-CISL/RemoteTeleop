"""Offline full-sequence BEV/XYZ search against the retained CooperScene recipe."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys
import time

for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(name, "1")
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

import numpy as np
from scripts.investigation.cooperscene_alignment.common import (
    DEFAULT_PRIOR,
    file_digest,
)


def serializable(value):
    if isinstance(value, dict):
        return {k: serializable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [serializable(v) for v in value]
    if isinstance(value, np.ndarray):
        return serializable(value.tolist())
    if isinstance(value, np.generic):
        return serializable(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def write_json(path, value):
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(serializable(value), indent=2, allow_nan=False) + "\n")
    temp.replace(path)


def summarize(records, total):
    result = dict(
        examined_frames=len(records),
        total_frames=total,
        complete=len(records) == total,
        methods={},
    )
    for method, metric in (("bev", "xy_m"), ("xyz", "xyz_m")):
        accepted = [r for r in records if r["methods"][method].get("accepted")]

        def candidate(r):
            m = r["methods"][method]
            return dict(
                frame=r["frame"],
                error=m["error"],
                registration=m["registration"],
                accepted=True,
            )

        best = min(
            accepted, key=lambda r: r["methods"][method]["error"][metric], default=None
        )
        first = next(
            (r for r in accepted if r["methods"][method]["error"][metric] < 0.1), None
        )
        best_xyz = min(
            accepted, key=lambda r: r["methods"][method]["error"]["xyz_m"], default=None
        )
        result["methods"][method] = dict(
            accuracy_metric=metric,
            strict_accuracy_threshold_m=0.1,
            accepted_frames=len(accepted),
            best_accepted=candidate(best) if best else None,
            earliest_qualifying=candidate(first) if first else None,
            best_accepted_xyz=candidate(best_xyz) if best_xyz else None,
            frame0=next(
                (
                    r["methods"][method]
                    for r in records
                    if r["frame"]["frame_index"] == 0
                ),
                None,
            ),
            fallback=(
                "frame0_recipe_prior_not_qualified"
                if not first and len(records) == total
                else None
            ),
        )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--prior", type=Path, default=DEFAULT_PRIOR)
    parser.add_argument(
        "--map",
        type=Path,
        default=Path("/home/jyue86/Downloads/riverside_for_justin/riverside_iowa.ply"),
    )
    parser.add_argument("--takes", type=int, nargs="+", default=list(range(1, 15)))
    parser.add_argument("--agents", type=int, nargs="+", default=[1, 2, 3])
    parser.add_argument(
        "--indices",
        type=int,
        nargs="+",
        help="Explicit pilot indices; omitted means every native frame",
    )
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    if args.indices is not None and (
        args.indices != sorted(set(args.indices)) or min(args.indices) < 0
    ):
        parser.error("--indices must be unique, nonnegative, and chronological")
    if len(set(args.takes)) != len(args.takes) or len(set(args.agents)) != len(
        args.agents
    ):
        parser.error("Duplicate takes or agents are not allowed")
    from scripts.investigation.cooperscene_alignment.search_inputs import (
        CooperSceneSearchSequence,
    )
    from scripts.investigation.cooperscene_alignment.search_evaluator import (
        SearchEvaluator,
    )
    from scripts.investigation.cooperscene_alignment.evaluation_utils import (
        registration_config,
    )
    from src.localization.enu_prior_fusion import EnuPriorFusionConfig

    args.output.mkdir(parents=True, exist_ok=True)
    recipe = (
        args.prior / "resolution/final-profile/recipe.json"
        if args.prior.is_dir()
        else args.prior
    )
    protocol = dict(
        recipe=str(recipe),
        recipe_sha256=file_digest(recipe),
        map=str(args.map),
        map_sha256=file_digest(args.map),
        takes=args.takes,
        agents=args.agents,
        indices=args.indices,
        map_min_opacity=0.0,
        map_stride=1,
        registration=asdict(registration_config("cooperscene")),
        fusion={
            m: asdict(EnuPriorFusionConfig(position_mode=m)) for m in ("bev", "xyz")
        },
        source="Full finite processed LiDAR scan; annotation vehicle boxes removed with 0.2m padding",
        reference="Retained recipe applied to interpolated raw GNSS/INS at LiDAR time",
        initialization="Each frame independently initialized from the same recipe reference; no warm start",
        prior="Recipe reference is ALSO the fusion prior. Initial reference error is zero by construction.",
        independent_ground_truth=False,
        reference_assisted_offline_selection=True,
        search="Every native frame chronologically; no percentage cutoff; retain best accepted and earliest qualifying",
        accuracy={"bev": "XY < 0.1m", "xyz": "XYZ < 0.1m"},
        acceptance="Shared scan accepted AND fusion converged AND final original registration gates accepted",
        fallback="frame0_recipe_prior_not_qualified if no qualifying candidate at full exhaustion",
        code_sha256={
            p.name: file_digest(p)
            for p in [
                Path(__file__),
                Path(__file__).with_name("search_inputs.py"),
                Path(__file__).with_name("search_evaluator.py"),
                Path(__file__).with_name("common.py"),
                ROOT / "src/localization/cooperscene_geodetic.py",
                Path(__file__).with_name("evaluation_utils.py"),
                ROOT / "src/localization/cooperscene.py",
                ROOT / "src/localization/cooperscene_reference.py",
                ROOT / "src/localization/registration.py",
                ROOT / "src/localization/enu_prior_fusion.py",
                ROOT / "src/localization/gaussian_map.py",
            ]
        },
    )
    protocol = serializable(protocol)
    path = args.output / "protocol.json"
    if path.exists():
        if not args.resume or json.loads(path.read_text()) != protocol:
            raise ValueError(
                "Output already exists or protocol differs; use a fresh output directory"
            )
    elif any(args.output.iterdir()):
        raise ValueError("Nonempty output lacks protocol")
    else:
        write_json(path, protocol)
    evaluator = SearchEvaluator(args.map)
    overall = {}
    for take in args.takes:
        for agent in args.agents:
            name = f"take{take:02d}-vehicle{agent}"
            directory = args.output / name
            directory.mkdir(exist_ok=True)
            records = []
            started = time.monotonic()
            with CooperSceneSearchSequence(take, agent, prior=args.prior) as sequence:
                from src.localization.cooperscene_geodetic import (
                    RELEASE,
                    CALIBRATION,
                    GEOREFERENCE,
                )

                inputs = {
                    str(p): file_digest(p)
                    for p in [
                        RELEASE / str(take) / str(agent) / "raw_gnss.csv",
                        RELEASE / str(take) / str(agent) / "metadata.json",
                        CALIBRATION / f"tower{agent}-tfs.pkl",
                        GEOREFERENCE,
                    ]
                }
                stat = sequence.archive_path.stat()
                inputs["archive_stat"] = dict(
                    path=str(sequence.archive_path),
                    size=stat.st_size,
                    mtime_ns=stat.st_mtime_ns,
                )
                inputs_path = directory / "inputs.json"
                if (
                    inputs_path.exists()
                    and json.loads(inputs_path.read_text()) != inputs
                ):
                    raise ValueError(f"{name}: input provenance changed")
                write_json(inputs_path, inputs)
                indices = (
                    args.indices if args.indices is not None else range(len(sequence))
                )
                for index in indices:
                    destination = directory / f"frame-{index:06d}.json"
                    if destination.exists():
                        record = json.loads(destination.read_text())
                    else:
                        frame = sequence.read_frame(index)
                        record = evaluator.evaluate(frame)
                        record["frame"] = frame.metadata
                        write_json(destination, record)
                    records.append(record)
                    summary = summarize(records, len(sequence))
                    summary["runtime_seconds_this_process"] = time.monotonic() - started
                    summary["pilot"] = args.indices is not None
                    write_json(directory / "summary.json", summary)
                    overall[name] = summary
                    write_json(args.output / "summary.json", overall)
                    print(
                        f"{name} frame {index}/{len(sequence)-1}: "
                        + ", ".join(
                            f'{m} accepted={record["methods"][m].get("accepted", False)}'
                            for m in ("bev", "xyz")
                        ),
                        flush=True,
                    )
    print("Finished requested evaluation", flush=True)


if __name__ == "__main__":
    main()

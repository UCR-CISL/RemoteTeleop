"""BEV tracking benchmark of the production BEVFusion tracker on cached CooperScene detections.

    # 1. native-ID GT cache (reads ~6k small YAMLs over the dataset mount once, ~45 s)
    .venv/bin/python -m scripts.investigation.cooperscene_tracking.trackbench build-gt
    # 2. one config over test+val (timed); knobs mirror ros_frame_adapter flags
    .venv/bin/python -m scripts.investigation.cooperscene_tracking.trackbench evaluate --birth 0.2
    # 3. baselines, raw-detection reference, val-only sweep, held-out test report
    .venv/bin/python -m scripts.investigation.cooperscene_tracking.trackbench study

Outputs go to artifacts/trackbench-20261012/ (``study.json``, ``summary.md``).
See src/evaluation/cooperscene_tracking_bench.py for the protocol.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import itertools
import json
from pathlib import Path
import time

from src.evaluation.cooperscene_tracking_bench import (
    GroundTruthTrackCache, SequenceData, SequenceScorer, TrackerConfig, TrackingBenchmark)

REPO = Path(__file__).resolve().parents[3]
OUT = REPO / "artifacts/trackbench-20261012"
DETECTIONS = REPO / "artifacts/detbench-20261012/full"
INFO = "/mnt/bcc-data/data/Cooperscene/release/250928_opv2v/cooperscene_infos_{split}.pkl"
DATA_ROOT = "/mnt/bcc-data/data/CooperScene"
VARIANTS = {"A": "A_lidarcam_baseline", "C": "C_lidar"}
PRIMARY = "fov_r50/allgt"
PRODUCTION = TrackerConfig()  # ros_frame_adapter defaults: --bevfusion-score-threshold 0.2, nothing else
PILOT = TrackerConfig(birth=0.2, continuation=0.1, confirmation_hits=1, coast_s=0.5, reacquisition_s=2.0,
                      duplicate_guard=True)
COLUMNS = ("MOTA", "IDF1", "HOTA", "DetA", "AssA", "id_switches", "fragmentation", "MT", "PT", "ML",
           "precision", "recall", "bev_center_rmse_m", "heading_err_mod180_deg")


def predictions(variant: str, split: str) -> Path:
    return DETECTIONS / VARIANTS[variant] / split / "predictions.npz"


def load(splits, variants) -> TrackingBenchmark:
    cache = GroundTruthTrackCache(OUT / "gt")
    return TrackingBenchmark({(s, v): SequenceData.load_split(s, cache, predictions(v, s))
                              for s in splits for v in variants})


def score(metrics: dict, region: str = PRIMARY) -> float:
    m = metrics["overall"][region]
    return m["IDF1"] + m["HOTA"]


def fmt(value) -> str:
    if value is None:
        return "-"
    return f"{value:.3f}" if isinstance(value, float) else str(value)


def table(rows: list[tuple[str, dict]], columns=COLUMNS) -> str:
    lines = ["| row | " + " | ".join(columns) + " |", "|---" * (len(columns) + 1) + "|"]
    lines += [f"| {name} | " + " | ".join(fmt(m.get(c)) for c in columns) + " |" for name, m in rows]
    return "\n".join(lines)


def config_from_args(args) -> TrackerConfig:
    return TrackerConfig(birth=args.birth, continuation=args.birth if args.continuation is None else args.continuation,
                         confirmation_hits=args.hits, coast_s=args.coast, reacquisition_s=args.reacquisition,
                         duplicate_guard=args.guard)


def build_gt(args) -> None:
    cache = GroundTruthTrackCache(OUT / "gt")
    for split in args.splits:
        verification = cache.build(split, data_root=DATA_ROOT, info_pkl=INFO.format(split=split),
                                   predictions_npz=predictions("A", split))
        print(split, json.dumps(verification), flush=True)


def evaluate(args) -> None:
    config = config_from_args(args)
    start = time.perf_counter()
    bench = load(args.splits, args.variants)
    loaded = time.perf_counter()
    keys = [(s, v) for s in args.splits for v in args.variants]
    (result,) = bench.evaluate([config], keys, gt_sets=SequenceScorer.GT_SETS)
    done = time.perf_counter()
    print(f"config {config.label}: load {loaded - start:.1f} s, track+score {done - loaded:.1f} s")
    for key in keys:
        rows = [(f"{key[0]}/{key[1]} {name} {region}", m) for name, per in result[key].items()
                for region, m in per.items() if name == "overall" or region == PRIMARY]
        print(table(rows))


def sweep_grid(base: TrackerConfig, stage: int):
    if stage in (1, 3):
        for birth, continuation, hits in itertools.product((0.1, 0.15, 0.2, 0.25, 0.3, 0.4),
                                                           (0.05, 0.1, 0.15, 0.2), (1, 2, 3)):
            if continuation <= birth:
                yield replace(base, birth=birth, continuation=continuation, confirmation_hits=hits)
    else:
        for coast, reacq, guard in itertools.product((0.3, 0.5, 0.8, 1.0, 1.5), (0.0, 1.0, 2.0, 3.0, 5.0),
                                                     (False, True)):
            yield replace(base, coast_s=coast, reacquisition_s=reacq, duplicate_guard=guard)


def study(args) -> None:
    splits, variants = ("val", "test"), tuple(VARIANTS)
    timings = {}
    t0 = time.perf_counter()
    bench = load(splits, variants)
    timings["load_s"] = time.perf_counter() - t0
    keys = [(s, v) for s in splits for v in variants]
    report: dict = {"protocol": dict(
        matching="BEV (world X/Y) center distance <= 2 m, class-agnostic vehicle; heading mod 180 deg; no z/size",
        regions={"fov_r50": "front camera FOV (benchmark calibration, any corner/center in 1920x1200, depth > 0.1) "
                 "and BEV range <= 50 m", "all_r50": "BEV range <= 50 m", "all": "PCD limit +-72 m"},
        gt_sets={"allgt": "all GT except the ego box", "gt5pts": "GT with >= 5 LiDAR points"},
        ignore="outputs matched to non-scored GT (out of region, ego, low points) are ignored; unmatched outputs "
               "count only inside the region; the tracker always sees all detections",
        selection="val only, maximize IDF1 + HOTA on fov_r50/allgt, detections A")}

    # Timed harness run: one config, test+val, both variants, all regions and GT sets.
    t = time.perf_counter()
    production, pilot = bench.evaluate([PRODUCTION, PILOT], keys, gt_sets=SequenceScorer.GT_SETS)
    timings["two_configs_test_val_AC_all_regions_s"] = time.perf_counter() - t
    t = time.perf_counter()
    bench.evaluate([PRODUCTION], [("val", "A"), ("test", "A")], gt_sets=SequenceScorer.GT_SETS)
    timings["one_config_test_val_A_all_regions_s"] = time.perf_counter() - t

    # Raw per-frame detections at the val best-F1 threshold (no tracker).
    thresholds = (0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.5)
    raw = {}
    for variant in variants:
        results = bench.evaluate(list(thresholds), [("val", variant)], regions=("fov_r50",))
        f1 = lambda m: 2 * m["tp"] / (2 * m["tp"] + m["fp"] + m["fn"]) if m["tp"] + m["fp"] + m["fn"] else 0.0
        best = max(range(len(thresholds)), key=lambda i: f1(results[i][("val", variant)]["overall"][PRIMARY]))
        (chosen,) = bench.evaluate([thresholds[best]], [("val", variant), ("test", variant)],
                                   gt_sets=SequenceScorer.GT_SETS)
        raw[variant] = dict(threshold=thresholds[best], metrics=chosen)

    # Val-only staged sweep per variant: score knobs, lifecycle knobs, score knobs again.
    sweeps, chosen_configs = {}, {}
    t = time.perf_counter()
    for variant in variants:
        best, history = PILOT, []
        for stage in (1, 2, 3):
            configs = list(dict.fromkeys(sweep_grid(best, stage)))
            results = bench.evaluate(configs, [("val", variant)], regions=("fov_r50",))
            for config, result in zip(configs, results):
                m = result[("val", variant)]["overall"][PRIMARY]
                history.append(dict(stage=stage, config=asdict(config), label=config.label,
                                    **{k: m[k] for k in ("IDF1", "HOTA", "MOTA", "id_switches", "precision", "recall")}))
            best = max(zip(configs, results), key=lambda cr: score(cr[1][("val", variant)]))[0]
        unique = {h["label"]: h for h in reversed(history)}  # keep each config's first evaluation
        sweeps[variant] = sorted(unique.values(), key=lambda h: (-(h["IDF1"] + h["HOTA"]), h["stage"]))
        chosen_configs[variant] = best
    timings["sweep_s"] = time.perf_counter() - t
    timings["sweep_unique_configs"] = sum(len(h) for h in sweeps.values())

    # Held-out report: A-selected config on both variants (+ C's own pick on C), with failure diagnostics.
    finals = {}
    for name, config in (("tuned_A", chosen_configs["A"]), ("tuned_C", chosen_configs["C"])):
        (finals[name],) = bench.evaluate([config], keys if name == "tuned_A" else [("val", "C"), ("test", "C")],
                                         gt_sets=SequenceScorer.GT_SETS, diagnose=True)
    (production_diag,) = bench.evaluate([PRODUCTION], [("val", "A"), ("test", "A")], regions=("fov_r50",),
                                        diagnose=True)
    timings["total_s"] = time.perf_counter() - t0
    report.update(timings=timings, configs=dict(production=asdict(PRODUCTION), pilot=asdict(PILOT),
                                                tuned_A=asdict(chosen_configs["A"]), tuned_C=asdict(chosen_configs["C"])),
                  raw_detections={v: dict(threshold=r["threshold"], metrics=_jsonable(r["metrics"])) for v, r in raw.items()},
                  production=_jsonable(production), pilot=_jsonable(pilot), production_diagnostics=_jsonable(production_diag),
                  tuned_A=_jsonable(finals["tuned_A"]), tuned_C=_jsonable(finals["tuned_C"]), sweeps=sweeps)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "study.json").write_text(json.dumps(report, indent=1, default=float) + "\n")
    (OUT / "summary.md").write_text(render(report) + "\n")
    print(json.dumps(timings, indent=1))
    print(f"chosen A: {chosen_configs['A'].label}; chosen C: {chosen_configs['C'].label}")


def _jsonable(result: dict) -> dict:
    return {f"{split}/{variant}": value for (split, variant), value in result.items()}


def render(report: dict) -> str:
    md = ["# CooperScene BEV tracking benchmark (production BevFusionKalmanTracker)", "",
          "Generated by `scripts/investigation/cooperscene_tracking/trackbench.py study`. Protocol: "
          + "; ".join(f"{k}: {v}" for k, v in report["protocol"].items() if isinstance(v, str)) + ".", "",
          "Regions: " + "; ".join(f"`{k}` = {v}" for k, v in report["protocol"]["regions"].items()) + ". "
          "HOTA uses similarity max(0, 1 - d/2 m) (TrackEval algorithm, alphas 0.05-0.95). "
          "Raw detections use frame-local IDs, so only detection columns are meaningful.", "",
          "Configs: " + "; ".join(f"**{k}** `{TrackerConfig(**v).label}`" for k, v in report["configs"].items()), ""]
    sources = [("raw det A", report["raw_detections"]["A"]["metrics"], "A"), ("raw det C", report["raw_detections"]["C"]["metrics"], "C"),
               ("production", report["production"], None), ("pilot", report["pilot"], None),
               ("tuned_A", report["tuned_A"], None), ("tuned_C", report["tuned_C"], "C")]
    for region in ("fov_r50/allgt", "fov_r50/gt5pts", "all_r50/allgt", "all/allgt"):
        md += [f"## {region}", ""]
        rows = []
        for split in ("val", "test"):
            for name, result, only in sources:
                for variant in (only,) if only else ("A", "C"):
                    key = f"{split}/{variant}"
                    if key in result:
                        label = name if not name.startswith("raw") else f"{name} @{report['raw_detections'][variant]['threshold']}"
                        rows.append((f"{split} {variant} {label}", result[key]["overall"][region]))
        md += [table(rows), ""]
    md += ["## Per take, tuned_A (fov_r50/allgt and all_r50/allgt)", ""]
    rows = [(f"{key} {take} {region}", per[region]) for key, result in report["tuned_A"].items()
            for take, per in result.items() if take != "overall" for region in ("fov_r50/allgt", "all_r50/allgt")]
    md += [table(rows), ""]
    md += ["## Failure modes (fov_r50/allgt, overall)", ""]
    for name, result in (("production", report["production_diagnostics"]), ("tuned_A", report["tuned_A"])):
        for key in ("val/A", "test/A"):
            m = result[key]["overall"]["fov_r50/allgt"]
            md += [f"### {name} {key}", "", "| cause | count |", "|---|---|"]
            md += [f"| {cause} | {count} |" for cause, count in m["failures"].items()]
            md += ["", "Localization: " + ", ".join(f"{k}={fmt(v)}" for k, v in m["localization"].items()), ""]
    md += ["## Sweep (val, fov_r50/allgt, top 15 by IDF1+HOTA)", ""]
    for variant, history in report["sweeps"].items():
        md += [f"### detections {variant}", "", "| config | IDF1 | HOTA | MOTA | IDSW | P | R |", "|---|---|---|---|---|---|---|"]
        md += [f"| {h['label']} | {h['IDF1']:.3f} | {h['HOTA']:.3f} | {h['MOTA']:.3f} | {h['id_switches']} | "
               f"{h['precision']:.3f} | {h['recall']:.3f} |" for h in history[:15]]
        md += [""]
    md += ["## Timings", "", "```", json.dumps(report["timings"], indent=1), "```"]
    return "\n".join(md)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    gt = sub.add_parser("build-gt")
    gt.add_argument("--splits", nargs="+", default=["val", "test"])
    ev = sub.add_parser("evaluate")
    ev.add_argument("--splits", nargs="+", default=["val", "test"])
    ev.add_argument("--variants", nargs="+", default=["A"], choices=list(VARIANTS))
    ev.add_argument("--birth", type=float, default=0.2)
    ev.add_argument("--continuation", type=float)
    ev.add_argument("--hits", type=int, default=1)
    ev.add_argument("--coast", type=float, default=0.5)
    ev.add_argument("--reacquisition", type=float, default=0.0)
    ev.add_argument("--guard", action="store_true")
    sub.add_parser("study")
    args = parser.parse_args()
    {"build-gt": build_gt, "evaluate": evaluate, "study": study}[args.command](args)


if __name__ == "__main__":
    main()

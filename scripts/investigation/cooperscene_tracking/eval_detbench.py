"""Evaluate batch_detect.py outputs on the host (numpy/scipy/shapely only).

    .venv/bin/python -m scripts.investigation.cooperscene_tracking.eval_detbench \
        artifacts/detbench-20261012/full

Writes ``<variant>/<split>/metrics.json`` and ``summary.md`` under the run root.
``--official-check`` also recomputes overall BEV/3D AP with CooperScene's own
OpenCOOD-derived code (thirdparty/CooperScene, needs torch) for comparison.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from src.evaluation.cooperscene_detection import DetectionBenchmark, FrontCameraFov, load_frames  # noqa: E402

INFO = "/mnt/bcc-data/data/Cooperscene/release/250928_opv2v/cooperscene_infos_{split}.pkl"
CALIBRATION = "/mnt/bcc-data/data/Cooperscene/benchmark_cooperbevfusion/250928/calibration_results/calibration_results_all.json"
REPO = Path(__file__).resolve().parents[3]


def official_ap(frames, thresholds=(0.3, 0.5, 0.7)) -> dict:
    """CooperScene EvalMetric.compute_metrics path (OpenCOOD polygon IoU, global sort)."""
    import torch
    path = REPO / "thirdparty/CooperScene/models/cooperative/evaluation/calculate_ap.py"
    spec = importlib.util.spec_from_file_location("cs_calculate_ap", path)
    cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)

    def corners(b):  # [x,y,zc,l,w,h,yaw] -> OpenCOOD hwl order
        return cs.boxes_to_corners_3d(torch.as_tensor(b[:, [0, 1, 2, 5, 4, 3, 6]], dtype=torch.float32), order="hwl") \
            if len(b) else torch.zeros((0, 8, 3))
    stats = {k: {t: {"tp": [], "fp": [], "gt": 0, "score": []} for t in thresholds} for k in ("bev", "3d")}
    for f in frames:
        pc, gc, sc = corners(f.pred), corners(f.gt), torch.as_tensor(f.scores, dtype=torch.float32)
        for t in thresholds:
            for mode in ("bev", "3d"):
                cs.caluclate_tp_fp(pc, sc, gc, stats[mode], t, iou_mode=mode)
    return {f"AP_{'BEV' if m == 'bev' else '3D'}@{t}": cs.calculate_ap(stats[m], t, True)[0]
            for m in ("bev", "3d") for t in thresholds}


def headline(res: dict) -> dict:
    best = res["center2m"][str(res["best_f1_threshold"])]
    e = best["errors"]
    return {"n_gt": res["n_gt"], **{k: res[k] for k in ("AP_BEV@0.3", "AP_BEV@0.5", "AP_BEV@0.7", "AP_3D@0.5")},
            "best_thr": res["best_f1_threshold"], "P": best["precision"], "R": best["recall"], "F1": best["f1"],
            "bev_rmse": e.get("bev_center_rmse"), "yaw_deg": e.get("yaw_err_mod180_deg_mean"),
            "dz_mean": e.get("z_center_err_mean"), "dz_bottom_mean": e.get("z_bottom_err_mean")}


def table(rows: list[tuple[str, dict]]) -> str:
    cols = ["n_gt", "AP_BEV@0.3", "AP_BEV@0.5", "AP_BEV@0.7", "AP_3D@0.5", "best_thr", "P", "R", "F1",
            "bev_rmse", "yaw_deg", "dz_mean", "dz_bottom_mean"]
    fmt = lambda v: "-" if v is None or (isinstance(v, float) and np.isnan(v)) else (f"{v:.3f}" if isinstance(v, float) else str(v))
    lines = ["| row | " + " | ".join(cols) + " |", "|---" * (len(cols) + 1) + "|"]
    lines += [f"| {name} | " + " | ".join(fmt(h[c]) for c in cols) + " |" for name, h in rows]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path, help="run root containing <variant>/<split>/predictions.npz")
    parser.add_argument("--official-check", action="store_true")
    args = parser.parse_args()
    fov = FrontCameraFov(json.loads(Path(CALIBRATION).read_text()))
    md = [f"# Detection benchmark: {args.root}", "",
          "Primary: BEV AP (OpenCOOD protocol) and BEV 2 m center matching at the best-F1 score threshold; "
          "bev_rmse/yaw on matched pairs. AP_3D and dz (pred - GT) are secondary diagnostics.", ""]
    for run in sorted(p.parent for p in args.root.glob("*/*/predictions.npz")):
        variant, split = run.parent.name, run.name
        frames = load_frames(str(run / "predictions.npz"), INFO.format(split=split))
        bench = DetectionBenchmark(fov)
        for f in frames:
            bench.add(f)
        metrics = {"overall": bench.summarize()}
        for seq in sorted({f.sequence for f in frames}, key=lambda s: (int(s[0]), int(s[1]))):
            metrics[f"take{seq[0]}_agent{seq[1]}"] = bench.summarize(lambda f, seq=seq: f.sequence == seq)
        if args.official_check:
            metrics["official_cooperscene_code"] = official_ap(frames)
        manifest = json.loads((run / "manifest.json").read_text())
        metrics["latency"] = {"forward_ms": manifest["forward_ms"], "shared_pass_wall_time_s": manifest["shared_pass_wall_time_s"],
                              "samples": manifest["samples"]}
        (run / "metrics.json").write_text(json.dumps(metrics, indent=1, default=float) + "\n")
        o = metrics["overall"]
        rows = [(f"overall {k}", headline(o[k])) for k in o if k != "frames"]
        rows += [(f"{name} all/allgt", headline(m["all/allgt"])) for name, m in metrics.items()
                 if name.startswith("take")]
        lat = manifest["forward_ms"]
        md += [f"## {variant} / {split}", "",
               f"{manifest['samples']} samples; forward mean {lat['mean']:.1f} ms, p95 {lat['p95']:.1f} ms; "
               f"shared-pass wall {manifest['shared_pass_wall_time_s']:.0f} s", ""]
        if args.official_check:
            md += ["CooperScene code: " + ", ".join(f"{k} {v:.4f}" for k, v in metrics["official_cooperscene_code"].items()), ""]
        md += [table(rows), ""]
        print(f"{variant}/{split}: " + json.dumps(headline(o["all/allgt"]), default=float), flush=True)
    (args.root / "summary.md").write_text("\n".join(md) + "\n")


if __name__ == "__main__":
    main()

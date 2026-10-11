"""Summarize exhaustive localization search without conflating fit and reference error."""
import argparse
import json
from statistics import median
from pathlib import Path


def number(value):
    return "—" if value is None else f"{value:.3f}"


def frame_label(candidate):
    if candidate is None:
        return "none"
    f = candidate["frame"]
    index = f.get("frame_index", f.get("frame_offset", f.get("index")))
    seconds = f.get("elapsed_seconds")
    percentage = f.get("percentage")
    return f"{index} / {number(seconds)} s / {number(percentage)}%"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    summaries = json.loads((args.output / "summary.json").read_text())
    is_pilot = any(s.get("pilot", False) for s in summaries.values())
    complete = [s for s in summaries.values() if s["complete"]]
    overview = [
        f"Completed sequences: **{len(complete)} / 42**. Examined frames: **{sum(s['examined_frames'] for s in summaries.values())} / 18,492**.",
        "",
        "| Method | Completed sequences with any accepted frame | Completed sequences meeting <0.1 m | Median best XY / XYZ (m), accepted sequences |",
        "|---|---:|---:|---|",
    ]
    for method in ("bev", "xyz"):
        entries = [s["methods"][method] for s in complete]
        errors = [e["best_accepted"]["error"] for e in entries if e["best_accepted"]]
        xy = median([e["xy_m"] for e in errors]) if errors else None
        xyz = median([e["xyz_m"] for e in errors]) if errors else None
        overview.append(
            f"| {method.upper()} | {len(errors)} | {sum(e['earliest_qualifying'] is not None for e in entries)} | {number(xy)} / {number(xyz)} |"
        )
    overview += [
        "",
        "Counts above include only completely searched sequences. Per-sequence best values below remain provisional while a search is incomplete.",
        "",
    ]
    lines = [
        "# CooperScene BEV / XYZ frame search",
        "**Pilot: selected indices only; not an exhaustive search.**" if is_pilot else "**Full native-frame search; results remain provisional until completion.**",
        "",
        "Reference: the retained raw-GNSS/INS calibration recipe. This same reference initializes each frame and supplies the ENU fusion prior; results measure map-registration consistency and prior agreement, **not independent ground-truth accuracy**.",
        "",
        "Every native frame is searched without a percentage cutoff. Best means lowest accepted XY error for BEV, and lowest accepted XYZ error for XYZ. Earliest qualifying additionally requires strict error <0.1 m. Rejected low-error fits never qualify. BEV’s XYZ error is reported separately.",
        "",
        *overview,
        "| Sequence | Progress | Method | Accepted frames | Best frame / elapsed / progress | Best XY (m) | Best Z signed (m) | Best XYZ (m) | Frame 0 XY / XYZ (m) | Earliest <0.1 m |",
        "|---|---|---|---:|---|---:|---:|---:|---|---|",
    ]
    for name, summary in sorted(summaries.items()):
        progress = f'{summary["examined_frames"]}/{summary["total_frames"]}'
        if summary["complete"]:
            progress += " complete"
        for method, entry in summary["methods"].items():
            best = entry["best_accepted"]
            error = best["error"] if best else {}
            zero_record = entry.get("frame0") or {}
            zero = zero_record.get("error") or {}
            zero_status = "accepted" if zero_record.get("accepted") else "rejected"
            lines.append(
                f'| {name} | {progress} | {method.upper()} | {entry["accepted_frames"]} | {frame_label(best)} | {number(error.get("xy_m"))} | {number(error.get("dz_m"))} | {number(error.get("xyz_m"))} | {number(zero.get("xy_m"))} / {number(zero.get("xyz_m"))} ({zero_status}) | {frame_label(entry["earliest_qualifying"])} |'
            )
    lines += [
        "",
        "## Interpretation",
        "",
        "- Frame 0 reports the optimized candidate, including rejected candidates; initial reference error is zero by construction.",
        "- No qualifying frame after complete exhaustion retains the labelled `frame0_recipe_prior_not_qualified` fallback. Incomplete searches make no exhaustion claim.",
        "- Frames with a mathematical rejection certificate have no optimized error: they cannot pass the unchanged registration gates within the allowed correction.",
        "- Full signed XYZ, orientation, inliers, residuals, rejection reasons, raw scan and fusion diagnostics are saved in per-frame JSON files.",
        "- Previous results and mounted input data are unchanged. No MCAPs or teleoperation batches are generated.",
        "",
    ]
    temp = args.output / "report.md.tmp"
    temp.write_text("\n".join(lines))
    temp.replace(args.output / "report.md")


if __name__ == "__main__":
    main()

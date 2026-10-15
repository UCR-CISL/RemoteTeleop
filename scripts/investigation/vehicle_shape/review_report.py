"""Figures (compare.png, vehicles.png), REVIEW.md and summary.json/csv for evaluate.py."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import cv2
import numpy as np

from src.viz.universal_vehicle import TYPE_NAMES, UniversalBodyProfile

RANGE_BUCKETS = ((3, 10), (10, 20), (20, 30), (30, 45), (45, 60))
BODY_LABELS = Path(__file__).with_name("body_labels.json")  # visual body labels (3 labelers, all v2 rows)
# Label vocabulary -> fitted types: wagons count as hatchbacks (long-roof, short-deck), coupes as sedans.
LABEL_TYPES = {"wagon": "hatchback", "coupe": "sedan"}
# Coarse silhouette groups for a forgiving accuracy.
BODY_GROUPS = {"sedan": "car", "hatchback": "car", "crossover": "suv", "full_suv": "suv", "minivan": "suv",
               "van": "van", "pickup": "pickup"}
MIN_HELDOUT_POINTS = 20  # held-out distance stats need this many target-frame returns
MAX_VEHICLE_ROWS = 10


def _bucket(range_m: float) -> str:
    for low, high in RANGE_BUCKETS:
        if low <= range_m < high:
            return f"{low}-{high} m"
    return f"{RANGE_BUCKETS[-1][1]}+ m"


def _hex(rgb) -> str:
    return "#" + "".join(f"{int(round(c * 255)):02x}" for c in rgb)


class FrameFigures:
    """compare.png (real | default | fitted) and vehicles.png (one diagnostic row per vehicle)."""

    def compare(self, result, path: Path) -> None:
        real = cv2.resize(result.real_bgr, result.render_default.shape[1::-1], interpolation=cv2.INTER_AREA)
        evaluated = {v.track_id for v in result.vehicles}
        panels = []
        variants = ((real, f"real camera  {result.spec.key}"), (result.render_default, "LEGACY primitive + tint"),
                    (result.render_means, "TYPE MEAN + tint"), (result.render_fitted, "FITTED shape + tint"))
        for image, title in (variant for variant in variants if variant[0] is not None):
            image = image.copy()
            self._labels(image, result, evaluated)
            cv2.rectangle(image, (0, 0), (len(title) * 13 + 14, 32), (0, 0, 0), -1)
            cv2.putText(image, title, (7, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)
            panels.append(image)
        separator = np.full((panels[0].shape[0], 6, 3), 255, np.uint8)
        stacked = [panels[0]]
        for panel in panels[1:]:
            stacked += [separator, panel]
        cv2.imwrite(str(path), np.hstack(stacked))

    @staticmethod
    def _labels(image, result, evaluated) -> None:
        camera_T_world = np.linalg.inv(result.world_T_camera)
        signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], dtype=float)
        for track_id, world_T_object in result.world_T_objects.items():
            camera_T_object = camera_T_world @ world_T_object
            corners = signs * (result.dims[track_id] / 2) @ camera_T_object[:3, :3].T + camera_T_object[:3, 3]
            if (corners[:, 2] <= 0.5).any():
                continue
            uv = (corners @ result.render_K.T)[:, :2] / corners[:, 2:3]
            # Top-center of the projected box, clipped into the image.
            u = float(np.clip((max(uv[:, 0].min(), 0) + min(uv[:, 0].max(), image.shape[1])) / 2, 0, image.shape[1] - 30))
            v = float(np.clip(uv[:, 1].min() - 3, 45, image.shape[0] - 5))
            if uv[:, 0].max() < 0 or uv[:, 0].min() >= image.shape[1] or uv[:, 1].min() >= image.shape[0]:
                continue
            color = (0, 255, 255) if track_id in evaluated else (170, 170, 170)
            cv2.putText(image, track_id, (int(u) - 8, int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(image, track_id, (int(u) - 8, int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

    def vehicles(self, result, path: Path) -> None:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        rows = result.vehicles[:MAX_VEHICLE_ROWS]
        if not rows:
            return
        widths = [2.4, 2.4, 2.4, 2.4, 0.7, 2.6, 2.6, 3.4]
        fig, axes = plt.subplots(len(rows), len(widths), figsize=(sum(widths) * 1.15, 2.5 * len(rows)),
                                 gridspec_kw=dict(width_ratios=widths), squeeze=False)
        scale = result.render_default.shape[1] / result.real_bgr.shape[1]
        for row, v in zip(axes, rows):
            x0, y0, x1, y1 = self._crop_box(v.crop_xyxy_native, result.real_bgr.shape)
            crop = cv2.cvtColor(result.real_bgr[y0:y1, x0:x1], cv2.COLOR_BGR2RGB)
            row[0].imshow(crop, extent=(x0, x1, y1, y0))
            for uv, style in ((v.sample_uv_rejected, dict(c="red", marker="x", s=6, linewidths=0.6)),
                              (v.sample_uv_ok, dict(c="yellow", s=4, edgecolors="black", linewidths=0.2))):
                if uv is not None and len(uv):
                    row[0].scatter(uv[:, 0], uv[:, 1], **style)
            row[0].set_xlim(x0, x1)
            row[0].set_ylim(y1, y0)
            row[0].set_title(f"#{v.track_id} real (t) + belt LiDAR", fontsize=8)
            for ax, image, title in ((row[1], result.render_default, "legacy+tint"), (row[2], result.render_means, "type mean+tint"),
                                     (row[3], result.render_fitted, "fitted+tint")):
                if image is None:
                    ax.axis("off")
                    continue
                r0, r1 = int(y0 * scale), max(int(y1 * scale), int(y0 * scale) + 2)
                c0, c1 = int(x0 * scale), max(int(x1 * scale), int(x0 * scale) + 2)
                ax.imshow(cv2.cvtColor(image[r0:r1, c0:c1], cv2.COLOR_BGR2RGB))
                ax.set_title(title, fontsize=8)
            self._swatch(row[4], v)
            self._profile(row[5], v, view="bev")
            self._profile(row[6], v, view="side")
            row[7].text(0, 1, self._text(v), va="top", ha="left", family="monospace", fontsize=7.2,
                        transform=row[7].transAxes)
            for ax in (row[0], row[1], row[2], row[3], row[4], row[7]):
                ax.set_xticks([])
                ax.set_yticks([])
            row[7].axis("off")
        fig.tight_layout(pad=0.4)
        fig.savefig(path, dpi=72)
        plt.close(fig)

    def callout(self, result, v, baseline: Path, path: Path) -> None:
        """Real | baseline fitted+tint | this run's fitted+tint crops of one vehicle, with color swatches."""
        x0, y0, x1, y1 = self._crop_box(v.crop_xyxy_native, result.real_bgr.shape)
        width, height = result.render_default.shape[1], result.render_default.shape[0]
        scale = width / result.real_bgr.shape[1]
        c0, r0, c1, r1 = (int(value * scale) for value in (x0, y0, x1, y1))
        real = cv2.resize(result.real_bgr, (width, height), interpolation=cv2.INTER_AREA)[r0:r1, c0:c1]
        old = cv2.imread(str(baseline / "frames" / result.spec.key / "compare.png"))
        if old is not None:  # the last panel is the baseline's fitted render
            last = (old.shape[1] + 6) // (width + 6) - 1
            old = old[r0:r1, last * (width + 6) + c0:last * (width + 6) + c1]
        else:
            old = np.zeros_like(real)
        panels = [real, old, result.render_fitted[r0:r1, c0:c1]]
        target = 180
        panels = [cv2.resize(panel, (max(1, int(panel.shape[1] * target / max(panel.shape[0], 1))), target))
                  for panel in panels]
        gap = np.full((target, 4, 3), 255, np.uint8)
        cv2.imwrite(str(path), np.hstack([panels[0], gap, panels[1], gap, panels[2]]))

    @staticmethod
    def swatch(rgb, path: Path, *, displayed: bool = True) -> None:
        """Published color (left) and, with ``displayed``, the albedo this renderer paints (right)."""
        image = np.full((40, 120 if displayed else 60, 3), 215, np.uint8)
        if rgb is not None:
            image[:, :60] = np.rint(np.asarray(rgb)[::-1] * 255)
            if displayed:
                image[:, 60:] = np.rint(displayed_albedo(rgb)[::-1] * 255)
        else:
            cv2.line(image, (0, 0), (image.shape[1] - 1, 39), (90, 90, 90), 1)
        cv2.imwrite(str(path), image)

    @staticmethod
    def _crop_box(xyxy, shape):
        height, width = shape[:2]
        x0, y0, x1, y1 = xyxy
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        half_w, half_h = max((x1 - x0) * 0.6, 48), max((y1 - y0) * 0.6, 36)
        return (int(max(cx - half_w, 0)), int(max(cy - half_h, 0)),
                int(min(cx + half_w, width)), int(min(cy + half_h, height)))

    @staticmethod
    def _swatch(ax, v) -> None:
        if v.color_rgb is not None:
            swatch = np.ones((4, 1, 3)) * np.asarray(v.color_rgb)
            if v.road_balanced_rgb is not None:
                swatch[3] = v.road_balanced_rgb  # bottom quarter: road-white-balanced diagnostic
            ax.imshow(swatch)
            ax.set_title(f"c={v.color_confidence:.2f}\nn={v.color_fused_observations}", fontsize=7)
        elif v.color_observations:
            last = v.color_observations[-1]
            ax.imshow(np.ones((4, 1, 3)) * np.asarray(last["rgb"]), alpha=0.45)
            ax.set_title(f"UNPUB\nlast c={last['confidence']:.2f}", fontsize=7, color="red")
        else:
            ax.imshow(np.full((4, 1, 3), 0.85))
            ax.set_title("no obs", fontsize=7, color="red")

    _legacy = []

    @classmethod
    def _outline(cls, shape, dims, view: str):
        """Body outline (metric, object frame) for the BEV or side view; wheels excluded.

        A universal ``shape`` draws its profile (side: roof/hood/bed-rail line and underside;
        BEV: body sides). ``None`` draws the legacy primitive's envelope.
        """
        dims = np.asarray(dims, dtype=float)
        if shape is not None:
            stations, rings, _ = UniversalBodyProfile.rings(shape.array, dims, shape.prior)
            x, ring = stations[0], rings[0]
            if view == "bev":
                upper, lower = ring[:, 3, 0], -ring[:, 3, 0]
            else:
                upper, lower = ring[:, 7, 1] - dims[2] / 2, ring[:, 0, 1] - dims[2] / 2
            return [(np.r_[x, x[::-1], x[:1]], np.r_[upper, lower[::-1], upper[:1]])]
        if not cls._legacy:
            from src.viz.primitive_car import PrimitiveCarAsset

            cls._legacy.append(PrimitiveCarAsset())
        asset = cls._legacy[0]
        vertices = asset.vertices_for_dimensions(dims)
        body = np.ones(len(vertices), bool)
        for wheel in asset._wheels:
            body[wheel.start:wheel.stop] = False
        vertices = vertices[body]
        column = 1 if view == "bev" else 2
        edges = np.linspace(vertices[:, 0].min(), vertices[:, 0].max(), 31)
        index = np.clip(np.digitize(vertices[:, 0], edges) - 1, 0, 29)
        x, upper, lower = [], [], []
        for bin_index in range(30):
            values = vertices[index == bin_index, column]
            if len(values):
                x.append((edges[bin_index] + edges[bin_index + 1]) / 2)
                upper.append(values.max())
                lower.append(-values.max() if view == "bev" else values.min())
        x, upper, lower = map(np.asarray, (x, upper, lower))
        return [(np.r_[x, x[::-1], x[:1]], np.r_[upper, lower[::-1], upper[:1]])]

    def _profile(self, ax, v, *, view: str) -> None:
        dims = np.asarray(v.dims_lwh)
        column = 1 if view == "bev" else 2
        if v.accumulated_points is not None and len(v.accumulated_points):
            ax.scatter(v.accumulated_points[:, 0], v.accumulated_points[:, column], s=2 + 2 * v.accumulated_weights,
                       c="tab:blue", alpha=0.5, linewidths=0)
        if v.heldout_xyz is not None and len(v.heldout_xyz):
            ax.scatter(v.heldout_xyz[:, 0], v.heldout_xyz[:, column], s=2, c="tab:orange", alpha=0.7, linewidths=0)
        for polygon in self._outline(None, dims, view):
            ax.plot(*polygon, "k--", lw=0.8)
        if v.published_shape is not None:
            for polygon in self._outline(v.published_shape, dims, view):
                ax.plot(*polygon, "-", color="tab:green", lw=1.3)
        ax.annotate("", xy=(dims[0] / 2 + 0.4, 0 if view == "bev" else -dims[2] / 2 - 0.15),
                    xytext=(dims[0] / 2 - 0.2, 0 if view == "bev" else -dims[2] / 2 - 0.15),
                    arrowprops=dict(arrowstyle="->", color="gray"))
        ax.set_aspect("equal")
        ax.set_xlim(-dims[0] / 2 - 0.5, dims[0] / 2 + 0.6)
        if view == "bev":
            ax.set_ylim(-dims[1] / 2 - 0.4, dims[1] / 2 + 0.4)
            ax.set_title("BEV (+x = GT front →)  blue=acc, orange=t", fontsize=7)
        else:
            ax.set_ylim(-dims[2] / 2 - 0.3, dims[2] / 2 + 0.3)
            ax.set_title("side  green=fit, dashed=legacy", fontsize=7)
        ax.tick_params(labelsize=6)

    @staticmethod
    def _text(v) -> str:
        shape = v.published_shape
        fit = ("not published" if shape is None else f"{shape.body_type}{' (prior only)' if v.prior_only else ''}")
        motion = {1: "+x", -1: "-x", 0: "static"}[v.motion_front_sign]
        lat = f"{np.mean(v.fit_latency_ms):.0f}/{np.max(v.fit_latency_ms):.0f}" if v.fit_latency_ms else "-"
        color = (f"{_hex(v.color_rgb)} c={v.color_confidence:.2f} n={v.color_fused_observations}"
                 if v.color_rgb is not None else f"unpublished ({v.color_frames_confident}/{v.color_frames_observed} conf/obs)")
        dd, df = v.heldout_default, v.heldout_fitted
        return "\n".join([
            f"#{v.track_id}  {v.range_m:.1f} m  vis {v.visible_fraction:.2f}",
            f"LWH {v.dims_lwh[0]:.2f} {v.dims_lwh[1]:.2f} {v.dims_lwh[2]:.2f}  snap {v.snap_shift_m:+.2f} m",
            f"hist {len(v.history_frames)} fr{' (+future)' if v.used_future_frames else ''}  raw pts {v.raw_points}",
            f"voxels {v.accumulator_voxels}  evidence {v.evidence_voxels}",
            f"fit: {fit}",
            f"  accepted {v.accepted_steps}/{v.n_fits} steps ({v.sweeps} sweeps), rms {v.fit_rms_m:.3f}",
            f"  cold: {v.cold_fit.get('body_type')} {v.cold_fit.get('forward_sign')} acc={v.cold_fit.get('accepted')}",
            f"  GT motion {v.gt_motion_m:.1f} m -> front {motion}",
            f"  latency mean/max {lat} ms",
            f"held-out ({v.heldout_points} pts) med/p90 m:",
            f"  legacy {dd[0]:.3f}/{dd[1]:.3f}",
            f"  mean   {v.heldout_means[0]:.3f}/{v.heldout_means[1]:.3f}",
            f"  fitted {df[0]:.3f}/{df[1]:.3f}",
            f"color: {color}",
            f"  L* belt/road at t: {v.target_body_L:.0f}/{v.target_road_L:.0f}"
            + (f"  road-WB {_hex(v.road_balanced_rgb)}" if v.road_balanced_rgb is not None else ""),
        ])


class ReviewReport:
    """Collect per-frame results; write figures as they arrive and the summaries at the end."""

    def __init__(self, output: Path, settings: dict, *, figures: bool = True, baseline: Path | None = None,
                 callouts: list[dict] | None = None) -> None:
        self.output, self.settings, self.figures = output, settings, figures
        self.baseline, self.callouts = baseline, callouts or []
        self.output.mkdir(parents=True, exist_ok=True)
        self.rows: list[dict] = []
        self.frames: list[dict] = []
        self._drawer = FrameFigures()

    def add(self, result) -> None:
        directory = self.output / "frames" / result.spec.key
        directory.mkdir(parents=True, exist_ok=True)
        if self.figures:
            self._drawer.compare(result, directory / "compare.png")
            self._drawer.vehicles(result, directory / "vehicles.png")
            for v in result.vehicles:
                if self.baseline is not None and any(c["frame_key"] == result.spec.key and c["track_id"] == v.track_id
                                                     for c in self.callouts):
                    self._drawer.callout(result, v, self.baseline, directory / f"callout_{v.track_id}.png")
                    self._drawer.swatch(v.color_rgb, directory / f"swatch_{v.track_id}.png")
        rows = [v.row(result.spec) for v in result.vehicles]
        for row, v in zip(rows, result.vehicles):
            row["color_observation_log"] = v.color_observations
        self.rows.extend(rows)
        spec = result.spec
        self.frames.append(dict(key=spec.key, split=spec.split, take=spec.take, agent=spec.agent,
                                frame_index=spec.frame_index, frame_id=spec.frame_id,
                                rendered_vehicles=len(result.world_T_objects), evaluated_vehicles=len(rows),
                                timings_s={k: round(t, 2) for k, t in result.timings_s.items()}))

    @classmethod
    def rebuild(cls, output: Path) -> None:
        """Rewrite REVIEW.md from an existing summary.json (e.g. after adding variants/)."""
        summary = json.loads((output / "summary.json").read_text())
        settings = summary["settings"]
        report = cls(output, settings, baseline=Path(settings["baseline"]) if settings.get("baseline") else None,
                     callouts=settings.get("callouts"))
        report.rows, report.frames = summary["vehicles"], summary["frames"]
        (output / "REVIEW.md").write_text(report._markdown(summary["aggregate"]))

    def _variants(self) -> dict[str, dict]:
        """Aggregates of metric/render variants copied under variants/<name>/summary.json."""
        variants = {}
        for path in sorted((self.output / "variants").glob("*/summary.json")):
            summary = json.loads(path.read_text())
            variants[path.parent.name] = dict(settings=summary["settings"], all=summary["aggregate"]["all"])
        return variants

    def finish(self) -> None:
        aggregate = self.aggregate(self.rows)
        problems = {frame["key"]: self._frame_problems([r for r in self.rows if r["frame_key"] == frame["key"]])
                    for frame in self.frames}
        for frame in self.frames:
            frame["problem_score"], frame["problems"] = problems[frame["key"]]
        (self.output / "summary.json").write_text(json.dumps(
            dict(settings=self.settings, aggregate=aggregate, frames=self.frames, vehicles=self.rows), indent=1,
            default=lambda value: value.item() if hasattr(value, "item") else str(value)))
        columns = [k for k in self.rows[0] if k != "color_observation_log"] if self.rows else []
        with (self.output / "summary.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, columns, extrasaction="ignore")
            writer.writeheader()
            for row in self.rows:
                writer.writerow({k: (json.dumps(v) if isinstance(v, list) else v) for k, v in row.items()})
        if self.figures:
            (self.output / "REVIEW.md").write_text(self._markdown(aggregate))

    @staticmethod
    def aggregate(rows: list[dict]) -> dict:
        groups: dict[str, list[dict]] = {}
        for row in rows:
            groups.setdefault(_bucket(row["range_m"]), []).append(row)
        groups["all"] = rows
        out = {}
        for name, group in groups.items():
            scored = [r for r in group if r["heldout_points"] >= MIN_HELDOUT_POINTS]
            fitted = [r for r in scored if r["shape_published"]]
            moving = [r for r in group if r["sign_matches_motion"] is not None]
            published = [r for r in group if r["shape_published"]]
            median = lambda key, rs: _median([r.get(key) for r in rs])
            out[name] = dict(
                vehicles=len(group),
                shape_published_rate=_rate(len(published), len(group)),
                color_published_rate=_rate(sum(r["color_published"] for r in group), len(group)),
                scored_vehicles=len(scored),
                heldout_median_default_m=median("heldout_default_median_m", scored),
                heldout_median_fitted_m=median("heldout_fitted_median_m", scored),
                heldout_p90_default_m=median("heldout_default_p90_m", scored),
                heldout_p90_fitted_m=median("heldout_fitted_p90_m", scored),
                heldout_median_means_m=median("heldout_means_median_m", scored),
                heldout_p90_means_m=median("heldout_means_p90_m", scored),
                prior_only_rate=_rate(sum(bool(r.get("prior_only")) for r in published), len(published)),
                published_only_p90_default_m=median("heldout_default_p90_m", fitted),
                published_only_p90_fitted_m=median("heldout_fitted_p90_m", fitted),
                fitted_better_p90_rate=_rate(sum(r["heldout_fitted_p90_m"] < r["heldout_default_p90_m"] - 1e-4
                                                 for r in fitted), len(fitted)),
                fitted_worse_p90_rate=_rate(sum(r["heldout_fitted_p90_m"] > r["heldout_default_p90_m"] + 1e-4
                                                for r in fitted), len(fitted)),
                forward_sign_vs_gt_x=_rate(sum(r["sign_matches_gt_x"] for r in published), len(published)),
                forward_sign_vs_motion=_rate(sum(r["sign_matches_motion"] for r in moving), len(moving)),
                moving_with_shape=len(moving),
                gt_x_vs_motion=_rate(sum(r["motion_front_sign"] == 1 for r in group if r["motion_front_sign"]),
                                     sum(bool(r["motion_front_sign"]) for r in group)),
                body_types={t: sum(r["body_type"] == t for r in published) for t in TYPE_NAMES},
                color_L_percentiles=_percentiles([r["color_L"] for r in group if r["color_published"]]),
                color_L_below_30=sum(r["color_published"] and r["color_L"] < 30 for r in group),
                fit_latency_ms_mean=_median([r["fit_latency_ms_mean"] for r in group]),
                fit_latency_ms_max=max([r["fit_latency_ms_max"] or 0 for r in group], default=None),
            )
        return out

    @staticmethod
    def _frame_problems(rows: list[dict]) -> tuple[float, list[str]]:
        score, notes = 0.0, []
        for r in rows:
            tid = r["track_id"]
            if r["shape_published"] and r["heldout_points"] >= MIN_HELDOUT_POINTS:
                worse = r["heldout_fitted_p90_m"] - r["heldout_default_p90_m"]
                if worse > 0.01:
                    score += 10 * worse
                    notes.append(f"#{tid} fit worse p90 +{worse:.2f} m")
            if r["sign_matches_motion"] is False or (r["sign_matches_motion"] is None and r["sign_matches_gt_x"] is False):
                score += 1
                notes.append(f"#{tid} forward_sign={r['forward_sign']} vs GT front")
            if r["range_m"] < 20 and r["visible_fraction"] >= 0.8 and not r["color_published"]:
                score += 1
                notes.append(f"#{tid} near ({r['range_m']:.0f} m) but no color")
            if r["color_published"] and _warm_tint(r["color_rgb"]):
                score += 1.5
                notes.append(f"#{tid} warm tint {_hex(r['color_rgb'])} (tail-light / low-sun cast?)")
            if not r["shape_published"] and r["evidence_voxels"] >= 70:
                score += 1
                notes.append(f"#{tid} shape rejected with {r['evidence_voxels']} voxels (rms {r['fit_rms_m']})")
        return round(score, 2), notes

    def _markdown(self, aggregate: dict) -> str:
        s = self.settings
        lines = [
            "# Vehicle proxy review: fitted shape + body tint (CooperScene, GT poses)",
            "",
            "Each frame is rendered through the gsplat compositor at a GT-registered pose (per-frame",
            "`xyz` splat alignment), with no MCAP or split-system replay. Every GT vehicle in view (3-60 m)",
            f"accumulates its in-box LiDAR returns over the previous **{s['history']} native frames (~{s['history'] / 10:.0f} s)**,",
            "using native `map_T_lidar` / `map_T_object` with GT box bottoms snapped onto the LiDAR ground.",
            "`ObjectPointAccumulator` stores them and `VehicleShapeFitter` refits after every scan, warm-started",
            "from the last accepted fit; as in the live estimator only the incumbent type is refit except every",
            f"{s.get('sweep_every', 5)}th fit (full type sweep). The last accepted fit is published; before the first",
            "accepted fit the dimension-prior type mean is published (\"prior only\").",
            "`VehicleColorEstimator.observe` runs on each history frame's native 1920x1200 camera image with",
            "the refined calibration, a full-scan `SparseDepthBuffer`, and nearer GT boxes as occluders.",
            "Frame *t* itself is held out: its in-box points give true point-to-triangle distances to the legacy",
            "primitive, to the published type's mean (normalized to the box) and to the published fit.",
            "",
            "Each frame has two images. `compare.png` shows real camera | legacy primitive | type mean | fitted",
            "shape, all with the same published body tint.",
            "Track IDs appear in yellow for evaluated vehicles and gray for those beyond 60 m.",
            "`vehicles.png` has one row per vehicle: a real crop with belt-band LiDAR samples (yellow = passed",
            "the depth test, red = rejected), the three render crops, a color swatch, BEV and side views, and metrics.",
            "In the BEV and side views, blue = accumulated voxels, orange = held-out frame *t*,",
            "dashed = legacy primitive envelope, green = fitted envelope, and the arrow marks GT +x.",
            "The swatch's bottom quarter is a diagnostic: the published color white-balanced so that the road",
            "ring around the car reads neutral. It is not used in the render.",
            "",
            "Command (worktree root):",
            "```",
            "PYTHONPATH=. ../RemoteTeleop/.venv/bin/python scripts/investigation/vehicle_shape/evaluate.py",
            "```",
            f"Settings: `{json.dumps({k: s.get(k) for k in ('history', 'fitter', 'accumulator', 'color', 'calibration', 'default_incumbent', 'sweep_every')})}`",
            "",
            *self._changes_since_baseline(aggregate),
            "## Aggregate (per range bucket)",
            "",
            f"Distances are medians over vehicles with >= {MIN_HELDOUT_POINTS} held-out points. A vehicle with no",
            "published shape scores the default profile in both columns. The 'published only' columns",
            "cover vehicles with a published shape. forward_sign vs motion uses GT displacement over the",
            "window (>= 0.8 m) and assumes forward driving.",
            "",
            "| range | n | shape pub. | color pub. | scored | med dist legacy→mean→fit (m) | p90 legacy→mean→fit (m) "
            "| published-only p90 def→fit | fit better / worse (p90) | fwd = GT +x | fwd vs motion (n) | GT +x = motion | fit ms (median of means / max) |",
            "|---|---|---|---|---|---|---|---|---|---|---|---|---|",
        ]
        order = [_bucket(low) for low, _ in RANGE_BUCKETS] + [f"{RANGE_BUCKETS[-1][1]}+ m", "all"]
        for name in order:
            a = aggregate.get(name)
            if a is None:
                continue
            lines.append(
                f"| {name} | {a['vehicles']} | {_pct(a['shape_published_rate'])} | {_pct(a['color_published_rate'])} "
                f"| {a['scored_vehicles']} | {_f(a['heldout_median_default_m'])} → {_f(a.get('heldout_median_means_m'))} → {_f(a['heldout_median_fitted_m'])} "
                f"| {_f(a['heldout_p90_default_m'])} → {_f(a.get('heldout_p90_means_m'))} → {_f(a['heldout_p90_fitted_m'])} "
                f"| {_f(a['published_only_p90_default_m'])} → {_f(a['published_only_p90_fitted_m'])} "
                f"| {_pct(a['fitted_better_p90_rate'])} / {_pct(a['fitted_worse_p90_rate'])} "
                f"| {_pct(a['forward_sign_vs_gt_x'])} | {_pct(a['forward_sign_vs_motion'])} ({a['moving_with_shape']}) "
                f"| {_pct(a['gt_x_vs_motion'])} | {_f(a['fit_latency_ms_mean'], 0)} / {_f(a['fit_latency_ms_max'], 0)} |")
        body = aggregate.get("all", {}).get("body_types", {})
        lines += ["", f"Published body types: {', '.join(f'{k}={v}' for k, v in body.items())}.", ""]
        variants = self._variants()
        if variants:
            lines += ["## Library-config variants (all ranges)", "",
                      "These runs use the same frames and harness with experimental settings. Their summaries are",
                      "in `variants/<name>/summary.json`.", "",
                      "| variant | settings | shape pub. | color pub. | med dist def→fit | p90 def→fit | fit better / worse | fwd = GT +x | fwd vs motion | body types | fit ms (median / max) |",
                      "|---|---|---|---|---|---|---|---|---|---|---|"]
            for name, item in [("baseline (this review)", dict(settings=s, all=aggregate["all"]))] + list(variants.items()):
                a, vs = item["all"], item["settings"]
                flags = {k: vs[k] for k in ("history", "fitter", "color", "default_incumbent", "fixed_forward_sign",
                                            "bright_majority_color") if vs.get(k) not in (None, {}, False)}
                lines.append(
                    f"| {name} | `{json.dumps(flags)}` | {_pct(a['shape_published_rate'])} | {_pct(a['color_published_rate'])} "
                    f"| {_f(a['heldout_median_default_m'])} → {_f(a['heldout_median_fitted_m'])} "
                    f"| {_f(a['heldout_p90_default_m'])} → {_f(a['heldout_p90_fitted_m'])} "
                    f"| {_pct(a['fitted_better_p90_rate'])} / {_pct(a['fitted_worse_p90_rate'])} "
                    f"| {_pct(a['forward_sign_vs_gt_x'])} | {_pct(a['forward_sign_vs_motion'])} "
                    f"| {', '.join(f'{k}={v}' for k, v in a['body_types'].items())} "
                    f"| {_f(a['fit_latency_ms_mean'], 0)} / {_f(a['fit_latency_ms_max'], 0)} |")
            lines.append("")
        worst = sorted(self.frames, key=lambda f: -f["problem_score"])[:8]
        lines += ["## Worst cases (largest problem score first)", "",
                  "Problem score = 10 x (fitted - default held-out p90, m) when the fit is worse, +1 per forward-sign",
                  "disagreement with the GT front, +1.5 per warm (red/orange/brown) published tint, which is usually",
                  "tail-light or low-sun contamination but is real paint on red cars, +1 per near (<20 m) well-visible",
                  "vehicle without a color, and +1 per shape rejected despite >= 70 evidence voxels.", ""]
        for frame in worst:
            lines.append(f"- **[{frame['key']}](#{_anchor(frame['key'])})** score {frame['problem_score']}: "
                         + ("; ".join(frame["problems"]) or "-"))
        lines.append("")
        for frame in worst[:8]:
            lines += [f"**{frame['key']}** (baseline)", "", f"![{frame['key']} compare](frames/{frame['key']}/compare.png)", ""]
            for name in variants:
                path = f"variants/{name}/frames/{frame['key']}/compare.png"
                if (self.output / path).is_file():
                    lines += [f"{frame['key']}, variant `{name}`", "", f"![{frame['key']} {name}]({path})", ""]
        takes = sorted({f["take"] for f in self.frames})
        for take in takes:
            lines += [f"## Take {take:02d}", ""]
            for agent in sorted({f["agent"] for f in self.frames if f["take"] == take}):
                lines += [f"### Take {take:02d} / agent {agent}", ""]
                for frame in sorted((f for f in self.frames if f["take"] == take and f["agent"] == agent),
                                    key=lambda f: f["frame_index"]):
                    lines += self._frame_section(frame)
        return "\n".join(lines) + "\n"

    def _changes_since_baseline(self, aggregate: dict) -> list[str]:
        if self.baseline is None or not (self.baseline / "summary.json").is_file():
            return []
        base = json.loads((self.baseline / "summary.json").read_text())
        frames = {r["frame_key"] for r in self.rows}  # compare like with like: this run's frames only
        base["vehicles"] = [r for r in base["vehicles"] if r["frame_key"] in frames]
        old_rows = {(r["frame_key"], r["track_id"]): r for r in base["vehicles"]}
        new_rows = {(r["frame_key"], r["track_id"]): r for r in self.rows}
        relative = f"../{self.baseline.name}"
        old, new = ReviewReport.aggregate(base["vehicles"])["all"], aggregate["all"]
        near_old = ReviewReport.aggregate([r for r in base["vehicles"] if r["range_m"] < 10]).get("all", {})
        near_new = ReviewReport.aggregate([r for r in self.rows if r["range_m"] < 10]).get("all", {})
        bodies = [body_label_accuracy(base["vehicles"]), body_label_accuracy(self.rows)]

        def label(accuracy, body):
            return "/".join(str(value) for value in accuracy[body]) if body in accuracy else "–"

        def lstar(a):
            return f"{'/'.join(f'{v:.0f}' for v in a['color_L_percentiles'])}" if a.get("color_L_percentiles") else "–"

        published = lambda a, rows: f"{a['color_L_below_30']}/{sum(r['color_published'] for r in rows)}"

        def displayed(rows, albedo=displayed_albedo):
            values = _percentiles([_lightness(albedo(r["color_rgb"])) for r in rows if r["color_published"]])
            return "/".join(f"{v:.0f}" for v in values) if values else "–"

        def held_out(a, key):
            return " → ".join(_f(a.get(f"heldout_{key}_{name}_m")) for name in ("default", "means", "fitted"))

        def accuracy(rows):
            result = body_label_accuracy(rows)
            exact = sum(v[0] for v in result.values()), sum(v[2] for v in result.values())
            group = sum(v[1] for v in result.values())
            return f"{exact[0]}/{exact[1]} exact, {group}/{exact[1]} group"

        table = [
            ("held-out median dist legacy → type mean → fitted (m, point-to-triangle)", "(other metric)",
             held_out(new, "median")),
            ("held-out p90 dist legacy → type mean → fitted (m, point-to-triangle)", "(other metric)",
             held_out(new, "p90")),
            ("baseline's own held-out p90 (legacy-profile distance)", _f(old["heldout_p90_fitted_m"]), "–"),
            ("shape published (of which prior only)", _pct(old["shape_published_rate"]),
             f"{_pct(new['shape_published_rate'])} ({_pct(new.get('prior_only_rate'))})"),
            ("published body types", ", ".join(f"{k}={v}" for k, v in old["body_types"].items()),
             ", ".join(f"{k}={v}" for k, v in new["body_types"].items())),
            ("body type vs labels (high/medium confidence)", accuracy(base["vehicles"]), accuracy(self.rows)),
            *((f"  label {body}: exact / group / n", label(bodies[0], body), label(bodies[1], body))
              for body in sorted(set(bodies[0]) | set(bodies[1]))),
            ("color published", _pct(old["color_published_rate"]), _pct(new["color_published_rate"])),
            ("published color L* p10/p50/p90", lstar(old), lstar(new)),
            ("published colors with L* < 30", published(old, base["vehicles"]), published(new, self.rows)),
            ("fit ms, median of per-vehicle means / max", f"{_f(old['fit_latency_ms_mean'], 1)} / {_f(old['fit_latency_ms_max'], 0)}",
             f"{_f(new['fit_latency_ms_mean'], 1)} / {_f(new['fit_latency_ms_max'], 0)}"),
        ]
        lines = [f"## Changes since {self.baseline.name}", "",
                 f"Baseline: [`{relative}/REVIEW.md`]({relative}/REVIEW.md) (same frames and harness).", "",
                 *self.settings.get("changes", []), "",
                 "| metric (all ranges unless noted) | baseline | this run |", "|---|---|---|",
                 *(f"| {name} | {before} | {after} |" for name, before, after in table), "",
                 f"Body labels: `{BODY_LABELS.name}` (three labelers over every vehicle row; high/medium confidence, "
                 "unknown excluded). Wagons count as hatchbacks and coupes as sedans; group accuracy compares "
                 "car (sedan, hatchback) / SUV (crossover, full_suv, minivan) / van / pickup. The baseline's "
                 "hatch_suv counts as the SUV group.", ""]
        if self.callouts:
            lines += ["### Called-out vehicles", "",
                      "Each crop is real camera | baseline fitted+tint | this run's fitted+tint at frame *t*. The baseline",
                      "swatch is its published body color; this run's swatch is the published color (left) and the albedo",
                      "the renderer paints after its dark-paint lift (right). A crossed gray swatch means no color.", ""]
            for item in self.callouts:
                key, track = item["frame_key"], item["track_id"]
                before, after = old_rows.get((key, track)), new_rows.get((key, track))
                if before is None or after is None:
                    continue

                def describe(r):
                    shape = (f"{r['body_type']}{' (prior only)' if r.get('prior_only') else ''}"
                             if r["shape_published"] else "no shape")
                    color = f"{_hex(r['color_rgb'])} (L* {r['color_L']:.0f})" if r["color_published"] else "no color"
                    return f"{shape}, {color}"

                lines += [f"**{key} #{track}** ({after['range_m']:.0f} m): {item['note']}", "",
                          f"baseline: {describe(before)} → this run: {describe(after)}  ",
                          f"[baseline frame]({relative}/frames/{key}/compare.png) · [this run's frame](frames/{key}/compare.png)"
                          f" · [baseline vehicles]({relative}/frames/{key}/vehicles.png) · [vehicles](frames/{key}/vehicles.png)", "",
                          f"![{key} #{track}](frames/{key}/callout_{track}.png) "
                          f"![baseline swatch](frames/{key}/swatch_v1_{track}.png) ![swatch](frames/{key}/swatch_{track}.png)", ""]
                FrameFigures.swatch(before["color_rgb"] if before["color_published"] else None,
                                    self.output / "frames" / key / f"swatch_v1_{track}.png", displayed=False)
                FrameFigures.swatch(after["color_rgb"] if after["color_published"] else None,
                                    self.output / "frames" / key / f"swatch_{track}.png")
        return lines

    def _frame_section(self, frame: dict) -> list[str]:
        key = frame["key"]
        rows = [r for r in self.rows if r["frame_key"] == key]
        lines = [f"#### {key}", "",
                 f"{frame['split']} take {frame['take']} agent {frame['agent']}, frame index {frame['frame_index']} "
                 f"(id {frame['frame_id']}); {frame['evaluated_vehicles']} evaluated / {frame['rendered_vehicles']} rendered. "
                 f"Problems: {'; '.join(frame['problems']) or 'none'}", "",
                 f"![compare](frames/{key}/compare.png)", "",
                 f"![vehicles](frames/{key}/vehicles.png)", "",
                 "| id | range | vis | raw pts / vox | shape | fwd (motion) | med legacy→mean→fit | p90 legacy→mean→fit | color (conf, n) | fit ms |",
                 "|---|---|---|---|---|---|---|---|---|---|"]
        for r in rows:
            shape = (f"{r['body_type']}{' (prior)' if r.get('prior_only') else ''}" if r["shape_published"]
                     else f"– ({r['evidence_voxels']} ev)")
            fwd = "–" if r["forward_sign"] is None else f"{r['forward_sign']:+d}"
            motion = {1: "+x", -1: "-x", 0: "static"}[r["motion_front_sign"]]
            color = (f"{_hex(r['color_rgb'])} ({r['color_confidence']:.2f}, {r['color_fused_observations']})"
                     if r["color_published"] else f"– ({r['color_frames_confident']}/{r['color_frames_observed']})")
            lines.append(
                f"| {r['track_id']} | {r['range_m']:.1f} | {r['visible_fraction']:.2f} | {r['raw_points']} / {r['accumulator_voxels']} "
                f"| {shape} | {fwd} ({motion}) | {_f(r['heldout_default_median_m'])} → {_f(r.get('heldout_means_median_m'))} → {_f(r['heldout_fitted_median_m'])} "
                f"| {_f(r['heldout_default_p90_m'])} → {_f(r.get('heldout_means_p90_m'))} → {_f(r['heldout_fitted_p90_m'])} | {color} | {_f(r['fit_latency_ms_mean'], 0)} |")
        return lines + [""]


def _warm_tint(rgb) -> bool:
    """Red-orange-brown hue with some chroma: rare as paint, common as tail-light/low-sun contamination."""
    _, a, b = cv2.cvtColor(np.float32(rgb).reshape(1, 1, 3), cv2.COLOR_RGB2Lab)[0, 0]
    return bool(np.hypot(a, b) >= 15 and 15 < np.degrees(np.arctan2(b, a)) < 85)


_ASSET = []


def displayed_albedo(rgb) -> np.ndarray:
    """Body albedo the proxy renderer paints for ``rgb`` (vertex_colors_for_tint without baked shading)."""
    if not _ASSET:
        from src.viz.primitive_car import PrimitiveCarAsset

        _ASSET.append(PrimitiveCarAsset())
    asset = _ASSET[0]
    body = (asset._material_colors == asset.BODY).all(axis=1)
    colors = asset.vertex_colors_for_tint(rgb)[body, :3] / asset._shade[body, None]
    return np.clip(np.median(colors, axis=0) / 255, 0, 1)


def _lightness(rgb) -> float:
    return float(cv2.cvtColor(np.float32(rgb).reshape(1, 1, 3), cv2.COLOR_RGB2Lab)[0, 0, 0])


def _percentiles(values, q=(10, 50, 90)):
    values = [v for v in values if v is not None]
    return None if not values else [round(float(v), 1) for v in np.percentile(values, q)]


def body_label_accuracy(rows: list[dict]) -> dict[str, list[int]]:
    """{label: [exact type matches, coarse group matches, published]} over high/medium-confidence labels."""
    if not BODY_LABELS.is_file():
        return {}
    rows_by_id = {(r["frame_key"], r["track_id"]): r for r in rows}
    accuracy: dict[str, list[int]] = {}
    for label in json.loads(BODY_LABELS.read_text()):
        if label.get("confidence", "high") not in ("high", "medium") or label["body"] == "unknown":
            continue
        r = rows_by_id.get((label["frame_key"], str(label["track_id"])))
        if r is None or not r["shape_published"]:
            continue
        body = LABEL_TYPES.get(label["body"], label["body"])
        fitted = {"hatch_suv": "crossover"}.get(r["body_type"], r["body_type"])  # v1/v2 preset names
        counts = accuracy.setdefault(body, [0, 0, 0])
        counts[0] += r["body_type"] == body
        counts[1] += BODY_GROUPS.get(fitted) == BODY_GROUPS.get(body)
        counts[2] += 1
    return accuracy


def _median(values):
    values = [v for v in values if v is not None]
    return None if not values else round(float(np.median(values)), 4)


def _rate(count: int, total: int):
    return None if total == 0 else round(count / total, 3)


def _pct(value) -> str:
    return "–" if value is None else f"{100 * value:.0f}%"


def _f(value, digits: int = 3) -> str:
    return "–" if value is None else f"{value:.{digits}f}"


def _anchor(text: str) -> str:
    return text.lower()

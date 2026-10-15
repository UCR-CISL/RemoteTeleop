"""Figures (compare.png, vehicles.png), REVIEW.md and summary.json/csv for evaluate.py."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import cv2
import numpy as np

from src.viz.primitive_vehicle import CarShapeParams, PrimitiveVehicleAsset, car_section_tables

RANGE_BUCKETS = ((3, 10), (10, 20), (20, 30), (30, 45), (45, 60))
BODY_LABELS = Path(__file__).with_name("body_labels.json")  # visual body labels of near vehicles
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
        for image, title in ((real, f"real camera  {result.spec.key}"),
                             (result.render_default, "gsplat + DEFAULT primitive"),
                             (result.render_fitted, "gsplat + FITTED shape + TINT")):
            image = image.copy()
            self._labels(image, result, evaluated)
            cv2.rectangle(image, (0, 0), (len(title) * 13 + 14, 32), (0, 0, 0), -1)
            cv2.putText(image, title, (7, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)
            panels.append(image)
        separator = np.full((panels[0].shape[0], 6, 3), 255, np.uint8)
        cv2.imwrite(str(path), np.hstack([panels[0], separator, panels[1], separator, panels[2]]))

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
        widths = [2.4, 2.4, 2.4, 0.7, 2.6, 2.6, 3.4]
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
            for ax, image, title in ((row[1], result.render_default, "default"), (row[2], result.render_fitted, "fitted+tint")):
                r0, r1 = int(y0 * scale), max(int(y1 * scale), int(y0 * scale) + 2)
                c0, c1 = int(x0 * scale), max(int(x1 * scale), int(x0 * scale) + 2)
                ax.imshow(cv2.cvtColor(image[r0:r1, c0:c1], cv2.COLOR_BGR2RGB))
                ax.set_title(title, fontsize=8)
            self._swatch(row[3], v)
            self._profile(row[4], v, view="bev")
            self._profile(row[5], v, view="side")
            row[6].text(0, 1, self._text(v), va="top", ha="left", family="monospace", fontsize=7.2,
                        transform=row[6].transAxes)
            for ax in (row[0], row[1], row[2], row[3], row[6]):
                ax.set_xticks([])
                ax.set_yticks([])
            row[6].axis("off")
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
        old = old[r0:r1, 2 * (width + 6) + c0:2 * (width + 6) + c1] if old is not None else np.zeros_like(real)
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

    @staticmethod
    def _outline(shape: CarShapeParams, dims, view: str):
        """Body and cabin outlines (metric, object frame) for the BEV or side view."""
        length, width, height = dims
        polygons = []
        for sections in car_section_tables(shape):
            x = sections[:, 0] * length * shape.forward_sign
            if view == "bev":
                half = sections[:, 1] * width
                polygons.append((np.r_[x, x[::-1], x[:1]], np.r_[half, -half[::-1], half[:1]]))
            else:
                bottom = PrimitiveVehicleAsset.car_profile_z(sections[:, 2], dims) - height / 2
                top = PrimitiveVehicleAsset.car_profile_z(sections[:, 3], dims) - height / 2
                polygons.append((np.r_[x, x[::-1], x[:1]], np.r_[top, bottom[::-1], top[:1]]))
        return polygons

    def _profile(self, ax, v, *, view: str) -> None:
        dims = np.asarray(v.dims_lwh)
        column = 1 if view == "bev" else 2
        if v.accumulated_points is not None and len(v.accumulated_points):
            ax.scatter(v.accumulated_points[:, 0], v.accumulated_points[:, column], s=2 + 2 * v.accumulated_weights,
                       c="tab:blue", alpha=0.5, linewidths=0)
        if v.heldout_xyz is not None and len(v.heldout_xyz):
            ax.scatter(v.heldout_xyz[:, 0], v.heldout_xyz[:, column], s=2, c="tab:orange", alpha=0.7, linewidths=0)
        for polygon in self._outline(CarShapeParams(), dims, view):
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
            ax.set_title("side  green=fit, dashed=default", fontsize=7)
        ax.tick_params(labelsize=6)

    @staticmethod
    def _text(v) -> str:
        shape = v.published_shape
        fit = (f"{shape.body_type} fwd={int(shape.forward_sign):+d}" if shape is not None else "not published")
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
            f"  accepted {v.accepted_steps}/{v.n_fits} steps, rms {v.fit_rms_m:.3f}",
            f"  cold: {v.cold_fit.get('body_type')} {v.cold_fit.get('forward_sign')} acc={v.cold_fit.get('accepted')}",
            f"  GT motion {v.gt_motion_m:.1f} m -> front {motion}",
            f"  latency mean/max {lat} ms",
            f"held-out ({v.heldout_points} pts) med/p90 m:",
            f"  default {dd[0]:.3f}/{dd[1]:.3f}",
            f"  fitted  {df[0]:.3f}/{df[1]:.3f}",
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
            median = lambda key, rs: _median([r[key] for r in rs])
            out[name] = dict(
                vehicles=len(group),
                shape_published_rate=_rate(len(published), len(group)),
                color_published_rate=_rate(sum(r["color_published"] for r in group), len(group)),
                scored_vehicles=len(scored),
                heldout_median_default_m=median("heldout_default_median_m", scored),
                heldout_median_fitted_m=median("heldout_fitted_median_m", scored),
                heldout_p90_default_m=median("heldout_default_p90_m", scored),
                heldout_p90_fitted_m=median("heldout_fitted_p90_m", scored),
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
                body_types={t: sum(r["body_type"] == t for r in published) for t in CarShapeParams.BODY_TYPES},
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
            "`ObjectPointAccumulator` stores them and `CarShapeFitter` refits after every scan, warm-started",
            "from the last accepted fit, as a live per-poll refit would; the last accepted fit is published.",
            "`VehicleColorEstimator.observe` runs on each history frame's native 1920x1200 camera image with",
            "the refined calibration, a full-scan `SparseDepthBuffer`, and nearer GT boxes as occluders.",
            "Frame *t* itself is held out: its in-box points give one-sided point-to-surface distances",
            "(vertical/lateral, as in `surface_distance`) for the default sedan and for the published fit.",
            "",
            "Each frame has two images. `compare.png` shows real camera | default primitive | fitted+tinted.",
            "Track IDs appear in yellow for evaluated vehicles and gray for those beyond 60 m.",
            "`vehicles.png` has one row per vehicle: a real crop with belt-band LiDAR samples (yellow = passed",
            "the depth test, red = rejected), the two render crops, a color swatch, BEV and side views, and metrics.",
            "In the BEV and side views, blue = accumulated voxels, orange = held-out frame *t*,",
            "dashed = default outline, green = fitted outline, and the arrow marks GT +x.",
            "The swatch's bottom quarter is a diagnostic: the published color white-balanced so that the road",
            "ring around the car reads neutral. It is not used in the render.",
            "",
            "Command (worktree root):",
            "```",
            "PYTHONPATH=. ../RemoteTeleop/.venv/bin/python scripts/investigation/vehicle_shape/evaluate.py",
            "```",
            f"Settings: `{json.dumps({k: s[k] for k in ('history', 'fitter', 'accumulator', 'color', 'calibration', 'default_incumbent')})}`",
            "",
            *self._changes_since_baseline(aggregate),
            "## Aggregate (per range bucket)",
            "",
            f"Distances are medians over vehicles with >= {MIN_HELDOUT_POINTS} held-out points. A vehicle with no",
            "published shape scores the default profile in both columns. The 'published only' columns",
            "cover vehicles with a published shape. forward_sign vs motion uses GT displacement over the",
            "window (>= 0.8 m) and assumes forward driving.",
            "",
            "| range | n | shape pub. | color pub. | scored | med dist def→fit (m) | p90 def→fit (m) "
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
                f"| {a['scored_vehicles']} | {_f(a['heldout_median_default_m'])} → {_f(a['heldout_median_fitted_m'])} "
                f"| {_f(a['heldout_p90_default_m'])} → {_f(a['heldout_p90_fitted_m'])} "
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
        old_rows = {(r["frame_key"], r["track_id"]): r for r in base["vehicles"]}
        new_rows = {(r["frame_key"], r["track_id"]): r for r in self.rows}
        relative = f"../{self.baseline.name}"
        old, new = ReviewReport.aggregate(base["vehicles"])["all"], aggregate["all"]
        near_old = ReviewReport.aggregate([r for r in base["vehicles"] if r["range_m"] < 10]).get("all", {})
        near_new = ReviewReport.aggregate([r for r in self.rows if r["range_m"] < 10]).get("all", {})
        bodies = [body_label_accuracy(base["vehicles"]), body_label_accuracy(self.rows)]

        def label(accuracy, body):
            return f"{accuracy[body][0]}/{accuracy[body][1]}" if body in accuracy else "–"

        def lstar(a):
            return f"{'/'.join(f'{v:.0f}' for v in a['color_L_percentiles'])}" if a.get("color_L_percentiles") else "–"

        published = lambda a, rows: f"{a['color_L_below_30']}/{sum(r['color_published'] for r in rows)}"

        def displayed(rows, albedo=displayed_albedo):
            values = _percentiles([_lightness(albedo(r["color_rgb"])) for r in rows if r["color_published"]])
            return "/".join(f"{v:.0f}" for v in values) if values else "–"

        table = [
            ("forward_sign = GT +x (all / 3-10 m)", f"{_pct(old['forward_sign_vs_gt_x'])} / {_pct(near_old.get('forward_sign_vs_gt_x'))}",
             f"{_pct(new['forward_sign_vs_gt_x'])} / {_pct(near_new.get('forward_sign_vs_gt_x'))}"),
            ("forward_sign vs GT motion", _pct(old["forward_sign_vs_motion"]), _pct(new["forward_sign_vs_motion"])),
            ("held-out median dist, fitted (m)", _f(old["heldout_median_fitted_m"]), _f(new["heldout_median_fitted_m"])),
            ("held-out p90 dist, fitted (m)", _f(old["heldout_p90_fitted_m"]), _f(new["heldout_p90_fitted_m"])),
            ("fit better / worse than default (p90)", f"{_pct(old['fitted_better_p90_rate'])} / {_pct(old['fitted_worse_p90_rate'])}",
             f"{_pct(new['fitted_better_p90_rate'])} / {_pct(new['fitted_worse_p90_rate'])}"),
            ("shape published", _pct(old["shape_published_rate"]), _pct(new["shape_published_rate"])),
            ("published body types", ", ".join(f"{k}={v}" for k, v in old["body_types"].items()),
             ", ".join(f"{k}={v}" for k, v in new["body_types"].items())),
            *((f"body type vs visual label: {body}", label(bodies[0], body), label(bodies[1], body))
              for body in ("sedan", "suv_hatch", "pickup")),
            ("color published", _pct(old["color_published_rate"]), _pct(new["color_published_rate"])),
            ("published color L* p10/p50/p90", lstar(old), lstar(new)),
            ("published colors with L* < 30", published(old, base["vehicles"]), published(new, self.rows)),
            ("rendered body albedo L* p10/p50/p90", displayed(base["vehicles"], _v1_albedo), displayed(self.rows)),
            ("warm (red/orange/brown) published tints", str(sum(r["color_published"] and _warm_tint(r["color_rgb"])
                                                               for r in base["vehicles"])),
             str(sum(r["color_published"] and _warm_tint(r["color_rgb"]) for r in self.rows))),
            ("fit ms, median of per-vehicle means / max", f"{_f(old['fit_latency_ms_mean'], 1)} / {_f(old['fit_latency_ms_max'], 0)}",
             f"{_f(new['fit_latency_ms_mean'], 1)} / {_f(new['fit_latency_ms_max'], 0)}"),
        ]
        lines = ["## Changes since v1", "",
                 f"Baseline: [`{relative}/REVIEW.md`]({relative}/REVIEW.md) (same frames and harness).", "",
                 *self.settings.get("changes", []), "",
                 "| metric (all ranges unless noted) | v1 | v2 |", "|---|---|---|",
                 *(f"| {name} | {before} | {after} |" for name, before, after in table), "",
                 f"Body labels: `{BODY_LABELS.name}` holds visual labels of near (< 25 m) vehicles from the camera crops;",
                 "pickups count as correct when fitted as hatch_suv (there is no pickup preset).", ""]
        if self.callouts:
            lines += ["### Called-out vehicles", "",
                      "Each crop is real camera | v1 fitted+tint | v2 fitted+tint at frame *t*. The v1 swatch is its published",
                      "body color; the v2 swatch is the published color (left) and the albedo v2's renderer paints after its",
                      "dark-paint lift (right). A crossed gray swatch means no color was published.", ""]
            for item in self.callouts:
                key, track = item["frame_key"], item["track_id"]
                before, after = old_rows.get((key, track)), new_rows.get((key, track))
                if before is None or after is None:
                    continue

                def describe(r):
                    shape = f"{r['body_type']} fwd {r['forward_sign']:+d}" if r["shape_published"] else "no shape"
                    color = f"{_hex(r['color_rgb'])} (L* {r['color_L']:.0f})" if r["color_published"] else "no color"
                    return f"{shape}, {color}"

                lines += [f"**{key} #{track}** ({after['range_m']:.0f} m): {item['note']}", "",
                          f"v1: {describe(before)} → v2: {describe(after)}  ",
                          f"[v1 frame]({relative}/frames/{key}/compare.png) · [v2 frame](frames/{key}/compare.png)", "",
                          f"![{key} #{track}](frames/{key}/callout_{track}.png) "
                          f"![v1 swatch](frames/{key}/swatch_v1_{track}.png) ![v2 swatch](frames/{key}/swatch_{track}.png)", ""]
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
                 "| id | range | vis | raw pts / vox | shape | fwd (motion) | med def→fit | p90 def→fit | color (conf, n) | fit ms |",
                 "|---|---|---|---|---|---|---|---|---|---|"]
        for r in rows:
            shape = f"{r['body_type']}" if r["shape_published"] else f"– ({r['evidence_voxels']} ev)"
            fwd = "–" if r["forward_sign"] is None else f"{r['forward_sign']:+d}"
            motion = {1: "+x", -1: "-x", 0: "static"}[r["motion_front_sign"]]
            color = (f"{_hex(r['color_rgb'])} ({r['color_confidence']:.2f}, {r['color_fused_observations']})"
                     if r["color_published"] else f"– ({r['color_frames_confident']}/{r['color_frames_observed']})")
            lines.append(
                f"| {r['track_id']} | {r['range_m']:.1f} | {r['visible_fraction']:.2f} | {r['raw_points']} / {r['accumulator_voxels']} "
                f"| {shape} | {fwd} ({motion}) | {_f(r['heldout_default_median_m'])} → {_f(r['heldout_fitted_median_m'])} "
                f"| {_f(r['heldout_default_p90_m'])} → {_f(r['heldout_fitted_p90_m'])} | {color} | {_f(r['fit_latency_ms_mean'], 0)} |")
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


def _v1_albedo(rgb) -> np.ndarray:
    """v1's vertex_colors_for_tint lift (gray added up to luma 0.16), for the before/after table."""
    rgb = np.clip(np.asarray(rgb, dtype=float), 0., 1.)
    luma = rgb @ (.299, .587, .114)
    return np.clip(rgb + max(.16 - luma, 0.), 0., 1.) * min(1., .92 / max(luma, .16))


def _lightness(rgb) -> float:
    return float(cv2.cvtColor(np.float32(rgb).reshape(1, 1, 3), cv2.COLOR_RGB2Lab)[0, 0, 0])


def _percentiles(values, q=(10, 50, 90)):
    values = [v for v in values if v is not None]
    return None if not values else [round(float(v), 1) for v in np.percentile(values, q)]


def body_label_accuracy(rows: list[dict]) -> dict[str, list[int]]:
    """{label: [published body types that match, published]}; pickups count as hatch_suv matches."""
    if not BODY_LABELS.is_file():
        return {}
    rows_by_id = {(r["frame_key"], r["track_id"]): r for r in rows}
    accuracy: dict[str, list[int]] = {}
    for label in json.loads(BODY_LABELS.read_text()):
        r = rows_by_id.get((label["frame_key"], label["track_id"]))
        if r is None or not r["shape_published"]:
            continue
        counts = accuracy.setdefault(label["body"], [0, 0])
        counts[0] += (r["body_type"] == "sedan") == (label["body"] == "sedan")
        counts[1] += 1
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

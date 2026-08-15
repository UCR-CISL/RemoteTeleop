"""Build dataset-time and wall-clock timelines from realtime JSONL metrics."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


@dataclass(frozen=True)
class Sam3DJob:
    request_id: str
    track_id: str
    status: str
    request_wall_us: float
    reconstruction_start_wall_us: float
    completion_wall_us: float
    queue_latency_ms: float
    reconstruction_latency_ms: float
    request_to_mesh_ms: float


@dataclass(frozen=True)
class TimelineData:
    replay: tuple[dict[str, Any], ...]
    compositor_frames: tuple[dict[str, Any], ...]
    compositor_assets: tuple[dict[str, Any], ...]
    sam3_frames: tuple[dict[str, Any], ...]
    sam3d_jobs: tuple[Sam3DJob, ...]
    wall_origin_us: float

    @property
    def replay_by_frame(self) -> dict[str, dict[str, Any]]:
        return {str(row["frame_id"]): row for row in self.replay}

    def wall_seconds(self, timestamp_us: float) -> float:
        return (timestamp_us - self.wall_origin_us) / 1_000_000.0

    def dataset_seconds_for_wall(self, timestamp_us: float) -> float:
        wall = np.asarray([float(row["recorded_at_us"]) for row in self.replay])
        source = np.asarray([float(row["source_timestamp_us"]) / 1_000_000 for row in self.replay])
        return float(np.interp(timestamp_us, wall, source, left=source[0], right=source[-1]))


class RealtimeTimelineAnalyzer:
    """Load, join, summarize, and plot one completed runtime metrics directory."""

    def __init__(self, metrics_dir: str | Path) -> None:
        self.metrics_dir = Path(metrics_dir)

    def load(self) -> TimelineData:
        replay = tuple(
            row for row in self._read_jsonl("replay.jsonl")
            if row.get("event") == "frame_published"
        )
        if not replay:
            raise ValueError("replay.jsonl contains no frame_published records")
        compositor = tuple(self._read_jsonl("compositor.jsonl"))
        sam3 = tuple(self._read_jsonl("sam3.jsonl"))
        sam3d = tuple(self._read_jsonl("sam3d.jsonl"))
        jobs = tuple(
            self._parse_job(row) for row in sam3d
            if row.get("request_id") and row.get("timestamp_us") is not None
        )
        return TimelineData(
            replay=replay,
            compositor_frames=tuple(
                row for row in compositor if row.get("event") == "compositor_frame"
            ),
            compositor_assets=tuple(
                row for row in compositor if row.get("event") == "compositor_asset"
            ),
            sam3_frames=tuple(
                row for row in sam3
                if row.get("event") == "mask_batch" and not row.get("end_of_scene")
            ),
            sam3d_jobs=jobs,
            wall_origin_us=float(replay[0]["recorded_at_us"]),
        )

    def write(self, output_dir: str | Path) -> dict[str, Path]:
        output = Path(output_dir)
        output.mkdir(parents=True, exist_ok=True)
        data = self.load()
        dataset_plot = output / "dataset_time_timeline.png"
        wall_plot = output / "wall_clock_timeline.png"
        fps_plot = output / "fps_timeline.png"
        summary_path = output / "timeline_summary.json"
        self._plot(data, dataset_plot, axis="dataset")
        self._plot(data, wall_plot, axis="wall")
        self._plot_fps(data, fps_plot)
        summary_path.write_text(json.dumps(self.summarize(data), indent=2), encoding="utf-8")
        return {
            "dataset_plot": dataset_plot,
            "wall_plot": wall_plot,
            "fps_plot": fps_plot,
            "summary": summary_path,
        }

    def summarize(self, data: TimelineData) -> dict[str, Any]:
        replay_ids = [str(row["frame_id"]) for row in data.replay]
        rendered_ids = {str(row["frame_id"]) for row in data.compositor_frames}
        missing = [frame_id for frame_id in replay_ids if frame_id not in rendered_ids]
        reported_drops = sum(int(row.get("discarded_frames", 0)) for row in data.compositor_frames)
        sam3_drops = sum(int(row.get("discarded_input_frames", 0)) for row in data.sam3_frames)
        inferred = self._asset_transitions(data)
        admissions = self._admission_sources(data)
        jobs = [self._job_summary(data, job) for job in data.sam3d_jobs]
        for job in jobs:
            admission = admissions.get(job["track_id"])
            if admission is not None:
                job.update(admission)
        compositor_drop_markers = [
            {
                "retained_frame_id": str(row["frame_id"]),
                "discarded_frames": int(row.get("discarded_frames", 0)),
                "dataset_seconds": float(row["timestamp_us"]) / 1_000_000,
                "wall_seconds": data.wall_seconds(float(row["recorded_at_us"])),
            }
            for row in data.compositor_frames if int(row.get("discarded_frames", 0)) > 0
        ]
        replay_fps = _reciprocal_ms(data.replay, "publish_interval_ms")
        render_capacity_fps = _reciprocal_ms(data.compositor_frames, "render_ms")
        compositor_output_fps = _event_fps(data.compositor_frames)
        active_sam3 = [
            row for row in data.sam3_frames if float(row.get("inference_ms", 0)) > 0
        ]
        sam3_capacity_fps = _reciprocal_ms(active_sam3, "inference_ms")
        return {
            "schema_version": 1,
            "input_metrics_dir": str(self.metrics_dir.resolve()),
            "dataset": {
                "published_frames": len(data.replay),
                "rendered_frames": len(data.compositor_frames),
                "source_duration_seconds": (
                    float(data.replay[-1]["source_timestamp_us"])
                    - float(data.replay[0]["source_timestamp_us"])
                ) / 1_000_000,
                "first_frame_id": replay_ids[0],
                "last_frame_id": replay_ids[-1],
                "missing_render_frame_ids": missing,
                "compositor_reported_discarded_frames": reported_drops,
                "compositor_drop_markers": compositor_drop_markers,
                "sam3_reported_discarded_input_frames": sam3_drops,
            },
            "wall_clock": {
                "replay_duration_seconds": data.wall_seconds(float(data.replay[-1]["recorded_at_us"])),
                "compositor_duration_seconds": data.wall_seconds(
                    float(data.compositor_frames[-1]["recorded_at_us"])
                ) if data.compositor_frames else None,
            },
            "latency_ms": {
                "replay_build": _statistics(data.replay, "build_latency_ms"),
                "replay_publish_interval": _statistics(data.replay, "publish_interval_ms"),
                "replay_frame_metadata_load": _statistics(
                    data.replay, "frame_metadata_load_ms"
                ),
                "replay_camera_read_decode": _statistics(
                    data.replay, "camera_read_decode_ms"
                ),
                "replay_pose_box_projection": _statistics(
                    data.replay, "pose_box_projection_ms"
                ),
                "replay_jpeg_encode": _statistics(data.replay, "jpeg_encode_ms"),
                "replay_pcd_read": _statistics(data.replay, "pcd_read_ms"),
                "replay_build_uninstrumented_residual": _statistics(
                    [
                        {
                            "value": float(row["build_latency_ms"])
                            - float(row.get("camera_read_decode_ms", 0))
                            - float(row.get("pose_box_projection_ms", 0))
                            - float(row.get("jpeg_encode_ms", 0))
                        }
                        for row in data.replay
                    ],
                    "value",
                ),
                "compositor_render": _statistics(data.compositor_frames, "render_ms"),
                "sam3_image_encode": _statistics(data.sam3_frames, "image_encode_ms", positive=True),
                "sam3_prompt_decode": _statistics(data.sam3_frames, "prompt_decode_ms", positive=True),
                "sam3_inference": _statistics(data.sam3_frames, "inference_ms", positive=True),
                "sam3_process_end_to_end_all_frames": _statistics(
                    data.sam3_frames, "process_end_to_end_ms"
                ),
                "sam3_process_end_to_end_active": _statistics(
                    [row for row in data.sam3_frames if float(row.get("inference_ms", 0)) > 0],
                    "process_end_to_end_ms",
                ),
                "sam3d_queue": _statistics(jobs, "queue_latency_ms"),
                "sam3d_reconstruction": _statistics(jobs, "reconstruction_latency_ms"),
                "sam3d_reconstruction_active": _statistics(
                    jobs, "reconstruction_latency_ms", positive=True
                ),
                "sam3d_request_to_mesh": _statistics(jobs, "request_to_mesh_ms"),
            },
            "fps": {
                "nominal_dataset_rate": 10.0,
                "replay_publication": _value_statistics(replay_fps),
                "compositor_output": _value_statistics(compositor_output_fps),
                "compositor_render_capacity": _value_statistics(render_capacity_fps),
                "sam3_active_inference_capacity": _value_statistics(sam3_capacity_fps),
                "interpretation": {
                    "replay_publication": "Reciprocal of publish_interval_ms; this is achieved source publication cadence.",
                    "compositor_output": "Reciprocal of consecutive compositor recorded_at_us intervals; this is achieved output delivery cadence.",
                    "compositor_render_capacity": "Reciprocal of render_ms; stage-local capacity, not end-to-end output FPS.",
                    "sam3_active_inference_capacity": "Reciprocal of inference_ms on active inference events only; sparse stage-local capacity.",
                },
            },
            "sam3d": {
                "job_count": len(jobs),
                "status_counts": _counts(job["status"] for job in jobs),
                "jobs": jobs,
                "dataset_axis_note": "Admission frame/source time is exact when one submitted track is identifiable; execution intervals are inferred by interpolating wall events against replay progress.",
            },
            "proxy_to_mesh": inferred,
            "instrumentation_limits": [
                "build_latency_ms combines camera read/decode, pose/calibration and box transformation/projection, JPEG encoding, message construction, and publisher.send; this replay path does not load or transform a LiDAR PCD inside the timer.",
                "render_ms combines GSplat rendering, proxy/mesh compositing, and PNG/video output work.",
                "SAM3 image_encode_ms and prompt_decode_ms overlap conceptually with inference_ms; inference_ms is their enclosing total, not an additive third stage.",
                "SAM3D request/start wall times are inferred from completion timestamp minus logged durations; no explicit start event is recorded.",
                "Compositor discard counts identify how many frames were superseded, while missing frame IDs are inferred by replay/compositor set difference.",
                "Asset events recover queued/running/ready/cancelled state, but do not carry frame_id or source time.",
            ],
            "recommended_timers": {
                "replay": [
                    "camera_read_decode_ms",
                    "pose_calibration_ms",
                    "box_transform_ms",
                    "box_projection_ms",
                    "jpeg_encode_ms",
                    "publish_send_ms",
                    "cadence_wait_ms",
                ],
                "compositor": [
                    "message_decode_ms",
                    "gsplat_render_ms",
                    "proxy_projection_ms",
                    "mesh_raster_ms",
                    "depth_composite_ms",
                    "png_encode_write_ms",
                    "video_encode_write_ms",
                ],
                "sam3d": [
                    "request_received_at_us",
                    "reconstruction_started_at_us",
                    "depth_pointmap_ms",
                    "stage1_ms",
                    "stage2_ms",
                    "mesh_decode_extract_ms",
                    "alignment_serialization_ms",
                ],
            },
        }

    def _plot(self, data: TimelineData, path: Path, *, axis: str) -> None:
        figure, axes = plt.subplots(
            4, 1, figsize=(18, 14), constrained_layout=True,
            gridspec_kw={"height_ratios": (1.1, 1.0, 1.2, 1.8)},
        )
        x_label = "Dataset time (s; source_timestamp_us)" if axis == "dataset" else "Wall time since first publish (s)"
        replay_x = self._record_x(data, data.replay, axis, source_key="source_timestamp_us")
        self._plot_series_with_roll(
            axes[0], replay_x, data.replay, "build_latency_ms", "Replay build", "tab:blue"
        )
        self._plot_series_with_roll(
            axes[0], replay_x, data.replay, "publish_interval_ms", "Publish interval", "tab:orange"
        )
        for key, label, color in (
            ("frame_metadata_load_ms", "Metadata load", "tab:purple"),
            ("camera_read_decode_ms", "Camera read/decode", "tab:green"),
            ("jpeg_encode_ms", "JPEG encode", "tab:red"),
        ):
            if any(row.get(key) is not None for row in data.replay):
                values = np.asarray([float(row.get(key, np.nan)) for row in data.replay])
                axes[0].plot(
                    replay_x, _rolling_quantile(values, 31, 0.5), color=color,
                    linewidth=1.1, label=f"{label} rolling median",
                )
        axes[0].set_ylabel("Replay latency (ms)")
        axes[0].set_title("Per-frame replay preparation and publication cadence")

        compositor_x = self._record_x(
            data, data.compositor_frames, axis, source_key="timestamp_us"
        )
        self._plot_series_with_roll(
            axes[1], compositor_x, data.compositor_frames, "render_ms", "Compositor render", "tab:green"
        )
        self._plot_drop_markers(data, axes[1], axis)
        axes[1].set_ylabel("Render latency (ms)")
        axes[1].set_title("GSplat/compositing/output work and superseded frames")

        sam3_rows = [row for row in data.sam3_frames if str(row.get("frame_id")) in data.replay_by_frame]
        sam3_x = self._record_x(data, sam3_rows, axis, frame_join=True)
        for key, label, color in (
            ("image_encode_ms", "Image encode", "tab:purple"),
            ("prompt_decode_ms", "Prompt decode", "tab:brown"),
            ("inference_ms", "Inference total", "tab:red"),
            ("process_end_to_end_ms", "Process end-to-end", "tab:cyan"),
        ):
            values = np.asarray([float(row.get(key, 0)) for row in sam3_rows])
            active = values > 0
            axes[2].scatter(np.asarray(sam3_x)[active], values[active], s=22, alpha=0.8, label=label, color=color)
        sam3_discard = [row for row in sam3_rows if int(row.get("discarded_input_frames", 0)) > 0]
        if sam3_discard:
            drop_x = self._record_x(data, sam3_discard, axis, frame_join=True)
            axes[2].scatter(drop_x, [0] * len(drop_x), marker="x", s=70, color="black", label="SAM3 input superseded")
        axes[2].set_ylabel("SAM3 latency (ms)")
        axes[2].set_title("Sparse SAM3 admission/inference frames (inference encloses encode + decode)")
        active_rows = [row for row in sam3_rows if float(row.get("inference_ms", 0)) > 0]
        if active_rows:
            active_x = self._record_x(data, active_rows, axis, frame_join=True)
            active_values = np.asarray([float(row["inference_ms"]) for row in active_rows])
            axes[2].plot(active_x, _rolling_quantile(active_values, 7, 0.5),
                         color="darkred", linewidth=1.8, label="Active inference rolling median")
            axes[2].plot(active_x, _rolling_quantile(active_values, 7, 0.95),
                         color="darkred", linewidth=1.2, linestyle="--",
                         label="Active inference rolling P95")
        axes[2].legend(ncol=5, fontsize=8, loc="upper left")

        self._plot_jobs(data, axes[3], axis)
        axes[3].set_xlabel(x_label)
        axes[3].set_ylabel("SAM3D track")
        axes[3].set_title("Asynchronous per-object SAM3D jobs: queue + reconstruction")
        for current in axes:
            current.grid(True, alpha=0.2)
            current.legend(fontsize=8, loc="upper right", ncol=3)
            if axis == "dataset":
                current.set_xlim(0, float(data.replay[-1]["source_timestamp_us"]) / 1_000_000)
        if axis == "dataset":
            tick_seconds = np.linspace(0, float(data.replay[-1]["source_timestamp_us"]) / 1_000_000, 6)
            frame_rows = data.replay
            frame_labels = []
            for second in tick_seconds:
                nearest = min(frame_rows, key=lambda row: abs(float(row["source_timestamp_us"]) / 1_000_000 - second))
                frame_labels.append(f"{second:.0f}s · {nearest['frame_id']}")
            axes[3].set_xticks(tick_seconds, frame_labels)
            axes[3].set_xlabel("Dataset time (s) / frame ID")
        title = "Dataset-time timeline" if axis == "dataset" else "Wall-clock execution and contention timeline"
        figure.suptitle(title, fontsize=16)
        figure.savefig(path, dpi=150)
        plt.close(figure)

    def _plot_fps(self, data: TimelineData, path: Path) -> None:
        figure, axes = plt.subplots(2, 1, figsize=(18, 9), constrained_layout=True)
        for axes_index, axis in enumerate(("dataset", "wall")):
            current = axes[axes_index]
            replay_x = self._record_x(
                data, data.replay, axis, source_key="source_timestamp_us"
            )
            replay_fps = _reciprocal_ms(data.replay, "publish_interval_ms")
            self._plot_fps_series(
                current, replay_x, replay_fps, "Replay publication", "tab:blue", 31
            )

            compositor_x = self._record_x(
                data, data.compositor_frames, axis, source_key="timestamp_us"
            )
            output_fps = _event_fps(data.compositor_frames)
            self._plot_fps_series(
                current, compositor_x[1:], output_fps,
                "Compositor output delivery", "tab:green", 31,
            )
            render_capacity = _reciprocal_ms(data.compositor_frames, "render_ms")
            self._plot_fps_series(
                current, compositor_x, render_capacity,
                "Render-only capacity", "tab:orange", 31, alpha=0.12,
            )

            active_sam3 = [
                row for row in data.sam3_frames
                if float(row.get("inference_ms", 0)) > 0
                and str(row.get("frame_id")) in data.replay_by_frame
            ]
            active_x = self._record_x(data, active_sam3, axis, frame_join=True)
            sam3_capacity = _reciprocal_ms(active_sam3, "inference_ms")
            current.scatter(
                active_x, sam3_capacity, s=22, color="tab:purple", alpha=0.55,
                label="Active SAM3 inference capacity (sparse)",
            )
            current.plot(
                active_x, _rolling_quantile(sam3_capacity, 7, 0.5),
                color="tab:purple", linewidth=1.8,
                label="Active SAM3 rolling median",
            )
            current.axhline(
                10.0, color="black", linestyle=":", linewidth=1.5,
                label="Nominal dataset rate (10 FPS)",
            )
            self._plot_fps_drop_markers(data, current, axis)
            current.set_ylim(bottom=0)
            current.set_ylabel("Frames/s or inferences/s")
            current.grid(True, alpha=0.2)
            current.legend(fontsize=8, ncol=3, loc="upper right")
            if axis == "dataset":
                duration = float(data.replay[-1]["source_timestamp_us"]) / 1_000_000
                current.set_xlim(0, duration)
                ticks = np.linspace(0, duration, 6)
                labels = []
                for second in ticks:
                    nearest = min(
                        data.replay,
                        key=lambda row: abs(
                            float(row["source_timestamp_us"]) / 1_000_000 - second
                        ),
                    )
                    labels.append(f"{second:.0f}s · {nearest['frame_id']}")
                current.set_xticks(ticks, labels)
                current.set_xlabel("Dataset time (s) / frame ID")
                current.set_title("Throughput aligned to the 50-second source take")
            else:
                current.set_xlabel("Wall time since first publish (s)")
                current.set_title("Throughput over actual execution and contention")
        figure.suptitle(
            "Realtime throughput: achieved cadence vs stage-local capacity", fontsize=16
        )
        figure.savefig(path, dpi=150)
        plt.close(figure)

    @staticmethod
    def _plot_fps_series(
        axes: Any, x: list[float], values: np.ndarray, label: str, color: str,
        window: int, *, alpha: float = 0.2,
    ) -> None:
        axes.plot(x, values, color=color, alpha=alpha, linewidth=0.8)
        axes.plot(
            x, _rolling_quantile(values, window, 0.5), color=color,
            linewidth=1.8, label=f"{label} rolling median",
        )
        axes.plot(
            x, _rolling_quantile(values, window, 0.05), color=color,
            linewidth=1.2, linestyle="--", label=f"{label} rolling P05",
        )

    def _plot_fps_drop_markers(self, data: TimelineData, axes: Any, axis: str) -> None:
        reports = [
            row for row in data.compositor_frames
            if int(row.get("discarded_frames", 0)) > 0
        ]
        for index, row in enumerate(reports):
            x = (
                data.wall_seconds(float(row["recorded_at_us"]))
                if axis == "wall"
                else float(row["timestamp_us"]) / 1_000_000
            )
            axes.axvline(
                x, color="red", alpha=0.65, linestyle="--", linewidth=1.1,
                label="Superseded frame" if index == 0 else None,
            )

    def _plot_series_with_roll(
        self, axes: Any, x: list[float], rows: Iterable[dict[str, Any]],
        key: str, label: str, color: str,
    ) -> None:
        values = np.asarray([float(row[key]) for row in rows])
        axes.plot(x, values, color=color, alpha=0.22, linewidth=0.8)
        median_values = _rolling_quantile(values, 31, 0.5)
        p95_values = _rolling_quantile(values, 31, 0.95)
        axes.plot(x, median_values, color=color, linewidth=1.8, label=f"{label} rolling median")
        axes.plot(x, p95_values, color=color, linewidth=1.2, linestyle="--", label=f"{label} rolling P95")

    def _plot_drop_markers(self, data: TimelineData, axes: Any, axis: str) -> None:
        rendered = {str(row["frame_id"]) for row in data.compositor_frames}
        missing = [row for row in data.replay if str(row["frame_id"]) not in rendered]
        reports = [row for row in data.compositor_frames if int(row.get("discarded_frames", 0)) > 0]
        ceiling = max((float(row["render_ms"]) for row in data.compositor_frames), default=1.0)
        if missing:
            x = self._record_x(data, missing, axis, source_key="source_timestamp_us")
            axes.scatter(x, [ceiling] * len(x), color="red", marker="x", s=75, label="Missing rendered frame ID")
        if reports:
            x = self._record_x(data, reports, axis, source_key="timestamp_us")
            axes.scatter(x, [ceiling * 0.92] * len(x), facecolors="none", edgecolors="red", s=75, label="Reported supersession")

    def _plot_jobs(self, data: TimelineData, axes: Any, axis: str) -> None:
        jobs = sorted(data.sam3d_jobs, key=lambda job: job.request_wall_us)
        tracks = [job.track_id for job in jobs]
        admissions = self._admission_sources(data)
        for index, job in enumerate(jobs):
            if axis == "wall":
                request = data.wall_seconds(job.request_wall_us)
                started = data.wall_seconds(job.reconstruction_start_wall_us)
                completed = data.wall_seconds(job.completion_wall_us)
            else:
                request = data.dataset_seconds_for_wall(job.request_wall_us)
                started = data.dataset_seconds_for_wall(job.reconstruction_start_wall_us)
                completed = data.dataset_seconds_for_wall(job.completion_wall_us)
            axes.barh(index, max(0, started - request), left=request, height=0.62,
                      color="gold", alpha=0.8, label="Queue" if index == 0 else None)
            color = "seagreen" if job.status == "ready" else "indianred"
            axes.barh(index, max(0, completed - started), left=started, height=0.62,
                      color=color, alpha=0.85, label=("Ready reconstruction" if job.status == "ready" else "Cancelled reconstruction") if not any(previous.status == job.status for previous in jobs[:index]) else None)
            axes.scatter([completed], [index], marker="D" if job.status == "ready" else "x", color=color, s=28)
            axes.text(completed, index, f" {job.track_id} {job.status}", va="center", fontsize=7)
            if axis == "dataset" and job.track_id in admissions:
                axes.scatter(
                    [admissions[job.track_id]["admission_dataset_seconds"]], [index],
                    marker="|", color="black", s=75,
                    label="Exact admission frame" if index == 0 else None,
                )
        for transition in self._asset_transitions(data):
            if transition["track_id"] not in tracks or transition["state"] not in {
                "proxy_first_displayed", "mesh_displayed"
            }:
                continue
            x = transition["wall_seconds"] if axis == "wall" else transition["dataset_seconds_inferred"]
            is_mesh = transition["state"] == "mesh_displayed"
            label = "First mesh display" if is_mesh else "First proxy display"
            axes.scatter(
                [x], [tracks.index(transition["track_id"])],
                marker="*" if is_mesh else ">", color="navy" if is_mesh else "gray",
                s=55 if is_mesh else 30,
                label=label if not any(c.get_label() == label for c in axes.collections) else None,
            )
        axes.set_yticks(range(len(tracks)), tracks, fontsize=7)

    def _record_x(
        self, data: TimelineData, rows: Iterable[dict[str, Any]], axis: str,
        *, source_key: str | None = None, frame_join: bool = False,
    ) -> list[float]:
        result = []
        by_frame = data.replay_by_frame
        for row in rows:
            if axis == "wall":
                result.append(data.wall_seconds(float(row["recorded_at_us"])))
            elif frame_join:
                result.append(float(by_frame[str(row["frame_id"])]["source_timestamp_us"]) / 1_000_000)
            else:
                result.append(float(row[source_key or "source_timestamp_us"]) / 1_000_000)
        return result

    def _asset_transitions(self, data: TimelineData) -> list[dict[str, Any]]:
        transitions = [
            {
                "track_id": str(row["track_id"]),
                "state": str(row["state"]),
                "wall_seconds": data.wall_seconds(float(row["recorded_at_us"])),
                "dataset_seconds_inferred": data.dataset_seconds_for_wall(float(row["recorded_at_us"])),
                "source": "compositor_asset",
            }
            for row in data.compositor_assets
        ]
        seen_proxy: set[str] = set()
        seen_mesh: set[str] = set()
        for row in data.compositor_frames:
            for track_id, state in row.get("track_states", {}).items():
                if state == "proxy" and track_id not in seen_proxy:
                    seen_proxy.add(track_id)
                    transitions.append({
                        "track_id": str(track_id),
                        "state": "proxy_first_displayed",
                        "frame_id": str(row["frame_id"]),
                        "wall_seconds": data.wall_seconds(float(row["recorded_at_us"])),
                        "dataset_seconds_inferred": float(row["timestamp_us"]) / 1_000_000,
                        "source": "compositor_frame.track_states",
                    })
                if state != "mesh" or track_id in seen_mesh:
                    continue
                seen_mesh.add(track_id)
                transitions.append({
                    "track_id": str(track_id),
                    "state": "mesh_displayed",
                    "frame_id": str(row["frame_id"]),
                    "wall_seconds": data.wall_seconds(float(row["recorded_at_us"])),
                    "dataset_seconds_inferred": float(row["timestamp_us"]) / 1_000_000,
                    "source": "compositor_frame.track_states",
                })
        return sorted(transitions, key=lambda row: row["wall_seconds"])

    @staticmethod
    def _admission_sources(data: TimelineData) -> dict[str, dict[str, Any]]:
        result: dict[str, dict[str, Any]] = {}
        replay = data.replay_by_frame
        for row in data.sam3_frames:
            submitted = int(row.get("reconstruction_requests_submitted", 0))
            tracks = [str(track) for track in row.get("stable_tracks_admitted", ())]
            frame_id = str(row.get("frame_id", ""))
            if submitted != 1 or len(tracks) != 1 or frame_id not in replay:
                continue
            result[tracks[0]] = {
                "admission_frame_id": frame_id,
                "admission_dataset_seconds": float(replay[frame_id]["source_timestamp_us"]) / 1_000_000,
            }
        return result

    @staticmethod
    def _parse_job(row: dict[str, Any]) -> Sam3DJob:
        completed = float(row["timestamp_us"])
        queue = float(row.get("queue_latency_ms", 0))
        reconstruction = float(row.get("reconstruction_latency_ms", 0))
        request_to_mesh = float(row.get("request_to_mesh_ms", queue + reconstruction))
        return Sam3DJob(
            request_id=str(row["request_id"]), track_id=str(row.get("track_id", "unknown")),
            status=str(row.get("state", "unknown")),
            request_wall_us=completed - request_to_mesh * 1_000,
            reconstruction_start_wall_us=completed - reconstruction * 1_000,
            completion_wall_us=completed, queue_latency_ms=queue,
            reconstruction_latency_ms=reconstruction, request_to_mesh_ms=request_to_mesh,
        )

    @staticmethod
    def _job_summary(data: TimelineData, job: Sam3DJob) -> dict[str, Any]:
        return {
            "request_id": job.request_id, "track_id": job.track_id, "status": job.status,
            "queue_latency_ms": job.queue_latency_ms,
            "reconstruction_latency_ms": job.reconstruction_latency_ms,
            "request_to_mesh_ms": job.request_to_mesh_ms,
            "request_wall_seconds": data.wall_seconds(job.request_wall_us),
            "reconstruction_start_wall_seconds": data.wall_seconds(job.reconstruction_start_wall_us),
            "completion_wall_seconds": data.wall_seconds(job.completion_wall_us),
            "request_dataset_seconds_inferred": data.dataset_seconds_for_wall(job.request_wall_us),
            "completion_dataset_seconds_inferred": data.dataset_seconds_for_wall(job.completion_wall_us),
        }

    def _read_jsonl(self, name: str) -> Iterable[dict[str, Any]]:
        path = self.metrics_dir / name
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except OSError as error:
            raise ValueError(f"unable to read {path}: {error}") from error
        for line_number, line in enumerate(lines, 1):
            try:
                value = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f"invalid JSON in {path}:{line_number}: {error}") from error
            if isinstance(value, dict):
                yield value


def _rolling_quantile(values: np.ndarray, window: int, quantile: float) -> np.ndarray:
    return np.asarray([
        np.quantile(values[max(0, index - window + 1):index + 1], quantile)
        for index in range(len(values))
    ])


def _reciprocal_ms(rows: Iterable[dict[str, Any]], key: str) -> np.ndarray:
    values = np.asarray([float(row[key]) for row in rows], dtype=np.float64)
    return np.divide(1_000.0, values, out=np.zeros_like(values), where=values > 0)


def _event_fps(rows: Iterable[dict[str, Any]]) -> np.ndarray:
    timestamps = np.asarray(
        [float(row["recorded_at_us"]) for row in rows], dtype=np.float64
    )
    if len(timestamps) < 2:
        return np.asarray([], dtype=np.float64)
    intervals_seconds = np.diff(timestamps) / 1_000_000.0
    return np.divide(
        1.0, intervals_seconds, out=np.zeros_like(intervals_seconds),
        where=intervals_seconds > 0,
    )


def _value_statistics(values: np.ndarray) -> dict[str, float | int | None]:
    finite = values[np.isfinite(values)]
    if not len(finite):
        return {
            "count": 0, "mean": None, "median": None,
            "p05": None, "p95": None, "min": None, "max": None,
        }
    return {
        "count": int(len(finite)),
        "mean": float(np.mean(finite)),
        "median": float(np.median(finite)),
        "p05": float(np.quantile(finite, 0.05)),
        "p95": float(np.quantile(finite, 0.95)),
        "min": float(np.min(finite)),
        "max": float(np.max(finite)),
    }


def _statistics(rows: Iterable[dict[str, Any]], key: str, *, positive: bool = False) -> dict[str, float | int | None]:
    values = [float(row[key]) for row in rows if row.get(key) is not None and (not positive or float(row[key]) > 0)]
    if not values:
        return {"count": 0, "mean": None, "median": None, "p95": None, "max": None}
    return {
        "count": len(values), "mean": float(np.mean(values)), "median": float(np.median(values)),
        "p95": float(np.quantile(values, 0.95)), "max": float(np.max(values)),
    }


def _counts(values: Iterable[str]) -> dict[str, int]:
    result: dict[str, int] = {}
    for value in values:
        result[value] = result.get(value, 0) + 1
    return result

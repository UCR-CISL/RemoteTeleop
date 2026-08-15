"""Summarize crash-tolerant realtime experiment artifacts."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from statistics import mean, median
from typing import Any, Iterable


@dataclass(frozen=True)
class RealtimeBenchmarkSummary:
    """One comparable record for a complete or failed runtime configuration."""

    variant: str
    status: str
    published_frames: int
    rendered_frames: int
    rendered_fraction: float
    observed_output_fps: float | None
    mean_render_fps: float | None
    median_render_ms: float | None
    p95_render_ms: float | None
    discarded_frames: int
    admitted_tracks: int
    masks_accepted: int
    mean_mask_inference_ms: float | None
    meshes_ready: int
    meshes_failed: int
    mean_reconstruction_ms: float | None
    peak_gpu_memory_mib: float | None
    failure: str | None

    def to_dict(self) -> dict[str, Any]:
        return dict(self.__dict__)


def summarize_experiment(output_root: str | Path, *, variant: str) -> RealtimeBenchmarkSummary:
    """Read one supervisor output directory without requiring a clean shutdown."""

    root = Path(output_root)
    admission = _read_json(root / "admission.json")
    replay = tuple(_read_jsonl(root / "metrics" / "replay.jsonl"))
    compositor = tuple(_read_jsonl(root / "metrics" / "compositor.jsonl"))
    masks = tuple(_read_jsonl(root / "metrics" / "sam3.jsonl"))
    meshes = tuple(_read_jsonl(root / "metrics" / "sam3d.jsonl"))

    published = [record for record in replay if record.get("event") == "frame_published"]
    rendered = [record for record in compositor if record.get("event") == "compositor_frame"]
    render_ms = [float(record["render_ms"]) for record in rendered if record.get("render_ms") is not None]
    admitted = {
        str(track)
        for record in masks
        if record.get("event") == "mask_batch"
        for track in record.get("stable_tracks_admitted", ())
    }
    masks_accepted = sum(
        int(record.get("accepted_masks", 0))
        for record in masks
        if record.get("event") == "mask_batch"
    )
    ready = sum(record.get("state") == "ready" for record in meshes)
    failed = sum(record.get("state") == "failed" for record in meshes)
    mask_ms = [
        float(record["inference_ms"])
        for record in masks
        if record.get("event") == "mask_batch"
        and record.get("inference_ms") is not None
        and float(record["inference_ms"]) > 0
    ]
    reconstruction_ms = [
        float(record["reconstruction_latency_ms"])
        for record in meshes
        if record.get("reconstruction_latency_ms") is not None
    ]
    peak_gpu = _peak_gpu_memory(root / "logs" / "gpu-monitor.log")
    status = str(admission.get("status", "missing"))
    failure = admission.get("error")
    return RealtimeBenchmarkSummary(
        variant=variant,
        status=status,
        published_frames=len(published),
        rendered_frames=len(rendered),
        rendered_fraction=len(rendered) / len(published) if published else 0.0,
        observed_output_fps=_event_rate(rendered),
        mean_render_fps=mean(1000.0 / value for value in render_ms if value > 0) if render_ms else None,
        median_render_ms=median(render_ms) if render_ms else None,
        p95_render_ms=_percentile(render_ms, 0.95),
        discarded_frames=sum(int(record.get("discarded_frames", 0)) for record in rendered),
        admitted_tracks=len(admitted),
        masks_accepted=masks_accepted,
        mean_mask_inference_ms=mean(mask_ms) if mask_ms else None,
        meshes_ready=ready,
        meshes_failed=failed,
        mean_reconstruction_ms=mean(reconstruction_ms) if reconstruction_ms else None,
        peak_gpu_memory_mib=peak_gpu,
        failure=None if failure is None else str(failure),
    )


def write_summary(output_root: str | Path, *, variant: str) -> Path:
    root = Path(output_root)
    path = root / "benchmark_summary.json"
    path.write_text(
        json.dumps(summarize_experiment(root, variant=variant).to_dict(), indent=2),
        encoding="utf-8",
    )
    return path


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _read_jsonl(path: Path) -> Iterable[dict[str, Any]]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return
    for line in lines:
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            yield value


def _peak_gpu_memory(path: Path) -> float | None:
    peak: float | None = None
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return None
    for line in lines:
        fields = [field.strip() for field in line.split(",")]
        if len(fields) < 3:
            continue
        try:
            used = float(fields[2])
        except ValueError:
            continue
        peak = used if peak is None else max(peak, used)
    return peak


def _percentile(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * quantile)))
    return ordered[index]


def _event_rate(records: list[dict[str, Any]]) -> float | None:
    if len(records) < 2:
        return None
    timestamps = [record.get("recorded_at_us") for record in records]
    if any(value is None for value in timestamps):
        return None
    elapsed = (float(timestamps[-1]) - float(timestamps[0])) / 1_000_000.0
    return (len(records) - 1) / elapsed if elapsed > 0 else None

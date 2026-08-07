#!/usr/bin/env python3
"""Summarize one real-time run using track-exit deadlines."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
from statistics import mean
from typing import Iterable


def _records(path: Path) -> list[dict[str, object]]:
    if not path.is_file():
        return []
    records = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as error:
            raise ValueError(f"invalid JSON at {path}:{line_number}: {error}") from error
        if isinstance(value, dict):
            records.append(value)
    return records


def _numbers(records: Iterable[dict[str, object]], key: str) -> list[float]:
    return [
        float(record[key])
        for record in records
        if isinstance(record.get(key), (int, float))
    ]


def _mean(records: Iterable[dict[str, object]], key: str) -> float | None:
    values = _numbers(records, key)
    return mean(values) if values else None


def _maximum(records: Iterable[dict[str, object]], key: str) -> float | None:
    values = _numbers(records, key)
    return max(values) if values else None


def _gpu_samples(path: Path) -> list[tuple[int, float, float, float]]:
    samples = []
    if not path.is_file():
        return samples
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = [field.strip() for field in line.split(",")]
        if len(fields) != 4:
            continue
        try:
            try:
                timestamp = datetime.strptime(fields[0], "%Y/%m/%d %H:%M:%S.%f")
            except ValueError:
                timestamp = datetime.strptime(fields[0], "%Y/%m/%d %H:%M:%S")
            timestamp_us = int(timestamp.timestamp() * 1_000_000)
            utilization, memory, power = (float(value) for value in fields[1:])
            samples.append((timestamp_us, utilization, memory, power))
        except ValueError:
            continue
    return samples


def _generation(
    requested: str | None,
    replay: list[dict[str, object]],
    sam3d: list[dict[str, object]],
) -> str:
    if requested:
        return requested
    completions = [
        str(record["scene_generation"])
        for record in replay
        if record.get("event") == "replay_complete" and record.get("scene_generation")
    ]
    if completions:
        return completions[-1]
    generations = [
        str(record["scene_generation"])
        for record in sam3d
        if record.get("scene_generation")
    ]
    if generations:
        return generations[-1]
    raise ValueError("could not infer scene generation from run metrics")


def summarize(output_root: Path, generation: str | None = None) -> dict[str, object]:
    metrics_root = output_root / "metrics"
    replay = _records(metrics_root / "replay.jsonl")
    sam3 = _records(metrics_root / "sam3.jsonl")
    sam3d = _records(metrics_root / "sam3d.jsonl")
    selected = _generation(generation, replay, sam3d)
    frames = [record for record in replay if record.get("event") == "frame_published"]
    generation_scoped_frames = any(record.get("scene_generation") for record in frames)
    if generation_scoped_frames:
        frames = [
            record for record in frames if record.get("scene_generation") == selected
        ]
    mask_batches = [
        record
        for record in sam3
        if record.get("event") == "mask_batch"
        and record.get("scene_generation") == selected
        and not record.get("end_of_scene", False)
    ]
    terminal_records = [
        record for record in sam3d if record.get("scene_generation") == selected
    ]
    terminals_by_track: dict[str, dict[str, object]] = {}
    for record in terminal_records:
        track_id = record.get("track_id")
        if isinstance(track_id, str):
            terminals_by_track[track_id] = record
    terminals = list(terminals_by_track.values())
    states = Counter(str(record.get("state")) for record in terminals)
    ready = [record for record in terminals if record.get("state") == "ready"]
    raw_deadline_hits = [
        record for record in ready if record.get("deadline_hit") is True
    ]
    cancelled = [record for record in terminals if record.get("state") == "cancelled"]

    track_timestamps: dict[str, list[int]] = {}
    for record in frames:
        timestamp = record.get("source_timestamp_us")
        if not isinstance(timestamp, int):
            continue
        for track_id in record.get("track_ids", []):
            if isinstance(track_id, str):
                track_timestamps.setdefault(track_id, []).append(timestamp)
    detected_tracks = set(track_timestamps)
    if not detected_tracks:
        detected_tracks = set(terminals_by_track)
    ordered_frames = sorted(
        (
            int(record["source_timestamp_us"]),
            {
                track_id
                for track_id in record.get("track_ids", [])
                if isinstance(track_id, str)
            },
        )
        for record in frames
        if isinstance(record.get("source_timestamp_us"), int)
    )
    track_exit_timestamps: dict[str, int] = {}
    for track_id in detected_tracks:
        visible_indices = [
            index
            for index, (_timestamp, active) in enumerate(ordered_frames)
            if track_id in active
        ]
        if not visible_indices:
            continue
        first_index = visible_indices[0]
        exit_timestamp = ordered_frames[-1][0]
        for timestamp, active in ordered_frames[first_index + 1 :]:
            if track_id not in active:
                exit_timestamp = timestamp
                break
        track_exit_timestamps[track_id] = exit_timestamp

    remaining_lifetimes = []
    residual_limits = (1.0, 0.5, 0.5)

    def quality_accepted(record: dict[str, object]) -> bool:
        values = record.get("dimension_residual_m")
        return (
            isinstance(values, list)
            and len(values) == 3
            and all(
                abs(float(value)) <= limit
                for value, limit in zip(values, residual_limits, strict=True)
            )
        )

    quality_ready = [record for record in ready if quality_accepted(record)]
    deadline_hits = [record for record in raw_deadline_hits if quality_accepted(record)]
    track_first_wall_us: dict[str, int] = {}
    track_exit_wall_us: dict[str, int] = {}
    if generation_scoped_frames:
        ordered_wall_frames = sorted(
            (
                int(record["recorded_at_us"]),
                {
                    track_id
                    for track_id in record.get("track_ids", [])
                    if isinstance(track_id, str)
                },
            )
            for record in frames
            if isinstance(record.get("recorded_at_us"), int)
        )
        scene_end_wall_us = max(
            (
                int(record["recorded_at_us"])
                for record in replay
                if record.get("event") == "scene_end_published"
                and record.get("scene_generation") == selected
                and isinstance(record.get("recorded_at_us"), int)
            ),
            default=ordered_wall_frames[-1][0] if ordered_wall_frames else 0,
        )
        for track_id in detected_tracks:
            visible_indices = [
                index
                for index, (_timestamp, active) in enumerate(ordered_wall_frames)
                if track_id in active
            ]
            if not visible_indices:
                continue
            first_index = visible_indices[0]
            track_first_wall_us[track_id] = ordered_wall_frames[first_index][0]
            exit_wall_us = scene_end_wall_us
            for timestamp, active in ordered_wall_frames[first_index + 1 :]:
                if track_id not in active:
                    exit_wall_us = timestamp
                    break
            track_exit_wall_us[track_id] = exit_wall_us

    detection_to_mesh = []
    for record in deadline_hits:
        track_id = record.get("track_id")
        ready_wall_us = record.get("timestamp_us")
        first_wall_us = track_first_wall_us.get(str(track_id))
        if isinstance(ready_wall_us, int) and first_wall_us is not None:
            ready_ms = (ready_wall_us - first_wall_us) / 1_000.0
        else:
            ready_ms = record.get("first_detection_to_mesh_ms")
        if isinstance(ready_ms, (int, float)):
            detection_to_mesh.append(float(ready_ms))
        exit_wall_us = track_exit_wall_us.get(str(track_id))
        if isinstance(ready_wall_us, int) and exit_wall_us is not None:
            remaining_lifetimes.append((exit_wall_us - ready_wall_us) / 1_000.0)
            continue
        timestamps = track_timestamps.get(str(track_id), [])
        if timestamps and isinstance(ready_ms, (int, float)):
            visible_ms = (
                track_exit_timestamps.get(str(track_id), max(timestamps))
                - min(timestamps)
            ) / 1_000.0
            remaining_lifetimes.append(visible_ms - float(ready_ms))

    residuals = [
        record["dimension_residual_m"]
        for record in ready
        if isinstance(record.get("dimension_residual_m"), list)
        and len(record["dimension_residual_m"]) == 3
    ]
    mean_residual = (
        [mean(abs(float(values[index])) for values in residuals) for index in range(3)]
        if residuals
        else None
    )
    inference_batches = [
        record for record in mask_batches if not record.get("inference_skipped", False)
    ]
    skipped_batches = [
        record for record in mask_batches if record.get("inference_skipped", False)
    ]
    prompts = sum(int(record.get("prompts", 0)) for record in inference_batches)
    inference_seconds = sum(_numbers(inference_batches, "inference_ms")) / 1_000.0
    admission_path = output_root / "admission.json"
    admission = (
        json.loads(admission_path.read_text(encoding="utf-8"))
        if admission_path.is_file()
        else {}
    )
    gpu_samples = _gpu_samples(output_root / "logs" / "gpu-monitor.log")
    replay_start_us = min(
        (
            int(record["recorded_at_us"])
            for record in frames
            if isinstance(record.get("recorded_at_us"), int)
        ),
        default=None,
    )
    execution_end_us = max(
        [
            int(record["recorded_at_us"])
            for record in replay
            if record.get("event") == "replay_complete"
            and isinstance(record.get("recorded_at_us"), int)
        ]
        + [
            int(record["timestamp_us"])
            for record in terminal_records
            if isinstance(record.get("timestamp_us"), int)
        ],
        default=None,
    )
    execution_gpu_samples = [
        sample
        for sample in gpu_samples
        if (replay_start_us is None or sample[0] >= replay_start_us)
        and (execution_end_us is None or sample[0] <= execution_end_us)
    ]
    if not execution_gpu_samples:
        execution_gpu_samples = gpu_samples
    executed = [
        record
        for record in terminals
        if isinstance(record.get("reconstruction_latency_ms"), (int, float))
    ]
    backend_metrics = [
        record["sam3d_backend"]
        for record in executed
        if isinstance(record.get("sam3d_backend"), dict)
    ]
    cache_hit_metrics = [
        record
        for record in backend_metrics
        if isinstance(record.get("pointmap_cache_hit"), bool)
    ]
    return {
        "output_root": str(output_root),
        "scene_generation": selected,
        "policy": {
            "sam3_prompt_mode": admission.get("sam3_prompt_mode"),
            "sam3_mask_policy": admission.get("sam3_mask_policy"),
            "sam3d_priority_policy": admission.get("sam3d_priority_policy"),
            "sam3d_coalesce_ms": admission.get("sam3d_coalesce_ms"),
            "sam3d_stage1_inference_steps": admission.get(
                "sam3d_stage1_inference_steps"
            ),
            "sam3d_stage2_inference_steps": admission.get(
                "sam3d_stage2_inference_steps"
            ),
            "sam3d_pointmap_cache_size": admission.get(
                "sam3d_pointmap_cache_size"
            ),
            "precision": admission.get("precision"),
        },
        "replay_frames": len(frames),
        "unique_tracks_detected": len(detected_tracks),
        "unique_tracks_requested": len(terminals),
        "request_coverage_rate": (
            len(terminals_by_track) / len(detected_tracks) if detected_tracks else 0.0
        ),
        "detected_tracks_without_request": len(detected_tracks - set(terminals_by_track)),
        "terminal_states": dict(states),
        "raw_mesh_ready_before_track_exit": len(raw_deadline_hits),
        "raw_deadline_hit_rate": (
            len(raw_deadline_hits) / len(detected_tracks) if detected_tracks else 0.0
        ),
        "mesh_ready_before_track_exit": len(deadline_hits),
        "deadline_hit_rate": (
            len(deadline_hits) / len(detected_tracks) if detected_tracks else 0.0
        ),
        "stale_jobs_cancelled": len(cancelled),
        "results_discarded_after_inference": sum(
            bool(record.get("result_discarded_after_inference")) for record in cancelled
        ),
        "first_detection_to_mesh_ms_mean": (
            mean(detection_to_mesh) if detection_to_mesh else None
        ),
        "remaining_visible_lifetime_ms_mean": (
            mean(remaining_lifetimes) if remaining_lifetimes else None
        ),
        "reconstruction_jobs_executed": len(executed),
        "queue_latency_ms_mean": _mean(terminals, "queue_latency_ms"),
        "reconstruction_latency_ms_mean": _mean(
            executed, "reconstruction_latency_ms"
        ),
        "sam3": {
            "input_frames": len(mask_batches),
            "inference_frames": len(inference_batches),
            "skipped_frames": len(skipped_batches),
            "prompts": prompts,
            "inference_ms_mean": _mean(inference_batches, "inference_ms"),
            "prompt_decode_ms_mean": _mean(inference_batches, "prompt_decode_ms"),
            "observed_prompt_throughput_per_second": (
                prompts / inference_seconds if inference_seconds > 0 else 0.0
            ),
            "inference_frame_throughput_fps": (
                len(inference_batches) / inference_seconds
                if inference_seconds > 0
                else 0.0
            ),
            "gpu_peak_allocated_mib": _maximum(
                inference_batches, "gpu_peak_allocated_mib"
            ),
            "gpu_peak_reserved_mib": _maximum(
                inference_batches, "gpu_peak_reserved_mib"
            ),
        },
        "sam3d_gpu_peak_allocated_mib": _maximum(
            terminal_records, "cuda_peak_allocated_mib"
        ),
        "sam3d_gpu_peak_reserved_mib": _maximum(
            terminal_records, "cuda_peak_reserved_mib"
        ),
        "sam3d_backend_inference_seconds_mean": _mean(
            backend_metrics, "inference_seconds"
        ),
        "sam3d_depth_model_call_seconds_mean": _mean(
            backend_metrics, "depth_model_call_seconds"
        ),
        "sam3d_pointmap_cache_hit_count": sum(
            bool(record["pointmap_cache_hit"]) for record in cache_hit_metrics
        ),
        "sam3d_pointmap_cache_observed_calls": len(cache_hit_metrics),
        "gpu_device_samples": len(execution_gpu_samples),
        "gpu_utilization_percent_mean": (
            mean(sample[1] for sample in execution_gpu_samples)
            if execution_gpu_samples
            else None
        ),
        "gpu_utilization_percent_max": (
            max(sample[1] for sample in execution_gpu_samples)
            if execution_gpu_samples
            else None
        ),
        "gpu_memory_used_mib_max": (
            max(sample[2] for sample in execution_gpu_samples)
            if execution_gpu_samples
            else None
        ),
        "gpu_power_watts_mean": (
            mean(sample[3] for sample in execution_gpu_samples)
            if execution_gpu_samples
            else None
        ),
        "gpu_full_process_memory_used_mib_max": (
            max(sample[2] for sample in gpu_samples) if gpu_samples else None
        ),
        "mean_absolute_dimension_residual_lwh_m": mean_residual,
        "mesh_quality_acceptance_rate": (
            len(quality_ready) / len(ready) if ready else 0.0
        ),
        "mesh_quality_residual_limits_lwh_m": list(residual_limits),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_root", type=Path)
    parser.add_argument("--scene-generation")
    parser.add_argument("--json-output", type=Path)
    args = parser.parse_args()
    summary = summarize(args.output_root, args.scene_generation)
    encoded = json.dumps(summary, indent=2, allow_nan=False)
    print(encoded)
    if args.json_output is not None:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(encoded + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()

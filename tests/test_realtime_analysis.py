import json

from scripts.analyze_realtime_run import summarize


def _write_jsonl(path, records):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(json.dumps(record) + "\n" for record in records),
        encoding="utf-8",
    )


def test_summary_filters_generation_and_scores_track_exit_deadline(tmp_path):
    root = tmp_path / "run"
    _write_jsonl(
        root / "metrics" / "replay.jsonl",
        [
            {
                "event": "frame_published",
                "scene_generation": "old",
                "source_timestamp_us": 0,
                "track_ids": ["ignored"],
            },
            {
                "event": "frame_published",
                "scene_generation": "current",
                "recorded_at_us": 1_000_000,
                "source_timestamp_us": 100_000,
                "track_ids": ["hit", "miss", "bad-mesh"],
            },
            {
                "event": "frame_published",
                "scene_generation": "current",
                "recorded_at_us": 2_000_000,
                "source_timestamp_us": 1_100_000,
                "track_ids": ["hit"],
            },
            {
                "event": "scene_end_published",
                "scene_generation": "current",
                "recorded_at_us": 2_100_000,
            },
            {
                "event": "replay_complete",
                "scene_generation": "current",
                "recorded_at_us": 2_100_000,
            },
        ],
    )
    _write_jsonl(
        root / "metrics" / "sam3.jsonl",
        [
            {
                "event": "mask_batch",
                "scene_generation": "current",
                "prompts": 2,
                "inference_ms": 200.0,
                "prompt_decode_ms": 80.0,
                "inference_skipped": False,
            },
            {
                "event": "mask_batch",
                "scene_generation": "current",
                "prompts": 0,
                "inference_ms": 0.0,
                "inference_skipped": True,
            },
        ],
    )
    _write_jsonl(
        root / "metrics" / "sam3d.jsonl",
        [
            {
                "scene_generation": "current",
                "track_id": "hit",
                "state": "ready",
                "timestamp_us": 1_600_000,
                "deadline_hit": True,
                "first_detection_to_mesh_ms": 600.0,
                "dimension_residual_m": [0.2, -0.1, 0.1],
            },
            {
                "scene_generation": "current",
                "track_id": "miss",
                "state": "cancelled",
                "deadline_hit": False,
                "stale_job_cancelled": True,
            },
            {
                "scene_generation": "current",
                "track_id": "bad-mesh",
                "state": "ready",
                "timestamp_us": 1_200_000,
                "deadline_hit": True,
                "first_detection_to_mesh_ms": 200.0,
                "dimension_residual_m": [2.0, 0.1, 0.1],
            },
        ],
    )
    gpu_log = root / "logs" / "gpu-monitor.log"
    gpu_log.parent.mkdir(parents=True, exist_ok=True)
    gpu_log.write_text(
        "2026/07/31 00:00:00.000, 40, 12000, 200.0\n"
        "2026/07/31 00:00:00.250, 80, 18000, 300.0\n",
        encoding="utf-8",
    )

    result = summarize(root)

    assert result["scene_generation"] == "current"
    assert result["replay_frames"] == 2
    assert result["unique_tracks_detected"] == 3
    assert result["request_coverage_rate"] == 1.0
    assert result["raw_mesh_ready_before_track_exit"] == 2
    assert result["raw_deadline_hit_rate"] == 2 / 3
    assert result["deadline_hit_rate"] == 1 / 3
    assert result["mesh_ready_before_track_exit"] == 1
    assert result["stale_jobs_cancelled"] == 1
    assert result["first_detection_to_mesh_ms_mean"] == 600.0
    assert result["remaining_visible_lifetime_ms_mean"] == 500.0
    assert result["sam3"]["inference_frames"] == 1
    assert result["sam3"]["skipped_frames"] == 1
    assert result["sam3"]["observed_prompt_throughput_per_second"] == 10.0
    assert result["mesh_quality_acceptance_rate"] == 0.5
    assert result["gpu_device_samples"] == 2
    assert result["gpu_utilization_percent_mean"] == 60.0
    assert result["gpu_memory_used_mib_max"] == 18_000.0
    assert result["gpu_power_watts_mean"] == 250.0

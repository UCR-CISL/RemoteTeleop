import json

from src.realtime.benchmark_summary import summarize_experiment


def _jsonl(path, records):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(record) + "\n" for record in records))


def test_summary_survives_failed_partial_experiment(tmp_path):
    (tmp_path / "admission.json").write_text(
        json.dumps({"status": "failed", "error": "oom"})
    )
    _jsonl(tmp_path / "metrics/replay.jsonl", [
        {"event": "frame_published"}, {"event": "frame_published"}
    ])
    _jsonl(tmp_path / "metrics/compositor.jsonl", [
        {"event": "compositor_frame", "render_ms": 50.0,
         "discarded_frames": 1, "recorded_at_us": 1000000}
    ])
    _jsonl(tmp_path / "metrics/sam3.jsonl", [{
        "event": "mask_batch", "stable_tracks_admitted": ["a"], "accepted_masks": 1
    }])
    _jsonl(tmp_path / "metrics/sam3d.jsonl", [
        {"state": "ready"}, {"state": "failed"}
    ])
    log = tmp_path / "logs/gpu-monitor.log"
    log.parent.mkdir(parents=True)
    log.write_text("date, 10, 1234, 50\ndate, 10, 2345, 50\n")

    result = summarize_experiment(tmp_path, variant="partial")

    assert result.status == "failed"
    assert result.published_frames == 2
    assert result.rendered_frames == 1
    assert result.rendered_fraction == 0.5
    assert result.mean_render_fps == 20.0
    assert result.observed_output_fps is None
    assert result.discarded_frames == 1
    assert result.admitted_tracks == 1
    assert result.meshes_ready == result.meshes_failed == 1
    assert result.peak_gpu_memory_mib == 2345.0
    assert result.failure == "oom"

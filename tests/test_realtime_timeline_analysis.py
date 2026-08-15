import json

import numpy as np
import pytest

from src.realtime.timeline_analysis import RealtimeTimelineAnalyzer, _rolling_quantile


def _jsonl(path, records):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(record) + "\n" for record in records))


def _metrics(tmp_path):
    metrics = tmp_path / "metrics"
    _jsonl(metrics / "replay.jsonl", [
        {"event": "frame_published", "frame_id": "f10", "source_timestamp_us": 0,
         "recorded_at_us": 10_000_000, "build_latency_ms": 10, "publish_interval_ms": 20},
        {"event": "frame_published", "frame_id": "f20", "source_timestamp_us": 100_000,
         "recorded_at_us": 20_000_000, "build_latency_ms": 20, "publish_interval_ms": 30},
        {"event": "frame_published", "frame_id": "f30", "source_timestamp_us": 200_000,
         "recorded_at_us": 30_000_000, "build_latency_ms": 30, "publish_interval_ms": 40},
    ])
    _jsonl(metrics / "compositor.jsonl", [
        {"event": "compositor_frame", "frame_id": "f10", "timestamp_us": 0,
         "recorded_at_us": 10_100_000, "discarded_frames": 0, "render_ms": 5,
         "track_states": {"t1": "proxy"}},
        {"event": "compositor_frame", "frame_id": "f30", "timestamp_us": 200_000,
         "recorded_at_us": 30_100_000, "discarded_frames": 1, "render_ms": 7,
         "track_states": {"t1": "mesh"}},
        {"event": "compositor_asset", "recorded_at_us": 13_000_000,
         "track_id": "t1", "state": "queued"},
        {"event": "compositor_asset", "recorded_at_us": 15_000_000,
         "track_id": "t1", "state": "running"},
        {"event": "compositor_asset", "recorded_at_us": 20_000_000,
         "track_id": "t1", "state": "ready"},
    ])
    _jsonl(metrics / "sam3.jsonl", [
        {"event": "mask_batch", "frame_id": "f10", "recorded_at_us": 10_010_000,
         "end_of_scene": False, "discarded_input_frames": 0, "image_encode_ms": 0,
         "prompt_decode_ms": 0, "inference_ms": 0, "process_end_to_end_ms": 1},
        {"event": "mask_batch", "frame_id": "f20", "recorded_at_us": 20_010_000,
         "end_of_scene": False, "discarded_input_frames": 1, "image_encode_ms": 2,
         "prompt_decode_ms": 3, "inference_ms": 7, "process_end_to_end_ms": 9},
        {"event": "mask_batch", "frame_id": "f30:scene-end", "recorded_at_us": 31_000_000,
         "end_of_scene": True, "image_encode_ms": 0, "prompt_decode_ms": 0,
         "inference_ms": 0, "process_end_to_end_ms": 0},
    ])
    _jsonl(metrics / "sam3d.jsonl", [
        {"timestamp_us": 20_000_000, "request_id": "request:t1", "track_id": "t1",
         "state": "ready", "queue_latency_ms": 2_000,
         "reconstruction_latency_ms": 5_000, "request_to_mesh_ms": 7_000},
    ])
    return metrics


def test_summary_joins_frames_and_reconstructs_async_intervals(tmp_path):
    analyzer = RealtimeTimelineAnalyzer(_metrics(tmp_path))
    data = analyzer.load()
    summary = analyzer.summarize(data)

    assert [row["frame_id"] for row in data.replay] == ["f10", "f20", "f30"]
    assert len(data.sam3_frames) == 2
    assert summary["dataset"]["missing_render_frame_ids"] == ["f20"]
    assert summary["dataset"]["compositor_reported_discarded_frames"] == 1
    assert summary["dataset"]["sam3_reported_discarded_input_frames"] == 1
    job = summary["sam3d"]["jobs"][0]
    assert job["request_wall_seconds"] == 3
    assert job["reconstruction_start_wall_seconds"] == 5
    assert job["completion_wall_seconds"] == 10
    assert job["request_dataset_seconds_inferred"] == pytest.approx(0.03)
    assert job["completion_dataset_seconds_inferred"] == pytest.approx(0.1)


def test_summary_uses_active_sam3_timings_and_recovers_mesh_display(tmp_path):
    analyzer = RealtimeTimelineAnalyzer(_metrics(tmp_path))
    summary = analyzer.summarize(analyzer.load())

    assert summary["latency_ms"]["sam3_image_encode"]["count"] == 1
    assert summary["latency_ms"]["sam3_image_encode"]["median"] == 2
    assert summary["latency_ms"]["sam3_process_end_to_end_all_frames"]["count"] == 2
    assert summary["latency_ms"]["sam3_process_end_to_end_active"]["count"] == 1
    mesh_display = [
        row for row in summary["proxy_to_mesh"] if row["state"] == "mesh_displayed"
    ]
    assert mesh_display == [{
        "track_id": "t1", "state": "mesh_displayed", "frame_id": "f30",
        "wall_seconds": 20.1, "dataset_seconds_inferred": 0.2,
        "source": "compositor_frame.track_states",
    }]
    proxy_display = [
        row for row in summary["proxy_to_mesh"]
        if row["state"] == "proxy_first_displayed"
    ]
    assert proxy_display[0]["frame_id"] == "f10"


def test_rolling_quantile_is_trailing():
    values = np.asarray([1.0, 2.0, 100.0, 4.0])
    assert np.allclose(_rolling_quantile(values, 3, 0.5), [1, 1.5, 2, 4])
    assert np.allclose(
        _rolling_quantile(values, 3, 0.95),
        [1, 1.95, 90.2, 90.4],
    )


def test_write_creates_both_plots_and_machine_summary(tmp_path):
    output = tmp_path / "new-artifacts"
    paths = RealtimeTimelineAnalyzer(_metrics(tmp_path)).write(output)

    assert set(paths) == {"dataset_plot", "wall_plot", "fps_plot", "summary"}
    assert all(path.is_file() and path.stat().st_size > 0 for path in paths.values())
    summary = json.loads(paths["summary"].read_text())
    assert summary["schema_version"] == 1
    assert summary["sam3d"]["status_counts"] == {"ready": 1}
    assert summary["fps"]["nominal_dataset_rate"] == 10
    assert summary["fps"]["replay_publication"]["median"] == pytest.approx(1000 / 30)
    assert summary["fps"]["compositor_output"]["count"] == 1

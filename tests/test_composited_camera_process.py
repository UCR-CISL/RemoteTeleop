import cv2
import numpy as np

from src.realtime.composited_camera_process import (
    CompositedFrameWriter,
    FrameCursor,
    OrderedPoseDetectionJoin,
    OrderedPoseMetrics,
)
from src.realtime.protocol import EgoPoseSample, FrameEnd, FrameObjectDetections, FrameSnapshot


def _pose(sequence, timestamp_us, generation="scene-1"):
    return EgoPoseSample(
        sequence=sequence,
        timestamp_us=timestamp_us,
        scene_generation=generation,
        frame_id=f"frame-{sequence}",
        world_T_ego=tuple(np.eye(4).reshape(-1)),
    )


def _detections(sequence, timestamp_us, generation="scene-1", frame_id=None):
    return FrameObjectDetections(
        sequence=sequence,
        timestamp_us=timestamp_us,
        scene_generation=generation,
        frame_id=frame_id or f"frame-{sequence}",
        boxes=(),
    )


def test_composited_frame_writer_saves_numbered_png_and_video(tmp_path):
    writer = CompositedFrameWriter(tmp_path, 10.0)
    path = writer.write("frame-1", np.full((12, 20, 3), 0.5, dtype=np.float32))
    writer.close()

    assert path.name == "000000_frame-1.png"
    assert cv2.imread(str(path)).shape == (12, 20, 3)
    assert (tmp_path / "composited.mp4").stat().st_size > 0


def test_composited_frame_writer_rejects_resolution_changes(tmp_path):
    writer = CompositedFrameWriter(tmp_path, 10.0)
    try:
        writer.write("frame-1", np.zeros((12, 20, 3), dtype=np.float32))
        with np.testing.assert_raises_regex(ValueError, "frame size changed"):
            writer.write("frame-2", np.zeros((10, 20, 3), dtype=np.float32))
    finally:
        writer.close()


def test_composited_frame_writer_recovers_contiguous_frames_and_rebuilds_mp4(tmp_path):
    first = CompositedFrameWriter(tmp_path, 10.0)
    first.write("frame-0", np.zeros((12, 20, 3), dtype=np.float32))
    # Simulate a crash after the durable PNG/cursor boundary, before MP4 close.
    second = CompositedFrameWriter(tmp_path, 10.0)
    assert second.count == 1
    second.write("frame-1", np.ones((12, 20, 3), dtype=np.float32))
    second.close()

    capture = cv2.VideoCapture(str(tmp_path / "composited.mp4"))
    assert int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) == 2
    capture.release()
    assert [path.name for path in sorted((tmp_path / "frames").glob("*.png"))] == [
        "000000_frame-0.png", "000001_frame-1.png"
    ]


def test_frame_writer_reconciles_png_written_before_cursor_commit(tmp_path):
    writer = CompositedFrameWriter(tmp_path, 10.0)
    writer.write("frame-0", np.zeros((12, 20, 3), dtype=np.float32))
    cursor = FrameCursor(tmp_path / "cursor.json", "scene-1")

    restarted = CompositedFrameWriter(tmp_path, 10.0)
    restarted.reconcile(cursor.next_sequence)
    path = restarted.write("frame-0", np.ones((12, 20, 3), dtype=np.float32))

    assert path.name == "000000_frame-0.png"
    assert len(list((tmp_path / "frames").glob("*.png"))) == 1


def test_ordered_pose_metrics_accounts_for_every_sample_without_drops():
    metrics = OrderedPoseMetrics()
    results = [metrics.observe(_pose(index, index * 100_000)) for index in range(10)]

    assert [result["sequence_gap"] for result in results] == [0] * 10
    assert [result["timestamp_delta_us"] for result in results] == [0] + [100_000] * 9
    assert [result["timestamp_gap_us"] for result in results] == [0] * 10


def test_ordered_pose_metrics_reports_gaps_and_rejects_bad_order_or_generation():
    metrics = OrderedPoseMetrics()
    first = metrics.observe(_pose(3, 300_000))
    second = metrics.observe(_pose(5, 500_000))
    assert first["sequence_gap"] == 3
    assert first["timestamp_gap_us"] == 300_000
    assert second["sequence_gap"] == 1
    assert second["timestamp_gap_us"] == 100_000
    with np.testing.assert_raises_regex(ValueError, "out-of-order"):
        metrics.observe(_pose(4, 600_000))
    with np.testing.assert_raises_regex(ValueError, "mismatched scene generation"):
        metrics.observe(_pose(6, 600_000, generation="scene-2"))


def test_ordered_join_requires_detection_immediately_before_its_matching_pose():
    join = OrderedPoseDetectionJoin()
    for sequence in range(10):
        join.observe_detection(_detections(sequence, sequence * 100_000))
        assert join.take_for_pose(_pose(sequence, sequence * 100_000)).sequence == sequence


def test_ordered_join_rejects_missing_repeated_or_mismatched_detections():
    join = OrderedPoseDetectionJoin()
    with np.testing.assert_raises_regex(ValueError, "preceded"):
        join.take_for_pose(_pose(0, 0))
    join.observe_detection(_detections(0, 0))
    with np.testing.assert_raises_regex(ValueError, "before their matching"):
        join.observe_detection(_detections(1, 100_000))
    with np.testing.assert_raises_regex(ValueError, "sequence mismatch"):
        join.take_for_pose(_pose(1, 0))
    join.observe_detection(_detections(2, 200_000, frame_id="wrong-frame"))
    with np.testing.assert_raises_regex(ValueError, "frame_id mismatch"):
        join.take_for_pose(_pose(2, 200_000, generation="scene-1"))


def test_frame_cursor_persists_exactly_once_sequence_across_reconnect(tmp_path):
    cursor_path = tmp_path / "cursor.json"
    first = FrameCursor(cursor_path, "scene-1")
    snapshot = FrameSnapshot(0, 0, "scene-1", "frame-0", tuple(np.eye(4).reshape(-1)))
    first.commit(snapshot)

    reconnected = FrameCursor(cursor_path, "scene-1")

    assert reconnected.next_sequence == 1
    assert reconnected.is_committed(snapshot)
    with np.testing.assert_raises_regex(ValueError, "does not follow cursor"):
        reconnected.commit(snapshot)


def test_frame_cursor_requires_every_frame_in_a_multiframe_pull_sequence(tmp_path):
    cursor = FrameCursor(tmp_path / "cursor.json", "scene-1")
    snapshots = [
        FrameSnapshot(index, index * 100_000, "scene-1", f"frame-{index}", tuple(np.eye(4).reshape(-1)))
        for index in range(4)
    ]
    for snapshot in snapshots:
        cursor.commit(snapshot)

    assert cursor.next_sequence == 4
    restarted = FrameCursor(tmp_path / "cursor.json", "scene-1")
    assert restarted.next_sequence == 4
    with np.testing.assert_raises_regex(ValueError, "does not follow cursor"):
        restarted.commit(FrameSnapshot(5, 500_000, "scene-1", "frame-5", tuple(np.eye(4).reshape(-1))))


def test_frame_cursor_enforces_snapshot_cadence_and_reports_zero_gaps(tmp_path):
    cursor = FrameCursor(tmp_path / "cursor.json", "scene-1")
    first = FrameSnapshot(0, 700_000, "scene-1", "frame-0", tuple(np.eye(4).reshape(-1)))
    assert cursor.validate(first) == {
        "sequence_gap": 0, "timestamp_gap_us": 0, "timestamp_delta_us": 0,
    }
    cursor.commit(first)
    second = FrameSnapshot(1, 800_000, "scene-1", "frame-1", tuple(np.eye(4).reshape(-1)))
    assert cursor.validate(second) == {
        "sequence_gap": 0, "timestamp_gap_us": 0, "timestamp_delta_us": 100_000,
    }
    with np.testing.assert_raises_regex(ValueError, "cadence"):
        cursor.validate(FrameSnapshot(1, 800_001, "scene-1", "bad", tuple(np.eye(4).reshape(-1))))


def test_frame_cursor_detects_changed_duplicate_snapshot_and_persists_completion(tmp_path):
    cursor = FrameCursor(tmp_path / "cursor.json", "scene-1")
    snapshot = FrameSnapshot(0, 0, "scene-1", "frame-0", tuple(np.eye(4).reshape(-1)))
    cursor.commit(snapshot)
    changed_pose = np.eye(4)
    changed_pose[0, 3] = 1.0
    changed = FrameSnapshot(0, 0, "scene-1", "frame-0", tuple(changed_pose.reshape(-1)))
    assert not cursor.is_committed(changed)

    cursor.complete(FrameEnd("scene-1", 0, 0))

    assert (tmp_path / "frame-complete.json").is_file()

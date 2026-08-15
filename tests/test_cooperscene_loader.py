import numpy as np
import pytest

import src.localization.cooperscene as cooperscene

from src.localization.cooperscene import (
    CooperSceneSequenceDataset,
    pose_to_matrix,
    read_ascii_xyzi_pcd,
)


def _write_frame(root, frame_id, pose, rows):
    root.mkdir(parents=True, exist_ok=True)
    (root / f"{frame_id}.yaml").write_text(f"lidar_pose: {pose}\n", encoding="utf-8")
    body = "\n".join(" ".join(str(value) for value in row) for row in rows)
    (root / f"{frame_id}.pcd").write_text(
        "# .PCD v0.7\nVERSION 0.7\nFIELDS intensity z x y\n"
        "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {len(rows)}\nHEIGHT 1\nPOINTS {len(rows)}\nDATA ascii\n{body}\n",
        encoding="ascii",
    )


def test_ascii_xyzi_reader_orders_named_fields_and_is_immutable(tmp_path):
    path = tmp_path / "points.pcd"
    _write_frame(tmp_path, "points", "[0, 0, 0, 0, 0, 0]", [[7, 3, 1, 2], [8, 6, 4, 5]])

    points = read_ascii_xyzi_pcd(path)

    np.testing.assert_array_equal(points, [[1, 2, 3, 7], [4, 5, 6, 8]])
    assert points.dtype == np.float32
    assert not points.flags.writeable
    with pytest.raises(ValueError):
        points[0, 0] = 0


def test_pose_to_matrix_matches_cooperscene_carla_euler_semantics():
    transform = pose_to_matrix([1, 2, 3, 90, 0, 90])

    np.testing.assert_allclose(
        transform,
        [[0, 1, 0, 1], [0, 0, 1, 2], [1, 0, 0, 3], [0, 0, 0, 1]],
        atol=1e-12,
    )
    assert not transform.flags.writeable


def test_sequence_reads_yaml_pose_orders_frames_and_uses_map_transform_name(tmp_path):
    sequence_root = tmp_path / "train" / "take-a" / "2"
    _write_frame(sequence_root, "10", "[10, 0, 0, 0, 90, 0]", [[9, 3, 1, 2]])
    _write_frame(sequence_root, "2", "[2, 0, 0, 0, 0, 0]", [[6, 3, 1, 2]])
    dataset = CooperSceneSequenceDataset(tmp_path)

    sequence = dataset.sequence("train", "take-a", 2)
    frames = list(sequence)

    assert (sequence.split, sequence.scenario, sequence.agent) == ("train", "take-a", "2")
    assert [frame.frame_id for frame in frames] == ["2", "10"]
    assert [frame.timestamp for frame in frames] == [2, 10]
    np.testing.assert_allclose(frames[0].map_T_lidar[:3, 3], [2, 0, 0])
    np.testing.assert_allclose(frames[1].map_T_lidar[:3, :3], [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-12)
    assert frames[0] is sequence[0]


def test_sequence_reads_camera_path_and_typed_vehicle_boxes(tmp_path):
    sequence_root = tmp_path / "train" / "take-a" / "1"
    _write_frame(sequence_root, "2", "[0, 0, 0, 0, 0, 0]", [[6, 3, 1, 2]])
    (sequence_root / "2_camera0.png").write_bytes(b"image")
    (sequence_root / "2.yaml").write_text(
        """lidar_pose: [0, 0, 0, 0, 0, 0]
vehicles:
  42:
    location: [10, 20, 30]
    center: [1, 2, 3]
    angle: [0, 90, 0]
    extent: [2, 1, 0.5]
""",
        encoding="utf-8",
    )

    frame = CooperSceneSequenceDataset(tmp_path).sequence("train", "take-a", 1)[0]

    assert frame.camera_path == sequence_root / "2_camera0.png"
    assert len(frame.vehicle_boxes) == 1
    box = frame.vehicle_boxes[0]
    assert box.track_id == "42"
    assert box.dimensions_lwh == (4.0, 2.0, 1.0)
    np.testing.assert_allclose(box.map_T_object[:3, 3], [8, 21, 33])
    np.testing.assert_allclose(
        box.map_T_object[:3, :3], [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-12
    )


def test_metadata_only_sequence_never_checks_or_reads_pcd(tmp_path, monkeypatch):
    sequence_root = tmp_path / "train" / "take-a" / "1"
    sequence_root.mkdir(parents=True)
    (sequence_root / "2.yaml").write_text(
        "lidar_pose: [2, 0, 0, 0, 0, 0]\nvehicles: {}\n", encoding="utf-8"
    )
    monkeypatch.setattr(
        cooperscene, "read_ascii_xyzi_pcd",
        lambda _path: (_ for _ in ()).throw(AssertionError("PCD read")),
    )

    frame = CooperSceneSequenceDataset.metadata_only(tmp_path).sequence(
        "train", "take-a", 1
    )[0]

    assert frame.frame_id == "2"
    assert frame.lidar_path == sequence_root / "2.pcd"
    assert frame.lidar_points is None
    assert not frame.has_lidar_points
    np.testing.assert_allclose(frame.map_T_lidar[:3, 3], [2, 0, 0])
    with pytest.raises(RuntimeError, match="metadata-only"):
        frame.require_lidar_points()


def test_metadata_only_preserves_all_501_ordered_poses_and_boxes(tmp_path, monkeypatch):
    sequence_root = tmp_path / "train" / "1" / "1"
    sequence_root.mkdir(parents=True)
    for frame_id in range(481260, 481761):
        (sequence_root / f"{frame_id}.yaml").write_text(
            f"""lidar_pose: [{frame_id}, 2, 3, 0, 0, 0]
vehicles:
  car:
    location: [10, 20, 30]
    center: [0, 0, 0]
    angle: [0, 0, 0]
    extent: [2, 1, 0.5]
""",
            encoding="utf-8",
        )
    reads = []
    monkeypatch.setattr(cooperscene, "read_ascii_xyzi_pcd", lambda path: reads.append(path))

    frames = list(
        CooperSceneSequenceDataset.metadata_only(tmp_path, cache_frames=False).sequence(
            "train", 1, 1
        )
    )

    assert len(frames) == 501
    assert [frame.frame_id for frame in frames] == [str(value) for value in range(481260, 481761)]
    assert frames[0].map_T_lidar[0, 3] == 481260
    assert frames[-1].vehicle_boxes[0].dimensions_lwh == (4.0, 2.0, 1.0)
    assert reads == []

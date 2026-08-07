import numpy as np
import pytest

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

import json
from threading import Event

import cv2
import numpy as np
import pytest

from src.data.processed_nuscenes import ProcessedNuScenesSequence
from src.deployment.nuscenes_frame_adapter import (
    NuScenesFrameAdapter,
    NuScenesFrameAdapterConfig,
    _parser,
    serve_processed_scene,
)
from src.deployment.vehicle import VehicleFrameSpool
from src.realtime.process_support import JsonlMetrics


def _write_scene(root, frame_count=2):
    for name in ("images", "intrinsics", "extrinsics", "lidar", "lidar_pose", "instances"):
        (root / name).mkdir(parents=True)
    np.savetxt(root / "intrinsics" / "0.txt", [50, 50, 50, 50, 0, 0, 0, 0, 0])
    anchor = np.eye(4)
    anchor[0, 3] = 10
    for index in range(frame_count):
        frame_id = f"{index:03d}"
        assert cv2.imwrite(str(root / "images" / f"{frame_id}_0.jpg"), np.zeros((100, 100, 3), np.uint8))
        np.savetxt(root / "extrinsics" / f"{frame_id}_0.txt", anchor)
        lidar_pose = anchor.copy()
        lidar_pose[0, 3] += index + 1
        np.savetxt(root / "lidar_pose" / f"{frame_id}.txt", lidar_pose)
        np.asarray([[1, 2, 3, 0.5]], dtype=np.float32).tofile(root / "lidar" / f"{frame_id}.bin")

    object_pose = anchor.copy()
    object_pose[2, 3] = 10
    annotations = {
        "frame_idx": list(range(frame_count)),
        "obj_to_world": [object_pose.tolist()] * frame_count,
        "box_size": [[4.0, 2.0, 2.0]] * frame_count,
    }
    instances = {
        "0": {"id": "stable-car", "class_name": "vehicle.car", "frame_annotations": annotations},
        "1": {"id": "ignored-truck", "class_name": "vehicle.truck", "frame_annotations": annotations},
    }
    frames = {str(index): [0, 1] for index in range(frame_count)}
    (root / "instances" / "instances_info.json").write_text(json.dumps(instances))
    (root / "instances" / "frame_instances.json").write_text(json.dumps(frames))


def test_processed_sequence_uses_bilateral_map_anchor_and_contiguous_10hz(tmp_path):
    root = tmp_path / "152"
    _write_scene(root)

    sequence = ProcessedNuScenesSequence(root, expected_frame_count=2)
    first, second = sequence

    assert len(sequence) == 2
    assert (first.timestamp_us, second.timestamp_us) == (0, 100_000)
    np.testing.assert_allclose(first.map_T_camera, np.eye(4))
    np.testing.assert_allclose(first.map_T_lidar[:3, 3], [1, 0, 0])
    np.testing.assert_allclose(second.map_T_lidar[:3, 3], [2, 0, 0])
    assert first.lidar_points.shape == (1, 4)
    assert [vehicle.track_id for vehicle in first.vehicles] == ["stable-car", "ignored-truck"]
    np.testing.assert_allclose(first.vehicles[0].map_T_object[:3, 3], [0, 0, 10])


def test_processed_sequence_applies_localized_gaussian_global_alignment(tmp_path):
    root = tmp_path / "152"
    _write_scene(root)
    gaussian_T_nuscenes_global = np.eye(4)
    gaussian_T_nuscenes_global[:3, 3] = [3, 4, 5]
    localization = tmp_path / "localization.json"
    localization.write_text(json.dumps({
        "gaussian_T_nuscenes_global": gaussian_T_nuscenes_global.tolist(),
    }))

    first = ProcessedNuScenesSequence(
        root, expected_frame_count=2, localization_transform=localization
    )[0]

    np.testing.assert_allclose(first.map_T_camera[:3, 3], [13, 4, 5])
    np.testing.assert_allclose(first.map_T_lidar[:3, 3], [14, 4, 5])
    np.testing.assert_allclose(first.vehicles[0].map_T_object[:3, 3], [13, 4, 15])


def test_processed_sequence_rejects_localization_for_another_scene(tmp_path):
    root = tmp_path / "152"
    _write_scene(root)
    localization = tmp_path / "localization.json"
    localization.write_text(json.dumps({
        "scene_id": "171",
        "gaussian_T_nuscenes_global": np.eye(4).tolist(),
    }))

    with pytest.raises(ValueError, match="scene 171, not 152"):
        ProcessedNuScenesSequence(
            root, expected_frame_count=2, localization_transform=localization
        )


def test_adapter_spools_only_visible_cars_with_exact_camera_and_lidar_poses(tmp_path):
    root = tmp_path / "152"
    _write_scene(root)
    sequence = ProcessedNuScenesSequence(root, expected_frame_count=2)
    spool = VehicleFrameSpool(tmp_path / "spool")
    metrics_path = tmp_path / "metrics.jsonl"
    clock_values = iter([0, 0, 0, 0, 0.1, 0.1, 0.1])
    adapter = NuScenesFrameAdapter(
        spool,
        NuScenesFrameAdapterConfig("nuscenes-152", minimum_area_px=1),
        metrics=JsonlMetrics(metrics_path),
        monotonic=lambda: next(clock_values),
        sleep=lambda _seconds: None,
    )

    assert adapter.process(sequence) == 2
    first = spool.get("nuscenes-152", 0)
    assert first is not None
    assert [box.track_id for box in first.boxes] == ["stable-car"]
    assert first.boxes[0].text_label == "car"
    np.testing.assert_allclose(np.asarray(first.world_T_ego).reshape(4, 4)[:3, 3], [1, 0, 0])
    np.testing.assert_allclose(np.asarray(first.world_T_camera).reshape(4, 4), np.eye(4))
    assert spool.end("nuscenes-152").final_sequence == 1
    events = [json.loads(line)["event"] for line in metrics_path.read_text().splitlines()]
    assert events == ["frame_spooled", "frame_spooled", "scene_spooled"]


def test_processed_sequence_rejects_frame_gaps(tmp_path):
    root = tmp_path / "152"
    _write_scene(root)
    (root / "images" / "001_0.jpg").rename(root / "images" / "002_0.jpg")

    try:
        ProcessedNuScenesSequence(root, expected_frame_count=None)
    except ValueError as error:
        assert "contiguous" in str(error)
    else:
        raise AssertionError("frame gap was accepted")


def test_cli_accepts_split_system_launcher_arguments(tmp_path):
    args = _parser().parse_args([
        "--scene-root", str(tmp_path / "152"),
        "--frames-endpoint", "tcp://0.0.0.0:5555",
        "--expected-samples", "191",
        "--scene-generation", "nuscenes:152",
        "--metrics-path", str(tmp_path / "metrics.jsonl"),
        "--spool-root", str(tmp_path / "spool"),
        "--frames-per-second", "10",
    ])

    assert args.scene_root == tmp_path / "152"
    assert args.expected_samples == 191
    assert args.frames_per_second == 10


def test_service_spools_scene_and_cleans_up_router(tmp_path):
    root = tmp_path / "152"
    _write_scene(root)
    args = _parser().parse_args([
        "--scene-root", str(root),
        "--frames-endpoint", "inproc://processed-nuscenes-service-test",
        "--expected-samples", "2",
        "--scene-generation", "nuscenes:152",
        "--metrics-path", str(tmp_path / "metrics.jsonl"),
        "--spool-root", str(tmp_path / "spool"),
        "--frames-per-second", "10",
        "--rate", "1000000000",
        "--minimum-area-px", "1",
    ])
    stop = Event()
    stop.set()

    assert serve_processed_scene(args, stop) == 2
    spool = VehicleFrameSpool(tmp_path / "spool")
    assert spool.get("nuscenes:152", 0) is not None
    assert spool.end("nuscenes:152").final_sequence == 1
    records = [json.loads(line) for line in (tmp_path / "metrics.jsonl").read_text().splitlines()]
    assert [record["source_timestamp_us"] for record in records if record["event"] == "frame_spooled"] == [0, 100_000]

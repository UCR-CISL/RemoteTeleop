from __future__ import annotations

import importlib.util
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest


def writer_type():
    path = Path(__file__).parents[1] / "docker/bevfusion/raw_predictions.py"
    spec = importlib.util.spec_from_file_location("raw_predictions", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.RawPredictionWriter


def test_raw_dump_retains_noncar_lowconfidence_and_both_origins(tmp_path):
    wire_path = Path(__file__).parents[1] / "docker/bevfusion/bevfusion_wire.py"
    spec = importlib.util.spec_from_file_location("docker_raw_wire", wire_path)
    wire = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(wire)
    request = wire.BevFusionRequest(request_id="take1:0", scene_generation="take1", sequence=0,
        frame_id="481260", timestamp_us=0, config_id="nuscenes", camera_intrinsic=(1,)*9,
        world_T_camera=tuple(np.eye(4).reshape(-1)), world_T_lidar=tuple(np.eye(4).reshape(-1)),
        image_width=1, image_height=1, image_bgr=b"\0\0\0", lidar_point_count=0, lidar_f32=b"")
    writer = writer_type()(tmp_path)
    boxes = np.asarray([[1, 2, -2, 4, 2, 2, 0], [3, 4, -1, 5, 2, 2, 0]])
    centers = np.asarray([[1, 2, -1], [3, 4, 0]])
    path = writer.write(request, boxes=boxes, gravity_centers=centers, scores=[.01, .9], labels=[1, 0], class_names=["car", "truck"])
    record = json.loads(path.read_text())
    assert record["predictions"][0]["class_name"] == "truck"
    assert record["predictions"][0]["score"] == .01
    assert record["predictions"][0]["box_tensor"][2] == -2
    assert record["predictions"][0]["gravity_center"][2] == -1
    assert record["request_id"] == request.request_id
    assert record["lidar_feature_count"] == 4
    assert record["lidar_payload_sha256"] == hashlib.sha256(request.lidar_f32).hexdigest()
    assert "image_bgr" not in record and "lidar_f32" not in record
    with pytest.raises(FileExistsError):
        writer.write(request, boxes=boxes, gravity_centers=centers, scores=[.01, .9], labels=[1, 0], class_names=["car", "truck"])

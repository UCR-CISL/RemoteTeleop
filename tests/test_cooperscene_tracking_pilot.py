import json
from types import SimpleNamespace

import numpy as np
import pytest

from scripts.investigation.cooperscene_tracking.pilot import TrackingPilot
from src.realtime.bevfusion import BevFusionBox, BevFusionResult


def test_pilot_records_exact_frame_and_keeps_gt_out_of_inference(tmp_path, monkeypatch):
    root = tmp_path / "inputs"
    root.mkdir()
    pose = np.eye(4)
    pose[0, 3] = 10
    image = np.zeros((10, 20, 3), dtype=np.uint8)
    np.savez(root / "frame.npz", image=image, lidar=np.zeros((2, 4), dtype=np.float32))
    row = dict(sequence_id="take01-vehicle1", frame_id="123", timestamp_us=123456,
               payload="frame.npz", camera_intrinsic=np.eye(3).tolist(),
               world_T_camera=np.eye(4).tolist(), world_T_lidar=pose.tolist(),
               gt=[TrackingPilot.box(pose, (4, 2, 1), "GT:must-not-enter-request")])
    (root / "inputs.jsonl").write_text(json.dumps(row) + "\n")
    (root / "manifest.json").write_text(json.dumps(dict(sequences=[dict(inputs="inputs.jsonl", count=1)])))
    requests = []

    class Client:
        def submit(self, request):
            requests.append(request)
            return True

        def poll(self, **kwargs):
            r = requests[-1]
            return BevFusionResult(r.request_id, r.scene_generation, r.sequence, r.frame_id,
                                   r.timestamp_us, r.config_id,
                                   boxes=(BevFusionBox((1, 0, 0), (4, 2, 1), 0, .9),))

        def close(self):
            pass

    monkeypatch.setattr("scripts.investigation.cooperscene_tracking.pilot.BevFusionClient.open", lambda *a, **k: Client())
    output = tmp_path / "output"
    TrackingPilot(root).infer("unused", output, 1)
    result = json.loads((output / "frames.jsonl").read_text())
    assert requests[0].timestamp_us == row["timestamp_us"]
    assert requests[0].frame_id == "123"
    assert "GT:must-not-enter-request" not in str(requests[0].metadata())
    assert result["gt"] == row["gt"]
    assert np.asarray(result["detections"][0]["world_T_object"])[0, 3] == 11
    assert result["tracks"][0]["track_id"] == "bevfusion:car:0"
    with pytest.raises(FileExistsError):
        TrackingPilot(root).infer("unused", output, 1)

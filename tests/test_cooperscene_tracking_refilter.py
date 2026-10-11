import json

import numpy as np
import pytest

from scripts.investigation.cooperscene_tracking.refilter import CachedVehicleClassTrial


def fixture_inputs(tmp_path):
    pose = np.eye(4).tolist()
    intrinsic = np.eye(3).tolist()
    raw_dir = tmp_path / "raw"
    raw_dir.mkdir()
    frames = []
    for index, category in enumerate(("car", "truck")):
        provenance = dict(scene_generation="take1", frame_id=str(index), timestamp_us=index * 100000,
            sequence=index, request_id=f"take1:{index}", world_T_lidar=pose, world_T_camera=pose,
            camera_intrinsic=intrinsic, image_width=100, image_height=100)
        prediction = dict(class_name=category, class_id=index, score=.8,
            box_tensor=[0, 0, 9, 2, 4, 2, 0], gravity_center=[0, 0, 10])
        pedestrian = dict(prediction, class_name="pedestrian", class_id=8, score=.99)
        raw = dict(provenance, predictions=[prediction, pedestrian])
        (raw_dir / f"{index}.json").write_text(json.dumps(raw))
        frame = dict(sequence_id="take1", frame_id=str(index), timestamp_us=index*100000,
            world_T_lidar=pose, world_T_camera=pose, camera_intrinsic=intrinsic, image_width=100, image_height=100,
            gt=[dict(track_id="GT", label="car", world_T_object=pose, dimensions_lwh=[4, 2, 2])], detections=[], tracks=[])
        frames.append(frame)
    source = tmp_path / "frames.jsonl"
    source.write_text("".join(json.dumps(f) + "\n" for f in frames))
    return source, raw_dir, frames


def test_grouping_preserves_detector_labels_geometry_and_excludes_nonvehicles(tmp_path):
    source, raw, original = fixture_inputs(tmp_path)
    output = tmp_path / "out"
    trial = CachedVehicleClassTrial(classes=["car", "truck"])
    manifest = trial.run(source, raw, output)
    result = list(map(json.loads, (output / "frames.jsonl").read_text().splitlines()))
    assert len(result[0]["detections"]) == len(result[1]["detections"]) == 1
    assert result[1]["detections"][0]["detector_class_name"] == "truck"
    assert result[1]["tracks"][0]["detector_class_name"] == "truck"
    assert result[0]["tracks"][0]["track_id"] == result[1]["tracks"][0]["track_id"]
    detection = result[0]["detections"][0]
    assert detection["dimensions_lwh"] == [4, 2, 2]
    np.testing.assert_allclose(np.asarray(detection["world_T_object"])[:3, 0], [0, 1, 0], atol=1e-9)
    assert manifest["gt_used_as_tracker_input"] is False
    for before, after in zip(original, result):
        assert before["timestamp_us"] == after["timestamp_us"]
        assert before["gt"][0]["world_T_object"] == after["gt"][0]["world_T_object"]
        assert after["gt"][0]["annotation_class_name"] == "car"
        assert after["gt"][0]["label"] == "vehicle"
    # Replace GT with arbitrary geometry: tracker and detector outputs remain
    # byte-equivalent, proving this path cannot derive tracking from GT.
    for frame in original:
        frame["gt"] = []
    source.write_text("".join(json.dumps(f) + "\n" for f in original))
    trial.run(source, raw, tmp_path / "without_gt")
    other = list(map(json.loads, (tmp_path / "without_gt/frames.jsonl").read_text().splitlines()))
    assert [f["tracks"] for f in result] == [f["tracks"] for f in other]
    assert [f["detections"] for f in result] == [f["detections"] for f in other]


def test_exact_provenance_required_before_output_directory_created(tmp_path):
    source, raw, _ = fixture_inputs(tmp_path)
    path = raw / "1.json"
    record = json.loads(path.read_text())
    record["timestamp_us"] += 1
    path.write_text(json.dumps(record))
    output = tmp_path / "out"
    with pytest.raises(ValueError, match="missing exact raw"):
        CachedVehicleClassTrial(classes=["car"]).run(source, raw, output)
    assert not output.exists()
    with pytest.raises(ValueError, match="motor vehicle"):
        CachedVehicleClassTrial(classes=["pedestrian"])


def test_low_score_continuation_preserves_primary_detections_and_never_consumes_gt(tmp_path):
    source, raw, original = fixture_inputs(tmp_path)
    path = raw / '1.json'
    record = json.loads(path.read_text())
    record['predictions'][0]['score'] = .15
    path.write_text(json.dumps(record))
    baseline = tmp_path / 'baseline'
    adapted = tmp_path / 'adapted'
    CachedVehicleClassTrial(classes=['car', 'truck']).run(source, raw, baseline)
    trial = CachedVehicleClassTrial(classes=['car', 'truck'], continuation_score_threshold=.1)
    trial.run(source, raw, adapted)
    before = list(map(json.loads, (baseline / 'frames.jsonl').read_text().splitlines()))
    after = list(map(json.loads, (adapted / 'frames.jsonl').read_text().splitlines()))
    assert [row['detections'] for row in before] == [row['detections'] for row in after]
    assert after[1]['detections'] == []
    assert after[0]['tracks'][0]['track_id'] == after[1]['tracks'][0]['track_id']
    assert after[1]['tracks'][0]['score'] == .15
    assert after[1]['tracks'][0]['detector_class_name'] == 'truck'
    for frame in original:
        frame['gt'] = []
    source.write_text(''.join(json.dumps(frame) + '\n' for frame in original))
    trial.run(source, raw, tmp_path / 'without_gt_low')
    other = list(map(json.loads, (tmp_path / 'without_gt_low/frames.jsonl').read_text().splitlines()))
    assert [row['tracks'] for row in after] == [row['tracks'] for row in other]


def test_native_generic_vehicle_class_keeps_ontology_and_rejects_pedestrians(tmp_path):
    source, raw, _ = fixture_inputs(tmp_path)
    for path in raw.glob('*.json'):
        record = json.loads(path.read_text())
        record['predictions'][0].update(class_name='vehicle', class_id=0)
        path.write_text(json.dumps(record))
    output = tmp_path / 'native'
    CachedVehicleClassTrial(classes=['vehicle']).run(source, raw, output)
    result = [json.loads(x) for x in (output/'frames.jsonl').read_text().splitlines()]
    assert all(len(x['detections']) == 1 for x in result)
    assert all(x['detections'][0]['detector_class_name'] == 'vehicle' for x in result)
    assert result[0]['tracks'][0]['track_id'] == result[1]['tracks'][0]['track_id']

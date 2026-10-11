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
    points = np.asarray([[1,2,3,1000],[4,5,6,22]],dtype=np.float32)
    np.savez(root / "frame.npz", image=image, lidar=points)
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
    TrackingPilot(root).infer("unused", output, 1, intensity_clip=255)
    result = json.loads((output / "frames.jsonl").read_text())
    assert requests[0].timestamp_us == row["timestamp_us"]
    assert requests[0].frame_id == "123"
    assert "GT:must-not-enter-request" not in str(requests[0].metadata())
    sent = np.frombuffer(requests[0].lidar_f32,dtype="<f4").reshape(-1,4)
    np.testing.assert_array_equal(sent[:,:3],points[:,:3])
    np.testing.assert_array_equal(sent[:,3],[255,22])
    with np.load(root / "frame.npz") as original:
        np.testing.assert_array_equal(original["lidar"],points)
    assert result["gt"] == row["gt"]
    assert np.asarray(result["detections"][0]["world_T_object"])[0, 3] == 11
    assert result["tracks"][0]["track_id"] == "bevfusion:car:0"
    with pytest.raises(FileExistsError):
        TrackingPilot(root).infer("unused", output, 1)


def _archived_inputs(tmp_path):
    import hashlib
    from src.localization.cooperscene_geodetic import FrozenProfile

    root, search = tmp_path / 'source', tmp_path / 'archive'
    root.mkdir()
    directory = search / 'take01/take01-vehicle1'
    directory.mkdir(parents=True)
    map_path = tmp_path / 'map.ply'
    map_path.write_bytes(b'fixed map provenance')
    recipe_hash = hashlib.sha256(FrozenProfile().path.read_bytes()).hexdigest()
    (directory.parent / 'protocol.json').write_text(json.dumps(dict(recipe_sha256=recipe_hash,
        map_sha256=hashlib.sha256(map_path.read_bytes()).hexdigest())))
    (directory / 'inputs.json').write_text('{}')
    (directory / 'summary.json').write_text(json.dumps(dict(methods=dict(xyz=dict(
        earliest_qualifying=dict(frame=dict(frame_index=0, frame_id='100')))))))
    rows = []
    for index, accepted in enumerate((True, False, False, True)):
        reference = np.eye(4)
        reference[0, 3] = index * 10
        estimated = reference.copy()
        estimated[1, 3] = 1 if index == 0 else 2
        xyz = dict(accepted=accepted, upstream_scan_accepted=accepted,
                   fusion_attempted=accepted, fusion_converged=accepted,
                   status='accepted' if accepted else 'rejected',
                   error=dict(reference_map_T_lidar=reference.tolist(),
                              estimated_map_T_lidar=estimated.tolist() if accepted else 'MUST NOT READ REJECTED POSE'))
        (directory / f'frame-{index:06d}.json').write_text(json.dumps(dict(
            frame=dict(frame_index=index, frame_id=str(100+index), timestamp_ns=(index+1)*100_000_000+999,
                       lidar_ns=(index+1)*100_000_000+999), methods=dict(xyz=xyz))))
        np.savez(root / f'{index}.npz', image=np.zeros((2,2,3),dtype=np.uint8), lidar=np.ones((1,4)))
        rows.append(dict(sequence_id='take01-vehicle1', frame_id=str(100+index), frame_index=index,
                         timestamp_us=(index+1)*100_000, payload=f'{index}.npz',
                         world_T_lidar=reference.tolist(), world_T_camera=reference.tolist(),
                         reference_world_T_lidar=reference.tolist(), gt=[{'track_id': 'GT-evaluation-only'}]))
    (root / 'inputs.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
    (root / 'manifest.json').write_text(json.dumps(dict(recipe_sha256=recipe_hash, sequences=[dict(
        sequence_id='take01-vehicle1', start_index=0, start_frame='100', count=4,
        inputs='inputs.jsonl', correction=np.eye(4).tolist())])))
    return root, search, directory, map_path, rows


def test_xyz_metadata_export_accepts_and_propagates_without_reading_rejected_proposals(tmp_path):
    import hashlib
    from src.localization.cooperscene_calibration import compose_map_T_camera, camera_T_lidar

    root, search, _, map_path, original = _archived_inputs(tmp_path)
    hashes = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in root.iterdir()}
    output = tmp_path / 'export'
    TrackingPilot(root).rebase_xyz(output, search, map_path)
    rows = [json.loads(line) for line in (output / 'inputs.jsonl').read_text().splitlines()]
    assert [row['localization']['accepted'] for row in rows] == [True, False, False, True]
    assert [row['localization']['source_accepted_frame_index'] for row in rows] == [0,0,0,3]
    assert [row['localization']['seconds_since_acceptance'] for row in rows] == [0,.1,.2,0]
    for index, row in enumerate(rows):
        pose = np.asarray(row['world_T_lidar'])
        np.testing.assert_array_equal(pose[:3,3], [10*index, 1 if index<3 else 2, 0])
        np.testing.assert_array_equal(row['world_T_camera'], compose_map_T_camera(pose,camera_T_lidar(1)))
        assert row['localization_xyz_error_m'] == (1 if index<3 else 2)
        for key in ('gt','reference_world_T_lidar','timestamp_us','frame_id','frame_index'):
            assert row[key] == original[index][key]
        assert (output / row['payload']).resolve() == (root / f'{index}.npz').resolve()
    assert not list(output.glob('**/*.npz'))
    assert hashes == {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in root.iterdir()}
    manifest = json.loads((output / 'manifest.json').read_text())
    assert manifest['continuous_xyz_estimator'] is False
    assert manifest['independent_per_frame_xyz'] is True
    assert manifest['sequences'][0]['accepted_frames'] == 2
    assert manifest['sequences'][0]['propagated_frames'] == 2
    assert manifest['sequences'][0]['xyz_pose_provenance']['map_hash_verified'] is True
    assert 'correction' not in manifest['sequences'][0]
    with pytest.raises(FileExistsError):
        TrackingPilot(root).rebase_xyz(output, search, map_path)


@pytest.mark.parametrize('mutation,message', [
    ('timestamp', 'timestamp mismatch'), ('frame_id', 'timestamp mismatch'),
    ('reference', 'reference differs'), ('rigid', 'rigid homogeneous'),
    ('flags', 'contradicts'), ('start', 'earliest strict'), ('recipe', 'recipe hash'),
    ('map', 'map hash'), ('first_rejected', 'start on an accepted'),
])
def test_xyz_export_rejects_mismatched_identity_geometry_or_provenance(tmp_path, mutation, message):
    root, search, directory, map_path, _ = _archived_inputs(tmp_path)
    record_path = directory / 'frame-000000.json'
    record = json.loads(record_path.read_text())
    if mutation == 'timestamp': record['frame']['timestamp_ns'] += 1000
    elif mutation == 'frame_id': record['frame']['frame_id'] = 'other'
    elif mutation == 'reference': record['methods']['xyz']['error']['reference_map_T_lidar'][0][3] += .01
    elif mutation == 'rigid': record['methods']['xyz']['error']['estimated_map_T_lidar'][0][0] = 2
    elif mutation == 'flags': record['methods']['xyz']['fusion_converged'] = False
    elif mutation == 'first_rejected': record['methods']['xyz']['accepted'] = False
    elif mutation == 'start':
        summary = json.loads((directory / 'summary.json').read_text())
        summary['methods']['xyz']['earliest_qualifying']['frame']['frame_index'] = 1
        (directory / 'summary.json').write_text(json.dumps(summary))
    elif mutation in ('recipe', 'map'):
        path = directory.parent / 'protocol.json'
        protocol = json.loads(path.read_text())
        protocol[mutation+'_sha256'] = 'wrong'
        path.write_text(json.dumps(protocol))
    record_path.write_text(json.dumps(record))
    output = tmp_path / 'export'
    with pytest.raises(ValueError, match=message):
        TrackingPilot(root).rebase_xyz(output, search, map_path)
    assert not output.exists()


def test_xyz_rejection_propagates_rigid_left_correction_and_handles_missing_proposal(tmp_path):
    from scripts.investigation.cooperscene_tracking.pilot import ArchivedXYZPosePolicy

    root, _, directory, _, rows = _archived_inputs(tmp_path)
    record_path = directory / 'frame-000000.json'
    record = json.loads(record_path.read_text())
    pose = np.eye(4)
    pose[:2,:2] = [[0,-1],[1,0]]
    pose[1,3] = 1
    record['methods']['xyz']['error']['estimated_map_T_lidar'] = pose.tolist()
    record_path.write_text(json.dumps(record))
    rejected_path = directory / 'frame-000001.json'
    record = json.loads(rejected_path.read_text())
    reference = record['methods']['xyz']['error']['reference_map_T_lidar']
    record['initial'] = dict(error=dict(reference_map_T_lidar=reference))
    record['methods']['xyz']['error'] = {}  # Certified rejection has no optimizer pose.
    rejected_path.write_text(json.dumps(record))
    recipe = json.loads((root/'manifest.json').read_text())['recipe_sha256']
    policy = ArchivedXYZPosePolicy(directory, recipe_sha256=recipe)
    policy.pose(rows[0])
    propagated, diagnostics = policy.pose(rows[1])
    expected = pose @ np.asarray(rows[1]['reference_world_T_lidar'])
    np.testing.assert_array_equal(propagated, expected)
    np.testing.assert_array_equal(propagated[:3,3], [0,11,0])
    assert diagnostics['propagated'] and not diagnostics['accepted']
    with pytest.raises(ValueError, match='consecutive'):
        policy.pose(rows[3])


def test_pilot_past_sweeps_use_sensor_poses_and_limit_development_frames(tmp_path, monkeypatch):
    root = tmp_path / 'inputs'
    root.mkdir()
    rows = []
    for i in range(3):
        pose = np.eye(4)
        pose[0, 3] = i
        np.savez(root / f'{i}.npz', image=np.zeros((1, 1, 3), np.uint8),
                 lidar=np.array([[3, 0, 0, 1000]], dtype=np.float32))
        rows.append(dict(sequence_id='take01-vehicle1', frame_id=str(i), timestamp_us=i*100000,
                         payload=f'{i}.npz', camera_intrinsic=np.eye(3).tolist(),
                         world_T_camera=pose.tolist(), world_T_lidar=pose.tolist(),
                         gt=[TrackingPilot.box(pose, (4, 2, 1), 'GT:private')]))
    (root / 'inputs.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    (root / 'manifest.json').write_text(json.dumps(dict(sequences=[dict(inputs='inputs.jsonl', count=3)])))
    requests = []
    class Client:
        def submit(self, value):
            requests.append(value)
            return True
        def poll(self, **kwargs):
            r = requests[-1]
            return BevFusionResult(r.request_id, r.scene_generation, r.sequence, r.frame_id, r.timestamp_us, r.config_id)
        def close(self):
            pass
    monkeypatch.setattr('scripts.investigation.cooperscene_tracking.pilot.BevFusionClient.open', lambda *a, **k: Client())
    output = tmp_path / 'output'
    TrackingPilot(root).infer('unused', output, 1, intensity_clip=255, past_sweeps=9, limit_frames=2)
    assert len(requests) == 2
    assert requests[1].lidar_feature_count == 5
    np.testing.assert_allclose(np.frombuffer(requests[1].lidar_f32, dtype='<f4').reshape(-1,5),
                               [[3,0,0,255,0],[2,0,0,255,.1]])
    result = [json.loads(x) for x in (output / 'frames.jsonl').read_text().splitlines()]
    assert [r['gt'] for r in result] == [r['gt'] for r in rows[:2]]
    assert all('GT:private' not in str(r.metadata()) for r in requests)
    assert json.loads((output / 'manifest.json').read_text())['sequences'][0]['count'] == 2
    with np.load(root / '0.npz') as original:
        assert original['lidar'][0,3] == 1000

import json
from types import SimpleNamespace

import numpy as np
import pytest

from src.localization.trajectory_metrics import TrajectoryMetrics
from src.deployment.ros_frame_adapter import RosFrameAdapter, RosFrameAdapterConfig


def test_xyz_includes_vertical_error_and_tracks_entire_trajectory(tmp_path):
    path = tmp_path / 'trajectory.jsonl'
    metrics = TrajectoryMetrics(path, reference_kind='shared_enu_prior', expected_samples=2)
    pose = np.eye(4)
    pose[:3, 3] = [3, 4, -12]
    row = metrics.record(sequence=0, timestamp_us=0, frame_id='a', estimated_pose=pose, reference_pose=np.eye(4))
    assert row['translation_m'] == 13
    assert row['horizontal_translation_m'] == 5
    assert row['dz_m'] == -12
    assert row['vertical_translation_m'] == 12
    assert not metrics.finish()['complete']
    metrics.record(sequence=1, timestamp_us=100000, frame_id='b', estimated_pose=np.eye(4), reference_pose=np.eye(4))
    summary = json.loads(path.with_suffix('.summary.json').read_text())
    assert summary['complete'] and summary['frame_count'] == 2
    assert summary['translation_m']['rmse'] == pytest.approx(13 / np.sqrt(2))
    assert summary['independent_ground_truth'] is False
    assert len(path.read_text().splitlines()) == 2
    with pytest.raises(ValueError, match='already exist'):
        TrajectoryMetrics(path, reference_kind='shared_enu_prior')


@pytest.mark.parametrize('dataset', ['cooperscene', 'nuscenes'])
def test_adapter_removes_stored_alignment_before_reference_comparison(tmp_path, dataset):
    stored, estimated, reference, dataset_pose = [np.eye(4) for _ in range(4)]
    stored[0, 3] = 100
    estimated[2, 3] = 3
    reference[2, 3] = 1
    dataset_pose[:3, 3] = [10, 20, 30]
    config = (RosFrameAdapterConfig(gaussian_T_cooperscene=estimated, stored_gaussian_T_cooperscene=stored)
              if dataset == 'cooperscene' else RosFrameAdapterConfig(gaussian_T_nuscenes_global=estimated))
    path = tmp_path / 'trajectory.jsonl'
    metrics = TrajectoryMetrics(path, reference_kind='shared_enu_prior', expected_samples=1)
    adapter = RosFrameAdapter(SimpleNamespace(send=lambda value: None), config,
                              localization_metrics=metrics, localization_reference_transform=reference)
    payload = dict(version=1, scene_generation=f'{dataset}:1', timestamp_us=0, frame_id='frame',
                   world_T_ego=(stored @ dataset_pose).reshape(-1).tolist(),
                   map_T_lidar=(stored @ dataset_pose).reshape(-1).tolist(), boxes=[],
                   gaussian_T_nuscenes_global=stored.reshape(-1).tolist())
    adapter.receive_metadata(payload)
    row = json.loads(path.read_text())
    assert row['dx_m'] == 0
    assert row['dz_m'] == 2
    assert row['translation_m'] == 2
    assert row['reference_map_T_lidar'][0][3] == 10


def test_camera_oriented_map_uses_dataset_vertical_axis(tmp_path):
    metrics = TrajectoryMetrics(tmp_path / 'errors.jsonl', reference_kind='recorded_pose_alignment_baseline')
    rotation = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
    pose = np.eye(4)
    pose[:3, 3] = rotation @ np.array([0., 0., 2.])
    row = metrics.record(sequence=0, timestamp_us=0, frame_id='0', estimated_pose=pose,
                         reference_pose=np.eye(4), map_R_dataset=rotation)
    assert row['dz_m'] == 2
    assert row['horizontal_translation_m'] == 0
    assert row['error_axes'] == 'dataset'

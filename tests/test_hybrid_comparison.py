"""Exercise the boundary between a height proposal, original gates, and evaluation."""
from dataclasses import asdict, dataclass
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from scripts.investigation import evaluate_hybrid_road_height as module
from src.localization.hybrid_road_height import HybridRoadHeightResult


@dataclass
class GateResult:
    accepted: bool


def comparison(tmp_path, monkeypatch, *, dataset='nuscenes', road_accepted=True,
               gate_accepted=True, reference_z=0., qualified=True):
    run_id = dataset + '-example'
    reference = np.eye(4)
    reference[2, 3] = reference_z
    box = np.eye(4)
    box[2, 3] = -2
    frame_path = tmp_path / 'frame.npz'
    tmp_path.mkdir(parents=True, exist_ok=True)
    np.savez(frame_path, points_lidar=np.array([[0, 0, -2, 1], [5, 0, -2, 1], [10, 0, -2, 1]]),
             dataset_T_lidar=np.eye(4), dataset_T_camera=np.eye(4),
             reference_map_T_dataset=reference, pipeline_initial_map_T_dataset=np.eye(4),
             boxes_T_dataset_object=np.array([box]), boxes_dimensions_lwh=np.ones((1, 3)),
             box_labels=np.array(['car']))
    calls = []
    runner = module.HybridComparison.__new__(module.HybridComparison)
    runner.output = tmp_path / 'results-root'
    runner.coverage = {'bev': {run_id: {'status': 'qualified' if qualified else 'fallback_frame_zero_inherited_transform'}}}
    runner.frame = lambda name, offset: dict(run_id=name, dataset=dataset, npz_path=str(frame_path), map_path='unused')
    runner.load_map = lambda path: None
    runner.frozen_transform = lambda *args: (np.eye(4), 'fixed test pose')
    runner.full_target = np.zeros((3, 3))
    runner.road_points = np.zeros((3, 3))
    runner.road_normals = np.tile([0, 0, 1], (3, 1))

    class Estimator:
        def estimate(self, points, **kwargs):
            calls.append(('estimate', points.copy(), kwargs))
            assert 'reference' not in kwargs
            candidate = kwargs['map_T_lidar'].copy()
            candidate[2, 3] += 1
            return HybridRoadHeightResult(road_accepted, 'test_candidate', candidate)

    class Registrar:
        def __init__(self, config):
            self.config = config
            assert asdict(config) == asdict(module.registration_config(dataset))

        def evaluate(self, source, target, transform, initial):
            # Estimates must be durably recorded before reference evaluation.
            assert (runner.output / 'predictions' / f'{run_id}-f000000.json').is_file()
            calls.append(('evaluate', source.copy(), transform.copy(), initial.copy()))
            return GateResult(gate_accepted)

    runner.estimator = Estimator()
    monkeypatch.setattr(module, 'PointCloudRegistrar', Registrar)
    return runner.run_one(run_id, 0), calls


@pytest.mark.parametrize('dataset,registration_count', [('nuscenes', 3), ('cooperscene', 2)])
def test_original_registration_inputs_and_gates_are_identical_across_methods(tmp_path, monkeypatch, dataset, registration_count):
    result, calls = comparison(tmp_path, monkeypatch, dataset=dataset)
    evaluations = [call for call in calls if call[0] == 'evaluate']
    assert len(evaluations) == 3
    assert len(calls[0][1]) == 2  # The road fit always excludes the annotated vehicle.
    for call in evaluations:
        assert len(call[1]) == registration_count
        np.testing.assert_array_equal(call[1], evaluations[0][1])
        np.testing.assert_array_equal(call[3], np.eye(4))
    assert result['methods']['hybrid']['accepted']


@pytest.mark.parametrize('road_accepted,gate_accepted', [(False, True), (True, False), (False, False)])
def test_failed_stage_retains_bev_pose_and_identity_applied_correction(tmp_path, monkeypatch, road_accepted, gate_accepted):
    result, _ = comparison(tmp_path, monkeypatch, road_accepted=road_accepted, gate_accepted=gate_accepted)
    assert not result['methods']['hybrid']['accepted']
    assert result['methods']['hybrid']['status'] == 'fallback_unchanged_bev'
    np.testing.assert_array_equal(result['methods']['hybrid']['map_T_dataset'], np.eye(4))
    np.testing.assert_array_equal(result['applied_sensor_correction_map_T_map'], np.eye(4))


def test_reference_and_historical_qualification_change_only_evaluation(tmp_path, monkeypatch):
    first, _ = comparison(tmp_path / 'first', monkeypatch, reference_z=0, qualified=True)
    changed, _ = comparison(tmp_path / 'changed', monkeypatch, reference_z=100, qualified=False)
    assert module.jsonable(first['road_height']) == module.jsonable(changed['road_height'])
    assert first['methods']['hybrid']['accepted'] == changed['methods']['hybrid']['accepted'] is True
    np.testing.assert_array_equal(first['methods']['hybrid']['map_T_dataset'], changed['methods']['hybrid']['map_T_dataset'])
    assert first['methods']['hybrid']['error']['dz_m'] != changed['methods']['hybrid']['error']['dz_m']


def test_missing_native_cache_never_calls_remote_staging(tmp_path):
    runner = module.HybridComparison.__new__(module.HybridComparison)
    runner.previous = tmp_path / 'previous'
    runner.entries = {'nuscenes-987': {'scene': '987'}}
    runner.nusc = SimpleNamespace(output=tmp_path / 'staging', _stage=lambda *args: pytest.fail('remote staging must not run'))
    (runner.nusc.output / 'recorded/987').mkdir(parents=True)
    with pytest.raises(FileNotFoundError, match='remote fetch disabled'):
        runner.frame('nuscenes-987', 123)

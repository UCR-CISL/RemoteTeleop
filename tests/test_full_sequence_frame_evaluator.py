from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial import cKDTree

from scripts.investigation import full_sequence_frame_evaluator as module
from src.localization.registration import RegistrationResult
from src.localization.enu_prior_fusion import EnuPriorFusionResult


@pytest.fixture
def setup(tmp_path, monkeypatch):
    points = np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]])
    path = tmp_path / 'frame.npz'
    arrays = dict(points_lidar=points, dataset_T_lidar=np.eye(4), dataset_T_camera=np.eye(4),
                  reference_map_T_dataset=np.eye(4), pipeline_initial_map_T_dataset=np.eye(4),
                  boxes_T_dataset_object=np.empty((0, 4, 4)), boxes_dimensions_lwh=np.empty((0, 3)))
    np.savez(path, **arrays)
    instance = object.__new__(module.FullSequenceFrameEvaluator)
    instance.output = tmp_path / 'output'
    instance.full_target = points
    instance._tree = cKDTree(points)
    instance.load_map = lambda path: None
    instance._identity = lambda case: ('version1', {})
    instance._fixed_case = lambda *args: None
    case = dict(run_id='cooperscene-1-1', frame_offset=0,
                metadata=dict(dataset='cooperscene', frame_id='frame0', npz_path=str(path), map_path='unused'))
    calls = dict(register=0, fusion=[], fail=False)

    class Registrar:
        def __init__(self, config):
            self.config = config

        def register(self, source, target, initial):
            calls['register'] += 1
            if calls['fail']:
                raise RuntimeError('synthetic optimizer nonconvergence')
            return self.evaluate(source, target, initial, initial)

        def evaluate(self, source, target, transform, initial):
            return RegistrationResult(np.asarray(transform), 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., True)

    class Fuser:
        def __init__(self, config):
            self.config = config

        def fuse(self, source, target, scan, reference, lidar):
            calls['fusion'].append(self.config.position_mode)
            return EnuPriorFusionResult(np.asarray(scan), 1, True, len(source), 0., 0.)

    monkeypatch.setattr(module, 'PointCloudRegistrar', Registrar)
    monkeypatch.setattr(module, 'EnuPriorFuser', Fuser)
    return instance, case, calls, arrays


def test_shared_scan_and_independent_fusions_resume_without_repeating_fit(setup):
    evaluator, case, calls, _ = setup
    first = evaluator.evaluate_case(case, ['bev'])
    assert first['methods']['bev']['accepted']
    assert calls == dict(register=1, fusion=['bev'], fail=False)
    resumed = evaluator.evaluate_case(case, ['bev', 'xyz'])
    assert resumed['methods']['xyz']['accepted']
    assert calls == dict(register=1, fusion=['bev', 'xyz'], fail=False)
    evaluator.evaluate_case(case, ['xyz'])
    assert calls['register'] == 1 and calls['fusion'] == ['bev', 'xyz']


def test_optimizer_failure_counts_as_explicit_failed_methods(setup):
    evaluator, case, calls, _ = setup
    calls['fail'] = True
    result = evaluator.evaluate_case(case, ['bev', 'xyz', 'bev_lidar_height'])
    assert set(result['methods']) == {'bev', 'xyz', 'bev_lidar_height'}
    for row in result['methods'].values():
        assert row['accepted'] is False
        assert row['status'] == 'optimizer_failed'
        assert row['error'] == {}


def test_data_and_cache_identity_failures_propagate_as_incomplete(setup):
    evaluator, case, _, arrays = setup
    evaluator.evaluate_case(case, ['bev'])
    evaluator._identity = lambda case: ('changed_configuration', {})
    with pytest.raises(ValueError, match='fingerprint'):
        evaluator.evaluate_case(case, ['bev'])
    evaluator.output = evaluator.output / 'new'
    arrays['dataset_T_lidar'][0, 0] = 2
    np.savez(case['metadata']['npz_path'], **arrays)
    with pytest.raises(ValueError, match='invalid dataset_T_lidar'):
        evaluator.evaluate_case(case, ['bev'])
    Path(case['metadata']['npz_path']).unlink()
    with pytest.raises(FileNotFoundError):
        evaluator.evaluate_case(case, ['bev'])


def test_full_gate_impossibility_certificate_skips_all_optimizers(setup):
    evaluator, case, calls, _ = setup
    evaluator.full_target = evaluator.full_target + 100
    evaluator._tree = cKDTree(evaluator.full_target)
    result = evaluator.evaluate_case(case, ['bev', 'xyz', 'bev_lidar_height'])
    assert result['rejection_certificate']['certified_impossible']
    assert calls['register'] == 0 and not calls['fusion']
    assert all(row['reason'] == 'certified_no_accepted_transform' for row in result['methods'].values())

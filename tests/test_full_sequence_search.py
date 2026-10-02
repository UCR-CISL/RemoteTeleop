"""Policy checks for chronology, independent thresholds and resumable evidence."""
from types import SimpleNamespace

import numpy as np
import pytest

from scripts.investigation import search_full_sequence_localization as search


class Frames:
    count = 5
    times = [0., 1., 2., 3., 4.]

    def __init__(self, *args):
        pass

    def frame(self, offset):
        return dict(frame_id=str(offset), npz_path='unused')

    def timing(self, offset):
        return dict(frame_offset=offset, frame_count=self.count, elapsed_seconds=float(offset),
                    duration_seconds=4., sequence_percent=25. * offset)

    def close(self):
        pass


class Evaluator:
    def __init__(self, *, fail=None, all_rejected=False):
        self.calls = []
        self.cache = {}
        self.fits = []
        self.fail = fail
        self.all_rejected = all_rejected

    def evaluate_case(self, case, required_methods):
        index = case['frame_offset']
        self.calls.append((index, list(required_methods)))
        if index == self.fail:
            raise OSError('source unavailable')
        errors = dict(bev=(.05 if index >= 1 else .1, .05 if index >= 4 else .2),
                      xyz=(.05, .05 if index >= 2 else .2),
                      bev_lidar_height=(.05, .05 if index >= 3 else .2))
        for method in required_methods:
            if (index, method) not in self.cache:
                self.fits.append((index, method))
                self.cache[(index, method)] = dict(accepted=not self.all_rejected,
                    error=dict(xy_m=errors[method][0], xyz_m=errors[method][1]), map_T_dataset=np.eye(4).tolist())
        return dict(methods={method: self.cache[(index, method)] for method in required_methods})


@pytest.fixture
def runner(tmp_path, monkeypatch):
    monkeypatch.setattr(search, 'SequenceFrames', Frames)
    instance = object.__new__(search.FullSequenceSearch)
    instance.args = SimpleNamespace(previous=tmp_path, output=tmp_path,
        trajectory_root=tmp_path, nuscenes_recorded_root=None, max_frames=None)
    instance.evaluator = Evaluator()
    return instance


ENTRY = dict(run_id='example', dataset='nuscenes', npz_path='first_frame')


def test_strict_threshold_and_independent_first_hits(runner):
    result = runner.run_sequence(ENTRY)
    assert result['status'] == 'complete'
    assert {key: row['selected']['frame_offset'] for key, row in result['methods'].items()} == {
        'bev': 1, 'xyz': 2, 'bev_lidar_height': 3, 'bev_xyz_benchmark': 4}
    assert runner.evaluator.calls[-1] == (4, ['bev'])
    assert all(row['every_frame_before_selection_tested'] for row in result['methods'].values())


def test_resume_uses_saved_evidence_without_new_fits(runner):
    first = runner.run_sequence(ENTRY)
    runner.evaluator.calls.clear()
    fits = list(runner.evaluator.fits)
    assert runner.run_sequence(ENTRY) == first
    assert runner.evaluator.calls  # Every resume still validates evaluator cache inputs.
    assert runner.evaluator.fits == fits


def test_data_failure_cannot_claim_exhaustion(runner):
    runner.evaluator.fail = 2
    result = runner.run_sequence(ENTRY)
    assert result['status'] == 'incomplete'
    assert result['methods']['bev']['selected']['frame_offset'] == 1
    assert result['methods']['xyz']['tested_frame_offsets'] == [0, 1]
    assert all(not row['exhausted'] for row in result['methods'].values())


def test_exhausted_only_after_every_native_frame(runner):
    runner.evaluator.all_rejected = True
    result = runner.run_sequence(ENTRY)
    assert result['status'] == 'complete'
    for row in result['methods'].values():
        assert row['exhausted']
        assert row['tested_frame_offsets'] == list(range(5))
        assert not row['frame_zero_inherited_fallback']['newly_accepted_localization']


def test_smoke_bound_does_not_mark_exhausted(runner):
    runner.args.max_frames = 2
    result = runner.run_sequence(ENTRY)
    assert result['status'] == 'incomplete_smoke_bound'
    assert not result['methods']['xyz']['exhausted']


@pytest.mark.parametrize('error', [-1., float('nan'), float('inf')])
def test_invalid_accepted_metrics_are_not_search_evidence(runner, error):
    runner.evaluator.cache[(0, 'xyz')] = dict(accepted=True,
        error=dict(xy_m=0., xyz_m=error), map_T_dataset=np.eye(4).tolist())
    result = runner.run_sequence(ENTRY)
    assert result['status'] == 'incomplete'
    assert result['methods']['xyz']['tested_frame_offsets'] == []


def test_accepted_pose_is_required_for_qualification(runner):
    runner.evaluator.cache[(0, 'xyz')] = dict(accepted=True,
        error=dict(xy_m=0., xyz_m=0.), map_T_dataset=None)
    assert runner.run_sequence(ENTRY)['status'] == 'incomplete'

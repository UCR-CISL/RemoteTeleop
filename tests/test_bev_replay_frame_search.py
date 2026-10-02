"""Replay selection must not turn rejected/reference-only fits into successes."""
import json

import numpy as np
import pytest

from scripts.investigation.search_bev_replay_frames import BevReplayFrameSearch


@pytest.fixture
def search(tmp_path):
    inherited = np.eye(4)
    inherited[0, 3] = 2.0
    npz = tmp_path / "frame.npz"
    np.savez(npz, inherited_map_T_dataset=inherited,
             reference_map_T_dataset=np.eye(4), dataset_T_lidar=np.eye(4))
    entry = dict(run_id="nuscenes-test", dataset="nuscenes", npz_path=str(npz))
    runner = object.__new__(BevReplayFrameSearch)
    runner.output = tmp_path
    runner.entries = [entry]
    return runner, entry


def rows(candidates):
    return [dict(frame_offset=index, frame_id=str(index), elapsed_seconds=index / 10,
                 accepted=accepted, target_hit=accepted and error < .1,
                 preferred=index == 0, error=dict(xy_m=error, xyz_m=error, dz_m=0),
                 registration=dict(map_T_source=np.eye(4).tolist()))
            for index, (accepted, error) in enumerate(candidates)]


def summarize(search, candidates, complete=True):
    runner, entry = search
    runner.summarize(entry, rows(candidates), count=9, complete=complete)
    return json.loads((runner.output / "selections.json").read_text())["sequences"][0]


def test_smallest_accepted_error_wins_even_when_later(search):
    result = summarize(search, [(False, .01), (True, .09), (True, .08)])
    assert result["status"] == "qualified"
    assert result["replay_start_frame"] == 2
    assert result["selected"]["error"]["xy_m"] == .08
    assert result["eligible_window_fully_tested"]
    assert result["best_accepted_before_15_percent"] is None


@pytest.mark.parametrize("candidates,reason", [
    ([(True, .12), (True, .1), (False, .01)], "no_accepted_fit_below_0.1m"),
    ([(False, .01), (False, .02), (False, .03)], "no_accepted_fit"),
])
def test_no_good_fit_uses_original_alignment_not_fitted_candidate(search, candidates, reason):
    result = summarize(search, candidates)
    assert result["status"] == "fallback_frame_zero"
    assert result["fallback_reason"] == reason
    assert result["replay_start_frame"] == 0
    assert result["selected"]["error"]["xy_m"] == 2.0
    assert result["selected"]["fallback_existing_alignment"]
    assert not result["selected"]["target_hit"]
    exported = json.loads((search[0].output / "transforms/nuscenes-test.json").read_text())
    assert exported["map_T_dataset"][0][3] == 2.0
    assert exported["fallback_existing_alignment"]


def test_partial_search_cannot_release_replay(search):
    result = summarize(search, [(True, .01)], complete=False)
    assert result["status"] == "searching"
    assert not result["eligible_window_fully_tested"]
    assert result["selected"] is None
    assert "replay_transform_path" not in result
    assert not (search[0].output / "transforms/nuscenes-test.json").exists()

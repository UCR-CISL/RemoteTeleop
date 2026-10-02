"""Queue lifecycle tests use fake launchers and never connect to replay hosts."""
import fcntl
import json
from pathlib import Path

import pytest

from scripts.deployment.run_remote_teleop_batch import ReplayBatch
from scripts.deployment.run_remote_teleop_simulation import LaunchError


def manifest(tmp_path, ids=("scene-1", "scene-2")):
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps({"version": 1, "runs": [{"id": name, "dataset": "nuscenes", "scene": "152", "mcap": "data/152.mcap", "splat": "data/152.ply", "expected_samples": 191} for name in ids]}))
    return path


def factory(calls, failures=()):
    class Launcher:
        def __init__(self, options, sam, **kwargs):
            self.options, self.sam, self.kwargs = options, sam, kwargs
        def run(self):
            calls.append((self.options, self.sam, self.kwargs))
            assert self.sam.asset_proxy
            assert self.options.bbox_source == "gt"
            if self.options.local_artifacts.parent.name in failures:
                raise LaunchError("example failure")
            return self.options.local_artifacts / "composited/composited.mp4"
    return Launcher


def test_dry_run_has_no_state_side_effects(tmp_path):
    calls = []
    root = tmp_path / "queue"
    assert ReplayBatch(manifest(tmp_path), root, launcher_factory=factory(calls)).run(dry_run=True) == 0
    assert not root.exists()
    assert len(calls) == 2
    assert all(call[2]["dry_run"] for call in calls)


def test_resume_skips_success_and_retries_failed_with_fresh_paths(tmp_path):
    calls = []
    path, root = manifest(tmp_path), tmp_path / "queue"
    assert ReplayBatch(path, root, launcher_factory=factory(calls, {"scene-2"})).run() == 1
    assert ReplayBatch(path, root, launcher_factory=factory(calls)).run() == 0
    assert [call[0].local_artifacts.name for call in calls] == ["attempt-001", "attempt-001", "attempt-002"]
    assert calls[1][0].output_root != calls[2][0].output_root
    state = json.loads((root / "state.json").read_text())
    assert state["runs"]["scene-1"]["status"] == "succeeded"
    assert len(state["runs"]["scene-2"]["attempts"]) == 2


@pytest.mark.parametrize("keep_going, count", [(False, 1), (True, 2)])
def test_failure_policy(tmp_path, keep_going, count):
    calls = []
    batch = ReplayBatch(manifest(tmp_path), tmp_path / "queue", launcher_factory=factory(calls, {"scene-1"}))
    assert batch.run(continue_on_error=keep_going) == 1
    assert len(calls) == count


def test_manifest_mismatch_and_duplicate_ids_rejected(tmp_path):
    path = manifest(tmp_path)
    root = tmp_path / "queue"
    ReplayBatch(path, root, launcher_factory=factory([])).run()
    value = json.loads(path.read_text())
    value["runs"][0]["expected_samples"] = 7
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError, match="manifest changed"):
        ReplayBatch(path, root, launcher_factory=factory([])).run()
    with pytest.raises(ValueError, match="unique"):
        ReplayBatch(manifest(tmp_path, ("same", "same")), root)


def test_concurrent_writer_rejected(tmp_path):
    path, root = manifest(tmp_path), tmp_path / "queue"
    root.mkdir()
    with (root / ".queue.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(LaunchError, match="another process"):
            ReplayBatch(path, root, launcher_factory=factory([])).run()
    assert not (root / "state.json").exists()


def test_interrupt_recorded_before_propagating(tmp_path):
    class Interrupted:
        def __init__(self, *args, **kwargs):
            pass
        def run(self):
            raise KeyboardInterrupt()
    root = tmp_path / "queue"
    with pytest.raises(KeyboardInterrupt):
        ReplayBatch(manifest(tmp_path), root, launcher_factory=Interrupted).run()
    state = json.loads((root / "state.json").read_text())
    assert state["runs"]["scene-1"]["status"] == "failed"
    assert "KeyboardInterrupt" in state["runs"]["scene-1"]["attempts"][0]["error"]
    assert state["runs"]["scene-2"]["status"] == "queued"


def test_unclean_interruption_requires_explicit_cleanup_acknowledgment(tmp_path):
    calls = []
    path, root = manifest(tmp_path), tmp_path / "queue"
    batch = ReplayBatch(path, root, launcher_factory=factory(calls))
    batch.run()
    state_path = root / "state.json"
    state = json.loads(state_path.read_text())
    entry = state["runs"]["scene-1"]
    entry["status"] = entry["attempts"][0]["status"] = "running"
    state_path.write_text(json.dumps(state))
    calls.clear()
    before = state_path.read_text()
    with pytest.raises(LaunchError, match="clean up.*--retry-interrupted"):
        batch.run()
    assert not calls
    assert state_path.read_text() == before
    assert batch.run(retry_interrupted=True) == 0
    assert len(calls) == 1
    assert calls[0][0].local_artifacts.name == "attempt-002"


def test_max_runs_leaves_remainder_queued_then_resumes(tmp_path):
    path, root = manifest(tmp_path), tmp_path / 'queue'
    calls = []
    batch = ReplayBatch(path, root, launcher_factory=factory(calls))
    assert batch.run(max_runs=1) == 0
    state = json.loads((root / 'state.json').read_text())
    assert state['runs']['scene-1']['status'] == 'succeeded'
    assert state['runs']['scene-2']['status'] == 'queued'
    assert batch.run(max_runs=1) == 0
    assert len(calls) == 2


def test_input_wait_keeps_queue_without_consuming_attempt(tmp_path):
    path, root, calls = manifest(tmp_path, ("scene-1",)), tmp_path / "queue", []
    sleeps = []
    class Readiness:
        last_error = "still copying"
        def ready(self, options, sam):
            state = json.loads((root / "state.json").read_text())
            assert state["runs"]["scene-1"] == {"status": "queued", "attempts": []}
            return bool(sleeps)
    batch = ReplayBatch(path, root, launcher_factory=factory(calls), readiness=Readiness(), sleep=sleeps.append)
    assert batch.run(wait_for_inputs=True) == 0
    assert sleeps == [30]
    assert len(calls) == 1


def test_input_timeout_retains_queued_state_and_dryrun_never_probes(tmp_path):
    root, calls = tmp_path / "queue", []
    class Readiness:
        last_error = "missing footer"
        def ready(self, *args):
            return False
    now = [0]
    def sleep(seconds):
        now[0] += seconds
    batch = ReplayBatch(manifest(tmp_path, ("scene-1",)), root, launcher_factory=factory(calls), readiness=Readiness(), sleep=sleep, clock=lambda: now[0])
    with pytest.raises(LaunchError, match="timed out waiting.*missing footer"):
        batch.run(wait_for_inputs=True, input_timeout_seconds=31)
    assert not calls
    assert json.loads((root / "state.json").read_text())["runs"]["scene-1"] == {"status": "queued", "attempts": []}
    class NoProbe:
        def ready(self, *args):
            raise AssertionError("network probe in dry-run")
    batch.readiness = NoProbe()
    assert batch.run(dry_run=True, wait_for_inputs=True) == 0


def test_remote_readiness_checks_mcap_and_splat_with_quoted_commands(tmp_path):
    import shlex
    import subprocess
    from scripts.deployment.run_remote_teleop_batch import ReplayInputReadiness
    calls = []
    def runner(command, **kwargs):
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(command, 0, "", "")
    batch = ReplayBatch(manifest(tmp_path), tmp_path / "queue")
    options, sam = batch._options(batch.runs[0], 1)
    readiness = ReplayInputReadiness(runner=runner)
    assert readiness.ready(options, sam)
    assert len(calls) == 2
    vehicle_args = shlex.split(calls[0][0][-1])
    assert vehicle_args[-2] == ReplayInputReadiness.MCAP_PROBE
    spec = json.loads(vehicle_args[-1])
    assert spec["expected"] == 191 and spec["agents"] == ["1"]
    assert calls[1][0][-1].startswith("test -s ")
    assert all(kwargs["timeout"] == 30 for _, kwargs in calls)
    compile(ReplayInputReadiness.MCAP_PROBE, "readiness_probe", "exec")

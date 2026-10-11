"""Preparation publishes a queue only after staging and inspection succeed."""
import json
from pathlib import Path
import subprocess
import shlex

import pytest

from scripts.deployment.prepare_remote_teleop_batch import BatchPreparer, parser


@pytest.fixture
def args(tmp_path):
    transform = tmp_path / "transform.json"
    transform.write_text(json.dumps({"config": {"enu_prior_position_mode": "xyz", "splat_path": "data/riverside_iowa.ply"}, "gaussian_T_cooperscene": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]}))
    return parser().parse_args([
        "--config", "cfg/alien3_lambda.yaml", "--output", str(tmp_path / "manifest.json"),
        "--cache", str(tmp_path / "cache"),
        "--cooperscene-localization-transform", str(transform),
        "--cooperscene-stored-localization-transform", str(transform),
        "--cooperscene-reference-transform", str(transform),
    ])


def test_dry_run_never_executes_or_writes(args):
    args.dry_run = True
    def forbidden(*a, **kw):
        raise AssertionError("dry-run executed a command")
    assert BatchPreparer(args, run=forbidden).prepare() is None
    assert not args.output.exists()
    assert not args.cache.exists()


def test_preparation_builds_all_23_runs(args):
    commands = []
    def run(command, **kwargs):
        commands.append(command)
        if command[0] == "ssh" and command[-1].startswith("cat "):
            path = Path(shlex.split(command[-1])[1])
            matrix = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
            key = "reference_T_dataset" if path.name == "reference.json" else "gaussian_T_nuscenes_global"
            return subprocess.CompletedProcess(command, 0, json.dumps({
                "scene_id": path.parent.name, "registration": {"accepted": True}, key: matrix}))
        inspection = "CompressedImage" in " ".join(command)
        payload = dict(expected_samples=191, camera_source_size=[1600, 900], metadata={
            "dataset": "nuscenes", "camera_intrinsic": json.dumps([1200, 0, 800, 0, 1201, 450, 0, 0, 1])})
        return subprocess.CompletedProcess(command, 0, json.dumps(payload) if inspection else "")
    manifest = BatchPreparer(args, run=run).prepare()
    assert len(manifest["runs"]) == 23
    assert len({r["id"] for r in manifest["runs"]}) == 23
    assert manifest["runs"][0]["camera_fx"] == 1200
    assert manifest["runs"][0]["expected_samples"] == 191
    assert {r["agent"] for r in manifest["runs"][8:]} == {1, 2, 3}
    assert all(Path(r["splat"]).name == "riverside_iowa.ply" for r in manifest["runs"][8:])
    assert all(r["localization_reference_kind"] == "shared_enu_prior" for r in manifest["runs"][8:])
    for run_spec in manifest["runs"][:8]:
        scene = run_spec["scene"]
        assert Path(run_spec["splat"]).name == f"{scene}.ply"
        assert Path(run_spec["nuscenes_localization_transform"]).parent.name == scene
        assert run_spec["localization_reference_kind"] == "recorded_pose_alignment_baseline"
    assert json.loads(args.output.read_text()) == manifest
    from scripts.deployment.run_remote_teleop_batch import ReplayBatch
    assert len(ReplayBatch(args.output, args.output.parent / "runs").runs) == 23
    rsyncs = [c for c in commands if c[0] == "rsync"]
    assert len(rsyncs) == 24  # 3 transforms + 16 map transfers + 5 takes
    assert all("--delete" not in c and "--partial" in c for c in rsyncs)
    assert all(not (":" in c[-1] and ":" in c[-2]) for c in rsyncs)


def test_failure_does_not_publish_manifest(args):
    def run(command, **kwargs):
        raise subprocess.CalledProcessError(255, command)
    with pytest.raises(subprocess.CalledProcessError):
        BatchPreparer(args, run=run).prepare()
    assert not args.output.exists()


@pytest.mark.parametrize("config", [
    {"enu_prior_position_mode": "xy", "splat_path": "data/riverside_iowa.ply"},
    {"enu_prior_position_mode": "xyz", "splat_path": "data/riverside_r3.spz"},
])
def test_rejects_incompatible_cooperscene_alignment_before_network(args, config):
    document = json.loads(args.cooperscene_localization_transform.read_text())
    document["config"] = config
    args.cooperscene_localization_transform.write_text(json.dumps(document))
    def forbidden(*a, **kw):
        raise AssertionError("invalid alignment triggered command")
    with pytest.raises(ValueError):
        BatchPreparer(args, run=forbidden).prepare()
    assert not args.output.exists()


@pytest.mark.parametrize("scene,accepted", [("164", True), ("152", False)])
def test_rejects_wrong_or_unaccepted_nuscenes_alignment(args, scene, accepted):
    def run(command, **kwargs):
        return subprocess.CompletedProcess(command, 0, json.dumps({
            "scene_id": scene, "registration": {"accepted": accepted}}))
    with pytest.raises(ValueError):
        BatchPreparer(args, run=run).nuscenes_localization("152")

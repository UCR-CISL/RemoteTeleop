import json
from pathlib import Path, PurePosixPath
import shlex
import subprocess

import pytest

from scripts.deployment.run_remote_teleop_simulation import (
    LaunchError,
    LaunchOptions,
    RemoteTeleopSimulationLauncher,
)


REPO_ROOT = Path(__file__).resolve().parents[1]


def _options(tmp_path, **overrides):
    values = dict(
        config_path=REPO_ROOT / "cfg/alien3_lambda.yaml",
        ros_sidecar="ros-humble-test",
        remote_python="conda run -n coop3r-slam python",
        mcap_path=PurePosixPath("artifacts/take.mcap"),
        splat_path=PurePosixPath("data/map.spz"),
        output_root=PurePosixPath("artifacts/launch-test"),
        local_artifacts=tmp_path / "received",
        expected_samples=2,
        settle_seconds=0,
        timeout_seconds=1,
        poll_seconds=0.01,
    )
    values.update(overrides)
    return LaunchOptions(**values)


def test_dry_run_builds_ordered_host_specific_commands_without_running(tmp_path):
    printed = []
    launcher = RemoteTeleopSimulationLauncher(
        _options(tmp_path), dry_run=True, output=printed.append
    )

    result = launcher.run()

    assert result == tmp_path / "received"
    assert len(printed) == 12
    assert "test ! -e" in printed[0]
    assert "test ! -e" in printed[1]
    assert "coop3r-slam@100.109.210.56" in printed[2]
    assert "src.realtime.composited_camera_process" in printed[2]
    assert "--frames-endpoint tcp://100.97.168.98:8768" in printed[2]
    assert "--assets-endpoint tcp://100.97.168.98:8769" in printed[2]
    assert "--scene-generation cooperscene:train:1:1" in printed[2]
    assert "--receiver-id alien3-compositor" in printed[2]
    assert "tcp://127.0.0.1:8770" in printed[2]
    assert "justin@100.97.168.98" in printed[3]
    assert "ros-humble-test" in printed[3]
    assert "source /opt/ros/humble/setup.sh" in printed[3]
    assert "--queue-depth 2048 --high-water-mark 2048" in printed[3]
    assert "--assets-endpoint tcp://0.0.0.0:8769" in printed[3]
    assert "--spool-root artifacts/launch-test/lambda/frame-spool" in printed[3]
    assert "--asset-store-root artifacts/launch-test/lambda/asset-store" in printed[3]
    assert "timeout --signal=INT 1s ros2 bag play -s mcap /workspace/artifacts/take.mcap" in printed[4]
    assert "scp -r" in printed[-1]


def test_retrieved_metrics_require_ordered_complete_gap_free_mp4(tmp_path):
    options = _options(tmp_path)
    metrics = options.local_artifacts / "metrics"
    mp4 = options.local_artifacts / "composited" / "composited.mp4"
    metrics.mkdir(parents=True)
    mp4.parent.mkdir(parents=True)
    mp4.write_bytes(b"video")
    frames = mp4.parent / "frames"
    frames.mkdir()
    for index in range(2):
        frames.joinpath(f"{index:06d}_frame-{index}.png").write_bytes(b"png")
    mp4.parent.joinpath("frame-cursor.json").write_text(
        json.dumps({"sequence": 1, "timestamp_us": 100_000}), encoding="utf-8"
    )
    mp4.parent.joinpath("frame-complete.json").write_text(
        json.dumps({"final_sequence": 1, "final_timestamp_us": 100_000}),
        encoding="utf-8",
    )
    metrics.joinpath("compositor.jsonl").write_text(
        "\n".join(json.dumps({
            "event": "compositor_frame",
            "sequence": i,
            "source_timestamp_us": i * 100_000,
            "sequence_gap": 0,
            "timestamp_gap_us": 0,
            "box_count": 1,
            "proxy_count": 1,
            "mesh_count": 0,
            "track_states": {"car": "proxy"},
        }) for i in range(2))
        + "\n"
    )
    vehicle = options.local_artifacts / "lambda"
    vehicle.mkdir()
    vehicle.joinpath("ros_ego_pose_adapter.jsonl").write_text(
        "\n".join(json.dumps({"sequence": i, "source_timestamp_us": i * 100_000}) for i in range(2)) + "\n"
    )

    RemoteTeleopSimulationLauncher(options)._validate_retrieved_metrics()

    metrics.joinpath("compositor.jsonl").write_text(json.dumps({"sequence": 0, "sequence_gap": 1, "timestamp_gap_us": 0}) + "\n")
    with pytest.raises(LaunchError, match="expected 2"):
        RemoteTeleopSimulationLauncher(options)._validate_retrieved_metrics()


class _CountRunner:
    def __init__(self):
        self.commands = []
        self.count = 0

    def run(self, command):
        self.commands.append(tuple(command))
        if "wc -l" in command[-1]:
            self.count += 1
            return subprocess.CompletedProcess(command, 0, stdout=str(self.count * 2), stderr="")
        return subprocess.CompletedProcess(command, 0, stdout="", stderr="")


def test_waits_for_expected_remote_count_without_latest_value_drain(tmp_path):
    runner = _CountRunner()
    launcher = RemoteTeleopSimulationLauncher(
        _options(tmp_path), runner=runner, sleep=lambda _: None,
    )

    launcher._wait_for_remote_metrics()

    assert runner.count == 1
    assert "wc -l" in runner.commands[0][-1]


def test_wait_reports_dead_compositor_even_when_partial_metrics_exist(tmp_path):
    class Runner:
        def run(self, command):
            script = command[-1]
            if "wc -l" in script:
                return subprocess.CompletedProcess(command, 0, stdout="5\n", stderr="")
            return subprocess.CompletedProcess(command, 1, stdout="", stderr="")

    launcher = RemoteTeleopSimulationLauncher(
        _options(tmp_path), runner=Runner(), sleep=lambda _: None
    )

    with pytest.raises(LaunchError, match="compositor exited"):
        launcher._wait_for_remote_metrics()

def test_ssh_keeps_a_script_with_spaces_and_metacharacters_as_one_shell_argument(tmp_path):
    launcher = RemoteTeleopSimulationLauncher(_options(tmp_path))
    script = "cd '/remote path' && echo '$HOME' > 'run log.txt'"

    command = launcher._ssh(launcher.remote, script)

    assert command[:2] == ("ssh", "coop3r-slam@100.109.210.56")
    assert shlex.split(command[2]) == ["sh", "-lc", script]


def test_background_script_only_backgrounds_the_process_after_synchronous_setup(tmp_path):
    launcher = RemoteTeleopSimulationLauncher(_options(tmp_path))

    script = launcher._remote_start_script()

    assert "mkdir -p /home/coop3r-slam/Documents/RemoteTeleop/artifacts/launch-test/logs && { setsid nohup" in script
    assert "2>&1 & echo $! > /home/coop3r-slam/Documents/RemoteTeleop/artifacts/launch-test/compositor.pid; }" in script


def test_defaults_target_verified_ros_sidecar_and_remote_conda(tmp_path):
    launcher = RemoteTeleopSimulationLauncher(_options(
        tmp_path,
        ros_sidecar="remote_teleop_ros_mcap",
        remote_python="/home/coop3r-slam/miniconda3/bin/conda run -n coop3r-slam python",
    ), dry_run=True)

    commands = launcher.planned_commands()

    assert "/home/coop3r-slam/miniconda3/bin/conda" in commands[2][-1]
    assert "remote_teleop_ros_mcap" in commands[3][-1]
    assert "/bin/kill -INT -- -$(cat" in commands[6][-1]


def test_alien4_vehicle_runs_locally_with_configured_artifact_directory(tmp_path):
    options = _options(
        tmp_path,
        config_path=REPO_ROOT / "cfg/alien4_alien3.yaml",
    )
    launcher = RemoteTeleopSimulationLauncher(options, dry_run=True)

    commands = launcher.planned_commands()

    assert commands[1][:2] == ("sh", "-lc")
    assert commands[3][:2] == ("sh", "-lc")
    assert commands[4][:2] == ("sh", "-lc")
    assert "ssh justin@100.101.224.74" not in commands[3][-1]
    assert "artifacts/launch-test/alien4/ros_ego_pose_adapter.jsonl" in commands[3][-1]
    assert "--frames-endpoint tcp://100.101.224.74:8768" in commands[2][-1]
    assert "--assets-endpoint tcp://100.101.224.74:8769" in commands[2][-1]
    assert commands[-1] == ("true",)


def test_vehicle_preflight_requires_ros_sidecar_and_mcap_mount(tmp_path):
    launcher = RemoteTeleopSimulationLauncher(_options(
        tmp_path, config_path=REPO_ROOT / "cfg/alien4_alien3.yaml",
    ))

    command = launcher._vehicle_preflight_script()

    assert "test -f /home/jyue86/Documents/CISL-Projects/RemoteTeleop/artifacts/take.mcap" in command
    assert "docker inspect --format '{{.State.Running}}' ros-humble-test | grep -qx true" in command
    assert "docker exec ros-humble-test test -f /workspace/artifacts/take.mcap" in command
    assert "source /opt/ros/humble/setup.sh && command -v ros2" in command


def test_non_dry_run_refuses_an_existing_local_artifact_destination(tmp_path):
    options = _options(tmp_path)
    options.local_artifacts.mkdir()

    with pytest.raises(LaunchError, match="already exists"):
        RemoteTeleopSimulationLauncher(options).run()

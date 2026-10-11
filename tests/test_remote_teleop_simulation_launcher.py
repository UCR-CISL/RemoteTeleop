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
        remote_python=".venv/bin/python",
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
    assert "test -f /home/coop3r-slam/Documents/RemoteTeleop/data/map.spz" in printed[0]
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


def test_localized_nuscenes_mcap_launcher_passes_the_transform_to_the_adapter(tmp_path):
    options = _options(
        tmp_path, source_mode="nuscenes_mcap", expected_samples=2,
        nuscenes_localization_transform=PurePosixPath("artifacts/nuscenes/152/localization.json"),
    )
    launcher = RemoteTeleopSimulationLauncher(options, dry_run=True, output=lambda _line: None)

    command = launcher._vehicle_adapter_script()
    preflight = launcher._vehicle_preflight_script()

    localization = str(launcher.vehicle.repo_path / options.nuscenes_localization_transform)
    assert f"--localization-transform {localization}" in command
    assert f"test -f {localization}" in preflight


def test_localized_cooperscene_mcap_launcher_passes_both_transforms(tmp_path):
    target = PurePosixPath("artifacts/cooperscene-new/localization.json")
    stored = PurePosixPath("artifacts/cooperscene-old/localization.json")
    options = _options(
        tmp_path,
        cooperscene_localization_transform=target,
        cooperscene_stored_localization_transform=stored,
    )
    launcher = RemoteTeleopSimulationLauncher(options, dry_run=True, output=lambda _line: None)

    command = launcher._vehicle_adapter_script()

    assert f"--localization-transform /workspace/{target}" in command
    assert f"--stored-localization-transform /workspace/{stored}" in command


def test_cooperscene_preserves_localized_object_height_by_default(tmp_path):
    launcher = RemoteTeleopSimulationLauncher(
        _options(tmp_path), dry_run=True, output=lambda _line: None
    )

    command = launcher._remote_start_script()
    assert "--ground-z" not in command
    assert "--preserve-object-z" not in command


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


def test_defaults_target_verified_ros_sidecar_and_remote_venv(tmp_path):
    launcher = RemoteTeleopSimulationLauncher(_options(
        tmp_path,
        ros_sidecar="remote_teleop_ros_mcap",
        remote_python=".venv/bin/python",
    ), dry_run=True)

    commands = launcher.planned_commands()

    assert "/home/coop3r-slam/Documents/RemoteTeleop/.venv/bin/python" in commands[2][-1]
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


def test_nuscenes_source_launches_direct_adapter_and_ply_compositor_without_ros(tmp_path):
    launcher = RemoteTeleopSimulationLauncher(_options(
        tmp_path,
        config_path=REPO_ROOT / "cfg/alien4_alien3.yaml",
        source_mode="nuscenes",
        nuscenes_scene_root=PurePosixPath(
            "thirdparty/bilateral-driving/data/nuscenes/processed_10Hz/trainval/152"
        ),
        splat_path=PurePosixPath(
            "thirdparty/bilateral-driving/ckpts/nuscenes_pretrained_checkpoints_6cams/"
            "152/exports/background_40000.ply"
        ),
        expected_samples=191,
        scene_generation="nuscenes:152",
    ), dry_run=True)

    commands = launcher.planned_commands()
    remote_start = commands[2][-1]
    adapter_start = commands[3][-1]

    assert len(commands) == 11
    assert "--camera-profile nuscenes" in remote_start
    assert "--render-width 480 --render-height 270" in remote_start
    assert "--camera-fx 1266.41720305" in remote_start
    assert "--camera-source-width 1600 --camera-source-height 900" in remote_start
    assert "--ground-z" not in remote_start
    assert "--assets-endpoint" not in remote_start
    assert "src.deployment.nuscenes_frame_adapter" in adapter_start
    assert "--expected-samples 191" in adapter_start
    assert "--frames-per-second 10" in adapter_start
    assert "docker" not in adapter_start
    assert not any("ros2 bag play" in command[-1] for command in commands)
    assert "test -d" in commands[1][-1]
    assert "background_40000.ply" in commands[0][-1]


def test_nuscenes_mcap_uses_direct_log_time_paced_replay(tmp_path):
    launcher = RemoteTeleopSimulationLauncher(_options(
        tmp_path,
        config_path=REPO_ROOT / "cfg/alien4_alien3.yaml",
        source_mode="nuscenes_mcap",
        mcap_path=PurePosixPath("artifacts/nuscenes/scene-152.mcap"),
        splat_path=PurePosixPath("artifacts/nuscenes/scene-152-background.ply"),
        expected_samples=191,
        scene_generation="nuscenes:152",
    ), dry_run=True)

    commands = launcher.planned_commands()
    remote_start = commands[2][-1]
    adapter_start = commands[3][-1]

    assert len(commands) == 11
    assert "--camera-profile nuscenes" in remote_start
    assert "--assets-endpoint" not in remote_start
    assert "src.deployment.ros_frame_adapter" in adapter_start
    assert "--mcap /home/jyue86/Documents/CISL-Projects/RemoteTeleop/artifacts/nuscenes/scene-152.mcap" in adapter_start
    assert "--mcap-rate 1.0" in adapter_start
    assert "--analysis-endpoint tcp://0.0.0.0:8771" in adapter_start
    assert "docker" not in adapter_start
    assert not any("ros2 bag play" in command[-1] for command in commands)
    assert "import mcap, zmq, cv2" in commands[1][-1]


def test_bevfusion_nuscenes_mcap_starts_a_narrowly_mounted_service_before_adapter(tmp_path):
    launcher = RemoteTeleopSimulationLauncher(_options(
        tmp_path,
        config_path=REPO_ROOT / "cfg/alien4_alien3.yaml",
        source_mode="nuscenes_mcap",
        bbox_source="bevfusion",
        mcap_path=PurePosixPath("artifacts/nuscenes/scene-152.mcap"),
        splat_path=PurePosixPath("artifacts/nuscenes/scene-152-background.ply"),
        expected_samples=191,
        scene_generation="nuscenes:152",
    ), dry_run=True)

    commands = launcher.planned_commands()
    service_preflight = commands[2][-1]
    service_start = commands[3][-1]
    readiness = commands[4][-1]
    adapter_start = commands[6][-1]

    assert len(commands) == 15
    assert "docker image inspect remote-teleop-bevfusion:latest" in service_preflight
    assert "docker run -d --rm --name remote-teleop-bevfusion-alien4" in service_start
    assert "--gpus device=0 --network host --ipc host" in service_start
    assert "type=bind,src=/home/jyue86/Documents/CISL-Projects/RemoteTeleop/docker/bevfusion,dst=/workspace,readonly" in service_start
    assert "/home/jyue86/Documents/CISL-Projects/RemoteTeleop:/workspace" not in service_start
    assert "bevfusion_converted.pth" in service_start
    assert "--config /workspace/nuscenes_front.py" in service_start
    assert "EVENT_HANDSHAKE_SUCCEEDED" in readiness
    assert "monitor.poll(1000)" in readiness
    assert "--bbox-source bevfusion" in adapter_start
    assert "--lidar-topic /lidar/points" in adapter_start
    assert "--bevfusion-endpoint tcp://127.0.0.1:8772" in adapter_start
    assert "--bevfusion-score-threshold 0.2" in adapter_start
    assert "docker logs remote-teleop-bevfusion-alien4" in commands[10][-1]
    assert "docker rm -f remote-teleop-bevfusion-alien4" in commands[10][-1]


def test_bevfusion_bbox_source_accepts_cooperscene_mcap(tmp_path):
    assert _options(tmp_path, bbox_source="bevfusion").source_mode == "mcap"


def test_non_dry_run_refuses_an_existing_local_artifact_destination(tmp_path):
    options = _options(tmp_path)
    options.local_artifacts.mkdir()

    with pytest.raises(LaunchError, match="already exists"):
        RemoteTeleopSimulationLauncher(options).run()


def _run_local_bevfusion_readiness_probe(tmp_path, endpoint):
    import sys

    launcher = RemoteTeleopSimulationLauncher(
        _options(tmp_path, bevfusion_endpoint=endpoint), dry_run=True
    )
    script = launcher._bevfusion_ready_script()
    assert script.startswith(launcher._bevfusion_alive_script() + " && ")
    probe = shlex.split(script)[-1]
    return subprocess.run(
        [sys.executable, "-c", probe], capture_output=True, text=True, timeout=5
    )


def test_bevfusion_readiness_rejects_tcp_accept_without_zmq_handshake(tmp_path):
    import socket
    from threading import Event, Thread

    listener = socket.create_server(("127.0.0.1", 0))
    listener.settimeout(5)
    stopped = Event()
    accepted = Event()

    def accept_without_protocol():
        with listener.accept()[0]:
            accepted.set()
            stopped.wait(5)

    thread = Thread(target=accept_without_protocol)
    thread.start()
    try:
        result = _run_local_bevfusion_readiness_probe(
            tmp_path, f"tcp://127.0.0.1:{listener.getsockname()[1]}"
        )
        assert accepted.is_set()
        assert result.returncode == 1
        assert result.stderr == ""
    finally:
        stopped.set()
        listener.close()
        thread.join(timeout=5)
    assert not thread.is_alive()


def test_bevfusion_readiness_accepts_router_without_sending_application_request(tmp_path):
    import zmq

    context = zmq.Context()
    router = context.socket(zmq.ROUTER)
    try:
        port = router.bind_to_random_port("tcp://127.0.0.1")
        result = _run_local_bevfusion_readiness_probe(tmp_path, f"tcp://127.0.0.1:{port}")
        assert result.returncode == 0, result.stderr
        assert not router.poll(0)
    finally:
        router.close(linger=0)
        context.term()

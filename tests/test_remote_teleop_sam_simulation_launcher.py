import json
from pathlib import Path, PurePosixPath
import subprocess

import pytest

from scripts.deployment.run_remote_teleop_sam_simulation import (
    SamLaunchOptions,
    SamRemoteTeleopSimulationLauncher,
)
from scripts.deployment.run_remote_teleop_simulation import (
    LaunchError,
    LaunchOptions,
    RemoteTeleopSimulationLauncher,
)


def _launcher(tmp_path):
    return SamRemoteTeleopSimulationLauncher(
        LaunchOptions(
            config_path=Path("cfg/alien4_alien3.yaml"),
            ros_sidecar="remote_teleop_ros_mcap",
            remote_python="python",
            mcap_path=PurePosixPath("artifacts/input.mcap"),
            splat_path=PurePosixPath("data/map.spz"),
            output_root=PurePosixPath("artifacts/sam-test"),
            local_artifacts=tmp_path / "result",
        ),
        SamLaunchOptions(stable_seconds=2.0),
        dry_run=True,
    )


def test_sam_launcher_keeps_render_and_analysis_endpoints_separate(tmp_path):
    commands = [" ".join(command) for command in _launcher(tmp_path).planned_commands()]
    adapter = next(command for command in commands if "src.deployment.ros_frame_adapter" in command)
    sam3 = next(command for command in commands if "nohup /home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python -m src.realtime.mask_process" in command)
    sam3d = next(command for command in commands if "nohup /home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python -m src.realtime.sam3d_worker" in command)
    compositor = next(command for command in commands if "composited_camera_process" in command)

    assert "--frames-endpoint tcp://0.0.0.0:8768" in adapter
    assert "--analysis-endpoint tcp://0.0.0.0:8771" in adapter
    assert "--analysis-endpoint tcp://127.0.0.1:8771" in sam3
    assert "--analysis-endpoint tcp://127.0.0.1:8771" in sam3d
    assert "--stable-seconds 2.0" in sam3
    assert "--mask-policy new_tracks" in sam3
    assert "--frames-endpoint tcp://100.101.224.74:8768" in compositor
    assert "image" not in compositor


def test_sam_launcher_uses_durable_asset_store_and_completion_marker(tmp_path):
    commands = [" ".join(command) for command in _launcher(tmp_path).planned_commands()]
    sam3d = next(command for command in commands if "nohup /home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python -m src.realtime.sam3d_worker" in command)
    adapter = next(command for command in commands if "src.deployment.ros_frame_adapter" in command)

    assert "--asset-store-root artifacts/sam-test/alien4/asset-store" in sam3d
    assert "--asset-store-root artifacts/sam-test/alien4/asset-store" in adapter
    assert "--completion-path artifacts/sam-test/alien4/sam3d-complete.json" in sam3d
    assert "--gpu-residency-lock artifacts/sam-test/alien4/gpu-residency.lock" in sam3d
    assert "--prewarm" in sam3d
    assert "--keep-cuda-resident" in sam3d
    assert "--ready-path artifacts/sam-test/alien4/sam3d-ready" in sam3d
    assert "--precision nf4" in sam3d
    sam3 = next(command for command in commands if "nohup /home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python -m src.realtime.mask_process" in command)
    assert "--gpu-residency-lock artifacts/sam-test/alien4/gpu-residency.lock" in sam3
    assert "--keep-cuda-resident" in sam3
    assert "--ready-path artifacts/sam-test/alien4/sam3-ready" in sam3
    assert "--precision fp16" in sam3

    sam3d_index = commands.index(sam3d)
    adapter_index = commands.index(adapter)
    sam3_index = commands.index(sam3)
    compositor_index = commands.index(next(command for command in commands if "composited_camera_process" in command))
    sam3d_ready_index = commands.index(next(
        command for command in commands
        if command.startswith("sh -lc test -f ") and command.endswith("sam3d-ready")
    ))
    sam3_ready_index = commands.index(next(
        command for command in commands
        if command.startswith("sh -lc test -f ") and command.endswith("sam3-ready")
    ))
    assert sam3d_index < sam3d_ready_index < sam3_index < sam3_ready_index < compositor_index < adapter_index


def test_sam_validation_allows_no_meshes_after_durable_scene_completion(
    tmp_path, monkeypatch
):
    launcher = _launcher(tmp_path)
    metrics = (
        launcher.options.local_artifacts
        / launcher.vehicle.name
        / "metrics"
    )
    metrics.mkdir(parents=True)
    metrics.joinpath("sam3d.jsonl").write_text(
        json.dumps({
            "event": "sam3d_complete",
            "final_sequence": launcher.options.expected_samples - 1,
        }) + "\n",
        encoding="utf-8",
    )
    monkeypatch.setattr(
        RemoteTeleopSimulationLauncher,
        "_validate_retrieved_metrics",
        lambda self: None,
    )

    launcher._validate_retrieved_metrics()


def test_sam_launcher_uses_direct_single_pass_mcap_adapter(tmp_path):
    commands = [" ".join(command) for command in _launcher(tmp_path).planned_commands()]
    plays = [command for command in commands if "ros2 bag play" in command]
    adapter = next(command for command in commands if "src.deployment.ros_frame_adapter" in command)

    assert not plays
    assert "nohup /home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python -m src.deployment.ros_frame_adapter" in adapter
    assert "--mcap /home/jyue86/Documents/CISL-Projects/RemoteTeleop/artifacts/input.mcap" in adapter
    assert "--mcap-rate 1.0" in adapter
    assert "docker exec" not in adapter
    preflight = next(command for command in commands if "import mcap, torch, zmq, cv2" in command)
    assert "test -f /home/jyue86/Documents/CISL-Projects/RemoteTeleop/artifacts/input.mcap" in preflight
    assert "import mcap, torch, zmq, cv2" in preflight
    assert "docker inspect" not in preflight
    assert all("docker exec" not in command for command in commands)


def test_sam_launcher_preflights_the_remote_pytorch3d_mesh_renderer(tmp_path):
    commands = [" ".join(command) for command in _launcher(tmp_path).planned_commands()]

    preflight = next(command for command in commands if "from pytorch3d.renderer" in command)
    assert "from pytorch3d.renderer import MeshRasterizer, TexturesVertex" in preflight
    assert "from pytorch3d.structures import Meshes" in preflight
    assert "torch.cuda.is_available" in preflight


def test_sam3d_prewarm_failure_stops_worker_before_remote_starts(tmp_path):
    class Runner:
        def __init__(self):
            self.commands = []

        def run(self, command):
            self.commands.append(command)
            script = command[-1]
            if "test -f " in script and "sam3d-ready" in script:
                return subprocess.CompletedProcess(command, 1, "", "")
            if "kill -0" in script:
                return subprocess.CompletedProcess(command, 1, "", "")
            return subprocess.CompletedProcess(command, 0, "", "")

    runner = Runner()
    launcher = SamRemoteTeleopSimulationLauncher(
        _launcher(tmp_path).options,
        SamLaunchOptions(stable_seconds=2.0),
        runner=runner,
        sleep=lambda _: None,
    )

    with pytest.raises(LaunchError, match="sam3d exited before completing startup prewarm"):
        launcher.run()

    commands = [" ".join(command) for command in runner.commands]
    assert any("sam3d_worker" in command for command in commands)
    assert any("sam3d.pid" in command and "/bin/kill -INT" in command for command in commands)
    assert not any("composited_camera_process" in command for command in commands)


def test_sam_launcher_starts_bevfusion_before_vehicle_adapter_for_nuscenes_mcap(tmp_path):
    launcher = SamRemoteTeleopSimulationLauncher(
        LaunchOptions(
            config_path=Path("cfg/alien3_lambda.yaml"),
            ros_sidecar="remote_teleop_ros_mcap",
            remote_python="python",
            mcap_path=PurePosixPath("artifacts/nuscenes/scene-152.mcap"),
            splat_path=PurePosixPath("data/map.spz"),
            output_root=PurePosixPath("artifacts/sam-bevfusion-test"),
            local_artifacts=tmp_path / "result",
            source_mode="nuscenes_mcap",
            bbox_source="bevfusion",
            nuscenes_localization_transform=PurePosixPath(
                "artifacts/nuscenes/152/localization.json"
            ),
            bevfusion_gpu=1,
            expected_samples=2,
        ),
        SamLaunchOptions(stable_seconds=2.0),
        dry_run=True,
    )

    commands = [" ".join(command) for command in launcher.planned_commands()]
    bevfusion = next(command for command in commands if "remote-teleop-bevfusion-lambda" in command and "docker run" in command)
    adapter = next(command for command in commands if "src.deployment.ros_frame_adapter" in command)
    sam3d = next(command for command in commands if "nohup" in command and "src.realtime.sam3d_worker" in command)

    assert commands.index(bevfusion) < commands.index(sam3d) < commands.index(adapter)
    assert "--network host --ipc host" in bevfusion
    assert "--gpus device=1" in bevfusion
    assert "type=bind,src=/home/justin/Documents/CISL-Projects/RemoteTeleop/docker/bevfusion,dst=/workspace,readonly" in bevfusion
    assert "--bbox-source bevfusion" in adapter
    assert "--lidar-topic /lidar/points" in adapter
    assert "--bevfusion-endpoint tcp://127.0.0.1:8772" in adapter
    localization = "/home/justin/Documents/CISL-Projects/RemoteTeleop/artifacts/nuscenes/152/localization.json"
    assert f"--localization-transform {localization}" in adapter
    preflight = next(command for command in commands if "import mcap, torch, zmq, cv2" in command)
    assert "cd /home/justin/Documents/CISL-Projects/RemoteTeleop" in preflight
    assert f"test -f {localization}" in preflight
    assert any("docker rm -f remote-teleop-bevfusion-lambda" in command for command in commands)


def test_asset_proxy_commands_skip_sam_and_asset_delivery(tmp_path):
    launcher = _launcher(tmp_path)
    launcher.sam = SamLaunchOptions(asset_proxy=True)
    commands = [" ".join(command) for command in launcher.planned_commands()]
    combined = "\n".join(commands)
    compositor = next(c for c in commands if "composited_camera_process" in c)
    adapter = next(c for c in commands if "nohup" in c and "ros_frame_adapter" in c)

    assert "--asset-proxy" in compositor
    assert "--assets-endpoint" not in combined
    assert "--asset-store-root" not in combined
    for forbidden in ("sam3d", "mask_process", "gpu-residency", "nvidia-smi"):
        assert forbidden not in combined
    assert "--analysis-spool-root" in adapter
    assert "--analysis-endpoint" in adapter
    assert "--mcap " in adapter
    assert "from pytorch3d.renderer" in combined
    preflight = launcher._sam_vehicle_preflight_script()
    assert "import mcap, numpy, zmq, cv2" in preflight
    assert "torch" not in preflight


def test_asset_proxy_run_keeps_replay_and_render_validation(tmp_path, monkeypatch):
    launcher = _launcher(tmp_path)
    launcher.sam = SamLaunchOptions(asset_proxy=True)
    launcher.dry_run = False
    calls = []
    monkeypatch.setattr(launcher, "_checked", lambda *args: None)
    monkeypatch.setattr(launcher, "sleep", lambda _: None)
    monkeypatch.setattr(launcher, "_require_alive", lambda host, path, label: calls.append(label))
    for name in ("_launch_remote", "_launch_vehicle_adapter", "_wait_for_remote_metrics", "_retrieve_artifacts"):
        monkeypatch.setattr(launcher, name, lambda name=name: calls.append(name))
    monkeypatch.setattr(launcher, "_wait_for_spool_end", lambda spool, label: calls.append(spool))
    monkeypatch.setattr(RemoteTeleopSimulationLauncher, "_validate_retrieved_metrics", lambda self: calls.append("validate"))
    monkeypatch.setattr(RemoteTeleopSimulationLauncher, "_stop_processes", lambda self: calls.append("stop"))
    for name in ("_launch_sam3d_worker", "_wait_for_sam3d_ready", "_launch_sam3_worker", "_wait_for_sam3_ready", "_wait_for_sam3d_completion", "_wait_for_asset_sync", "_sam_stop_script"):
        def unexpected(*args, name=name):
            pytest.fail(f"proxy mode called {name}")
        monkeypatch.setattr(launcher, name, unexpected)
    launcher._started = True

    assert launcher.run() == launcher.options.local_artifacts / "composited" / "composited.mp4"
    assert "frame-spool" in calls and "analysis-spool" in calls
    assert "_wait_for_remote_metrics" in calls
    assert "validate" in calls and "stop" in calls
    assert "SAM3 worker" not in calls and "SAM3D worker" not in calls


def test_asset_proxy_keeps_bevfusion_lifecycle(tmp_path):
    from dataclasses import replace

    launcher = _launcher(tmp_path)
    launcher.sam = SamLaunchOptions(asset_proxy=True)
    launcher.options = replace(launcher.options, bbox_source="bevfusion", source_mode="nuscenes_mcap")
    commands = [" ".join(command) for command in launcher.planned_commands()]
    bevfusion = next(c for c in commands if "docker run" in c)
    adapter = next(c for c in commands if "nohup" in c and "ros_frame_adapter" in c)
    assert commands.index(bevfusion) < commands.index(adapter)
    assert "--bbox-source bevfusion" in adapter
    assert any("docker rm -f remote-teleop-bevfusion" in c for c in commands)


def test_asset_proxy_cli_forwarding(tmp_path, monkeypatch):
    from scripts.deployment import run_remote_teleop_sam_simulation as module

    seen = []
    monkeypatch.setattr(module.SamRemoteTeleopSimulationLauncher, "run", lambda self: seen.append(self.sam.asset_proxy))
    assert module.main(["--asset-proxy", "--dry-run"]) == 0
    assert seen == [True]


def test_foxglove_agent_selection_matches_reader_and_renderer(tmp_path):
    launcher = _launcher(tmp_path)
    launcher.sam = SamLaunchOptions(mcap_agent="3")
    assert "--mcap-agent 3" in launcher._vehicle_adapter_script()
    assert "--agent 3" in launcher._remote_start_script()


def test_localization_metrics_forwarded_and_full_trajectory_required(tmp_path):
    from dataclasses import replace
    launcher = _launcher(tmp_path)
    launcher.options = replace(launcher.options, expected_samples=2,
        localization_reference_transform=PurePosixPath('config/reference.json'),
        localization_reference_kind='shared_enu_prior')
    command = launcher._vehicle_adapter_script()
    assert '--localization-reference-kind shared_enu_prior' in command
    assert '--localization-metrics-path' in command
    root = launcher.options.local_artifacts / launcher.vehicle.name
    root.mkdir(parents=True)
    rows = root / 'localization-trajectory.jsonl'
    rows.write_text('\n'.join(json.dumps(dict(sequence=i, timestamp_us=i*100000)) for i in range(2)))
    summary = root / 'localization-trajectory.summary.json'
    summary.write_text(json.dumps(dict(frame_count=2, trajectory_evaluated=True, position_mode='xyz')))
    launcher._validate_localization_metrics()
    rows.write_text(json.dumps(dict(sequence=0, timestamp_us=0)))
    with pytest.raises(LaunchError, match='full trajectory'):
        launcher._validate_localization_metrics()

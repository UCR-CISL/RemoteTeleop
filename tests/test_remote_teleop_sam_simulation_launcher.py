from pathlib import Path, PurePosixPath

from scripts.deployment.run_remote_teleop_sam_simulation import (
    SamLaunchOptions,
    SamRemoteTeleopSimulationLauncher,
)
from scripts.deployment.run_remote_teleop_simulation import LaunchOptions


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
    adapter = next(command for command in commands if "ros_frame_adapter.py" in command)
    sam3 = next(command for command in commands if "nohup .venv/bin/python -m src.realtime.mask_process" in command)
    sam3d = next(command for command in commands if "nohup .venv/bin/python -m src.realtime.sam3d_worker" in command)
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
    sam3d = next(command for command in commands if "nohup .venv/bin/python -m src.realtime.sam3d_worker" in command)
    adapter = next(command for command in commands if "ros_frame_adapter.py" in command)

    assert "--asset-store-root artifacts/sam-test/alien4/asset-store" in sam3d
    assert "--asset-store-root artifacts/sam-test/alien4/asset-store" in adapter
    assert "--completion-path artifacts/sam-test/alien4/sam3d-complete.json" in sam3d
    assert "--gpu-residency-lock artifacts/sam-test/alien4/gpu-residency.lock" in sam3d
    sam3 = next(command for command in commands if "nohup .venv/bin/python -m src.realtime.mask_process" in command)
    assert "--gpu-residency-lock artifacts/sam-test/alien4/gpu-residency.lock" in sam3


def test_sam_launcher_replays_metadata_then_images_without_competing_ros_traffic(tmp_path):
    commands = [" ".join(command) for command in _launcher(tmp_path).planned_commands()]
    plays = [command for command in commands if "ros2 bag play" in command]

    assert len(plays) == 2
    assert "--topics /camera/frame_detections" in plays[0]
    assert "--rate 1.0" in plays[0]
    assert "--topics /camera/image_raw" in plays[1]
    assert "--rate 0.5" in plays[1]
    assert all("--wait-for-all-acked 0" in command for command in plays)
    assert all("--read-ahead-queue-size 10" in command for command in plays)

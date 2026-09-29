#!/usr/bin/env python3
"""Launch and collect a configured two-host dataset teleoperation replay.

The launcher keeps the vehicle replay adapter and remote GSplat process
independent. It supports legacy ROS/MCAP playback as well as direct processed
nuScenes and log-time-paced nuScenes MCAP sources.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path, PurePosixPath
import shlex
import subprocess
import sys
import time
from typing import Callable, Protocol, Sequence

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.common.config import EndpointConfig, RemoteTeleopConfig


class LaunchError(RuntimeError):
    """Raised when a launched split-system replay does not validate."""


class CommandRunner(Protocol):
    def run(self, command: Sequence[str]) -> subprocess.CompletedProcess[str]: ...


class SubprocessRunner:
    """Small injectable wrapper around non-shell subprocess execution."""

    def run(self, command: Sequence[str]) -> subprocess.CompletedProcess[str]:
        return subprocess.run(list(command), text=True, capture_output=True, check=False)


@dataclass(frozen=True)
class LaunchOptions:
    config_path: Path
    ros_sidecar: str
    remote_python: str
    mcap_path: PurePosixPath
    splat_path: PurePosixPath
    output_root: PurePosixPath
    local_artifacts: Path
    expected_samples: int = 501
    settle_seconds: float = 3.0
    timeout_seconds: float = 180.0
    poll_seconds: float = 1.0
    scene_generation: str = "cooperscene:train:1:1"
    health_port: int = 8770
    source_mode: str = "mcap"
    nuscenes_scene_root: PurePosixPath | None = None
    nuscenes_localization_transform: PurePosixPath | None = None
    cooperscene_localization_transform: PurePosixPath | None = None
    cooperscene_stored_localization_transform: PurePosixPath | None = None
    vehicle_python: str = ".venv/bin/python"
    camera_fx: float = 1266.41720305
    camera_fy: float = 1266.41720305
    camera_cx: float = 816.26701974
    camera_cy: float = 491.50706579
    camera_source_size: tuple[int, int] = (1600, 900)
    bbox_source: str = "gt"
    bevfusion_image: str = "remote-teleop-bevfusion:latest"
    bevfusion_gpu: int = 0
    bevfusion_endpoint: str = "tcp://127.0.0.1:8772"
    bevfusion_runtime_dir: PurePosixPath = PurePosixPath("docker/bevfusion")
    bevfusion_checkpoint: PurePosixPath = PurePosixPath(
        "checkpoints/bevfusion_converted.pth"
    )
    bevfusion_config: PurePosixPath = PurePosixPath("nuscenes_front.py")
    bevfusion_score_threshold: float = 0.2

    def __post_init__(self) -> None:
        if self.expected_samples <= 0:
            raise ValueError("expected_samples must be positive")
        if self.settle_seconds < 0 or self.timeout_seconds <= 0 or self.poll_seconds <= 0:
            raise ValueError("settle/timeout/poll durations must be positive (settle may be zero)")
        if self.source_mode not in ("mcap", "nuscenes", "nuscenes_mcap"):
            raise ValueError(
                "source_mode must be 'mcap', 'nuscenes', or 'nuscenes_mcap'"
            )
        if self.source_mode == "nuscenes" and self.nuscenes_scene_root is None:
            raise ValueError("nuscenes_scene_root is required for the nuscenes source mode")
        if self.nuscenes_localization_transform is not None and self.source_mode != "nuscenes_mcap":
            raise ValueError("nuscenes_localization_transform requires source_mode='nuscenes_mcap'")
        if (self.cooperscene_localization_transform is None) != (
            self.cooperscene_stored_localization_transform is None
        ):
            raise ValueError(
                "CooperScene relocalization requires both target and stored transforms"
            )
        if self.cooperscene_localization_transform is not None and self.source_mode != "mcap":
            raise ValueError("cooperscene_localization_transform requires source_mode='mcap'")
        if any(value <= 0 for value in self.camera_source_size):
            raise ValueError("camera_source_size dimensions must be positive")
        if self.bbox_source not in ("gt", "bevfusion"):
            raise ValueError("bbox_source must be 'gt' or 'bevfusion'")
        if self.bbox_source == "bevfusion" and self.source_mode != "nuscenes_mcap":
            raise ValueError("BEVFusion bbox source requires source_mode='nuscenes_mcap'")
        if not self.bevfusion_image:
            raise ValueError("bevfusion_image must be non-empty")
        if isinstance(self.bevfusion_gpu, bool) or not isinstance(self.bevfusion_gpu, int) or self.bevfusion_gpu < 0:
            raise ValueError("bevfusion_gpu must be a non-negative integer")
        if not self.bevfusion_endpoint.startswith("tcp://127.0.0.1:"):
            raise ValueError("bevfusion_endpoint must be a vehicle-local tcp://127.0.0.1 endpoint")
        if self.bevfusion_runtime_dir.is_absolute() or ".." in self.bevfusion_runtime_dir.parts:
            raise ValueError("bevfusion_runtime_dir must be a relative repository path")
        if self.bevfusion_checkpoint.is_absolute() or ".." in self.bevfusion_checkpoint.parts:
            raise ValueError("bevfusion_checkpoint must be relative to bevfusion_runtime_dir")
        if self.bevfusion_config.is_absolute() or ".." in self.bevfusion_config.parts:
            raise ValueError("bevfusion_config must be relative to bevfusion_runtime_dir")
        if not 0.0 <= self.bevfusion_score_threshold <= 1.0:
            raise ValueError("bevfusion_score_threshold must be between zero and one")


class RemoteTeleopSimulationLauncher:
    """Lifecycle manager for one vehicle publisher and one remote compositor."""

    def __init__(
        self,
        options: LaunchOptions,
        *,
        runner: CommandRunner | None = None,
        sleep: Callable[[float], None] = time.sleep,
        clock: Callable[[], float] = time.monotonic,
        dry_run: bool = False,
        output: Callable[[str], None] = print,
    ) -> None:
        self.options = options
        self.config = RemoteTeleopConfig.from_yaml(options.config_path)
        self.runner = runner or SubprocessRunner()
        self.sleep = sleep
        self.clock = clock
        self.dry_run = dry_run
        self.output = output
        self.remote = self.config.remote
        self.vehicle = self.config.vehicle
        self._started = False
        self._bevfusion_started = False

    def run(self) -> Path:
        """Launch, verify all samples, stop services, and retrieve artifacts."""
        if self.dry_run:
            for command in self.planned_commands():
                self.output(shlex.join(command))
            return self.options.local_artifacts
        if self.options.local_artifacts.exists():
            raise LaunchError(
                f"local artifact destination already exists: {self.options.local_artifacts}; "
                "choose a new --local-artifacts path"
            )
        try:
            self._checked(self._ssh(self.remote, self._remote_preflight_script()), "remote output preflight")
            self._checked(self._ssh(self.vehicle, self._vehicle_preflight_script()), "vehicle output preflight")
            if self._uses_bevfusion:
                self._checked(self._ssh(self.vehicle, self._bevfusion_preflight_script()), "BEVFusion preflight")
                self._launch_bevfusion()
                self._wait_for_bevfusion_ready()
            self._launch_remote()
            self._launch_vehicle_adapter()
            self.sleep(self.options.settle_seconds)
            self._require_alive(self.vehicle, self._vehicle_output_root() / "adapter.pid", "vehicle pose adapter")
            self._require_alive(self.remote, self._remote_output_root() / "compositor.pid", "remote compositor")
            if self.options.source_mode == "mcap":
                self._play_mcap()
            self._wait_for_remote_metrics()
        finally:
            if self._started:
                self._stop_processes()
        self._retrieve_artifacts()
        self._validate_retrieved_metrics()
        return self.options.local_artifacts / "composited" / "composited.mp4"

    def planned_commands(self) -> tuple[tuple[str, ...], ...]:
        """Return the launch order as display-safe argv commands."""
        commands = [
            self._ssh(self.remote, self._remote_preflight_script()),
            self._ssh(self.vehicle, self._vehicle_preflight_script()),
        ]
        if self._uses_bevfusion:
            commands.extend((
                self._ssh(self.vehicle, self._bevfusion_preflight_script()),
                self._ssh(self.vehicle, self._bevfusion_start_script()),
                self._ssh(self.vehicle, self._bevfusion_ready_script()),
            ))
        commands.extend((
            self._ssh(self.remote, self._remote_start_script()),
            self._ssh(self.vehicle, self._vehicle_adapter_script()),
        ))
        if self.options.source_mode == "mcap":
            commands.append(self._ssh(self.vehicle, self._bag_play_script()))
        commands.extend((
            self._ssh(self.remote, self._metrics_count_script()),
            self._ssh(self.remote, self._stop_remote_script()),
            self._ssh(self.vehicle, self._stop_vehicle_script()),
            *(
                (self._ssh(self.vehicle, self._stop_bevfusion_script()),)
                if self._uses_bevfusion else ()
            ),
            self._scp_from(self.remote, self._remote_output_root() / "composited"),
            self._scp_from(self.remote, self._remote_output_root() / "metrics"),
            self._scp_from(self.remote, self._remote_output_root() / "logs"),
            self._scp_from(self.vehicle, self._vehicle_output_root()),
        ))
        return tuple(commands)

    def _launch_remote(self) -> None:
        self._checked(self._ssh(self.remote, self._remote_start_script()), "remote compositor")
        self._started = True

    def _launch_vehicle_adapter(self) -> None:
        self._checked(self._ssh(self.vehicle, self._vehicle_adapter_script()), "vehicle pose adapter")

    @property
    def _uses_bevfusion(self) -> bool:
        return self.options.bbox_source == "bevfusion"

    def _launch_bevfusion(self) -> None:
        self._checked(self._ssh(self.vehicle, self._bevfusion_start_script()), "BEVFusion service")
        # The service owns the vehicle GPU and must be removed if readiness or
        # later launch steps fail, even before the remote compositor starts.
        self._bevfusion_started = True
        self._started = True

    def _wait_for_bevfusion_ready(self) -> None:
        deadline = self.clock() + self.options.timeout_seconds
        while self.clock() < deadline:
            if self.runner.run(self._ssh(self.vehicle, self._bevfusion_ready_script())).returncode == 0:
                return
            if self.runner.run(self._ssh(self.vehicle, self._bevfusion_alive_script())).returncode != 0:
                raise LaunchError("BEVFusion service exited before becoming ready")
            self.sleep(self.options.poll_seconds)
        raise LaunchError("timed out waiting for BEVFusion service readiness")

    def _play_mcap(self) -> None:
        self._checked(self._ssh(self.vehicle, self._bag_play_script()), "MCAP playback")

    def _wait_for_remote_metrics(self) -> None:
        deadline = self.clock() + self.options.timeout_seconds
        last_count = 0
        while self.clock() < deadline:
            result = self.runner.run(self._ssh(self.remote, self._metrics_count_script()))
            if result.returncode == 0:
                try:
                    last_count = int(result.stdout.strip() or "0")
                except ValueError as error:
                    raise LaunchError(f"invalid remote metric count: {result.stdout!r}") from error
                completed = self.runner.run(
                    self._ssh(self.remote, self._completion_script())
                )
                if completed.returncode == 0:
                    return
            if not self._is_alive(
                self.remote, self._remote_output_root() / "compositor.pid"
            ):
                raise LaunchError("remote compositor exited before producing all pose metrics")
            if self._uses_bevfusion and self.runner.run(
                self._ssh(self.vehicle, self._bevfusion_alive_script())
            ).returncode != 0:
                raise LaunchError("BEVFusion service exited during playback")
            self.sleep(self.options.poll_seconds)
        raise LaunchError(
            f"remote compositor timed out after {self.options.timeout_seconds:g}s: "
            f"received {last_count}/{self.options.expected_samples} pose samples"
        )

    def _retrieve_artifacts(self) -> None:
        self.options.local_artifacts.mkdir(parents=True, exist_ok=True)
        for path in ("composited", "metrics", "logs"):
            self._checked(self._scp_from(self.remote, self._remote_output_root() / path), f"remote {path}")
        self._checked(self._scp_from(self.vehicle, self._vehicle_output_root()), "vehicle artifacts")

    def _validate_retrieved_metrics(self) -> None:
        metrics = self.options.local_artifacts / "metrics" / "compositor.jsonl"
        if not metrics.is_file():
            raise LaunchError(f"remote compositor metrics were not retrieved: {metrics}")
        all_records = [
            json.loads(line)
            for line in metrics.read_text(encoding="utf-8").splitlines()
            if line
        ]
        records = [record for record in all_records if record.get("event") == "compositor_frame"]
        if len(records) != self.options.expected_samples:
            raise LaunchError(f"expected {self.options.expected_samples} compositor metrics, got {len(records)}")
        for index, record in enumerate(records):
            if record.get("sequence") != index:
                raise LaunchError(f"unexpected compositor sequence at record {index}: {record.get('sequence')!r}")
            if record.get("source_timestamp_us") != index * 100_000:
                raise LaunchError(f"unexpected compositor timestamp at record {index}: {record}")
            if record.get("sequence_gap") != 0 or record.get("timestamp_gap_us") != 0:
                raise LaunchError(f"pose gap reported at sequence {index}: {record}")
            box_count = record.get("box_count")
            proxy_count = record.get("proxy_count")
            mesh_count = record.get("mesh_count")
            track_states = record.get("track_states")
            if (
                not isinstance(box_count, int)
                or not isinstance(proxy_count, int)
                or not isinstance(mesh_count, int)
                or not isinstance(track_states, dict)
                or box_count != proxy_count + mesh_count
                or box_count != len(track_states)
            ):
                raise LaunchError(f"inconsistent bbox render accounting at sequence {index}: {record}")
        if not any(record["box_count"] > 0 for record in records):
            raise LaunchError("remote compositor did not render any detected objects")
        frames = sorted((self.options.local_artifacts / "composited" / "frames").glob("*.png"))
        if len(frames) != self.options.expected_samples:
            raise LaunchError(
                f"expected {self.options.expected_samples} durable PNG frames, got {len(frames)}"
            )
        cursor_path = self.options.local_artifacts / "composited" / "frame-cursor.json"
        if not cursor_path.is_file():
            raise LaunchError("remote durable frame cursor was not retrieved")
        cursor = json.loads(cursor_path.read_text(encoding="utf-8"))
        if (
            cursor.get("sequence") != self.options.expected_samples - 1
            or cursor.get("timestamp_us") != (self.options.expected_samples - 1) * 100_000
        ):
            raise LaunchError(f"remote frame cursor is incomplete: {cursor}")
        complete_path = self.options.local_artifacts / "composited" / "frame-complete.json"
        if not complete_path.is_file():
            raise LaunchError("remote frame completion marker was not retrieved")
        complete = json.loads(complete_path.read_text(encoding="utf-8"))
        if (
            complete.get("final_sequence") != self.options.expected_samples - 1
            or complete.get("final_timestamp_us")
            != (self.options.expected_samples - 1) * 100_000
        ):
            raise LaunchError(f"remote frame completion marker is invalid: {complete}")
        mp4 = self.options.local_artifacts / "composited" / "composited.mp4"
        if not mp4.is_file() or mp4.stat().st_size == 0:
            raise LaunchError(f"remote compositor MP4 was not retrieved: {mp4}")
        vehicle_metrics = self.options.local_artifacts / self.vehicle.name / self._adapter_metrics_name()
        if not vehicle_metrics.is_file():
            raise LaunchError(f"vehicle adapter metrics were not retrieved: {vehicle_metrics}")
        vehicle_records = [
            json.loads(line)
            for line in vehicle_metrics.read_text(encoding="utf-8").splitlines()
            if line
        ]
        if self.options.source_mode == "nuscenes":
            vehicle_records = [
                record for record in vehicle_records
                if record.get("event") == "frame_spooled"
            ]
        elif self.options.source_mode == "nuscenes_mcap":
            vehicle_records = [
                record for record in vehicle_records
                if record.get("event") == "ros_frame_snapshot_spooled"
            ]
        if len(vehicle_records) != self.options.expected_samples:
            raise LaunchError(f"expected {self.options.expected_samples} vehicle metrics, got {len(vehicle_records)}")
        for index, record in enumerate(vehicle_records):
            if record.get("sequence") != index or record.get("source_timestamp_us") != index * 100_000:
                raise LaunchError(f"unexpected vehicle pose metric at record {index}: {record}")
        if self._uses_bevfusion:
            detector_metrics = (
                self.options.local_artifacts / self.vehicle.name
                / "metrics" / "bevfusion.jsonl"
            )
            if not detector_metrics.is_file():
                raise LaunchError("BEVFusion metrics were not retrieved")
            detector_records = [
                json.loads(line) for line in detector_metrics.read_text(encoding="utf-8").splitlines()
                if line
            ]
            results = [
                record for record in detector_records
                if record.get("event") == "bevfusion_result"
            ]
            if not results:
                raise LaunchError("BEVFusion produced no inference results")
            if any(record.get("error") for record in results):
                raise LaunchError("BEVFusion reported an inference error")

    def _remote_start_script(self) -> str:
        root = self._remote_output_root()
        command = self._remote_python_command() + [
            "-m", "src.realtime.composited_camera_process",
            "--frames-endpoint", self.vehicle.endpoint_uri,
            "--scene-generation", self.options.scene_generation,
            "--receiver-id", f"{self.remote.name}-compositor",
            "--health-endpoint", f"tcp://127.0.0.1:{self.options.health_port}",
            "--splat", str(self.options.splat_path), "--output-dir", str(root / "composited"),
            "--metrics-path", str(root / "metrics" / "compositor.jsonl"),
        ]
        if self.options.source_mode == "mcap":
            command.extend([
                "--assets-endpoint", self.vehicle.asset_endpoint_uri,
                "--agent", "1", "--render-width", "480", "--render-height", "300",
            ])
        else:
            source_width, source_height = self.options.camera_source_size
            command.extend([
                "--camera-profile", "nuscenes",
                "--camera-fx", str(self.options.camera_fx),
                "--camera-fy", str(self.options.camera_fy),
                "--camera-cx", str(self.options.camera_cx),
                "--camera-cy", str(self.options.camera_cy),
                "--camera-source-width", str(source_width),
                "--camera-source-height", str(source_height),
                "--ego-t-camera",
                "1", "0", "0", "0", "0", "1", "0", "0",
                "0", "0", "1", "0", "0", "0", "0", "1",
                "--render-width", "480", "--render-height", "270",
            ])
        return self._background_script(self.remote.repo_path, command, root / "logs" / "compositor.log", root / "compositor.pid")

    def _vehicle_adapter_script(self) -> str:
        root = self._vehicle_output_root()
        if self.options.source_mode == "nuscenes":
            assert self.options.nuscenes_scene_root is not None
            command = self._vehicle_python_command() + [
                "-m", "src.deployment.nuscenes_frame_adapter",
                "--scene-root", str(self.vehicle.repo_path / self.options.nuscenes_scene_root),
                "--frames-endpoint", "tcp://0.0.0.0:" + str(self.vehicle.port),
                "--expected-samples", str(self.options.expected_samples),
                "--scene-generation", self.options.scene_generation,
                "--metrics-path", str(root / self._adapter_metrics_name()),
                "--spool-root", str(root / "frame-spool"),
                "--frames-per-second", "10",
            ]
            return self._background_script(
                self.vehicle.repo_path, command, root / "logs" / "adapter.log", root / "adapter.pid"
            )
        if self.options.source_mode == "nuscenes_mcap":
            if self.vehicle.analysis_port is None:
                raise LaunchError("nuscenes_mcap source requires vehicle analysis_port")
            command = self._vehicle_python_command() + [
                "-m", "src.deployment.ros_frame_adapter",
                "--mcap", str(self.vehicle.repo_path / self.options.mcap_path),
                "--mcap-rate", "1.0",
                "--frames-endpoint", "tcp://0.0.0.0:" + str(self.vehicle.port),
                "--analysis-endpoint", "tcp://0.0.0.0:" + str(self.vehicle.analysis_port),
                "--queue-depth", "2048", "--high-water-mark", "2048",
                "--expected-samples", str(self.options.expected_samples),
                "--scene-generation", self.options.scene_generation,
                "--metrics-path", str(root / self._adapter_metrics_name()),
                "--spool-root", str(root / "frame-spool"),
                "--analysis-spool-root", str(root / "analysis-spool"),
            ]
            if self._uses_bevfusion:
                command.extend([
                    "--bbox-source", "bevfusion",
                    "--lidar-topic", "/lidar/points",
                    "--bevfusion-endpoint", self.options.bevfusion_endpoint,
                    "--bevfusion-score-threshold", str(self.options.bevfusion_score_threshold),
                ])
            if self.options.nuscenes_localization_transform is not None:
                command.extend([
                    "--localization-transform",
                    str(self.vehicle.repo_path / self.options.nuscenes_localization_transform),
                ])
            if self.options.cooperscene_localization_transform is not None:
                command.extend([
                    "--localization-transform",
                    str(self.vehicle.repo_path / self.options.cooperscene_localization_transform),
                    "--stored-localization-transform",
                    str(
                        self.vehicle.repo_path
                        / self.options.cooperscene_stored_localization_transform
                    ),
                ])
            return self._background_script(
                self.vehicle.repo_path, command, root / "logs" / "adapter.log", root / "adapter.pid"
            )
        inner = (
            "source /opt/ros/humble/setup.sh && cd /workspace && PYTHONPATH=/workspace${PYTHONPATH:+:$PYTHONPATH} "
            + shlex.join([
                "/usr/bin/python3", "src/deployment/ros_frame_adapter.py",
                "--frames-endpoint", "tcp://0.0.0.0:" + str(self.vehicle.port),
                "--assets-endpoint", "tcp://0.0.0.0:" + str(self.vehicle.asset_port),
                "--queue-depth", "2048", "--high-water-mark", "2048",
                "--expected-samples", str(self.options.expected_samples),
                "--scene-generation", self.options.scene_generation,
                "--metrics-path", str(self.options.output_root / self.vehicle.name / "ros_ego_pose_adapter.jsonl"),
                "--spool-root", str(self.options.output_root / self.vehicle.name / "frame-spool"),
                "--asset-store-root", str(self.options.output_root / self.vehicle.name / "asset-store"),
                *(
                    [
                        "--localization-transform",
                        str(PurePosixPath("/workspace") / self.options.cooperscene_localization_transform),
                        "--stored-localization-transform",
                        str(
                            PurePosixPath("/workspace")
                            / self.options.cooperscene_stored_localization_transform
                        ),
                    ]
                    if self.options.cooperscene_localization_transform is not None
                    else []
                ),
            ])
        )
        command = ["docker", "exec", self.options.ros_sidecar, "bash", "-lc", inner]
        return self._background_script(self.vehicle.repo_path, command, root / "logs" / "adapter.log", root / "adapter.pid")

    def _bag_play_script(self) -> str:
        in_container_mcap = PurePosixPath("/workspace") / self.options.mcap_path
        timeout_seconds = max(1, math.ceil(self.options.timeout_seconds))
        inner = (
            "source /opt/ros/humble/setup.sh && timeout --signal=INT "
            + f"{timeout_seconds}s ros2 bag play -s mcap " + shlex.quote(str(in_container_mcap))
        )
        command = ["docker", "exec", self.options.ros_sidecar, "bash", "-lc", inner]
        return "cd " + shlex.quote(str(self.vehicle.repo_path)) + " && " + shlex.join(command)

    def _remote_preflight_script(self) -> str:
        return " && ".join((
            "test ! -e " + shlex.quote(str(self._remote_output_root())),
            "test -f " + shlex.quote(
                str(self.remote.repo_path / self.options.splat_path)
            ),
        ))

    def _vehicle_preflight_script(self) -> str:
        if self.options.source_mode == "nuscenes":
            assert self.options.nuscenes_scene_root is not None
            return " && ".join((
                "test ! -e " + shlex.quote(str(self.vehicle.repo_path / self.options.output_root)),
                "test -d " + shlex.quote(
                    str(self.vehicle.repo_path / self.options.nuscenes_scene_root)
                ),
            ))
        if self.options.source_mode == "nuscenes_mcap":
            python = shlex.join(self._vehicle_python_command())
            return " && ".join((
                "test ! -e " + shlex.quote(str(self.vehicle.repo_path / self.options.output_root)),
                "test -f " + shlex.quote(str(self.vehicle.repo_path / self.options.mcap_path)),
                *(
                    ("test -f " + shlex.quote(
                        str(self.vehicle.repo_path / self.options.nuscenes_localization_transform)
                    ),)
                    if self.options.nuscenes_localization_transform is not None else ()
                ),
                python + " -c " + shlex.quote("import mcap, zmq, cv2"),
            ))
        host_mcap = self.vehicle.repo_path / self.options.mcap_path
        container_mcap = PurePosixPath("/workspace") / self.options.mcap_path
        sidecar = shlex.quote(self.options.ros_sidecar)
        return " && ".join((
            "test ! -e " + shlex.quote(str(self.vehicle.repo_path / self.options.output_root)),
            "test -f " + shlex.quote(str(host_mcap)),
            *(
                (
                    "test -f " + shlex.quote(str(
                        self.vehicle.repo_path
                        / self.options.cooperscene_localization_transform
                    )),
                    "test -f " + shlex.quote(str(
                        self.vehicle.repo_path
                        / self.options.cooperscene_stored_localization_transform
                    )),
                )
                if self.options.cooperscene_localization_transform is not None else ()
            ),
            "docker inspect --format '{{.State.Running}}' " + sidecar + " | grep -qx true",
            "docker exec " + sidecar + " test -f " + shlex.quote(str(container_mcap)),
            "docker exec " + sidecar + " bash -lc " + shlex.quote(
                "source /opt/ros/humble/setup.sh && command -v ros2"
            ),
        ))

    def _bevfusion_runtime_root(self) -> PurePosixPath:
        return self.vehicle.repo_path / self.options.bevfusion_runtime_dir

    def _bevfusion_container_name(self) -> str:
        # Docker names cannot contain the colon used by scene generations.
        return f"remote-teleop-bevfusion-{self.vehicle.name}"

    def _bevfusion_preflight_script(self) -> str:
        runtime = self._bevfusion_runtime_root()
        checkpoint = runtime / self.options.bevfusion_checkpoint
        config = runtime / self.options.bevfusion_config
        return " && ".join((
            "test -d " + shlex.quote(str(runtime)),
            "test -f " + shlex.quote(str(runtime / "bevfusion_server.py")),
            "test -f " + shlex.quote(str(checkpoint)),
            "test -f " + shlex.quote(str(config)),
            "docker image inspect " + shlex.quote(self.options.bevfusion_image) + " >/dev/null",
            "! docker container inspect " + shlex.quote(self._bevfusion_container_name()) + " >/dev/null 2>&1",
        ))

    def _bevfusion_start_script(self) -> str:
        root = self._vehicle_output_root()
        runtime = self._bevfusion_runtime_root()
        container = self._bevfusion_container_name()
        command = [
            "docker", "run", "-d", "--rm", "--name", container,
            "--gpus", f"device={self.options.bevfusion_gpu}", "--network", "host", "--ipc", "host",
            "--mount", f"type=bind,src={runtime},dst=/workspace,readonly",
            self.options.bevfusion_image,
            "python", "/workspace/bevfusion_server.py",
            "--endpoint", self.options.bevfusion_endpoint,
            "--config", "/workspace/" + str(self.options.bevfusion_config),
            "--checkpoint", "/workspace/" + str(self.options.bevfusion_checkpoint),
            "--score-threshold", str(self.options.bevfusion_score_threshold),
            "--metrics-path", "/dev/stdout",
        ]
        # Keep a durable vehicle-side launch record without granting the
        # container write access to the repository or output directories.
        return " && ".join((
            "mkdir -p " + shlex.quote(str(root / "logs")),
            shlex.join(command) + " >> " + shlex.quote(str(root / "logs" / "bevfusion.log")) + " 2>&1",
        ))

    def _bevfusion_alive_script(self) -> str:
        return (
            "docker inspect --format '{{.State.Running}}' "
            + shlex.quote(self._bevfusion_container_name())
            + " | grep -qx true"
        )

    def _bevfusion_ready_script(self) -> str:
        endpoint = self.options.bevfusion_endpoint.removeprefix("tcp://")
        host, port_text = endpoint.rsplit(":", 1)
        probe = (
            "import socket; "
            f"socket.create_connection(({host!r}, {int(port_text)}), timeout=1).close()"
        )
        return self._bevfusion_alive_script() + " && " + shlex.join(
            self._vehicle_python_command() + ["-c", probe]
        )

    def _stop_bevfusion_script(self) -> str:
        container = shlex.quote(self._bevfusion_container_name())
        metrics = shlex.quote(str(self._vehicle_output_root() / "metrics" / "bevfusion.jsonl"))
        log = shlex.quote(str(self._vehicle_output_root() / "logs" / "bevfusion-container.log"))
        return " && ".join((
            "mkdir -p " + shlex.quote(str(self._vehicle_output_root() / "metrics"))
            + " " + shlex.quote(str(self._vehicle_output_root() / "logs")),
            "docker logs " + container + " > " + log + " 2>&1 || true",
            "sed -n '/^{/p' " + log + " > " + metrics,
            "docker rm -f " + container + " >/dev/null 2>&1 || true",
        ))

    def _metrics_count_script(self) -> str:
        path = self._remote_output_root() / "metrics" / "compositor.jsonl"
        return "test -f " + shlex.quote(str(path)) + " && wc -l < " + shlex.quote(str(path))

    def _completion_script(self) -> str:
        path = self._remote_output_root() / "composited" / "frame-complete.json"
        return "test -f " + shlex.quote(str(path))

    def _stop_remote_script(self) -> str:
        return self._stop_script(self._remote_output_root() / "compositor.pid", process_group=True)

    def _stop_vehicle_script(self) -> str:
        if self.options.source_mode in ("nuscenes", "nuscenes_mcap"):
            return self._stop_script(self._vehicle_output_root() / "adapter.pid", process_group=True)
        return " && ".join((
            self._stop_script(self._vehicle_output_root() / "adapter.pid"),
            "docker exec " + shlex.quote(self.options.ros_sidecar)
            + " pkill -INT -f " + shlex.quote("src/deployment/ros_frame_adapter.py")
            + " 2>/dev/null || true",
        ))

    @staticmethod
    def _stop_script(pid_path: PurePosixPath, *, process_group: bool = False) -> str:
        quoted = shlex.quote(str(pid_path))
        target = f"-$(cat {quoted})" if process_group else f"$(cat {quoted})"
        return f"if test -f {quoted}; then /bin/kill -INT -- {target} 2>/dev/null || true; fi"

    @staticmethod
    def _background_script(repo: PurePosixPath, command: list[str], log: PurePosixPath, pid: PurePosixPath) -> str:
        # The braces make the setup commands synchronous.  Without them the
        # trailing ``&`` backgrounds the entire ``cd && mkdir && setsid``
        # chain, allowing the PID write to race the directory creation.
        return " && ".join((
            "cd " + shlex.quote(str(repo)),
            "mkdir -p " + shlex.quote(str(log.parent)),
            "{ setsid nohup " + shlex.join(command) + " > " + shlex.quote(str(log))
            + " 2>&1 & echo $! > " + shlex.quote(str(pid)) + "; }",
        ))

    def _remote_python_command(self) -> list[str]:
        command = shlex.split(self.options.remote_python)
        if not command:
            raise LaunchError("--remote-python must not be empty")
        if not PurePosixPath(command[0]).is_absolute():
            command[0] = str(PurePosixPath(self.remote.repo_path) / command[0])
        return command

    def _vehicle_python_command(self) -> list[str]:
        command = shlex.split(self.options.vehicle_python)
        if not command:
            raise LaunchError("--vehicle-python must not be empty")
        if not PurePosixPath(command[0]).is_absolute():
            command[0] = str(PurePosixPath(self.vehicle.repo_path) / command[0])
        return command

    def _adapter_metrics_name(self) -> str:
        return (
            "nuscenes_frame_adapter.jsonl"
            if self.options.source_mode == "nuscenes"
            else "ros_ego_pose_adapter.jsonl"
        )

    def _remote_output_root(self) -> PurePosixPath:
        return self.remote.repo_path / self.options.output_root

    def _vehicle_output_root(self) -> PurePosixPath:
        return self.vehicle.repo_path / self.options.output_root / self.vehicle.name

    @staticmethod
    def _ssh(endpoint: EndpointConfig, script: str) -> tuple[str, ...]:
        # OpenSSH concatenates arguments following the target into one remote
        # command.  Keep the complete shell invocation in one argument so the
        # script remains the single argument consumed by ``sh -lc``.
        if endpoint.is_local:
            return ("sh", "-lc", script)
        return ("ssh", f"{endpoint.user}@{endpoint.address}", "sh -lc " + shlex.quote(script))

    def _scp_from(self, endpoint: EndpointConfig, source: PurePosixPath) -> tuple[str, ...]:
        if endpoint.is_local:
            # Local vehicle artifacts are already rooted under local_artifacts.
            return ("true",)
        return ("scp", "-r", f"{endpoint.user}@{endpoint.address}:{source}", str(self.options.local_artifacts))

    def _stop_processes(self) -> None:
        result = self.runner.run(self._ssh(self.remote, self._stop_remote_script()))
        if result.returncode != 0:
            self.output(f"warning: could not stop remote process: {result.stderr.strip()}")
        elif not self._wait_for_exit(self.remote, self._remote_output_root() / "compositor.pid", timeout_seconds=10.0):
            self.output("warning: remote compositor did not exit within 10s; forcing shutdown")
            self._force_stop_process_group(
                self.remote, self._remote_output_root() / "compositor.pid"
            )
        result = self.runner.run(self._ssh(self.vehicle, self._stop_vehicle_script()))
        if result.returncode != 0:
            self.output(f"warning: could not stop vehicle process: {result.stderr.strip()}")
        elif not self._wait_for_exit(
            self.vehicle,
            self._vehicle_output_root() / "adapter.pid",
            timeout_seconds=10.0,
        ):
            self.output("warning: vehicle adapter did not exit within 10s; forcing shutdown")
            self._force_stop_process_group(
                self.vehicle, self._vehicle_output_root() / "adapter.pid"
            )
        if self._bevfusion_started:
            result = self.runner.run(self._ssh(self.vehicle, self._stop_bevfusion_script()))
            if result.returncode != 0:
                self.output(f"warning: could not stop BEVFusion service: {result.stderr.strip()}")

    def _force_stop_process_group(
        self, endpoint: EndpointConfig, pid_path: PurePosixPath
    ) -> None:
        for signal, timeout_seconds in (("TERM", 5.0), ("KILL", 2.0)):
            quoted = shlex.quote(str(pid_path))
            script = (
                f"if test -f {quoted}; then /bin/kill -{signal} -- "
                f'"-$(cat {quoted})" 2>/dev/null || true; fi'
            )
            self.runner.run(self._ssh(endpoint, script))
            if self._wait_for_exit(
                endpoint, pid_path, timeout_seconds=timeout_seconds
            ):
                return

    def _wait_for_exit(self, endpoint: EndpointConfig, pid_path: PurePosixPath, *, timeout_seconds: float) -> bool:
        deadline = self.clock() + timeout_seconds
        while self.clock() < deadline:
            if not self._is_alive(endpoint, pid_path):
                return True
            self.sleep(min(self.options.poll_seconds, 0.25))
        return not self._is_alive(endpoint, pid_path)

    def _require_alive(self, endpoint: EndpointConfig, pid_path: PurePosixPath, label: str) -> None:
        if not self._is_alive(endpoint, pid_path):
            raise LaunchError(f"{label} exited during startup; inspect its retrieved log")

    def _is_alive(self, endpoint: EndpointConfig, pid_path: PurePosixPath) -> bool:
        quoted = shlex.quote(str(pid_path))
        result = self.runner.run(self._ssh(endpoint, f"test -f {quoted} && kill -0 \"$(cat {quoted})\""))
        return result.returncode == 0

    def _checked(self, command: Sequence[str], label: str) -> subprocess.CompletedProcess[str]:
        result = self.runner.run(command)
        if result.returncode != 0:
            raise LaunchError(f"{label} failed ({shlex.join(command)}): {result.stderr.strip()}")
        return result


def _path(value: str) -> PurePosixPath:
    path = PurePosixPath(value)
    if path.is_absolute() or ".." in path.parts:
        raise argparse.ArgumentTypeError("must be a relative remote repository path")
    return path


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("cfg/alien4_alien3.yaml"))
    parser.add_argument("--ros-sidecar", default="remote_teleop_ros_mcap")
    parser.add_argument("--remote-python", default=".venv/bin/python")
    parser.add_argument("--vehicle-python", default=".venv/bin/python")
    parser.add_argument(
        "--source-mode", choices=("mcap", "nuscenes", "nuscenes_mcap"), default="mcap"
    )
    parser.add_argument("--mcap", type=_path, default=PurePosixPath("artifacts/cooperscene_take_1_agent_1.mcap"))
    parser.add_argument("--nuscenes-scene-root", type=_path)
    parser.add_argument("--nuscenes-localization-transform", type=_path)
    parser.add_argument("--cooperscene-localization-transform", type=_path)
    parser.add_argument("--cooperscene-stored-localization-transform", type=_path)
    parser.add_argument("--splat", type=_path, default=PurePosixPath("data/riverside_r3.spz"))
    parser.add_argument("--output-root", type=_path, default=PurePosixPath("artifacts/split_system_simulation"))
    parser.add_argument("--local-artifacts", type=Path, default=Path("artifacts/split_system_simulation"))
    parser.add_argument("--expected-samples", type=int)
    parser.add_argument("--settle-seconds", type=float, default=3.0)
    parser.add_argument("--timeout-seconds", type=float, default=180.0)
    parser.add_argument("--poll-seconds", type=float, default=1.0)
    parser.add_argument("--scene-generation", default="cooperscene:train:1:1")
    parser.add_argument("--health-port", type=int, default=8770)
    parser.add_argument("--camera-fx", type=float, default=1266.41720305)
    parser.add_argument("--camera-fy", type=float, default=1266.41720305)
    parser.add_argument("--camera-cx", type=float, default=816.26701974)
    parser.add_argument("--camera-cy", type=float, default=491.50706579)
    parser.add_argument("--camera-source-width", type=int, default=1600)
    parser.add_argument("--camera-source-height", type=int, default=900)
    parser.add_argument("--bbox-source", choices=("gt", "bevfusion"), default="gt")
    parser.add_argument("--bevfusion-image", default="remote-teleop-bevfusion:latest")
    parser.add_argument("--bevfusion-gpu", type=int, default=0)
    parser.add_argument("--bevfusion-endpoint", default="tcp://127.0.0.1:8772")
    parser.add_argument("--bevfusion-runtime-dir", type=_path, default=PurePosixPath("docker/bevfusion"))
    parser.add_argument(
        "--bevfusion-checkpoint", type=_path,
        default=PurePosixPath("checkpoints/bevfusion_converted.pth"),
    )
    parser.add_argument(
        "--bevfusion-config", type=_path,
        default=PurePosixPath("nuscenes_front.py"),
    )
    parser.add_argument("--bevfusion-score-threshold", type=float, default=0.2)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    if args.source_mode in ("nuscenes", "nuscenes_mcap") and args.expected_samples is None:
        parser.error(f"{args.source_mode} source requires --expected-samples")
    try:
        launcher = RemoteTeleopSimulationLauncher(LaunchOptions(
            config_path=args.config, ros_sidecar=args.ros_sidecar,
            remote_python=args.remote_python, vehicle_python=args.vehicle_python,
            mcap_path=args.mcap, splat_path=args.splat, output_root=args.output_root,
            local_artifacts=args.local_artifacts,
            expected_samples=501 if args.expected_samples is None else args.expected_samples,
            settle_seconds=args.settle_seconds, timeout_seconds=args.timeout_seconds,
            poll_seconds=args.poll_seconds, scene_generation=args.scene_generation, health_port=args.health_port,
            source_mode=args.source_mode, nuscenes_scene_root=args.nuscenes_scene_root,
            nuscenes_localization_transform=args.nuscenes_localization_transform,
            cooperscene_localization_transform=args.cooperscene_localization_transform,
            cooperscene_stored_localization_transform=(
                args.cooperscene_stored_localization_transform
            ),
            camera_fx=args.camera_fx, camera_fy=args.camera_fy,
            camera_cx=args.camera_cx, camera_cy=args.camera_cy,
            camera_source_size=(args.camera_source_width, args.camera_source_height),
            bbox_source=args.bbox_source, bevfusion_image=args.bevfusion_image,
            bevfusion_gpu=args.bevfusion_gpu,
            bevfusion_endpoint=args.bevfusion_endpoint,
            bevfusion_runtime_dir=args.bevfusion_runtime_dir,
            bevfusion_checkpoint=args.bevfusion_checkpoint,
            bevfusion_config=args.bevfusion_config,
            bevfusion_score_threshold=args.bevfusion_score_threshold,
        ), dry_run=args.dry_run)
        result = launcher.run()
    except (LaunchError, ValueError) as error:
        parser.error(str(error))
    if not args.dry_run:
        print(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

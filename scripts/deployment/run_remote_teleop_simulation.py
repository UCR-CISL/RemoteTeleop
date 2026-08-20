#!/usr/bin/env python3
"""Launch and collect a configured two-host CooperScene teleoperation replay.

The launcher deliberately keeps the vehicle ROS sidecar and the remote GSplat
process independent.  It only coordinates their lifecycle over SSH, so the
same script can be used with another two-host ``RemoteTeleopConfig`` YAML.
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

    def __post_init__(self) -> None:
        if self.expected_samples <= 0:
            raise ValueError("expected_samples must be positive")
        if self.settle_seconds < 0 or self.timeout_seconds <= 0 or self.poll_seconds <= 0:
            raise ValueError("settle/timeout/poll durations must be positive (settle may be zero)")


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
            self._launch_remote()
            self._launch_vehicle_adapter()
            self.sleep(self.options.settle_seconds)
            self._require_alive(self.vehicle, self._vehicle_output_root() / "adapter.pid", "vehicle pose adapter")
            self._require_alive(self.remote, self._remote_output_root() / "compositor.pid", "remote compositor")
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
        return (
            self._ssh(self.remote, self._remote_preflight_script()),
            self._ssh(self.vehicle, self._vehicle_preflight_script()),
            self._ssh(self.remote, self._remote_start_script()),
            self._ssh(self.vehicle, self._vehicle_adapter_script()),
            self._ssh(self.vehicle, self._bag_play_script()),
            self._ssh(self.remote, self._metrics_count_script()),
            self._ssh(self.remote, self._stop_remote_script()),
            self._ssh(self.vehicle, self._stop_vehicle_script()),
            self._scp_from(self.remote, self._remote_output_root() / "composited"),
            self._scp_from(self.remote, self._remote_output_root() / "metrics"),
            self._scp_from(self.remote, self._remote_output_root() / "logs"),
            self._scp_from(self.vehicle, self._vehicle_output_root()),
        )

    def _launch_remote(self) -> None:
        self._checked(self._ssh(self.remote, self._remote_start_script()), "remote compositor")
        self._started = True

    def _launch_vehicle_adapter(self) -> None:
        self._checked(self._ssh(self.vehicle, self._vehicle_adapter_script()), "vehicle pose adapter")

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
        vehicle_metrics = self.options.local_artifacts / self.vehicle.name / "ros_ego_pose_adapter.jsonl"
        if not vehicle_metrics.is_file():
            raise LaunchError(f"vehicle adapter metrics were not retrieved: {vehicle_metrics}")
        vehicle_records = [json.loads(line) for line in vehicle_metrics.read_text(encoding="utf-8").splitlines() if line]
        if len(vehicle_records) != self.options.expected_samples:
            raise LaunchError(f"expected {self.options.expected_samples} vehicle metrics, got {len(vehicle_records)}")
        for index, record in enumerate(vehicle_records):
            if record.get("sequence") != index or record.get("source_timestamp_us") != index * 100_000:
                raise LaunchError(f"unexpected vehicle pose metric at record {index}: {record}")

    def _remote_start_script(self) -> str:
        root = self._remote_output_root()
        command = self._remote_python_command() + [
            "-m", "src.realtime.composited_camera_process",
            "--frames-endpoint", self.vehicle.endpoint_uri,
            "--assets-endpoint", self.vehicle.asset_endpoint_uri,
            "--scene-generation", self.options.scene_generation,
            "--receiver-id", f"{self.remote.name}-compositor",
            "--health-endpoint", f"tcp://127.0.0.1:{self.options.health_port}",
            "--splat", str(self.options.splat_path), "--output-dir", str(root / "composited"),
            "--metrics-path", str(root / "metrics" / "compositor.jsonl"),
            "--agent", "1", "--render-width", "480", "--render-height", "300",
        ]
        return self._background_script(self.remote.repo_path, command, root / "logs" / "compositor.log", root / "compositor.pid")

    def _vehicle_adapter_script(self) -> str:
        root = self._vehicle_output_root()
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
        return "test ! -e " + shlex.quote(str(self._remote_output_root()))

    def _vehicle_preflight_script(self) -> str:
        host_mcap = self.vehicle.repo_path / self.options.mcap_path
        container_mcap = PurePosixPath("/workspace") / self.options.mcap_path
        sidecar = shlex.quote(self.options.ros_sidecar)
        return " && ".join((
            "test ! -e " + shlex.quote(str(self.vehicle.repo_path / self.options.output_root)),
            "test -f " + shlex.quote(str(host_mcap)),
            "docker inspect --format '{{.State.Running}}' " + sidecar + " | grep -qx true",
            "docker exec " + sidecar + " test -f " + shlex.quote(str(container_mcap)),
            "docker exec " + sidecar + " bash -lc " + shlex.quote(
                "source /opt/ros/humble/setup.sh && command -v ros2"
            ),
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
            self.output("warning: remote compositor did not exit within 10s; MP4 may not be fully flushed")
        result = self.runner.run(self._ssh(self.vehicle, self._stop_vehicle_script()))
        if result.returncode != 0:
            self.output(f"warning: could not stop vehicle process: {result.stderr.strip()}")

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
    parser.add_argument("--mcap", type=_path, default=PurePosixPath("artifacts/cooperscene_take_1_agent_1.mcap"))
    parser.add_argument("--splat", type=_path, default=PurePosixPath("data/riverside_r3.spz"))
    parser.add_argument("--output-root", type=_path, default=PurePosixPath("artifacts/split_system_simulation"))
    parser.add_argument("--local-artifacts", type=Path, default=Path("artifacts/split_system_simulation"))
    parser.add_argument("--expected-samples", type=int, default=501)
    parser.add_argument("--settle-seconds", type=float, default=3.0)
    parser.add_argument("--timeout-seconds", type=float, default=180.0)
    parser.add_argument("--poll-seconds", type=float, default=1.0)
    parser.add_argument("--scene-generation", default="cooperscene:train:1:1")
    parser.add_argument("--health-port", type=int, default=8770)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    try:
        launcher = RemoteTeleopSimulationLauncher(LaunchOptions(
            config_path=args.config, ros_sidecar=args.ros_sidecar, remote_python=args.remote_python,
            mcap_path=args.mcap, splat_path=args.splat, output_root=args.output_root,
            local_artifacts=args.local_artifacts, expected_samples=args.expected_samples,
            settle_seconds=args.settle_seconds, timeout_seconds=args.timeout_seconds,
            poll_seconds=args.poll_seconds, scene_generation=args.scene_generation, health_port=args.health_port,
        ), dry_run=args.dry_run)
        result = launcher.run()
    except (LaunchError, ValueError) as error:
        parser.error(str(error))
    if not args.dry_run:
        print(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

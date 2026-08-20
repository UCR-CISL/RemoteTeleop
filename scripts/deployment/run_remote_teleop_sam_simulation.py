#!/usr/bin/env python3
"""Run the lossless split replay plus asynchronous vehicle-local SAM workers."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path, PurePosixPath
import shlex
import sys
import time
from typing import Sequence

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.deployment.run_remote_teleop_simulation import (
    LaunchError,
    LaunchOptions,
    RemoteTeleopSimulationLauncher,
    _path,
)


@dataclass(frozen=True)
class SamLaunchOptions:
    vehicle_python: str = ".venv/bin/python"
    stable_seconds: float = 2.0
    sam_timeout_seconds: float = 1_800.0
    precision: str = "fp16"

    def __post_init__(self) -> None:
        if self.stable_seconds < 0 or self.sam_timeout_seconds <= 0:
            raise ValueError("SAM stability must be non-negative and timeout positive")
        if self.precision not in {"default", "fp16", "nf4"}:
            raise ValueError("SAM precision must be default, fp16, or nf4")


class SamRemoteTeleopSimulationLauncher(RemoteTeleopSimulationLauncher):
    """Coordinate reliable rendering while SAM meshes arrive asynchronously."""

    def __init__(self, options: LaunchOptions, sam: SamLaunchOptions, **kwargs) -> None:
        super().__init__(options, **kwargs)
        self.sam = sam

    def run(self) -> Path:
        if self.dry_run:
            for command in self.planned_commands():
                self.output(shlex.join(command))
            return self.options.local_artifacts
        if self.options.local_artifacts.exists():
            raise LaunchError(
                f"local artifact destination already exists: {self.options.local_artifacts}"
            )
        try:
            self._checked(self._ssh(self.remote, self._remote_preflight_script()), "remote output preflight")
            self._checked(self._ssh(self.vehicle, self._sam_vehicle_preflight_script()), "vehicle SAM preflight")
            self._launch_remote()
            self._launch_vehicle_adapter()
            self._launch_sam_workers()
            self.sleep(self.options.settle_seconds)
            self._require_alive(self.vehicle, self._vehicle_output_root() / "adapter.pid", "vehicle adapter")
            self._require_alive(self.vehicle, self._vehicle_output_root() / "sam3.pid", "SAM3 worker")
            self._require_alive(self.vehicle, self._vehicle_output_root() / "sam3d.pid", "SAM3D worker")
            self._require_alive(self.remote, self._remote_output_root() / "compositor.pid", "remote compositor")
            self._play_mcap()
            # Authoritative pose+bbox rendering completes independently of SAM.
            self._wait_for_remote_metrics()
            self._wait_for_sam3d_completion()
            self._wait_for_asset_sync()
        finally:
            if self._started:
                self._stop_processes()
        self._retrieve_artifacts()
        self._validate_retrieved_metrics()
        return self.options.local_artifacts / "composited" / "composited.mp4"

    def planned_commands(self) -> tuple[tuple[str, ...], ...]:
        base = super().planned_commands()
        starts = (
            self._ssh(self.remote, self._remote_start_script()),
            self._ssh(self.vehicle, self._vehicle_adapter_script()),
            self._ssh(self.vehicle, self._sam3d_start_script()),
            self._ssh(self.vehicle, self._sam3_start_script()),
        )
        stops = tuple(
            self._ssh(self.vehicle, self._sam_stop_script(name))
            for name in ("sam3", "sam3d")
        )
        return (
            (base[0], self._ssh(self.vehicle, self._sam_vehicle_preflight_script()))
            + starts
            + (
                self._ssh(self.vehicle, self._topic_play_script("/camera/frame_detections")),
                self._ssh(self.vehicle, self._topic_play_script("/camera/image_raw", rate=0.5)),
                base[5],
            )
            + stops
            + base[6:]
        )

    def _play_mcap(self) -> None:
        """Replay metadata and images separately across the ROS durability boundary.

        Raw image publication can otherwise starve small metadata callbacks in
        Humble's rosbag/DDS path.  The metadata pass is authoritative for remote
        rendering; the slower image pass fills the independent local SAM spool.
        Each pass must install its durable end marker before launch proceeds.
        """

        self._checked(
            self._ssh(
                self.vehicle,
                self._topic_play_script("/camera/frame_detections"),
            ),
            "MCAP metadata playback",
        )
        self._wait_for_spool_end("frame-spool", "render snapshot")
        self._require_alive(
            self.remote,
            self._remote_output_root() / "compositor.pid",
            "remote compositor",
        )
        self._checked(
            self._ssh(
                self.vehicle,
                self._topic_play_script("/camera/image_raw", rate=0.5),
            ),
            "MCAP image playback",
        )
        self._wait_for_spool_end("analysis-spool", "analysis frame")

    def _topic_play_script(self, topic: str, *, rate: float = 1.0) -> str:
        if not topic.startswith("/") or rate <= 0:
            raise ValueError("ROS playback topic and rate must be valid")
        in_container_mcap = PurePosixPath("/workspace") / self.options.mcap_path
        timeout_seconds = max(1, math.ceil(self.options.timeout_seconds))
        inner = (
            "source /opt/ros/humble/setup.sh && timeout --signal=INT "
            + f"{timeout_seconds}s ros2 bag play -s mcap "
            + shlex.quote(str(in_container_mcap))
            + " --disable-keyboard-controls --read-ahead-queue-size 10"
            + " --wait-for-all-acked 0 --rate "
            + shlex.quote(str(rate))
            + " --topics "
            + shlex.quote(topic)
        )
        command = ["docker", "exec", self.options.ros_sidecar, "bash", "-lc", inner]
        return (
            "cd "
            + shlex.quote(str(self.vehicle.repo_path))
            + " && "
            + shlex.join(command)
        )

    def _wait_for_spool_end(self, spool_name: str, label: str) -> None:
        root = self._vehicle_output_root() / spool_name
        script = (
            "test \"$(find "
            + shlex.quote(str(root))
            + " -name end.json -type f 2>/dev/null | wc -l)\" -eq 1"
        )
        deadline = self.clock() + self.options.timeout_seconds
        while self.clock() < deadline:
            if self.runner.run(self._ssh(self.vehicle, script)).returncode == 0:
                return
            self._require_alive(
                self.vehicle,
                self._vehicle_output_root() / "adapter.pid",
                "vehicle adapter",
            )
            self.sleep(self.options.poll_seconds)
        raise LaunchError(f"timed out waiting for complete durable {label} spool")

    def _vehicle_adapter_script(self) -> str:
        root = self._vehicle_output_root()
        analysis_port = self.vehicle.analysis_port
        if analysis_port is None:
            raise LaunchError("vehicle configuration requires analysis_port")
        inner = (
            "source /opt/ros/humble/setup.sh && cd /workspace && "
            "PYTHONPATH=/workspace${PYTHONPATH:+:$PYTHONPATH} "
            + shlex.join([
                "/usr/bin/python3", "src/deployment/ros_frame_adapter.py",
                "--frames-endpoint", f"tcp://0.0.0.0:{self.vehicle.port}",
                "--assets-endpoint", f"tcp://0.0.0.0:{self.vehicle.asset_port}",
                "--analysis-endpoint", f"tcp://0.0.0.0:{analysis_port}",
                "--queue-depth", "2048", "--high-water-mark", "2048",
                "--expected-samples", str(self.options.expected_samples),
                "--scene-generation", self.options.scene_generation,
                "--metrics-path", str(self.options.output_root / self.vehicle.name / "ros_ego_pose_adapter.jsonl"),
                "--spool-root", str(self.options.output_root / self.vehicle.name / "frame-spool"),
                "--analysis-spool-root", str(self.options.output_root / self.vehicle.name / "analysis-spool"),
                "--asset-store-root", str(self.options.output_root / self.vehicle.name / "asset-store"),
            ])
        )
        command = ["docker", "exec", self.options.ros_sidecar, "bash", "-lc", inner]
        return self._background_script(
            self.vehicle.repo_path, command, root / "logs" / "adapter.log", root / "adapter.pid"
        )

    def _launch_sam_workers(self) -> None:
        self._checked(self._ssh(self.vehicle, self._sam3d_start_script()), "SAM3D worker")
        self._checked(self._ssh(self.vehicle, self._sam3_start_script()), "SAM3 worker")

    def _sam3d_start_script(self) -> str:
        root = self._vehicle_output_root()
        residency_lock = self.options.output_root / self.vehicle.name / "gpu-residency.lock"
        command = self._vehicle_python_command() + [
            "-m", "src.realtime.sam3d_worker",
            "--analysis-endpoint", self._local_analysis_endpoint(),
            "--scene-generation", self.options.scene_generation,
            "--reconstruction-endpoint", "tcp://127.0.0.1:5557",
            "--asset-store-root", str(self.options.output_root / self.vehicle.name / "asset-store"),
            "--health-endpoint", f"tcp://127.0.0.1:{self.options.health_port}",
            "--output-root", str(self.options.output_root / self.vehicle.name / "sam3d"),
            "--metrics-path", str(self.options.output_root / self.vehicle.name / "metrics" / "sam3d.jsonl"),
            "--completion-path", str(self.options.output_root / self.vehicle.name / "sam3d-complete.json"),
            "--gpu-residency-lock", str(residency_lock),
            "--precision", self.sam.precision,
        ]
        return self._background_script(
            self.vehicle.repo_path, command, root / "logs" / "sam3d.log", root / "sam3d.pid"
        )

    def _sam3_start_script(self) -> str:
        root = self._vehicle_output_root()
        residency_lock = self.options.output_root / self.vehicle.name / "gpu-residency.lock"
        command = self._vehicle_python_command() + [
            "-m", "src.realtime.mask_process",
            "--analysis-endpoint", self._local_analysis_endpoint(),
            "--scene-generation", self.options.scene_generation,
            "--reconstruction-endpoint", "tcp://127.0.0.1:5557",
            "--masks-endpoint", "tcp://127.0.0.1:5556",
            "--health-endpoint", f"tcp://127.0.0.1:{self.options.health_port}",
            "--metrics-path", str(self.options.output_root / self.vehicle.name / "metrics" / "sam3.jsonl"),
            "--mask-policy", "new_tracks",
            "--stable-seconds", str(self.sam.stable_seconds),
            "--gpu-residency-lock", str(residency_lock),
            "--precision", self.sam.precision,
        ]
        return self._background_script(
            self.vehicle.repo_path, command, root / "logs" / "sam3.log", root / "sam3.pid"
        )

    def _vehicle_python_command(self) -> list[str]:
        command = shlex.split(self.sam.vehicle_python)
        if not command:
            raise LaunchError("--vehicle-python must not be empty")
        return command

    def _local_analysis_endpoint(self) -> str:
        if self.vehicle.analysis_port is None:
            raise LaunchError("vehicle configuration requires analysis_port")
        return f"tcp://127.0.0.1:{self.vehicle.analysis_port}"

    def _sam_vehicle_preflight_script(self) -> str:
        python = shlex.join(self._vehicle_python_command())
        return " && ".join((
            self._vehicle_preflight_script(),
            "command -v nvidia-smi >/dev/null",
            python + " -c " + shlex.quote(
                "import torch, zmq, cv2; assert torch.cuda.is_available(); "
                "import src.realtime.mask_process, src.realtime.sam3d_worker"
            ),
        ))

    def _wait_for_sam3d_completion(self) -> None:
        marker = self._vehicle_output_root() / "sam3d-complete.json"
        deadline = self.clock() + self.sam.sam_timeout_seconds
        while self.clock() < deadline:
            result = self.runner.run(self._ssh(self.vehicle, "test -f " + shlex.quote(str(marker))))
            if result.returncode == 0:
                return
            if not self._is_alive(self.vehicle, self._vehicle_output_root() / "sam3d.pid"):
                raise LaunchError("SAM3D exited before its durable completion marker")
            self.sleep(self.options.poll_seconds)
        raise LaunchError("timed out waiting for SAM3D reconstruction queue to drain")

    def _wait_for_asset_sync(self) -> None:
        vehicle_store = self._vehicle_output_root() / "asset-store"
        remote_cache = self._remote_output_root() / "composited" / "mesh-cache"
        deadline = self.clock() + self.sam.sam_timeout_seconds
        while self.clock() < deadline:
            produced = self.runner.run(self._ssh(
                self.vehicle,
                "find " + shlex.quote(str(vehicle_store)) + " -name '*.manifest.json' -type f 2>/dev/null | wc -l",
            ))
            received = self.runner.run(self._ssh(
                self.remote,
                "find " + shlex.quote(str(remote_cache)) + " -type f ! -name '*.partial' 2>/dev/null | wc -l",
            ))
            if produced.returncode == 0 and received.returncode == 0:
                produced_count = int(produced.stdout.strip() or "0")
                received_count = int(received.stdout.strip() or "0")
                if received_count >= produced_count:
                    return
            self.sleep(self.options.poll_seconds)
        raise LaunchError("timed out waiting for durable meshes to reach the remote cache")

    def _stop_processes(self) -> None:
        for name in ("sam3", "sam3d"):
            self.runner.run(self._ssh(self.vehicle, self._sam_stop_script(name)))
            pid_path = self._vehicle_output_root() / f"{name}.pid"
            if not self._wait_for_exit(self.vehicle, pid_path, timeout_seconds=10.0):
                self.runner.run(
                    self._ssh(self.vehicle, self._sam_kill_script(name, "TERM"))
                )
                if not self._wait_for_exit(
                    self.vehicle, pid_path, timeout_seconds=5.0
                ):
                    self.runner.run(
                        self._ssh(self.vehicle, self._sam_kill_script(name, "KILL"))
                    )
                    self._wait_for_exit(
                        self.vehicle, pid_path, timeout_seconds=2.0
                    )
        super()._stop_processes()

    def _sam_stop_script(self, name: str) -> str:
        return self._stop_script(
            self._vehicle_output_root() / f"{name}.pid", process_group=True
        )

    def _sam_kill_script(self, name: str, signal: str) -> str:
        if signal not in {"TERM", "KILL"}:
            raise ValueError("unsupported cleanup signal")
        pid = shlex.quote(str(self._vehicle_output_root() / f"{name}.pid"))
        return (
            f"if test -f {pid}; then /bin/kill -{signal} -- "
            f'"-$(cat {pid})" 2>/dev/null || true; fi'
        )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("cfg/alien4_alien3.yaml"))
    parser.add_argument("--ros-sidecar", default="remote_teleop_ros_mcap")
    parser.add_argument("--remote-python", default="/home/coop3r-slam/miniconda3/bin/conda run -n coop3r-slam python")
    parser.add_argument("--vehicle-python", default=".venv/bin/python")
    parser.add_argument("--mcap", type=_path, default=PurePosixPath("artifacts/cooperscene_take_1_agent_1.mcap"))
    parser.add_argument("--splat", type=_path, default=PurePosixPath("data/riverside_r3.spz"))
    parser.add_argument("--output-root", type=_path, default=PurePosixPath("artifacts/split_system_sam_simulation"))
    parser.add_argument("--local-artifacts", type=Path, default=Path("artifacts/split_system_sam_simulation"))
    parser.add_argument("--expected-samples", type=int, default=501)
    parser.add_argument("--settle-seconds", type=float, default=3.0)
    parser.add_argument("--timeout-seconds", type=float, default=180.0)
    parser.add_argument("--sam-timeout-seconds", type=float, default=1_800.0)
    parser.add_argument("--poll-seconds", type=float, default=1.0)
    parser.add_argument("--stable-seconds", type=float, default=2.0)
    parser.add_argument("--precision", choices=("default", "fp16", "nf4"), default="fp16")
    parser.add_argument("--scene-generation", default="cooperscene:train:1:1")
    parser.add_argument("--health-port", type=int, default=8770)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    try:
        options = LaunchOptions(
            config_path=args.config, ros_sidecar=args.ros_sidecar, remote_python=args.remote_python,
            mcap_path=args.mcap, splat_path=args.splat, output_root=args.output_root,
            local_artifacts=args.local_artifacts, expected_samples=args.expected_samples,
            settle_seconds=args.settle_seconds, timeout_seconds=args.timeout_seconds,
            poll_seconds=args.poll_seconds, scene_generation=args.scene_generation,
            health_port=args.health_port,
        )
        launcher = SamRemoteTeleopSimulationLauncher(
            options,
            SamLaunchOptions(
                vehicle_python=args.vehicle_python,
                stable_seconds=args.stable_seconds,
                sam_timeout_seconds=args.sam_timeout_seconds,
                precision=args.precision,
            ),
            dry_run=args.dry_run,
        )
        result = launcher.run()
    except (LaunchError, ValueError) as error:
        parser.error(str(error))
    if not args.dry_run:
        print(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

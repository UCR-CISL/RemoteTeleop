"""Launch the isolated nuScenes, SAM 3.1, SAM3D, and Rerun processes."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

import zmq

from src.realtime.protocol import WorkerHealth, WorkerState
from src.realtime.transport import SubscriberTransport


@dataclass(frozen=True)
class RuntimeEndpoints:
    frames: str = "tcp://127.0.0.1:5555"
    masks: str = "tcp://127.0.0.1:5556"
    reconstruction: str = "tcp://127.0.0.1:5557"
    assets: str = "tcp://127.0.0.1:5558"
    health: str = "tcp://127.0.0.1:5559"


class RealtimePipelineSupervisor:
    """Gate replay on resident-model readiness and stop on terminal failures."""

    def __init__(
        self,
        *,
        project_root: Path,
        dataroot: Path,
        scene: str,
        version: str,
        output_root: Path,
        python: Path,
        sam3d_config: Path,
        checkpoint: Path | None = None,
        offline_mask_root: Path | None = None,
        endpoints: RuntimeEndpoints = RuntimeEndpoints(),
        readiness_timeout_seconds: float = 900.0,
        viewer: bool = True,
        spawn_viewer: bool = True,
        fp16: bool = False,
        sam3_prompt_mode: str = "serial_grounding",
        sam3_mask_policy: str = "new_tracks",
        sam3d_priority_policy: str = "quality",
        sam3d_coalesce_ms: float = 0.0,
        sam3d_stage1_inference_steps: int | None = None,
        sam3d_stage2_inference_steps: int | None = None,
        sam3d_pointmap_cache_size: int = 0,
    ) -> None:
        self.project_root = project_root.resolve()
        self.dataroot = dataroot.resolve()
        self.scene = scene
        self.version = version
        self.output_root = output_root.resolve()
        self.python = (
            python if python.is_absolute() else self.project_root / python
        ).absolute()
        self.sam3d_config = sam3d_config
        self.checkpoint = checkpoint
        self.offline_mask_root = offline_mask_root
        self.endpoints = endpoints
        self.readiness_timeout_seconds = readiness_timeout_seconds
        self.viewer = viewer
        self.spawn_viewer = spawn_viewer
        self.fp16 = fp16
        self.sam3_prompt_mode = sam3_prompt_mode
        self.sam3_mask_policy = sam3_mask_policy
        self.sam3d_priority_policy = sam3d_priority_policy
        self.sam3d_coalesce_ms = sam3d_coalesce_ms
        self.sam3d_stage1_inference_steps = sam3d_stage1_inference_steps
        self.sam3d_stage2_inference_steps = sam3d_stage2_inference_steps
        self.sam3d_pointmap_cache_size = sam3d_pointmap_cache_size
        self._processes: dict[str, subprocess.Popen[str]] = {}
        self._logs: dict[str, object] = {}
        self._health: dict[str, WorkerHealth] = {}
        self._context = zmq.Context()
        self._health_subscriber = SubscriberTransport.open(
            self._context,
            endpoints.health,
            bind=True,
            topics=("worker_health",),
            high_water_mark=100,
        )

    def run(self) -> int:
        self._validate()
        self.output_root.mkdir(parents=True, exist_ok=True)
        try:
            self._launch("gpu-monitor", self._gpu_monitor_command())
            # Warm SAM 3.1 first, then load SAM3D while it remains resident.
            # Synthetic SAM3D inputs proved unsafe because their generated
            # geometry has unbounded density; live OOM remains fail-fast.
            self._launch("sam3-mask", self._mask_command())
            self._wait_ready({"sam3-mask"})
            self._launch("sam3d-object", self._sam3d_command())
            self._wait_ready({"sam3-mask", "sam3d-object"})
            if self.viewer:
                self._launch("viewer", self._viewer_command())
                self._wait_ready({"sam3d-object", "sam3-mask", "viewer"})
            self._write_admission_report("ready")

            self._launch("nuscenes-replay", self._replay_command())
            replay_code = self._wait_for_replay()
            if replay_code != 0:
                raise RuntimeError(f"nuScenes replay exited with status {replay_code}")
            self._wait_until_drained()
            return 0
        except Exception as error:
            self._write_admission_report("failed", error=str(error))
            raise
        finally:
            self.close()

    def close(self) -> None:
        for process in reversed(tuple(self._processes.values())):
            if process.poll() is None:
                process.terminate()
        deadline = time.monotonic() + 10.0
        for process in reversed(tuple(self._processes.values())):
            if process.poll() is None:
                try:
                    process.wait(timeout=max(0.1, deadline - time.monotonic()))
                except subprocess.TimeoutExpired:
                    process.kill()
        for stream in self._logs.values():
            stream.close()
        self._health_subscriber.close()
        self._context.term()

    def _launch(self, name: str, command: list[str]) -> None:
        log_path = self.output_root / "logs" / f"{name}.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        stream = log_path.open("w", encoding="utf-8")
        environment = dict(os.environ)
        environment["PYTHONPATH"] = str(self.project_root)
        environment["PATH"] = os.pathsep.join(
            (str(self.python.parent), environment.get("PATH", ""))
        )
        environment["MAX_JOBS"] = "2"
        environment["CUDA_VISIBLE_DEVICES"] = "0"
        environment["PYTORCH_CUDA_ALLOC_CONF"] = "expandable_segments:True"
        self._logs[name] = stream
        self._processes[name] = subprocess.Popen(
            command,
            cwd=self.project_root,
            env=environment,
            stdout=stream,
            stderr=subprocess.STDOUT,
            text=True,
        )

    def _wait_ready(self, required: set[str]) -> None:
        deadline = time.monotonic() + self.readiness_timeout_seconds
        while time.monotonic() < deadline:
            self._receive_health(500)
            self._raise_for_failures()
            if all(
                worker_id in self._health
                and self._health[worker_id].state is WorkerState.READY
                for worker_id in required
            ):
                return
        states = {
            worker_id: self._health.get(worker_id).state.value
            if worker_id in self._health
            else "missing"
            for worker_id in required
        }
        raise TimeoutError(f"workers did not become ready: {states}")

    def _wait_for_replay(self) -> int:
        replay = self._processes["nuscenes-replay"]
        while replay.poll() is None:
            self._receive_health(500)
            self._raise_for_failures()
        return int(replay.returncode)

    def _wait_until_drained(self) -> None:
        stable = 0
        deadline = time.monotonic() + self.readiness_timeout_seconds
        while time.monotonic() < deadline:
            self._receive_health(500)
            self._raise_for_failures()
            mask = self._health.get("sam3-mask")
            sam3d = self._health.get("sam3d-object")
            drained = (
                mask is not None
                and sam3d is not None
                and mask.state is WorkerState.READY
                and mask.queue_depth == 0
                and sam3d.state is WorkerState.READY
                and sam3d.queue_depth == 0
            )
            stable = stable + 1 if drained else 0
            if stable >= 4:
                return
        raise TimeoutError("reconstruction pipeline did not drain")

    def _receive_health(self, timeout_ms: int) -> None:
        if not self._health_subscriber.socket.poll(timeout=timeout_ms):
            self._raise_for_exited_processes()
            return
        _, message = self._health_subscriber.receive()
        if isinstance(message, WorkerHealth):
            self._health[message.worker_id] = message
        self._raise_for_exited_processes()

    def _raise_for_failures(self) -> None:
        for worker_id, health in self._health.items():
            if health.state is WorkerState.FAILED:
                raise RuntimeError(f"{worker_id} failed: {health.detail}")
        self._raise_for_exited_processes()

    def _raise_for_exited_processes(self) -> None:
        for name, process in self._processes.items():
            if name == "nuscenes-replay":
                continue
            status = process.poll()
            if status is not None:
                raise RuntimeError(f"{name} exited unexpectedly with status {status}")

    def _sam3d_command(self) -> list[str]:
        command = [
            str(self.python),
            "-m",
            "src.realtime.sam3d_worker",
            "--frames-endpoint",
            self.endpoints.frames,
            "--reconstruction-endpoint",
            self.endpoints.reconstruction,
            "--assets-endpoint",
            self.endpoints.assets,
            "--health-endpoint",
            self.endpoints.health,
            "--output-root",
            str(self.output_root / "meshes"),
            "--metrics-path",
            str(self.output_root / "metrics" / "sam3d.jsonl"),
            "--sam3d-repository",
            "thirdparty/sam-3d-objects",
            "--sam3d-config",
            str(self.sam3d_config),
            "--priority-policy",
            self.sam3d_priority_policy,
            "--coalesce-ms",
            str(self.sam3d_coalesce_ms),
            "--pointmap-cache-size",
            str(self.sam3d_pointmap_cache_size),
        ]
        if self.fp16:
            command.extend(["--precision", "fp16"])
        if self.sam3d_stage1_inference_steps is not None:
            command.extend(
                ["--stage1-inference-steps", str(self.sam3d_stage1_inference_steps)]
            )
        if self.sam3d_stage2_inference_steps is not None:
            command.extend(
                ["--stage2-inference-steps", str(self.sam3d_stage2_inference_steps)]
            )
        return command

    @staticmethod
    def _gpu_monitor_command() -> list[str]:
        return [
            "nvidia-smi",
            "--query-gpu=timestamp,utilization.gpu,memory.used,power.draw",
            "--format=csv,noheader,nounits",
            "--loop-ms=250",
        ]

    def _mask_command(self) -> list[str]:
        command = [
            str(self.python),
            "-m",
            "src.realtime.mask_process",
            "--frames-endpoint",
            self.endpoints.frames,
            "--masks-endpoint",
            self.endpoints.masks,
            "--reconstruction-endpoint",
            self.endpoints.reconstruction,
            "--health-endpoint",
            self.endpoints.health,
            "--metrics-path",
            str(self.output_root / "metrics" / "sam3.jsonl"),
            "--prompt-mode",
            self.sam3_prompt_mode,
            "--mask-policy",
            self.sam3_mask_policy,
        ]
        if self.offline_mask_root is not None:
            command.extend(
                ["--backend", "offline", "--offline-mask-root", str(self.offline_mask_root)]
            )
        elif self.checkpoint is not None:
            command.extend(["--checkpoint", str(self.checkpoint)])
        if self.fp16:
            command.extend(["--precision", "fp16"])
        return command

    def _viewer_command(self) -> list[str]:
        command = [
            str(self.python),
            "-m",
            "src.realtime.viewer_process",
            "--frames-endpoint",
            self.endpoints.frames,
            "--assets-endpoint",
            self.endpoints.assets,
            "--health-endpoint",
            self.endpoints.health,
            "--metrics-path",
            str(self.output_root / "metrics" / "viewer.jsonl"),
        ]
        if not self.spawn_viewer:
            command.append("--no-spawn")
        return command

    def _replay_command(self) -> list[str]:
        return [
            str(self.python),
            "-m",
            "src.realtime.nuscenes_replay",
            "--dataroot",
            str(self.dataroot),
            "--version",
            self.version,
            "--scene",
            self.scene,
            "--frames-endpoint",
            self.endpoints.frames,
            "--health-endpoint",
            self.endpoints.health,
            "--metrics-path",
            str(self.output_root / "metrics" / "replay.jsonl"),
        ]

    def _validate(self) -> None:
        for label, path in (
            ("nuScenes dataroot", self.dataroot),
            ("runtime Python", self.python),
        ):
            if not path.exists():
                raise FileNotFoundError(f"{label} does not exist: {path}")
        if self.viewer and self.spawn_viewer:
            rerun = self.python.parent / "rerun"
            if not rerun.is_file() or not os.access(rerun, os.X_OK):
                raise FileNotFoundError(
                    f"Rerun viewer executable does not exist: {rerun}; "
                    "run scripts/run_uv.sh"
                )
        if shutil.which("nvidia-smi") is None:
            raise FileNotFoundError("nvidia-smi is required for CUDA admission metrics")

    def _write_admission_report(self, status: str, *, error: str | None = None) -> None:
        report = {
            "status": status,
            "scene": self.scene,
            "version": self.version,
            "sam3_backend": "offline" if self.offline_mask_root else "sam3.1",
            "precision": "fp16" if self.fp16 else "default",
            "sam3_prompt_mode": self.sam3_prompt_mode,
            "sam3_mask_policy": self.sam3_mask_policy,
            "sam3d_priority_policy": self.sam3d_priority_policy,
            "sam3d_coalesce_ms": self.sam3d_coalesce_ms,
            "sam3d_stage1_inference_steps": self.sam3d_stage1_inference_steps,
            "sam3d_stage2_inference_steps": self.sam3d_stage2_inference_steps,
            "sam3d_pointmap_cache_size": self.sam3d_pointmap_cache_size,
            "workers": {
                worker_id: {
                    "state": health.state.value,
                    "device": health.device,
                    "cuda_allocated_mib": health.cuda_allocated_mib,
                    "cuda_reserved_mib": health.cuda_reserved_mib,
                    "detail": health.detail,
                }
                for worker_id, health in self._health.items()
            },
            "error": error,
        }
        path = self.output_root / "admission.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataroot", type=Path, required=True)
    parser.add_argument("--scene", default="scene-0061")
    parser.add_argument("--version", default="v1.0-mini")
    parser.add_argument("--output-root", type=Path, default=Path("artifacts/realtime"))
    parser.add_argument("--python", type=Path, default=Path(".venv/bin/python"))
    parser.add_argument(
        "--sam3d-config", type=Path, default=Path("checkpoints/hf/pipeline.yaml")
    )
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--offline-mask-root", type=Path)
    parser.add_argument("--readiness-timeout-seconds", type=float, default=900.0)
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--no-spawn-viewer", action="store_true")
    parser.add_argument("--fp16", action="store_true")
    parser.add_argument(
        "--sam3-prompt-mode",
        choices=("serial_grounding", "interactive_batch"),
        default="serial_grounding",
    )
    parser.add_argument(
        "--sam3-mask-policy",
        choices=("new_tracks", "all_visible"),
        default="new_tracks",
    )
    parser.add_argument(
        "--sam3d-priority-policy",
        choices=("quality", "deadline"),
        default="quality",
    )
    parser.add_argument("--sam3d-coalesce-ms", type=float, default=0.0)
    parser.add_argument("--sam3d-stage1-inference-steps", type=int)
    parser.add_argument("--sam3d-stage2-inference-steps", type=int)
    parser.add_argument("--sam3d-pointmap-cache-size", type=int, default=0)
    args = parser.parse_args()
    supervisor = RealtimePipelineSupervisor(
        project_root=Path.cwd(),
        dataroot=args.dataroot,
        scene=args.scene,
        version=args.version,
        output_root=args.output_root,
        python=args.python,
        sam3d_config=args.sam3d_config,
        checkpoint=args.checkpoint,
        offline_mask_root=args.offline_mask_root,
        readiness_timeout_seconds=args.readiness_timeout_seconds,
        viewer=not args.no_viewer,
        spawn_viewer=not args.no_spawn_viewer,
        fp16=args.fp16,
        sam3_prompt_mode=args.sam3_prompt_mode,
        sam3_mask_policy=args.sam3_mask_policy,
        sam3d_priority_policy=args.sam3d_priority_policy,
        sam3d_coalesce_ms=args.sam3d_coalesce_ms,
        sam3d_stage1_inference_steps=args.sam3d_stage1_inference_steps,
        sam3d_stage2_inference_steps=args.sam3d_stage2_inference_steps,
        sam3d_pointmap_cache_size=args.sam3d_pointmap_cache_size,
    )
    raise SystemExit(supervisor.run())


if __name__ == "__main__":
    main()

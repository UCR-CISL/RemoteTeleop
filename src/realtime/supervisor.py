"""Launch isolated replay, SAM 3.1, SAM3D, and visualization processes."""

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
from src.reconstruction.sam3d_upstream import validate_sam3d_device


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
        sam3_precision: str | None = None,
        sam3d_precision: str | None = None,
        sam3_prompt_mode: str | None = None,
        sam3_mask_policy: str = "new_tracks",
        sam3d_priority_policy: str = "quality",
        sam3d_coalesce_ms: float = 0.0,
        sam3d_stage1_inference_steps: int | None = None,
        sam3d_stage2_inference_steps: int | None = None,
        sam3d_pointmap_cache_size: int | None = None,
        source: str = "nuscenes",
        split: str = "train",
        agent: str = "1",
        splat_path: Path | None = None,
        localization_transform: Path | None = None,
        overlap_manifest: Path | None = None,
        stop_at_overlap: bool = False,
        viewer_mode: str | None = None,
        render_downsample: int = 2,
        stable_seconds: float | None = None,
        maximum_gap_seconds: float = 0.25,
        maximum_admission_batch: int = 5,
        minimum_projected_area_px: float | None = None,
        minimum_visible_fraction: float | None = None,
        sam3_device: str = "cuda",
        sam3d_device: str = "cuda:0",
        runtime_mode: str = "resident",
    ) -> None:
        if source not in {"nuscenes", "cooperscene"}:
            raise ValueError("source must be nuscenes or cooperscene")
        selected_viewer = viewer_mode or ("composited" if source == "cooperscene" else "rerun")
        if selected_viewer not in {"rerun", "composited"}:
            raise ValueError("viewer_mode must be rerun or composited")
        if runtime_mode not in {"resident", "take_turns"}:
            raise ValueError("runtime_mode must be resident or take_turns")
        selected_sam3_precision = sam3_precision or (
            "fp16" if fp16 and sam3_device.startswith("cuda") else "default"
        )
        selected_sam3d_precision = sam3d_precision or ("fp16" if fp16 else "default")
        for name, precision in (
            ("sam3_precision", selected_sam3_precision),
            ("sam3d_precision", selected_sam3d_precision),
        ):
            if precision not in {"default", "fp16", "nf4"}:
                raise ValueError(f"{name} must be default, fp16, or nf4")
        if selected_sam3_precision == "nf4" and not sam3_device.startswith("cuda"):
            raise ValueError("SAM3 NF4 inference requires a CUDA device")
        if selected_sam3d_precision == "nf4" and not sam3d_device.startswith("cuda"):
            raise ValueError("SAM3D NF4 inference requires a CUDA device")
        if runtime_mode == "take_turns" and not (
            sam3_device.casefold().startswith("cuda")
            and sam3d_device.casefold().startswith("cuda")
            and offline_mask_root is None
        ):
            raise ValueError(
                "take_turns requires CUDA SAM3, CUDA SAM3D, and the online SAM3 backend"
            )
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
        self.sam3_precision = selected_sam3_precision
        self.sam3d_precision = selected_sam3d_precision
        # The installed SAM 3.1 multiplex checkpoint does not initialize the
        # upstream interactive instance predictor. Serial box grounding still
        # shares one image encoding for every same-frame admission group.
        self.sam3_prompt_mode = sam3_prompt_mode or "serial_grounding"
        self.sam3_mask_policy = sam3_mask_policy
        self.sam3d_priority_policy = sam3d_priority_policy
        self.sam3d_coalesce_ms = sam3d_coalesce_ms
        self.sam3d_stage1_inference_steps = sam3d_stage1_inference_steps
        self.sam3d_stage2_inference_steps = sam3d_stage2_inference_steps
        self.sam3d_pointmap_cache_size = (
            5 if source == "cooperscene" else 0
        ) if sam3d_pointmap_cache_size is None else sam3d_pointmap_cache_size
        self.source = source
        self.split = split
        self.agent = agent
        self.splat_path = None if splat_path is None else splat_path.resolve()
        self.localization_transform = (
            None if localization_transform is None else localization_transform.resolve()
        )
        self.overlap_manifest = None if overlap_manifest is None else overlap_manifest.resolve()
        self.stop_at_overlap = stop_at_overlap
        self.viewer_mode = selected_viewer
        self.render_downsample = render_downsample
        self.stable_seconds = (2.0 if source == "cooperscene" else 0.0) if stable_seconds is None else stable_seconds
        self.maximum_gap_seconds = maximum_gap_seconds
        self.maximum_admission_batch = maximum_admission_batch
        self.minimum_projected_area_px = (
            1024.0 if source == "cooperscene" else 0.0
        ) if minimum_projected_area_px is None else minimum_projected_area_px
        self.minimum_visible_fraction = (
            0.5 if source == "cooperscene" else 0.0
        ) if minimum_visible_fraction is None else minimum_visible_fraction
        self.sam3_device = sam3_device
        self.sam3d_device = sam3d_device
        self.runtime_mode = runtime_mode
        self.gpu_residency_lock = self.output_root / "runtime" / "cuda-model.lock"
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
        self.output_root.mkdir(parents=True, exist_ok=True)
        try:
            self._validate()
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

            self._launch("replay", self._replay_command())
            replay_code = self._wait_for_replay()
            if replay_code != 0:
                raise RuntimeError(f"{self.source} replay exited with status {replay_code}")
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
        replay = self._processes["replay"]
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
            if name == "replay":
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
            "--device",
            self.sam3d_device,
        ]
        if self.sam3d_precision != "default":
            command.extend(["--precision", self.sam3d_precision])
        if self.sam3d_stage1_inference_steps is not None:
            command.extend(
                ["--stage1-inference-steps", str(self.sam3d_stage1_inference_steps)]
            )
        if self.sam3d_stage2_inference_steps is not None:
            command.extend(
                ["--stage2-inference-steps", str(self.sam3d_stage2_inference_steps)]
            )
        if self.runtime_mode == "take_turns":
            command.extend(
                ["--gpu-residency-lock", str(self.gpu_residency_lock)]
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
            "--stable-seconds",
            str(self.stable_seconds),
            "--maximum-gap-seconds",
            str(self.maximum_gap_seconds),
            "--maximum-admission-batch",
            str(self.maximum_admission_batch),
            "--minimum-projected-area-px",
            str(self.minimum_projected_area_px),
            "--minimum-visible-fraction",
            str(self.minimum_visible_fraction),
            "--device",
            self.sam3_device,
        ]
        if self.offline_mask_root is not None:
            command.extend(
                ["--backend", "offline", "--offline-mask-root", str(self.offline_mask_root)]
            )
        elif self.checkpoint is not None:
            command.extend(["--checkpoint", str(self.checkpoint)])
        if self.sam3_precision != "default" and self.offline_mask_root is None:
            command.extend(["--precision", self.sam3_precision])
        if self.runtime_mode == "take_turns":
            command.extend(
                [
                    "--gpu-residency-lock",
                    str(self.gpu_residency_lock),
                    "--warmup-iterations",
                    "1",
                ]
            )
        return command

    def _viewer_command(self) -> list[str]:
        if self.viewer_mode == "composited":
            assert self.splat_path is not None
            return [
                str(self.python),
                "-m",
                "src.realtime.composited_camera_process",
                "--frames-endpoint",
                self.endpoints.frames,
                "--assets-endpoint",
                self.endpoints.assets,
                "--health-endpoint",
                self.endpoints.health,
                "--splat",
                str(self.splat_path),
                "--output-dir",
                str(self.output_root / "composited"),
                "--metrics-path",
                str(self.output_root / "metrics" / "compositor.jsonl"),
                "--render-downsample",
                str(self.render_downsample),
            ]
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
        if self.source == "cooperscene":
            assert self.localization_transform is not None
            command = [
                str(self.python),
                "-m",
                "src.realtime.cooperscene_replay",
                "--data-root",
                str(self.dataroot),
                "--split",
                self.split,
                "--scenario",
                self.scene,
                "--agent",
                self.agent,
                "--transform",
                str(self.localization_transform),
                "--frames-endpoint",
                self.endpoints.frames,
                "--health-endpoint",
                self.endpoints.health,
                "--metrics-path",
                str(self.output_root / "metrics" / "replay.jsonl"),
            ]
            if self.overlap_manifest is not None:
                command.extend(["--overlap-manifest", str(self.overlap_manifest)])
            if self.stop_at_overlap:
                command.append("--stop-at-overlap")
            return command
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
        sam3_device = self.sam3_device.strip().casefold()
        sam3_cuda = sam3_device == "cuda" or (
            sam3_device.startswith("cuda:")
            and sam3_device.removeprefix("cuda:").isdigit()
        )
        if sam3_device != "cpu" and not sam3_cuda:
            raise ValueError("SAM3 device must be 'cpu' or a CUDA device")
        validate_sam3d_device(self.sam3d_device)
        for label, path in (("dataset root", self.dataroot), ("runtime Python", self.python)):
            if not path.exists():
                raise FileNotFoundError(f"{label} does not exist: {path}")
        if self.source == "cooperscene":
            for label, path in (
                ("Gaussian splat", self.splat_path),
                ("localization transform", self.localization_transform),
            ):
                if path is None or not path.is_file():
                    raise FileNotFoundError(f"{label} does not exist: {path}")
            if self.stop_at_overlap and (
                self.overlap_manifest is None or not self.overlap_manifest.is_file()
            ):
                raise FileNotFoundError(
                    f"overlap manifest does not exist: {self.overlap_manifest}"
                )
        if self.viewer and self.viewer_mode == "rerun" and self.spawn_viewer:
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
            "source": self.source,
            "split": self.split,
            "agent": self.agent,
            "viewer_mode": self.viewer_mode if self.viewer else "none",
            "version": self.version,
            "sam3_backend": "offline" if self.offline_mask_root else "sam3.1",
            "precision": "fp16" if self.fp16 else "default",
            "sam3_precision": self.sam3_precision,
            "sam3d_precision": self.sam3d_precision,
            "sam3_device": self.sam3_device,
            "sam3d_device": self.sam3d_device,
            "residency_mode": self._residency_mode(),
            "runtime_mode": self.runtime_mode,
            "gpu_residency_lock": (
                str(self.gpu_residency_lock)
                if self.runtime_mode == "take_turns"
                else None
            ),
            "sam3_prompt_mode": self.sam3_prompt_mode,
            "sam3_mask_policy": self.sam3_mask_policy,
            "sam3d_priority_policy": self.sam3d_priority_policy,
            "sam3d_coalesce_ms": self.sam3d_coalesce_ms,
            "sam3d_stage1_inference_steps": self.sam3d_stage1_inference_steps,
            "sam3d_stage2_inference_steps": self.sam3d_stage2_inference_steps,
            "sam3d_pointmap_cache_size": self.sam3d_pointmap_cache_size,
            "stable_seconds": self.stable_seconds,
            "maximum_gap_seconds": self.maximum_gap_seconds,
            "maximum_admission_batch": self.maximum_admission_batch,
            "minimum_projected_area_px": self.minimum_projected_area_px,
            "minimum_visible_fraction": self.minimum_visible_fraction,
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

    def _residency_mode(self) -> str:
        sam3_gpu = self.sam3_device.casefold().startswith("cuda")
        sam3d_gpu = self.sam3d_device.casefold().startswith("cuda")
        if sam3_gpu and sam3d_gpu:
            return "both_gpu"
        if sam3_gpu:
            return "sam3_gpu_sam3d_cpu"
        if sam3d_gpu:
            return "sam3_cpu_sam3d_gpu"
        return "both_cpu"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataroot", type=Path, required=True)
    parser.add_argument("--source", choices=("nuscenes", "cooperscene"), default="nuscenes")
    parser.add_argument("--scene", default="scene-0061")
    parser.add_argument("--version", default="v1.0-mini")
    parser.add_argument("--split", default="train")
    parser.add_argument("--agent", choices=("1", "2", "3"), default="1")
    parser.add_argument("--splat", type=Path)
    parser.add_argument("--localization-transform", type=Path)
    parser.add_argument("--overlap-manifest", type=Path)
    parser.add_argument("--stop-at-overlap", action="store_true")
    parser.add_argument("--viewer-mode", choices=("rerun", "composited"))
    parser.add_argument("--render-downsample", type=int, default=2)
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
        "--sam3-precision", choices=("default", "fp16", "nf4")
    )
    parser.add_argument(
        "--sam3d-precision", choices=("default", "fp16", "nf4")
    )
    parser.add_argument("--sam3-device", default="cuda")
    parser.add_argument("--sam3d-device", default="cuda:0")
    parser.add_argument(
        "--runtime-mode",
        choices=("resident", "take_turns"),
        default="resident",
    )
    parser.add_argument(
        "--sam3-prompt-mode",
        choices=("serial_grounding", "interactive_batch"),
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
    parser.add_argument("--sam3d-pointmap-cache-size", type=int)
    parser.add_argument("--stable-seconds", type=float)
    parser.add_argument("--maximum-gap-seconds", type=float, default=0.25)
    parser.add_argument("--maximum-admission-batch", type=int, default=5)
    parser.add_argument("--minimum-projected-area-px", type=float)
    parser.add_argument("--minimum-visible-fraction", type=float)
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
        sam3_precision=args.sam3_precision,
        sam3d_precision=args.sam3d_precision,
        sam3_prompt_mode=args.sam3_prompt_mode,
        sam3_mask_policy=args.sam3_mask_policy,
        sam3d_priority_policy=args.sam3d_priority_policy,
        sam3d_coalesce_ms=args.sam3d_coalesce_ms,
        sam3d_stage1_inference_steps=args.sam3d_stage1_inference_steps,
        sam3d_stage2_inference_steps=args.sam3d_stage2_inference_steps,
        sam3d_pointmap_cache_size=args.sam3d_pointmap_cache_size,
        source=args.source,
        split=args.split,
        agent=args.agent,
        splat_path=args.splat,
        localization_transform=args.localization_transform,
        overlap_manifest=args.overlap_manifest,
        stop_at_overlap=args.stop_at_overlap,
        viewer_mode=args.viewer_mode,
        render_downsample=args.render_downsample,
        stable_seconds=args.stable_seconds,
        maximum_gap_seconds=args.maximum_gap_seconds,
        maximum_admission_batch=args.maximum_admission_batch,
        minimum_projected_area_px=args.minimum_projected_area_px,
        minimum_visible_fraction=args.minimum_visible_fraction,
        sam3_device=args.sam3_device,
        sam3d_device=args.sam3d_device,
        runtime_mode=args.runtime_mode,
    )
    raise SystemExit(supervisor.run())


if __name__ == "__main__":
    main()

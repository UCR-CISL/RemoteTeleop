from pathlib import Path

import pytest

from src.realtime.supervisor import RealtimePipelineSupervisor, RuntimeEndpoints


def _supervisor(tmp_path, **kwargs):
    values = dict(
        project_root=tmp_path,
        dataroot=tmp_path / "data",
        scene="1",
        version="v1.0-mini",
        output_root=tmp_path / "out",
        python=Path(".venv/bin/python"),
        sam3d_config=Path("checkpoints/hf/pipeline.yaml"),
        endpoints=RuntimeEndpoints(health="inproc://supervisor-test-health"),
        viewer=False,
    )
    values.update(kwargs)
    return RealtimePipelineSupervisor(**values)


def test_cooperscene_commands_select_compositor_replay_and_stable_admission(tmp_path):
    supervisor = _supervisor(
        tmp_path,
        source="cooperscene",
        viewer=True,
        splat_path=tmp_path / "map.spz",
        localization_transform=tmp_path / "localization.json",
        overlap_manifest=tmp_path / "overlap.json",
    )
    try:
        assert "src.realtime.cooperscene_replay" in supervisor._replay_command()
        assert "--stop-at-overlap" not in supervisor._replay_command()
        assert "src.realtime.composited_camera_process" in supervisor._viewer_command()
        mask = supervisor._mask_command()
        assert mask[mask.index("--stable-seconds") + 1] == "2.0"
        assert mask[mask.index("--maximum-admission-batch") + 1] == "5"
        assert mask[mask.index("--prompt-mode") + 1] == "serial_grounding"
        sam3d = supervisor._sam3d_command()
        assert sam3d[sam3d.index("--pointmap-cache-size") + 1] == "5"
    finally:
        supervisor.close()


def test_cooperscene_overlap_cutoff_is_explicit(tmp_path):
    supervisor = _supervisor(
        tmp_path,
        source="cooperscene",
        splat_path=tmp_path / "map.spz",
        localization_transform=tmp_path / "localization.json",
        overlap_manifest=tmp_path / "overlap.json",
        stop_at_overlap=True,
    )
    try:
        command = supervisor._replay_command()
        assert "--overlap-manifest" in command
        assert "--stop-at-overlap" in command
    finally:
        supervisor.close()


def test_nuscenes_defaults_keep_immediate_admission_and_rerun(tmp_path):
    supervisor = _supervisor(tmp_path)
    try:
        assert "src.realtime.nuscenes_replay" in supervisor._replay_command()
        assert "src.realtime.viewer_process" in supervisor._viewer_command()
        mask = supervisor._mask_command()
        assert mask[mask.index("--stable-seconds") + 1] == "0.0"
        assert mask[mask.index("--minimum-projected-area-px") + 1] == "0.0"
    finally:
        supervisor.close()


def test_worker_devices_are_forwarded_and_reported(tmp_path):
    supervisor = _supervisor(
        tmp_path,
        sam3_device="cpu",
        sam3d_device="cuda:0",
        fp16=True,
    )
    try:
        mask = supervisor._mask_command()
        sam3d = supervisor._sam3d_command()
        assert mask[mask.index("--device") + 1] == "cpu"
        assert sam3d[sam3d.index("--device") + 1] == "cuda:0"
        assert "--precision" not in mask
        assert sam3d[sam3d.index("--precision") + 1] == "fp16"
        assert supervisor._residency_mode() == "sam3_cpu_sam3d_gpu"
        supervisor._write_admission_report("configured")
        report = (tmp_path / "out" / "admission.json").read_text(encoding="utf-8")
        assert '"sam3_device": "cpu"' in report
        assert '"sam3d_device": "cuda:0"' in report
        assert '"sam3_precision": "default"' in report
        assert '"sam3d_precision": "fp16"' in report
    finally:
        supervisor.close()


def test_independent_nf4_precisions_are_forwarded_and_reported(tmp_path):
    supervisor = _supervisor(
        tmp_path,
        sam3_precision="default",
        sam3d_precision="nf4",
    )
    try:
        assert "--precision" not in supervisor._mask_command()
        sam3d = supervisor._sam3d_command()
        assert sam3d[sam3d.index("--precision") + 1] == "nf4"
        supervisor._write_admission_report("configured")
        report = (tmp_path / "out" / "admission.json").read_text(encoding="utf-8")
        assert '"sam3_precision": "default"' in report
        assert '"sam3d_precision": "nf4"' in report
    finally:
        supervisor.close()


def test_take_turn_runtime_forwards_one_shared_residency_lock(tmp_path):
    supervisor = _supervisor(tmp_path, runtime_mode="take_turns")
    try:
        mask = supervisor._mask_command()
        sam3d = supervisor._sam3d_command()
        mask_lock = mask[mask.index("--gpu-residency-lock") + 1]
        sam3d_lock = sam3d[sam3d.index("--gpu-residency-lock") + 1]

        assert mask_lock == sam3d_lock
        assert mask_lock.endswith("out/runtime/cuda-model.lock")
        assert mask[mask.index("--warmup-iterations") + 1] == "1"
        supervisor._write_admission_report("configured")
        report = (tmp_path / "out" / "admission.json").read_text(encoding="utf-8")
        assert '"runtime_mode": "take_turns"' in report
    finally:
        supervisor.close()


@pytest.mark.parametrize(
    ("sam3_device", "sam3d_device"),
    (("cpu", "cuda:0"), ("cuda", "cpu")),
)
def test_take_turn_runtime_requires_two_cuda_models(
    tmp_path, sam3_device, sam3d_device
):
    with pytest.raises(ValueError, match="take_turns requires CUDA"):
        _supervisor(
            tmp_path,
            runtime_mode="take_turns",
            sam3_device=sam3_device,
            sam3d_device=sam3d_device,
        )


def test_sam3d_cpu_is_capability_gated_before_process_launch(tmp_path):
    supervisor = _supervisor(
        tmp_path,
        sam3_device="cuda",
        sam3d_device="cpu",
    )
    try:
        with pytest.raises(ValueError, match="CUDA-only"):
            supervisor._validate()
        assert supervisor._residency_mode() == "sam3_gpu_sam3d_cpu"
    finally:
        supervisor.close()


def test_unknown_sam3_device_is_rejected_before_launch(tmp_path):
    supervisor = _supervisor(tmp_path, sam3_device="accelerator")
    try:
        with pytest.raises(ValueError, match="SAM3 device"):
            supervisor._validate()
    finally:
        supervisor.close()


def test_sam3d_cpu_run_writes_failed_capability_report(tmp_path):
    supervisor = _supervisor(tmp_path, sam3d_device="cpu")
    with pytest.raises(ValueError, match="sparse operators"):
        supervisor.run()
    report = (tmp_path / "out" / "admission.json").read_text(encoding="utf-8")
    assert '"status": "failed"' in report
    assert '"residency_mode": "sam3_gpu_sam3d_cpu"' in report
    assert "CUDA-only" in report

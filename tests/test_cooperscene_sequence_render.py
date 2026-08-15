import json
import numpy as np

import src.localization.cooperscene as cooperscene

from src.localization.cooperscene import CooperSceneFrame
from src.localization.cooperscene_sequence_render import (
    CooperSceneSequenceRenderConfig, CooperSceneSequenceRenderer, _mark_low_overlap,
)
from src.localization.gaussian_map import GaussianMap
from src.viz.localization_visualizer import CameraRender


class _Sequence:
    def __init__(self, frames): self.frames = frames
    def __len__(self): return len(self.frames)
    def __getitem__(self, index): return self.frames[index]


class _Dataset:
    def __init__(self, frames): self.frames = frames
    def sequence(self, split, scenario, agent): return _Sequence(self.frames)


def _frame(frame_id, x):
    pose = np.eye(4); pose[0, 3] = x
    # Along +X is in front of CooperScene camera0.
    points = np.array([[5, 0, 0, 1], [5.1, 0, 0, 1]], dtype=np.float32)
    return CooperSceneFrame("train", "1", "1", str(frame_id), None, None, points, pose)


def _map():
    means = np.array([[5, 0, 0], [5.1, 0, 0]], dtype=np.float32)
    return GaussianMap(means, np.zeros((2, 4), np.float32), np.ones((2, 3), np.float32),
                       np.ones(2, np.float32), np.zeros((2, 1, 3), np.float32), 0, False)


def _config(tmp_path, **kwargs):
    transform = tmp_path / "transform.json"
    transform.write_text(json.dumps({"gaussian_T_cooperscene": np.eye(4).tolist()}))
    values = dict(data_root=tmp_path, splat_path=tmp_path / "map.spz", transform_path=transform,
                  output_dir=tmp_path / "out", overlap_point_stride=1, overlap_map_stride=1,
                  baseline_frames=1, minimum_frustum_points=1, stop_after_low_overlap=2,
                  render_device="cpu")
    values.update(kwargs)
    return CooperSceneSequenceRenderConfig(**values)


def test_marks_first_frame_of_configured_consecutive_low_run():
    records = [{"frustum_point_count": 2, "fine_ratio": fine, "median_m": median,
                "low_overlap": False} for fine, median in
               [(1, .1), (.1, 1), (1, .1), (.1, 1), (.1, 1), (.1, 1)]]
    assert _mark_low_overlap(records, .5, .5, 1, 2) == 3
    assert records[-1]["low_overlap"]


def test_prepass_excludes_confirmation_lows_from_rendering(tmp_path):
    frames = [_frame("1", 0), _frame("2", 10), _frame("3", 0), _frame("4", 10), _frame("5", 10)]
    rendered = []
    def renderer(*args, **kwargs):
        rendered.append(args[-1]); return CameraRender(args[-1], "cpu-projective-splats")
    result = CooperSceneSequenceRenderer(
        _config(tmp_path, stop_at_overlap=True), dataset_factory=lambda _: _Dataset(frames), map_loader=lambda _: _map(),
        camera_renderer=renderer).run()
    assert result["first_low_frame_id"] == "4"
    assert result["rendered_count"] == 3
    assert result["stop_reason"] == "consecutive_low_overlap"
    assert (tmp_path / "out" / "overlap.csv").is_file()


def test_default_renders_all_frames_without_overlap_diagnostics(tmp_path):
    frames = [_frame("1", 0), _frame("2", 10), _frame("3", 10)]
    rendered = []

    def renderer(*args, **kwargs):
        rendered.append(args[-1])
        return CameraRender(args[-1], "cpu-projective-splats")

    result = CooperSceneSequenceRenderer(
        _config(tmp_path),
        dataset_factory=lambda _: _Dataset(frames),
        map_loader=lambda _: _map(),
        camera_renderer=renderer,
    ).run()

    assert result["first_low_frame_id"] is None
    assert result["criterion"]["enabled"] is False
    assert result["rendered_count"] == 3
    assert result["stop_reason"] == "end_of_sequence"
    assert not (tmp_path / "out" / "overlap.csv").exists()


def test_probe_only_never_calls_renderer(tmp_path):
    result = CooperSceneSequenceRenderer(
        _config(tmp_path, probe_only=True, max_frames=1),
        dataset_factory=lambda _: _Dataset([_frame("1", 0)]), map_loader=lambda _: _map(),
        camera_renderer=lambda *a, **k: (_ for _ in ()).throw(AssertionError())).run()
    assert result["rendered_count"] == 0


def test_cuda_request_rejects_cpu_fallback(tmp_path):
    with np.testing.assert_raises_regex(RuntimeError, "CUDA render was requested"):
        CooperSceneSequenceRenderer(
            _config(tmp_path, render_device="cuda", max_frames=1),
            dataset_factory=lambda _: _Dataset([_frame("1", 0)]), map_loader=lambda _: _map(),
            camera_renderer=lambda *a, **k: CameraRender(a[-1], "cpu-projective-splats")).run()


def test_default_metadata_render_reads_no_pcd_and_keeps_all_501_frames(
    tmp_path, monkeypatch
):
    sequence_root = tmp_path / "train" / "1" / "1"
    sequence_root.mkdir(parents=True)
    for frame_id in range(481260, 481761):
        (sequence_root / f"{frame_id}.yaml").write_text(
            f"lidar_pose: [{frame_id - 481260}, 0, 0, 0, 0, 0]\nvehicles: {{}}\n",
            encoding="utf-8",
        )
    monkeypatch.setattr(
        cooperscene, "read_ascii_xyzi_pcd",
        lambda _path: (_ for _ in ()).throw(AssertionError("PCD read")),
    )
    rendered = []

    def renderer(_splats, world_T_camera, _intrinsic, _size, output, **_kwargs):
        rendered.append((output.stem, float(world_T_camera[0, 3])))
        return CameraRender(output, "cpu-projective-splats")

    result = CooperSceneSequenceRenderer(
        _config(tmp_path), map_loader=lambda _path: object(), camera_renderer=renderer
    ).run()

    assert result["processed_count"] == result["rendered_count"] == 501
    assert [row["frame_id"] for row in result["frames"]] == [
        str(value) for value in range(481260, 481761)
    ]
    assert all(row["lidar_points_loaded"] is False for row in result["frames"])
    assert rendered[0][0].endswith("481260")
    assert rendered[-1][0].endswith("481760")

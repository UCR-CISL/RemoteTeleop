"""Stream camera renders along a CooperScene trajectory until map overlap is lost."""

from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import time
from typing import Callable

import numpy as np
from scipy.spatial import cKDTree

from src.localization.cooperscene import CooperSceneSequenceDataset
from src.localization.cooperscene_calibration import camera_T_lidar, camera_intrinsic, compose_map_T_camera
from src.localization.cooperscene_localization import _as_transform
from src.localization.gaussian_map import GaussianMap, load_spz_v3
from src.viz.localization_visualizer import CameraRender, render_gaussian_camera_view


@dataclass(frozen=True)
class CooperSceneSequenceRenderConfig:
    data_root: Path
    splat_path: Path
    transform_path: Path
    output_dir: Path
    split: str = "train"
    scenario: str = "1"
    agent: str = "1"
    frame_stride: int = 1
    max_frames: int | None = None
    overlap_point_stride: int = 4
    overlap_map_stride: int = 8
    baseline_frames: int = 30
    fine_distance_m: float = 0.20
    broad_distance_m: float = 0.50
    minimum_frustum_points: int = 1000
    minimum_fine_ratio: float = 0.25
    fine_baseline_fraction: float = 0.50
    maximum_median_m: float = 0.35
    median_baseline_factor: float = 1.75
    stop_after_low_overlap: int = 5
    stop_at_overlap: bool = False
    probe_only: bool = False
    render_device: str | None = "cuda"
    render_downsample: int = 2


class CooperSceneSequenceRenderer:
    """Render metadata frames, adding a LiDAR overlap prepass only on request."""

    def __init__(self, config: CooperSceneSequenceRenderConfig, *,
                 dataset_factory: Callable[[Path], CooperSceneSequenceDataset] | None = None,
                 map_loader: Callable[[Path], GaussianMap] = load_spz_v3,
                 camera_renderer: Callable[..., CameraRender | Path] = render_gaussian_camera_view) -> None:
        integers = (config.frame_stride, config.overlap_point_stride, config.overlap_map_stride,
                    config.baseline_frames, config.minimum_frustum_points,
                    config.stop_after_low_overlap, config.render_downsample)
        if min(integers) < 1:
            raise ValueError("strides, counts, and render_downsample must be positive")
        if config.max_frames is not None and config.max_frames < 1:
            raise ValueError("max_frames must be positive")
        if min(config.fine_distance_m, config.broad_distance_m) <= 0:
            raise ValueError("overlap distances must be positive")
        if config.fine_distance_m > config.broad_distance_m:
            raise ValueError("fine_distance_m must not exceed broad_distance_m")
        if not 0 <= config.minimum_fine_ratio <= 1 or config.fine_baseline_fraction < 0:
            raise ValueError("overlap ratio thresholds must be valid")
        if config.render_device not in (None, "cpu", "cuda"):
            raise ValueError("render_device must be None, 'cpu', or 'cuda'")
        self.config = config
        self._dataset_factory = dataset_factory
        self._map_loader = map_loader
        self._camera_renderer = camera_renderer

    def run(self) -> dict:
        transform = _load_transform(self.config.transform_path)
        splats = (
            self._map_loader(self.config.splat_path)
            if self.config.stop_at_overlap or not self.config.probe_only else None
        )
        dataset = (
            self._dataset_factory(self.config.data_root)
            if self._dataset_factory is not None
            else (
                CooperSceneSequenceDataset(self.config.data_root, cache_frames=False)
                if self.config.stop_at_overlap
                else CooperSceneSequenceDataset.metadata_only(
                    self.config.data_root, cache_frames=False
                )
            )
        )
        sequence = dataset.sequence(self.config.split, self.config.scenario, self.config.agent)
        indices = list(range(0, len(sequence), self.config.frame_stride))
        if self.config.max_frames is not None:
            indices = indices[:self.config.max_frames]

        loaded_frames = None
        if self.config.stop_at_overlap:
            assert splats is not None
            tree = cKDTree(_overlap_proxy(splats, self.config.overlap_map_stride))
            records = self._overlap_prepass(sequence, indices, transform, tree)
            baseline = _baseline(records, self.config.baseline_frames)
            fine_gate = max(self.config.minimum_fine_ratio,
                            self.config.fine_baseline_fraction * baseline["fine_ratio"])
            median_gate = max(self.config.maximum_median_m,
                              self.config.median_baseline_factor * baseline["median_m"])
            if baseline["fine_ratio"] < 0.30 or baseline["median_m"] > 0.30:
                raise RuntimeError(
                    "initial overlap baseline is unhealthy; verify gaussian_T_cooperscene "
                    f"(fine={baseline['fine_ratio']:.3f}, median={baseline['median_m']:.3f} m)"
                )
            first_low = _mark_low_overlap(
                records, fine_gate, median_gate, self.config.minimum_frustum_points,
                self.config.stop_after_low_overlap,
            )
            render_limit = first_low if first_low is not None else len(records)
        else:
            baseline = fine_gate = median_gate = first_low = None
            records, loaded_frames = self._metadata_records(sequence, indices)
            render_limit = len(records)
        rendered_count = 0
        if not self.config.probe_only:
            frames_dir = self.config.output_dir / "frames"
            frames_dir.mkdir(parents=True, exist_ok=True)
            assert splats is not None
            for render_index, record in enumerate(records[:render_limit]):
                if loaded_frames is None:
                    load_started = time.perf_counter()
                    frame = sequence[record["sequence_index"]]
                    record["render_frame_load_ms"] = (
                        time.perf_counter() - load_started
                    ) * 1_000.0
                else:
                    frame = loaded_frames[render_index]
                output = frames_dir / f"{rendered_count:06d}_{frame.frame_id}.png"
                rendered = self._camera_renderer(
                    splats, compose_map_T_camera(transform @ frame.map_T_lidar, camera_T_lidar(self.config.agent)),
                    camera_intrinsic(self.config.agent), (1920, 1200), output,
                    downsample=self.config.render_downsample, device=self.config.render_device,
                )
                backend = getattr(rendered, "backend", None)
                if self.config.render_device == "cuda" and backend != "gsplat-cuda":
                    raise RuntimeError("CUDA render was requested but gsplat-CUDA could not allocate or initialize")
                record.update(rendered=True, render_path=str(output.relative_to(self.config.output_dir)))
                if backend is not None:
                    record["render_backend"] = backend
                rendered_count += 1

        stop_reason = "consecutive_low_overlap" if self.config.stop_at_overlap and first_low is not None else (
            "max_frames" if self.config.max_frames is not None and len(indices) < len(sequence) else "end_of_sequence"
        )
        result = {
            "config": {k: str(v) if isinstance(v, Path) else v for k, v in asdict(self.config).items()},
            "criterion": {
                "enabled": self.config.stop_at_overlap,
                "region": "front_camera_frustum_z_2_to_60_m",
                "baseline": baseline,
                "fine_gate": fine_gate,
                "median_gate_m": median_gate,
                "low_definition": "fine_ratio < fine_gate AND median_m > median_gate_m",
                "consecutive_frames": self.config.stop_after_low_overlap,
            },
            "processed_count": len(records), "rendered_count": rendered_count,
            "first_low_frame_id": records[first_low]["frame_id"] if first_low is not None else None,
            "stop_reason": stop_reason, "frames": records,
        }
        self.config.output_dir.mkdir(parents=True, exist_ok=True)
        (self.config.output_dir / "render_manifest.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        if self.config.stop_at_overlap:
            _write_csv(self.config.output_dir / "overlap.csv", records)
        return result

    def _metadata_records(self, sequence, indices):
        records = []
        frames = []
        for sequence_index in indices:
            started = time.perf_counter()
            frame = sequence[sequence_index]
            elapsed_ms = (time.perf_counter() - started) * 1_000.0
            frames.append(frame)
            records.append({
                "frame_id": frame.frame_id,
                "sequence_index": sequence_index,
                "frame_metadata_load_ms": elapsed_ms,
                "lidar_points_loaded": frame.has_lidar_points,
                "rendered": False,
            })
        return records, frames

    def _overlap_prepass(self, sequence, indices, transform, tree) -> list[dict]:
        intrinsic = camera_intrinsic(self.config.agent)
        camera_from_lidar = camera_T_lidar(self.config.agent)
        records = []
        for sequence_index in indices:
            load_started = time.perf_counter()
            frame = sequence[sequence_index]
            load_ms = (time.perf_counter() - load_started) * 1_000.0
            raw = frame.require_lidar_points()[::self.config.overlap_point_stride, :3]
            frustum = _front_camera_points(raw, camera_from_lidar, intrinsic, (1920, 1200))
            records.append({
                "frame_id": frame.frame_id, "sequence_index": sequence_index,
                "frame_load_with_pcd_ms": load_ms,
                "lidar_points_loaded": True,
                **_overlap_metrics(frustum, transform @ frame.map_T_lidar, tree,
                                   self.config.fine_distance_m, self.config.broad_distance_m),
                "rendered": False, "low_overlap": False,
            })
        return records


def _front_camera_points(points, camera_from_lidar, intrinsic, image_size):
    points = np.asarray(points, dtype=np.float64)
    camera = points @ camera_from_lidar[:3, :3].T + camera_from_lidar[:3, 3]
    z = camera[:, 2]
    uvw = camera @ intrinsic.T
    uv = uvw[:, :2] / np.maximum(z[:, None], 1e-8)
    width, height = image_size
    keep = np.isfinite(camera).all(axis=1) & (z >= 2.0) & (z <= 60.0)
    keep &= (uv[:, 0] >= 0) & (uv[:, 0] < width) & (uv[:, 1] >= 0) & (uv[:, 1] < height)
    return points[keep]


def _overlap_metrics(points, map_T_lidar, tree, fine_distance, broad_distance):
    if len(points) == 0:
        return {"frustum_point_count": 0, "fine_ratio": 0.0, "broad_ratio": 0.0,
                "median_m": float("inf"), "p90_m": float("inf")}
    mapped = points @ map_T_lidar[:3, :3].T + map_T_lidar[:3, 3]
    distances, _ = tree.query(mapped, k=1, workers=1)
    return {"frustum_point_count": len(points), "fine_ratio": float(np.mean(distances <= fine_distance)),
            "broad_ratio": float(np.mean(distances <= broad_distance)),
            "median_m": float(np.median(distances)), "p90_m": float(np.percentile(distances, 90))}


def _baseline(records, count):
    if not records:
        raise ValueError("sequence contains no frames")
    initial = records[:min(count, len(records))]
    return {key: float(np.median([r[key] for r in initial])) for key in
            ("fine_ratio", "broad_ratio", "median_m", "p90_m")}


def _mark_low_overlap(records, fine_gate, median_gate, minimum_points, consecutive_frames):
    run = 0
    run_start = None
    first_confirmed = None
    for index, record in enumerate(records):
        low = record["frustum_point_count"] < minimum_points or (
            record["fine_ratio"] < fine_gate and record["median_m"] > median_gate
        )
        record["low_overlap"] = low
        if low:
            run_start = index if run == 0 else run_start
            run += 1
            if run >= consecutive_frames and first_confirmed is None:
                first_confirmed = run_start
        else:
            run, run_start = 0, None
    return first_confirmed


def _load_transform(path):
    if path.suffix == ".npz":
        with np.load(path) as archive:
            for key in ("gaussian_T_cooperscene", "map_T_source", "transform"):
                if key in archive: return _as_transform(archive[key], path)
    elif path.suffix == ".json":
        payload = json.loads(path.read_text(encoding="utf-8"))
        for key in ("gaussian_T_cooperscene", "map_T_source", "transform"):
            if key in payload: return _as_transform(payload[key], path)
    else: raise ValueError("transform_path must end in .npz or .json")
    raise ValueError(f"no Gaussian-to-CooperScene transform found in {path}")


def _overlap_proxy(splats, stride):
    means, opacities = np.asarray(splats.means, float), np.asarray(splats.opacities, float)
    keep = np.isfinite(means).all(axis=1) & np.isfinite(opacities) & (opacities > .05)
    proxy = means[keep][::stride]
    if not len(proxy): raise ValueError("Gaussian map has no finite opaque centers")
    return proxy


def _write_csv(path, records):
    fields = ("frame_id", "sequence_index", "frustum_point_count", "fine_ratio", "broad_ratio",
              "median_m", "p90_m", "low_overlap", "rendered")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader(); writer.writerows(records)

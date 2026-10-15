"""Offline single-frame evaluation of shape-fitted, color-tinted vehicle proxies on CooperScene.

For each eval frame t, every GT vehicle in the camera view accumulates its in-box LiDAR returns
over the previous K native frames (GT poses, GT boxes snapped onto the LiDAR ground) through
ObjectPointAccumulator, is refitted after every scan by CarShapeFitter (warm-started from the
last accepted fit, like a live per-poll refit), and is colored by VehicleColorEstimator on those
K frames' camera images.  Frame t is held out: its points score the fitted and default profiles.
Frame t is then rendered through the gsplat compositor twice (default primitive vs. hints).
No MCAP / split-system replay is involved.
"""
from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass, field, replace
from pathlib import Path

import cv2
import numpy as np

from src.deployment.lidar_ground_snap import LidarGroundSnapper
from src.deployment.vehicle_color import (SparseDepthBuffer, VehicleColorConfig, VehicleColorEstimator,
                                          _projected_box_xyxy, project_points_to_image)
from src.deployment.vehicle_shape import CarShapeFitter, ObjectPointAccumulator, _box_mask, surface_distance
from src.localization.cooperscene import CooperSceneSequenceDataset
from src.localization.cooperscene_calibration import camera_intrinsic, camera_T_lidar
from src.viz.primitive_vehicle import CarShapeParams

ROOT = Path(__file__).resolve().parents[3]
MAIN_REPO = Path("/home/jyue86/Documents/CISL-Projects/RemoteTeleop")
DATA_ROOT = Path("/mnt/bcc-data/data/CooperScene")
MAP_PLY = MAIN_REPO / "artifacts/reference-map-20261011/riverside_iowa.ply"
ALIGNMENT = MAIN_REPO / "artifacts/worktree-archive-20261011/alignment_artifacts/recipe-frame-search-full-20261004"
DEFAULT_OUTPUT = MAIN_REPO / "artifacts/pcd-vehicle-assets-20261015"
RENDER_SIZE = (960, 600)
FRAME_PERIOD_US = 100_000  # CooperScene frame ids advance by one per 10 Hz scan


@dataclass(frozen=True)
class EvalFrameSpec:
    split: str
    take: int
    agent: int
    frame_index: int
    frame_id: str
    scout_tracks: tuple[str, ...] = ()

    @property
    def key(self) -> str:
        return f"take{self.take:02d}_a{self.agent}_{self.frame_id}"

    @classmethod
    def load_all(cls, path: Path) -> list[EvalFrameSpec]:
        return [cls(item["split"], int(item["take"]), int(item["agent"]), int(item["frame_index"]),
                    str(item["frame_id"]), tuple(v["track_id"] for v in item.get("vehicles", ())))
                for item in json.loads(path.read_text())]


def _rigid_inverse(transform: np.ndarray) -> np.ndarray:
    inverse = np.eye(4)
    inverse[:3, :3] = transform[:3, :3].T
    inverse[:3, 3] = -transform[:3, :3].T @ transform[:3, 3]
    return inverse


def _transform(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


class SnappedFrame:
    """One native frame: LiDAR scan plus GT boxes (LiDAR frame) with bottoms snapped to the LiDAR ground."""

    def __init__(self, frame, snapper: LidarGroundSnapper, *, crop_range_m: float = 80.0) -> None:
        self.frame = frame
        self.frame_id = frame.frame_id
        self.timestamp_us = int(frame.frame_id) * FRAME_PERIOD_US
        points = frame.require_lidar_points()[:, :3].astype(np.float64)
        self.scan = points[np.isfinite(points).all(axis=1)]
        self.points = self.scan[np.linalg.norm(self.scan[:, :2], axis=1) <= crop_range_m]
        lidar_T_map = _rigid_inverse(frame.map_T_lidar)
        self.dims, raw = {}, {}
        for box in frame.vehicle_boxes:
            lidar_T_object = lidar_T_map @ box.map_T_object
            if np.linalg.norm(lidar_T_object[:2, 3]) <= crop_range_m:
                raw[box.track_id], self.dims[box.track_id] = lidar_T_object, np.asarray(box.dimensions_lwh)
        ids = list(raw)
        shifts = snapper.offsets(self.points, np.array([raw[i][:3, 3] for i in ids]).reshape(-1, 3),
                                 np.array([np.arctan2(raw[i][1, 0], raw[i][0, 0]) for i in ids]),
                                 np.array([self.dims[i] for i in ids]).reshape(-1, 3)) if ids else []
        self.snap_shift_m = dict(zip(ids, (float(s) for s in shifts)))
        self.lidar_T_object = {}
        for track_id, shift in self.snap_shift_m.items():
            snapped = raw[track_id].copy()
            snapped[2, 3] += shift
            self.lidar_T_object[track_id] = snapped
        self._local: dict[str, np.ndarray] = {}

    def is_ego(self, track_id: str) -> bool:
        """True for the recording vehicle's own GT box (its footprint contains the LiDAR origin)."""
        origin = _rigid_inverse(self.lidar_T_object[track_id])[:2, 3]
        return bool(np.all(np.abs(origin) <= self.dims[track_id][:2] / 2 + 0.3))

    def map_T_object(self, track_id: str) -> np.ndarray:
        return self.frame.map_T_lidar @ self.lidar_T_object[track_id]

    def _near(self, track_id: str, reach_m: float) -> np.ndarray:
        center = self.lidar_T_object[track_id][:2, 3]
        return np.flatnonzero(np.sum((self.points[:, :2] - center) ** 2, axis=1) <= reach_m ** 2)

    def object_points(self, track_id: str, *, margin_m: float = 0.15, exclusion_m: float = 0.2) -> np.ndarray:
        """Metric object-frame points inside the (inflated) box; points inside a neighbor's box
        inflated by ``exclusion_m`` are dropped (ambiguous ownership)."""
        if track_id in self._local:
            return self._local[track_id]
        half = self.dims[track_id] / 2
        near = self._near(track_id, float(np.linalg.norm(half[:2])) + margin_m + 0.5)
        local = _transform(self.points[near], _rigid_inverse(self.lidar_T_object[track_id]))
        inside = np.all(np.abs(local) <= half + margin_m, axis=1)
        near, local = near[inside], local[inside]
        world = self.points[near]
        keep = np.ones(len(near), bool)
        for other, other_T in self.lidar_T_object.items():
            if other == track_id or not len(near):
                continue
            other_half = self.dims[other] / 2
            if np.linalg.norm(other_T[:2, 3] - self.lidar_T_object[track_id][:2, 3]) > \
                    np.linalg.norm(half[:2]) + np.linalg.norm(other_half[:2]) + 1.0:
                continue
            other_local = _transform(world, _rigid_inverse(other_T))
            keep &= ~np.all(np.abs(other_local) <= other_half + exclusion_m, axis=1)
        self._local[track_id] = local[keep]
        return self._local[track_id]


class FrameLoader:
    """Native CooperScene frames (with LiDAR) plus GT-box ground snapping; small LRU cache."""

    def __init__(self, root: Path = DATA_ROOT, cache_size: int = 24) -> None:
        self.dataset = CooperSceneSequenceDataset(root, cache_frames=False)
        self.snapper = LidarGroundSnapper()
        self._sequences, self._cache, self.cache_size = {}, {}, cache_size

    def sequence(self, split: str, take: int, agent: int):
        key = (split, take, agent)
        if key not in self._sequences:
            self._sequences[key] = self.dataset.sequence(split, str(take), str(agent))
        return self._sequences[key]

    def load(self, split: str, take: int, agent: int, index: int) -> SnappedFrame:
        key = (split, take, agent, index)
        if key not in self._cache:
            if len(self._cache) >= self.cache_size:
                self._cache.pop(next(iter(self._cache)))
            self._cache[key] = SnappedFrame(self.sequence(split, take, agent)[index], self.snapper)
        return self._cache[key]


@dataclass
class VehicleResult:
    """Per-vehicle evaluation record; array fields feed the figures and are not serialized."""

    track_id: str
    range_m: float
    visible_fraction: float
    in_scout_set: bool
    dims_lwh: tuple[float, float, float]
    snap_shift_m: float
    history_frames: list[str] = field(default_factory=list)
    used_future_frames: bool = False
    raw_points: int = 0
    accumulator_voxels: int = 0
    evidence_voxels: int = 0
    n_fits: int = 0
    fit_latency_ms: list[float] = field(default_factory=list)
    accepted_final: bool = False
    accepted_steps: int = 0
    published_shape: CarShapeParams | None = None
    fit_rms_m: float = float("nan")
    cold_fit: dict = field(default_factory=dict)
    heldout_points: int = 0
    heldout_default: tuple[float, float] = (float("nan"), float("nan"))
    heldout_fitted: tuple[float, float] = (float("nan"), float("nan"))
    gt_motion_m: float = 0.0
    motion_front_sign: int = 0
    color_rgb: tuple[float, float, float] | None = None
    color_confidence: float = 0.0
    color_fused_observations: int = 0
    color_frames_observed: int = 0
    color_frames_confident: int = 0
    color_observations: list[dict] = field(default_factory=list)
    crop_xyxy_native: tuple[float, ...] | None = None
    target_body_L: float = float("nan")  # median L* of visible belt pixels in frame t
    target_road_L: float = float("nan")  # median L* of road-ring pixels around the car in frame t
    target_road_rgb: tuple[float, float, float] | None = None  # median road-ring RGB (illuminant cue)

    @property
    def road_balanced_rgb(self) -> tuple[float, float, float] | None:
        """Diagnostic: published color white-balanced so the road ring reads neutral gray (von Kries)."""
        if self.color_rgb is None or self.target_road_rgb is None or min(self.target_road_rgb) <= 0.02:
            return None
        road = np.asarray(self.target_road_rgb)
        return tuple(float(c) for c in np.clip(np.asarray(self.color_rgb) * road.mean() / road, 0, 1))
    # figure-only payloads
    accumulated_points: np.ndarray | None = None
    accumulated_weights: np.ndarray | None = None
    heldout_xyz: np.ndarray | None = None
    sample_uv_ok: np.ndarray | None = None
    sample_uv_rejected: np.ndarray | None = None

    def row(self, spec: EvalFrameSpec) -> dict:
        shape = self.published_shape
        sign = None if shape is None else int(shape.forward_sign)
        return dict(
            frame_key=spec.key, split=spec.split, take=spec.take, agent=spec.agent, frame_index=spec.frame_index,
            frame_id=spec.frame_id, track_id=self.track_id, range_m=round(self.range_m, 2),
            visible_fraction=round(self.visible_fraction, 2), in_scout_set=self.in_scout_set,
            dims_lwh=[round(v, 3) for v in self.dims_lwh], snap_shift_m=round(self.snap_shift_m, 3),
            history_frames=len(self.history_frames), used_future_frames=self.used_future_frames,
            raw_points=self.raw_points, accumulator_voxels=self.accumulator_voxels,
            evidence_voxels=self.evidence_voxels, n_fits=self.n_fits,
            fit_latency_ms_mean=_round(np.mean(self.fit_latency_ms)) if self.fit_latency_ms else None,
            fit_latency_ms_max=_round(np.max(self.fit_latency_ms)) if self.fit_latency_ms else None,
            accepted_final=self.accepted_final, accepted_steps=self.accepted_steps, shape_published=shape is not None,
            body_type=None if shape is None else shape.body_type, forward_sign=sign,
            fit_rms_m=_round(self.fit_rms_m), shape_vector=None if shape is None else [round(v, 4) for v in shape.to_vector()],
            cold_fit_accepted=self.cold_fit.get("accepted"), cold_fit_body_type=self.cold_fit.get("body_type"),
            cold_fit_forward_sign=self.cold_fit.get("forward_sign"), cold_fit_rms_m=_round(self.cold_fit.get("rms")),
            heldout_points=self.heldout_points,
            heldout_default_median_m=_round(self.heldout_default[0]), heldout_default_p90_m=_round(self.heldout_default[1]),
            heldout_fitted_median_m=_round(self.heldout_fitted[0]), heldout_fitted_p90_m=_round(self.heldout_fitted[1]),
            gt_motion_m=round(self.gt_motion_m, 2), motion_front_sign=self.motion_front_sign,
            sign_matches_gt_x=None if sign is None else sign == 1,
            sign_matches_motion=None if sign is None or self.motion_front_sign == 0 else sign == self.motion_front_sign,
            color_published=self.color_rgb is not None,
            color_rgb=None if self.color_rgb is None else [round(v, 3) for v in self.color_rgb],
            color_confidence=round(self.color_confidence, 3), color_fused_observations=self.color_fused_observations,
            color_frames_observed=self.color_frames_observed, color_frames_confident=self.color_frames_confident,
            target_road_rgb=None if self.target_road_rgb is None else [round(c, 3) for c in self.target_road_rgb],
            color_rgb_road_balanced=None if self.road_balanced_rgb is None else [round(c, 3) for c in self.road_balanced_rgb],
            target_body_L=_round(self.target_body_L, 1), target_road_L=_round(self.target_road_L, 1),
            color_L=None if self.color_rgb is None else round(float(cv2.cvtColor(
                np.float32(self.color_rgb).reshape(1, 1, 3), cv2.COLOR_RGB2Lab)[0, 0, 0]), 1),
            crop_xyxy_native=None if self.crop_xyxy_native is None else [round(v, 1) for v in self.crop_xyxy_native],
        )


def _round(value, digits: int = 4):
    return None if value is None or not np.isfinite(value) else round(float(value), digits)


class VehicleEvidencePipeline:
    """Accumulate, fit and color every target vehicle over a K-frame history (one frame spec at a time)."""

    def __init__(self, loader: FrameLoader, *, history: int = 10, fitter_kwargs: dict | None = None,
                 accumulator_kwargs: dict | None = None, color_config: VehicleColorConfig | None = None,
                 cold_fit: bool = True, default_incumbent: bool = False) -> None:
        self.loader, self.history = loader, history
        self.default_incumbent = default_incumbent
        self.fitter_kwargs, self.accumulator_kwargs = fitter_kwargs or {}, accumulator_kwargs or {}
        self.color_config = color_config or VehicleColorConfig()
        self.cold_fit = cold_fit

    def history_indices(self, spec: EvalFrameSpec) -> tuple[list[int], bool]:
        previous = list(range(max(0, spec.frame_index - self.history), spec.frame_index))
        if len(previous) >= self.history:
            return previous, False
        # Sequence start: fill with following frames (still disjoint from held-out frame t).
        count = len(self.loader.sequence(spec.split, spec.take, spec.agent))
        future = list(range(spec.frame_index + 1, min(count, spec.frame_index + 1 + self.history - len(previous))))
        return previous + future, bool(future)

    def run(self, spec: EvalFrameSpec, target: SnappedFrame, vehicles: list[VehicleResult]) -> None:
        agent = str(spec.agent)
        K, cam_T_lidar = camera_intrinsic(agent, "refined"), camera_T_lidar(agent, "refined")
        accumulator = ObjectPointAccumulator(**self.accumulator_kwargs)
        fitter = CarShapeFitter(**self.fitter_kwargs)
        estimator = VehicleColorEstimator(self.color_config)
        indices, used_future = self.history_indices(spec)
        by_id = {v.track_id: v for v in vehicles}
        for v in vehicles:
            v.used_future_frames = used_future
        first_center: dict[str, np.ndarray] = {}
        for index in indices:
            frame = self.loader.load(spec.split, spec.take, spec.agent, index)
            present = [t for t in by_id if t in frame.lidar_T_object]
            for track_id in present:
                v = by_id[track_id]
                first_center.setdefault(track_id, frame.map_T_object(track_id)[:3, 3])
                points = frame.object_points(track_id)
                dims = frame.dims[track_id]
                v.history_frames.append(frame.frame_id)
                v.raw_points += len(points)
                accumulator.add(track_id, points / dims, dims, frame.timestamp_us)
                normalized, weights = accumulator.points(track_id)
                start = time.perf_counter()
                previous = v.published_shape
                if previous is None and self.default_incumbent:
                    previous = CarShapeParams()  # variant: the default sedan, +x forward, is the incumbent
                fit = fitter.fit(normalized * dims, dims, previous, weights=weights)
                v.fit_latency_ms.append(1e3 * (time.perf_counter() - start))
                v.n_fits += 1
                v.accepted_final, v.fit_rms_m, v.evidence_voxels = fit.accepted, fit.cost, fit.evidence_voxels
                if fit.accepted:
                    v.accepted_steps += 1
                    v.published_shape = fit.params
            self._observe_colors(frame, present, by_id, estimator, K, cam_T_lidar)
        for track_id, v in by_id.items():
            dims = np.asarray(v.dims_lwh)
            normalized, weights = accumulator.points(track_id)
            v.accumulator_voxels = len(normalized)
            v.accumulated_points, v.accumulated_weights = normalized * dims, weights
            if self.cold_fit and len(normalized):
                cold = fitter.fit(normalized * dims, dims, None, weights=weights)
                v.cold_fit = dict(accepted=cold.accepted, body_type=cold.params.body_type,
                                  forward_sign=int(cold.params.forward_sign), rms=cold.cost)
            estimate = estimator.color(track_id)
            if estimate is not None:
                v.color_rgb, v.color_confidence = estimate.rgb, estimate.confidence
                v.color_fused_observations = estimate.observation_count
            if track_id in first_center:
                motion = target.map_T_object(track_id)[:3, 3] - first_center[track_id]
                v.gt_motion_m = float(np.linalg.norm(motion[:2]))
                along = float(motion @ target.map_T_object(track_id)[:3, 0])
                # >= ~0.8 m over the window (~3 km/h) counts as driving; assume driving forward.
                v.motion_front_sign = int(np.sign(along)) if v.gt_motion_m >= 0.8 and abs(along) >= 0.6 else 0
            self._held_out(target, v, fitter)

    def _observe_colors(self, frame: SnappedFrame, present, by_id, estimator, K, cam_T_lidar) -> None:
        if frame.frame.camera_path is None or not present:
            return
        image = cv2.imread(str(frame.frame.camera_path))
        if image is None:
            return
        height, width = image.shape[:2]
        buffer = SparseDepthBuffer(_transform(frame.scan, cam_T_lidar), K, (width, height),
                                   self.color_config.depth_cell_px)
        boxes = {}
        for track_id, lidar_T_object in frame.lidar_T_object.items():
            camera_T_object = cam_T_lidar @ lidar_T_object
            if camera_T_object[2, 3] <= 0 or frame.is_ego(track_id):
                continue  # the recording vehicle's own GT box would "occlude" the whole lower image
            xyxy = _projected_box_xyxy(camera_T_object, K, tuple(frame.dims[track_id]), (width, height))
            if xyxy is not None:
                boxes[track_id] = (float(np.linalg.norm(camera_T_object[:3, 3])), camera_T_object, xyxy)
        for track_id in present:
            if track_id not in boxes:
                continue
            distance, camera_T_object, _ = boxes[track_id]
            occluders = [xyxy for other, (d, _, xyxy) in boxes.items() if other != track_id and d < distance]
            observation = estimator.observe(track_id, image, K, camera_T_object, frame.object_points(track_id),
                                            frame.dims[track_id], occluder_xyxy=occluders, depth_buffer=buffer)
            v = by_id[track_id]
            if observation is not None:
                v.color_frames_observed += 1
                v.color_frames_confident += observation.confidence >= self.color_config.min_confidence
                v.color_observations.append(dict(frame_id=frame.frame_id, rgb=[round(c, 3) for c in observation.rgb],
                                                 confidence=round(observation.confidence, 3),
                                                 samples=observation.sample_count))

    def _held_out(self, target: SnappedFrame, v: VehicleResult, fitter: CarShapeFitter) -> None:
        dims = np.asarray(v.dims_lwh)
        points = target.object_points(v.track_id)
        points = points[_box_mask(points, dims, margin_m=fitter.margin_m, ground_clearance_m=fitter.ground_clearance_m)]
        v.heldout_xyz, v.heldout_points = points, len(points)
        if len(points) < 5:
            return

        def stats(shape: CarShapeParams) -> tuple[float, float]:
            local = np.column_stack([shape.forward_sign * points[:, 0], np.abs(points[:, 1]), points[:, 2] + dims[2] / 2])
            distance = np.abs(surface_distance(local, dims, shape))
            return float(np.median(distance)), float(np.percentile(distance, 90))

        v.heldout_default = stats(CarShapeParams())
        v.heldout_fitted = stats(v.published_shape) if v.published_shape is not None else v.heldout_default


class ProxyRenderHarness:
    """gsplat background + vehicle proxies at a GT-registered pose, one backend per agent."""

    def __init__(self, map_path: Path = MAP_PLY) -> None:
        from src.localization.gaussian_map import load_gaussian_map
        from src.viz.composited_camera import (CudaGaussianBufferRenderer, OpacityThresholdDepth,
                                               PyTorch3DObjectMeshRenderer)
        from src.viz.vehicle_proxy import VehicleProxyRenderer

        self.mesh = PyTorch3DObjectMeshRenderer(device="cuda")
        self.gaussians = CudaGaussianBufferRenderer(load_gaussian_map(map_path), occluder_depth=OpacityThresholdDepth())
        self.proxies = VehicleProxyRenderer(self.mesh)
        self._backends = {}

    def backend(self, agent: int):
        from src.viz.composited_camera import PoseOnlyCompositedCameraBackend, RemoteCameraConfig

        if agent not in self._backends:
            camera = RemoteCameraConfig.for_cooperscene_agent(agent, image_size=RENDER_SIZE)  # refined calibration
            self._backends[agent] = PoseOnlyCompositedCameraBackend(self.gaussians, camera, self.mesh,
                                                                    vehicle_proxies=self.proxies)
        return self._backends[agent]

    @staticmethod
    def splat_lidar_pose(spec: EvalFrameSpec) -> np.ndarray | None:
        path = ALIGNMENT / f"take{spec.take:02d}/take{spec.take:02d}-vehicle{spec.agent}/frame-{spec.frame_index:06d}.json"
        method = json.loads(path.read_text())["methods"]["xyz"]
        if not method.get("accepted"):
            return None
        return np.asarray(method["error"]["estimated_map_T_lidar"], dtype=np.float64).reshape(4, 4)

    def render(self, spec: EvalFrameSpec, world_T_lidar: np.ndarray, prompts) -> np.ndarray:
        from src.realtime.protocol import EgoPoseSample

        pose = EgoPoseSample(sequence=spec.frame_index, timestamp_us=int(spec.frame_id) * FRAME_PERIOD_US,
                             scene_generation="eval", frame_id=spec.frame_id,
                             world_T_ego=tuple(world_T_lidar.reshape(-1)))
        out = self.backend(spec.agent).render_pose(pose, tuple(prompts))
        return cv2.cvtColor((np.clip(out.rgb, 0, 1) * 255).astype(np.uint8), cv2.COLOR_RGB2BGR)


@dataclass
class FrameResult:
    spec: EvalFrameSpec
    vehicles: list[VehicleResult]
    real_bgr: np.ndarray
    render_default: np.ndarray
    render_fitted: np.ndarray
    world_T_camera: np.ndarray
    render_K: np.ndarray
    world_T_objects: dict[str, np.ndarray]
    dims: dict[str, np.ndarray]
    timings_s: dict[str, float]


class VehicleProxyEvaluation:
    """Run the full per-frame evaluation (evidence + held-out metrics + two renders)."""

    def __init__(self, pipeline: VehicleEvidencePipeline, renderer: ProxyRenderHarness | None,
                 *, max_range_m: float = 60.0, min_range_m: float = 3.0) -> None:
        self.pipeline, self.renderer = pipeline, renderer
        self.max_range_m, self.min_range_m = max_range_m, min_range_m

    def evaluate(self, spec: EvalFrameSpec) -> FrameResult | None:
        from src.realtime.protocol import BoxPrompt
        from src.reconstruction.models import VehicleDimensions
        from src.reconstruction.projection import project_vehicle_box

        started = time.perf_counter()
        splat_lidar = ProxyRenderHarness.splat_lidar_pose(spec)
        if splat_lidar is None:
            print(f"skip {spec.key}: no accepted splat alignment", flush=True)
            return None
        target = self.pipeline.loader.load(spec.split, spec.take, spec.agent, spec.frame_index)
        G = splat_lidar @ _rigid_inverse(target.frame.map_T_lidar)
        agent = str(spec.agent)
        cam_T_lidar = camera_T_lidar(agent, "refined")
        K_native = camera_intrinsic(agent, "refined")
        world_T_camera = splat_lidar @ _rigid_inverse(cam_T_lidar)
        K_render = K_native.copy()
        K_render[0] *= RENDER_SIZE[0] / 1920.0
        K_render[1] *= RENDER_SIZE[1] / 1200.0
        vehicles, world_T_objects = [], {}
        for track_id, lidar_T_object in target.lidar_T_object.items():
            world_T_object = G @ target.map_T_object(track_id)
            projected = project_vehicle_box(world_T_object=world_T_object, world_T_camera=world_T_camera,
                                            intrinsics=K_render, dimensions=VehicleDimensions(*target.dims[track_id]),
                                            image_size=RENDER_SIZE[::-1], minimum_area_px=16)
            if projected is None:
                continue
            camera_T_object = cam_T_lidar @ lidar_T_object
            distance = float(np.linalg.norm(camera_T_object[:3, 3]))
            if distance < self.min_range_m:
                continue
            world_T_objects[track_id] = world_T_object
            if distance > self.max_range_m:
                continue  # rendered with the default primitive in both variants, not evaluated
            v = VehicleResult(track_id, distance, float(projected.visible_fraction), track_id in spec.scout_tracks,
                              tuple(float(d) for d in target.dims[track_id]), target.snap_shift_m[track_id])
            v.crop_xyxy_native = tuple(float(c) for c in _projected_box_xyxy(camera_T_object, K_native,
                                                                          tuple(target.dims[track_id]), (1920, 1200)))
            vehicles.append(v)
        vehicles.sort(key=lambda v: v.range_m)
        t_load = time.perf_counter()
        self.pipeline.run(spec, target, vehicles)
        t_evidence = time.perf_counter()
        real = cv2.imread(str(target.frame.camera_path))
        self._target_samples(real, target, vehicles, K_native, cam_T_lidar)
        by_id = {v.track_id: v for v in vehicles}
        default_prompts, fitted_prompts = [], []
        for track_id, world_T_object in world_T_objects.items():
            projected = project_vehicle_box(world_T_object=world_T_object, world_T_camera=world_T_camera,
                                            intrinsics=K_render, dimensions=VehicleDimensions(*target.dims[track_id]),
                                            image_size=RENDER_SIZE[::-1], minimum_area_px=16)
            base = BoxPrompt(track_id=track_id, xyxy=tuple(map(float, projected.xyxy)),
                             dimensions_lwh=tuple(float(d) for d in target.dims[track_id]),
                             world_T_object=tuple(world_T_object.reshape(-1)), text_label="car")
            default_prompts.append(base)
            v = by_id.get(track_id)
            hints = {}
            if v is not None and v.published_shape is not None:
                hints["vehicle_shape"] = v.published_shape.to_vector()
            if v is not None and v.color_rgb is not None:
                hints.update(body_color_rgb=v.color_rgb, color_confidence=v.color_confidence)
            fitted_prompts.append(replace(base, **hints))
        renders = (None, None)
        if self.renderer is not None:
            renders = (self.renderer.render(spec, splat_lidar, default_prompts),
                       self.renderer.render(spec, splat_lidar, fitted_prompts))
        done = time.perf_counter()
        timings = dict(load=t_load - started, evidence=t_evidence - t_load, render=done - t_evidence)
        return FrameResult(spec, vehicles, real, *renders, world_T_camera, K_render, world_T_objects,
                           {t: target.dims[t] for t in world_T_objects}, timings)

    def _target_samples(self, image, target: SnappedFrame, vehicles, K, cam_T_lidar) -> None:
        """Belt-band target-frame points as the color estimator would sample them (for the figures)."""
        config = self.pipeline.color_config
        height, width = image.shape[:2]
        buffer = SparseDepthBuffer(_transform(target.scan, cam_T_lidar), K, (width, height), config.depth_cell_px)
        for v in vehicles:
            dims = np.asarray(v.dims_lwh)
            points = target.object_points(v.track_id)
            low, high = (-dims[2] / 2 + f * dims[2] for f in config.belt_band)
            points = points[(points[:, 2] >= low) & (points[:, 2] <= high)]
            camera_points = _transform(points, cam_T_lidar @ target.lidar_T_object[v.track_id])
            uv, depth, valid = project_points_to_image(camera_points, K, (width, height))
            uv, depth = uv[valid], depth[valid]
            visible = buffer.visible(uv, depth, config.depth_tolerance_m) if len(uv) else np.zeros(0, bool)
            v.sample_uv_ok, v.sample_uv_rejected = uv[visible], uv[~visible]
            v.target_body_L = _median_lightness(image, v.sample_uv_ok)
            # Exposure reference: road returns in a ring 0.3-1.5 m around the footprint, near its floor.
            object_T_lidar = _rigid_inverse(target.lidar_T_object[v.track_id])
            near = target._near(v.track_id, float(np.linalg.norm(dims[:2] / 2)) + 1.6)
            local = _transform(target.points[near], object_T_lidar)
            outside = np.maximum(np.abs(local[:, 0]) - dims[0] / 2, np.abs(local[:, 1]) - dims[1] / 2)
            ring = target.points[near][(outside > 0.3) & (outside < 1.5) & (np.abs(local[:, 2] + dims[2] / 2) < 0.2)]
            uv, depth, valid = project_points_to_image(_transform(ring, cam_T_lidar), K, (width, height))
            uv, depth = uv[valid], depth[valid]
            if len(uv):
                uv = uv[buffer.visible(uv, depth, config.depth_tolerance_m)]
            v.target_road_L = _median_lightness(image, uv)
            if len(uv) >= 8:
                road = image[np.rint(uv[:, 1]).astype(int), np.rint(uv[:, 0]).astype(int)][:, ::-1] / 255.0
                v.target_road_rgb = tuple(float(c) for c in np.median(road, axis=0))


def _median_lightness(image_bgr: np.ndarray, uv: np.ndarray) -> float:
    """Median CIE L* (0-100) of the image at the given pixel positions; NaN when fewer than 8."""
    if uv is None or len(uv) < 8:
        return float("nan")
    pixels = image_bgr[np.rint(uv[:, 1]).astype(int), np.rint(uv[:, 0]).astype(int)]
    lab = cv2.cvtColor(pixels.reshape(-1, 1, 3), cv2.COLOR_BGR2Lab).reshape(-1, 3).astype(float)
    return float(np.median(lab[:, 0]) * 100 / 255)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--frames", type=Path, default=Path(__file__).with_name("eval_frames.json"))
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--history", type=int, default=10, help="previous native frames accumulated (10 = ~1 s)")
    parser.add_argument("--limit", type=int, default=None, help="evaluate only the first N frames")
    parser.add_argument("--only", nargs="*", default=None, help="frame keys like take03_a1_496960")
    parser.add_argument("--fitter", type=json.loads, default={}, help="CarShapeFitter kwargs override (JSON)")
    parser.add_argument("--accumulator", type=json.loads, default={}, help="ObjectPointAccumulator kwargs (JSON)")
    parser.add_argument("--color", type=json.loads, default={}, help="VehicleColorConfig overrides (JSON)")
    parser.add_argument("--default-incumbent", action="store_true",
                        help="variant: first fit treats the default sedan (+x forward) as the incumbent, so a body "
                             "type or forward-sign change must win by the fitter's switch_margin")
    parser.add_argument("--baseline", type=Path, default=None,
                        help="earlier review directory: REVIEW.md gets a 'Changes since' comparison against it")
    parser.add_argument("--changes", type=Path, default=None,
                        help='JSON {"changes": [markdown lines], "callouts": [{frame_key, track_id, note}]} '
                             "describing the change and the vehicles to compare against --baseline")
    parser.add_argument("--no-render", action="store_true", help="metrics only (no gsplat renders / figures)")
    parser.add_argument("--rebuild-review", action="store_true",
                        help="only rewrite REVIEW.md from <output>/summary.json (+ variants/*/summary.json)")
    args = parser.parse_args()

    from scripts.investigation.vehicle_shape.review_report import ReviewReport

    if args.rebuild_review:
        ReviewReport.rebuild(args.output)
        return

    specs = EvalFrameSpec.load_all(args.frames)
    if args.only:
        specs = [s for s in specs if s.key in set(args.only)]
    specs = specs[:args.limit] if args.limit else specs
    settings = dict(history=args.history, fitter=args.fitter, accumulator=args.accumulator, color=args.color,
                    frames_file=str(args.frames), map=str(MAP_PLY), render_size=RENDER_SIZE,
                    calibration="refined", render=not args.no_render,
                    default_incumbent=args.default_incumbent,
                    baseline=None if args.baseline is None else str(args.baseline.resolve()),
                    **(json.loads(args.changes.read_text()) if args.changes else {}))
    pipeline = VehicleEvidencePipeline(FrameLoader(), history=args.history, fitter_kwargs=args.fitter,
                                       accumulator_kwargs=args.accumulator,
                                       color_config=VehicleColorConfig(**args.color),
                                       default_incumbent=args.default_incumbent)
    evaluation = VehicleProxyEvaluation(pipeline, None if args.no_render else ProxyRenderHarness())
    report = ReviewReport(args.output, settings, figures=not args.no_render, baseline=args.baseline,
                          callouts=settings.get("callouts"))
    for number, spec in enumerate(specs, 1):
        result = evaluation.evaluate(spec)
        if result is None:
            continue
        report.add(result)
        accepted = sum(v.published_shape is not None for v in result.vehicles)
        colored = sum(v.color_rgb is not None for v in result.vehicles)
        print(f"[{number}/{len(specs)}] {spec.key}: {len(result.vehicles)} vehicles, shape {accepted}, color {colored}, "
              + " ".join(f"{k}={t:.1f}s" for k, t in result.timings_s.items()), flush=True)
    report.finish()
    print("wrote", args.output)


if __name__ == "__main__":
    main()

"""Per-track vehicle shape and paint hints for BoxPrompts (vehicle side).

Composes the LiDAR shape accumulator/fitter and the camera color estimator
behind one bounded per-poll update. Inputs are the detector request's LiDAR
scan (LiDAR frame), the tracker's detection matches at that scan's timestamp
and, for color, the full-resolution image with the camera calibration used to
render it. Only compact hints leave this module.
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Sequence

import numpy as np

from src.common.vehicle_categories import vehicle_proxy_kind
from src.deployment.lidar_ground_snap import _BevGrid
from src.deployment.vehicle_color import SparseDepthBuffer, VehicleColorEstimator
from src.deployment.vehicle_shape import CarShapeFitter, ObjectPointAccumulator
from src.reconstruction.models import VehicleDimensions
from src.reconstruction.projection import project_vehicle_box
from src.viz.primitive_vehicle import CarShapeParams

# Thin or rider-dominated vehicles have no reliable body panels to sample.
_NO_PAINT_KINDS = frozenset({"motorcycle", "bicycle"})


@dataclass(frozen=True)
class VehicleAppearanceConfig:
    """Feature switches and work bounds; distances are metres."""

    shape: bool = True
    color: bool = True
    crop_margin_m: float = .15  # box padding when cropping a scan to a track
    neighbor_margin_m: float = .3  # drop returns this close to another track's box
    max_crop_points: int = 4096  # stride dense near-range crops down to this
    refit_growth: float = .25  # refit when accumulated voxels grow by this fraction ...
    refit_every_scans: int = 10  # ... or after this many accumulated scans
    max_fits_per_update: int = 2  # each fit costs ~25-60 ms ...
    fit_budget_ms: float = 60.  # ... and no further fit starts once this much was spent in one update
    max_color_tracks_per_update: int = 8
    min_param_change: float = .02  # fraction of a parameter's range that counts as a new shape
    freeze_after_stable_fits: int = 3
    unfreeze_dimension_change: float = .1

    def __post_init__(self) -> None:
        if (self.crop_margin_m < 0 or self.neighbor_margin_m < 0 or self.max_crop_points < 1
                or self.refit_growth <= 0 or self.refit_every_scans < 1 or self.max_fits_per_update < 0
                or self.fit_budget_ms < 0
                or self.max_color_tracks_per_update < 0 or self.min_param_change < 0
                or self.freeze_after_stable_fits < 1 or self.unfreeze_dimension_change <= 0):
            raise ValueError("invalid vehicle appearance configuration")


@dataclass(frozen=True)
class AppearanceCamera:
    """Full-resolution BGR image with the CV-axes calibration it is rendered with."""

    image_bgr: np.ndarray
    camera_intrinsic: np.ndarray  # 3x3
    camera_T_lidar: np.ndarray  # 4x4, CV camera axes


@dataclass
class _ShapeTrack:
    dimensions: np.ndarray
    scans_since_fit: int = 0
    fit_evidence: int = 0
    last_fit_tick: int = -1
    fitted: CarShapeParams | None = None
    published: CarShapeParams | None = None
    stable_fits: int = 0
    frozen_dimensions: np.ndarray | None = None


class VehicleAppearanceEstimator:
    """Bounded per-track shape fitting and paint color estimation.

    Only detections matched to confirmed tracks feed evidence, cropped with the
    corrected track pose at the scan timestamp; coasted tracks contribute
    nothing. Shapes are refit round-robin (oldest fit first), at most
    ``max_fits_per_update`` per update within ``fit_budget_ms``, published only
    on a meaningful change, and frozen after ``freeze_after_stable_fits``
    agreeing refits until the tracked dimensions move by
    ``unfreeze_dimension_change``. State is dropped when a track dies
    or its identity is reacquired by a new tracker filter (the reacquisition
    may be another car).
    """

    def __init__(self, config: VehicleAppearanceConfig | None = None, *,
                 accumulator: ObjectPointAccumulator | None = None, fitter: CarShapeFitter | None = None,
                 color_estimator: VehicleColorEstimator | None = None) -> None:
        self.config = config or VehicleAppearanceConfig()
        self.accumulator = accumulator or ObjectPointAccumulator()
        self.fitter = fitter or CarShapeFitter()
        self.color_estimator = color_estimator or VehicleColorEstimator()
        self._filters: dict[str, str] = {}  # published track id -> tracker filter id
        self._shapes: dict[str, _ShapeTrack] = {}
        self._color_ticks: dict[str, int] = {}
        self._tick = 0
        self.last_update_ms = 0.
        self.fit_count = 0

    def update(self, matches: Sequence[Any], live_tracks: Sequence[Any], *, timestamp_us: int,
               world_T_lidar: np.ndarray, lidar_points: np.ndarray, camera: AppearanceCamera | None = None) -> None:
        """Ingest one scan. ``matches`` are tracker ``DetectionMatch``es at ``timestamp_us``;
        ``live_tracks`` are all confirmed tracks then (deaths, neighbors and occluders)."""
        started = time.perf_counter()
        self._tick += 1
        self._forget(matches, live_tracks)
        lidar_T_world = np.linalg.inv(np.asarray(world_T_lidar, dtype=np.float64).reshape(4, 4))
        candidates = [match for match in matches if self._wants(match.track.label)]
        xyz = np.asarray(lidar_points, dtype=np.float32).reshape(len(lidar_points), -1)[:, :3] \
            if candidates and len(lidar_points) else np.empty((0, 3), np.float32)
        grid = _BevGrid(xyz[:, 0], xyz[:, 1], 2.) if len(xyz) else None
        if grid is None or grid.empty:
            self.last_update_ms = (time.perf_counter() - started) * 1e3
            return
        neighbors = [(lidar_T_world @ np.asarray(track.world_T_object, dtype=np.float64),
                      np.asarray(track.dimensions_lwh, dtype=np.float64), track.track_id) for track in live_tracks]
        color_ids = self._color_round_robin(candidates, camera)
        frame = _CameraFrame(camera, xyz, live_tracks, lidar_T_world) if color_ids else None
        for match in candidates:
            track = match.track
            lidar_T_object = lidar_T_world @ np.asarray(track.world_T_object, dtype=np.float64)
            dimensions = np.asarray(track.dimensions_lwh, dtype=np.float64)
            points = self._crop(xyz, grid, lidar_T_object, dimensions, track.track_id, neighbors)
            if self.config.shape and vehicle_proxy_kind(track.label) == "car":
                self._accumulate(track.track_id, points, dimensions, timestamp_us)
            if track.track_id in color_ids:
                assert frame is not None
                self._color_ticks[track.track_id] = self._tick
                camera_T_object = frame.camera_T_lidar @ lidar_T_object
                self.color_estimator.observe(
                    track.track_id, frame.image_bgr, frame.intrinsic, camera_T_object, points, dimensions,
                    occluder_xyxy=frame.nearer_xyxy(camera_T_object[2, 3], track.track_id),
                    depth_buffer=frame.depth_buffer)
        self._refit_round_robin()
        self.last_update_ms = (time.perf_counter() - started) * 1e3

    def prompt_fields(self, track_id: str) -> dict[str, Any]:
        """BoxPrompt keyword hints for ``track_id``; empty when nothing is published."""
        fields: dict[str, Any] = {}
        shape = self._shapes.get(track_id)
        if shape is not None and shape.published is not None:
            fields["vehicle_shape"] = shape.published.to_vector()
        color = self.color_estimator.color(track_id) if self.config.color else None
        if color is not None:
            fields["body_color_rgb"] = color.rgb
            fields["color_confidence"] = color.confidence
        return fields

    def drop(self, track_id: str) -> None:
        self._filters.pop(track_id, None)
        self._shapes.pop(track_id, None)
        self._color_ticks.pop(track_id, None)
        self.accumulator.drop(track_id)
        self.color_estimator.drop(track_id)

    def _wants(self, label: str) -> bool:
        kind = vehicle_proxy_kind(label)
        return (self.config.shape and kind == "car") or (self.config.color and kind is not None
                                                         and kind not in _NO_PAINT_KINDS)

    def _forget(self, matches: Sequence[Any], live_tracks: Sequence[Any]) -> None:
        live = {track.track_id for track in live_tracks}
        for track_id in [track_id for track_id in self._filters if track_id not in live]:
            self.drop(track_id)
        for match in matches:
            if self._filters.get(match.track.track_id, match.filter_id) != match.filter_id:
                self.drop(match.track.track_id)
            self._filters[match.track.track_id] = match.filter_id

    def _crop(self, xyz, grid, lidar_T_object, dimensions, track_id, neighbors) -> np.ndarray:
        """Metric object-frame scan returns inside the padded box, away from other tracks."""
        margin = self.config.crop_margin_m
        reach = float(np.hypot(*dimensions[:2] / 2)) + margin
        nearby = grid.near(lidar_T_object[:2, 3], reach)
        if len(nearby) > self.config.max_crop_points:
            nearby = nearby[::-(-len(nearby) // self.config.max_crop_points)]
        local = (xyz[nearby].astype(np.float64) - lidar_T_object[:3, 3]) @ lidar_T_object[:3, :3]
        half = dimensions / 2
        with np.errstate(invalid="ignore"):
            keep = np.all(np.abs(local[:, :2]) <= half[:2] + margin, axis=1) & (np.abs(local[:, 2]) <= half[2] + margin)
        local = local[keep]
        center = lidar_T_object[:3, 3]
        for other_T, other_dimensions, other_id in neighbors:
            if other_id == track_id or not len(local):
                continue
            other_reach = float(np.hypot(*other_dimensions[:2] / 2)) + self.config.neighbor_margin_m
            if np.linalg.norm(other_T[:2, 3] - center[:2]) > reach + other_reach:
                continue
            in_other = (local @ lidar_T_object[:3, :3].T + center - other_T[:3, 3]) @ other_T[:3, :3]
            local = local[~np.all(np.abs(in_other) <= other_dimensions / 2 + self.config.neighbor_margin_m, axis=1)]
        return local

    def _accumulate(self, track_id: str, points: np.ndarray, dimensions: np.ndarray, timestamp_us: int) -> None:
        state = self._shapes.setdefault(track_id, _ShapeTrack(dimensions))
        state.dimensions = dimensions
        if state.frozen_dimensions is not None:
            if np.max(np.abs(dimensions / state.frozen_dimensions - 1)) <= self.config.unfreeze_dimension_change:
                return
            state.frozen_dimensions, state.stable_fits = None, 0
        self.accumulator.add(track_id, points / dimensions, dimensions, timestamp_us)
        state.scans_since_fit += 1

    def _refit_round_robin(self) -> None:
        config = self.config
        due = []
        for track_id, state in self._shapes.items():
            evidence = self.accumulator.evidence(track_id)
            if state.frozen_dimensions is not None or evidence < self.fitter.min_evidence_voxels \
                    or state.scans_since_fit == 0:
                continue
            if evidence >= state.fit_evidence * (1 + config.refit_growth) \
                    or state.scans_since_fit >= config.refit_every_scans:
                due.append((state.last_fit_tick, track_id))
        started = time.perf_counter()
        for _, track_id in sorted(due)[:config.max_fits_per_update]:
            if (time.perf_counter() - started) * 1e3 >= config.fit_budget_ms:
                break
            self._refit(track_id, self._shapes[track_id])

    def _refit(self, track_id: str, state: _ShapeTrack) -> None:
        points, weights = self.accumulator.points(track_id)
        fit = self.fitter.fit(points * state.dimensions, state.dimensions, state.fitted, weights=weights)
        self.fit_count += 1
        state.fit_evidence, state.scans_since_fit, state.last_fit_tick = (
            self.accumulator.evidence(track_id), 0, self._tick)
        if not fit.accepted:
            state.stable_fits = 0
            return
        state.fitted = fit.params
        if state.published is None or self._changed(state.published, fit.params):
            state.published, state.stable_fits = fit.params, 0
            return
        state.stable_fits += 1
        if state.stable_fits >= self.config.freeze_after_stable_fits:
            state.frozen_dimensions = state.dimensions.copy()

    def _changed(self, published: CarShapeParams, fitted: CarShapeParams) -> bool:
        if (published.body_type, published.forward_sign) != (fitted.body_type, fitted.forward_sign):
            return True
        return any(abs(getattr(fitted, name) - getattr(published, name))
                   > self.config.min_param_change * (upper - lower)
                   for name, (lower, upper) in CarShapeParams.BOUNDS.items())

    def _color_round_robin(self, candidates: Sequence[Any], camera: AppearanceCamera | None) -> set[str]:
        if not self.config.color or camera is None:
            return set()
        painted = [match.track.track_id for match in candidates
                   if vehicle_proxy_kind(match.track.label) not in _NO_PAINT_KINDS]
        painted.sort(key=lambda track_id: self._color_ticks.get(track_id, -1))
        return set(painted[:self.config.max_color_tracks_per_update])


class _CameraFrame:
    """Per-scan camera products shared by every track: depth buffer and track image boxes."""

    def __init__(self, camera: AppearanceCamera, xyz: np.ndarray, live_tracks: Sequence[Any],
                 lidar_T_world: np.ndarray) -> None:
        self.image_bgr = np.asarray(camera.image_bgr)
        self.intrinsic = np.asarray(camera.camera_intrinsic, dtype=np.float64).reshape(3, 3)
        self.camera_T_lidar = np.asarray(camera.camera_T_lidar, dtype=np.float64).reshape(4, 4)
        height, width = self.image_bgr.shape[:2]
        scan_camera = xyz.astype(np.float64) @ self.camera_T_lidar[:3, :3].T + self.camera_T_lidar[:3, 3]
        self.depth_buffer = SparseDepthBuffer(scan_camera, self.intrinsic, (width, height))
        self._boxes = []
        for track in live_tracks:
            camera_T_object = self.camera_T_lidar @ lidar_T_world @ np.asarray(track.world_T_object, dtype=np.float64)
            projected = project_vehicle_box(
                world_T_object=camera_T_object, world_T_camera=np.eye(4), intrinsics=self.intrinsic,
                dimensions=VehicleDimensions(*track.dimensions_lwh), image_size=(height, width),
                minimum_area_px=0., clip_near_plane=True)
            if projected is not None:
                self._boxes.append((float(camera_T_object[2, 3]), track.track_id, projected.xyxy))

    def nearer_xyxy(self, depth: float, track_id: str) -> np.ndarray:
        nearer = [xyxy for other_depth, other_id, xyxy in self._boxes if other_id != track_id and other_depth < depth]
        return np.asarray(nearer, dtype=np.float64).reshape(-1, 4)

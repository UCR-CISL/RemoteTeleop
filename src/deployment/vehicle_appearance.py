"""Per-track vehicle shape and paint hints for BoxPrompts (vehicle side).

Composes the LiDAR shape accumulator/fitter and the camera color estimator
behind one bounded per-poll update. Inputs are the detector request's LiDAR
scan (LiDAR frame), the tracker's detection matches at that scan's timestamp
and, for color, the full-resolution image with the camera calibration used to
render it. Only compact hints leave this module.
"""

from __future__ import annotations

from dataclasses import dataclass
import itertools
import threading
import time
from typing import Any, Sequence

import numpy as np

from src.common.vehicle_categories import vehicle_proxy_kind
from src.deployment.lidar_ground_snap import _BevGrid
from src.deployment.vehicle_color import SparseDepthBuffer, VehicleColorEstimator
from src.deployment.vehicle_shape import ObjectPointAccumulator, VehicleShapeFit, VehicleShapeFitter
from src.reconstruction.models import VehicleDimensions
from src.reconstruction.projection import project_vehicle_box
from src.viz.universal_vehicle import BOUNDS, UniversalVehicleShape

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
    max_fits_per_update: int = 2  # fits started per update (~2 ms incumbent refit, ~15 ms full sweep) ...
    fit_budget_ms: float = 60.  # ... and no further fit starts once this much was spent in one update
    fit_in_background: bool = True  # False fits synchronously, for deterministic tests and offline replay
    sweep_every_refits: int = 5  # refit only the incumbent body type except every Nth fit
    max_color_tracks_per_update: int = 8
    min_param_change: float = .02  # fraction of a parameter's range that counts as a new shape
    freeze_after_stable_fits: int = 3
    unfreeze_dimension_change: float = .1
    unfreeze_new_cells: int = 2  # ... or when a scan covers this many new coarse surface cells
    publish_prior_shape: bool = True  # dimension-prior type mean until the first accepted fit

    def __post_init__(self) -> None:
        if (self.crop_margin_m < 0 or self.neighbor_margin_m < 0 or self.max_crop_points < 1
                or self.refit_growth <= 0 or self.refit_every_scans < 1 or self.max_fits_per_update < 0
                or self.fit_budget_ms < 0 or self.sweep_every_refits < 1
                or self.max_color_tracks_per_update < 0 or self.min_param_change < 0
                or self.freeze_after_stable_fits < 1 or self.unfreeze_dimension_change <= 0
                or self.unfreeze_new_cells < 1):
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
    generation: int  # identifies this track state to fits finishing after a drop or reset
    scans_since_fit: int = 0
    fit_evidence: int = 0
    last_fit_tick: int = -1
    fitted: UniversalVehicleShape | None = None
    fit_cost: float | None = None  # robust cost of ``fitted`` for incumbent-only refits
    fits: int = 0
    fitting: bool = False
    published: UniversalVehicleShape | None = None
    stable_fits: int = 0
    frozen_dimensions: np.ndarray | None = None
    coverage: np.ndarray | None = None  # coarse surface cells covered by the frozen fit's evidence


@dataclass(frozen=True)
class _FitRequest:
    """Immutable snapshot of one track's evidence; safe to fit on another thread."""

    track_id: str
    generation: int
    points: np.ndarray  # metric object frame
    weights: np.ndarray
    dimensions: np.ndarray
    previous: UniversalVehicleShape | None
    incumbent_cost: float | None

    def run(self, fitter: VehicleShapeFitter) -> VehicleShapeFit:
        return fitter.fit(self.points, self.dimensions, self.previous, weights=self.weights,
                          incumbent_cost=self.incumbent_cost)


class ShapeFitWorker:
    """Runs shape fits on one daemon thread, off the caller's (publishing) thread.

    At most one request per track is queued (a newer one replaces it) and at
    most ``max_pending`` overall. Results are only handed back through
    ``finished``, so the caller applies them on its own thread; requests are
    immutable snapshots and the fitter holds no mutable state.
    """

    def __init__(self, fitter: VehicleShapeFitter, *, max_pending: int = 8) -> None:
        if max_pending < 1:
            raise ValueError("max_pending must be positive")
        self.fitter, self.max_pending = fitter, max_pending
        self._condition = threading.Condition()
        self._pending: dict[str, _FitRequest] = {}
        self._finished: list[tuple[_FitRequest, VehicleShapeFit | Exception]] = []
        self._closed = False
        self._thread: threading.Thread | None = None

    def submit(self, request: _FitRequest) -> bool:
        """Queue ``request`` without blocking; False when closed or the backlog is full."""
        with self._condition:
            if self._closed or (request.track_id not in self._pending and len(self._pending) >= self.max_pending):
                return False
            self._pending.pop(request.track_id, None)
            self._pending[request.track_id] = request
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name="vehicle-shape-fit", daemon=True)
                self._thread.start()
            self._condition.notify()
            return True

    def finished(self) -> list[tuple[_FitRequest, VehicleShapeFit | Exception]]:
        """Completed (request, fit or error) pairs since the last call, oldest first."""
        with self._condition:
            finished, self._finished = self._finished, []
        return finished

    def close(self, timeout_s: float = 2.) -> None:
        """Drop queued work and join the thread after its current fit."""
        with self._condition:
            self._closed = True
            self._pending.clear()
            self._condition.notify_all()
            thread = self._thread
        if thread is not None:
            thread.join(timeout_s)

    def _run(self) -> None:
        while True:
            with self._condition:
                while not self._pending and not self._closed:
                    self._condition.wait()
                if self._closed:
                    return
                request = self._pending.pop(next(iter(self._pending)))
            try:
                result: VehicleShapeFit | Exception = request.run(self.fitter)
            except Exception as error:  # reported to the caller's thread
                result = error
            with self._condition:
                self._finished.append((request, result))


class VehicleAppearanceEstimator:
    """Bounded per-track shape fitting and paint color estimation.

    Only detections matched to confirmed tracks feed evidence, cropped with the
    corrected track pose at the scan timestamp; coasted tracks contribute
    nothing. Shapes are refit round-robin (oldest fit first), at most
    ``max_fits_per_update`` per update within ``fit_budget_ms``, published only
    on a meaningful change, and frozen after ``freeze_after_stable_fits``
    agreeing refits until the tracked dimensions move by
    ``unfreeze_dimension_change`` or a scan covers ``unfreeze_new_cells`` coarse
    surface cells the frozen evidence did not. Until the first accepted fit the
    dimension-prior type mean is published (``publish_prior_shape``). Refits try
    only the incumbent body type except every ``sweep_every_refits``-th fit. With ``fit_in_background``
    fits run on a ShapeFitWorker and finished fits are applied on a later
    update; call ``close`` on shutdown. State is dropped when a track dies
    or its identity is reacquired by a new tracker filter (the reacquisition
    may be another car).
    """

    def __init__(self, config: VehicleAppearanceConfig | None = None, *,
                 accumulator: ObjectPointAccumulator | None = None, fitter: VehicleShapeFitter | None = None,
                 color_estimator: VehicleColorEstimator | None = None) -> None:
        self.config = config or VehicleAppearanceConfig()
        self.accumulator = accumulator or ObjectPointAccumulator()
        self.fitter = fitter or VehicleShapeFitter()
        self.color_estimator = color_estimator or VehicleColorEstimator()
        self._filters: dict[str, str] = {}  # published track id -> tracker filter id
        self._shapes: dict[str, _ShapeTrack] = {}
        self._color_ticks: dict[str, int] = {}
        self._generations = itertools.count()
        self._worker = ShapeFitWorker(self.fitter) if self.config.fit_in_background else None
        self._tick = 0
        self.last_update_ms = 0.
        self.fit_count = 0
        self.fit_error_count = 0

    def update(self, matches: Sequence[Any], live_tracks: Sequence[Any], *, timestamp_us: int,
               world_T_lidar: np.ndarray, lidar_points: np.ndarray, camera: AppearanceCamera | None = None) -> None:
        """Ingest one scan. ``matches`` are tracker ``DetectionMatch``es at ``timestamp_us``;
        ``live_tracks`` are all confirmed tracks then (deaths, neighbors and occluders)."""
        started = time.perf_counter()
        self._tick += 1
        self._forget(matches, live_tracks)
        if self._worker is not None:
            for request, fit in self._worker.finished():
                if isinstance(fit, Exception):
                    # A failed fit must not stop publishing: keep the last shape and retry later.
                    self.fit_error_count += 1
                    state = self._shapes.get(request.track_id)
                    if state is not None and state.generation == request.generation:
                        state.fitting = False
                    continue
                self._apply_fit(request, fit)
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

    def close(self) -> None:
        """Stop the background fit worker, if any; no further fits start."""
        if self._worker is not None:
            self._worker.close()

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
        state = self._shapes.get(track_id)
        if state is None:
            state = self._shapes[track_id] = _ShapeTrack(dimensions, next(self._generations))
        state.dimensions = dimensions
        if state.frozen_dimensions is not None:
            new_cells = _coverage(points / dimensions, minimum_points=5) & ~state.coverage
            if (np.max(np.abs(dimensions / state.frozen_dimensions - 1)) <= self.config.unfreeze_dimension_change
                    and new_cells.sum() < self.config.unfreeze_new_cells):
                return
            state.frozen_dimensions, state.stable_fits = None, 0
        if self.accumulator.add(track_id, points / dimensions, dimensions, timestamp_us) \
                and state.fitted is None and self.config.publish_prior_shape:
            state.published = self.fitter.prior_shape(dimensions)  # LiDAR-supported track, no fit yet
        state.scans_since_fit += 1

    def _refit_round_robin(self) -> None:
        config = self.config
        due = []
        for track_id, state in self._shapes.items():
            evidence = self.accumulator.evidence(track_id)
            if state.frozen_dimensions is not None or state.fitting or evidence < self.fitter.min_evidence_voxels \
                    or state.scans_since_fit == 0:
                continue
            if evidence >= state.fit_evidence * (1 + config.refit_growth) \
                    or state.scans_since_fit >= config.refit_every_scans:
                due.append((state.last_fit_tick, track_id))
        started = time.perf_counter()
        for _, track_id in sorted(due)[:config.max_fits_per_update]:
            if (time.perf_counter() - started) * 1e3 >= config.fit_budget_ms:
                break
            state = self._shapes[track_id]
            points, weights = self.accumulator.points(track_id)
            request = _FitRequest(track_id, state.generation, points * state.dimensions, weights,
                                  state.dimensions.copy(), state.fitted,
                                  None if state.fits % config.sweep_every_refits == 0 else state.fit_cost)
            if self._worker is not None and not self._worker.submit(request):
                break
            state.fitting = True
            state.fit_evidence, state.scans_since_fit, state.last_fit_tick = (
                self.accumulator.evidence(track_id), 0, self._tick)
            if self._worker is None:
                self._apply_fit(request, request.run(self.fitter))

    def _apply_fit(self, request: _FitRequest, fit: VehicleShapeFit) -> None:
        state = self._shapes.get(request.track_id)
        if state is None or state.generation != request.generation:
            return  # the track died or was reset while fitting
        self.fit_count += 1
        state.fitting, state.fits = False, state.fits + 1
        if not fit.accepted:
            state.fit_cost, state.stable_fits = None, 0
            return
        state.fitted, state.fit_cost = fit.params, fit.robust_cost
        if state.published is None or self._changed(state.published, fit.params):
            state.published, state.stable_fits = fit.params, 0
            return
        state.stable_fits += 1
        if state.stable_fits >= self.config.freeze_after_stable_fits:
            state.frozen_dimensions = state.dimensions.copy()
            state.coverage = _coverage(self.accumulator.points(request.track_id)[0], minimum_points=3)

    def _changed(self, published: UniversalVehicleShape, fitted: UniversalVehicleShape) -> bool:
        if published.body_type != fitted.body_type:
            return True
        return bool(np.any(np.abs(fitted.array - published.array)
                           > self.config.min_param_change * (BOUNDS[:, 1] - BOUNDS[:, 0])))

    def _color_round_robin(self, candidates: Sequence[Any], camera: AppearanceCamera | None) -> set[str]:
        if not self.config.color or camera is None:
            return set()
        painted = [match.track.track_id for match in candidates
                   if vehicle_proxy_kind(match.track.label) not in _NO_PAINT_KINDS]
        painted.sort(key=lambda track_id: self._color_ticks.get(track_id, -1))
        return set(painted[:self.config.max_color_tracks_per_update])


def _coverage(points_normalized, *, minimum_points: int) -> np.ndarray:
    """Occupied coarse surface cells (8 along X, center/side by |Y|, 4 in Z) of box-normalized points."""
    points = np.asarray(points_normalized, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    x = np.clip(((points[:, 0] + .5) * 8).astype(int), 0, 7)
    side = (np.abs(points[:, 1]) > .3).astype(int)
    z = np.clip(((points[:, 2] + .5) * 4).astype(int), 0, 3)
    return np.bincount((x * 2 + side) * 4 + z, minlength=64) >= minimum_points


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

"""Estimate per-track vehicle body paint color from LiDAR-supported camera samples (vehicle side).

With a vehicle shape, the visible body panels of its projected universal mesh (hood, roof, trunk
lid, sides and ends; not glass, lights or wheels) are sampled as small patch means and balanced per
panel group; without one, track points in the body "belt" band (between wheels and glass) are
projected into the image. Samples are kept when the sparse LiDAR depth buffer and nearer tracks'
boxes say they are visible.  The dominant Lab cluster's median is the per-frame observation and
is fused per track over time, with the visible road ring's color as the camera exposure
reference.  Colors are sRGB floats in 0..1, RGB order (input images are BGR).
With a depth buffer, the visible road ring around the box floor is the per-frame gray reference:
a paint chroma that it explains as an illuminant cast (low sun) is reported as neutral.
White and silver paint whose sampled face is mostly shaded reads as dark gray; a persistent, bright,
neutral and spatially spread sample group is then published instead of the dark neutral color.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace

import cv2
import numpy as np


@dataclass(frozen=True)
class VehicleColorConfig:
    """Thresholds for :class:`VehicleColorEstimator`; distances are metres, sizes pixels."""

    belt_band: tuple[float, float] = (0.3, 0.6)  # fractions of box height above the box bottom
    box_margin_m: float = 0.1  # horizontal slack when cropping track points to the box
    patch_radius_px: int = 1  # 3x3 patch means
    highlight_value: int = 250  # any patch channel >= this is a clipped highlight
    max_highlight_fraction: float = 0.3  # more clipped samples than this is bright paint (white in sun): keep
    depth_tolerance_m: float = 0.25
    depth_cell_px: int = 4
    max_occluder_overlap: float = 0.3  # skip a track whose box is this covered by nearer tracks
    min_box_area_px: float = 900.0
    min_samples: int = 12
    full_samples: int = 60  # samples at which sample sufficiency saturates
    luminance_weight: float = 0.5  # L weight in the clustering metric
    merge_distance: float = 8.0  # weighted-Lab center distance under which K=2 clusters are one paint
    # Trim (tires, bumpers, glass, plates, shadow) is achromatic, so a colored minority cluster is paint.
    chromatic_chroma: float = 20.0  # cluster median chroma sqrt(a^2 + b^2) that counts as colored
    achromatic_chroma: float = 10.0  # cluster median chroma below which a cluster may be trim
    chromatic_saturation: float = 0.6  # dark paint has low chroma: also colored if C/sqrt(C^2+L^2) >= this
    chromatic_min_fraction: float = 0.2
    # ... but only over a darker majority: a bright achromatic majority is white/silver paint, and its
    # colored minority is tail lights and reflectors.
    trim_max_lightness: float = 60.0
    luminance_percentile: float = 75.0  # reported L: shaded paint reads darker than lit paint
    # Neutral paint: chroma below neutral_chroma is snapped to a = b = 0 unless it is a saturated dark
    # color (chroma >= neutral_dark_chroma and >= neutral_saturation * sqrt(C^2 + L^2), e.g. navy).
    neutral_chroma: float = 8.0
    neutral_dark_chroma: float = 4.0
    neutral_saturation: float = 0.45
    road_ring_m: tuple[float, float] = (0.3, 1.5)  # road reference: floor-plane ring outside the box
    min_road_samples: int = 20
    road_lightness: tuple[float, float] = (10.0, 95.0)  # usable road L*: not black or clipped
    min_confidence: float = 0.5  # observations below this are reported but not fused
    min_observations: int = 3  # fused observations before a track color is published
    chroma_window: int = 10  # fused a, b: median of the last confident observations ...
    luminance_window: int = 20  # ... and L over a longer window: exposure changes move L most
    # Shaded white/silver: when the fused paint is neutral, an L*-split bright group with this support,
    # lightness, margin over the dark group, road-cast-corrected chroma (below the larger of the chroma
    # or saturation * L*) and spread over >= bright_min_columns of 4 image columns (>= 2 samples each)
    # in >= bright_persistence and at least half of the fused observations is published instead.
    bright_min_fraction: float = 0.2
    bright_min_lightness: float = 40.0
    bright_min_delta_lightness: float = 15.0
    bright_neutral_chroma: float = 8.0
    bright_neutral_saturation: float = 0.15
    bright_min_columns: int = 3
    bright_persistence: int = 3
    # Body-panel crop (with a shape): visible BODY pixels of the projected universal mesh instead of belt LiDAR.
    crop_sampling: bool = True
    crop_min_height: float = 0.3  # fraction of box height above the box bottom (below: wheels, rockers)
    crop_erode_fraction: float = 0.01  # mask erosion, fraction of the crop's larger side (thin hoods survive)
    crop_max_samples: int = 400
    background_tolerance_m: float = 1.0  # a pixel whose LiDAR cell is only this much farther sees background
    # Side and end panels whose nearest LiDAR return (3x3 cells) is farther or nearer than this are dropped: a
    # loose mesh or a box larger than the vehicle would otherwise sample road and shadow past the body.
    support_tolerance_m: float = 0.5
    support_min_coverage: float = 0.5  # below this LiDAR coverage of the panels, unsupported samples are kept

    def __post_init__(self) -> None:
        low, high = self.belt_band
        if not 0 <= low < high <= 1 or self.box_margin_m < 0 or self.patch_radius_px < 0 \
                or not 0 < self.highlight_value <= 255 or not 0 <= self.max_highlight_fraction <= 1 \
                or self.depth_tolerance_m <= 0 or self.depth_cell_px < 1 or not 0 <= self.max_occluder_overlap <= 1 \
                or self.min_box_area_px < 0 or self.min_samples < 2 or self.full_samples < self.min_samples \
                or self.luminance_weight < 0 or self.merge_distance < 0 or not 0 <= self.min_confidence <= 1 \
                or not 0 <= self.achromatic_chroma <= self.chromatic_chroma or not 0 < self.chromatic_saturation <= 1 \
                or not 0 < self.chromatic_min_fraction <= 1 or not 0 <= self.luminance_percentile <= 100 \
                or not 0 <= self.neutral_dark_chroma <= self.neutral_chroma or not 0 <= self.neutral_saturation <= 1 \
                or not 0 <= self.road_ring_m[0] < self.road_ring_m[1] or self.min_road_samples < 1 \
                or not 0 <= self.road_lightness[0] < self.road_lightness[1] <= 100 \
                or self.min_observations < 1 or not 1 <= self.chroma_window <= self.luminance_window \
                or not 0 < self.bright_min_fraction <= 1 or self.bright_min_delta_lightness < 0 \
                or self.bright_neutral_chroma < 0 or self.bright_neutral_saturation < 0 \
                or not 1 <= self.bright_min_columns <= 4 or self.bright_persistence < 1 \
                or not 0 <= self.crop_min_height < 1 or not 0 <= self.crop_erode_fraction < 0.5 \
                or self.crop_max_samples < self.min_samples or self.background_tolerance_m <= 0 \
                or self.support_tolerance_m <= 0 or not 0 <= self.support_min_coverage <= 1:
            raise ValueError("invalid vehicle color configuration")


@dataclass(frozen=True)
class ColorObservation:
    """One frame's paint estimate for a track."""

    rgb: tuple[float, float, float]
    lab: tuple[float, float, float]
    confidence: float  # dominant cluster fraction x sample sufficiency
    sample_count: int
    dominant_fraction: float
    bright_lab: tuple[float, float, float] | None = None  # shaded-white/silver alternative (neutral)
    road_rgb: tuple[float, float, float] | None = None  # median visible road ring (camera exposure reference)


@dataclass(frozen=True)
class VehicleColorEstimate:
    """Temporally fused paint color for a track."""

    rgb: tuple[float, float, float]
    confidence: float
    observation_count: int
    road_rgb: tuple[float, float, float] | None = None  # fused road ring: paint/road ratio for exposure matching


def project_points_to_image(points_camera_cv: np.ndarray, K: np.ndarray,
                            image_size: tuple[int, int]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Pinhole-project CV-axes camera points; return ``(uv (N,2), depth (N,), valid (N,))``.

    ``image_size`` is ``(width, height)``.  Valid points are finite, in front of the camera and
    land inside ``[0, width-1] x [0, height-1]`` (pixel centers at integer coordinates).
    """
    points = np.asarray(points_camera_cv, dtype=np.float64).reshape(-1, 3)
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    depth = points[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        uv = (points @ K.T)[:, :2] / depth[:, None]
    width, height = image_size
    valid = np.isfinite(uv).all(axis=1) & (depth > 1e-3)
    valid &= (uv[:, 0] >= 0) & (uv[:, 0] <= width - 1) & (uv[:, 1] >= 0) & (uv[:, 1] <= height - 1)
    return uv, depth, valid


class SparseDepthBuffer:
    """Nearest LiDAR depth per coarse image cell (min over a 3x3 cell neighbourhood); build once per frame."""

    def __init__(self, points_camera_cv: np.ndarray, K: np.ndarray, image_size: tuple[int, int],
                 cell_px: int = 4) -> None:
        self.cell_px = int(cell_px)
        width, height = image_size
        grid = np.full((-(-height // self.cell_px), -(-width // self.cell_px)), np.inf, dtype=np.float32)
        uv, depth, valid = project_points_to_image(points_camera_cv, K, image_size)
        cells = np.rint(uv[valid]).astype(np.int64) // self.cell_px
        np.minimum.at(grid, (cells[:, 1], cells[:, 0]), depth[valid].astype(np.float32))
        self.nearest = grid  # per cell, before the neighbourhood minimum
        self.grid = cv2.erode(grid, np.ones((3, 3), np.uint8), borderType=cv2.BORDER_REPLICATE)

    def visible(self, uv: np.ndarray, depth: np.ndarray, tolerance_m: float) -> np.ndarray:
        """Return which in-image samples are no farther than the nearest nearby LiDAR surface + tolerance."""
        cells = np.rint(uv).astype(np.int64) // self.cell_px
        return depth <= self.grid[cells[:, 1], cells[:, 0]] + tolerance_m

    def supported(self, uv: np.ndarray, depth: np.ndarray, tolerance_m: float,
                  min_coverage: float = 0.5) -> np.ndarray:
        """Return which samples' nearest LiDAR return nearby (3x3 cell neighbourhood) is within
        ``tolerance_m`` of their depth. When fewer than ``min_coverage`` of the samples have any
        return nearby (dark paint returns little), samples without one are kept too."""
        cells = np.rint(uv).astype(np.int64) // self.cell_px
        nearest = self.grid[cells[:, 1], cells[:, 0]]
        close = np.abs(nearest - depth) <= tolerance_m
        covered = np.isfinite(nearest)
        return close if covered.mean() >= min_coverage else close | ~covered

    def behind(self, uv: np.ndarray, depth: np.ndarray, tolerance_m: float) -> np.ndarray:
        """Return which samples' own cell holds only LiDAR returns farther than depth + tolerance:
        the pixel sees background (or road) past the sampled surface."""
        cells = np.rint(uv).astype(np.int64) // self.cell_px
        nearest = self.nearest[cells[:, 1], cells[:, 0]]
        return np.isfinite(nearest) & (nearest > depth + tolerance_m)


@dataclass(frozen=True)
class PanelSamples:
    """Visible paint-panel pixels of one vehicle: full-resolution ``uv``, camera ``depth`` (m),
    ``rgb`` patch means (0..1, RGB) and panel ``group`` (``BodyPanelSampler.TOP/SIDE/END``)."""

    uv: np.ndarray
    depth: np.ndarray
    rgb: np.ndarray
    group: np.ndarray


class BodyPanelSampler:
    """Image pixels of a vehicle's visible body panels from its projected universal mesh.

    Every front-facing triangle (body, glass, lights, wheels) is painted far to near into a label
    image of the box crop (downscaled to ``max_crop_px``), so glass, lights, wheels and the car's
    own far side are excluded; the body mask is eroded off silhouette and material borders. Pixels
    are area means of the crop, so each sample is a small patch mean. Panels are grouped by their
    object-frame normal: sky-facing (hood, roof, trunk lid), sides and ends.
    """

    TOP, SIDE, END = range(3)

    def __init__(self, *, max_crop_px: int = 160, max_samples: int = 400, min_height: float = 0.3,
                 top_normal_z: float = 0.5, erode_fraction: float = 0.01) -> None:
        from src.viz.universal_vehicle import UniversalVehicleMesh

        self.mesh = UniversalVehicleMesh()
        self.max_crop_px, self.max_samples = int(max_crop_px), int(max_samples)
        self.min_height, self.top_normal_z = float(min_height), float(top_normal_z)
        self.erode_fraction = float(erode_fraction)

    def sample(self, image_bgr: np.ndarray, K: np.ndarray, camera_T_object: np.ndarray, shape,
               dims_lwh) -> PanelSamples | None:
        """``None`` when no body panel is visible; ``shape`` is a ``UniversalVehicleShape``."""
        from src.viz.universal_vehicle import BODY, RIM, TIRE

        dims = np.asarray(dims_lwh, dtype=np.float64)
        triangles, materials = self.mesh.surface(shape, dims)
        tires = np.isin(materials, (TIRE, RIM))  # four equal, contiguous wheel blocks
        wheel = np.where(tires, (np.cumsum(tires) - 1) // max(int(tires.sum()) // 4, 1), -1)
        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
        rotation, translation = camera_T_object[:3, :3], camera_T_object[:3, 3]
        camera = triangles @ rotation.T + translation
        camera_normals = normals @ rotation.T
        front = (np.einsum("ij,ij->i", camera_normals, camera.mean(axis=1)) < 0) & (camera[..., 2] > 0.1).all(axis=1)
        if not front.any():
            return None
        index = np.flatnonzero(front)
        uv = (camera[index] @ np.asarray(K, dtype=np.float64).T)
        uv = uv[..., :2] / uv[..., 2:]
        height, width = image_bgr.shape[:2]
        x0, y0 = np.maximum(np.floor(uv.reshape(-1, 2).min(axis=0)).astype(int), 0)
        x1, y1 = np.minimum(np.ceil(uv.reshape(-1, 2).max(axis=0)).astype(int) + 1, (width, height))
        if x1 - x0 < 4 or y1 - y0 < 4:
            return None
        scale = min(1.0, self.max_crop_px / max(x1 - x0, y1 - y0))
        size = (max(int(round((x1 - x0) * scale)), 1), max(int(round((y1 - y0) * scale)), 1))
        labels = np.full(size[::-1], -1, dtype=np.int32)
        fixed = np.rint(((uv - (x0, y0) + 0.5) * scale - 0.5) * 16).astype(np.int32)  # 4 fractional bits
        # Painter's order, far to near; each (convex) wheel is drawn once as the hull of its visible faces.
        items = [(depth, fixed[face], int(index[face])) for face, depth in enumerate(camera[index, :, 2].mean(axis=1))
                 if wheel[index[face]] < 0]
        for number in range(4):
            faces = np.flatnonzero(wheel[index] == number)
            if len(faces):
                items.append((float(camera[index[faces], :, 2].mean()), cv2.convexHull(fixed[faces].reshape(-1, 2)),
                              int(index[faces[0]])))
        for _, polygon, label in sorted(items, key=lambda item: -item[0]):
            cv2.fillConvexPoly(labels, polygon, label, lineType=cv2.LINE_8, shift=4)
        mask = (labels >= 0) & (materials == BODY)[np.maximum(labels, 0)]
        radius = max(1, int(round(self.erode_fraction * max(size))))
        mask = cv2.erode(mask.astype(np.uint8), np.ones((2 * radius + 1,) * 2, np.uint8), borderValue=0).astype(bool)
        rows, columns = np.nonzero(mask)
        if not len(rows):
            return None
        stride = int(np.ceil(np.sqrt(len(rows) / self.max_samples)))
        keep = (rows % stride == 0) & (columns % stride == 0)
        rows, columns = rows[keep], columns[keep]
        uv = np.column_stack([x0 + (columns + 0.5) / scale - 0.5, y0 + (rows + 0.5) / scale - 0.5])
        faces = labels[rows, columns]
        # Depth along each pixel ray to its face plane; keep pixels above the wheels' lower body.
        rays = np.column_stack([uv, np.ones(len(uv))]) @ np.linalg.inv(np.asarray(K, dtype=np.float64)).T
        plane_normals = camera_normals[faces]
        depth = np.einsum("ij,ij->i", plane_normals, camera[faces, 0]) / np.einsum("ij,ij->i", plane_normals, rays)
        height_above_floor = ((rays * depth[:, None] - translation) @ rotation)[:, 2] + dims[2] / 2
        keep = height_above_floor >= self.min_height * dims[2]
        if not keep.any():
            return None
        uv, depth, faces, rows, columns = uv[keep], depth[keep], faces[keep], rows[keep], columns[keep]
        crop = np.ascontiguousarray(image_bgr[y0:y1, x0:x1])
        crop = cv2.resize(crop, size, interpolation=cv2.INTER_AREA) if scale < 1 else cv2.blur(crop, (3, 3))
        rgb = crop[rows, columns][:, ::-1].astype(np.float64) / 255.0
        group = np.where(normals[faces, 2] >= self.top_normal_z, self.TOP,
                         np.where(np.abs(normals[faces, 0]) > np.abs(normals[faces, 1]), self.END, self.SIDE))
        return PanelSamples(uv, depth, rgb, group)


class VehicleColorEstimator:
    """Per-track body paint color from camera images and the track's LiDAR points."""

    def __init__(self, config: VehicleColorConfig | None = None) -> None:
        self.config = config or VehicleColorConfig()
        self._tracks: dict[str, _FusedColor] = {}
        self.panels = BodyPanelSampler(max_samples=self.config.crop_max_samples, min_height=self.config.crop_min_height,
                                       erode_fraction=self.config.crop_erode_fraction)

    def observe(self, track_id: str, image_bgr: np.ndarray, K: np.ndarray, camera_T_object: np.ndarray,
                object_points: np.ndarray, dims_lwh, scan_points_camera: np.ndarray | None = None,
                occluder_xyxy=None, depth_buffer: SparseDepthBuffer | None = None,
                shape=None) -> ColorObservation | None:
        """Estimate this frame's color and fuse it when confident; ``None`` when unobservable.

        ``object_points`` are metric object-frame points (+x forward, y left, z up, origin at the
        box center); ``camera_T_object`` maps them to CV camera axes.  ``scan_points_camera`` (or a
        prebuilt ``depth_buffer`` shared by all tracks of the frame) enables the occlusion test;
        ``occluder_xyxy`` are image boxes of nearer tracks.  With a ``shape`` (UniversalVehicleShape,
        e.g. the published fit or the box's prior type mean) the visible body panels are sampled
        (``crop_sampling``); without one, the belt-band LiDAR points.
        """
        samples = self._samples(image_bgr, K, camera_T_object, object_points, dims_lwh,
                                scan_points_camera, occluder_xyxy, depth_buffer, shape)
        if samples is None:
            return None
        rgb, road, columns, upright = samples
        # Sky-facing panels mirror the sky on dark paint: they only vote for shaded-white/silver paint.
        paint = rgb[upright] if upright.sum() >= self.config.min_samples else rgb
        observation = replace(self._neutralized(self._dominant_color(paint), road),
                              bright_lab=self._bright_alternative(rgb, columns, road),
                              road_rgb=self._road_reference(road))
        if observation.confidence >= self.config.min_confidence:
            self._tracks.setdefault(str(track_id), _FusedColor(self.config)).update(observation)
        return observation

    def color(self, track_id: str) -> VehicleColorEstimate | None:
        """Return the fused color once ``min_observations`` confident observations were fused."""
        fused = self._tracks.get(str(track_id))
        if fused is None or fused.count < self.config.min_observations:
            return None
        lab = fused.lab
        if self._is_neutral(lab):
            lab[1:] = 0.0
            bright = fused.bright_lab
            if bright is not None and bright[0] > lab[0]:
                lab = bright
        road = fused.road_rgb
        return VehicleColorEstimate(_lab_to_rgb(lab), fused.confidence, fused.count,
                                    None if road is None else tuple(float(value) for value in road))

    def drop(self, track_id: str) -> None:
        self._tracks.pop(str(track_id), None)

    def _samples(self, image_bgr, K, camera_T_object, object_points, dims_lwh, scan_points_camera,
                 occluder_xyxy, depth_buffer, shape=None) -> tuple[np.ndarray, ...] | None:
        """Return visible paint patch means (body-panel crop with a shape, else belt band) and
        road-ring pixels as (N, 3) RGB floats in 0..1, plus each patch's image column and whether
        it is on an upright (side or end) panel.

        ``None`` when the paint is unobservable; the road ring is empty without a depth buffer.
        """
        config = self.config
        image = np.asarray(image_bgr)
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError("image_bgr must be an (H, W, 3) uint8 BGR image")
        height, width = image.shape[:2]
        length, box_width, box_height = (float(value) for value in dims_lwh)
        camera_T_object = np.asarray(camera_T_object, dtype=np.float64).reshape(4, 4)

        box_xyxy = _projected_box_xyxy(camera_T_object, K, (length, box_width, box_height), (width, height))
        if box_xyxy is None or _area(box_xyxy) < config.min_box_area_px:
            return None
        occluders = np.asarray(occluder_xyxy if occluder_xyxy is not None else [], dtype=np.float64).reshape(-1, 4)
        if len(occluders) and _covered_fraction(box_xyxy, occluders) > config.max_occluder_overlap:
            return None

        if depth_buffer is None and scan_points_camera is not None:
            depth_buffer = SparseDepthBuffer(scan_points_camera, K, (width, height), config.depth_cell_px)
        if shape is not None and config.crop_sampling:
            paint = self._panel_samples(image, K, camera_T_object, shape, (length, box_width, box_height),
                                        occluders, depth_buffer)
            if paint is None:
                return None
            road = np.empty((0, 3))
            if depth_buffer is not None:
                road = self._road_ring(image, K, camera_T_object, (length, box_width, box_height), depth_buffer,
                                       occluders)
            rgb, columns, upright = paint
            return rgb, road, columns, upright

        points = np.asarray(object_points, dtype=np.float64).reshape(-1, 3)
        low, high = (-box_height / 2 + fraction * box_height for fraction in config.belt_band)
        keep = (points[:, 2] >= low) & (points[:, 2] <= high)
        keep &= (np.abs(points[:, 0]) <= length / 2 + config.box_margin_m)
        keep &= (np.abs(points[:, 1]) <= box_width / 2 + config.box_margin_m)
        camera_points = points[keep] @ camera_T_object[:3, :3].T + camera_T_object[:3, 3]
        uv, depth, valid = project_points_to_image(camera_points, K, (width, height))
        radius = config.patch_radius_px
        valid &= (uv[:, 0] >= radius) & (uv[:, 0] <= width - 1 - radius)
        valid &= (uv[:, 1] >= radius) & (uv[:, 1] <= height - 1 - radius)
        uv, depth = uv[valid], depth[valid]
        if depth_buffer is not None and len(uv):
            visible = depth_buffer.visible(uv, depth, config.depth_tolerance_m)
            uv, depth = uv[visible], depth[visible]
        if len(occluders) and len(uv):
            uv = uv[~_inside_any(uv[:, 0], uv[:, 1], occluders)]
        if len(uv) < config.min_samples:
            return None
        pixels = np.rint(uv).astype(np.int64)
        # One sample per patch-sized cell: dense near-range points must not over-weight one spot.
        _, first = np.unique(pixels // (2 * radius + 1), axis=0, return_index=True)
        pixels = pixels[np.sort(first)]
        if len(pixels) < config.min_samples:
            return None

        offsets = np.arange(-radius, radius + 1)
        rows = pixels[:, 1, None, None] + offsets[None, :, None]
        columns = pixels[:, 0, None, None] + offsets[None, None, :]
        patches = image[rows, columns].reshape(len(pixels), -1, 3)
        clipped = patches.max(axis=(1, 2)) >= config.highlight_value
        if clipped.mean() <= config.max_highlight_fraction:
            patches, pixels = patches[~clipped], pixels[~clipped]
        if len(patches) < config.min_samples:
            return None
        road = np.empty((0, 3))
        if depth_buffer is not None:
            road = self._road_ring(image, K, camera_T_object, (length, box_width, box_height), depth_buffer,
                                   occluders)
        return (patches.mean(axis=1)[:, ::-1] / 255.0, road, pixels[:, 0].astype(np.float64),
                np.ones(len(patches), dtype=bool))

    def _panel_samples(self, image, K, camera_T_object, shape, dims_lwh, occluders,
                       depth_buffer) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        """Visible, unoccluded body-panel patch means (RGB 0..1), their image columns and which are
        upright (side and end) panels."""
        config = self.config
        panels = self.panels.sample(image, K, camera_T_object, shape, dims_lwh)
        if panels is None:
            return None
        keep = np.ones(len(panels.uv), dtype=bool)
        if depth_buffer is not None:
            keep &= depth_buffer.visible(panels.uv, panels.depth, config.depth_tolerance_m)
            keep &= ~depth_buffer.behind(panels.uv, panels.depth, config.background_tolerance_m)
            upright = panels.group != BodyPanelSampler.TOP
            if upright.any():
                keep[upright] &= depth_buffer.supported(panels.uv[upright], panels.depth[upright],
                                                        config.support_tolerance_m, config.support_min_coverage)
        if len(occluders):
            keep &= ~_inside_any(panels.uv[:, 0], panels.uv[:, 1], occluders)
        rgb, columns, group = panels.rgb[keep], panels.uv[keep, 0], panels.group[keep]
        clipped = rgb.max(axis=1) >= config.highlight_value / 255.0
        if len(rgb) and clipped.mean() <= config.max_highlight_fraction:
            rgb, columns, group = rgb[~clipped], columns[~clipped], group[~clipped]
        if len(rgb) < config.min_samples:
            return None
        return rgb, columns, group != BodyPanelSampler.TOP

    def _road_reference(self, road_rgb: np.ndarray) -> tuple[float, float, float] | None:
        """Median road-ring RGB when it is a usable exposure reference (enough, not black or clipped)."""
        config = self.config
        if len(road_rgb) < config.min_road_samples:
            return None
        road = np.median(road_rgb, axis=0)
        if not config.road_lightness[0] <= _rgb_to_lab(road)[0, 0] <= config.road_lightness[1]:
            return None
        return tuple(float(value) for value in road)

    def _road_ring(self, image, K, camera_T_object, dims_lwh, depth_buffer, occluders) -> np.ndarray:
        """Visible pixels of the box floor plane in a ring around the footprint (the road), RGB 0..1."""
        length, width, height = dims_lwh
        near, far = self.config.road_ring_m
        x, y = (grid.ravel() for grid in np.meshgrid(np.linspace(-length / 2 - far, length / 2 + far, 40),
                                                     np.linspace(-width / 2 - far, width / 2 + far, 24)))
        outside = np.maximum(np.abs(x) - length / 2, np.abs(y) - width / 2)
        ring = (outside > near) & (outside < far)
        points = np.column_stack([x[ring], y[ring], np.full(ring.sum(), -height / 2)])
        image_height, image_width = image.shape[:2]
        uv, depth, valid = project_points_to_image(points @ camera_T_object[:3, :3].T + camera_T_object[:3, 3], K,
                                                   (image_width, image_height))
        uv, depth = uv[valid], depth[valid]
        uv = uv[depth_buffer.visible(uv, depth, self.config.depth_tolerance_m)] if len(uv) else uv
        if len(occluders) and len(uv):
            uv = uv[~_inside_any(uv[:, 0], uv[:, 1], occluders)]
        pixels = np.rint(uv).astype(np.int64)
        return image[pixels[:, 1], pixels[:, 0]][:, ::-1] / 255.0

    def _is_neutral(self, lab) -> bool:
        config = self.config
        chroma = float(np.hypot(lab[1], lab[2]))
        return chroma < config.neutral_chroma and (
            chroma < config.neutral_dark_chroma or chroma < config.neutral_saturation * np.hypot(chroma, lab[0]))

    def _cast_residual(self, lab: np.ndarray, road_rgb: np.ndarray) -> np.ndarray:
        """``lab`` with the road ring's illuminant cast removed (for neutrality tests only).

        The road is near-neutral, so its chroma, scaled by the paint/road lightness ratio, estimates
        the cast. Only the paint chroma along the cast, up to its size, is removed.
        """
        config = self.config
        residual = np.asarray(lab, dtype=np.float64).copy()
        if len(road_rgb) >= config.min_road_samples:
            road = np.median(_rgb_to_lab(road_rgb), axis=0)
            if config.road_lightness[0] <= road[0] <= config.road_lightness[1]:
                cast = road[1:] * min(residual[0] / road[0], 2.0)
                size = float(np.linalg.norm(cast))
                along = float(residual[1:] @ cast) / size if size > 1e-6 else 0.0
                if along > 0:
                    residual[1:] -= min(along, size) * cast / size
        return residual

    def _neutralized(self, observation: ColorObservation, road_rgb: np.ndarray) -> ColorObservation:
        """Snap neutral paint to a = b = 0, after removing the road ring's illuminant cast; colored
        paint keeps its observed hue."""
        lab = np.asarray(observation.lab, dtype=np.float64)
        residual = self._cast_residual(lab, road_rgb)
        if not self._is_neutral(residual) or not lab[1:].any():
            return observation
        lab[1:] = 0.0
        return replace(observation, rgb=_lab_to_rgb(lab), lab=tuple(float(value) for value in lab))

    def _dominant_color(self, samples_rgb: np.ndarray) -> ColorObservation:
        """Cluster samples in luminance-down-weighted Lab (K=2) and summarize the paint cluster.

        The paint cluster is the larger one, unless the larger is achromatic and darker than
        ``trim_max_lightness`` and the smaller is colored (chroma, or saturation for dark paint) and
        holds at least ``chromatic_min_fraction``: trim is dark and achromatic, so the colored
        cluster is paint and the achromatic one is explained away.  Its evidence is then scored
        relative to that minimum, ``min_confidence * fraction / chromatic_min_fraction`` (capped
        at 1), so the smallest qualifying colored cluster just reaches the publish threshold.
        The color is the cluster's median a, b with an upper-percentile L (less shadow-biased).
        """
        config = self.config
        lab = _rgb_to_lab(samples_rgb)
        features = lab * np.array([config.luminance_weight, 1.0, 1.0])
        labels, centers = _two_means(features)
        if np.linalg.norm(centers[0] - centers[1]) < config.merge_distance:
            members = np.ones(len(lab), dtype=bool)
            evidence = 1.0
        else:
            larger = int(np.argmax(np.bincount(labels, minlength=2)))
            lightness, chroma = zip(*((np.median(lab[labels == index, 0]),
                                       np.hypot(*np.median(lab[labels == index, 1:], axis=0))) for index in range(2)))
            smaller = 1 - larger
            colored = chroma[smaller] >= config.chromatic_chroma or (
                chroma[smaller] >= config.achromatic_chroma
                and chroma[smaller] >= config.chromatic_saturation * np.hypot(chroma[smaller], lightness[smaller]))
            members = labels == larger
            evidence = float(members.mean())
            if chroma[larger] < config.achromatic_chroma and lightness[larger] < config.trim_max_lightness \
                    and colored and (labels == smaller).mean() >= config.chromatic_min_fraction:
                members = labels != larger
                evidence = min(1.0, config.min_confidence * members.mean() / config.chromatic_min_fraction)
        fraction = float(members.mean())
        color = np.median(lab[members], axis=0)
        color[0] = np.percentile(lab[members, 0], config.luminance_percentile)
        sufficiency = min(1.0, len(lab) / config.full_samples)
        return ColorObservation(_lab_to_rgb(color), tuple(float(value) for value in color),
                                evidence * sufficiency, len(lab), fraction)

    def _bright_alternative(self, samples_rgb: np.ndarray, columns: np.ndarray,
                            road_rgb: np.ndarray) -> tuple[float, float, float] | None:
        """Neutral Lab of a bright sample group that may be shaded-white/silver paint, else ``None``.

        Samples are split by L* alone (2-means). The bright group qualifies with enough support,
        lightness and margin over the dark group, a near-neutral road-cast-corrected chroma, and
        samples spread across the image columns (a reflection or one lit panel is localized).
        """
        config = self.config
        lab = _rgb_to_lab(samples_rgb)
        labels, centers = _two_means(lab[:, :1])
        bright = labels == int(np.argmax(centers[:, 0]))
        if bright.mean() < config.bright_min_fraction or bright.all() or np.ptp(columns) == 0:
            return None
        lightness = float(np.median(lab[bright, 0]))
        if lightness < config.bright_min_lightness \
                or lightness - np.median(lab[~bright, 0]) < config.bright_min_delta_lightness:
            return None
        color = np.median(lab[bright], axis=0)
        color[0] = np.percentile(lab[bright, 0], config.luminance_percentile)
        residual = self._cast_residual(color, road_rgb)
        if np.hypot(*residual[1:]) >= max(config.bright_neutral_chroma, config.bright_neutral_saturation * color[0]):
            return None
        counts = np.histogram(columns[bright], np.linspace(columns.min(), columns.max(), 5))[0]
        if (counts >= 2).sum() < config.bright_min_columns:
            return None
        return float(color[0]), 0.0, 0.0


class _FusedColor:
    """Per-channel Lab medians over ring buffers of confident observations.

    Medians reject a minority of incompatible readings (tail lights, a shaded frame) instead of
    averaging them into a color that was never observed; L uses the longer window.
    """

    def __init__(self, config: VehicleColorConfig) -> None:
        self._observations: deque[tuple[tuple[float, float, float], float]] = deque(maxlen=config.luminance_window)
        self._bright: deque[tuple[float, float, float] | None] = deque(maxlen=config.luminance_window)
        self._road: deque[tuple[float, float, float] | None] = deque(maxlen=config.luminance_window)
        self._chroma_window = config.chroma_window
        self._bright_persistence = config.bright_persistence
        self.count = 0

    def update(self, observation: ColorObservation) -> None:
        self.count += 1
        self._observations.append((observation.lab, observation.confidence))
        self._bright.append(observation.bright_lab)
        self._road.append(observation.road_rgb)

    @property
    def lab(self) -> np.ndarray:
        lab = np.array([lab for lab, _ in self._observations])
        return np.array([np.median(lab[:, 0]), *np.median(lab[-self._chroma_window:, 1:], axis=0)])

    @property
    def bright_lab(self) -> np.ndarray | None:
        """Median bright alternative once it persists in enough (and at least half) of the window."""
        bright = [lab for lab in self._bright if lab is not None]
        if len(bright) < max(self._bright_persistence, len(self._bright) / 2):
            return None
        return np.array([np.median([lab[0] for lab in bright]), 0.0, 0.0])

    @property
    def road_rgb(self) -> np.ndarray | None:
        """Per-channel median road ring over the same window as the fused L (paint/road stays consistent)."""
        road = [rgb for rgb in self._road if rgb is not None]
        return np.median(road, axis=0) if road else None

    @property
    def confidence(self) -> float:
        return float(np.mean([confidence for _, confidence in self._observations][-self._chroma_window:]))


def _two_means(features: np.ndarray, iterations: int = 10) -> tuple[np.ndarray, np.ndarray]:
    """Deterministic 2-means: seed with the median and the sample farthest from it."""
    first = np.median(features, axis=0)
    centers = np.stack([first, features[np.argmax(np.linalg.norm(features - first, axis=1))]])
    labels = np.zeros(len(features), dtype=np.int64)
    for _ in range(iterations):
        labels = np.argmin(np.linalg.norm(features[:, None] - centers[None], axis=2), axis=1)
        updated = np.stack([features[labels == index].mean(axis=0) if (labels == index).any() else centers[index]
                            for index in range(2)])
        if np.allclose(updated, centers):
            break
        centers = updated
    return labels, centers


def _rgb_to_lab(rgb: np.ndarray) -> np.ndarray:
    rgb = np.clip(np.asarray(rgb, dtype=np.float32).reshape(-1, 1, 3), 0.0, 1.0)
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2Lab).reshape(-1, 3).astype(np.float64)


def _lab_to_rgb(lab: np.ndarray) -> tuple[float, float, float]:
    rgb = cv2.cvtColor(np.asarray(lab, dtype=np.float32).reshape(1, 1, 3), cv2.COLOR_Lab2RGB).reshape(3)
    return tuple(float(value) for value in np.clip(rgb, 0.0, 1.0))


def _projected_box_xyxy(camera_T_object: np.ndarray, K: np.ndarray, dims_lwh: tuple[float, float, float],
                        image_size: tuple[int, int]) -> np.ndarray | None:
    """Image-clipped xyxy of the box corners in front of the camera, or ``None`` when off-screen."""
    signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], dtype=np.float64)
    corners = signs * (np.asarray(dims_lwh) / 2) @ camera_T_object[:3, :3].T + camera_T_object[:3, 3]
    corners = corners[corners[:, 2] > 0.1]
    if not len(corners):
        return None
    uv = (corners @ np.asarray(K, dtype=np.float64).reshape(3, 3).T)[:, :2] / corners[:, 2:3]
    width, height = image_size
    xyxy = np.array([max(uv[:, 0].min(), 0), max(uv[:, 1].min(), 0),
                     min(uv[:, 0].max(), width - 1), min(uv[:, 1].max(), height - 1)])
    return xyxy if xyxy[2] > xyxy[0] and xyxy[3] > xyxy[1] else None


def _area(xyxy: np.ndarray) -> float:
    return float(max(xyxy[2] - xyxy[0], 0) * max(xyxy[3] - xyxy[1], 0))


def _covered_fraction(box_xyxy: np.ndarray, occluders: np.ndarray) -> float:
    """Fraction of ``box_xyxy`` covered by the union of ``occluders`` (rasterized on a coarse grid)."""
    xs = np.linspace(box_xyxy[0], box_xyxy[2], 24)
    ys = np.linspace(box_xyxy[1], box_xyxy[3], 24)
    u, v = (grid.ravel() for grid in np.meshgrid(xs, ys))
    return float(_inside_any(u, v, occluders).mean())


def _inside_any(u: np.ndarray, v: np.ndarray, boxes_xyxy: np.ndarray) -> np.ndarray:
    return ((u[:, None] >= boxes_xyxy[:, 0]) & (u[:, None] <= boxes_xyxy[:, 2])
            & (v[:, None] >= boxes_xyxy[:, 1]) & (v[:, None] <= boxes_xyxy[:, 3])).any(axis=1)

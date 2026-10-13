"""Depth-aware GSplat camera composition for asynchronous object meshes."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Mapping, Protocol

import cv2
import numpy as np

if TYPE_CHECKING:
    from src.viz.vehicle_proxy import VehicleProxyRenderer

from src.common.vehicle_categories import vehicle_proxy_kind
from src.localization.gaussian_map import GaussianMap
from src.localization.ground_plane import GroundPlane
from src.localization.ground_surface import GroundSurface
from src.realtime.protocol import AssetEvent, AssetState, BoxPrompt, EgoPoseSample, FrameDetections
from src.viz.localization_visualizer import (
    GaussianRenderBuffers,
    _rasterize,
    _single_camera_array,
    prepare_camera_render,
)


@dataclass(frozen=True)
class RemoteCameraConfig:
    """Stable remote-side camera calibration used by the pose-only renderer.

    ``ego_T_camera`` maps CV camera coordinates into the ego/LiDAR frame.  In
    CooperScene this is the inverse of the documented ``camera_T_lidar``.
    """

    ego_T_camera: np.ndarray
    camera_intrinsic: np.ndarray
    image_size: tuple[int, int]

    def __post_init__(self) -> None:
        ego_T_camera = _rigid_transform(self.ego_T_camera, "ego_T_camera")
        intrinsic = np.asarray(self.camera_intrinsic, dtype=np.float64)
        if intrinsic.shape != (3, 3) or not np.isfinite(intrinsic).all():
            raise ValueError("camera_intrinsic must be a finite 3x3 matrix")
        width, height = self.image_size
        if width <= 0 or height <= 0:
            raise ValueError("image_size width and height must be positive")
        object.__setattr__(self, "ego_T_camera", ego_T_camera.copy())
        object.__setattr__(self, "camera_intrinsic", intrinsic.copy())
        object.__setattr__(self, "image_size", (int(width), int(height)))

    @classmethod
    def for_cooperscene_agent(
        cls, agent: str | int, *, image_size: tuple[int, int] = (480, 300)
    ) -> "RemoteCameraConfig":
        """Return the front-camera calibration held entirely on the remote side."""

        from src.localization.cooperscene_calibration import camera_T_lidar, camera_intrinsic

        intrinsic = camera_intrinsic(agent).copy()
        scale_x = image_size[0] / 1920.0
        scale_y = image_size[1] / 1200.0
        intrinsic[0] *= scale_x
        intrinsic[1] *= scale_y
        return cls(
            ego_T_camera=np.linalg.inv(camera_T_lidar(agent)),
            camera_intrinsic=intrinsic,
            image_size=image_size,
        )

    @classmethod
    def from_source_calibration(
        cls,
        *,
        ego_T_camera: np.ndarray,
        camera_intrinsic: np.ndarray,
        source_image_size: tuple[int, int],
        image_size: tuple[int, int],
    ) -> "RemoteCameraConfig":
        """Scale a generic source-camera calibration to the render size."""

        source_width, source_height = source_image_size
        if source_width <= 0 or source_height <= 0:
            raise ValueError("source_image_size width and height must be positive")
        intrinsic = np.asarray(camera_intrinsic, dtype=np.float64).copy()
        if intrinsic.shape != (3, 3):
            raise ValueError("camera_intrinsic must be a 3x3 matrix")
        intrinsic[0] *= image_size[0] / source_width
        intrinsic[1] *= image_size[1] / source_height
        return cls(ego_T_camera, intrinsic, image_size)


@dataclass(frozen=True)
class PoseOnlyCompositedFrame:
    """A GSplat frame rendered solely from an ego pose and local calibration."""

    sequence: int
    timestamp_us: int
    scene_generation: str
    world_T_camera: np.ndarray
    rgb: np.ndarray
    track_states: Mapping[str, str]
    ground_placements: Mapping[str, Mapping[str, object]] = field(default_factory=dict)
    new_track_ids: frozenset[str] = frozenset()


class NewTrackHighlighter:
    """Remember each track ID's first-seen frame time so new tracks can be tinted.

    IDs unseen for ``forget_seconds`` are dropped; a timestamp moving backwards
    (scene restart) or ``reset()`` clears all history.
    """

    def __init__(
        self, highlight_seconds: float = 1.0, *, forget_seconds: float = 5.0,
        color: tuple[float, float, float] = (1.0, 0.35, 0.0), strength: float = 0.75,
    ) -> None:
        if not np.isfinite(highlight_seconds) or highlight_seconds <= 0.0:
            raise ValueError("highlight_seconds must be finite and positive")
        self._highlight_us = highlight_seconds * 1e6
        self._forget_us = max(forget_seconds, highlight_seconds) * 1e6
        self.color = np.asarray(color, dtype=np.float32)
        self._strength = float(strength)
        self._first_seen: dict[str, int] = {}
        self._last_seen: dict[str, int] = {}
        self._last_timestamp: int | None = None

    def reset(self) -> None:
        self._first_seen.clear()
        self._last_seen.clear()
        self._last_timestamp = None

    def observe(self, track_ids, timestamp_us: int) -> frozenset[str]:
        """Record one frame's IDs and return those first seen within the window."""

        if self._last_timestamp is not None and timestamp_us < self._last_timestamp:
            self.reset()
        self._last_timestamp = timestamp_us
        for track_id in track_ids:
            self._first_seen.setdefault(track_id, timestamp_us)
            self._last_seen[track_id] = timestamp_us
        for track_id in [k for k, seen in self._last_seen.items() if timestamp_us - seen > self._forget_us]:
            del self._first_seen[track_id], self._last_seen[track_id]
        return frozenset(
            track_id for track_id in track_ids
            if timestamp_us - self._first_seen[track_id] < self._highlight_us
        )

    def tint(self, overlay: "MeshRenderBuffers") -> "MeshRenderBuffers":
        rgb = overlay.rgb * (1.0 - self._strength) + self.color * self._strength
        return MeshRenderBuffers(rgb, overlay.depth, overlay.alpha)


@dataclass(frozen=True)
class MeshAssetStatus:
    """Observable result of activating a received mesh on the remote host."""

    track_id: str
    request_id: str
    state: str
    path: Path | None = None
    error: str | None = None


class PoseOnlyCompositedCameraBackend:
    """Render ordered pose samples with bbox proxies and cached mesh assets.

    Camera calibration stays on the remote machine.  The caller supplies the
    detection batch joined to each pose sample, so both a proxy and its mesh
    replacement always use the same object transform.
    """

    def __init__(
        self,
        gaussian_renderer: "GaussianBufferRenderer",
        camera: RemoteCameraConfig,
        mesh_renderer: "ObjectMeshRenderer | None" = None,
        *,
        proxy_color: tuple[float, float, float] = (0.0, 0.7, 1.0),
        proxy_thickness: int = 2,
        alpha_threshold: float = 0.01,
        depth_tolerance_m: float = 0.03,
        ground_z: float | None = None,
        vehicle_proxies: "VehicleProxyRenderer | None" = None,
        ground_plane: GroundPlane | GroundSurface | None = None,
        new_tracks: NewTrackHighlighter | None = None,
        track_id_labels: bool = False,
        proxy_occlusion_depth: str = "occluder",
    ) -> None:
        if proxy_thickness <= 0:
            raise ValueError("proxy_thickness must be positive")
        if proxy_occlusion_depth not in PROXY_OCCLUSION_DEPTHS:
            raise ValueError(f"proxy_occlusion_depth must be one of {PROXY_OCCLUSION_DEPTHS}")
        self._gaussian_renderer = gaussian_renderer
        # Vehicle proxies are depth-tested against the renderer's occluder depth
        # (see OpacityThresholdDepth); "expected" keeps the legacy test for A/B.
        self._proxy_occlusion_depth = proxy_occlusion_depth
        self.camera = camera
        self._new_tracks = new_tracks
        self._track_id_labels = track_id_labels
        self._mesh_renderer = mesh_renderer
        self._proxy_color = np.asarray(proxy_color, dtype=np.float32)
        self._proxy_thickness = proxy_thickness
        self._alpha_threshold = float(alpha_threshold)
        self._depth_tolerance_m = float(depth_tolerance_m)
        if ground_plane is not None and ground_z is not None:
            raise ValueError("ground_plane requires ground_z=None")
        self._ground_plane = ground_plane
        self._ground_z = None if ground_z is None else float(ground_z)
        self._scene_generation: str | None = None
        self._retired_generations: set[str] = set()
        self._assets: dict[str, _MeshAsset] = {}
        self._mesh_statuses: list[MeshAssetStatus] = []
        self._vehicle_proxies = vehicle_proxies

    def handle_asset_event(self, event: AssetEvent) -> None:
        """Load a verified local mesh once; failures deliberately retain bboxes."""

        if self._mesh_renderer is None:
            return
        generation = str(event.scene_generation)
        if generation in self._retired_generations:
            return
        if self._scene_generation is None:
            self._scene_generation = generation
        elif generation != self._scene_generation:
            return
        previous = self._assets.get(event.track_id)
        if previous is not None and previous.request_id != event.request_id:
            return
        if previous is not None and previous.state in {
            AssetState.READY, AssetState.FAILED, AssetState.CANCELLED
        }:
            return
        if event.state is not AssetState.READY:
            self._assets[event.track_id] = _MeshAsset(event.state, event.request_id)
            return
        path = Path(event.aligned_mesh_path)  # validated by AssetEvent
        if not path.is_file():
            self._assets[event.track_id] = _MeshAsset(AssetState.FAILED, event.request_id)
            self._record_mesh_status(event, "load_failed", path, "mesh path does not exist")
            return
        try:
            handle = self._mesh_renderer.load(path)
        except Exception as error:
            self._assets[event.track_id] = _MeshAsset(AssetState.FAILED, event.request_id)
            self._record_mesh_status(event, "load_failed", path, _error_detail(error))
            return
        self._assets[event.track_id] = _MeshAsset(
            AssetState.READY, event.request_id, path, handle,
            event.source_sequence, event.source_timestamp_us,
        )
        self._record_mesh_status(event, "loaded", path)

    def drain_mesh_statuses(self) -> tuple[MeshAssetStatus, ...]:
        """Return mesh load/render outcomes for durable compositor metrics."""

        statuses = tuple(self._mesh_statuses)
        self._mesh_statuses.clear()
        return statuses

    def render_pose(
        self, pose_sample: EgoPoseSample, boxes: tuple[BoxPrompt, ...] = ()
    ) -> PoseOnlyCompositedFrame:
        """Derive the camera pose locally and depth-test lightweight box proxies."""

        generation = str(pose_sample.scene_generation)
        self._activate_generation(generation)
        world_T_ego = _rigid_transform(
            np.asarray(pose_sample.world_T_ego, dtype=np.float64).reshape(4, 4),
            "world_T_ego",
        )
        exact_camera_pose = getattr(pose_sample, "world_T_camera", None)
        world_T_camera = (
            world_T_ego @ self.camera.ego_T_camera
            if exact_camera_pose is None
            else _rigid_transform(
                np.asarray(exact_camera_pose, dtype=np.float64).reshape(4, 4),
                "world_T_camera",
            )
        )
        background = self._gaussian_renderer.render(
            world_T_camera, self.camera.camera_intrinsic, self.camera.image_size
        )
        if background.rgb.shape[:2] != self.camera.image_size[::-1]:
            raise ValueError("Gaussian renderer returned an unexpected image size")
        rgb = np.clip(background.rgb.copy(), 0.0, 1.0)
        object_depth = np.full(background.depth.shape, np.inf, dtype=np.float32)
        track_states: dict[str, str] = {}
        ground_placements: dict[str, dict] = {}
        new_ids = (frozenset() if self._new_tracks is None else
                   self._new_tracks.observe([box.track_id for box in boxes], int(pose_sample.timestamp_us)))
        labels: list[tuple[BoxPrompt, np.ndarray, bool]] = []
        for box in boxes:
            placed_pose = _flat_ground_box_pose(
                box, ground_z=self._ground_z, ground_plane=self._ground_plane,
            )
            highlight = box.track_id in new_ids
            if highlight or self._track_id_labels:
                labels.append((box, placed_pose, highlight))
            status = ("explicit_plane" if self._ground_plane is not None else
                      "fixed_ground_z" if self._ground_z is not None else "preserve_input_pose")
            ground_placements[box.track_id] = dict(getattr(
                self._ground_plane, "last_placement", {"status": status},
            ))
            kind = vehicle_proxy_kind(box.text_label)
            if self._vehicle_proxies is not None and kind is not None:
                overlay = self._vehicle_proxies.render(
                    kind, box.dimensions_lwh,
                    placed_pose,
                    world_T_camera, self.camera.camera_intrinsic, self.camera.image_size,
                )
                if highlight:
                    overlay = self._new_tracks.tint(overlay)
                rgb = _composite_mesh(
                    rgb, self._proxy_occluder_depth(background), background.alpha, overlay,
                    object_depth=object_depth,
                    alpha_threshold=self._alpha_threshold,
                    depth_tolerance_m=self._depth_tolerance_m,
                )
                track_states[box.track_id] = "mesh"
                continue
            asset = self._assets.get(box.track_id)
            if (
                asset is not None
                and asset.state is AssetState.READY
                and (
                    asset.source_sequence <= pose_sample.sequence
                    if asset.source_sequence is not None
                    else asset.source_timestamp_us is not None
                    and asset.source_timestamp_us <= pose_sample.timestamp_us
                )
            ):
                assert self._mesh_renderer is not None
                try:
                    overlay = self._mesh_renderer.render(
                        asset.handle,
                        placed_pose,
                        world_T_camera,
                        self.camera.camera_intrinsic,
                        self.camera.image_size,
                    )
                    if highlight:
                        overlay = self._new_tracks.tint(overlay)
                    rgb = _composite_mesh(
                        rgb, background.depth, background.alpha, overlay, object_depth=object_depth,
                        alpha_threshold=self._alpha_threshold,
                        depth_tolerance_m=self._depth_tolerance_m,
                    )
                    track_states[box.track_id] = "mesh"
                    continue
                except Exception as error:
                    asset.state = AssetState.FAILED
                    self._mesh_statuses.append(MeshAssetStatus(
                        box.track_id, asset.request_id, "render_failed", asset.path,
                        _error_detail(error),
                    ))
            _draw_box_proxy(
                rgb,
                background.depth,
                background.alpha,
                box,
                world_T_camera,
                self.camera.camera_intrinsic,
                self._new_tracks.color if highlight else self._proxy_color,
                self._proxy_thickness,
                self._alpha_threshold,
                self._depth_tolerance_m,
                ground_z=self._ground_z,
                ground_plane=self._ground_plane, object_depth=object_depth, placed_pose=placed_pose,
            )
            track_states[box.track_id] = "proxy"
        for box, placed_pose, highlight in labels:
            _draw_track_label(rgb, box, placed_pose, world_T_camera, self.camera.camera_intrinsic,
                              self._new_tracks.color if highlight else self._proxy_color)
        return PoseOnlyCompositedFrame(
            sequence=int(pose_sample.sequence),
            timestamp_us=int(pose_sample.timestamp_us),
            scene_generation=generation,
            world_T_camera=world_T_camera,
            rgb=rgb,
            track_states=track_states,
            ground_placements=ground_placements,
            new_track_ids=new_ids,
        )

    def _proxy_occluder_depth(self, background: GaussianRenderBuffers) -> np.ndarray:
        """Splat depth that occludes vehicle proxies; renderers without one fall back to ED."""

        if self._proxy_occlusion_depth == "expected" or background.occluder_depth is None:
            return background.depth
        return background.occluder_depth

    def _activate_generation(self, generation: str) -> None:
        if self._scene_generation == generation:
            return
        if self._scene_generation is not None:
            raise ValueError(
                f"mismatched scene generation {generation!r}; expected {self._scene_generation!r}"
            )
        self._scene_generation = generation
        if self._new_tracks is not None:
            self._new_tracks.reset()

    def _record_mesh_status(
        self, event: AssetEvent, state: str, path: Path | None, error: str | None = None
    ) -> None:
        self._mesh_statuses.append(MeshAssetStatus(
            event.track_id, event.request_id, state, path, error
        ))


@dataclass(frozen=True)
class MeshRenderBuffers:
    """One transparent object render in the GSplat camera."""

    rgb: np.ndarray
    depth: np.ndarray
    alpha: np.ndarray

    def __post_init__(self) -> None:
        rgb = np.asarray(self.rgb, dtype=np.float32)
        depth = np.asarray(self.depth, dtype=np.float32)
        alpha = np.asarray(self.alpha, dtype=np.float32)
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("mesh rgb must have shape (height, width, 3)")
        if depth.shape != rgb.shape[:2] or alpha.shape != rgb.shape[:2]:
            raise ValueError("mesh depth and alpha must match the RGB image size")
        object.__setattr__(self, "rgb", rgb)
        object.__setattr__(self, "depth", depth)
        object.__setattr__(self, "alpha", alpha)


@dataclass(frozen=True)
class CompositedCameraFrame:
    """Composited RGB and the representation chosen for every visible track."""

    frame_id: str
    timestamp_us: int
    rgb: np.ndarray
    track_states: Mapping[str, str]


class GaussianBufferRenderer(Protocol):
    def render(
        self,
        world_T_camera: np.ndarray,
        camera_intrinsic: np.ndarray,
        image_size: tuple[int, int],
    ) -> GaussianRenderBuffers: ...


class ObjectMeshRenderer(Protocol):
    def load(self, path: Path) -> object: ...

    def render(
        self,
        mesh: object,
        world_T_object: np.ndarray,
        world_T_camera: np.ndarray,
        camera_intrinsic: np.ndarray,
        image_size: tuple[int, int],
    ) -> MeshRenderBuffers: ...


PROXY_OCCLUSION_DEPTHS = ("occluder", "expected")


class OpacityThresholdDepth:
    """Per-pixel depth where accumulated splat opacity first reaches ``threshold``.

    Expected depth averages every splat on a ray, so on grazing road it sits
    short of the surface.  One extra gsplat pass renders, for each log-spaced
    depth edge, the opacity of splats centred no farther than that edge;
    gsplat's front-to-back depth order makes these values exact at the edges.
    The crossing is interpolated in log depth.  Pixels that never reach the
    threshold are ``inf``; opaque pixels crossing beyond ``far_m`` get ``far_m``.

    ``ground_max_tilt_deg`` drops ground-like splats from this pass, so the
    road cannot occlude the grounded vehicles drawn on it: splats whose flattest
    axis is within that angle of map +z and, with ``ground_band_m``, whose
    centre is at most that far above the local ground height (20th percentile
    height of such splats in its ``ground_cell_m`` map cell).  Upright occluders
    (poles, trunks, walls) and raised flat ones (awnings, ledges) remain.  The
    low default ``threshold`` keeps semi-transparent splat occluders (glass,
    foliage) hiding what is behind them.
    """

    def __init__(
        self, *, threshold: float = 0.3, bins: int = 64, near_m: float = 1.0, far_m: float = 150.0,
        ground_max_tilt_deg: float | None = 30.0, ground_band_m: float | None = 0.6, ground_cell_m: float = 2.0,
    ) -> None:
        if not 0.0 < threshold < 1.0:
            raise ValueError("threshold must be in (0, 1)")
        if bins < 2 or not 0.0 < near_m < far_m:
            raise ValueError("need bins >= 2 and 0 < near_m < far_m")
        if ground_max_tilt_deg is not None and not 0.0 < ground_max_tilt_deg < 90.0:
            raise ValueError("ground_max_tilt_deg must be in (0, 90)")
        self.threshold = float(threshold)
        self._log_edges = np.linspace(np.log(near_m), np.log(far_m), int(bins))
        self._ground_min_up = None if ground_max_tilt_deg is None else float(np.cos(np.radians(ground_max_tilt_deg)))
        self._ground_band_m = ground_band_m
        self._ground_cell_m = float(ground_cell_m)
        self._keep: tuple[object, object] | None = None  # (source means tensor, kept-splat mask)

    def _occluder_splats(self, tensors: Mapping[str, object]) -> dict[str, object]:
        names = ("means", "quats", "scales", "opacities")
        if self._ground_min_up is None:
            return {name: tensors[name] for name in names}
        if self._keep is None or self._keep[0] is not tensors["means"]:
            self._keep = (tensors["means"], self.non_ground_mask(
                tensors["means"], tensors["quats"], tensors["scales"], self._ground_min_up,
                band_m=self._ground_band_m, cell_m=self._ground_cell_m,
            ))
        keep = self._keep[1]
        return {name: tensors[name][keep] for name in names}

    @staticmethod
    def non_ground_mask(means, quats, scales, min_up: float, *, band_m: float | None = None, cell_m: float = 2.0):
        """False for ground splats: flattest axis within ``acos(min_up)`` of +z (and, with
        ``band_m``, centre at most ``band_m`` above its cell's 20th-percentile such splat)."""

        import torch

        w, x, y, z = torch.nn.functional.normalize(quats, dim=-1).unbind(-1)
        # z components of the rotation matrix columns (the splat's local axes).
        axis_z = torch.stack((2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)), dim=-1)
        flattest = torch.argmin(scales, dim=-1, keepdim=True)
        ground = torch.gather(axis_z, -1, flattest)[..., 0].abs() >= min_up
        if band_m is not None and bool(ground.any()):
            _, cell = torch.unique(torch.floor(means[:, :2] / cell_m).long(), dim=0, return_inverse=True)
            flat_cell, flat_z = cell[ground], means[ground, 2]
            by_z = torch.argsort(flat_z)
            order = by_z[torch.argsort(flat_cell[by_z], stable=True)]  # grouped by cell, ascending z
            counts = torch.bincount(flat_cell, minlength=int(cell.max()) + 1)
            starts = torch.cumsum(counts, 0) - counts
            pick = (starts + (counts - 1) // 5).clamp(min=0, max=len(order) - 1)
            height = torch.where(counts > 0, flat_z[order][pick], torch.full_like(pick, -1e9, dtype=means.dtype))
            ground &= means[:, 2] <= height[cell] + band_m
        return ~ground

    def render(self, tensors: Mapping[str, object], width: int, height: int, *, antialiased: bool):
        """Return a ``(1, height, width)`` torch depth image for one camera."""

        import torch
        from gsplat import rasterization

        splats, viewmats = self._occluder_splats(tensors), tensors["viewmats"]
        means = splats["means"]
        log_edges = torch.as_tensor(self._log_edges, dtype=means.dtype, device=means.device)
        camera_z = means @ viewmats[0, 2, :3] + viewmats[0, 2, 3]
        below_edge = (camera_z[:, None] <= torch.exp(log_edges)[None]).to(means.dtype)
        cumulative, alpha, _ = rasterization(
            means, splats["quats"], splats["scales"], splats["opacities"], below_edge,
            viewmats, tensors["Ks"], width, height, sh_degree=None, packed=False,
            rasterize_mode="antialiased" if antialiased else "classic",
        )
        depth = self.crossing_depth(cumulative, log_edges, self.threshold)
        beyond = torch.isinf(depth) & (alpha[..., 0] >= self.threshold)
        return torch.where(beyond, torch.exp(log_edges[-1]), depth)

    @staticmethod
    def crossing_depth(cumulative, log_edges, threshold: float):
        """Interpolate where a non-decreasing ``(..., bins)`` opacity crosses ``threshold``."""

        import torch

        reached = cumulative >= threshold
        upper = torch.argmax(reached.to(torch.uint8), dim=-1, keepdim=True)
        lower = (upper - 1).clamp(min=0)
        first = upper == 0
        opacity_upper = torch.gather(cumulative, -1, upper)
        opacity_lower = torch.where(first, 0.0, torch.gather(cumulative, -1, lower))
        log_upper = log_edges[upper]
        step = log_edges[1] - log_edges[0]
        log_lower = torch.where(first, log_edges[0] - step, log_edges[lower])
        fraction = ((threshold - opacity_lower) / (opacity_upper - opacity_lower).clamp(min=1e-6)).clamp(0.0, 1.0)
        depth = torch.exp(log_lower + fraction * (log_upper - log_lower))[..., 0]
        return torch.where(reached.any(dim=-1), depth, float("inf"))


class CudaGaussianBufferRenderer:
    """Keep static Gaussian tensors resident while rendering changing cameras."""

    def __init__(
        self,
        gaussian_map: GaussianMap,
        *,
        device: str = "cuda",
        downsample: int = 1,
        occluder_depth: OpacityThresholdDepth | None = None,
    ) -> None:
        self._gaussian_map = gaussian_map
        self._device = device
        self._downsample = downsample
        self._occluder_depth = occluder_depth
        self._tensors: dict[str, object] | None = None

    def render(self, world_T_camera, camera_intrinsic, image_size) -> GaussianRenderBuffers:
        if not self._device.startswith("cuda"):
            raise ValueError("depth-aware Gaussian rendering requires a CUDA device")
        import torch

        viewmat, intrinsic, width, height = prepare_camera_render(
            world_T_camera, camera_intrinsic, image_size, downsample=self._downsample
        )
        if self._tensors is None:
            self._tensors = {
                "means": torch.as_tensor(self._gaussian_map.means, dtype=torch.float32, device=self._device),
                "quats": torch.as_tensor(self._gaussian_map.quats, dtype=torch.float32, device=self._device),
                "scales": torch.as_tensor(self._gaussian_map.scales, dtype=torch.float32, device=self._device),
                "opacities": torch.as_tensor(self._gaussian_map.opacities, dtype=torch.float32, device=self._device),
                "colors": torch.as_tensor(self._gaussian_map.sh_coeffs, dtype=torch.float32, device=self._device),
            }
        tensors = {
            **self._tensors,
            "viewmats": torch.as_tensor(viewmat[None], dtype=torch.float32, device=self._device),
            "Ks": torch.as_tensor(intrinsic[None], dtype=torch.float32, device=self._device),
        }
        rendered, alphas, _ = _rasterize(
            tensors, self._gaussian_map, width, height, render_mode="RGB+ED"
        )
        rendered = _single_camera_array(rendered, "rendered RGB+depth")
        alpha = _single_camera_array(alphas, "rendered alpha")
        if alpha.shape == (height, width, 1):
            alpha = alpha[..., 0]
        occluder = None if self._occluder_depth is None else _single_camera_array(
            self._occluder_depth.render(tensors, width, height, antialiased=self._gaussian_map.antialiased),
            "rendered occluder depth",
        )
        return GaussianRenderBuffers(rendered[..., :3], rendered[..., 3], alpha, occluder_depth=occluder)


class PyTorch3DObjectMeshRenderer:
    """Cache a decimated PyTorch3D GLB representation for camera overlays."""

    def __init__(self, *, device: str = "cuda", max_faces: int = 100_000) -> None:
        if max_faces < 4:
            raise ValueError("max_faces must be at least four")
        self._device = device
        self._max_faces = max_faces

    def load(self, path: Path) -> object:
        try:
            import torch
            import trimesh
            from pytorch3d.renderer import TexturesVertex
            from pytorch3d.structures import Meshes
        except ImportError as error:
            raise RuntimeError("PyTorch3D, trimesh, and torch are required for mesh overlays") from error

        scene = trimesh.load(path, force="scene", process=False)
        if not isinstance(scene, trimesh.Scene) or not scene.geometry:
            raise ValueError(f"mesh contains no geometry: {path}")
        # GLB nodes may carry transforms; bake them before constructing the
        # PyTorch3D mesh so the aligned asset stays in its canonical object frame.
        geometry = scene.to_geometry()
        return self.load_geometry(geometry)

    def load_geometry(self, geometry, *, preserve_topology: bool = False) -> object:
        """Upload colored geometry; articulated proxies retain vertex indices."""
        import torch
        from pytorch3d.renderer import TexturesVertex
        from pytorch3d.structures import Meshes

        vertices = np.asarray(geometry.vertices, dtype=np.float32)
        faces = np.asarray(geometry.faces, dtype=np.int64)
        colors = np.full((len(vertices), 3), 0.7, dtype=np.float32)
        vertex_colors = getattr(geometry.visual, "vertex_colors", None)
        if vertex_colors is not None and len(vertex_colors) == len(vertices):
            colors = np.asarray(vertex_colors[:, :3], dtype=np.float32) / 255.0
        if not preserve_topology:
            vertices, faces, colors = _decimate_mesh_for_render(
                vertices, faces, colors, max_faces=self._max_faces
            )
        return Meshes(
            verts=[torch.as_tensor(vertices, device=self._device)],
            faces=[torch.as_tensor(faces, device=self._device)],
            textures=TexturesVertex([torch.as_tensor(colors, device=self._device)]),
        )

    def with_vertices(self, mesh, vertices):
        """Instance new local vertices without changing cached topology/colors."""
        import torch

        return mesh.update_padded(
            torch.as_tensor(vertices, dtype=torch.float32, device=self._device)[None]
        )

    def render(
        self, mesh, world_T_object, world_T_camera, camera_intrinsic, image_size,
        *, cull_backfaces: bool = False,
    ) -> MeshRenderBuffers:
        import torch
        from pytorch3d.ops import interpolate_face_attributes
        from pytorch3d.renderer import MeshRasterizer, RasterizationSettings
        from pytorch3d.utils import cameras_from_opencv_projection

        width, height = image_size
        dtype = torch.float32
        device = self._device
        object_pose = torch.as_tensor(world_T_object, dtype=dtype, device=device)
        local_vertices = mesh.verts_padded()
        world_vertices = local_vertices @ object_pose[:3, :3].T + object_pose[:3, 3]
        world_mesh = mesh.update_padded(world_vertices)

        camera_T_world = torch.linalg.inv(
            torch.as_tensor(world_T_camera, dtype=dtype, device=device)
        )
        rotation = camera_T_world[:3, :3][None]
        translation = camera_T_world[:3, 3][None]
        intrinsic = torch.as_tensor(camera_intrinsic, dtype=dtype, device=device)[None]
        cameras = cameras_from_opencv_projection(
            R=rotation,
            tvec=translation,
            camera_matrix=intrinsic,
            image_size=torch.tensor([[height, width]], dtype=dtype, device=device),
        )
        fragments = MeshRasterizer(
            cameras=cameras,
            raster_settings=RasterizationSettings(
                image_size=(height, width), blur_radius=0.0, faces_per_pixel=1,
                cull_backfaces=cull_backfaces,
            ),
        )(world_mesh)
        alpha = fragments.pix_to_face[0, ..., 0] >= 0
        colors = world_mesh.sample_textures(fragments)[0, ..., 0, :3]

        camera_vertices = world_vertices @ rotation[0].T + translation[0]
        faces = world_mesh.faces_packed()
        face_depth = camera_vertices[0, faces, 2, None]
        depth = interpolate_face_attributes(
            fragments.pix_to_face, fragments.bary_coords, face_depth
        )[0, ..., 0, 0]
        depth = torch.where(alpha, depth, torch.full_like(depth, float("inf")))
        return MeshRenderBuffers(
            colors.detach().cpu().numpy(),
            depth.detach().cpu().numpy(),
            alpha.detach().float().cpu().numpy(),
        )


def _decimate_mesh_for_render(
    vertices: np.ndarray, faces: np.ndarray, colors: np.ndarray, *, max_faces: int
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Reduce a dense mesh once with deterministic vertex clustering.

    The source GLB remains untouched.  Clustering preserves broad surface
    coverage better than sampling isolated faces and yields a compact immutable
    mesh that can remain resident on the compositor GPU.
    """

    if len(faces) <= max_faces:
        return vertices, faces, colors
    lower = vertices.min(axis=0)
    extent = vertices.max(axis=0) - lower
    active = extent > np.finfo(np.float32).eps
    if not np.any(active):
        return vertices, faces, colors
    target_vertices = max(4, max_faces // 2)
    for _ in range(8):
        volume = float(np.prod(extent[active]))
        base = (target_vertices / volume) ** (1.0 / active.sum())
        bins = np.ones(3, dtype=np.int64)
        bins[active] = np.maximum(1, np.floor(extent[active] * base).astype(np.int64))
        coordinates = np.minimum(
            ((vertices - lower) / np.where(active, extent, 1.0) * bins).astype(np.int64), bins - 1
        )
        _, inverse = np.unique(coordinates, axis=0, return_inverse=True)
        counts = np.bincount(inverse).astype(np.float32)
        reduced_vertices = np.empty((len(counts), 3), dtype=np.float32)
        reduced_colors = np.empty((len(counts), 3), dtype=np.float32)
        for axis in range(3):
            reduced_vertices[:, axis] = np.bincount(
                inverse, weights=vertices[:, axis], minlength=len(counts)
            ) / counts
            reduced_colors[:, axis] = np.bincount(
                inverse, weights=colors[:, axis], minlength=len(counts)
            ) / counts
        reduced_faces = inverse[faces]
        reduced_faces = reduced_faces[
            (reduced_faces[:, 0] != reduced_faces[:, 1])
            & (reduced_faces[:, 1] != reduced_faces[:, 2])
            & (reduced_faces[:, 0] != reduced_faces[:, 2])
        ]
        if len(reduced_faces) == 0:
            return vertices, faces, colors
        canonical = np.sort(reduced_faces, axis=1)
        _, unique = np.unique(canonical, axis=0, return_index=True)
        reduced_faces = reduced_faces[np.sort(unique)]
        used, reduced_faces = np.unique(reduced_faces.reshape(-1), return_inverse=True)
        reduced_faces = reduced_faces.reshape(-1, 3).astype(np.int64)
        reduced_vertices = reduced_vertices[used]
        reduced_colors = reduced_colors[used]
        if len(reduced_faces) <= max_faces:
            return reduced_vertices, reduced_faces, reduced_colors
        target_vertices = max(4, target_vertices // 2)
    return reduced_vertices, reduced_faces, reduced_colors


@dataclass
class _MeshAsset:
    state: AssetState
    request_id: str
    path: Path | None = None
    handle: object | None = None
    source_sequence: int | None = None
    source_timestamp_us: int | None = None


def _error_detail(error: Exception) -> str:
    """Keep renderer failures searchable without retaining exception objects."""

    return f"{type(error).__name__}: {error}"


class CompositedCameraBackend:
    """Own proxy-to-mesh state while producing non-blocking camera overlays.

    A ready mesh is cached for its scene generation, even while its track is
    temporarily absent. Changing scene generation clears all mesh handles and
    makes delayed events from the retired scene harmless.
    """

    def __init__(
        self,
        gaussian_renderer: GaussianBufferRenderer,
        mesh_renderer: ObjectMeshRenderer,
        *,
        proxy_color: tuple[float, float, float] = (0.0, 0.7, 1.0),
        proxy_thickness: int = 2,
        alpha_threshold: float = 0.01,
        depth_tolerance_m: float = 0.03,
        ground_z: float | None = None,
    ) -> None:
        if proxy_thickness <= 0:
            raise ValueError("proxy_thickness must be positive")
        self._gaussian_renderer = gaussian_renderer
        self._mesh_renderer = mesh_renderer
        self._proxy_color = np.asarray(proxy_color, dtype=np.float32)
        self._proxy_thickness = proxy_thickness
        self._alpha_threshold = float(alpha_threshold)
        self._depth_tolerance_m = float(depth_tolerance_m)
        self._ground_z = None if ground_z is None else float(ground_z)
        self._generation: str | None = None
        self._retired_generations: set[str] = set()
        self._assets: dict[str, _MeshAsset] = {}
        self._live_tracks: set[str] = set()

    @property
    def live_tracks(self) -> frozenset[str]:
        return frozenset(self._live_tracks)

    def handle_asset_event(self, event: AssetEvent) -> None:
        """Cache a ready mesh; failed assets intentionally remain proxy-only."""

        generation = str(event.scene_generation)
        if generation in self._retired_generations:
            return
        if self._generation is None:
            self._generation = generation
        if self._generation is not None and generation != self._generation:
            return
        previous = self._assets.get(event.track_id)
        if previous is not None and previous.request_id != event.request_id:
            return
        if previous is not None and previous.state in {
            AssetState.READY, AssetState.FAILED, AssetState.CANCELLED
        }:
            return
        if event.state is AssetState.READY:
            path = Path(event.aligned_mesh_path)  # validated by AssetEvent
            if not path.is_file():
                self._assets[event.track_id] = _MeshAsset(
                    AssetState.FAILED, event.request_id
                )
                return
            try:
                handle = self._mesh_renderer.load(path)
            except Exception:
                self._assets[event.track_id] = _MeshAsset(
                    AssetState.FAILED, event.request_id
                )
                return
            self._assets[event.track_id] = _MeshAsset(
                AssetState.READY, event.request_id, path, handle
            )
            return
        self._assets[event.track_id] = _MeshAsset(event.state, event.request_id)

    def render_frame(self, frame: FrameDetections) -> CompositedCameraFrame:
        """Render the latest complete world snapshot without invoking reconstruction."""

        generation = str(frame.scene_generation)
        self._activate_generation(generation)
        image_size = _jpeg_size(frame.image_jpeg)
        intrinsic = np.asarray(frame.camera_intrinsic, dtype=np.float64).reshape(3, 3)
        world_T_camera = np.asarray(frame.world_T_camera, dtype=np.float64).reshape(4, 4)
        background = self._gaussian_renderer.render(
            world_T_camera, intrinsic, image_size
        )
        rgb = np.clip(background.rgb.copy(), 0.0, 1.0)
        object_depth = np.full(background.depth.shape, np.inf, dtype=np.float32)
        actual_size = (rgb.shape[1], rgb.shape[0])
        if actual_size != image_size:
            scale_x = actual_size[0] / image_size[0]
            scale_y = actual_size[1] / image_size[1]
            intrinsic = intrinsic.copy()
            intrinsic[0] *= scale_x
            intrinsic[1] *= scale_y
            image_size = actual_size

        self._live_tracks = {box.track_id for box in frame.boxes}
        states: dict[str, str] = {}
        for box in frame.boxes:
            asset = self._assets.get(box.track_id)
            if asset is not None and asset.state is AssetState.READY:
                try:
                    overlay = self._mesh_renderer.render(
                        asset.handle,
                        _flat_ground_box_pose(box, ground_z=self._ground_z),
                        world_T_camera,
                        intrinsic,
                        image_size,
                    )
                    rgb = _composite_mesh(
                        rgb, background.depth, background.alpha, overlay, object_depth=object_depth,
                        alpha_threshold=self._alpha_threshold,
                        depth_tolerance_m=self._depth_tolerance_m,
                    )
                    states[box.track_id] = "mesh"
                    continue
                except Exception:
                    asset.state = AssetState.FAILED
            states[box.track_id] = "proxy"
            _draw_box_proxy(
                rgb,
                background.depth,
                background.alpha,
                box,
                world_T_camera,
                intrinsic,
                self._proxy_color,
                self._proxy_thickness,
                self._alpha_threshold,
                self._depth_tolerance_m,
                ground_z=self._ground_z, object_depth=object_depth,
            )
        return CompositedCameraFrame(frame.frame_id, frame.timestamp_us, rgb, states)

    def _activate_generation(self, generation: str) -> None:
        if self._generation == generation:
            return
        if self._generation is not None:
            self._retired_generations.add(self._generation)
        self._generation = generation
        self._assets.clear()
        self._live_tracks.clear()


def _draw_box_proxy(
    rgb: np.ndarray,
    scene_depth: np.ndarray,
    scene_alpha: np.ndarray,
    box: BoxPrompt,
    world_T_camera: np.ndarray,
    intrinsic: np.ndarray,
    color: np.ndarray,
    thickness: int,
    alpha_threshold: float,
    depth_tolerance_m: float,
    *,
    ground_z: float | None = None,
    ground_plane: GroundPlane | GroundSurface | None = None,
    object_depth: np.ndarray | None = None,
    placed_pose: np.ndarray | None = None,
) -> None:
    local = _box_local_corners(box.dimensions_lwh)
    world_T_object = (placed_pose if placed_pose is not None else
                      _flat_ground_box_pose(box, ground_z=ground_z, ground_plane=ground_plane))
    world = local @ world_T_object[:3, :3].T + world_T_object[:3, 3]
    camera_T_world = np.linalg.inv(world_T_camera)
    camera = world @ camera_T_world[:3, :3].T + camera_T_world[:3, 3]
    if np.any(camera[:, 2] <= 1e-3):
        return
    projected = camera @ intrinsic.T
    pixels = projected[:, :2] / projected[:, 2, None]
    for start, end in _BOX_EDGES:
        _draw_depth_tested_edge(
            rgb, scene_depth, scene_alpha, pixels[start], pixels[end],
            camera[start, 2], camera[end, 2], color, thickness,
            alpha_threshold, depth_tolerance_m, shared_depth=object_depth,
        )


def _draw_track_label(rgb, box, placed_pose, world_T_camera, intrinsic, color) -> None:
    """Draw the short track ID above the box's top face center."""

    top = placed_pose[:3, :3] @ np.array([0.0, 0.0, box.dimensions_lwh[2] / 2.0]) + placed_pose[:3, 3]
    camera_T_world = np.linalg.inv(world_T_camera)
    point = camera_T_world[:3, :3] @ top + camera_T_world[:3, 3]
    if point[2] <= 1e-3:
        return
    u, v = (intrinsic @ point)[:2] / point[2]
    height, width = rgb.shape[:2]
    if not (0 <= u < width and 0 <= v < height):
        return
    text = box.track_id.rsplit(":", 1)[-1]
    (text_w, text_h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    x, y = int(u - text_w / 2), int(v) - 6
    cv2.rectangle(rgb, (x - 2, y - text_h - 3), (x + text_w + 2, y + 3), tuple(float(c) for c in color), -1)
    cv2.putText(rgb, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (1.0, 1.0, 1.0), 1, cv2.LINE_AA)


def _flat_ground_box_pose(
    box: BoxPrompt, *, ground_z: float | None = None,
    ground_plane: GroundPlane | GroundSurface | None = None,
) -> np.ndarray:
    """Anchor an oriented GT box to the flat GSplat ground without changing x/y."""

    world_T_object = _matrix4(box.world_T_object).copy()
    if ground_plane is not None:
        if ground_z is not None:
            raise ValueError("ground_plane requires ground_z=None")
        return ground_plane.place_box(world_T_object, box.dimensions_lwh)
    if ground_z is None:
        return world_T_object
    local = _box_local_corners(box.dimensions_lwh)
    lowest_z = (local @ world_T_object[:3, :3].T + world_T_object[:3, 3])[:, 2].min()
    world_T_object[2, 3] += ground_z - lowest_z
    return world_T_object


def _box_local_corners(dimensions_lwh: tuple[float, float, float]) -> np.ndarray:
    dimensions = np.asarray(dimensions_lwh, dtype=np.float64)
    signs = np.array(
        [[-1, -1, -1], [-1, -1, 1], [-1, 1, -1], [-1, 1, 1],
         [1, -1, -1], [1, -1, 1], [1, 1, -1], [1, 1, 1]], dtype=np.float64
    )
    return signs * dimensions / 2.0


_BOX_EDGES = (
    (0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
    (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7),
)


def _draw_depth_tested_edge(
    rgb, scene_depth, scene_alpha, start, end, start_depth, end_depth,
    color, thickness, alpha_threshold, depth_tolerance_m, *, shared_depth=None,
) -> None:
    length = int(np.ceil(np.max(np.abs(end - start)))) + 1
    if length <= 1:
        return
    parameter = np.linspace(0.0, 1.0, length)
    pixels = np.rint(start[None] * (1.0 - parameter[:, None]) + end[None] * parameter[:, None]).astype(int)
    inverse_depth = (1.0 - parameter) / start_depth + parameter / end_depth
    depths = 1.0 / inverse_depth
    height, width = scene_depth.shape
    radius = thickness // 2
    for offset_y in range(-radius, radius + 1):
        for offset_x in range(-radius, radius + 1):
            x = pixels[:, 0] + offset_x
            y = pixels[:, 1] + offset_y
            inside = (x >= 0) & (x < width) & (y >= 0) & (y < height)
            x, y, object_depth = x[inside], y[inside], depths[inside]
            if not len(x):
                continue
            gaussian_valid = (
                np.isfinite(scene_depth[y, x])
                & (scene_depth[y, x] > 0.0)
                & (scene_alpha[y, x] > alpha_threshold)
            )
            visible = ~gaussian_valid | (object_depth <= scene_depth[y, x] + depth_tolerance_m)
            if shared_depth is not None:
                visible &= object_depth < shared_depth[y, x]
                np.minimum.at(shared_depth, (y[visible], x[visible]), object_depth[visible])
            rgb[y[visible], x[visible]] = color


def _composite_mesh(
    background_rgb, scene_depth, scene_alpha, mesh: MeshRenderBuffers, *,
    alpha_threshold: float, depth_tolerance_m: float,
    object_depth: np.ndarray | None = None,
) -> np.ndarray:
    if mesh.rgb.shape != background_rgb.shape:
        raise ValueError("mesh and Gaussian RGB sizes differ")
    gaussian_valid = (
        np.isfinite(scene_depth) & (scene_depth > 0.0) & (scene_alpha > alpha_threshold)
    )
    mesh_valid = (
        (mesh.alpha > 0.0) & np.isfinite(mesh.depth) & (mesh.depth > 0.0)
        & (~gaussian_valid | (mesh.depth <= scene_depth + depth_tolerance_m))
    )
    if object_depth is not None:
        # Gaussian tolerance compensates for noisy splat depths; applying it to
        # vehicles would allow a farther surface to overwrite a nearer one.
        mesh_valid &= mesh.depth < object_depth
        object_depth[mesh_valid] = mesh.depth[mesh_valid]
    alpha = np.where(mesh_valid, np.clip(mesh.alpha, 0.0, 1.0), 0.0)[..., None]
    return mesh.rgb * alpha + background_rgb * (1.0 - alpha)


def _matrix4(values) -> np.ndarray:
    matrix = np.asarray(values, dtype=np.float64).reshape(4, 4)
    if not np.isfinite(matrix).all() or not np.allclose(matrix[3], [0, 0, 0, 1]):
        raise ValueError("pose must be a finite homogeneous transform")
    return matrix


def _rigid_transform(values: np.ndarray, name: str) -> np.ndarray:
    transform = _matrix4(values)
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not np.isclose(
        np.linalg.det(rotation), 1.0, atol=1e-6
    ):
        raise ValueError(f"{name} rotation must be orthonormal and right-handed")
    return transform


def _jpeg_size(payload: bytes) -> tuple[int, int]:
    image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("image_jpeg is not a decodable JPEG image")
    return image.shape[1], image.shape[0]

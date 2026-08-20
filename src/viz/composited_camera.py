"""Depth-aware GSplat camera composition for asynchronous object meshes."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Protocol

import cv2
import numpy as np

from src.localization.gaussian_map import GaussianMap
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


@dataclass(frozen=True)
class PoseOnlyCompositedFrame:
    """A GSplat frame rendered solely from an ego pose and local calibration."""

    sequence: int
    timestamp_us: int
    scene_generation: str
    world_T_camera: np.ndarray
    rgb: np.ndarray
    track_states: Mapping[str, str]


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
    ) -> None:
        if proxy_thickness <= 0:
            raise ValueError("proxy_thickness must be positive")
        self._gaussian_renderer = gaussian_renderer
        self.camera = camera
        self._mesh_renderer = mesh_renderer
        self._proxy_color = np.asarray(proxy_color, dtype=np.float32)
        self._proxy_thickness = proxy_thickness
        self._alpha_threshold = float(alpha_threshold)
        self._depth_tolerance_m = float(depth_tolerance_m)
        self._scene_generation: str | None = None
        self._retired_generations: set[str] = set()
        self._assets: dict[str, _MeshAsset] = {}
        self._mesh_statuses: list[MeshAssetStatus] = []

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
        world_T_camera = world_T_ego @ self.camera.ego_T_camera
        background = self._gaussian_renderer.render(
            world_T_camera, self.camera.camera_intrinsic, self.camera.image_size
        )
        if background.rgb.shape[:2] != self.camera.image_size[::-1]:
            raise ValueError("Gaussian renderer returned an unexpected image size")
        rgb = np.clip(background.rgb.copy(), 0.0, 1.0)
        track_states: dict[str, str] = {}
        for box in boxes:
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
                        _matrix4(box.world_T_object),
                        world_T_camera,
                        self.camera.camera_intrinsic,
                        self.camera.image_size,
                    )
                    rgb = _composite_mesh(
                        rgb, background.depth, background.alpha, overlay,
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
                self._proxy_color,
                self._proxy_thickness,
                self._alpha_threshold,
                self._depth_tolerance_m,
            )
            track_states[box.track_id] = "proxy"
        return PoseOnlyCompositedFrame(
            sequence=int(pose_sample.sequence),
            timestamp_us=int(pose_sample.timestamp_us),
            scene_generation=generation,
            world_T_camera=world_T_camera,
            rgb=rgb,
            track_states=track_states,
        )

    def _activate_generation(self, generation: str) -> None:
        if self._scene_generation == generation:
            return
        if self._scene_generation is not None:
            raise ValueError(
                f"mismatched scene generation {generation!r}; expected {self._scene_generation!r}"
            )
        self._scene_generation = generation

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


class CudaGaussianBufferRenderer:
    """Keep static Gaussian tensors resident while rendering changing cameras."""

    def __init__(
        self,
        gaussian_map: GaussianMap,
        *,
        device: str = "cuda",
        downsample: int = 1,
    ) -> None:
        self._gaussian_map = gaussian_map
        self._device = device
        self._downsample = downsample
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
        return GaussianRenderBuffers(rendered[..., :3], rendered[..., 3], alpha)


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
        vertices = np.asarray(geometry.vertices, dtype=np.float32)
        faces = np.asarray(geometry.faces, dtype=np.int64)
        colors = np.full((len(vertices), 3), 0.7, dtype=np.float32)
        vertex_colors = getattr(geometry.visual, "vertex_colors", None)
        if vertex_colors is not None and len(vertex_colors) == len(vertices):
            colors = np.asarray(vertex_colors[:, :3], dtype=np.float32) / 255.0
        vertices, faces, colors = _decimate_mesh_for_render(
            vertices, faces, colors, max_faces=self._max_faces
        )
        return Meshes(
            verts=[torch.as_tensor(vertices, device=self._device)],
            faces=[torch.as_tensor(faces, device=self._device)],
            textures=TexturesVertex([torch.as_tensor(colors, device=self._device)]),
        )

    def render(
        self, mesh, world_T_object, world_T_camera, camera_intrinsic, image_size
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
                image_size=(height, width), blur_radius=0.0, faces_per_pixel=1
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
    ) -> None:
        if proxy_thickness <= 0:
            raise ValueError("proxy_thickness must be positive")
        self._gaussian_renderer = gaussian_renderer
        self._mesh_renderer = mesh_renderer
        self._proxy_color = np.asarray(proxy_color, dtype=np.float32)
        self._proxy_thickness = proxy_thickness
        self._alpha_threshold = float(alpha_threshold)
        self._depth_tolerance_m = float(depth_tolerance_m)
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
                        _matrix4(box.world_T_object),
                        world_T_camera,
                        intrinsic,
                        image_size,
                    )
                    rgb = _composite_mesh(
                        rgb, background.depth, background.alpha, overlay,
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
) -> None:
    dimensions = np.asarray(box.dimensions_lwh, dtype=np.float64)
    signs = np.array(
        [[-1, -1, -1], [-1, -1, 1], [-1, 1, -1], [-1, 1, 1],
         [1, -1, -1], [1, -1, 1], [1, 1, -1], [1, 1, 1]], dtype=np.float64
    )
    local = signs * dimensions / 2.0
    world_T_object = _matrix4(box.world_T_object)
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
            alpha_threshold, depth_tolerance_m,
        )


_BOX_EDGES = (
    (0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
    (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7),
)


def _draw_depth_tested_edge(
    rgb, scene_depth, scene_alpha, start, end, start_depth, end_depth,
    color, thickness, alpha_threshold, depth_tolerance_m,
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
            rgb[y[visible], x[visible]] = color


def _composite_mesh(
    background_rgb, scene_depth, scene_alpha, mesh: MeshRenderBuffers, *,
    alpha_threshold: float, depth_tolerance_m: float,
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

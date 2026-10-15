"""Cached vehicle templates with independently positioned metric wheels."""
from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from src.viz.primitive_vehicle import PrimitiveVehicleAsset
from src.viz.universal_vehicle import UniversalVehicleMesh, UniversalVehicleShape

if TYPE_CHECKING:
    from src.viz.composited_camera import PyTorch3DObjectMeshRenderer


class VehicleProxyRenderer:
    """Cache one GPU template per kind; update vertices without GLB roundtrips.

    Car proxies with a shape hint use one cached universal template instead of
    the legacy primitive; without a hint every kind renders its primitive.
    """

    def __init__(self, renderer: PyTorch3DObjectMeshRenderer, *, asset_dir: Path | None = None):
        self._renderer = renderer
        self._asset_dir = asset_dir
        self._templates: dict[str, tuple[PrimitiveVehicleAsset, object]] = {}
        self._universal: tuple[UniversalVehicleMesh, object] | None = None

    def render(self, kind, dimensions, world_T_object, world_T_camera, intrinsic, image_size,
               shape: UniversalVehicleShape | None = None, body_color_rgb=None):
        """``shape`` applies to car proxies only; ``body_color_rgb`` repaints body panels."""
        if kind == "car" and shape is not None:
            if self._universal is None:
                mesh = UniversalVehicleMesh()
                self._universal = mesh, self._renderer.load_geometry(mesh.template_geometry(),
                                                                     preserve_topology=True)
            mesh, handle = self._universal
            vertices, colors = mesh.instance(shape, dimensions, body_color_rgb)
        else:
            if kind not in self._templates:
                asset = PrimitiveVehicleAsset(kind)
                handle = self._renderer.load_geometry(asset.mesh, preserve_topology=True)
                if self._asset_dir is not None:
                    asset.export(self._asset_dir / f"{kind}.glb")
                self._templates[kind] = asset, handle
            asset, handle = self._templates[kind]
            vertices = asset.vertices_for_dimensions(dimensions)
            colors = None if body_color_rgb is None else asset.vertex_colors_for_tint(body_color_rgb)
        if kind == "car":
            object_T_camera = np.linalg.inv(np.asarray(world_T_object, dtype=np.float64)) @ np.asarray(
                world_T_camera, dtype=np.float64)
            if np.all(np.abs(object_T_camera[:3, 3]) <= np.asarray(dimensions, dtype=np.float64) / 2):
                # An enclosing car is not an external vehicle overlay. Its hood
                # can still face the camera even with backface culling enabled.
                from src.viz.composited_camera import MeshRenderBuffers

                width, height = image_size
                return MeshRenderBuffers(
                    np.zeros((height, width, 3), dtype=np.float32),
                    np.full((height, width), np.inf, dtype=np.float32),
                    np.zeros((height, width), dtype=np.float32),
                )
        instance = self._renderer.with_vertices(handle, vertices)
        with_vertex_colors = getattr(self._renderer, "with_vertex_colors", None)
        if colors is not None and with_vertex_colors is not None:
            # Per-instance colors (paint; universal glass and bed follow the shape).
            instance = with_vertex_colors(instance, colors)
        # The car templates' exterior views are verified with culling. Other
        # templates retain two-sided rendering for their open surface details.
        return self._renderer.render(instance, world_T_object, world_T_camera, intrinsic, image_size,
                                     cull_backfaces=kind == "car")

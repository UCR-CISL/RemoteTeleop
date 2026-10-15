"""Cached vehicle templates with independently positioned metric wheels."""
from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from src.viz.primitive_vehicle import CarShapeParams, PrimitiveVehicleAsset

if TYPE_CHECKING:
    from src.viz.composited_camera import PyTorch3DObjectMeshRenderer


class VehicleProxyRenderer:
    """Cache one GPU template per kind; update vertices without GLB roundtrips."""

    def __init__(self, renderer: PyTorch3DObjectMeshRenderer, *, asset_dir: Path | None = None):
        self._renderer = renderer
        self._asset_dir = asset_dir
        self._templates: dict[str, tuple[PrimitiveVehicleAsset, object]] = {}

    def render(self, kind, dimensions, world_T_object, world_T_camera, intrinsic, image_size,
               shape: CarShapeParams | None = None, body_color_rgb=None):
        """``shape`` applies to car proxies only; ``body_color_rgb`` repaints body panels."""
        if kind not in self._templates:
            asset = PrimitiveVehicleAsset(kind)
            handle = self._renderer.load_geometry(asset.mesh, preserve_topology=True)
            if self._asset_dir is not None:
                asset.export(self._asset_dir / f"{kind}.glb")
            self._templates[kind] = asset, handle
        asset, handle = self._templates[kind]
        vertices = asset.vertices_for_dimensions(dimensions, shape if kind == "car" else None)
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
        if body_color_rgb is not None and with_vertex_colors is not None:
            # Per-instance colors; the cached template texture stays shared.
            instance = with_vertex_colors(instance, asset.vertex_colors_for_tint(body_color_rgb))
        # The car template's exterior views are verified with culling. Other
        # templates retain two-sided rendering for their open surface details.
        return self._renderer.render(instance, world_T_object, world_T_camera, intrinsic, image_size,
                                     cull_backfaces=kind == "car")

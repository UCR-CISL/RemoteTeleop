"""Cached vehicle templates with independently positioned metric wheels."""
from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from src.viz.primitive_vehicle import PrimitiveVehicleAsset

if TYPE_CHECKING:
    from src.viz.composited_camera import PyTorch3DObjectMeshRenderer


class VehicleProxyRenderer:
    """Cache one GPU template per kind; update vertices without GLB roundtrips."""

    def __init__(self, renderer: PyTorch3DObjectMeshRenderer, *, asset_dir: Path | None = None):
        self._renderer = renderer
        self._asset_dir = asset_dir
        self._templates: dict[str, tuple[PrimitiveVehicleAsset, object]] = {}

    def render(self, kind, dimensions, world_T_object, world_T_camera, intrinsic, image_size):
        if kind not in self._templates:
            asset = PrimitiveVehicleAsset(kind)
            handle = self._renderer.load_geometry(asset.mesh, preserve_topology=True)
            if self._asset_dir is not None:
                asset.export(self._asset_dir / f"{kind}.glb")
            self._templates[kind] = asset, handle
        asset, handle = self._templates[kind]
        instance = self._renderer.with_vertices(handle, asset.vertices_for_dimensions(dimensions))
        return self._renderer.render(instance, world_T_object, world_T_camera, intrinsic, image_size)

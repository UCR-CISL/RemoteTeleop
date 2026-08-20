"""Lazy vehicle/remote deployment boundary for dynamic mesh overlays."""

from __future__ import annotations

from importlib import import_module
from typing import Any


_EXPORT_MODULES = {
    "RemoteAssetCache": "src.deployment.remote",
    "RemoteCompositionRuntime": "src.deployment.remote",
    "ResumableRemoteAssetCache": "src.deployment.remote",
    "RosFrameAdapter": "src.deployment.ros_frame_adapter",
    "RosFrameAdapterConfig": "src.deployment.ros_frame_adapter",
    "AssetStorePublisher": "src.deployment.vehicle",
    "VehicleMeshRuntime": "src.deployment.vehicle",
    "VehicleMeshRuntimeConfig": "src.deployment.vehicle",
    "VehicleProcessGraph": "src.deployment.vehicle",
    "VehicleAssetStore": "src.deployment.vehicle",
    "VehicleAssetSession": "src.deployment.vehicle",
    "VehicleFrameSession": "src.deployment.vehicle",
    "VehicleFrameSpool": "src.deployment.vehicle",
    "publish_mesh_asset": "src.deployment.vehicle",
}

__all__ = list(_EXPORT_MODULES)


def __getattr__(name: str) -> Any:
    """Import role-specific dependencies only when that export is requested."""
    try:
        module_name = _EXPORT_MODULES[name]
    except KeyError as error:
        raise AttributeError(name) from error
    value = getattr(import_module(module_name), name)
    globals()[name] = value
    return value

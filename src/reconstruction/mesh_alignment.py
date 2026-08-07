"""Metric normalization of reconstructed meshes."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from .models import VehicleDimensions


@dataclass(frozen=True)
class MeshAlignment:
    output_path: Path
    object_T_raw_mesh: NDArray[np.float64]
    uniform_scale: float
    predicted_dimensions: NDArray[np.float64]
    aligned_dimensions: NDArray[np.float64]
    dimension_residual: NDArray[np.float64]
    ground_offset: float
    center_error: float
    ground_error: float


class MeshAligner:
    """Center, orient, isotropically scale, and ground a mesh scene."""

    def align(
        self,
        input_path: Path,
        output_path: Path,
        measured: VehicleDimensions,
        *,
        canonical_T_raw: NDArray[np.float64] | None = None,
        canonicalize_principal_axes: bool = True,
    ) -> MeshAlignment:
        """Align a GLB while preserving its scene graph and material assets.

        ``canonical_T_raw`` expresses any camera-relative orientation cue
        resolved by the caller.  The default assumes the reconstructed mesh
        already uses project axes (X forward, Y left, Z up).
        """

        try:
            import trimesh
        except ImportError as exc:
            raise RuntimeError(
                "trimesh is not installed; install the reconstruction dependency group"
            ) from exc

        scene = trimesh.load(input_path, force="scene", process=False)
        if not isinstance(scene, trimesh.Scene) or len(scene.geometry) == 0:
            raise ValueError(f"mesh contains no geometry: {input_path}")
        scene = scene.copy()
        orientation = (
            np.eye(4, dtype=np.float64)
            if canonical_T_raw is None
            else np.asarray(canonical_T_raw, dtype=np.float64)
        )
        if orientation.shape != (4, 4) or not np.all(np.isfinite(orientation)):
            raise ValueError("canonical_T_raw must be a finite 4x4 matrix")
        rotation = orientation[:3, :3]
        if (
            not np.allclose(orientation[3], (0.0, 0.0, 0.0, 1.0), atol=1e-8)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6)
        ):
            raise ValueError("canonical_T_raw must be a right-handed rigid transform")
        scene.apply_transform(orientation)
        axis_transform = np.eye(4, dtype=np.float64)
        if canonicalize_principal_axes:
            axis_transform = _principal_axis_transform(scene)
            scene.apply_transform(axis_transform)

        bounds = np.asarray(scene.bounds, dtype=np.float64)
        if bounds.shape != (2, 3) or not np.all(np.isfinite(bounds)):
            raise ValueError("mesh has invalid bounds")
        predicted = bounds[1] - bounds[0]
        if np.any(predicted <= 1e-9):
            raise ValueError("mesh has degenerate bounds")
        scale = float(np.median(measured.as_array / predicted))
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError("computed mesh scale is invalid")

        center = (bounds[0] + bounds[1]) / 2.0
        center_transform = np.eye(4)
        center_transform[:3, 3] = -center
        scale_transform = np.diag((scale, scale, scale, 1.0))
        scene.apply_transform(center_transform)
        scene.apply_transform(scale_transform)

        scaled_bounds = np.asarray(scene.bounds, dtype=np.float64)
        # nuScenes object poses are at the 3D-box center. Put the mesh's lowest
        # point on that box's metric ground plane, not at the object origin.
        ground_offset = float(-measured.height / 2.0 - scaled_bounds[0, 2])
        ground_transform = np.eye(4)
        ground_transform[2, 3] = ground_offset
        scene.apply_transform(ground_transform)

        final_bounds = np.asarray(scene.bounds, dtype=np.float64)
        aligned = np.asarray(final_bounds[1] - final_bounds[0], dtype=np.float64)
        center_error = float(np.linalg.norm(final_bounds.mean(axis=0)))
        ground_error = float(abs(final_bounds[0, 2] + measured.height / 2.0))
        object_T_raw = (
            ground_transform
            @ scale_transform
            @ center_transform
            @ axis_transform
            @ orientation
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)
        scene.export(output_path)
        if not output_path.is_file() or output_path.stat().st_size == 0:
            raise RuntimeError(f"failed to export aligned mesh to {output_path}")
        return MeshAlignment(
            output_path=output_path,
            object_T_raw_mesh=object_T_raw,
            uniform_scale=scale,
            predicted_dimensions=predicted,
            aligned_dimensions=aligned,
            dimension_residual=aligned - measured.as_array,
            ground_offset=ground_offset,
            center_error=center_error,
            ground_error=ground_error,
        )


def _principal_axis_transform(scene: Any) -> NDArray[np.float64]:
    """Map the longest/middle/shortest mesh axes to canonical X/Y/Z.

    A single view cannot reliably determine a car's front/back sign. The
    authoritative track pose supplies that semantic heading; this transform
    only establishes a deterministic right-handed length/width/height basis.
    """

    combined = scene.to_geometry()
    vertices = np.asarray(combined.vertices, dtype=np.float64)
    if vertices.ndim != 2 or vertices.shape[0] < 3 or vertices.shape[1] != 3:
        raise ValueError("mesh does not contain enough vertices to determine its axes")
    centered = vertices - vertices.mean(axis=0)
    eigenvalues, eigenvectors = np.linalg.eigh(centered.T @ centered)
    axes = eigenvectors[:, np.argsort(eigenvalues)[::-1]]
    # Make the basis deterministic while preserving a right-handed frame.
    for column in (0, 2):
        dominant = int(np.argmax(np.abs(axes[:, column])))
        if axes[dominant, column] < 0:
            axes[:, column] *= -1
    axes[:, 1] = np.cross(axes[:, 2], axes[:, 0])
    axes[:, 1] /= np.linalg.norm(axes[:, 1])
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = axes.T
    return transform

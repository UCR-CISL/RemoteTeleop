"""Estimate and apply a fixed NuScenes-global to Gaussian-map alignment.

Unlike the legacy replay path, this module does not assume that the Gaussian
map is anchored at the first front camera.  It registers one or more LiDAR
scans expressed in NuScenes global coordinates, then applies the resulting
fixed transform to the complete sensor trajectory.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import TYPE_CHECKING, Callable, Iterable

import numpy as np
from numpy.typing import NDArray

from src.localization.gaussian_map import GaussianMap, load_gaussian_map
from src.localization.registration import PointCloudRegistrar, RegistrationResult

if TYPE_CHECKING:
    from src.data.processed_nuscenes import ProcessedNuScenesSequence


@dataclass(frozen=True)
class NuScenesLocalizationFrame:
    """A LiDAR scan and calibrated sensor poses in NuScenes global coordinates."""

    frame_id: str
    lidar_points: NDArray[np.floating]
    global_T_lidar: NDArray[np.floating]
    global_T_camera: NDArray[np.floating]

    def __post_init__(self) -> None:
        points = np.asarray(self.lidar_points, dtype=np.float64)
        if (
            points.ndim != 2
            or points.shape[1] < 3
            or len(points) == 0
            or not np.isfinite(points[:, :3]).all()
        ):
            raise ValueError("lidar_points must have shape (N, >=3) with finite XYZ coordinates")
        object.__setattr__(self, "lidar_points", points)
        object.__setattr__(self, "global_T_lidar", _as_transform(self.global_T_lidar, "global_T_lidar"))
        object.__setattr__(self, "global_T_camera", _as_transform(self.global_T_camera, "global_T_camera"))


@dataclass(frozen=True)
class NuScenesLocalizationConfig:
    """Configuration for a fixed global-to-Gaussian-map registration."""

    splat_path: Path | None = None
    selected_frame_ids: tuple[str, ...] | None = None
    source_stride: int = 1
    map_stride: int = 1
    min_opacity: float = 0.0
    use_legacy_initial_transform: bool = False


@dataclass(frozen=True)
class NuScenesLocalizationResult:
    """Fixed alignment and the resulting Gaussian-map sensor trajectory."""

    gaussian_T_nuscenes_global: NDArray[np.float64]
    frame_ids: tuple[str, ...]
    gaussian_T_lidar: NDArray[np.float64]
    gaussian_T_camera: NDArray[np.float64]
    selected_frame_ids: tuple[str, ...]
    registration: RegistrationResult


class NuScenesGaussianLocalizer:
    """Localize NuScenes global poses in a static Gaussian-map point proxy.

    The registrar receives selected LiDAR points transformed into the NuScenes
    global frame.  Its returned ``map_T_source`` is consequently
    ``gaussian_T_nuscenes_global`` and is valid for every frame in the scene.
    """

    def __init__(
        self,
        config: NuScenesLocalizationConfig,
        *,
        registrar: PointCloudRegistrar,
        map_loader: Callable[[Path], GaussianMap] = load_gaussian_map,
    ) -> None:
        if config.source_stride < 1 or config.map_stride < 1:
            raise ValueError("source_stride and map_stride must be at least one")
        if not np.isfinite(config.min_opacity):
            raise ValueError("min_opacity must be finite")
        if config.selected_frame_ids is not None and not config.selected_frame_ids:
            raise ValueError("selected_frame_ids must not be empty")
        self.config = config
        self._registrar = registrar
        self._map_loader = map_loader

    def run(self, frames: Iterable[NuScenesLocalizationFrame]) -> NuScenesLocalizationResult:
        """Load the configured Gaussian map and localize ``frames`` against it."""

        if self.config.splat_path is None:
            raise ValueError("splat_path is required when using run()")
        return self.localize(frames, self._map_loader(self.config.splat_path))

    def localize_processed_sequence(
        self, sequence: ProcessedNuScenesSequence
    ) -> NuScenesLocalizationResult:
        """Localize a processed scene using its original NuScenes-global poses.

        Processed scenes retain their legacy map-space poses.  Undoing the
        sequence's fixed ``map_T_global`` recovers the source global poses;
        this is solely an input bridge and does not preserve that legacy map
        anchor as the localization result.
        """

        global_T_map = np.linalg.inv(
            _as_transform(sequence.map_T_global, "sequence.map_T_global")
        )
        frames = (
            NuScenesLocalizationFrame(
                frame_id=frame.frame_id,
                lidar_points=frame.lidar_points,
                global_T_lidar=global_T_map @ frame.map_T_lidar,
                global_T_camera=global_T_map @ frame.map_T_camera,
            )
            for frame in sequence
        )
        if self.config.splat_path is None:
            raise ValueError("splat_path is required when localizing a processed sequence")
        gaussian_map = self._map_loader(self.config.splat_path)
        initial_transform = (
            sequence.map_T_global
            if self.config.use_legacy_initial_transform else None
        )
        return self.localize(
            frames, gaussian_map, initial_transform=initial_transform
        )

    def localize(
        self,
        frames: Iterable[NuScenesLocalizationFrame],
        gaussian_map: GaussianMap,
        *,
        initial_transform: NDArray[np.floating] | None = None,
    ) -> NuScenesLocalizationResult:
        """Estimate one alignment and propagate it to LiDAR and camera poses."""

        sequence = tuple(frames)
        if not sequence:
            raise ValueError("at least one NuScenes frame is required")
        frame_by_id = {frame.frame_id: frame for frame in sequence}
        if len(frame_by_id) != len(sequence):
            raise ValueError("NuScenes frame IDs must be unique")
        selected_ids = self.config.selected_frame_ids or (sequence[0].frame_id,)
        if len(set(selected_ids)) != len(selected_ids):
            raise ValueError("selected NuScenes frame IDs must be unique")
        missing = tuple(frame_id for frame_id in selected_ids if frame_id not in frame_by_id)
        if missing:
            raise ValueError(f"selected NuScenes frame IDs are unavailable: {', '.join(missing)}")

        source = np.concatenate(
            [
                _transform_points(
                    frame_by_id[frame_id].lidar_points[:: self.config.source_stride, :3],
                    frame_by_id[frame_id].global_T_lidar,
                )
                for frame_id in selected_ids
            ]
        )
        target = self._map_proxy(
            gaussian_map,
            min_opacity=self.config.min_opacity,
            stride=self.config.map_stride,
        )
        registration = (
            self._registrar.register(source, target)
            if initial_transform is None
            else self._registrar.register(
                source,
                target,
                _as_transform(initial_transform, "initial_transform"),
            )
        )
        if not registration.accepted:
            raise RuntimeError(
                "NuScenes-global registration rejected: " + ", ".join(registration.rejection_reasons)
            )
        gaussian_T_global = _as_transform(registration.map_T_source, "registration.map_T_source")
        return NuScenesLocalizationResult(
            gaussian_T_nuscenes_global=gaussian_T_global,
            frame_ids=tuple(frame.frame_id for frame in sequence),
            gaussian_T_lidar=np.stack([gaussian_T_global @ frame.global_T_lidar for frame in sequence]),
            gaussian_T_camera=np.stack([gaussian_T_global @ frame.global_T_camera for frame in sequence]),
            selected_frame_ids=selected_ids,
            registration=registration,
        )

    @staticmethod
    def _map_proxy(gaussian_map: GaussianMap, *, min_opacity: float, stride: int) -> NDArray[np.float64]:
        means = np.asarray(gaussian_map.means, dtype=np.float64)
        opacities = np.asarray(gaussian_map.opacities, dtype=np.float64)
        keep = np.isfinite(means).all(axis=1) & np.isfinite(opacities) & (opacities > min_opacity)
        keep &= np.abs(means).max(axis=1) <= 10_000.0
        proxy = means[keep][::stride]
        if len(proxy) == 0:
            raise ValueError("Gaussian map has no finite, sufficiently opaque centers")
        return proxy


def _transform_points(points: NDArray[np.floating], target_T_source: NDArray[np.floating]) -> NDArray[np.float64]:
    transform = _as_transform(target_T_source, "target_T_source")
    xyz = np.asarray(points, dtype=np.float64)
    return xyz @ transform[:3, :3].T + transform[:3, 3]


def _as_transform(value: NDArray[np.floating], label: str) -> NDArray[np.float64]:
    transform = np.asarray(value, dtype=np.float64)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError(f"{label} must be a finite 4x4 transform")
    if not np.allclose(transform[3], (0.0, 0.0, 0.0, 1.0), atol=1e-7):
        raise ValueError(f"{label} must be homogeneous")
    return transform


def write_nuscenes_localization_artifacts(
    result: NuScenesLocalizationResult,
    output_dir: str | Path,
    *,
    scene_id: str | None = None,
) -> dict:
    """Write the fixed alignment plus a per-frame Gaussian-map trajectory."""

    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    np.savez(
        output / "localized_trajectory.npz",
        gaussian_T_nuscenes_global=result.gaussian_T_nuscenes_global,
        scene_id="" if scene_id is None else scene_id,
        frame_ids=np.asarray(result.frame_ids),
        gaussian_T_lidar=result.gaussian_T_lidar,
        gaussian_T_camera=result.gaussian_T_camera,
    )
    registration = asdict(result.registration)
    registration["map_T_source"] = result.registration.map_T_source.tolist()
    payload = {
        "gaussian_T_nuscenes_global": result.gaussian_T_nuscenes_global.tolist(),
        "selected_frame_ids": list(result.selected_frame_ids),
        "registration": registration,
        "trajectory_path": "localized_trajectory.npz",
    }
    if scene_id is not None:
        payload["scene_id"] = scene_id
    (output / "localization.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return payload


def main(argv: list[str] | None = None) -> None:
    """Create a per-scene NuScenes-global-to-Gaussian-map artifact."""

    parser = argparse.ArgumentParser(
        description="Localize processed NuScenes LiDAR in a Gaussian-splat map."
    )
    parser.add_argument("--scene-root", type=Path, required=True)
    parser.add_argument("--splat", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--expected-frame-count", type=int, default=191)
    parser.add_argument("--frame-id", action="append", dest="selected_frame_ids")
    parser.add_argument("--source-stride", type=int, default=1)
    parser.add_argument("--map-stride", type=int, default=1)
    parser.add_argument("--min-opacity", type=float, default=0.0)
    parser.add_argument(
        "--use-legacy-initial-transform",
        action="store_true",
        help=(
            "Initialize ICP from the first-camera anchor, while retaining "
            "registration acceptance as the localization authority."
        ),
    )
    args = parser.parse_args(argv)

    from src.data.processed_nuscenes import ProcessedNuScenesSequence

    sequence = ProcessedNuScenesSequence(args.scene_root, expected_frame_count=args.expected_frame_count)
    config = NuScenesLocalizationConfig(
        splat_path=args.splat,
        selected_frame_ids=tuple(args.selected_frame_ids) if args.selected_frame_ids else None,
        source_stride=args.source_stride,
        map_stride=args.map_stride,
        min_opacity=args.min_opacity,
        use_legacy_initial_transform=args.use_legacy_initial_transform,
    )
    result = NuScenesGaussianLocalizer(
        config, registrar=PointCloudRegistrar()
    ).localize_processed_sequence(sequence)
    print(json.dumps(write_nuscenes_localization_artifacts(
        result, args.output_dir, scene_id=args.scene_root.name
    ), indent=2))


if __name__ == "__main__":  # pragma: no cover
    main()

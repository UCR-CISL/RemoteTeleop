"""Offline CooperScene-to-Gaussian-map localization orchestration."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Callable

import numpy as np

from src.localization.cooperscene import CooperSceneSequenceDataset
from src.localization.cooperscene_calibration import (
    camera_T_lidar,
    camera_intrinsic,
    compose_map_T_camera,
)
from src.localization.gaussian_map import GaussianMap, load_spz_v3
from src.localization.registration import PointCloudRegistrar, RegistrationConfig, RegistrationResult
from src.viz.localization_visualizer import plot_localization_trajectory, render_gaussian_camera_view


@dataclass(frozen=True)
class CooperSceneLocalizationConfig:
    data_root: Path
    splat_path: Path
    output_dir: Path
    split: str = "test"
    scenario: str = "1"
    agent: str = "1"
    frame_id: str | None = None
    source_stride: int = 8
    map_stride: int = 8
    refinement_map_stride: int = 1
    initial_transform_path: Path | None = None
    coarse_agents: str = "all"
    skip_coarse: bool = False
    render: bool = False
    render_device: str | None = None
    render_downsample: int = 2


class CooperSceneGaussianLocalizer:
    """Align one synchronized cooperative cloud and propagate that map pose."""

    def __init__(
        self,
        config: CooperSceneLocalizationConfig,
        *,
        dataset_factory: Callable[[Path], CooperSceneSequenceDataset] = CooperSceneSequenceDataset,
        map_loader: Callable[[Path], GaussianMap] = load_spz_v3,
        registrar: PointCloudRegistrar | None = None,
        refinement_registrar: PointCloudRegistrar | None = None,
        identity_prior_refinement_registrar: PointCloudRegistrar | None = None,
        trajectory_plotter: Callable[[np.ndarray, Path], Path] = plot_localization_trajectory,
        camera_renderer: Callable[..., Path] = render_gaussian_camera_view,
    ) -> None:
        if min(config.source_stride, config.map_stride, config.refinement_map_stride, config.render_downsample) < 1:
            raise ValueError("strides and render_downsample must be at least one")
        if config.render_device not in (None, "cpu", "cuda"):
            raise ValueError("render_device must be None, 'cpu', or 'cuda'")
        if config.coarse_agents not in ("all", "selected"):
            raise ValueError("coarse_agents must be 'all' or 'selected'")
        self.config = config
        self._dataset_factory = dataset_factory
        self._map_loader = map_loader
        # Initialization is a global problem: allow the map origin to differ
        # arbitrarily from CooperScene and use a coarse-to-fine metric schedule.
        self._registrar = registrar or PointCloudRegistrar(
            RegistrationConfig(
                coarse_voxel_size=1.0,
                refinement_voxel_sizes=(0.5, 0.25),
                min_inlier_ratio=0.05,
                max_median_residual=1.0,
                max_p95_residual=3.0,
                max_correction_translation=float("inf"),
                max_correction_rotation_degrees=180.0,
            )
        )
        self._refinement_registrar = refinement_registrar or PointCloudRegistrar(
            RegistrationConfig(
                coarse_voxel_size=0.25,
                refinement_voxel_sizes=(0.25, 0.10),
                icp_distance_factor=2.0,
                min_inlier_ratio=0.35,
                max_median_residual=0.20,
                max_p95_residual=0.60,
                max_correction_translation=1.0,
                max_correction_rotation_degrees=3.0,
            )
        )
        # Without a cooperative/global stage, identity is only a site prior.
        # Permit the first refinement to move farther while retaining the same
        # correspondence-quality gates.
        self._identity_prior_refinement_registrar = identity_prior_refinement_registrar or PointCloudRegistrar(
            RegistrationConfig(
                coarse_voxel_size=0.25,
                refinement_voxel_sizes=(0.25, 0.10),
                icp_distance_factor=2.0,
                min_inlier_ratio=0.35,
                max_median_residual=0.20,
                max_p95_residual=0.60,
                max_correction_translation=5.0,
                max_correction_rotation_degrees=10.0,
            )
        )
        self._trajectory_plotter = trajectory_plotter
        self._camera_renderer = camera_renderer

    def run(self) -> dict:
        dataset = self._dataset_factory(self.config.data_root)
        vehicle_sequence = dataset.sequence(self.config.split, self.config.scenario, self.config.agent)
        coarse_registration = None
        initial_transform = self._load_initial_transform()
        splats = self._map_loader(self.config.splat_path)
        if self.config.skip_coarse:
            selected_frame_id = self._selected_frame_id(vehicle_sequence)
        elif self.config.coarse_agents == "selected":
            selected_frame_id = self._selected_frame_id(vehicle_sequence)
            coarse_source = self._world_points(
                next(frame for frame in vehicle_sequence if frame.frame_id == selected_frame_id),
                stride=self.config.source_stride,
            )
            coarse_proxy = self._map_proxy(splats, min_opacity=0.05, stride=self.config.map_stride)
            coarse_registration = self._registrar.register(coarse_source, coarse_proxy, initial_transform)
        else:
            sequences = {
                agent: vehicle_sequence if agent == self.config.agent
                else dataset.sequence(self.config.split, self.config.scenario, agent)
                for agent in ("0", "1", "2", "3")
            }
            selected_frame_id = self._synchronized_frame_id(sequences)
            coarse_source = self._cooperative_world_cloud(sequences, selected_frame_id)
            splats = self._map_loader(self.config.splat_path)
            coarse_proxy = self._map_proxy(splats, min_opacity=0.05, stride=self.config.map_stride)
            coarse_registration = self._registrar.register(coarse_source, coarse_proxy, initial_transform)

        target_frame = next(frame for frame in vehicle_sequence if frame.frame_id == selected_frame_id)
        refinement_source = self._world_points(target_frame, stride=1)
        refinement_proxy = self._map_proxy(splats, min_opacity=0.0, stride=self.config.refinement_map_stride)
        if coarse_registration is not None:
            refinement_initial = coarse_registration.map_T_source
            refinement_registrar = self._refinement_registrar
        else:
            refinement_initial = initial_transform if initial_transform is not None else np.eye(4, dtype=np.float64)
            refinement_registrar = (
                self._refinement_registrar if initial_transform is not None else self._identity_prior_refinement_registrar
            )
        final_registration = refinement_registrar.register(refinement_source, refinement_proxy, refinement_initial)
        if not final_registration.accepted:
            raise RuntimeError(f"target-agent refinement rejected: {', '.join(final_registration.rejection_reasons)}")

        cooper_to_map = np.asarray(final_registration.map_T_source, dtype=np.float64)
        trajectory = np.stack([cooper_to_map @ frame.map_T_lidar for frame in vehicle_sequence])
        frame_ids = np.asarray([frame.frame_id for frame in vehicle_sequence])
        selected_index = next(index for index, frame in enumerate(vehicle_sequence) if frame.frame_id == selected_frame_id)
        self.config.output_dir.mkdir(parents=True, exist_ok=True)
        np.savez(self.config.output_dir / "localized_trajectory.npz", frame_ids=frame_ids, map_T_lidar=trajectory)
        self._trajectory_plotter(trajectory, self.config.output_dir / "trajectory.png")

        selected = vehicle_sequence[selected_index]
        map_T_camera = compose_map_T_camera(trajectory[selected_index], camera_T_lidar(self.config.agent))
        result = {
            "config": {key: str(value) if isinstance(value, Path) else value for key, value in asdict(self.config).items()},
            "synchronized_frame_id": selected_frame_id,
            "coarse_mode": "skipped" if self.config.skip_coarse else self.config.coarse_agents,
            "gaussian_T_cooperscene": cooper_to_map.tolist(),
            "selected_map_T_lidar": trajectory[selected_index].tolist(),
            "selected_map_T_camera": map_T_camera.tolist(),
            "coarse_registration": _registration_json(coarse_registration) if coarse_registration is not None else None,
            "final_registration": _registration_json(final_registration),
            "trajectory_path": "localized_trajectory.npz",
            "trajectory_plot": "trajectory.png",
        }
        if self.config.render:
            rendered = self._camera_renderer(
                splats,
                map_T_camera,
                camera_intrinsic(self.config.agent),
                (1920, 1200),
                self.config.output_dir / "camera_render.png",
                downsample=self.config.render_downsample,
                device=self.config.render_device,
            )
            result["camera_render"] = "camera_render.png"
            if hasattr(rendered, "backend"):
                result["camera_render_backend"] = rendered.backend
        (self.config.output_dir / "localization.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        return result

    def _selected_frame_id(self, sequence: object) -> str:
        if self.config.frame_id is None:
            try:
                return sequence[0].frame_id
            except IndexError as error:
                raise ValueError(f"agent {self.config.agent} has no CooperScene frames") from error
        frame_ids = {frame.frame_id for frame in sequence}
        if self.config.frame_id not in frame_ids:
            raise ValueError(f"frame {self.config.frame_id} is not available for agent {self.config.agent}")
        return self.config.frame_id

    def _synchronized_frame_id(self, sequences: dict[str, object]) -> str:
        available = [{frame.frame_id for frame in sequence} for sequence in sequences.values()]
        synchronized = set.intersection(*available)
        if self.config.frame_id is not None:
            if self.config.frame_id not in synchronized:
                raise ValueError(f"frame {self.config.frame_id} is not synchronized across agents 0-3")
            return self.config.frame_id
        if not synchronized:
            raise ValueError("no synchronized CooperScene frame exists across agents 0-3")
        return min(synchronized, key=lambda frame: (not frame.isdigit(), int(frame) if frame.isdigit() else frame))

    def _cooperative_world_cloud(self, sequences: dict[str, object], frame_id: str) -> np.ndarray:
        clouds = []
        for sequence in sequences.values():
            frame = next(frame for frame in sequence if frame.frame_id == frame_id)
            clouds.append(self._world_points(frame, stride=self.config.source_stride))
        return np.concatenate(clouds, axis=0)

    @staticmethod
    def _world_points(frame, *, stride: int) -> np.ndarray:
        lidar = np.asarray(frame.lidar_points[::stride, :3], dtype=np.float64)
        return lidar @ frame.map_T_lidar[:3, :3].T + frame.map_T_lidar[:3, 3]

    @staticmethod
    def _map_proxy(splats: GaussianMap, *, min_opacity: float, stride: int) -> np.ndarray:
        """Use stable, finite Gaussian centers as the deliberately simple proxy."""

        means = np.asarray(splats.means, dtype=np.float64)
        opacities = np.asarray(splats.opacities, dtype=np.float64)
        # Broad bounds only reject corrupt or numerically explosive centers;
        # they do not impose a site-specific alignment prior.
        keep = np.isfinite(means).all(axis=1) & np.isfinite(opacities) & (opacities > min_opacity)
        keep &= np.abs(means).max(axis=1) <= 10_000.0
        proxy = means[keep][::stride]
        if len(proxy) == 0:
            raise ValueError("Gaussian map has no finite, sufficiently opaque centers")
        return proxy

    def _load_initial_transform(self) -> np.ndarray | None:
        path = self.config.initial_transform_path
        if path is None:
            return None
        if path.suffix == ".npz":
            archive = np.load(path)
            for key in ("gaussian_T_cooperscene", "map_T_source", "transform"):
                if key in archive:
                    return _as_transform(archive[key], path)
            raise ValueError(f"no transform key found in {path}")
        if path.suffix == ".json":
            payload = json.loads(path.read_text(encoding="utf-8"))
            for key in ("gaussian_T_cooperscene", "map_T_source", "transform"):
                if key in payload:
                    return _as_transform(payload[key], path)
            raise ValueError(f"no transform key found in {path}")
        raise ValueError("initial_transform_path must end in .npz or .json")


def _registration_json(result: RegistrationResult) -> dict:
    value = asdict(result)
    value["map_T_source"] = np.asarray(result.map_T_source).tolist()
    return value


def _as_transform(values, path: Path) -> np.ndarray:
    transform = np.asarray(values, dtype=np.float64)
    if transform.shape != (4, 4) or not np.isfinite(transform).all() or not np.allclose(
        transform[3], [0.0, 0.0, 0.0, 1.0]
    ):
        raise ValueError(f"initial transform in {path} must be a finite 4x4 homogeneous matrix")
    return transform


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Localize CooperScene LiDAR in a Gaussian-splat map.")
    parser.add_argument("--data-root", type=Path, default=Path("data/mini"))
    parser.add_argument("--splat", type=Path, default=Path("data/riverside_r3.spz"))
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/cooperscene_localization"))
    parser.add_argument("--split", default="test")
    parser.add_argument("--scenario", default="1")
    parser.add_argument("--agent", default="1", choices=("1", "2", "3"))
    parser.add_argument("--frame-id")
    parser.add_argument("--source-stride", type=int, default=8)
    parser.add_argument("--map-stride", type=int, default=8)
    parser.add_argument("--refinement-map-stride", type=int, default=1)
    parser.add_argument("--initial-transform", type=Path)
    parser.add_argument(
        "--coarse-agents", choices=("all", "selected"), default="all",
        help="Use all synchronized agents (default), or only the selected agent, for coarse alignment.",
    )
    parser.add_argument("--skip-coarse", action="store_true", help="Skip coarse alignment and refine from a prior or identity.")
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--render-device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--render-downsample", type=int, default=2)
    args = parser.parse_args(argv)
    result = CooperSceneGaussianLocalizer(CooperSceneLocalizationConfig(
        data_root=args.data_root, splat_path=args.splat, output_dir=args.output_dir,
        split=args.split, scenario=args.scenario, agent=args.agent, frame_id=args.frame_id,
        source_stride=args.source_stride, map_stride=args.map_stride,
        refinement_map_stride=args.refinement_map_stride, initial_transform_path=args.initial_transform,
        coarse_agents=args.coarse_agents,
        skip_coarse=args.skip_coarse,
        render=args.render, render_device=None if args.render_device == "auto" else args.render_device,
        render_downsample=args.render_downsample,
    )).run()
    print(json.dumps(result, indent=2))


if __name__ == "__main__":  # pragma: no cover
    main()

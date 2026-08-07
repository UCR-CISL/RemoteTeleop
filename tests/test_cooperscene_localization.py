import json

import numpy as np

from src.localization.cooperscene import CooperSceneFrame
from src.localization.cooperscene_calibration import compose_map_T_camera
from src.localization.cooperscene_localization import (
    CooperSceneGaussianLocalizer,
    CooperSceneLocalizationConfig,
)
from src.localization.gaussian_map import GaussianMap
from src.localization.registration import RegistrationResult


class _Sequence:
    def __init__(self, frames):
        self.frames = frames

    def __iter__(self):
        return iter(self.frames)

    def __getitem__(self, index):
        return self.frames[index]


class _Dataset:
    def __init__(self, sequences):
        self.sequences = sequences

    def sequence(self, split, scenario, agent):
        return self.sequences[agent]


class _Registrar:
    def __init__(self, transform):
        self.transform = transform
        self.source = None
        self.target = None
        self.initial_map_T_source = None

    def register(self, source, target, initial_map_T_source=None):
        self.source, self.target = source, target
        self.initial_map_T_source = initial_map_T_source
        return RegistrationResult(
            map_T_source=self.transform,
            inlier_ratio=0.9,
            median_residual=0.1,
            p90_residual=0.2,
            p95_residual=0.3,
            all_point_median_residual=0.1,
            all_point_p90_residual=0.2,
            all_point_p95_residual=0.3,
            correction_translation=0.0,
            correction_rotation_degrees=0.0,
            runtime_seconds=0.01,
            accepted=True,
        )


def _frame(agent, frame_id, x):
    pose = np.eye(4)
    pose[0, 3] = x
    return CooperSceneFrame(
        split="test", scenario="1", agent=agent, frame_id=frame_id,
        lidar_path=None, annotation_path=None,
        lidar_points=np.array([[0, 0, 0, 1], [1, 0, 0, 1]], dtype=np.float32),
        map_T_lidar=pose,
    )


def test_orchestration_registers_synchronized_cooperative_cloud_and_saves_artifacts(tmp_path):
    sequences = {
        agent: _Sequence([_frame(agent, "10", float(index)), _frame(agent, "11", float(index) + 0.1)])
        for index, agent in enumerate(("0", "1", "2", "3"))
    }
    transform = np.array([[1, 0, 0, 10], [0, 1, 0, 20], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
    registrar = _Registrar(transform)
    refinement_registrar = _Registrar(transform)
    splats = GaussianMap(
        means=np.array([[1, 2, 3], [np.nan, 0, 0], [4, 5, 6]], dtype=np.float32),
        quats=np.zeros((3, 4), dtype=np.float32), scales=np.ones((3, 3), dtype=np.float32),
        opacities=np.array([1, 1, 0.01], dtype=np.float32), sh_coeffs=np.zeros((3, 1, 3), dtype=np.float32),
        sh_degree=0, antialiased=False,
    )
    plots, renders = [], []
    config = CooperSceneLocalizationConfig(
        tmp_path, tmp_path / "map.spz", tmp_path / "out", frame_id="10", render=True, source_stride=1
    )
    localizer = CooperSceneGaussianLocalizer(
        config,
        dataset_factory=lambda _: _Dataset(sequences), map_loader=lambda _: splats, registrar=registrar,
        refinement_registrar=refinement_registrar,
        trajectory_plotter=lambda poses, path: (plots.append((poses.copy(), path)), path)[1],
        camera_renderer=lambda *args, **kwargs: (renders.append((args, kwargs)), args[-1])[1],
    )

    result = localizer.run()

    assert registrar.source.shape == (8, 3)
    np.testing.assert_allclose(registrar.source[2], [1, 0, 0])
    np.testing.assert_allclose(registrar.target, [[1, 2, 3]])
    assert refinement_registrar.source.shape == (2, 3)
    np.testing.assert_allclose(refinement_registrar.target, [[1, 2, 3], [4, 5, 6]])
    assert result["synchronized_frame_id"] == "10"
    np.testing.assert_allclose(np.asarray(result["selected_map_T_lidar"])[:3, 3], [11, 20, 0])
    assert len(plots) == len(renders) == 1
    assert (config.output_dir / "localized_trajectory.npz").is_file()
    saved = np.load(config.output_dir / "localized_trajectory.npz")
    assert saved["map_T_lidar"].shape == (2, 4, 4)
    assert json.loads((config.output_dir / "localization.json").read_text())["camera_render"] == "camera_render.png"
    assert "coarse_registration" in result and "final_registration" in result


def test_camera_pose_composition_inverts_lidar_to_camera():
    map_T_lidar = np.eye(4)
    map_T_lidar[:3, 3] = [4, 5, 6]
    camera_T_lidar = np.eye(4)
    camera_T_lidar[0, 3] = 2

    map_T_camera = compose_map_T_camera(map_T_lidar, camera_T_lidar)

    np.testing.assert_allclose(map_T_camera[:3, 3], [2, 5, 6])


def test_skip_coarse_loads_only_the_selected_agent_and_refines_from_identity(tmp_path):
    vehicle_sequence = _Sequence([_frame("1", "10", 1.0), _frame("1", "11", 1.1)])

    class _SelectedAgentOnlyDataset:
        def sequence(self, split, scenario, agent):
            if agent != "1":
                raise AssertionError(f"skip-coarse accessed non-selected agent {agent}")
            return vehicle_sequence

    splats = GaussianMap(
        means=np.array([[1, 2, 3]], dtype=np.float32),
        quats=np.zeros((1, 4), dtype=np.float32), scales=np.ones((1, 3), dtype=np.float32),
        opacities=np.ones(1, dtype=np.float32), sh_coeffs=np.zeros((1, 1, 3), dtype=np.float32),
        sh_degree=0, antialiased=False,
    )
    unused_coarse = _Registrar(np.eye(4))
    normal_refinement = _Registrar(np.eye(4))
    identity_refinement = _Registrar(np.eye(4))
    config = CooperSceneLocalizationConfig(
        tmp_path, tmp_path / "map.spz", tmp_path / "out", skip_coarse=True, source_stride=1
    )
    result = CooperSceneGaussianLocalizer(
        config,
        dataset_factory=lambda _: _SelectedAgentOnlyDataset(),
        map_loader=lambda _: splats,
        registrar=unused_coarse,
        refinement_registrar=normal_refinement,
        identity_prior_refinement_registrar=identity_refinement,
        trajectory_plotter=lambda poses, path: path,
    ).run()

    assert result["synchronized_frame_id"] == "10"
    assert result["coarse_registration"] is None
    assert unused_coarse.source is None
    assert normal_refinement.source is None
    np.testing.assert_allclose(identity_refinement.initial_map_T_source, np.eye(4))
    assert (config.output_dir / "localized_trajectory.npz").is_file()


def test_selected_agent_coarse_registers_without_accessing_other_agents(tmp_path):
    vehicle_sequence = _Sequence([_frame("1", "10", 1.0), _frame("1", "11", 1.1)])

    class _SelectedAgentOnlyDataset:
        def sequence(self, split, scenario, agent):
            if agent != "1":
                raise AssertionError(f"selected coarse accessed non-selected agent {agent}")
            return vehicle_sequence

    splats = GaussianMap(
        means=np.array([[1, 2, 3]], dtype=np.float32),
        quats=np.zeros((1, 4), dtype=np.float32), scales=np.ones((1, 3), dtype=np.float32),
        opacities=np.ones(1, dtype=np.float32), sh_coeffs=np.zeros((1, 1, 3), dtype=np.float32),
        sh_degree=0, antialiased=False,
    )
    coarse_transform = np.eye(4)
    coarse_transform[0, 3] = 5.0
    coarse = _Registrar(coarse_transform)
    refinement = _Registrar(coarse_transform)
    config = CooperSceneLocalizationConfig(
        tmp_path, tmp_path / "map.spz", tmp_path / "out", coarse_agents="selected", source_stride=1
    )
    result = CooperSceneGaussianLocalizer(
        config,
        dataset_factory=lambda _: _SelectedAgentOnlyDataset(),
        map_loader=lambda _: splats,
        registrar=coarse,
        refinement_registrar=refinement,
        trajectory_plotter=lambda poses, path: path,
    ).run()

    assert result["synchronized_frame_id"] == "10"
    assert result["coarse_mode"] == "selected"
    assert result["coarse_registration"]["accepted"]
    np.testing.assert_allclose(coarse.source, [[1, 0, 0], [2, 0, 0]])
    np.testing.assert_allclose(refinement.initial_map_T_source, coarse_transform)

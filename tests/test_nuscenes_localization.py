import numpy as np
import pytest

from src.localization.gaussian_map import GaussianMap
from src.localization.nuscenes_localization import (
    NuScenesGaussianLocalizer,
    NuScenesLocalizationConfig,
    NuScenesLocalizationFrame,
    write_nuscenes_localization_artifacts,
)
from src.localization.registration import RegistrationResult


class _Registrar:
    def __init__(self, transform, *, accepted=True):
        self.transform = transform
        self.accepted = accepted
        self.source = None
        self.target = None
        self.initial_transform = None

    def register(self, source, target, initial_transform=None):
        self.source = source
        self.target = target
        self.initial_transform = initial_transform
        return RegistrationResult(
            map_T_source=self.transform, inlier_ratio=0.9, median_residual=0.1,
            p90_residual=0.2, p95_residual=0.3, all_point_median_residual=0.1,
            all_point_p90_residual=0.2, all_point_p95_residual=0.3,
            correction_translation=0.0, correction_rotation_degrees=0.0,
            runtime_seconds=0.01, accepted=self.accepted,
            rejection_reasons=() if self.accepted else ("residual",),
        )


def _transform(x=0, y=0, z=0):
    transform = np.eye(4)
    transform[:3, 3] = [x, y, z]
    return transform


def _frame(frame_id, *, lidar_x, camera_y):
    return NuScenesLocalizationFrame(
        frame_id=frame_id,
        lidar_points=np.array([[0, 0, 0, 1], [1, 0, 0, 1]], dtype=np.float32),
        global_T_lidar=_transform(x=lidar_x),
        global_T_camera=_transform(y=camera_y),
    )


def _map():
    return GaussianMap(
        means=np.array([[1, 2, 3], [np.nan, 0, 0], [4, 5, 6]], dtype=np.float32),
        quats=np.zeros((3, 4), dtype=np.float32), scales=np.ones((3, 3), dtype=np.float32),
        opacities=np.array([1, 1, 0.01], dtype=np.float32),
        sh_coeffs=np.zeros((3, 1, 3), dtype=np.float32), sh_degree=0, antialiased=False,
    )


def test_registers_global_lidar_points_once_and_propagates_all_sensor_poses():
    gaussian_T_global = _transform(x=10, z=3)
    registrar = _Registrar(gaussian_T_global)
    localizer = NuScenesGaussianLocalizer(
        NuScenesLocalizationConfig(selected_frame_ids=("001", "000"), min_opacity=0.05), registrar=registrar
    )

    result = localizer.localize([_frame("000", lidar_x=2, camera_y=4), _frame("001", lidar_x=7, camera_y=9)], _map())

    np.testing.assert_allclose(registrar.source, [[7, 0, 0], [8, 0, 0], [2, 0, 0], [3, 0, 0]])
    np.testing.assert_allclose(registrar.target, [[1, 2, 3]])
    np.testing.assert_allclose(result.gaussian_T_nuscenes_global, gaussian_T_global)
    np.testing.assert_allclose(result.gaussian_T_lidar[:, :3, 3], [[12, 0, 3], [17, 0, 3]])
    np.testing.assert_allclose(result.gaussian_T_camera[:, :3, 3], [[10, 4, 3], [10, 9, 3]])
    assert result.selected_frame_ids == ("001", "000")


def test_rejected_registration_is_not_propagated():
    localizer = NuScenesGaussianLocalizer(NuScenesLocalizationConfig(), registrar=_Registrar(np.eye(4), accepted=False))

    with pytest.raises(RuntimeError, match="rejected: residual"):
        localizer.localize([_frame("000", lidar_x=0, camera_y=0)], _map())


def test_selected_frame_must_be_present():
    localizer = NuScenesGaussianLocalizer(
        NuScenesLocalizationConfig(selected_frame_ids=("missing",)), registrar=_Registrar(np.eye(4))
    )

    with pytest.raises(ValueError, match="unavailable: missing"):
        localizer.localize([_frame("000", lidar_x=0, camera_y=0)], _map())


def test_processed_sequence_bridge_recovers_raw_global_sensor_poses_and_writes_artifacts(tmp_path):
    class _ProcessedFrame:
        frame_id = "000"
        lidar_points = np.array([[1, 0, 0, 0]], dtype=np.float32)
        map_T_lidar = _transform(x=8)
        map_T_camera = _transform(y=4)

    class _ProcessedSequence:
        map_T_global = _transform(x=-2)

        def __iter__(self):
            return iter([_ProcessedFrame()])

    registrar = _Registrar(_transform(x=10))
    localizer = NuScenesGaussianLocalizer(NuScenesLocalizationConfig(
                                              splat_path=tmp_path / "map.ply",
                                              use_legacy_initial_transform=True,
                                          ), registrar=registrar,
                                          map_loader=lambda _: _map())

    result = localizer.localize_processed_sequence(_ProcessedSequence())
    payload = write_nuscenes_localization_artifacts(
        result, tmp_path / "out", scene_id="152"
    )

    # inverse(map_T_global) @ map_T_lidar recovers global_T_lidar at x=10.
    np.testing.assert_allclose(registrar.source, [[11, 0, 0]])
    np.testing.assert_allclose(registrar.initial_transform, _transform(x=-2))
    np.testing.assert_allclose(result.gaussian_T_lidar[0, :3, 3], [20, 0, 0])
    np.testing.assert_allclose(result.gaussian_T_camera[0, :3, 3], [12, 4, 0])
    assert payload["gaussian_T_nuscenes_global"] == _transform(x=10).tolist()
    assert payload["scene_id"] == "152"
    assert (tmp_path / "out" / "localization.json").is_file()
    trajectory = np.load(tmp_path / "out" / "localized_trajectory.npz")
    np.testing.assert_allclose(
        trajectory["gaussian_T_nuscenes_global"], result.gaussian_T_nuscenes_global
    )
    assert str(trajectory["scene_id"]) == "152"
    assert trajectory["frame_ids"].tolist() == ["000"]

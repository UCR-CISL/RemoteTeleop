import numpy as np
import pytest

from src.common import CameraObservation, Pose3D, VehicleState


def test_pose_round_trip_composition_and_inverse():
    angle = np.pi / 2
    world_T_ego = Pose3D(
        translation=[4.0, -2.0, 1.0],
        quaternion_xyzw=[0.0, 0.0, np.sin(angle / 2), np.cos(angle / 2)],
    )
    ego_T_object = Pose3D(translation=[3.0, 0.0, 0.0], quaternion_xyzw=[0, 0, 0, 2])

    world_T_object = world_T_ego.compose(ego_T_object)

    np.testing.assert_allclose(world_T_object.translation, [4.0, 1.0, 1.0], atol=1e-8)
    np.testing.assert_allclose(
        world_T_ego.inverse().compose(world_T_object).matrix, ego_T_object.matrix, atol=1e-8
    )
    np.testing.assert_allclose(Pose3D.from_matrix(world_T_object.matrix).matrix, world_T_object.matrix)


def test_pose_rejects_non_rigid_and_zero_quaternion():
    with pytest.raises(ValueError, match="must not be zero"):
        Pose3D([0, 0, 0], [0, 0, 0, 0])
    scaled = np.diag([2.0, 1.0, 1.0, 1.0])
    with pytest.raises(ValueError, match="orthonormal"):
        Pose3D.from_matrix(scaled)


def test_vehicle_state_is_metric_lwh_and_ego_relative():
    state = VehicleState(
        track_id="car-1",
        timestamp_us=100,
        world_T_object=Pose3D([8, 1, 0], [0, 0, 0, 1]),
        dimensions_lwh=[4.5, 1.8, 1.4],
        source="nuscenes_gt",
    )

    np.testing.assert_allclose(state.dimensions, [4.5, 1.8, 1.4])
    np.testing.assert_allclose(
        state.ego_T_object(Pose3D([3, 1, 0], [0, 0, 0, 1])).translation, [5, 0, 0]
    )
    with pytest.raises(ValueError):
        state.dimensions[0] = 10


def test_camera_observation_copies_and_freezes_inputs():
    image = np.zeros((8, 12, 3), dtype=np.uint8)
    mask = np.zeros((8, 12), dtype=bool)
    observation = CameraObservation(
        image=image,
        mask=mask,
        camera_intrinsic=np.eye(3),
        world_T_camera=Pose3D.identity(),
        timestamp_us=42,
        crop_xyxy=(1, 2, 10, 7),
        metadata={"sample": "abc"},
    )
    image[0, 0] = 255

    np.testing.assert_array_equal(observation.image[0, 0], [0, 0, 0])
    assert observation.mask.dtype == np.bool_
    assert observation.metadata["sample"] == "abc"
    with pytest.raises(TypeError):
        observation.metadata["sample"] = "changed"

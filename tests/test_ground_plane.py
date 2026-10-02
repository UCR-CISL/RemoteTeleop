import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from src.localization.ground_plane import GroundPlane
from src.viz.composited_camera import _box_local_corners, _flat_ground_box_pose
from src.realtime.protocol import BoxPrompt


@pytest.mark.parametrize("rotation", [np.eye(3), Rotation.from_euler("xyz", [85, 4, 31], degrees=True).as_matrix()])
def test_plane_placement_contacts_bottom_and_preserves_tangent_location_heading(rotation):
    normal = rotation[:, 2]
    plane = GroundPlane(tuple(normal), -3)
    pose = np.eye(4)
    pose[:3, :3] = rotation @ Rotation.from_euler("xyz", [8, -5, 36], degrees=True).as_matrix()
    pose[:3, 3] = [7, 8, 9]
    original = pose.copy()
    placed = plane.place_box(pose, (4.6, 1.85, 1.5))
    corners = _box_local_corners((4.6, 1.85, 1.5)) @ placed[:3, :3].T + placed[:3, 3]
    heights = corners @ normal + plane.offset_m
    np.testing.assert_allclose(np.sort(heights), [0, 0, 0, 0, 1.5, 1.5, 1.5, 1.5], atol=1e-12)
    np.testing.assert_allclose(np.cross(placed[:3, 3] - pose[:3, 3], normal), 0, atol=1e-12)
    projected_forward = pose[:3, 0] - normal * (normal @ pose[:3, 0])
    np.testing.assert_allclose(placed[:3, 0], projected_forward / np.linalg.norm(projected_forward))
    np.testing.assert_allclose(placed[:3, :3].T @ placed[:3, :3], np.eye(3), atol=1e-12)
    assert np.linalg.det(placed[:3, :3]) == pytest.approx(1)
    np.testing.assert_array_equal(pose, original)


def test_robust_ground_fit_in_camera_map_axes_rejects_wall_outliers():
    rng = np.random.default_rng(5)
    rotation = Rotation.from_euler("xyz", [90, 5, 20], degrees=True).as_matrix()
    ground = np.column_stack((rng.uniform(-20, 20, (700, 2)), rng.normal(2, .015, 700)))
    outliers = rng.uniform(-20, 20, (300, 3))
    points = np.vstack((ground, outliers)) @ rotation.T
    plane, stats = GroundPlane.fit(points, up_hint=rotation[:, 2])
    assert np.dot(plane.normal, rotation[:, 2]) > .9999
    assert plane.offset_m == pytest.approx(-2, abs=.01)
    assert .69 < stats["inlier_ratio"] < .72
    assert stats["p95_residual_m"] < .04


def test_reject_unsupported_or_collinear_fit():
    points = np.column_stack((np.arange(200), np.zeros((200, 2))))
    with pytest.raises(ValueError):
        GroundPlane.fit(points, up_hint=[0, 0, 1])
    with pytest.raises(ValueError):
        GroundPlane.fit(np.zeros((2, 3)), up_hint=[0, 0, 1])


def test_plane_round_trip_and_compositor_dispatch():
    plane = GroundPlane.from_dict({"normal": [0, 0, 2], "offset_m": -6})
    assert plane.to_dict() == {"normal": [0., 0., 1.], "offset_m": -3.}
    box = BoxPrompt(track_id="car", xyxy=(0, 0, 10, 10), dimensions_lwh=(4, 2, 2), world_T_object=tuple(np.eye(4).ravel()))
    placed = _flat_ground_box_pose(box, ground_z=None, ground_plane=plane)
    assert placed[2, 3] == 4
    with pytest.raises(ValueError, match="ground_z"):
        _flat_ground_box_pose(box, ground_plane=plane)

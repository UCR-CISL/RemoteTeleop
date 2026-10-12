from unittest.mock import Mock

import numpy as np
import pytest

pytest.importorskip("trimesh")
from src.viz.vehicle_proxy import VehicleProxyRenderer
from src.viz.primitive_vehicle import PrimitiveVehicleAsset


def test_templates_cached_by_kind_but_instances_have_current_dimensions(tmp_path):
    renderer = Mock()
    renderer.load_geometry.side_effect = ["car-handle", "truck-handle"]
    renderer.with_vertices.return_value = "instance"
    proxy = VehicleProxyRenderer(renderer, asset_dir=tmp_path)
    identity = np.eye(4)
    camera = np.eye(4)
    camera[2, 3] = -10
    dimensions = [(4.6, 1.85, 1.5), (9., 1.85, 1.5), (4.6, 1.85, 2.7)]
    for size in dimensions:
        proxy.render("car", size, identity, camera, np.eye(3), (32, 24))
    renderer.load_geometry.assert_called_once()
    assert renderer.load_geometry.call_args.kwargs == {"preserve_topology": True}
    for call, size in zip(renderer.with_vertices.call_args_list, dimensions):
        assert call.args[0] == "car-handle"
        np.testing.assert_allclose(np.ptp(call.args[1], axis=0), size, atol=1e-6)
    assert (tmp_path / "car.glb").is_file()
    proxy.render("truck", (8., 2.5, 3.5), identity, identity, np.eye(3), (32, 24))
    assert renderer.load_geometry.call_count == 2
    assert (tmp_path / "truck.glb").is_file()
    assert renderer.render.call_args.args[0] == "instance"
    np.testing.assert_array_equal(renderer.render.call_args.args[1], identity)
    assert [call.kwargs for call in renderer.render.call_args_list] == [
        {"cull_backfaces": True}, {"cull_backfaces": True},
        {"cull_backfaces": True}, {"cull_backfaces": False},
    ]


def test_template_load_failure_propagates_without_caching():
    renderer = Mock()
    renderer.load_geometry.side_effect = RuntimeError("upload failed")
    proxy = VehicleProxyRenderer(renderer)
    with pytest.raises(RuntimeError, match="upload failed"):
        proxy.render("car", (4.6, 1.85, 1.5), np.eye(4), np.eye(4), np.eye(3), (32, 24))
    assert not proxy._templates


def test_enclosing_car_is_transparent_without_changing_poses():
    renderer = Mock()
    proxy = VehicleProxyRenderer(renderer)
    # The camera is inside the metric box but above the hood. Front-facing
    # hood surfaces would otherwise obscure the recorded lower road view.
    world_T_object = np.eye(4)
    world_T_camera = np.eye(4)
    world_T_camera[:3, 3] = (.005, -.205, .502)
    transform = np.array([[0., -1., 0., 10.], [1., 0., 0., -7.],
                          [0., 0., 1., 3.], [0., 0., 0., 1.]])
    object_pose, camera_pose = transform @ world_T_object, transform @ world_T_camera
    original_object, original_camera = object_pose.copy(), camera_pose.copy()
    result = proxy.render("car", (4.89, 2.11, 1.78), object_pose, camera_pose,
                          np.eye(3), (32, 24))
    assert result.alpha.shape == (24, 32)
    assert not result.alpha.any() and not result.rgb.any()
    assert np.isinf(result.depth).all()
    renderer.render.assert_not_called()
    renderer.with_vertices.assert_not_called()
    np.testing.assert_array_equal(object_pose, original_object)
    np.testing.assert_array_equal(camera_pose, original_camera)


def test_nearby_external_car_still_renders():
    renderer = Mock()
    proxy = VehicleProxyRenderer(renderer)
    camera = np.eye(4)
    camera[0, 3] = 2.5  # Outside the box, despite being only 2.5m away.
    proxy.render("car", (4.89, 2.11, 1.78), np.eye(4), camera, np.eye(3), (32, 24))
    renderer.render.assert_called_once()
    assert renderer.render.call_args.kwargs == {"cull_backfaces": True}


@pytest.mark.parametrize("kind", [kind for kind in PrimitiveVehicleAsset.SUPPORTED_KINDS if kind != "car"])
def test_other_primitive_kinds_preserve_two_sided_rendering(kind):
    renderer = Mock()
    proxy = VehicleProxyRenderer(renderer)
    proxy.render(kind, PrimitiveVehicleAsset.SPECS[kind][0], np.eye(4), np.eye(4), np.eye(3), (32, 24))
    assert renderer.render.call_args.kwargs == {"cull_backfaces": False}

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
    dimensions = [(4.6, 1.85, 1.5), (9., 1.85, 1.5), (4.6, 1.85, 2.7)]
    for size in dimensions:
        proxy.render("car", size, identity, identity, np.eye(3), (32, 24))
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


@pytest.mark.parametrize("kind", [kind for kind in PrimitiveVehicleAsset.SUPPORTED_KINDS if kind != "car"])
def test_other_primitive_kinds_preserve_two_sided_rendering(kind):
    renderer = Mock()
    proxy = VehicleProxyRenderer(renderer)
    proxy.render(kind, PrimitiveVehicleAsset.SPECS[kind][0], np.eye(4), np.eye(4), np.eye(3), (32, 24))
    assert renderer.render.call_args.kwargs == {"cull_backfaces": False}

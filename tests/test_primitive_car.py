"""Geometry contracts for procedural vehicle templates and bbox adaptation."""
from pathlib import Path

import numpy as np
import pytest

trimesh = pytest.importorskip("trimesh")

from src.viz.primitive_car import PrimitiveCarAsset
from src.viz.primitive_vehicle import PrimitiveVehicleAsset, SUPPORTED_KINDS


@pytest.mark.parametrize("kind", SUPPORTED_KINDS)
def test_nominal_template_geometry(kind):
    asset = PrimitiveVehicleAsset(kind)
    mesh = asset.mesh
    assert asset.mesh is mesh
    assert np.isfinite(mesh.vertices).all()
    np.testing.assert_allclose(mesh.bounds, [-asset.nominal_dimensions / 2, asset.nominal_dimensions / 2])
    assert 0 < len(mesh.faces) < 3000
    assert mesh.faces.min() >= 0
    assert mesh.faces.max() < len(mesh.vertices)
    assert (mesh.area_faces > 0).all()
    assert mesh.visual.kind == "vertex"
    assert mesh.visual.vertex_colors.shape == (len(mesh.vertices), 4)
    assert len(np.unique(mesh.visual.vertex_colors, axis=0)) >= 5
    assert np.all(mesh.visual.vertex_colors[:, 3] == 255)
    np.testing.assert_allclose(asset.vertices_for_dimensions(asset.nominal_dimensions), mesh.vertices)


@pytest.mark.parametrize("kind", SUPPORTED_KINDS)
def test_glb_roundtrip(tmp_path: Path, kind):
    asset = PrimitiveVehicleAsset(kind)
    dimensions = asset.nominal_dimensions * [1.5, 1.3, 1.7]
    expected = asset.mesh_for_dimensions(dimensions)
    path = tmp_path / "nested" / f"{kind}.glb"
    assert asset.export(path, dimensions) == path
    loaded = trimesh.load(path, force="mesh")
    np.testing.assert_allclose(loaded.bounds, expected.bounds, atol=1e-6)
    assert len(loaded.faces) == len(expected.faces)
    np.testing.assert_array_equal(loaded.visual.vertex_colors, expected.visual.vertex_colors)


@pytest.mark.parametrize("kind", SUPPORTED_KINDS)
@pytest.mark.parametrize("scale", [(2., 1., 1.), (1., 1., 1.8), (1., 1.6, 1.), (.02, .02, .02), (2., 1., .03)])
def test_resizing_keeps_topology_bounds_and_circular_wheels(kind, scale):
    asset = PrimitiveVehicleAsset(kind)
    dimensions = asset.nominal_dimensions * scale
    original = asset.mesh.vertices.copy()
    vertices = asset.vertices_for_dimensions(dimensions)
    assert vertices.shape == original.shape
    assert np.isfinite(vertices).all()
    assert np.all(np.abs(vertices) <= dimensions / 2 + 1e-10)
    np.testing.assert_allclose(vertices[:, 2].min(), -dimensions[2] / 2)
    np.testing.assert_allclose(vertices[:, 2].max(), dimensions[2] / 2)
    np.testing.assert_array_equal(asset.mesh.vertices, original)
    for wheel in asset._wheels:
        wheel_vertices = vertices[wheel.start:wheel.stop]
        nominal_vertices = original[wheel.start:wheel.stop]
        ratio = np.ptp(wheel_vertices, axis=0) / np.ptp(nominal_vertices, axis=0)
        # Uniform scaling of the entire assembly preserves tire circles and rim
        # proportions even under the safety cap for physically impossible boxes.
        np.testing.assert_allclose(ratio, np.repeat(ratio[0], 3), atol=1e-10)
        if min(scale) >= 1:
            np.testing.assert_allclose(ratio, [1., 1., 1.])
        np.testing.assert_allclose(wheel_vertices[:, 2].min(), -dimensions[2] / 2)
        center_x = (wheel_vertices[:, 0].max() + wheel_vertices[:, 0].min()) / 2
        center_z = (wheel_vertices[:, 2].max() + wheel_vertices[:, 2].min()) / 2
        radii = np.linalg.norm(wheel_vertices[:, [0, 2]] - [center_x, center_z], axis=1)
        assert radii.max() <= wheel.radius + 1e-10
        assert (radii > 0).any()


@pytest.mark.parametrize("dimensions", [[], [1, 2], [1, 2, 0], [-1, 2, 3], [np.nan, 1, 1], [1, np.inf, 1]])
def test_rejects_invalid_dimensions(dimensions):
    with pytest.raises(ValueError, match="finite positive"):
        PrimitiveCarAsset().vertices_for_dimensions(dimensions)


def test_compatibility_class_and_unknown_kind():
    assert PrimitiveCarAsset().kind == "car"
    with pytest.raises(ValueError, match="Unsupported"):
        PrimitiveVehicleAsset("unicorn")


@pytest.mark.parametrize("kind", ["bicycle", "motorcycle"])
@pytest.mark.parametrize("scale", [(2., 1., 1.), (1., 1., 1.8), (.4, 1., 1.)])
def test_cycle_forks_remain_attached_to_resized_wheel_axles(kind, scale):
    asset = PrimitiveVehicleAsset(kind)
    dimensions = asset.nominal_dimensions * scale
    vertices = asset.vertices_for_dimensions(dimensions)
    np.testing.assert_allclose(vertices[:, 0].min(), -dimensions[0] / 2)
    np.testing.assert_allclose(vertices[:, 0].max(), dimensions[0] / 2)
    for start, stop, fraction, _, weights in asset._axle_links:
        wheel = next(wheel for wheel in asset._wheels if wheel.longitudinal_fraction == fraction)
        center, _ = asset._wheel_placement(dimensions, wheel)
        # The cap nearest the axle retains its center at the wheel hub. Cylinder
        # circumference samples can straddle the cap plane, so use its center.
        endpoint = vertices[start:stop][np.argmax(weights)]
        assert np.linalg.norm(endpoint - center) < .06 * max(scale)


@pytest.mark.parametrize("kind", ["car", "police"])
@pytest.mark.parametrize("height", [.8, 1.0])
def test_low_car_hood_clears_fixed_wheels(kind, height):
    asset = PrimitiveVehicleAsset(kind)
    dimensions = asset.nominal_dimensions.copy()
    dimensions[2] = height
    vertices = asset.vertices_for_dimensions(dimensions)
    # Authored front-axle hood section at X=.31 and normalized height=.55.
    nominal = asset.mesh.vertices / asset.nominal_dimensions
    hood = np.isclose(nominal[:, 0], .31) & np.isclose(nominal[:, 2] + .5, .55)
    assert hood.any()
    wheel = asset._wheels[-1]
    wheel_top = vertices[wheel.start:wheel.stop, 2].max()
    assert vertices[hood, 2].min() > wheel_top
    np.testing.assert_allclose(np.ptp(vertices[wheel.start:wheel.stop, 2]), 2 * wheel.radius)
    np.testing.assert_allclose(vertices[:, 2].max(), height / 2)


@pytest.mark.parametrize("kind,dimensions", [
    ("truck", (4.8, 2., 2.5)),
    ("trailer", (4., 2., 2.5)),
    ("truck", (.3, .2, .2)),
    ("trailer", (.3, .2, .2)),
    ("car", (.3, .2, .2)),
    ("bicycle", (.3, .2, .2)),
])
def test_short_bboxes_separate_adjacent_axle_tires(kind, dimensions):
    asset = PrimitiveVehicleAsset(kind)
    dimensions = np.asarray(dimensions)
    vertices = asset.vertices_for_dimensions(dimensions)
    assert np.all(np.abs(vertices) <= dimensions / 2 + 1e-10)
    for side in sorted({wheel.side for wheel in asset._wheels}):
        wheels = sorted((wheel for wheel in asset._wheels if wheel.side == side),
                        key=lambda wheel: wheel.longitudinal_fraction)
        previous_right = None
        for wheel in wheels:
            tire_vertices = vertices[wheel.start:wheel.stop]
            left, right = tire_vertices[:, 0].min(), tire_vertices[:, 0].max()
            if previous_right is not None:
                assert left > previous_right
            previous_right = right
            extents = np.ptp(tire_vertices, axis=0)
            np.testing.assert_allclose(extents[0], extents[2])
            if dimensions[0] >= 4:
                # A short but feasible box moves axles without shrinking tires.
                np.testing.assert_allclose(extents[0], 2 * wheel.radius)


@pytest.mark.parametrize("roof", [1., .935])
def test_cabin_roof_and_pillars_share_one_closed_shell(monkeypatch, roof):
    asset = PrimitiveCarAsset()
    captured = []
    monkeypatch.setattr(asset, "_append", lambda mesh, *args, **kwargs: captured.append((mesh, kwargs)))
    asset._car_cabin(roof)
    assert len(captured) == 1
    mesh, options = captured[0]
    assert mesh.is_watertight
    assert mesh.body_count == 1
    assert options["smooth"] is True
    # Glass/body boundaries partition the same surface rather than overlaying
    # patches or leaving roof-to-pillar gaps.
    colors = np.asarray(options["face_colors"])
    assert colors.shape == (len(mesh.faces), 4)
    assert set(map(tuple, colors)) == {asset.BODY, asset.GLASS}
    assert len(np.unique(np.sort(mesh.faces, axis=1), axis=0)) == len(mesh.faces)

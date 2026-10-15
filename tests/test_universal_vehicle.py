"""Universal vehicle shape vectors and the fixed-topology mesh they instance."""
import subprocess
import sys

import numpy as np
import pytest

from src.realtime.protocol import BoxPrompt
from src.viz.universal_vehicle import (BOUNDS, GLASS, PARAMETERS, SCHEMA_VERSION, TRIM, TYPE_NAMES, MATERIAL_COLORS,
                                       UniversalVehicleMesh, UniversalVehicleShape, prior_moments, sanitize)

MESH = UniversalVehicleMesh()
BOXES = [None, (5.1, 2.22, 1.9), (4.2, 1.8, 1.4), (6.4, 2.5, 2.4)]


def test_module_imports_numpy_only():
    code = ("import sys, src.viz.universal_vehicle; "
            "print([name for name in ('trimesh', 'torch', 'pytorch3d') if name in sys.modules])")
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True)
    assert result.stdout.strip() == "[]"


@pytest.mark.parametrize("body_type", TYPE_NAMES)
def test_vector_roundtrip_fits_the_box_prompt(body_type):
    shape = UniversalVehicleShape.mean(body_type, (5.1, 2.2, 1.9), inflation_lwh_m=(.3, .36, .32))
    vector = shape.to_vector()
    assert vector[:2] == (SCHEMA_VERSION, TYPE_NAMES.index(body_type)) and len(vector) == 2 + len(PARAMETERS)
    assert UniversalVehicleShape.from_vector(vector) == shape
    assert UniversalVehicleShape.from_vector(np.asarray(vector)) == shape
    prompt = BoxPrompt("t", (1, 2, 3, 4), (5.1, 2.2, 1.9), tuple(np.eye(4).reshape(-1)), vehicle_shape=vector)
    assert BoxPrompt.from_dict(prompt.to_dict()).vehicle_shape == vector


GOOD = UniversalVehicleShape.mean("sedan").to_vector()


@pytest.mark.parametrize("vector", [
    GOOD[:-1], (1.,) + GOOD[1:], (SCHEMA_VERSION, 7.) + GOOD[2:], (SCHEMA_VERSION, .5) + GOOD[2:],
    GOOD[:4] + (np.nan,) + GOOD[5:], GOOD[:2] + (BOUNDS[0, 1] + 1,) + GOOD[3:],
    # rear-window base ahead of the roof rear
    GOOD[:6] + (GOOD[7] + .05,) + GOOD[7:],
    (0., 1.) + (.3,) * 9,  # a legacy 11-float CarShapeParams vector
])
def test_bad_vectors_are_rejected(vector):
    with pytest.raises(ValueError):
        UniversalVehicleShape.from_vector(vector)


def test_sanitize_is_an_idempotent_projection():
    rng = np.random.default_rng(0)
    rows = rng.uniform(BOUNDS[:, 0] - .2, BOUNDS[:, 1] + .2, (200, len(PARAMETERS)))
    once = sanitize(rows)
    np.testing.assert_allclose(sanitize(once), once)
    for row in once:
        UniversalVehicleShape("sedan", tuple(row))  # validates


@pytest.mark.parametrize("body_type", TYPE_NAMES)
@pytest.mark.parametrize("box", BOXES)
def test_instances_keep_topology_stay_in_the_box_and_face_outward(body_type, box):
    shape = UniversalVehicleShape.mean(body_type, box, inflation_lwh_m=(.3, .36, .32))
    dimensions = np.asarray(shape.prior.archetype_lwh if box is None else box)
    vertices, colors = MESH.instance(shape, dimensions)
    assert vertices.shape == (MESH.vertex_count, 3) and colors.shape == (MESH.vertex_count, 4)
    assert colors.dtype == np.uint8 and np.isfinite(vertices).all()
    triangles = vertices[MESH.faces]
    cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    assert (np.linalg.norm(cross, axis=1) > 1e-8).all()  # no degenerate triangles
    # Wheel sidewalls stand up to 2 cm proud of the box side and light decals 4 mm off the end caps.
    assert np.all(np.abs(vertices) <= dimensions / 2 + [.005, .021, 1e-9])
    body = triangles[:len(MESH._body)]
    # The closed body has positive volume with the emitted winding (outward normals).
    volume = np.einsum("ij,ij->i", body[:, 0], np.cross(body[:, 1], body[:, 2])).sum() / 6
    assert volume > .3 * np.prod(dimensions)
    roof = vertices[:3 * len(MESH._body), 2].max() + dimensions[2] / 2
    if box is not None and box[2] >= 1.8:  # inflated detector boxes: the roof sits below the box top
        assert roof < dimensions[2] - .15


def test_pickup_bed_is_open_lined_and_faces_up():
    shape = UniversalVehicleShape.mean("pickup")
    vertices, colors = MESH.instance(shape, shape.prior.archetype_lwh)
    triangles = vertices[MESH.faces]
    body = len(MESH._body)
    liner = np.flatnonzero((colors[MESH.faces[:body, 0], :3] < 70).all(axis=1) & (MESH._half_edge[:body] == 9)
                           & (MESH._span[:body] == 3))
    assert len(liner)
    normals = np.cross(triangles[liner, 1] - triangles[liner, 0], triangles[liner, 2] - triangles[liner, 0])
    assert np.all(normals[:, 2] > 0)  # the floor is seen from above under backface culling
    floor = triangles[liner, :, 2].mean() + shape.prior.archetype_lwh[2] / 2
    rail = shape["top_rear_window"] * shape.prior.archetype_lwh[2]
    assert rail - floor > .35
    sedan = UniversalVehicleShape.mean("sedan")
    _, sedan_colors = MESH.instance(sedan, sedan.prior.archetype_lwh)
    sedan_trim = (sedan_colors[MESH.faces[:body, 0], :3] == np.rint(MATERIAL_COLORS[TRIM, :3] * .78)).all(axis=1)
    assert not ((MESH._span[:body] == 3) & sedan_trim).any()


@pytest.mark.parametrize("body_type", TYPE_NAMES)
def test_glass_regions_and_body_only_tint(body_type):
    shape = UniversalVehicleShape.mean(body_type)
    _, plain = MESH.instance(shape, shape.prior.archetype_lwh)
    _, red = MESH.instance(shape, shape.prior.archetype_lwh, (.8, .1, .1))
    changed = (plain != red).any(axis=1)
    assert changed.mean() > .3
    # Glass, lights and wheels keep their colors; only paint follows the tint.
    np.testing.assert_array_equal(plain[~changed], red[~changed])
    assert np.all(red[changed, 0] > 2 * red[changed, 1])
    faces = MESH.faces[:len(MESH._body), 0]
    glass = np.isclose(plain[faces, :3].astype(float) / MATERIAL_COLORS[GLASS, :3], plain[faces, :1] /
                       MATERIAL_COLORS[GLASS, 0], atol=.02).all(axis=1) & ~changed[faces]
    roles = MESH._span[:len(MESH._body)]
    assert glass[(roles == 11) | (roles == 12)].any()  # windshield
    rear_doors = glass[(roles == 8) & (MESH._half_edge[:len(MESH._body)] == 5)].any()
    assert rear_doors == shape.prior.rear_side_glass  # cargo vans have no rear side windows


def test_prior_moments_follow_the_box_estimate_not_the_box():
    mean, std = prior_moments("sedan", (5.2, 2.25, 1.95), inflation_lwh_m=(.3, .36, .32))
    assert .7 < mean[PARAMETERS.index("top_roof_front")] < .85
    assert .78 < mean[PARAMETERS.index("half_width_mid")] < .9
    assert mean[PARAMETERS.index("x_nose")] < .49 and (std > 0).all()

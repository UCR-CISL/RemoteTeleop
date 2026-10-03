"""Known surface geometry and unsupported-patch checks for road diagnostics."""
import numpy as np

from scripts.investigation.audit_local_road_separation import local_plane


def test_local_height_is_measured_at_requested_xy_on_sloped_road():
    x, y = np.meshgrid(np.linspace(-3, 3, 41), np.linspace(-3, 3, 41))
    points = np.c_[x.ravel(), y.ravel(), (2 + .02*x - .03*y).ravel()]
    result = local_plane(points, np.array([.4, -.6]))
    assert result is not None
    np.testing.assert_allclose(result['height_m'], 2.026, atol=1e-10)
    np.testing.assert_allclose(result['normal'], np.array([-.02, .03, 1]) / np.linalg.norm([-.02, .03, 1]), atol=1e-10)


def test_local_patch_rejects_one_sided_extrapolation():
    x, y = np.meshgrid(np.linspace(.2, 2, 20), np.linspace(-2, 2, 30))
    assert local_plane(np.c_[x.ravel(), y.ravel(), np.zeros(x.size)], np.zeros(2)) is None

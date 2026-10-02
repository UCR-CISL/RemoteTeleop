"""Reference changes must not rotate physical error axes or mutate saved poses."""
import numpy as np
from scipy.spatial.transform import Rotation

from scripts.investigation.rescore_recovered_reference import disagreement


def test_tilted_reference_keeps_enu_height_error():
    reference = np.eye(4)
    reference[:3, :3] = Rotation.from_euler('y', 20, degrees=True).as_matrix()
    estimate = reference.copy()
    estimate[:3, 3] += [0., 0., 1.]
    result = disagreement(estimate, reference, np.eye(4), np.eye(3))
    assert np.isclose(result['dz_m'], 1.)
    assert np.isclose(result['xy_m'], 0.)
    assert np.isclose(result['xyz_m'], 1.)


def test_error_is_measured_at_sensor_not_dataset_origin():
    reference = np.eye(4)
    estimate = np.eye(4)
    estimate[:3, :3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
    sensor = np.eye(4)
    sensor[:3, 3] = [10., 0., 0.]
    copies = [x.copy() for x in (estimate, reference, sensor)]
    result = disagreement(estimate, reference, sensor, np.eye(3))
    assert np.isclose(result['dx_m'], -10.)
    assert np.isclose(result['dy_m'], 10.)
    assert np.isclose(result['xyz_m'], np.sqrt(200.))
    assert np.isclose(result['yaw_deg'], 90.)
    for actual, previous in zip((estimate, reference, sensor), copies):
        np.testing.assert_array_equal(actual, previous)


def test_camera_anchored_map_uses_global_error_axes():
    axes = Rotation.from_euler('x', 90, degrees=True).as_matrix()
    reference = np.eye(4)
    reference[:3, :3] = axes
    estimate = reference.copy()
    estimate[:3, 3] = axes @ np.array([1., 2., 3.])
    result = disagreement(estimate, reference, np.eye(4), axes)
    np.testing.assert_allclose([result['dx_m'], result['dy_m'], result['dz_m']], [1.,2.,3.])

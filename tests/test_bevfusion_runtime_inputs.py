"""Exercise the actual registered input transform without importing CUDA models."""
import ast
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import cv2
import numpy as np
import pytest


def runtime_transform(monkeypatch):
    registered = {}
    def lidar_points(points, *, points_dim, attribute_dims):
        assert points.shape[1] == points_dim
        assert points.flags.writeable
        return points
    def register_module(**kwargs):
        def register(cls):
            registered['transform'] = cls
            return cls
        return register
    modules = {
        'mmcv.transforms': dict(BaseTransform=object),
        'mmdet3d.registry': dict(TRANSFORMS=SimpleNamespace(register_module=register_module)),
        'mmdet3d.structures.points': dict(LiDARPoints=lidar_points),
    }
    for name, values in modules.items():
        module = ModuleType(name)
        module.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, module)
    tree = ast.parse((Path(__file__).parents[1] / 'docker/bevfusion/server.py').read_text())
    node = next(x for x in tree.body if isinstance(x, ast.FunctionDef) and x.name == '_register_runtime_transform')
    scope = dict(np=np, cv2=cv2)
    exec(compile(ast.Module(body=[node], type_ignores=[]), 'server.py', 'exec'), scope)
    scope['_register_runtime_transform']()
    return registered['transform']()


def test_runtime_preserves_explicit_lag_and_appends_zero_only_for_single_scan(monkeypatch):
    transform = runtime_transform(monkeypatch)
    bgr = np.array([[[1, 2, 3]]], dtype=np.uint8)
    current = np.array([[1, 2, 3, 100]], dtype=np.float32)
    for points in (current, np.column_stack((current, [.2])).astype(np.float32)):
        original = points.copy()
        result = transform.transform(dict(bevfusion_image_bgr=bgr, bevfusion_points=points))
        np.testing.assert_array_equal(result['img'][0], [[[3, 2, 1]]])
        np.testing.assert_array_equal(result['points'][:, :4], current)
        np.testing.assert_array_equal(result['points'][:, 4], [0] if points.shape[1] == 4 else original[:, 4])
        np.testing.assert_array_equal(points, original)


def test_native_four_feature_model_retains_xyzi_and_rejects_sweep_input(monkeypatch):
    transform = type(runtime_transform(monkeypatch))(point_feature_count=4)
    points = np.array([[1, 2, 3, .5]], dtype=np.float32)
    points.flags.writeable = False
    result = transform.transform(dict(bevfusion_image_bgr=np.zeros((1,1,3),np.uint8), bevfusion_points=points))
    np.testing.assert_array_equal(result['points'], points)
    with pytest.raises(ValueError, match='four-feature'):
        transform.transform(dict(bevfusion_image_bgr=np.zeros((1,1,3),np.uint8), bevfusion_points=np.column_stack((points,[.1]))))


def test_native_server_normalization_matches_host_trial_without_mutating_source(monkeypatch):
    cls = type(runtime_transform(monkeypatch))
    transform = cls(point_feature_count=4, intensity_scale=1/65535)
    points = np.array([[1,2,3,1000],[4,5,6,65535]],dtype=np.float32)
    before = points.copy()
    expected = before.copy()
    expected[:,3] *= 1/65535
    result = transform.transform(dict(bevfusion_image_bgr=np.zeros((1,1,3),np.uint8),bevfusion_points=points))
    np.testing.assert_array_equal(result['points'],expected)
    np.testing.assert_array_equal(points,before)
    for scale in (0,-1,np.nan,np.inf):
        with pytest.raises(ValueError,match='scale'):
            cls(point_feature_count=4,intensity_scale=scale)


@pytest.mark.parametrize('points', [np.zeros((1, 6)), np.array([[1, 2, 3, 4, -.1]]), np.array([[1, 2, 3, 4, np.nan]])])
def test_runtime_rejects_invalid_feature_shape_or_lag(monkeypatch, points):
    with pytest.raises(ValueError):
        runtime_transform(monkeypatch).transform(dict(bevfusion_image_bgr=np.zeros((1, 1, 3), np.uint8), bevfusion_points=points))

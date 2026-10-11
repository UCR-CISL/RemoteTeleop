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
    def register_module(**kwargs):
        def register(cls):
            registered['transform'] = cls
            return cls
        return register
    modules = {
        'mmcv.transforms': dict(BaseTransform=object),
        'mmdet3d.registry': dict(TRANSFORMS=SimpleNamespace(register_module=register_module)),
        'mmdet3d.structures.points': dict(LiDARPoints=lambda points, **kwargs: points),
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


@pytest.mark.parametrize('points', [np.zeros((1, 6)), np.array([[1, 2, 3, 4, -.1]]), np.array([[1, 2, 3, 4, np.nan]])])
def test_runtime_rejects_invalid_feature_shape_or_lag(monkeypatch, points):
    with pytest.raises(ValueError):
        runtime_transform(monkeypatch).transform(dict(bevfusion_image_bgr=np.zeros((1, 1, 3), np.uint8), bevfusion_points=points))

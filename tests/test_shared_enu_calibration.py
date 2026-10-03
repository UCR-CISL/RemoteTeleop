"""Synthetic geometry checks for offline shared-frame registration."""
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scripts.investigation.calibrate_shared_enu_map import SharedMapCalibration, transform, metrics


def test_recovers_shared_full_se3_from_orthogonal_planes():
    rng=np.random.default_rng(45)
    points=[];normals=[]
    for axis in range(3):
        p=rng.uniform(-7,7,(1200,3));p[:,axis]=axis*3
        n=np.zeros_like(p);n[:,axis]=1
        points.append(p);normals.append(n)
    target=np.concatenate(points)
    true=np.eye(4);true[:3,:3]=Rotation.from_euler('xyz',[1.2,-1.5,.8],degrees=True).as_matrix();true[:3,3]=[.12,-.18,.22]
    instance=SharedMapCalibration.__new__(SharedMapCalibration)
    instance.target=target;instance.tree=cKDTree(target);instance.normals=np.concatenate(normals);instance.normal_valid=np.ones(len(target),bool)
    instance.groups={'1_0':transform(target,np.linalg.inv(true)), '1_1':transform(target,np.linalg.inv(true))}
    got,trace=instance.fit()
    np.testing.assert_allclose(got,true,atol=2e-5)
    W=np.eye(4);W[:3,3]=[1.2,-3.4,-.4]
    np.testing.assert_allclose(transform(transform(target,W),got),transform(target,got@W),atol=1e-12)
    assert trace[-1]['support_by_group']['1_0']['correspondence_count']>3000


def test_empty_support_is_explicit():
    assert metrics(np.empty((0,3)),cKDTree([[0,0,0]]))==dict(point_count=0,status='insufficient_support')

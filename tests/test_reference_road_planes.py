import numpy as np
from scipy.spatial.transform import Rotation
from scripts.investigation.compare_reference_road_planes import transform_plane, height_at, inside


def test_plane_transform_preserves_point_residuals():
    normal=np.array([.1,-.2,1.]);normal/=np.linalg.norm(normal)
    offset=2.
    pose=np.eye(4);pose[:3,:3]=Rotation.from_euler('xyz',[4,7,20],degrees=True).as_matrix();pose[:3,3]=[10,-20,3]
    n,d=transform_plane(normal,offset,pose)
    source=np.array([[2,3,4],[-1,6,2],[0,0,0.]])
    target=source@pose[:3,:3].T+pose[:3,3]
    np.testing.assert_allclose(target@n+d,source@normal+offset,atol=1e-12)


def test_heights_use_same_xy_not_plane_intercept():
    n=np.array([.1,0,1.]);n/=np.linalg.norm(n)
    d=-2/np.sqrt(1.01)
    assert np.isclose(height_at(n,d,np.array([10.,0.])),1.)
    assert np.isclose(height_at(n,d,np.array([0.,0.])),2.)


def test_support_does_not_allow_extrapolation_outside_hull():
    support=np.array([[0,0,0],[10,0,0],[0,10,0],[10,10,0]])
    assert inside(support,np.array([5,5]))
    assert not inside(support,np.array([20,5]))

import cv2
import numpy as np

from src.viz.cooperscene_tracking import TrackingOverlayRenderer


def _box(identifier, center):
    pose = np.eye(4)
    pose[:3, 3] = center
    return dict(track_id=identifier, world_T_object=pose.tolist(),
                dimensions_lwh=[4, 2, 2], label='car', score=.9)


def test_overlay_keeps_world_poses_and_labels_geometric_failures(tmp_path):
    renderer = TrackingOverlayRenderer(bev_size=300, extent_m=20)
    gt = [_box('gt-match', [0, 0, 12]), _box('gt-miss', [6, 4, 12])]
    predictions = [_box('p-switch', [.5, 0, 12]), _box('p-fp', [-6, -4, 12])]
    original = np.asarray(predictions[0]['world_T_object']).copy()
    image = renderer.render(
        np.zeros((240, 320, 3), dtype=np.uint8), world_T_camera=np.eye(4),
        intrinsic=np.array([[180, 0, 160], [0, 180, 120], [0, 0, 1]]),
        lidar_world=np.array([[0, 0, 0], [np.nan, 2, 0]]), predictions=predictions, gt=gt,
        assignment=dict(matches=[dict(gt_index=0, pred_index=0)],
                        missed_gt_indices=[1], false_positive_indices=[1],
                        id_switches=[dict(pred_id='p-switch')]), title='synthetic frame',
    )
    assert image.shape == (376, 700, 3)
    # Check box colors below the legend, in camera and BEV diagnostic regions.
    for color in (renderer.MISS_COLOR, renderer.FP_COLOR, renderer.SWITCH_COLOR):
        assert np.any(np.all(image[76:] == color, axis=2))
    np.testing.assert_array_equal(predictions[0]['world_T_object'], original)
    path = tmp_path / 'nested' / 'overlay.png'
    renderer.save_png(path, image)
    np.testing.assert_array_equal(cv2.imread(str(path)), image)


def test_camera_near_plane_clipping_and_empty_cloud():
    renderer = TrackingOverlayRenderer(bev_size=100)
    image = renderer.render(
        np.zeros((100, 100, 3), np.uint8), world_T_camera=np.eye(4),
        intrinsic=[[60, 0, 50], [0, 60, 50], [0, 0, 1]],
        lidar_world=[], predictions=[_box('cross-camera', [0, 0, .2])], gt=[],
        assignment=dict(false_positive_indices=[0]),
    )
    assert image.shape == (176, 200, 3)
    assert np.any(image[76:, :100])


def test_camera_stroke_scales_with_width_and_has_dark_halo():
    renderer = TrackingOverlayRenderer(bev_size=100)
    assert renderer._camera_stroke(np.zeros((1200, 1920, 3), np.uint8)) == (3, 5)
    assert renderer._camera_stroke(np.zeros((2400, 3840, 3), np.uint8))[0] == 6
    image = np.full((50, 50, 3), 255, np.uint8)
    renderer._line(image, (5, 25), (45, 25), renderer.GT_COLOR, 3, 7)
    assert np.all(image[25, 25] == renderer.GT_COLOR)
    assert np.all(image[21, 25] == renderer.OUTLINE_COLOR) and np.all(image[15, 25] == 255)

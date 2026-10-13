"""CooperScene camera calibration selection and detector-request isolation."""
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from src.localization.cooperscene_calibration import (
    BENCHMARK_CAMERA_CALIBRATION, CAMERA_CALIBRATION, DEFAULT_CALIBRATION,
    REFINED_CAMERA_CALIBRATION,
    camera_intrinsic, camera_metadata, camera_T_lidar, compose_map_T_camera,
)


def test_default_is_refined_and_older_calibrations_are_explicit():
    assert DEFAULT_CALIBRATION == "refined"
    for agent in ("1", "2", "3"):
        np.testing.assert_array_equal(camera_T_lidar(agent), REFINED_CAMERA_CALIBRATION[agent][1])
        np.testing.assert_array_equal(camera_intrinsic(int(agent)), BENCHMARK_CAMERA_CALIBRATION[agent][0])
        np.testing.assert_array_equal(camera_T_lidar(agent, "benchmark"), BENCHMARK_CAMERA_CALIBRATION[agent][1])
        np.testing.assert_array_equal(camera_T_lidar(agent, "baseline"), CAMERA_CALIBRATION[agent][1])
        np.testing.assert_array_equal(camera_intrinsic(agent, "baseline"), CAMERA_CALIBRATION[agent][0])
    with pytest.raises(ValueError, match="unknown CooperScene calibration"):
        camera_T_lidar("1", "carla")
    with pytest.raises(ValueError, match="no front camera"):
        camera_intrinsic("4")


@pytest.mark.parametrize("agent", ["1", "2", "3"])
def test_benchmark_is_a_small_rigid_correction_of_baseline(agent):
    benchmark, baseline = camera_T_lidar(agent, "benchmark"), camera_T_lidar(agent, "baseline")
    np.testing.assert_allclose(benchmark[:3, :3] @ benchmark[:3, :3].T, np.eye(3), atol=1e-6)
    angle = np.degrees(Rotation.from_matrix(benchmark[:3, :3] @ baseline[:3, :3].T).magnitude())
    assert 2.0 < angle < 5.0
    assert np.linalg.norm(benchmark[:3, 3] - baseline[:3, 3]) < 0.02
    # Only agent 3's intrinsic was refit by the benchmark.
    same_k = np.array_equal(camera_intrinsic(agent, "benchmark"), camera_intrinsic(agent, "baseline"))
    assert same_k == (agent != "3")


@pytest.mark.parametrize("agent", ["1", "2", "3"])
def test_refined_is_a_camera_rotation_of_benchmark_with_unchanged_k(agent):
    refined, benchmark = camera_T_lidar(agent), camera_T_lidar(agent, "benchmark")
    correction = refined @ np.linalg.inv(benchmark)
    np.testing.assert_allclose(correction[:3, :3] @ correction[:3, :3].T, np.eye(3), atol=1e-6)
    np.testing.assert_allclose(correction[:3, 3], 0, atol=1e-12)
    np.testing.assert_array_equal(camera_intrinsic(agent), camera_intrinsic(agent, "benchmark"))


def test_camera_metadata_rederives_camera_from_lidar_pose():
    pose = np.eye(4)
    pose[:3, 3] = (5, -2, 1)
    fields = camera_metadata(2, pose, "baseline")
    assert fields["camera_calibration"] == "baseline" and fields["camera_agent"] == "2"
    np.testing.assert_array_equal(
        np.reshape(fields["world_T_camera"], (4, 4)),
        compose_map_T_camera(pose, camera_T_lidar("2", "baseline")))
    np.testing.assert_array_equal(np.reshape(fields["camera_intrinsic"], (3, 3)),
                                  camera_intrinsic("2", "baseline"))


class _Client:
    def __init__(self):
        self.requests = []

    def submit(self, request):
        self.requests.append(request)
        return True

    def poll(self):
        return None


def _submit(metadata, **options):
    from src.deployment.ros_frame_adapter import LatestBevFusionBoxes

    client = _Client()
    latest = LatestBevFusionBoxes(client, score_threshold=.2, **options)
    header = SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0))
    latest.receive_image(SimpleNamespace(header=header, height=2, width=2, step=6,
                                         encoding="bgr8", data=bytes(12)))
    latest.receive_lidar(SimpleNamespace(header=header, points=np.zeros((1, 4), np.float32)))
    latest.receive_metadata(metadata, 0)
    (request,) = client.requests
    return request


def _metadata(pose, **camera):
    return dict(scene_generation="cooperscene:test", frame_id="f", timestamp_us=1_000_000,
                map_T_lidar=pose.reshape(-1).tolist(), world_T_ego=pose.reshape(-1).tolist(), **camera)


@pytest.mark.parametrize("request_calibration", ["baseline", "benchmark"])
def test_detector_request_calibration_is_independent_of_replay_camera(request_calibration):
    pose = np.eye(4)
    pose[:3, 3] = (10, 20, 1)
    options = {} if request_calibration == "baseline" else dict(request_calibration=request_calibration)
    request = _submit(_metadata(pose, **camera_metadata("3", pose, "benchmark")), **options)
    expected = camera_metadata("3", pose, request_calibration)
    np.testing.assert_array_equal(request.camera_intrinsic, expected["camera_intrinsic"])
    np.testing.assert_array_equal(request.world_T_camera, expected["world_T_camera"])


def test_untagged_or_disabled_detector_camera_is_sent_unchanged():
    pose = np.eye(4)
    camera = dict(camera_intrinsic=[2., 0, 1, 0, 2, 1, 0, 0, 1], world_T_camera=np.eye(4).reshape(-1).tolist())
    request = _submit(_metadata(pose, **camera))  # e.g. nuScenes: no calibration tag
    assert request.camera_intrinsic == tuple(camera["camera_intrinsic"])
    tagged = dict(camera_metadata("1", pose, "benchmark"))
    request = _submit(_metadata(pose, **tagged), request_calibration=None)
    np.testing.assert_array_equal(request.camera_intrinsic, tagged["camera_intrinsic"])
    np.testing.assert_array_equal(request.world_T_camera, tagged["world_T_camera"])

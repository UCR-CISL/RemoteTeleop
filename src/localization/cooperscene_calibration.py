"""Documented front-camera calibration for CooperScene vehicle agents."""

from __future__ import annotations

import numpy as np

CALIBRATIONS = ("refined", "benchmark", "baseline")
DEFAULT_CALIBRATION = "refined"

# "baseline": copied from CooperScene's dataset converter (SensorPlatform
# calibration). The CooperScene BEVFusion checkpoint was trained with it.
# The extrinsics are camera_T_lidar in CV camera coordinates (right, down,
# forward), not CARLA sensor coordinates.
CAMERA_CALIBRATION = {
    "1": ([[1810.6983377214349, 0.0, 970.0037900558976], [0.0, 1808.4746114412712, 526.7243033929532], [0.0, 0.0, 1.0]], [[1.97675856e-02, -9.98998871e-01, -4.01310140e-02, -1.14236290e-04], [-4.47204289e-02, 3.92152089e-02, -9.98229558e-01, -1.75248879e-01], [9.98803948e-01, 2.15272644e-02, -4.39004680e-02, -5.25987245e-02], [0.0, 0.0, 0.0, 1.0]]),
    "2": ([[1807.294188309111, 0.0, 959.1274184107824], [0.0, 1805.1344921703185, 562.0235332672737], [0.0, 0.0, 1.0]], [[0.00383388, -0.99998521, -0.00385735, -0.0013977], [-0.00271154, 0.00384697, -0.99998892, -0.23015363], [0.99998897, 0.0038443, -0.00269675, -0.07767752], [0.0, 0.0, 0.0, 1.0]]),
    "3": ([[1814.1641900346326, 0.0, 951.718698093622], [0.0, 1812.1352460466578, 568.1954332460546], [0.0, 0.0, 1.0]], [[0.064023, -0.99792386, -0.00700149, -0.00332843], [-0.01444825, 0.00608825, -0.99987708, -0.26612881], [0.99784383, 0.06411629, -0.01402846, -0.0864176], [0.0, 0.0, 0.0, 1.0]]),
}

# "benchmark": refined calibration from the CooperBEVFusion benchmark,
# /mnt/bcc-data/data/Cooperscene/benchmark_cooperbevfusion/250928/
# calibration_results/calibration_results_all.json ("T_lidar_to_cam" is
# camera_T_lidar in CV camera axes; calib_key t1/t2/t3). Values are identical
# for every scene of an agent, so one entry per agent is embedded. Relative to
# baseline it corrects a few degrees of camera rotation (agent3 also refits K).
BENCHMARK_CAMERA_CALIBRATION = {
    "1": ([[1810.6983377214349, 0.0, 970.0037900558976], [0.0, 1808.4746114412712, 526.7243033929532], [0.0, 0.0, 1.0]], [[0.01971678264201862, -0.9969902485818083, -0.07497794810686183, -0.006304513963065674], [-0.0017998508623980856, 0.07495701047360195, -0.9971851417621606, -0.177264365335627], [0.9998039856696714, 0.019796231826720766, -0.000316522712081109, -0.0449044134307572], [0.0, 0.0, 0.0, 1.0]]),
    "2": ([[1807.294188309111, 0.0, 959.1274184107824], [0.0, 1805.1344921703185, 562.0235332672737], [0.0, 0.0, 1.0]], [[0.0038338799999999996, -0.9999852099999998, -0.0038573499999999994, -0.0013976999999999998], [0.04962755504867806, 0.0040428929778995515, -0.9987596069293269, -0.2339035397367567], [0.9987604309095965, 0.0036376966668690905, 0.049642322162699266, -0.06552575534965442], [0.0, 0.0, 0.0, 1.0]]),
    "3": ([[1723.4559805329009, 0.0, 951.718698093622], [0.0, 1721.5284837443248, 568.1954332460546], [0.0, 0.0, 1.0]], [[0.012401962235324929, -0.9995735923853756, -0.026435212942085076, -0.004253486143733532], [0.03758394197950837, 0.02688455285877321, -0.998931760145861, -0.2702537648357407], [0.9992165151578138, 0.011395174948396682, 0.03790134232604989, -0.07244607646915167], [0.0, 0.0, 0.0, 1.0]]),
}

# "refined": artifacts/camera-rotation-fit-20261012/corrected_camera_T_lidar.json,
# camera_only variant. Dense image/render rotation fits at XYZ-localized poses
# were corrected for LiDAR-pose tilt using LiDAR-ground vs map-ground normals.
# Each camera_T_lidar is benchmark composed with its fitted camera rotation;
# intrinsics are unchanged from benchmark.
REFINED_CAMERA_CALIBRATION = {
    "1": (BENCHMARK_CAMERA_CALIBRATION["1"][0], [[0.02186118441284714, -0.9997576420895049, -0.0025972448085600707, 0.006451112262567737], [0.00896780463547343, 0.0027938541261423155, -0.9999558853768382, -0.17766739851545124], [0.9997207949412308, 0.02183692843892967, 0.00902670813696243, -0.043260687870641204], [0.0, 0.0, 0.0, 1.0]]),
    "2": (BENCHMARK_CAMERA_CALIBRATION["2"][0], [[-0.011968418892247815, -0.999715756427019, -0.020619441005494423, -0.004041364657616932], [0.06046706323416231, 0.019859588934745313, -0.9979726061680075, -0.23455413230455643], [0.9980984332317835, -0.013190948347617356, 0.060212189158060744, -0.06304340148959303], [0.0, 0.0, 0.0, 1.0]]),
    "3": (BENCHMARK_CAMERA_CALIBRATION["3"][0], [[0.016012179914323715, -0.9997455824079596, -0.01588635592908941, -0.0017309831544342284], [0.054318502804854965, 0.016734690183815813, -0.9983834150917203, -0.2714504667828488], [0.9983952703581692, 0.01512337227429984, 0.05457264723839468, -0.06793578827638293], [0.0, 0.0, 0.0, 1.0]]),
}


def _calibration(agent: str | int, calibration: str) -> tuple[list, list]:
    tables = {"refined": REFINED_CAMERA_CALIBRATION,
              "benchmark": BENCHMARK_CAMERA_CALIBRATION, "baseline": CAMERA_CALIBRATION}
    if calibration not in tables:
        raise ValueError(f"unknown CooperScene calibration {calibration!r}; expected one of {CALIBRATIONS}")
    try:
        return tables[calibration][str(agent)]
    except KeyError as error:
        raise ValueError(f"CooperScene agent {agent} has no front camera calibration") from error


def camera_intrinsic(agent: str | int, calibration: str = DEFAULT_CALIBRATION) -> np.ndarray:
    """Return the camera0 intrinsic matrix for vehicle ``agent``."""

    return np.asarray(_calibration(agent, calibration)[0], dtype=np.float64)


def camera_T_lidar(agent: str | int, calibration: str = DEFAULT_CALIBRATION) -> np.ndarray:
    """Return the CV-camera-from-LiDAR transform for ``agent``."""

    return np.asarray(_calibration(agent, calibration)[1], dtype=np.float64)


def camera_metadata(agent: str | int, map_T_lidar: np.ndarray, calibration: str) -> dict:
    """Return replay-metadata camera fields re-derived from the LiDAR pose."""

    return {
        "camera_intrinsic": camera_intrinsic(agent, calibration).reshape(-1).tolist(),
        "world_T_camera": compose_map_T_camera(
            map_T_lidar, camera_T_lidar(agent, calibration)
        ).reshape(-1).tolist(),
        "camera_calibration": calibration,
        "camera_agent": str(agent),
    }


def compose_map_T_camera(map_T_lidar: np.ndarray, lidar_to_camera: np.ndarray) -> np.ndarray:
    """Compose camera pose from ``map_T_lidar`` and ``camera_T_lidar``."""

    map_T_lidar = np.asarray(map_T_lidar, dtype=np.float64)
    lidar_to_camera = np.asarray(lidar_to_camera, dtype=np.float64)
    if map_T_lidar.shape != (4, 4) or lidar_to_camera.shape != (4, 4):
        raise ValueError("map_T_lidar and lidar_to_camera must both be 4x4 transforms")
    return map_T_lidar @ np.linalg.inv(lidar_to_camera)

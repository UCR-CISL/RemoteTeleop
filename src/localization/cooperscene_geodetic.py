"""Retained CooperScene geodetic pose composition for offline evaluation.

The map-assisted recipe is a conditional reference, not independently measured
pose ground truth. The 3 cm antenna offset remains an unverified assumption.
"""
from pathlib import Path
import csv
import json
import pickle

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from src.localization.cooperscene_reference import CooperSceneReference

RELEASE = Path("/mnt/bcc-data/data/Cooperscene/release/250928")
CALIBRATION = Path("/mnt/bcc-data/data/Cooperscene/backup/calib_data/250928")
GEOREFERENCE = Path("/home/jyue86/Downloads/riverside_for_justin/georeference.json")
DEFAULT_RECIPE = (
    Path(__file__).resolve().parents[2]
    / "config/cooperscene_geodetic_alignment_recipe.json"
)


class GeodeticPoses:
    """Raw RTK GNSS/INS poses of one take/vehicle, in the map's own ENU frame."""

    def __init__(
        self, take, agent, geo, *, release_root=RELEASE, calibration_root=CALIBRATION
    ):
        rows = list(
            csv.DictReader(
                (Path(release_root) / str(take) / str(agent) / "raw_gnss.csv")
                .read_text()
                .splitlines()
            )
        )
        self.t = np.array([int(r["gnss_ns"]) for r in rows], dtype=np.int64)
        order = np.argsort(self.t)
        self.t = self.t[order]
        rows = [rows[i] for i in order]
        lla = np.array(
            [
                [float(r["Latitude"]), float(r["Longitude"]), float(r["Altitude"])]
                for r in rows
            ]
        )
        origin = (
            geo["enu_origin"]["latitude_deg"],
            geo["enu_origin"]["longitude_deg"],
            geo["z_reference"]["z_ref_m"],
        )
        ref = CooperSceneReference(origin, np.eye(4), np.eye(4))
        self.p = np.array(
            [
                ref.enu_position(dict(Latitude=a, Longitude=b, Altitude=c))
                for a, b, c in lla
            ]
        )
        q = np.array(
            [
                [
                    float(r["Quat_qx"]),
                    float(r["Quat_qy"]),
                    float(r["Quat_qz"]),
                    float(r["Quat_qw"]),
                ]
                for r in rows
            ]
        )
        F = np.diag([1.0, -1.0, -1.0]) if agent == 1 else np.eye(3)
        self.R = Rotation.from_matrix(Rotation.from_quat(q).as_matrix() @ F)
        self.slerp = Slerp(self.t.astype(float), self.R)
        words = np.array([int(r["StatusWord"]) for r in rows], dtype=np.int64)
        self.rtk = (words >> 27) & 3
        with (Path(calibration_root) / f"tower{agent}-tfs.pkl").open("rb") as stream:
            E = np.asarray(
                pickle.load(stream)[f"tower{agent}-xsens,tower{agent}-lidar"]
            )
        self.E_inv = np.linalg.inv(E)

    def lidar_pose(self, t_ns, antenna_up_m=0.0):
        """antenna_up_m: logged position taken as the antenna, mounted this far above the Xsens along its z axis."""
        if not self.t[0] <= t_ns <= self.t[-1]:
            raise ValueError(
                "LiDAR timestamp exceeds GNSS support; clamping is forbidden"
            )
        t = float(t_ns)
        T = np.eye(4)
        T[:3, :3] = self.slerp([t]).as_matrix()[0]
        T[:3, 3] = [np.interp(t, self.t, self.p[:, k]) for k in range(3)]
        T[:3, 3] -= T[:3, :3] @ np.array([0.0, 0.0, antenna_up_m])
        return T @ self.E_inv

    def rtk_status(self, t_ns):
        return int(
            self.rtk[int(np.clip(np.searchsorted(self.t, t_ns), 0, len(self.t) - 1))]
        )


class FrozenProfile:
    """One shared V1 mounting delta and V1/take3-only map-frame position bias."""

    def __init__(self, recipe=DEFAULT_RECIPE):
        path = Path(recipe)
        self.path = (
            path / "resolution/final-profile/recipe.json" if path.is_dir() else path
        )
        self.recipe = json.loads(self.path.read_text())
        self.map_transform = self._matrix("G_frozen")
        self.mounting_delta = self._matrix("delta_mounting")
        self.take3_bias = self._matrix("take3_B_map_left")

    def _matrix(self, name):
        value = np.asarray(self.recipe[name], dtype=float)
        if value.shape != (4, 4) or not np.isfinite(value).all():
            raise ValueError(f"Invalid matrix {name}")
        if not np.allclose(value[3], [0, 0, 0, 1], atol=1e-12, rtol=0):
            raise ValueError(f"Invalid homogeneous row in {name}")
        return self._validated_pose(value)

    def map_T_lidar(self, geodetic_T_lidar, *, take, agent):
        """Place a raw-geodetic LiDAR pose using the retained recipe."""
        if agent not in (1, 2, 3) or take not in range(1, 15):
            raise ValueError("Expected CooperScene take 1–14 and vehicle 1–3")
        pose = self.map_transform @ self._validated_pose(geodetic_T_lidar)
        if agent == 1:
            pose = pose @ self.mounting_delta
            if take == 3:
                pose = self.take3_bias @ pose
        return pose

    @staticmethod
    def _validated_pose(value):
        matrix = np.asarray(value, dtype=float)
        if (
            matrix.shape != (4, 4)
            or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-12, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-6)
        ):
            raise ValueError("Expected a finite rigid homogeneous pose")
        return matrix

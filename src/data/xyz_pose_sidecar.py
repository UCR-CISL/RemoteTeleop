"""Strict, opt-in recorded-frame XYZ poses; never transform released poses."""
from pathlib import Path
import hashlib
import json

import numpy as np


def _rigid(value):
    pose = np.asarray(value, dtype=np.float64).reshape(4, 4)
    if (not np.isfinite(pose).all() or not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(pose[:3, :3]), 1., atol=1e-6)):
        raise ValueError("XYZ sidecar requires finite rigid poses")
    return pose


class VerifiedXYZPoseSidecar:
    """Bind an exact contiguous source range to verified poses and sensor bytes."""

    def __init__(self, path):
        self.path = Path(path)
        self.manifest = json.loads(self.path.read_text())
        m = self.manifest
        if m.get("version") != 1 or isinstance(m.get("take"), bool) or not isinstance(m.get("take"), int) or m["take"] <= 0 or str(m.get("agent")) not in ("1", "2", "3"):
            raise ValueError("invalid XYZ sidecar version/take/agent")
        for key in ("recipe_sha256", "map_sha256", "source_inputs_sha256"):
            self._digest(m["provenance"][key])
        if not m["provenance"].get("policy"):
            raise ValueError("XYZ sidecar must declare its pose policy")
        self.intrinsic = np.asarray(m["calibration"]["camera_intrinsic"], dtype=float).reshape(3, 3)
        if not np.isfinite(self.intrinsic).all() or min(self.intrinsic[0, 0], self.intrinsic[1, 1]) <= 0:
            raise ValueError("invalid XYZ camera intrinsic")
        for key in ("image_width", "image_height"):
            value = m["calibration"][key]
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError("XYZ image dimensions must be positive integers")
        self.image_shape = (m["calibration"]["image_height"], m["calibration"]["image_width"], 3)
        self.extrinsic = _rigid(m["calibration"]["camera_T_lidar"])
        self.frames = m["frames"]
        if not self.frames:
            raise ValueError("empty XYZ sidecar")
        indices = [x["source_index"] for x in self.frames]
        if any(isinstance(i, bool) or not isinstance(i, int) or i < 0 for i in indices) or indices != list(range(indices[0], indices[0] + len(indices))):
            raise ValueError("XYZ source indices must be unique and contiguous")
        if len({x["frame_id"] for x in self.frames}) != len(self.frames):
            raise ValueError("duplicate XYZ frame IDs")
        previous = -1
        recorded_previous = -1
        for row in self.frames:
            if not isinstance(row["frame_id"], str) or not row["frame_id"] or any(k in row for k in ("gt", "boxes", "reference_world_T_lidar")):
                raise ValueError("XYZ sidecar rows must be GT-free")
            for key in ("timestamp_us", "lidar_timestamp_ns", "image_timestamp_ns", "recorded_timestamp_us"):
                if isinstance(row[key], bool) or not isinstance(row[key], int) or row[key] < 0:
                    raise ValueError("XYZ timestamps must be exact nonnegative integers")
            if row["lidar_timestamp_ns"] // 1000 != row["timestamp_us"] or row["timestamp_us"] <= previous:
                raise ValueError("XYZ capture timestamps inconsistent/nonincreasing")
            previous = row["timestamp_us"]
            if row["image_timestamp_ns"] <= 0 or row["recorded_timestamp_us"] <= recorded_previous:
                raise ValueError("XYZ image/recorded clock invalid")
            recorded_previous = row["recorded_timestamp_us"]
            _rigid(row["world_T_lidar"])
            self._digest(row["image_bgr_sha256"])
            self._digest(row["lidar_f32_sha256"])
        self.start_index, self.end_index = indices[0], indices[-1]
        self._next = self.start_index

    @staticmethod
    def _digest(value):
        if not isinstance(value, str) or len(value) != 64 or any(c not in "0123456789abcdef" for c in value):
            raise ValueError("invalid XYZ provenance/payload SHA256")

    def validate_mcap(self, path, agent):
        path = Path(path)
        expected = self.manifest["mcap"]
        stat = path.stat()
        if (str(agent) != str(self.manifest["agent"]) or path.name != expected["name"]
                or stat.st_size != expected["size"] or stat.st_mtime_ns != expected["mtime_ns"]):
            raise ValueError("XYZ sidecar MCAP identity mismatch")
        if "sha256" in expected:
            digest = hashlib.sha256()
            with path.open("rb") as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(block)
            if digest.hexdigest() != expected["sha256"]:
                raise ValueError("XYZ sidecar MCAP checksum mismatch")

    def bind(self, source_index, metadata, image_bgr, lidar_points):
        if source_index != self._next or not self.start_index <= source_index <= self.end_index:
            raise ValueError("XYZ sidecar unexpected/missing source index")
        if np.asarray(image_bgr).dtype != np.uint8 or np.asarray(image_bgr).shape != self.image_shape:
            raise ValueError("XYZ replay image shape/dtype mismatch")
        row = self.frames[source_index - self.start_index]
        if str(metadata["frame_id"]) != row["frame_id"] or int(metadata["timestamp_us"]) != row["recorded_timestamp_us"]:
            raise ValueError("XYZ sidecar frame/recorded clock mismatch")
        if not np.array_equal(np.asarray(metadata["camera_intrinsic"]).reshape(3, 3), self.intrinsic):
            raise ValueError("XYZ sidecar camera K mismatch")
        recorded_lidar = metadata.get("map_T_lidar", metadata.get("world_T_ego"))
        relative = np.linalg.inv(_rigid(metadata["world_T_camera"])) @ _rigid(recorded_lidar)
        if not np.allclose(relative, self.extrinsic, atol=1e-10, rtol=0):
            raise ValueError("XYZ sidecar camera extrinsic mismatch")
        if lidar_points is None:
            supplement = row.get("points_npz")
            if supplement is None:
                raise ValueError("XYZ replay has no LiDAR; explicit supplemental NPZ required")
            path = (self.path.parent / supplement["path"]).resolve()
            if hashlib.sha256(path.read_bytes()).hexdigest() != supplement["sha256"]:
                raise ValueError("XYZ supplemental NPZ checksum mismatch")
            with np.load(path, allow_pickle=False) as payload:
                lidar_points = payload["lidar"].copy()
        points = np.ascontiguousarray(lidar_points, dtype="<f4")
        if points.ndim != 2 or points.shape[1] != 4 or not np.isfinite(points).all():
            raise ValueError("XYZ replay requires finite XYZI")
        for data, digest in ((np.ascontiguousarray(image_bgr).tobytes(), row["image_bgr_sha256"]),
                             (points.tobytes(), row["lidar_f32_sha256"])):
            if hashlib.sha256(data).hexdigest() != digest:
                raise ValueError("XYZ replay sensor payload checksum mismatch")
        pose = _rigid(row["world_T_lidar"])
        self._next += 1
        # GT and references never propagate. Ego is explicitly the LiDAR alias.
        result = dict(version=1, scene_generation=metadata["scene_generation"], frame_id=row["frame_id"],
                      request_id=f"{metadata['scene_generation']}:{row['frame_id']}",
                      end_of_scene=source_index == self.end_index,
                      timestamp_us=row["timestamp_us"], image_timestamp_ns=row["image_timestamp_ns"],
                      lidar_timestamp_ns=row["lidar_timestamp_ns"], recorded_timestamp_us=metadata["timestamp_us"],
                      recorded_source_sequence=source_index, source_sequence=source_index - self.start_index,
                      camera_intrinsic=self.intrinsic.reshape(-1).tolist(),
                      world_T_camera=(pose @ np.linalg.inv(self.extrinsic)).reshape(-1).tolist(),
                      map_T_lidar=pose.reshape(-1).tolist(), world_T_ego=pose.reshape(-1).tolist())
        return result, points

    def finish(self):
        if self._next != self.end_index + 1:
            raise ValueError("incomplete XYZ sidecar range")

"""Read-only native CooperScene scans for offline localization frame search.

No map fitting, cloud downsampling, range filtering or surface classification
occurs here. Recorded/geodetic poses are evaluation references, not independent
ground truth. The corrected geodetic source coordinate system retains ENU axes.
"""
from dataclasses import dataclass
import json
from pathlib import Path
import zipfile

import numpy as np
import yaml

from scripts.investigation.cooperscene_alignment.common import (
    DEFAULT_PRIOR,
    FrozenProfile,
)
from src.localization.cooperscene_geodetic import GEOREFERENCE, RELEASE, GeodeticPoses
from src.localization.cooperscene import _vehicle_boxes, pose_to_matrix


DEFAULT_ARCHIVE_ROOT = Path("/mnt/bcc-data/websites/data.ucr.edu-datasets-cooperscene")


@dataclass(frozen=True)
class SearchFrame:
    metadata: dict
    points_lidar: np.ndarray
    reference_map_T_lidar: np.ndarray
    source_T_lidar: np.ndarray
    reference_map_T_source: np.ndarray
    initial_map_T_source: np.ndarray

    @property
    def points_source(self):
        """Static LiDAR points placed in the corrected geodetic ENU source."""
        return (
            self.points_lidar @ self.source_T_lidar[:3, :3].T
            + self.source_T_lidar[:3, 3]
        )


class CooperSceneSearchSequence:
    """All native processed scans for one take/vehicle, ordered by capture time.

    Use as a context manager to close the read-only archive deterministically.
    ``frame_metadata`` does not decode PCDs; ``read_frame`` returns full finite,
    annotated-dynamic-masked XYZ points. A frame index is a chronological index,
    distinct from the released frame ID. No frame is silently skipped.
    """

    def __init__(
        self,
        take,
        agent,
        *,
        prior=DEFAULT_PRIOR,
        archive_root=DEFAULT_ARCHIVE_ROOT,
        georeference=GEOREFERENCE,
    ):
        self.take, self.agent = int(take), int(agent)
        if self.take not in range(1, 15) or self.agent not in (1, 2, 3):
            raise ValueError("Expected CooperScene take 1–14 and vehicle 1–3")
        paths = sorted(Path(archive_root).glob(f"*/{self.take}.zip"))
        if len(paths) != 1:
            raise ValueError(
                f"Expected exactly one archive for take {self.take}: {paths}"
            )
        self.archive_path = paths[0]
        self.profile = FrozenProfile(prior)
        self.map_T_source = self.profile.map_transform.copy()
        self._source_T_map = np.linalg.inv(self.map_T_source)
        self._archive = zipfile.ZipFile(self.archive_path, mode="r")
        try:
            prefix = f"{self.take}/{self.agent}/"
            names = set(self._archive.namelist())
            archive_meta = json.loads(self._archive.read(prefix + "metadata.json"))
            raw_meta = json.loads(
                (
                    RELEASE / str(self.take) / str(self.agent) / "metadata.json"
                ).read_text()
            )
            yaml_ids = {
                name[len(prefix) : -5]
                for name in names
                if name.startswith(prefix) and name.endswith(".yaml")
            }
            pcd_ids = {
                name[len(prefix) : -4]
                for name in names
                if name.startswith(prefix) and name.endswith(".pcd")
            }
            if yaml_ids != pcd_ids:
                raise ValueError(
                    f"Incomplete annotation/PCD pairs in {prefix}: {yaml_ids ^ pcd_ids}"
                )
            ids = [
                name[len(prefix) : -5]
                for name in names
                if name.startswith(prefix)
                and name.endswith(".yaml")
                and name[:-5] + ".pcd" in names
            ]
            self.frame_ids = sorted(
                ids, key=lambda fid: (int(archive_meta[fid]["lidar_ns"]), int(fid))
            )
            if not self.frame_ids:
                raise ValueError(f"No complete scans in {prefix}")
            self._metadata = archive_meta
            self.timestamps_ns = np.array(
                [int(archive_meta[fid]["lidar_ns"]) for fid in self.frame_ids],
                dtype=np.int64,
            )
            if np.any(np.diff(self.timestamps_ns) <= 0):
                raise ValueError("Duplicate or non-increasing scan timestamps")
            for fid in self.frame_ids:
                if int(archive_meta[fid]["lidar_ns"]) != int(raw_meta[fid]["lidar_ns"]):
                    raise ValueError(
                        f"Archive/raw LiDAR timestamp mismatch: {prefix}{fid}"
                    )
            self.geodetic = GeodeticPoses(
                self.take, self.agent, json.loads(Path(georeference).read_text())
            )
            if np.any(np.diff(self.geodetic.t) <= 0):
                raise ValueError("Duplicate or non-increasing raw GNSS timestamps")
            if (
                self.timestamps_ns[0] < self.geodetic.t[0]
                or self.timestamps_ns[-1] > self.geodetic.t[-1]
            ):
                raise ValueError(
                    "Scan timestamps exceed raw GNSS support; clamping is forbidden"
                )
            self.duration_s = float(
                (int(self.timestamps_ns[-1]) - int(self.timestamps_ns[0])) / 1e9
            )
        except Exception:
            self.close()
            raise

    def __len__(self):
        return len(self.frame_ids)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def close(self):
        self._archive.close()

    def frame_metadata(self, index):
        if not 0 <= index < len(self):
            raise IndexError(index)
        fid = self.frame_ids[index]
        stamp = int(self.timestamps_ns[index])
        elapsed = (stamp - int(self.timestamps_ns[0])) / 1e9
        return dict(
            dataset="cooperscene",
            take=self.take,
            agent=self.agent,
            run_id=f"cooperscene-take{self.take}-agent{self.agent}",
            frame_id=fid,
            frame_index=int(index),
            frame_count=len(self),
            timestamp_ns=stamp,
            lidar_ns=stamp,
            elapsed_capture_seconds=elapsed,
            elapsed_seconds=elapsed,
            percentage=100 * elapsed / self.duration_s if self.duration_s else 0.0,
            duration_seconds=self.duration_s,
            rtk=self.geodetic.rtk_status(stamp),
            lidar_minus_gnss_ms=(stamp - int(self._metadata[fid]["gnss_ns"])) / 1e6,
            archive=str(self.archive_path),
            profile_recipe=str(self.profile.path),
            dynamic_mask_applied=True,
            dynamic_box_padding_m=0.2,
            reference_kind="frozen corrected raw INS/GNSS geodetic reference; not independent GT",
            source_frame="corrected geodetic ENU; source_T_lidar=inverse(G)@reference_map_T_lidar",
            timestamp_clamped=False,
        )

    @staticmethod
    def _pcd_xyz(raw):
        """Decode the release's ASCII PCD layout with field/count validation."""
        header, marker, body = raw.partition(b"DATA ascii\n")
        if not marker:
            header, marker, body = raw.partition(b"DATA ascii\r\n")
        if not marker:
            raise ValueError("Expected ASCII PCD in processed CooperScene archive")
        fields = {}
        for line in header.decode("ascii").splitlines():
            words = line.split()
            if words and not words[0].startswith("#"):
                fields[words[0]] = words[1:]
        names = fields["FIELDS"]
        counts = [int(v) for v in fields.get("COUNT", ["1"] * len(names))]
        if len(counts) != len(names) or any(c <= 0 for c in counts):
            raise ValueError("Invalid PCD field counts")
        values = np.fromstring(body.decode("ascii"), sep=" ")
        columns = sum(counts)
        if values.size % columns:
            raise ValueError("Truncated PCD data")
        values = values.reshape(-1, columns)
        if "POINTS" in fields and len(values) != int(fields["POINTS"][0]):
            raise ValueError("PCD point-count mismatch")
        offsets = np.cumsum([0] + counts[:-1])
        indexes = []
        for axis in ("x", "y", "z"):
            i = names.index(axis)
            if counts[i] != 1:
                raise ValueError(f"PCD {axis} must be scalar")
            indexes.append(offsets[i])
        xyz = values[:, indexes]
        return xyz[np.isfinite(xyz).all(axis=1)]

    def read_frame(self, index):
        metadata = self.frame_metadata(index)
        stamp, fid = metadata["timestamp_ns"], metadata["frame_id"]
        # GeodeticPoses clips internally; explicitly reject beforehand.
        if not int(self.geodetic.t[0]) <= stamp <= int(self.geodetic.t[-1]):
            raise ValueError(f"Cannot interpolate GNSS at {stamp}; clamping forbidden")
        prefix = f"{self.take}/{self.agent}/{fid}"
        annotation = yaml.safe_load(self._archive.read(prefix + ".yaml"))
        points = self._pcd_xyz(self._archive.read(prefix + ".pcd"))
        finite_count = len(points)
        released = np.asarray(pose_to_matrix(annotation["lidar_pose"]), dtype=float)
        world = points @ released[:3, :3].T + released[:3, 3]
        keep = np.ones(len(points), dtype=bool)
        for box in _vehicle_boxes(annotation.get("vehicles"), Path(prefix + ".yaml")):
            pose = box.map_T_object
            box_points = (world - pose[:3, 3]) @ pose[:3, :3]
            keep &= ~np.all(
                abs(box_points) <= np.asarray(box.dimensions_lwh) / 2 + 0.2, axis=1
            )
        points = points[keep]
        reference = self.profile.map_T_lidar(
            self.geodetic.lidar_pose(stamp, 0.03), take=self.take, agent=self.agent
        )
        metadata.update(
            finite_points=finite_count,
            dynamic_removed_points=int((~keep).sum()),
            static_points=len(points),
        )
        return SearchFrame(
            metadata,
            points,
            reference,
            self._source_T_map @ reference,
            self.map_T_source.copy(),
            self.map_T_source.copy(),
        )

    def __iter__(self):
        for index in range(len(self)):
            yield self.read_frame(index)

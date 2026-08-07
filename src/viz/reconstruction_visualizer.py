"""Rerun validation view for one metric reconstructed vehicle."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import rerun as rr

from src.data import NuScenesFrame, VehicleBox
from src.reconstruction.projection import ProjectedBox


class ReconstructionRerunVisualizer:
    """Log authoritative frames, geometry, mask, and aligned asset to Rerun."""

    def show(
        self,
        *,
        frame: NuScenesFrame,
        vehicle: VehicleBox,
        mask: np.ndarray,
        aligned_mesh: Path,
        projected_box: ProjectedBox | None = None,
        raw_mesh: Path | None = None,
        object_T_raw_mesh: np.ndarray | None = None,
        spawn: bool = True,
    ) -> None:
        rr.init("teleop_sam3d_validation", spawn=spawn)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

        world_points = _transform_points(frame.world_T_lidar, frame.lidar_points[:, :3])
        rr.log("world/lidar", rr.Points3D(world_points, radii=0.025))
        self._log_pose("world/ego", frame.world_T_ego)
        self._log_pose("world/camera", frame.world_T_camera)
        height, width = frame.image.shape[:2]
        rr.log(
            "world/camera",
            rr.Pinhole(
                image_from_camera=frame.camera_intrinsic,
                resolution=[width, height],
                camera_xyz=rr.ViewCoordinates.RDF,
            ),
        )
        self._log_pose("world/car", vehicle.world_T_object)
        rr.log(
            "world/car/gt_box",
            rr.Boxes3D(half_sizes=[vehicle.dimensions_lwh / 2.0], colors=[0, 255, 0]),
        )
        rr.log("world/car/aligned_mesh", rr.Asset3D(path=aligned_mesh))
        if raw_mesh is not None and object_T_raw_mesh is not None:
            self._log_pose("world/car/raw_mesh", object_T_raw_mesh)
            rr.log("world/car/raw_mesh", rr.Asset3D(path=raw_mesh))
        rr.log("world/camera/image", rr.Image(frame.image))
        rr.log("world/camera/mask", rr.SegmentationImage(mask.astype(np.uint8)))
        if projected_box is not None:
            x0, y0, x1, y1 = projected_box.xyxy
            rr.log(
                "world/camera/image/projected_box",
                rr.Boxes2D(
                    mins=[[x0, y0]],
                    sizes=[[x1 - x0, y1 - y0]],
                    colors=[0, 255, 0],
                    labels=["GT vehicle.car"],
                ),
            )

    @staticmethod
    def _log_pose(entity: str, world_T_entity: np.ndarray) -> None:
        rr.log(
            entity,
            rr.Transform3D(
                translation=world_T_entity[:3, 3],
                mat3x3=world_T_entity[:3, :3],
            ),
        )


def _transform_points(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]

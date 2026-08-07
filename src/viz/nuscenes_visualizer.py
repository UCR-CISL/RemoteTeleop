"""Rerun visualization for NuScenes teleoperation sequences."""

from __future__ import annotations

import argparse

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from src.data.nuscenes_loader import NuScenesFrame, NuScenesSequence, NuScenesSequenceDataset


class NuScenesRerunVisualizer:
    """Log an entire NuScenes scene to Rerun on a frame-indexed timeline."""

    def __init__(self, sequence: NuScenesSequence, *, point_stride: int = 8) -> None:
        if not sequence:
            raise ValueError("Cannot visualize an empty NuScenes sequence")
        if point_stride < 1:
            raise ValueError("point_stride must be at least 1")
        self._sequence = sequence
        self._point_stride = point_stride

    def show(self) -> None:
        rr.init(f"nuscenes_{self._sequence.name}", spawn=True)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        rr.send_blueprint(self._blueprint())

        for frame_index, frame in enumerate(self._sequence):
            rr.set_time_sequence("frame", frame_index)
            rr.set_time_nanos("timestamp", frame.timestamp_us * 1_000)
            self._log_frame(frame)

    def _log_frame(self, frame: NuScenesFrame) -> None:
        rr.log(
            "world/lidar",
            rr.Transform3D(
                translation=frame.world_T_lidar[:3, 3],
                mat3x3=frame.world_T_lidar[:3, :3],
                relation=rr.TransformRelation.ParentFromChild,
            ),
        )
        points = frame.lidar_points[:: self._point_stride]
        intensity = np.clip(points[:, 3:4] * 255.0, 0, 255).astype(np.uint8)
        rr.log(
            "world/lidar/points",
            rr.Points3D(points[:, :3], colors=np.repeat(intensity, 3, axis=1), radii=0.03),
        )

        self._log_camera(frame)
        self._log_vehicle_boxes(frame)
        rr.log(
            "world/lidar/ego_gt_trajectory",
            rr.LineStrips3D([frame.trajectory], colors=[255, 0, 0], radii=0.12),
        )

    @staticmethod
    def _log_camera(frame: NuScenesFrame) -> None:
        height, width = frame.image.shape[:2]
        rr.log(
            "world/camera",
            rr.Transform3D(
                translation=frame.world_T_camera[:3, 3],
                mat3x3=frame.world_T_camera[:3, :3],
                relation=rr.TransformRelation.ParentFromChild,
            ),
            rr.Pinhole(
                image_from_camera=frame.camera_intrinsic,
                resolution=[width, height],
                camera_xyz=rr.ViewCoordinates.RDF,
            ),
        )
        rr.log("world/camera/image", rr.Image(frame.image))

    @staticmethod
    def _log_vehicle_boxes(frame: NuScenesFrame) -> None:
        if not frame.vehicle_boxes:
            rr.log("world/lidar/vehicle_boxes", rr.Boxes3D.from_fields(clear_unset=True))
            return
        centers = np.asarray([box.center for box in frame.vehicle_boxes])
        sizes_wlh = np.asarray([box.size_wlh for box in frame.vehicle_boxes])
        quaternions = [
            rr.Quaternion(xyzw=[0.0, 0.0, np.sin(box.yaw / 2), np.cos(box.yaw / 2)])
            for box in frame.vehicle_boxes
        ]
        rr.log(
            "world/lidar/vehicle_boxes",
            rr.Boxes3D(
                centers=centers,
                half_sizes=sizes_wlh[:, [1, 0, 2]] / 2,
                quaternions=quaternions,
                labels=[box.category for box in frame.vehicle_boxes],
                colors=[0, 255, 0],
            ),
        )

    @staticmethod
    def _blueprint() -> rrb.Blueprint:
        return rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial3DView(name="LiDAR", origin="world", contents=["/**"]),
                rrb.Spatial2DView(name="CAM_FRONT", origin="world/camera", contents=["/**"]),
            )
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Log one NuScenes scene to the Rerun viewer.")
    parser.add_argument("dataroot", help="Parent NuScenes directory containing v1.0-mini/ and samples/.")
    parser.add_argument("--scene", default="0", help="Scene index or NuScenes scene name (default: 0).")
    parser.add_argument("--version", default="v1.0-mini", help="NuScenes version directory.")
    parser.add_argument("--future-steps", type=int, default=12, help="GT trajectory poses per keyframe.")
    parser.add_argument("--point-stride", type=int, default=8, help="Log every Nth LiDAR point.")
    args = parser.parse_args()

    dataset = NuScenesSequenceDataset(args.dataroot, version=args.version, future_steps=args.future_steps)
    try:
        scene_index = int(args.scene)
    except ValueError:
        matches = [index for index, sequence in enumerate(dataset) if sequence.name == args.scene]
        if not matches:
            parser.error(f"No scene named {args.scene!r}")
        scene_index = matches[0]
    NuScenesRerunVisualizer(dataset[scene_index], point_stride=args.point_stride).show()


if __name__ == "__main__":
    main()

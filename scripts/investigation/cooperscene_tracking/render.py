"""Render labelled recorded-camera and LiDAR diagnostics from fixed pilot results."""
import argparse
import json
from pathlib import Path

import numpy as np

from src.viz.cooperscene_tracking import TrackingOverlayRenderer
from src.reconstruction.models import VehicleDimensions
from src.reconstruction.projection import project_vehicle_box


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frames",type=Path)
    parser.add_argument("--inputs",type=Path,required=True)
    parser.add_argument("--assignments",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--front-camera-only",action="store_true")
    args = parser.parse_args()
    assignments = [json.loads(x) for x in args.assignments.read_text().splitlines()]
    frames = [json.loads(x) for x in args.frames.read_text().splitlines()]
    if len(assignments)!=len(frames):
        raise ValueError("Assignments and frames have different coverage")
    args.output.mkdir(parents=True,exist_ok=False)
    renderer = TrackingOverlayRenderer()
    for kind in ("detections","tracks"):
        def images():
            for index,(frame,assigned) in enumerate(zip(frames,assignments,strict=True)):
                if any(frame[k]!=assigned[k] for k in ("sequence_id","frame_id","timestamp_us")):
                    raise ValueError("Assignment provenance mismatch")
                with np.load(args.inputs/frame["payload"]) as payload:
                    pose = np.asarray(frame["world_T_lidar"])
                    cloud = payload["lidar"][:,:3]@pose[:3,:3].T+pose[:3,3]
                    truth, predictions, association = frame["gt"], frame[kind], assigned[kind]
                    if args.front_camera_only:
                        def visible(box):
                            return project_vehicle_box(world_T_object=np.asarray(box["world_T_object"]),
                                world_T_camera=np.asarray(frame["world_T_camera"]),intrinsics=np.asarray(frame["camera_intrinsic"]),
                                dimensions=VehicleDimensions(*box["dimensions_lwh"]),image_size=(frame["image_height"],frame["image_width"]),
                                minimum_area_px=16) is not None
                        truth, predictions = list(filter(visible,truth)), list(filter(visible,predictions))
                        association = assigned["front_camera"][kind]
                    image = renderer.render(payload["image"], world_T_camera=frame["world_T_camera"],
                        intrinsic=frame["camera_intrinsic"],lidar_world=cloud,predictions=predictions,gt=truth,
                        assignment=association, title=f"{frame['sequence_id']} frame {frame['frame_id']} {'front FOV' if args.front_camera_only else 'all LiDAR'}",prediction_kind=kind)
                if index%10==0 or assigned[kind]["id_switches"] or index==len(frames)-1:
                    renderer.save_png(args.output/f"{kind}-{index:04d}.png",image)
                yield image
        renderer.write_video(args.output/f"{kind}.mp4",images(),fps=10)


if __name__=="__main__":
    main()

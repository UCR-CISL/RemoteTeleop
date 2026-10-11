"""Deterministic offline detector/tracker pilot; never starts vehicle services."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import zmq

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from src.localization.cooperscene import CooperSceneSequenceDataset
from src.localization.cooperscene_calibration import camera_intrinsic, compose_map_T_camera, camera_T_lidar
from src.localization.cooperscene_geodetic import FrozenProfile, GeodeticPoses, GEOREFERENCE
from src.realtime.bevfusion import BevFusionClient, BevFusionRequest
from src.realtime.bevfusion_tracker import BevFusionKalmanTracker, WorldDetection


class TrackingPilot:
    """Prepare read-only native recordings, then infer every selected frame."""

    def __init__(self, root: Path):
        self.root = root

    @staticmethod
    def box(pose, dimensions, track_id, score=1.0):
        return dict(world_T_object=np.asarray(pose).tolist(), dimensions_lwh=list(dimensions),
                    track_id=track_id, label="car", score=float(score))

    def prepare(self, data_root, search_root, count, selections):
        self.root.mkdir(parents=True, exist_ok=False)
        dataset = CooperSceneSequenceDataset(data_root, cache_frames=False)
        profile = FrozenProfile()
        geo = json.loads(GEOREFERENCE.read_text())
        manifest = dict(policy="earliest strict XYZ-qualified archived frame; frozen XYZ correction thereafter",
                        recipe_sha256=hashlib.sha256(profile.path.read_bytes()).hexdigest(),
                        reference="retained raw GNSS/INS ENU recipe; conditional map-assisted reference",
                        gt_policy="released GT transformed through released lidar inverse into reference lidar pose; G never applied to released poses",
                        gt_scope="take-namespaced IDs; each agent evaluated independently; annotated vehicles with lidar |x|,|y|<=54 m",
                        requested_count=count, sequences=[])
        for take, agent in selections:
            summary_path = search_root / f"take{take:02d}/take{take:02d}-vehicle{agent}/summary.json"
            summary = json.loads(summary_path.read_text())
            candidate = summary["methods"]["xyz"]["earliest_qualifying"]
            if candidate is None:
                raise ValueError(f"No qualified XYZ start for {take}/{agent}")
            split = next(s for s in ("train", "validate", "test") if (data_root/s/str(take)/str(agent)).is_dir())
            sequence = dataset.sequence(split, str(take), str(agent))
            metadata = json.loads((Path("/mnt/bcc-data/data/Cooperscene/release/250928")/str(take)/str(agent)/"metadata.json").read_text())
            geodetic = GeodeticPoses(take, agent, geo)
            correction = np.asarray(candidate["registration"]["map_T_source"]) @ np.linalg.inv(profile.map_transform)
            start = candidate["frame"]["frame_index"]
            name = f"take{take:02d}-vehicle{agent}"
            folder = self.root/name
            folder.mkdir()
            rows = []
            for index in range(start, min(start+count, len(sequence))):
                frame = sequence[index]
                meta = metadata[frame.frame_id]
                stamp = int(meta["lidar_ns"])
                reference = profile.map_T_lidar(geodetic.lidar_pose(stamp, .03), take=take, agent=agent)
                estimated = correction @ reference
                image = cv2.imread(str(frame.camera_path))
                if image is None:
                    raise ValueError(f"Missing image {frame.camera_path}")
                points = frame.require_lidar_points()
                points = points[np.isfinite(points).all(axis=1)]
                # GT is used only for evaluation and never added to detector/tracker requests.
                gt = []
                for box in frame.vehicle_boxes:
                    lidar_box = np.linalg.inv(frame.map_T_lidar) @ box.map_T_object
                    if max(abs(lidar_box[0,3]), abs(lidar_box[1,3])) <= 54:
                        gt.append(self.box(reference @ lidar_box, box.dimensions_lwh, f"take{take}:{box.track_id}"))
                path = folder/f"{index:06d}.npz"
                np.savez_compressed(path, image=image, lidar=points)
                rows.append(dict(sequence_id=name, frame_id=frame.frame_id, frame_index=index,
                                 timestamp_us=stamp//1000, image_minus_lidar_ms=(int(meta["image_ns"])-stamp)/1e6,
                                 payload=str(path.relative_to(self.root)), image_width=image.shape[1], image_height=image.shape[0],
                                 camera_intrinsic=camera_intrinsic(agent).tolist(),
                                 world_T_lidar=estimated.tolist(), world_T_camera=compose_map_T_camera(estimated,camera_T_lidar(agent)).tolist(),
                                 reference_world_T_lidar=reference.tolist(), gt=gt,
                                 localization_xyz_error_m=float(np.linalg.norm(estimated[:3,3]-reference[:3,3]))))
            (folder/"inputs.jsonl").write_text("".join(json.dumps(row)+"\n" for row in rows))
            manifest["sequences"].append(dict(sequence_id=name, start_index=start, start_frame=candidate["frame"]["frame_id"],
                                               count=len(rows), inputs=f"{name}/inputs.jsonl", correction=correction.tolist(),
                                               start_xyz_error_m=candidate["error"]["xyz_m"],
                                               archived_summary_sha256=hashlib.sha256(summary_path.read_bytes()).hexdigest()))
        (self.root/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")

    def infer(self, endpoint, output, timeout_seconds, selected_sequence=None):
        output.mkdir(parents=True, exist_ok=False)
        manifest = json.loads((self.root/"manifest.json").read_text())
        if selected_sequence:
            manifest["sequences"] = [s for s in manifest["sequences"] if s["sequence_id"] == selected_sequence]
            if not manifest["sequences"]:
                raise ValueError(f"Unknown sequence {selected_sequence}")
        (output/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
        context = zmq.Context()
        client = BevFusionClient.open(context, endpoint, identity="cooperscene-tracking-pilot", timeout_ms=100)
        try:
            with (output/"frames.jsonl").open("w") as stream:
                for sequence in manifest["sequences"]:
                    tracker = BevFusionKalmanTracker()
                    for index, line in enumerate((self.root/sequence["inputs"]).read_text().splitlines()):
                        row = json.loads(line)
                        with np.load(self.root/row["payload"]) as payload:
                            image = payload["image"]
                            row["image_width"], row["image_height"] = image.shape[1], image.shape[0]
                            points = np.ascontiguousarray(payload["lidar"],dtype="<f4")
                            request = BevFusionRequest(f"{row['sequence_id']}:{index}", row["sequence_id"], index,
                                row["frame_id"], row["timestamp_us"], "retained-xyz-pilot",
                                tuple(np.asarray(row["camera_intrinsic"]).ravel()),tuple(np.asarray(row["world_T_camera"]).ravel()),
                                tuple(np.asarray(row["world_T_lidar"]).ravel()),image.shape[1],image.shape[0],image.tobytes(),len(points),points.tobytes())
                        begun = time.monotonic()
                        if not client.submit(request):
                            raise RuntimeError("Unexpected in-flight detector request")
                        result = None
                        while result is None and time.monotonic()-begun < timeout_seconds:
                            result = client.poll(timeout_ms=100)
                        if result is None or result.error:
                            raise RuntimeError(f"Detector failure {request.request_id}: {None if result is None else result.error}")
                        world = np.asarray(row["world_T_lidar"])
                        detections = []
                        for box in result.boxes:
                            c,s = np.cos(box.yaw_rad),np.sin(box.yaw_rad)
                            pose = np.eye(4)
                            pose[:3,:3] = [[c,-s,0],[s,c,0],[0,0,1]]
                            pose[:3,3] = box.center_xyz
                            detections.append(WorldDetection(world@pose,box.dimensions_lwh,box.class_name,box.score))
                        tracker.update(detections,timestamp_us=row["timestamp_us"])
                        row["detections"] = [self.box(d.world_T_object,d.dimensions_lwh,f"det:{index}:{i}",d.score) for i,d in enumerate(detections)]
                        row["tracks"] = [self.box(t.world_T_object,t.dimensions_lwh,t.track_id,t.score) for t in tracker.boxes_at(row["timestamp_us"])]
                        row["detector_timings_ms"] = dict(result.timings_ms)
                        row["client_latency_ms"] = (time.monotonic()-begun)*1000
                        stream.write(json.dumps(row)+"\n")
                        stream.flush()
                        print(f"{row['sequence_id']} {index+1}/{sequence['count']} detections={len(detections)}",flush=True)
        finally:
            client.close()
            context.term()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("prepare","infer"))
    parser.add_argument("--inputs",type=Path,required=True)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--count",type=int,default=60)
    parser.add_argument("--data-root",type=Path,default=Path("/mnt/bcc-data/data/CooperScene"))
    parser.add_argument("--search-root",type=Path,default=Path("artifacts/worktree-archive-20261011/alignment_artifacts/recipe-frame-search-full-20261004"))
    parser.add_argument("--endpoint",default="tcp://127.0.0.1:18766")
    parser.add_argument("--timeout-seconds",type=float,default=120)
    parser.add_argument("--sequence", help="Infer only this prepared sequence; start with take01-vehicle1")
    args = parser.parse_args()
    pilot = TrackingPilot(args.inputs)
    if args.mode == "prepare":
        if args.count<=0: parser.error("count must be positive")
        pilot.prepare(args.data_root,args.search_root,args.count,[(1,1),(3,1),(4,2)])
    else:
        if args.output is None: parser.error("infer requires --output")
        pilot.infer(args.endpoint,args.output,args.timeout_seconds,args.sequence)


if __name__ == "__main__":
    main()

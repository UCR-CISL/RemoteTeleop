"""Deterministic offline detector/tracker pilot; never starts vehicle services."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
from pathlib import Path
import sys
import time
from zipfile import ZIP_DEFLATED, ZipFile

import cv2
import numpy as np
import zmq

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from src.localization.cooperscene import CooperSceneSequenceDataset
from src.localization.cooperscene_calibration import camera_intrinsic, compose_map_T_camera, camera_T_lidar
from src.localization.cooperscene_geodetic import FrozenProfile, GeodeticPoses, GEOREFERENCE
from src.realtime.bevfusion import BevFusionClient, BevFusionRequest
from src.realtime.bevfusion_tracker import BevFusionKalmanTracker, WorldDetection


class ArchivedXYZPosePolicy:
    """Accepted independent XYZ estimates with explicit raw-pose propagation.

    Rejections never supply an estimator pose. The last accepted map correction
    follows the current retained raw GNSS/INS reference until a new acceptance.
    This is an offline archived-frame policy, not a continuous XYZ estimator.
    """

    def __init__(self, directory: Path, *, recipe_sha256: str, map_sha256=None):
        self.directory = directory
        protocol_path = directory.parent / "protocol.json"
        protocol_bytes = protocol_path.read_bytes()
        protocol = json.loads(protocol_bytes)
        if protocol["recipe_sha256"] != recipe_sha256:
            raise ValueError("Archived XYZ recipe hash differs from prepared/current recipe")
        if map_sha256 is not None and protocol["map_sha256"] != map_sha256:
            raise ValueError("Archived XYZ map hash differs from selected map")
        self.provenance = dict(recipe_sha256=recipe_sha256, map_sha256=protocol["map_sha256"],
                               map_hash_verified=map_sha256 is not None,
                               protocol_sha256=hashlib.sha256(protocol_bytes).hexdigest(),
                               archive_inputs_sha256=hashlib.sha256((directory / "inputs.json").read_bytes()).hexdigest())
        self.record_hash = hashlib.sha256()
        self.last_index = self.last_timestamp = None
        self.accepted_index = self.accepted_timestamp = None
        self.correction = None

    def pose(self, row):
        index, timestamp = row["frame_index"], row["timestamp_us"]
        if self.last_index is not None and (index != self.last_index + 1 or timestamp <= self.last_timestamp):
            raise ValueError("Archived XYZ input frames must be consecutive with increasing timestamps")
        raw = (self.directory / f"frame-{index:06d}.json").read_bytes()
        record = json.loads(raw)
        metadata, xyz = record["frame"], record["methods"]["xyz"]
        if (metadata["frame_index"] != index or metadata["frame_id"] != row["frame_id"]
                or metadata["timestamp_ns"] // 1000 != timestamp
                or metadata["lidar_ns"] != metadata["timestamp_ns"]):
            raise ValueError("Archived XYZ frame ID/index/capture timestamp mismatch")
        reference = FrozenProfile._validated_pose(row["reference_world_T_lidar"])
        # Certified rejections may have no optimized error/pose at all.
        archived_reference = xyz.get("error", {}).get("reference_map_T_lidar")
        if archived_reference is None:
            archived_reference = record["initial"]["error"]["reference_map_T_lidar"]
        archived_reference = FrozenProfile._validated_pose(archived_reference)
        if not np.allclose(reference, archived_reference, atol=1e-8, rtol=0):
            raise ValueError("Archived XYZ reference differs from current retained raw recipe pose")
        accepted = xyz.get("accepted") is True
        if accepted:
            if not all(xyz.get(key) is True for key in
                       ("upstream_scan_accepted", "fusion_attempted", "fusion_converged")):
                raise ValueError("Archived XYZ acceptance contradicts upstream/fusion flags")
            estimated = FrozenProfile._validated_pose(xyz["error"]["estimated_map_T_lidar"])
            self.correction = estimated @ np.linalg.inv(reference)
            self.accepted_index, self.accepted_timestamp = index, timestamp
        else:
            if self.correction is None:
                raise ValueError("Archived XYZ policy must start on an accepted frame")
            estimated = self.correction @ reference
        self.last_index, self.last_timestamp = index, timestamp
        self.record_hash.update(raw)
        return estimated, dict(policy="archived_independent_xyz" if accepted else "raw_recipe_propagated_last_accepted_xyz_correction",
                               accepted=accepted, propagated=not accepted,
                               source_accepted_frame_index=self.accepted_index,
                               seconds_since_acceptance=(timestamp-self.accepted_timestamp)/1e6,
                               archive_status=xyz.get("status"),
                               archive_record_sha256=hashlib.sha256(raw).hexdigest())


class TrackingPilot:
    """Prepare read-only native recordings, then infer every selected frame."""

    def __init__(self, root: Path, *, payload_compression_level: int | None = None):
        if payload_compression_level is not None and (
            isinstance(payload_compression_level, bool)
            or not isinstance(payload_compression_level, int)
            or not 0 <= payload_compression_level <= 9
        ):
            raise ValueError("payload compression level must be an integer from 0 to 9")
        self.root = root
        self.payload_compression_level = payload_compression_level

    def _write_sensor_payload(self, path: Path, image: np.ndarray, points: np.ndarray) -> None:
        if self.payload_compression_level is None:
            np.savez_compressed(path, image=image, lidar=points)
            return
        with ZipFile(path, "x", compression=ZIP_DEFLATED,
                     compresslevel=self.payload_compression_level) as archive:
            for name, array in (("image", image), ("lidar", points)):
                with archive.open(f"{name}.npy", "w", force_zip64=True) as stream:
                    np.lib.format.write_array(stream, array, allow_pickle=False)

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
        if self.payload_compression_level is not None:
            manifest["payload_storage"] = dict(compression="ZIP_DEFLATED",
                                               level=self.payload_compression_level, lossless=True)
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
                self._write_sensor_payload(path, image, points)
                rows.append(dict(sequence_id=name, frame_id=frame.frame_id, frame_index=index,
                                 timestamp_us=stamp//1000, image_timestamp_ns=int(meta["image_ns"]),
                                 image_minus_lidar_ms=(int(meta["image_ns"])-stamp)/1e6,
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

    def rebase_xyz(self, output, search_root, map_path=None):
        """Export corrected metadata while reusing each original NPZ read-only."""
        if output.exists():
            raise FileExistsError(output)
        source_bytes = (self.root / "manifest.json").read_bytes()
        manifest = json.loads(source_bytes)
        recipe_hash = hashlib.sha256(FrozenProfile().path.read_bytes()).hexdigest()
        if manifest["recipe_sha256"] != recipe_hash:
            raise ValueError("Prepared inputs differ from current retained recipe")
        map_hash = None if map_path is None else hashlib.sha256(map_path.read_bytes()).hexdigest()
        exported = []
        for sequence in manifest["sequences"]:
            match = re.fullmatch(r"take(\d+)-vehicle([123])", sequence["sequence_id"])
            if not match:
                raise ValueError("Unknown CooperScene sequence")
            take, agent = map(int, match.groups())
            directory = search_root / f"take{take:02d}" / sequence["sequence_id"]
            summary_path = directory / "summary.json"
            candidate = json.loads(summary_path.read_text())["methods"]["xyz"]["earliest_qualifying"]
            if (candidate is None or candidate["frame"]["frame_index"] != sequence["start_index"]
                    or candidate["frame"]["frame_id"] != sequence["start_frame"]):
                raise ValueError("Prepared start differs from earliest strict XYZ-qualified archived frame")
            policy = ArchivedXYZPosePolicy(directory, recipe_sha256=recipe_hash, map_sha256=map_hash)
            input_path = self.root / sequence["inputs"]
            input_bytes = input_path.read_bytes()
            rows = []
            for line in input_bytes.decode().splitlines():
                row = json.loads(line)
                if row["sequence_id"] != sequence["sequence_id"]:
                    raise ValueError("Input sequence ID differs from manifest")
                if not rows and (row["frame_index"] != sequence["start_index"] or row["frame_id"] != sequence["start_frame"]):
                    raise ValueError("First input differs from prepared start")
                estimated, diagnostics = policy.pose(row)
                payload = (self.root / row["payload"]).resolve()
                if not payload.is_file():
                    raise FileNotFoundError(payload)
                row["payload"] = os.path.relpath(payload, output.resolve())
                row["world_T_lidar"] = estimated.tolist()
                row["world_T_camera"] = compose_map_T_camera(estimated, camera_T_lidar(agent)).tolist()
                reference = np.asarray(row["reference_world_T_lidar"])
                row["localization_xyz_error_m"] = float(np.linalg.norm(estimated[:3,3]-reference[:3,3]))
                row["localization"] = diagnostics
                rows.append(row)
            if len(rows) != sequence["count"] or not rows:
                raise ValueError("Input count differs from manifest or is empty")
            revised = {key: value for key, value in sequence.items() if key != "correction"}
            revised.update(xyz_pose_provenance=policy.provenance,
                           source_inputs_sha256=hashlib.sha256(input_bytes).hexdigest(),
                           archived_summary_sha256=hashlib.sha256(summary_path.read_bytes()).hexdigest(),
                           archive_records_sha256=policy.record_hash.hexdigest(),
                           accepted_frames=sum(row["localization"]["accepted"] for row in rows),
                           propagated_frames=sum(row["localization"]["propagated"] for row in rows))
            exported.append((revised, rows))
        manifest.update(policy="earliest strict XYZ-qualified start; independent per-frame archived XYZ acceptances; rejected frames propagate last accepted correction on current retained raw recipe",
                        continuous_xyz_estimator=False, independent_per_frame_xyz=True,
                        rejected_pose_policy="last accepted pose @ inverse(reference at acceptance) @ current raw recipe reference; rejected optimizer proposals never consumed",
                        source_manifest_sha256=hashlib.sha256(source_bytes).hexdigest(),
                        payload_policy="reuse original read-only NPZ via relative paths; no array copies or rewrites",
                        sequences=[sequence for sequence, _ in exported])
        output.mkdir(parents=True, exist_ok=False)
        for sequence, rows in exported:
            destination = output / sequence["inputs"]
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text("".join(json.dumps(row)+"\n" for row in rows))
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")

    def infer(self, endpoint, output, timeout_seconds, selected_sequence=None, intensity_clip=None,
              past_sweeps=0, sweep_max_age_seconds=1.0, limit_frames=None, intensity_scale=1.0):
        if intensity_clip is not None and (not np.isfinite(intensity_clip) or intensity_clip <= 0):
            raise ValueError("intensity clip must be finite and positive")
        if not np.isfinite(intensity_scale) or intensity_scale <= 0:
            raise ValueError("intensity scale must be finite and positive")
        if isinstance(past_sweeps, bool) or not isinstance(past_sweeps, int) or not 0 <= past_sweeps <= 9:
            raise ValueError("past_sweeps must be an integer from zero to nine")
        if not np.isfinite(sweep_max_age_seconds) or sweep_max_age_seconds <= 0:
            raise ValueError("sweep maximum age must be finite and positive")
        if limit_frames is not None and (isinstance(limit_frames, bool) or not isinstance(limit_frames, int) or limit_frames < 1):
            raise ValueError("limit_frames must be a positive integer")
        output.mkdir(parents=True, exist_ok=False)
        manifest = json.loads((self.root/"manifest.json").read_text())
        if selected_sequence:
            manifest["sequences"] = [s for s in manifest["sequences"] if s["sequence_id"] == selected_sequence]
            if not manifest["sequences"]:
                raise ValueError(f"Unknown sequence {selected_sequence}")
        if limit_frames is not None:
            for sequence in manifest["sequences"]:
                sequence["count"] = min(sequence["count"], limit_frames)
        if past_sweeps:
            manifest["detector_sweeps"] = dict(maximum_past_sweeps=past_sweeps,
                maximum_age_seconds=sweep_max_age_seconds, compensation="past-only archived XYZ ego poses",
                feature_layout="x,y,z,intensity,past_time_lag_seconds", object_motion_compensated=False)
        (output/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
        context = zmq.Context()
        client = BevFusionClient.open(context, endpoint, identity="cooperscene-tracking-pilot", timeout_ms=100)
        try:
            with (output/"frames.jsonl").open("w") as stream:
                for sequence in manifest["sequences"]:
                    tracker = BevFusionKalmanTracker()
                    sweeps = None
                    if past_sweeps:
                        from src.realtime.bevfusion_sweeps import PastSweepAssembler
                        sweeps = PastSweepAssembler(maximum_past_sweeps=past_sweeps, maximum_age_seconds=sweep_max_age_seconds)
                    lines = (self.root/sequence["inputs"]).read_text().splitlines()
                    for index, line in enumerate(lines[:sequence["count"]]):
                        row = json.loads(line)
                        with np.load(self.root/row["payload"]) as payload:
                            image = payload["image"]
                            row["image_width"], row["image_height"] = image.shape[1], image.shape[0]
                            points = np.ascontiguousarray(payload["lidar"],dtype="<f4")
                            if intensity_scale != 1.0:
                                points[:,3] *= intensity_scale
                                row["lidar_intensity_scale"] = intensity_scale
                            if intensity_clip is not None:
                                points[:,3] = np.clip(points[:,3],0,intensity_clip)
                            row["lidar_intensity_clip"] = intensity_clip
                            if sweeps is not None:
                                points = sweeps.add(points, np.asarray(row["world_T_lidar"]), row["timestamp_us"], row["sequence_id"])
                                row["detector_sweeps"] = dict(maximum_past_sweeps=past_sweeps,
                                    maximum_age_seconds=sweep_max_age_seconds, input_point_count=len(points),
                                    distinct_time_lags_seconds=np.unique(points[:,4]).tolist())
                            request = BevFusionRequest(f"{row['sequence_id']}:{index}", row["sequence_id"], index,
                                row["frame_id"], row["timestamp_us"], "retained-xyz-pilot",
                                tuple(np.asarray(row["camera_intrinsic"]).ravel()),tuple(np.asarray(row["world_T_camera"]).ravel()),
                                tuple(np.asarray(row["world_T_lidar"]).ravel()),image.shape[1],image.shape[0],image.tobytes(),len(points),points.tobytes(),
                                lidar_feature_count=points.shape[1])
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
    parser.add_argument("mode", choices=("prepare","infer","rebase-xyz"))
    parser.add_argument("--inputs",type=Path,required=True)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--count",type=int,default=60)
    parser.add_argument("--payload-compression-level", type=int, choices=range(10),
                        help="Lossless NPZ compression level for prepare; 1 favors speed")
    parser.add_argument("--data-root",type=Path,default=Path("/mnt/bcc-data/data/CooperScene"))
    parser.add_argument("--search-root",type=Path,default=Path("artifacts/worktree-archive-20261011/alignment_artifacts/recipe-frame-search-full-20261004"))
    parser.add_argument("--endpoint",default="tcp://127.0.0.1:18766")
    parser.add_argument("--timeout-seconds",type=float,default=120)
    parser.add_argument("--sequence", help="Infer only this prepared sequence; start with take01-vehicle1")
    parser.add_argument("--intensity-clip",type=float,help="Offline input experiment; clamp only intensity, preserving XYZ and cached inputs")
    parser.add_argument("--intensity-scale",type=float,default=1.0,help="Explicit model input intensity scaling before optional clipping")
    parser.add_argument("--past-sweeps",type=int,default=0,help="Offline past-only XYZ ego-compensated sweep trial; default single scan")
    parser.add_argument("--sweep-max-age-seconds",type=float,default=1.0)
    parser.add_argument("--limit-frames",type=int,help="Limit inference to the first N prepared frames per sequence")
    parser.add_argument("--map", type=Path, help="Verify archived XYZ map SHA256 during metadata export")
    args = parser.parse_args()
    pilot = TrackingPilot(args.inputs, payload_compression_level=args.payload_compression_level)
    if args.mode == "prepare":
        if args.count<=0: parser.error("count must be positive")
        selections = [(1,1)]
        if args.sequence:
            match = re.fullmatch(r"take(\d+)-vehicle([123])",args.sequence)
            if not match: parser.error("sequence must be takeNN-vehicleN")
            selections = [(int(match[1]),int(match[2]))]
        pilot.prepare(args.data_root,args.search_root,args.count,selections)
    elif args.mode == "rebase-xyz":
        if args.output is None: parser.error("rebase-xyz requires --output")
        pilot.rebase_xyz(args.output, args.search_root, args.map)
    else:
        if args.output is None: parser.error("infer requires --output")
        pilot.infer(args.endpoint,args.output,args.timeout_seconds,args.sequence,args.intensity_clip,
                    args.past_sweeps,args.sweep_max_age_seconds,args.limit_frames,args.intensity_scale)


if __name__ == "__main__":
    main()

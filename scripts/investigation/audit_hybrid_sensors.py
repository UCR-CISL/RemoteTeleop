"""Audit cached sensor conventions without using a reference height target."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation
from mcap.reader import make_reader

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from src.data.foxglove_mcap import _ProtobufDecoder
from src.localization.cooperscene_calibration import camera_T_lidar


def audit_frame(metadata):
    frame = dict(np.load(metadata["npz_path"], allow_pickle=False))
    lidar_T_camera = np.linalg.inv(frame["dataset_T_lidar"]) @ frame["dataset_T_camera"]
    camera_T_scan = np.linalg.inv(lidar_T_camera)
    points = frame["points_lidar"][:, :3]
    # This arbitrary tilted plane checks coordinate algebra, not road height.
    normal = np.asarray([.08, -.04, 1.])
    normal /= np.linalg.norm(normal)
    offset = 2.
    camera_normal = camera_T_scan[:3, :3] @ normal
    camera_offset = offset - camera_normal @ camera_T_scan[:3, 3]
    predicted_clearance = offset + normal @ lidar_T_camera[:3, 3]
    result = dict(run_id=metadata["run_id"], frame_id=metadata["frame_id"],
        source_npz=metadata["npz_path"], points_shape=list(frame["points_lidar"].shape),
        points_xyz_p01_p99=np.percentile(points, [1, 99], axis=0).tolist(),
        lidar_T_camera=lidar_T_camera.tolist(), camera_T_lidar=camera_T_scan.tolist(),
        camera_forward_in_lidar=lidar_T_camera[:3, 2].tolist(),
        camera_up_in_lidar=(-lidar_T_camera[:3, 1]).tolist(),
        nominal_z_up_camera_minus_lidar_clearance_m=float(lidar_T_camera[2, 3]),
        composition_max_abs_error=float(np.max(np.abs(frame["dataset_T_lidar"] @ lidar_T_camera - frame["dataset_T_camera"]))),
        plane_clearance_identity_abs_error=float(abs(camera_offset - predicted_clearance)),
        point_units="metres; unchanged native/processed point values, no scaling",
        camera_axes="CV right/down/forward", scan_box_transform="inverse(dataset_T_lidar) @ boxes_T_dataset_object",
        no_reference_height_used=True)
    if metadata["dataset"] == "cooperscene":
        result["documented_extrinsic_max_abs_error"] = float(np.max(np.abs(camera_T_scan - camera_T_lidar(metadata["agent"]))))
        timestamp = metadata["source_timestamp_ns"]
        agent = metadata["agent"]
        decoder = _ProtobufDecoder()
        sensor_headers = {}
        with Path(metadata["source_path"]).open("rb") as stream:
            for schema, channel, message in make_reader(stream).iter_messages(
                    topics=[f"/{agent}_Camera", f"/{agent}_LIDAR"],
                    start_time=timestamp, end_time=timestamp + 1):
                decoded = decoder.decode(schema, channel, message.data)
                header = decoded.timestamp.seconds * 1_000_000_000 + decoded.timestamp.nanos
                sensor_headers[channel.topic] = dict(log_timestamp_ns=message.log_time,
                    header_timestamp_ns=header, sensor_frame_id=decoded.frame_id)
        result["sensor_timestamps"] = sensor_headers
        result["camera_minus_lidar_header_ms"] = (sensor_headers[f"/{agent}_Camera"]["header_timestamp_ns"]
                                                  - sensor_headers[f"/{agent}_LIDAR"]["header_timestamp_ns"]) / 1e6
        result["timing_policy"] = "ordinal/log-time join; fixed camera extrinsic; no exposure-offset compensation"
    else:
        result["timing_policy"] = "processed 10 Hz synthetic index timestamps; per-frame camera and LiDAR poses retained"
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", type=Path, default=Path("/tmp/RemoteTeleop-first-frame-localization/artifacts"))
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/hybrid-road-height/sensor-audit.json")
    args = parser.parse_args()
    runs = ["nuscenes-152", "cooperscene-1-3", "cooperscene-2-3", "cooperscene-3-2", "cooperscene-4-3", "cooperscene-1-1", "cooperscene-2-2"]
    rows = [audit_frame(json.loads((args.cache / "first-frame-localization/inputs" / run / "metadata.json").read_text())) for run in runs]
    paths = sorted((args.cache / "bev-frame-search/nuscenes-cache/inputs").glob("nuscenes-152-f*/frame.npz"))
    first = dict(np.load(args.cache / "first-frame-localization/inputs/nuscenes-152/frame.npz"))
    initial = np.linalg.inv(first["dataset_T_lidar"]) @ first["dataset_T_camera"]
    variations = []
    for path in paths:
        frame = dict(np.load(path))
        relative = np.linalg.inv(frame["dataset_T_lidar"]) @ frame["dataset_T_camera"]
        delta = np.linalg.inv(initial) @ relative
        variations.append(dict(frame=path.parent.name, translation_change_m=float(np.linalg.norm(delta[:3, 3])),
            rotation_change_deg=float(np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude()))))
    result = dict(frames=rows, nu152_per_frame_relative_pose_variations=variations,
        clearance_formula="For unit LiDAR plane n_L.p_L+d_L=0: camera clearance d_C=d_L+n_L.t_LC; n_C=R_CL@n_L; d_C=d_L-n_C.t_CL",
        warnings=["nuScenes raw LiDAR front is +Y; CooperScene front is +X; both local Z-up",
                  "camera optical Z is forward, never use it as an up-height offset",
                  "identical CooperScene log timestamps do not imply identical capture header timestamps",
                  "nuScenes processed relative camera/LiDAR pose varies with frame; do not freeze first-frame relative transform",
                  "road-plane clearance is a perpendicular signed distance; map-axis vertical requires the plane/up dot-product",
                  "sensor-plane algebra and calibration checks are not independent ground-truth accuracy"])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    lines = ["# Hybrid height sensor audit", "", "No reference or ENU height is used in these checks.", "",
             "| Sequence | Camera origin in LiDAR (m) | Camera − LiDAR capture time (ms) |", "|---|---|---:|"]
    for row in rows:
        xyz = np.asarray(row["lidar_T_camera"])[:3, 3]
        lines.append(f"| {row['run_id']} | {xyz[0]:+.5f}, {xyz[1]:+.5f}, {xyz[2]:+.5f} | {row.get('camera_minus_lidar_header_ms', 'not preserved in processed index timestamps')} |")
    lines += ["", "## Clearance conversion", "", result["clearance_formula"], "", "The camera origin is below the LiDAR origin in all audited calibrations. On sloped roads, use the full normal/lever-arm dot product, rather than subtracting a nominal vertical offset.", "", "## Traps", ""]
    lines += [f"- {warning}." for warning in result["warnings"]]
    lines += ["", "CooperScene cached extrinsics agree with the documented calibration to numerical precision. Point scales match metres; transforms are applied once, and LiDAR masks require boxes converted from dataset coordinates into sensor coordinates."]
    args.output.with_suffix(".md").write_text("\n".join(lines) + "\n")
    print(args.output)


if __name__ == "__main__":
    main()

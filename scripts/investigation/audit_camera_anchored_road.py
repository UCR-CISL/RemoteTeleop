"""Derive nuScenes first-camera road planes and diagnose later plane stationarity.

No Gaussian map is read. No localization/replay is run. Recorded relative sensor
poses are used only for the later-frame stationarity diagnostic, never fitting a
sensor-height correction to evaluation errors.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def transform_plane(normal, offset, target_T_source):
    normal_target = target_T_source[:3, :3] @ np.asarray(normal)
    return normal_target, float(offset - normal_target @ target_T_source[:3, 3])


def audit(measurements: list[dict]) -> dict:
    initial, later = [], []
    frames = {row['run_id']: row for row in measurements
              if row['run_id'].startswith('nuscenes-') and row['frame_offset'] == 0}
    for run_id, row in sorted(frames.items()):
        entry = dict(run_id=run_id, accepted=row['accepted'], reason=row['reason'])
        if not row['accepted']:
            initial.append(entry)
            continue
        with np.load(row['source_npz'], allow_pickle=False) as frame:
            map_T_dataset = np.linalg.inv(frame['dataset_T_camera'])
            map_T_lidar = map_T_dataset @ frame['dataset_T_lidar']
            identity_error = float(np.max(np.abs(map_T_dataset @ frame['dataset_T_camera'] - np.eye(4))))
        plane = row['lidar_plane_sensor']
        normal, offset = transform_plane(plane['normal'], plane['offset_m'], map_T_lidar)
        up = map_T_dataset[:3, :3] @ np.array([0., 0., 1.])
        entry.update(normal=normal.tolist(), offset_m=offset, map_up=up.tolist(),
                     map_T_first_lidar=map_T_lidar.tolist(),
                     lidar_normal_clearance_m=row['lidar_normal_clearance_m'],
                     lidar_vertical_clearance_m=row['lidar_vertical_clearance_m'],
                     first_camera_normal_clearance_m=offset,
                     first_camera_vertical_clearance_m=offset / float(normal @ up),
                     first_camera_identity_error=identity_error,
                     source_npz=row['source_npz'])
        initial.append(entry)
        if run_id != 'nuscenes-152':
            continue
        for current in measurements:
            if current['run_id'] != run_id or current['frame_offset'] not in (36, 95, 190):
                continue
            result = dict(run_id=run_id, frame_offset=current['frame_offset'],
                          accepted=current['accepted'], reason=current['reason'],
                          source_npz=current['source_npz'])
            if current['accepted']:
                with np.load(current['source_npz'], allow_pickle=False) as frame:
                    current_pose = map_T_dataset @ frame['dataset_T_lidar']
                current_plane = current['lidar_plane_sensor']
                current_normal, current_offset = transform_plane(
                    current_plane['normal'], current_plane['offset_m'], current_pose)
                predicted = float((normal @ current_pose[:3, 3] + offset) / (normal @ up))
                observed = float((current_normal @ current_pose[:3, 3] + current_offset) / (current_normal @ up))
                delta = current_pose[:3, 3] - map_T_lidar[:3, 3]
                result.update(initial_plane_predicted_vertical_clearance_m=predicted,
                              current_plane_measured_vertical_clearance_m=observed,
                              clearance_prediction_minus_measurement_m=predicted - observed,
                              local_road_height_minus_initial_plane_m=predicted - observed,
                              horizontal_distance_from_initial_lidar_m=float(np.linalg.norm(delta - up * (up @ delta))),
                              normal_disagreement_deg=float(np.degrees(np.arccos(np.clip(normal @ current_normal, -1, 1)))),
                              current_plane_native_map=dict(normal=current_normal.tolist(), offset_m=current_offset))
            later.append(result)
    return dict(map_chart='native map is first front-camera frame',
                assumption='Metric map shares the documented training camera chart and accurately represents local road.',
                map_geometry_read=False, reference_alignment_read=False,
                first_camera_identity_is_constructed_not_independent_export_verification=True,
                later_pose_usage='Recorded relative sensor poses diagnose plane stationarity only; no height estimator correction fitted.',
                initial_planes=initial, later_stationarity=later)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--measurements', type=Path, default=Path('artifacts/clearance-transfer/measurements.json'))
    parser.add_argument('--output', type=Path, default=Path('artifacts/clearance-transfer'))
    args = parser.parse_args()
    result = audit(json.loads(args.measurements.read_text()))
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'camera-anchored-road.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    lines = ['# Camera-anchored initial road calibration', '',
             'Under the stated metric accurate-road-map assumption, the known first-camera map anchor closes the road datum at frame zero. Transform the measured first LiDAR plane through the calibrated camera/LiDAR relative pose. No Gaussian fitting or ENU target height is needed.', '',
             'This establishes a local plane. Later measurements below independently diagnose whether extrapolating it is justified, using recorded relative sensor poses only for that diagnostic.', '',
             '| Scene | Native map normal | Offset d (m) | LiDAR normal clearance (m) | First camera vertical clearance (m) |',
             '|---|---|---:|---:|---:|']
    for row in result['initial_planes']:
        if row['accepted']:
            lines.append(f"| {row['run_id']} | " + ', '.join(f'{v:+.5f}' for v in row['normal']) + f" | {row['offset_m']:.3f} | {row['lidar_normal_clearance_m']:.3f} | {row['first_camera_vertical_clearance_m']:.3f} |")
        else:
            lines.append(f"| {row['run_id']} | Rejected: {row['reason']} | — | — | — |")
    lines += ['', '## nuScenes 152 initial-plane stationarity', '',
              '| Frame | Distance from first LiDAR (m) | Initial-plane predicted clearance (m) | Current measured clearance (m) | Difference (m) | Normal difference (deg) |',
              '|---|---:|---:|---:|---:|---:|']
    for row in result['later_stationarity']:
        if row['accepted']:
            fields=['horizontal_distance_from_initial_lidar_m','initial_plane_predicted_vertical_clearance_m','current_plane_measured_vertical_clearance_m','clearance_prediction_minus_measurement_m','normal_disagreement_deg']
            lines.append(f"| {row['frame_offset']} | " + ' | '.join(f'{row[k]:.3f}' for k in fields) + ' |')
        else:
            lines.append(f"| {row['frame_offset']} | Rejected: {row['reason']} | — | — | — | — |")
    lines += ['', 'Positive difference means the newly measured local road is above the extrapolated initial plane at the current sensor footprint. Differences combine real road shape, sensor/pose error, and plane-fit error; they are not independent ground-truth map errors. Rejected source plane fits remain rejected. No registration gates or replay outputs are changed.', '']
    (args.output / 'camera-anchored-road.md').write_text('\n'.join(lines))


if __name__ == '__main__':
    main()

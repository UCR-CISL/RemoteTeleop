#!/usr/bin/env python3
"""Measure and transfer LiDAR-only sensor clearances without GSplat geometry."""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.investigation.first_frame_xyz import source_points
from src.localization.hybrid_road_height import HybridRoadHeightEstimator


MEASURES = ('lidar_normal_clearance_m', 'lidar_vertical_clearance_m',
            'camera_normal_clearance_m', 'camera_vertical_clearance_m')


def measure(metadata: dict, frame_offset: int, provenance: list[str]) -> dict:
    dataset = metadata['dataset']
    run_id = f"nuscenes-{metadata['scene']}" if dataset == 'nuscenes' else f"cooperscene-{metadata['scene']}-{metadata['agent']}"
    record = dict(run_id=run_id, frame_offset=frame_offset, dataset=dataset,
                  scene=str(metadata['scene']), agent=str(metadata['agent']),
                  frame_id=metadata['frame_id'], source_npz=metadata['npz_path'],
                  provenance=provenance, accepted=False, reference_height_used=False,
                  gsplat_used=False,
                  static_mask='Recorded vehicle/pedestrian/animal boxes padded 0.2 m, including parked vehicles; not online segmentation.')
    estimator = HybridRoadHeightEstimator()
    c = estimator.config
    record['source_fit_config'] = {key: value for key, value in asdict(c).items()
                                  if key.startswith('lidar_') or key in ('min_radius_m', 'max_radius_m', 'max_tilt_deg', 'min_span_m', 'voxel_m', 'max_fit_points')}
    # Explicit allowlist excludes all stored map/reference alignment matrices.
    with np.load(metadata['npz_path']) as cached:
        keys = ['points_lidar', 'dataset_T_lidar', 'dataset_T_camera',
                'boxes_T_dataset_object', 'boxes_dimensions_lwh', 'box_labels']
        frame = {key: cached[key] for key in keys if key in cached}
    sensor = frame['dataset_T_lidar']
    local_dataset = source_points(frame, mask_dynamic=True) - sensor[:3, 3]
    local_dataset = local_dataset[np.isfinite(local_dataset).all(axis=1)]
    radius = np.linalg.norm(local_dataset[:, :2], axis=1)
    candidates = local_dataset[(radius >= c.min_radius_m) & (radius <= c.max_radius_m)
                               & (local_dataset[:, 2] <= -c.lidar_below_sensor_m[0])
                               & (local_dataset[:, 2] >= -c.lidar_below_sensor_m[1])]
    candidates = estimator._balanced(candidates)
    record.update(raw_point_count=len(frame['points_lidar']), static_point_count=len(local_dataset),
                  lidar_candidate_count=len(candidates))
    try:
        plane, quality, _ = estimator._plane(candidates, lidar=True)
    except ValueError as error:
        record.update(reason='insufficient_lidar_ground', detail=str(error))
        return record
    normal = np.asarray(plane.normal)
    lidar_T_camera = np.linalg.inv(sensor) @ frame['dataset_T_camera']
    camera_lever_dataset = sensor[:3, :3] @ lidar_T_camera[:3, 3]
    camera_normal_clearance = float(plane.offset_m + normal @ camera_lever_dataset)
    record.update(accepted=True, reason='lidar_ground_fit_passed', fit=quality,
                  lidar_plane_dataset_up=plane.to_dict(),
                  lidar_plane_sensor=dict(normal=(sensor[:3, :3].T @ normal).tolist(), offset_m=plane.offset_m),
                  lidar_T_camera=lidar_T_camera.tolist(),
                  lidar_normal_clearance_m=plane.offset_m,
                  lidar_vertical_clearance_m=plane.offset_m / normal[2],
                  camera_normal_clearance_m=camera_normal_clearance,
                  camera_vertical_clearance_m=camera_normal_clearance / normal[2],
                  camera_lever_normal_component_m=float(normal @ camera_lever_dataset),
                  camera_optical_forward_dot_ground_normal=float(normal @ frame['dataset_T_camera'][:3, 2]),
                  camera_down_dot_ground_normal=float(normal @ frame['dataset_T_camera'][:3, 1]))
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--first-inputs', type=Path, default=Path('/tmp/RemoteTeleop-first-frame-localization/artifacts/first-frame-localization/inputs'))
    parser.add_argument('--pilot-inputs', type=Path, default=Path('artifacts/hybrid-road-height/pilot-v3/inputs'))
    parser.add_argument('--output', type=Path, default=Path('artifacts/clearance-transfer'))
    args = parser.parse_args()
    jobs = {}
    for path in sorted(args.first_inputs.glob('*/metadata.json')):
        metadata = json.loads(path.read_text())
        jobs[(metadata['dataset'], str(metadata['scene']), str(metadata['agent']), 0)] = (metadata, ['first_frame'])
    for path in sorted(args.pilot_inputs.glob('*.json')):
        metadata = json.loads(path.read_text())
        offset = int(metadata.get('frame_offset', metadata.get('sequence', 0)))
        key = (metadata['dataset'], str(metadata['scene']), str(metadata['agent']), offset)
        if key in jobs:
            jobs[key][1].append('pilot')
        else:
            jobs[key] = (metadata, ['pilot'])
    results = [measure(metadata, key[-1], provenance) for key, (metadata, provenance) in sorted(jobs.items())]
    first = {r['run_id']: r for r in results if r['frame_offset'] == 0}
    for record in results:
        nominal_id = 'nuscenes-152' if record['dataset'] == 'nuscenes' else f"cooperscene-1-{record['agent']}"
        nominal = first[nominal_id]
        record['nominal_run_id'] = nominal_id
        record['nominal_frame_offset'] = 0
        record['transfer_group'] = ('calibration' if record['run_id'] == nominal_id and record['frame_offset'] == 0
                                    else 'first_frame_transfer' if record['frame_offset'] == 0 else 'held_out_frame')
        record['transfer_available'] = record['accepted'] and nominal['accepted']
        if record['transfer_available']:
            for metric in MEASURES:
                # Positive means the new scene/frame measures greater clearance.
                difference = record[metric] - nominal[metric]
                record['nominal_' + metric] = nominal[metric]
                record['transfer_difference_' + metric] = difference
                record['within_01_' + metric] = bool(abs(difference) < .1)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'measurements.json').write_text(json.dumps(results, indent=2) + '\n')
    fields = ['run_id', 'frame_offset', 'frame_id', 'dataset', 'scene', 'agent', 'transfer_group',
              'accepted', 'reason', 'nominal_run_id', 'transfer_available', *MEASURES,
              *['nominal_' + name for name in MEASURES],
              *['transfer_difference_' + name for name in MEASURES],
              *['within_01_' + name for name in MEASURES]]
    with (args.output / 'measurements.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader(); writer.writerows(results)
    summary = dict(unique_frames=len(results), successful_ground_fits=sum(r['accepted'] for r in results),
                   first_frames=sum(r['frame_offset'] == 0 for r in results),
                   nonzero_pilot_frames=sum(r['frame_offset'] != 0 for r in results), groups=[])
    for dataset in ('nuscenes', 'cooperscene'):
        for group in ('first_frame_transfer', 'held_out_frame'):
            subset = [r for r in results if r['dataset'] == dataset and r['transfer_group'] == group]
            available = [r for r in subset if r['transfer_available']]
            entry = dict(dataset=dataset, group=group, frames=len(subset), available=len(available), metrics={})
            for metric in MEASURES:
                differences = np.array([r['transfer_difference_' + metric] for r in available])
                entry['metrics'][metric] = dict(within_01=int(np.count_nonzero(abs(differences) < .1)),
                    min_signed_difference_m=float(differences.min()) if len(differences) else None,
                    max_signed_difference_m=float(differences.max()) if len(differences) else None,
                    median_absolute_difference_m=float(np.median(abs(differences))) if len(differences) else None)
            summary['groups'].append(entry)
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()

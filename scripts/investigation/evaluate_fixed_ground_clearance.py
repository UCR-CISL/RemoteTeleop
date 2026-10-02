"""Conditional clearance transfer onto the assumed Riverside native Z=-2.8 plane.

This offline experiment does not establish a physical map road surface. Frozen
BEV horizontal placement and rotation are preserved; reference poses are used
only after each prediction is saved. No localization or replay is rerun.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from scripts.investigation.evaluate_hybrid_road_height import HybridComparison
from scripts.investigation.first_frame_xyz import pose_error, registration_config, source_points
from src.localization.registration import PointCloudRegistrar
import numpy as np


class FixedGroundClearanceComparison(HybridComparison):
    """Evaluate two fixed clearance hypotheses with the original replay gates."""

    GROUND_Z = -2.8

    def run_clearance(self, measurement):
        run_id = measurement['run_id']
        metadata = self.frame(run_id, 0)
        frame = dict(np.load(metadata['npz_path'], allow_pickle=False))
        bev, bev_origin = self.frozen_transform(run_id, 'bev', frame)
        xyz, xyz_origin = self.frozen_transform(run_id, 'xyz', frame)
        pose = bev @ frame['dataset_T_lidar']
        up = bev[:3, :3] @ np.array([0., 0., 1.])
        proposals = {}
        for name, clearance, supported in (
                ('measured', measurement.get('lidar_normal_clearance_m'), measurement['accepted']),
                ('transferred', measurement.get('nominal_lidar_normal_clearance_m'),
                 measurement.get('nominal_lidar_normal_clearance_m') is not None)):
            valid = bool(supported and clearance is not None and np.isfinite(clearance)
                         and clearance > 0 and abs(up[2]) > 1e-6)
            alpha = (float(clearance) - (pose[2, 3] - self.GROUND_Z)) / up[2] if valid else 0.
            transform = bev.copy()
            transform[:3, 3] += alpha * up
            projected_shift = bev[:3, :3].T @ (transform[:3, 3] - bev[:3, 3])
            assert np.linalg.norm(projected_shift[:2]) < 1e-8
            assert np.array_equal(transform[:3, :3], bev[:3, :3])
            height = float((transform @ frame['dataset_T_lidar'])[2, 3] - self.GROUND_Z)
            if valid:
                assert abs(height - clearance) < 1e-8
            proposals[name] = dict(valid_clearance=valid, clearance_m=clearance,
                source=run_id if name == 'measured' else measurement.get('nominal_run_id'),
                reason='conditional_flat_plane_proposal' if valid else 'no_valid_source_clearance',
                shift_along_bev_up_m=alpha, native_map_translation_m=alpha * up,
                frozen_bev_horizontal_shift_m=float(np.linalg.norm(projected_shift[:2])),
                resulting_native_plane_clearance_m=height, map_T_dataset=transform)
        prediction = dict(run_id=run_id, frame_offset=0, metadata=metadata,
            assumption='Native map Z=-2.8 is assumed flat road; not independently verified physical terrain',
            normal_clearance_transfer='Measured perpendicular clearance is assigned as native horizontal-plane clearance; differing road slopes are not corrected',
            plane=dict(normal=[0., 0., 1.], offset_m=-self.GROUND_Z), bev_up=up,
            formula='alpha=(clearance-(native_Z_lidar+2.8))/BEV_up_Z',
            proposals=proposals, source_measurement=measurement,
            reference_used_for_proposals=False, rendering_only_object_grounding_applied=False)
        self.write(self.output / 'predictions' / f'{run_id}.json', prediction)

        # Evaluation starts after both proposals are persisted. The original
        # initial transform, source mask, map filter, and thresholds are retained.
        self.load_map(metadata['map_path'])
        registrar = PointCloudRegistrar(registration_config(metadata['dataset']))
        source = source_points(frame, mask_dynamic=True)
        methods = {}
        transforms = [('xyz', xyz), ('bev', bev)] + [
            (name + '_candidate', item['map_T_dataset']) for name, item in proposals.items()]
        for name, transform in transforms:
            registration = registrar.evaluate(source, self.full_target, transform,
                frame['pipeline_initial_map_T_dataset'])
            methods[name] = dict(map_T_dataset=transform,
                map_T_lidar=transform @ frame['dataset_T_lidar'],
                map_T_camera=transform @ frame['dataset_T_camera'],
                error=pose_error(transform, frame['reference_map_T_dataset'], frame['dataset_T_lidar']),
                registration=asdict(registration), accepted=registration.accepted)
        for name, proposal in proposals.items():
            accepted = bool(proposal['valid_clearance'] and methods[name + '_candidate']['accepted'])
            methods[name] = dict(methods[name + '_candidate'] if accepted else methods['bev'],
                accepted=accepted, status='accepted' if accepted else 'fallback_unchanged_bev',
                reason='all_gates_passed' if accepted else proposal['reason'] if not proposal['valid_clearance']
                else 'original_registration_gates_failed')
        result = dict(prediction, methods=methods, bev_transform_source=bev_origin,
            xyz_transform_source=xyz_origin, registration_config=asdict(registrar.config),
            registration_source_mask='Original CooperScene baseline: annotation-masked',
            reference_used_after_prediction_only=True,
            historical_reference_qualification_used_for_acceptance=False)
        self.write(self.output / 'results' / f'{run_id}.json', result)
        print(json.dumps(dict(run_id=run_id, bev_z=methods['bev']['error']['dz_m'],
            measured_z=methods['measured_candidate']['error']['dz_m'],
            transferred_z=methods['transferred_candidate']['error']['dz_m'],
            measured_accepted=methods['measured']['accepted'],
            transferred_accepted=methods['transferred']['accepted'])), flush=True)
        return result

    def report(self, results):
        lines = ['# Fixed-ground clearance transfer', '',
            'Conditional experiment: native Riverside Z=-2.8 is assumed to be a flat road. '
            'This is the documented rendering plane, not a verified physical road surface. '
            'Measured LiDAR normal clearance is assigned to that horizontal plane; slope differences are not corrected. '
            'All inputs are first frames. nuScenes is excluded because no documented common road level exists.', '',
            'Proposals preserve frozen BEV horizontal translation and rotation. Reference errors are evaluation only, '
            'not independent ground truth. Original registration gates are unchanged. '
            'Rejected or unavailable proposals retain the frozen BEV pose; fallback does not imply that BEV passed registration.', '',
            '| Take/agent | XYZ Z / XYZ (m) | BEV Z / XYZ (m) | Measured proposal Z / XYZ (m) | Gate | Transferred proposal Z / XYZ (m) | Gate |',
            '|---|---:|---:|---:|---|---:|---|']
        for result in results:
            methods = result['methods']
            def error(name):
                e = methods[name]['error']
                return f"{e['dz_m']:+.3f} / {e['xyz_m']:.3f}"
            def status(name):
                return 'PASS' if methods[name]['accepted'] else methods[name]['reason']
            lines.append(f"| {result['run_id'].removeprefix('cooperscene-')} | {error('xyz')} | {error('bev')} | "
                         f"{error('measured_candidate')} | {status('measured')} | {error('transferred_candidate')} | {status('transferred')} |")
        lines += ['', 'Each result JSON includes signed XYZ, horizontal/XYZ error, orientation disagreement, residuals, '
                  'inlier statistics, rejection reasons, the candidate transform, and the gated fallback transform.', '']
        lines += ['## Aggregate comparison', '',
                  '| Method | Accepted proposals / 15 | Median absolute reference Z (m) | Median reference XYZ (m) | Reference XYZ <0.1 m |',
                  '|---|---:|---:|---:|---:|']
        for name in ('xyz', 'bev', 'measured_candidate', 'transferred_candidate', 'measured', 'transferred'):
            methods = [row['methods'][name] for row in results]
            lines.append(f"| {name} | {sum(m['accepted'] for m in methods)} | "
                f"{np.median([abs(m['error']['dz_m']) for m in methods]):.3f} | "
                f"{np.median([m['error']['xyz_m'] for m in methods]):.3f} | "
                f"{sum(m['error']['xyz_m'] < .1 for m in methods)} |")
        lines += ['', 'Candidate rows include rejected hypothetical placements. Measured/transferred rows use unchanged BEV '
                  'fallback whenever proposal gates reject. Preservation of horizontal placement is defined in the frozen BEV axes; '
                  'native map XY and reference-axis XY can change when their up directions differ.', '']
        (self.output / 'report.md').write_text('\n'.join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--previous', type=Path, default=Path('/tmp/RemoteTeleop-first-frame-localization/artifacts'))
    parser.add_argument('--measurements', type=Path, default=ROOT / 'artifacts/clearance-transfer/measurements.json')
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts/clearance-transfer/placement')
    args = parser.parse_args()
    measurements = json.loads(args.measurements.read_text())
    selected = {row['run_id']: row for row in measurements
                if row['dataset'] == 'cooperscene' and row['frame_offset'] == 0}
    expected = [f'cooperscene-{take}-{agent}' for take in range(1, 6) for agent in range(1, 4)]
    missing = set(expected) - selected.keys()
    if missing:
        raise ValueError(f'Missing fixed-frame clearance measurements: {sorted(missing)}')
    for measurement in selected.values():
        nominal = selected[measurement['nominal_run_id']]
        measurement['nominal_lidar_normal_clearance_m'] = (
            nominal['lidar_normal_clearance_m'] if nominal['accepted'] else None)
    runner = FixedGroundClearanceComparison(args.previous, args.output)
    results = [runner.run_clearance(selected[run_id]) for run_id in expected]
    runner.write(args.output / 'summary.json', results)
    runner.report(results)


if __name__ == '__main__':
    main()

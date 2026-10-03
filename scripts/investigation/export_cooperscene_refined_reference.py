"""Reproduce the superseded metadata-as-raw anchor diagnostic for auditability.

WARNING: archive attitude already includes left E calibration. This historical
hypothesis does not undo it. Use export_cooperscene_raw_normalized_reference.py
for the corrected, explicitly conditional sensor/marker hypothesis.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import zipfile

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from src.localization.cooperscene_reference import CooperSceneReference
from audit_cooperscene_release import matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit-root', type=Path, default=Path(__file__).resolve().parents[2] / 'artifacts/reference-refinement/release-audit')
    parser.add_argument('--physical-references', type=Path, default=Path('/tmp/RemoteTeleop-reference-orientation/artifacts/reference-orientation/axes-audit/physical-references.json'))
    parser.add_argument('--native-root', type=Path, default=Path('/mnt/bcc-data/websites/data.ucr.edu-datasets-cooperscene'))
    args = parser.parse_args()
    audit = json.loads((args.audit_root / 'release-audit.json').read_text())
    physical = json.loads(args.physical_references.read_text())
    w = np.asarray(physical['map_T_enu'])
    profiles, records, checks = {}, [], {}
    original_records = {}
    for row in physical['records']:
        original_records.setdefault(row['run_id'], []).append(row)
    for take in audit['takes']:
        take_id = take['take']
        frame0 = take['selected_available_frame']
        archive_path = args.native_root / ('test' if take_id == 2 else 'train') / f'{take_id}.zip'
        with zipfile.ZipFile(archive_path) as archive:
            for agent in ('1', '2', '3'):
                run_id = f'cooperscene-{take_id}-{agent}'
                metadata_bytes = archive.read(f'{take_id}/{agent}/metadata.json')
                metadata = json.loads(metadata_bytes)
                e = np.asarray(physical['calibrations'][agent][f'tower{agent}-xsens,tower{agent}-lidar'])
                sensor = CooperSceneReference(tuple(physical['origin_lla']), np.eye(4), e)
                p0, b0 = sensor.map_T_lidar(metadata[frame0]), matrix(metadata[frame0])
                c = np.asarray(take['corrections'][agent]['C'])
                k = c @ b0 @ np.linalg.inv(p0)
                anchor_error = float(np.max(np.abs(k @ p0 - c @ b0)))
                assert anchor_error < 1e-9, (run_id, anchor_error)
                profile = dict(anchor_frame_id=frame0, anchor_method='reproduced_original_selected_frame_ICP',
                    anchor_agreement_max_abs=anchor_error, C_original=c.tolist(), B_anchor=b0.tolist(),
                    physical_ENU_T_lidar_anchor=p0.tolist(), K_refined_world_T_physical_ENU=k.tolist(),
                    map_T_enu=(w @ k).tolist(), lidar_T_xsens=e.tolist(), origin_lla=physical['origin_lla'],
                    vehicle_metadata_source=f'{archive_path}:{take_id}/{agent}/metadata.json',
                    vehicle_metadata_sha256=hashlib.sha256(metadata_bytes).hexdigest(),
                    infrastructure_pcd_required_for_use=False)
                profiles[run_id] = profile
                repaired = CooperSceneReference(tuple(physical['origin_lla']), w @ k, e)
                distances, angles, physical_errors = [], [], []
                for old in original_records[run_id]:
                    m = metadata[old['frame_id']]
                    pose = repaired.map_T_lidar(m)
                    native = c @ matrix(m)
                    released_map = w @ native
                    physical_errors.append(float(np.max(np.abs(w @ sensor.map_T_lidar(m) - np.asarray(old['map_T_lidar'])))))
                    distances.append(float(np.linalg.norm(pose[:3,3]-released_map[:3,3])))
                    angles.append(float(np.degrees(Rotation.from_matrix(pose[:3,:3] @ released_map[:3,:3].T).magnitude())))
                    records.append({key: value for key, value in old.items() if key not in ('map_T_lidar', 'map_T_dataset')} |
                        dict(map_T_lidar=pose.tolist(), map_T_dataset=(pose @ np.linalg.inv(native)).tolist()))
                assert max(physical_errors) < 1e-9, (run_id, max(physical_errors))
                checks[run_id] = dict(anchor_frame_id=frame0, anchor_max_abs=anchor_error, frames=len(distances),
                    physical_baseline_reconstruction_max_abs=max(physical_errors),
                    versus_released_position_median_m=float(np.median(distances)),
                    versus_released_position_max_m=max(distances),
                    versus_released_rotation_median_deg=float(np.median(angles)),
                    versus_released_rotation_max_deg=max(angles))
                print(run_id, json.dumps(checks[run_id]), flush=True)
    shared = dict(reference_kind='released_ICP_anchor_with_repaired_sensor_relative_motion_candidate',
        composition='W @ K_a @ P_a(t); K_a = C_a @ B_a(t0) @ inverse(P_a(t0)); P_a = Pose(R_INS,p_GNSS_ENU) @ inverse(lidar_T_xsens)',
        candidate_not_ground_truth=True, original_map_T_enu=w.tolist(),
        physical_reference_source=str(args.physical_references),
        physical_reference_sha256=hashlib.sha256(args.physical_references.read_bytes()).hexdigest(),
        caveats=['Retains original infrastructure anchor, whose independent survey provenance is unverified.',
                 'Preserves released refined pose exactly at the proven ICP fitting frame; changes motion away from it.',
                 'Requires heldout geometric validation; candidate is not a demonstrated improvement.',
                 'Raw GNSS origin, altitude datum, capture-time skew and scan deskew qualifications remain.',
                 'No Gaussian-map geometry or localization output was used to generate this candidate.',
                 'Applying profiles requires vehicle metadata and calibration, not infrastructure point clouds.'])
    (args.audit_root/'refinement-anchor-profiles.json').write_text(json.dumps(shared | dict(profiles=profiles,checks=checks),indent=2)+'\n')
    (args.audit_root/'anchor-repaired-references.json').write_text(json.dumps(shared | dict(profiles=profiles,checks=checks,records=records),indent=2)+'\n')
    print('Exported',len(records),'candidate poses')


if __name__ == '__main__':
    main()

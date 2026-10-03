"""Export conditional sensor references after undoing metadata attitude processing.

Original artifacts are preserved. MoCap marker/sensor alignment remains assumed.
"""
import json
import hashlib
from pathlib import Path
import sys
import zipfile

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from src.localization.cooperscene_reference import CooperSceneReference
from audit_cooperscene_release import matrix

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / 'artifacts/reference-refinement/release-audit'
PHYSICAL = Path('/tmp/RemoteTeleop-reference-orientation/artifacts/reference-orientation/axes-audit/physical-references.json')
NATIVE = Path('/mnt/bcc-data/websites/data.ucr.edu-datasets-cooperscene')


def normalized_metadata(metadata, calibration):
    """Recover R_original_INS @ F_agent by removing the proven left E rotation."""
    r = calibration[:3,:3].T @ matrix(metadata)[:3,:3]
    q = Rotation.from_matrix(r).as_quat()
    result = dict(metadata)
    result.update({f'Quat_q{k}': float(v) for k,v in zip(('x','y','z','w'), q)})
    return result


def main():
    grid = json.loads(PHYSICAL.read_text())
    audit = json.loads((OUTPUT/'release-audit.json').read_text())
    w = np.asarray(grid['map_T_enu'])
    grouped = {}
    for row in grid['records']:
        grouped.setdefault(row['run_id'],[]).append(row)
    baselines, anchored, ins_axes, profiles, checks = [], [], [], {}, {}
    for take in audit['takes']:
        t=take['take']; anchor=take['selected_available_frame']
        archive_path=NATIVE/('test' if t==2 else 'train')/f'{t}.zip'
        with zipfile.ZipFile(archive_path) as archive:
            for a in ('1','2','3'):
                run=f'cooperscene-{t}-{a}'
                metadata=json.loads(archive.read(f'{t}/{a}/metadata.json'))
                e=np.asarray(grid['calibrations'][a][f'tower{a}-xsens,tower{a}-lidar'])
                sensor=CooperSceneReference(tuple(grid['origin_lla']),np.eye(4),e)
                axis=CooperSceneReference(tuple(grid['origin_lla']),np.eye(4),np.eye(4))
                p0=sensor.map_T_lidar_from_release_metadata(metadata[anchor])
                b0=matrix(metadata[anchor]);c=np.asarray(take['corrections'][a]['C'])
                k=c@b0@np.linalg.inv(p0)
                anchor_error=float(np.max(abs(k@p0-c@b0)))
                assert anchor_error<1e-9
                profiles[run]=dict(anchor_frame_id=anchor,map_T_enu=(w@k).tolist(),lidar_T_xsens=e.tolist(),
                    K=k.tolist(),origin_lla=grid['origin_lla'],C_original=c.tolist(),B_anchor=b0.tolist(),
                    conditional_sensor_pose_anchor=p0.tolist(),undo_metadata_left_E_rotation_required=True,
                    infrastructure_pcd_required_for_use=False,anchor_max_abs=anchor_error)
                baseline_diff=[];anchor_diff=[]
                for old in grouped[run]:
                    m=metadata[old['frame_id']];normalized=normalized_metadata(m,e)
                    p=sensor.map_T_lidar_from_release_metadata(m)
                    baseline=w@p;refined=w@k@p;axes=w@axis.map_T_lidar(normalized)
                    n=c@matrix(m)
                    common={key:value for key,value in old.items() if key not in ('map_T_lidar','map_T_dataset')}
                    for collection,pose in ((baselines,baseline),(anchored,refined),(ins_axes,axes)):
                        collection.append(common|dict(map_T_lidar=pose.tolist(),map_T_dataset=(pose@np.linalg.inv(n)).tolist()))
                    baseline_diff.append(float(np.linalg.norm(baseline[:3,3]-(w@n)[:3,3])))
                    anchor_diff.append(float(np.linalg.norm(refined[:3,3]-(w@n)[:3,3])))
                checks[run]=dict(anchor_max_abs=anchor_error,frames=len(grouped[run]),
                    baseline_vs_released_median_m=float(np.median(baseline_diff)),
                    anchored_vs_released_median_m=float(np.median(anchor_diff)),
                    anchored_vs_released_max_m=max(anchor_diff))
                print(run,json.dumps(checks[run]),flush=True)
    shared=dict(candidate_not_ground_truth=True,origin_lla=grid['origin_lla'],map_T_enu=w.tolist(),
        metadata_attitude_undo='R_normalized_INS = transpose(E.rotation) @ R_metadata = R_original_INS @ F_agent; F1=Rx180,F2=F3=I',
        calibration_semantics='Conditional MoCap marker-to-marker transform interpreted as normalized INS sensor to LiDAR sensor mounting calibration.',
        caveats=['Original MoCap recording and marker-to-physical-sensor calibration are unavailable.',
                 'GNSS antenna/output origin and altitude datum remain unverified.',
                 'GNSS timestamps are used; no capture-time interpolation or deskew.',
                 'Anchor version preserves infrastructure-based original refinement at fitting frame only.',
                 'No Gaussian map or evaluation errors used for reference generation.'],
        supersedes_interpretation_of_earlier_physical_reference=True,
        original_frame_grid_path=str(PHYSICAL),original_frame_grid_sha256=hashlib.sha256(PHYSICAL.read_bytes()).hexdigest(),
        raw_attitude_proof=str(OUTPUT/'raw-attitude-audit.json'))
    for filename,kind,composition,rows in [
        ('raw-normalized-references.json','conditional_raw_normalized_INS_with_MoCap_sensor_extrinsic','W @ Pose(E.R.T @ R_metadata,p_raw) @ inverse(E)',baselines),
        ('raw-normalized-anchor-references.json','conditional_raw_normalized_INS_preserving_original_ICP_anchor','W @ K @ P(t); K=C B(t0) inverse(P(t0)); P=Pose(E.R.T @ R_metadata,p_raw) inverse(E)',anchored),
        ('normalized-ins-axis-references.json','raw_normalized_INS_axes_without_sensor_extrinsic','W @ Pose(E.R.T @ R_metadata,p_raw)',ins_axes)]:
        (OUTPUT/filename).write_text(json.dumps(shared|dict(reference_kind=kind,composition=composition,profiles=profiles,checks=checks,records=rows),indent=2)+'\n')
    (OUTPUT/'raw-normalized-anchor-profiles.json').write_text(json.dumps(shared|dict(profiles=profiles,checks=checks),indent=2)+'\n')
    print('Exported',len(baselines),'poses for each of three explicitly conditional references')


if __name__=='__main__':
    main()

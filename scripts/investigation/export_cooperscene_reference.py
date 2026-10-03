"""Export offline physical LiDAR references from raw GNSS/INS and calibration."""
import json
import argparse
import hashlib
import pickle
from pathlib import Path
import sys
import zipfile

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from src.localization.cooperscene_reference import CooperSceneReference
from src.localization.cooperscene_calibration import camera_T_lidar
from audit_cooperscene_reference import OUTPUT, REPO, NATIVE, ORIGIN, rigid

CALIBRATION_ROOT = Path('/mnt/bcc-data/data/Cooperscene/backup/calib_data/250928')
ORIGINAL_ROOT = Path('/mnt/bcc-data/data/Cooperscene/release/250928')


def stored_pose(metadata):
    pose = np.eye(4)
    pose[:3, 3] = [metadata[k] for k in ('enu_e', 'enu_n', 'enu_u')]
    pose[:3, :3] = Rotation.from_quat([metadata[k] for k in ('Quat_qx','Quat_qy','Quat_qz','Quat_qw')]).as_matrix()
    return pose


def main():
    audit = json.loads((OUTPUT/'audit.json').read_text())
    calibrations = {}
    for agent in ('1', '2', '3'):
        path = CALIBRATION_ROOT / f'tower{agent}-tfs.pkl'
        # These local calibration files were inspected with pickletools.dis;
        # they contain only dictionaries of two numpy float64 4x4 arrays.
        matrices = pickle.loads(path.read_bytes())
        calibrations[agent] = {key: value.tolist() for key, value in matrices.items()}
        calibrations[agent].update(source_path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    (OUTPUT/'sensor-extrinsics.json').write_text(json.dumps(calibrations,indent=2)+'\n')
    map_T_enu = np.asarray(json.loads((REPO/'config/cooperscene_riverside_iowa_enu_prior.json').read_text())['gaussian_T_cooperscene'])
    records, checks, sequence_checks = [], {}, {}
    for agent in ('1', '2', '3'):
        e = np.asarray(calibrations[agent][f'tower{agent}-xsens,tower{agent}-lidar'])
        rows = [r for r in audit['native_records'] if r['run_id'].endswith('-'+agent)]
        recovered = rigid([r['raw_gnss_enu'] for r in rows], [r['stored_enu'] for r in rows])
        checks[agent] = dict(
            stored_enu_T_raw_gnss_enu=np.asarray(recovered['transform']).tolist(),
            stored_enu_fit_matches_lidar_T_xsens_max_abs=float(np.max(np.abs(np.asarray(recovered['transform'])-e))),
            stored_enu_fit_position_max_m=recovered['residual_norm_max_m'],
            camera_extrinsic_matches_repo_max_abs=float(np.max(np.abs(np.asarray(calibrations[agent][f'tower{agent}-lidar,tower{agent}-camera'])-camera_T_lidar(agent)))))
    for take in range(1, 6):
        archive_path = NATIVE/('test' if take==2 else 'train')/f'{take}.zip'
        with zipfile.ZipFile(archive_path) as archive:
            for agent in ('1','2','3'):
                run_id = f'cooperscene-{take}-{agent}'
                e = np.asarray(calibrations[agent][f'tower{agent}-xsens,tower{agent}-lidar'])
                reference = CooperSceneReference(ORIGIN, map_T_enu, e)
                sampled = [r for r in audit['native_records'] if r['run_id']==run_id]
                # Recover the sequence's second export transform without map geometry.
                first = sampled[0]
                native_T_stored = np.asarray(first['native_matrix']) @ np.linalg.inv(stored_pose(first['raw_metadata']))
                icp_path = ORIGINAL_ROOT / str(take) / agent / 'icp_correction.json'
                icp = json.loads(icp_path.read_text())
                sequence_checks[run_id] = dict(native_T_stored=native_T_stored.tolist(),
                    native_reconstruction_max_abs=float(max(np.max(np.abs(native_T_stored@stored_pose(r['raw_metadata'])-r['native_matrix'])) for r in sampled)),
                    icp_source_path=str(icp_path),
                    icp_source_matrix_match_max_abs=float(max(np.max(np.abs(np.asarray(icp[r['frame_id']])-native_T_stored)) for r in sampled)))
                prefix = f'{take}/{agent}/'
                metadata = json.loads(archive.read(prefix+'metadata.json'))
                frame_ids = sorted(p[len(prefix):-5] for p in archive.namelist()
                                   if p.startswith(prefix) and p.endswith('.yaml') and p[len(prefix):-5].isdigit())
                for offset, frame_id in enumerate(frame_ids):
                    m = metadata[frame_id]
                    pose = reference.map_T_lidar(m)
                    native = np.asarray(icp[frame_id]) @ stored_pose(m)
                    records.append(dict(run_id=run_id, frame_offset=offset, frame_id=frame_id,
                        map_T_lidar=pose.tolist(), map_T_dataset=(pose@np.linalg.inv(native)).tolist(),
                        raw_gnss_timestamp_ns=m['gnss_ns'], lidar_timestamp_ns=m['lidar_ns'], image_timestamp_ns=m['image_ns']))
    payload = dict(reference_kind='raw_gnss_ins_with_sensor_extrinsic_at_gnss_timestamp',
        composition='map_T_lidar = map_T_ENU @ ENU_T_xsens(raw LLA, raw quaternion) @ inverse(lidar_T_xsens)',
        origin_lla=ORIGIN,map_T_enu=map_T_enu.tolist(),calibrations=calibrations,
        metadata_calibration_checks=checks,sequence_export_checks=sequence_checks,
        caveats=['Raw GNSS altitude datum is unspecified; existing numeric origin offset retained.',
                 'Assumes raw GNSS position represents Xsens origin; GNSS antenna lever-arm configuration unavailable.',
                 'Poses are at GNSS timestamps, without LiDAR capture-time interpolation or scan deskew.',
                 'This independent sensor-derived baseline does not certify absolute surveyed ground truth.'],records=records)
    (OUTPUT/'physical-references.json').write_text(json.dumps(payload,indent=2)+'\n')
    print('Exported',len(records),'physical references')
    print(json.dumps(checks,indent=2))
    print('max reconstructed native error',max(r['native_reconstruction_max_abs'] for r in sequence_checks.values()))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit-root', type=Path, default=OUTPUT)
    parser.add_argument('--native-root', type=Path, default=NATIVE)
    parser.add_argument('--calibration-root', type=Path, default=CALIBRATION_ROOT)
    parser.add_argument('--original-root', type=Path, default=ORIGINAL_ROOT)
    args = parser.parse_args()
    OUTPUT, NATIVE = args.audit_root, args.native_root
    CALIBRATION_ROOT, ORIGINAL_ROOT = args.calibration_root, args.original_root
    main()

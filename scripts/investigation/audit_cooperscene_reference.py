"""Read-only native pose, raw GNSS/INS and cache convention audit.

Run with RemoteTeleop's .venv/bin/python; all source paths are absolute.
Fits use raw GNSS positions, never Gaussian registration or evaluation errors.
"""
from pathlib import Path
import io
import argparse
import json
import sys
import zipfile

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
from src.localization.cooperscene import pose_to_matrix
from src.localization.cooperscene_calibration import camera_T_lidar

ATTEMPTS = Path('/tmp/RemoteTeleop-hybrid-road-height/artifacts/full-sequence-localization/attempts')
NATIVE = Path('/mnt/bcc-data/websites/data.ucr.edu-datasets-cooperscene')
OUTPUT = REPO / 'artifacts/reference-orientation/axes-audit'
ORIGIN = (33.975676, -117.339873, 273.0)


def ecef(lla):
    lat, lon = np.radians(lla[:2])
    height = lla[2]
    a, e2 = 6378137., 6.6943799901413165e-3
    n = a / np.sqrt(1 - e2 * np.sin(lat)**2)
    return np.array([(n + height)*np.cos(lat)*np.cos(lon),
                     (n + height)*np.cos(lat)*np.sin(lon),
                     (n*(1-e2) + height)*np.sin(lat)])


def enu(lla):
    lat, lon = np.radians(ORIGIN[:2])
    basis = np.array([[-np.sin(lon), np.cos(lon), 0],
                      [-np.sin(lat)*np.cos(lon), -np.sin(lat)*np.sin(lon), np.cos(lat)],
                      [np.cos(lat)*np.cos(lon), np.cos(lat)*np.sin(lon), np.sin(lat)]])
    return basis @ (ecef(lla) - ecef(ORIGIN))


def rigid(source, target):
    a, b = np.asarray(source), np.asarray(target)
    u, singular, vt = np.linalg.svd((a-a.mean(0)).T @ (b-b.mean(0)))
    r = vt.T @ np.diag([1, 1, np.linalg.det(vt.T @ u.T)]) @ u.T
    t = b.mean(0) - r @ a.mean(0)
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = r, t
    errors = a @ r.T + t - b
    return dict(transform=transform.tolist(), rpy_xyz_deg=Rotation.from_matrix(r).as_euler('xyz', degrees=True).tolist(),
                singular_values=singular.tolist(), residual_xyz_rmse_m=np.sqrt(np.mean(errors**2, axis=0)).tolist(),
                residual_norm_median_m=float(np.median(np.linalg.norm(errors, axis=1))),
                residual_norm_max_m=float(np.max(np.linalg.norm(errors, axis=1))))


def main():
    cache_checks, records = [], []
    for take in range(1, 6):
        archive_path = NATIVE / ('test' if take == 2 else 'train') / f'{take}.zip'
        with zipfile.ZipFile(archive_path) as archive:
            for agent in ('1', '2', '3'):
                run_id = f'cooperscene-{take}-{agent}'
                prefix = f'{take}/{agent}/'
                frame_ids = sorted(p[len(prefix):-5] for p in archive.namelist()
                                   if p.startswith(prefix) and p.endswith('.yaml') and p[len(prefix):-5].isdigit())
                metadata = json.loads(archive.read(prefix + 'metadata.json'))
                selected = [frame_ids[i] for i in np.unique(np.linspace(0, len(frame_ids)-1, 21).astype(int))]
                for frame_id in selected:
                    annotation = yaml.safe_load(archive.read(prefix + frame_id + '.yaml'))
                    native_pose = pose_to_matrix(annotation['lidar_pose'])
                    m = metadata[frame_id]
                    raw_r = Rotation.from_quat([m[k] for k in ('Quat_qx','Quat_qy','Quat_qz','Quat_qw')]).as_matrix()
                    raw_enu = enu([m[k] for k in ('Latitude','Longitude','Altitude')])
                    row = dict(run_id=run_id, frame_id=frame_id, archive=str(archive_path),
                               yaml_pose=annotation['lidar_pose'], native_matrix=native_pose.tolist(),
                               raw_gnss_enu=raw_enu.tolist(), stored_enu=[m[k] for k in ('enu_e','enu_n','enu_u')],
                               raw_ins_rotation=raw_r.tolist(), raw_metadata=m,
                               world_delta_rpy_deg=Rotation.from_matrix(native_pose[:3,:3] @ raw_r.T).as_euler('xyz',degrees=True).tolist(),
                               body_delta_rpy_deg=Rotation.from_matrix(raw_r.T @ native_pose[:3,:3]).as_euler('xyz',degrees=True).tolist())
                    records.append(row)
                result = json.loads((ATTEMPTS/run_id/'f000000.json').read_text())
                meta = result['metadata']
                with np.load(meta['npz_path']) as cache:
                    annotation = yaml.safe_load(archive.read(prefix+meta['frame_id']+'.yaml'))
                    native_pose = pose_to_matrix(annotation['lidar_pose'])
                    pcd = archive.read(prefix+meta['frame_id']+'.pcd').split(b'DATA ascii\n',1)[1]
                    points = np.loadtxt(io.BytesIO(pcd),dtype=np.float32)
                    check = dict(run_id=run_id, frame_id=meta['frame_id'], npz_path=meta['npz_path'],
                        native_pose_max_abs_error=float(np.max(np.abs(native_pose-cache['dataset_T_lidar']))),
                        native_points_max_abs_error=float(np.max(np.abs(points-cache['points_lidar']))),
                        camera_T_lidar_max_abs_error=float(np.max(np.abs(np.linalg.inv(cache['dataset_T_camera'])@cache['dataset_T_lidar']-camera_T_lidar(agent)))),
                        determinant=float(np.linalg.det(native_pose[:3,:3])),
                        orthogonality_max_abs_error=float(np.max(np.abs(native_pose[:3,:3].T@native_pose[:3,:3]-np.eye(3)))),
                        reference_map_T_dataset=cache['reference_map_T_dataset'].tolist(),
                        inherited_map_T_dataset=cache['inherited_map_T_dataset'].tolist())
                    cache_checks.append(check)
        print(f'audited native take {take}', flush=True)
    groups = {'all': records}
    groups.update({f'take-{t}':[r for r in records if r['run_id'].split('-')[1]==str(t)] for t in range(1,6)})
    groups.update({f'agent-{a}':[r for r in records if r['run_id'].endswith('-'+a)] for a in ('1','2','3')})
    groups.update({run:[r for r in records if r['run_id']==run] for run in sorted({r['run_id'] for r in records})})
    fits = {}
    for name, rows in groups.items():
        fit = {}
        for field in ('raw_gnss_enu','stored_enu'):
            fit[field+'_T_native'] = rigid([np.asarray(r['native_matrix'])[:3,3] for r in rows], [r[field] for r in rows])
        fit['world_delta_rpy_mean_std_deg'] = [np.mean([r['world_delta_rpy_deg'] for r in rows],axis=0).tolist(),np.std([r['world_delta_rpy_deg'] for r in rows],axis=0).tolist()]
        fits[name] = fit
    nuscenes = []
    for path in sorted(ATTEMPTS.glob('nuscenes-*/f000000.json')):
        m = json.loads(path.read_text())['metadata']
        root = Path(m['source_path'])
        if not (root/'extrinsics/000_0.txt').exists():
            nuscenes.append(dict(run_id=m['run_id'], unavailable_source_path=str(root)))
            continue
        with np.load(m['npz_path']) as f:
            expected_reference = np.linalg.inv(np.loadtxt(root/'extrinsics/000_0.txt'))
            lidar = np.loadtxt(root/'lidar_pose'/f"{m['frame_id']}.txt")
            camera = np.loadtxt(root/'extrinsics'/f"{m['frame_id']}_0.txt")
            nuscenes.append(dict(run_id=m['run_id'], reference_max_abs_error=float(np.max(np.abs(expected_reference-f['reference_map_T_dataset']))),
                                 lidar_max_abs_error=float(np.max(np.abs(lidar-f['dataset_T_lidar']))),
                                 camera_max_abs_error=float(np.max(np.abs(camera-f['dataset_T_camera']))),
                                 map_up=expected_reference[:3,2].tolist()))
    payload = dict(gnss_origin_lla=ORIGIN, cache_checks=cache_checks, native_records=records, position_fits=fits, nuscenes_checks=nuscenes,
                   caveat='Raw GNSS position is INS/GNSS antenna/body position, not guaranteed LiDAR origin. Fits diagnose coordinate structure, not ground truth localization.')
    (OUTPUT/'audit.json').write_text(json.dumps(payload, indent=2)+'\n')
    print(json.dumps(fits['all'], indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--attempts-root', type=Path, default=ATTEMPTS)
    parser.add_argument('--native-root', type=Path, default=NATIVE)
    parser.add_argument('--output-root', type=Path, default=OUTPUT)
    args = parser.parse_args()
    ATTEMPTS, NATIVE, OUTPUT = args.attempts_root, args.native_root, args.output_root
    OUTPUT.mkdir(parents=True, exist_ok=True)
    main()

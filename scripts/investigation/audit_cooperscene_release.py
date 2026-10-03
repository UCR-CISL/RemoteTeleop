"""Read-only release provenance and optional selected-frame ICP reproduction.

No registration against the evaluation map. Writes only --output.
"""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

SOURCE = Path('/home/jyue86/Documents/CISL-Projects/SensorPlatform')
RELEASE = Path('/mnt/bcc-data/data/Cooperscene/release/250928')
FILES = ['icp/create_transformation_matrix_icp.py', 'icp/README.md',
         'data_formater/sync_generate_frames.py', 'data_formater/post_format.py',
         'utils/constants.py', 'utils/annotation_labeling.py', 'viz/mcap_writer.py']


def matrix(metadata):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat([metadata['Quat_' + k] for k in ('qx', 'qy', 'qz', 'qw')]).as_matrix()
    t[:3, 3] = [metadata['enu_' + k] for k in 'enu']
    return t


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[2] / 'artifacts/reference-refinement/release-audit')
    parser.add_argument('--takes', type=int, nargs='+', default=[1, 2, 3, 4, 5])
    parser.add_argument('--reproduce-icp', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    sha = subprocess.check_output(['git', '-C', str(SOURCE), 'rev-parse', '1ad727f'], text=True).strip()
    provenance = []
    for filename in FILES:
        content = subprocess.check_output(['git', '-C', str(SOURCE), 'show', f'{sha}:{filename}'])
        destination = args.output / 'source-snapshots' / filename
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(content)
        provenance.append(dict(repository=str(SOURCE), commit=sha, path=filename,
                               sha256=hashlib.sha256(content).hexdigest()))
    for revision, filename in [('659476d', 'viz/mcap_writer.py'), ('1144b95', 'late_fusion/gen_local_labels.py')]:
        commit = subprocess.check_output(['git', '-C', str(SOURCE), 'rev-parse', revision], text=True).strip()
        content = subprocess.check_output(['git', '-C', str(SOURCE), 'show', f'{commit}:{filename}'])
        destination = args.output / 'source-snapshots' / revision / filename
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(content)
        provenance.append(dict(repository=str(SOURCE), commit=commit, path=filename,
                               sha256=hashlib.sha256(content).hexdigest()))
    results = []
    for take in args.takes:
        root = RELEASE / str(take)
        metadata = {a: json.loads((root / a / 'metadata.json').read_text()) for a in '0123'}
        common = sorted(set.intersection(*(set(m) for m in metadata.values())), key=int)
        # Historical code lists agent-1 PCDs, not all metadata; retained files may
        # be a subset of those present before post_format cropped the release.
        available = [f for f in common if all((root/a/'sensor/lidar'/f'{f}.pcd').exists() for a in '0123')]
        def score(frame):
            points = [matrix(metadata[a][frame])[:3, 3] for a in '0123']
            return sum(np.linalg.norm(points[i]-points[j]) for i in range(4) for j in range(i+1, 4))
        selected = min(available, key=score)
        all_metadata_selected = min(common, key=score)
        base = {a: matrix(metadata[a][selected]) for a in '0123'}
        order = sorted('123', key=lambda a: np.linalg.norm(base[a][:3, 3]-base['0'][:3, 3]))
        result = dict(take=take, metadata_frames=len(common), available_frames=len(available),
                      selected_available_frame=selected, selected_all_metadata_frame=all_metadata_selected,
                      available_selection_score_m=float(score(selected)), agent_order=order, corrections={})
        for a in '123':
            path = root/a/'icp_correction.json'
            content = path.read_bytes()
            corrections = json.loads(content)
            arr = np.asarray(list(corrections.values()))
            result['corrections'][a] = dict(path=str(path), sha256=hashlib.sha256(content).hexdigest(),
                frames=len(arr), unique_matrices=len({json.dumps(v) for v in corrections.values()}),
                C=corrections[selected], rotation_xyz_deg=Rotation.from_matrix(arr[0,:3,:3]).as_euler('xyz',degrees=True).tolist(),
                translation_m=arr[0,:3,3].tolist())
        if args.reproduce_icp:
            import open3d as o3d
            clouds = {}
            for a in '0123':
                clouds[a] = o3d.io.read_point_cloud(str(root/a/'sensor/lidar'/f'{selected}.pcd'))
                clouds[a].transform(base[a])
            fused = clouds['0']
            checks = []
            for index, a in enumerate(order):
                src, target = (fused, clouds[a]) if index == 0 else (clouds[a], fused)
                reg = o3d.pipelines.registration.registration_icp(src, target, 3.0, np.eye(4),
                    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))
                calculated = np.linalg.inv(reg.transformation) if index == 0 else reg.transformation
                stored = np.asarray(result['corrections'][a]['C'])
                delta = calculated @ np.linalg.inv(stored)
                checks.append(dict(agent=a, fitness=reg.fitness, inlier_rmse=reg.inlier_rmse,
                    calculated_C=calculated.tolist(), matrix_max_abs_difference=float(np.max(abs(calculated-stored))),
                    stored_vs_calculated_angle_deg=float(np.degrees(Rotation.from_matrix(delta[:3,:3]).magnitude())),
                    stored_vs_calculated_translation_m=float(np.linalg.norm(delta[:3,3]))))
                clouds[a].transform(calculated)
                fused += clouds[a]
            result['icp_reproduction'] = checks
        results.append(result)
        print(json.dumps({k:v for k,v in result.items() if k != 'corrections'}), flush=True)
    (args.output/'release-audit.json').write_text(json.dumps(dict(source_provenance=provenance, takes=results,
        qualification='Numerical reproduction tests execution provenance. Source inputs are retained release metadata and point clouds; default reproduction uses 3 m, 200 iterations and current Open3D. No evaluation-map geometry is used.'),indent=2)+'\n')


if __name__ == '__main__':
    main()

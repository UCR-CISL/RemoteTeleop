"""Re-score unchanged saved poses against a GNSS-recovered evaluation reference.

The historical selected frames and method acceptance are preserved. This is a
reference audit, not a new localization evaluation or a new selection policy.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.investigation.first_frame_xyz import map_proxy, source_points


def disagreement(estimate, reference, sensor, axes):
    estimated_pose, reference_pose = estimate @ sensor, reference @ sensor
    delta = axes.T @ (estimated_pose[:3, 3] - reference_pose[:3, 3])
    rotation = axes.T @ (estimated_pose[:3, :3] @ reference_pose[:3, :3].T) @ axes
    rpy = Rotation.from_matrix(rotation).as_euler('xyz', degrees=True)
    return dict(dx_m=float(delta[0]), dy_m=float(delta[1]), dz_m=float(delta[2]),
                xy_m=float(np.linalg.norm(delta[:2])), xyz_m=float(np.linalg.norm(delta)),
                roll_deg=float(rpy[0]), pitch_deg=float(rpy[1]), yaw_deg=float(rpy[2]))


class ReferenceRescorer:
    def __init__(self, attempts, references, output):
        self.attempts, self.output = attempts, output
        output.mkdir(parents=True, exist_ok=True)
        self.reference_document = json.loads(references.read_text())
        self.physical = 'records' in self.reference_document
        self.references = ({(r['run_id'], r['frame_offset']): r for r in self.reference_document['records']}
                           if self.physical else self.reference_document['references'])
        self.reference_path = references.resolve()
        self.reference_hash = hashlib.sha256(references.read_bytes()).hexdigest()
        self.target = None
        self.map_path = None

    def score(self, run_id, offset, method, visualize=False):
        path = self.attempts/run_id/f'f{offset:06d}.json'
        record = json.loads(path.read_text())
        result = record['methods'].get(method, {})
        reference_key = (run_id, offset) if self.physical else run_id
        if run_id.startswith('cooperscene') and reference_key not in self.references:
            raise ValueError(f'Missing corrected CooperScene reference: {reference_key}')
        row = dict(run_id=run_id, frame_offset=offset, method=method,
                   original_accepted=result.get('accepted', False),
                   status='scored' if 'map_T_dataset' in result else 'no_saved_pose',
                   reference_changed=reference_key in self.references,
                   independent_ground_truth=False)
        if 'map_T_dataset' not in result:
            return row
        frame = dict(np.load(record['metadata']['npz_path'], allow_pickle=False))
        estimate = np.asarray(result['map_T_dataset'])
        original = frame['reference_map_T_dataset']
        recovered = np.asarray(self.references[reference_key]['map_T_dataset']) if reference_key in self.references else original
        # CooperScene map metadata explicitly declares ENU. Keep measurement axes
        # fixed; do not rotate reported vertical with the newly recovered dataset frame.
        axes = np.eye(3) if run_id.startswith('cooperscene') else original[:3, :3]
        row['error_axes'] = 'map_ENU' if run_id.startswith('cooperscene') else 'nuscenes_global'
        for label, reference in [('original', original), ('recovered', recovered)]:
            row.update({label+'_'+k: v for k,v in disagreement(estimate, reference, frame['dataset_T_lidar'], axes).items()})
        row['attempt_path'] = str(path)
        if visualize:
            self.plot(record, frame, estimate, original, recovered, row)
        return row

    def plot(self, record, frame, estimate, original, recovered, row):
        md = record['metadata']
        if self.map_path != md['map_path']:
            self.target = map_proxy(md['map_path']); self.map_path = md['map_path']
        source = source_points(frame, mask_dynamic=md['dataset']=='cooperscene')
        center = (recovered @ frame['dataset_T_lidar'])[:3, 3]
        local = self.target - center
        local = local[(abs(local[:, :2])<65).all(axis=1) & (abs(local[:, 2])<20)]
        rng = np.random.default_rng(0)
        local = local[rng.choice(len(local), min(len(local),100000), replace=False)]
        ids = rng.choice(len(source), min(len(source),45000), replace=False)
        source = source[ids]
        fig, axes = plt.subplots(3,3, figsize=(18,12), layout='constrained')
        variants = [('Original translation-only reference', original, '#1698b5'),
                    ('Raw INS/GNSS + correct sensor composition' if self.physical else 'GNSS-recovered per-sequence reference', recovered, '#2c9d58'),
                    ('Unchanged estimated localization', estimate, '#db5325')]
        for i,(label,transform,color) in enumerate(variants):
            cloud = source @ transform[:3,:3].T + transform[:3,3] - center
            for j,(dims,ylim) in enumerate([((0,1),(-60,60)),((0,2),(-7,7)),((1,2),(-7,7))]):
                ax=axes[i,j];x,y=dims
                ax.scatter(local[:,x],local[:,y],s=.3,c='#646e7d',alpha=.35,rasterized=True)
                ax.scatter(cloud[:,x],cloud[:,y],s=.4,c=color,alpha=.5,rasterized=True)
                ax.set(xlim=(-60,60) if j==0 else (-30,30),ylim=ylim,
                       xlabel=f'Map {"XYZ"[x]} (m)',ylabel=f'Map {"XYZ"[y]} (m)',title=label if j==0 else ('X/Z side view' if j==1 else 'Y/Z side view'))
                ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2)
        name=f"{row['run_id']}-{row['method']}-f{row['frame_offset']:06d}"
        fig.suptitle(name+'\nSame saved localization: original / recovered reference XYZ disagreement '
                     +f"{row['original_xyz_m']:.3f} / {row['recovered_xyz_m']:.3f} m; signed Z "
                     +f"{row['original_dz_m']:+.3f} / {row['recovered_dz_m']:+.3f} m",fontsize=15)
        fig.supxlabel('Map gray; each row applies a different transform to the same static LiDAR. Axes centered on recovered GNSS pose.\nReference reconstruction uses no map fitting; altitude datum, capture timing and GNSS reference-point assumptions remain unverified.',fontsize=10)
        fig.savefig(self.output/(name+'.png'),dpi=140);plt.close(fig)

    def write(self, name, rows):
        fields = list(dict.fromkeys(k for row in rows for k in row))
        with (self.output/(name+'.csv')).open('w') as f:
            writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--attempts',type=Path,default=Path('/tmp/RemoteTeleop-hybrid-road-height/artifacts/full-sequence-localization/attempts'))
    parser.add_argument('--references',type=Path,default=Path('artifacts/reference-orientation/axes-audit/recovered-references.json'))
    parser.add_argument('--cases',type=Path,default=Path('/tmp/localization-over-0p5m-pcd/summary.csv'))
    parser.add_argument('--output',type=Path,default=Path('artifacts/reference-orientation/rescored'))
    args=parser.parse_args(); scorer=ReferenceRescorer(args.attempts,args.references,args.output)
    cases=list(csv.DictReader(args.cases.open()))
    rows=[scorer.score(r['sequence'],int(r['frame']),r['method'],visualize=True) for r in cases]
    scorer.write('previous-seven-cases',rows)
    first=[scorer.score(p.name,0,m) for p in sorted(args.attempts.iterdir()) if p.is_dir() for m in ('xyz','bev','bev_lidar_height')]
    scorer.write('all-23-first-frame',first)
    (args.output/'provenance.json').write_text(json.dumps(dict(reference_path=str(scorer.reference_path),reference_sha256=scorer.reference_hash,
        cases_path=str(args.cases),cases_sha256=hashlib.sha256(args.cases.read_bytes()).hexdigest(),
        saved_poses_changed=False,original_acceptance_changed=False,frames_reselected=False,
        independent_ground_truth=False,caveats=scorer.reference_document['caveats']),indent=2)+'\n')
    for row in rows:
        print(row['run_id'],row['method'],f"XYZ {row['original_xyz_m']:.3f} -> {row['recovered_xyz_m']:.3f}; Z {row['original_dz_m']:+.3f} -> {row['recovered_dz_m']:+.3f}",flush=True)


if __name__=='__main__':
    main()

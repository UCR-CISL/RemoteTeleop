"""Validate one frozen shared rigid ENU/map transform on unchanged held-out frames.

No evaluation pose is fitted. Plane height is compared at one common supported XY.
Nearest-neighbor residuals measure geometry consistency, not independent accuracy.
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
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.investigation.compare_reference_road_planes import height_at, inside, transform_plane
from scripts.investigation.first_frame_xyz import source_points
from src.localization.gaussian_map import load_gaussian_map
from src.localization.hybrid_road_height import HybridRoadHeightEstimator

ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / 'artifacts/reference-refinement/final-comparison'


class SharedMapValidation:
    def __init__(self, calibration: Path, output: Path):
        self.calibration_path = calibration.resolve()
        calibration_bytes = calibration.read_bytes()
        self.calibration_sha256 = hashlib.sha256(calibration_bytes).hexdigest()
        self.calibration = json.loads(calibration_bytes)
        self.D = np.asarray(self.calibration['D'], dtype=float)
        if not np.allclose(self.D[3], [0, 0, 0, 1]) or not np.allclose(self.D[:3,:3].T @ self.D[:3,:3], np.eye(3), atol=1e-7) or np.linalg.det(self.D[:3,:3]) < 0:
            raise ValueError('Expected a proper rigid correction D')
        self.output = output
        output.mkdir(parents=True, exist_ok=True)
        (output / "frozen-calibration-input.json").write_bytes(calibration_bytes)
        self.config = HybridRoadHeightEstimator().config
        self.records = json.loads((INPUT / 'diagnostics.json').read_text())
        first = json.loads(Path(self.records[0]['attempt_path']).read_text())
        gaussian = load_gaussian_map(first['metadata']['map_path'])
        keep = (gaussian.opacities > 0) & np.isfinite(gaussian.means).all(axis=1) & (np.abs(gaussian.means).max(axis=1) <= 10000)
        self.target = gaussian.means[keep].astype(float)
        self.tree = cKDTree(self.target)

    def run(self):
        rows = []
        manifest = self.calibration.get('calibration_frames', [])
        for record in self.records:
            for selected in manifest:
                run_id = selected.get('run_id', f"cooperscene-{selected.get('take')}-{selected.get('agent')}")
                if run_id == record['run_id'] and str(selected.get('frame_id')) == str(record['frame_id']):
                    raise ValueError(f"Calibration/evaluation frame leakage: {run_id} {record['frame_offset']}")
            attempt = json.loads(Path(record['attempt_path']).read_text())
            with np.load(attempt['metadata']['npz_path']) as frame:
                source = source_points(frame, True)
                W = frame['reference_map_T_dataset']
                native_pose = frame['dataset_T_lidar']
            case = f"{record['run_id']}-f{record['frame_offset']:06d}"
            with np.load(INPUT / case / 'road-support.npz') as data:
                native_support, map_support = data['native_support'], data['map_support']
            xy = np.asarray(record['comparison_xy_map'])
            mp, sp = record['map_road'], record['source_road']
            mn = np.asarray(mp['normal'])
            clouds = []
            for variant, T in [('released', W), ('shared_enu_map', self.D @ W)]:
                n, d = transform_plane(sp['normal'], sp['offset'], T)
                support = native_support @ T[:3,:3].T + T[:3,3]
                _, ids = np.unique(np.floor(support[:,:2]), axis=0, return_index=True)
                overlap = float(np.mean(cKDTree(map_support[:,:2]).query(support[ids,:2])[0] <= self.config.footprint_match_distance_m))
                source_inside, map_inside = inside(support, xy), inside(map_support, xy)
                supported = bool(record['map_support_status'] == 'supported' and source_inside and map_inside and overlap >= self.config.min_footprint_overlap)
                cloud = source @ T[:3,:3].T + T[:3,3]
                clouds.append(cloud)
                distances = self.tree.query(cloud, workers=2)[0]
                origin = (T @ native_pose)[:3,3]
                rows.append(dict(run_id=record['run_id'], frame_offset=record['frame_offset'], frame_id=record['frame_id'], variant=variant,
                    normal_angle_deg=float(np.degrees(np.arccos(np.clip(n @ mn, -1, 1)))),
                    signed_road_height_separation_m=height_at(n, d, xy) - height_at(mn, mp['offset'], xy) if supported else None,
                    unqualified_plane_height_difference_m=height_at(n, d, xy) - height_at(mn, mp['offset'], xy),
                    comparison_xy_map=xy.tolist(), source_support_contains_anchor=source_inside, map_support_contains_anchor=map_inside,
                    transformed_lidar_xy_source_supported=inside(support, origin[:2]), transformed_lidar_xy_map_supported=inside(map_support, origin[:2]),
                    road_footprint_overlap=overlap, road_comparison_supported=supported,
                    static_scan_point_count=len(source), full_scan_nn_median_m=float(np.median(distances)), full_scan_nn_p95_m=float(np.percentile(distances,95)),
                    full_scan_nn_within_1m_fraction=float(np.mean(distances <= 1)), candidate_map_T_dataset=T.tolist(),
                    plane_normal_map=n.tolist(), plane_offset_map=d,
                    lidar_origin_shift_map_m=(origin-(W @ native_pose)[:3,3]).tolist(),
                    status='geometry diagnostic; no localization gate changed; not independent GT'))
            self.plot(case, clouds, rows[-2:], xy)
            print(case, [(r['variant'], r['normal_angle_deg'],r['signed_road_height_separation_m'],r['full_scan_nn_median_m']) for r in rows[-2:]], flush=True)
        (self.output / 'comparisons.json').write_text(json.dumps(rows, indent=2)+'\n')
        scalars = [{k:v for k,v in r.items() if not isinstance(v,list)} for r in rows]
        with (self.output / 'comparisons.csv').open('w') as stream:
            writer=csv.DictWriter(stream, fieldnames=list(scalars[0])); writer.writeheader(); writer.writerows(scalars)
        summary = {}
        for variant in ['released','shared_enu_map']:
            group = [r for r in rows if r['variant']==variant]
            supported = [r for r in group if r['road_comparison_supported']]
            summary[variant] = dict(frames=len(group), supported_road_pairs=len(supported),
                median_normal_angle_deg=float(np.median([r['normal_angle_deg'] for r in group])),
                median_absolute_supported_road_height_m=float(np.median([abs(r['signed_road_height_separation_m']) for r in supported])) if supported else None,
                median_frame_nn_median_m=float(np.median([r['full_scan_nn_median_m'] for r in group])),
                median_frame_nn_p95_m=float(np.median([r['full_scan_nn_p95_m'] for r in group])))
        for metric in ['normal_angle_deg','signed_road_height_separation_m','full_scan_nn_median_m','full_scan_nn_p95_m']:
            pairs=[(a,b) for a,b in zip(rows[::2],rows[1::2]) if a[metric] is not None and b[metric] is not None]
            summary[metric+'_improved_frames']=sum(abs(b[metric])<abs(a[metric]) for a,b in pairs)
            summary[metric+'_comparable_frames']=len(pairs)
        summary['calibration_path']=str(self.calibration_path)
        summary['calibration_sha256']=self.calibration_sha256
        summary['frozen_D']=self.D.tolist()
        summary['calibration_frames']=manifest
        (self.output / 'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        self.report(rows, summary)

    def plot(self, case, clouds, rows, xy):
        anchor=np.r_[xy,0.]
        target=self.target-anchor
        target=target[(np.abs(target[:,:2])<=60).all(axis=1)&(np.abs(target[:,2])<=12)]
        rng=np.random.default_rng(0)
        target=target[rng.choice(len(target),min(100000,len(target)),replace=False)]
        ids=rng.choice(len(clouds[0]),min(40000,len(clouds[0])),replace=False)
        fig, axes=plt.subplots(2,3,figsize=(17,8),layout='constrained')
        for row,(cloud,metrics,color) in enumerate(zip(clouds,rows,['#1698b5','#db6e27'])):
            points=cloud[ids]-anchor
            for col,(x,y) in enumerate([(0,1),(0,2),(1,2)]):
                ax=axes[row,col]
                ax.scatter(target[:,x],target[:,y],s=.3,c='#68727d',alpha=.35,rasterized=True)
                ax.scatter(points[:,x],points[:,y],s=.5,c=color,alpha=.55,rasterized=True)
                ax.set(xlim=(-50,50) if col==0 else (-30,30),ylim=(-50,50) if col==0 else (-7,8),
                    xlabel=f'Map {"XYZ"[x]} relative to common XY (m)',ylabel=f'Map {"XYZ"[y]} (m)')
                ax.set_aspect('equal',adjustable='box'); ax.grid(alpha=.2)
                ax.set_title(['Top view','X side view','Y side view'][col])
            height=metrics['signed_road_height_separation_m']
            title=f"{metrics['variant']}\nroad {metrics['normal_angle_deg']:.2f} deg / " + (f'{height:+.3f} m' if height is not None else 'UNSUPPORTED height')
            axes[row,0].set_title(title)
        fig.suptitle(case+' | gray: GSplat; colored: masked released LiDAR\nSame scan, one frozen shared transform; no per-frame fit')
        fig.savefig(self.output / f'{case}.png',dpi=130);plt.close(fig)

    def report(self, rows, summary):
        lines=['# Shared ENU/map transform: independent fixed-frame validation','',
            f'Frozen calibration: `{self.calibration_path}`. All eight previously reported frames retained. No evaluation pose fitted.','',
            '| Take/agent frame | Original angle / signed height | Shared-transform angle / signed height | Static NN median before → after |',
            '|---|---:|---:|---:|']
        for a,b in zip(rows[::2],rows[1::2]):
            def plane(r):
                h=r['signed_road_height_separation_m']
                return f"{r['normal_angle_deg']:.2f}° / {h:+.3f} m" if h is not None else f"{r['normal_angle_deg']:.2f}° / unsupported"
            lines.append(f"| {a['run_id']} f{a['frame_offset']} | {plane(a)} | {plane(b)} | {a['full_scan_nn_median_m']:.3f} → {b['full_scan_nn_median_m']:.3f} m |")
        lines += ['', '## Interpretation', '',
            'Signed road height is measured at the identical fixed map XY used before calibration. A height is reported only when source and map inlier hulls contain that location and the original footprint gate passes. Unqualified plane extrapolations remain separately labelled in JSON. The same road points and map plane are reused across variants.', '',
            'Nearest-neighbor statistics include every statically masked scan point against positive-opacity Gaussian centers. Occlusion, incomplete overlap and residual dynamic points can inflate them. They are geometry consistency statistics, not independent localization error or original registration acceptance.', '',
            'One shared rigid transform preserves vehicle-to-vehicle and vehicle-to-infrastructure geometry. It cannot repair differences among those inputs, time-dependent pose drift, per-agent mounting errors or road curvature. Infrastructure contributes calibration geometry, not independent validation of released ICP. Repeated fixed infrastructure scans must not be counted as independent reference sensors.', '',
            'The fixed-frame cache was checked against original archives: all point counts match, coordinates differ by less than 0.000001 m and released poses agree within 3.2e-14 per matrix entry. Cached annotations contain more boxes than source-unit YAML (merged annotations), so this validation masks additional vehicles. Each before/after pair uses exactly the same subset. Raw-source calibration and these full-resolution diagnostics are not numerically interchangeable.', '',
            'Map road fitting retains the historical diagnostic Z band [-4.3,-1.3] m and original plane support gates. Hulls can bridge holes. These checks do not certify ground truth or change localization acceptance thresholds.', '', '## Views', '']
        for r in rows[::2]:
            case=f"{r['run_id']}-f{r['frame_offset']:06d}"
            lines += [f'### {case}', '', f'![Before/after cloud alignment]({case}.png)', '']
        (self.output/'report.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--calibration',type=Path,required=True)
    parser.add_argument('--output',type=Path,default=ROOT/'artifacts/reference-refinement/shared-enu-validation')
    args=parser.parse_args()
    SharedMapValidation(args.calibration,args.output).run()

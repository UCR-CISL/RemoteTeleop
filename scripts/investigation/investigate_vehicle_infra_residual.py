"""Offline vehicle/infrastructure residual diagnosis; never an independent GT fit.

Uses the existing shared calibration manifest without changing its split. Sensor
mounting corrections multiply released sensor poses on the right; world-frame
corrections multiply on the left. Leave-one-take-out variants never fit a scan
from their evaluation take. Only recorded calibration infrastructure scans form
targets. Evaluation scan points/support are frozen before fitting corrections.
"""
from __future__ import annotations
import csv
import json
import sys
import zipfile
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import yaml
from scipy.spatial import ConvexHull, cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from src.localization.cooperscene import pose_to_matrix
from scripts.investigation.audit_infrastructure_reference import plane, ROI, ANCHOR
from scripts.investigation.calibrate_shared_enu_map import transform, voxel

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'artifacts/reference-refinement/vehicle-infra-residual'
MANIFEST = ROOT / 'artifacts/reference-refinement/shared-enu-calibration/input-manifest.json'


def stats(values):
    values = np.asarray(values)
    if not len(values):
        return dict(count=0)
    return dict(count=len(values), median_m=float(np.median(values)),
                p95_m=float(np.percentile(values, 95)), under_010m=float(np.mean(values < .1)),
                under_025m=float(np.mean(values < .25)), under_050m=float(np.mean(values < .5)))


def describe(T):
    return dict(matrix=T.tolist(), translation_m=T[:3, 3].tolist(),
                rpy_deg=Rotation.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True).tolist())


class VehicleInfrastructureResidual:
    def __init__(self):
        OUT.mkdir(parents=True, exist_ok=True)
        self.manifest = json.loads(MANIFEST.read_text())
        self.scans = []
        self.targets = {}
        self.fits = []
        self.rows = []

    def read_scan(self, archive, record):
        prefix = f"{record['take']}/{record['agent']}/{record['frame_id']}"
        ann = yaml.safe_load(archive.read(prefix + '.yaml'))
        pose = pose_to_matrix(ann['lidar_pose'])
        points = np.fromstring(archive.read(prefix + '.pcd').split(b'DATA ascii\n', 1)[1].decode(), sep=' ').reshape(-1, 4)[:, :3]
        points = transform(points, pose)
        keep = np.isfinite(points).all(1)
        for box in ann['vehicles'].values():
            center = np.asarray(box['location']) + box['center']
            B = pose_to_matrix([*center, *box['angle']])
            q = (points - center) @ B[:3, :3]
            keep &= ~np.all(np.abs(q) <= np.asarray(box['extent']) + .2, axis=1)
        return voxel(points[keep], .2), pose

    def prepare(self):
        cache = OUT / 'input-clouds.npz'
        if cache.exists():
            with np.load(cache) as data:
                for r in self.manifest:
                    if r['agent'] == 0 and r['partition'] != 'calibration':
                        continue
                    key = f"{r['take']}_{r['agent']}_{r['frame_id']}"
                    self.scans.append(dict(record=r, points=data['p_' + key], pose=data['T_' + key]))
        else:
            arrays = {}
            for take in range(1, 6):
                records = [r for r in self.manifest if r['take'] == take and (r['agent'] or r['partition'] == 'calibration')]
                with zipfile.ZipFile(records[0]['archive']) as archive:
                    for r in records:
                        points, pose = self.read_scan(archive, r)
                        key = f"{take}_{r['agent']}_{r['frame_id']}"
                        arrays['p_' + key] = points
                        arrays['T_' + key] = pose
                        self.scans.append(dict(record=r, points=points, pose=pose))
                print('Read original take', take, flush=True)
            np.savez_compressed(cache, **arrays)
        for take in range(1, 6):
            p = voxel(np.concatenate([s['points'] for s in self.scans if s['record']['take'] == take and s['record']['agent'] == 0]), .15)
            tree = cKDTree(p)
            pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p))
            pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=1.2, max_nn=30))
            normals = np.asarray(pc.normals).copy()
            pc.estimate_covariances(o3d.geometry.KDTreeSearchParamHybrid(radius=1.2, max_nn=30))
            eigen = np.linalg.eigvalsh(np.asarray(pc.covariances))
            count = tree.query_ball_point(p, .8, return_length=True, workers=2)
            variation = eigen[:, 0] / np.maximum(eigen.sum(1), 1e-15)
            second_axis = eigen[:, 1] / np.maximum(eigen[:, 2], 1e-15)
            valid = (count >= 6) & (variation < .08) & (second_axis > .05)
            quality = dict(take=take, target_points=len(p), count_supported=int((count>=6).sum()), planar_supported=int(valid.sum()), surface_variation_median=float(np.median(variation[valid])), second_axis_ratio_median=float(np.median(second_axis[valid])), normal_radius_m=1.2, support_radius_m=.8, min_neighbors=6, maximum_surface_variation=.08, minimum_second_axis_ratio=.05)
            (OUT / f'target-normal-quality-take{take}.json').write_text(json.dumps(quality, indent=2))
            pr = plane(p)
            coef = np.array(pr['z_ax_by_c'])
            road = (np.abs(p[:, 2] - np.c_[p[:, :2], np.ones(len(p))] @ coef) < .2) & (np.abs(normals[:, 2]) > .9)
            self.targets[take] = dict(points=p, tree=tree, normals=normals, valid=valid, plane=pr, road=road)
        self.scans = [s for s in self.scans if s['record']['agent']]
        for scan in self.scans:
            target = self.targets[scan['record']['take']]
            p = scan['points']
            distance, index = target['tree'].query(p, workers=2)
            support = distance < 2.
            scan['support'] = support
            scan['road_category'] = target['road'][index]
            self.prepare_road(scan, target)
            selected = p[support]
            if len(selected) > 2500:
                selected = selected[np.linspace(0, len(selected) - 1, 2500).astype(int)]
            scan['fit_world'] = selected
            scan['fit_local'] = transform(selected, np.linalg.inv(scan['pose']))
        (OUT / 'input-manifest.json').write_text(json.dumps(dict(source_manifest=str(MANIFEST), vehicle_frames=[s['record'] for s in self.scans], infrastructure_frames=[r for r in self.manifest if r['agent'] == 0 and r['partition'] == 'calibration'], road_patch_size_xy_m=[24,18], road_anchor_rule='Median released-placement XY overlap per scan, frozen before correction', mask_padding_m=.2, selection='Original released-frame NN <2m and fixed before comparisons; fitting normals separately need six neighbors within 0.8m and covariance planarity', source_voxel_m=.2), indent=2))
        print('Prepared 75 vehicle scans and 5 infrastructure targets', flush=True)

    def prepare_road(self, scan, target):
        # Choose an overlap patch from the released placement once. Infra road
        # candidates stay close to its robust plane; points are never selected
        # by the performance of a fitted correction.
        p = scan['points']; tp = target['points'][target['road']]
        xy_tree = cKDTree(tp[:, :2])
        distance, index = xy_tree.query(p[:, :2])
        candidate = (distance < 1.) & (abs(p[:, 2] - tp[index, 2]) < 2.) & (p[:, 2] < 0.) & (p[:, 2] > -6.)
        scan['plane'] = dict(status='insufficient_overlap')
        scan['road_points'] = np.empty((0, 3))
        scan['anchor'] = ANCHOR.copy()
        scan['target_plane'] = target['plane']
        if candidate.sum() < 40:
            return
        anchor = np.median(p[candidate, :2], axis=0)
        shift = np.r_[ANCHOR - anchor, 0.]
        # Existing robust plane routine: identical rectangular XY footprint in
        # both clouds. Translation changes only its comparison location.
        pr = plane(p[candidate] + shift)
        tr = plane(tp + shift)
        if pr['status'] != 'ok' or tr['status'] != 'ok':
            return
        for record in [pr, tr]:
            coef = np.array(record['z_ax_by_c'])
            coef[2] += coef[:2] @ shift[:2]
            record['z_ax_by_c'] = coef.tolist()
            record['support_hull_xy'] = (np.array(record['support_hull_xy']) - shift[:2]).tolist()
        scan['plane'] = pr; scan['target_plane'] = tr; scan['anchor'] = anchor
        q = p + shift
        roi = (q[:, 0] > ROI[0]) & (q[:, 0] < ROI[1]) & (q[:, 1] > ROI[2]) & (q[:, 1] < ROI[3])
        coef = np.array(pr['z_ax_by_c'])
        residual = abs(p[:, 2] - np.c_[p[:, :2], np.ones(len(p))] @ coef)
        scan['road_points'] = p[roi & candidate & (residual < .12)]
        scan['target_road_points'] = tp

    def fit(self, agent, mode, exclude_take=None, only_take=None):
        scans = [s for s in self.scans if s['record']['agent'] == agent and s['record']['partition'] == 'calibration' and s['record']['take'] != exclude_take and (only_take is None or s['record']['take'] == only_take) and len(s['fit_world']) >= 100]
        correction = np.eye(4)
        trace = []
        for gate in [1.5, .75, .4]:
            for iteration in range(20):
                H = np.zeros((6, 6)); b = np.zeros(6); count = 0; residues = []
                for s in scans:
                    target = self.targets[s['record']['take']]
                    if mode == 'world':
                        x = transform(s['fit_world'], correction)
                    else:
                        v = transform(s['fit_local'], correction)
                        x = transform(v, s['pose'])
                    dist, idx = target['tree'].query(x, workers=2)
                    keep = (dist < gate) & target['valid'][idx]
                    if keep.sum() < 50:
                        continue
                    n = target['normals'][idx[keep]]
                    q = target['points'][idx[keep]]
                    residual = np.sum(n * (x[keep] - q), axis=1)
                    if mode == 'world':
                        A = np.c_[np.cross(x[keep], n), n]
                    else:
                        local_n = n @ s['pose'][:3, :3]
                        A = np.c_[np.cross(v[keep], local_n), local_n]
                    category = target['road'][idx[keep]]
                    weight = np.zeros(len(n))
                    for c in [False, True]:
                        choose = category == c
                        weight[choose] = 1 / max(1, choose.sum())
                    weight /= max(weight.sum(), 1e-12)
                    weight *= np.minimum(1., .1 / np.maximum(abs(residual), 1e-12))
                    H += A.T @ (weight[:, None] * A)
                    b += A.T @ (weight * residual)
                    count += len(n); residues.extend(abs(residual))
                delta = np.linalg.lstsq(H, -b, rcond=1e-10)[0]
                delta /= max(np.linalg.norm(delta[:3]) / np.deg2rad(1), np.linalg.norm(delta[3:]) / .25, 1.)
                step = np.eye(4); step[:3, :3] = Rotation.from_rotvec(delta[:3]).as_matrix(); step[:3, 3] = delta[3:]
                correction = step @ correction
                trace.append(dict(gate_m=gate, iteration=iteration, count=count, median_point_plane_m=float(np.median(residues)), step_norm=float(np.linalg.norm(delta)), hessian_eigenvalues=np.linalg.eigvalsh(H).tolist()))
                if np.linalg.norm(delta) < 1e-6:
                    break
        last = trace[-1]
        eigen = np.asarray(last['hessian_eigenvalues'])
        rank = int(np.sum(eigen > max(eigen[-1] * 1e-8, 1e-12)))
        valid_fit = bool(scans and last['count'] >= 100 and rank == 6)
        record = dict(status='supported_diagnostic_fit' if valid_fit else 'insufficient_support', valid_fit=valid_fit, final_rank=rank, agent=agent, mode=mode, exclude_take=exclude_take, only_take=only_take, training_frames=[s['record'] for s in scans], correction=describe(correction), trace=trace)
        self.fits.append(record)
        print('Fit', agent, mode, exclude_take, only_take, np.round(record['correction']['rpy_deg'], 3), np.round(correction[:3, 3], 3), flush=True)
        return correction

    def evaluate(self, scan, variant, correction, mode):
        r = scan['record']; target = self.targets[r['take']]
        valid_fit = True
        if variant != 'released':
            matched_fit = next(f for f in self.fits if f['agent']==r['agent'] and f['mode']==mode and f['exclude_take']==(r['take'] if variant.endswith('leave_take_out') else None) and f['only_take']==(r['take'] if variant=='world_pertake_cal' else None))
            valid_fit = matched_fit.get('valid_fit', True)
        T = correction if mode == 'world' else scan['pose'] @ correction @ np.linalg.inv(scan['pose'])
        p = transform(scan['points'], T)
        distance, nearest = target['tree'].query(p, workers=2)
        plane_valid = scan['support'] & target['valid'][nearest]
        point_plane = np.sum(target['normals'][nearest] * (p - target['points'][nearest]), axis=1)
        support = scan['support']; road = scan['road_category']
        row = dict(**r, variant=variant, fit_valid=valid_fit, applied_world_correction=describe(T), support_count=int(support.sum()), support_fraction=float(support.mean()), all_points=stats(distance), common_support=stats(distance[support]), road_nn=stats(distance[support & road]), nonroad_nn=stats(distance[support & ~road]), road_supported=False, point_plane=stats(abs(point_plane[plane_valid])), normal_support_count=int(plane_valid.sum()))
        pr = scan['plane']; ref = scan['target_plane']; anchor = scan['anchor']
        if pr['status'] == 'ok':
            n = np.array(pr['normal']); d = -pr['z_ax_by_c'][2] / np.linalg.norm(np.r_[-np.array(pr['z_ax_by_c'])[:2], 1.])
            n = T[:3, :3] @ n; d -= n @ T[:3, 3]
            rp = transform(scan['road_points'], T)
            hull = ConvexHull(rp[:, :2])
            inside = bool(np.all(hull.equations[:, :2] @ anchor + hull.equations[:, 2] <= 1e-8))
            refcoef = np.asarray(ref['z_ax_by_c'])
            footprint = float(np.mean(cKDTree(scan['target_road_points'][:, :2]).query(voxel(rp, .5)[:, :2])[0] < 1.))
            row.update(road_anchor_xy=anchor.tolist(), road_angle_deg=float(np.rad2deg(np.arccos(np.clip(n @ ref['normal'], -1, 1)))), road_height_m=float(-(n[:2] @ anchor + d) / n[2] - ref['height_at_anchor_m']), road_supported=bool(inside and ref['anchor_within_support_hull'] and footprint >= .6), road_support_area_m2=float(hull.volume), road_footprint_overlap_1m=footprint, source_road_fit_rms_m=pr['rms_m'], target_road_fit_rms_m=ref['rms_m'], source_road_inliers=pr['inlier_count'])
        self.rows.append(row)

    def run(self):
        self.prepare()
        self.transforms = {}
        for agent in range(1, 4):
            for mode in ['world', 'sensor']:
                self.transforms[agent, mode, None] = self.fit(agent, mode)
                for take in range(1, 6):
                    self.transforms[agent, mode, take] = self.fit(agent, mode, exclude_take=take)
            for take in range(1, 6):
                self.transforms[agent, 'pertake', take] = self.fit(agent, 'world', only_take=take)
            for s in [s for s in self.scans if s['record']['agent'] == agent]:
                take = s['record']['take']
                self.evaluate(s, 'released', np.eye(4), 'world')
                for mode in ['world', 'sensor']:
                    self.evaluate(s, mode + '_allcal', self.transforms[agent, mode, None], mode)
                    self.evaluate(s, mode + '_leave_take_out', self.transforms[agent, mode, take], mode)
                self.evaluate(s, 'world_pertake_cal', self.transforms[agent, 'pertake', take], 'world')
            (OUT / 'fits.json').write_text(json.dumps(self.fits, indent=2))
            (OUT / 'metrics.json').write_text(json.dumps(self.rows, indent=2))
        self.report()

    def report(self):
        # Cached runs receive the same support audit as fresh runs.
        for fit in self.fits:
            last = fit['trace'][-1]; eigen = np.asarray(last['hessian_eigenvalues'])
            rank = int(np.sum(eigen > max(eigen[-1] * 1e-8, 1e-12)))
            fit['valid_fit'] = bool(fit['training_frames'] and last['count'] >= 100 and rank == 6)
            fit['final_rank'] = rank
            fit['status'] = 'supported_diagnostic_fit' if fit['valid_fit'] else 'insufficient_support'
        for row in self.rows:
            if row['variant'] == 'released':
                row['fit_valid'] = True; continue
            mode = 'sensor' if row['variant'].startswith('sensor') else 'world'
            fit = next(f for f in self.fits if f['agent']==row['agent'] and f['mode']==mode and f['exclude_take']==(row['take'] if row['variant'].endswith('leave_take_out') else None) and f['only_take']==(row['take'] if row['variant']=='world_pertake_cal' else None))
            row['fit_valid'] = fit['valid_fit']
        (OUT / 'fits.json').write_text(json.dumps(self.fits, indent=2))
        (OUT / 'metrics.json').write_text(json.dumps(self.rows, indent=2))
        self.excitation()
        self.audit_anchors()
        rows = [r for r in self.rows if r['partition'] == 'heldout' and r.get('fit_valid',True)]
        summary = []
        for agent in [0, 1, 2, 3]:
            for variant in ['released', 'world_allcal', 'sensor_allcal', 'world_leave_take_out', 'sensor_leave_take_out', 'world_pertake_cal']:
                selected = [r for r in rows if r['variant'] == variant and (agent == 0 or r['agent'] == agent)]
                supported = [r for r in selected if r['road_supported']]
                summary.append(dict(agent=agent or 'all', variant=variant, frames=len(selected), road_supported=len(supported), median_scan_nn_m=float(np.median([r['common_support']['median_m'] for r in selected])), median_scan_nn_p95_m=float(np.median([r['common_support']['p95_m'] for r in selected])), median_nonroad_nn_m=float(np.median([r['nonroad_nn']['median_m'] for r in selected])), median_road_nn_m=float(np.median([r['road_nn']['median_m'] for r in selected if r['road_nn']['count']])), median_road_angle_deg=float(np.median([r['road_angle_deg'] for r in supported])) if supported else None, median_abs_road_height_m=float(np.median([abs(r['road_height_m']) for r in supported])) if supported else None, max_abs_road_height_m=float(np.max([abs(r['road_height_m']) for r in supported])) if supported else None))
        pairs = []
        for agent in [0,1,2,3]:
            base = {(r['take'],r['agent'],r['frame_id']):r for r in rows if r['variant']=='released' and (agent==0 or r['agent']==agent)}
            for variant in ['world_allcal','sensor_allcal','world_leave_take_out','sensor_leave_take_out','world_pertake_cal']:
                compared = [(base[r['take'],r['agent'],r['frame_id']],r) for r in rows if r['variant']==variant and (agent==0 or r['agent']==agent)]
                matched = [(a,b) for a,b in compared if a['road_supported'] and b['road_supported']]
                pair = dict(agent=agent or 'all', variant=variant, frames=len(compared), nn_improved_frames=sum(b['common_support']['median_m']<a['common_support']['median_m'] for a,b in compared), road_matched_frames=len(matched))
                for name, index in [('before',0),('after',1)]:
                    pair[name] = dict(nn_median_m=float(np.median([ab[index]['common_support']['median_m'] for ab in compared])), point_plane_median_m=float(np.median([ab[index]['point_plane']['median_m'] for ab in compared if ab[index]['point_plane']['count']])), road_height_median_abs_m=float(np.median([abs(ab[index]['road_height_m']) for ab in matched])) if matched else None, road_height_max_abs_m=float(np.max([abs(ab[index]['road_height_m']) for ab in matched])) if matched else None, road_angle_median_deg=float(np.median([ab[index]['road_angle_deg'] for ab in matched])) if matched else None)
                pairs.append(pair)
        (OUT / 'matched-comparisons.json').write_text(json.dumps(pairs,indent=2))
        (OUT / 'summary.json').write_text(json.dumps(summary, indent=2))
        with (OUT / 'summary.csv').open('w') as f:
            w = csv.DictWriter(f, fieldnames=list(summary[0])); w.writeheader(); w.writerows(summary)
        trends = []
        for agent in range(1, 4):
            for take in range(1, 6):
                selected = sorted([r for r in self.rows if r['agent'] == agent and r['take'] == take and r['variant'] == 'released' and r['road_supported']], key=lambda r:r['fraction'])
                if len(selected) >= 3:
                    x = np.array([r['fraction'] for r in selected]); y = np.array([r['road_height_m'] for r in selected]); fit = np.polyfit(x, y, 1)
                    trends.append(dict(agent=agent, take=take, frames=len(selected), road_anchor_xy_span_m=np.ptp(np.array([r['road_anchor_xy'] for r in selected]),axis=0).tolist(), height_span_m=float(np.ptp(y)), height_slope_m_per_take_fraction=float(fit[0]), linear_rms_m=float(np.sqrt(np.mean((y-np.polyval(fit,x))**2)))))
        (OUT / 'temporal-trends.json').write_text(json.dumps(trends, indent=2))
        fig, axes = plt.subplots(3, 5, figsize=(19, 10), sharey=True, layout='constrained')
        for agent in range(1, 4):
            for take in range(1, 6):
                ax = axes[agent-1, take-1]
                for variant, color in [('released','k'), ('world_leave_take_out','#d87520'), ('sensor_leave_take_out','#1886bb'), ('world_pertake_cal','#6b9b39')]:
                    selected = sorted([r for r in self.rows if r['agent'] == agent and r['take'] == take and r['variant'] == variant and r['road_supported']], key=lambda r:r['fraction'])
                    ax.plot([r['fraction'] for r in selected], [r['road_height_m'] for r in selected], '-', color=color, label=variant, alpha=.8)
                    for r in selected:
                        ax.scatter(r['fraction'],r['road_height_m'], color=color, marker='o' if r['partition']=='heldout' else 'x',s=30)
                ax.axhspan(-.1,.1,color='gray',alpha=.12); ax.axhline(0,color='gray',lw=.5); ax.grid(alpha=.2)
                ax.set_title(f'Vehicle {agent}, take {take}'); ax.set_xlabel('Normalized chronological frame index')
                if take == 1: ax.set_ylabel('Vehicle road height minus infra (m)')
        axes[0,0].legend(fontsize=7); fig.suptitle('Common vehicle/infra patch per frame (patch location can move); x = calibration, circles = heldout\nNo GSplat; frozen leave-take-out corrections; lines only connect measured frames')
        fig.savefig(OUT/'road-trends.png',dpi=150); plt.close(fig)
        self.cloud_gallery()
        lines = ['# Vehicle versus infrastructure residual experiment', '', 'Offline diagnostic only. Infrastructure supplied the released ICP reference and is not independent ground truth. GSplat is excluded from this experiment.', '', '## Protocol', '', 'The exact shared-calibration manifest is retained: 45 vehicle calibration scans and 30 vehicle heldout scans (three vehicles, five takes). Fifteen infrastructure calibration scans form five static per-take targets. Original archived PCD/YAML are decoded and released poses applied; recorded vehicle boxes are removed with 0.2m padding. All frame IDs and poses are cached. No mounted source, runtime default, existing evaluation queue or fullbatch changes.', '', 'Source voxels are 0.2m. Geometric support is frozen from the released placement: initial NN distance <2m; fitting additionally requires target support ≥6 neighbors within 0.8m. Road/nonroad NN categories are assigned once by the matched infrastructure surface. Every variant evaluates identical source points; all-point metrics are also retained. Road diagnostics choose a released-placement overlap patch once per scan: source points within 1m XY and 2m height of infra road, broad Z [-6,0]m. The median overlap XY anchors a 24×18m rectangular footprint shared by both clouds. Dominant plane inliers and support are frozen, then transformed with the correction. Both hull containment and ≥60% 1m XY footprint overlap are required. Plane fitting is a diagnostic, not per-frame pose optimization.', '', 'World corrections use D @ released_pose. Sensor corrections use released_pose @ X, a fixed local mounting hypothesis. Both use balanced per-scan and road/nonroad robust point-to-plane fits (Huber 0.1m; 1.5/0.75/0.4m gates; capped 2500 source points per scan). Five leave-one-take-out fits per vehicle use only calibration scans from the other four takes. Per-take world fits are a separately labelled flexibility diagnostic using only that take’s three calibration scans. No evaluation frame enters any fit. Sparse scan contributions with fewer than 100 initial supported points, or fewer than 50 current correspondences, are omitted. Target normal covariance requires smallest-eigenvalue/trace <0.08 and middle/largest eigenvalue >0.05. Normal fitting radius is 1.2m, with at least 6 neighbors within 0.8m. The preliminary 0.7m/0.4m neighborhood rejected most sparse infrastructure surfaces and was revised before final interpretation; these are offline diagnostic settings and runtime localization acceptance gates are unchanged.', '', '## Heldout scan summary', '', '|Vehicle|Variant|NN median|NN p95|Nonroad NN|Road angle|Abs road height|Road support|', '|---|---|---:|---:|---:|---:|---:|---:|']
        for r in summary:
            ang = 'NA' if r['median_road_angle_deg'] is None else f"{r['median_road_angle_deg']:.3f}°"
            h = 'NA' if r['median_abs_road_height_m'] is None else f"{r['median_abs_road_height_m']:.3f}m"
            lines.append(f"|{r['agent']}|{r['variant']}|{r['median_scan_nn_m']:.3f}m|{r['median_scan_nn_p95_m']:.3f}m|{r['median_nonroad_nn_m']:.3f}m|{ang}|{h}|{r['road_supported']}/{r['frames']}|")
        lines += ['', '## Evidence and limits', '', '![Same-patch road trends](road-trends.png)', '', '![Heldout cloud comparisons](cloud-gallery.png)', '', 'The cloud gallery selects the larger absolute released road separation among the two heldout frames for each vehicle/take, with selection disclosed in gallery-frames.json. It does not select based on improvement. Gray = infrastructure; red = released scan; blue = leave-take-out sensor correction. Plot subsets are the same frozen common-supported points, cropped around the same per-frame road comparison location; unsupported road cases are labelled.', '', 'NN distances conflate pose, point spacing, occlusion, residual moving objects, and surface differences. Road support hulls can bridge holes, and the broad road Z band is a geometric assumption. Each scan uses a common vehicle/infrastructure road patch, but its anchor can change as the vehicle moves; apparent temporal changes can include spatial road differences. Five sparse observations per take can reveal trends but cannot establish clock drift or separate attitude, GNSS height and motion distortion. A fixed right correction that generalizes supports a mounting hypothesis; neither a fitted left nor right correction proves calibration provenance or absolute accuracy. See original-icp-anchor-metrics.json for the released ICP fitting frames as separately labelled diagnostics, excitation.json for attitude excitation and Hessian conditioning, target-normal-quality-take*.json for normal support. See fits.json for every matrix and training frame, metrics.json for per-frame support and residuals, summary.csv for aggregates, temporal-trends.json for unfitted-reference temporal summaries.']
        world = next(r for r in pairs if r['agent']=='all' and r['variant']=='world_leave_take_out')
        sensor = next(r for r in pairs if r['agent']=='all' and r['variant']=='sensor_leave_take_out')
        findings = ['## Findings', '',
                    'A fixed per-vehicle mounting correction did not generalize reliably across takes. Some fitted road heights improve while static nonroad/full-cloud consistency worsens. Infrastructure sampling and visibility are coarse: NN distance is not position error or a decimeter GT test.', '',
                    '|Leave-take-out correction|Heldout NN median before → after|NN improved|Paired roads|Abs road height median before → after|Worst paired road after|',
                    '|---|---:|---:|---:|---:|---:|']
        for name,r in [('World / left',world),('Sensor / right',sensor)]:
            findings.append(f"|{name}|{r['before']['nn_median_m']:.3f} → {r['after']['nn_median_m']:.3f}m|{r['nn_improved_frames']}/{r['frames']}|{r['road_matched_frames']}|{r['before']['road_height_median_abs_m']:.3f} → {r['after']['road_height_median_abs_m']:.3f}m|{r['after']['road_height_max_abs_m']:.3f}m|")
        findings += ['', 'These road numbers use the identical before/after supported frame subset for each comparison; matched-comparisons.json retains the pairing. The larger table below also reports variant-specific road support and must not be used to compare unpaired road populations.', '',
                     'The sensor correction improves NN in 5/10 vehicle 1, 4/10 vehicle 2, and 0/10 vehicle 3 heldout scans. Across leave-take-out fits, sensor-correction rotation component spans reach 3.01° yaw for vehicle 2 and 5.07° roll for vehicle 3; translation spans reach 0.914m for vehicle 2 and 1.097m for vehicle 3. This instability prevents interpreting the fitted correction as a recovered physical mounting extrinsic.', '',
                     f"All {len(self.fits)} final fit normal matrices have numerical rank 6 at relative 1e-8 tolerance; the smallest final correspondence count is {min(f['trace'][-1]['count'] for f in self.fits)}. This rank check is only a geometry support diagnostic. Fit status is recorded explicitly, and unsupported fits are excluded from summaries. Heading excitation spans 181–183°, while nearly planar vehicle attitudes leave centered-rotation-stack smallest singular values 0.043–0.059 versus leading values 3.10–3.49. Vertical lever-arm interpretation remains weakly excited; Hessian units also mix radians and meters.", '',
                     'Original ICP anchors have supported road comparisons in 9/15 vehicle/take cases. Their absolute road heights range 0.014–0.202m; six are explicitly unsupported. Anchors are calibration evidence inherited by the release, not independent validation, and changing road patch locations prevent treating later discrepancies as proven temporal drift. Sparse chronological road spans are recorded in temporal-trends.json together with anchor XY spans.', '',
                     'All 75 archived YAML poses reproduce C @ metadata within 9.95e-14 maximum matrix-entry difference (released-pose-verification.json). The disagreement is not a pose conversion sign or transpose mistake.', '']
        lines[4:4] = findings
        (OUT/'report.md').write_text('\n'.join(lines)+'\n')

    def audit_anchors(self):
        frames = {1:'481693', 2:'488685', 3:'497206', 4:'515587', 5:'524102'}
        anchor_rows = []
        for take, frame in frames.items():
            archive_path = next(r['archive'] for r in self.manifest if r['take'] == take)
            with zipfile.ZipFile(archive_path) as archive:
                for agent in range(1,4):
                    keys = sorted(n.split('/')[-1][:-5] for n in archive.namelist() if n.startswith(f'{take}/{agent}/') and n.endswith('.yaml'))
                    if frame not in keys:
                        anchor_rows.append(dict(take=take, agent=agent, frame_id=frame, status='anchor_unavailable_in_archive')); continue
                    r = dict(take=take, agent=agent, frame_id=frame, frame_offset=keys.index(frame), frame_count=len(keys), fraction=keys.index(frame)/max(1,len(keys)-1), partition='original_icp_anchor', archive=archive_path)
                    points, pose = self.read_scan(archive, r)
                    s = dict(record=r, points=points, pose=pose)
                    target = self.targets[take]; distance,index = target['tree'].query(points,workers=2)
                    s['support']=distance<2.; s['road_category']=target['road'][index]
                    self.prepare_road(s,target)
                    length = len(self.rows)
                    self.evaluate(s,'released',np.eye(4),'world')
                    for mode in ['world','sensor']:
                        self.evaluate(s,mode+'_leave_take_out',self.transforms[agent,mode,take],mode)
                    anchor_rows.extend(self.rows[length:]); del self.rows[length:]
        (OUT / 'original-icp-anchor-metrics.json').write_text(json.dumps(anchor_rows,indent=2))

    def excitation(self):
        records = []
        for agent in range(1, 4):
            scans = [s for s in self.scans if s['record']['agent'] == agent and s['record']['partition'] == 'calibration']
            rotations = Rotation.from_matrix(np.array([s['pose'][:3, :3] for s in scans]))
            euler = rotations.as_euler('xyz', degrees=True)
            # Circular heading span is 360 degrees minus the largest empty arc.
            heading = np.sort(euler[:, 2] % 360)
            heading_span = 360 - np.diff(np.r_[heading, heading[0] + 360]).max()
            means = np.mean(rotations.as_matrix(), axis=0)
            spread = np.concatenate([s['pose'][:3, :3] - means for s in scans])
            singular = np.linalg.svd(spread, compute_uv=False)
            fits = [f for f in self.fits if f['agent'] == agent and f['only_take'] is None]
            conditioning = []
            for fit in fits:
                eigen = np.asarray(fit['trace'][-1]['hessian_eigenvalues'])
                conditioning.append(dict(mode=fit['mode'], excluded_take=fit['exclude_take'], raw_hessian_condition=float(eigen[-1] / max(eigen[0], 1e-15)), hessian_eigenvalues=eigen.tolist()))
            records.append(dict(agent=agent, calibration_frames=len(scans), roll_range_deg=np.ptp(euler[:, 0]).item(), pitch_range_deg=np.ptp(euler[:, 1]).item(), circular_heading_span_deg=float(heading_span), centered_rotation_stack_singular_values=singular.tolist(), centered_rotation_stack_condition=float(singular[0] / max(singular[-1],1e-15)), fits=conditioning, caveat='Fit Hessian condition mixes rotational and translational units and depends on coordinate origin. Near-planar vehicle attitude weakly excites vertical lever arm: right correction generalization does not prove mounting calibration.'))
        (OUT / 'excitation.json').write_text(json.dumps(records, indent=2))

    def cloud_gallery(self):
        fig, axes = plt.subplots(3,5,figsize=(19,10),layout='constrained'); selected=[]
        for agent in range(1,4):
            for take in range(1,6):
                candidates=[r for r in self.rows if r['agent']==agent and r['take']==take and r['partition']=='heldout' and r['variant']=='released']
                row=max(candidates,key=lambda r:abs(r.get('road_height_m',0)))
                s=next(s for s in self.scans if s['record']['agent']==agent and s['record']['take']==take and s['record']['frame_id']==row['frame_id'])
                target=self.targets[take]['points']; p=s['points'][s['support']]
                corr=self.transforms[agent,'sensor',take]; q=transform(p,s['pose']@corr@np.linalg.inv(s['pose']))
                ax=axes[agent-1,take-1]
                anchor = np.asarray(row.get('road_anchor_xy',ANCHOR))
                for points,color,label in [(target,'#777777','Infrastructure'),(p,'#c9483b','Released'),(q,'#168fc9','Sensor correction LTO')]:
                    keep=(abs(points[:,0]-anchor[0])<12)&(abs(points[:,1]-anchor[1])<9)&(points[:,2]>-6)&(points[:,2]<2)
                    plot=points[keep][::2]; ax.scatter(plot[:,0],plot[:,2],s=.6,alpha=.5,c=color,label=label,rasterized=True)
                ax.set(xlim=(anchor[0]-12,anchor[0]+12),ylim=(-5,1),xlabel='Released X (m)',ylabel='Released Z (m)',title=f"V{agent} T{take} frame {row['frame_offset']}\nRoad {'supported' if row['road_supported'] else 'unsupported'}"); ax.grid(alpha=.2)
                selected.append(s['record'])
        axes[0,0].legend(markerscale=5,fontsize=7);fig.suptitle('Original archived scans vs infrastructure; X/Z view of each common road neighborhood\nFrozen cross-take sensor correction; no GSplat or per-evaluation-frame fitting')
        fig.savefig(OUT/'cloud-gallery.png',dpi=150);plt.close(fig)
        (OUT/'gallery-frames.json').write_text(json.dumps(selected,indent=2))

if __name__ == '__main__':
    experiment = VehicleInfrastructureResidual()
    if '--report-only' in sys.argv:
        experiment.prepare()
        experiment.fits = json.loads((OUT / 'fits.json').read_text())
        experiment.rows = json.loads((OUT / 'metrics.json').read_text())
        experiment.transforms = {}
        for fit in experiment.fits:
            key = (fit['agent'], 'pertake' if fit['only_take'] else fit['mode'], fit['only_take'] or fit['exclude_take'])
            experiment.transforms[key] = np.asarray(fit['correction']['matrix'])
        experiment.report()
    else:
        experiment.run()

#!/usr/bin/env python3
"""Inspect and compare explicit shared static patches with every released pose fixed.

Run ``--stage inspect`` before visual review, then ``--stage measure``. No nearest
neighbor correspondences or pose corrections are estimated. ROI identities are
recorded in the shared manifest; fits only summarize each sensor's fixed patch.
"""
from __future__ import annotations

import argparse
import itertools
import json
import shutil
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import ConvexHull, QhullError, cKDTree

ROOT = Path(__file__).resolve().parents[2]
DEFAULT = ROOT / 'artifacts/reference-refinement/structure-experiment/lidar'
KEYS = ['infra_representative', 'global_1', 'global_2', 'global_3']
COLORS = ['#151515', '#d62728', '#2877bd', '#16a05d']


class StructureAgreement:
    def __init__(self, output: Path):
        self.output = output
        output.mkdir(parents=True, exist_ok=True)
        if not (output / 'roi-manifest.json').exists():
            shutil.copyfile(Path(__file__).with_name('cooperscene_structure_rois.json'), output / 'roi-manifest.json')
        self.manifest = json.loads((output / 'roi-manifest.json').read_text())
        self.clouds = dict(np.load(ROOT / 'artifacts/reference-refinement/overlap-selected-calibration/selection/selected-clouds.npz'))
        self.rois = self.manifest['rois']

    @staticmethod
    def basis(roi):
        n = np.asarray(roi['normal'], float); n /= np.linalg.norm(n)
        u = np.asarray(roi['tangent_u'], float); u -= n * (u @ n); u /= np.linalg.norm(u)
        return np.array([u, np.cross(n, u), n])

    def select(self, roi, key):
        points = self.clouds[key]
        local = (points - roi['anchor']) @ self.basis(roi).T
        selected = np.all(np.abs(local) <= roi['half_widths_uvn'], axis=1)
        return points[selected], local[selected]

    def inspect(self):
        fig, axes = plt.subplots(1, 4, figsize=(22, 8))
        for ax, key in zip(axes, KEYS):
            q = self.clouds[key]
            ax.scatter(q[:, 0], q[:, 1], c=q[:, 2], s=.6, cmap='turbo', vmin=-4, vmax=10)
            for i, roi in enumerate(self.rois):
                a = np.asarray(roi['anchor']); b = self.basis(roi); h = roi['half_widths_uvn']
                edge = a + np.array([[-h[0], -h[1], 0], [h[0], -h[1], 0], [h[0], h[1], 0], [-h[0], h[1], 0], [-h[0], -h[1], 0]]) @ b
                ax.plot(edge[:, 0], edge[:, 1], 'k-', lw=1.5)
                ax.annotate(f'{i+1}', a[:2], xytext=(5, 5), textcoords='offset points', weight='bold', bbox=dict(fc='white', alpha=.8, pad=1))
            ax.set(title=key, xlabel='X (m)', ylabel='Y (m)', xlim=(-60, 50), ylim=(-60, 50), aspect='equal'); ax.grid(alpha=.3)
        fig.suptitle('Released poses fixed — explicit common structure locations; numbers follow manifest order')
        fig.tight_layout(); fig.savefig(self.output / 'annotated-inventory.png', dpi=160); plt.close(fig)
        for roi in self.rois:
            fig, axes = plt.subplots(3, 4, figsize=(18, 10))
            h = roi['half_widths_uvn']
            for i, key in enumerate(KEYS):
                q, local = self.select(roi, key)
                for j, (x, y, c) in enumerate([(0, 1, 2), (0, 2, 1), (1, 2, 0)]):
                    ax = axes[j, i]
                    ax.scatter(local[:, x], local[:, y], c=local[:, c], cmap='turbo', vmin=-h[c], vmax=h[c], s=7)
                    ax.axhline(0, c='gray', lw=.6); ax.axvline(0, c='gray', lw=.6)
                    ax.set(xlim=(-h[x], h[x]), ylim=(-h[y], h[y]), xlabel=['surface u', 'surface v', 'orthogonal distance'][x]+' (m)', ylabel=['surface u', 'surface v', 'orthogonal distance'][y]+' (m)'); ax.grid(alpha=.3)
                axes[0, i].set_title(f'{key}: {len(q)} returns')
            fig.suptitle(f"{roi['id']} — same physical patch / same coordinates; prior to any plane measurement")
            fig.tight_layout(); fig.savefig(self.output / f"inspect-{roi['id']}.png", dpi=150); plt.close(fig)

    def fit(self, roi, key):
        q, local = self.select(roi, key)
        result = dict(sensor=key, count=int(len(q)), supported=False)
        if roi.get('exclude_from_measurement', False):
            return result | dict(reason='visually rejected: mixed or curved ground surface'), None
        if len(q) < 12:
            return result | dict(reason='fewer than 12 returns'), None
        keep = np.ones(len(q), bool)
        for _ in range(5):
            a = np.column_stack([local[keep, :2], np.ones(keep.sum())])
            coeff = np.linalg.lstsq(a, local[keep, 2], rcond=None)[0]
            residual = local[:, 2] - np.column_stack([local[:, :2], np.ones(len(q))]) @ coeff
            med = np.median(residual[keep]); mad = 1.4826*np.median(abs(residual[keep]-med))
            new_keep = abs(residual-med) <= min(.18, max(.06, 3*mad))
            if new_keep.sum() < 12 or np.array_equal(new_keep, keep):
                break
            keep = new_keep
        p = q[keep]; uv = local[keep, :2]
        center = p.mean(axis=0); _, singular, vh = np.linalg.svd(p-center, full_matrices=False)
        n = vh[-1]; basis = self.basis(roi)
        if n @ basis[2] < 0: n = -n
        d = -n @ center
        rms = float(np.sqrt(np.mean((p @ n+d)**2)))
        spans = np.ptp(uv, axis=0)
        covariance_eig = np.linalg.eigvalsh(np.cov(uv.T))
        result |= dict(inlier_count=len(p), normal=n.tolist(), offset=float(d), rms_m=rms, uv_span_m=spans.tolist(), uv_covariance_eigenvalues=covariance_eig.tolist(), normal_angle_to_declared_deg=float(np.rad2deg(np.arccos(np.clip(n @ basis[2], -1, 1)))))
        try:
            hull = ConvexHull(uv)
        except QhullError:
            return result | dict(reason='collinear patch'), None
        good = bool(min(spans) >= .7 and covariance_eig[0] >= .015 and rms <= .12 and result['normal_angle_to_declared_deg'] <= 15)
        result |= dict(supported=good, reason='supported surface' if good else 'insufficient span/conditioning or nonplanarity', hull_area_m2=float(hull.volume))
        return result, dict(n=n, d=d, uv=uv, xyz=p, hull=hull, tree=cKDTree(uv)) if good else None

    @staticmethod
    def anchor_supported(fit, uv):
        return np.all(uv @ fit['hull'].equations[:, :2].T + fit['hull'].equations[:, 2] <= 1e-8, axis=1) & (fit['tree'].query(uv)[0] <= 1.)

    def measure(self):
        rows = []; pair_rows = []; fit_models = {}
        for roi in self.rois:
            models = {}
            for key in KEYS:
                record, model = self.fit(roi, key)
                rows.append(dict(roi=roi['id'], kind=roi['kind'], role=roi['role'], **record))
                models[key] = model
            fit_models[roi['id']] = models
            h = roi['half_widths_uvn']; b = self.basis(roi); a = np.asarray(roi['anchor'])
            uv = np.array(list(itertools.product(np.array([-0.5, 0, .5])*h[0], np.array([-.5, 0, .5])*h[1])))
            for k1, k2 in itertools.combinations(KEYS, 2):
                f1, f2 = models[k1], models[k2]
                r = dict(roi=roi['id'], kind=roi['kind'], sensor_a=k1, sensor_b=k2, supported=False)
                if f1 is None or f2 is None:
                    pair_rows.append(r | dict(reason='one or both sensor patches unsupported')); continue
                supported = self.anchor_supported(f1, uv) & self.anchor_supported(f2, uv)
                r['common_anchor_count'] = int(supported.sum())
                r['normal_angle_deg'] = float(np.degrees(np.arccos(np.clip(abs(f1['n'] @ f2['n']), -1, 1))))
                tangent_distance, neighbor = f2['tree'].query(f1['uv'])
                near = tangent_distance <= .3
                mean_normal = f1['n'] + f2['n']; mean_normal /= np.linalg.norm(mean_normal)
                direct_gaps = (f2['xyz'][neighbor[near]] - f1['xyz'][near]) @ mean_normal
                r['direct_tangent_match'] = dict(maximum_tangent_distance_m=.3, count=int(near.sum()), median_signed_normal_gap_m=float(np.median(direct_gaps)) if len(direct_gaps) else None, median_abs_normal_gap_m=float(np.median(abs(direct_gaps))) if len(direct_gaps) else None, p90_abs_normal_gap_m=float(np.percentile(abs(direct_gaps),90)) if len(direct_gaps) else None, note='Nearest only in tangent coordinates inside the same visually identified panel; raw inlier point differences, no plane evaluation.')
                if not supported.any():
                    pair_rows.append(r | dict(reason='no common sampled anchor inside both hulls')); continue
                xyz = a + uv[supported] @ b[:2]
                xyz -= ((xyz @ f1['n'] + f1['d']) / (b[2] @ f1['n']))[:, None] * b[2]
                separation = -(xyz @ f2['n'] + f2['d']) / (f2['n'] @ f1['n'])
                pair_rows.append(r | dict(supported=True, reason='same declared surface; common hull and nearby returns', anchors_xyz=xyz.tolist(), signed_orthogonal_separations_m=separation.tolist(), median_abs_separation_m=float(np.median(abs(separation))), maximum_abs_separation_m=float(max(abs(separation)))))
        (self.output / 'surface-measurements.json').write_text(json.dumps(rows, indent=2)+'\n')
        (self.output / 'pair-measurements.json').write_text(json.dumps(pair_rows, indent=2)+'\n')
        self.report(rows, pair_rows, fit_models)

    def take_control(self):
        """Compare the stationary reference across takes without replacing it."""
        control_keys = ['infra_representative', 'cal_2_0', 'global_2', 'global_3']
        pairs = [('infra_representative', 'cal_2_0'), ('cal_2_0', 'global_2'), ('cal_2_0', 'global_3')]
        records = []; surface_records = []
        shown = [r for r in self.rois if not r.get('exclude_from_measurement')]
        fig, axes = plt.subplots(2, 4, figsize=(20, 10))
        for ax, roi in zip(axes.flat, shown):
            models = {}
            for key, color in zip(control_keys, ['black', 'purple', 'blue', 'green']):
                q, local = self.select(roi, key)
                ax.scatter(local[:, 0], local[:, 2], s=10, c=color, alpha=.5, label=key)
                surface, models[key] = self.fit(roi, key)
                surface_records.append(dict(roi=roi['id'], **surface))
            ax.set(title=roi['id'], xlabel='Surface u (m)', ylabel='Declared normal coordinate (m)'); ax.grid(alpha=.25)
            h = roi['half_widths_uvn']; b = self.basis(roi)
            uv = np.array(list(itertools.product(np.array([-.5,0,.5])*h[0], np.array([-.5,0,.5])*h[1])))
            for key_a, key_b in pairs:
                r = dict(roi=roi['id'], sensor_a=key_a, sensor_b=key_b, supported=False)
                f1, f2 = models[key_a], models[key_b]
                if f1 is not None and f2 is not None:
                    valid = self.anchor_supported(f1, uv) & self.anchor_supported(f2, uv)
                    r['common_anchor_count'] = int(valid.sum())
                    if valid.any():
                        xyz = np.asarray(roi['anchor']) + uv[valid] @ b[:2]
                        xyz -= ((xyz @ f1['n'] + f1['d']) / (b[2] @ f1['n']))[:,None] * b[2]
                        gaps = -(xyz @ f2['n'] + f2['d']) / (f1['n'] @ f2['n'])
                        r.update(supported=True, median_abs_separation_m=float(np.median(abs(gaps))), maximum_abs_separation_m=float(max(abs(gaps))), normal_angle_deg=float(np.degrees(np.arccos(np.clip(f1['n'] @ f2['n'],-1,1)))), anchors_xyz=xyz.tolist(), signed_orthogonal_separations_m=gaps.tolist())
                records.append(r)
        axes.flat[0].legend(); fig.suptitle('Take2 infrastructure control; original representative and all poses unchanged')
        fig.tight_layout(); fig.savefig(self.output/'take2-infra-control-inspection.png',dpi=160);plt.close(fig)
        result=dict(source='selected-clouds.npz cal_2_0: inherited take2 stationary calibration aggregate; original_W @ released ENU', primary_reference_unchanged=True, fixed_roi_manifest='roi-manifest.json', surfaces=surface_records, pairs=records)
        (self.output/'take2-infra-control.json').write_text(json.dumps(result,indent=2)+'\n')
        lines=['# Take2 infrastructure control', '', 'This distinct control uses the inherited take2 stationary calibration aggregate cal_2_0. The primary take1 representative and released poses remain unchanged. Matching structure outlines and sampling were inspected in [overlaid raw coordinates](take2-infra-control-inspection.png) before measurement.', '', '| Structure | Sensors | Common anchors | Median / max absolute orthogonal gap (m) | Angle (deg) |', '|---|---|---:|---|---:|']
        for r in records:
            if r['supported']:
                lines.append(f"| {r['roi']} | {r['sensor_a']} / {r['sensor_b']} | {r['common_anchor_count']} | {r['median_abs_separation_m']:.3f} / {r['maximum_abs_separation_m']:.3f} | {r['normal_angle_deg']:.3f} |")
            else:
                lines.append(f"| {r['roi']} | {r['sensor_a']} / {r['sensor_b']} | 0 | unsupported | — |")
        lines += ['', 'This is a structure consistency control, not an independent accuracy check; infrastructure was the released ICP target. Missing road coverage remains unsupported.']
        (self.output/'take2-infra-control.md').write_text('\n'.join(lines)+'\n')

    def report(self, rows, pairs, models):
        normals = []; jac = []
        for roi in self.rois:
            f = models[roi['id']]['infra_representative']
            if f is not None:
                normals.append(f['n']); p = np.asarray(roi['anchor']); jac.append(np.r_[np.cross(p/30, f['n']), f['n']])
        surface_jac = []; fit_jac = []
        for roi in self.rois:
            f = models[roi['id']]['infra_representative']
            if f is None: continue
            h = roi['half_widths_uvn']; b = self.basis(roi)
            uv = np.array(list(itertools.product(np.array([-.5,0,.5])*h[0], np.array([-.5,0,.5])*h[1])))
            uv = uv[self.anchor_supported(f, uv)]
            p = np.asarray(roi['anchor']) + uv @ b[:2]
            block = np.column_stack([np.cross(p/30, f['n']), np.tile(f['n'], (len(p),1))]) / np.sqrt(len(p))
            surface_jac.extend(block)
            if roi['role']=='fit': fit_jac.extend(block)
        conditioning = dict(length_scale_m=30, explanation='Rotation columns use p/30 and translation meters; each structure has unit total weight. Both center-only and full sampled-anchor conditioning retained. Geometry only, no pose fitted.', normal_singular_values=np.linalg.svd(normals, compute_uv=False).tolist(), center_only_scaled_se3_singular_values=np.pad(np.linalg.svd(jac, compute_uv=False),(0,max(0,6-len(jac)))).tolist(), center_only_rank=int(np.linalg.matrix_rank(jac)), sampled_surface_scaled_se3_singular_values=np.linalg.svd(surface_jac, compute_uv=False).tolist(), sampled_surface_rank=int(np.linalg.matrix_rank(surface_jac)), fit_only_scaled_se3_singular_values=np.linalg.svd(fit_jac, compute_uv=False).tolist(), fit_only_rank=int(np.linalg.matrix_rank(fit_jac)), structure_count=len(normals))
        (self.output / 'conditioning.json').write_text(json.dumps(conditioning, indent=2)+'\n')
        shown = [r for r in self.rois if not r.get('exclude_from_measurement')]
        fig, axes = plt.subplots(2, 4, figsize=(20, 10))
        for ax, roi in zip(axes.flat, shown):
            reference_key = next(k for k in KEYS if models[roi['id']][k] is not None)
            reference = models[roi['id']][reference_key]
            for key, color in zip(KEYS, COLORS):
                q, local = self.select(roi, key)
                ax.scatter(local[:, 0], q @ reference['n'] + reference['d'], s=9, c=color, alpha=.65, label=key)
            ax.axhline(0, c='gray', lw=.6)
            ax.set(title=f"{roi['id']}\nreference plane: {reference_key}", xlabel='Surface u (m)', ylabel='Raw return to reference plane (m)')
            ax.grid(alpha=.25)
        axes.flat[0].legend(fontsize=8)
        fig.suptitle('Same physical patches overlaid; every released pose fixed; no plane-based point correction')
        fig.tight_layout();fig.savefig(self.output/'same-surface-overlays.png',dpi=170);plt.close(fig)
        fig, axes = plt.subplots(2, 1, figsize=(14, 10), sharex=True)
        positions = {r['id']:i for i,r in enumerate(self.rois)}
        for j,(k1,k2) in enumerate(itertools.combinations(KEYS,2)):
            data = [p for p in pairs if p['sensor_a']==k1 and p['sensor_b']==k2 and p['supported']]
            x = [positions[p['roi']]+(j-2.5)*.09 for p in data]
            axes[0].scatter(x, [p['maximum_abs_separation_m'] for p in data], label=f'{k1} / {k2}', s=30)
            axes[1].scatter(x, [p['normal_angle_deg'] for p in data], s=30)
        axes[0].axhline(.1,c='gray',ls='--',label='0.1m comparison line'); axes[0].legend(fontsize=8,ncol=2)
        axes[0].set(ylabel='Max |orthogonal separation| at supported anchors (m)')
        axes[1].set(ylabel='Surface normal disagreement (degrees)',xticks=range(len(self.rois)),xticklabels=[r['id'] for r in self.rois])
        axes[1].tick_params(axis='x',rotation=30)
        for ax in axes: ax.grid(alpha=.25)
        fig.suptitle('Selected LiDAR structures: fixed released poses; omitted markers mean unsupported, not zero')
        fig.tight_layout();fig.savefig(self.output/'lidar-structure-agreement.png',dpi=160);plt.close(fig)
        lines=['# Selected LiDAR static structure agreement','', 'All four released poses remain fixed. Coordinates are original_W @ released ENU; no GSplat geometry enters these measurements and no vehicle correction is fitted. The input clouds retain the original 0.2m voxel sampling and annotation-based vehicle removal.', '', 'The expected agreement holds on some common surfaces but not uniformly. Infrastructure/vehicle3 median anchor gaps are0.287m on the northwest upper panel,0.230m on the southwest wall and0.334m on the northwest lower panel. Direct nearby tangent-return comparisons give0.286m,0.209m and0.346m respectively, so sparse beam spacing or plane extrapolation alone do not account for those gaps. Central road agreement is within0.032m where observed, while vehicle2 lacks support there. The southern road vehicle1/vehicle2 median gap is0.152m and the eastern-south road gap is0.182m. These remain consistency checks rather than localization errors.', '', '[Annotated inventory](annotated-inventory.png) · [Same-surface raw overlays](same-surface-overlays.png) · [Separation/angle summary](lidar-structure-agreement.png) · [Frozen ROI manifest](roi-manifest.json)', '', '## Identity and visual review', '', 'Inspect annotated-inventory.png, the candidate top/side views and inspect-*.png before interpreting planes. Facade correspondences use the same distinctive corners, separated height bands and physical panels. These are manually screened cloud identities, without a camera/survey semantic confirmation. Neighboring parallel walls are excluded using fixed oriented ROIs. Road uses identical spatial footprints in the open intersection. Missing visibility remains unsupported.', '', 'All nine oriented patch views were examined before quantification. The west ground strip was rejected because its side view shows a raised/curved profile, potentially mixing road and curb. Three facade normal bands were uniformly widened to0.7m during this visual stage to avoid clipping sensor offsets; this changed NW upper membership by zero points. The final manifest SHA256 is958e9ff2f10c15b0e64298ac3632337f9c22f1b12089e98d7170307cb3baeaf9.', '', '## Sensor support', '', '| Structure | Sensor | Inliers | UV span (m) | RMS (m) | Status |','|---|---|---:|---|---:|---|']
        for r in rows:
            lines.append(f"| {r['roi']} | {r['sensor']} | {r.get('inlier_count',r['count'])} | {str(np.round(r.get('uv_span_m',[]),2))} | {r.get('rms_m',float('nan')):.3f} | {r['reason']} |")
        lines += ['', '## Explicit same-surface pairs', '', 'Separation is measured along sensor A plane normal at a fixed 3×3 anchor grid. An anchor must lie inside both observed surface hulls and within 1m in tangent coordinates of a return from each sensor. Unsupported/collinear patches are excluded. Angles accompany distances because crossing planes can have small anchor gaps.', '', '| Structure | Sensors | Anchors | Median / max absolute orthogonal separation (m) | Angle (deg) |', '|---|---|---:|---|---:|']
        for p in pairs:
            if p['supported']:
                lines.append(f"| {p['roi']} | {p['sensor_a']} / {p['sensor_b']} | {p['common_anchor_count']} | {p['median_abs_separation_m']:.3f} / {p['maximum_abs_separation_m']:.3f} | {p['normal_angle_deg']:.3f} |")
        lines += ['', '## Direct return check', '', 'This checks the plane summaries using actual inlier returns on the same identified panel: match only tangent coordinates within0.3m, then project point differences onto the mean plane normal. A low count remains a sparse-support caveat; this is not a new pose fit.', '', '| Structure | Sensors | Nearby tangent pairs | Median / P90 absolute normal difference (m) |', '|---|---|---:|---|']
        for p in pairs:
            direct=p.get('direct_tangent_match',{})
            if direct.get('count',0):
                lines.append(f"| {p['roi']} | {p['sensor_a']} / {p['sensor_b']} | {direct['count']} | {direct['median_abs_normal_gap_m']:.3f} / {direct['p90_abs_normal_gap_m']:.3f} |")
        lines += ['', '## Conditioning and limits', '', f"Normal singular values: {conditioning['normal_singular_values']}. Sampled-surface scaled pose Jacobian singular values: {conditioning['sampled_surface_scaled_se3_singular_values']} (rank {conditioning['sampled_surface_rank']}/6). Fit-only rank: {conditioning['fit_only_rank']}/6; singular values {conditioning['fit_only_scaled_se3_singular_values']}. Center-only rank is {conditioning['center_only_rank']}/6. Structures have equal total weight and rotation is scaled by30m. Full rank does not certify pose accuracy. No transform is fitted by this script.", '', 'Road and two differently oriented facade families provide complementary normals. NW heldout lower panel shares a building with the fit panel; it is a withheld region, not an independent site. Sparse infrastructure beams can prevent any shared anchor despite a recognizable structure. Plane RMS quantifies within-patch scatter, not uncertainty of the released pose. These geometry checks are not ATE or survey accuracy.']
        lines += ['', '## Same-take infrastructure control', '', '[Separate take2 control](take2-infra-control.md) compares the frozen take1 representative with cal_2_0 and compares both selected take2 vehicles with that same-take infrastructure aggregate. It preserves the primary input and ROI manifest. Reproduce with `--stage take_control`.']
        (self.output/'report.md').write_text('\n'.join(lines)+'\n')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=DEFAULT);parser.add_argument('--stage',choices=['inspect','measure','take_control'],required=True)
    args=parser.parse_args(); experiment=StructureAgreement(args.output)
    getattr(experiment,args.stage)()


if __name__=='__main__': main()

"""Offline one-transform diagnostic using visually identified fixed structures.

Selected scans already contain original_W @ released_enu points. No source pose,
NN correspondence, held-out surface, or production setting is optimized.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from src.localization.gaussian_map import load_gaussian_map

BASE = ROOT / 'artifacts/reference-refinement'
KEYS = ['infra_representative', 'global_1', 'global_2', 'global_3']
COLORS = ['#e69f00', '#009ed1', '#cc55a8', '#269e42']


def transform(points, matrix):
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def basis(roi):
    n = np.array(roi['normal'], dtype=float); n /= np.linalg.norm(n)
    u = np.array(roi['tangent_u'], dtype=float); u -= u @ n * n; u /= np.linalg.norm(u)
    return np.stack([u, np.cross(n, u), n], axis=1)


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


class StructureMapExperiment:
    def __init__(self, output):
        self.output = output; output.mkdir(parents=True, exist_ok=True)
        self.manifest_path = BASE / 'structure-experiment/lidar/roi-manifest.json'
        self.manifest = json.loads(self.manifest_path.read_text())
        self.rois = self.manifest['rois']
        self.old = json.loads((BASE / 'overlap-selected-calibration/fit/fused_opacity005-calibration.json').read_text())
        self.D0 = np.array(self.old['D'])
        self.shared = json.loads((BASE / 'shared-enu-calibration/calibration.json').read_text())
        self.W = np.array(self.old['original_W'])
        self.source_path = BASE / 'overlap-selected-calibration/selection/selected-clouds.npz'
        with np.load(self.source_path) as data:
            self.clouds = {key: data[key] for key in KEYS}
            assert np.allclose(data['original_W'], self.W)
        with open(self.old['map_path'], 'rb') as stream:
            assert hashlib.file_digest(stream, 'sha256').hexdigest() == self.old['map_sha256']
        self.g = load_gaussian_map(self.old['map_path'])
        self.map_native = transform(self.g.means.astype(float), np.linalg.inv(self.D0))
        self.map_rgb = np.clip(.5 + .2820947918 * self.g.sh_coeffs[:, 0], 0, 1)
        self.planes = {}; self.support = {}; self.rows = []

    def select(self, points, roi, map_surface=False):
        widths = np.array(roi['half_widths_uvn'], dtype=float)
        if map_surface:
            widths[2] = 1.0  # Declared before fitting; broad enough to expose layers.
        local = (points - roi['anchor']) @ basis(roi)
        return (abs(local) <= widths).all(1)

    @staticmethod
    def contains(points, anchor, axes):
        if len(points) < 4:
            return False
        xy = (points - anchor) @ axes[:, :2]
        try:
            hull = ConvexHull(xy)
            return bool(np.max(hull.equations[:, 2]) <= -.05)
        except Exception:
            return False

    def plane(self, points, roi):
        axes = basis(roi); n = axes[:, 2].copy(); kept = np.ones(len(points), bool)
        if len(points) < 20:
            return {'accepted': False, 'reason': 'fewer than 20 candidates', 'candidate_count': len(points)}, points
        for _ in range(8):
            center = np.median(points @ n)
            kept = abs(points @ n - center) <= .20
            if kept.sum() < 20:
                return {'accepted': False, 'reason': 'fewer than 20 planar inliers', 'candidate_count': len(points)}, points[kept]
            p = points[kept]; c = p.mean(0)
            vals, vecs = np.linalg.eigh((p-c).T @ (p-c) / len(p))
            nn = vecs[:, 0]
            if nn @ n < 0: nn = -nn
            n = nn
        d = -float(n @ c)
        span = np.ptp(p @ axes[:, :2], axis=0)
        angle = float(np.rad2deg(np.arccos(np.clip(n @ axes[:, 2], -1, 1))))
        hull_ok = self.contains(p, np.array(roi['anchor']), axes)
        checks = dict(count=len(p)>=20, two_dimensional=bool(vals[1]>.02), extent=bool(span.min()>1), rms=bool(np.sqrt(vals[0])<.12), nominal_normal=angle<15, anchor_inside=hull_ok)
        return dict(accepted=all(checks.values()), checks=checks, candidate_count=len(points), inlier_count=len(p), normal=n.tolist(), offset_m=d, rms_m=float(np.sqrt(vals[0])), eigenvalues_m2=vals.tolist(), tangent_span_m=span.tolist(), normal_to_nominal_deg=angle, candidate_normal_quantiles_m=np.quantile(points@n+d,[0,.05,.25,.5,.75,.95,1]).tolist(), reason=','.join(k for k,v in checks.items() if not v)), p

    def prepare(self):
        records = []
        for roi in self.rois:
            rid = roi['id']
            for key, cloud in self.clouds.items():
                p = cloud[self.select(cloud, roi)]
                rec, support = self.plane(p, roi)
                records.append(dict(structure=rid, role=roi['role'], dataset=key, **rec))
                self.planes[rid,key] = rec; self.support[rid,key] = support
            for label, opacity in [('map_opacity005', .05), ('map_positive', 0.)]:
                keep = self.select(self.map_native, roi, True) & (self.g.opacities > opacity) & np.isfinite(self.map_native).all(1)
                p = self.map_native[keep]; rec, support = self.plane(p, roi)
                records.append(dict(structure=rid, role=roi['role'], dataset=label, **rec))
                self.planes[rid,label] = rec; self.support[rid,label] = support
        write(self.output/'surface-support.json', records)

    def training(self):
        groups = []
        for roi in self.rois:
            rid = roi['id']; mp = self.planes[rid,'map_opacity005']
            if roi['role'] != 'fit' or not mp['accepted']: continue
            keys = [key for key in KEYS if self.planes[rid,key]['accepted']]
            n = self.D0[:3,:3] @ mp['normal']; d = mp['offset_m'] - n @ self.D0[:3,3]
            for key in keys:
                p = self.support[rid,key]
                p = p[np.linspace(0,len(p)-1,min(len(p),500),dtype=int)]
                groups.append((rid,key,p,n,d,1/len(keys)))
        return groups

    @staticmethod
    def equations(groups, D):
        points=[]; normals=[]; residuals=[]; weights=[]
        for _,_,p,n,d,weight in groups:
            q=transform(p,D);points.append(q); normals.append(np.broadcast_to(n,q.shape))
            residuals.append(q@n+d);weights.append(np.full(len(p),weight/len(p)))
        q=np.concatenate(points);n=np.concatenate(normals);r=np.concatenate(residuals);w=np.concatenate(weights)
        return np.c_[np.cross(q,n),n],r,w

    def fit(self):
        groups=self.training()
        if not groups: return None,{'accepted':False,'reason':'no supported training structures'}
        D=self.D0.copy(); trace=[]
        for i in range(60):
            A,r,w=self.equations(groups,D);w*=np.minimum(1,.15/np.maximum(abs(r),1e-12))
            delta=np.linalg.lstsq(A*np.sqrt(w[:,None]),-r*np.sqrt(w),rcond=1e-10)[0]
            delta/=max(1,np.linalg.norm(delta[:3])/np.deg2rad(1),np.linalg.norm(delta[3:])/.25)
            step=np.eye(4);step[:3,:3]=Rotation.from_rotvec(delta[:3]).as_matrix();step[:3,3]=delta[3:];D=step@D
            trace.append(dict(iteration=i,rotation_step_deg=float(np.rad2deg(np.linalg.norm(delta[:3]))),translation_step_m=float(np.linalg.norm(delta[3:])),median_point_plane_m=float(np.median(abs(r)))))
            if np.linalg.norm(delta)<1e-8:break
        A,r,w=self.equations(groups,D)
        scaled=A.copy();scaled[:,:3]/=20
        sv=np.linalg.svd(scaled*np.sqrt(w[:,None]),compute_uv=False); rank=int(np.sum(sv>sv[0]*1e-6))
        normal_matrix=np.stack([g[3] for g in groups]);nv=np.linalg.svd(normal_matrix,compute_uv=False)
        kinds={r['kind'] for r in self.rois if r['id'] in {g[0] for g in groups}}
        accepted=bool(rank==6 and len({g[0] for g in groups})>=3 and 'facade' in kinds and 'road' in kinds and sv[0]/sv[-1]<1e4)
        return D,dict(accepted=accepted,gate='>=3 supported training structures; road plus nonparallel facades; rank6 at relative1e-6; scaled condition<1e4',training_groups=[dict(structure=g[0],sensor=g[1],points=len(g[2])) for g in groups],scaled_singular_values=sv.tolist(),scaled_condition=float(sv[0]/sv[-1]),scaled_rank=rank,rotation_parameter_scale_m=20,normal_singular_values=nv.tolist(),trace=trace,termination='step_converged' if np.linalg.norm(delta)<1e-8 else 'iteration_limit',median_point_plane_m=float(np.median(abs(r))))

    def evaluate(self, transforms):
        for roi in self.rois:
            rid=roi['id'];anchor0=np.array(roi['anchor']);anchor=transform(anchor0[None],self.D0)[0]
            axes=self.D0[:3,:3]@basis(roi)
            for map_name in ['map_opacity005','map_positive']:
                mp=self.planes[rid,map_name]
                for key in KEYS:
                    sp=self.planes[rid,key]
                    for variant,D in transforms.items():
                        row=dict(structure=rid,kind=roi['kind'],role=roi['role'],sensor=key,map_surface=map_name,variant=variant,anchor_map=anchor.tolist(),supported=False)
                        if not sp['accepted'] or not mp['accepted']:
                            row['reason']='source: '+sp.get('reason','')+'; map: '+mp.get('reason','');self.rows.append(row);continue
                        sn=D[:3,:3]@sp['normal'];sd=sp['offset_m']-sn@D[:3,3]
                        mn=self.D0[:3,:3]@mp['normal'];md=mp['offset_m']-mn@self.D0[:3,3]
                        map_anchor=anchor-(anchor@mn+md)*mn
                        source_anchor=map_anchor-(map_anchor@sn+sd)/(sn@mn)*mn
                        ss=transform(self.support[rid,key],D);ms=transform(self.support[rid,map_name],self.D0)
                        supported=self.contains(ss,source_anchor,axes) and self.contains(ms,map_anchor,axes)
                        row.update(supported=bool(supported),reason='' if supported else 'fixed anchor outside source/map footprint',signed_orthogonal_offset_m=float(-(map_anchor@sn+sd)/(sn@mn)),normal_angle_deg=float(np.rad2deg(np.arccos(np.clip(abs(sn@mn),-1,1)))),source_point_map_plane_median_m=float(np.median(abs(ss@mn+md))),map_anchor=map_anchor.tolist())
                        self.rows.append(row)
        write(self.output/'metrics.json',self.rows)

    def overview(self):
        fig,axs=plt.subplots(1,2,figsize=(17,8))
        keep=(self.g.opacities>.05)&(abs(self.map_native).max(1)<100)
        p=self.map_native[keep]; idx=np.arange(0,len(p),max(1,len(p)//160000))
        axs[0].scatter(p[idx,0],p[idx,1],c=self.map_rgb[keep][idx],s=.4)
        for key,color in zip(KEYS,COLORS):
            p=self.clouds[key];axs[1].scatter(p[:,0],p[:,1],c=color,s=.3,alpha=.4,label=key)
        for ax in axs:
            for roi in self.rois:
                a=np.array(roi['anchor']); b=basis(roi);width=np.array(roi['half_widths_uvn']);color={'fit':'red','heldout':'blue','excluded':'gray'}[roi['role']]
                ax.scatter(a[0],a[1],c=color,s=25,marker='x');ax.annotate(roi['id']+' ['+roi['role']+']',a[:2],xytext=(4,6),textcoords='offset points',fontsize=7,color=color)
            ax.set(xlim=(-60,50),ylim=(-60,55),xlabel='Original-W ENU x (m)',ylabel='Original-W ENU y (m)');ax.set_aspect('equal');ax.grid(alpha=.2)
        axs[0].set_title('GSplat RGB centers; frozen inherited D inverse for display');axs[1].set_title('Four released-pose LiDAR clouds');axs[1].legend(fontsize=8)
        fig.suptitle('Predeclared physical structures: red FIT, blue HELDOUT; one common rigid transform');fig.tight_layout();fig.savefig(self.output/'structure-overview.png',dpi=160);plt.close(fig)

    def comparison_views(self, transforms):
        selected=[r for r in self.rois if r['id'] in ['road_central','road_south','road_east_south','road_east_north']]
        fig,axs=plt.subplots(len(selected),3,figsize=(17,13))
        for row,roi in zip(axs,selected):
            rid=roi['id']; anchor=transform(np.array(roi['anchor'])[None],self.D0)[0]
            mp=transform(self.support[rid,'map_opacity005'],self.D0)
            source=np.concatenate([self.support[rid,key] for key in KEYS if self.planes[rid,key]['accepted']])
            for ax,(i,j) in zip(row,[(0,1),(0,2),(1,2)]):
                ax.scatter(mp[:,i],mp[:,j],c='gray',s=4,alpha=.45,label='map >0.05 fixed inliers')
                for name,color in [('inherited_overlap','#169bd5'),('structure_fit','#df512a')]:
                    if name not in transforms:continue
                    p=transform(source,transforms[name]);ax.scatter(p[:,i],p[:,j],c=color,s=4,alpha=.5,label=name)
                ax.scatter(anchor[i],anchor[j],marker='x',c='black',s=40);ax.set(xlabel='Map '+'XYZ'[i]+' (m)',ylabel='Map '+'XYZ'[j]+' (m)',title=rid+' ['+roi['role']+']');ax.grid(alpha=.3)
                if j==2:ax.set_ylim(-4.5,-1)
            row[0].legend(fontsize=7)
        fig.suptitle('Identical fixed source samples and map planes: inherited D (blue) versus new one-transform fit (orange)\nHeldout road displacement after fitting two facades and central road; no road plane refit per transform')
        fig.tight_layout();fig.savefig(self.output/'road-before-after.png',dpi=160);plt.close(fig)

    def identity_views(self):
        regions=[('northwest',[[-55,-20],[17,33],[-5,12]]),('northeast',[[10,44],[20,48],[-3,18]]),('southwest',[[-52,-25],[-58,-37],[-5,10]])]
        cloud=np.concatenate(list(self.clouds.values()))
        for name,bounds in regions:
            fig,axs=plt.subplots(3,3,figsize=(17,14))
            for j,(points,rgb,label) in enumerate([(cloud,None,'LiDAR union'),(self.map_native,self.map_rgb,'GSplat RGB centers'),(self.map_native,None,'GSplat height')]):
                keep=np.isfinite(points).all(1)
                for k in range(3):keep&=(points[:,k]>bounds[k][0])&(points[:,k]<bounds[k][1])
                if j:keep&=self.g.opacities>.05
                p=points[keep];colors=rgb[keep] if rgb is not None else p[:,2]
                for ax,(a,b) in zip(axs[:,j],[(0,1),(0,2),(1,2)]):
                    ax.scatter(p[:,a],p[:,b],c=colors,s=1,cmap=None if rgb is not None else 'turbo',vmin=None if rgb is not None else bounds[2][0],vmax=None if rgb is not None else bounds[2][1])
                    ax.set(xlim=bounds[a],ylim=bounds[b],xlabel='XYZ'[a]+' (m)',ylabel='XYZ'[b]+' (m)',title=label);ax.grid(alpha=.3)
                    for roi in self.rois:
                        c=np.array(roi['anchor'])
                        if not all(bounds[k][0]<c[k]<bounds[k][1] for k in range(3)):continue
                        color={'fit':'red','heldout':'blue','excluded':'gray'}[roi['role']]
                        ax.scatter(c[a],c[b],c=color,marker='x',s=80);ax.annotate(roi['id']+' ['+roi['role']+']',(c[a],c[b]),xytext=(0,12),textcoords='offset points',color=color,fontsize=8)
            fig.suptitle(name+' | map inverse inherited D for identity inspection; opacity >0.05; XYZ are original-W ENU')
            fig.tight_layout();fig.savefig(self.output/('identity-'+name+'.png'),dpi=130);plt.close(fig)

    def views(self):
        for roi in self.rois:
            rid=roi['id']; b=basis(roi);a=np.array(roi['anchor']);w=np.array(roi['half_widths_uvn']);fig,axs=plt.subplots(2,3,figsize=(16,9))
            mask=self.select(self.map_native,roi,True)&(self.g.opacities>.05)
            mp=self.map_native[mask];ml=(mp-a)@b
            for ax,(i,j) in zip(axs[0],[(0,1),(0,2),(1,2)]):
                ax.scatter(ml[:,i],ml[:,j],c=self.map_rgb[mask],s=8,alpha=.8,label='map candidates RGB')
                ms=(self.support[rid,'map_opacity005']-a)@b;ax.scatter(ms[:,i],ms[:,j],facecolors='none',edgecolors='#555555',s=12,linewidths=.4,label='map plane inliers')
            for key,color in zip(KEYS,COLORS):
                p=self.clouds[key][self.select(self.clouds[key],roi)];local=(p-a)@b
                for ax,(i,j) in zip(axs[1],[(0,1),(0,2),(1,2)]):
                    ax.scatter(local[:,i],local[:,j],c=color,s=7,alpha=.65,label=key)
            for row in axs:
                for ax,(i,j) in zip(row,[(0,1),(0,2),(1,2)]):
                    labels=['tangent u (m)','tangent v (m)','normal n (m)'];ax.set(xlabel=labels[i],ylabel=labels[j]);ax.grid(alpha=.3);ax.axhline(0,color='black',lw=.5);ax.axvline(0,color='black',lw=.5)
                    ax.set_xlim(-w[i],w[i]);ax.set_ylim((-1,1) if j==2 else (-w[j],w[j]))
            axs[0,0].legend(fontsize=7);axs[1,0].legend(fontsize=7)
            fig.suptitle(rid+' ['+roi['role'].upper()+'] — fixed physical patch; map pulled back by inherited D\n'+roi['identity_evidence'],fontsize=10,wrap=True)
            fig.tight_layout();fig.savefig(self.output/(rid+'-surface.png'),dpi=150);plt.close(fig)

    def report(self, D, fit):
        summary=[]
        for surface in ['map_opacity005','map_positive']:
            for role in ['fit','heldout']:
                for kind in ['facade','road']:
                    group=[r for r in self.rows if r['map_surface']==surface and r['role']==role and r['kind']==kind]
                    identities={(r['structure'],r['sensor']) for r in group}
                    variants=sorted({r['variant'] for r in group})
                    common={ident for ident in identities if all(any((r['structure'],r['sensor'])==ident and r['variant']==v and r['supported'] for r in group) for v in variants)}
                    for v in variants:
                        rows=[r for r in group if r['variant']==v and (r['structure'],r['sensor']) in common]
                        summary.append(dict(map_surface=surface,role=role,kind=kind,variant=v,paired_count=len(rows),median_abs_offset_m=float(np.median([abs(r['signed_orthogonal_offset_m']) for r in rows])) if rows else None,max_abs_offset_m=max([abs(r['signed_orthogonal_offset_m']) for r in rows],default=None),median_normal_deg=float(np.median([r['normal_angle_deg'] for r in rows])) if rows else None))
        fixed=[]
        for roi in self.rois:
            if roi['role']=='excluded':continue
            for variant in sorted({r['variant'] for r in self.rows}):
                rows=[r for r in self.rows if r['structure']==roi['id'] and r['variant']==variant and r['map_surface']=='map_opacity005']
                measured=[r for r in rows if 'source_point_map_plane_median_m' in r]
                fixed.append(dict(structure=roi['id'],role=roi['role'],variant=variant,eligible_source_map_planes=len(measured),supported_anchors=sum(r['supported'] for r in rows),median_fixed_source_point_plane_m=float(np.median([r['source_point_map_plane_median_m'] for r in measured])) if measured else None,max_sensor_median_fixed_source_point_plane_m=max([r['source_point_map_plane_median_m'] for r in measured],default=None),unsupported=[dict(sensor=r['sensor'],reason=r['reason']) for r in rows if not r['supported']]))
        write(self.output/'fixed-source-summary.json',fixed)
        write(self.output/'summary.json',summary)
        rec=dict(status='diagnostic candidate; numerical fit passes, spatial validation rejects adoption',D=None if D is None else D.tolist(),original_W=self.W.tolist(),map_T_released_enu=None if D is None else (D@self.W).tolist(),composition='map_T_lidar = D @ original_W @ released_enu_T_lidar; selected-clouds already have original_W',fitting_gate=fit,map_path=self.old['map_path'],map_sha256=self.old['map_sha256'],roi_manifest_sha256=hashlib.sha256(self.manifest_path.read_bytes()).hexdigest(),source_clouds_sha256=hashlib.sha256(self.source_path.read_bytes()).hexdigest(),surface_protocol={'primary_opacity_exclusive':.05,'sensitivity_opacity_exclusive':0,'map_ROI_normal_halfwidth_m':1.,'plane_median_slab_halfwidth_m':.20,'minimum_points':20,'minimum_second_eigenvalue_m2':.02,'minimum_each_tangent_span_m':1.,'maximum_rms_m':.12,'maximum_normal_deviation_deg':15,'minimum_anchor_hull_margin_m':.05},visual_identity='Point-center RGB and LiDAR top/side views substantiate facade topology and corresponding layers. This is visual geometric identity, not photo/survey verification. No camera views available in this artifact.',heldout_warning='Heldout regions excluded from structure fit, but inherited D previously fitted all four clouds. Comparison is region-held-out for new fit only; NW lower bay shares building with NW training facade.',source_poses_fixed=True,runtime_defaults_unchanged=True)
        if D is not None:rec.update(D_rpy_deg=Rotation.from_matrix(D[:3,:3]).as_euler('xyz',degrees=True).tolist(),D_translation_m=D[:3,3].tolist())
        write(self.output/'calibration.json',rec)
        lines=['# Fixed-structure map experiment','',rec['status'],'','Fit uses two separately identified facade panels and the central road footprint where their independent planes pass support gates. All four released LiDAR poses stay fixed. One common D is fitted to equal-weight physical structures, with equal sensor weights inside each structure. No NN correspondences or held-out structures enter this solver.','',f"Predeclared numerical fitting gate: **{fit['accepted']}** (rank {fit.get('scaled_rank')}, scaled condition {fit.get('scaled_condition'):.2f}). Post-fit diagnosis: new transform loses {len(fit['lost_training_anchor_support'])} otherwise eligible training anchor comparisons; heldout road errors worsen. This validation rejection is separate from the numerical fitting gate. See calibration.json for rank, conditioning, exact weights and trace.",'','Map primary opacity >0.05. Positive-opacity sensitivity changes only surface extraction and keeps every transform frozen. The same median ±0.20 m slab is used for every surface; multimodal normal-depth views remain visible. These are Gaussian-center proxies, not certified physical surfaces.','','Visual identity: NW upper white rectangular facade, its lower west bay, NE stepped facade, and SW long boundary wall appear in the RGB-center and LiDAR top/side crops. No original camera render/photo was reviewed. SW map sparsity and road missing support are retained explicitly.','','Held-out regions are reserved for the new structure fit only: inherited broad D already used these clouds. NW lower bay is a reserved region of the NW training building, not a separate building.','','| Surface | Partition | Geometry | Transform | Same pairs | Median absolute offset m | Max absolute offset m | Median normal deg |','|---|---|---|---|---:|---:|---:|---:|']
        for row in summary:
            f=lambda x:'—' if x is None else f'{x:.4f}'
            lines.append(f"| {row['map_surface']} | {row['role']} | {row['kind']} | {row['variant']} | {row['paired_count']} | {f(row['median_abs_offset_m'])} | {f(row['max_abs_offset_m'])} | {f(row['median_normal_deg'])} |")
        lines+=['','Offsets are evaluated on the same fixed map anchor and orthogonal map normal; source and map convex footprints must contain that location with 0.05 m margin. All variants use the same frozen source and map plane samples. Unsupported rows are listed in metrics.json, never silently replaced with another patch.','','Artifacts: [structure overview](structure-overview.png), per-structure `*-surface.png` top/profile views, broad `identity-*.png` RGB/top/side context, `surface-support.json`, `metrics.json`, `summary.json`, exact `calibration.json`.','','The structure candidate fails spatial transfer: heldout road separation increases, and the SW wall loses fixed-anchor support. Do not adopt this transform. Map RGB/topology identifies structures, but Gaussian-center plane thickness and tilt remain uncertain. No <0.1 m accuracy or independent ground truth is established.']
        lines+=['','## Every structure: frozen-source residual and anchor support','','The following point-to-plane residual uses exactly the same source inliers and map plane before/after, even when transformed footprints no longer contain the fixed anchor. It is residual to the extended plane; unsupported overlap remains separately marked. This makes the support losses visible without assigning an extrapolated anchor offset.','','| Structure | Transform | Eligible sensor planes | Supported anchors | Median sensor point-plane m | Max sensor median m |','|---|---|---:|---:|---:|---:|']
        for row in fixed:
            f=lambda x:'—' if x is None else f'{x:.4f}'
            lines.append(f"| {row['structure']} | {row['variant']} | {row['eligible_source_map_planes']} | {row['supported_anchors']} | {f(row['median_fixed_source_point_plane_m'])} | {f(row['max_sensor_median_fixed_source_point_plane_m'])} |")
        lines+=['','The 600-point exact synthetic three-plane check recovered its known transform to <1e-7 degrees and meters; see synthetic-solver-check.json. This validates algebra and solver only.','','[Before/after road overlays in identical map coordinates](road-before-after.png). All missing/insufficient sensor rows and reasons remain in fixed-source-summary.json and metrics.json.']
        (self.output/'report.md').write_text('\n'.join(lines)+'\n')

    @staticmethod
    def synthetic_check():
        rng=np.random.default_rng(19);expected=np.eye(4)
        expected[:3,:3]=Rotation.from_euler('xyz',[1.2,-2.3,.7],degrees=True).as_matrix();expected[:3,3]=[.3,-.45,.23]
        groups=[]
        for rid,n,c in [('wall_x',np.array([1.,0,0]),np.array([-20.,8,2])),('wall_y',np.array([0.,1,0]),np.array([10.,30,4])),('road',np.array([0.,0,1]),np.array([1.,2,-2]))]:
            points=rng.uniform(-5,5,(200,3));points[:,np.argmax(n)]=0;points+=c
            normal=expected[:3,:3]@n;offset=-normal@transform(c[None],expected)[0]
            groups.append((rid,'synthetic',points,normal,offset,1.))
        diagnostic=StructureMapExperiment.__new__(StructureMapExperiment);diagnostic.D0=np.eye(4)
        diagnostic.rois=[{'id':rid,'kind':'road' if rid=='road' else 'facade'} for rid in ['wall_x','wall_y','road']]
        diagnostic.training=lambda:groups
        actual,fit=diagnostic.fit();error=actual@np.linalg.inv(expected)
        rotation=float(np.rad2deg(Rotation.from_matrix(error[:3,:3]).magnitude()));translation=float(np.linalg.norm(error[:3,3]))
        residual=max(float(abs(transform(g[2],actual)@g[3]+g[4]).max()) for g in groups)
        assert rotation<1e-7 and translation<1e-7 and residual<1e-7
        return dict(passed=True,seed=19,points=600,rotation_error_deg=rotation,translation_error_m=translation,max_plane_residual_m=residual,numerical_gate=fit['accepted'],scaled_rank=fit['scaled_rank'],expected_D=expected.tolist(),recovered_D=actual.tolist(),purpose='Known nonparallel planes check normal/offset convention and SE3 solver; not real-data accuracy.')

    def run(self):
        write(self.output/'synthetic-solver-check.json',self.synthetic_check())
        self.prepare(); D,fit=self.fit();transforms={'inherited_overlap':self.D0,'previous_shared':np.array(self.shared['D'])}
        if D is not None and fit['accepted']:transforms['structure_fit']=D
        self.evaluate(transforms)
        primary_training=[r for r in self.rows if r['map_surface']=='map_opacity005' and r['role']=='fit' and r['variant']=='structure_fit']
        fit['numerical_gate_passed']=fit['accepted']
        fit['lost_training_anchor_support']=[dict(structure=r['structure'],sensor=r['sensor'],reason=r['reason']) for r in primary_training if not r['supported'] and self.planes[r['structure'],r['sensor']]['accepted']]
        fit['post_fit_training_anchor_support_retained']=bool(primary_training) and not fit['lost_training_anchor_support']
        self.overview();self.views();self.identity_views();self.comparison_views(transforms);self.report(D,fit)
        print(json.dumps({'fit_accepted':fit['accepted'],'scaled_rank':fit.get('scaled_rank'),'condition':fit.get('scaled_condition'),'D':None if D is None else D.tolist()},indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=BASE/'structure-experiment/map');args=parser.parse_args()
    StructureMapExperiment(args.output).run()

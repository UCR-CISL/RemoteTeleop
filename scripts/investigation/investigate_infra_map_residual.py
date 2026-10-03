"""Infrastructure-only offline map tie and surface sensitivity; never independent GT.

Reuses the previously declared calibration/heldout split and robust SE(3) solver.
No per-evaluation-frame pose is fitted. All surface filters are labelled diagnostic
sensitivities, fixed in code before evaluation, not selected by reference error.
"""
from __future__ import annotations
import json
from pathlib import Path
import sys
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.investigation.calibrate_shared_enu_map import SharedMapCalibration, transform, voxel, metrics
from scripts.investigation.compare_reference_road_planes import inside, height_at, transform_plane
from scripts.investigation.validate_shared_enu_map import SharedMapValidation
from src.localization.gaussian_map import load_gaussian_map
from src.localization.hybrid_road_height import HybridRoadHeightEstimator

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/reference-refinement/infra-map-residual'
SHARED=ROOT/'artifacts/reference-refinement/shared-enu-calibration'

class InfrastructureMapInvestigation:
    def __init__(self):
        OUT.mkdir(parents=True,exist_ok=True)
        self.fit=SharedMapCalibration();self.fit.prepare()
        self.fit.groups={k:p for k,p in self.fit.groups.items() if k.endswith('_0')}
        self.manifest=[r for r in self.fit.manifest if r['partition']=='calibration' and r['agent']==0]
        self.infra=[(r,p) for r,p in self.fit.heldout if r['agent']==0]
        self.shared=json.loads((SHARED/'calibration.json').read_text())
        self.gaussian=load_gaussian_map(self.fit.map_path)
        self.planes=HybridRoadHeightEstimator()
        self.variants={};self.results=[]

    def write(self,name,value):
        (OUT/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')

    def target(self,opacity,max_scale=None):
        g=self.gaussian
        keep=np.isfinite(g.means).all(1)&np.isfinite(g.opacities)&(g.opacities>opacity)&(abs(g.means).max(1)<=10000)
        if max_scale is not None:keep &= np.isfinite(g.scales).all(1)&(g.scales.max(1)<=max_scale)
        points=voxel(g.means[keep].astype(float),.15)
        tree=cKDTree(points)
        pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points));pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.65,max_nn=30))
        normals=np.asarray(pc.normals)
        counts=tree.query_ball_point(points,.4,return_length=True,workers=2)
        valid=(counts>=6)&np.isfinite(normals).all(1)
        return points,tree,normals,valid

    def configure(self,data):
        self.fit.target,self.fit.tree,self.fit.normals,self.fit.normal_valid=data

    def fit_variants(self):
        for name,opacity,scale in [('infra_positive',0,None),('opacity005',.05,None),('opacity050',.5,None),('compact005',.05,.5)]:
            data=self.target(opacity,scale);self.configure(data)
            D,trace=self.fit.fit()
            result=dict(status='offline diagnostic calibration, not independent GT',variant=name,D=D.tolist(),original_W=self.fit.W.tolist(),map_T_released_enu=(D@self.fit.W).tolist(),D_rpy_deg=Rotation.from_matrix(D[:3,:3]).as_euler('xyz',degrees=True).tolist(),D_translation_m=D[:3,3].tolist(),calibration_frames=self.manifest,map_sha256=self.shared['map_sha256'],map_path=self.fit.map_path,minimum_opacity_exclusive=opacity,max_gaussian_scale_m=scale,target_points=len(data[0]),valid_normal_points=int(data[3].sum()),trace=trace,algorithm='Unchanged SharedMapCalibration.fit; five infrastructure take groups only, 6000/group, .15m Huber, gates 3/1.5/.75/.4m; no vehicle source points',sensitivity_only=name!='infra_positive')
            self.write(name+'-calibration.json',result)
            self.variants[name]=(D,data);self.results.append(result)
            print('FIT',name,result['D_rpy_deg'],result['D_translation_m'],flush=True)
        self.configure(self.variants['infra_positive'][1]);D=self.variants['infra_positive'][0]
        stability=[];cross=np.array([[0,0,0],[30,0,0],[0,30,0],[-30,0,0],[0,-30,0]])
        for take in range(1,6):
            T,trace=self.fit.fit(exclude_take=take);relative=T@np.linalg.inv(D)
            stability.append(dict(excluded_take=take,D=T.tolist(),relative_rotation_deg=float(np.rad2deg(Rotation.from_matrix(relative[:3,:3]).magnitude())),max_shift_30m_cross_m=float(np.linalg.norm(transform(cross,T)-transform(cross,D),axis=1).max()),trace=trace))
            print('STABILITY',take,stability[-1]['max_shift_30m_cross_m'],flush=True)
        self.write('leave-take-out.json',stability)
        self.stability=stability

    def heldout(self):
        data=self.variants['infra_positive'][1];target,tree,normals,normal_valid=data
        variants={'original':np.eye(4),'shared':np.array(self.shared['D']),**{k:v[0] for k,v in self.variants.items()}}
        rows=[]
        for r,p in self.fit.heldout:
            supported=tree.query(p,workers=2)[0]<3.
            for name,D in variants.items():
                q=transform(p,D);dist,ids=tree.query(q,workers=2)
                valid=supported&(dist<.75)&normal_valid[ids]
                residual=np.sum((q-target[ids])*normals[ids],axis=1)
                row=dict(**r,variant=name,all_points=metrics(q,tree),original_tie_supported=metrics(q[supported],tree),original_support_fraction=float(supported.mean()),point_plane_correspondence_count=int(valid.sum()),point_plane_selection='NN < .75m after variant; conditional diagnostic, not same subset',point_plane_abs_median_m=float(np.median(abs(residual[valid]))),point_plane_abs_p95_m=float(np.percentile(abs(residual[valid]),95)))
                if name in self.variants:row['variant_target_nn']=metrics(q,self.variants[name][1][1])
                for label,mask in [('horizontal',np.abs(normals[ids,2])>.85),('vertical',np.abs(normals[ids,2])<.3)]:
                    mask &= valid
                    row[label]=dict(count=int(mask.sum()),nn_median_m=float(np.median(dist[mask])) if mask.any() else None,orthogonal_abs_median_m=float(np.median(abs(residual[mask]))) if mask.any() else None,tangent_median_m=float(np.median(np.sqrt(np.maximum(0,dist[mask]**2-residual[mask]**2)))) if mask.any() else None)
                rows.append(row)
        self.write('heldout-metrics.json',rows);self.heldout_rows=rows
        print('HELDOUT finished',flush=True)

    def road_patches(self):
        # Fixed map-coordinate grid, radius 8m; identical source candidates and
        # points retained across all D variants. Fits measure surfaces, not poses.
        rows=[];map_cache={}
        anchors=[(x,y) for x in [-42,-30,-18,-6,6,18] for y in [-24,-12,0,12,24,36]]
        for xy in anchors:
            xy=np.array(xy,dtype=float)
            for name,(_,data) in self.variants.items():
                p=data[0];p=p[(np.linalg.norm(p[:,:2]-xy,axis=1)<=8)&(p[:,2]>=-4.3)&(p[:,2]<=-1.3)]
                try:
                    p=self.planes._balanced(p);plane,quality,mask=self.planes._plane(p,lidar=False)
                    if not quality['extent_accepted'] or not inside(p[mask],xy):continue
                    map_cache[(*xy,name)]=(plane,quality,p[mask],p)
                except ValueError:continue
        for r,p in self.infra:
            for xy in anchors:
                xy=np.array(xy,dtype=float)
                source=p[(np.linalg.norm(p[:,:2]-xy,axis=1)<=10)&(p[:,2]>=-6)&(p[:,2]<=0)]
                try:
                    source=self.planes._balanced(source);sp,sq,mask=self.planes._plane(source,lidar=True);support=source[mask]
                except ValueError:continue
                comparisons=[('original',np.eye(4),'infra_positive'),('shared',np.array(self.shared['D']),'infra_positive')]
                comparisons += [(k,v[0],k) for k,v in self.variants.items()]
                comparisons += [('baseline_pose_'+k,self.variants['infra_positive'][0],k) for k in self.variants if k!='infra_positive']
                comparisons += [('baseline_surface_'+k,v[0],'infra_positive') for k,v in self.variants.items() if k!='infra_positive']
                for name,D,target_name in comparisons:
                    if (*xy,target_name) not in map_cache:continue
                    mp,mq,ms,mc=map_cache[(*xy,target_name)]
                    ss=transform(support,D);n,d=transform_plane(sp.normal,sp.offset_m,D)
                    _,idx=np.unique(np.floor(ss[:,:2]),axis=0,return_index=True)
                    overlap=float(np.mean(cKDTree(ms[:,:2]).query(ss[idx,:2])[0]<=2.))
                    ok=inside(ss,xy) and overlap>=self.planes.config.min_footprint_overlap
                    residual=mc@mp.normal+mp.offset_m
                    rows.append(dict(take=r['take'],frame_id=r['frame_id'],variant=name,xy_map=xy.tolist(),supported=bool(ok),signed_height_m=height_at(n,d,xy)-height_at(mp.normal,mp.offset_m,xy) if ok else None,normal_angle_deg=float(np.rad2deg(np.arccos(np.clip(n@mp.normal,-1,1)))),footprint_overlap=overlap,source_count=len(source),source_inliers=len(support),map_count=len(mc),map_inliers=len(ms),source_plane_quality=sq,map_plane_quality=mq,map_candidate_residual_quantiles_m=np.quantile(residual,[.05,.25,.5,.75,.95]).tolist(),map_candidate_fraction_over_030m=float(np.mean(abs(residual)>.3))))
        self.write('road-patches.json',rows);self.road_rows=rows
        print('ROAD patches',len(rows),'supported',sum(r['supported'] for r in rows),flush=True)

    @staticmethod
    def planar_fit(points,normal):
        # A median-centered slab followed by PCA rejects unrelated parallel layers.
        n=normal.copy();center=np.median(points,axis=0)
        for _ in range(4):
            residual=points@n;offset=np.median(residual)
            keep=abs(residual-offset)<.15
            if keep.sum()<30:raise ValueError('fewer than 30 planar samples')
            center=points[keep].mean(0);cov=(points[keep]-center).T@(points[keep]-center)/keep.sum()
            values,vectors=np.linalg.eigh(cov);new=vectors[:,0]
            if new@n<0:new=-new
            n=new
        if values[1]<.15**2 or values[0]/values[1]>.05:raise ValueError('insufficient planar extent')
        tangent=vectors[:,1:];spans=np.ptp(points[keep]@tangent,axis=0)
        if spans.min()<1.5:raise ValueError('insufficient 2D span')
        return n,-float(n@center),points[keep],dict(inliers=int(keep.sum()),candidate_count=len(points),rms_m=float(np.sqrt(values[0])),tangent_spans_m=spans.tolist())

    def walls(self):
        # Patches chosen entirely from baseline map geometry before inspecting
        # infrastructure error. They are vertical geometric surfaces, not semantic
        # verification of every wall, pole or vegetation return.
        target,tree,normals,valid=self.variants['infra_positive'][1]
        q=target[valid&(abs(normals[:,2])<.3)&(target[:,2]>-1.3)&(target[:,2]<8)]
        cells=np.floor(q/8).astype(int);patches=[]
        for cell in np.unique(cells,axis=0):
            p=q[(cells==cell).all(1)]
            if len(p)<80:continue
            _,_,axes=np.linalg.svd(p-p.mean(0),full_matrices=False)
            try:n,d,s,quality=self.planar_fit(p,axes[-1])
            except ValueError:continue
            if abs(n[2])>.3:continue
            patches.append((cell,n,d,s,quality))
        rows=[];D=self.variants['infra_positive'][0]
        for r,p in self.infra:
            transformed=transform(p,D)
            for cell,n,d,ms,mq in patches:
                # Broad geometry-only selection; source plane need not coincide.
                dist=cKDTree(ms).query(transformed,workers=2)[0]
                candidates=transformed[dist<1.]
                if len(candidates)<30:continue
                try:sn,sd,ss,sq=self.planar_fit(candidates,n)
                except ValueError:continue
                angle=float(np.rad2deg(np.arccos(np.clip(sn@n,-1,1))))
                anchor=ms.mean(0);separation=-float((anchor@sn+sd)/(sn@n))
                # Source and map support must overlap around same plane anchor.
                tangent=np.linalg.svd(ms-ms.mean(0),full_matrices=False)[2][:2].T
                source_2d=(ss-anchor)@tangent;map_2d=(ms-anchor)@tangent
                support=inside(np.c_[source_2d,np.zeros(len(ss))],[0,0]) and inside(np.c_[map_2d,np.zeros(len(ms))],[0,0])
                rows.append(dict(take=r['take'],frame_id=r['frame_id'],cell=cell.tolist(),anchor_map=anchor.tolist(),source_normal=sn.tolist(),map_normal=n.tolist(),supported=bool(support),signed_plane_separation_along_map_normal_m=separation if support else None,normal_angle_deg=angle,source_quality=sq,map_quality=mq,nn_median_m=float(np.median(cKDTree(ms).query(ss)[0])),source_point_map_plane_abs_median_m=float(np.median(abs(ss@n+d)))))
        self.write('vertical-patches.json',dict(selection='8m map cells; map normals |nz|<.3, -1.3<z<8; >=80 candidates, robust .15m slab; >=30 planar inliers, minor tangent variance>.15^2, min tangent span1.5m; source NN<1m; no pose fitting',map_patches=len(patches),rows=rows))
        self.wall_rows=rows
        print('VERTICAL patches',len(patches),'comparisons',len(rows),'supported',sum(r['supported'] for r in rows),flush=True)

    def vehicle_fixed(self):
        # Original eight frame identities, fixed road points/map planes/anchors,
        # identical support gates, and no evaluation pose fitting.
        validation=SharedMapValidation(OUT/'infra_positive-calibration.json',OUT/'vehicle-eight')
        validation.run()
        self.vehicle_summary=json.loads((OUT/'vehicle-eight/summary.json').read_text())

    def surface_views(self):
        """Show Gaussian layers under the same frozen infrastructure pose."""
        D=self.variants['infra_positive'][0]
        source=transform(self.infra[0][1],D)
        anchors=[[-42,12],[-30,0],[-18,-12],[-18,0]]
        fig,axes=plt.subplots(2,2,figsize=(14,9),layout='constrained')
        for ax,xy in zip(axes.flat,anchors):
            xy=np.array(xy,dtype=float)
            for name,color in [('infra_positive','#999999'),('opacity005','#ac5cc9'),('opacity050','#d48024')]:
                p=self.variants[name][1][0]
                p=p[(np.linalg.norm(p[:,:2]-xy,axis=1)<8)&(p[:,2]>=-4.3)&(p[:,2]<=-1.3)]
                cut=p[abs(p[:,1]-xy[1])<1.5]
                ax.scatter(cut[:,0]-xy[0],cut[:,2],s=2,alpha=.25,c=color)
                try:
                    p=self.planes._balanced(p);plane,quality,mask=self.planes._plane(p,lidar=False)
                    xx=np.linspace(-8,8,100);zz=-(plane.normal[0]*(xx+xy[0])+plane.normal[1]*xy[1]+plane.offset_m)/plane.normal[2]
                    ax.plot(xx,zz,color=color,label=f'{name} plane')
                except ValueError:pass
            cut=source[(abs(source[:,0]-xy[0])<8)&(abs(source[:,1]-xy[1])<1.5)&(source[:,2]>-4.3)&(source[:,2]<-1.3)]
            ax.scatter(cut[:,0]-xy[0],cut[:,2],s=9,c='#168cab',label='Held-out infrastructure, fixed D')
            ax.set(xlim=(-8,8),ylim=(-4.3,-1.3),xlabel='Map X from anchor (m)',ylabel='Map Z (m)',title=f'Map anchor {xy.tolist()}; |Y-anchor Y| <1.5m')
            ax.legend(fontsize=7);ax.grid(alpha=.2)
        fig.suptitle('Surface-only sensitivity: every panel uses the same baseline infrastructure pose\nPlane fits use full 8m-radius patches; scatter uses a narrow Y slice')
        fig.savefig(OUT/'surface-layers.png',dpi=160);plt.close(fig)

    def vehicle_surface_sensitivity(self):
        """Keep all eight source planes/anchors; only fixed D or map filter varies."""
        records=json.loads((ROOT/'artifacts/reference-refinement/final-comparison/diagnostics.json').read_text())
        g=self.gaussian
        keep=np.isfinite(g.means).all(1)&np.isfinite(g.opacities)&(g.opacities>.05)&(abs(g.means).max(1)<=10000)
        target=g.means[keep].astype(float);rows=[]
        for r in records:
            xy=np.array(r['comparison_xy_map']);case=f"{r['run_id']}-f{r['frame_offset']:06d}"
            with np.load(ROOT/'artifacts/reference-refinement/final-comparison'/case/'road-support.npz') as f:
                native=f['native_support'].copy();baseline_support=f['map_support'].copy()
            candidates=target[(np.linalg.norm(target[:,:2]-xy,axis=1)<=15)&(target[:,2]>=-4.3)&(target[:,2]<=-1.3)]
            candidates=self.planes._balanced(candidates)
            try:
                plane,quality,mask=self.planes._plane(candidates,lidar=False)
                filtered=(np.array(plane.normal),plane.offset_m,candidates[mask],quality['extent_accepted'])
            except ValueError:
                filtered=None
            baseline=(np.array(r['map_road']['normal']),r['map_road']['offset'],baseline_support,r['map_support_status']=='supported')
            for name,D,surface in [('shared_original_surface',np.array(self.shared['D']),baseline),('infra_original_surface',self.variants['infra_positive'][0],baseline),('shared_opacity005_surface',np.array(self.shared['D']),filtered),('infra_opacity005_surface',self.variants['infra_positive'][0],filtered),('opacity005_fit_and_surface',self.variants['opacity005'][0],filtered)]:
                if surface is None:
                    rows.append(dict(run_id=r['run_id'],frame_offset=r['frame_offset'],variant=name,supported=False,reason='filtered map fit failed'));continue
                mn,md,ms,extent=surface
                T=D@self.fit.W;n,d=transform_plane(r['source_road']['normal'],r['source_road']['offset'],T)
                ss=transform(native,T);_,idx=np.unique(np.floor(ss[:,:2]),axis=0,return_index=True)
                overlap=float(np.mean(cKDTree(ms[:,:2]).query(ss[idx,:2])[0]<=2.))
                valid=extent and inside(ms,xy) and inside(ss,xy) and overlap>=self.planes.config.min_footprint_overlap
                rows.append(dict(run_id=r['run_id'],frame_offset=r['frame_offset'],frame_id=r['frame_id'],variant=name,supported=bool(valid),signed_height_m=height_at(n,d,xy)-height_at(mn,md,xy) if valid else None,normal_angle_deg=float(np.rad2deg(np.arccos(np.clip(n@mn,-1,1)))),map_height_m=height_at(mn,md,xy),source_height_m=height_at(n,d,xy),map_normal=mn.tolist(),map_inlier_count=len(ms),footprint_overlap=overlap,comparison_xy_map=xy.tolist()))
        self.write('vehicle-surface-sensitivity.json',rows)
        names=list(dict.fromkeys(r['variant'] for r in rows))
        lines=['# Same-eight vehicle surface sensitivity','', 'Each row uses the original source plane, source inliers, frame identity and common XY. No vehicle evaluation pose is fitted. The >0.05 opacity map plane is refitted at the original 15m radius and [-4.3,-1.3]m band with the existing geometric support gates. This is a surface sensitivity, not a promoted reference.','', '| Case | Shared / original surface | Infra / original surface | Shared / opacity005 | Infra / opacity005 | Opacity005 fit+surface |','|---|---:|---:|---:|---:|---:|']
        for r in records:
            subset=[x for x in rows if x['run_id']==r['run_id'] and x['frame_offset']==r['frame_offset']]
            cells=[f"{x['signed_height_m']:+.3f}m / {x['normal_angle_deg']:.2f}°" if x['supported'] else 'unsupported' for x in subset]
            lines.append(f"| {r['run_id']} f{r['frame_offset']} | "+' | '.join(cells)+' |')
        lines+=['','| Variant | Supported frames | Median absolute height | Maximum absolute height |','|---|---:|---:|---:|']
        for name in names:
            group=[r for r in rows if r['variant']==name and r['supported']]
            lines.append(f"| {name} | {len(group)} | {np.median([abs(r['signed_height_m']) for r in group]):.3f}m | {max(abs(r['signed_height_m']) for r in group):.3f}m |")
        (OUT/'vehicle-surface-sensitivity.md').write_text('\n'.join(lines)+'\n')

    def report(self):
        variants=['original','shared',*self.variants]
        lines=['# Infrastructure-only GSplat map tie and residual attribution','',
            'Offline diagnostic only. Fifteen designated infrastructure calibration scans (20%, 50%, 80% in takes 1–5) fit one SE(3), with the original ten infrastructure and thirty vehicle held-out scans (35%, 65%). The same eight previous vehicle comparisons are evaluated separately. Infrastructure is stationary and its surveyed placement is unverified; repeated observations are not independent sensors. No runtime defaults, acceptance gates, earlier outputs or source archives changed.','',
            'The remaining discrepancy has at least two components. Infrastructure road planes are sensitive to the Gaussian surface representation: with its baseline pose fixed, excluding opacity <=0.05 centers changes median absolute road separation from 0.163m to 0.029m across the same 37 supported patch/frame pairs (maximum 0.138m). At map XY (-30,0), the median signed gap changes from +0.293m to -0.013m, while (-42,12) retains +0.118m. This is a surface-selection effect, not proof that a chosen opacity threshold is physically correct.','',
            'The same filter does not resolve the eight vehicle cases. Keeping shared D fixed gives median/max absolute height 0.409/0.628m; using the separately frozen opacity >0.05 infrastructure fit gives 0.395/0.631m. Some gaps therefore survive both a stationary-only map fit and a substantially different map-road surface selection. The experiment does not support a <0.1m reference.','',
            '## Frozen transforms and surface sensitivity','',
            'Baseline uses positive-opacity Gaussian centers and exactly the existing shared calibration solver with only unit 0 groups. The three additional fixed filters are sensitivity experiments, not promoted candidates or physical confidence. All NN comparisons below use the same baseline target; separate sensitivity-target metrics are retained in JSON.','',
            '| Variant | Euler XYZ correction (degrees) | Translation at map origin (m) | Target centers |','|---|---|---|---:|']
        for r in self.results:lines.append(f"| {r['variant']} | {np.round(r['D_rpy_deg'],5).tolist()} | {np.round(r['D_translation_m'],5).tolist()} | {r['target_points']} |")
        lines+=['','Column vectors: `map_T_released_enu = D @ original_W`; then `map_T_lidar = map_T_released_enu @ released_enu_T_lidar`. Transforms, map hash, frame manifests and full solver traces are in each `*-calibration.json`. Opacity thresholds are >0, >0.05, >0.5; compact005 additionally keeps maximum Gaussian scale <=0.5m. Gaussian scales are standard deviations, not surface accuracy bounds.','',
            '## Identical held-out scans','',
            '| Variant | Infra NN median | Vehicle NN median | Infra orthogonal median | Infra vertical NN / orthogonal median |','|---|---:|---:|---:|---:|']
        for name in variants:
            infra=[r for r in self.heldout_rows if r['agent']==0 and r['variant']==name];vehicle=[r for r in self.heldout_rows if r['agent']>0 and r['variant']==name]
            val=lambda rows,key:np.median([r[key] for r in rows])
            lines.append(f"| {name} | {np.median([r['all_points']['median_m'] for r in infra]):.3f} | {np.median([r['all_points']['median_m'] for r in vehicle]):.3f} | {val(infra,'point_plane_abs_median_m'):.3f} | {np.median([r['vertical']['nn_median_m'] for r in infra]):.3f} / {np.median([r['vertical']['orthogonal_abs_median_m'] for r in infra]):.3f} |")
        lines+=['','Values are medians of frame medians in meters, not pose errors. Orthogonal residuals use supported target normals and NN <0.75m after each transform, so their subset varies and must not be used as a same-point ranking. Nonzero tangent/NN distance includes Gaussian center spacing and source sampling. A small point-to-plane distance does not guarantee complete registration.','',
            '## Infrastructure road patches','',
            'Fixed 12m-spaced XY grid, 8m map radius and 10m source radius; source candidates selected once in original coordinates and reused across transforms. Map road candidates use the historical [-4.3,-1.3]m band. Standard plane fitting and footprint/hull gates apply. Surface planes measure residuals; they do not adjust held-out poses.','',
            '| Variant | Supported patch/frame pairs | Median absolute height | Maximum absolute height | Median normal angle |','|---|---:|---:|---:|---:|']
        road_variants=[*variants,*['baseline_pose_'+k for k in self.variants if k!='infra_positive'],*['baseline_surface_'+k for k in self.variants if k!='infra_positive']]
        for name in road_variants:
            group=[r for r in self.road_rows if r['variant']==name and r['supported']]
            if group:lines.append(f"| {name} | {len(group)} | {np.median([abs(r['signed_height_m']) for r in group]):.3f} | {max(abs(r['signed_height_m']) for r in group):.3f} | {np.median([r['normal_angle_deg'] for r in group]):.3f}° |")
        lines+=['','`baseline_pose_FILTER` keeps baseline D frozen and changes only the map surface; `baseline_surface_FILTER` uses the filtered fit D with the original map surface. These isolate surface-selection and transform effects. Aggregates can have different supported patch counts; per-patch JSON retains every supported and unsupported comparison. Convex hull support can bridge gaps; road curvature and changing scan visibility remain limitations. `map_candidate_residual_quantiles_m` and `map_candidate_fraction_over_030m` expose candidate layer thickness and off-plane clutter, including possible floaters; off-plane centers are not automatically errors.','',
            '## Vertical surface patches','']
        supported=[r for r in self.wall_rows if r['supported']]
        if supported:lines+=[f"After the frozen infrastructure baseline fit, {len(supported)} supported patch/frame comparisons have median absolute plane separation {np.median([abs(r['signed_plane_separation_along_map_normal_m']) for r in supported]):.3f}m, maximum {max(abs(r['signed_plane_separation_along_map_normal_m']) for r in supported):.3f}m, median normal disagreement {np.median([r['normal_angle_deg'] for r in supported]):.3f}°. Their median NN residual is {np.median([r['nn_median_m'] for r in supported]):.3f}m. Full geometric selection and support are in `vertical-patches.json`. These are vehicle-masked vertical geometric patches, not semantic certification of individual walls; vegetation/occlusion can remain."]
        else:lines+=['No vertical plane patch passed common-anchor support; do not infer nonroad surface agreement from conditional point-to-plane residuals alone.']
        lines+=['','## Stability and unchanged vehicle cases','',f"Leaving out one calibration take changes the baseline correction by at most {max(r['relative_rotation_deg'] for r in self.stability):.4f}° and moves the tested origin/±30m cross by at most {max(r['max_shift_30m_cross_m'] for r in self.stability):.4f}m. This measures repeated-station sensitivity, not geodetic uncertainty.",'',
            '[Identical eight vehicle cases with infrastructure-only map tie](vehicle-eight/report.md). The reused validator calls the candidate `shared_enu_map`; its frozen input is explicitly the infrastructure-only calibration in this directory. No vehicle data entered its fit.','',
            '[Same-eight vehicle surface sensitivity](vehicle-surface-sensitivity.md) isolates the opacity >0.05 map surface under frozen shared and infrastructure transforms, then checks the separate opacity >0.05 infrastructure calibration.','',
            f"All eight retain their original point subsets and common XY. Infrastructure-only calibration leaves median absolute supported road-height separation {self.vehicle_summary['shared_enu_map']['median_absolute_supported_road_height_m']:.3f}m, compared with 0.328m for the previous shared vehicle/infrastructure transform. Correcting the stationary map tie alone therefore does not resolve their vehicle road discrepancies.",'',
            'The experiment separates a stationary map/infrastructure alignment issue from vehicle-specific remaining disagreement. It cannot establish absolute GT, a uniquely physical Gaussian surface, or a <0.1m reference. Filters and residuals must not be chosen to reward a localization result.','',
            '## Plots','', '![Held-out and spatial diagnostics](residual-diagnostics.png)','',
            '![Fixed-pose Gaussian surface-layer sensitivity](surface-layers.png)','',
            '## Reproduce','', '```sh','OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 /home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python scripts/investigation/investigate_infra_map_residual.py','```','']
        (OUT/'report.md').write_text('\n'.join(lines))
        fig,axes=plt.subplots(2,2,figsize=(14,10),layout='constrained')
        for agent in range(4):
            values=[np.median([r['all_points']['median_m'] for r in self.heldout_rows if r['agent']==agent and r['variant']==name]) for name in variants]
            axes[0,0].plot(variants,values,'o-',label=f'unit {agent}')
        axes[0,0].set(ylabel='Median held-out NN residual (m)',title='Identical points and baseline target');axes[0,0].tick_params(axis='x',rotation=25);axes[0,0].legend()
        rows=[r for r in self.road_rows if r['variant']=='infra_positive' and r['supported']]
        xy=sorted(set(tuple(r['xy_map']) for r in rows))
        if xy:
            values=[np.median([r['signed_height_m'] for r in rows if tuple(r['xy_map'])==p]) for p in xy];xy=np.array(xy)
            s=axes[0,1].scatter(xy[:,0],xy[:,1],c=values,cmap='coolwarm',vmin=-.3,vmax=.3,s=250);fig.colorbar(s,ax=axes[0,1],label='Median signed road height (m)')
            for p,v in zip(xy,values):axes[0,1].annotate(f'{v:+.2f}',p,ha='center',fontsize=8)
        axes[0,1].set(xlabel='Map X (m)',ylabel='Map Y (m)',title='Infrastructure-only fit: supported road patches');axes[0,1].set_aspect('equal')
        for name in ['shared',*self.variants]:
            group=[r for r in self.road_rows if r['variant']==name and r['supported']]
            if group:axes[1,0].scatter([r['xy_map'][0] for r in group],[r['signed_height_m'] for r in group],s=12,alpha=.5,label=name)
        axes[1,0].axhline(0,color='k',lw=.7);axes[1,0].set(xlabel='Map patch X (m)',ylabel='Signed height (m)',title='All supported road comparisons; no per-frame pose fit');axes[1,0].legend(fontsize=8)
        for label in ['horizontal','vertical']:
            group=[r for r in self.heldout_rows if r['agent']==0 and r['variant']=='infra_positive']
            axes[1,1].scatter([r[label]['nn_median_m'] for r in group],[r[label]['orthogonal_abs_median_m'] for r in group],label=label)
        axes[1,1].plot([0,.3],[0,.3],'k--',lw=.7);axes[1,1].set(xlabel='Point-to-point NN median (m)',ylabel='Orthogonal point-to-plane median (m)',title='Sampling vs surface-normal discrepancy');axes[1,1].legend()
        fig.savefig(OUT/'residual-diagnostics.png',dpi=150);plt.close(fig)

    def run(self):
        self.fit_variants();self.heldout();self.road_patches();self.walls();self.vehicle_fixed();self.surface_views();self.vehicle_surface_sensitivity();self.report()

if __name__=='__main__':InfrastructureMapInvestigation().run()

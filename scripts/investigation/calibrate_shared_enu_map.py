"""Offline shared released-ENU to GSplat SE(3) calibration, never independent GT.

All four units and all five takes are balanced. Static boxes are removed before
voxelization; chronological split and legacy holdouts are declared before fitting.
No runtime localization acceptance gates or production transforms are modified.
"""
from __future__ import annotations
import hashlib, json, sys, zipfile
from pathlib import Path
import numpy as np
import open3d as o3d
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from src.localization.cooperscene import pose_to_matrix
from scripts.investigation.first_frame_xyz import map_proxy

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/reference-refinement/shared-enu-calibration'
ARCHIVE=Path('/mnt/bcc-data/websites/data.ucr.edu-datasets-cooperscene')
OLD=ROOT/'artifacts/reference-refinement/final-comparison/diagnostics.json'
LEGACY=ROOT/'scripts/investigation/cooperscene_legacy_cases.json'
RECORD=ROOT/'scripts/investigation/cooperscene_shared_enu_map.json'


def transform(p,T): return p@T[:3,:3].T+T[:3,3]

def voxel(p,size):
    _,i=np.unique(np.floor(p/size).astype(np.int64),axis=0,return_index=True)
    return p[i]

def metrics(p,tree):
    if len(p)==0:return dict(point_count=0,status="insufficient_support")
    d=tree.query(p,workers=2)[0]
    return dict(point_count=len(p),median_m=float(np.median(d)),p95_m=float(np.percentile(d,95)),fraction_under_020m=float(np.mean(d<.2)),fraction_under_050m=float(np.mean(d<.5)),fraction_under_075m=float(np.mean(d<.75)))

class SharedMapCalibration:
    def __init__(self):
        OUT.mkdir(parents=True,exist_ok=True)
        if OLD.exists():
            legacy=json.loads(OLD.read_text())
            attempt=json.loads(Path(legacy[0]['attempt_path']).read_text())
            self.map_path=attempt['metadata']['map_path']
            with np.load(attempt['metadata']['npz_path']) as f:self.W=f['reference_map_T_dataset'].copy()
        else:
            # Lost git-ignored inputs: committed case list plus the committed calibration record.
            legacy=json.loads(LEGACY.read_text())['cases']
            record=json.loads(RECORD.read_text())
            self.map_path=record['map_path'];self.W=np.asarray(record['original_W'])
        self.excluded={(int(r['run_id'].split('-')[1]),int(r['run_id'].split('-')[2]),str(r['frame_id'])) for r in legacy}
        expected_W=np.eye(4);expected_W[:3,3]=[1.2891715943262518,-3.381634466657298,-.43000102971778076]
        if not np.allclose(self.W,expected_W,atol=1e-10):raise ValueError('Unexpected inherited map tie')
        self.target=voxel(map_proxy(self.map_path,min_opacity=0),.15)
        self.tree=cKDTree(self.target)
        self.manifest=[];self.groups={};self.heldout=[]

    def read(self,z,take,agent,frame):
        prefix=f'{take}/{agent}/{frame}';ann=yaml.safe_load(z.read(prefix+'.yaml'))
        raw=z.read(prefix+'.pcd').split(b'DATA ascii\n',1)[1]
        points=np.fromstring(raw.decode(),sep=' ').reshape(-1,4)[:,:3]
        points=transform(points,pose_to_matrix(ann['lidar_pose']))
        keep=np.isfinite(points).all(1)
        for box in ann['vehicles'].values():
            center=np.array(box['location'])+np.array(box['center']);B=pose_to_matrix([*center,*box['angle']]);q=(points-center)@B[:3,:3]
            keep &= ~np.all(np.abs(q)<=np.array(box['extent'])+.2,axis=1)
        return transform(voxel(points[keep],.20),self.W)

    def prepare(self):
        cache=OUT/'input-clouds.npz';manifest=OUT/'input-manifest.json'
        if cache.exists() and manifest.exists():
            self.manifest=json.loads(manifest.read_text())
            with np.load(cache) as f:
                self.groups={k[4:]:f[k] for k in f.files if k.startswith('cal_')}
                self.heldout=[(r,f[r['cache_key']]) for r in self.manifest if r['partition']=='heldout']
            print('Loaded cached clouds',flush=True);return
        arrays={}
        for take in range(1,6):
            path=next(ARCHIVE.glob(f'*/{take}.zip'))
            with zipfile.ZipFile(path) as z:
                for agent in range(4):
                    frames=sorted(n.split('/')[-1][:-5] for n in z.namelist() if n.startswith(f'{take}/{agent}/') and n.endswith('.yaml'))
                    cal=[]
                    for partition,fractions in [('calibration',[.2,.5,.8]),('heldout',[.35,.65])]:
                        for fraction in fractions:
                            idx=round(fraction*(len(frames)-1))
                            while partition=='calibration' and (take,agent,frames[idx]) in self.excluded:idx+=1
                            frame=frames[idx];p=self.read(z,take,agent,frame)
                            record=dict(take=take,agent=agent,frame_id=frame,frame_offset=idx,frame_count=len(frames),fraction=idx/(len(frames)-1),partition=partition,archive=str(path),cache_key=f'hold_{take}_{agent}_{frame}')
                            self.manifest.append(record)
                            if partition=='calibration':cal.append(p)
                            else:self.heldout.append((record,p));arrays[record['cache_key']]=p
                    group=f'{take}_{agent}';p=voxel(np.concatenate(cal),.25);self.groups[group]=p;arrays['cal_'+group]=p
                    print('Prepared',group,len(p),flush=True)
        np.savez_compressed(cache,**arrays);manifest.write_text(json.dumps(self.manifest,indent=2))

    def fit(self,exclude_take=None,initial=None,exclude_agent=None):
        # Fixed per-unit/take cap, then equal per-group weight each iteration.
        group_points=[];group_ids=[]
        for number,(name,p) in enumerate(sorted(self.groups.items())):
            if exclude_take is not None and int(name.split('_')[0])==exclude_take:continue
            if exclude_agent is not None and int(name.split('_')[1])==exclude_agent:continue
            if len(p)>6000:p=p[np.linspace(0,len(p)-1,6000).astype(int)]
            group_points.append(p);group_ids.extend([number]*len(p))
        source=np.concatenate(group_points);ids=np.array(group_ids)
        D=np.eye(4) if initial is None else initial.copy();trace=[]
        for gate,iterations in [(3.,20),(1.5,20),(.75,20),(.4,20)]:
            for iteration in range(iterations):
                p=transform(source,D);dist,index=self.tree.query(p,workers=2)
                keep=(dist<gate)&self.normal_valid[index]
                q=self.target[index[keep]];n=self.normals[index[keep]];x=p[keep];gid=ids[keep]
                if len(x)<100:raise RuntimeError('Insufficient supported correspondences')
                residual=np.sum(n*(x-q),axis=1)
                A=np.c_[np.cross(x,n),n]
                # Balance each unit/take AND road-like/non-road normal categories.
                category=(np.abs(n[:,2])>.85).astype(int);weight=np.zeros(len(x))
                for g in np.unique(gid):
                    for c in [0,1]:
                        choose=(gid==g)&(category==c);weight[choose]=1/max(int(choose.sum()),1)
                for g in np.unique(gid):
                    choose=gid==g;weight[choose]/=weight[choose].sum()
                weight*=np.minimum(1.,.15/np.maximum(np.abs(residual),1e-12))
                H=A.T@(weight[:,None]*A);b=A.T@(weight*residual)
                delta=np.linalg.lstsq(H,-b,rcond=1e-10)[0]
                # Bound a single optimization step, without reference-pose tuning.
                scale=max(np.linalg.norm(delta[:3])/np.deg2rad(2),np.linalg.norm(delta[3:])/.5,1.)
                delta/=scale;step=np.eye(4);step[:3,:3]=Rotation.from_rotvec(delta[:3]).as_matrix();step[:3,3]=delta[3:];D=step@D
                trace.append(dict(gate_m=gate,iteration=iteration,correspondences=len(x),roadlike_count=int(category.sum()),nonroad_count=int((category==0).sum()),point_plane_median_m=float(np.median(abs(residual))),delta_rotation_deg=float(np.rad2deg(np.linalg.norm(delta[:3]))),delta_translation_m=float(np.linalg.norm(delta[3:])),normal_matrix_eigenvalues=np.linalg.eigvalsh(H).tolist(),support_by_group={name:dict(selected_source_count=int((ids==g).sum()),correspondence_count=int((gid==g).sum()),roadlike_count=int(((gid==g)&(category==1)).sum()),nonroad_count=int(((gid==g)&(category==0)).sum())) for g,name in enumerate(sorted(self.groups)) if np.any(ids==g)}))
                if np.linalg.norm(delta)<1e-6:break
        return D,trace

    def run(self):
        self.prepare()
        pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(self.target));pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.65,max_nn=30));self.normals=np.asarray(pc.normals)
        # Require local geometric neighborhood support; no confidence invented.
        counts=self.tree.query_ball_point(self.target,.4,return_length=True,workers=2)
        self.normal_valid=(counts>=6)&np.isfinite(self.normals).all(1)
        D,trace=self.fit();self.D=D
        result=dict(status='offline_map_calibration_candidate_not_independent_ground_truth',D=D.tolist(),original_W=self.W.tolist(),map_T_released_enu=(D@self.W).tolist(),composition='map_T_lidar = D @ original_W @ released_ENU_T_lidar',D_translation_m=D[:3,3].tolist(),D_rpy_deg=Rotation.from_matrix(D[:3,:3]).as_euler('xyz',degrees=True).tolist(),map_path=self.map_path,map_sha256=hashlib.file_digest(open(self.map_path,'rb'),'sha256').hexdigest(),map_filter='finite centers, finite opacity >0, absolute coordinates <=10000m; 0.15m voxel first representative',target_points=len(self.target),valid_normal_points=int(self.normal_valid.sum()),calibration_frames=[r for r in self.manifest if r['partition']=='calibration'],excluded_legacy_frames=[dict(take=t,agent=a,frame_id=f) for t,a,f in sorted(self.excluded)],trace=trace,calibration_algorithm=dict(gates_m=[3,1.5,.75,.4],huber_m=.15,max_points_per_take_unit=6000,groups=20,category_balance='equal weights per take/unit and normal |nz| >0.85 versus other',runtime_acceptance_gates_unchanged=True))
        (OUT/'calibration.json').write_text(json.dumps(result,indent=2));print('Frozen transform',result['D_rpy_deg'],result['D_translation_m'],flush=True)
        held=[]
        for r,p in self.heldout:
            # Common supported subset chosen ONCE using original tie, a broad 3m
            # envelope. Also report every point so cropping cannot hide failures.
            support=self.tree.query(p,workers=2)[0]<3.
            for variant,T in [('original',np.eye(4)),('shared',D)]:
                transformed=transform(p,T)
                held.append(dict(**r,variant=variant,all_points=metrics(transformed,self.tree),original_tie_supported=metrics(transformed[support],self.tree),original_support_fraction=float(support.mean())))
        (OUT/'heldout-metrics.json').write_text(json.dumps(held,indent=2));print('Heldout finished',flush=True)
        stability=[]
        for take in range(1,6):
            T,tr=self.fit(exclude_take=take)
            relative=T@np.linalg.inv(D)
            shift=np.linalg.norm(transform(np.array([[0,0,0],[30,0,0],[0,30,0],[-30,0,0],[0,-30,0]]),T)-transform(np.array([[0,0,0],[30,0,0],[0,30,0],[-30,0,0],[0,-30,0]]),D),axis=1)
            stability.append(dict(excluded_take=take,D=T.tolist(),relative_translation_m=relative[:3,3].tolist(),relative_rotation_deg=float(np.rad2deg(Rotation.from_matrix(relative[:3,:3]).magnitude())),max_shift_within_30m_cross_m=float(shift.max()),trace=tr))
            print('Leave take out',take,stability[-1]['relative_rotation_deg'],shift.max(),flush=True)
        (OUT/'leave-take-out.json').write_text(json.dumps(stability,indent=2))
        unit_stability=[]
        for agent in range(4):
            T,tr=self.fit(exclude_agent=agent);relative=T@np.linalg.inv(D)
            unit_stability.append(dict(excluded_agent=agent,D=T.tolist(),relative_translation_m=relative[:3,3].tolist(),relative_rotation_deg=float(np.rad2deg(Rotation.from_matrix(relative[:3,:3]).magnitude())),trace=tr))
            print('Leave unit out',agent,unit_stability[-1]['relative_rotation_deg'],flush=True)
        (OUT/'leave-unit-out.json').write_text(json.dumps(unit_stability,indent=2))

if __name__=='__main__':SharedMapCalibration().run()

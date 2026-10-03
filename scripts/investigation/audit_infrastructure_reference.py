"""Read-only, GSplat-independent infrastructure/vehicle geometry comparison."""
from __future__ import annotations
import csv,json,sys,zipfile
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial import cKDTree, ConvexHull
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from src.localization.cooperscene import pose_to_matrix
from scripts.investigation.first_frame_xyz import source_points

OLD=Path('/tmp/RemoteTeleop-reference-orientation/artifacts/reference-orientation')
OUT=Path(__file__).resolve().parents[2]/'artifacts/reference-refinement/infrastructure-audit'
RELEASE=Path('/mnt/bcc-data/data/Cooperscene/release/250928')
ARCHIVE=Path('/mnt/bcc-data/websites/data.ucr.edu-datasets-cooperscene')
ATTEMPTS=Path('/tmp/RemoteTeleop-hybrid-road-height/artifacts/full-sequence-localization/attempts')
ROI=(-42.,-18.,-8.,10.)
ANCHOR=np.array([-30.,0.])

def transform(p,T):return p@T[:3,:3].T+T[:3,3]

def plane(points):
    """Fixed spatial road patch, robust dominant nonvertical plane; no leveling."""
    keep=(points[:,0]>ROI[0])&(points[:,0]<ROI[1])&(points[:,1]>ROI[2])&(points[:,1]<ROI[3])&(points[:,2]>-6)&(points[:,2]<0)
    p=points[keep]
    if len(p)<40:return {'status':'insufficient_points','candidate_count':len(p)}
    # Equalize density so a dense scan line does not dominate the plane.
    _,idx=np.unique(np.floor(p/.20).astype(int),axis=0,return_index=True);p=p[idx]
    if len(p)<30:return {'status':'insufficient_voxels','candidate_count':len(p)}
    rng=np.random.default_rng(71);best=np.zeros(len(p),bool)
    for _ in range(500):
        tri=p[rng.choice(len(p),3,replace=False)];n=np.cross(tri[1]-tri[0],tri[2]-tri[0]);norm=np.linalg.norm(n)
        if norm<1e-8:continue
        n/=norm
        if abs(n[2])<np.cos(np.deg2rad(18)):continue
        inlier=np.abs((p-tri[0])@n)<.12
        if inlier.sum()>best.sum():best=inlier
    if best.sum()<30:return {'status':'no_supported_plane','candidate_count':len(p)}
    for _ in range(3):
        coef=np.linalg.lstsq(np.c_[p[best,:2],np.ones(best.sum())],p[best,2],rcond=None)[0]
        best=np.abs(p[:,2]-np.c_[p[:,:2],np.ones(len(p))]@coef)<.12
    n=np.r_[-coef[:2],1.];n/=np.linalg.norm(n)
    hull=ConvexHull(p[best,:2]); inside=bool(np.all(hull.equations[:,:2]@ANCHOR+hull.equations[:,2]<=1e-9))
    return dict(anchor_within_support_hull=inside,support_area_m2=float(hull.volume),support_hull_xy=p[best,:2][hull.vertices].tolist(),status='ok',candidate_count=len(p),inlier_count=int(best.sum()),inlier_fraction=float(best.mean()),normal=n.tolist(),z_ax_by_c=coef.tolist(),height_at_anchor_m=float(np.r_[ANCHOR,1]@coef),tilt_deg=float(np.degrees(np.arccos(n[2]))),rms_m=float(np.sqrt(np.mean((p[best,2]-np.c_[p[best,:2],np.ones(best.sum())]@coef)**2))),support_xy_min=p[best,:2].min(0).tolist(),support_xy_max=p[best,:2].max(0).tolist())

class InfrastructureAudit:
    def __init__(self):
        OUT.mkdir(parents=True,exist_ok=True)
        self.physical=json.loads((OLD/'axes-audit/physical-references.json').read_text())
        self.refs={(r['run_id'],r['frame_offset']):r for r in self.physical['records']}
        self.map_T_enu=np.array(self.physical['map_T_enu'])
        self.archives={}
        self.manifest=[]
    def archive(self,take):
        if take not in self.archives:
            path=next(ARCHIVE.glob(f'*/{take}.zip'));self.archives[take]=zipfile.ZipFile(path)
        return self.archives[take]
    def infra(self,take,frame):
        z=self.archive(take);prefix=f'{take}/0/{frame}';ann=yaml.safe_load(z.read(prefix+'.yaml'));T=pose_to_matrix(ann['lidar_pose'])
        data=z.read(prefix+'.pcd').split(b'DATA ascii\n',1)[1];p=np.fromstring(data.decode(),sep=' ').reshape(-1,4)[:,:3];p=transform(p,T);keep=np.ones(len(p),bool)
        for box in ann['vehicles'].values():
            center=np.array(box['location'])+np.array(box['center']);B=pose_to_matrix([*center,*box['angle']]);q=(p-center)@B[:3,:3];keep &= ~np.all(abs(q)<=np.array(box['extent'])+.3,axis=1)
        self.manifest.append(dict(archive=z.filename,members=[prefix+'.yaml',prefix+'.pcd']))
        return p[keep],T
    def run(self):
        gallery=list(csv.DictReader((OLD/'physical-rescored/previous-seven-cases.csv').open()))
        selection={(r['run_id'],int(r['frame_offset'])):'original_gallery' for r in gallery}
        # First and middle existing saved frame of all 15 runs; no quality-based selection.
        for run in sorted(ATTEMPTS.glob('cooperscene-*')):
            paths=sorted(run.glob('f*.json'));offsets=[int(p.stem[1:]) for p in paths]
            for offset in [offsets[0],offsets[len(offsets)//2]]:selection.setdefault((run.name,offset),'deterministic_heldout')
        target_by_take={};infra_report=[]
        for take in range(1,6):
            z=self.archive(take);frames=sorted(n.split('/')[-1][:-5] for n in z.namelist() if n.startswith(f'{take}/0/') and n.endswith('.yaml'))
            chosen=[frames[i] for i in np.linspace(0,len(frames)-1,9).astype(int)]
            arrays=[]
            for frame in chosen:p,T=self.infra(take,frame);arrays.append(p)
            p=np.concatenate(arrays);_,idx=np.unique(np.floor(p/.15).astype(int),axis=0,return_index=True);p=p[idx]
            target_by_take[take]=(p,cKDTree(p),cKDTree(p[:,:2]));pr=plane(p);infra_report.append(dict(take=take,frames=chosen,point_count=len(p),native_T_infra=T.tolist(),plane=pr))
            np.savez_compressed(OUT/f'infra-take{take}-aggregate.npz',points_native=p,points_map_origin_only=transform(p,self.map_T_enu),native_T_infra=T,map_T_enu=self.map_T_enu,road_roi_native=np.array(ROI),road_anchor_native=ANCHOR)
        rows=[];details=[]
        for (run,offset),partition in sorted(selection.items()):
            attempt=ATTEMPTS/run/f'f{offset:06d}.json';r=json.loads(attempt.read_text());take,agent=map(int,run.split('-')[1:]);md=r['metadata'];frame=md['frame_id']
            with np.load(md['npz_path']) as f: native=source_points(f,True);N=f['dataset_T_lidar'].copy()
            native=native[::3]
            C=np.array(json.loads((RELEASE/str(take)/str(agent)/'icp_correction.json').read_text())[frame])
            physical=np.linalg.inv(self.map_T_enu)@np.array(self.refs[(run,offset)]['map_T_lidar'])@np.linalg.inv(N)
            variants={'released_refined':np.eye(4),'released_before_icp':np.linalg.inv(C),'physical_ins_gnss':physical}
            target,tree,tree2=target_by_take[take]
            # Same vehicle points for every variant. Geometric support selected once in released frame.
            support=(tree2.query(native[:,:2],workers=2)[0]<1.)&(native[:,2]>-6)&(native[:,2]<8)
            selected=native[support];refplane=infra_report[take-1]['plane']
            for name,T in variants.items():
                p=transform(selected,T);d=tree.query(p,workers=2)[0];pl=plane(transform(native,T))
                row=dict(run_id=run,frame_offset=offset,frame_id=frame,partition=partition,variant=name,source_count=len(native),common_support_count=len(selected),support_fraction=float(support.mean()),nn_median_m=float(np.median(d)),nn_p95_m=float(np.percentile(d,95)),nn_under_025m=float(np.mean(d<.25)),nn_under_050m=float(np.mean(d<.5)),plane_status=pl['status'],anchor_within_support_hull=pl.get('anchor_within_support_hull',False),plane_support_area_m2=pl.get('support_area_m2',0.),plane_angle_to_infra_deg=None,plane_height_minus_infra_m=None)
                if pl['status']=='ok' and refplane['status']=='ok':
                    row['plane_angle_to_infra_deg']=float(np.degrees(np.arccos(np.clip(np.dot(pl['normal'],refplane['normal']),-1,1))))
                    row['plane_height_minus_infra_m']=pl['height_at_anchor_m']-refplane['height_at_anchor_m']
                rows.append(row);details.append(dict(**row,plane=pl,native_variant_T_native=T.tolist()))
            print(run,offset,len(selected),flush=True)
        (OUT/'infra-planes.json').write_text(json.dumps(infra_report,indent=2));(OUT/'comparisons.json').write_text(json.dumps(details,indent=2))
        with (OUT/'comparisons.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
        (OUT/'manifest.json').write_text(json.dumps(dict(selection=[dict(run_id=k[0],frame_offset=k[1],partition=v) for k,v in sorted(selection.items())],infra_sources=self.manifest,road_roi_native=ROI,road_anchor_native=ANCHOR.tolist(),physical_reference_file=str(OLD/'axes-audit/physical-references.json'),limitations=['Geometry independent of GSplat but not independent of released ICP that used infrastructure.','Support points selected once in released frame; NN coverage is geometry consistency, not pose accuracy.','Same road patch and anchor for all planes; no flat-road assumption. Sparse limited road support and slopes must be checked.','Moving boxes removed with released annotations; residual motion and imperfect labels possible.']),indent=2))
if __name__=='__main__':InfrastructureAudit().run()

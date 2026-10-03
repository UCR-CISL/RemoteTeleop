"""Freeze one vehicle-road roll/pitch/Z map-tie calibration; evaluate eight cases.

Offline diagnostic, map-calibrated rather than independent ground truth. No
infrastructure data, per-evaluation-frame fitting, or runtime defaults changed.
"""
from __future__ import annotations
import csv
from dataclasses import asdict
import json
from pathlib import Path
import sys
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scripts.investigation.compare_reference_road_planes import transform_plane,height_at,inside
from scripts.investigation.first_frame_xyz import source_points
from src.localization.gaussian_map import load_gaussian_map
from src.localization.hybrid_road_height import HybridRoadHeightEstimator

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/reference-refinement/vehicle-road-calibration'
INPUT=ROOT/'artifacts/reference-refinement/road-comparison-with-cameras'
CALIBRATION=Path('/tmp/RemoteTeleop-hybrid-road-height/artifacts/full-sequence-localization/attempts/cooperscene-1-1/f000433.json')

class VehicleRoadCalibration:
    def __init__(self):
        OUT.mkdir(parents=True,exist_ok=True)
        self.fit=HybridRoadHeightEstimator()
        self.calibration=json.loads(CALIBRATION.read_text())
        gaussian=load_gaussian_map(self.calibration['metadata']['map_path'])
        keep=(gaussian.opacities>0)&np.isfinite(gaussian.means).all(axis=1)&(abs(gaussian.means).max(axis=1)<=10000)
        self.target=gaussian.means[keep].astype(float)
        self.tree=cKDTree(self.target)

    def road(self,points,lidar):
        balanced=self.fit._balanced(points)
        plane,quality,mask=self.fit._plane(balanced,lidar=lidar)
        return np.asarray(plane.normal),plane.offset_m,quality,balanced[mask]

    def calibrate(self):
        a=self.calibration
        if a['metadata']['frame_id']!='481693':raise ValueError('Unexpected predeclared calibration frame')
        with np.load(a['metadata']['npz_path']) as frame:
            source=source_points(frame,True);native_pose=frame['dataset_T_lidar'];W=frame['reference_map_T_dataset']
        origin=native_pose[:3,3];local=source-origin;rad=np.linalg.norm(local[:,:2],axis=1);c=self.fit.config
        candidates=local[(rad>=c.min_radius_m)&(rad<=c.max_radius_m)&(local[:,2]<=-c.lidar_below_sensor_m[0])&(local[:,2]>=-c.lidar_below_sensor_m[1])]
        sn,sd,sq,ss=self.road(candidates,True);sd-=sn@origin;ss+=origin
        source_n,source_d=transform_plane(sn,sd,W);source_support=ss@W[:3,:3].T+W[:3,3]
        anchor=(W@native_pose)[:3,3]
        candidates=self.target[(np.linalg.norm(self.target[:,:2]-anchor[:2],axis=1)<=15)&(self.target[:,2]>=-4.3)&(self.target[:,2]<=-1.3)]
        mn,md,mq,ms=self.road(candidates,False)
        _,ids=np.unique(np.floor(source_support[:,:2]),axis=0,return_index=True)
        overlap=float(np.mean(cKDTree(ms[:,:2]).query(source_support[ids,:2])[0]<=c.footprint_match_distance_m))
        supported=sq['extent_accepted'] and mq['extent_accepted'] and inside(source_support,anchor[:2]) and inside(ms,anchor[:2]) and overlap>=c.min_footprint_overlap
        if not supported:raise ValueError('Predeclared calibration road plane fails common support gates')
        def rotation(x):return Rotation.from_euler('xyz',[x[0],x[1],0]).as_matrix()
        solution=least_squares(lambda x:rotation(x)@source_n-mn,[0.,0.],xtol=1e-13,ftol=1e-13,gtol=1e-13)
        R=rotation(solution.x)
        if not solution.success or np.linalg.norm(R@source_n-mn)>1e-8:raise ValueError('Could not align normals with zero-yaw rotation')
        D=np.eye(4);D[:3,:3]=R;D[2,3]=(source_d-md)/mn[2]
        self.D=D
        result=dict(status='frozen_before_heldout_evaluation',calibration_run='cooperscene-1-1',calibration_offset=433,calibration_frame_id='481693',calibration_attempt=str(CALIBRATION),predeclared_reason='Original released ICP anchor for take1agent1; excluded from all eight evaluation cases.',classification='map-calibrated candidate, not independent ground truth',infrastructure_input=False,composition='candidate_map_T_native = D @ original_W',rotation_parameterization='R=Ry(pitch)@Rx(roll), Euler xyz yaw fixed to zero',translation_constraint='D.tx=D.ty=0; rotation pivot is fixed map origin; only D.tz fitted',D=D.tolist(),roll_pitch_yaw_deg=Rotation.from_matrix(R).as_euler('xyz',degrees=True).tolist(),translation_m=D[:3,3].tolist(),calibration_xy_map=anchor[:2].tolist(),source_plane_map=dict(normal=source_n.tolist(),offset=float(source_d),quality=sq),map_plane=dict(normal=mn.tolist(),offset=float(md),quality=mq),common_support_overlap=overlap,common_support_passed=bool(supported),map_band_z_m=[-4.3,-1.3],plane_fit_config=asdict(c),calibration_normal_fit_error=float(np.linalg.norm(R@source_n-mn)),limitations=['One local vehicle road plane constrains roll/pitch/Z only. Yaw and XY translation held fixed.','Map road candidate band is an explicit diagnostic semantic assumption; road slope measured, not leveled.','Calibration can absorb local pose error or road geometry error; no guarantee of global frame correction.','No choice among calibration frames based on heldout errors; all eight evaluation cases predetermined.'])
        (OUT/'frozen-calibration.json').write_text(json.dumps(result,indent=2)+'\n')
        np.savez_compressed(OUT/'calibration-road-support.npz',source_support_map=source_support,map_support=ms,comparison_anchor_map=anchor)
        print('Frozen D',result['roll_pitch_yaw_deg'],result['translation_m'],flush=True)

    def evaluate(self):
        rows=[]
        for record in json.loads((INPUT/'diagnostics.json').read_text()):
            if record['run_id']=='cooperscene-1-1' and record['frame_offset']==433:raise ValueError('Calibration leaked into evaluation')
            a=json.loads(Path(record['attempt_path']).read_text())
            with np.load(a['metadata']['npz_path']) as frame:
                source=source_points(frame,True);W=frame['reference_map_T_dataset'];native_pose=frame['dataset_T_lidar']
            name=f"{record['run_id']}-f{record['frame_offset']:06d}"
            with np.load(INPUT/name/'road-support.npz') as support:ns=support['native_support'];ms=support['map_support']
            xy=np.asarray(record['comparison_xy_map']);mp=record['map_road'];sp=record['source_road'];mn=np.asarray(mp['normal']);mh=height_at(mn,mp['offset'],xy)
            for variant,T in [('released',W),('single_vehicle_road_map_tie',self.D@W)]:
                n,d=transform_plane(sp['normal'],sp['offset'],T);s=ns@T[:3,:3].T+T[:3,3]
                _,ids=np.unique(np.floor(s[:,:2]),axis=0,return_index=True)
                overlap=float(np.mean(cKDTree(ms[:,:2]).query(s[ids,:2])[0]<=self.fit.config.footprint_match_distance_m))
                source_inside=inside(s,xy);map_inside=inside(ms,xy)
                xyz=source@T[:3,:3].T+T[:3,3];dist=self.tree.query(xyz,workers=2)[0]
                delta=(T@native_pose)[:3,3]-(W@native_pose)[:3,3]
                rows.append(dict(run_id=record['run_id'],frame_offset=record['frame_offset'],frame_id=record['frame_id'],variant=variant,normal_angle_deg=float(np.degrees(np.arccos(np.clip(n@mn,-1,1)))),signed_road_height_separation_m=height_at(n,d,xy)-mh,comparison_xy_map=xy.tolist(),source_support_contains_anchor=source_inside,map_support_contains_anchor=map_inside,road_footprint_overlap=overlap,road_comparison_supported=bool(record['map_support_status']=='supported' and source_inside and map_inside and overlap>=self.fit.config.min_footprint_overlap),static_scan_point_count=len(source),full_scan_nn_median_m=float(np.median(dist)),full_scan_nn_p95_m=float(np.percentile(dist,95)),lidar_origin_shift_map_m=delta.tolist(),candidate_map_T_dataset=T.tolist()))
            print(name,[(r['variant'],round(r['normal_angle_deg'],3),round(r['signed_road_height_separation_m'],3),round(r['full_scan_nn_median_m'],3)) for r in rows[-2:]],flush=True)
        (OUT/'heldout-comparisons.json').write_text(json.dumps(rows,indent=2)+'\n')
        scalar=[{k:v for k,v in row.items() if not isinstance(v,list)} for row in rows]
        with (OUT/'heldout-comparisons.csv').open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(scalar[0]));writer.writeheader();writer.writerows(scalar)
        summary={}
        for variant in ['released','single_vehicle_road_map_tie']:
            group=[r for r in rows if r['variant']==variant]
            summary[variant]=dict(frames=len(group),supported_planes=sum(r['road_comparison_supported'] for r in group),median_normal_angle_deg=float(np.median([r['normal_angle_deg'] for r in group])),median_absolute_road_height_m=float(np.median([abs(r['signed_road_height_separation_m']) for r in group])),median_frame_nn_median_m=float(np.median([r['full_scan_nn_median_m'] for r in group])),median_frame_nn_p95_m=float(np.median([r['full_scan_nn_p95_m'] for r in group])))
        for key in ['normal_angle_deg','signed_road_height_separation_m','full_scan_nn_median_m','full_scan_nn_p95_m']:
            summary[key+'_improved_frames']=sum(abs(rows[i+1][key])<abs(rows[i][key]) for i in range(0,len(rows),2))
        (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))

if __name__=='__main__':
    experiment=VehicleRoadCalibration()
    try:experiment.calibrate()
    except ValueError as error:
        (OUT/'frozen-calibration.json').write_text(json.dumps(dict(status='calibration_failed',reason=str(error)),indent=2));raise
    experiment.evaluate()

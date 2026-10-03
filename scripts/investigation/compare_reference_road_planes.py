"""Same-frame reference comparisons with measured local road-plane diagnostics.

No poses are fitted here. Map-road candidate band is an explicit visualization
assumption, not a localization gate or a ground-truth height constraint.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import ConvexHull, cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.investigation.first_frame_xyz import source_points, jsonable
from src.localization.gaussian_map import load_gaussian_map
from src.localization.hybrid_road_height import HybridRoadHeightEstimator
from src.viz.localization_visualizer import render_gaussian_camera_buffers

ATTEMPTS = Path('/tmp/RemoteTeleop-hybrid-road-height/artifacts/full-sequence-localization/attempts')
PHYSICAL = Path('/tmp/RemoteTeleop-reference-orientation/artifacts/reference-orientation/axes-audit/physical-references.json')
CASES = [('cooperscene-1-3',464),('cooperscene-2-3',871),('cooperscene-3-1',0),
         ('cooperscene-3-3',500),('cooperscene-5-1',180),('cooperscene-5-3',71),
         ('cooperscene-4-2',134),('cooperscene-1-1',0)]


def transform_plane(normal, offset, transform):
    """Transform n·p+d=0 using target_T_source, including translation."""
    n = np.asarray(transform)[:3,:3] @ np.asarray(normal)
    return n, float(offset - n @ np.asarray(transform)[:3,3])


def height_at(normal, offset, xy):
    normal=np.asarray(normal)
    if abs(normal[2]) < 1e-8:
        raise ValueError('Plane has no finite vertical height')
    return float(-(normal[:2] @ xy + offset)/normal[2])


def inside(points, xy):
    if len(points)<3:
        return False
    hull=ConvexHull(points[:,:2])
    return bool(np.all(hull.equations[:,:2]@xy+hull.equations[:,2] <= 1e-7))


class ReferenceRoadComparison:
    def __init__(self, output, physical, repaired=None, cameras=True, map_tie=None):
        self.output=output;output.mkdir(parents=True,exist_ok=True)
        self.physical={(r['run_id'],r['frame_offset']):r for r in json.loads(physical.read_text())['records']}
        self.repaired={} if repaired is None else {(r['run_id'],r['frame_offset']):r for r in json.loads(repaired.read_text())['records']}
        self.cameras=cameras
        self.map_tie = None if map_tie is None else np.asarray(json.loads(map_tie.read_text())['D'])
        self.fit=HybridRoadHeightEstimator()
        self.map_path=None
        self.records=[]

    def road(self, points, lidar):
        balanced=self.fit._balanced(points)
        plane,quality,mask=self.fit._plane(balanced,lidar=lidar)
        support=balanced[mask]
        return dict(normal=list(plane.normal),offset=plane.offset_m,quality=quality),support

    def run(self, run_id, offset):
        path=ATTEMPTS/run_id/f'f{offset:06d}.json'
        a=json.loads(path.read_text());md=a['metadata'];frame=dict(np.load(md['npz_path'],allow_pickle=False))
        if self.map_path!=md['map_path']:
            self.gaussian=load_gaussian_map(md['map_path'])
            valid=(self.gaussian.opacities>0)&np.isfinite(self.gaussian.means).all(axis=1)&(abs(self.gaussian.means).max(axis=1)<=10000)
            self.target=self.gaussian.means[valid].astype(float);self.map_path=md['map_path']
        ref=frame['reference_map_T_dataset']
        transforms=[('released','Released ICP-refined + supplied ENU map tie',ref,None,'#1698b5'),
                    ('physical','Normalized INS + body-extrinsic hypothesis',np.asarray(self.physical[(run_id,offset)]['map_T_dataset']),None,'#289a53')]
        if self.repaired:
            transforms.append(('repaired','ICP-anchor + normalized sensor-motion hypothesis',np.asarray(self.repaired[(run_id,offset)]['map_T_dataset']),None,'#99621a'))
        if self.map_tie is not None:
            transforms.append(('map_tied_release','Released + fixed vehicle-calibrated map tie',self.map_tie@ref,None,'#bb9d11'))
        for method,label,color in [('xyz','Existing XYZ estimate','#de5625'),('bev_lidar_height','Existing BEV + height estimate','#9c4abd')]:
            m=a['methods'].get(method,{})
            if 'map_T_dataset' in m:
                transforms.append((method,label,np.asarray(m['map_T_dataset']),m.get('accepted',False),color))
        source=source_points(frame,mask_dynamic=True)
        lidar_origin=frame['dataset_T_lidar'][:3,3]
        local=source-lidar_origin;rad=np.linalg.norm(local[:,:2],axis=1);c=self.fit.config
        candidates=local[(rad>=c.min_radius_m)&(rad<=c.max_radius_m)&(local[:,2]<=-c.lidar_below_sensor_m[0])&(local[:,2]>=-c.lidar_below_sensor_m[1])]
        native_plane,native_support=self.road(candidates,True)
        native_plane['offset']-=np.asarray(native_plane['normal'])@lidar_origin
        native_support+=lidar_origin
        anchor=(ref@frame['dataset_T_lidar'])[:3,3]
        map_candidates=self.target[(np.linalg.norm(self.target[:,:2]-anchor[:2],axis=1)<=15)&(self.target[:,2]>=-4.3)&(self.target[:,2]<=-1.3)]
        map_plane=None;map_support=np.empty((0,3));map_reason=None
        try:
            map_plane,map_support=self.road(map_candidates,False)
            if not map_plane['quality']['extent_accepted']:
                map_reason='insufficient_2d_map_road_extent'
            elif not inside(map_support,anchor[:2]):
                map_reason='comparison_location_outside_map_road_support'
        except ValueError as error:
            map_reason=str(error)
        name=f'{run_id}-f{offset:06d}';directory=self.output/name;directory.mkdir(exist_ok=True)
        record=dict(run_id=run_id,frame_offset=offset,frame_id=md['frame_id'],attempt_path=str(path),comparison_xy_map=anchor[:2].tolist(),
                    source_road=native_plane,map_road=map_plane,map_support_status=map_reason or 'supported',
                    map_candidate_selection=dict(radius_m=15,height_band_m=[-4.3,-1.3],opacity='positive, matching registration input',
                       note='Explicit broad map-road height band around previously visualized Riverside road. No reference/estimated sensor height sets this band. Plane slope is fitted, not forced to gravity. Fit is geometric, not semantic proof.'),
                    source_mask='Recorded dynamic/parked vehicle boxes padded0.2m, same as saved registration.',variants=[],
                    pose_changes=False,acceptance_gates_changed=False,independent_ground_truth=False)
        clouds={}
        for key,label,T,accepted,color in transforms:
            n,d=transform_plane(native_plane['normal'],native_plane['offset'],T)
            support=native_support@T[:3,:3].T+T[:3,3]
            clouds[key]=source@T[:3,:3].T+T[:3,3]
            row=dict(method=key,label=label,saved_registration_accepted=accepted,normal=n.tolist(),offset=d,
                     source_height_at_comparison_xy_m=height_at(n,d,anchor[:2]),source_support_contains_comparison=inside(support,anchor[:2]))
            if map_plane:
                mn=np.asarray(map_plane['normal']);mh=height_at(mn,map_plane['offset'],anchor[:2])
                _,ids=np.unique(np.floor(support[:,:2]),axis=0,return_index=True)
                overlap=float(np.mean(cKDTree(map_support[:,:2]).query(support[ids,:2])[0]<=2.))
                row.update(map_height_at_comparison_xy_m=mh,
                           signed_road_height_separation_m=row['source_height_at_comparison_xy_m']-mh,
                           normal_angle_deg=float(np.rad2deg(np.arccos(np.clip(n@mn,-1,1)))),
                           road_footprint_overlap_2m=overlap,
                           road_comparison_supported=bool(map_reason is None and row['source_support_contains_comparison'] and overlap>=c.min_footprint_overlap),
                           slope_difference_dz_dx_dy=(-n[:2]/n[2]+mn[:2]/mn[2]).tolist())
            record['variants'].append(row)
        np.savez_compressed(directory/'road-support.npz',native_support=native_support,map_support=map_support,comparison_anchor=anchor)
        (directory/'diagnostics.json').write_text(json.dumps(jsonable(record),indent=2)+'\n')
        self.cloud_views(directory,record,transforms,clouds,anchor)
        if self.cameras:self.camera_views(directory,md,frame,transforms)
        self.records.append(record)
        (self.output/'diagnostics.json').write_text(json.dumps(jsonable(self.records),indent=2)+'\n')
        print(name,[(v['method'],round(v.get('normal_angle_deg',float('nan')),2),round(v.get('signed_road_height_separation_m',float('nan')),3),v.get('road_comparison_supported')) for v in record['variants']],flush=True)

    def cloud_views(self,directory,record,transforms,clouds,anchor):
        target=self.target-anchor
        target=target[(abs(target[:,:2])<=60).all(axis=1)&(abs(target[:,2])<=18)]
        rng=np.random.default_rng(0);target=target[rng.choice(len(target),min(120000,len(target)),replace=False)]
        ids=rng.choice(len(next(iter(clouds.values()))),min(45000,len(next(iter(clouds.values())))),replace=False)
        fig,axes=plt.subplots(len(transforms),3,figsize=(18,3.7*len(transforms)),layout='constrained')
        for row,((key,label,T,accepted,color),metrics) in enumerate(zip(transforms,record['variants'])):
            cloud=clouds[key][ids]-anchor
            suffix='' if accepted is None else (' [accepted]' if accepted else ' [REJECTED diagnostic]')
            for column,((x,y),ylim) in enumerate([((0,1),(-55,55)),((0,2),(-8,8)),((1,2),(-8,8))]):
                ax=axes[row,column]
                ax.scatter(target[:,x],target[:,y],s=.25,c='#68727d',alpha=.3,rasterized=True)
                ax.scatter(cloud[:,x],cloud[:,y],s=.4,c=color,alpha=.5,rasterized=True)
                ax.set(xlim=(-55,55) if column==0 else (-30,30),ylim=ylim,xlabel=f'Map {"XYZ"[x]} from common anchor (m)',ylabel=f'{"XYZ"[y]} (m)')
                ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2)
                if column:
                    xx=np.linspace(-15,15,100);xy=np.repeat(anchor[None,:2],100,axis=0);xy[:,x]+=xx
                    ns=np.asarray(metrics['normal']);zs=-(xy@ns[:2]+metrics['offset'])/ns[2]-anchor[2]
                    ax.plot(xx,zs,c=color,lw=2)
                    if record['map_road']:
                        mp=record['map_road'];mn=np.asarray(mp['normal']);zm=-(xy@mn[:2]+mp['offset'])/mn[2]-anchor[2];ax.plot(xx,zm,'k--',lw=1.5)
                    ax.set_title('Plane cuts through common XY; dashed black = map')
                else:ax.set_title(label+suffix,fontsize=10)
            if 'normal_angle_deg' in metrics:
                axes[row,2].set_xlabel(f"Normal Δ {metrics['normal_angle_deg']:.2f}°; road ΔZ {metrics['signed_road_height_separation_m']:+.3f}m\n"+('Supported common location' if metrics['road_comparison_supported'] else 'LIMITED SUPPORT / extrapolation — diagnostic only'))
        fig.suptitle(f"{record['run_id']} | frame {record['frame_offset']} | identical scan for every method\nReference candidates are not certified ground truth; registration acceptance is unchanged",fontsize=15)
        fig.savefig(directory/'clouds.png',dpi=130);plt.close(fig)

    @staticmethod
    def label(image,text):
        image=cv2.copyMakeBorder(image,35,0,0,0,cv2.BORDER_CONSTANT,value=(248,248,248))
        cv2.putText(image,text,(8,23),cv2.FONT_HERSHEY_SIMPLEX,.43,(10,10,10),1,cv2.LINE_AA)
        return image

    def camera_views(self,directory,metadata,frame,transforms):
        original=cv2.imread(metadata['camera_path']);height,width=original.shape[:2]
        dist=frame['camera_distortion']
        if dist.size and np.any(dist):original=cv2.undistort(original,frame['camera_intrinsic'],dist)
        original=cv2.resize(original,(width//4,height//4));rows=[];stats=[]
        for key,label,T,accepted,color in transforms:
            render=render_gaussian_camera_buffers(self.gaussian,T@frame['dataset_T_camera'],frame['camera_intrinsic'],(width,height),downsample=4,device='cuda')
            rgb=np.clip(render.rgb[...,::-1]*255,0,255).astype(np.uint8)
            overlay=cv2.addWeighted(original,.5,rgb,.5,0)
            status='' if accepted is None else (' accepted' if accepted else ' REJECTED')
            rows.append(np.hstack([self.label(original,key+status+' / recorded'),self.label(rgb,label),self.label(overlay,'50% camera / 50% GSplat')]))
            stats.append(dict(method=key,backend=render.backend,saved_registration_accepted=accepted))
        cv2.imwrite(str(directory/'camera-overlays.png'),np.vstack(rows))
        (directory/'camera-overlays.json').write_text(json.dumps(stats,indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('artifacts/reference-refinement/road-comparison'))
    parser.add_argument('--physical',type=Path,default=PHYSICAL)
    parser.add_argument('--repaired',type=Path)
    parser.add_argument('--no-cameras',action='store_true')
    parser.add_argument('--map-tie',type=Path)
    args=parser.parse_args();runner=ReferenceRoadComparison(args.output,args.physical,args.repaired,not args.no_cameras,args.map_tie)
    for case in CASES:runner.run(*case)

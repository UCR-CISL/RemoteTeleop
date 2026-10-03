"""Forensic infrastructure versus GSplat road comparison; never fits a pose."""
from pathlib import Path
import json
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scripts.investigation.compare_reference_road_planes import transform_plane,height_at,inside
from src.localization.gaussian_map import load_gaussian_map
from src.localization.hybrid_road_height import HybridRoadHeightEstimator


def main():
    output=Path(__file__).resolve().parents[2]/'artifacts/reference-refinement/infrastructure-audit'
    infrastructure=json.loads((output/'infra-planes.json').read_text())
    with np.load(output/'infra-take1-aggregate.npz') as data:
        W=data['map_T_enu'];roi=data['road_roi_native'];native_anchor=data['road_anchor_native']
    anchor=(W@np.r_[native_anchor,0,1])[:2]
    gaussian=load_gaussian_map('/home/jyue86/Downloads/riverside_for_justin/riverside_iowa.ply')
    p=gaussian.means.astype(float);native=(p-W[:3,3])@W[:3,:3]
    keep=(gaussian.opacities>0)&np.isfinite(p).all(1)&(native[:,0]>roi[0])&(native[:,0]<roi[1])&(native[:,1]>roi[2])&(native[:,1]<roi[3])&(p[:,2]>=-4.3)&(p[:,2]<=-1.3)
    fit=HybridRoadHeightEstimator();balanced=fit._balanced(p[keep]);plane,quality,mask=fit._plane(balanced,lidar=False);support=balanced[mask]
    map_n=np.array(plane.normal);map_h=height_at(map_n,plane.offset_m,anchor)
    records=[]
    for take in infrastructure:
        source=take['plane'];n=np.array(source['normal']);coef=np.array(source['z_ax_by_c']);d=-coef[2]*n[2]
        mapped_n,mapped_d=transform_plane(n,d,W)
        hull=np.array(source['support_hull_xy']);hull3=np.c_[hull,np.c_[hull,np.ones(len(hull))]@coef];mapped_hull=hull3@W[:3,:3].T+W[:3,3]
        source_inside=inside(mapped_hull,anchor);map_inside=inside(support,anchor)
        records.append(dict(take=take['take'],infra_normal_map=mapped_n.tolist(),infra_offset_map=mapped_d,infra_height_at_common_xy_m=height_at(mapped_n,mapped_d,anchor),map_height_at_common_xy_m=map_h,signed_infra_minus_map_height_m=height_at(mapped_n,mapped_d,anchor)-map_h,normal_angle_deg=float(np.degrees(np.arccos(np.clip(mapped_n@map_n,-1,1)))),infra_support_contains_anchor=source_inside,map_support_contains_anchor=map_inside,supported=bool(source_inside and map_inside and quality['extent_accepted']),infra_fit_rms_m=source['rms_m']))
    result=dict(purpose='Offline forensic attribution only. Infrastructure is not required by localization or candidate reference construction.',map_T_released_world=W.tolist(),comparison_xy_native=native_anchor.tolist(),comparison_xy_map=anchor.tolist(),native_roi=roi.tolist(),map_candidate_band_z_m=[-4.3,-1.3],map_candidate_count=int(keep.sum()),map_plane=dict(normal=list(plane.normal),offset=plane.offset_m,quality=quality),records=records,limitations=['Supplied origin-only map tie retained unchanged; no transform or leveling is fitted.','Height band is a diagnostic semantic assumption around visible Riverside road, not an independent road segmentation or ground truth.','Measured disagreement can reflect infrastructure anchor calibration, map georeference/geometry, local road surface selection, or their combination.','Infrastructure raw GNSS is copied vehicle data; independent survey accuracy is not established.'])
    (output/'infra-versus-map-road.json').write_text(json.dumps(result,indent=2)+'\n')
    np.savez_compressed(output/'infra-versus-map-road-support.npz',map_support=support,map_candidates=balanced,comparison_xy_map=anchor)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()

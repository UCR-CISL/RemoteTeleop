"""Check cached MCAP poses/points against native archives and quantify tilt effects."""
from __future__ import annotations
import io
import json
from pathlib import Path
import sys
import zipfile

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import numpy as np
import yaml
from src.localization.cooperscene import pose_to_matrix
from src.localization.cooperscene_calibration import camera_T_lidar, camera_intrinsic
from src.localization.gaussian_map import load_gaussian_map


def main():
    root=Path('artifacts/first-frame-localization')
    mounted=Path('/mnt/bcc-data/websites/data.ucr.edu-datasets-cooperscene')
    records=json.loads((root/'inputs/manifest.json').read_text())
    rows=[]
    archives={}
    try:
        for record in records:
            if record['dataset']!='cooperscene':
                continue
            take,agent,frame_id=record['scene'],record['agent'],record['frame_id']
            if take not in archives:
                path=mounted/('test' if take=='2' else 'train')/f'{take}.zip'
                archives[take]=zipfile.ZipFile(path)
            archive=archives[take]
            prefix=f'{take}/{agent}/{frame_id}'
            annotation=yaml.safe_load(archive.read(prefix+'.yaml'))
            payload=archive.read(prefix+'.pcd').split(b'DATA ascii\n',1)[1]
            native=np.loadtxt(io.BytesIO(payload),dtype=np.float32)
            cached=np.load(record['npz_path'])
            native_pose=pose_to_matrix(annotation['lidar_pose'])
            actual_extrinsic=np.linalg.inv(cached['dataset_T_camera'])@cached['dataset_T_lidar']
            inherited=cached['inherited_map_T_dataset']
            reference=cached['reference_map_T_dataset']
            position=cached['dataset_T_lidar'][:3,3]
            tilt=(inherited[:3,:3]-reference[:3,:3])@position
            offset=inherited[:3,3]-reference[:3,3]
            rows.append(dict(run_id=record['run_id'],frame_id=frame_id,
                native_archive=archive.filename,point_count=len(native),
                points_same_shape=native.shape==cached['points_lidar'].shape,
                points_max_abs_difference=float(np.max(np.abs(native-cached['points_lidar']))),
                pose_max_abs_difference=float(np.max(np.abs(native_pose-cached['dataset_T_lidar']))),
                camera_extrinsic_max_abs_difference=float(np.max(np.abs(actual_extrinsic-camera_T_lidar(agent)))),
                intrinsic_max_abs_difference=float(np.max(np.abs(cached['camera_intrinsic']-camera_intrinsic(agent)))),
                lidar_position_dataset=position.tolist(),
                inherited_vertical_rotation_contribution_m=float(tilt[2]),
                inherited_vertical_translation_contribution_m=float(offset[2]),
                inherited_vertical_total_m=float((tilt+offset)[2])))
            print(record['run_id'],rows[-1]['points_max_abs_difference'],flush=True)
    finally:
        for archive in archives.values():
            archive.close()
    maps=[]
    for path in dict.fromkeys(record['map_path'] for record in records):
        gaussian_map=load_gaussian_map(path)
        opacity=gaussian_map.opacities
        finite=np.isfinite(gaussian_map.means).all(axis=1)&np.isfinite(opacity)
        finite&=np.abs(gaussian_map.means).max(axis=1)<=10000
        maps.append(dict(path=path,count=len(opacity),valid_count=int(finite.sum()),
            positive_opacity_count=int(np.sum(finite&(opacity>0))),
            opacity_above_005_count=int(np.sum(finite&(opacity>.05))),
            opacity_quantiles=np.quantile(opacity[finite],[0,.01,.1,.5,.9,.99,1]).tolist()))
    (root/'input-audit.json').write_text(json.dumps(dict(cooperscene_native_checks=rows,map_opacity=maps),indent=2)+'\n')


if __name__=='__main__':
    main()

"""Compare processed-release coordinates with native CooperScene code, read-only."""
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import zipfile
import numpy as np
import open3d as o3d
import yaml
from scipy.spatial import cKDTree
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from src.localization.cooperscene import pose_to_matrix, read_ascii_xyzi_pcd

NATIVE=Path('/home/jyue86/Documents/CISL-Projects/RemoteTeleop/thirdparty/CooperScene')
OUT=ROOT/'artifacts/reference-refinement/processed-conventions-audit'


def native_functions(path, names):
    tree=ast.parse(path.read_text())
    subset=ast.Module(body=[node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name in names],type_ignores=[])
    scope={'np':np}
    exec(compile(subset,str(path),'exec'),scope)
    return scope


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    path=NATIVE/'models/cooperative/datasets/coop_dataset.py'
    native=native_functions(path,{'pose_to_matrix'})['pose_to_matrix']
    util=NATIVE/'vendor/opencood/utils/transformation_utils.py'
    native_relative=native_functions(util,{'x_to_world','x1_to_x2'})['x1_to_x2']
    selection=json.loads((ROOT/'artifacts/reference-refinement/overlap-selected-calibration/selection/manifest.json').read_text())
    archives={r['take']:r['archive'] for r in selection['per_take_selected']}
    points_to_check={(1,0,'481510'),(1,1,'481677'),(2,2,'488685'),(2,3,'488684'),(2,3,'488664'),(2,3,'488700'),(2,3,'488702')}
    exported=np.load(ROOT/'artifacts/reference-refinement/overlap-selected-calibration/selection/selected-clouds.npz')
    temporal=np.load(ROOT/'artifacts/reference-refinement/structure-experiment/temporal/window-clouds.npz')
    temporal_rows=json.loads((ROOT/'artifacts/reference-refinement/structure-experiment/temporal/window-manifest.json').read_text())
    temporal_keys={(r['take'],r['agent'],r['frame_id']):r['key'] for r in temporal_rows}
    selected_keys={(1,0,'481510'):'infra_representative',(1,1,'481677'):'global_1',(2,2,'488685'):'global_2',(2,3,'488684'):'global_3'}
    poses=[];points=[]
    with tempfile.TemporaryDirectory(prefix='processed-conventions-') as temporary:
        for take,archive_path in sorted(archives.items()):
            with zipfile.ZipFile(archive_path) as archive:
                for agent in range(4):
                    frames=sorted(n.split('/')[-1][:-5] for n in archive.namelist() if n.startswith(f'{take}/{agent}/') and n.endswith('.yaml'))
                    chosen={frames[0],frames[len(frames)//2],frames[-1]}|{f for t,a,f in points_to_check if (t,a)==(take,agent)}
                    for frame in sorted(chosen):
                        annotation=yaml.load(archive.read(f'{take}/{agent}/{frame}.yaml'),Loader=yaml.CSafeLoader)
                        raw=annotation['lidar_pose'];ours=pose_to_matrix(raw);theirs=native(raw)
                        infra=yaml.load(archive.read(f'{take}/0/{frame}.yaml'),Loader=yaml.CSafeLoader)
                        target=native(infra['lidar_pose'])
                        rel=np.linalg.inv(pose_to_matrix(infra['lidar_pose']))@ours
                        expected=native_relative(theirs,target)
                        row=dict(take=take,agent=agent,frame_id=frame,pose_shape=list(np.shape(raw)),pose_max_abs_difference=float(abs(ours-theirs).max()),relative_pose_max_abs_difference=float(abs(rel-expected).max()),rotation_determinant=float(np.linalg.det(ours[:3,:3])),orthonormal_max_error=float(abs(ours[:3,:3].T@ours[:3,:3]-np.eye(3)).max()))
                        poses.append(row)
                        if (take,agent,frame) in points_to_check:
                            p=Path(temporary)/'scan.pcd';p.write_bytes(archive.read(f'{take}/{agent}/{frame}.pcd'))
                            xyz=read_ascii_xyzi_pcd(p)[:,:3]
                            native_xyz=o3d.t.io.read_point_cloud(str(p)).point.positions.numpy()
                            # Independent native-style homogeneous column product versus ours row product.
                            homogeneous=np.c_[native_xyz,np.ones(len(native_xyz))]
                            world_native=(theirs@homogeneous.T).T[:,:3]
                            world_ours=xyz@ours[:3,:3].T+ours[:3,3]
                            relative_native=(expected@homogeneous.T).T[:,:3]
                            relative_ours=(world_ours-target[:3,3])@target[:3,:3]
                            identity=(take,agent,frame)
                            stored=exported[selected_keys[identity]] if identity in selected_keys else temporal[temporal_keys[identity]]
                            W=exported['original_W']; displayed=world_native@W[:3,:3].T+W[:3,3]
                            exported_error=float(cKDTree(displayed).query(stored)[0].max())
                            points.append(dict(exported_static_cloud_max_nearest_native_difference_m=exported_error,take=take,agent=agent,frame_id=frame,count=len(xyz),local_xyz_max_difference_m=float(abs(xyz-native_xyz).max()),world_xyz_max_difference_m=float(abs(world_native-world_ours).max()),infra_relative_xyz_max_difference_m=float(abs(relative_native-relative_ours).max())))
            print('Audited take',take,flush=True)
    rng=np.random.default_rng(18);synthetic=rng.uniform(-90,90,(100,6))
    synthetic_diff=max(float(abs(pose_to_matrix(p)-native(p)).max()) for p in synthetic)
    summary=dict(pose_count=len(poses),point_cloud_count=len(points),point_count=sum(r['count'] for r in points),all_pose_shapes=sorted({str(r['pose_shape']) for r in poses}),maximum_pose_difference=max(r['pose_max_abs_difference'] for r in poses),maximum_relative_pose_difference=max(r['relative_pose_max_abs_difference'] for r in poses),maximum_local_xyz_difference_m=max(r['local_xyz_max_difference_m'] for r in points),maximum_world_xyz_difference_m=max(r['world_xyz_max_difference_m'] for r in points),maximum_relative_xyz_difference_m=max(r['infra_relative_xyz_max_difference_m'] for r in points),maximum_exported_static_cloud_difference_m=max(r['exported_static_cloud_max_nearest_native_difference_m'] for r in points),synthetic_six_value_pose_maximum_difference=synthetic_diff)
    result=dict(summary=summary,poses=poses,point_clouds=points,native_revision=subprocess.check_output(['git','-C',str(NATIVE),'rev-parse','HEAD'],text=True).strip(),native_source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [path,util]},method='Execute exact native top-level pose/relative-transform function ASTs without detector framework; Open3D tensor PCD reader used by native devkit; no ICP/extrinsics applied to processed PCD/YAML',source_archives_read_only=True)
    (OUT/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(summary,indent=2))
    assert summary['maximum_pose_difference']<1e-12
    assert summary['maximum_local_xyz_difference_m']<1e-6
    assert summary['maximum_relative_xyz_difference_m']<1e-9
    assert summary['maximum_exported_static_cloud_difference_m']<1e-4


if __name__=='__main__':main()

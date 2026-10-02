"""Fixed-frame offline comparison of existing replay poses and road-height proposals."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[name] = '2'
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation

from scripts.investigation.advance_cooperscene_frames import IndexedCooperFrameReader
from scripts.investigation.advance_nuscenes_frames import NuScenesFrameAdvance
from scripts.investigation.first_frame_xyz import source_points, pose_error, registration_config, jsonable
from src.localization.gaussian_map import load_gaussian_map
from src.localization.hybrid_road_height import HybridRoadHeightEstimator
from src.localization.registration import PointCloudRegistrar


class HybridComparison:
    """Own immutable inputs, estimation-before-evaluation, and original acceptance gates."""

    def __init__(self, previous: Path, output: Path):
        self.previous, self.output = previous, output
        self.first = previous / 'first-frame-localization'
        self.entries = {r['run_id']: r for r in json.loads((self.first / 'inputs/manifest.json').read_text())}
        self.manifests = {}
        self.coverage = {}
        for method, directory in [('xyz', 'set1-xyz-bevfusion'), ('bev', 'set2-bev-grounded')]:
            path = previous / 'localization-evaluation' / directory
            data = json.loads((path / 'manifest.json').read_text())
            self.manifests[method] = (path, {r['id']: r for r in data['runs']})
            self.coverage[method] = {r['id']: r for r in data['evaluation']['coverage']}
        self.nusc = NuScenesFrameAdvance(output / 'staging/nuscenes', [], ['baseline'])
        self.nusc.original = self.first
        self.map_path = None
        self.estimator = HybridRoadHeightEstimator()

    @staticmethod
    def write(path, data):
        path.parent.mkdir(parents=True, exist_ok=True)
        temp = path.with_suffix('.tmp')
        temp.write_text(json.dumps(jsonable(data), indent=2, allow_nan=False) + '\n')
        temp.replace(path)

    def frame(self, run_id, offset):
        if offset == 0:
            return dict(self.entries[run_id])
        candidates = [self.previous / 'frame-advance-localization/nuscenes/inputs' / f'{run_id}-f{offset:03d}',
                      self.previous / 'frame-advance-localization/cooperscene/inputs' / f'{run_id}-f{offset:06d}',
                      self.previous / 'bev-frame-search/cooperscene-cache/inputs' / f'{run_id}-f{offset:06d}',
                      self.previous / 'bev-frame-search/nuscenes-cache/inputs' / f'{run_id}-f{offset:03d}']
        candidates += [ROOT / 'artifacts/hybrid-road-height/staging/nuscenes/inputs' / f'{run_id}-f{offset:03d}',
                       ROOT / 'artifacts/hybrid-road-height/staging/cooperscene/inputs' / f'{run_id}-f{offset:06d}']
        for directory in candidates:
            if (directory / 'metadata.json').exists():
                return json.loads((directory / 'metadata.json').read_text())
        if run_id.startswith('nuscenes'):
            scene = self.entries[run_id]['scene']
            copied = self.nusc.output / 'recorded' / scene
            if not copied.exists():
                shutil.copytree(self.previous / 'frame-advance-localization/nuscenes/recorded' / scene, copied)
            required = ['scene_info.json', f'lidar/{offset:03d}.bin', f'lidar_pose/{offset:03d}.txt',
                        f'extrinsics/{offset:03d}_0.txt', f'images/{offset:03d}_0.jpg']
            if any(not (copied / name).exists() for name in required):
                raise FileNotFoundError(f'Missing local cache for {run_id} frame {offset}; remote fetch disabled')
            return self.nusc._stage(scene, offset)
        reader = IndexedCooperFrameReader(self.first / 'inputs', run_id,
            Path('/tmp/RemoteTeleop-mcap-replay/artifacts/proxy-batch/xyz-results'))
        try:
            return reader.extract(offset, self.output / 'staging/cooperscene/inputs')
        finally:
            reader.close()

    def frozen_transform(self, run_id, method, frame):
        root, _ = self.manifests[method]
        selected = root / 'transforms' / f'{run_id}.json'
        if selected.exists():
            data = json.loads(selected.read_text())
            key = 'gaussian_T_nuscenes_global' if run_id.startswith('nuscenes') else 'gaussian_T_cooperscene'
            return np.asarray(data[key]), str(selected)
        return frame['inherited_map_T_dataset'].copy(), 'previous replay inherited transform; frame-zero fallback'

    def load_map(self, path):
        if path == self.map_path:
            return
        gaussian = load_gaussian_map(path)
        keep = np.isfinite(gaussian.means).all(axis=1) & np.isfinite(gaussian.opacities) & (gaussian.opacities > 0)
        keep &= np.abs(gaussian.means).max(axis=1) <= 10000
        self.full_target = gaussian.means[keep].astype(float)
        # Geometric surfel quality only; no reference elevation, initial road plane,
        # or previously generated road-surface model enters height estimation.
        scales = np.sort(gaussian.scales, axis=1)
        planar = keep & (scales[:, 0] < .08) & (scales[:, 0] / np.maximum(scales[:, 1], 1e-9) < .4)
        indices = np.flatnonzero(planar)
        rotations = Rotation.from_quat(gaussian.quats[indices][:, [1, 2, 3, 0]]).as_matrix()
        self.road_normals = rotations[np.arange(len(indices)), :, gaussian.scales[indices].argmin(axis=1)]
        self.road_points = gaussian.means[indices].astype(float)
        self.map_path = path

    def run_one(self, run_id, offset, *, sensitivity=False):
        stem = f'{run_id}-f{offset:06d}'
        destination = self.output / 'results' / f'{stem}.json'
        if destination.exists():
            return json.loads(destination.read_text())
        metadata = self.frame(run_id, offset)
        self.write(self.output / 'inputs' / f'{stem}.json', metadata)
        frame = dict(np.load(metadata['npz_path'], allow_pickle=False))
        self.load_map(metadata['map_path'])
        bev, bev_origin = self.frozen_transform(run_id, 'bev', frame)
        xyz, xyz_origin = self.frozen_transform(run_id, 'xyz', frame)
        dataset_T_lidar = frame['dataset_T_lidar']
        lidar_T_camera = np.linalg.inv(dataset_T_lidar) @ frame['dataset_T_camera']
        static = source_points(frame, mask_dynamic=True)
        raw_static = (static - dataset_T_lidar[:3, 3]) @ dataset_T_lidar[:3, :3]
        pose = bev @ dataset_T_lidar
        up = bev[:3, :3] @ np.array([0., 0., 1.])
        # Boundary: the estimator receives no recorded-reference target height.
        estimate = self.estimator.estimate(raw_static, map_T_lidar=pose,
            lidar_T_camera=lidar_T_camera, map_up=up, map_points=self.road_points,
            map_normals=self.road_normals)
        prediction = dict(run_id=run_id, frame_offset=offset, metadata=metadata,
            lidar_T_camera=lidar_T_camera, map_up=up, bev_map_T_dataset=bev,
            road_height=asdict(estimate), raw_frame_sha256=hashlib.sha256(Path(metadata['npz_path']).read_bytes()).hexdigest(),
            static_mask='Recorded global vehicle/pedestrian/animal boxes padded 0.2 m; includes parked vehicles; not online segmentation',
            lidar_original_count=len(frame['points_lidar']), static_lidar_count=len(raw_static),
            road_map_filter=dict(min_opacity=0., max_min_scale_m=.08, max_min_mid_scale_ratio=.4),
            reference_height_used_by_estimator=False, reference_conditioned_bev_baseline=True)
        if sensitivity:
            stresses = []
            for axis in [0, 1]:
                for degrees in [-1., 1.]:
                    stress = pose.copy()
                    world_axis = bev[:3, :3][:, axis]
                    stress[:3, :3] = Rotation.from_rotvec(np.deg2rad(degrees) * world_axis).as_matrix() @ pose[:3, :3]
                    result = self.estimator.estimate(raw_static, map_T_lidar=stress,
                        lidar_T_camera=lidar_T_camera, map_up=up, map_points=self.road_points, map_normals=self.road_normals)
                    stresses.append(dict(axis='roll' if axis == 0 else 'pitch', degrees=degrees,
                        accepted=result.accepted, reason=result.reason, height_shift_m=result.diagnostics.get('height_shift_m')))
            prediction['orientation_sensitivity'] = stresses
        self.write(self.output / 'predictions' / f'{stem}.json', prediction)
        # Evaluation begins only after the height proposal has been persisted.
        registrar = PointCloudRegistrar(registration_config(metadata['dataset']))
        initial = frame['pipeline_initial_map_T_dataset']
        reference = frame['reference_map_T_dataset']
        candidate = estimate.map_T_lidar @ np.linalg.inv(dataset_T_lidar)
        methods = {}
        for name, transform in [('xyz', xyz), ('bev', bev), ('hybrid_candidate', candidate)]:
            registration_source = source_points(frame, mask_dynamic=metadata['dataset'] == 'cooperscene')
            registration = registrar.evaluate(registration_source, self.full_target, transform, initial)
            methods[name] = dict(map_T_dataset=transform, map_T_lidar=transform @ dataset_T_lidar,
                map_T_camera=transform @ frame['dataset_T_camera'], error=pose_error(transform, reference, dataset_T_lidar),
                registration=asdict(registration), accepted=registration.accepted)
        upstream_qualified = self.coverage['bev'][run_id]['status'] == 'qualified'
        accepted = bool(estimate.accepted and methods['hybrid_candidate']['accepted'])
        methods['hybrid'] = dict(methods['hybrid_candidate'] if accepted else methods['bev'], accepted=accepted,
            status='accepted' if accepted else 'fallback_unchanged_bev',
            reason=estimate.reason if not estimate.accepted else 'original_registration_gates_failed' if not methods['hybrid_candidate']['accepted'] else 'all_gates_passed')
        correction = np.eye(4)
        correction[:3, 3] = estimate.map_T_lidar[:3, 3] - pose[:3, 3]
        applied_correction = correction if accepted else np.eye(4)
        result = dict(prediction, methods=methods, bev_transform_source=bev_origin, xyz_transform_source=xyz_origin,
            bev_upstream_qualified=upstream_qualified, bev_replay_status=self.coverage['bev'][run_id]['status'],
            sensor_height_proposal_map_T_map=correction, applied_sensor_correction_map_T_map=applied_correction,
            rendering_only_object_grounding_transform=None, rendering_grounding_applied=False,
            registration_source_mask='original dataset baseline: CooperScene annotation-masked; nuScenes unmasked; identical for all methods',
            registration_config=asdict(registrar.config),
            reference_used_after_prediction_only=True, historical_reference_qualification_used_for_acceptance=False,
            horizontal_reference_change_m=methods['hybrid_candidate']['error']['xy_m']-methods['bev']['error']['xy_m'],
            vertical_absolute_reference_change_m=abs(methods['hybrid_candidate']['error']['dz_m'])-abs(methods['bev']['error']['dz_m']))
        self.write(destination, result)
        print(json.dumps(dict(run=run_id, offset=offset, road=estimate.reason, accepted=accepted,
            bev_z=methods['bev']['error']['dz_m'], candidate_z=methods['hybrid_candidate']['error']['dz_m'],
            xyz_z=methods['xyz']['error']['dz_m'])), flush=True)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--previous', type=Path, default=Path('/tmp/RemoteTeleop-first-frame-localization/artifacts'))
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts/hybrid-road-height')
    parser.add_argument('--plan', type=Path, default=ROOT / 'scripts/investigation/hybrid_road_height_pilot.json')
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text())
    runner = HybridComparison(args.previous, args.output)
    rows = []
    for sequence in plan['sequences']:
        for index, offset in enumerate(sequence['frame_offsets']):
            rows.append(runner.run_one(sequence['run_id'], offset, sensitivity=index == 0))
    runner.write(args.output / 'summary.json', rows)


if __name__ == '__main__':
    main()

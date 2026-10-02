"""Fresh fixed-frame BEV localization followed by a separate LiDAR-height method.

The original BEV/XYZ implementations and deployed queues are unchanged. Height
predictions precede reference evaluation; Cooper BEV still uses its existing ENU
horizontal/yaw prior. Selected frames are frozen historical privileged choices.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.investigation.evaluate_hybrid_road_height import HybridComparison
from scripts.investigation.evaluate_clearance_transfer import measure
from scripts.investigation.first_frame_bev import GravityConstrainedRegistrar, TwoWorkerTree
from scripts.investigation.first_frame_xyz import jsonable, map_proxy, pose_error, registration_config, source_points
from src.localization.bev_lidar_height import BevLidarHeightConfig, BevLidarHeightEstimator
from src.localization.enu_prior_fusion import EnuPriorFuser, EnuPriorFusionConfig
import src.localization.enu_prior_fusion as fusion_module
from src.localization.ground_plane import GroundPlane
from src.localization.registration import PointCloudRegistrar
import numpy as np


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        while chunk := stream.read(8 * 1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


class BevLidarHeightComparison(HybridComparison):
    """Own fixed cases, fresh BEV fitting, independent height prediction and gates."""

    def __init__(self, previous, output):
        super().__init__(previous, output)
        fusion_module.cKDTree = TwoWorkerTree
        self.measurements = {}
        self.first_frames = {}

    def load_map(self, path):
        if path != self.map_path:
            self.full_target = map_proxy(path)
            self.map_path = path

    def plan(self):
        cases, map_hashes = [], {}
        for run_id in self.entries:
            indices = sorted({0, int(self.coverage['bev'][run_id]['start_sequence'])})
            for offset in indices:
                metadata = self.frame(run_id, offset)
                path = metadata['map_path']
                if path not in map_hashes:
                    map_hashes[path] = sha256(path)
                cases.append(dict(run_id=run_id, frame_offset=offset, metadata=metadata,
                    raw_frame_sha256=sha256(metadata['npz_path']), map_sha256=map_hashes[path],
                    frame_policy='first_frame' if offset == 0 else 'previous_privileged_BEV_selection_no_new_search'))
        plan = dict(method_id='bev_lidar_height', comparator_method_id='bev_lidar_height_nominal',
            cases=cases, frame_selection_frozen=True,
            acceptance='valid height proposal AND upstream scan accepted AND BEV fusion converged AND original fixed-transform registration gates',
            original_source_masks='Cooper annotation masked; nuScenes unmasked for registration; both masked for road fit',
            map_planes='Cooper documented native Z=-2.8 assumed road; nuScenes scene-specific first-scan road in first-camera map chart',
            height_reference_tuning=False, production_defaults_changed=False)
        self.write(self.output / 'plan.json', plan)
        # Source-only measurements: no map or reference alignment is passed in.
        for case in cases:
            key = (case['run_id'], case['frame_offset'])
            self.measurements[key] = measure(case['metadata'], case['frame_offset'], ['bev_lidar_height_fixed_plan'])
            if case['frame_offset'] == 0:
                self.first_frames[case['run_id']] = dict(np.load(case['metadata']['npz_path'], allow_pickle=False))
        self.write(self.output / 'measurements.json', list(self.measurements.values()))
        return cases

    def cached_scan(self, case, frame, source, config):
        run_id, offset = case['run_id'], case['frame_offset']
        path = (self.first / 'xyz/baseline' / f'{run_id}.json' if offset == 0 else
            self.previous / 'frame-advance-localization/cooperscene/results/baseline' / f'{run_id}-f{offset:06d}.json')
        if not path.exists():
            return None, dict(reused=False, reason='same_frame_scan_cache_missing')
        data = json.loads(path.read_text())
        expected_id = run_id if offset == 0 else f'{run_id}-f{offset:06d}'
        checks = dict(run_id=data.get('run_id') == expected_id,
            frame_id=data.get('frame_id') == case['metadata']['frame_id'],
            dataset=data.get('dataset') == 'cooperscene',
            variant=data.get('variant') == 'baseline', source_mask=data.get('dynamic_mask_applied') is True,
            source_count=data.get('source_points') == len(source), target_count=data.get('target_points') == len(self.full_target),
            registration_config=data.get('registration_config') == jsonable(asdict(config)),
            fit_config=data.get('fit_config') == jsonable(asdict(config)),
            initial=np.allclose(data.get('initial_map_T_dataset', np.zeros((4, 4))), frame['pipeline_initial_map_T_dataset'], atol=1e-12, rtol=0),
            scan_available='scan_registration' in data)
        audit = dict(reused=all(checks.values()), path=str(path), result_sha256=sha256(path), checks=checks,
            current_source_npz=case['metadata']['npz_path'], current_source_sha256=case['raw_frame_sha256'],
            current_map_sha256=case['map_sha256'],
            historical_source_hash_available=False,
            provenance='Baseline result associated with same immutable staged frame; identity/config/count/initializer checked; scan gates re-evaluated on current exact inputs')
        return (data['scan_registration'] if audit['reused'] else None), audit

    def fresh_bev(self, case, frame, source, registrar):
        initial = frame['pipeline_initial_map_T_dataset']
        result = dict(fresh_bev_fitting=True, source_count=len(source),
            registration_config=asdict(registrar.config),
            dynamic_mask_applied=case['metadata']['dataset'] == 'cooperscene')
        if case['metadata']['dataset'] == 'cooperscene':
            scan, audit = self.cached_scan(case, frame, source, registrar.config)
            if scan is None:
                scan = asdict(registrar.register(source, self.full_target, initial))
            checked_scan = registrar.evaluate(source, self.full_target, np.asarray(scan['map_T_source']), initial)
            fusion_config = EnuPriorFusionConfig(position_mode='bev')
            fusion = EnuPriorFuser(fusion_config).fuse(source, self.full_target,
                np.asarray(scan['map_T_source']), frame['reference_map_T_dataset'], frame['dataset_T_lidar'])
            registration = registrar.evaluate(source, self.full_target, fusion.map_T_source, initial)
            result.update(scan_cache=audit, scan_registration=scan, scan_rechecked=asdict(checked_scan),
                upstream_scan_accepted=bool(scan['accepted'] and checked_scan.accepted),
                fusion_config=asdict(fusion_config), fusion=asdict(fusion), fusion_converged=fusion.converged,
                reference_prior='Existing Cooper ENU XY/yaw prior retained; no height reference supplied to height estimator')
        else:
            constrained = GravityConstrainedRegistrar(registrar.config, frame['dataset_T_lidar'])
            registration = constrained.register(source, self.full_target, initial)
            result.update(level_diagnostics=constrained.level_diagnostics, upstream_scan_accepted=True,
                fusion_converged=True, scan_cache=dict(reused=False, reason='fresh_gravity_constrained_ICP'),
                upstream_scan_policy='not_applicable; successful BEV optimizer completion followed by candidate original residual/correction gates',
                constraints='Original initializer vertical/roll/pitch retained; optimize XY/yaw')
        result['registration'] = asdict(registration)
        result['accepted'] = bool(registration.accepted and result['upstream_scan_accepted'] and result['fusion_converged'])
        return registration.map_T_source, result

    def plane(self, case):
        if case['metadata']['dataset'] == 'cooperscene':
            return GroundPlane((0., 0., 1.), 2.8), dict(policy='Documented Riverside native Z=-2.8 rendering level assumed flat physical road; unverified terrain', conditional=True)
        first = self.first_frames[case['run_id']]
        calibration = self.measurements[(case['run_id'], 0)]
        if not calibration['accepted']:
            raise ValueError('Cannot calibrate scene initial plane: source road fit rejected')
        camera_T_lidar = np.linalg.inv(first['dataset_T_camera']) @ first['dataset_T_lidar']
        sensor_plane = calibration['lidar_plane_sensor']
        normal = camera_T_lidar[:3, :3] @ np.asarray(sensor_plane['normal'])
        offset = sensor_plane['offset_m'] - normal @ camera_T_lidar[:3, 3]
        return GroundPlane(tuple(normal), offset), dict(policy='Scene first LiDAR road plane transported into documented first-front-camera map chart',
            source_run_id=case['run_id'], source_frame_offset=0, map_T_initial_lidar=camera_T_lidar,
            conditional=True, same_scan_calibration=case['frame_offset'] == 0,
            later_frame_flat_plane_extrapolation=case['frame_offset'] != 0,
            reference_height_used=False, exporter_chart_independently_revalidated=False)

    def run_case(self, case):
        started = time.monotonic()
        run_id, offset, metadata = case['run_id'], case['frame_offset'], case['metadata']
        stem = f'{run_id}-f{offset:06d}'
        frame = dict(np.load(metadata['npz_path'], allow_pickle=False))
        self.load_map(metadata['map_path'])
        source = source_points(frame, mask_dynamic=metadata['dataset'] == 'cooperscene')
        registrar = PointCloudRegistrar(registration_config(metadata['dataset']))
        bev, fresh = self.fresh_bev(case, frame, source, registrar)
        plane, plane_policy = self.plane(case)
        measurement = self.measurements[(run_id, offset)]
        nominal_id = f"cooperscene-1-{metadata['agent']}" if metadata['dataset'] == 'cooperscene' else 'nuscenes-152'
        nominal = self.measurements[(nominal_id, 0)]
        lidar_T_camera = np.linalg.inv(frame['dataset_T_lidar']) @ frame['dataset_T_camera']
        pose, up = bev @ frame['dataset_T_lidar'], bev[:3, :3] @ np.array([0., 0., 1.])
        proposals = {}
        for name, mode in [('bev_lidar_height', 'measured'), ('bev_lidar_height_nominal', 'nominal')]:
            config = BevLidarHeightConfig(clearance_mode=mode, ground_plane_policy=plane_policy['policy'])
            estimate = BevLidarHeightEstimator(config).estimate(map_T_lidar=pose, map_up=up,
                ground_plane=plane, lidar_normal_clearance_m=measurement.get('lidar_normal_clearance_m'),
                source_fit_accepted=measurement['accepted'], lidar_T_camera=lidar_T_camera,
                nominal_normal_clearance_m=nominal.get('lidar_normal_clearance_m') if nominal['accepted'] else None,
                nominal_fit_accepted=nominal['accepted'])
            proposals[name] = dict(config=asdict(config), result=asdict(estimate))
        prediction = dict(case, method_id='bev_lidar_height', fresh_bev=fresh, bev_map_T_dataset=bev,
            map_up=up, lidar_T_camera=lidar_T_camera, ground_plane=plane.to_dict(), ground_plane_policy=plane_policy,
            road_measurement=measurement, nominal_source_run_id=nominal_id, nominal_measurement=nominal,
            proposals=proposals, height_reference_used=False,
            bev_reference_conditioned=True,
            bev_reference_conditioning_kind=('existing_ENU_horizontal_yaw_prior' if metadata['dataset'] == 'cooperscene'
                else 'documented_first_camera_map_chart_initializer_with_recorded_vertical_and_gravity'),
            render_only_object_grounding_applied=False)
        self.write(self.output / 'predictions' / f'{stem}.json', prediction)
        # Reference-error reporting begins only after predictions are persisted.
        methods = {}
        def evaluate(name, transform, upstream=True):
            registration = registrar.evaluate(source, self.full_target, transform, frame['pipeline_initial_map_T_dataset'])
            methods[name] = dict(map_T_dataset=transform, map_T_lidar=transform @ frame['dataset_T_lidar'],
                map_T_camera=transform @ frame['dataset_T_camera'], registration=asdict(registration),
                error=pose_error(transform, frame['reference_map_T_dataset'], frame['dataset_T_lidar']),
                accepted=bool(upstream and registration.accepted))
        upstream = fresh['upstream_scan_accepted'] and fresh['fusion_converged']
        evaluate('bev', bev, upstream)
        for method, proposal in proposals.items():
            estimate = proposal['result']
            candidate_pose = estimate['proposed_map_T_lidar'] if estimate['proposal_valid'] else pose
            candidate = np.asarray(candidate_pose) @ np.linalg.inv(frame['dataset_T_lidar'])
            evaluate(method + '_candidate', candidate, upstream and estimate['proposal_valid'])
            accepted = methods[method + '_candidate']['accepted']
            methods[method] = dict(methods[method + '_candidate'] if accepted else methods['bev'],
                accepted=accepted, status='accepted' if accepted else 'fallback_fresh_bev',
                reason='all_gates_passed' if accepted else estimate['reason'] if not estimate['proposal_valid'] else
                'upstream_scan_or_fusion_rejected' if not upstream else 'original_registration_gates_failed')
        prior, prior_origin = self.frozen_transform(run_id, 'bev', frame)
        evaluate('previous_replay_bev', prior)
        corrections = {}
        for method in proposals:
            proposed_correction = np.eye(4)
            proposed_correction[:3, 3] = methods[method + '_candidate']['map_T_lidar'][:3, 3] - pose[:3, 3]
            corrections[method] = dict(proposed_sensor_map_T_map=proposed_correction,
                applied_sensor_map_T_map=proposed_correction if methods[method]['accepted'] else np.eye(4))
        result = dict(prediction, methods=methods, sensor_corrections=corrections,
            previous_replay_bev_source=prior_origin,
            runtime_seconds=time.monotonic() - started,
            original_bev_baseline_changed=False, historical_reference_qualification_used_for_acceptance=False)
        self.write(self.output / 'results' / f'{stem}.json', result)
        print(json.dumps(dict(run_id=run_id, frame_offset=offset, bev_accepted=methods['bev']['accepted'],
            height_accepted=methods['bev_lidar_height']['accepted'],
            bev_xyz=methods['bev']['error']['xyz_m'], candidate_xyz=methods['bev_lidar_height_candidate']['error']['xyz_m'],
            reused_scan=fresh['scan_cache']['reused'])), flush=True)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--previous', type=Path, default=Path('/tmp/RemoteTeleop-first-frame-localization/artifacts'))
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts/bev-lidar-height')
    args = parser.parse_args()
    runner = BevLidarHeightComparison(args.previous, args.output)
    cases = runner.plan()
    results = []
    for case in cases:
        try:
            results.append(runner.run_case(case))
        except Exception as error:
            failure = dict(case, method_id='bev_lidar_height', exception=f'{type(error).__name__}: {error}', accepted=False)
            runner.write(args.output / 'failures' / f"{case['run_id']}-f{case['frame_offset']:06d}.json", failure)
            print(json.dumps(failure), flush=True)
            results.append(failure)
        runner.write(args.output / 'summary.json', results)


if __name__ == '__main__':
    main()

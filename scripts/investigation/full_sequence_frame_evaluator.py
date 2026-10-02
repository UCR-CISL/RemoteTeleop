"""Exact-input cached, independent-method evaluation of one native replay frame."""
from __future__ import annotations

from dataclasses import asdict
import hashlib
from importlib.metadata import PackageNotFoundError, version
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial import cKDTree

from scripts.investigation.advance_cooperscene_frames import strict_rejection_certificate
from scripts.investigation.evaluate_clearance_transfer import measure
from scripts.investigation.first_frame_bev import GravityConstrainedRegistrar
from scripts.investigation.first_frame_xyz import jsonable, pose_error, registration_config, source_points
from scripts.investigation.run_bev_lidar_height import BevLidarHeightComparison, ROOT, sha256
from src.localization.bev_lidar_height import BevLidarHeightConfig, BevLidarHeightEstimator
from src.localization.enu_prior_fusion import EnuPriorFuser, EnuPriorFusionConfig
from src.localization.registration import PointCloudRegistrar


class FullSequenceFrameEvaluator(BevLidarHeightComparison):
    """Share the exact scan fit while retaining each method's original gates."""

    METHODS = {'bev', 'xyz', 'bev_lidar_height'}

    def __init__(self, previous: Path, output: Path):
        super().__init__(previous, output)
        self._hashes = {}
        self._tree = None
        files = ['scripts/investigation/full_sequence_frame_evaluator.py',
                 'scripts/investigation/run_bev_lidar_height.py',
                 'scripts/investigation/first_frame_bev.py', 'scripts/investigation/first_frame_xyz.py',
                 'scripts/investigation/evaluate_clearance_transfer.py',
                 'scripts/investigation/advance_cooperscene_frames.py',
                 'src/localization/registration.py', 'src/localization/enu_prior_fusion.py',
                 'src/localization/bev_lidar_height.py', 'src/localization/hybrid_road_height.py',
                 'src/localization/ground_plane.py', 'src/localization/gaussian_map.py']
        self.code_hashes = {name: sha256(ROOT / name) for name in files}
        self.library_versions = {}
        for package in ("numpy", "scipy", "open3d"):
            try:
                self.library_versions[package] = version(package)
            except PackageNotFoundError:
                self.library_versions[package] = "unavailable"

    def _hash(self, path):
        path = Path(path)
        stat = path.stat()
        key = (str(path.resolve()), stat.st_size, stat.st_mtime_ns)
        if key not in self._hashes:
            self._hashes[key] = sha256(path)
        return self._hashes[key]

    def load_map(self, path):
        changed = path != self.map_path
        super().load_map(path)
        if changed:
            self._tree = cKDTree(self.full_target)

    @staticmethod
    def _failure(reason, error=None):
        return dict(accepted=False, error={}, status='optimizer_failed' if error else reason,
                    reason=reason, exception=None if error is None else f'{type(error).__name__}: {error}')

    def _identity(self, case):
        metadata = case['metadata']
        seed = self.entries[case['run_id']]['npz_path']
        settings = dict(code=self.code_hashes, library_versions=self.library_versions, input_sha256=self._hash(metadata['npz_path']),
                        map_sha256=self._hash(metadata['map_path']), initial_frame_sha256=self._hash(seed),
                        run_id=case['run_id'], frame_offset=case['frame_offset'], frame_id=metadata['frame_id'],
                        dataset=metadata['dataset'], registration_config=asdict(registration_config(metadata['dataset'])),
                        xyz_fusion=asdict(EnuPriorFusionConfig(position_mode='xyz')),
                        bev_fusion=asdict(EnuPriorFusionConfig(position_mode='bev')),
                        source_mask='Cooper annotation masked; nuScenes unmasked; height always annotation masked',
                        height_mode='measured; configured road plane; unchanged upstream/candidate gates')
        encoded = json.dumps(jsonable(settings), sort_keys=True, allow_nan=False).encode()
        return hashlib.sha256(encoded).hexdigest(), settings

    def _fixed_case(self, case, identity, source_count):
        """Reuse recorded optimization output only with exact file/config checks.

        Original gates are still re-evaluated on current exact points. Historical
        first-frame/advance fits lacking stored input hashes are not used here.
        """
        path = ROOT / 'artifacts/bev-lidar-height/results' / f"{case['run_id']}-f{case['frame_offset']:06d}.json"
        if not path.exists():
            return None
        data = json.loads(path.read_text())
        fresh = data.get('fresh_bev', {})
        checks = [data.get('raw_frame_sha256') == identity['input_sha256'],
                  data.get('map_sha256') == identity['map_sha256'],
                  data.get('run_id') == case['run_id'], data.get('frame_offset') == case['frame_offset'],
                  fresh.get('registration_config') == jsonable(identity['registration_config']),
                  fresh.get('source_count') == source_count,
                  fresh.get('dynamic_mask_applied') == (case['metadata']['dataset'] == 'cooperscene')]
        if case['metadata']['dataset'] == 'cooperscene':
            checks.append(fresh.get('fusion_config') == jsonable(identity['bev_fusion']))
        if not all(checks):
            return None
        data['_reuse_provenance'] = dict(path=str(path), sha256=self._hash(path),
            exact_input_and_map_hashes=True, config_and_source_count_match=True,
            original_gates_rechecked=True)
        return data

    def _calibration(self, case):
        run_id = case['run_id']
        if run_id not in self.first_frames:
            entry = self.entries[run_id]
            self.first_frames[run_id] = dict(np.load(entry['npz_path'], allow_pickle=False))
            self.measurements[(run_id, 0)] = measure(entry, 0, ['full_sequence_initial_plane'])
        key = (run_id, case['frame_offset'])
        if key not in self.measurements:
            self.measurements[key] = measure(case['metadata'], case['frame_offset'], ['full_sequence_native_frame'])
        return self.measurements[key]

    def evaluate_case(self, case: dict, required_methods: list[str]) -> dict:
        required = set(required_methods)
        if not required or not required <= self.METHODS:
            raise ValueError(f'unknown/empty required methods: {required_methods}')
        started = time.monotonic()
        fingerprint, identity = self._identity(case)
        stem = f"{case['run_id']}-f{case['frame_offset']:06d}"
        destination = self.output / 'results' / f'{stem}.json'
        record = json.loads(destination.read_text()) if destination.exists() else {}
        if record and record.get('fingerprint') != fingerprint:
            raise ValueError(f'{stem}: cached evaluation fingerprint differs; use a new output directory')
        methods = record.setdefault('methods', {})
        if required <= methods.keys():
            return record
        needed = required - methods.keys()
        metadata = case['metadata']
        # Data reading and validation deliberately happen outside optimizer catches.
        frame = dict(np.load(metadata['npz_path'], allow_pickle=False))
        for key in ('pipeline_initial_map_T_dataset', 'reference_map_T_dataset', 'dataset_T_lidar', 'dataset_T_camera'):
            matrix = frame[key]
            if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
                    or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9)
                    or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6)
                    or np.linalg.det(matrix[:3, :3]) < .999):
                raise ValueError(f'{stem}: invalid {key}')
        self.load_map(metadata['map_path'])
        source = source_points(frame, mask_dynamic=metadata['dataset'] == 'cooperscene')
        if len(source) < 3 or not np.isfinite(source).all() or len(self.full_target) < 3:
            raise ValueError(f'{stem}: invalid registration point data')
        initial = frame['pipeline_initial_map_T_dataset']
        registrar = PointCloudRegistrar(registration_config(metadata['dataset']))
        record.update(run_id=case['run_id'], frame_offset=case['frame_offset'], metadata=metadata,
                      fingerprint=fingerprint, identity=identity, methods=methods,
                      independent_frame_initialization=True, no_warm_start=True,
                      original_gates_unchanged=True)

        def evaluated(transform, upstream=True):
            registration = registrar.evaluate(source, self.full_target, transform, initial)
            return dict(map_T_dataset=transform, registration=asdict(registration),
                        error=pose_error(transform, frame['reference_map_T_dataset'], frame['dataset_T_lidar']),
                        accepted=bool(upstream and registration.accepted))

        if metadata['dataset'] == 'cooperscene':
            points = source @ initial[:3, :3].T + initial[:3, 3]
            distances = self._tree.query(points, workers=2)[0]
            certificate = strict_rejection_certificate(points, distances, registrar.config)
            record['rejection_certificate'] = certificate
            if certificate['certified_impossible']:
                for method in needed:
                    methods[method] = self._failure('certified_no_accepted_transform')
                record['runtime_seconds'] = time.monotonic() - started
                self.write(destination, record)
                return jsonable(record)
        fixed = self._fixed_case(case, identity, len(source))
        if fixed:
            record['fixed_case_reuse'] = fixed['_reuse_provenance']
        scan = record.get('shared_scan')
        if metadata['dataset'] == 'cooperscene' and scan is None:
            try:
                cached_scan = fixed['fresh_bev'].get('scan_registration') if fixed else None
                scan = cached_scan or asdict(registrar.register(source, self.full_target, initial))
                checked = registrar.evaluate(source, self.full_target, np.asarray(scan['map_T_source']), initial)
                scan = dict(scan, accepted=bool(scan['accepted'] and checked.accepted), rechecked=asdict(checked))
                record['shared_scan'] = scan
            except (RuntimeError, ValueError, np.linalg.LinAlgError) as error:
                for method in needed:
                    methods[method] = self._failure('shared_scan_optimizer_failed', error)
                self.write(destination, record)
                return jsonable(record)

        bev = None
        upstream = False
        if needed & {'bev', 'bev_lidar_height'}:
            try:
                if 'bev' in methods and 'map_T_dataset' in methods['bev']:
                    bev = np.asarray(methods['bev']['map_T_dataset'])
                    upstream = record['bev_upstream_accepted']
                elif fixed:
                    bev = np.asarray(fixed['bev_map_T_dataset'])
                    upstream = bool(fixed['fresh_bev']['upstream_scan_accepted'] and fixed['fresh_bev']['fusion_converged'])
                    if scan is not None:
                        upstream = upstream and scan['accepted']
                    record['bev_fit'] = fixed['fresh_bev']
                elif metadata['dataset'] == 'cooperscene':
                    fusion = EnuPriorFuser(EnuPriorFusionConfig(position_mode='bev')).fuse(
                        source, self.full_target, np.asarray(scan['map_T_source']), frame['reference_map_T_dataset'], frame['dataset_T_lidar'])
                    bev, upstream = fusion.map_T_source, bool(scan['accepted'] and fusion.converged)
                    record['bev_fit'] = dict(fusion=asdict(fusion), upstream_scan_accepted=scan['accepted'])
                else:
                    constrained = GravityConstrainedRegistrar(registrar.config, frame['dataset_T_lidar'])
                    fit = constrained.register(source, self.full_target, initial)
                    bev, upstream = fit.map_T_source, True  # optimizer completion; candidate still faces original gates
                    record['bev_fit'] = dict(registration=asdict(fit), level_diagnostics=constrained.level_diagnostics)
                record['bev_upstream_accepted'] = upstream
                if 'bev' in needed:
                    methods['bev'] = evaluated(bev, upstream)
            except (RuntimeError, ValueError, np.linalg.LinAlgError) as error:
                for method in needed & {'bev', 'bev_lidar_height'}:
                    methods[method] = self._failure('bev_optimizer_failed', error)
                bev = None

        if 'xyz' in needed:
            try:
                xyz_scan = scan if scan is not None else asdict(registrar.register(source, self.full_target, initial))
                transform = np.asarray(xyz_scan['map_T_source'])
                xyz_upstream = bool(xyz_scan['accepted'])
                if metadata['dataset'] == 'cooperscene' and xyz_upstream:
                    fusion = EnuPriorFuser(EnuPriorFusionConfig(position_mode='xyz')).fuse(
                        source, self.full_target, transform, frame['reference_map_T_dataset'], frame['dataset_T_lidar'])
                    transform, xyz_upstream = fusion.map_T_source, bool(fusion.converged)
                    record['xyz_fusion'] = asdict(fusion)
                methods['xyz'] = evaluated(transform, xyz_upstream)
                record['xyz_scan'] = xyz_scan
            except (RuntimeError, ValueError, np.linalg.LinAlgError) as error:
                methods['xyz'] = self._failure('xyz_optimizer_failed', error)

        if 'bev_lidar_height' in needed and bev is not None:
            # Calibration data failures must propagate, not count as examined failures.
            measurement = self._calibration(case)
            if metadata['dataset'] == 'nuscenes' and not self.measurements[(case['run_id'], 0)]['accepted']:
                methods['bev_lidar_height'] = self._failure('initial_road_plane_calibration_rejected')
            else:
                plane, policy = self.plane(case)
                estimate = BevLidarHeightEstimator(BevLidarHeightConfig(ground_plane_policy=policy['policy'])).estimate(
                    map_T_lidar=bev @ frame['dataset_T_lidar'], map_up=bev[:3, :3][:, 2], ground_plane=plane,
                    lidar_normal_clearance_m=measurement.get('lidar_normal_clearance_m'), source_fit_accepted=measurement['accepted'],
                    lidar_T_camera=np.linalg.inv(frame['dataset_T_lidar']) @ frame['dataset_T_camera'])
                prediction = dict(ground_plane=plane.to_dict(), policy=policy, measurement=measurement, proposal=asdict(estimate))
                self.write(self.output / 'predictions' / f'{stem}.json', prediction)
                record['height_prediction'] = prediction
                if estimate.proposal_valid:
                    candidate = estimate.proposed_map_T_lidar @ np.linalg.inv(frame['dataset_T_lidar'])
                    proposal = evaluated(candidate, upstream)
                    proposal['proposal_diagnostics'] = estimate.diagnostics
                    proposal['reason'] = 'all_gates_passed' if proposal['accepted'] else 'upstream_or_original_registration_gates_rejected'
                    methods['bev_lidar_height'] = proposal
                else:
                    methods['bev_lidar_height'] = self._failure(estimate.reason)
        record['runtime_seconds'] = time.monotonic() - started
        self.write(destination, record)
        return jsonable(record)

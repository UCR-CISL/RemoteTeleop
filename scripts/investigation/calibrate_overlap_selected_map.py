"""Fit one rigid map tie from three overlap-selected vehicles and one fixed station.

Released ENU poses are frozen. Full statically masked clouds, rather than only
station-overlapping subsets, contribute unique spatial voxels. Offline geometry
consistency only: no independent ground truth or runtime gate modification.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.investigation.calibrate_shared_enu_map import transform, voxel, metrics
from scripts.investigation.compare_reference_road_planes import height_at, inside, transform_plane
from scripts.investigation.validate_shared_enu_map import SharedMapValidation
from src.localization.gaussian_map import load_gaussian_map
from src.localization.hybrid_road_height import HybridRoadHeightEstimator

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'artifacts/reference-refinement'


class MetricsOnlyValidation(SharedMapValidation):
    """Reuse identical validation calculations without duplicate control figures."""

    def plot(self, case, clouds, rows, xy):
        pass

    def report(self, rows, summary):
        pass


class OverlapSelectedMapCalibration:
    def __init__(self, selection: Path, output: Path):
        self.selection = selection
        self.output = output
        output.mkdir(parents=True, exist_ok=True)
        self.manifest = json.loads((selection / 'manifest.json').read_text())
        with np.load(selection / 'selected-clouds.npz') as data:
            self.clouds = {key: data[key].copy() for key in data.files}
        self.shared = json.loads((BASE / 'shared-enu-calibration/calibration.json').read_text())
        with Path(self.shared['map_path']).open('rb') as stream:
            if hashlib.file_digest(stream, 'sha256').hexdigest() != self.shared['map_sha256']:
                raise ValueError('Map content differs from prior calibration provenance')
        self.W = np.asarray(self.shared['original_W'])
        if not np.allclose(self.W, self.clouds['original_W']):
            raise ValueError('Selection and previous calibration use different original W')
        self.gaussian = load_gaussian_map(self.shared['map_path'])
        self.targets = {name: self.target(opacity) for name, opacity in [('opacity005', .05), ('positive', 0)]}
        self.sources = {'infra_only': voxel(self.clouds['infra_representative'], .25)}
        self.sources['fused'] = voxel(np.concatenate([self.sources['infra_only']] + [self.clouds[f'global_{a}'] for a in [1, 2, 3]]), .25)
        self.transforms = {'released': np.eye(4), 'original_shared': np.asarray(self.shared['D'])}
        self.results = {}

    def write(self, name, value):
        (self.output / name).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')

    def target(self, opacity):
        g = self.gaussian
        keep = np.isfinite(g.means).all(1) & np.isfinite(g.opacities) & (g.opacities > opacity) & (abs(g.means).max(1) <= 10000)
        points = voxel(g.means[keep].astype(float), .15)
        tree = cKDTree(points)
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.65, max_nn=30))
        normals = np.asarray(pc.normals).copy()
        valid = (tree.query_ball_point(points, .4, return_length=True, workers=2) >= 6) & np.isfinite(normals).all(1)
        return points, tree, normals, valid

    def fit(self, source, target_name):
        target, tree, normals, valid = self.targets[target_name]
        D = np.eye(4)
        trace = []
        for gate, iterations in [(3., 20), (1.5, 20), (.75, 20), (.4, 20)]:
            for iteration in range(iterations):
                points = transform(source, D)
                distances, ids = tree.query(points, workers=2)
                keep = (distances < gate) & valid[ids]
                x, q, n = points[keep], target[ids[keep]], normals[ids[keep]]
                if len(x) < 100:
                    raise RuntimeError('Insufficient supported geometric correspondences')
                residual = np.sum(n * (x - q), axis=1)
                A = np.c_[np.cross(x, n), n]
                roadlike = np.abs(n[:, 2]) > .85
                # One representative per occupied source voxel. Equal aggregate
                # normal-category weight avoids horizontal surface domination.
                weights = np.zeros(len(x))
                for mask in [roadlike, ~roadlike]:
                    weights[mask] = 1 / max(int(mask.sum()), 1)
                weights *= np.minimum(1., .15 / np.maximum(np.abs(residual), 1e-12))
                H = A.T @ (weights[:, None] * A)
                delta = np.linalg.lstsq(H, -(A.T @ (weights * residual)), rcond=1e-10)[0]
                delta /= max(np.linalg.norm(delta[:3]) / np.deg2rad(2), np.linalg.norm(delta[3:]) / .5, 1.)
                step = np.eye(4)
                step[:3, :3] = Rotation.from_rotvec(delta[:3]).as_matrix()
                step[:3, 3] = delta[3:]
                D = step @ D
                trace.append(dict(gate_m=gate, iteration=iteration, correspondences=len(x), roadlike_count=int(roadlike.sum()), nonroad_count=int((~roadlike).sum()), point_plane_median_m=float(np.median(abs(residual))), delta_rotation_deg=float(np.rad2deg(np.linalg.norm(delta[:3]))), delta_translation_m=float(np.linalg.norm(delta[3:])), normal_matrix_eigenvalues=np.linalg.eigvalsh(H).tolist()))
                trace[-1]['normal_matrix_rank_rcond_1e10'] = int(np.linalg.matrix_rank(H, tol=np.linalg.norm(H, 2) * 1e-10))
                trace[-1]['step_converged'] = bool(np.linalg.norm(delta) < 1e-6)
                if np.linalg.norm(delta) < 1e-6:
                    break
        return D, trace

    def calibration_record(self, name, D, trace, frames, source, target_name):
        target = self.targets[target_name]
        return dict(status='offline geometry diagnostic; not independent ground truth', variant=name, D=D.tolist(), original_W=self.W.tolist(), map_T_released_enu=(D @ self.W).tolist(), composition='map_T_lidar = D @ original_W @ released_enu_T_lidar', D_rpy_deg=Rotation.from_matrix(D[:3, :3]).as_euler('xyz', degrees=True).tolist(), D_translation_m=D[:3, 3].tolist(), calibration_frames=frames, map_path=self.shared['map_path'], map_sha256=self.shared['map_sha256'], source_points=len(source), target_points=len(target[0]), valid_target_normals=int(target[3].sum()), minimum_opacity_exclusive=.05 if target_name == 'opacity005' else 0, source_filter='original annotation vehicle boxes +0.2m removed; complete static clouds; frozen released ENU; fused unique 0.25m spatial voxels', algorithm=dict(gates_m=[3, 1.5, .75, .4], huber_m=.15, maximum_iterations_per_gate=20, category_balance='equal aggregate target normal |nz| >0.85 versus other; voxel-balanced geometry, no per-agent weighting', maximum_source_points=None, normal_radius_m=.65, normal_max_neighbors=30, valid_normal_min_neighbors_within_040m=6), runtime_acceptance_gates_unchanged=True, released_ICP_unchanged=True, final_solver_support=trace[-1], solver_diagnostic_status='full_rank_converged' if trace[-1]['normal_matrix_rank_rcond_1e10'] == 6 and trace[-1]['step_converged'] else 'inspect_final_rank_and_iteration_limit', trace=trace)

    def fits(self):
        infra = [self.manifest['infrastructure_representative']]
        frames = infra + self.manifest['global_selected']
        for name, source_name, target_name in [('fused_opacity005', 'fused', 'opacity005'), ('infra_only_opacity005', 'infra_only', 'opacity005'), ('fused_positive', 'fused', 'positive'), ('infra_only_positive', 'infra_only', 'positive')]:
            source = self.sources[source_name]
            D, trace = self.fit(source, target_name)
            result = self.calibration_record(name, D, trace, frames if source_name == 'fused' else infra, source, target_name)
            self.transforms[name] = D
            self.results[name] = result
            self.write(name + '-calibration.json', result)
            print('FIT', name, result['D_rpy_deg'], result['D_translation_m'], flush=True)
        for name, path in [('previous_infra_positive', 'infra_positive-calibration.json'), ('previous_infra_opacity005', 'opacity005-calibration.json')]:
            record = json.loads((BASE / 'infra-map-residual' / path).read_text())
            self.transforms[name] = np.asarray(record['D'])
        rows = []
        cross = np.array([[0, 0, 0], [30, 0, 0], [0, 30, 0], [-30, 0, 0], [0, -30, 0]])
        primary = self.transforms['fused_opacity005']
        for take in [1, 2, 3, 4, 5, 'all15']:
            takes = range(1, 6) if take == 'all15' else [take]
            source = voxel(np.concatenate([self.sources['infra_only']] + [self.clouds[f'cal_{t}_{a}'] for t in takes for a in [1, 2, 3]]), .25)
            D, trace = self.fit(source, 'opacity005')
            delta = D @ np.linalg.inv(primary)
            selected = [r for r in self.manifest['per_take_selected'] if r['take'] in takes]
            record = self.calibration_record(f'selected_{take}', D, trace, infra + selected, source, 'opacity005')
            record.update(relative_rotation_deg=float(np.rad2deg(Rotation.from_matrix(delta[:3, :3]).magnitude())), max_shift_30m_cross_m=float(np.linalg.norm(transform(cross, D) - transform(cross, primary), axis=1).max()))
            rows.append(record)
            print('STABILITY', take, record['relative_rotation_deg'], record['max_shift_30m_cross_m'], flush=True)
        self.write('stability.json', rows)

    @staticmethod
    def occupied(points, size):
        return set(map(tuple, np.floor(points / size).astype(np.int64)))

    def coverage(self):
        rows = []
        for name in ['released', 'fused_opacity005']:
            D = self.transforms[name]
            for target_name, (target, tree, normals, valid) in self.targets.items():
                for size in [.25, .5, 1.]:
                    target_cells = self.occupied(target, size)
                    infra = transform(self.sources['infra_only'], D)
                    fused = transform(self.sources['fused'], D)
                    i_cells, f_cells = self.occupied(infra, size), self.occupied(fused, size)
                    i_map, f_map = i_cells & target_cells, f_cells & target_cells
                    rows.append(dict(variant=name, target=target_name, voxel_m=size, target_occupied=len(target_cells), infrastructure_occupied=len(i_cells), fused_occupied=len(f_cells), added_source_occupied=len(f_cells-i_cells), infrastructure_map_occupied=len(i_map), fused_map_occupied=len(f_map), added_map_occupied=len(f_map-i_map), infrastructure_map_coverage_fraction=len(i_map)/len(target_cells), fused_map_coverage_fraction=len(f_map)/len(target_cells)))
                for source_name in ['infra_only', 'fused']:
                    q = transform(self.sources[source_name], D)
                    distances, ids = tree.query(q, workers=2)
                    supported = (distances < .75) & valid[ids]
                    categories = {}
                    for category, mask in [('roadlike', abs(normals[ids, 2]) > .85), ('nonroad', abs(normals[ids, 2]) <= .85), ('vertical', abs(normals[ids, 2]) < .3)]:
                        mask &= supported
                        categories[category] = dict(source_count=int(mask.sum()), unique_target_voxels=len(np.unique(ids[mask])), nn_median_m=float(np.median(distances[mask])) if mask.any() else None)
                    rows.append(dict(variant=name, target=target_name, source=source_name, full_source_count=len(q), source_xyz_span_m=np.ptp(q, axis=0).tolist(), supported_count=int(supported.sum()), normal_categories=categories))
        self.write('coverage.json', rows)

    def heldout(self):
        manifest = json.loads((BASE / 'shared-enu-calibration/input-manifest.json').read_text())
        selected = {(r['take'], r['agent'], r['frame_id']) for r in self.results['fused_opacity005']['calibration_frames']}
        rows = []
        with np.load(BASE / 'shared-enu-calibration/input-clouds.npz') as data:
            for record in manifest:
                if record['partition'] != 'heldout':
                    continue
                if (record['take'], record['agent'], record['frame_id']) in selected:
                    raise ValueError('Calibration/heldout leakage')
                source = data[record['cache_key']]
                for target_name, (target, tree, normals, valid) in self.targets.items():
                    original_dist, original_ids = tree.query(source, workers=2)
                    support = original_dist < 3.
                    roadlike = abs(normals[original_ids, 2]) > .85
                    for variant, D in self.transforms.items():
                        q = transform(source, D)
                        rows.append(dict(**record, variant=variant, target=target_name, all_points=metrics(q, tree), original_tie_supported=metrics(q[support], tree), original_support_fraction=float(support.mean()), fixed_original_roadlike=metrics(q[support & roadlike & valid[original_ids]], tree), fixed_original_nonroad=metrics(q[support & ~roadlike & valid[original_ids]], tree)))
        self.write('heldout-metrics.json', rows)
        summary = []
        for target_name in self.targets:
            for variant in self.transforms:
                for unit in ['infrastructure', 'vehicle']:
                    group = [r for r in rows if r['target'] == target_name and r['variant'] == variant and (r['agent'] == 0) == (unit == 'infrastructure')]
                    summary.append(dict(target=target_name, variant=variant, unit=unit, frames=len(group), median_frame_nn_median_m=float(np.median([r['all_points']['median_m'] for r in group])), median_frame_nn_p95_m=float(np.median([r['all_points']['p95_m'] for r in group])), median_original_supported_nn_median_m=float(np.median([r['original_tie_supported']['median_m'] for r in group]))))
        self.write('heldout-summary.json', summary)
        print('HELDOUT complete', flush=True)

    def fixed_eight(self):
        previous = BASE / 'shared-enu-validation/summary.json'
        old_summary = json.loads(previous.read_text())
        if not np.allclose(old_summary['frozen_D'], self.transforms['original_shared'], atol=1e-12, rtol=0):
            raise ValueError('Existing original-shared validation has a different D')
        summaries = {'original_shared': {**old_summary, 'reused_existing_summary': str(previous)}}
        for name in ['infra_only_opacity005', 'fused_opacity005', 'fused_positive', 'infra_only_positive']:
            calibration = self.output / (name + '-calibration.json')
            output = self.output / 'fixed-eight' / name
            validator = SharedMapValidation if name == 'fused_opacity005' else MetricsOnlyValidation
            validator(calibration, output).run()
            summaries[name] = json.loads((output / 'summary.json').read_text())
        self.write('fixed-eight-summary.json', summaries)
        self.filtered_eight()

    def filtered_eight(self):
        # Fixed source planes, anchors and gates; fit a common >.05 map plane
        # once at each original anchor, then compare every frozen transform.
        estimator = HybridRoadHeightEstimator()
        target, _, _, _ = self.targets['opacity005']
        records = json.loads((BASE / 'final-comparison/diagnostics.json').read_text())
        rows = []
        for record in records:
            xy = np.asarray(record['comparison_xy_map'])
            case = f"{record['run_id']}-f{record['frame_offset']:06d}"
            with np.load(BASE / 'final-comparison' / case / 'road-support.npz') as data:
                source = data['native_support'].copy()
            candidates = target[(np.linalg.norm(target[:, :2] - xy, axis=1) <= 15) & (target[:, 2] >= -4.3) & (target[:, 2] <= -1.3)]
            try:
                candidates = estimator._balanced(candidates)
                plane, quality, mask = estimator._plane(candidates, lidar=False)
                support = candidates[mask]
            except ValueError as error:
                rows.append(dict(run_id=record['run_id'], frame_id=record['frame_id'], supported=False, reason=str(error)))
                continue
            for variant, D in self.transforms.items():
                T = D @ self.W
                n, d = transform_plane(record['source_road']['normal'], record['source_road']['offset'], T)
                ss = transform(source, T)
                _, ids = np.unique(np.floor(ss[:, :2]), axis=0, return_index=True)
                overlap = float(np.mean(cKDTree(support[:, :2]).query(ss[ids, :2])[0] <= estimator.config.footprint_match_distance_m))
                ok = quality['extent_accepted'] and inside(support, xy) and inside(ss, xy) and overlap >= estimator.config.min_footprint_overlap
                rows.append(dict(run_id=record['run_id'], frame_id=record['frame_id'], frame_offset=record['frame_offset'], variant=variant, supported=bool(ok), signed_height_m=height_at(n, d, xy)-height_at(plane.normal, plane.offset_m, xy) if ok else None, normal_angle_deg=float(np.degrees(np.arccos(np.clip(n @ plane.normal, -1, 1)))), comparison_xy_map=xy.tolist(), road_footprint_overlap=overlap, map_inliers=len(support), map_filter='opacity >.05; .15m voxels; historical Z band and radius; identical plane for every transform'))
        self.write('fixed-eight-opacity005-surface.json', rows)

    def plot(self):
        target = self.targets['opacity005'][0]
        target = target[(abs(target[:, :2]) < 90).all(1) & (target[:, 2] > -8) & (target[:, 2] < 18)]
        fig, axes = plt.subplots(2, 3, figsize=(17, 10), layout='constrained')
        colors = ['#db9225', '#138cb0', '#b9417c', '#579746']
        for row, variant in enumerate(['released', 'fused_opacity005']):
            for col, (x, y) in enumerate([(0, 1), (0, 2), (1, 2)]):
                ax = axes[row, col]
                ax.scatter(target[:, x], target[:, y], s=.4, c='#747a82', alpha=.2, rasterized=True, label='GSplat opacity >.05')
                for key, color, label in zip(['infra_representative', 'global_1', 'global_2', 'global_3'], colors, ['stationary infrastructure', 'vehicle 1', 'vehicle 2', 'vehicle 3']):
                    p = transform(self.clouds[key], self.transforms[variant])
                    ax.scatter(p[:, x], p[:, y], s=.8, c=color, alpha=.45, rasterized=True, label=label)
                ax.set(xlim=(-80, 70), ylim=(-70, 70) if col == 0 else (-7, 18), xlabel=f'Map {"XYZ"[x]} (m)', ylabel=f'Map {"XYZ"[y]} (m)', title=f'{variant}: '+['top', 'X side', 'Y side'][col])
                ax.set_aspect('equal', adjustable='box')
                ax.grid(alpha=.2)
            axes[row, 0].legend(fontsize=7, loc='upper right')
        fig.suptitle('One frame per vehicle + one stationary infrastructure frame; all released relative poses frozen\nFull static clouds shown, one common SE(3); geometry consistency, not independent GT')
        fig.savefig(self.output / 'combined-source-target.png', dpi=160)
        plt.close(fig)

    def run(self):
        self.fits()
        self.coverage()
        self.heldout()
        self.fixed_eight()
        self.plot()
        self.report()

    def report(self):
        heldout = json.loads((self.output / 'heldout-summary.json').read_text())
        stability = json.loads((self.output / 'stability.json').read_text())
        eight = json.loads((self.output / 'fixed-eight-summary.json').read_text())
        primary = self.results['fused_opacity005']
        selected_nn = next(r['median_frame_nn_median_m'] for r in heldout if r['variant'] == 'fused_opacity005' and r['target'] == 'opacity005' and r['unit'] == 'vehicle')
        previous_nn = next(r['median_frame_nn_median_m'] for r in heldout if r['variant'] == 'original_shared' and r['target'] == 'opacity005' and r['unit'] == 'vehicle')
        lines = ['# Overlap-selected common ENU/map tie', '',
            'Exactly three vehicle frames and one stationary infrastructure frame; all released poses frozen. Full static clouds form one unique 0.25m voxel union. Only one common SE(3) is fitted. No runtime acceptance gates changed.', '',
            f'The selected fusion adds geometric coverage but does not improve the reserved vehicle residuals over the previous shared tie: median frame NN median against the same >0.05 target is {selected_nn:.3f}m versus {previous_nn:.3f}m. The unchanged eight-case road-plane comparison likewise retains discrepancies. This experiment does not establish a more accurate reference.', '',
            'The primary opacity >0.05 target follows the preceding surface-layer sensitivity. The positive-opacity control retains the earlier representation. Neither filter certifies a physical surface.', '',
            '| Training unit | Take | Frame | Offset |', '|---|---:|---|---:|']
        for r in primary['calibration_frames']:
            lines.append(f"| {r['agent']} | {r['take']} | {r['frame_id']} | {r['frame_offset']} |")
        lines += ['', f"Fused source: {primary['source_points']:,} voxels. Target: {primary['target_points']:,} centers.", '',
            '`map_T_released_enu = D @ original_W`. [Primary transform, provenance and traces](fused_opacity005-calibration.json).', '',
            f"Final fitting support: {primary['final_solver_support']['correspondences']:,} correspondences, {primary['final_solver_support']['roadlike_count']:,} roadlike and {primary['final_solver_support']['nonroad_count']:,} nonroad. Normal-matrix numerical rank is {primary['final_solver_support']['normal_matrix_rank_rcond_1e10']}/6. The fixed iteration limit was reached; the final update was {primary['final_solver_support']['delta_rotation_deg']:.7f} degrees and {primary['final_solver_support']['delta_translation_m']:.7f}m. Full rank alone is not an accuracy certificate.", '',
            '## Forty reserved scans', '',
            'Median of full static scan NN medians, in meters. All transforms use the identical points and target within each column. Ten stationary observations are repeated measurements of one unit.', '',
            '| Transform | >0.05 infrastructure | >0.05 vehicles | Positive infrastructure | Positive vehicles |', '|---|---:|---:|---:|---:|']
        for variant in self.transforms:
            values = [next(r['median_frame_nn_median_m'] for r in heldout if r['variant'] == variant and r['target'] == target and r['unit'] == unit) for target in ['opacity005', 'positive'] for unit in ['infrastructure', 'vehicle']]
            lines.append(f'| {variant} | ' + ' | '.join(f'{v:.3f}' for v in values) + ' |')
        lines += ['', '## Eight original road cases', '',
            'Identical original source and positive-opacity map planes, common XY anchors, points and support gates. The primary transform was fitted to the >0.05 target; this table measures it against the unchanged original planes.', '',
            '| Transform | Supported cases | Median absolute height | Median normal angle |', '|---|---:|---:|---:|']
        for variant, summary in eight.items():
            r = summary['shared_enu_map']
            height = r['median_absolute_supported_road_height_m']
            value = f'{height:.3f}m' if height is not None else 'unsupported'
            lines.append(f"| {variant} | {r['supported_road_pairs']} | {value} | {r['median_normal_angle_deg']:.3f}° |")
        lines += ['', '[Filtered map-plane sensitivity](fixed-eight-opacity005-surface.json) uses a common >0.05 map plane for every transform, with all original source planes and anchors retained.', '',
            '## Source selection stability', '',
            '| Source alternative | Relative rotation | Maximum displacement on origin and ±30m cross |', '|---|---:|---:|']
        for r in stability:
            lines.append(f"| {r['variant']} | {r['relative_rotation_deg']:.4f}° | {r['max_shift_30m_cross_m']:.4f}m |")
        lines += ['', '[Incremental coverage and roadlike/nonroad geometry support](coverage.json) includes original and final poses at 0.25, 0.5 and 1m occupancy grids. Exact map-voxel intersection is grid-boundary sensitive. Roadlike means target normal |nz| >0.85, not a semantic road label.', '',
            'The new fit uses every unique fused source voxel and balances horizontal versus other target normals globally. The earlier shared fit capped and balanced separate take/unit groups. Thus comparison with the earlier result changes source selection and weighting together. The new station-only controls use the same new solver and each identical target to isolate adding the selected vehicle geometry.', '',
            'NN and plane discrepancies measure consistency with map geometry, not independent ground-truth or localization error. Sampling, occlusion, incomplete overlap and remaining dynamic returns affect them. A shared map tie preserves all relative released geometry; it cannot establish that the released ICP poses or Gaussian surfaces are physically correct. Source-selection stability measures coverage sensitivity, not geodetic uncertainty.', '',
            '![Combined full source clouds and map, top and side views](combined-source-target.png)', '']
        (self.output / 'report.md').write_text('\n'.join(lines))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selection', type=Path, default=BASE / 'overlap-selected-calibration/selection')
    parser.add_argument('--output', type=Path, default=BASE / 'overlap-selected-calibration/fit')
    args = parser.parse_args()
    OverlapSelectedMapCalibration(args.selection, args.output).run()

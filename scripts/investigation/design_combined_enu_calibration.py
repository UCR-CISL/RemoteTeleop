"""Design, fit and validate one shared CooperScene ENU-to-GSplat rigid transform.

Released poses stay frozen: every source cloud already contains
``original_W @ released_ENU`` and only one common correction ``D`` is fitted,
giving ``map_T_lidar = D @ original_W @ released_ENU_T_lidar``.

Calibration items, spatial calibration tiles, weighting, map-surface filters and
classification thresholds are declared in ``CONFIG`` and written before any fit.
Validation uses the 40 previously reserved scans (all five takes, all units) and
the untouched checkerboard tiles. Results are geometric consistency with the
GSplat map, not independent ground truth.
"""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
from dataclasses import dataclass, field
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.investigation.calibrate_overlap_selected_map import OverlapSelectedMapCalibration
from scripts.investigation.calibrate_shared_enu_map import transform, voxel
from src.localization.gaussian_map import load_gaussian_map

BASE = ROOT / 'artifacts/reference-refinement'
SELECTION = BASE / 'overlap-selected-calibration/selection'
UNIT_NAMES = {0: 'infrastructure', 1: 'vehicle 1', 2: 'vehicle 2', 3: 'vehicle 3'}
# Validated categorical palette (dataviz all-pairs check, light surface).
UNIT_COLORS = {0: '#2a78d6', 1: '#eb6834', 2: '#1baf7a', 3: '#4a3aa7'}
MAP_COLOR = '#8a8d91'

CONFIG = dict(
    declared_before_fitting=True,
    tile_m=20.0,
    calibration_tile_parity=0,
    tile_rule='W-frame tiles floor(x/20), floor(y/20); parity (ix+iy) mod 2; parity 0 calibrates, parity 1 is untouched validation',
    source_voxel_m=0.25,
    source_normal_radius_m=0.6,
    ground_rule='source normal |nz| > 0.9 and height above the item 2 m cell minimum < 0.3 m',
    vertical_rule='source normal |nz| < 0.3',
    other_rule='remaining points (vegetation, slopes, clutter); evaluated, never fitted',
    fit_classes=['ground', 'vertical'],
    weighting='equal class totals; within class equal unit totals; within unit equal (tile, vertical azimuth quadrant) cells; uniform within a cell',
    cap_points_per_item_cell=300,
    huber_m=0.15,
    gates_m=[3.0, 1.5, 0.75, 0.4],
    max_iterations_per_gate=25,
    final_gate_iterations=60,
    weight_renormalisation='balanced weights recomputed every iteration on the surviving correspondences (run1 balanced only at design time; preserved separately)',
    normal_compatibility_deg=30.0,
    map_voxel_m=0.15,
    map_neighbourhood_radius_m=0.6,
    map_neighbourhood_k=20,
    map_min_neighbours=8,
    primary_target=dict(min_opacity=0.05, plane_point='local_centroid', max_local_sigma_m=0.10),
    primary_calibration_set='all_pool',
    attribution_thresholds=dict(min_cell_support=100, min_cell_support_note='lowered from 200 after run1 showed most 20 m unit/class cells below 200 capped samples; diagnostic labels only', ambiguous_map_fraction=0.5, ambiguous_sigma_m=0.10, inconsistent_source_m=0.10, min_source_pairs=100, high_residual_m=0.15),
    validation='40 reserved single scans (two per take/unit) never used by any fit; plus untouched parity-1 tiles; predeclared structure ROIs as fixed patches',
)


@dataclass
class SourceItem:
    name: str
    unit: int
    take: int
    kind: str
    points: np.ndarray
    normals: np.ndarray = field(default=None)
    cls: np.ndarray = field(default=None)

    def __post_init__(self):
        self.points = voxel(self.points, CONFIG['source_voxel_m'])
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(self.points))
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=CONFIG['source_normal_radius_m'], max_nn=30))
        self.normals = np.asarray(pc.normals).copy()
        nz = np.abs(self.normals[:, 2])
        cell = np.floor(self.points[:, :2] / 2).astype(np.int64)
        _, inverse = np.unique(cell, axis=0, return_inverse=True)
        cell_min = np.full(inverse.max() + 1, np.inf)
        np.minimum.at(cell_min, inverse.ravel(), self.points[:, 2])
        above = self.points[:, 2] - cell_min[inverse.ravel()]
        self.cls = np.full(len(self.points), 'other', dtype=object)
        self.cls[nz < .3] = 'vertical'
        self.cls[(nz > .9) & (above < .3)] = 'ground'

    @property
    def tiles(self):
        return np.floor(self.points[:, :2] / CONFIG['tile_m']).astype(np.int64)

    @property
    def parity(self):
        t = self.tiles
        return (t[:, 0] + t[:, 1]) % 2

    @property
    def azimuth_bin(self):
        return (np.floor(np.mod(np.arctan2(self.normals[:, 1], self.normals[:, 0]), np.pi) / (np.pi / 4))).astype(int)

    @property
    def cell_keys(self):
        if getattr(self, '_cell_keys', None) is None:
            t, az = self.tiles, np.where(self.cls == 'vertical', self.azimuth_bin, -1)
            self._cell_keys = np.asarray([f'{a}|{b}|{z}' for (a, b), z in zip(t, az)])
        return self._cell_keys

    def label(self):
        return dict(name=self.name, unit=self.unit, take=self.take, kind=self.kind, points=len(self.points))


class MapSurface:
    """GSplat centers with geometry-only local surface statistics."""

    def __init__(self, gaussian, min_opacity):
        keep = np.isfinite(gaussian.means).all(1) & np.isfinite(gaussian.opacities) & (gaussian.opacities > min_opacity) & (np.abs(gaussian.means).max(1) <= 1e4)
        self.min_opacity = min_opacity
        self.points = voxel(gaussian.means[keep].astype(float), CONFIG['map_voxel_m'])
        tree = cKDTree(self.points)
        d, i = tree.query(self.points, k=CONFIG['map_neighbourhood_k'], distance_upper_bound=CONFIG['map_neighbourhood_radius_m'], workers=8)
        ok = np.isfinite(d)
        self.count = ok.sum(1)
        P = np.where(ok[..., None], self.points[np.minimum(i, len(self.points) - 1)], np.nan)
        self.centroid = np.nanmean(P, 1)
        X = np.nan_to_num(P - self.centroid[:, None])
        w, v = np.linalg.eigh(np.einsum('nki,nkj->nij', X, X) / np.maximum(self.count - 1, 1)[:, None, None])
        self.sigma = np.sqrt(np.clip(w[:, 0], 0, None))
        self.normals = v[:, :, 0] * np.where(v[:, 2, 0] < 0, -1, 1)[:, None]
        self.valid = self.count >= CONFIG['map_min_neighbours']

    def target(self, plane_point, max_sigma):
        keep = self.valid & (self.sigma < (max_sigma if max_sigma is not None else np.inf))
        return Target(self.points[keep], (self.centroid if plane_point == 'local_centroid' else self.points)[keep], self.normals[keep], self.sigma[keep], f'opacity>{self.min_opacity:g}, plane={plane_point}, sigma<{max_sigma}')

    def stats(self):
        nz = np.abs(self.normals[:, 2])
        out = dict(min_opacity=self.min_opacity, centers=len(self.points), valid_neighbourhoods=int(self.valid.sum()))
        for name, m in [('all', self.valid), ('roadlike_nz_gt_085', self.valid & (nz > .85)), ('vertical_nz_lt_03', self.valid & (nz < .3))]:
            s = self.sigma[m]
            out[name] = dict(count=int(m.sum()), sigma_quantiles_m={str(q): float(np.quantile(s, q)) for q in [.1, .25, .5, .75, .9]}, fraction_sigma_lt_010=float(np.mean(s < .10)), fraction_sigma_lt_005=float(np.mean(s < .05)))
        return out


@dataclass
class Target:
    points: np.ndarray
    plane_points: np.ndarray
    normals: np.ndarray
    sigma: np.ndarray
    label: str

    def __post_init__(self):
        self.tree = cKDTree(self.points)


class BalancedSolver:
    """Robust point-to-plane Gauss-Newton for one common SE(3)."""

    def __init__(self, target: Target):
        self.target = target

    @staticmethod
    def design(items, parity, rng):
        """Subsample and weight once; weights depend only on source geometry."""
        rows = []
        for idx, item in enumerate(items):
            mask = np.isin(item.cls, CONFIG['fit_classes'])
            if parity is not None:
                mask &= item.parity == parity
            sel = np.flatnonzero(mask)
            keys = np.char.add(np.char.add(item.cls[sel].astype(str), '|'), item.cell_keys[sel].astype(str))
            chosen = []
            for key in np.unique(keys):
                ids = sel[keys == key]
                chosen.append(ids if len(ids) <= CONFIG['cap_points_per_item_cell'] else rng.choice(ids, CONFIG['cap_points_per_item_cell'], replace=False))
            chosen = np.sort(np.concatenate(chosen)) if chosen else np.array([], int)
            rows.append(dict(points=item.points[chosen], normals=item.normals[chosen], cls=item.cls[chosen], unit=np.full(len(chosen), item.unit), take=np.full(len(chosen), item.take), item=np.full(len(chosen), idx), cell=item.cell_keys[chosen]))
        cat = {k: np.concatenate([r[k] for r in rows]) for k in ['points', 'normals', 'cls', 'unit', 'take', 'item', 'cell']}
        return cat

    @staticmethod
    def balanced_weights(cls, unit, cell):
        w = np.zeros(len(cls))
        classes = np.unique(cls)
        for c in classes:
            mc = cls == c
            units = np.unique(unit[mc])
            for u in units:
                mu = mc & (unit == u)
                cells, inverse, counts = np.unique(cell[mu], return_inverse=True, return_counts=True)
                w[mu] = 1 / (len(classes) * len(units) * len(cells) * counts[inverse])
        return w

    def correspond(self, design, D, gate):
        x = transform(design['points'], D)
        dist, ids = self.target.tree.query(x, workers=8)
        ok = dist < gate
        ids = np.where(ok, ids, 0)
        n = self.target.normals[ids]
        src_n = design['normals'] @ D[:3, :3].T
        ok &= np.abs(np.sum(n * src_n, 1)) > np.cos(np.deg2rad(CONFIG['normal_compatibility_deg']))
        r = np.sum(n * (x - self.target.plane_points[ids]), 1)
        return x, n, r, ok, ids, dist

    def fit(self, design, weighting='balanced', init=None):
        D = np.eye(4) if init is None else init.copy()
        base = self.balanced_weights(design['cls'], design['unit'], design['cell']) if weighting == 'balanced' else np.ones(len(design['cls']))
        trace = []
        for gate in CONFIG['gates_m']:
            for iteration in range(CONFIG['final_gate_iterations'] if gate == CONFIG['gates_m'][-1] else CONFIG['max_iterations_per_gate']):
                x, n, r, ok, ids, _ = self.correspond(design, D, gate)
                if ok.sum() < 200:
                    raise RuntimeError('insufficient correspondences')
                A = np.c_[np.cross(x[ok], n[ok]), n[ok]]
                if weighting == 'balanced':
                    base = np.zeros(len(ok))
                    base[ok] = self.balanced_weights(design['cls'][ok], design['unit'][ok], design['cell'][ok])
                w = base[ok] / base[ok].sum() * np.minimum(1, CONFIG['huber_m'] / np.maximum(np.abs(r[ok]), 1e-12))
                H = A.T @ (w[:, None] * A)
                delta = np.linalg.lstsq(H, -(A.T @ (w * r[ok])), rcond=1e-12)[0]
                delta /= max(np.linalg.norm(delta[:3]) / np.deg2rad(2), np.linalg.norm(delta[3:]) / .5, 1.)
                step = np.eye(4)
                step[:3, :3] = Rotation.from_rotvec(delta[:3]).as_matrix()
                step[:3, 3] = delta[3:]
                D = step @ D
                trace.append(dict(gate_m=gate, iteration=iteration, correspondences=int(ok.sum()), median_abs_residual_m=float(np.median(np.abs(r[ok]))), step_norm=float(np.linalg.norm(delta))))
                if np.linalg.norm(delta) < 1e-7:
                    break
        x, n, r, ok, ids, _ = self.correspond(design, D, CONFIG['gates_m'][-1])
        if weighting == 'balanced':
            base = np.zeros(len(ok))
            base[ok] = self.balanced_weights(design['cls'][ok], design['unit'][ok], design['cell'][ok])
        tail = [t['step_norm'] for t in trace[-10:]]
        info = self.information(x[ok], n[ok], base[ok] / base[ok].sum())
        support = dict(correspondences=int(ok.sum()), by_class={c: int((ok & (design['cls'] == c)).sum()) for c in CONFIG['fit_classes']}, by_unit={UNIT_NAMES[u]: int((ok & (design['unit'] == u)).sum()) for u in range(4)}, weight_share_by_unit={UNIT_NAMES[u]: float(base[ok & (design['unit'] == u)].sum() / base[ok].sum()) for u in range(4)}, weight_share_by_class={c: float(base[ok & (design['cls'] == c)].sum() / base[ok].sum()) for c in CONFIG['fit_classes']}, converged=bool(trace[-1]['step_norm'] < 1e-7), last10_max_step_norm=float(max(tail)), **info)
        return D, trace, support

    @staticmethod
    def information(x, n, w, lever=30.):
        A = np.c_[np.cross(x, n) / lever, n]
        H = A.T @ (w[:, None] * A)
        e = np.linalg.eigvalsh(H)
        return dict(scaled_eigenvalues=e.tolist(), condition=float(e[-1] / max(e[0], 1e-300)), lever_m=lever)


def describe(D, reference=None):
    grid = np.array([[x, y, z] for x in (-60, 0, 60) for y in (-60, 0, 60) for z in (-3, 10)], float)
    out = dict(D=D.tolist(), rpy_deg=Rotation.from_matrix(D[:3, :3]).as_euler('xyz', degrees=True).tolist(), translation_m=D[:3, 3].tolist())
    if reference is not None:
        delta = D @ np.linalg.inv(reference)
        disp = np.linalg.norm(transform(grid, D) - transform(grid, reference), axis=1)
        dz = transform(grid, D)[:, 2] - transform(grid, reference)[:, 2]
        out.update(relative_rotation_deg=float(np.rad2deg(Rotation.from_matrix(delta[:3, :3]).magnitude())), max_displacement_120m_box_m=float(disp.max()), rms_displacement_120m_box_m=float(np.sqrt(np.mean(disp ** 2))), max_abs_height_change_m=float(np.abs(dz).max()))
    return out


class CombinedCloudCalibration:
    def __init__(self, output: Path, bootstrap: int):
        self.output = output
        self.bootstrap = bootstrap
        output.mkdir(parents=True, exist_ok=True)
        self.figures = output / 'figures'
        self.figures.mkdir(exist_ok=True)
        self.shared = json.loads((BASE / 'shared-enu-calibration/calibration.json').read_text())
        with Path(self.shared['map_path']).open('rb') as stream:
            if hashlib.file_digest(stream, 'sha256').hexdigest() != self.shared['map_sha256']:
                raise ValueError('Map content differs from prior provenance')
        self.prior_overlap = json.loads((BASE / 'overlap-selected-calibration/fit/fused_opacity005-calibration.json').read_text())
        self.transforms = {'released_W_only': np.eye(4), 'previous_shared': np.asarray(self.shared['D']), 'overlap_selected': np.asarray(self.prior_overlap['D'])}
        self.gaussian = load_gaussian_map(self.shared['map_path'])
        self.rng = np.random.default_rng(20261003)
        self.results = dict(config=CONFIG)
        self.write('config.json', CONFIG)

    def write(self, name, value):
        (self.output / name).write_text(json.dumps(value, indent=1, allow_nan=False, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o)) + '\n')

    # ------------------------------------------------------------------ inputs
    def load(self):
        with np.load(SELECTION / 'selected-clouds.npz') as s, np.load(BASE / 'shared-enu-calibration/input-clouds.npz') as a:
            if not np.allclose(s['original_W'], self.shared['original_W']):
                raise ValueError('W mismatch')
            self.prior_clouds = {k: s[k].copy() for k in ['infra_representative', 'global_1', 'global_2', 'global_3']}
            pool = [SourceItem('infra_rep_t1_f481510', 0, 1, 'scan', s['infra_representative'])]
            pool += [SourceItem(f'infra_agg_t{t}', 0, t, 'aggregate3', s[f'cal_{t}_0']) for t in range(1, 6)]
            for t in range(1, 6):
                for u in range(1, 4):
                    pool.append(SourceItem(f'overlap_t{t}_v{u}', u, t, 'scan', s[f'cal_{t}_{u}']))
                    pool.append(SourceItem(f'aggregate_t{t}_v{u}', u, t, 'aggregate3', a[f'cal_{t}_{u}']))
            manifest = json.loads((BASE / 'shared-enu-calibration/input-manifest.json').read_text())
            self.heldout = [SourceItem(f"heldout_t{r['take']}_u{r['agent']}_o{r['frame_offset']}", r['agent'], r['take'], 'scan', a[r['cache_key']]) for r in manifest if r['partition'] == 'heldout']
        self.pool = pool
        prior_names = {'infra_rep_t1_f481510', 'overlap_t1_v1', 'overlap_t2_v2', 'overlap_t2_v3'}
        self.sets = {'prior_overlap_set': [i for i in pool if i.name in prior_names], 'all_pool': pool}
        for g, name in zip(['global_1', 'global_2', 'global_3'], ['overlap_t1_v1', 'overlap_t2_v2', 'overlap_t2_v3']):
            if not np.array_equal(voxel(self.prior_clouds[g], .25), next(i for i in pool if i.name == name).points):
                raise ValueError(f'{g} is not {name}')
        selected = {(r['take'], r['agent'], r['frame_id']) for r in self.prior_overlap['calibration_frames']}
        held = {(r['take'], r['agent'], r['frame_id']) for r in manifest if r['partition'] == 'heldout'}
        sel_manifest = json.loads((SELECTION / 'manifest.json').read_text())
        pooled = selected | {(r['take'], r['agent'], r['frame_id']) for r in sel_manifest['per_take_selected']} | {(r['take'], r['agent'], r['frame_id']) for r in manifest if r['partition'] == 'calibration'}
        if pooled & held:
            raise ValueError('calibration/validation leakage')
        self.results['inputs'] = dict(pool=[i.label() for i in pool], heldout=[i.label() for i in self.heldout], leakage_check='no calibration frame identity occurs among the 40 validation scans')
        print('pool', len(pool), 'heldout', len(self.heldout), flush=True)

    # --------------------------------------------------------- 1 reproduction
    def reproduce(self):
        stub = object.__new__(OverlapSelectedMapCalibration)
        stub.gaussian = self.gaussian
        stub.targets = {'opacity005': stub.target(.05)}
        source = voxel(np.concatenate([voxel(self.prior_clouds['infra_representative'], .25)] + [self.prior_clouds[f'global_{a}'] for a in [1, 2, 3]]), .25)
        D, trace = stub.fit(source, 'opacity005')
        diff = float(np.abs(D - self.transforms['overlap_selected']).max())
        self.transforms['overlap_selected_reproduced'] = D
        self.results['reproduction'] = dict(max_abs_matrix_difference=diff, stored=describe(self.transforms['overlap_selected']), reproduced=describe(D, self.transforms['overlap_selected']), source_voxels=len(source), final=trace[-1])
        print('REPRODUCE max diff', diff, flush=True)

    # --------------------------------------------------------------- 5 map
    def map_surfaces(self):
        self.surfaces = {o: MapSurface(self.gaussian, o) for o in [0., .05, .3]}
        self.results['map_surface'] = {str(o): s.stats() for o, s in self.surfaces.items()}
        p = CONFIG['primary_target']
        self.targets = {
            'primary_o005_centroid_thin': self.surfaces[.05].target('local_centroid', p['max_local_sigma_m']),
            'o005_nearest_all': self.surfaces[.05].target('nearest_center', None),
            'o005_centroid_all': self.surfaces[.05].target('local_centroid', None),
            'o0_centroid_thin': self.surfaces[0.].target('local_centroid', p['max_local_sigma_m']),
            'o03_centroid_thin': self.surfaces[.3].target('local_centroid', p['max_local_sigma_m']),
        }
        self.primary = self.targets['primary_o005_centroid_thin']
        self.results['targets'] = {k: dict(label=t.label, points=len(t.points)) for k, t in self.targets.items()}
        print('targets', self.results['targets'], flush=True)

    # --------------------------------------------------------- 2 selection
    def footprint(self, item):
        """GSplat-independent complementary-coverage signature of one item."""
        keys = set()
        for c, (x, y), a in zip(item.cls, np.floor(item.points[:, :2] / 2).astype(int), item.azimuth_bin):
            if c != 'other':
                keys.add((x, y, -1 if c == 'ground' else int(a)))
        return keys

    def selection(self):
        prints = {i.name: self.footprint(i) for i in self.pool}
        infos = {}
        for i in self.pool:
            m = np.isin(i.cls, CONFIG['fit_classes'])
            A = np.c_[np.cross(i.points[m], i.normals[m]) / 30, i.normals[m]]
            infos[i.name] = (A.T @ A, m.sum())
        by_unit = {u: [i for i in self.pool if i.unit == u] for u in range(4)}
        rows = []
        for combo in itertools.product(*[by_unit[u] for u in range(4)]):
            cover = set().union(*[prints[i.name] for i in combo])
            # Per-unit normalised information so no unit dominates conditioning.
            H = sum(infos[i.name][0] / infos[i.name][1] for i in combo)
            e = np.linalg.eigvalsh(H)
            vert = [k for k in cover if k[2] >= 0]
            az = np.bincount([k[2] for k in vert], minlength=4)
            rows.append(dict(items=[i.name for i in combo], cells=len(cover), ground_cells=len(cover) - len(vert), vertical_cells=len(vert), vertical_azimuth_balance=float(az.min() / max(az.max(), 1)), min_eigenvalue=float(e[0]), condition=float(e[-1] / e[0]), takes=len({i.take for i in combo})))
        prior = next(r for r in rows if set(r['items']) == {i.name for i in self.sets['prior_overlap_set']})
        eligible = [r for r in rows if r['min_eigenvalue'] >= prior['min_eigenvalue'] and r['vertical_azimuth_balance'] >= prior['vertical_azimuth_balance']]
        best = max(eligible, key=lambda r: (r['cells'], r['min_eigenvalue']))
        self.sets['coverage_designed_set'] = [i for i in self.pool if i.name in best['items']]
        kinds = {i.name: i.kind for i in self.pool}
        scans = [r for r in eligible if all(kinds[n] == 'scan' for n in r['items'])]
        best_scans = max(scans, key=lambda r: (r['cells'], r['min_eigenvalue']))
        self.sets['coverage_designed_scans'] = [i for i in self.pool if i.name in best_scans['items']]
        cells = np.array([r['cells'] for r in rows])
        self.results['selection'] = dict(rule='one item per unit; maximise unique (2 m cell, ground or vertical-azimuth quadrant) footprint keys from source geometry only; constraints: min eigenvalue and azimuth balance not below the prior set', combinations=len(rows), prior=prior, prior_cells_percentile=float(np.mean(cells <= prior['cells']) * 100), designed=best, designed_single_scans=best_scans, all_pool_cells=len(set().union(*prints.values())), per_item={i.name: dict(unit=i.unit, take=i.take, kind=i.kind, footprint_keys=len(prints[i.name]), points=len(i.points)) for i in self.pool})
        self.prints = prints
        print('SELECTION prior', prior['cells'], 'designed', best['items'], best['cells'], flush=True)

    # ---------------------------------------------------------------- 3,4,6 fits
    def fit_variant(self, name, items, target_name='primary_o005_centroid_thin', parity=CONFIG['calibration_tile_parity'], weighting='balanced', init=None):
        solver = BalancedSolver(self.targets[target_name])
        design = solver.design(items, parity, np.random.default_rng(7))
        D, trace, support = solver.fit(design, weighting, init)
        self.transforms[name] = D
        rec = dict(items=[i.name for i in items], target=target_name, parity=parity, weighting=weighting, design_points=len(design['cls']), support=support, iterations=len(trace), trace_tail=trace[-3:], **describe(D, self.transforms.get('all_pool_balanced', None)))
        self.results.setdefault('fits', {})[name] = rec
        print('FIT', name, np.round(rec['rpy_deg'], 4), np.round(rec['translation_m'], 4), support['correspondences'], flush=True)
        return D

    def fits(self):
        pool = self.pool
        self.fit_variant('all_pool_balanced', pool)  # predeclared primary
        self.results['fits']['all_pool_balanced'].update(describe(self.transforms['all_pool_balanced']))
        self.fit_variant('all_pool_balanced_from_previous_init', pool, init=self.transforms['previous_shared'])
        self.fit_variant('prior_set_balanced', self.sets['prior_overlap_set'])
        self.fit_variant('prior_set_balanced_all_tiles', self.sets['prior_overlap_set'], parity=None)
        self.fit_variant('designed_set_balanced', self.sets['coverage_designed_set'])
        self.fit_variant('designed_scans_balanced', self.sets['coverage_designed_scans'])
        self.fit_variant('all_pool_uniform_weights', pool, weighting='uniform')
        self.fit_variant('all_pool_balanced_all_tiles', pool, parity=None)
        self.fit_variant('all_pool_balanced_parity1', pool, parity=1)
        for t in ['o005_nearest_all', 'o005_centroid_all', 'o0_centroid_thin', 'o03_centroid_thin']:
            self.fit_variant(f'all_pool_target_{t}', pool, target_name=t)
        for u in range(4):
            self.fit_variant(f'leave_out_{UNIT_NAMES[u].replace(" ", "")}', [i for i in pool if i.unit != u])
        for t in range(1, 6):
            self.fit_variant(f'leave_out_take{t}', [i for i in pool if i.take != t])
        for u in range(4):
            self.fit_variant(f'only_{UNIT_NAMES[u].replace(" ", "")}', [i for i in pool if i.unit == u])
        self.fit_variant('vehicles_only', [i for i in pool if i.unit > 0])
        boot = np.random.default_rng(11)
        for b in range(self.bootstrap):
            items = []
            for u in range(4):
                group = [i for i in pool if i.unit == u]
                items += [group[j] for j in boot.integers(0, len(group), len(group))]
            self.fit_variant(f'bootstrap_{b:02d}', items)
        self.class_only_fits()

    def class_only_fits(self):
        """Ground-only and vertical-only fits expose which DoF each class drives."""
        for cls in CONFIG['fit_classes']:
            saved = CONFIG['fit_classes']
            CONFIG['fit_classes'] = [cls]
            try:
                self.fit_variant(f'{cls}_only_class', self.pool, init=self.transforms['all_pool_balanced'])
            except RuntimeError as error:
                self.results['fits'][f'{cls}_only_class'] = dict(error=str(error))
            finally:
                CONFIG['fit_classes'] = saved

    # ---------------------------------------------------------- 6 validation
    def evaluate(self, names):
        target = self.primary
        nearest = self.targets['o005_nearest_all']
        rows = []
        per_point = {}
        for item in self.heldout + self.pool:
            partition = 'heldout_scan' if item.name.startswith('heldout') else 'calibration_item'
            ok_all = np.ones(len(item.points), bool)
            cache = {}
            for name in names:
                x = transform(item.points, self.transforms[name])
                d, ids = target.tree.query(x, workers=8)
                ids = np.minimum(ids, len(target.points) - 1)
                n = target.normals[ids]
                compat = np.abs(np.sum(n * (item.normals @ self.transforms[name][:3, :3].T), 1)) > np.cos(np.deg2rad(CONFIG['normal_compatibility_deg']))
                ok_all &= (d < 1.0) & compat
                r = np.sum(n * (x - target.plane_points[ids]), 1)
                nn = nearest.tree.query(x, workers=8)[0]
                cache[name] = (r, nn, ids)
            for name in names:
                r, nn, ids = cache[name]
                per_point.setdefault(partition, {}).setdefault(name, []).append(np.c_[r[ok_all], nn[ok_all], item.parity[ok_all], np.full(ok_all.sum(), item.unit), np.full(ok_all.sum(), item.take), np.select([item.cls[ok_all] == 'ground', item.cls[ok_all] == 'vertical'], [0, 1], 2), item.points[ok_all][:, :2]])
        self.eval_points = {p: {n: np.concatenate(v) for n, v in d.items()} for p, d in per_point.items()}
        summary = []
        for partition, by_name in self.eval_points.items():
            for name, a in by_name.items():
                for parity in [0, 1, None]:
                    for cls, cname in [(0, 'ground'), (1, 'vertical'), (2, 'other'), (None, 'all')]:
                        m = np.ones(len(a), bool)
                        if parity is not None:
                            m &= a[:, 2] == parity
                        if cls is not None:
                            m &= a[:, 5] == cls
                        if m.sum() == 0:
                            continue
                        summary.append(dict(partition=partition, transform=name, tiles='calibration' if parity == 0 else 'validation' if parity == 1 else 'all', cls=cname, points=int(m.sum()), median_abs_plane_m=float(np.median(np.abs(a[m, 0]))), p90_abs_plane_m=float(np.quantile(np.abs(a[m, 0]), .9)), median_signed_plane_m=float(np.median(a[m, 0])), median_nn_center_m=float(np.median(a[m, 1]))))
        self.results['validation_summary'] = summary
        per_unit = []
        a_all = self.eval_points['heldout_scan']
        for name, a in a_all.items():
            for u in range(4):
                for t in range(1, 6):
                    for cls, cname in [(0, 'ground'), (1, 'vertical')]:
                        m = (a[:, 3] == u) & (a[:, 4] == t) & (a[:, 5] == cls)
                        if m.sum() >= 50:
                            per_unit.append(dict(transform=name, unit=u, take=t, cls=cname, points=int(m.sum()), median_abs_plane_m=float(np.median(np.abs(a[m, 0]))), median_signed_plane_m=float(np.median(a[m, 0]))))
        self.results['validation_per_unit_take'] = per_unit
        self.tile_road(names)
        self.roi_checks(names)

    def tile_road(self, names):
        """10 m tiles: heldout ground height bias and source/map road-plane normal disagreement."""
        rows = []
        target = self.primary
        pts = {n: [] for n in names}
        for item in self.heldout:
            g = item.cls == 'ground'
            for name in names:
                x = transform(item.points[g], self.transforms[name])
                pts[name].append(np.c_[x, item.points[g][:, :2], np.full(g.sum(), item.unit)])
        for name in names:
            a = np.concatenate(pts[name])
            d, ids = target.tree.query(a[:, :3], workers=8)
            ids = np.minimum(ids, len(target.points) - 1)
            keep = (d < 1.0) & (np.abs(target.normals[ids, 2]) > .9)
            a, ids = a[keep], ids[keep]
            tile = np.floor(a[:, 3:5] / 10).astype(int)
            for key in np.unique(tile, axis=0):
                m = (tile == key).all(1)
                if m.sum() < 150:
                    continue
                src = a[m, :3]
                c = src.mean(0)
                n_src = np.linalg.svd(src - c)[2][2]
                n_src *= np.sign(n_src[2])
                q = target.plane_points[ids[m]]
                n_map = np.linalg.svd(q - q.mean(0))[2][2]
                n_map *= np.sign(n_map[2])
                h = (np.dot(n_map, c - q.mean(0)))
                parity = int((np.floor(key[0] / 2) + np.floor(key[1] / 2)) % 2)
                rows.append(dict(transform=name, tile_xy_W=(key * 10 + 5).tolist(), tile_parity=parity, points=int(m.sum()), units=sorted(set(a[m, 5].astype(int).tolist())), height_m=float(h), normal_angle_deg=float(np.degrees(np.arccos(np.clip(n_src @ n_map, -1, 1))))))
        self.results['tile_road'] = rows

    def roi_checks(self, names):
        rois = json.loads((ROOT / 'scripts/investigation/cooperscene_structure_rois.json').read_text())['rois']
        rows = []
        for roi in rois:
            n = np.asarray(roi['normal'], float); n /= np.linalg.norm(n)
            u = np.asarray(roi['tangent_u'], float); u -= n * (u @ n); u /= np.linalg.norm(u)
            v = np.cross(n, u)
            B = np.c_[u, v, n]
            hw = np.asarray(roi['half_widths_uvn'])
            anchor = np.asarray(roi['anchor'])
            local = lambda p: (p - anchor) @ B
            inside = lambda q: (np.abs(q) <= hw).all(1)
            parity = int((np.floor(anchor[0] / 20) + np.floor(anchor[1] / 20)) % 2)
            for name in names:
                Dinv = np.linalg.inv(self.transforms[name])
                mq = local(transform(self.primary.points, Dinv))
                mq = mq[inside(mq)]
                for unit in range(4):
                    s = np.concatenate([local(i.points) for i in self.heldout if i.unit == unit])
                    s = s[inside(s)]
                    if len(s) < 20 or len(mq) < 20:
                        rows.append(dict(roi=roi['id'], kind=roi['kind'], roi_tile_parity=parity, transform=name, unit=unit, supported=False, source_points=len(s), map_points=len(mq)))
                        continue
                    rows.append(dict(roi=roi['id'], kind=roi['kind'], roi_tile_parity=parity, transform=name, unit=unit, supported=True, source_points=len(s), map_points=len(mq), signed_separation_m=float(np.median(s[:, 2]) - np.median(mq[:, 2]))))
        self.results['roi_checks'] = rows

    # ---------------------------------------------------- 8 conflict analysis
    def conflicts(self):
        D = self.transforms['all_pool_balanced']
        # Unfiltered local-centroid surface: thick/ambiguous map regions stay in the analysis.
        self.diag_target = self.targets['o005_centroid_all']
        solver = BalancedSolver(self.diag_target)
        design = solver.design(self.pool, None, np.random.default_rng(7))
        x, n, r, ok, ids, _ = solver.correspond(design, D, .75)
        base = np.zeros(len(ok))
        base[ok] = solver.balanced_weights(design['cls'][ok], design['unit'][ok], design['cell'][ok])
        lever = 30.
        A = np.c_[np.cross(x, n) / lever, n]
        tiles = np.floor(design['points'][:, :2] / CONFIG['tile_m']).astype(int)
        parity = (tiles[:, 0] + tiles[:, 1]) % 2
        quadrant = np.where(design['points'][:, 1] >= 0, 'N', 'S').astype(object) + np.where(design['points'][:, 0] >= 0, 'E', 'W').astype(object)
        # D-invariant source consistency: vehicle points against same-take infrastructure surfaces.
        infra = {i.take: i for i in self.pool if i.kind == 'aggregate3' and i.unit == 0}
        src_sep = np.full(len(x), np.nan)
        for t, item in infra.items():
            tree = cKDTree(item.points)
            m = (design['take'] == t) & (design['unit'] > 0)
            d, j = tree.query(design['points'][m], workers=8)
            sep = np.abs(np.sum(item.normals[j] * (design['points'][m] - item.points[j]), 1))
            compat = np.abs(np.sum(item.normals[j] * design['normals'][m], 1)) > np.cos(np.deg2rad(30))
            src_sep[np.flatnonzero(m)[(d < .5) & compat]] = sep[(d < .5) & compat]
        # Noise floor of the same metric: stationary infrastructure of take t against take t+1.
        floor = []
        for t in range(1, 5):
            a, b = infra[t], infra[t + 1]
            d, j = cKDTree(b.points).query(a.points, workers=8)
            keep = (d < .5) & np.isin(a.cls, CONFIG['fit_classes']) & (np.abs(np.sum(b.normals[j] * a.normals, 1)) > np.cos(np.deg2rad(30)))
            sep = np.abs(np.sum(b.normals[j] * (a.points - b.points[j]), 1))[keep]
            floor.append(dict(pair=f'infra_agg_t{t} vs t{t + 1}', pairs=int(keep.sum()), median_m=float(np.median(sep)), p90_m=float(np.quantile(sep, .9))))
        self.results['source_consistency_floor'] = floor
        groups = {}
        for u in range(4):
            groups[f'unit:{UNIT_NAMES[u]}'] = design['unit'] == u
        for t in range(1, 6):
            groups[f'take:{t}'] = design['take'] == t
        for c in CONFIG['fit_classes']:
            groups[f'class:{c}'] = design['cls'] == c
        for q in ['NE', 'NW', 'SE', 'SW']:
            groups[f'quadrant:{q}'] = quadrant == q
        for p in [0, 1]:
            groups[f'tiles:{"calibration" if p == 0 else "validation"}'] = parity == p
        for u in range(4):
            for c in CONFIG['fit_classes']:
                groups[f'{UNIT_NAMES[u]}|{c}'] = (design['unit'] == u) & (design['cls'] == c)
        thr = CONFIG['attribution_thresholds']
        H_all = A[ok].T @ ((base[ok] / base[ok].sum())[:, None] * A[ok])
        e_all = np.linalg.eigvalsh(H_all)
        rows = []
        for gname, gm in groups.items():
            m = ok & gm
            if m.sum() < 50:
                rows.append(dict(group=gname, support=int(m.sum()), status='insufficient support'))
                continue
            w = base[m] / base[m].sum() * np.minimum(1, CONFIG['huber_m'] / np.maximum(np.abs(r[m]), 1e-12))
            Am, rm = A[m], r[m]
            H = Am.T @ (w[:, None] * Am)
            b = Am.T @ (w * rm)
            e, V = np.linalg.eigh(H)
            observable = e > 1e-3 * e_all[-1]
            Hinv = (V[:, observable] / e[observable]) @ V[:, observable].T
            delta = -Hinv @ b
            meat = (Am * (w * rm)[:, None]).T @ (Am * (w * rm)[:, None])
            cov = Hinv @ meat @ Hinv
            std = np.sqrt(np.clip(np.diag(cov), 0, None))
            rot_deg = np.rad2deg(delta[:3] / lever)
            amb = float(np.mean(self.diag_target.sigma[ids[m]] > thr['ambiguous_sigma_m']))
            ss = src_sep[m]
            ss = ss[np.isfinite(ss)]
            rows.append(dict(group=gname, support=int(m.sum()), observable_dof=int(observable.sum()), median_abs_residual_m=float(np.median(np.abs(rm))), median_signed_residual_m=float(np.median(rm)), preferred_rotation_deg=rot_deg.tolist(), preferred_translation_m=delta[3:].tolist(), sandwich_std_rotation_deg=np.rad2deg(std[:3] / lever).tolist(), sandwich_std_translation_m=std[3:].tolist(), map_sigma_gt_threshold_fraction=amb, vehicle_vs_infra_pairs=len(ss), vehicle_vs_infra_median_m=float(np.median(ss)) if len(ss) else None))
        self.results['conflicts'] = rows
        self.attribution(design, x, n, r, ok, ids, src_sep)

    def attribution(self, design, x, n, r, ok, ids, src_sep):
        """Per (unit, 20 m tile, class) cell: label residual cause category."""
        thr = CONFIG['attribution_thresholds']
        tiles = np.floor(design['points'][:, :2] / CONFIG['tile_m']).astype(int)
        rows = []
        for u in range(4):
            for c in CONFIG['fit_classes']:
                base = (design['unit'] == u) & (design['cls'] == c)
                for key in np.unique(tiles[base], axis=0):
                    cell = base & (tiles == key).all(1)
                    m = cell & ok
                    ss = src_sep[m]
                    ss = ss[np.isfinite(ss)]
                    amb = float(np.mean(self.diag_target.sigma[ids[m]] > thr['ambiguous_sigma_m'])) if m.any() else None
                    med = float(np.median(np.abs(r[m]))) if m.any() else None
                    if m.sum() < thr['min_cell_support']:
                        label = 'insufficient support'
                    elif u > 0 and len(ss) >= thr['min_source_pairs'] and np.median(ss) > thr['inconsistent_source_m']:
                        label = 'inconsistent source geometry'
                    elif amb > thr['ambiguous_map_fraction']:
                        label = 'ambiguous map surface'
                    elif med > thr['high_residual_m']:
                        label = 'unexplained high residual'
                    else:
                        label = 'consistent'
                    rows.append(dict(unit=u, cls=c, tile_center_W=(key * CONFIG['tile_m'] + CONFIG['tile_m'] / 2).tolist(), tile_parity=int((key[0] + key[1]) % 2), candidates=int(cell.sum()), support=int(m.sum()), median_abs_residual_m=med, median_signed_residual_m=float(np.median(r[m])) if m.any() else None, map_ambiguous_fraction=amb, vehicle_vs_infra_pairs=len(ss), vehicle_vs_infra_median_m=float(np.median(ss)) if len(ss) else None, label=label))
        self.results['attribution'] = rows

    def per_item_consistency(self):
        """Per scan/item: map residual (depends on D) beside infrastructure residual (D-invariant)."""
        infra = [i for i in self.pool if i.kind == 'aggregate3' and i.unit == 0]
        ip = np.concatenate([i.points for i in infra]); inn = np.concatenate([i.normals for i in infra]); icls = np.concatenate([i.cls for i in infra])
        trees = {c: (cKDTree(ip[icls == c]), ip[icls == c], inn[icls == c]) for c in CONFIG['fit_classes']}
        rows = []
        for item in self.heldout + self.pool:
            row = dict(**item.label())
            for c in CONFIG['fit_classes']:
                m = item.cls == c
                pts, nrm = item.points[m], item.normals[m]
                if item.unit > 0:
                    tree, tp, tn = trees[c]
                    d, j = tree.query(pts, workers=8)
                    nn = tn[j] * np.where(tn[j][:, 2] < 0, -1, 1)[:, None] if c == 'ground' else tn[j]
                    keep = (d < .5) & (np.abs(np.sum(nn * nrm, 1)) > np.cos(np.deg2rad(30)))
                    sep = np.sum(nn * (pts - tp[j]), 1)[keep]
                    row[f'{c}_vs_infra'] = dict(pairs=int(keep.sum()), median_signed_m=float(np.median(sep)) if keep.sum() >= 30 else None, median_abs_m=float(np.median(np.abs(sep))) if keep.sum() >= 30 else None)
                for name in ['overlap_selected', 'all_pool_balanced']:
                    x = transform(pts, self.transforms[name])
                    d, j = self.primary.tree.query(x, workers=8)
                    keep = (d < 1.0) & (np.abs(np.sum(self.primary.normals[j] * (nrm @ self.transforms[name][:3, :3].T), 1)) > np.cos(np.deg2rad(30)))
                    r = np.sum(self.primary.normals[j] * (x - self.primary.plane_points[j]), 1)[keep]
                    row[f'{c}_vs_map_{name}'] = dict(points=int(keep.sum()), median_signed_m=float(np.median(r)) if keep.sum() >= 30 else None, median_abs_m=float(np.median(np.abs(r))) if keep.sum() >= 30 else None)
            rows.append(row)
        self.results['per_item_consistency'] = dict(note='infrastructure reference = union of five stationary aggregates (take-to-take agreement checked separately); vertical signed values follow arbitrary normal orientation, use median_abs', rows=rows)

    def tilt_conflict(self):
        """Roll/pitch implied by ground-only versus vertical-only constraints, per unit."""
        self.load()
        self.map_surfaces()
        self.transforms.update({k: np.asarray(v) for k, v in json.loads((self.output / 'transforms.json').read_text()).items()})
        ref = self.transforms['all_pool_balanced']
        rows = []
        saved = CONFIG['fit_classes']
        for label, items in [('infrastructure', [i for i in self.pool if i.unit == 0])] + [(UNIT_NAMES[u], [i for i in self.pool if i.unit == u]) for u in (1, 2, 3)] + [('all units', self.pool)]:
            for cls in ['ground', 'vertical']:
                CONFIG['fit_classes'] = [cls]
                try:
                    solver = BalancedSolver(self.primary)
                    design = solver.design(items, None, np.random.default_rng(7))
                    D, trace, support = solver.fit(design, 'balanced', ref)
                    delta = D @ np.linalg.inv(ref)
                    # Tilt = rotation of the map up-axis implied by the correction.
                    up = delta[:3, :3] @ np.array([0, 0, 1.])
                    rows.append(dict(group=label, cls=cls, correspondences=support['correspondences'], scaled_eigenvalues=support['scaled_eigenvalues'], tilt_change_deg=float(np.degrees(np.arccos(np.clip(up[2], -1, 1)))), tilt_direction_xy=up[:2].tolist(), delta_rpy_deg=Rotation.from_matrix(delta[:3, :3]).as_euler('xyz', degrees=True).tolist(), delta_translation_m=delta[:3, 3].tolist()))
                    print('TILT', label, cls, np.round(rows[-1]['delta_rpy_deg'], 3), np.round(rows[-1]['delta_translation_m'], 3), flush=True)
                except RuntimeError as error:
                    rows.append(dict(group=label, cls=cls, error=str(error)))
                finally:
                    CONFIG['fit_classes'] = saved
        self.write('tilt-conflict.json', dict(note='each fit starts at the candidate and uses one surface class; ground constrains roll/pitch/z, vertical constrains x/y/yaw and roll/pitch through wall height; delta = fit @ inverse(candidate)', rows=rows))

    # ------------------------------------------------------------- run
    def run(self):
        self.load()
        self.map_surfaces()
        self.reproduce()
        self.selection()
        self.fits()
        self.stability()
        main = ['released_W_only', 'previous_shared', 'overlap_selected', 'prior_set_balanced', 'designed_set_balanced', 'designed_scans_balanced', 'all_pool_uniform_weights', 'all_pool_balanced', 'all_pool_balanced_parity1']
        self.evaluate(main)
        self.conflicts()
        self.per_item_consistency()
        self.write('results.json', self.results)
        self.write('transforms.json', {k: v.tolist() for k, v in self.transforms.items()})
        np.savez_compressed(self.output / 'eval-points.npz', **{f'{p}__{n}': a.astype(np.float32) for p, d in self.eval_points.items() for n, a in d.items()})
        print('DONE', flush=True)

    def stability(self):
        ref = self.transforms['all_pool_balanced']
        self.results['stability'] = {name: describe(D, ref) for name, D in self.transforms.items()}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=BASE / 'combined-cloud-calibration')
    parser.add_argument('--bootstrap', type=int, default=20)
    parser.add_argument('--stage', choices=['all', 'tilt'], default='all')
    args = parser.parse_args()
    job = CombinedCloudCalibration(args.output, args.bootstrap)
    job.tilt_conflict() if args.stage == 'tilt' else job.run()

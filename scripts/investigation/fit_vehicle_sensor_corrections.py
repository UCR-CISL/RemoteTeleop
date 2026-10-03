"""Test per-vehicle corrections of released CooperScene vehicle poses.

Question: can one fixed correction per vehicle make vehicle LiDAR agree with the
stationary infrastructure across all takes, and does it belong in the world
frame (``delta @ W @ N``) or in the sensor frame (``W @ N @ delta``)?

Reference: stationary infrastructure, which agrees with itself across takes to
about 1.6 cm. GSplat is not used to fit any vehicle correction. The released
poses are unchanged on disk; corrections exist only inside this experiment.

Validation is time- and take-disjoint: leave-one-take-out folds, the 30 reserved
vehicle scans, and the 123 earlier time-window frames, none of which are fitted.
Geometric consistency only, not independent ground truth.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
import json
from pathlib import Path
import sys
import zipfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.investigation.calibrate_shared_enu_map import transform, voxel
from scripts.investigation.design_combined_enu_calibration import BASE, SELECTION, UNIT_COLORS, UNIT_NAMES, MAP_COLOR, BalancedSolver, CONFIG as MAP_CONFIG, MapSurface, SourceItem, describe
from src.localization.cooperscene import pose_to_matrix
from src.localization.gaussian_map import load_gaussian_map

ARCHIVE = Path('/mnt/bcc-data/websites/data.ucr.edu-datasets-cooperscene')
SETTINGS = dict(
    declared_before_fitting=True,
    trajectory_frames_per_take_vehicle=32,
    exclusion_radius_frames=5,
    local_voxel_m=0.2,
    min_range_m=3.0,
    box_padding_m=0.2,
    normal_radius_m=0.6,
    ground_rule='world normal |nz|>0.9 and <0.3 m above the 2 m cell minimum',
    vertical_rule='world normal |nz|<0.3',
    cap_points_per_frame_class=800,
    gates_m=[1.0, 0.6, 0.4],
    frame_membership='frames with >=150 class-matched correspondences within 1.0 m under the uncorrected pose; membership fixed for the whole fit (run 1 let frames drop out, which collapsed support)',
    prior='weak zero-mean prior, sigma 1 deg rotation and 0.3 m translation, weight equal to 0.01 m^2 per unit sigma (about one frame at 0.1 m residual)',
    prior_sigma_deg=1.0,
    prior_sigma_m=0.3,
    prior_weight=0.01,
    iterations_per_gate=30,
    huber_m=0.10,
    min_frame_correspondences=150,
    normal_compatibility_deg=30.0,
    evaluation_gate_m=1.0,
    evaluation_min_points=50,
    frame_weighting='each frame equal total weight; ground and vertical equal within a frame',
    reference='union of stationary infrastructure: 5 take aggregates, the representative scan and 10 reserved infrastructure scans',
    tile_spread_m=5.0,
)


def pose_frames():
    """Released and audit-alternative W-frame poses keyed by (take, agent, frame_id)."""
    out = {}
    for name, path in [('anchor_candidate', 'raw-normalized-anchor-references.json'), ('raw_sensor_baseline', 'raw-normalized-references.json')]:
        data = json.loads((BASE / 'release-audit' / path).read_text())
        out[name] = {(int(r['run_id'].split('-')[1]), int(r['run_id'].split('-')[2]), str(r['frame_id'])): np.asarray(r['map_T_lidar']) for r in data['records']}
    return out


def read_take(job):
    """Worker: read requested frames of one take; archives are opened read-only."""
    take, requests, W = job
    path = next(ARCHIVE.glob(f'*/{take}.zip'))
    out = []
    with zipfile.ZipFile(path) as z:
        for agent, frame, role in requests:
            prefix = f'{take}/{agent}/{frame}'
            ann = yaml.safe_load(z.read(prefix + '.yaml'))
            raw = z.read(prefix + '.pcd').split(b'DATA ascii\n', 1)[1]
            local = np.fromstring(raw.decode(), sep=' ').reshape(-1, 4)[:, :3]
            N = np.array(pose_to_matrix(ann['lidar_pose']))
            world = transform(local, N)
            keep = np.isfinite(local).all(1) & (np.linalg.norm(local[:, :2], axis=1) > SETTINGS['min_range_m'])
            for box in (ann.get('vehicles') or {}).values():
                center = np.array(box['location']) + np.array(box['center'])
                B = pose_to_matrix([*center, *box['angle']])
                q = (world - center) @ B[:3, :3]
                keep &= ~np.all(np.abs(q) <= np.array(box['extent']) + SETTINGS['box_padding_m'], axis=1)
            out.append(dict(take=take, agent=agent, frame_id=str(frame), role=role, archive=str(path), local=voxel(local[keep], SETTINGS['local_voxel_m']).astype(np.float32), released=W @ N))
    return out


@dataclass
class Frame:
    take: int
    agent: int
    frame_id: str
    offset: int
    role: str
    local: np.ndarray
    poses: dict
    normals: np.ndarray = None
    cls: np.ndarray = None

    def prepare(self):
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(self.local.astype(float)))
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=SETTINGS['normal_radius_m'], max_nn=30))
        self.normals = np.asarray(pc.normals)
        R = self.poses['released'][:3, :3]
        world = transform(self.local.astype(float), self.poses['released'])
        nz = np.abs(self.normals @ R.T)[:, 2]
        cell = np.floor(world[:, :2] / 2).astype(np.int64)
        _, inverse = np.unique(cell, axis=0, return_inverse=True)
        low = np.full(inverse.max() + 1, np.inf)
        np.minimum.at(low, inverse.ravel(), world[:, 2])
        self.cls = np.full(len(world), 'other', dtype=object)
        self.cls[nz < .3] = 'vertical'
        self.cls[(nz > .9) & (world[:, 2] - low[inverse.ravel()] < .3)] = 'ground'

    @property
    def yaw_deg(self):
        R = self.poses['released']
        return float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))

    @property
    def key(self):
        return f't{self.take}_v{self.agent}_{self.frame_id}'


class InfrastructureReference:
    def __init__(self):
        with np.load(SELECTION / 'selected-clouds.npz') as s, np.load(BASE / 'shared-enu-calibration/input-clouds.npz') as a:
            parts = [s['infra_representative']] + [s[f'cal_{t}_0'] for t in range(1, 6)] + [a[k] for k in a.files if k.startswith('hold_') and k.split('_')[2] == '0']
        self.item = SourceItem('infrastructure_union', 0, 0, 'union', np.concatenate(parts))
        self.tree = cKDTree(self.item.points)


class CorrectionFitter:
    """Robust point-to-plane fit of one rigid correction for a group of frames."""

    def __init__(self, reference: InfrastructureReference, kind: str, pose_family: str):
        self.ref, self.kind, self.family = reference, kind, pose_family

    def world(self, frame, delta, points=None):
        p = frame.local.astype(float) if points is None else points
        M = frame.poses[self.family]
        return transform(transform(p, delta), M) if self.kind == 'sensor' else transform(transform(p, M), delta)

    def sample(self, frames, rng):
        out = []
        for f in frames:
            ids = []
            for c in ['ground', 'vertical']:
                sel = np.flatnonzero(f.cls == c)
                ids.append(sel if len(sel) <= SETTINGS['cap_points_per_frame_class'] else rng.choice(sel, SETTINGS['cap_points_per_frame_class'], replace=False))
            ids = np.concatenate(ids)
            out.append((f, f.local[ids].astype(float), f.normals[ids], f.cls[ids]))
        return out

    def correspond(self, f, p, nl, cls, delta, gate):
        M = f.poses[self.family]
        x = self.world(f, delta, p)
        R = (M[:3, :3] @ delta[:3, :3]) if self.kind == 'sensor' else (delta[:3, :3] @ M[:3, :3])
        d, j = self.ref.tree.query(x, workers=8)
        n = self.ref.item.normals[j]
        ok = (d < gate) & (np.abs(np.sum(n * (nl @ R.T), 1)) > np.cos(np.deg2rad(SETTINGS['normal_compatibility_deg']))) & (self.ref.item.cls[j] == cls)
        r = np.sum(n * (x - self.ref.item.points[j]), 1)
        return x, n, r, ok

    def fit(self, frames, init=None):
        if not frames:
            return None, dict(status='no frames')
        samples = self.sample(frames, np.random.default_rng(3))
        delta = np.eye(4) if init is None else init.copy()
        samples = [s for s in samples if self.correspond(*s, delta, 1.0)[3].sum() >= SETTINGS['min_frame_correspondences']]
        if len(samples) < 3:
            return None, dict(status='insufficient supported frames', used_frames=len(samples), frames=len(frames))
        P = SETTINGS['prior_weight'] * np.diag([1 / np.deg2rad(SETTINGS['prior_sigma_deg']) ** 2] * 3 + [1 / SETTINGS['prior_sigma_m'] ** 2] * 3)
        for gate in SETTINGS['gates_m']:
            for _ in range(SETTINGS['iterations_per_gate']):
                A, b, used = np.zeros((6, 6)), np.zeros(6), 0
                for f, p, nl, cls in samples:
                    x, n, r, ok = self.correspond(f, p, nl, cls, delta, gate)
                    if ok.sum() < 20:
                        continue
                    used += 1
                    if self.kind == 'sensor':
                        M = f.poses[self.family]
                        m = n[ok] @ M[:3, :3]
                        q = transform(p[ok], delta)
                        J = np.c_[np.cross(q, m), m]
                    else:
                        J = np.c_[np.cross(x[ok], n[ok]), n[ok]]
                    w = np.zeros(ok.sum())
                    for c in ['ground', 'vertical']:
                        mc = cls[ok] == c
                        if mc.any():
                            w[mc] = 1 / mc.sum()
                    w /= w.sum()
                    w *= min(1., ok.sum() / SETTINGS['min_frame_correspondences'])
                    w *= np.minimum(1, SETTINGS['huber_m'] / np.maximum(np.abs(r[ok]), 1e-12))
                    A += J.T @ (w[:, None] * J)
                    b += J.T @ (w * r[ok])
                xi = np.r_[Rotation.from_matrix(delta[:3, :3]).as_rotvec(), delta[:3, 3]]
                step = np.linalg.solve(A + P, -(b + P @ xi))
                step /= max(np.linalg.norm(step[:3]) / np.deg2rad(1), np.linalg.norm(step[3:]) / .3, 1.)
                T = np.eye(4)
                T[:3, :3] = Rotation.from_rotvec(step[:3]).as_matrix()
                T[:3, 3] = step[3:]
                delta = T @ delta
                if np.linalg.norm(step) < 1e-7:
                    break
        e = np.linalg.eigvalsh(A)
        return delta, dict(status='ok', used_frames=len(samples), frames=len(frames), rpy_deg=Rotation.from_matrix(delta[:3, :3]).as_euler('xyz', degrees=True).tolist(), translation_m=delta[:3, 3].tolist(), information_eigenvalues=e.tolist(), last_step_norm=float(np.linalg.norm(step)))


class VehicleCorrectionExperiment:
    def __init__(self, output: Path):
        self.output = output
        self.figures = output / 'figures'
        self.figures.mkdir(parents=True, exist_ok=True)
        self.results = dict(settings=SETTINGS)
        self.write('settings.json', SETTINGS)

    def write(self, name, value):
        (self.output / name).write_text(json.dumps(value, indent=1, allow_nan=False, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o)) + '\n')

    # ------------------------------------------------------------ frames
    def requests(self):
        manifest = json.loads((BASE / 'shared-enu-calibration/input-manifest.json').read_text())
        windows = json.loads((BASE / 'structure-experiment/temporal/window-manifest.json').read_text())
        req = {}
        protected = {}
        for r in manifest:
            if r['partition'] == 'heldout' and r['agent'] > 0:
                req[(r['take'], r['agent'], r['frame_id'])] = ('reserved', r['frame_offset'])
                protected.setdefault((r['take'], r['agent']), []).append(r['frame_offset'])
        for r in windows:
            req[(r['take'], r['agent'], r['frame_id'])] = ('window', r['frame_offset'])
            protected.setdefault((r['take'], r['agent']), []).append(r['frame_offset'])
        for take in range(1, 6):
            path = next(ARCHIVE.glob(f'*/{take}.zip'))
            with zipfile.ZipFile(path) as z:
                for agent in (1, 3, 2):
                    frames = sorted(n.split('/')[-1][:-5] for n in z.namelist() if n.startswith(f'{take}/{agent}/') and n.endswith('.yaml'))
                    for idx in np.unique(np.round(np.linspace(0, len(frames) - 1, SETTINGS['trajectory_frames_per_take_vehicle'])).astype(int)):
                        if any(abs(idx - o) <= SETTINGS['exclusion_radius_frames'] for o in protected.get((take, agent), [])):
                            continue
                        req.setdefault((take, agent, frames[idx]), ('trajectory', int(idx)))
        return req

    def load_frames(self):
        cache = self.output / 'frames.npz'
        alt = pose_frames()
        W = np.asarray(json.loads((BASE / 'shared-enu-calibration/calibration.json').read_text())['original_W'])
        req = self.requests()
        if not cache.exists():
            jobs = [(t, [(a, f, role) for (tt, a, f), (role, _) in req.items() if tt == t], W) for t in range(1, 6)]
            with ProcessPoolExecutor(5) as pool:
                rows = [r for chunk in pool.map(read_take, jobs) for r in chunk]
            np.savez_compressed(cache, **{f"{r['take']}|{r['agent']}|{r['frame_id']}|{r['role']}": r['local'] for r in rows}, **{f"pose|{r['take']}|{r['agent']}|{r['frame_id']}": r['released'] for r in rows})
            print('read', len(rows), 'frames', flush=True)
        self.frames = []
        with np.load(cache) as data:
            for key in data.files:
                if key.startswith('pose|'):
                    continue
                t, a, f, role = key.split('|')
                t, a = int(t), int(a)
                poses = dict(released=data[f'pose|{t}|{a}|{f}'])
                for name, table in alt.items():
                    poses[name] = table[(t, a, f)]
                frame = Frame(t, a, f, req[(t, a, f)][1], role, data[key], poses)
                frame.prepare()
                self.frames.append(frame)
        # Sanity: the anchor candidate equals the released pose at each take/vehicle ICP anchor frame (audit identity).
        diffs = [float(np.linalg.norm(f.poses['anchor_candidate'][:3, 3] - f.poses['released'][:3, 3])) for f in self.frames]
        self.results['anchor_vs_released_translation_median_m'] = float(np.median(diffs))
        self.results['frames'] = {role: int(sum(f.role == role for f in self.frames)) for role in ['trajectory', 'reserved', 'window']}
        print('frames', self.results['frames'], flush=True)

    # ------------------------------------------------------------ evaluation
    def evaluate_frame(self, fitter, frame, delta):
        p = frame.local.astype(float)
        x, n, r, ok = fitter.correspond(frame, p, frame.normals, frame.cls, np.eye(4) if delta is None else delta, SETTINGS['evaluation_gate_m'])
        out = dict(key=frame.key, take=frame.take, agent=frame.agent, role=frame.role, yaw_deg=frame.yaw_deg)
        for c in ['ground', 'vertical']:
            m = ok & (frame.cls == c)
            out[c] = dict(points=int(m.sum()), median_signed_m=float(np.median(r[m])) if m.sum() >= SETTINGS['evaluation_min_points'] else None, median_abs_m=float(np.median(np.abs(r[m]))) if m.sum() >= SETTINGS['evaluation_min_points'] else None)
        return out

    @staticmethod
    def summarize(rows):
        out = {}
        for v in [1, 2, 3, None]:
            sub = [r for r in rows if v is None or r['agent'] == v]
            g = [r['ground']['median_signed_m'] for r in sub if r['ground']['median_signed_m'] is not None]
            vv = [r['vertical']['median_abs_m'] for r in sub if r['vertical']['median_abs_m'] is not None]
            out['all' if v is None else f'vehicle {v}'] = dict(frames=len(sub), ground_frames=len(g), median_abs_ground_height_m=float(np.median(np.abs(g))) if g else None, p90_abs_ground_height_m=float(np.quantile(np.abs(g), .9)) if g else None, ground_height_spread_std_m=float(np.std(g)) if g else None, vertical_frames=len(vv), median_vertical_abs_m=float(np.median(vv)) if vv else None)
        return out

    # ------------------------------------------------------------ experiments
    def run_models(self):
        ref = self.ref = InfrastructureReference()
        traj = [f for f in self.frames if f.role == 'trajectory']
        val_all = [f for f in self.frames if f.role in ('reserved', 'window')]
        models = self.model_specs()
        self.models = models
        fits, evals = {}, {}
        for name, spec in models.items():
            fitter = CorrectionFitter(ref, spec['kind'], spec['family'])
            deltas = {}
            if spec['group'] == 'vehicle':
                for v in (1, 2, 3):
                    deltas[(None, v)], fits[f'{name}|v{v}|all_takes'] = fitter.fit([f for f in traj if f.agent == v])
            elif spec['group'] == 'take_vehicle':
                for t in range(1, 6):
                    for v in (1, 2, 3):
                        deltas[(t, v)], fits[f'{name}|v{v}|t{t}'] = fitter.fit([f for f in traj if f.agent == v and f.take == t])
            rows = []
            for f in val_all:
                d = deltas.get((None, f.agent), deltas.get((f.take, f.agent))) if spec['group'] else np.eye(4)
                if spec['group'] and d is None:
                    rows.append(dict(key=f.key, take=f.take, agent=f.agent, role=f.role, yaw_deg=f.yaw_deg, ground=dict(points=0, median_signed_m=None, median_abs_m=None), vertical=dict(points=0, median_signed_m=None, median_abs_m=None), status='no correction fitted'))
                    continue
                rows.append(self.evaluate_frame(fitter, f, d))
            evals[name] = rows
            print('MODEL', name, json.dumps(self.summarize(rows)['all']), flush=True)
            self.deltas_store = getattr(self, 'deltas_store', {})
            self.deltas_store[name] = deltas
        # Leave-one-take-out for vehicle-level corrections.
        loto = {}
        for name in ['released+vehicle_world', 'released+vehicle_sensor', 'anchor_candidate+vehicle_world', 'anchor_candidate+vehicle_sensor']:
            spec = models[name]
            fitter = CorrectionFitter(ref, spec['kind'], spec['family'])
            rows = []
            self.deltas_store[name + '|loto'] = {}
            for t in range(1, 6):
                for v in (1, 2, 3):
                    d, info = fitter.fit([f for f in traj if f.agent == v and f.take != t])
                    self.deltas_store[name + '|loto'][(t, v)] = d
                    fits[f'{name}|v{v}|without_t{t}'] = info
                    for f in [f for f in self.frames if f.take == t and f.agent == v]:
                        if d is not None:
                            rows.append(self.evaluate_frame(fitter, f, d))
            loto[name] = rows
            print('LOTO', name, json.dumps(self.summarize(rows)['all']), flush=True)
        base_rows = {}
        for family in ['released', 'anchor_candidate']:
            fitter = CorrectionFitter(ref, 'world', family)
            base_rows[family] = [self.evaluate_frame(fitter, f, None) for f in self.frames]
            print('LOTO-BASE', family, json.dumps(self.summarize(base_rows[family])['all']), flush=True)
        self.results.update(fits=fits, validation={k: dict(summary=self.summarize(v), rows=v) for k, v in evals.items()}, loto={k: dict(summary=self.summarize(v), rows=v) for k, v in loto.items()}, loto_baselines={k: dict(summary=self.summarize(v), rows=v) for k, v in base_rows.items()})

    @staticmethod
    def model_specs():
        models = {}
        for family in ['released', 'anchor_candidate', 'raw_sensor_baseline']:
            models[family] = dict(family=family, kind='world', group=None)
        for family in ['released', 'anchor_candidate']:
            for kind in ['world', 'sensor']:
                models[f'{family}+vehicle_{kind}'] = dict(family=family, kind=kind, group='vehicle')
                models[f'{family}+take_vehicle_{kind}'] = dict(family=family, kind=kind, group='take_vehicle')
        return models

    def load_saved(self):
        """Evaluation-only stage: reuse saved corrections instead of refitting."""
        self.load_frames()
        self.ref = InfrastructureReference()
        self.models = self.model_specs()
        self.results = json.loads((self.output / 'results-fit.json').read_text())
        saved = json.loads((self.output / 'corrections.json').read_text())
        self.deltas_store = {name: {(None if k.split('|')[0] == 'None' else int(k.split('|')[0]), int(k.split('|')[1])): np.asarray(d) for k, d in table.items()} for name, table in saved.items()}

    def anchor_time(self):
        """Error versus time from each take's release ICP fitting frame, stratified by distance to infrastructure."""
        anchor = {1: 481693, 2: 488685, 3: 497206, 4: 515587, 5: 524102}  # release-audit reproduction
        with np.load(SELECTION / 'selected-clouds.npz') as s:
            infra_xy = transform(s['infra_representative_pose'][:3, 3][None], np.asarray(json.loads((BASE / 'shared-enu-calibration/calibration.json').read_text())['original_W']))[0, :2]
        pos = {f.key: f.poses['released'][:3, 3] for f in self.frames}
        out = {}
        for name in ['released', 'released+vehicle_sensor', 'released+vehicle_world', 'anchor_candidate']:
            rows = []
            for r in self.results['common_support']['leave_one_take_out'][name]['rows']:
                if r['ground']['median_signed_m'] is None:
                    continue
                fid = int(r['key'].split('_')[2])
                rows.append(dict(key=r['key'], agent=r['agent'], seconds_from_icp_frame=abs(fid - anchor[r['take']]) / 10, distance_to_infrastructure_m=float(np.linalg.norm(pos[r['key']][:2] - infra_xy)), ground_abs_m=abs(r['ground']['median_signed_m']), vertical_abs_m=r['vertical']['median_abs_m']))
            strata = []
            for dlo, dhi in [(15, 30), (30, 200)]:
                for tlo, thi in [(0, 3), (3, 10), (10, 200)]:
                    m = [x for x in rows if dlo <= x['distance_to_infrastructure_m'] < dhi and tlo <= x['seconds_from_icp_frame'] < thi]
                    strata.append(dict(distance_m=[dlo, dhi], seconds=[tlo, thi], frames=len(m), ground_median_m=float(np.median([x['ground_abs_m'] for x in m])) if m else None, vertical_median_m=float(np.nanmedian([x['vertical_abs_m'] if x['vertical_abs_m'] is not None else np.nan for x in m])) if m else None))
            out[name] = dict(rows=rows, strata=strata)
        self.results['icp_frame_time'] = out
        fig, axes = plt.subplots(1, 2, figsize=(16, 5.5), layout='constrained')
        for ax, key, label in [(axes[0], 'ground_abs_m', '|frame median ground height| vs infrastructure (m)'), (axes[1], 'vertical_abs_m', 'frame median |wall separation| vs infrastructure (m)')]:
            for name, color in [('released', '#2a78d6'), ('released+vehicle_sensor', '#eb6834'), ('released+vehicle_world', '#1baf7a')]:
                rows = out[name]['rows']
                t = np.array([x['seconds_from_icp_frame'] for x in rows]); v = np.array([np.nan if x[key] is None else x[key] for x in rows])
                ax.scatter(t, v, s=10, c=color, alpha=.35, lw=0)
                bins = np.array([0, 1, 2, 3, 5, 7.5, 10, 15, 20, 30, 45, 90])
                idx = np.digitize(t, bins) - 1
                med = [np.nanmedian(v[idx == k]) if (idx == k).sum() >= 5 else np.nan for k in range(len(bins) - 1)]
                ax.plot((bins[:-1] + bins[1:]) / 2, med, color=color, lw=2.2, label=f'{name} (binned median)')
            ax.set(xscale='symlog', xlim=(0, 90), ylim=(0, 1.0), xlabel='seconds from the release ICP fitting frame of that take', ylabel=label)
            ax.axhline(.1, color='#e34948', ls='--', lw=.8)
            ax.legend(fontsize=8)
        fig.suptitle('Leave-one-take-out frames, shared point set per frame. Error grows with time from the frame where the release ICP was fitted.', fontsize=12)
        fig.savefig(self.figures / 'error-vs-icp-frame-time.png', dpi=140)
        plt.close(fig)

    def delta_for(self, name, frame, loto=False):
        spec = self.models[name]
        if not spec['group']:
            return np.eye(4)
        store = self.deltas_store[name + '|loto'] if loto else self.deltas_store[name]
        return store.get((frame.take, frame.agent)) if (loto or spec['group'] == 'take_vehicle') else store.get((None, frame.agent))

    def common_support_eval(self):
        """Score every model on one point set per frame: points within 1 m of the infrastructure
        under ANY model. Residuals are not gated afterwards; distances above 1 m are capped at 1 m."""
        ref = self.ref
        groups = {
            'reserved_and_window': ([f for f in self.frames if f.role in ('reserved', 'window')], [n for n in self.models], False),
            'reserved_only': ([f for f in self.frames if f.role == 'reserved'], [n for n in self.models], False),
            'leave_one_take_out': (self.frames, ['released', 'anchor_candidate', 'released+vehicle_world', 'released+vehicle_sensor', 'anchor_candidate+vehicle_world', 'anchor_candidate+vehicle_sensor'], True),
        }
        out = {}
        for gname, (frames, names, loto) in groups.items():
            per = {n: [] for n in names}
            for f in frames:
                p = f.local.astype(float)
                fitted = {}
                for n in names:
                    d = self.delta_for(n, f, loto and bool(self.models[n]['group']))
                    if d is None:
                        break
                    spec = self.models[n]
                    fitted[n] = CorrectionFitter(ref, spec['kind'], spec['family']).world(f, d, p)
                if len(fitted) < len(names):
                    continue  # a model has no correction for this frame; drop the frame for all models
                support = np.zeros(len(p), bool)
                cache = {}
                for n, x in fitted.items():
                    dist, j = ref.tree.query(x, workers=8)
                    cache[n] = (x, dist, j)
                    support |= (dist < 1.0)
                support &= np.isin(f.cls, ['ground', 'vertical'])
                if support.sum() < 100:
                    continue
                for n, (x, dist, j) in cache.items():
                    r = np.sum(ref.item.normals[j] * (x - ref.item.points[j]), 1)
                    r = np.where(dist < 1.0, r, np.where(r >= 0, 1.0, -1.0))  # capped penalty for points that left the surface
                    row = dict(key=f.key, take=f.take, agent=f.agent, role=f.role, yaw_deg=f.yaw_deg)
                    for c in ['ground', 'vertical']:
                        m = support & (f.cls == c)
                        row[c] = dict(points=int(m.sum()), median_signed_m=float(np.median(r[m])) if m.sum() >= 30 else None, median_abs_m=float(np.median(np.abs(r[m]))) if m.sum() >= 30 else None, fraction_within_010m=float(np.mean(np.abs(r[m]) < .1)) if m.sum() >= 30 else None)
                    per[n].append(row)
            out[gname] = {n: dict(summary=self.summarize(rows), rows=rows) for n, rows in per.items()}
            for n in names:
                print('COMMON', gname, n, json.dumps(out[gname][n]['summary']['all']), flush=True)
        self.results['common_support'] = out

    def consistency_of_corrections(self):
        """Are per-take sensor-frame corrections the same physical offset across takes?"""
        rows = []
        for name in ['released+take_vehicle_sensor', 'released+take_vehicle_world', 'anchor_candidate+take_vehicle_sensor']:
            for v in (1, 2, 3):
                ds = {t: self.deltas_store[name].get((t, v)) for t in range(1, 6)}
                ds = {t: d for t, d in ds.items() if d is not None}
                if len(ds) < 2:
                    continue
                rpy = np.array([Rotation.from_matrix(d[:3, :3]).as_euler('xyz', degrees=True) for d in ds.values()])
                tr = np.array([d[:3, 3] for d in ds.values()])
                rows.append(dict(model=name, vehicle=v, takes=list(ds), rpy_deg=rpy.tolist(), translation_m=tr.tolist(), rpy_std_deg=rpy.std(0).tolist(), translation_std_m=tr.std(0).tolist()))
        self.results['per_take_correction_spread'] = rows

    def tile_spread(self):
        """Map-free: 5 m tiles, spread of per-sensor median ground height on validation frames."""
        out = {}
        val = [f for f in self.frames if f.role in ('reserved', 'window')]
        infra = self.ref.item
        ig = infra.points[infra.cls == 'ground']
        for name in ['released', 'anchor_candidate', 'released+vehicle_sensor', 'released+vehicle_world', 'anchor_candidate+vehicle_sensor', 'released+take_vehicle_world', 'released+take_vehicle_sensor']:
            spec = self.models[name]
            fitter = CorrectionFitter(self.ref, spec['kind'], spec['family'])
            pts = [np.c_[ig, np.zeros(len(ig))]]
            for f in val:
                d = np.eye(4) if not spec['group'] else self.deltas_store[name].get((None, f.agent), self.deltas_store[name].get((f.take, f.agent)))
                if d is None:
                    continue
                g = f.cls == 'ground'
                pts.append(np.c_[fitter.world(f, d)[g], np.full(g.sum(), f.agent)])
            a = np.concatenate(pts)
            tile = np.floor(a[:, :2] / SETTINGS['tile_spread_m']).astype(int)
            keys, inverse = np.unique(tile, axis=0, return_inverse=True)
            spreads = []
            for k in range(len(keys)):
                m = inverse.ravel() == k
                med = [np.median(a[m & (a[:, 3] == u), 2]) for u in range(4) if (m & (a[:, 3] == u)).sum() >= 30]
                if len(med) >= 2:
                    spreads.append(max(med) - min(med))
            sp = np.array(spreads)
            out[name] = dict(tiles=len(sp), median_spread_m=float(np.median(sp)), p90_spread_m=float(np.quantile(sp, .9)), fraction_over_010m=float(np.mean(sp > .1)), fraction_over_030m=float(np.mean(sp > .3)))
            print('SPREAD', name, out[name], flush=True)
        self.results['tile_spread'] = out

    def map_stage(self):
        """Refit the shared map D with released versus corrected vehicles; compare stability."""
        g = load_gaussian_map(json.loads((BASE / 'shared-enu-calibration/calibration.json').read_text())['map_path'])
        p = MAP_CONFIG['primary_target']
        target = MapSurface(g, p['min_opacity']).target(p['plane_point'], p['max_local_sigma_m'])
        traj = [f for f in self.frames if f.role == 'trajectory']
        val = [f for f in self.frames if f.role in ('reserved', 'window')]
        out = {}
        for name in ['released', 'released+vehicle_sensor', 'released+vehicle_world']:
            spec = self.models[name]
            fitter = CorrectionFitter(self.ref, spec['kind'], spec['family'])
            def cloud(frames, v):
                parts = []
                for f in frames:
                    d = np.eye(4) if not spec['group'] else self.deltas_store[name].get((None, f.agent))
                    if f.agent == v and d is not None:
                        parts.append(fitter.world(f, d))
                return np.concatenate(parts)
            if spec['group'] and any(self.deltas_store[name].get((None, v)) is None for v in (1, 2, 3)):
                out[name] = dict(status='skipped: a vehicle correction could not be fitted')
                continue
            items = [SourceItem('infrastructure', 0, 0, 'union', self.ref.item.points)] + [SourceItem(f'vehicle{v}', v, 0, 'trajectory', voxel(cloud(traj, v), .25)) for v in (1, 2, 3)]
            solver = BalancedSolver(target)
            Ds = {}
            for label, subset in [('all', items)] + [(f'leave_out_{UNIT_NAMES[u]}', [i for i in items if i.unit != u]) for u in range(4)]:
                D, trace, support = solver.fit(solver.design(subset, None, np.random.default_rng(7)))
                Ds[label] = D
            stab = {k: describe(D, Ds['all']) for k, D in Ds.items()}
            # Held-out vehicle frames against the map under this family's own D.
            res = []
            for f in val:
                d = np.eye(4) if not spec['group'] else self.deltas_store[name].get((None, f.agent))
                if d is None:
                    continue
                x = transform(fitter.world(f, d), Ds['all'])
                dd, j = target.tree.query(x, workers=8)
                for c in ['ground', 'vertical']:
                    m = (dd < 1.0) & (f.cls == c)
                    if m.sum() >= SETTINGS['evaluation_min_points']:
                        r = np.sum(target.normals[j[m]] * (x[m] - target.plane_points[j[m]]), 1)
                        res.append(dict(key=f.key, agent=f.agent, cls=c, median_signed_m=float(np.median(r)), median_abs_m=float(np.median(np.abs(r)))))
            summary = {c: dict(frames=len([r for r in res if r['cls'] == c]), median_abs_of_frame_signed_m=float(np.median([abs(r['median_signed_m']) for r in res if r['cls'] == c])), median_frame_abs_m=float(np.median([r['median_abs_m'] for r in res if r['cls'] == c]))) for c in ['ground', 'vertical']}
            out[name] = dict(D=Ds['all'].tolist(), stability=stab, max_leave_one_out_displacement_m=float(max(s['max_displacement_120m_box_m'] for k, s in stab.items() if k != 'all')), heldout_vs_map=summary)
            print('MAP', name, out[name]['max_leave_one_out_displacement_m'], summary, flush=True)
        self.results['map_stage'] = out

    # ------------------------------------------------------------ figures
    def figures_heading(self):
        names = ['released', 'released+vehicle_world', 'released+vehicle_sensor', 'released+take_vehicle_world']
        fig, axes = plt.subplots(len(names), 3, figsize=(18, 4 * len(names)), layout='constrained', sharey=True)
        for row, name in enumerate(names):
            rows = self.results['validation'][name]['rows']
            for col, v in enumerate((1, 2, 3)):
                ax = axes[row, col]
                for role, marker in [('reserved', 'o'), ('window', '.')]:
                    sub = [r for r in rows if r['agent'] == v and r['role'] == role and r['ground']['median_signed_m'] is not None]
                    ax.scatter([r['yaw_deg'] for r in sub], [r['ground']['median_signed_m'] for r in sub], c=UNIT_COLORS[v], marker=marker, s=40 if marker == 'o' else 30, label=f'{role} frames')
                ax.axhline(0, color='#0b0b0b', lw=.6)
                ax.axhspan(-.1, .1, color='#e4e3df', zorder=0)
                ax.set(xlim=(-180, 180), ylim=(-.6, .6), xlabel='released heading (deg)', ylabel='ground height vs infrastructure (m)', title=f'{name}: {UNIT_NAMES[v]}')
                if row == 0 and col == 0:
                    ax.legend(fontsize=8)
        fig.suptitle('Validation frames only (never fitted). Gray band: +/-0.1 m. Heading-dependent error is the signature of a sensor-frame offset.', fontsize=12)
        fig.savefig(self.figures / 'ground-height-vs-heading.png', dpi=140)
        plt.close(fig)

    def figures_summary(self):
        names = list(self.results['validation'])
        fig, axes = plt.subplots(1, 2, figsize=(18, 0.38 * len(names) + 2.5), layout='constrained', sharey=True)
        y = np.arange(len(names))
        for ax, key, label in [(axes[0], 'median_abs_ground_height_m', 'median |frame ground height| vs infrastructure (m)'), (axes[1], 'median_vertical_abs_m', 'median frame |vertical separation| vs infrastructure (m)')]:
            for k, v in enumerate((1, 2, 3)):
                vals = [self.results['validation'][n]['summary'][f'vehicle {v}'][key] for n in names]
                ax.scatter([np.nan if x is None else x for x in vals], y + (k - 1) * .2, c=UNIT_COLORS[v], s=30, label=UNIT_NAMES[v])
            for n, name in enumerate(names):
                lo = self.results['loto'].get(name)
                if lo:
                    ax.scatter(lo['summary']['all'][key], n, marker='|', c='#0b0b0b', s=200)
            ax.axvline(.1, color='#e34948', ls='--', lw=.8)
            ax.set(xlabel=label)
        axes[0].set_yticks(y, names, fontsize=8)
        axes[0].invert_yaxis()
        axes[0].legend(fontsize=8)
        fig.suptitle('Reserved scans and time-window frames (never fitted). Black tick: all vehicles, leave-one-take-out. Red line: 0.1 m', fontsize=12)
        fig.savefig(self.figures / 'model-comparison.png', dpi=140)
        plt.close(fig)

    def figures_crops(self):
        """NW upper facade and central road, validation frames, released versus corrected."""
        rois = {r['id']: r for r in json.loads((ROOT / 'scripts/investigation/cooperscene_structure_rois.json').read_text())['rois']}
        names = ['released', 'released+vehicle_sensor', 'anchor_candidate+vehicle_sensor', 'released+take_vehicle_world']
        ids = ['NW_upper_facade', 'SW_boundary_wall', 'road_central', 'road_south']
        val = [f for f in self.frames if f.role in ('reserved', 'window')]
        fig, axes = plt.subplots(len(names), len(ids), figsize=(5 * len(ids), 3.6 * len(names)), layout='constrained', squeeze=False)
        for col, rid in enumerate(ids):
            roi = rois[rid]
            n = np.asarray(roi['normal'], float); n /= np.linalg.norm(n)
            u = np.asarray(roi['tangent_u'], float); u -= n * (u @ n); u /= np.linalg.norm(u)
            B = np.c_[u, np.cross(n, u), n]
            hw = np.asarray(roi['half_widths_uvn'])
            anchor = np.asarray(roi['anchor'])
            for row, name in enumerate(names):
                ax = axes[row, col]
                spec = self.models[name]
                fitter = CorrectionFitter(self.ref, spec['kind'], spec['family'])
                q = (self.ref.item.points - anchor) @ B
                q = q[(np.abs(q[:, 0]) <= hw[0] + 1) & (np.abs(q[:, 1]) <= hw[1]) & (np.abs(q[:, 2]) <= 1.5)]
                infra_q = q
                for v in (1, 2, 3):
                    parts = []
                    for f in val:
                        if f.agent != v:
                            continue
                        d = np.eye(4) if not spec['group'] else self.deltas_store[name].get((None, v), self.deltas_store[name].get((f.take, v)))
                        if d is None:
                            continue
                        parts.append(fitter.world(f, d))
                    if not parts:
                        continue
                    q = (np.concatenate(parts) - anchor) @ B
                    q = q[(np.abs(q[:, 0]) <= hw[0] + 1) & (np.abs(q[:, 1]) <= hw[1]) & (np.abs(q[:, 2]) <= 1.5)]
                    ax.scatter(q[:, 0], q[:, 2], s=4, c=UNIT_COLORS[v], alpha=.6, label=UNIT_NAMES[v])
                ax.scatter(infra_q[:, 0], infra_q[:, 2], s=9, c=UNIT_COLORS[0], edgecolor='#0b0b0b', lw=.3, label=UNIT_NAMES[0] + ' (reference, drawn on top)')
                ax.set(xlim=(-hw[0] - 1, hw[0] + 1), ylim=(-1.5, 1.5), xlabel='along surface (m)', ylabel='height (m)' if roi['kind'] == 'road' else 'normal offset (m)', title=f'{rid}\n{name}')
                if row == 0 and col == 0:
                    ax.legend(fontsize=7, markerscale=2)
        fig.suptitle('Fixed structure patches, validation frames only (W frame, no map). Infrastructure (blue, outlined) is the reference. Frames whose take has no fitted correction are omitted in that row.', fontsize=12)
        fig.savefig(self.figures / 'structure-crops.png', dpi=140)
        plt.close(fig)

    def run(self):
        self.load_frames()
        self.run_models()
        self.consistency_of_corrections()
        self.write('corrections.json', {name: {f'{t}|{v}': d.tolist() for (t, v), d in ds.items() if d is not None} for name, ds in self.deltas_store.items()})
        self.write('results-fit.json', self.results)
        self.common_support_eval()
        self.write('results-fit.json', self.results)
        self.anchor_time()
        self.tile_spread()
        self.map_stage()
        self.write('results.json', self.results)
        self.figures_heading()
        self.figures_summary()
        self.figures_crops()
        print('DONE', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=BASE / 'vehicle-sensor-corrections')
    parser.add_argument('--stage', choices=['all', 'evaluate'], default='all')
    args = parser.parse_args()
    job = VehicleCorrectionExperiment(args.output)
    if args.stage == 'evaluate':
        job.load_saved()
        job.anchor_time()
        job.write('results.json', job.results)
        job.tile_spread()
        job.map_stage()
        job.write('results.json', job.results)
        job.figures_heading()
        job.figures_summary()
        job.figures_crops()
        print('DONE', flush=True)
    else:
        job.run()

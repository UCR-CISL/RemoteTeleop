"""Smooth time-varying refinement of CooperScene vehicle trajectories.

Base pose: the zero-parameter timing-anchored pose from
``diagnose_vehicle_pose_timing.py`` (released pose interpolated to LiDAR time,
equal to the released pose at the release ICP frame). On top of it, each take
and vehicle gets one smooth correction over time: a uniform cubic B-spline of a
rotation about the vehicle's own position plus a translation,

    pose(t) = M(xi(t), c(t)) @ base(t),   M = [R(w), c - R(w) c + tau],

fitted to stationary infrastructure and to the other vehicles at the same frame
ID. Knot spacing is chosen by blocked cross-validation over time within each
take. Final scoring uses reserved scans and time-window frames that no fit sees.

Released files are never modified. Geometric consistency, not ground truth.
"""
from __future__ import annotations

import argparse
import multiprocessing
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.investigation.calibrate_shared_enu_map import transform, voxel
from scripts.investigation.design_combined_enu_calibration import BASE, UNIT_NAMES, BalancedSolver, CONFIG as MAP_CONFIG, MapSurface, SourceItem, describe
from scripts.investigation.diagnose_vehicle_pose_timing import ICP_FRAME, PoseTimeline
from scripts.investigation.fit_vehicle_sensor_corrections import Frame, InfrastructureReference, read_take
from src.localization.gaussian_map import load_gaussian_map

TIMELINES = BASE / 'vehicle-pose-timing/timelines.json'
FIT_WORKERS = 2  # five parallel takes exhausted memory (about 12 GB); two keep peak near 5-7 GB
SETTINGS = dict(
    declared_before_fitting=True,
    base_pose='timing_anchor: released pose interpolated to LiDAR time, anchored to equal the released pose at the release ICP frame',
    correction='per take and vehicle, uniform cubic B-spline over time of [rotation vector about the vehicle position, translation]; left-multiplied in world axes',
    training_frames='every 5th frame ID per take (2 Hz) for vehicles 1-3, excluding frames within 5 frame IDs of any reserved or time-window frame of that vehicle',
    validation_frames='30 reserved scans and 123 time-window frames; never used by any fit',
    knot_spacings_s=[None, 10.0, 5.0, 2.5],
    knot_spacing_note='None = one constant correction per take and vehicle',
    cross_validation='5 contiguous time blocks per take; fit on 4, score the held-out block; spacing chosen by mean held-out score over all takes (ground height + wall separation medians)',
    constraints='point-to-plane to stationary infrastructure (class-matched); vehicle-to-vehicle point-to-plane at identical frame IDs',
    vehicle_pair_weight=1.0,
    points_per_frame_class=400,
    gates_m=[1.0, 0.6, 0.4],
    iterations_per_gate=12,
    huber_m=0.10,
    min_correspondences=150,
    prior=dict(weight=0.01, sigma_deg=1.0, sigma_m=0.3),
    smoothness=dict(weight=0.01, sigma_deg=0.2, sigma_m=0.05, penalty='second difference of control points'),
    scoring='shared point set per frame (points within 1 m of infrastructure under any compared pose family), residuals capped at 1 m; vehicle pairs at identical frame IDs scored by median |point-to-plane| within 1 m',
    adoption_rule='candidate reference only if, versus timing_anchor and released, median ground height and wall separation both improve by >=10% on held-out CV blocks AND on reserved/window frames, and vehicle-pair separation does not worsen',
)


class SplineBasis:
    """Uniform cubic B-spline (or a single constant) over a time interval."""

    def __init__(self, t0, t1, spacing):
        self.t0, self.spacing = t0, spacing
        self.count = 1 if spacing is None else int(np.ceil((t1 - t0) / spacing)) + 3

    def weights(self, t):
        if self.spacing is None:
            return np.array([0]), np.array([1.])
        u = (t - self.t0) / self.spacing
        i = int(np.clip(np.floor(u), 0, self.count - 4))
        s = u - i
        w = np.array([(1 - s) ** 3, 3 * s ** 3 - 6 * s ** 2 + 4, -3 * s ** 3 + 3 * s ** 2 + 3 * s + 1, s ** 3]) / 6
        return np.arange(i, i + 4), w

    def second_difference(self):
        if self.count < 3:
            return np.zeros((0, self.count))
        D = np.zeros((self.count - 2, self.count))
        for k in range(self.count - 2):
            D[k, k:k + 3] = [1, -2, 1]
        return D


def correction_matrix(xi, center):
    R = Rotation.from_rotvec(xi[:3]).as_matrix()
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = center - R @ center + xi[3:]
    return M


class TakeRefiner:
    """Joint smooth corrections for the three vehicles of one take."""

    def __init__(self, frames, ref, spacing, use_pairs=True):
        self.frames, self.ref, self.spacing, self.use_pairs = frames, ref, spacing, use_pairs
        times = [f.seconds for f in frames]
        self.basis = {v: SplineBasis(min(times) - 1e-3, max(times) + 1e-3, spacing) for v in (1, 2, 3)}
        self.offset = {}
        n = 0
        for v in (1, 2, 3):
            self.offset[v] = n
            n += 6 * self.basis[v].count
        self.n = n
        self.C = np.zeros(n)
        self._pair_cache = {}
        rng = np.random.default_rng(5)
        self.sample = {}
        for f in frames:
            ids = []
            for c in ['ground', 'vertical']:
                sel = np.flatnonzero(f.cls == c)
                ids.append(sel if len(sel) <= SETTINGS['points_per_frame_class'] else rng.choice(sel, SETTINGS['points_per_frame_class'], replace=False))
            ids = np.concatenate(ids)
            self.sample[f.key] = (f.local[ids].astype(float), f.normals[ids], f.cls[ids])

    # -- parameter plumbing
    def xi(self, f):
        idx, w = self.basis[f.agent].weights(f.seconds)
        block = self.C[self.offset[f.agent]:self.offset[f.agent] + 6 * self.basis[f.agent].count].reshape(-1, 6)
        return w @ block[idx]

    def pose(self, f):
        base = f.poses['timing_anchor']
        return correction_matrix(self.xi(f), base[:3, 3]) @ base

    def scatter(self, f, J):
        """Map per-point 6-column Jacobian to global parameter columns."""
        idx, w = self.basis[f.agent].weights(f.seconds)
        cols = (self.offset[f.agent] + 6 * idx[:, None] + np.arange(6)[None]).ravel()
        return cols, (J[:, None, :] * w[None, :, None]).reshape(len(J), -1)

    # -- fitting
    def fit(self):
        prior = SETTINGS['prior']; smooth = SETTINGS['smoothness']
        sig = lambda s: np.array([np.deg2rad(s['sigma_deg'])] * 3 + [s['sigma_m']] * 3)
        P = np.zeros((self.n, self.n))
        for v in (1, 2, 3):
            o, m = self.offset[v], self.basis[v].count
            P[o:o + 6 * m, o:o + 6 * m] += prior['weight'] * np.diag(np.tile(1 / sig(prior) ** 2, m))
            D = self.basis[v].second_difference()
            if len(D):
                S = np.kron(D, np.diag(1 / sig(smooth)))
                P[o:o + 6 * m, o:o + 6 * m] += smooth['weight'] * S.T @ S
        members = [f for f in self.frames if self.infra_terms(f, 1.0, count_only=True) >= SETTINGS['min_correspondences']]
        by_id = {}
        for f in self.frames:
            by_id.setdefault(f.frame_id, {})[f.agent] = f
        pairs = []
        if self.use_pairs:
            for group in by_id.values():
                for a in group:
                    for b in group:
                        if a != b and self.pair_terms(group[a], group[b], 1.0, count_only=True) >= SETTINGS['min_correspondences']:
                            pairs.append((group[a], group[b]))
        self.support = dict(infrastructure_frames={v: sum(f.agent == v for f in members) for v in (1, 2, 3)}, vehicle_pairs=len(pairs))
        if not members:
            raise RuntimeError('no infrastructure-supported frames; absolute frame unconstrained')
        for gate in SETTINGS['gates_m']:
            for _ in range(SETTINGS['iterations_per_gate']):
                H = P.copy()
                g = P @ self.C
                for f in members:
                    out = self.infra_terms(f, gate)
                    if out is None:
                        continue
                    cols, J, r, w = out
                    H[np.ix_(cols, cols)] += J.T @ (w[:, None] * J)
                    g[cols] += J.T @ (w * r)
                for a, b in pairs:
                    out = self.pair_terms(a, b, gate)
                    if out is None:
                        continue
                    cols, J, r, w = out
                    H[np.ix_(cols, cols)] += J.T @ (w[:, None] * J)
                    g[cols] += J.T @ (w * r)
                step = np.linalg.solve(H, -g)
                scale = max(np.abs(step.reshape(-1, 6)[:, :3]).max() / np.deg2rad(.5), np.abs(step.reshape(-1, 6)[:, 3:]).max() / .2, 1.)
                self.C += step / scale
                if np.abs(step).max() < 1e-7:
                    break
        return self

    def weights(self, cls_, r, ok):
        w = np.zeros(ok.sum())
        for c in ['ground', 'vertical']:
            m = cls_[ok] == c
            if m.any():
                w[m] = 1 / m.sum()
        w /= max(w.sum(), 1e-12)
        w *= min(1., ok.sum() / SETTINGS['min_correspondences'])
        return w * np.minimum(1, SETTINGS['huber_m'] / np.maximum(np.abs(r[ok]), 1e-12))

    def infra_terms(self, f, gate, count_only=False):
        p, nl, cls_ = self.sample[f.key]
        T = self.pose(f)
        x = transform(p, T)
        d, j = self.ref.tree.query(x, workers=4)
        n = self.ref.item.normals[j]
        ok = (d < gate) & (self.ref.item.cls[j] == cls_) & (np.abs(np.sum(n * (nl @ T[:3, :3].T), 1)) > np.cos(np.deg2rad(30)))
        if count_only:
            return int(ok.sum())
        if ok.sum() < 20:
            return None
        r = np.sum(n * (x - self.ref.item.points[j]), 1)
        c = T[:3, 3]
        J6 = np.c_[np.cross(x[ok] - c, n[ok]), n[ok]]
        cols, J = self.scatter(f, J6)
        return cols, J, r[ok], self.weights(cls_, r, ok)

    def pair_terms(self, a, b, gate, count_only=False):
        """Points of vehicle a against surfaces of vehicle b at the same frame ID.

        b's base-pose cloud is indexed once; queries are mapped into it through the
        inverse of b's current correction, so no tree is rebuilt per iteration."""
        if b.key not in self._pair_cache:
            world_b = transform(b.local.astype(float), b.poses['timing_anchor'])
            keep = np.isin(b.cls, ['ground', 'vertical'])
            self._pair_cache[b.key] = (cKDTree(world_b[keep]), world_b[keep], (b.normals @ b.poses['timing_anchor'][:3, :3].T)[keep], b.cls[keep])
        tree, qb, nb, cb = self._pair_cache[b.key]
        p, nl, cls_ = self.sample[a.key]
        Ta, Tb = self.pose(a), self.pose(b)
        Mb = Tb @ np.linalg.inv(b.poses['timing_anchor'])
        x = transform(p, Ta)
        d, j = tree.query(transform(x, np.linalg.inv(Mb)), workers=4)
        q = transform(qb[j], Mb)
        n = nb[j] @ Mb[:3, :3].T
        ok = (d < gate) & (cb[j] == cls_) & (np.abs(np.sum(n * (nl @ Ta[:3, :3].T), 1)) > np.cos(np.deg2rad(30)))
        if count_only:
            return int(ok.sum())
        if ok.sum() < 20:
            return None
        r = np.sum(n * (x - q), 1)
        ca, cb_ = Ta[:3, 3], Tb[:3, 3]
        cols_a, Ja = self.scatter(a, np.c_[np.cross(x[ok] - ca, n[ok]), n[ok]])
        cols_b, Jb = self.scatter(b, -np.c_[np.cross(q[ok] - cb_, n[ok]), n[ok]])
        w = self.weights(cls_, r, ok) * SETTINGS['vehicle_pair_weight']
        return np.r_[cols_a, cols_b], np.c_[Ja, Jb], r[ok], w

    def describe_controls(self):
        out = {}
        for v in (1, 2, 3):
            o, m = self.offset[v], self.basis[v].count
            C = self.C[o:o + 6 * m].reshape(-1, 6)
            out[v] = dict(control_points=m, max_rotation_deg=float(np.degrees(np.linalg.norm(C[:, :3], axis=1)).max()), max_translation_m=float(np.linalg.norm(C[:, 3:], axis=1).max()))
        return out


class Scorer:
    """Shared point set per frame against infrastructure; vehicle pairs at identical frame IDs."""

    def __init__(self, ref):
        self.ref = ref

    def infra(self, frames, families):
        rows = {n: [] for n in families}
        for f in frames:
            res = {}
            for n, pose_of in families.items():
                T = pose_of(f)
                if T is None:
                    break
                x = transform(f.local.astype(float), T)
                d, j = self.ref.tree.query(x, workers=4)
                res[n] = (d, np.sum(self.ref.item.normals[j] * (x - self.ref.item.points[j]), 1))
            if len(res) < len(families):
                continue
            support = np.isin(f.cls, ['ground', 'vertical']) & np.any([d < 1. for d, _ in res.values()], axis=0)
            if support.sum() < 100:
                continue
            for n, (d, r) in res.items():
                r = np.where(d < 1., r, np.where(r >= 0, 1., -1.))
                row = dict(key=f.key, take=f.take, agent=f.agent, role=f.role, seconds=f.seconds)
                for c in ['ground', 'vertical']:
                    m = support & (f.cls == c)
                    row[c] = float(np.median(np.abs(r[m]))) if m.sum() >= 30 else None
                    row[c + '_signed'] = float(np.median(r[m])) if m.sum() >= 30 else None
                rows[n].append(row)
        return rows

    @staticmethod
    def pairs(frames, families):
        by_id = {}
        for f in frames:
            by_id.setdefault((f.take, f.frame_id), {})[f.agent] = f
        rows = {n: [] for n in families}
        for group in by_id.values():
            for a in group:
                for b in group:
                    if a >= b:
                        continue
                    fa, fb = group[a], group[b]
                    for n, pose_of in families.items():
                        Ta, Tb = pose_of(fa), pose_of(fb)
                        if Ta is None or Tb is None:
                            continue
                        xa = transform(fa.local[np.isin(fa.cls, ['ground', 'vertical'])].astype(float), Ta)
                        keep = np.isin(fb.cls, ['ground', 'vertical'])
                        xb = transform(fb.local[keep].astype(float), Tb)
                        nb = (fb.normals @ Tb[:3, :3].T)[keep]
                        d, j = cKDTree(xb).query(xa, workers=4)
                        ok = d < 1.
                        if ok.sum() < 100:
                            continue
                        rows[n].append(dict(take=fa.take, frame_id=fa.frame_id, pair=f'{a}-{b}', role=fa.role, points=int(ok.sum()), median_abs_m=float(np.median(np.abs(np.sum(nb[j[ok]] * (xa[ok] - xb[j[ok]]), 1))))))
        return rows

    @staticmethod
    def summary(rows):
        out = {}
        for label, keep in [('all', lambda r: True)] + [(f'vehicle {v}', lambda r, v=v: r['agent'] == v) for v in (1, 2, 3)]:
            sub = [r for r in rows if keep(r)]
            g = [abs(r['ground_signed']) for r in sub if r['ground_signed'] is not None]
            w = [r['vertical'] for r in sub if r['vertical'] is not None]
            out[label] = dict(frames=len(sub), ground_height_median_m=float(np.median(g)) if g else None, wall_median_m=float(np.median(w)) if w else None, ground_height_p90_m=float(np.quantile(g, .9)) if g else None, wall_p90_m=float(np.quantile(w, .9)) if w else None)
        return out


def load_take_frames(take, cache, timelines):
    frames = []
    with np.load(cache) as data:
        for key in data.files:
            if key.startswith('pose|'):
                continue
            t, a, fid, role = key.split('|')
            if int(t) != take:
                continue
            f = Frame(int(t), int(a), fid, 0, role, data[key], dict(released=data[f'pose|{t}|{a}|{fid}']))
            f.prepare()
            tl = timelines[(f.take, f.agent)]
            f.poses['timing_anchor'] = tl.timing_anchor(f.frame_id, ICP_FRAME[f.take])
            f.seconds = tl.seconds_from(f.frame_id, ICP_FRAME[f.take])
            frames.append(f)
    return frames


def run_take(job):
    """Worker: cross-validation, final fits and validation scoring for one take."""
    take, cache = job
    raw = json.loads(TIMELINES.read_text())
    timelines = {(int(t), int(a)): PoseTimeline(r) for t, d in raw.items() for a, r in d.items()}
    frames = load_take_frames(take, cache, timelines)
    ref = InfrastructureReference()
    scorer = Scorer(ref)
    train = [f for f in frames if f.role == 'trajectory']
    val = [f for f in frames if f.role in ('reserved', 'window')]
    log = []
    # Blocked cross-validation over time.
    edges = np.quantile([f.seconds for f in train], np.linspace(0, 1, 6))
    block = {f.key: int(np.clip(np.searchsorted(edges, f.seconds, side='right') - 1, 0, 4)) for f in train}
    cv_rows = []
    for spacing in SETTINGS['knot_spacings_s']:
        for k in range(5):
            fit_frames = [f for f in train if block[f.key] != k]
            held = [f for f in train if block[f.key] == k]
            try:
                model = TakeRefiner(fit_frames, ref, spacing).fit()
            except RuntimeError as error:
                log.append(f'take {take} spacing {spacing} fold {k}: {error}')
                continue
            fams = {'released': lambda f: f.poses['released'], 'timing_anchor': lambda f: f.poses['timing_anchor'], 'refined': model.pose}
            infra = scorer.infra(held, fams)
            pairs = scorer.pairs(held, fams)
            for n in fams:
                for r in infra[n]:
                    cv_rows.append(dict(kind='infrastructure', spacing=spacing, fold=k, family=n, **r))
                for r in pairs[n]:
                    cv_rows.append(dict(kind='pair', spacing=spacing, fold=k, family=n, **r))
            log.append(f'take {take} spacing {spacing} fold {k}: support {model.support}')
    # Final fits on all training frames, joint and infrastructure-only.
    finals, poses, controls = {}, {}, {}
    for spacing in SETTINGS['knot_spacings_s']:
        for use_pairs in (True, False):
            name = f"spline_{'const' if spacing is None else f'{spacing:g}s'}_{'joint' if use_pairs else 'infra_only'}"
            try:
                model = TakeRefiner(train, ref, spacing, use_pairs).fit()
            except RuntimeError as error:
                log.append(f'take {take} {name}: {error}')
                continue
            finals[name] = model
            controls[name] = dict(support=model.support, controls=model.describe_controls(), C=model.C.tolist(), offsets=model.offset, counts={v: model.basis[v].count for v in (1, 2, 3)}, spacing=spacing, t0={v: model.basis[v].t0 for v in (1, 2, 3)})
            poses[name] = {f.key: model.pose(f).tolist() for f in frames}
    fams = {'released': lambda f: f.poses['released'], 'timing_anchor': lambda f: f.poses['timing_anchor']}
    fams.update({n: (lambda f, m=m: m.pose(f)) for n, m in finals.items()})
    val_infra = scorer.infra(val, fams)
    val_pairs = scorer.pairs(val, fams)
    return dict(take=take, cv_rows=cv_rows, val_infra=val_infra, val_pairs=val_pairs, poses=poses, controls=controls, log=log, frames=[dict(key=f.key, agent=f.agent, role=f.role, seconds=f.seconds) for f in frames])


class TrajectoryRefinementExperiment:
    def __init__(self, output: Path):
        self.output = output
        self.figures = output / 'figures'
        self.figures.mkdir(parents=True, exist_ok=True)
        self.results = dict(settings=SETTINGS)
        self.write('settings.json', SETTINGS)

    def write(self, name, value):
        (self.output / name).write_text(json.dumps(value, indent=1, allow_nan=False, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o)) + '\n')

    def requests(self):
        manifest = json.loads((BASE / 'shared-enu-calibration/input-manifest.json').read_text())
        windows = json.loads((BASE / 'structure-experiment/temporal/window-manifest.json').read_text())
        req, protected = {}, {}
        for r in manifest:
            if r['partition'] == 'heldout' and r['agent'] > 0:
                req[(r['take'], r['agent'], str(r['frame_id']))] = 'reserved'
                protected.setdefault((r['take'], r['agent']), []).append(int(r['frame_id']))
        for r in windows:
            req[(r['take'], r['agent'], str(r['frame_id']))] = 'window'
            protected.setdefault((r['take'], r['agent']), []).append(int(r['frame_id']))
        timelines = json.loads(TIMELINES.read_text())
        for take in range(1, 6):
            for agent in (1, 2, 3):
                ids = timelines[str(take)][str(agent)]['ids']
                for fid in ids[::5]:
                    if any(abs(fid - p) <= 5 for p in protected.get((take, agent), [])):
                        continue
                    req.setdefault((take, agent, str(fid)), 'trajectory')
        return req

    def read(self):
        cache = self.output / 'frames-2hz.npz'
        if cache.exists():
            return cache
        W = np.asarray(json.loads((BASE / 'shared-enu-calibration/calibration.json').read_text())['original_W'])
        req = self.requests()
        jobs = [(t, [(a, f, role) for (tt, a, f), role in req.items() if tt == t], W) for t in range(1, 6)]
        # Reading does not use Open3D, so forked workers are safe here.
        with ProcessPoolExecutor(5) as pool:
            rows = [r for chunk in pool.map(read_take, jobs) for r in chunk]
        np.savez_compressed(cache, **{f"{r['take']}|{r['agent']}|{r['frame_id']}|{r['role']}": r['local'] for r in rows}, **{f"pose|{r['take']}|{r['agent']}|{r['frame_id']}": r['released'] for r in rows})
        self.results['frames_read'] = {role: sum(r['role'] == role for r in rows) for role in ['trajectory', 'reserved', 'window']}
        print('read', self.results['frames_read'], flush=True)
        return cache

    def run(self):
        cache = self.read()
        # Spawned workers: forked workers deadlocked in Open3D normal estimation.
        with ProcessPoolExecutor(FIT_WORKERS, mp_context=multiprocessing.get_context('spawn')) as pool:
            takes = list(pool.map(run_take, [(t, cache) for t in range(1, 6)]))
        self.takes = takes
        for t in takes:
            for line in t['log']:
                print(line, flush=True)
        self.analyze()
        self.write('results.json', self.results)
        self.write('poses.json', {t['take']: t['poses'] for t in takes})
        self.write('controls.json', {t['take']: t['controls'] for t in takes})
        self.map_stage()
        self.write('results.json', self.results)
        self.figures_cv()
        self.figures_time()
        self.figures_controls()
        print('DONE', flush=True)

    # ------------------------------------------------------------ analysis
    def analyze(self):
        cv = [r for t in self.takes for r in t['cv_rows']]
        table = {}
        for spacing in SETTINGS['knot_spacings_s']:
            key = 'const' if spacing is None else f'{spacing:g}s'
            for fam in ['released', 'timing_anchor', 'refined']:
                infra = [r for r in cv if r['kind'] == 'infrastructure' and r['spacing'] == spacing and r['family'] == fam]
                pairs = [r for r in cv if r['kind'] == 'pair' and r['spacing'] == spacing and r['family'] == fam]
                s = Scorer.summary(infra)
                table.setdefault(key, {})[fam] = dict(**s, pair_median_m=float(np.median([r['median_abs_m'] for r in pairs])) if pairs else None, pair_count=len(pairs))
        score = {k: v['refined']['all']['ground_height_median_m'] + v['refined']['all']['wall_median_m'] for k, v in table.items() if v['refined']['all']['ground_height_median_m'] is not None}
        chosen = min(score, key=score.get)
        self.chosen = chosen
        val = {}
        names = list(self.takes[0]['val_infra'])
        for n in names:
            rows = [r for t in self.takes for r in t['val_infra'].get(n, [])]
            pairs = [r for t in self.takes for r in t['val_pairs'].get(n, [])]
            val[n] = dict(**Scorer.summary(rows), pair_median_m=float(np.median([r['median_abs_m'] for r in pairs])) if pairs else None, pair_count=len(pairs))
        controls = {t['take']: {n: dict(support=c['support'], controls=c['controls']) for n, c in t['controls'].items()} for t in self.takes}
        self.results.update(cross_validation=table, cv_selection_score=score, chosen_spacing=chosen, validation=val, controls=controls, cv_rows=cv, validation_rows={t['take']: t['val_infra'] for t in self.takes}, validation_pair_rows={t['take']: t['val_pairs'] for t in self.takes})
        best = f'spline_{chosen}_joint'
        rule = self.adoption(table[chosen], val, best)
        self.results['adoption'] = rule
        print('CV', json.dumps({k: {f: (v[f]['all']['ground_height_median_m'], v[f]['all']['wall_median_m'], v[f]['pair_median_m']) for f in v} for k, v in table.items()}), flush=True)
        print('CHOSEN', chosen, 'VALIDATION', json.dumps({n: (v['all']['ground_height_median_m'], v['all']['wall_median_m'], v['pair_median_m'], v['all']['frames']) for n, v in val.items()}), flush=True)
        print('ADOPTION', rule, flush=True)

    @staticmethod
    def adoption(cv, val, best):
        def gain(new, old):
            return (old - new) / old if (old and new is not None) else None
        out = {}
        for label, block, ref_names in [('cv_heldout_blocks', cv, ['released', 'timing_anchor']), ('reserved_and_window', val, ['released', 'timing_anchor'])]:
            mine = block['refined'] if label == 'cv_heldout_blocks' else block[best]
            for ref in ref_names:
                other = block[ref]
                out[f'{label}_vs_{ref}'] = dict(ground_gain=gain(mine['all']['ground_height_median_m'], other['all']['ground_height_median_m']), wall_gain=gain(mine['all']['wall_median_m'], other['all']['wall_median_m']), pair_change_m=(mine['pair_median_m'] - other['pair_median_m']) if (mine['pair_median_m'] is not None and other['pair_median_m'] is not None) else None)
        out['meets_rule'] = all(v['ground_gain'] is not None and v['ground_gain'] >= .1 and v['wall_gain'] >= .1 and (v['pair_change_m'] is None or v['pair_change_m'] <= 0) for v in out.values() if isinstance(v, dict))
        return out

    def map_stage(self):
        g = load_gaussian_map(json.loads((BASE / 'shared-enu-calibration/calibration.json').read_text())['map_path'])
        p = MAP_CONFIG['primary_target']
        target = MapSurface(g, p['min_opacity']).target(p['plane_point'], p['max_local_sigma_m'])
        ref = InfrastructureReference()
        cache = self.output / 'frames-2hz.npz'
        raw = json.loads(TIMELINES.read_text())
        timelines = {(int(t), int(a)): PoseTimeline(r) for t, d in raw.items() for a, r in d.items()}
        frames = [f for t in range(1, 6) for f in load_take_frames(t, cache, timelines)]
        poses = {t['take']: t['poses'] for t in self.takes}
        best = f'spline_{self.chosen}_joint'
        out = {}
        for name in ['released', 'timing_anchor', best]:
            pose_of = (lambda f: f.poses[name]) if name in ('released', 'timing_anchor') else (lambda f: np.asarray(poses[f.take][best][f.key]) if best in poses[f.take] else None)
            items = [SourceItem('infrastructure', 0, 0, 'union', ref.item.points)]
            for v in (1, 2, 3):
                parts = [transform(f.local.astype(float), pose_of(f)) for f in frames if f.agent == v and f.role == 'trajectory' and pose_of(f) is not None]
                items.append(SourceItem(f'vehicle{v}', v, 0, 'trajectory', voxel(np.concatenate(parts), .25)))
            solver = BalancedSolver(target)
            Ds = {label: solver.fit(solver.design(sub, None, np.random.default_rng(7)))[0] for label, sub in [('all', items)] + [(f'leave_out_{UNIT_NAMES[u]}', [i for i in items if i.unit != u]) for u in range(4)]}
            res = {'ground': [], 'vertical': []}
            for f in frames:
                if f.role not in ('reserved', 'window') or pose_of(f) is None:
                    continue
                x = transform(transform(f.local.astype(float), pose_of(f)), Ds['all'])
                d, j = target.tree.query(x, workers=8)
                for c in res:
                    m = (d < 1.) & (f.cls == c)
                    if m.sum() >= 50:
                        res[c].append(float(np.median(np.abs(np.sum(target.normals[j[m]] * (x[m] - target.plane_points[j[m]]), 1)))))
            stab = {k: describe(D, Ds['all']) for k, D in Ds.items()}
            out[name] = dict(D=Ds['all'].tolist(), max_leave_one_out_displacement_m=float(max(s['max_displacement_120m_box_m'] for k, s in stab.items() if k != 'all')), stability=stab, validation_vs_map_median_m={c: float(np.median(v)) for c, v in res.items()})
            print('MAP', name, out[name]['max_leave_one_out_displacement_m'], out[name]['validation_vs_map_median_m'], flush=True)
        self.results['map_stage'] = out

    # ------------------------------------------------------------ figures
    def figures_cv(self):
        table = self.results['cross_validation']
        keys = list(table)
        fig, axes = plt.subplots(1, 3, figsize=(18, 5), layout='constrained')
        for ax, metric, label in [(axes[0], 'ground_height_median_m', 'held-out ground height (m)'), (axes[1], 'wall_median_m', 'held-out wall separation (m)'), (axes[2], 'pair_median_m', 'held-out vehicle-pair separation (m)')]:
            for fam, color in [('released', '#2a78d6'), ('timing_anchor', '#eb6834'), ('refined', '#1baf7a')]:
                vals = [table[k][fam]['all'][metric] if metric != 'pair_median_m' else table[k][fam][metric] for k in keys]
                ax.plot(range(len(keys)), vals, marker='o', lw=2, color=color, label=fam)
            ax.set_xticks(range(len(keys)), keys)
            ax.set(xlabel='knot spacing of the refinement', ylabel=label, title=label)
            ax.legend(fontsize=8)
        fig.suptitle(f'Blocked cross-validation over time (5 folds per take, all takes). Chosen spacing: {self.chosen}', fontsize=12)
        fig.savefig(self.figures / 'cross-validation.png', dpi=140)
        plt.close(fig)

    def figures_time(self):
        fig, axes = plt.subplots(1, 2, figsize=(16, 5.5), layout='constrained')
        cv = [r for r in self.results['cv_rows'] if r['kind'] == 'infrastructure' and ((r['spacing'] is None and self.chosen == 'const') or (r['spacing'] is not None and f"{r['spacing']:g}s" == self.chosen))]
        for ax, key, label in [(axes[0], 'ground_signed', '|frame median ground height| (m)'), (axes[1], 'vertical', 'frame median |wall separation| (m)')]:
            for fam, color in [('released', '#2a78d6'), ('timing_anchor', '#eb6834'), ('refined', '#1baf7a')]:
                sub = [r for r in cv if r['family'] == fam and r[key] is not None]
                t = np.abs([r['seconds'] for r in sub]); v = np.abs([r[key] for r in sub])
                bins = np.array([0, 2, 5, 10, 20, 40, 90])
                idx = np.digitize(t, bins) - 1
                med = [np.median(v[idx == k]) if (idx == k).sum() >= 5 else np.nan for k in range(len(bins) - 1)]
                ax.plot((bins[:-1] + bins[1:]) / 2, med, color=color, lw=2.2, marker='o', label=f'{fam} (held-out CV blocks)')
            ax.axhline(.1, color='#e34948', ls='--', lw=.8)
            ax.set(xscale='symlog', xlim=(0, 90), ylim=(0, .6), xlabel='seconds from the release ICP fitting frame', ylabel=label)
            ax.legend(fontsize=8)
        fig.suptitle(f'Held-out time blocks, chosen spacing {self.chosen}, binned medians. Red line: 0.1 m.', fontsize=12)
        fig.savefig(self.figures / 'heldout-error-vs-time.png', dpi=140)
        plt.close(fig)

    def figures_controls(self):
        best = f'spline_{self.chosen}_joint'
        fig, axes = plt.subplots(5, 3, figsize=(18, 17), layout='constrained', squeeze=False)
        for t in self.takes:
            c = t['controls'].get(best)
            if c is None:
                continue
            for v in (1, 2, 3):
                ax = axes[t['take'] - 1, v - 1]
                o, m, spacing, t0 = c['offsets'][v], c['counts'][v], c['spacing'], c['t0'][v]
                C = np.asarray(c['C'][o:o + 6 * m]).reshape(-1, 6)
                basis = SplineBasis(t0, t0 + 1, spacing)
                basis.count = m
                times = np.linspace(t0, t0 + (m - 3) * spacing, 200) if spacing else np.array([t0, t0 + 1])
                xi = np.array([basis.weights(s)[1] @ C[basis.weights(s)[0]] for s in times])
                ax.plot(times, np.degrees(np.hypot(xi[:, 0], xi[:, 1])), color='#4a3aa7', lw=2, label='tilt (deg)')
                ax.plot(times, np.degrees(xi[:, 2]), color='#eb6834', lw=2, label='yaw (deg)')
                ax.plot(times, xi[:, 5], color='#1baf7a', lw=2, label='height (m)')
                ax.plot(times, np.hypot(xi[:, 3], xi[:, 4]), color='#2a78d6', lw=2, label='horizontal (m)')
                fr = [x for x in t['frames'] if x['agent'] == v and x['role'] == 'trajectory']
                ax.plot([x['seconds'] for x in fr], np.full(len(fr), -0.95), '|', color='#8a8d91', ms=6)
                ax.axhline(0, color='#0b0b0b', lw=.5)
                ax.set(ylim=(-1, 1.5), xlabel='seconds from ICP frame', title=f'take {t["take"]}, {UNIT_NAMES[v]}')
                if t['take'] == 1 and v == 1:
                    ax.legend(fontsize=7)
        fig.suptitle(f'Fitted smooth correction over time ({best}); gray ticks mark training frames', fontsize=12)
        fig.savefig(self.figures / 'correction-trajectories.png', dpi=130)
        plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=BASE / 'vehicle-trajectory-refinement')
    TrajectoryRefinementExperiment(parser.parse_args().output).run()

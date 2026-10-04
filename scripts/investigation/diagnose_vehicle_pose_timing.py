"""Diagnose time-varying CooperScene vehicle pose error: timing, drift or scan distortion.

Released vehicle poses are sampled at GNSS time; each LiDAR scan is stamped later
by a per-take/vehicle gap of 0-90 ms. The release ICP correction, fitted at one
frame per take, absorbs that gap times the velocity at that frame only. This
experiment tests whether the remaining error is explained by that timing gap,
without fitting any per-frame or per-vehicle parameter:

    timing_anchor(f) = S(a) @ inverse(I(a)) @ I(f)

``S`` is the released W-frame pose at GNSS time, ``I(x)`` is the released pose
sequence interpolated to scan ``x``'s LiDAR timestamp, and ``a`` is the release
ICP fitting frame. The result equals the released pose exactly at ``a``.
One optional global clock offset is fitted only inside leave-one-take-out folds.

Reference: stationary infrastructure. Released files are never modified; this is
geometric consistency, not independent ground truth.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path
import sys
import zipfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import spearmanr
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.investigation.calibrate_shared_enu_map import transform, voxel
from scripts.investigation.design_combined_enu_calibration import BASE, UNIT_COLORS, UNIT_NAMES, BalancedSolver, CONFIG as MAP_CONFIG, MapSurface, SourceItem, describe
import scripts.investigation.fit_vehicle_sensor_corrections as FV
from scripts.investigation.fit_vehicle_sensor_corrections import ARCHIVE, CorrectionFitter, InfrastructureReference, VehicleCorrectionExperiment
from src.localization.cooperscene import pose_to_matrix
from src.localization.gaussian_map import load_gaussian_map

ICP_FRAME = {1: 481693, 2: 488685, 3: 497206, 4: 515587, 5: 524102}  # release-audit reproduction
FRAMES = BASE / 'vehicle-sensor-corrections'
SETTINGS = dict(
    declared_before_scoring=True,
    hypotheses=dict(
        timing='pose error = gap x (velocity change since the ICP frame); removed by interpolating released poses to LiDAR time, anchored at the ICP frame',
        drift='error grows with time since the ICP frame independently of motion',
        scan_distortion='within-scan error depends on azimuth and grows with turn rate',
    ),
    decision_rules=dict(
        timing_supported='zero-parameter timing_anchor lowers leave-one-take-out median ground AND wall error versus released by >=20% on the shared point set, AND measured per-frame horizontal sensor displacement regresses on the predicted displacement with slope 0.5-1.5 and R^2 >= 0.3',
        residual_drift='after the best timing model, Spearman of frame error versus seconds from the ICP frame >= 0.3 with p < 0.01',
        scan_distortion='Spearman of non-rigid azimuthal ground residual versus |turn rate| >= 0.3 with p < 0.01',
    ),
    interpolation='translation linear, rotation geodesic between the two released poses bracketing the target time on the GNSS clock; linear extrapolation at sequence ends',
    clock_offset_grid_ms=list(range(-100, 101, 10)),
    clock_offset_selection='global offset added to every LiDAR timestamp; chosen on the four training takes by mean per-frame capped residual on trajectory frames; scored on the held-out take',
    shared_point_set='points within 1 m of infrastructure under any compared pose family; residuals beyond 1 m capped at 1 m',
    primary_comparison='released, timing_anchor and timing_anchor_offset share one point set; unanchored interpolation is scored pairwise with released only',
    per_frame_measurement='diagnostic only, never applied: near-unregularised (prior weight 1e-4) world-frame rigid fit of a single frame to infrastructure; frames kept when >=400 correspondences and smallest/largest information eigenvalue >= 1e-3',
    azimuth_sectors=12,
)


def read_timeline(job):
    take, W = job
    out = {}
    with zipfile.ZipFile(next(ARCHIVE.glob(f'*/{take}.zip'))) as z:
        names = set(z.namelist())
        for agent in (1, 2, 3):
            meta = json.loads(z.read(f'{take}/{agent}/metadata.json'))
            ids = sorted(int(n.split('/')[-1][:-5]) for n in names if n.startswith(f'{take}/{agent}/') and n.endswith('.yaml'))
            poses = [W @ np.array(pose_to_matrix(yaml.safe_load(z.read(f'{take}/{agent}/{i}.yaml'))['lidar_pose'])) for i in ids]
            out[agent] = dict(ids=ids, gnss=[int(meta[str(i)]['gnss_ns']) for i in ids], lidar=[int(meta[str(i)]['lidar_ns']) for i in ids], poses=np.array(poses).tolist())
    return take, out


class PoseTimeline:
    """Released W-frame poses of one take/vehicle on the GNSS clock."""

    def __init__(self, record):
        self.ids = np.asarray(record['ids'])
        self.index = {int(i): k for k, i in enumerate(self.ids)}
        self.gnss = np.asarray(record['gnss'], dtype=np.int64)
        self.lidar = np.asarray(record['lidar'], dtype=np.int64)
        self.poses = np.asarray(record['poses'])

    def at(self, t_ns):
        i = int(np.clip(np.searchsorted(self.gnss, t_ns), 1, len(self.gnss) - 1))
        a, b = i - 1, i
        alpha = (t_ns - self.gnss[a]) / (self.gnss[b] - self.gnss[a])
        Ra, Rb = self.poses[a][:3, :3], self.poses[b][:3, :3]
        T = np.eye(4)
        T[:3, :3] = Ra @ Rotation.from_rotvec(alpha * Rotation.from_matrix(Ra.T @ Rb).as_rotvec()).as_matrix()
        T[:3, 3] = self.poses[a][:3, 3] + alpha * (self.poses[b][:3, 3] - self.poses[a][:3, 3])
        return T

    def released(self, frame_id):
        return self.poses[self.index[int(frame_id)]]

    def scan_time(self, frame_id, offset_ns=0):
        return self.at(self.lidar[self.index[int(frame_id)]] + offset_ns)

    def timing_anchor(self, frame_id, anchor_id, offset_ns=0):
        K = self.released(anchor_id) @ np.linalg.inv(self.scan_time(anchor_id, offset_ns))
        return K @ self.scan_time(frame_id, offset_ns)

    def motion(self, frame_id):
        k = self.index[int(frame_id)]
        a, b = max(k - 1, 0), min(k + 1, len(self.ids) - 1)
        dt = (self.gnss[b] - self.gnss[a]) / 1e9
        v = (self.poses[b][:3, 3] - self.poses[a][:3, 3]) / dt
        w = Rotation.from_matrix(self.poses[a][:3, :3].T @ self.poses[b][:3, :3]).as_rotvec() / dt
        return v, float(np.linalg.norm(w)), float(np.degrees(np.arctan2(*(self.poses[b][:3, :3] @ self.poses[a][:3, :3].T)[[1, 0], [0, 0]])) / dt)

    def seconds_from(self, frame_id, anchor_id):
        return float(self.lidar[self.index[int(frame_id)]] - self.lidar[self.index[int(anchor_id)]]) / 1e9

    def gap_ms(self, frame_id):
        k = self.index[int(frame_id)]
        return float(self.lidar[k] - self.gnss[k]) / 1e6


class TimingDiagnostic:
    def __init__(self, output: Path):
        self.output = output
        self.figures = output / 'figures'
        self.figures.mkdir(parents=True, exist_ok=True)
        self.results = dict(settings=SETTINGS)
        self.write('settings.json', SETTINGS)

    def write(self, name, value):
        (self.output / name).write_text(json.dumps(value, indent=1, allow_nan=False, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o)) + '\n')

    # ------------------------------------------------------------ inputs
    def timelines_cache(self):
        """Released poses and GNSS/LiDAR timestamps for every vehicle frame, read once from the archives."""
        W = np.asarray(json.loads((BASE / 'shared-enu-calibration/calibration.json').read_text())['original_W'])
        cache = self.output / 'timelines.json'
        if not cache.exists():
            with ProcessPoolExecutor(5) as pool:
                data = dict(pool.map(read_timeline, [(t, W) for t in range(1, 6)]))
            cache.write_text(json.dumps({str(t): {str(a): r for a, r in d.items()} for t, d in data.items()}))
        return cache

    def load(self):
        raw = json.loads(self.timelines_cache().read_text())
        self.timelines = {(int(t), int(a)): PoseTimeline(r) for t, d in raw.items() for a, r in d.items()}
        loader = VehicleCorrectionExperiment.__new__(VehicleCorrectionExperiment)
        loader.output, loader.results = FRAMES, {}
        loader.load_frames()
        self.frames = loader.frames
        self.ref = InfrastructureReference()
        worst = 0.
        for f in self.frames:
            tl = self.timelines[(f.take, f.agent)]
            worst = max(worst, float(np.abs(tl.released(f.frame_id) - f.poses['released']).max()))
            f.poses['timing_raw'] = tl.scan_time(f.frame_id)
            f.poses['timing_anchor'] = tl.timing_anchor(f.frame_id, ICP_FRAME[f.take])
            f.seconds = tl.seconds_from(f.frame_id, ICP_FRAME[f.take])
            f.gap_ms = tl.gap_ms(f.frame_id)
            f.velocity, f.turn_rate, f.yaw_rate_deg = tl.motion(f.frame_id)
        anchor_identity = max(float(np.abs(self.timelines[(t, a)].timing_anchor(ICP_FRAME[t], ICP_FRAME[t]) - self.timelines[(t, a)].released(ICP_FRAME[t])).max()) for t in range(1, 6) for a in (1, 2, 3))
        self.results['checks'] = dict(cached_vs_archive_released_pose_max_abs=worst, timing_anchor_equals_released_at_icp_frame_max_abs=anchor_identity, frames=len(self.frames))
        self.results['gaps_ms'] = {f'take{t}_vehicle{a}': dict(median=float(np.median((tl.lidar - tl.gnss) / 1e6)), p05=float(np.quantile((tl.lidar - tl.gnss) / 1e6, .05)), p95=float(np.quantile((tl.lidar - tl.gnss) / 1e6, .95))) for (t, a), tl in self.timelines.items()}
        print('checks', self.results['checks'], flush=True)

    # ------------------------------------------------------------ scoring
    def residuals(self, f, pose, points=None):
        x = transform(f.local.astype(float) if points is None else points, pose)
        d, j = self.ref.tree.query(x, workers=8)
        r = np.sum(self.ref.item.normals[j] * (x - self.ref.item.points[j]), 1)
        return d, r

    def score(self, families, frames):
        """Shared point set per frame; capped residuals."""
        rows = {n: [] for n in families}
        for f in frames:
            res = {n: self.residuals(f, pose_of(f)) for n, pose_of in families.items()}
            support = np.isin(f.cls, ['ground', 'vertical']) & np.any([d < 1. for d, _ in res.values()], axis=0)
            if support.sum() < 100:
                continue
            for n, (d, r) in res.items():
                r = np.where(d < 1., r, np.where(r >= 0, 1., -1.))
                row = dict(key=f.key, take=f.take, agent=f.agent, role=f.role, seconds=f.seconds, gap_ms=f.gap_ms, speed=float(np.linalg.norm(f.velocity[:2])), turn_rate=f.turn_rate)
                for c in ['ground', 'vertical']:
                    m = support & (f.cls == c)
                    row[c] = float(np.median(np.abs(r[m]))) if m.sum() >= 30 else None
                    row[c + '_signed'] = float(np.median(r[m])) if m.sum() >= 30 else None
                rows[n].append(row)
        return rows

    @staticmethod
    def summary(rows):
        out = {}
        for label, keep in [('all', lambda r: True)] + [(f'vehicle {v}', lambda r, v=v: r['agent'] == v) for v in (1, 2, 3)]:
            sub = [r for r in rows if keep(r)]
            g = [abs(r['ground_signed']) for r in sub if r['ground_signed'] is not None]
            w = [r['vertical'] for r in sub if r['vertical'] is not None]
            out[label] = dict(frames=len(sub), ground_height_median_m=float(np.median(g)) if g else None, ground_height_p90_m=float(np.quantile(g, .9)) if g else None, wall_median_m=float(np.median(w)) if w else None, wall_p90_m=float(np.quantile(w, .9)) if w else None)
        return out

    def clock_offset(self):
        """Leave-one-take-out choice of one global LiDAR clock offset."""
        traj = [f for f in self.frames if f.role == 'trajectory']
        curves = {}
        for f in traj:
            d0, _ = self.residuals(f, f.poses['released'])
            keep = (d0 < 1.) & np.isin(f.cls, ['ground', 'vertical'])
            if keep.sum() < 150:
                continue
            tl = self.timelines[(f.take, f.agent)]
            for off in SETTINGS['clock_offset_grid_ms']:
                d, r = self.residuals(f, tl.timing_anchor(f.frame_id, ICP_FRAME[f.take], off * 1_000_000), f.local[keep].astype(float))
                curves.setdefault(off, []).append((f.take, f.agent, float(np.mean(np.minimum(np.abs(r), 1.)))))
        table = {off: np.array(v, dtype=float) for off, v in curves.items()}
        folds = {}
        for t in range(1, 6):
            cost = {off: float(a[a[:, 0] != t, 2].mean()) for off, a in table.items()}
            folds[t] = min(cost, key=cost.get)
        per_vehicle = {v: {off: float(a[a[:, 1] == v, 2].mean()) for off, a in table.items() if (a[:, 1] == v).any()} for v in (1, 2, 3)}
        self.offsets = folds
        self.results['clock_offset'] = dict(fold_choice_ms={str(t): o for t, o in folds.items()}, all_takes_curve={str(o): float(a[:, 2].mean()) for o, a in table.items()}, per_vehicle_curve={str(v): {str(o): c for o, c in d.items()} for v, d in per_vehicle.items()}, trajectory_frames_used=int(len(next(iter(table.values())))))
        for f in self.frames:
            f.poses['timing_anchor_offset'] = self.timelines[(f.take, f.agent)].timing_anchor(f.frame_id, ICP_FRAME[f.take], folds[f.take] * 1_000_000)
        print('clock offset folds', folds, flush=True)

    def compare(self):
        families = {n: (lambda f, n=n: f.poses[n]) for n in ['released', 'timing_anchor', 'timing_anchor_offset']}
        rows = self.score(families, self.frames)
        self.rows = rows
        # Unanchored interpolation, scored pairwise with released only: shows why the ICP anchor matters.
        pair = self.score({n: (lambda f, n=n: f.poses[n]) for n in ['released', 'timing_raw']}, self.frames)
        self.results['unanchored_pairwise'] = {n: self.summary(v) for n, v in pair.items()}
        out = {}
        for subset, keep in [('all_frames', lambda r: True), ('reserved_and_window', lambda r: r['role'] in ('reserved', 'window')), ('more_than_10s_from_icp_frame', lambda r: abs(r['seconds']) > 10), ('within_3s_of_icp_frame', lambda r: abs(r['seconds']) <= 3)]:
            out[subset] = {n: self.summary([r for r in v if keep(r)]) for n, v in rows.items()}
            print('COMPARE', subset, {n: (s['all']['ground_height_median_m'], s['all']['wall_median_m'], s['all']['frames']) for n, s in out[subset].items()}, flush=True)
        self.results['comparison'] = out
        self.results['comparison_rows'] = rows

    # ------------------------------------------------------------ physics check
    def measured_vs_predicted(self):
        fitter = CorrectionFitter(self.ref, 'world', 'released')
        rows = []
        saved_prior = FV.SETTINGS['prior_weight']
        FV.SETTINGS['prior_weight'] = 1e-4  # near-unregularised measurement; never applied as a correction
        try:
            fits = {f.key: fitter.fit([f], min_frames=1) for f in self.frames}
        finally:
            FV.SETTINGS['prior_weight'] = saved_prior
        for f in self.frames:
            d, info = fits[f.key]
            if d is None:
                continue
            x = transform(f.local.astype(float), d @ f.poses['released'])
            dist, j = self.ref.tree.query(x, workers=8)
            ok = (dist < .4) & np.isin(f.cls, ['ground', 'vertical']) & (self.ref.item.cls[j] == f.cls)
            n = self.ref.item.normals[j[ok]]
            J = np.c_[np.cross(x[ok], n) / 30., n]  # rotation scaled by a 30 m lever so units are comparable
            e = np.linalg.eigvalsh(J.T @ J) if ok.sum() else np.zeros(6)
            if ok.sum() < 400 or e[-1] <= 0 or e[0] / e[-1] < 1e-3:
                continue
            p = f.poses['released'][:3, 3]
            measured = transform(p[None], d)[0] - p
            predicted = f.poses['timing_anchor'][:3, 3] - p
            rot = Rotation.from_matrix(d[:3, :3]).as_rotvec()
            rows.append(dict(key=f.key, agent=f.agent, take=f.take, role=f.role, frame_id=int(f.frame_id), seconds=f.seconds, turn_rate=f.turn_rate, speed=float(np.linalg.norm(f.velocity[:2])), measured_xyz=measured.tolist(), predicted_xyz=predicted.tolist(), measured_rotvec_deg=np.degrees(rot).tolist(), yaw_measured_deg=float(np.degrees(rot[2])), yaw_predicted_deg=float(np.degrees(Rotation.from_matrix(f.poses['timing_anchor'][:3, :3] @ f.poses['released'][:3, :3].T).as_rotvec()[2])), correspondences=int(ok.sum()), scaled_condition=float(e[-1] / e[0])))
        out = dict(frames=len(rows), rows=rows)
        if len(rows) >= 10:
            M = np.array([r['measured_xyz'][:2] for r in rows]).ravel()
            P = np.array([r['predicted_xyz'][:2] for r in rows]).ravel()
            slope = float(P @ M / (P @ P))
            ym = np.array([r['yaw_measured_deg'] for r in rows]); yp = np.array([r['yaw_predicted_deg'] for r in rows])
            tilt = np.array([np.hypot(*r['measured_rotvec_deg'][:2]) for r in rows])
            out.update(horizontal_slope=slope, horizontal_r2=float(1 - np.sum((M - slope * P) ** 2) / np.sum((M - M.mean()) ** 2)), horizontal_rms_measured_m=float(np.sqrt(np.mean(M ** 2))), horizontal_rms_after_prediction_m=float(np.sqrt(np.mean((M - P) ** 2))), yaw_spearman=float(spearmanr(ym, yp)[0]), median_measured_tilt_deg=float(np.median(tilt)), median_measured_abs_yaw_deg=float(np.median(np.abs(ym))), median_measured_vertical_m=float(np.median(np.abs([r['measured_xyz'][2] for r in rows]))), tilt_vs_turn_rate=dict(zip(['spearman', 'p'], map(float, spearmanr(tilt, [r['turn_rate'] for r in rows])))), tilt_vs_seconds=dict(zip(['spearman', 'p'], map(float, spearmanr(tilt, np.abs([r['seconds'] for r in rows]))))))
        # Smoothness on consecutive 10 Hz window frames: frame-to-frame change versus spread.
        smooth = {}
        for (t, a) in sorted({(r['take'], r['agent']) for r in rows if r['role'] == 'window'}):
            seq = sorted([r for r in rows if r['role'] == 'window' and r['take'] == t and r['agent'] == a], key=lambda r: r['frame_id'])
            ids = np.array([r['frame_id'] for r in seq])
            cons = np.flatnonzero(np.diff(ids) == 1)
            if len(cons) < 5:
                continue
            comp = {'horizontal_m': np.array([r['measured_xyz'][:2] for r in seq]), 'vertical_m': np.array([[r['measured_xyz'][2]] for r in seq]), 'tilt_deg': np.array([r['measured_rotvec_deg'][:2] for r in seq]), 'yaw_deg': np.array([[r['yaw_measured_deg']] for r in seq])}
            smooth[f'take{t}_vehicle{a}'] = dict(frames=len(seq), consecutive_pairs=len(cons), **{k: dict(spread_rms=float(np.sqrt(np.mean(np.sum((v - v.mean(0)) ** 2, 1)))), step_rms=float(np.sqrt(np.mean(np.sum((v[cons + 1] - v[cons]) ** 2, 1))))) for k, v in comp.items()})
        out['window_smoothness'] = smooth
        self.results['measured_vs_predicted'] = out
        print('MEASURED vs PREDICTED', {k: v for k, v in out.items() if k != 'rows'}, flush=True)

    def measurement_noise_floor(self):
        """Same per-frame fit on reserved stationary infrastructure scans, against a reference
        that excludes them. True motion is zero, so the result is the measurement floor."""
        from scipy.spatial import cKDTree
        from scripts.investigation.design_combined_enu_calibration import SELECTION
        with np.load(SELECTION / 'selected-clouds.npz') as s, np.load(BASE / 'shared-enu-calibration/input-clouds.npz') as a:
            ref_points = np.concatenate([s['infra_representative']] + [s[f'cal_{t}_0'] for t in range(1, 6)])
            tests = {k: a[k] for k in a.files if k.startswith('hold_') and k.split('_')[2] == '0'}
        ref = InfrastructureReference.__new__(InfrastructureReference)
        ref.item = SourceItem('infrastructure_without_reserved', 0, 0, 'union', ref_points)
        ref.tree = cKDTree(ref.item.points)
        fitter = CorrectionFitter(ref, 'world', 'released')
        saved_prior = FV.SETTINGS['prior_weight']
        FV.SETTINGS['prior_weight'] = 1e-4
        rows = []
        try:
            for key, points in tests.items():
                f = FV.Frame(int(key.split('_')[1]), 0, key.split('_')[3], 0, 'infrastructure_control', points.astype(np.float32), dict(released=np.eye(4)))
                f.prepare()
                d, info = fitter.fit([f], min_frames=1)
                if d is not None:
                    rv = np.degrees(Rotation.from_matrix(d[:3, :3]).as_rotvec())
                    rows.append(dict(scan=key, tilt_deg=float(np.hypot(*rv[:2])), yaw_deg=float(rv[2]), translation_at_sensor_m=d[:3, 3].tolist()))
        finally:
            FV.SETTINGS['prior_weight'] = saved_prior
        tilt = np.array([r['tilt_deg'] for r in rows])
        out = dict(note='per-frame measurement applied to reserved stationary infrastructure scans against a reference that excludes them; true motion is zero, so this is the measurement noise floor', scans=len(rows), tilt_median_deg=float(np.median(tilt)), tilt_max_deg=float(tilt.max()), rows=rows)
        self.write('measurement-noise-floor.json', out)
        self.results['measurement_noise_floor'] = {k: v for k, v in out.items() if k != 'rows'}
        print('NOISE FLOOR', self.results['measurement_noise_floor'], flush=True)

    # ------------------------------------------------------------ drift and scan distortion
    def residual_structure(self, best):
        rows = [r for r in self.rows[best] if r['ground'] is not None and r['vertical'] is not None]
        s = np.array([abs(r['seconds']) for r in rows])
        g = np.array([r['ground'] for r in rows]); w = np.array([r['vertical'] for r in rows])
        drift = dict(model=best, ground=dict(zip(['spearman', 'p'], map(float, spearmanr(s, g)))), wall=dict(zip(['spearman', 'p'], map(float, spearmanr(s, w)))))
        sectors = []
        for f in self.frames:
            g_mask = f.cls == 'ground'
            d, r = self.residuals(f, f.poses[best], f.local[g_mask].astype(float))
            keep = d < .5
            if keep.sum() < 200:
                continue
            p = f.local[g_mask][keep].astype(float)
            A = np.c_[np.ones(len(p)), p[:, 0], p[:, 1]]
            rem = r[keep] - A @ np.linalg.lstsq(A, r[keep], rcond=None)[0]
            az = np.floor((np.arctan2(p[:, 1], p[:, 0]) + np.pi) / (2 * np.pi / SETTINGS['azimuth_sectors'])).astype(int)
            med = [np.median(rem[az == k]) for k in range(SETTINGS['azimuth_sectors']) if (az == k).sum() >= 20]
            if len(med) >= 6:
                sectors.append((f.turn_rate, float(np.linalg.norm(f.velocity[:2])), float(np.sqrt(np.mean(np.square(med))))))
        a = np.array(sectors)
        distortion = dict(frames=len(a), turn_rate=dict(zip(['spearman', 'p'], map(float, spearmanr(a[:, 0], a[:, 2])))), speed=dict(zip(['spearman', 'p'], map(float, spearmanr(a[:, 1], a[:, 2])))), median_nonrigid_rms_m=float(np.median(a[:, 2])), points=a.tolist())
        self.results['residual_drift'] = drift
        self.results['scan_distortion'] = distortion
        print('DRIFT', drift, 'DISTORTION', {k: v for k, v in distortion.items() if k != 'points'}, flush=True)

    def tile_spread(self, names):
        out = {}
        ig = self.ref.item.points[self.ref.item.cls == 'ground']
        for name in names:
            parts = [np.c_[ig, np.zeros(len(ig))]]
            for f in self.frames:
                g = f.cls == 'ground'
                parts.append(np.c_[transform(f.local[g].astype(float), f.poses[name]), np.full(g.sum(), f.agent)])
            a = np.concatenate(parts)
            tile = np.floor(a[:, :2] / 5).astype(int)
            keys, inverse = np.unique(tile, axis=0, return_inverse=True)
            sp = []
            for k in range(len(keys)):
                m = inverse.ravel() == k
                med = [np.median(a[m & (a[:, 3] == u), 2]) for u in range(4) if (m & (a[:, 3] == u)).sum() >= 30]
                if len(med) >= 2:
                    sp.append(max(med) - min(med))
            sp = np.array(sp)
            out[name] = dict(tiles=len(sp), median_spread_m=float(np.median(sp)), p90_spread_m=float(np.quantile(sp, .9)), fraction_over_010m=float(np.mean(sp > .1)))
            print('SPREAD', name, out[name], flush=True)
        self.results['tile_spread'] = out

    def map_stage(self, names):
        g = load_gaussian_map(json.loads((BASE / 'shared-enu-calibration/calibration.json').read_text())['map_path'])
        p = MAP_CONFIG['primary_target']
        target = MapSurface(g, p['min_opacity']).target(p['plane_point'], p['max_local_sigma_m'])
        traj = [f for f in self.frames if f.role == 'trajectory']
        val = [f for f in self.frames if f.role in ('reserved', 'window')]
        out = {}
        for name in names:
            items = [SourceItem('infrastructure', 0, 0, 'union', self.ref.item.points)] + [SourceItem(f'vehicle{v}', v, 0, 'trajectory', voxel(np.concatenate([transform(f.local.astype(float), f.poses[name]) for f in traj if f.agent == v]), .25)) for v in (1, 2, 3)]
            solver = BalancedSolver(target)
            Ds = {}
            for label, subset in [('all', items)] + [(f'leave_out_{UNIT_NAMES[u]}', [i for i in items if i.unit != u]) for u in range(4)]:
                Ds[label] = solver.fit(solver.design(subset, None, np.random.default_rng(7)))[0]
            stab = {k: describe(D, Ds['all']) for k, D in Ds.items()}
            res = {'ground': [], 'vertical': []}
            for f in val:
                x = transform(transform(f.local.astype(float), f.poses[name]), Ds['all'])
                d, j = target.tree.query(x, workers=8)
                for c in res:
                    m = (d < 1.) & (f.cls == c)
                    if m.sum() >= 50:
                        res[c].append(float(np.median(np.abs(np.sum(target.normals[j[m]] * (x[m] - target.plane_points[j[m]]), 1)))))
            out[name] = dict(D=Ds['all'].tolist(), max_leave_one_out_displacement_m=float(max(s['max_displacement_120m_box_m'] for k, s in stab.items() if k != 'all')), stability=stab, heldout_vs_map_median_m={c: float(np.median(v)) for c, v in res.items()})
            print('MAP', name, out[name]['max_leave_one_out_displacement_m'], out[name]['heldout_vs_map_median_m'], flush=True)
        self.results['map_stage'] = out

    # ------------------------------------------------------------ figures
    def figures_physics(self):
        if self.results['measured_vs_predicted']['frames'] < 10:
            return
        mv = self.results['measured_vs_predicted']['rows']
        fig, axes = plt.subplots(1, 3, figsize=(19, 6), layout='constrained')
        for ax, idx, label in [(axes[0], 0, 'x (east-ish)'), (axes[1], 1, 'y (north-ish)')]:
            for v in (1, 2, 3):
                sub = [r for r in mv if r['agent'] == v]
                ax.scatter([r['predicted_xyz'][idx] for r in sub], [r['measured_xyz'][idx] for r in sub], c=UNIT_COLORS[v], s=24, label=UNIT_NAMES[v], edgecolor='white', lw=.5)
            lim = 1.0
            ax.plot([-lim, lim], [-lim, lim], color='#8a8d91', ls='--', lw=.8)
            ax.set(xlim=(-lim, lim), ylim=(-lim, lim), aspect='equal', xlabel=f'predicted by timing gap, {label} (m)', ylabel=f'measured against infrastructure, {label} (m)', title=f'Sensor displacement {label}')
            ax.legend(fontsize=8)
        ax = axes[2]
        for v in (1, 2, 3):
            sub = [r for r in mv if r['agent'] == v]
            ax.scatter([r['yaw_predicted_deg'] for r in sub], [r['yaw_measured_deg'] for r in sub], c=UNIT_COLORS[v], s=24, edgecolor='white', lw=.5)
        ax.plot([-1.5, 1.5], [-1.5, 1.5], color='#8a8d91', ls='--', lw=.8)
        ax.set(xlim=(-1.5, 1.5), ylim=(-1.5, 1.5), aspect='equal', xlabel='predicted yaw correction (deg)', ylabel='measured yaw correction (deg)', title='Yaw')
        s = self.results['measured_vs_predicted']
        fig.suptitle(f"Per-frame correction measured against infrastructure versus zero-parameter timing prediction ({s['frames']} well-constrained frames; horizontal slope {s['horizontal_slope']:.2f}, R^2 {s['horizontal_r2']:.2f})", fontsize=12)
        fig.savefig(self.figures / 'measured-vs-predicted.png', dpi=140)
        plt.close(fig)

    def figures_time(self, names):
        fig, axes = plt.subplots(1, 2, figsize=(16, 5.5), layout='constrained')
        colors = ['#2a78d6', '#eb6834', '#1baf7a', '#4a3aa7']
        for ax, key, label in [(axes[0], 'ground_signed', '|frame median ground height| (m)'), (axes[1], 'vertical', 'frame median |wall separation| (m)')]:
            for name, color in zip([n for n in names if n in self.rows], colors):
                rows = [r for r in self.rows[name] if r[key] is not None]
                t = np.abs([r['seconds'] for r in rows]); v = np.abs([r[key] for r in rows])
                bins = np.array([0, 1, 2, 3, 5, 7.5, 10, 15, 20, 30, 45, 90])
                idx = np.digitize(t, bins) - 1
                med = [np.median(v[idx == k]) if (idx == k).sum() >= 5 else np.nan for k in range(len(bins) - 1)]
                ax.plot((bins[:-1] + bins[1:]) / 2, med, color=color, lw=2.2, marker='o', ms=4, label=name)
            ax.axhline(.1, color='#e34948', ls='--', lw=.8)
            ax.set(xscale='symlog', xlim=(0, 90), ylim=(0, .7), xlabel='seconds from the release ICP fitting frame', ylabel=label)
            ax.legend(fontsize=8)
        fig.suptitle('All 584 frames, shared point set per frame, binned medians. Red line: 0.1 m.', fontsize=12)
        fig.savefig(self.figures / 'error-vs-time-timing.png', dpi=140)
        plt.close(fig)

    def figures_offset(self):
        c = self.results['clock_offset']
        fig, ax = plt.subplots(figsize=(9, 5), layout='constrained')
        offs = [int(o) for o in c['all_takes_curve']]
        ax.plot(offs, list(c['all_takes_curve'].values()), color='#0b0b0b', lw=2.4, label='all vehicles')
        for v in (1, 2, 3):
            d = c['per_vehicle_curve'][str(v)]
            ax.plot([int(o) for o in d], list(d.values()), color=UNIT_COLORS[v], lw=1.6, label=UNIT_NAMES[v])
        ax.axvline(0, color='#8a8d91', lw=.8)
        ax.set(xlabel='extra clock offset added to LiDAR timestamps (ms)', ylabel='mean capped residual on trajectory frames (m)', title='Timing-anchored poses: residual versus an extra clock offset')
        ax.legend(fontsize=8)
        fig.savefig(self.figures / 'clock-offset-curve.png', dpi=140)
        plt.close(fig)

    def figures_crops(self, names):
        rois = {r['id']: r for r in json.loads((ROOT / 'scripts/investigation/cooperscene_structure_rois.json').read_text())['rois']}
        ids = ['NW_upper_facade', 'SW_boundary_wall', 'road_central', 'road_south']
        val = [f for f in self.frames if f.role in ('reserved', 'window')]
        fig, axes = plt.subplots(len(names), len(ids), figsize=(5 * len(ids), 3.6 * len(names)), layout='constrained', squeeze=False)
        for col, rid in enumerate(ids):
            roi = rois[rid]
            n = np.asarray(roi['normal'], float); n /= np.linalg.norm(n)
            u = np.asarray(roi['tangent_u'], float); u -= n * (u @ n); u /= np.linalg.norm(u)
            B = np.c_[u, np.cross(n, u), n]
            hw = np.asarray(roi['half_widths_uvn']); anchor = np.asarray(roi['anchor'])
            crop = lambda p: (lambda q: q[(np.abs(q[:, 0]) <= hw[0] + 1) & (np.abs(q[:, 1]) <= hw[1]) & (np.abs(q[:, 2]) <= 1.5)])((p - anchor) @ B)
            iq = crop(self.ref.item.points)
            for row, name in enumerate(names):
                ax = axes[row, col]
                for v in (1, 2, 3):
                    q = crop(np.concatenate([transform(f.local.astype(float), f.poses[name]) for f in val if f.agent == v]))
                    ax.scatter(q[:, 0], q[:, 2], s=4, c=UNIT_COLORS[v], alpha=.6, label=UNIT_NAMES[v])
                ax.scatter(iq[:, 0], iq[:, 2], s=9, c=UNIT_COLORS[0], edgecolor='#0b0b0b', lw=.3, label='infrastructure (reference)')
                ax.set(xlim=(-hw[0] - 1, hw[0] + 1), ylim=(-1.5, 1.5), xlabel='along surface (m)', ylabel='height (m)' if roi['kind'] == 'road' else 'normal offset (m)', title=f'{rid}\n{name}')
                if row == 0 and col == 0:
                    ax.legend(fontsize=7, markerscale=2)
        fig.suptitle('Fixed patches, reserved and time-window frames (W frame, no map). Infrastructure outlined blue is the reference.', fontsize=12)
        fig.savefig(self.figures / 'structure-crops-timing.png', dpi=140)
        plt.close(fig)

    def run(self):
        self.load()
        self.clock_offset()
        self.compare()
        self.write('results.json', self.results)
        self.measured_vs_predicted()
        self.measurement_noise_floor()
        cmp = self.results['comparison']['all_frames']
        best = min(['timing_anchor', 'timing_anchor_offset'], key=lambda n: cmp[n]['all']['wall_median_m'] + cmp[n]['all']['ground_height_median_m'])
        self.results['best_timing_family'] = best
        self.residual_structure(best)
        self.write('results.json', self.results)
        names = ['released', 'timing_raw', 'timing_anchor', 'timing_anchor_offset']
        self.tile_spread(names)
        self.map_stage(['released', best])
        self.write('results.json', self.results)
        self.figures_physics()
        self.figures_time(names)
        self.figures_offset()
        self.figures_crops(['released', best])
        print('DONE', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=BASE / 'vehicle-pose-timing')
    parser.add_argument('--stage', choices=['all', 'timelines'], default='all')
    args = parser.parse_args()
    job = TimingDiagnostic(args.output)
    job.timelines_cache() if args.stage == 'timelines' else job.run()

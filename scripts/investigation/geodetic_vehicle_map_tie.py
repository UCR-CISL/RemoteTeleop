"""Geodetic CooperScene-vehicle to GSplat map tie, independent of infrastructure and ICP.

The GSplat map is georeferenced ENU about (33.9757064853, -117.3398869498) with
z = orthometric elevation - 273.43 m. Each CooperScene vehicle logs raw RTK
GNSS/INS at 400 Hz in the raw release. This experiment builds, per scan,

    map_T_lidar = Pose(R_INS(t) @ F_a, p_GNSS(t)) @ inverse(E_a)

at the scan's LiDAR timestamp, with p in the map's own ENU frame and E_a the
MoCap xsens->lidar calibration applied on the vehicle side. No D, no W fit, no
ICP correction and no infrastructure are used. GSplat and vehicle-to-vehicle
LiDAR agreement then test the result. One per-vehicle rigid correction is
fitted only inside leave-one-take-out folds, to measure what remains.

Conditional assumptions (recorded, not verified): GNSS position denotes the
Xsens origin; normalised INS axes coincide with the MoCap Xsens marker frame;
logged Altitude shares the map's elevation datum. Released files are unchanged.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import pickle
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.investigation.calibrate_shared_enu_map import transform
from scripts.investigation.design_combined_enu_calibration import BASE, UNIT_COLORS, UNIT_NAMES, CONFIG as MAP_CONFIG, MapSurface, describe
from scripts.investigation.diagnose_vehicle_pose_timing import ICP_FRAME, PoseTimeline
from scripts.investigation.refine_vehicle_trajectories import load_take_frames, TIMELINES
from src.localization.cooperscene_reference import CooperSceneReference
from src.localization.gaussian_map import load_gaussian_map

RELEASE = Path('/mnt/bcc-data/data/Cooperscene/release/250928')
CALIBRATION = Path('/mnt/bcc-data/data/Cooperscene/backup/calib_data/250928')
GEOREFERENCE = Path('/home/jyue86/Downloads/riverside_for_justin/georeference.json')
FRAMES = BASE / 'vehicle-trajectory-refinement/frames-2hz.npz'
SETTINGS = dict(
    declared_before_scoring=True,
    pose='Pose(R_original_INS(t_lidar) @ F_a, p_GNSS(t_lidar) in map ENU) @ inverse(E_a); raw 400 Hz log interpolated to the scan LiDAR timestamp (translation linear, rotation slerp)',
    map_frame='map georeference: ENU at the published origin, z = elevation - z_ref; ECEF->ENU exact',
    F='agent 1: diag(1,-1,-1); agents 2,3: identity (release device-axis normalisation)',
    comparisons=['released @ shared D (provisional reference)', 'geodetic at GNSS time', 'geodetic at LiDAR time'],
    frames='2 Hz trajectory, reserved and time-window frames from vehicle-trajectory-refinement/frames-2hz.npz',
    map_target='opacity > 0.05, local-centroid plane, local thickness < 0.10 m',
    scoring='per frame, class-matched point-to-surface residual within 1 m of the map surface; ground signed median and wall absolute median; vehicle pairs at identical frame IDs scored LiDAR-to-LiDAR (no map)',
    correction_fit='one rigid world-frame correction per vehicle, fitted on four takes and scored on the fifth (point-to-plane to the map, ground and walls equal weight, Huber 0.10 m)',
    decision_rules=dict(
        geodetic_supported_for_vehicle='with no fitted correction: median |frame road height| <= 0.15 m AND median within-take std of frame road height <= 0.10 m AND median vehicle-pair separation not worse than released @ D',
        common_offset='if leave-one-take-out per-vehicle corrections agree across vehicles within 0.1 m and 0.2 deg, the remainder is attributed to the map georeference and shared',
    ),
)


class GeodeticPoses:
    """Raw RTK GNSS/INS poses of one take/vehicle, in the map's own ENU frame."""

    def __init__(self, take, agent, geo):
        rows = list(csv.DictReader((RELEASE / str(take) / str(agent) / 'raw_gnss.csv').read_text().splitlines()))
        self.t = np.array([int(r['gnss_ns']) for r in rows], dtype=np.int64)
        order = np.argsort(self.t)
        self.t = self.t[order]
        rows = [rows[i] for i in order]
        lla = np.array([[float(r['Latitude']), float(r['Longitude']), float(r['Altitude'])] for r in rows])
        origin = (geo['enu_origin']['latitude_deg'], geo['enu_origin']['longitude_deg'], geo['z_reference']['z_ref_m'])
        ref = CooperSceneReference(origin, np.eye(4), np.eye(4))
        self.p = np.array([ref.enu_position(dict(Latitude=a, Longitude=b, Altitude=c)) for a, b, c in lla])
        q = np.array([[float(r['Quat_qx']), float(r['Quat_qy']), float(r['Quat_qz']), float(r['Quat_qw'])] for r in rows])
        F = np.diag([1., -1., -1.]) if agent == 1 else np.eye(3)
        self.R = Rotation.from_matrix(Rotation.from_quat(q).as_matrix() @ F)
        self.slerp = Slerp(self.t.astype(float), self.R)
        words = np.array([int(r['StatusWord']) for r in rows], dtype=np.int64)
        self.rtk = (words >> 27) & 3
        E = np.asarray(pickle.load(open(CALIBRATION / f'tower{agent}-tfs.pkl', 'rb'))[f'tower{agent}-xsens,tower{agent}-lidar'])
        self.E_inv = np.linalg.inv(E)

    def lidar_pose(self, t_ns):
        t = float(np.clip(t_ns, self.t[0], self.t[-1]))
        T = np.eye(4)
        T[:3, :3] = self.slerp([t]).as_matrix()[0]
        T[:3, 3] = [np.interp(t, self.t, self.p[:, k]) for k in range(3)]
        return T @ self.E_inv

    def rtk_status(self, t_ns):
        return int(self.rtk[int(np.clip(np.searchsorted(self.t, t_ns), 0, len(self.t) - 1))])


class GeodeticTieExperiment:
    def __init__(self, output: Path):
        self.output = output
        self.figures = output / 'figures'
        self.figures.mkdir(parents=True, exist_ok=True)
        self.results = dict(settings=SETTINGS)
        self.write('settings.json', SETTINGS)

    def write(self, name, value):
        (self.output / name).write_text(json.dumps(value, indent=1, allow_nan=False, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o)) + '\n')

    def load(self):
        geo = json.loads(GEOREFERENCE.read_text())
        cal = json.loads((BASE / 'shared-enu-calibration/calibration.json').read_text())
        self.D, W = np.asarray(cal['D']), np.asarray(cal['original_W'])
        # The inherited W must be the geodetic offset between the two ENU origins.
        lat0 = geo['enu_origin']['latitude_deg']
        offset = [(-117.339873 - geo['enu_origin']['longitude_deg']) * 111412.8 * math.cos(math.radians(lat0)), (33.975676 - lat0) * 111132.9, 273.0 - geo['z_reference']['z_ref_m']]
        self.results['W_vs_geodetic_origin_offset_m'] = (W[:3, 3] - np.array(offset)).tolist()
        raw = json.loads(TIMELINES.read_text())
        self.timelines = {(int(t), int(a)): PoseTimeline(r) for t, d in raw.items() for a, r in d.items()}
        self.geodetic = {(t, a): GeodeticPoses(t, a, geo) for t in range(1, 6) for a in (1, 2, 3)}
        self.frames = [f for t in range(1, 6) for f in load_take_frames(t, FRAMES, self.timelines)]
        rtk = {}
        for f in self.frames:
            tl, g = self.timelines[(f.take, f.agent)], self.geodetic[(f.take, f.agent)]
            k = tl.index[int(f.frame_id)]
            f.poses['released_D'] = self.D @ f.poses['released']
            f.poses['geodetic_gnss_time'] = g.lidar_pose(tl.gnss[k])
            f.poses['geodetic_lidar_time'] = g.lidar_pose(tl.lidar[k])
            f.rtk = g.rtk_status(tl.lidar[k])
            rtk.setdefault(f'take{f.take}_vehicle{f.agent}', []).append(f.rtk)
        self.results['rtk_status_fraction'] = {k: {name: float(np.mean(np.array(v) == code)) for code, name in enumerate(['none', 'float', 'fixed', 'reserved'])} for k, v in rtk.items()}
        # Consistency check: released metadata attitude must equal E.R @ R_INS @ F at the GNSS sample.
        worst = 0.
        for f in self.frames[::40]:
            tl, g = self.timelines[(f.take, f.agent)], self.geodetic[(f.take, f.agent)]
            k = tl.index[int(f.frame_id)]
            i = int(np.argmin(np.abs(g.t - tl.gnss[k])))
            worst = max(worst, abs(int(g.t[i]) - int(tl.gnss[k])) / 1e6)
        self.results['nearest_raw_sample_to_metadata_gnss_ms_max'] = worst
        g = load_gaussian_map(cal['map_path'])
        p = MAP_CONFIG['primary_target']
        self.target = MapSurface(g, p['min_opacity']).target(p['plane_point'], p['max_local_sigma_m'])
        print('loaded', len(self.frames), 'frames; W - geodetic offset', np.round(self.results['W_vs_geodetic_origin_offset_m'], 4), flush=True)

    # ------------------------------------------------------------ scoring
    def map_residuals(self, f, T):
        x = transform(f.local.astype(float), T)
        d, j = self.target.tree.query(x, workers=8)
        n = self.target.normals[j]
        R = T[:3, :3]
        nz = np.abs((f.normals @ R.T)[:, 2])
        r = np.sum(n * (x - self.target.plane_points[j]), 1)
        ground = (d < 1.) & (nz > .9) & (np.abs(n[:, 2]) > .9) & (f.cls == 'ground')
        wall = (d < 1.) & (nz < .3) & (np.abs(n[:, 2]) < .3) & (f.cls == 'vertical')
        return r, ground, wall

    def score_map(self, names):
        rows = {n: [] for n in names}
        for f in self.frames:
            for n in names:
                r, g, w = self.map_residuals(f, f.poses[n])
                rows[n].append(dict(key=f.key, take=f.take, agent=f.agent, role=f.role, seconds=f.seconds, rtk=f.rtk, ground_signed=float(np.median(r[g])) if g.sum() >= 50 else None, wall_abs=float(np.median(np.abs(r[w]))) if w.sum() >= 50 else None))
        return rows

    @staticmethod
    def score_pairs(frames, names):
        by_id = {}
        for f in frames:
            by_id.setdefault((f.take, f.frame_id), {})[f.agent] = f
        rows = {n: [] for n in names}
        for group in by_id.values():
            for a in group:
                for b in group:
                    if a >= b:
                        continue
                    fa, fb = group[a], group[b]
                    keep_b = np.isin(fb.cls, ['ground', 'vertical'])
                    for n in names:
                        xa = transform(fa.local[np.isin(fa.cls, ['ground', 'vertical'])].astype(float), fa.poses[n])
                        xb = transform(fb.local[keep_b].astype(float), fb.poses[n])
                        nb = (fb.normals @ fb.poses[n][:3, :3].T)[keep_b]
                        d, j = cKDTree(xb).query(xa, workers=8)
                        ok = d < 1.
                        if ok.sum() >= 100:
                            rows[n].append(dict(take=fa.take, pair=f'{a}-{b}', points=int(ok.sum()), median_abs_m=float(np.median(np.abs(np.sum(nb[j[ok]] * (xa[ok] - xb[j[ok]]), 1))))))
        return rows

    @staticmethod
    def summarize(rows):
        out = {}
        for v in (1, 2, 3):
            sub = [r for r in rows if r['agent'] == v]
            g = np.array([r['ground_signed'] for r in sub if r['ground_signed'] is not None])
            w = np.array([r['wall_abs'] for r in sub if r['wall_abs'] is not None])
            stds = [np.std([r['ground_signed'] for r in sub if r['take'] == t and r['ground_signed'] is not None]) for t in range(1, 6) if sum(r['take'] == t and r['ground_signed'] is not None for r in sub) >= 5]
            out[f'vehicle {v}'] = dict(frames=len(sub), ground_frames=len(g), median_abs_road_height_m=float(np.median(np.abs(g))) if len(g) else None, median_signed_road_height_m=float(np.median(g)) if len(g) else None, p90_abs_road_height_m=float(np.quantile(np.abs(g), .9)) if len(g) else None, median_within_take_std_m=float(np.median(stds)) if stds else None, wall_frames=len(w), median_wall_m=float(np.median(w)) if len(w) else None)
        return out

    # ------------------------------------------------------------ remaining correction
    def fit_correction(self, frames, base):
        """One rigid world-frame correction (about the scene origin) to the map."""
        rng = np.random.default_rng(1)
        data = []
        for f in frames:
            r, g, w = self.map_residuals(f, f.poses[base])
            for m in (g, w):
                ids = np.flatnonzero(m)
                if len(ids) > 300:
                    ids = rng.choice(ids, 300, replace=False)
                if len(ids):
                    data.append((transform(f.local[ids].astype(float), f.poses[base]), 1 / len(ids)))
        if len(data) < 10:
            return None
        X = np.concatenate([x for x, _ in data]); w0 = np.concatenate([np.full(len(x), w) for x, w in data])
        D = np.eye(4)
        for gate in (1., .6, .4):
            for _ in range(20):
                x = transform(X, D)
                d, j = self.target.tree.query(x, workers=8)
                n = self.target.normals[j]
                r = np.sum(n * (x - self.target.plane_points[j]), 1)
                ok = d < gate
                A = np.c_[np.cross(x[ok] / 30., n[ok]), n[ok]]
                w = w0[ok] * np.minimum(1, .10 / np.maximum(np.abs(r[ok]), 1e-12))
                step = np.linalg.lstsq(A.T @ (w[:, None] * A), -(A.T @ (w * r[ok])), rcond=None)[0]
                step[:3] /= 30.
                T = np.eye(4); T[:3, :3] = Rotation.from_rotvec(step[:3]).as_matrix(); T[:3, 3] = step[3:]
                D = T @ D
                if np.linalg.norm(step) < 1e-7:
                    break
        return D

    def fit_body_correction(self, frames, base):
        """One rigid correction in the vehicle (LiDAR) frame: pose = base @ delta (lever arm + mounting)."""
        rng = np.random.default_rng(1)
        data = []
        for f in frames:
            r, g, w = self.map_residuals(f, f.poses[base])
            for m in (g, w):
                ids = np.flatnonzero(m)
                if len(ids) > 300:
                    ids = rng.choice(ids, 300, replace=False)
                if len(ids):
                    data.append((f.local[ids].astype(float), f.poses[base], 1 / len(ids)))
        if len(data) < 10:
            return None
        delta = np.eye(4)
        for gate in (1., .6, .4):
            for _ in range(20):
                H, b = np.zeros((6, 6)), np.zeros(6)
                for p, M, w0 in data:
                    q = transform(p, delta)
                    x = transform(q, M)
                    d, j = self.target.tree.query(x, workers=8)
                    ok = d < gate
                    if not ok.any():
                        continue
                    n = self.target.normals[j[ok]]
                    r = np.sum(n * (x[ok] - self.target.plane_points[j[ok]]), 1)
                    m = n @ M[:3, :3]
                    A = np.c_[np.cross(q[ok] / 30., m), m]
                    w = w0 * np.minimum(1, .10 / np.maximum(np.abs(r), 1e-12))
                    H += A.T @ (w[:, None] * A); b += A.T @ (w * r)
                step = np.linalg.lstsq(H, -b, rcond=None)[0]
                step[:3] /= 30.
                T = np.eye(4); T[:3, :3] = Rotation.from_rotvec(step[:3]).as_matrix(); T[:3, 3] = step[3:]
                delta = T @ delta
                if np.linalg.norm(step) < 1e-7:
                    break
        return delta

    def body_corrections(self, base):
        folds, full = {}, {}
        for v in (1, 2, 3):
            frames = [f for f in self.frames if f.agent == v]
            d = self.fit_body_correction(frames, base)
            full[f'vehicle {v}'] = describe(d) if d is not None else None
            for t in range(1, 6):
                folds[(v, t)] = self.fit_body_correction([f for f in frames if f.take != t], base)
        name = f'{base}+loto_body_correction'
        for f in self.frames:
            d = folds[(f.agent, f.take)]
            f.poses[name] = f.poses[base] @ (d if d is not None else np.eye(4))
        spread = {}
        for v in (1, 2, 3):
            Ds = [folds[(v, t)] for t in range(1, 6) if folds[(v, t)] is not None]
            t_ = np.array([D[:3, 3] for D in Ds]); a_ = np.array([Rotation.from_matrix(D[:3, :3]).as_euler('xyz', degrees=True) for D in Ds])
            spread[f'vehicle {v}'] = dict(translation_mean_m=t_.mean(0).tolist(), translation_std_m=t_.std(0).tolist(), rpy_mean_deg=a_.mean(0).tolist(), rpy_std_deg=a_.std(0).tolist())
        self.results[f'body_corrections_{base}'] = dict(full=full, folds=spread)
        print('BODY', json.dumps(spread), flush=True)
        return name

    def corrections(self, base):
        out, folds = {}, {}
        for v in (1, 2, 3):
            frames = [f for f in self.frames if f.agent == v]
            full = self.fit_correction(frames, base)
            out[f'vehicle {v}'] = describe(full) if full is not None else None
            for t in range(1, 6):
                D = self.fit_correction([f for f in frames if f.take != t], base)
                folds[(v, t)] = D
        name = f'{base}+loto_vehicle_correction'
        for f in self.frames:
            D = folds[(f.agent, f.take)]
            f.poses[name] = (D if D is not None else np.eye(4)) @ f.poses[base]
        spread = {}
        for v in (1, 2, 3):
            Ds = [folds[(v, t)] for t in range(1, 6) if folds[(v, t)] is not None]
            spread[f'vehicle {v}'] = dict(fold_translation_m=[D[:3, 3].tolist() for D in Ds], fold_rpy_deg=[Rotation.from_matrix(D[:3, :3]).as_euler('xyz', degrees=True).tolist() for D in Ds])
        self.results[f'corrections_{base}'] = dict(full=out, folds=spread)
        return name

    # ------------------------------------------------------------ run
    def run(self):
        self.load()
        names = ['released_D', 'geodetic_gnss_time', 'geodetic_lidar_time']
        corrected = self.corrections('geodetic_lidar_time')
        names.append(corrected)
        names.append(self.body_corrections('geodetic_lidar_time'))
        rows = self.score_map(names)
        pairs = self.score_pairs(self.frames, names)
        self.rows = rows
        self.results['map_scores'] = {n: self.summarize(v) for n, v in rows.items()}
        self.results['pair_scores'] = {n: dict(pairs=len(v), median_abs_m=float(np.median([r['median_abs_m'] for r in v])) if v else None, by_pair={p: float(np.median([r['median_abs_m'] for r in v if r['pair'] == p])) for p in ('1-2', '1-3', '2-3') if any(r['pair'] == p for r in v)}) for n, v in pairs.items()}
        self.results['map_rows'] = rows
        rel = self.results['pair_scores']['released_D']['by_pair']
        verdict = {}
        for v in (2, 3):
            s = self.results['map_scores']['geodetic_lidar_time'][f'vehicle {v}']
            geo_pairs = self.results['pair_scores']['geodetic_lidar_time']['by_pair']
            pair_ok = all(geo_pairs.get(p, 9) <= rel.get(p, 9) for p in rel if str(v) in p)
            verdict[f'vehicle {v}'] = dict(road_ok=s['median_abs_road_height_m'] is not None and s['median_abs_road_height_m'] <= .15, stability_ok=s['median_within_take_std_m'] is not None and s['median_within_take_std_m'] <= .10, pairs_not_worse=pair_ok)
            verdict[f'vehicle {v}']['supported'] = all(verdict[f'vehicle {v}'].values())
        self.results['decision'] = verdict
        self.write('results.json', self.results)
        for n in names:
            print('MAP', n, json.dumps(self.results['map_scores'][n]), flush=True)
            print('PAIRS', n, json.dumps(self.results['pair_scores'][n]), flush=True)
        print('RTK', json.dumps(self.results['rtk_status_fraction']), flush=True)
        print('CORRECTIONS', json.dumps(self.results['corrections_geodetic_lidar_time']['full']), flush=True)
        print('DECISION', verdict, flush=True)
        self.figures_time(names)
        print('DONE', flush=True)

    def figures_time(self, names):
        colors = {'released_D': '#2a78d6', 'geodetic_gnss_time': '#eda100', 'geodetic_lidar_time': '#1baf7a', 'geodetic_lidar_time+loto_vehicle_correction': '#e87ba4', 'geodetic_lidar_time+loto_body_correction': '#4a3aa7'}
        fig, axes = plt.subplots(5, 3, figsize=(20, 19), layout='constrained', squeeze=False)
        for t in range(1, 6):
            for v in (1, 2, 3):
                ax = axes[t - 1, v - 1]
                for n in names:
                    sub = sorted([r for r in self.rows[n] if r['take'] == t and r['agent'] == v and r['ground_signed'] is not None], key=lambda r: r['seconds'])
                    ax.plot([r['seconds'] for r in sub], [r['ground_signed'] for r in sub], '.-', ms=4, lw=1, color=colors.get(n, '#4a3aa7'), label=n)
                ax.axhspan(-.1, .1, color='#e4e3df', zorder=0)
                ax.axhline(0, color='#0b0b0b', lw=.6)
                ax.set(ylim=(-.8, .8), title=f'take {t}, {UNIT_NAMES[v]}', xlabel='seconds from release ICP frame', ylabel='road height, LiDAR - map (m)')
                if t == 1 and v == 1:
                    ax.legend(fontsize=7)
        fig.suptitle('Frame median road height against GSplat over time. Geodetic poses use raw RTK GNSS/INS and the map georeference only (no D, no ICP, no infrastructure). Gray band +/-0.1 m.', fontsize=12)
        fig.savefig(self.figures / 'road-height-over-time.png', dpi=120)
        plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=BASE / 'geodetic-vehicle-map-tie')
    GeodeticTieExperiment(parser.parse_args().output).run()

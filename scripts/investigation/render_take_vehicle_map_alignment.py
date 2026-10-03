"""Per take and sensor: how well CooperScene LiDAR lands on the GSplat map.

Composition for every frame: map_T_lidar = D @ W @ released_ENU_T_lidar, with the
provisional shared D (shared-enu-calibration/calibration.json) and released
poses unchanged. Each LiDAR point is compared with the GSplat surface (opacity
> 0.05, local-centroid plane, local thickness < 0.10 m) along the map normal.

Outputs one figure per take (top view coloured by signed residual, plus residual
over time for every cached frame), a take x sensor summary grid and index.html.
Geometric consistency with the map, not independent ground truth.
"""
from __future__ import annotations

import argparse
import html
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.investigation.calibrate_shared_enu_map import transform, voxel
from scripts.investigation.design_combined_enu_calibration import BASE, SELECTION, MAP_COLOR, UNIT_NAMES, CONFIG as MAP_CONFIG, MapSurface
from scripts.investigation.diagnose_vehicle_pose_timing import ICP_FRAME, PoseTimeline
from scripts.investigation.fit_vehicle_sensor_corrections import Frame, VehicleCorrectionExperiment
from src.localization.gaussian_map import load_gaussian_map

GATE_M = 1.0
LIMIT_M = 0.5


class MapAlignmentAtlas:
    def __init__(self, output: Path):
        self.output = output
        self.figures = output / 'figures'
        self.figures.mkdir(parents=True, exist_ok=True)
        cal = json.loads((BASE / 'shared-enu-calibration/calibration.json').read_text())
        self.D = np.asarray(cal['D'])
        g = load_gaussian_map(cal['map_path'])
        p = MAP_CONFIG['primary_target']
        self.target = MapSurface(g, p['min_opacity']).target(p['plane_point'], p['max_local_sigma_m'])
        keep = np.isfinite(g.means).all(1) & (g.opacities > .05) & (np.abs(g.means).max(1) <= 1e4)
        self.map_points = voxel(g.means[keep].astype(float), .25)
        self.timelines = {(int(t), int(a)): PoseTimeline(r) for t, d in json.loads((BASE / 'vehicle-pose-timing/timelines.json').read_text()).items() for a, r in d.items()}

    def load(self):
        loader = VehicleCorrectionExperiment.__new__(VehicleCorrectionExperiment)
        loader.output, loader.results = BASE / 'vehicle-sensor-corrections', {}
        loader.load_frames()
        frames = loader.frames
        for f in frames:
            f.seconds = self.timelines[(f.take, f.agent)].seconds_from(f.frame_id, ICP_FRAME[f.take])
        # Stationary infrastructure: reserved scans and three-frame aggregates already in W @ released ENU.
        with np.load(SELECTION / 'selected-clouds.npz') as s, np.load(BASE / 'shared-enu-calibration/input-clouds.npz') as a:
            infra = [(int(k.split('_')[1]), k.split('_')[3], 'reserved', a[k]) for k in a.files if k.startswith('hold_') and k.split('_')[2] == '0']
            infra += [(t, f'aggregate_t{t}', 'aggregate', s[f'cal_{t}_0']) for t in range(1, 6)]
        for take, fid, role, pts in infra:
            f = Frame(take, 0, fid, 0, role, pts.astype(np.float32), dict(released=np.eye(4)))
            f.prepare()
            f.seconds = (int(fid) - ICP_FRAME[take]) / 10 if fid.isdigit() else 0.
            frames.append(f)
        self.frames = frames
        print('frames', len(frames), flush=True)

    def residuals(self, f):
        x = transform(transform(f.local.astype(float), f.poses['released']), self.D)
        d, j = self.target.tree.query(x, workers=8)
        r = np.sum(self.target.normals[j] * (x - self.target.plane_points[j]), 1)
        return x, d < GATE_M, r

    def measure(self):
        rows = []
        self.cache = {}
        for f in self.frames:
            x, ok, r = self.residuals(f)
            row = dict(take=f.take, sensor=f.agent, frame=f.frame_id, role=f.role, seconds=f.seconds, map_supported_fraction=float(ok.mean()))
            for c in ['ground', 'vertical']:
                m = ok & (f.cls == c)
                row[f'{c}_points'] = int(m.sum())
                row[f'{c}_median_signed_m'] = float(np.median(r[m])) if m.sum() >= 50 else None
                row[f'{c}_median_abs_m'] = float(np.median(np.abs(r[m]))) if m.sum() >= 50 else None
            rows.append(row)
            self.cache[(f.take, f.agent, f.frame_id, f.role)] = (x, ok, r)
        self.rows = rows
        summary = {}
        for t in range(1, 6):
            for s in range(4):
                sub = [r for r in rows if r['take'] == t and r['sensor'] == s]
                g = [abs(r['ground_median_signed_m']) for r in sub if r['ground_median_signed_m'] is not None]
                w = [r['vertical_median_abs_m'] for r in sub if r['vertical_median_abs_m'] is not None]
                summary[f'{t}|{s}'] = dict(frames=len(sub), ground_frames=len(g), ground_height_median_m=float(np.median(g)) if g else None, ground_within_010m_fraction=float(np.mean(np.array(g) < .1)) if g else None, wall_frames=len(w), wall_median_m=float(np.median(w)) if w else None, wall_within_020m_fraction=float(np.mean(np.array(w) < .2)) if w else None)
        self.summary = summary
        (self.output / 'measurements.json').write_text(json.dumps(dict(composition='map_T_lidar = D @ W @ released_ENU_T_lidar; D = shared-enu-calibration provisional tie', D=self.D.tolist(), gate_m=GATE_M, rows=rows, summary=summary), indent=1) + '\n')

    def representative(self, take, sensor):
        cands = [f for f in self.frames if f.take == take and f.agent == sensor and f.role != 'aggregate']
        return min(cands, key=lambda f: abs(f.seconds)) if cands else None

    def take_figure(self, take):
        fig = plt.figure(figsize=(22, 21), layout='constrained')
        gs = fig.add_gridspec(4, 3, width_ratios=[1.15, 1, 1])
        norm = TwoSlopeNorm(vcenter=0, vmin=-LIMIT_M, vmax=LIMIT_M)
        bg = self.map_points[(np.abs(self.map_points[:, :2]) < 80).all(1)]
        bg = bg[np.random.default_rng(0).random(len(bg)) < .4]
        sc = None
        for s in range(4):
            f = self.representative(take, s)
            ax = fig.add_subplot(gs[s, 0])
            ax.scatter(bg[:, 0], bg[:, 1], s=.2, c=MAP_COLOR, alpha=.25, lw=0, rasterized=True)
            if f is not None:
                x, ok, r = self.cache[(f.take, f.agent, f.frame_id, f.role)]
                ax.scatter(x[~ok, 0], x[~ok, 1], s=.6, c='#c3c2b7', lw=0, rasterized=True)
                order = np.argsort(np.abs(r[ok]))
                sc = ax.scatter(x[ok][order, 0], x[ok][order, 1], s=1.4, c=r[ok][order], cmap='RdBu_r', norm=norm, lw=0, rasterized=True)
                ax.set_title(f'{UNIT_NAMES[s]}, frame {f.frame_id} ({f.seconds:+.1f} s from ICP frame)\ncolour = LiDAR minus map surface; tan = no map surface within 1 m', fontsize=9)
            ax.set(xlim=(-75, 75), ylim=(-75, 75), aspect='equal', xlabel='map X (m)', ylabel='map Y (m)')
            sub = sorted([r for r in self.rows if r['take'] == take and r['sensor'] == s], key=lambda r: r['seconds'])
            for col, (key, label, band) in enumerate([('ground_median_signed_m', 'road height, LiDAR - map (m)', .1), ('vertical_median_abs_m', '|wall separation| to map (m)', .2)], start=1):
                ax = fig.add_subplot(gs[s, col])
                pts = [(r['seconds'], r[key], r['role']) for r in sub if r[key] is not None]
                for role, marker, color in [('trajectory', 'o', '#2a78d6'), ('reserved', 's', '#eb6834'), ('window', '.', '#4a3aa7'), ('aggregate', 'D', '#1baf7a')]:
                    p = [(t, v) for t, v, rr in pts if rr == role]
                    if p:
                        ax.scatter(*zip(*p), marker=marker, s=26 if marker != '.' else 36, color=color, label=role, zorder=3)
                ax.axvline(0, color='#0b0b0b', lw=.8, ls=':')
                ax.axhspan(-band if col == 1 else 0, band, color='#e4e3df', zorder=0)
                if col == 1:
                    ax.axhline(0, color='#0b0b0b', lw=.6)
                ax.set(ylim=(-.8, .8) if col == 1 else (0, .8), xlabel='seconds from the release ICP fitting frame', ylabel=label, title=f'{UNIT_NAMES[s]}: frame medians, gray band {band:g} m')
                ax.title.set_fontsize(9)
                if s == 0 and col == 1:
                    ax.legend(fontsize=7, loc='upper right')
        if sc is not None:
            fig.colorbar(sc, ax=fig.axes[-3], location='bottom', shrink=.8, label='signed distance, LiDAR minus GSplat surface (m)')
        fig.suptitle(f'Take {take}: map_T_lidar = D @ W @ released_ENU_T_lidar with one shared provisional D. Dotted line: release ICP fitting frame {ICP_FRAME[take]}.', fontsize=13)
        path = self.figures / f'take{take}-alignment.png'
        fig.savefig(path, dpi=110)
        plt.close(fig)
        return path.name

    def summary_figure(self):
        fig, axes = plt.subplots(1, 3, figsize=(20, 6.5), layout='constrained')
        for ax, key, label, fmt, cmap, vmax in [(axes[0], 'ground_height_median_m', 'median |road height| LiDAR - map (m)', '{:.2f}', 'Purples', .4), (axes[1], 'wall_median_m', 'median |wall separation| to map (m)', '{:.2f}', 'Purples', .5), (axes[2], 'ground_within_010m_fraction', 'fraction of frames with |road height| < 0.1 m', '{:.0%}', 'Greens', 1.)]:
            M = np.full((5, 4), np.nan)
            for t in range(1, 6):
                for s in range(4):
                    v = self.summary[f'{t}|{s}'][key]
                    M[t - 1, s] = np.nan if v is None else v
            im = ax.imshow(M, cmap=cmap, vmin=0, vmax=vmax)
            for t in range(5):
                for s in range(4):
                    info = self.summary[f'{t + 1}|{s}']
                    text = 'n/a' if np.isnan(M[t, s]) else fmt.format(M[t, s])
                    n = info['ground_frames'] if key != 'wall_median_m' else info['wall_frames']
                    ax.text(s, t, f'{text}\n({n} frames)', ha='center', va='center', fontsize=8, color='#0b0b0b' if np.isnan(M[t, s]) or M[t, s] < .6 * vmax else 'white')
            ax.set_xticks(range(4), [UNIT_NAMES[s] for s in range(4)])
            ax.set_yticks(range(5), [f'take {t}' for t in range(1, 6)])
            ax.set_title(label)
            fig.colorbar(im, ax=ax, shrink=.8)
        fig.suptitle('Per take and sensor, all cached frames, one shared provisional D. Geometric agreement with GSplat, not ground truth.', fontsize=12)
        fig.savefig(self.figures / 'take-sensor-summary.png', dpi=130)
        plt.close(fig)

    def index(self, takes):
        def cell(t, s):
            v = self.summary[f'{t}|{s}']
            g = 'n/a' if v['ground_height_median_m'] is None else f"{v['ground_height_median_m']:.2f} m"
            w = 'n/a' if v['wall_median_m'] is None else f"{v['wall_median_m']:.2f} m"
            frac = 'n/a' if v['ground_within_010m_fraction'] is None else f"{v['ground_within_010m_fraction']:.0%}"
            return f'<td>{g}<br>{w}<br>{frac}</td>'
        rows = ''.join(f'<tr><th>Take {t}</th>' + ''.join(cell(t, s) for s in range(4)) + '</tr>' for t in range(1, 6))
        sections = ''.join(f'<section><h2>Take {t}</h2><a href="figures/{name}"><img src="figures/{name}" alt="Take {t} alignment" loading="lazy"></a></section>' for t, name in takes)
        page = f'''<!doctype html><html><head><meta charset="utf-8"><title>Take and sensor map alignment</title>
<style>body{{font:14px system-ui;margin:0 auto;max-width:1500px;padding:16px;background:#fcfcfb;color:#0b0b0b}}img{{max-width:100%;border:1px solid #e4e3df}}table{{border-collapse:collapse}}td,th{{border:1px solid #e4e3df;padding:6px 10px;text-align:center}}.note{{color:#52514e}}</style></head><body>
<h1>CooperScene to GSplat alignment per take and sensor</h1>
<p>Every frame uses <code>map_T_lidar = D @ W @ released_ENU_T_lidar</code> with one shared provisional <code>D</code>; released poses unchanged. Points are compared with the GSplat surface along its normal.</p>
<p class="note">Each cell: median |road height| to map, median |wall separation| to map, fraction of frames with |road height| under 0.1 m. Geometric agreement with the map, not independent ground truth. The map surface itself is about 0.35 m thick, so wall separations below roughly 0.1-0.2 m are not resolvable.</p>
<table><tr><th></th>{''.join(f'<th>{UNIT_NAMES[s]}</th>' for s in range(4))}</tr>{rows}</table>
<section><h2>Summary</h2><img src="figures/take-sensor-summary.png" alt="summary grid"></section>{sections}</body></html>'''
        (self.output / 'index.html').write_text(page)

    def run(self):
        self.load()
        self.measure()
        self.summary_figure()
        takes = [(t, self.take_figure(t)) for t in range(1, 6)]
        self.index(takes)
        print('DONE', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=BASE / 'take-vehicle-map-alignment')
    MapAlignmentAtlas(parser.parse_args().output).run()

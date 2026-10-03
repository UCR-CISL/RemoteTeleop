"""Render figures and an HTML index for design_combined_enu_calibration.py outputs.

Reads stored transforms (never rounded report values). Map points are GSplat
centers with opacity > 0.05; source clouds keep frozen released poses.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.investigation.calibrate_shared_enu_map import transform, voxel
from scripts.investigation.design_combined_enu_calibration import BASE, SELECTION, UNIT_COLORS, UNIT_NAMES, MAP_COLOR
from src.localization.gaussian_map import load_gaussian_map

VIEW = [('released_W_only', 'Before: released ENU + original W'), ('overlap_selected', 'Previous: overlap-selected fit'), ('all_pool_balanced', 'Candidate: balanced all-pool fit')]
SERIES = ['previous_shared', 'overlap_selected', 'prior_set_balanced', 'designed_set_balanced', 'all_pool_balanced']
SERIES_COLORS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#4a3aa7']
plt.rcParams.update({'font.size': 9, 'axes.titlesize': 10, 'axes.edgecolor': '#52514e', 'axes.labelcolor': '#0b0b0b', 'xtick.color': '#52514e', 'ytick.color': '#52514e', 'axes.grid': True, 'grid.color': '#e4e3df', 'grid.linewidth': .6})


class CalibrationFigures:
    def __init__(self, folder: Path):
        self.folder = folder
        self.fig = folder / 'figures'
        self.fig.mkdir(exist_ok=True)
        self.results = json.loads((folder / 'results.json').read_text())
        self.T = {k: np.asarray(v) for k, v in json.loads((folder / 'transforms.json').read_text()).items()}
        shared = json.loads((BASE / 'shared-enu-calibration/calibration.json').read_text())
        g = load_gaussian_map(shared['map_path'])
        keep = np.isfinite(g.means).all(1) & (g.opacities > .05) & (np.abs(g.means).max(1) <= 1e4)
        self.map = voxel(g.means[keep].astype(float), .15)
        with np.load(SELECTION / 'selected-clouds.npz') as s, np.load(BASE / 'shared-enu-calibration/input-clouds.npz') as a:
            self.prior = {0: s['infra_representative'], 1: s['global_1'], 2: s['global_2'], 3: s['global_3']}
            manifest = json.loads((BASE / 'shared-enu-calibration/input-manifest.json').read_text())
            self.heldout = {u: np.concatenate([a[r['cache_key']] for r in manifest if r['partition'] == 'heldout' and r['agent'] == u]) for u in range(4)}
            self.pool = {0: np.concatenate([s['infra_representative']] + [s[f'cal_{t}_0'] for t in range(1, 6)])}
            for u in range(1, 4):
                self.pool[u] = np.concatenate([s[f'cal_{t}_{u}'] for t in range(1, 6)] + [a[f'cal_{t}_{u}'] for t in range(1, 6)])
        self.rois = {r['id']: r for r in json.loads((ROOT / 'scripts/investigation/cooperscene_structure_rois.json').read_text())['rois']}

    def save(self, fig, name):
        fig.savefig(self.fig / name, dpi=150)
        plt.close(fig)
        print('wrote', name, flush=True)

    # ------------------------------------------------------- overview
    def overview(self, clouds, name, title):
        rng = np.random.default_rng(0)
        mp = self.map[(np.abs(self.map[:, :2]) < 75).all(1)]
        mp = mp[rng.random(len(mp)) < .5]
        order = [1, 2, 3, 0]  # stationary infrastructure drawn last so it stays visible
        fig, axes = plt.subplots(1, 3, figsize=(21, 7.6), layout='constrained')
        for ax, (key, label) in zip(axes, VIEW):
            ax.scatter(mp[:, 0], mp[:, 1], s=.2, c=MAP_COLOR, alpha=.25, rasterized=True, lw=0)
            for u in order:
                p = transform(clouds[u], self.T[key])
                p = p[rng.random(len(p)) < min(1, 40000 / len(p))]
                ax.scatter(p[:, 0], p[:, 1], s=.6, c=UNIT_COLORS[u], alpha=.55, rasterized=True, lw=0, label=UNIT_NAMES[u])
            ax.add_patch(Rectangle((-75, 0), 150, 4, fill=False, ec='#0b0b0b', lw=.8, ls='--'))
            ax.add_patch(Rectangle((-2, -75), 4, 150, fill=False, ec='#0b0b0b', lw=.8, ls=':'))
            ax.set(xlim=(-70, 70), ylim=(-70, 70), aspect='equal', title=label, xlabel='Map X (m)', ylabel='Map Y (m)')
        axes[0].legend(loc='lower left', markerscale=12, fontsize=8, framealpha=.9)
        fig.suptitle(title + ' - top view (GSplat gray). Dashed/dotted outlines mark the side slabs.', fontsize=12)
        self.save(fig, name + '-top.png')
        fig, axes = plt.subplots(6, 1, figsize=(18, 17), layout='constrained')
        for s_idx, (a, b, lo, hi, axis_name, slab) in enumerate([(0, 1, 0, 4, 'X', 'dashed slab Y in [0, 4] m'), (1, 0, -2, 2, 'Y', 'dotted slab X in [-2, 2] m')]):
            for v_idx, (key, label) in enumerate(VIEW):
                ax = axes[s_idx * 3 + v_idx]
                sel = lambda p: p[(p[:, b] >= lo) & (p[:, b] <= hi)]
                m = sel(self.map)
                ax.scatter(m[:, a], m[:, 2], s=1.5, c=MAP_COLOR, alpha=.5, rasterized=True, lw=0)
                for u in order:
                    q = sel(transform(clouds[u], self.T[key]))
                    ax.scatter(q[:, a], q[:, 2], s=2, c=UNIT_COLORS[u], alpha=.75, rasterized=True, lw=0, label=UNIT_NAMES[u])
                ax.set(xlim=(-65, 65), ylim=(-4.5, 1), xlabel=f'Map {axis_name} (m)', ylabel='Map Z (m)', title=f'{label}: {slab} (vertical axis zoomed to the road band)')
                if s_idx == 0 and v_idx == 0:
                    ax.legend(loc='upper left', markerscale=6, fontsize=8, ncol=4)
        fig.suptitle(title + ' - side slabs', fontsize=12)
        self.save(fig, name + '-side.png')

    def per_sensor(self):
        fig, axes = plt.subplots(2, 5, figsize=(24, 10), layout='constrained')
        D = self.T['all_pool_balanced']
        rng = np.random.default_rng(1)
        mp = self.map[(np.abs(self.map[:, :2]) < 75).all(1)]
        mp = mp[rng.random(len(mp)) < .35]
        for row, (clouds, label) in enumerate([(self.prior, 'starting 4-cloud set'), (self.pool, 'all 36 pooled calibration items')]):
            for col in range(5):
                ax = axes[row, col]
                ax.scatter(mp[:, 0], mp[:, 1], s=.2, c=MAP_COLOR, alpha=.25, rasterized=True, lw=0)
                units = range(4) if col == 4 else [col]
                for u in units:
                    p = transform(clouds[u], D)
                    p = p[rng.random(len(p)) < min(1, 50000 / len(p))]
                    ax.scatter(p[:, 0], p[:, 1], s=.6, c=UNIT_COLORS[u], alpha=.6, rasterized=True, lw=0, label=UNIT_NAMES[u])
                ax.set(xlim=(-70, 70), ylim=(-70, 70), aspect='equal', title=f'{"combined" if col == 4 else UNIT_NAMES[col]} - {label}', xlabel='Map X (m)', ylabel='Map Y (m)')
                if col == 4:
                    ax.legend(loc='lower left', markerscale=12, fontsize=8)
        fig.suptitle('Individual sensor coverage under the candidate transform (frozen released poses; map gray)', fontsize=12)
        self.save(fig, 'per-sensor-coverage.png')

    # ------------------------------------------------------- crops
    def basis(self, roi):
        n = np.asarray(roi['normal'], float); n /= np.linalg.norm(n)
        u = np.asarray(roi['tangent_u'], float); u -= n * (u @ n); u /= np.linalg.norm(u)
        return np.asarray(roi['anchor'], float), np.c_[u, np.cross(n, u), n], np.asarray(roi['half_widths_uvn'], float)

    def crops(self, clouds, kind, name, title):
        ids = [k for k, r in self.rois.items() if r['kind'] == kind]
        fig, axes = plt.subplots(len(VIEW), len(ids), figsize=(4.4 * len(ids), 3.4 * len(VIEW)), layout='constrained', squeeze=False)
        for col, rid in enumerate(ids):
            anchor, B, hw = self.basis(self.rois[rid])
            for row, (key, label) in enumerate(VIEW):
                ax = axes[row, col]
                Dinv = np.linalg.inv(self.T[key])
                def local(p):
                    q = (p - anchor) @ B
                    keep = (np.abs(q[:, 0]) <= hw[0] + 1) & (np.abs(q[:, 1]) <= hw[1]) & (np.abs(q[:, 2]) <= 1.5)
                    return q[keep]
                m = local(transform(self.map, Dinv))
                ax.scatter(m[:, 0], m[:, 2], s=5, c=MAP_COLOR, alpha=.55, lw=0, label='GSplat')
                for u, p in clouds.items():
                    q = local(p)
                    if len(q):
                        ax.scatter(q[:, 0], q[:, 2], s=5, c=UNIT_COLORS[u], alpha=.75, lw=0, label=UNIT_NAMES[u])
                if len(m):
                    ax.axhline(np.median(m[:, 2]), color='#0b0b0b', lw=.8, ls='--')
                ax.set(xlim=(-hw[0] - 1, hw[0] + 1), ylim=(-1.5, 1.5), xlabel='along surface u (m)', ylabel='height (m)' if kind == 'road' else 'normal offset (m)')
                role = self.rois[rid]['role']
                ax.set_title(f'{rid} [{role} in earlier structure test]\n{label}', fontsize=8)
                if row == 0 and col == 0:
                    ax.legend(fontsize=7, markerscale=2, loc='upper right')
        fig.suptitle(title + '\nDashed line: median GSplat surface in the crop. Crops in the W frame; map moved by inverse D so sources stay fixed.', fontsize=11)
        self.save(fig, name)

    # ------------------------------------------------------- selection
    def coverage(self):
        sel = self.results['selection']
        sets = [('Starting overlap set', sel['prior']['items']), ('Coverage-designed set', sel['designed']['items'])]
        fig, axes = plt.subplots(1, 3, figsize=(20, 7), layout='constrained')
        rng = np.random.default_rng(2)
        from scripts.investigation.design_combined_enu_calibration import CONFIG
        names = {}
        with np.load(SELECTION / 'selected-clouds.npz') as s, np.load(BASE / 'shared-enu-calibration/input-clouds.npz') as a:
            names['infra_rep_t1_f481510'] = (0, s['infra_representative'])
            for t in range(1, 6):
                names[f'infra_agg_t{t}'] = (0, s[f'cal_{t}_0'])
                for u in range(1, 4):
                    names[f'overlap_t{t}_v{u}'] = (u, s[f'cal_{t}_{u}'])
                    names[f'aggregate_t{t}_v{u}'] = (u, a[f'cal_{t}_{u}'])
        for ax, (label, items) in zip(axes, sets + [('All 36 pooled items', list(names))]):
            for ix in range(-4, 4):
                for iy in range(-4, 4):
                    if (ix + iy) % 2 == CONFIG['calibration_tile_parity']:
                        ax.add_patch(Rectangle((ix * 20, iy * 20), 20, 20, color='#f2e8c9', zorder=0, lw=0))
            for item in items:
                u, p = names[item]
                p = p[rng.random(len(p)) < min(1, 15000 / len(p))]
                ax.scatter(p[:, 0], p[:, 1], s=.5, c=UNIT_COLORS[u], alpha=.5, lw=0, rasterized=True)
            for u in range(4):
                ax.scatter([], [], c=UNIT_COLORS[u], label=UNIT_NAMES[u])
            short = ', '.join(items) if len(items) <= 4 else f'{len(items)} items'
            ax.set(xlim=(-80, 80), ylim=(-80, 80), aspect='equal', title=f'{label}\n{short}', xlabel='W-frame X (m)', ylabel='W-frame Y (m)')
            ax.legend(loc='lower left', fontsize=8)
        fig.suptitle('Calibration footprints. Shaded 20 m tiles calibrate; white tiles are untouched validation regions.', fontsize=12)
        self.save(fig, 'selection-footprints.png')

    # ------------------------------------------------------- residuals
    def distributions(self):
        with np.load(self.folder / 'eval-points.npz') as e:
            data = {k: e[k] for k in e.files}
        fig, axes = plt.subplots(2, 3, figsize=(18, 9), layout='constrained')
        for row, (cls, cname) in enumerate([(0, 'ground'), (1, 'vertical')]):
            for col, (parity, pname) in enumerate([(0, 'calibration tiles'), (1, 'validation tiles')]):
                ax = axes[row, col]
                for name, color in zip(SERIES, SERIES_COLORS):
                    a = data[f'heldout_scan__{name}']
                    m = (a[:, 5] == cls) & (a[:, 2] == parity)
                    v = np.sort(np.abs(a[m, 0]))
                    ax.plot(v, np.linspace(0, 1, len(v)), color=color, lw=2, label=f'{name} (median {np.median(v):.3f} m)')
                ax.set(xlim=(0, .6), ylim=(0, 1), xlabel='|point-to-surface residual| (m)', ylabel='fraction of held-out points', title=f'{cname}, {pname}, 40 held-out scans (n={m.sum():,})')
                ax.legend(fontsize=7, loc='lower right')
            ax = axes[row, 2]
            for name, color in zip(SERIES, SERIES_COLORS):
                a = data[f'heldout_scan__{name}']
                m = a[:, 5] == cls
                ax.hist(a[m, 0], bins=np.linspace(-.6, .6, 81), histtype='step', color=color, lw=1.6, density=True, label=name)
            ax.axvline(0, color='#0b0b0b', lw=.8)
            ax.set(xlabel='signed residual along map normal (m; + = LiDAR above/outside map)', ylabel='density', title=f'{cname}: signed residual, all tiles')
            ax.legend(fontsize=7)
        fig.suptitle('Held-out residual distributions on a common support set (same points for every transform)', fontsize=12)
        self.save(fig, 'heldout-residual-distributions.png')

    def unit_signed(self):
        rows = self.results['validation_per_unit_take']
        fig, axes = plt.subplots(1, 2, figsize=(17, 5.5), layout='constrained')
        for ax, cls in zip(axes, ['ground', 'vertical']):
            labels = []
            for k, (name, color) in enumerate(zip(SERIES, SERIES_COLORS)):
                sub = [r for r in rows if r['transform'] == name and r['cls'] == cls]
                sub.sort(key=lambda r: (r['unit'], r['take']))
                xs = np.arange(len(sub)) + (k - 2) * .15
                ax.scatter(xs, [r['median_signed_plane_m'] for r in sub], s=22, color=color, label=name, zorder=3)
                labels = [f"{UNIT_NAMES[r['unit']][:4]}{r['unit'] if r['unit'] else ''} t{r['take']}" for r in sub]
            ax.axhline(0, color='#0b0b0b', lw=.8)
            ax.set_xticks(np.arange(len(labels)), labels, rotation=60, fontsize=7)
            ax.set(ylabel='median signed residual (m)', title=f'{cls}: held-out median signed residual per unit and take')
            ax.legend(fontsize=7)
        self.save(fig, 'heldout-signed-per-unit-take.png')

    def tiles(self):
        rows = self.results['tile_road']
        fig, axes = plt.subplots(2, 2, figsize=(14, 12), layout='constrained')
        for col, name in enumerate(['overlap_selected', 'all_pool_balanced']):
            sub = [r for r in rows if r['transform'] == name]
            for row, (key, lim, cmap, label) in enumerate([('height_m', .4, 'RdBu_r', 'signed height LiDAR - map (m)'), ('normal_angle_deg', 4, 'Purples', 'road normal disagreement (deg)')]):
                ax = axes[row, col]
                for r in sub:
                    x, y = r['tile_xy_W']
                    v = r[key]
                    if key == 'normal_angle_deg' and v > 20:
                        ax.add_patch(Rectangle((x - 5, y - 5), 10, 10, fill=False, hatch='//', ec='#8a8d91'))
                        ax.text(x, y, 'n/a', ha='center', va='center', fontsize=5.5)
                        continue
                    norm = (v + lim) / (2 * lim) if key == 'height_m' else v / lim
                    ax.add_patch(Rectangle((x - 5, y - 5), 10, 10, color=plt.get_cmap(cmap)(np.clip(norm, 0, 1)), ec='#0b0b0b' if r['tile_parity'] == 1 else 'none', lw=.6))
                    ax.text(x, y, f'{v:+.2f}' if key == 'height_m' else f'{v:.1f}', ha='center', va='center', fontsize=5.5, color='#0b0b0b')
                ax.set(xlim=(-70, 70), ylim=(-70, 70), aspect='equal', title=f'{name}: {label}', xlabel='W-frame X (m)', ylabel='W-frame Y (m)')
                sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(-lim if key == 'height_m' else 0, lim))
                fig.colorbar(sm, ax=ax, shrink=.7, label=label)
        fig.suptitle('10 m road tiles from held-out ground points; outlined tiles lie in untouched validation regions', fontsize=12)
        self.save(fig, 'road-height-normal-tiles.png')

    def stability(self):
        st = self.results['stability']
        names = [n for n in st if n != 'all_pool_balanced' and 'relative_rotation_deg' in st[n]]
        order = sorted(names, key=lambda n: (n.startswith('bootstrap'), st[n]['max_displacement_120m_box_m']))
        fig, axes = plt.subplots(1, 3, figsize=(19, 0.32 * len(order) + 2), layout='constrained', sharey=True)
        y = np.arange(len(order))
        for ax, key, label in zip(axes, ['relative_rotation_deg', 'max_displacement_120m_box_m', 'max_abs_height_change_m'], ['rotation vs candidate (deg)', 'max displacement, 120 m box (m)', 'max |height change|, 120 m box (m)']):
            vals = [st[n][key] for n in order]
            colors = ['#8a8d91' if n.startswith('bootstrap') else '#2a78d6' if n.startswith('leave_out') else '#eb6834' if n in ('previous_shared', 'overlap_selected', 'overlap_selected_reproduced', 'released_W_only') else '#1baf7a' for n in order]
            ax.scatter(vals, y, c=colors, s=26, zorder=3)
            ax.set(xlabel=label, title=label)
            ax.axvline(.1, color='#e34948', lw=.8, ls='--')
        axes[0].set_yticks(y, order, fontsize=7)
        axes[0].invert_yaxis()
        fig.suptitle('Transform variation relative to the predeclared candidate (blue: leave-one-out; orange: earlier transforms; green: other fits; gray: bootstrap). Red dashed line: 0.1 m', fontsize=11)
        self.save(fig, 'transform-stability.png')

    def conflicts(self):
        rows = [r for r in self.results['conflicts'] if 'preferred_translation_m' in r]
        comps = [('tz (m)', 'preferred_translation_m', 2, 'sandwich_std_translation_m', 1), ('tx (m)', 'preferred_translation_m', 0, 'sandwich_std_translation_m', 1), ('ty (m)', 'preferred_translation_m', 1, 'sandwich_std_translation_m', 1), ('roll (deg)', 'preferred_rotation_deg', 0, 'sandwich_std_rotation_deg', 1), ('pitch (deg)', 'preferred_rotation_deg', 1, 'sandwich_std_rotation_deg', 1), ('yaw (deg)', 'preferred_rotation_deg', 2, 'sandwich_std_rotation_deg', 1)]
        fig, axes = plt.subplots(1, 6, figsize=(24, .3 * len(rows) + 2), layout='constrained', sharey=True)
        y = np.arange(len(rows))
        for ax, (label, key, idx, skey, _) in zip(axes, comps):
            v = np.array([r[key][idx] for r in rows]); s = np.array([r[skey][idx] for r in rows])
            ax.errorbar(v, y, xerr=2 * s, fmt='o', color='#2a78d6', ms=4, ecolor='#9bbbe6', capsize=2)
            ax.axvline(0, color='#0b0b0b', lw=.8)
            ax.set(title=label, xlabel='preferred extra correction')
        axes[0].set_yticks(y, [f"{r['group']} (n={r['support']})" for r in rows], fontsize=7)
        axes[0].invert_yaxis()
        fig.suptitle('Group-preferred corrections relative to the candidate (one Gauss-Newton step on each group alone; bars = 2x sandwich std, optimistic because neighbouring points correlate)', fontsize=11)
        self.save(fig, 'constraint-conflicts.png')

    def attribution(self):
        rows = self.results['attribution']
        palette = {'consistent': '#1baf7a', 'inconsistent source geometry': '#e34948', 'ambiguous map surface': '#eda100', 'insufficient support': '#c3c2b7', 'unexplained high residual': '#4a3aa7'}
        fig, axes = plt.subplots(2, 4, figsize=(22, 11), layout='constrained')
        for ci, cls in enumerate(['ground', 'vertical']):
            for u in range(4):
                ax = axes[ci, u]
                for r in [r for r in rows if r['unit'] == u and r['cls'] == cls]:
                    x, y = r['tile_center_W']
                    ax.add_patch(Rectangle((x - 10, y - 10), 20, 20, color=palette[r['label']], alpha=.85, ec='#0b0b0b' if r['tile_parity'] == 1 else 'white', lw=.8))
                    if r['median_abs_residual_m'] is not None and r['support'] >= 50:
                        ax.text(x, y, f"{r['median_abs_residual_m']:.2f}", ha='center', va='center', fontsize=6)
                ax.set(xlim=(-80, 80), ylim=(-80, 80), aspect='equal', title=f'{UNIT_NAMES[u]} - {cls}', xlabel='W X (m)', ylabel='W Y (m)')
        for k, c in palette.items():
            axes[0, 0].scatter([], [], marker='s', c=c, label=k)
        axes[0, 0].legend(fontsize=7, loc='lower left')
        fig.suptitle('Residual attribution per 20 m tile (label + median |residual| m under the candidate; black outline = validation tile)', fontsize=12)
        self.save(fig, 'residual-attribution.png')

    def unit_ground_tiles(self):
        with np.load(self.folder / 'eval-points.npz') as e:
            data = {k: e[k] for k in e.files}
        fig, axes = plt.subplots(2, 4, figsize=(22, 11.5), layout='constrained')
        lim = .4
        for row, name in enumerate(['overlap_selected', 'all_pool_balanced']):
            a = np.concatenate([data[f'heldout_scan__{name}'], data[f'calibration_item__{name}']])
            a = a[a[:, 5] == 0]
            for u in range(4):
                ax = axes[row, u]
                b = a[a[:, 3] == u]
                tile = np.floor(b[:, 6:8] / 10).astype(int)
                keys, inverse = np.unique(tile, axis=0, return_inverse=True)
                for k, key in enumerate(keys):
                    v = b[inverse.ravel() == k, 0]
                    if len(v) < 50:
                        continue
                    med = float(np.median(v))
                    x, y = key * 10 + 5
                    ax.add_patch(Rectangle((x - 5, y - 5), 10, 10, color=plt.get_cmap('RdBu_r')(np.clip((med + lim) / (2 * lim), 0, 1)), lw=0))
                    ax.text(x, y, f'{med:+.2f}', ha='center', va='center', fontsize=5)
                ax.set(xlim=(-70, 70), ylim=(-70, 70), aspect='equal', title=f'{name}: {UNIT_NAMES[u]} ground', xlabel='W X (m)', ylabel='W Y (m)')
            fig.colorbar(plt.cm.ScalarMappable(cmap='RdBu_r', norm=plt.Normalize(-lim, lim)), ax=axes[row, :], shrink=.7, label='median signed LiDAR - map height (m)')
        fig.suptitle('Ground height disagreement per sensor, 10 m tiles (all calibration items and held-out scans pooled per sensor)', fontsize=12)
        self.save(fig, 'ground-height-per-sensor-tiles.png')

    def per_item(self):
        rows = self.results['per_item_consistency']['rows']
        fig, axes = plt.subplots(1, 3, figsize=(20, 6.5), layout='constrained')
        for ax, (cls, xkey, ykey, title) in zip(axes, [('ground', 'median_signed_m', 'median_signed_m', 'ground height: vs infrastructure (x) and vs map (y)'), ('vertical', 'median_abs_m', 'median_abs_m', 'vertical |separation|: vs infrastructure (x) and vs map (y)')]):
            for u in range(1, 4):
                for kind, marker in [('heldout', 'o'), ('pool', 's')]:
                    sub = [r for r in rows if r['unit'] == u and r['name'].startswith('heldout') == (kind == 'heldout') and r.get(f'{cls}_vs_infra', {}).get(xkey) is not None and r[f'{cls}_vs_map_all_pool_balanced'][ykey] is not None]
                    ax.scatter([r[f'{cls}_vs_infra'][xkey] for r in sub], [r[f'{cls}_vs_map_all_pool_balanced'][ykey] for r in sub], c=UNIT_COLORS[u], marker=marker, s=40, edgecolor='white', lw=.8, label=f'{UNIT_NAMES[u]} {"held-out scan" if kind == "heldout" else "calibration item"}')
            lo, hi = ax.get_xlim()
            ax.plot([-2, 2], [-2, 2], color='#8a8d91', lw=.8, ls='--')
            ax.set(xlim=(lo, hi), title=title, xlabel=f'vs stationary infrastructure (m), transform-independent', ylabel='vs GSplat under candidate (m)')
            ax.axhline(0, color='#0b0b0b', lw=.5); ax.axvline(0, color='#0b0b0b', lw=.5)
        axes[0].legend(fontsize=7)
        ax = axes[2]
        infra = [r for r in rows if r['unit'] == 0]
        sub = [r for r in rows if r['unit'] > 0 and r.get('ground_vs_infra', {}).get('median_signed_m') is not None]
        sub.sort(key=lambda r: r['ground_vs_infra']['median_signed_m'])
        ax.barh(np.arange(len(sub)), [r['ground_vs_infra']['median_signed_m'] for r in sub], color=[UNIT_COLORS[r['unit']] for r in sub], height=.7)
        ax.set_yticks(np.arange(len(sub)), [r['name'] for r in sub], fontsize=5)
        ax.axvline(0, color='#0b0b0b', lw=.6)
        ax.set(xlabel='median signed ground height vs stationary infrastructure (m)', title='Every vehicle scan/item, ground height vs infrastructure')
        fig.suptitle('Per-scan consistency. Dashed diagonal: map disagreement fully explained by disagreement with the stationary sensor.', fontsize=12)
        self.save(fig, 'per-scan-consistency.png')

    def inter_sensor_spread(self):
        """Per 10 m tile: spread of per-sensor median ground height. Nearly invariant to a shared D."""
        with np.load(self.folder / 'eval-points.npz') as e:
            data = {k: e[k] for k in e.files}
        out = {}
        fig, ax = plt.subplots(figsize=(9, 5), layout='constrained')
        for name, color in [('overlap_selected', '#eb6834'), ('all_pool_balanced', '#4a3aa7')]:
            a = np.concatenate([data[f'heldout_scan__{name}'], data[f'calibration_item__{name}']])
            a = a[a[:, 5] == 0]
            tile = np.floor(a[:, 6:8] / 10).astype(int)
            spreads, rows = [], []
            for key in np.unique(tile, axis=0):
                m = (tile == key).all(1)
                med = {int(u): float(np.median(a[m & (a[:, 3] == u), 0])) for u in range(4) if (m & (a[:, 3] == u)).sum() >= 50}
                if len(med) >= 2:
                    spreads.append(max(med.values()) - min(med.values()))
                    rows.append(dict(tile_center_W=(key * 10 + 5).tolist(), unit_medians_m=med, spread_m=spreads[-1]))
            sp = np.array(spreads)
            out[name] = dict(tiles=len(sp), median_spread_m=float(np.median(sp)), p90_spread_m=float(np.quantile(sp, .9)), fraction_over_010m=float(np.mean(sp > .1)), fraction_over_030m=float(np.mean(sp > .3)), rows=rows)
            ax.hist(sp, bins=np.linspace(0, 1.2, 25), histtype='step', lw=2, color=color, label=f'{name}: median {np.median(sp):.2f} m over {len(sp)} tiles')
        ax.axvline(.1, color='#e34948', ls='--', lw=.8)
        ax.set(xlabel='max - min of per-sensor median ground height in one 10 m tile (m)', ylabel='tiles', title='Sensors observing the same road tile disagree in height\n(shared rigid D barely changes this; red line 0.1 m)')
        ax.legend(fontsize=8)
        self.save(fig, 'inter-sensor-ground-spread.png')
        (self.folder / 'inter-sensor-ground-spread.json').write_text(json.dumps(out, indent=1) + '\n')

    def run(self):
        self.inter_sensor_spread()
        self.unit_ground_tiles()
        self.per_item()
        self.overview(self.prior, 'before-after-starting-set', 'Starting experiment: 1 infrastructure + 3 vehicle frames, frozen released poses, one common transform')
        self.overview(self.heldout, 'before-after-heldout-scans', 'Validation: 40 untouched scans across takes 1-5, same transforms')
        self.per_sensor()
        self.crops(self.prior, 'road', 'road-crops-starting-set.png', 'Road cross-sections, starting 4-cloud set')
        self.crops(self.prior, 'facade', 'facade-crops-starting-set.png', 'Facade cross-sections (top-down, normal offset), starting 4-cloud set')
        self.crops(self.heldout, 'road', 'road-crops-heldout.png', 'Road cross-sections, 40 held-out scans')
        self.crops(self.heldout, 'facade', 'facade-crops-heldout.png', 'Facade cross-sections, 40 held-out scans')
        self.coverage()
        self.distributions()
        self.unit_signed()
        self.tiles()
        self.stability()
        self.conflicts()
        self.attribution()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', type=Path, default=BASE / 'combined-cloud-calibration')
    CalibrationFigures(parser.parse_args().folder).run()

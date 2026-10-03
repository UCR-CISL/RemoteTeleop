"""Summarize the executed shared calibration and independent frame diagnostics."""
from __future__ import annotations

import html
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'artifacts/reference-refinement'
OUT = BASE / 'shared-enu-calibration'
VALIDATION = BASE / 'shared-enu-validation'


def main():
    calibration = json.loads((OUT / 'calibration.json').read_text())
    heldout = json.loads((OUT / 'heldout-metrics.json').read_text())
    summary = json.loads((VALIDATION / 'summary.json').read_text())
    checks = json.loads((VALIDATION / 'comparisons.json').read_text())
    stability = json.loads((OUT / 'leave-take-out.json').read_text())
    unit_stability = json.loads((OUT / 'leave-unit-out.json').read_text())
    before = [r for r in heldout if r['variant'] == 'original']
    after = [r for r in heldout if r['variant'] == 'shared']
    assert [(r['take'], r['agent'], r['frame_id']) for r in before] == [
        (r['take'], r['agent'], r['frame_id']) for r in after]
    groups = sorted({(r['take'], r['agent']) for r in before})
    values = []
    lines = [
        '# Shared CooperScene ENU → GSplat calibration', '',
        'One rigid transform was fitted using all three vehicles and infrastructure across takes 1–5. '
        'Released ICP refinements are retained. This establishes a map-relative calibration candidate, '
        'not independent geodetic ground truth. Production defaults and localization acceptance gates are unchanged.', '',
        '## Transform', '',
        'Column-vector convention; translation in meters. Apply the matrix below directly to points or poses '
        'already expressed in the released CooperScene ENU frame:', '',
        '```text', 'map_T_lidar = map_T_released_enu @ released_enu_T_lidar',
        np.array2string(np.asarray(calibration['map_T_released_enu']), precision=9, suppress_small=True),
        '```', '',
        'Equivalently, `map_T_released_enu = D @ original_W`. Do not apply both the final matrix and '
        '`original_W`, and do not apply it to raw INS or LiDAR-local points without the released pose.', '',
        f"Correction D Euler XYZ angles (degrees): `{calibration['D_rpy_deg']}`. "
        f"Correction translation (meters): `{calibration['D_translation_m']}`. "
        'The Z translation is about the map origin; it is not a constant road-height correction at every location.', '',
        '[Machine-readable transform and fitting diagnostics](calibration.json)', '',
        '## Data and separation', '',
        '- Original archives are read-only. Each of 20 take/unit groups contributes three calibration scans '
        'near 20%, 50%, and 80% of its original sequence. Recorded vehicle boxes are removed with 0.2 m padding.',
        '- Two separate scans near 35% and 65% per group provide 40 held-out scans. '
        'The eight previously inspected vehicle frames are explicitly excluded from calibration.',
        '- Calibration and its 40 held-out scans use each unit’s original annotations. '
        'The eight cached comparison frames have additional fused vehicle boxes and therefore a different static mask. '
        'Every before/after pair uses an identical point subset; compare aggregates only within each experiment.',
        '- Repeated infrastructure scans are voxel merged within each group. Unit/take contributions are balanced. '
        'This avoids point density domination but does not make repeated infrastructure observations independent.',
        '- Full SE(3) point-to-plane fitting uses both near-horizontal and other surface normals. '
        'The normal categories are geometric, not verified semantic road/building labels. '
        'Gaussian centers use positive opacity and 0.15 m voxelization; opacity is not calibrated physical confidence.',
        '- Offline calibration correspondence radii and robust loss are recorded in calibration.json. '
        'They are separate from the unchanged replay localization acceptance requirements.', '',
        '## Held-out geometry across all units', '',
        'Each row is the median of two held-out scans’ all-point nearest-neighbor medians, in meters. '
        'Every statically masked point contributes, including areas outside map coverage. '
        'These residuals are not sensor-position errors or ATE.', '',
        '| Take | Unit | Before | After |', '|---|---|---:|---:|',
    ]
    for take, agent in groups:
        a = float(np.median([r['all_points']['median_m'] for r in before if (r['take'], r['agent']) == (take, agent)]))
        b = float(np.median([r['all_points']['median_m'] for r in after if (r['take'], r['agent']) == (take, agent)]))
        values.append((a, b))
        lines.append(f'| {take} | {"Infrastructure" if agent == 0 else f"Vehicle {agent}"} | {a:.3f} | {b:.3f} |')
    improved = sum(b['all_points']['median_m'] < a['all_points']['median_m'] for a, b in zip(before, after))
    lines += ['', f'All-point median residual improved in **{improved}/{len(before)} held-out scans**. '
              'Supported-subset metrics and support fractions are also retained in [heldout-metrics.json](heldout-metrics.json).', '',
              '## Previously inspected frames', '',
              'Road-height separation is evaluated at the identical supported map XY before and after; '
              'no evaluation pose or road plane is refitted to improve the result.', '',
              '| Metric, median over eight frames | Before | After |', '|---|---:|---:|']
    for key, label, unit in [
        ('median_normal_angle_deg', 'Road-normal disagreement', '°'),
        ('median_absolute_supported_road_height_m', 'Absolute road-height separation', ' m'),
        ('median_frame_nn_median_m', 'Full static-cloud NN median', ' m'),
        ('median_frame_nn_p95_m', 'Full static-cloud NN p95', ' m'),
    ]:
        lines.append(f"| {label} | {summary['released'][key]:.3f}{unit} | {summary['shared_enu_map'][key]:.3f}{unit} |")
    lines += ['', '[Per-frame measurements and cloud views](../shared-enu-validation/report.md)', '',
              '## Stability and limits', '',
              f"Leaving out each take changes rotation by at most {max(r['relative_rotation_deg'] for r in stability):.3f}° "
              f"and changes transformed positions by at most {max(r['max_shift_within_30m_cross_m'] for r in stability):.3f} m "
              'on the five checked locations (origin and ±30 m along each map horizontal axis). '
              'These are sensitivity checks, not statistical uncertainty bounds. See [leave-take-out.json](leave-take-out.json).', '',
              f"Leaving out one unit changes rotation by up to {max(r['relative_rotation_deg'] for r in unit_stability):.3f}°. "
              'See [leave-unit-out.json](leave-unit-out.json). Infrastructure contributes useful constraints, '
              'but its released placement is not independently surveyed truth.', '',
              'The largest remaining supported road-height separation among the eight fixed frames is '
              '**+0.647 m (take 5, vehicle 3, frame 71)**. Take 1, vehicle 1, frame 0 worsens '
              '**−0.356 → −0.405 m**. This is not a complete height-reference correction.', '',
              'A shared rigid transform cannot correct disagreement already present between vehicles, '
              'vehicle-specific mounting errors, timing effects, or pose drift. Remaining road-height gaps '
              'and incomplete map coverage must remain visible. Normal estimation uses local geometric support; '
              'ambiguous or multilayer Gaussian surfaces remain a limitation.', '',
              'Recommendation: retain this transform as an explicit map-calibration profile for further evaluation. '
              'Do not label it independent GT or reinterpret past decimeter qualification counts without '
              'recomputing them under a clearly declared reference. No teleoperation batch was launched.', '',
              '## Visual evidence', '', '[Before/after cloud and camera gallery](index.html)', '',
              '![Held-out residuals by unit](heldout-residuals.png)', '']
    (OUT / 'report.md').write_text('\n'.join(lines))
    fig, ax = plt.subplots(figsize=(13, 6), layout='constrained')
    x = np.arange(len(groups)); values = np.asarray(values)
    ax.bar(x-.2, values[:,0], .4, label='Original ENU tie', color='#1698b5')
    ax.bar(x+.2, values[:,1], .4, label='Shared calibration', color='#db6e27')
    ax.set_xticks(x, [f'T{t}/{"infra" if a == 0 else "V"+str(a)}' for t,a in groups], rotation=55)
    ax.set_ylabel('Median of held-out scan NN medians (m)')
    ax.set_title('All units across five takes — geometry consistency, not localization ATE')
    ax.legend(); ax.grid(axis='y', alpha=.2)
    fig.savefig(OUT / 'heldout-residuals.png', dpi=150); plt.close(fig)
    sections = ['<!doctype html><meta charset="utf-8"><title>Shared ENU calibration</title>',
        '<style>body{font:18px system-ui;max-width:1600px;margin:30px auto;background:#f5f6f8;color:#18232d}img{width:100%;background:white}section{margin:45px 0}a{color:#17659b}</style>',
        '<h1>Shared CooperScene ENU → GSplat calibration</h1>',
        '<p>Original released pose versus one shared all-unit calibration. Same recorded frames; no per-frame fitting. Map consistency is not independent ground truth.</p>',
        '<p><a href="report.md">Report</a> · <a href="calibration.json">Transform</a></p>',
        '<img src="heldout-residuals.png">']
    for row in checks[::2]:
        case = f"{row['run_id']}-f{row['frame_offset']:06d}"
        sections += [f'<section><h2>{html.escape(case)}</h2>',
                     '<p>Clouds: original above; shared calibration below. Gray = GSplat.</p>',
                     f'<img loading="lazy" src="../shared-enu-validation/{case}.png">',
                     '<p>Camera: original above; shared calibration below. Columns: recorded, GSplat, overlay.</p>',
                     f'<img loading="lazy" src="../shared-enu-cameras/{case}/camera-overlays.png"></section>']
    (OUT / 'index.html').write_text('\n'.join(sections))
    print(f'Report and gallery: {OUT}', flush=True)


if __name__ == '__main__':
    main()

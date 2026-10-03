"""Check spatial road separation on fixed supported patches without fitting poses.

Uses the previously saved road inliers, so this is conditional on that original
road extraction. Local patch dispersion is a sensitivity diagnostic, not a pose
accuracy estimate or independent statistical confidence interval.
"""
from __future__ import annotations

import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import QhullError

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.investigation.compare_reference_road_planes import inside

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'artifacts/reference-refinement'
OUT = BASE / 'local-road-separation'


def local_plane(points, xy, radius=2.5):
    selected = points[np.linalg.norm(points[:, :2] - xy, axis=1) <= radius]
    # Balance projected density so repeated rays/centers do not dominate.
    _, indices = np.unique(np.floor(selected[:, :2] / .15), axis=0, return_index=True)
    selected = selected[indices]
    if len(selected) < 30:
        return None
    try:
        if not inside(selected, xy):
            return None
    except QhullError:
        return None
    centered = selected[:, :2] - xy
    if np.linalg.eigvalsh(np.cov(centered.T))[0] < .2:
        return None
    design = np.c_[centered, np.ones(len(selected))]
    weights = np.ones(len(selected))
    for _ in range(5):
        coefficients = np.linalg.lstsq(design * np.sqrt(weights[:, None]), selected[:, 2] * np.sqrt(weights), rcond=None)[0]
        residual = selected[:, 2] - design @ coefficients
        weights = np.minimum(1., .05 / np.maximum(abs(residual), 1e-9))
    normal = np.r_[-coefficients[:2], 1.]
    normal /= np.linalg.norm(normal)
    return dict(height_m=float(coefficients[2]), normal=normal.tolist(), points=len(selected),
                residual_median_m=float(np.median(abs(residual))), residual_p95_m=float(np.percentile(abs(residual), 95)))


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    calibration = json.loads((BASE / 'shared-enu-calibration/calibration.json').read_text())
    D = np.asarray(calibration['D'])
    records = json.loads((BASE / 'final-comparison/diagnostics.json').read_text())
    results = []
    fig, axes = plt.subplots(4, 2, figsize=(14, 20), layout='constrained')
    scatter = matplotlib.cm.ScalarMappable(norm=matplotlib.colors.Normalize(-1, 1), cmap='coolwarm')
    for record, ax in zip(records, axes.flat):
        case = f"{record['run_id']}-f{record['frame_offset']:06d}"
        attempt = json.loads(Path(record['attempt_path']).read_text())
        with np.load(attempt['metadata']['npz_path']) as data:
            W = data['reference_map_T_dataset']
        with np.load(BASE / 'final-comparison' / case / 'road-support.npz') as data:
            native, target = data['native_support'], data['map_support']
        clouds = {name: native @ T[:3, :3].T + T[:3, 3]
                  for name, T in [('original', W), ('shared', D @ W)]}
        anchor = np.asarray(record['comparison_xy_map'])
        patches = []
        for x in range(-12, 13, 4):
            for y in range(-12, 13, 4):
                xy = anchor + [x, y]
                planes = {'map': local_plane(target, xy)}
                planes.update({name: local_plane(points, xy) for name, points in clouds.items()})
                if any(plane is None for plane in planes.values()):
                    continue
                row = dict(xy_map=xy.tolist(), xy_relative=[x, y], planes=planes)
                for name in clouds:
                    row[name + '_height_m'] = planes[name]['height_m'] - planes['map']['height_m']
                    row[name + '_angle_deg'] = float(np.rad2deg(np.arccos(np.clip(
                        np.dot(planes[name]['normal'], planes['map']['normal']), -1., 1.))))
                patches.append(row)
        summary = dict(case=case, patches=patches, patch_count=len(patches))
        for name in clouds:
            heights = [p[name + '_height_m'] for p in patches]
            summary[name] = dict(median_signed_height_m=float(np.median(heights)),
                                 min_height_m=float(np.min(heights)), max_height_m=float(np.max(heights)),
                                 median_absolute_height_m=float(np.median(np.abs(heights)))) if heights else None
        results.append(summary)
        ax.scatter(target[:, 0] - anchor[0], target[:, 1] - anchor[1], s=.3, color='#9ca6ad', alpha=.4)
        if patches:
            xy = np.asarray([p['xy_relative'] for p in patches])
            scatter = ax.scatter(xy[:, 0], xy[:, 1], c=[p['shared_height_m'] for p in patches],
                                 cmap='coolwarm', vmin=-1, vmax=1, s=160, edgecolors='black', linewidths=.4)
            for p in patches:
                ax.annotate(f"{p['shared_height_m']:+.2f}", p['xy_relative'], xytext=(0, 9), textcoords='offset points', ha='center', fontsize=7)
        ax.set(title=f'{case}\n{len(patches)} patches supported before and after', xlim=(-16, 16), ylim=(-16, 16),
               xlabel='Map X from fixed comparison point (m)', ylabel='Map Y from fixed comparison point (m)')
        ax.set_aspect('equal'); ax.grid(alpha=.15)
        print(case, summary['shared'], flush=True)
    fig.colorbar(scatter, ax=list(axes.flat), label='Shared-calibration LiDAR road minus map road (m)', shrink=.5)
    fig.suptitle('Local road separation after shared calibration\nSame supported XY; 2.5 m patch radius; no pose fitting', fontsize=16)
    fig.savefig(OUT / 'local-road-separation.png', dpi=140); plt.close(fig)
    (OUT / 'results.json').write_text(json.dumps(dict(
        correction_D=D.tolist(), settings=dict(radius_m=2.5, grid_spacing_m=4, min_xy_voxels=30,
            min_xy_covariance_eigenvalue_m2=.2, robust_height_loss_m=.05), records=results), indent=2) + '\n')
    lines = ['# Local road separation after shared calibration', '',
             'No poses were fitted. The same previously extracted road inliers are examined in small patches. '
             'Both original and corrected source clouds, plus map points, must surround the same comparison XY. '
             'This is conditional on the original road extraction, not a fresh semantic road segmentation.', '',
             '| Case | Supported patches | Median signed separation before → after | Corrected patch range |',
             '|---|---:|---:|---:|']
    for r in results:
        a, b = r['original'], r['shared']
        lines.append(f"| {r['case']} | {r['patch_count']} | {a['median_signed_height_m']:+.3f} → {b['median_signed_height_m']:+.3f} m | {b['min_height_m']:+.3f} to {b['max_height_m']:+.3f} m |" if a and b else f"| {r['case']} | 0 | insufficient support | insufficient support |")
    lines += ['', 'Patch ranges are spatial variation, not independent confidence intervals. '
              'Normals and height are fitted locally; no flat-road assumption is imposed. Original/global road '
              'inlier selection may omit other road layers, and convex hulls can bridge gaps.', '',
              '![Local height separation](local-road-separation.png)', '']
    (OUT / 'report.md').write_text('\n'.join(lines))


if __name__ == '__main__':
    main()

"""Compare the separate height method with fresh BEV on identical frames."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, default=Path('artifacts/bev-lidar-height'))
    args = parser.parse_args()
    results = json.loads((args.directory / 'summary.json').read_text())
    fig, axes = plt.subplots(2, 2, figsize=(17, 9), layout='constrained')
    for col, dataset in enumerate(('nuscenes', 'cooperscene')):
        rows = [r for r in results if r['metadata']['dataset'] == dataset and 'methods' in r]
        x = np.arange(len(rows))
        labels = [r['run_id'].removeprefix(dataset + '-') + '/' + str(r['frame_offset']) for r in rows]
        for row, key in enumerate(('dz_m', 'xyz_m')):
            ax = axes[row, col]
            for name, delta, marker, label in [('bev', -.13, 'o', 'Fresh BEV'),
                    ('bev_lidar_height_candidate', .13, 'x', 'Height proposal (including rejected)')]:
                y = [r['methods'][name]['error'][key] for r in rows]
                ax.scatter(x + delta, y, marker=marker, s=28, label=label)
            applied = [i for i, r in enumerate(rows) if r['methods']['bev_lidar_height']['accepted']]
            ax.scatter(x[applied] + .13,
                       [rows[i]['methods']['bev_lidar_height']['error'][key] for i in applied],
                       marker='o', facecolors='none', edgecolors='green', s=85, label='Height accepted')
            ax.set_xticks(x, labels, rotation=65, ha='right', fontsize=8)
            ax.set(title=f'{dataset}: ' + ('signed vertical error' if row == 0 else 'XYZ error'),
                   ylabel='Recorded-reference error (m)', xlabel='Scene or take-agent / frame offset')
            ax.grid(alpha=.2)
            if row:
                ax.set_yscale('log')
                ax.axhline(.1, color='gray', linestyle='--', linewidth=.8)
            else:
                ax.axhline(0, color='gray', linewidth=.8)
            if col == 0 and row == 0:
                ax.legend(fontsize=8)
    fig.suptitle('Separate method: BEV + LiDAR height\nSame frames; conditional ground planes; reference agreement is not independent ground truth')
    fig.savefig(args.directory / 'comparison.png', dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    main()

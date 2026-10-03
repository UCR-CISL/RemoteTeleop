"""Plot every measured overlap score and the predeclared selection decisions."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'artifacts/reference-refinement/overlap-selected-calibration/selection'
RELEASE_ANCHORS = {1: '481693', 2: '488685', 3: '497206', 4: '515587', 5: '524102'}


def main():
    manifest = json.loads((OUT / 'manifest.json').read_text())
    with (OUT / 'all-frame-scores.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    selected = {(r['take'], r['agent']): r for r in manifest['per_take_selected']}
    global_ids = {(r['take'], r['agent'], r['frame_id']) for r in manifest['global_selected']}
    comparisons = []
    fig, axes = plt.subplots(5, 3, figsize=(17, 16), layout='constrained', sharex=True)
    for take in range(1, 6):
        for agent in range(1, 4):
            ax = axes[take - 1, agent - 1]
            group = [r for r in rows if (int(r['take']), int(r['agent'])) == (take, agent)]
            group.sort(key=lambda r: int(r['frame_offset']))
            x = np.array([float(r['fraction']) * 100 for r in group])
            coverage = np.array([float(r['infrastructure_coverage']) * 100 for r in group])
            ax.plot(x, coverage, color='#25799c', lw=1)
            reserved = np.array([bool(r['exclusion_reason']) for r in group])
            ax.scatter(x[reserved], coverage[reserved], marker='x', s=20, color='#838383', label='Reserved evaluation frame')
            winner = selected[take, agent]
            is_global = (take, agent, winner['frame_id']) in global_ids
            ax.scatter(winner['fraction'] * 100, winner['infrastructure_coverage'] * 100,
                       marker='*' if is_global else 'o', s=120 if is_global else 45,
                       color='#ce6627', zorder=4, label='Selected; star = global vehicle winner')
            anchor = next((r for r in group if r['frame_id'] == RELEASE_ANCHORS[take]), None)
            if anchor:
                ax.axvline(float(anchor['fraction']) * 100, color='#7657aa', alpha=.6, ls='--', lw=1, label='Original released ICP anchor')
            comparisons.append(dict(take=take, agent=agent, selected_frame_id=winner['frame_id'],
                selected_shared_voxels=winner['shared_voxels'], original_icp_anchor_frame_id=RELEASE_ANCHORS[take],
                original_icp_anchor_shared_voxels=int(anchor['shared_voxels']) if anchor else None))
            ax.set_title(f"Take {take} / vehicle {agent}\nselected offset {winner['frame_offset']} ({winner['fraction'] * 100:.1f}%)", fontsize=10)
            ax.set(xlim=(0, 100), ylim=(0, max(coverage.max() * 1.15, 1)))
            ax.grid(alpha=.2)
            if agent == 1:
                ax.set_ylabel('Fixed infrastructure voxels covered (%)')
            if take == 5:
                ax.set_xlabel('Sequence completed (%)')
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='outside lower center', ncol=3, fontsize=9)
    fig.suptitle('Exhaustive infrastructure-overlap selection — released poses unchanged\nOne fixed infrastructure target; no GSplat geometry or localization-error ranking', fontsize=16)
    fig.savefig(OUT / 'overlap-through-sequences.png', dpi=150)
    plt.close(fig)
    (OUT / 'original-anchor-comparison.json').write_text(json.dumps(comparisons, indent=2) + '\n')
    print(f'Plotted {len(rows)} scores across 15 sequences', flush=True)


if __name__ == '__main__':
    main()

"""Plot measured clearance transfer; no map or reference poses are loaded."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, default=Path('artifacts/clearance-transfer'))
    args = parser.parse_args()
    rows = json.loads((args.directory / 'measurements.json').read_text())
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), layout='constrained')
    for axis, dataset in zip(axes, ('nuscenes', 'cooperscene')):
        selected = [r for r in rows if r['dataset'] == dataset and r['frame_offset'] == 0]
        x = np.arange(len(selected))
        for metric, offset, label in [('lidar_normal_clearance_m', -.12, 'LiDAR'),
                                      ('camera_normal_clearance_m', .12, 'Camera')]:
            y = [r.get('transfer_difference_' + metric, np.nan) for r in selected]
            axis.scatter(x + offset, y, label=label, s=40)
        axis.axhspan(-.1, .1, color='green', alpha=.10, label='±0.1 m transfer difference')
        axis.axhline(0, color='gray', linewidth=.8)
        axis.set_xticks(x, [r['run_id'].replace('nuscenes-', '').replace('cooperscene-', '') for r in selected], rotation=30)
        axis.set(title=f'{dataset}: first-frame clearance minus fixed calibration clearance',
                 ylabel='Signed clearance difference (m)', xlabel='Scene / take-agent')
        axis.grid(alpha=.2)
        axis.legend(fontsize=8)
    fig.suptitle('Clearance transfer uses LiDAR and full camera extrinsics only\nCalibration: nuScenes 152; CooperScene take 1 separately for each agent')
    fig.savefig(args.directory / 'clearance-transfer.png', dpi=160)
    plt.close(fig)
    audit = json.loads((args.directory / 'camera-anchored-road.json').read_text())
    later = audit['later_stationarity']
    initial = next(r for r in audit['initial_planes'] if r['run_id'] == 'nuscenes-152')
    x = [0] + [r['frame_offset'] for r in later]
    measured = [initial['lidar_vertical_clearance_m']] + [r['current_plane_measured_vertical_clearance_m'] for r in later]
    predicted = [initial['lidar_vertical_clearance_m']] + [r['initial_plane_predicted_vertical_clearance_m'] for r in later]
    fig, axis = plt.subplots(figsize=(9, 4.5), layout='constrained')
    axis.plot(x, measured, 'o-', label='Current LiDAR road fit')
    axis.plot(x, predicted, 's--', label='Initial road plane extrapolated using recorded poses')
    axis.set(title='nuScenes 152: stable sensor clearance does not imply a constant road plane',
             xlabel='Frame offset', ylabel='LiDAR vertical clearance (m)')
    axis.legend()
    axis.grid(alpha=.2)
    fig.savefig(args.directory / 'initial-plane-extrapolation.png', dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    main()

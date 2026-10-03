"""Render identical recorded frames before/after an offline shared map calibration."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.investigation.compare_reference_road_planes import (
    ATTEMPTS, CASES, ReferenceRoadComparison,
)
from src.localization.gaussian_map import load_gaussian_map


class SharedCalibrationCameraReview:
    def __init__(self, calibration: Path, output: Path):
        self.calibration = json.loads(calibration.read_text())
        self.correction = np.asarray(self.calibration['D'], dtype=float)
        if self.correction.shape != (4, 4):
            raise ValueError('D must be a homogeneous target_T_source matrix')
        self.output = output
        output.mkdir(parents=True, exist_ok=True)
        self.gaussian = None
        self.map_path = None

    # Reuse the established camera composition, undistortion and render pipeline.
    label = staticmethod(ReferenceRoadComparison.label)
    camera_views = ReferenceRoadComparison.camera_views

    def run(self):
        manifest = []
        for run_id, offset in CASES:
            attempt_path = ATTEMPTS / run_id / f'f{offset:06d}.json'
            attempt = json.loads(attempt_path.read_text())
            metadata = attempt['metadata']
            with np.load(metadata['npz_path'], allow_pickle=False) as data:
                frame = dict(data)
            if self.map_path != metadata['map_path']:
                self.gaussian = load_gaussian_map(metadata['map_path'])
                self.map_path = metadata['map_path']
            original = frame['reference_map_T_dataset']
            transforms = [
                ('original', 'Released ICP + original ENU tie', original, None, '#1698b5'),
                ('shared', 'Released ICP + shared all-agent map calibration',
                 self.correction @ original, None, '#bb9d11'),
            ]
            directory = self.output / f'{run_id}-f{offset:06d}'
            directory.mkdir(exist_ok=True)
            self.camera_views(directory, metadata, frame, transforms)
            manifest.append(dict(
                run_id=run_id, frame_offset=offset, attempt_path=str(attempt_path),
                original_map_T_camera=(original @ frame['dataset_T_camera']).tolist(),
                calibrated_map_T_camera=(self.correction @ original @ frame['dataset_T_camera']).tolist(),
                independent_ground_truth=False,
            ))
            print(f'Rendered {run_id} frame {offset}', flush=True)
        (self.output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    SharedCalibrationCameraReview(args.calibration, args.output).run()

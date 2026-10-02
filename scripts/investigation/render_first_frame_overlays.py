"""Render recorded first cameras and explicitly labelled registration candidates."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import cv2
import numpy as np
from src.localization.gaussian_map import load_gaussian_map
from src.viz.localization_visualizer import render_gaussian_camera_buffers


class FirstFrameRenderer:
    def __init__(self, root: Path):
        self.root = root
        self.output = root / "camera-overlays"
        self.output.mkdir(exist_ok=True)
        self.map_path = None
        self.gaussian_map = None

    @staticmethod
    def label(image, text):
        result = cv2.copyMakeBorder(image, 32, 0, 0, 0, cv2.BORDER_CONSTANT, value=(245,245,245))
        cv2.putText(result, text, (8,22), cv2.FONT_HERSHEY_SIMPLEX, .48, (10,10,10), 1, cv2.LINE_AA)
        return result

    def render(self, metadata):
        run_id = metadata['run_id']
        frame = np.load(metadata['npz_path'])
        if self.map_path != metadata['map_path']:
            self.gaussian_map = load_gaussian_map(metadata['map_path'])
            self.map_path = metadata['map_path']
        original = cv2.imread(metadata['camera_path'])
        height, width = original.shape[:2]
        distortion = frame['camera_distortion']
        if distortion.size and np.any(distortion):
            original = cv2.undistort(original, frame['camera_intrinsic'], distortion)
        original = cv2.resize(original, (width//4, height//4))
        xyz = json.loads((self.root/'xyz'/'baseline'/f'{run_id}.json').read_text())
        bev = json.loads((self.root/'bev'/f'{run_id}_baseline.json').read_text())
        candidates = [('Inherited', frame['inherited_map_T_dataset']),
                      ('Reference (not ground truth)', frame['reference_map_T_dataset']),
                      ('Fresh XYZ '+('accepted' if xyz['accepted'] else 'REJECTED'), np.array(xyz['map_T_dataset'])),
                      ('Fresh BEV '+('accepted' if bev['accepted'] else 'REJECTED'),
                       np.array(bev['registration']['map_T_source']) if 'registration' in bev else None)]
        rows = []
        statistics = []
        for name, transform in candidates:
            if transform is None:
                blank=np.zeros_like(original)
                rows.append(np.hstack([self.label(original,run_id+' recorded'),
                    self.label(blank,name+' / no estimate'),self.label(blank,'Insufficient correspondences')]))
                statistics.append(dict(candidate=name,backend=None,reason=bev.get('exception','No estimate')))
                continue
            buffers = render_gaussian_camera_buffers(self.gaussian_map,
                transform @ frame['dataset_T_camera'], frame['camera_intrinsic'],
                (width,height), downsample=4, device='cuda')
            rendered = np.clip(buffers.rgb[...,::-1]*255,0,255).astype(np.uint8)
            overlay = cv2.addWeighted(original,.5,rendered,.5,0)
            rows.append(np.hstack([self.label(original, run_id+' recorded'),
                                  self.label(rendered, name), self.label(overlay,'50% camera / 50% GSplat')]))
            statistics.append(dict(candidate=name,backend=buffers.backend,
                alpha_above_half_fraction=float(np.mean(buffers.alpha>.5))))
        path = self.output/f'{run_id}.png'
        cv2.imwrite(str(path),np.vstack(rows))
        (self.output/f'{run_id}.json').write_text(json.dumps(dict(run_id=run_id,
            note='Visual alignment evidence only; rejected candidates are diagnostic, not accepted poses.',
            renders=statistics),indent=2)+'\n')
        print(run_id, path, flush=True)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path('artifacts/first-frame-localization'))
    args=parser.parse_args()
    renderer=FirstFrameRenderer(args.root)
    for metadata in json.loads((args.root/'inputs'/'manifest.json').read_text()):
        if not (renderer.output/f"{metadata['run_id']}.json").exists():
            renderer.render(metadata)

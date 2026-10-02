#!/usr/bin/env python3
"""Build replay queues using the best accepted observed localization per scene."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path, PurePosixPath


class LocalizationEvaluationPreparer:
    def __init__(self, manifest: Path, output: Path, remote_root: PurePosixPath):
        self.original = json.loads(manifest.read_text())
        self.output = output
        self.remote_root = remote_root

    @staticmethod
    def best_xyz(attempts: Path) -> dict:
        selected = {}
        for row in csv.DictReader(attempts.open()):
            if row['variant'] != 'baseline' or row['final_accepted'].lower() != 'true':
                continue
            if row['within_hard_cutoff'].lower() != 'true' or float(row['sequence_percent']) > 25:
                continue
            error = float(row['xyz_m'])
            if not math.isfinite(error):
                continue
            key = (error, int(row['frame_offset']))
            old = selected.get(row['original_run_id'])
            if old is None or key < old['key']:
                result = json.loads(Path(row['result_path']).read_text())
                if result.get('final_accepted', result.get('accepted')) is not True:
                    continue
                selected[row['original_run_id']] = dict(
                    key=key, frame_offset=int(row['frame_offset']), error_m=error,
                    map_T_dataset=result['map_T_dataset'], source_result=row['result_path'],
                )
        return selected

    @staticmethod
    def best_bev(selections: Path) -> dict:
        selected = {}
        for item in json.loads(selections.read_text())['sequences']:
            choice = item.get('selected')
            if not item.get('search_complete') or not choice or choice.get('accepted') is not True or choice.get('fallback_existing_alignment'):
                continue
            if float(choice['sequence_percent']) > 25:
                continue
            selected[item['run_id']] = dict(frame_offset=int(choice['frame_offset']),
                error_m=float(choice['error']['xy_m']),
                map_T_dataset=choice['registration']['map_T_source'],
                source_result=item['replay_transform_path'])
        return selected

    def prepare(self, selected: dict, *, mode: str, bbox_source: str, config: Path,
                ground_planes: dict | None = None) -> dict:
        self.output.mkdir(parents=True, exist_ok=True)
        transforms = self.output / 'transforms'
        transforms.mkdir(exist_ok=True)
        runs, coverage = [], []
        for original in self.original['runs']:
            run = dict(original)
            run_id = run['id']
            choice = selected.get(run_id)
            fallback = choice is None or choice['error_m'] >= .1
            start = 0 if fallback else choice['frame_offset']
            if not 0 <= start < run['expected_samples']:
                raise ValueError(f'{run_id}: invalid selected start')
            if ground_planes is not None and run_id not in ground_planes:
                coverage.append(dict(id=run_id, status='blocked_no_ground_plane'))
                continue
            if not fallback:
                transform_key = 'gaussian_T_nuscenes_global' if run['dataset'] == 'nuscenes' else 'gaussian_T_cooperscene'
                payload = {transform_key: choice['map_T_dataset'], 'selected_source_sequence': start,
                           'selection_error_m': choice['error_m'], 'position_mode': mode,
                           'source_result': choice['source_result']}
                (transforms / f'{run_id}.json').write_text(json.dumps(payload, indent=2) + '\n')
                option_key = 'nuscenes_localization_transform' if run['dataset'] == 'nuscenes' else 'cooperscene_localization_transform'
                run[option_key] = str(self.remote_root / 'transforms' / f'{run_id}.json')
            run.update(mcap_start_sequence=start, expected_samples=run['expected_samples'] - start,
                       localization_position_mode=mode, bbox_source=bbox_source)
            if ground_planes is not None:
                run['ground_plane'] = ground_planes[run_id]
            runs.append(run)
            coverage.append(dict(id=run_id, status='fallback_frame_zero_inherited_transform' if fallback else 'qualified',
                                 start_sequence=start, error_m=None if choice is None else choice['error_m'], expected_samples=run['expected_samples']))
        defaults = {**self.original.get('defaults', {}), 'config_path': str(config.resolve())}
        manifest = dict(version=1, defaults=defaults, runs=runs,
                        evaluation=dict(position_mode=mode, selection='minimum accepted observed error below 0.1m within first 25 percent; otherwise original frame-zero inherited transform',
                                        coverage=coverage, detector_note='CooperScene uses a nuScenes-trained detector' if bbox_source == 'bevfusion' else 'recorded boxes'))
        (self.output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--remote-root', type=PurePosixPath, required=True)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument('--xyz-attempts', type=Path)
    selection.add_argument('--bev-selections', type=Path)
    parser.add_argument('--bbox-source', choices=('gt', 'bevfusion'), required=True)
    parser.add_argument('--config', type=Path, default=Path('cfg/alien3_lambda_proxy_batch.yaml'))
    parser.add_argument('--ground-planes', type=Path, help='JSON mapping run ID to remote audited plane path')
    args = parser.parse_args()
    preparer = LocalizationEvaluationPreparer(args.manifest, args.output, args.remote_root)
    selected = preparer.best_xyz(args.xyz_attempts) if args.xyz_attempts else preparer.best_bev(args.bev_selections)
    result = preparer.prepare(selected, mode='xyz' if args.xyz_attempts else 'xy', bbox_source=args.bbox_source,
                              config=args.config, ground_planes=None if args.ground_planes is None else json.loads(args.ground_planes.read_text()))
    print(json.dumps(dict(ready=len(result['runs']), coverage=result['evaluation']['coverage']), indent=2))


if __name__ == '__main__':
    main()

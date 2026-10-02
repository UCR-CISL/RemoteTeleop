"""Chronological full-sequence first-hit evaluation with independent method stops.

Reference-error thresholds are offline evaluation labels, not online confidence.
All native frames before each hit are checked. No 15/25 percent cutoff applies.
Data failures remain incomplete rather than being mislabeled as exhaustion.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import sys

for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[name] = '2'
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import numpy as np

from scripts.investigation.advance_cooperscene_frames import IndexedCooperFrameReader
from scripts.investigation.advance_nuscenes_frames import NuScenesFrameAdvance
from scripts.investigation.evaluate_hybrid_road_height import HybridComparison


TARGETS = {
    'bev': ('bev', 'xy_m'),
    'xyz': ('xyz', 'xyz_m'),
    'bev_lidar_height': ('bev_lidar_height', 'xyz_m'),
    'bev_xyz_benchmark': ('bev', 'xyz_m'),
}


class SequenceFrames:
    """Read cached native frames and stage missing frames into this experiment."""

    def __init__(self, entry, previous, output, trajectory_root, recorded_root=None):
        self.entry, self.previous, self.output = entry, previous, output
        self.reader = None
        self.nu = None
        self.recorded_root = recorded_root
        run_id = entry['run_id']
        if entry['dataset'] == 'cooperscene':
            self.reader = IndexedCooperFrameReader(previous / 'first-frame-localization/inputs', run_id, trajectory_root)
            self.count = len(self.reader.rows)
            self.times = [float(row['timestamp_us']) / 1e6 for row in self.reader.rows]
            self.timestamp_basis = 'original replay normalized 10 Hz offsets; not sensor capture timestamps'
        else:
            self.nu = NuScenesFrameAdvance(output / 'staging/nuscenes', [], ['baseline'])
            self.nu.original = previous / 'first-frame-localization'
            for name in ('scene_info.json', 'instances/instances_info.json', 'instances/frame_instances.json', 'intrinsics/0.txt'):
                self.copy_recorded(name)
            recorded = self.nu.output / 'recorded' / entry['scene']
            if any(not (recorded / name).exists() for name in ('scene_info.json', 'instances/instances_info.json', 'instances/frame_instances.json', 'intrinsics/0.txt')):
                self.nu._fetch(entry['scene'], None)
            info = self.nu._scene(entry['scene'])
            self.count = info['frame_count']
            self.times = [index * info['timestamp_step_us'] / 1e6 for index in range(self.count)]
            self.timestamp_basis = 'processed native 10 Hz index timestamps; exposure timestamps unavailable'
        if self.count < 1 or any(b < a for a, b in zip(self.times, self.times[1:])):
            raise ValueError(f'{run_id}: invalid native sequence timestamps')

    def copy_recorded(self, name):
        destination = self.nu.output / 'recorded' / self.entry['scene'] / name
        if destination.exists():
            return
        roots = ([self.recorded_root] if self.recorded_root else []) + [
            self.previous / 'frame-advance-localization/nuscenes/recorded',
            self.previous / 'bev-frame-search/nuscenes-cache/recorded',
            ROOT / 'artifacts/hybrid-road-height/staging/nuscenes/recorded']
        for root in roots:
            source = root / self.entry['scene'] / name
            if source.exists():
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, destination)
                return

    def frame(self, offset):
        if offset == 0:
            return dict(self.entry)
        run_id, dataset = self.entry['run_id'], self.entry['dataset']
        suffix = f'{run_id}-f{offset:06d}' if dataset == 'cooperscene' else f'{run_id}-f{offset:03d}'
        candidates = [
            self.previous / f'frame-advance-localization/{dataset}/inputs' / suffix,
            self.previous / f'bev-frame-search/{dataset}-cache/inputs' / suffix,
            ROOT / f'artifacts/hybrid-road-height/staging/{dataset}/inputs' / suffix,
            self.output / f'staging/{dataset}/inputs' / suffix]
        for directory in candidates:
            path = directory / 'metadata.json'
            if path.exists():
                metadata = json.loads(path.read_text())
                if Path(metadata['npz_path']).exists():
                    return metadata
        if self.reader:
            return self.reader.extract(offset, self.output / 'staging/cooperscene/inputs')
        names = (f'lidar/{offset:03d}.bin', f'lidar_pose/{offset:03d}.txt',
                 f'extrinsics/{offset:03d}_0.txt', f'images/{offset:03d}_0.jpg')
        for name in names:
            self.copy_recorded(name)
        if any(not (self.nu.output / 'recorded' / self.entry['scene'] / name).exists() for name in names):
            self.nu._fetch(self.entry['scene'], offset)
        return self.nu._stage(self.entry['scene'], offset)

    def timing(self, offset):
        duration = self.times[-1] - self.times[0]
        elapsed = self.times[offset] - self.times[0]
        return dict(frame_offset=offset, frame_count=self.count, elapsed_seconds=elapsed,
            duration_seconds=duration, sequence_percent=100 * elapsed / duration if duration else 0.,
            timestamp_basis=self.timestamp_basis,
            frame_percent=100 * offset / (self.count - 1) if self.count > 1 else 0.)

    def close(self):
        if self.reader:
            self.reader.close()


class FullSequenceSearch:
    """Persist chronological evidence and stop each threshold independently."""

    def __init__(self, args):
        from scripts.investigation.full_sequence_frame_evaluator import FullSequenceFrameEvaluator
        self.args = args
        self.entries = json.loads((args.previous / 'first-frame-localization/inputs/manifest.json').read_text())
        self.evaluator = FullSequenceFrameEvaluator(args.previous, args.output / 'evaluation')

    write = staticmethod(HybridComparison.write)

    def save_sequence(self, entry, frames, selections, checked, status, error=None):
        methods = {}
        for target, (method, metric) in TARGETS.items():
            selected = selections.get(target)
            exhausted = selected is None and len(checked[target]) == frames.count
            methods[target] = dict(method_id=method, metric=metric, strict_threshold_m=.1,
                status='qualified' if selected else 'exhausted_no_qualifying_frame' if exhausted else 'incomplete',
                selected=selected, tested_frame_offsets=checked[target],
                exhausted=exhausted, every_frame_before_selection_tested=(selected is not None and checked[target] == list(range(selected['frame_offset'] + 1))),
                frame_zero_inherited_fallback=(dict(frame_offset=0,
                    source_npz=entry['npz_path'], transform_key='inherited_map_T_dataset',
                    newly_accepted_localization=False) if exhausted else None))
        summary = dict(run_id=entry['run_id'], dataset=entry['dataset'], frame_count=frames.count,
            duration_seconds=frames.times[-1] - frames.times[0], status=status, error=error,
            methods=methods, reference_privileged_qualification=True, independent_ground_truth=False,
            full_sequence=True, hard_cutoff_percent=None, method_settings_changed=False)
        self.write(self.args.output / 'sequences' / f"{entry['run_id']}.json", summary)
        return summary

    def run_sequence(self, entry):
        frames = SequenceFrames(entry, self.args.previous, self.args.output,
            self.args.trajectory_root, self.args.nuscenes_recorded_root)
        selections, checked = {}, {key: [] for key in TARGETS}
        try:
            for offset in range(frames.count):
                unresolved = [key for key in TARGETS if key not in selections]
                if not unresolved:
                    break
                required = sorted({TARGETS[key][0] for key in unresolved})
                path = self.args.output / 'attempts' / entry['run_id'] / f'f{offset:06d}.json'
                try:
                    # The evaluator owns input/map/config fingerprint validation.
                    # Even on resume, never trust an orchestrator attempt file as
                    # independently valid fit evidence or bypass that validation.
                    metadata = frames.frame(offset)
                    case = dict(run_id=entry['run_id'], metadata=metadata, **frames.timing(offset))
                    evaluated = self.evaluator.evaluate_case(case, required_methods=required)
                    record = dict(case, methods=evaluated['methods'], evaluation=evaluated,
                        reference_privileged_qualification=True)
                    if not set(required).issubset(record['methods']):
                        raise ValueError(f'evaluator omitted required methods: {required}')
                    for method in required:
                        item = record['methods'][method]
                        if type(item.get('accepted')) is not bool:
                            raise ValueError(f'{method} result lacks explicit boolean acceptance')
                        if item.get('accepted') is True:
                            transform = np.asarray(item.get('map_T_dataset'), dtype=float)
                            if transform.shape != (4, 4) or not np.isfinite(transform).all():
                                raise ValueError(f'accepted {method} result lacks a finite 4x4 transform')
                            for metric in {metric for candidate, metric in TARGETS.values() if candidate == method}:
                                error = item.get('error', {}).get(metric)
                                if error is None or not math.isfinite(error) or error < 0:
                                    raise ValueError(f'accepted {method} result has invalid {metric}')
                    self.write(path, record)
                except Exception as error:
                    failure = dict(run_id=entry['run_id'], **frames.timing(offset),
                        required_methods=required, exception=f'{type(error).__name__}: {error}',
                        status='incomplete_unexamined_frame')
                    self.write(self.args.output / 'failures' / f"{entry['run_id']}-f{offset:06d}.json", failure)
                    return self.save_sequence(entry, frames, selections, checked, 'incomplete', failure)
                for target in unresolved:
                    method, metric = TARGETS[target]
                    result = record['methods'][method]
                    checked[target].append(offset)
                    error = result.get('error', {}).get(metric)
                    if result.get('accepted') is True and error is not None and math.isfinite(error) and 0 <= error < .1:
                        selections[target] = dict(**frames.timing(offset), frame_id=record['metadata']['frame_id'],
                            metric=metric, error=result['error'], accepted=True,
                            map_T_dataset=result.get('map_T_dataset'), source_attempt=str(path))
                summary = self.save_sequence(entry, frames, selections, checked, 'searching')
                print(json.dumps(dict(run_id=entry['run_id'], frame_offset=offset, frame_count=frames.count,
                    newly_found=[key for key, row in selections.items() if row['frame_offset'] == offset],
                    unresolved=[key for key in TARGETS if key not in selections])), flush=True)
                complete = all(selections.get(key) or len(checked[key]) == frames.count for key in TARGETS)
                if self.args.max_frames is not None and offset + 1 >= self.args.max_frames and not complete:
                    return self.save_sequence(entry, frames, selections, checked, 'incomplete_smoke_bound')
            return self.save_sequence(entry, frames, selections, checked,
                'complete' if all(selections.get(key) or len(checked[key]) == frames.count for key in TARGETS) else 'incomplete')
        finally:
            frames.close()

    def run(self):
        selected = [entry for entry in self.entries if not self.args.run_id or entry['run_id'] in self.args.run_id]
        self.write(self.args.output / 'plans' / f'{self.args.worker_id}.json', dict(
            run_ids=[entry['run_id'] for entry in selected], targets=TARGETS,
            threshold_m=.1, comparison='strictly less than threshold AND unchanged own acceptance gates',
            search_order='chronological every native frame; stop each target independently',
            hard_cutoff_percent=None, source_data_read_only=True, queues_changed=False))
        summaries = []
        for entry in selected:
            try:
                summaries.append(self.run_sequence(entry))
            except Exception as error:
                failure = dict(run_id=entry['run_id'], status='incomplete_sequence_initialization',
                    exception=f'{type(error).__name__}: {error}')
                self.write(self.args.output / 'sequences' / f"{entry['run_id']}.json", failure)
                summaries.append(failure)
            self.write(self.args.output / 'workers' / f'{self.args.worker_id}.json', summaries)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--previous', type=Path, default=Path('/tmp/RemoteTeleop-first-frame-localization/artifacts'))
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts/full-sequence-localization')
    parser.add_argument('--trajectory-root', type=Path, default=Path('/tmp/RemoteTeleop-mcap-replay/artifacts/proxy-batch/xyz-results'))
    parser.add_argument('--nuscenes-recorded-root', type=Path)
    parser.add_argument('--run-id', action='append')
    parser.add_argument('--worker-id', default='all')
    parser.add_argument('--max-frames', type=int, help='Optional smoke bound; never marked exhausted')
    FullSequenceSearch(parser.parse_args()).run()


if __name__ == '__main__':
    main()

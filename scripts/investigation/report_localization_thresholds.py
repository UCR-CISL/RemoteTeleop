"""Summarize best tested accepted frames and looser offline accuracy thresholds."""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import io
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.investigation.report_full_sequence_localization import write_atomic

TARGETS = {'bev_xy': ('bev', 'xy_m'), 'xyz': ('xyz', 'xyz_m'),
           'bev_lidar_height': ('bev_lidar_height', 'xyz_m'),
           'bev_xyz': ('bev', 'xyz_m')}
THRESHOLDS = (.10, .15, .20, .25, .50)


def summarize_frames(attempts, method, metric, frame_count):
    """Accepted fits alone qualify; missing frames never prove exhaustion."""
    tested, accepted, rejected = [], [], []
    for attempt in sorted(attempts, key=lambda row: row['frame_offset']):
        result = attempt.get('methods', {}).get(method)
        if result is None:
            continue
        tested.append(attempt['frame_offset'])
        error = result.get('error', {}).get(metric)
        if error is None or not math.isfinite(error) or error < 0:
            continue
        row = dict(frame_offset=attempt['frame_offset'], elapsed_seconds=attempt['elapsed_seconds'],
                   sequence_percent=attempt['sequence_percent'], metric_error_m=error,
                   accepted=result.get('accepted') is True, error=result['error'],
                   registration=result.get('registration'), reason=result.get('reason'))
        (accepted if row['accepted'] else rejected).append(row)
    full = tested == list(range(frame_count))
    first = {}
    for threshold in THRESHOLDS:
        hit = next((row for row in accepted if row['metric_error_m'] < threshold), None)
        # A gap before a candidate would prevent an earliest-frame claim.
        if hit and tested[:hit['frame_offset'] + 1] != list(range(hit['frame_offset'] + 1)):
            hit = None
        first[f'{threshold:.2f}'] = dict(status='qualified' if hit else 'exhausted' if full else 'unresolved', selected=hit)
    return dict(tested_frames=len(tested), full_sequence_evaluated=full,
                best_scope='entire_sequence' if full else 'evaluated_frames_only',
                best_accepted=min(accepted, key=lambda r: (r['metric_error_m'], r['frame_offset'])) if accepted else None,
                best_rejected_diagnostic=min(rejected, key=lambda r: (r['metric_error_m'], r['frame_offset'])) if rejected else None,
                thresholds=first)


class ThresholdReport:
    def __init__(self, root):
        self.root = root
        self.cache = {}

    def refresh(self):
        rows = []
        sequences = [json.loads(p.read_text()) for p in sorted((self.root / 'sequences').glob('*.json'))]
        for sequence in sequences:
            if 'frame_count' not in sequence:
                continue
            attempts = []
            for path in sorted((self.root / 'attempts' / sequence['run_id']).glob('f*.json')):
                stamp = path.stat().st_mtime_ns
                if path not in self.cache or self.cache[path][0] != stamp:
                    self.cache[path] = (stamp, json.loads(path.read_text()))
                attempts.append(self.cache[path][1])
            for target, (method, metric) in TARGETS.items():
                rows.append(dict(run_id=sequence['run_id'], target=target, method=method, metric=metric,
                    **summarize_frames(attempts, method, metric, sequence['frame_count'])))
        complete = len(sequences) == 23 and all(s['status'] == 'complete' for s in sequences)
        snapshot = dict(generated_at_utc=datetime.now(timezone.utc).isoformat(), search_complete=complete,
                        registration_gates_changed=False, selection_policy_changed=False, rows=rows)
        write_atomic(self.root / 'threshold-analysis.json', json.dumps(snapshot, indent=2, allow_nan=False) + '\n')
        lines = ['# Best tested localization and accuracy-threshold sensitivity', '',
            f"Snapshot: {snapshot['generated_at_utc']} · {'Search complete' if complete else 'Interim; searches still unresolved'}", '',
            'Only registration-accepted poses qualify. These are offline recorded/ENU reference errors, not independent ground-truth accuracy. No running search policy or acceptance gate was changed.', '',
            'Each method stops after its original 0.1 m hit. Therefore best-tested is not necessarily the global best frame. For higher thresholds, the saved chronological prefix is sufficient to establish the earliest hit. Unresolved rows cannot yet establish failure at a threshold.', '',
            '## Qualified sequences / 23', '',
            '| Target | <0.10 m | <0.15 m | <0.20 m | <0.25 m | <0.50 m |',
            '|---|---:|---:|---:|---:|---:|']
        for target in TARGETS:
            group = [r for r in rows if r['target'] == target]
            cells = []
            for threshold in THRESHOLDS:
                counts = Counter(r['thresholds'][f'{threshold:.2f}']['status'] for r in group)
                cells.append(f"{counts['qualified']} ({23-counts['qualified']-counts['exhausted']} unresolved)")
            lines.append(f"| {target} | " + ' | '.join(cells) + ' |')
        lines += ['', 'BEV XY evaluates horizontal accuracy only. Compare `bev_xyz`, `xyz` and `bev_lidar_height` for the same full-XYZ requirement.', '',
            '## Best registration-accepted error among tested frames', '',
            '| Sequence | Target | Best error (m) | Frame | Time (s) | Through sequence | Coverage |',
            '|---|---|---:|---:|---:|---:|---|']
        flat = []
        for row in rows:
            best = row['best_accepted']
            label = 'entire sequence' if row['full_sequence_evaluated'] else f"{row['tested_frames']} evaluated frames only"
            values = [f"{best['metric_error_m']:.4f}", str(best['frame_offset']), f"{best['elapsed_seconds']:.1f}", f"{best['sequence_percent']:.1f}%"] if best else ['—'] * 4
            lines.append(f"| {row['run_id']} | {row['target']} | " + ' | '.join(values) + f' | {label} |')
            for threshold, result in row['thresholds'].items():
                hit = result['selected'] or {}
                flat.append(dict(run_id=row['run_id'], target=row['target'], metric=row['metric'],
                    threshold_m=threshold, status=result['status'], first_frame=hit.get('frame_offset'),
                    elapsed_seconds=hit.get('elapsed_seconds'), sequence_percent=hit.get('sequence_percent'),
                    error_m=hit.get('metric_error_m'), best_tested_error_m=best['metric_error_m'] if best else None,
                    best_tested_frame=best['frame_offset'] if best else None, best_scope=row['best_scope']))
        lines += ['', '[Numeric threshold table](threshold-analysis.csv) · [Full diagnostics, including rejected low-error candidates](threshold-analysis.json)', '',
                  'A low reference error from a registration-rejected frame is diagnostic only and never contributes to these counts. Greater coverage under a looser accuracy threshold alone does not establish an appropriate operational threshold.', '']
        write_atomic(self.root / 'threshold-analysis.md', '\n'.join(lines))
        stream = io.StringIO()
        if flat:
            writer = csv.DictWriter(stream, fieldnames=flat[0]); writer.writeheader(); writer.writerows(flat)
        write_atomic(self.root / 'threshold-analysis.csv', stream.getvalue())
        return complete


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT / 'artifacts/full-sequence-localization')
    parser.add_argument('--watch-seconds', type=float, default=0)
    args = parser.parse_args()
    report = ThresholdReport(args.root)
    while True:
        if report.refresh() or args.watch_seconds <= 0:
            break
        time.sleep(args.watch_seconds)


if __name__ == '__main__':
    main()

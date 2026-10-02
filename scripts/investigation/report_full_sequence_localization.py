"""Read-only reporting of live or completed chronological localization searches."""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import io
import json
from pathlib import Path
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
TARGETS = {
    'bev': ('BEV · XY target', 'bev'),
    'xyz': ('XYZ · XYZ target', 'xyz'),
    'bev_lidar_height': ('BEV + LiDAR height · XYZ target', 'bev_lidar_height'),
    'bev_xyz_benchmark': ('BEV · XYZ benchmark', 'bev'),
}
ERRORS = ('dx_m', 'dy_m', 'dz_m', 'xy_m', 'xyz_m', 'roll_deg', 'pitch_deg', 'yaw_deg')
RESIDUALS = ('inlier_ratio', 'median_residual', 'p95_residual', 'all_point_median_residual',
             'all_point_p95_residual', 'correction_translation', 'correction_rotation_degrees')


def write_atomic(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, prefix=path.name + '.', suffix='.tmp', delete=False) as stream:
        stream.write(text)
        temporary = Path(stream.name)
    temporary.replace(path)


def number(value, digits=3, signed=False):
    return '—' if value is None else format(value, f'+.{digits}f' if signed else f'.{digits}f')


class FullSequenceReport:
    """Materialize snapshot reports without importing or changing search code."""

    def __init__(self, root, manifest):
        self.root = root.resolve()
        self.entries = json.loads(manifest.read_text())
        self.cache = {}
        self.configs = {}

    def read(self, path):
        path = Path(path)
        if path not in self.cache:
            self.cache[path] = json.loads(path.read_text()) if path.exists() else {}
        return self.cache[path]

    def worker_states(self):
        workers = {}
        for launch in self.read(self.root / 'launch.json') or []:
            pid = launch.get('pid')
            command = Path(f'/proc/{pid}/cmdline')
            try:
                running = b'search_full_sequence_localization.py' in command.read_bytes()
            except (FileNotFoundError, ProcessLookupError):
                running = False
            for run_id in launch['run_ids']:
                workers[run_id] = dict(worker=launch['worker'], worker_pid=pid, worker_running=running)
        return workers

    def attempt(self, run_id, offset):
        return self.read(self.root / 'attempts' / run_id / f'f{offset:06d}.json') if offset is not None else {}

    def gate_fields(self, attempt, method):
        result = attempt.get('methods', {}).get(method, {})
        evaluation = attempt.get('evaluation', {})
        registration = result.get('registration', {})
        reasons = list(registration.get('rejection_reasons', []))
        if result.get('reason'):
            reasons.append(result['reason'])
        if result.get('exception'):
            reasons.append(result['exception'])
        upstream = evaluation.get('bev_upstream_accepted') if method != 'xyz' else evaluation.get('xyz_scan', {}).get('accepted')
        fusion = evaluation.get('xyz_fusion', {}) if method == 'xyz' else evaluation.get('bev_fit', {}).get('fusion', {})
        certificate = evaluation.get('rejection_certificate', {})
        if certificate.get('certified_impossible'):
            reasons += ['certified_no_accepted_transform'] + certificate.get('rejection_reasons', [])
        return dict(accepted=result.get('accepted'), registration_accepted=registration.get('accepted'),
            upstream_accepted=upstream, fusion_converged=fusion.get('converged'),
            certified_no_accepted_transform=certificate.get('certified_impossible', False),
            rejection_reasons='; '.join(dict.fromkeys(reasons)), **{key: registration.get(key) for key in RESIDUALS})

    def rows(self):
        self.cache, self.configs = {}, {}
        workers = self.worker_states()
        rows = []
        for entry in self.entries:
            run_id = entry['run_id']
            sequence = self.read(self.root / 'sequences' / f'{run_id}.json')
            for target, (_, method) in TARGETS.items():
                state = sequence.get('methods', {}).get(target, {})
                selected = state.get('selected')
                checked = state.get('tested_frame_offsets', [])
                status = state.get('status', 'queued')
                if not selected and not state.get('exhausted'):
                    if sequence.get('error') or sequence.get('exception') or sequence.get('status', '').startswith('incomplete'):
                        status = 'incomplete'
                    elif workers.get(run_id, {}).get('worker_running') is False:
                        status = 'incomplete_worker_stopped'
                    elif checked:
                        status = 'searching'
                row = dict(run_id=run_id, dataset=entry['dataset'], scene=entry['scene'], agent=entry['agent'],
                    target=target, method_id=method, metric=state.get('metric', 'xy_m' if target == 'bev' else 'xyz_m'),
                    strict_threshold_m=state.get('strict_threshold_m', .1), status=status,
                    sequence_status=sequence.get('status', 'queued'), frame_count=sequence.get('frame_count'),
                    tested_frames=len(checked), earliest_proven=state.get('every_frame_before_selection_tested', False),
                    exhausted=state.get('exhausted', False),
                    fallback_inherited_frame0=bool(state.get('frame_zero_inherited_fallback')),
                    selected_frame=None, selected_frame_id=None, elapsed_seconds=None, sequence_percent=None,
                    **workers.get(run_id, {}))
                chosen = self.attempt(run_id, selected['frame_offset']) if selected else {}
                if selected:
                    row.update(selected_frame=selected['frame_offset'], selected_frame_id=selected['frame_id'],
                        elapsed_seconds=selected['elapsed_seconds'], sequence_percent=selected['sequence_percent'],
                        timestamp_basis=selected.get('timestamp_basis'), source_attempt=selected.get('source_attempt'),
                        **{key: selected['error'].get(key) for key in ERRORS}, **self.gate_fields(chosen, method))
                latest = self.attempt(run_id, checked[-1]) if checked else {}
                latest_method = latest.get('methods', {}).get(method, {})
                row.update(latest_tested_frame=checked[-1] if checked else None,
                    latest_elapsed_seconds=latest.get('elapsed_seconds'), latest_sequence_percent=latest.get('sequence_percent'),
                    latest_accepted=latest_method.get('accepted'),
                    **{f'latest_{key}': latest_method.get('error', {}).get(key) for key in ERRORS},
                    **{f'latest_{key}': value for key, value in self.gate_fields(latest, method).items() if key != 'accepted'})
                failure = sequence.get('error') or sequence.get('exception')
                row['blocking_error'] = failure.get('exception') if isinstance(failure, dict) else failure
                identity = (chosen or latest).get('evaluation', {}).get('identity', {})
                if identity:
                    self.configs[entry['dataset']] = {key: identity[key] for key in ('registration_config', 'xyz_fusion', 'bev_fusion', 'source_mask', 'height_mode') if key in identity}
                rows.append(row)
        return rows

    @staticmethod
    def selection_cell(row):
        if row['status'] == 'qualified':
            return (f"f{row['selected_frame']} · {number(row['elapsed_seconds'], 1)}s · {number(row['sequence_percent'], 1)}%"
                    f"; XY {number(row.get('xy_m'))} / XYZ {number(row.get('xyz_m'))}m")
        if row['exhausted']:
            return f"No qualifying frame ({row['tested_frames']}/{row['frame_count']}); inherited f0 fallback"
        if row['status'] == 'queued':
            return 'Queued; not evaluated'
        return f"{row['status'].replace('_', ' ')} ({row['tested_frames']}/{row['frame_count'] or '?'})"

    def generate(self):
        rows = self.rows()
        timestamp = datetime.now(timezone.utc).isoformat(timespec='seconds')
        complete = all(row['status'] == 'qualified' or row['exhausted'] for row in rows)
        summary = dict(generated_at_utc=timestamp, complete=complete, planned_sequences=len(self.entries),
            planned_target_rows=len(rows), status_counts=dict(Counter(row['status'] for row in rows)),
            gates=self.configs, rows=rows)
        write_atomic(self.root / 'report.json', json.dumps(summary, indent=2) + '\n')
        fields = list(dict.fromkeys(key for row in rows for key in row))
        stream = io.StringIO()
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)
        write_atomic(self.root / 'report.csv', stream.getvalue())

        lines = ['# Full-sequence localization comparison', '',
            f"**{'Complete' if complete else 'Interim — search is not complete'}** · snapshot {timestamp}", '',
            'This is an earliest qualifying native-frame search, not a minimum-error search. '
            'Each target stops independently at its first accepted result strictly below 0.1 m; every preceding native frame must have been examined. '
            'There is no 15% or 25% cutoff.', '',
            'BEV’s primary target uses XY error. XYZ and BEV + LiDAR height use XYZ error. '
            'The separate **BEV XYZ benchmark** uses the same XYZ criterion for a fair comparison; BEV XY success alone does not establish height accuracy.', '',
            '[Full numeric CSV](report.csv) · [Machine-readable snapshot and actual configurations](report.json)', '',
            '## Progress', '', '| Target | Qualified | Exhausted without a hit | Searching | Queued | Incomplete |', '|---|---:|---:|---:|---:|---:|']
        for target, (label, _) in TARGETS.items():
            counts = Counter(row['status'] for row in rows if row['target'] == target)
            lines.append(f"| {label} | {counts['qualified']} | {counts['exhausted_no_qualifying_frame']} | {counts['searching']} | {counts['queued']} | {sum(n for key, n in counts.items() if key.startswith('incomplete'))} |")
        lines += ['', 'Queued, searching, and incomplete cases are unresolved. They are not failures across the full sequence. '
                  'An inherited frame-zero fallback is listed only after genuine exhaustion; it is not a newly accepted localization.', '']
        groups = [('nuScenes', [entry['run_id'] for entry in self.entries if entry['dataset'] == 'nuscenes'])]
        for take in range(1, 6):
            groups.append((f'CooperScene take {take}', [entry['run_id'] for entry in self.entries
                if entry['dataset'] == 'cooperscene' and str(entry['scene']) == str(take)]))
        by_key = {(row['run_id'], row['target']): row for row in rows}
        for title, run_ids in groups:
            lines += [f'## {title}', '', '| Sequence | BEV: XY <0.1m | XYZ: XYZ <0.1m | BEV + LiDAR height: XYZ <0.1m | BEV: XYZ <0.1m benchmark |', '|---|---|---|---|---|']
            for run_id in run_ids:
                lines.append('| ' + run_id + ' | ' + ' | '.join(self.selection_cell(by_key[(run_id, target)]) for target in TARGETS) + ' |')
            lines.append('')
        lines += ['## Chosen-frame diagnostics', '',
            'Only accepted, qualifying selections appear below. Signed errors are in the recorded-reference dataset axes. '
            'Residuals compare registration points to map Gaussian centers. Both are distinct from independently surveyed ground-truth accuracy.', '',
            '| Sequence / target | Signed X / Y / Z (m) | XY / XYZ (m) | Roll / pitch / yaw (deg) | Inlier ratio | Inlier median / p95 (m) | All-point median / p95 (m) | Original gates |',
            '|---|---:|---:|---:|---:|---:|---:|---|']
        for row in rows:
            if row['status'] != 'qualified':
                continue
            label = f"{row['run_id']} / {row['target']}"
            source = row.get('source_attempt')
            if source:
                try:
                    label = f'[{label}]({Path(source).relative_to(self.root)})'
                except ValueError:
                    pass
            signed = ' / '.join(number(row.get(key), signed=True) for key in ('dx_m', 'dy_m', 'dz_m'))
            angle = ' / '.join(number(row.get(key), signed=True) for key in ('roll_deg', 'pitch_deg', 'yaw_deg'))
            lines.append(f"| {label} | {signed} | {number(row.get('xy_m'))} / {number(row.get('xyz_m'))} | {angle} | "
                f"{number(row.get('inlier_ratio'))} | {number(row.get('median_residual'))} / {number(row.get('p95_residual'))} | "
                f"{number(row.get('all_point_median_residual'))} / {number(row.get('all_point_p95_residual'))} | {'PASS' if row.get('accepted') is True else 'Evidence unavailable'} |")
        lines += ['', '## Unchanged acceptance policy', '',
            'No acceptance thresholds are relaxed. CooperScene retains accepted upstream scan registration, converged fusion, '
            'and original final registration gates. nuScenes BEV requires a completed constrained optimizer; its height candidate '
            'is evaluated using the original residual/correction gates. A valid height proposal does not itself establish registration acceptance. '
            'Conservative rejection certificates can prove that no transform within those same gates is acceptable; they are not heuristic overlap skips.', '',
            '| Dataset | Minimum inlier ratio | Maximum inlier median / p95 (m) | Maximum all-point median / p95 (m) | Maximum correction translation / rotation |', '|---|---:|---:|---:|---:|']
        for dataset, config in sorted(self.configs.items()):
            gates = config['registration_config']
            def limit(key):
                return 'unbounded' if gates.get(key) is None else number(gates[key])
            lines.append(f"| {dataset} | {limit('min_inlier_ratio')} | {limit('max_median_residual')} / {limit('max_p95_residual')} | "
                f"{limit('max_all_point_median_residual')} / {limit('max_all_point_p95_residual')} | "
                f"{limit('max_correction_translation')}m / {limit('max_correction_rotation_degrees')}° |")
        lines += ['', '## Interpretation and recommendations', '',
            'Qualification uses recorded/ENU-reference agreement and is privileged offline evaluation. '
            'It is not a deployable success detector or independent ground truth. CooperScene’s existing BEV horizontal/yaw ENU prior remains; '
            'nuScenes retains its first-camera map-chart initialization and imposed gravity/vertical assumptions.', '',
            'Recorded object annotations provide the existing static masks (CooperScene registration and both datasets’ road fits); '
            'nuScenes registration retains its original unmasked points. These source masks are another offline input, not a demonstrated live detector.', '',
            'The LiDAR-height method assumes Riverside’s documented native Z=−2.8 rendering plane is physical road. '
            'For nuScenes, it uses each scene’s first LiDAR road plane transported into its first-camera map chart; '
            'later positions extrapolate that initial plane. Full-sequence search does not validate these terrain assumptions.', '',
            'Times are normalized replay offsets (CooperScene) or processed 10 Hz index timestamps (nuScenes), not exact sensor exposure times. '
            'Sequence percentage uses that full native replay span. CSV also separates selected-frame results from the latest tested frame’s diagnostics.', '']
        if not complete:
            lines += ['**Recommendations are deferred until the full search finishes.** Current successes establish only the listed first qualifying frames. '
                      'Unresolved sequences cannot support a no-solution claim or a dataset-wide method ranking. '
                      'Keep the current production defaults and queues unchanged while these results remain incomplete.', '']
        else:
            for dataset in ('nuscenes', 'cooperscene'):
                subset = [row for row in rows if row['dataset'] == dataset]
                counts = {target: sum(row['status'] == 'qualified' for row in subset if row['target'] == target) for target in TARGETS}
                lines.append(f"- {dataset}: accepted XYZ <0.1m was found in {counts['xyz']} sequences with XYZ, "
                    f"{counts['bev_xyz_benchmark']} with BEV, and {counts['bev_lidar_height']} with BEV + LiDAR height. "
                    f"BEV’s separate XY-only target qualified {counts['bev']} sequences.")
            lines += ['', 'Use the XYZ benchmark when choosing a height-aware method. For exhausted cases, inspect static map coverage, '
                'reference alignment and road-plane validity before changing localization settings; no threshold relaxation is justified by failure alone. '
                'Reference-selected start frames remain offline evaluation results. Do not automatically deploy the first-frame road-plane extrapolation '
                'or convert inherited fallbacks into accepted localizations.', '']
        write_atomic(self.root / 'report.md', '\n'.join(lines))
        return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT / 'artifacts/full-sequence-localization')
    parser.add_argument('--manifest', type=Path, default=Path('/tmp/RemoteTeleop-first-frame-localization/artifacts/first-frame-localization/inputs/manifest.json'))
    parser.add_argument('--watch-seconds', type=float, default=0., help='Refresh until all targets resolve; zero writes one snapshot')
    args = parser.parse_args()
    if args.watch_seconds < 0:
        parser.error('--watch-seconds must be nonnegative')
    reporter = FullSequenceReport(args.root, args.manifest)
    while True:
        summary = reporter.generate()
        print(json.dumps({key: summary[key] for key in ('generated_at_utc', 'complete', 'status_counts')}), flush=True)
        if not args.watch_seconds or summary['complete']:
            break
        time.sleep(args.watch_seconds)


if __name__ == '__main__':
    main()

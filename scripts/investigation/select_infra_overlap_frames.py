"""Exhaustive offline overlap selection; released poses remain unchanged.

Scoring uses only stationary infrastructure calibration geometry. It does not
read GSplat points, fitted map transforms, or evaluation residuals. Original
archives are read-only; resumable per-group scan caches live under /tmp.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import csv
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import zipfile

for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[name] = '1'
import numpy as np
import yaml
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from src.localization.cooperscene import pose_to_matrix

BASE = ROOT / 'artifacts/reference-refinement/shared-enu-calibration'
OUT = ROOT / 'artifacts/reference-refinement/overlap-selected-calibration/selection'
CACHE = Path('/tmp/cooperscene-infrastructure-overlap-scan-v1')


def transform(points, matrix):
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def voxel_keys(points, size):
    cells = np.floor(points / size).astype(np.int64)
    if np.any(abs(cells) >= 2**20):
        raise ValueError('Coordinates exceed collision-free voxel encoding bounds')
    cells += 2**20
    return (cells[:, 0] << 42) | (cells[:, 1] << 21) | cells[:, 2]


def voxel(points, size):
    _, indices = np.unique(voxel_keys(points, size), return_index=True)
    return points[np.sort(indices)]


class InfrastructureOverlapSelector:
    """Owns fixed targets, exclusions, exhaustive scores and selected exports."""

    def __init__(self, workers=3):
        self.workers = workers
        self.old_manifest = json.loads((BASE / 'input-manifest.json').read_text())
        legacy = json.loads((ROOT / 'artifacts/reference-refinement/final-comparison/diagnostics.json').read_text())
        self.exclusions = {(r['take'], r['agent'], r['frame_id']): 'previous_heldout'
                           for r in self.old_manifest if r['partition'] == 'heldout'}
        self.exclusions.update({(int(r['run_id'].split('-')[1]), int(r['run_id'].split('-')[2]),
                                 str(r['frame_id'])): 'previous_eight_case' for r in legacy})
        # Read only the inherited translation; never load the fitted transform.
        self.W = np.eye(4)
        self.W[:3, 3] = [1.2891715943262518, -3.381634466657298, -.43000102971778076]
        with np.load(BASE / 'input-clouds.npz') as data:
            self.infra = {f'cal_{take}_0': data[f'cal_{take}_0'] for take in range(1, 6)}
        self.target = voxel(np.concatenate(list(self.infra.values())), .20)
        self.target_keys = np.unique(voxel_keys(self.target, .5))
        self.definition = dict(
            coordinates='original_W @ released_ENU; original_W is translation only; no fitted transform',
            primary='shared occupied 0.5 m 3D voxel count (equivalently fixed infrastructure coverage fraction)',
            tie_break='larger voxel Dice, then lower take, then earlier frame offset',
            infrastructure_coverage='shared / fixed infrastructure occupied voxel count',
            source_overlap='shared / source occupied voxel count',
            overlap_area_proxy='shared occupied 3D voxel count * 0.25 m2; not measured surface area',
            target='fixed union of five previous cal_{take}_0 calibration-only aggregates, merged at 0.20m',
            target_voxel_count=len(self.target_keys), target_point_count=len(self.target),
            voxel_origin=[0, 0, 0], voxel_size_m=.5, static_box_padding_m=.2,
            score_input='all finite static original PCD points at unchanged released pose',
            output_voxel_m=.20, selection_uses_gsplat=False, selection_refits_pose=False,
            heldout_policy='score all frames; exclude previous 40 heldouts and eight case frames from eligible winners; also report unrestricted winners',
            caveat='Most infrastructure overlap does not guarantee greatest complementary map coverage.')
        signature = hashlib.sha256(json.dumps(dict(definition=self.definition, exclusions=sorted(self.exclusions)), sort_keys=True).encode())
        signature.update(self.target_keys.tobytes())
        self.signature = signature.hexdigest()

    def read_scan(self, archive, take, agent, frame):
        prefix = f'{take}/{agent}/{frame}'
        ann = yaml.load(archive.read(prefix + '.yaml'), Loader=yaml.CSafeLoader)
        raw = archive.read(prefix + '.pcd').split(b'DATA ascii\n', 1)[1]
        points = np.fromstring(raw.decode('ascii'), sep=' ').reshape(-1, 4)[:, :3]
        pose = pose_to_matrix(ann['lidar_pose'])
        points = transform(points, pose)
        keep = np.isfinite(points).all(axis=1)
        for box in ann['vehicles'].values():
            center = np.asarray(box['location']) + box['center']
            rotation = pose_to_matrix([*center, *box['angle']])[:3, :3]
            extent = np.asarray(box['extent']) + .2
            # Exact oriented-box mask after a cheap conservative AABB filter.
            bounds = np.abs(rotation) @ extent
            candidate = np.flatnonzero(keep & (np.abs(points[:, 0] - center[0]) <= bounds[0])
                                       & (np.abs(points[:, 1] - center[1]) <= bounds[1])
                                       & (np.abs(points[:, 2] - center[2]) <= bounds[2]))
            local = (points[candidate] - center) @ rotation
            keep[candidate[np.all(np.abs(local) <= extent, axis=1)]] = False
        return transform(points[keep], self.W), pose, len(points)

    @staticmethod
    def rank(row):
        return (row['shared_voxels'], row['voxel_dice'], -row['take'], -row['frame_offset'])

    def scan_group(self, take, agent):
        started = time.monotonic()
        cache = CACHE / f'{take}_{agent}.json'
        if cache.exists():
            result = json.loads(cache.read_text())
            if result.get('signature') == self.signature:
                print(f'Cached take {take} vehicle {agent}: {len(result["rows"])} scans', flush=True)
                return result
        path = next(r['archive'] for r in self.old_manifest if r['take'] == take)
        rows = []
        with zipfile.ZipFile(path) as archive:
            names = set(archive.namelist())
            frames = sorted(n.split('/')[-1][:-4] for n in names
                            if n.startswith(f'{take}/{agent}/') and n.endswith('.pcd'))
            for offset, frame in enumerate(frames):
                if f'{take}/{agent}/{frame}.yaml' not in names:
                    raise ValueError(f'Missing released pose for {take}/{agent}/{frame}')
                points, _, raw_count = self.read_scan(archive, take, agent, frame)
                keys = np.unique(voxel_keys(points, .5))
                shared = len(np.intersect1d(keys, self.target_keys, assume_unique=True))
                source_n = len(keys)
                target_n = len(self.target_keys)
                rows.append(dict(take=take, agent=agent, frame_id=frame, frame_offset=offset,
                                 frame_count=len(frames), fraction=offset / max(len(frames) - 1, 1),
                                 archive=path, exclusion_reason=self.exclusions.get((take, agent, frame), ''),
                                 raw_point_count=raw_count, static_point_count=len(points),
                                 source_voxels=source_n, infrastructure_voxels=target_n, shared_voxels=shared,
                                 infrastructure_coverage=shared / target_n,
                                 source_overlap_fraction=shared / source_n if source_n else 0,
                                 voxel_dice=2 * shared / (source_n + target_n),
                                 overlap_area_proxy_m2=shared * .25))
                if (offset + 1) % 100 == 0:
                    print(f'Take {take} vehicle {agent}: {offset + 1}/{len(frames)} ({time.monotonic()-started:.1f}s)', flush=True)
        result = dict(signature=self.signature, rows=rows, runtime_s=time.monotonic()-started)
        cache.write_text(json.dumps(result))
        print(f'Finished take {take} vehicle {agent}: {len(rows)} scans in {result["runtime_s"]:.1f}s', flush=True)
        return result

    def enrich_timing(self, archive, row):
        prefix = f'{row["take"]}/{row["agent"]}/'
        first = min(n.split('/')[-1][:-4] for n in archive.namelist()
                    if n.startswith(prefix) and n.endswith('.pcd'))
        metadata = json.loads(archive.read(prefix + 'metadata.json'))
        first_ns = int(metadata[first]['lidar_ns'])
        frame_ns = int(metadata[row['frame_id']]['lidar_ns'])
        row.update(sequence_percentage=100 * row['fraction'],
                   sequence_first_frame_id=first, sequence_first_lidar_ns=first_ns,
                   lidar_ns=frame_ns, elapsed_lidar_seconds=(frame_ns - first_ns) / 1e9,
                   timing_source='original metadata lidar_ns relative first available PCD')

    def fine_metrics(self, points):
        tree = cKDTree(self.target)
        distances = tree.query(points, workers=1)[0]
        reverse = cKDTree(points).query(self.target, workers=1)[0]
        return dict(source_to_infra_median_m=float(np.median(distances)),
                    source_to_infra_p95_m=float(np.percentile(distances, 95)),
                    source_fraction_under_025m=float(np.mean(distances < .25)),
                    source_fraction_under_050m=float(np.mean(distances < .5)),
                    source_fraction_under_100m=float(np.mean(distances < 1.)),
                    infra_to_source_median_m=float(np.median(reverse)),
                    infra_fraction_under_025m=float(np.mean(reverse < .25)),
                    infra_fraction_under_050m=float(np.mean(reverse < .5)),
                    infra_fraction_under_100m=float(np.mean(reverse < 1.)))

    def run(self):
        started = time.monotonic()
        OUT.mkdir(parents=True, exist_ok=True)
        CACHE.mkdir(parents=True, exist_ok=True)
        (OUT / 'score-definition.json').write_text(json.dumps(self.definition, indent=2))
        groups = []
        with ProcessPoolExecutor(max_workers=self.workers) as pool:
            futures = [pool.submit(self.scan_group, take, agent) for take in range(1, 6) for agent in range(1, 4)]
            for future in as_completed(futures):
                groups.append(future.result())
        rows = sorted((r for group in groups for r in group['rows']), key=lambda r: (r['take'], r['agent'], r['frame_offset']))
        with (OUT / 'all-frame-scores.csv').open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        selected, unrestricted = [], []
        for take in range(1, 6):
            for agent in range(1, 4):
                candidates = [r for r in rows if r['take'] == take and r['agent'] == agent]
                unrestricted.append(max(candidates, key=self.rank).copy())
                selected.append(max((r for r in candidates if not r['exclusion_reason']), key=self.rank).copy())
        arrays = dict(self.infra, infra_all=self.target, original_W=self.W)
        for row in selected:
            with zipfile.ZipFile(row['archive']) as archive:
                points, pose, _ = self.read_scan(archive, row['take'], row['agent'], row['frame_id'])
                self.enrich_timing(archive, row)
            points = voxel(points, .2)
            row['cache_key'] = f'cal_{row["take"]}_{row["agent"]}'
            row['exported_point_count'] = len(points)
            row['fine_nn'] = self.fine_metrics(points)
            arrays[row['cache_key']] = points
            arrays[f'pose_{row["take"]}_{row["agent"]}'] = pose
        global_selected = []
        for agent in range(1, 4):
            row = max((r for r in selected if r['agent'] == agent), key=self.rank).copy()
            arrays[f'global_{agent}'] = arrays[row['cache_key']]
            row['cache_key'] = f'global_{agent}'
            global_selected.append(row)
        representative = next(r.copy() for r in self.old_manifest if r['take'] == 1 and r['agent'] == 0 and r['fraction'] == .5)
        with zipfile.ZipFile(representative['archive']) as archive:
            points, pose, _ = self.read_scan(archive, 1, 0, representative['frame_id'])
            self.enrich_timing(archive, representative)
        representative.update(cache_key='infra_representative', selection='Predeclared take1 middle calibration scan; no overlap or map optimization')
        arrays['infra_representative'] = voxel(points, .2)
        arrays['infra_representative_pose'] = pose
        np.savez_compressed(OUT / 'selected-clouds.npz', **arrays)
        result = dict(score_definition=self.definition, signature=self.signature,
                      frame_count_scored=len(rows), eligible_count=sum(not r['exclusion_reason'] for r in rows),
                      reserved_identity_count=len(self.exclusions),
                      reserved_vehicle_count=sum(a != 0 for (_, a, _) in self.exclusions),
                      reserved_infrastructure_count=sum(a == 0 for (_, a, _) in self.exclusions),
                      original_W=self.W.tolist(), per_take_selected=selected, global_selected=global_selected,
                      unrestricted_per_take_winners=unrestricted,
                      unrestricted_global_winners=[max((r for r in rows if r['agent'] == a), key=self.rank) for a in range(1, 4)],
                      infrastructure_representative=representative,
                      infrastructure_target_frames=[r for r in self.old_manifest if r['agent'] == 0 and r['partition'] == 'calibration'],
                      reserved_frames=[dict(take=t, agent=a, frame_id=f, reason=reason) for (t, a, f), reason in sorted(self.exclusions.items())],
                      total_runtime_s=time.monotonic()-started, sum_group_scan_runtime_s=sum(g['runtime_s'] for g in groups),
                      workers=self.workers, original_archives_read_only=True)
        (OUT / 'manifest.json').write_text(json.dumps(result, indent=2))
        self.write_report(result)
        print(f'Completed {len(rows)} frames in {result["total_runtime_s"]:.1f}s', flush=True)

    def write_report(self, result):
        lines = ['# Exhaustive infrastructure-overlap frame selection', '',
                 'Every original vehicle PCD in takes 1–5 is scored in its unchanged released pose. '
                 'The fixed scoring target contains only the prior 15 infrastructure calibration scans; '
                 'no GSplat geometry or fitted map correction participates in selection.', '',
                 'Primary score: **shared occupied 0.5 m voxels**, equivalent to fixed infrastructure coverage. '
                 'Ties prefer higher Dice (2 × shared / [source voxels + infrastructure voxels]), then the earlier take/frame. The voxel grid is fixed in the '
                 'original-W translated ENU frame. Original recorded vehicle boxes are removed with 0.2 m padding. '
                 'The shared-voxel count × 0.25 m² is an occupied-cell area proxy, not measured surface area.', '',
                 f'Scored **{result["frame_count_scored"]}** original frames; **{result["eligible_count"]}** eligible. '
                 'All previous 40 heldouts (10 infrastructure + 30 vehicle) and eight vehicle case frames were reserved before selection. '
                 'Of those 48 identities, 38 are vehicle frames excluded from eligible winners; '
                 'the 10 infrastructure identities are outside this vehicle scan. '
                 'Reserved vehicle frames are still scored and unrestricted winners are retained in the manifest.', '',
                 '| Take | Vehicle | Offset / last | Frame ID | Dice | Infra coverage | Source fraction | Shared voxels | Area proxy m² |',
                 '|---|---|---|---|---|---|---|---|---|']
        for r in result['per_take_selected']:
            lines.append(f'| {r["take"]} | {r["agent"]} | {r["frame_offset"]}/{r["frame_count"]-1} | {r["frame_id"]} | {r["voxel_dice"]:.4f} | {r["infrastructure_coverage"]:.3f} | {r["source_overlap_fraction"]:.3f} | {r["shared_voxels"]} | {r["overlap_area_proxy_m2"]:.2f} |')
        lines += ['', 'Exactly one frame per physical vehicle, taking the best eligible score across all five takes:', '']
        for r in result['global_selected']:
            lines.append(f'- Vehicle {r["agent"]}: take {r["take"]}, offset {r["frame_offset"]} '
                         f'({100*r["fraction"]:.1f}% of sequence), frame {r["frame_id"]}; '
                         f'{r["shared_voxels"]} shared voxels, {100*r["infrastructure_coverage"]:.1f}% infrastructure coverage, '
                         f'Dice {r["voxel_dice"]:.4f}, elapsed {r.get("elapsed_lidar_seconds", float("nan")):.3f} s from original LiDAR timestamps.')
        changed = [r for r in result['unrestricted_per_take_winners'] if r['exclusion_reason']]
        lines += ['', f'{len(changed)} of 15 unrestricted group winners are reserved; their identities and scores are explicit in manifest.json.', '',
                  'The primary map-fit export also contains one predeclared stationary infrastructure frame: take 1 / '
                  'frame 481510 / offset 250. Aggregated infrastructure is a scoring target and is separately labeled.', '',
                  'Most infrastructure overlap does not guarantee greatest complementary map coverage. '
                  'Voxel overlap depends on sampling and grid boundaries; it is geometry consistency, not pose accuracy. '
                  'Fine nearest-neighbor statistics for each selected cloud are secondary diagnostics, never ranking inputs.', '',
                  f'Runtime: {result["total_runtime_s"]:.1f} s wall time, '
                  f'{result["sum_group_scan_runtime_s"]:.1f} s sum of group scan times; '
                  f'{self.workers} worker processes, numerical-library threads capped at one.', '',
                  'Artifacts: [manifest](manifest.json), [all exhaustive scores](all-frame-scores.csv), '
                  '[selected clouds](selected-clouds.npz), [predeclared score](score-definition.json).']
        for original in changed:
            eligible = next(r for r in result['per_take_selected'] if r['take'] == original['take'] and r['agent'] == original['agent'])
            lines += ['', f'Reserved unrestricted winner: take {original["take"]}, vehicle {original["agent"]}, '
                      f'offset {original["frame_offset"]}, frame {original["frame_id"]}, '
                      f'{original["shared_voxels"]} shared cells ({100*original["infrastructure_coverage"]:.2f}% infrastructure coverage). '
                      f'Eligible selection: offset {eligible["frame_offset"]}, frame {eligible["frame_id"]}, '
                      f'{eligible["shared_voxels"]} shared cells ({100*eligible["infrastructure_coverage"]:.2f}% coverage).']
        (OUT / 'report.md').write_text('\n'.join(lines) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--workers', type=int, default=3)
    InfrastructureOverlapSelector(parser.parse_args().workers).run()

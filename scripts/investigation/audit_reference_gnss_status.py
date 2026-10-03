"""Decode recorded GNSS status as provenance evidence, not pose-accuracy proof."""
from __future__ import annotations
import csv
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'artifacts/reference-refinement'
RELEASE = Path('/mnt/bcc-data/data/Cooperscene/release/250928')
DOC = 'https://www.xsens.com/hubfs/Downloads/Manuals/MT_Low-Level_Documentation.pdf'


def decode(word):
    rtk = (word >> 27) & 3
    return dict(word_decimal=word, word_hex=hex(word), filter_valid=bool(word & 2),
                gnss_fix=bool(word & 4), filter_mode_bits=(word >> 23) & 7,
                rtk_bits=rtk, rtk_status={0:'none', 1:'float', 2:'fixed', 3:'reserved'}[rtk])


def main():
    output = BASE / 'gnss-status-audit'
    output.mkdir(parents=True, exist_ok=True)
    selected = json.loads((BASE / 'shared-enu-calibration/input-manifest.json').read_text())
    for record in json.loads((BASE / 'final-comparison/diagnostics.json').read_text()):
        take, agent = map(int, record['run_id'].split('-')[1:])
        selected.append(dict(take=take, agent=agent, frame_id=record['frame_id'], partition='previous_eight'))
    groups, frames = [], []
    for take in range(1, 6):
        for agent in range(1, 4):
            source = RELEASE / str(take) / str(agent) / 'raw_gnss.csv'
            with source.open() as stream:
                rows = list(csv.DictReader(stream))
            timestamps = np.array([int(row['gnss_ns']) for row in rows], dtype=np.int64)
            words = np.array([int(row['StatusWord']) for row in rows], dtype=np.uint32)
            states = (words >> 27) & 3
            groups.append(dict(take=take, agent=agent, source=str(source), raw_log_samples=len(rows),
                raw_log_rtk_counts={name:int(np.sum(states == value)) for value,name in enumerate(['none','float','fixed','reserved'])},
                raw_log_filter_invalid_samples=int(np.sum((words & 2) == 0)), raw_log_gnss_no_fix_samples=int(np.sum((words & 4) == 0))))
            metadata = json.loads((source.parent / 'metadata.json').read_text())
            for record in selected:
                if (record['take'], record['agent']) != (take, agent):
                    continue
                requested = int(metadata[str(record['frame_id'])]['gnss_ns'])
                idx = int(np.argmin(np.abs(timestamps - requested)))
                frames.append(dict(**record, requested_gnss_ns=requested,
                    matched_gnss_ns=int(timestamps[idx]), timestamp_difference_ns=int(timestamps[idx]-requested),
                    **decode(int(words[idx]))))
    result = dict(source_document=DOC, source_section='Table 27, printed pages 53–54; MTi-680 listed as supported',
                  bit_definitions='bit1 filter valid; bit2 GNSS fix; bits23:25 filter mode; bits27:28 RTK status',
                  groups=groups, selected_frames=frames,
                  limitations=['Raw-log totals can include samples outside the released PCD frame range.',
                      'RTK fixed does not certify LiDAR pose, mounting axes, geodetic datum, map tie, or smoothed INS accuracy.',
                      'Nearest raw GNSS timestamp is recorded; no sensor data interpolated or corrected.'])
    (output / 'audit.json').write_text(json.dumps(result, indent=2) + '\n')
    lines = ['# Recorded GNSS status audit', '',
        f'Decoding follows [Xsens low-level documentation, Table 27]({DOC}). Bits 27–28 distinguish none, float, and fixed RTK; bit 1 marks filter validity and bit 2 GNSS fix.', '',
        '| Take/vehicle | Raw log samples | Fixed / float / none RTK | Invalid filter | No GNSS fix |',
        '|---|---:|---:|---:|---:|']
    for row in groups:
        counts = row['raw_log_rtk_counts']
        lines.append(f"| {row['take']}/{row['agent']} | {row['raw_log_samples']} | {counts['fixed']} / {counts['float']} / {counts['none']} | {row['raw_log_filter_invalid_samples']} | {row['raw_log_gnss_no_fix_samples']} |")
    selected_counts = {name:sum(row['rtk_status'] == name for row in frames) for name in ['none','float','fixed','reserved']}
    lines += ['', f'Selected vehicle calibration/held-out/eight-case records: {len(frames)}. RTK states: `{selected_counts}`. '
              f"Largest timestamp mismatch to raw GNSS: {max(abs(r['timestamp_difference_ns']) for r in frames)} ns.", '',
              'Whole-log totals can include samples outside the released PCD range. RTK fixed indicates a receiver solution state; '
              'it does not prove accurate LiDAR pose, attitude calibration, GNSS antenna lever arm, altitude datum, or GSplat alignment. '
              'This audit cannot attribute a geometric discrepancy to GNSS solely from its status bits.', '']
    (output / 'report.md').write_text('\n'.join(lines))
    print(json.dumps(selected_counts), flush=True)


if __name__ == '__main__':
    main()

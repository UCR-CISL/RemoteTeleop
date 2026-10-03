"""Compare release metadata attitude to original INS rows (read-only inputs)."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np
from scipy.spatial.transform import Rotation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[2] / 'artifacts/reference-refinement/release-audit')
    parser.add_argument('--release', type=Path, default=Path('/mnt/bcc-data/data/Cooperscene/release/250928'))
    parser.add_argument('--calibrations', type=Path, default=Path('/tmp/RemoteTeleop-reference-orientation/artifacts/reference-orientation/axes-audit/sensor-extrinsics.json'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    calibrations = json.loads(args.calibrations.read_text())
    checks = []
    for take in range(1, 6):
        for agent in ('1', '2', '3'):
            root = args.release / str(take) / agent
            meta = json.loads((root / 'metadata.json').read_text())
            raw_bytes = (root / 'raw_gnss.csv').read_bytes()
            rows = list(csv.DictReader(raw_bytes.decode().splitlines()))
            timestamps = np.asarray([int(row['gnss_ns']) for row in rows], dtype=np.int64)
            e = np.asarray(calibrations[agent][f'tower{agent}-xsens,tower{agent}-lidar'])
            f = np.diag([1., -1., -1.]) if agent == '1' else np.eye(3)
            frame_ids = sorted(meta, key=int)
            samples = []
            for index in np.unique(np.linspace(0, len(frame_ids)-1, 21).astype(int)):
                frame_id = frame_ids[index]
                m = meta[frame_id]
                raw_index = int(np.argmin(abs(timestamps - int(m['gnss_ns']))))
                raw = rows[raw_index]
                r_meta = Rotation.from_quat([m['Quat_q'+q] for q in ('x','y','z','w')]).as_matrix()
                r_raw = Rotation.from_quat([float(raw['Quat_q'+q]) for q in ('x','y','z','w')]).as_matrix()
                samples.append(dict(frame_id=frame_id, metadata_gnss_ns=int(m['gnss_ns']),
                    raw_gnss_ns=int(timestamps[raw_index]), delta_ns=int(timestamps[raw_index])-int(m['gnss_ns']),
                    R_meta=r_meta.tolist(), R_raw=r_raw.tolist(),
                    E_Rraw_F_max_abs=float(np.max(abs(r_meta-e[:3,:3]@r_raw@f))),
                    raw_vs_metadata_angle_deg=float(np.degrees(Rotation.from_matrix(r_meta@r_raw.T).magnitude()))))
            checks.append(dict(run_id=f'cooperscene-{take}-{agent}', samples=samples,
                body_axis_conversion_F=f.tolist(), E=e.tolist(),
                raw_source=str(root/'raw_gnss.csv'), raw_source_sha256=hashlib.sha256(raw_bytes).hexdigest(),
                maximum_attitude_reconstruction_error=max(s['E_Rraw_F_max_abs'] for s in samples),
                timestamp_matched_samples=sum(abs(s['delta_ns']) <= 1000 for s in samples),
                matched_maximum_attitude_reconstruction_error=max(s['E_Rraw_F_max_abs'] for s in samples if abs(s['delta_ns']) <= 1000),
                maximum_timestamp_difference_ns=max(abs(s['delta_ns']) for s in samples)))
            print(checks[-1]['run_id'], checks[-1]['maximum_attitude_reconstruction_error'], flush=True)
    repo = Path('/home/jyue86/Documents/CISL-Projects/MultiModalCalibration')
    revision = subprocess.check_output(['git','-C',str(repo),'rev-parse','4e2e877'],text=True).strip()
    provenance = []
    for filename in ('scripts/extract_or_read.py','src/data_cleaning/tf_utils.py','src/data_cleaning/clean_mocap_data.py'):
        content = subprocess.check_output(['git','-C',str(repo),'show',f'{revision}:{filename}'])
        path = args.output/'source-snapshots'/'MultiModalCalibration-4e2e877'/filename
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_bytes(content)
        provenance.append(dict(repository=str(repo),commit=revision,path=filename,sha256=hashlib.sha256(content).hexdigest()))
    payload = dict(finding='Release metadata quaternions already include left E rotation and agent1 right Rx180 conversion.',
        corrected_release_chain='N(t) = C @ E @ Pose(R_original_INS(t), p_original_GNSS_ENU(t)) @ F_agent; F1=Rx180, F2=F3=I',
        qualifications=['The earlier physical candidate treated preprocessed metadata attitude as raw INS; that premise is false.',
                        'Matching generator produces MoCap rigid-body relative transforms; exact original MoCap recording unavailable.',
                        'MoCap marker frames versus physical sensor axes/origins and INS alignment configuration remain unverified.'],
        checks=checks, calibration_generator_provenance=provenance)
    (args.output/'raw-attitude-audit.json').write_text(json.dumps(payload,indent=2)+'\n')


if __name__ == '__main__':
    main()

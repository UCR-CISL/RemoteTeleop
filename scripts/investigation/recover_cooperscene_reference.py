"""Recover sequence coordinate frames from independent GNSS, with heldout checks."""
import json
import argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from audit_cooperscene_reference import rigid, OUTPUT, REPO


def main():
    data = json.loads((OUTPUT/'audit.json').read_text())
    origin_map = np.asarray(json.loads((REPO/'config/cooperscene_riverside_iowa_enu_prior.json').read_text())['gaussian_T_cooperscene'])
    references = {}
    for run_id in sorted({r['run_id'] for r in data['native_records']}):
        rows = [r for r in data['native_records'] if r['run_id'] == run_id]
        # Index split is fixed before inspecting errors and independent of registration.
        calibration, heldout = rows[::2], rows[1::2]
        recovered = rigid([np.asarray(r['native_matrix'])[:3,3] for r in calibration], [r['raw_gnss_enu'] for r in calibration])
        t = np.asarray(recovered['transform'])
        errors = np.asarray([t[:3,:3]@np.asarray(r['native_matrix'])[:3,3]+t[:3,3]-r['raw_gnss_enu'] for r in heldout])
        angle_residuals = np.asarray([t[:3,:3] @ np.asarray(r['native_matrix'])[:3,:3] @ np.asarray(r['raw_ins_rotation']).T for r in rows])
        mean_rotation = Rotation.from_matrix(angle_residuals).mean()
        angle_scatter = (Rotation.from_matrix(angle_residuals)*mean_rotation.inv()).magnitude()
        dt_lidar = np.asarray([(r['raw_metadata']['lidar_ns']-r['raw_metadata']['gnss_ns'])*1e-9 for r in rows])
        dt_image = np.asarray([(r['raw_metadata']['image_ns']-r['raw_metadata']['gnss_ns'])*1e-9 for r in rows])
        speeds = np.asarray([np.linalg.norm([r['raw_metadata'][k] for k in ('Vel_E','Vel_N','Vel_U')]) for r in rows])
        references[run_id] = dict(
            reference_kind='native_frame_recovered_from_raw_gnss',
            map_T_dataset=(origin_map @ t).tolist(), raw_gnss_enu_T_dataset=t.tolist(),
            fit_calibration_frame_ids=[r['frame_id'] for r in calibration], heldout_frame_ids=[r['frame_id'] for r in heldout],
            heldout_position_max_m=float(np.linalg.norm(errors,axis=1).max()),
            heldout_position_rmse_m=float(np.sqrt(np.mean(np.sum(errors**2,axis=1)))),
            fit=recovered,
            corrected_lidar_R_raw_ins_mean=mean_rotation.as_matrix().tolist(),
            corrected_lidar_R_raw_ins_rpy_xyz_deg=mean_rotation.as_euler('xyz',degrees=True).tolist(),
            corrected_lidar_R_raw_ins_scatter_max_deg=float(np.degrees(angle_scatter.max())),
            lidar_minus_gnss_time_range_s=[float(dt_lidar.min()),float(dt_lidar.max())],
            image_minus_gnss_time_range_s=[float(dt_image.min()),float(dt_image.max())],
            speed_times_lidar_gnss_skew_max_m=float(np.max(speeds*np.abs(dt_lidar))),
            native_archive=rows[0]['archive'], metadata_member=f"{run_id.split('-')[1]}/{run_id.split('-')[2]}/metadata.json",
        )
    payload = dict(description='Independent recovery of per-sequence exported pose coordinate distortions using original raw LLA. No Gaussian points, localizer outputs, or evaluation errors enter these fits.',
                   caveats=['GNSS altitude datum remains unspecified; the original map origin numeric height offset is retained.',
                            'Recovered LiDAR orientation retains per-agent constant offset relative raw INS; this is not independent orientation ground truth.',
                            'Raw GNSS metadata captures differ from LiDAR and image timestamps; no motion compensation performed.',
                            'This recovers coordinate conventions of exported data. It does not prove GNSS absolute accuracy or a surveyed LiDAR lever arm.'],
                   origin_map_T_raw_gnss_enu=origin_map.tolist(), references=references)
    (OUTPUT/'recovered-references.json').write_text(json.dumps(payload,indent=2)+'\n')
    print('max heldout position residual:',max(r['heldout_position_max_m'] for r in references.values()))
    print('max angular scatter deg:',max(r['corrected_lidar_R_raw_ins_scatter_max_deg'] for r in references.values()))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit-root', type=Path, default=OUTPUT)
    OUTPUT = parser.parse_args().audit_root
    main()

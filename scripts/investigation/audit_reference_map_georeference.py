"""Audit map georeference without fitting localization evaluation frames."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import struct

import numpy as np
from scipy.spatial.transform import Rotation


def enu_basis(latitude_deg: float, longitude_deg: float) -> np.ndarray:
    """ECEF columns representing east, north, up at a geodetic origin."""
    lat, lon = np.deg2rad([latitude_deg, longitude_deg])
    return np.array([[-np.sin(lon), -np.sin(lat)*np.cos(lon), np.cos(lat)*np.cos(lon)],
                     [np.cos(lon), -np.sin(lat)*np.sin(lon), np.cos(lat)*np.sin(lon)],
                     [0., np.cos(lat), np.sin(lat)]])


def camera_centers(path: Path) -> np.ndarray:
    """Read COLMAP's little-endian images.bin; skip feature observations."""
    centers = []
    with path.open('rb') as stream:
        count, = struct.unpack('<Q', stream.read(8))
        for _ in range(count):
            values = struct.unpack('<idddddddi', stream.read(64))
            rotation = Rotation.from_quat([*values[2:5], values[1]]).as_matrix()
            centers.append(-rotation.T @ np.array(values[5:8]))
            while True:
                byte = stream.read(1)
                if byte == b'\0':
                    break
                if not byte:
                    raise ValueError('Truncated COLMAP image name')
            observations, = struct.unpack('<Q', stream.read(8))
            stream.seek(24*observations, 1)
        if stream.read(1):
            raise ValueError('Unexpected trailing COLMAP image data')
    return np.asarray(centers)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-directory', type=Path, default=Path('/home/jyue86/Downloads/riverside_for_justin'))
    parser.add_argument('--reference', type=Path, default=Path('config/cooperscene_riverside_iowa_enu_prior.json'))
    parser.add_argument('--output', type=Path, default=Path('artifacts/reference-orientation/map-georeference.json'))
    args = parser.parse_args()
    reference = json.loads(args.reference.read_text())
    metadata = json.loads((args.map_directory/'georeference.json').read_text())
    source = reference['cooperscene_enu_origin_lla']
    target = reference['gsplat_enu_origin_lla']
    expected = enu_basis(*target[:2]).T @ enu_basis(*source[:2])
    centers = camera_centers(args.map_directory/'colmap/images.bin')
    unique = np.unique(np.round(centers, 5), axis=0)
    design = np.column_stack((unique[:, :2], np.ones(len(unique))))
    coefficients, *_ = np.linalg.lstsq(design, unique[:, 2], rcond=None)
    residual = unique[:, 2] - design @ coefficients
    report = dict(map_metadata=metadata,
        reference_path=str(args.reference.resolve()),
        enu_origin_basis_rotation=expected.tolist(),
        enu_origin_basis_rotation_deg=float(np.rad2deg(Rotation.from_matrix(expected).magnitude())),
        enu_origin_rotation_displacement_at_100m_m=float(100*np.linalg.norm(expected-np.eye(3), ord=2)),
        map_camera_count=len(centers), unique_camera_centers_rounded_10um=len(unique),
        camera_center_bounds_m=dict(minimum=centers.min(axis=0).tolist(), maximum=centers.max(axis=0).tolist()),
        camera_center_z_std_m=float(centers[:, 2].std()),
        camera_center_plane_z_equals_ax_plus_by_plus_c=coefficients.tolist(),
        camera_center_plane_tilt_deg=float(np.rad2deg(np.arctan(np.linalg.norm(coefficients[:2])))),
        camera_center_plane_abs_residual_p50_p90_m=np.quantile(abs(residual), [.5,.9]).tolist(),
        interpretation=[
            'The configured nearby ENU origins cannot explain a multi-degree rotation.',
            'Camera-center height distribution audits the supplied map frame; it is not an independent gravity measurement.',
            'Camera-center plane slope may include terrain grade, vehicle height and map error; do not flatten it as an orientation correction.',
            'Map metadata declares orthometric heights. CooperScene vertical datum remains unverified.',
            'No localization result or evaluation-frame fit was used in this audit.'])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='map_metadata'}, indent=2))


if __name__ == '__main__':
    main()

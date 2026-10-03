> **Superseded by the release-refinement audit.** The metadata quaternion was already preprocessed. The earlier claim that only position was transformed, and the interpretation of the old physical reference as a correction, are withdrawn. See [the updated investigation](investigations/reference-refinement.md).

# CooperScene native pose and calibration audit

## Conclusion

The current shared ENU translation is applied to poses that are **not raw ENU
poses**. Two upstream operations can be recovered directly from independent
metadata and calibration files. The first is a sensor-extrinsic composition
error; the second is a distinct stored ICP correction for each take/agent.
No Gaussian-map alignment or localization error was used to establish this.

For agent `a`, let `E_a = lidar_T_xsens` from the original calibration pickle,
`p` be raw Latitude/Longitude/Altitude converted to ENU, `R` the raw Xsens
quaternion, and `C_sa` the original take/agent ICP correction. The archived
poses numerically obey:

```text
stored_metadata_enu = E_a.rotation @ p + E_a.translation
native_dataset_T_lidar = C_sa @ Pose(R, stored_metadata_enu)
```

This rotates a world position with a sensor mounting rotation and leaves the
world attitude quaternion unchanged. A sensor extrinsic must instead compose
on the right of the world sensor pose:

```text
map_T_lidar = map_T_ENU @ Pose(R, p) @ inverse(E_a)
```

That formula supplies the right attitude multiplication and rotates the
sensor lever arm with the vehicle. `CooperSceneReference` implements it as an
explicit offline utility; existing runtime/loading defaults are unchanged.

## Independent evidence

Sources are mounted read-only:

- Native ZIPs: `/mnt/bcc-data/websites/data.ucr.edu-datasets-cooperscene/{train,test}/{take}.zip`.
  Each contains `{take}/{agent}/metadata.json`, with raw LLA, INS quaternion,
  stored ENU coordinates, GNSS time, LiDAR time and camera time.
- Physical calibration:
  `/mnt/bcc-data/data/Cooperscene/backup/calib_data/250928/tower{agent}-tfs.pkl`.
  Dictionary key `towerN-xsens,towerN-lidar` supplies `E_a`.
- Original per-sequence correction:
  `/mnt/bcc-data/data/Cooperscene/release/250928/{take}/{agent}/icp_correction.json`.
- Map origin: the existing `config/cooperscene_riverside_iowa_enu_prior.json`.

The audit sampled 21 frames distributed over each of 15 sequences, 315 total.
Position correspondences first revealed the two transformations; the actual
calibration and original correction files then independently confirmed them.

| Check | Maximum discrepancy |
|---|---:|
| Native PCD versus cached points, all 15 first frames | exactly 0 |
| Native YAML matrix versus cached `dataset_T_lidar` | 1.23e-13 |
| Cached camera/LiDAR relative transform versus repo calibration | 1.96e-14 |
| Inferred raw-GNSS-to-stored-ENU matrix versus actual sensor calibration `E_a` | 3.18e-10 |
| Stored ENU versus `E_a @ raw_GNSS_ENU` across 315 samples | under 2e-9 m |
| Inferred sequence correction versus original ICP JSON | 8.45e-13 |
| Reconstructed native pose versus YAML across 315 samples | 8.20e-13 |
| Same pickle's LiDAR-to-camera matrix versus repo camera calibration | 4.65e-9 |

The camera calibration check independently verifies the pickle's source-to-
target key convention. The files' SHA-256 values accompany the exported
calibration records. The exact source code that first wrote the altered ENU
fields has not been located; the numeric identity to the original mounting
calibration establishes the operation without depending on that code.

Historical SensorPlatform source is supporting context, not proof of the exact
release generator. Its `utils/cooperscene_util.py` at commit `1144b95` derives
ENU from the configured LLA origin. `viz/mcap_writer.py` on `origin/main` reads
stored ENU fields and quaternions and applies an ICP correction. These historical
versions are not identical in multiplication order to the released data;
the direct released-file identities above take precedence.

## Axes, rotations and camera calibration

The native six-vector is `[x,y,z,roll,yaw,pitch]`, angles in degrees. Its matrix
is equivalent to `Rz(yaw) @ Ry(-pitch) @ Rx(-roll)`. This differs from ordinary
positive-angle XYZ Euler interpretation, but the established native converter
and MCAP/cache agree. Native rotation determinants are +1 and maximum
orthogonality residual is 2.23e-16. No reflection or transpose fix is justified.

Transforms use active column-vector composition `target_T_source @ point`.
Camera coordinates are CV right/down/forward and LiDAR coordinates are
forward/left/up. Camera pose composition
`dataset_T_lidar @ inverse(camera_T_lidar)` passes the numerical checks.

After only recovering raw GNSS positions, the remaining attitude discrepancy
is a constant **left** rotation, identical across all five takes per agent:

| Agent | Remaining XYZ Euler roll / pitch / yaw (degrees) |
|---|---|
| 1 | -2.326067 / -2.558348 / -3.193894 |
| 2 | -0.055268 / +0.001044 / -0.337059 |
| 3 | -0.062508 / -0.368889 / +0.239892 |

Those are exactly `inverse(E_a).rotation`. They explain why the earlier
position-only recovered reference must not be promoted to orientation ground
truth. The physical reconstruction uses raw INS and the actual right-composed
extrinsic. A constant map-to-dataset transform generally cannot express that
repair over arbitrary vehicle headings: the frame-specific reference transform
is `physical_map_T_lidar @ inverse(native_dataset_T_lidar)`.

## Reference artifacts and timing

`physical-references.json` contains 6,705 native frames, keyed by each record's
`run_id`, `frame_offset`, and `frame_id`. It exports `map_T_lidar`, the equivalent
frame-specific `map_T_dataset`, and all three capture timestamps. These physical
poses are computed directly from raw metadata and physical calibration, without
fitting evaluation frames to the Gaussian map.

`recovered-references.json` is retained as a diagnostic position/frame baseline.
Its eleven calibration and ten heldout frames per sequence have a maximum
heldout position residual of 2.18e-9 m, confirming exact export transformations.
It is superseded by the physical sensor reconstruction for pose evaluation.

The baseline is independently sensor-derived, with three remaining qualifications:

1. Raw altitude datum is not documented. The map is orthometric and the old
   numeric origin-height difference is retained, so absolute height is not
   independently surveyed.
2. Raw GNSS position is assumed to denote the Xsens origin. The device's GNSS
   antenna lever-arm configuration is unavailable.
3. Metadata poses are at GNSS time. Across all 6,705 exported frames,
   LiDAR-GNSS skew ranges from -0.546304 to +90.904576 ms. In the 315 sampled
   frames, speed times skew reaches 0.836 m. The MCAP reader joins by
   ordinal and uses GNSS-based record times, which cannot detect this physical
   exposure skew. No interpolation or scan deskew is performed here. Recovering
   capture-time poses from original `raw_gnss.csv` is an appropriate next check
   before treating a 0.5 m threshold as absolute sensor accuracy.

nuScenes uses a different first-front-camera map convention and is unaffected
by the CooperScene calibration diagnosis. The parent audit verifies that
separately against the staged processed files.

## Reproduce

From `/tmp/RemoteTeleop-reference-orientation`, use
`/home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python` to run:

```text
scripts/investigation/audit_cooperscene_reference.py
scripts/investigation/recover_cooperscene_reference.py
scripts/investigation/export_cooperscene_reference.py
-m pytest -q tests/test_cooperscene_reference.py
```

The three focused tests pass: right-side rotation and rotated lever arm, raw
LLA precedence over altered ENU fields, and invalid reflection rejection.
No vehicle processes, existing queue, old evaluation artifacts, or mounted
source files were changed.

# Processed CooperScene convention recheck

## Conclusion

No reader-side axis, pose-order, duplicated ICP or duplicated extrinsic error was found in the tested processed-release paths. Our placement reproduces the native CooperScene devkit, including the vehicle3 scans with large structure offsets. This verifies interpretation of the processed release; it does not establish the release as independent ground truth or explain the remaining physical disagreement.

SensorPlatform processes raw recordings. It is provenance evidence, not preprocessing to rerun on these processed PCD/YAML or MCAP inputs.

## Fresh checks

Native devkit checkout: `thirdparty/CooperScene`, clean revision `0945b52ce7a9765ae17d9c8ffa5e2e8573fef19a`. Execute its exact `pose_to_matrix` and OpenCOOD `x1_to_x2` function bodies independently of our implementation; read PCD coordinates with the native Open3D tensor reader. Source SHA256s and all sample identities are recorded.

| Check | Coverage | Maximum difference |
|---|---|---:|
| Native YAML pose conversion | 66 poses, takes1–5, units0–3; start/middle/end plus selected/extreme cases | 0 |
| Native source-to-infrastructure relative transform | Same66 poses | 0 |
| PCD local XYZ | 7 scans / 626,905 points | 0m |
| World XYZ: native homogeneous-column vs our row-vector placement | Same points | 0m |
| Relative XYZ: native direct transform vs our world-then-infrastructure placement | Same points | 5.69e-14m |
| Actual exported static clouds used in structure/temporal plots vs native placed points | Same7 cases | 8.28e-7m |
| Current MCAP reader points vs processed native archive | Take1/frame481260, all3 vehicles | 0 |
| Current MCAP reader pose vs processed native archive | Same3 frames | 6.51e-14 |
| Camera extrinsic constants vs backup calibration pickles | All3 vehicles | 4.65e-9 |

The exported-cloud check finds each retained static point in the native placed cloud: masking/voxel selection removes points but adds no extra transformation. Its sub-micrometer discrepancy comes from double-precision ASCII parsing versus native float32 storage. Tested vehicle3 native frames488664/488684/488700/488702 include the selected frame and nearby large-disagreement cases.

Separately, the existing15-sequence first-frame cache audit reports identical points, pose differences at most1.23e-13, and camera composition differences at most1.96e-14. That historical audit is labeled separately from the fresh checks above.

## Confirmed conventions

- Native PCD XYZ is LiDAR-local; no axis swap, sign flip, unit rescaling, mounting extrinsic or ICP is applied to the point payload by our reader.
- Tested YAML poses are six-element `[x,y,z,roll_deg,yaw_deg,pitch_deg]` vectors. They use the exact CooperScene/OpenCOOD rotation formula, not a naive standard `Rz(yaw) @ Ry(pitch) @ Rx(roll)`. Its rotation matrices are proper rotations. A synthetic100-pose check also matches exactly.
- `T_release_lidar = pose_to_matrix(lidar_pose)`. With column vectors, `p_release = T_release_lidar @ p_lidar`; our row-vector equivalent is `points @ R.T + t`.
- Relative placement matches the native cooperative loader: `T_infra_vehicle = inverse(T_release_infra) @ T_release_vehicle`.
- The released YAML pose already contains preprocessing/refinement. Our processed-data loading path does not multiply the SensorPlatform mounting matrix or released ICP correction again.
- The MCAP `/tf` map-to-`{agent}_base_link` pose is consumed directly as LiDAR pose. Sample PointCloud messages declare that base_link frame. Their embedded pose is left at protobuf defaults, including an all-zero quaternion; it is not an additional valid transform. Empirical equality with local native points validates using the external TF for this exporter.
- Camera pose is `T_release_lidar @ inverse(camera_T_lidar)`. That extrinsic is applied to derive the camera pose, not again to LiDAR world points. Its constants match the separate calibration pickles to rounding precision. Native ZIP annotations contain no camera calibration, so this is provenance consistency, not physical calibration validation.
- Map alignment is separate: `T_gsplat_lidar = D @ W @ T_release_lidar`. Selected structure clouds already contain `W`; map-fitting scripts apply only `D` to those clouds. `world_T_ego` aliases LiDAR pose in this playback path; it is not a separately calibrated chassis origin.

## Limits and next step

The reader currently does not validate PointCloud `frame_id`/embedded pose inside `_point_cloud`; a different exporter needs that contract checked. Actual sampled source messages follow the current convention. Stream association/skew checks do not prove physical acquisition synchronization, but this audit found no evidence requiring a clock correction.

No conventions fix is justified by these results. Preserve the released baseline. If continuing the discrepancy investigation, treat released trajectory/calibration and scan acquisition semantics as unverified physical properties, while holding the now-checked reader composition fixed. Do not reapply raw-data preprocessing or label improved GSplat agreement as ground-truth proof.

## Reproduction and artifacts

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
/home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python \
  scripts/investigation/audit_processed_cooperscene_conventions.py
```

`artifacts/reference-refinement/processed-conventions-audit/results.json` contains the reproducible native/point checks. `reader-check.json` records the independent reviewer’s fresh MCAP and calibration checks and prior-cache audit provenance. Source paths are read-only; temporary PCD extraction uses an automatically cleaned `/tmp` directory. No production code, thresholds, evaluation outputs or queues were changed.

Source locations: `src/localization/cooperscene.py` (`pose_to_matrix`, `read_ascii_xyzi_pcd`, dataset `_frame`), `src/data/foxglove_mcap.py` (`_point_cloud`, `__iter__`), `src/localization/cooperscene_calibration.py`, `scripts/investigation/advance_cooperscene_frames.py`, and native `models/cooperative/datasets/coop_dataset.py`.

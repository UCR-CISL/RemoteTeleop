# CooperScene evaluation-reference orientation correction

## Conclusion

The tilt is substantially explained by a proven upstream coordinate-composition error. The old shared translation-only ENU evaluation reference is invalid for the exported CooperScene poses. Reconstruct a separate evaluation reference from raw INS/GNSS and each vehicle’s calibrated sensor extrinsic. Do not replace the reference with a scan fit, and do not choose the reference that minimizes map residuals.

## Provenance and cause

Let `E = lidar_T_xsens` be the per-vehicle sensor calibration and `C` the take/agent export correction. The archived native pose is reproduced by:

```text
stored_position = E @ homogeneous(raw_GNSS_ENU_position)
native_pose = C @ Pose(raw_INS_rotation, stored_position)
```

This applies a sensor extrinsic to a world-coordinate position, while leaving its attitude in a different basis. It is not a valid composition of sensor poses. All 15 take/agent sequences have their own export correction.

The independently supplied calibration files reproduce the inferred position operation within 3.18e-10 matrix-entry difference. Original `icp_correction.json` files reproduce the per-sequence correction within 8.45e-13. The full chain reconstructs 315 sampled exported poses within 8.20e-13. These are numerical consistency checks, not claims of sensor accuracy.

Sources (read-only):

- Raw metadata: `/mnt/bcc-data/websites/data.ucr.edu-datasets-cooperscene/{train,test}/{take}.zip`, member `{take}/{agent}/metadata.json`.
- Sensor calibration: `/mnt/bcc-data/data/Cooperscene/backup/calib_data/250928/tower{agent}-tfs.pkl`.
- Original export correction: `/mnt/bcc-data/data/Cooperscene/release/250928/{take}/{agent}/icp_correction.json`.
- Map georeference: `/home/jyue86/Downloads/riverside_for_justin/georeference.json`.

## Implemented reference reconstruction

```text
map_T_lidar = map_T_ENU @ ENU_T_xsens(raw_LLA, raw_quaternion) @ inverse(lidar_T_xsens)
```

`src/localization/cooperscene_reference.py` implements this composition, including right-multiplied orientation and the rotated lever arm. It ignores the modified archive `enu_*` fields. Exported 6,705 per-frame references across all 15 CooperScene sequences. No map points or registration results were used to construct these references. Existing runtime defaults, saved poses, acceptance gates and frame selections remain unchanged.

`rescore_recovered_reference.py` keeps CooperScene error axes fixed to map ENU. Missing CooperScene references fail explicitly; nuScenes references intentionally remain unchanged. Seven focused composition, error-axis and failure-mode tests pass.

## Validation against the same map and scans

An independent reconstruction agrees with the direct raw-metadata export on all 76 sampled CooperScene frames to 1.72e-9 maximum matrix-entry difference. The physical reference improves scan/map median residuals in **70/76 frames**, and improves the per-run median in **all 15 runs**. Median across frame medians falls **0.744 → 0.345 m**; median across frame p95 values falls **1.836 → 1.038 m**. These are sampled nearest-neighbor geometry diagnostics, not localization ATE or rerun acceptance gates.

A single global scan-derived correction worsened two run medians. A position-only correction fitted the map slightly better overall than the complete sensor chain, but is not selected as the reference: pose construction follows the calibration, not whichever candidate fits the map best.

## Same seven previously visualized cases

No localization was rerun or frame reselected. Signed Z and XYZ are disagreement with the indicated reference, in meters. New values use raw INS/GNSS with sensor composition at the GNSS timestamp.

| Take/agent | Method | Frame | Original Z | Corrected Z | Original XYZ | Corrected XYZ |
|---|---|---:|---:|---:|---:|---:|
| 1 / 3 | BEV + height | 464 | -0.501 | -0.544 | 0.507 | 1.528 |
| 2 / 3 | BEV + height | 871 | -0.560 | -0.617 | 0.565 | 1.099 |
| 3 / 1 | BEV + height | 0 | -1.238 | +0.215 | 1.239 | 3.499 |
| 3 / 3 | BEV + height | 500 | -2.023 | -0.830 | 2.026 | 1.185 |
| 5 / 1 | XYZ | 180 | -0.538 | -0.031 | 0.553 | 0.640 |
| 5 / 1 | BEV + height | 180 | -0.649 | -0.142 | 0.659 | 0.660 |
| 5 / 3 | BEV + height | 71 | -1.582 | -0.365 | 1.584 | 0.833 |

Some vertical disagreements shrink markedly, while horizontal/total disagreements increase. The saved methods used the old reference as an ENU prior; this re-scoring is not a fair rerun with corrected inputs. Prior sub-0.1 m qualification counts should not be interpreted as physical accuracy.

## nuScenes

Read-only retrieval of the eight scene-specific native first-camera and LiDAR poses from Lambda confirms that every cached reference equals `inverse(global_T_first_front_camera)` exactly. Cached camera/LiDAR poses agree within 4.55e-13. This is a separate camera-anchored convention; there is no evidence for applying the CooperScene correction to nuScenes. It verifies the loader convention, not independent map accuracy.

## Remaining limits and recommendation

- The reconstructed pose is at the raw GNSS timestamp. Across all 6,705 exported frames, LiDAR minus GNSS skew spans −0.55 to +90.90 ms; sampled speed-times-skew reaches 0.836 m. Time alignment must be addressed before centimeter-level evaluation.
- Confirm whether recorded LLA refers to the INS origin or an antenna/lever-arm-corrected point. The calibration establishes INS-to-LiDAR geometry, not this GNSS convention.
- CooperScene altitude datum remains unverified; the map explicitly uses orthometric elevation. Retaining the existing origin-height difference is an assumption.
- Map georeferencing and physical map geometry have their own uncertainty; remaining map residuals are not proof that the reconstructed reference is wrong.

Use the reconstructed reference as an explicitly labelled, corrected INS/GNSS baseline. Retire the old shared ENU reference for CooperScene accuracy claims. Resolve timestamp/reference-point/altitude conventions before certifying <0.1 m accuracy or re-ranking the methods. Then reevaluate identical frames with corrected priors; defer a full teleoperation batch.

## Evidence

- Artifact root: `/tmp/RemoteTeleop-reference-orientation/artifacts/reference-orientation/`.
- `index.html`: corrected-reference visual gallery.
- `axes-audit/physical-references.json`: source-derived per-frame poses and calibration provenance.
- `physical-rescored/previous-seven-cases.csv`: before/after errors.
- `physical-rescored/all-23-first-frame.csv`: same-frame re-scoring across all 23 sequences; missing saved poses remain labelled.
- `tilt-validation/report.md`: all 15 per-run geometry comparisons and rejected hypotheses.
- `nuscenes-audit/checks.json`: scene-specific source-pose checks.

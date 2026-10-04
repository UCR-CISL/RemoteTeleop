# Handoff: CooperScene ENU to GSplat map transform

This file summarizes the CooperScene-to-GSplat investigation for whoever
continues it. The detailed reports it cites are in `docs/investigations/`.

## Where things are

- **Worktree.** `~/Documents/CISL-Projects/RemoteTeleop-reference-refinement`,
  branch `investigate/cooperscene-reference-refinement`. It was moved there from
  `/tmp` after a reboot wiped `/tmp`.
- **Python.** `/home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python`.
- **Map.** `/home/jyue86/Downloads/riverside_for_justin/riverside_iowa.ply`, with
  `georeference.json` and `README.md` beside it.
- **CooperScene data, read-only.**
  - Processed release: `/mnt/bcc-data/websites/data.ucr.edu-datasets-cooperscene/{train,test}/{take}.zip`.
  - Raw release, including 400 Hz `raw_gnss.csv`, per-frame `metadata.json` and
    `icp_correction.json`: `/mnt/bcc-data/data/Cooperscene/release/250928/{take}/{agent}/`.
  - Mounting calibration `lidar_T_xsens`: `/mnt/bcc-data/data/Cooperscene/backup/calib_data/250928/tower{N}-tfs.pkl`,
    key `tower{N}-xsens,tower{N}-lidar`.
  - Original Xsens logs: `/mnt/bcc-data/data/Cooperscene/backup/250928_xsens/t{1,2,3}/`.
  - The mount is manual and does not survive reboots. Agents are 0 for the
    infrastructure and 1–3 for the vehicles. Takes are 1–5.
- **Artifacts.** `artifacts/reference-refinement/` in the worktree is
  git-ignored. Most outputs from before the reboot were lost. Everything is
  reproducible from the committed scripts; see the last section.

## Rules the user set

- **Read-only sources.** Keep `/mnt` read-only and never modify released poses.
- **Leave production alone.** Do not change production localization defaults,
  interrupt evaluation queues or launch full teleoperation batches. Never
  automate anything that involves the real vehicle or the remote side.
- **Validate honestly.** Predeclare validation, and never choose validation
  data from favourable residuals. Agreement with GSplat is consistency, not
  independent ground truth; never claim otherwise.
- **Preserve prior results.** Put each new run in a new output directory.
- **Code style.** Keep diffs minimal and prefer classes for stateful code; see
  `AGENTS.md`.
- **Mounting facts from the user.** The GNSS antenna sits directly on top of the
  Xsens. The 3 cm vertical offset is the user's estimate and cannot be verified. The recorded `lidar_T_xsens` is the LiDAR-to-Xsens
  transform. The original lever arm can no longer be measured. The
  MultiModalCalibration repository is a separate project; don't rely on it.

## Main conclusion

There is a reliable transform between GSplat and the geodetic CooperScene ENU
frame, the frame defined by GNSS. There is no reliable single transform for the
released CooperScene ENU frame.

```text
map_T_cooperscene_geodetic_enu = G @ W
map_T_lidar(t) = G @ Pose(R_INS(t) @ F_a, p_geo(t) - R_INS(t) @ [0, 0, 0.03]) @ inverse(lidar_T_xsens_a)
```

- **`W`.** A pure geodetic translation, (1.289, −3.382, −0.430) m. It equals
  the offset between the CooperScene ENU origin (33.975676, −117.339873, 273.0)
  and the map origin (33.9757064853, −117.3398869498, z_ref 273.43 m) to 6 mm.
- **`p_geo(t)`.** Raw `raw_gnss.csv` latitude, longitude and altitude,
  interpolated to the scan's LiDAR timestamp and converted directly into map
  ENU with the georeference. `F_a` is `diag(1,-1,-1)` for agent 1 and the
  identity for agents 2 and 3.
- **`G`.** One correction shared across vehicles, applied about the map origin:
  (−0.061, 0.344, −0.475) m and roll, pitch, yaw (0.03, 0.35, −0.02)°. Matrix
  and provenance: `scripts/investigation/cooperscene_geodetic_map_tie.json`.
- **Reliability.**
  - Across leave-one-take-out folds, `G` varies by at most 2.8 cm and 0.03°.
  - Fitting with vehicles 2 and 3, or with all three, gives the same `G` within
    4 cm and 0.03°.
  - On held-out takes, median road height against the map is 0.059 m for
    vehicle 2 (p90 0.097 m) and 0.071 m for vehicle 3 (p90 0.154 m).
- **Likely cause of `G`.** A difference between the map's Street View-derived
  georeference and RTK GNSS. The −0.48 m vertical is probably a height-datum
  difference.
- **Sensitivity to the 3 cm antenna estimate.** The estimate is unverified.
  Because the antenna is directly above the Xsens, an error in it is almost
  purely vertical. It shifts `G`'s vertical term roughly one-for-one and cannot
  be separated from the datum offset. Going from 0 to 3 cm moved the uncorrected
  road height by about 2.6 cm. `G`'s horizontal terms, its rotation and the
  held-out accuracy do not depend on it.

Details: `geodetic-vehicle-map-tie.md`.

## Why the released frame fails

- **The release composed poses wrongly.** It applied `lidar_T_xsens` on the
  world side, `B = E @ S_raw @ F`. It then added one ICP correction per take and
  vehicle, fitted at a single frame. The result is error that changes with
  heading and time, from about 1–2 cm near each take's ICP frame to 0.1–0.8 m
  elsewhere. See `cooperscene_release_refinement_audit.md`,
  `vehicle-pose-timing-diagnostic.md` and `take-vehicle-map-alignment.md`.
- **Raw geodetic poses don't drift.** They stay nearly flat within each take,
  where released poses swing by up to ±0.8 m. See `geodetic-vehicle-map-tie.md`.
- **The infrastructure has no GNSS.** Its `raw_gnss.csv` is byte-identical to
  vehicle 1's. Its pose exists only in the released frame, so placing it in
  GSplat still needs its own tie, for example through the geodetically placed
  vehicles.

## Approaches tried and rejected

All are in `docs/investigations/` on this branch.

| Approach | Result | Report |
|---|---|---|
| One shared `D` fitted from combined infrastructure and vehicle LiDAR, released poses | Unstable: up to about 0.9 m at 60 m when one vehicle is left out. Vehicles disagree with each other by 0.26 m median per road tile. | `combined-cloud-enu-calibration.md` |
| Constant correction per vehicle or per take and vehicle, world or sensor frame | Worse than released poses on unseen frames | `vehicle-specific-pose-corrections.md` |
| Interpolating poses to LiDAR time (timing gap) | Real effect, mainly yaw, but walls improve only about 9% | `vehicle-pose-timing-diagnostic.md` |
| Smooth spline trajectory refinement | Fails the predeclared rule. Helps only where the infrastructure anchors a vehicle, and hurts vehicle 3. | `vehicle-trajectory-refinement.md` |

## Open items

1. **Vehicle-specific error.** Under `G`, vehicles 2 and 3 still disagree by
   about 0.33 m LiDAR-to-LiDAR. A per-vehicle correction in the vehicle frame
   reduces this to about 0.25 m. Vehicle 3's correction is very stable across
   folds, under 1 cm and 0.03°: (0.05, −0.37, −0.53) m and (−1.00, 1.21, 0.17)°.
   The most likely cause is error in the recorded `lidar_T_xsens`. The user
   expects vehicle-specific, and possibly take-specific, terms here. Fit them as
   per-vehicle pose or calibration terms, not as part of `G`. Validate leave-one-
   take-out and with vehicle-to-vehicle agreement, which needs no map.
2. **Take-specific segments.** Vehicle 2 sits 0.2–0.3 m low in the first 10 s
   of takes 1 and 4. Vehicle 3 drops at the ends of takes 1, 2 and 5; every pose
   family drops there, so the local map surface is the likely cause.
3. **Vehicle 1.** Its vehicle-frame correction of about (1.9, 2.2, 3.1)° nearly
   cancels its 3.46° recorded mounting rotation, so `tower1` `lidar_T_xsens` is
   probably wrong. Take 3 of vehicle 1 is RTK-float.
4. **Map limits.** The GSplat surface is a layer about 0.35 m thick, so wall
   residuals floor around 0.13–0.27 m. A 0.1 m accuracy claim against this map
   is marginal.
5. **Infrastructure in GSplat.** It still needs its own tie; see above.
6. **Unconfirmed device settings.** It is unknown whether the Xsens output
   position is at the antenna or the IMU; at 3 cm this barely matters. The
   altitude datum is also unconfirmed. The configuration block in the `.mtb`
   files is encoded, so only Xsens MT Manager could decode it.

## Scripts and reproduction

The core pipeline, in dependency order:

```bash
PY=/home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python
cd ~/Documents/CISL-Projects/RemoteTeleop-reference-refinement
OMP_NUM_THREADS=4 $PY scripts/investigation/calibrate_shared_enu_map.py
OMP_NUM_THREADS=2 $PY scripts/investigation/select_infra_overlap_frames.py --workers 3
OMP_NUM_THREADS=2 $PY scripts/investigation/audit_structure_time_windows.py --prepare
OMP_NUM_THREADS=2 $PY scripts/investigation/diagnose_vehicle_pose_timing.py --stage timelines
OMP_NUM_THREADS=2 $PY scripts/investigation/refine_vehicle_trajectories.py
OMP_NUM_THREADS=4 $PY scripts/investigation/geodetic_vehicle_map_tie.py
```

- **Shared calibration** (about 10 min). Writes the shared calibration inputs
  and `D`. The rebuilt `D` matched the committed one to 7e-16.
- **Overlap selection** (about 17 min). Matched all 15 documented winners.
- **Time-window manifest.** 123 frames.
- **Pose timelines.** 6,705 vehicle frames.
- **Trajectory refinement** (about 20 min). Also creates the 2 Hz frame cache
  that the geodetic script needs.
- **Geodetic tie** (about 20 min). Produces the main result.

`calibrate_shared_enu_map.py` and `select_infra_overlap_frames.py` read the
eight original comparison frames from `cooperscene_legacy_cases.json` when the
lost `final-comparison/diagnostics.json` is absent.

Other experiment scripts have their own `--help`:

- `design_combined_enu_calibration.py`
- `render_combined_enu_calibration.py`
- `fit_vehicle_sensor_corrections.py`
- `diagnose_vehicle_pose_timing.py`
- `render_take_vehicle_map_alignment.py`

Committed records:

- `cooperscene_geodetic_map_tie.json` holds `G` and its composition.
- `cooperscene_shared_enu_map.json` holds the old `D` and `W`.
- `cooperscene_combined_enu_candidates.json` holds the rejected combined-cloud
  candidates.
- `cooperscene_legacy_cases.json` holds the eight comparison frames.

Branch commits since `a6b7390`, newest first: 4ea968b, 1befe7a, 3498300,
4ab1664, 2a3d90f, 62928e1, d25f1c7, 5fc7dc7, 1fcc67e, 60a9d80.

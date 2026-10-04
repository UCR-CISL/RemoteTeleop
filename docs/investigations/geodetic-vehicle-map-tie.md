# Geodetic map tie for vehicles 2 and 3

## Answer

One transform serves both vehicles 2 and 3. It maps their raw RTK GNSS/INS
poses into the GSplat map and needs no infrastructure, release ICP or fitted
lever arm. On held-out takes it brings median road height to 0.059 m for
vehicle 2 and 0.071 m for vehicle 3.

```text
map_T_lidar(t) = G @ Pose(R_INS(t) @ F_a, p_geo(t) - R_INS(t) @ [0, 0, 0.03]) @ inverse(lidar_T_xsens_a)
```

- **`p_geo(t)`.** Raw `Latitude`, `Longitude` and `Altitude` from
  `raw_gnss.csv` at 400 Hz, interpolated to the scan's LiDAR timestamp and
  converted into the map's georeferenced ENU frame. The origin is
  (33.9757064853, −117.3398869498), with z equal to elevation minus 273.43 m.
- **`R_INS(t)`.** Raw Xsens orientation. `F_a` is the identity for vehicles 2
  and 3.
- **`[0, 0, 0.03]`.** The antenna, mounted directly on top of the Xsens about
  3 cm up, as you described. Nothing is fitted here.
- **`lidar_T_xsens_a`.** The recorded calibration in `tower{N}-tfs.pkl`,
  applied on the vehicle side.
- **`G`.** One rigid correction shared by both vehicles, applied about the map
  origin:

| | Value | Spread across held-out-take folds |
|---|---|---|
| Translation (east, north, up) | (−0.061, 0.344, −0.475) m | ≤2.8 cm |
| Rotation (roll, pitch, yaw) | (0.03, 0.35, −0.02)° | ≤0.03° |

The exact matrix is in `scripts/investigation/cooperscene_geodetic_map_tie.json`.

## Validation

`G` is fitted on four takes and scored on the fifth, for every take. Medians are
over frames.

| Vehicle | Pose | Road height, median / p90 | Within-take spread | Walls |
|---|---|---|---|---|
| 2 | Released + `D` | 0.107 / 0.400 m | 0.041 m | 0.287 m |
| 2 | Geodetic, no `G` | +0.307 / 0.474 m | 0.089 m | 0.317 m |
| 2 | Geodetic + shared `G` | 0.059 / 0.097 m | 0.048 m | 0.268 m |
| 3 | Released + `D` | 0.342 / 0.403 m | 0.169 m | 0.381 m |
| 3 | Geodetic, no `G` | +0.460 / 0.603 m | 0.104 m | 0.227 m |
| 3 | Geodetic + shared `G` | 0.071 / 0.154 m | 0.098 m | 0.160 m |

## Why one shared transform is credible

- **The vehicles agree on it.** Fitting `G` on vehicles 2 and 3 gives
  (−0.06, 0.34, −0.48) m and 0.35° of pitch. Adding vehicle 1 gives
  (−0.10, 0.33, −0.48) m and 0.38°. A transform shared across vehicles points to
  the map's georeference rather than to any one vehicle.
- **It is stable.** Across held-out-take folds it varies by at most 2.8 cm and
  0.03°.
- **Its size fits a georeference difference.** The map's horizontal reference
  comes from Google Street View panorama GPS, and its heights from Street View
  elevations. A 0.34 m north offset, a 0.48 m height offset and a 0.35° tilt over
  the 70 m site are plausible for that source against RTK GNSS.
- **Geodetic poses don't drift.** With no fitting at all, they stay nearly flat
  within each take, where released poses swing by up to ±0.8 m. The release
  applied the mounting calibration on the world side, which produces exactly
  that kind of heading-dependent, time-varying error.

## What remains

- **Vehicle 2 and 3 still disagree with each other.** A shared world transform
  cannot change their relative geometry, so their median separation stays
  0.33 m. Released poses give 0.35 m. A per-vehicle correction in the vehicle
  frame reduces it to 0.25 m. That correction is very stable for vehicle 3,
  within 1 cm and 0.03° across folds. So part of the remaining disagreement is
  vehicle-specific, most likely error in the recorded `lidar_T_xsens`.
- **Some segments still miss.** Vehicle 2 sits 0.2–0.3 m low in the first 10 s
  of take 1 and take 4. Vehicle 3 drops at the end of takes 1, 2 and 5. Every
  pose family drops together at those take ends, so the local map surface is
  the likely cause.
- **Walls stop near 0.16–0.27 m.** That is close to the floor set by the GSplat
  surface, which is about 0.35 m thick.
- **Vehicle 1 is not covered.** Its vehicle-frame correction nearly cancels its
  full 3.5° mounting rotation, which suggests its recorded `lidar_T_xsens`
  rotation is wrong. It is also RTK-float in take 3.
- **The map is the reference.** `G` is fitted to GSplat, so map agreement is
  consistency, not independent ground truth. The independent supports are that
  `G` is shared across vehicles and stable, and that the geodetic poses need no
  ICP and show no drift.

## Supporting facts

- **`W` is geodetic.** The inherited `W` equals the offset between the
  CooperScene and map ENU origins to 6 mm.
- **The GNSS was RTK-fixed.** That held for all sampled frames except vehicle 1
  in take 3.
- **The mounting tilts differ by vehicle.** The `lidar_T_xsens` tilts are 3.46°
  for vehicle 1, 0.06° for vehicle 2 and 0.37° for vehicle 3. The infrastructure
  has no GNSS of its own: its `raw_gnss.csv` matches vehicle 1's byte for byte.
- **No lever arm is readable from the logs.** The Xsens `.mtb` files contain a
  device header and an encoded 1,320-byte settings block. The `.txt` exports
  record the RTK filter profile with orientation and position smoothing, but no
  lever arm. The SensorPlatform sources contain none either.

## Recommendation

- **Candidate reference for vehicles 2 and 3.** Use the geodetic pose with the
  shared `G`, labelled as conditional on the recorded `lidar_T_xsens` and the
  3 cm antenna offset. It needs no infrastructure or release ICP, and it can be
  computed for every frame.
- **For the remaining vehicle-specific error, choose one:**
  - Report it as part of the reference uncertainty, about 0.1–0.15 m at the
    p90 of road height.
  - Adopt the vehicle-frame correction for vehicle 3 only, which is the vehicle
    where it is very stable. Treat it as a calibration estimate fitted to the
    map.
- **Vehicle 1.** Recheck its `lidar_T_xsens` rotation before using it.

## Artifacts and reproduction

Everything is in `artifacts/reference-refinement/geodetic-vehicle-map-tie/`.
`results.json` holds every score, transform and per-frame row.
`figures/road-height-over-time.png` plots road height for every take, vehicle
and pose family. `run1-world-correction/` and `run2-fitted-lever-arm/` hold the
earlier runs.

```bash
OMP_NUM_THREADS=4 /home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python \
  scripts/investigation/geodetic_vehicle_map_tie.py
```

It reads the raw release logs, the tower calibration files, the map georeference
and the 2 Hz frame cache from `refine_vehicle_trajectories.py`. All sources are
read-only.

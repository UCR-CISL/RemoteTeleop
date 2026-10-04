# Geodetic map tie for vehicles 2 and 3

## Answer

A per-vehicle transform from CooperScene to the GSplat map can be estimated for
vehicles 2 and 3 without the infrastructure and without the release ICP:

- **Geodetic pose.** Raw RTK GNSS/INS, interpolated to each scan's LiDAR time
  and converted into the map's own ENU frame through the map georeference.
- **Mounting composition.** The release mounting calibration applied on the
  vehicle side.
- **One fixed vehicle-frame correction per vehicle**, fitted to the map, which
  absorbs lever arm and mounting error.

Fitted on four takes and tested on the fifth, the result is:

| Vehicle | Pose | Road height, median / p90 | Within-take spread | Walls |
|---|---|---|---|---|
| 2 | Released + `D` | 0.107 / 0.400 m | 0.041 m | 0.287 m |
| 2 | Geodetic + world-frame correction | 0.059 / 0.079 m | 0.046 m | 0.223 m |
| 2 | Geodetic + vehicle-frame correction | 0.087 / 0.214 m | 0.107 m | 0.218 m |
| 3 | Released + `D` | 0.342 / 0.403 m | 0.169 m | 0.381 m |
| 3 | Geodetic + vehicle-frame correction | 0.049 / 0.117 m | 0.071 m | 0.130 m |

The correction is fitted to the GSplat map, so map agreement alone is not
independent proof. Three findings support it without relying on that fit:

- **Stable across takes.** Fitted on different four-take subsets, vehicle 3's
  vehicle-frame correction varies by under 1 cm and 0.03°, and vehicle 2's by
  under 7.5 cm and 0.1°.
- **Better vehicle-to-vehicle agreement, with no map involved.** Median
  separation at identical frame IDs falls from 0.340 m to 0.259 m. Vehicles 1–2
  go from 0.303 m to 0.131 m and vehicles 2–3 from 0.354 m to 0.244 m.
- **No drift without any fitting.** Raw geodetic poses stay nearly flat within
  each take where released poses swing by up to ±0.8 m.

The predeclared rule for raw geodetic poses, with no correction, is not met.
They sit +0.33 m (vehicle 2) and +0.48 m (vehicle 3) above the map, and they do
not improve vehicle-pair agreement. The fixed correction is what makes them
usable.

Released poses, production defaults, evaluation queues and `/mnt` sources are
unchanged. This is consistency with GSplat and between LiDARs, not ground truth.

## Evidence

- **The map is georeferenced.** Its ENU origin is (33.9757064853,
  −117.3398869498), with z equal to orthometric elevation minus 273.43 m.
  Heights are tied to Street View elevations to about 0.15 m (90th percentile).
- **`W` is geodetic.** The inherited `W` equals the offset between the
  CooperScene ENU origin and the map origin to 6 mm. The tie in use so far was
  geodesy plus the fitted `D`.
- **The vehicles' GNSS was RTK-fixed.** That held for every sampled frame
  except vehicle 1 in take 3, which was RTK-float throughout.
- **Mounting tilts differ by vehicle.** The `xsens→lidar` calibration tilts
  are 3.46° for vehicle 1, 0.06° for vehicle 2 and 0.37° for vehicle 3. The
  release applied them on the world side, which produces heading-dependent,
  time-varying error. That matches the drift found in earlier reports. The
  infrastructure has no GNSS of its own: its `raw_gnss.csv` is byte-identical
  to vehicle 1's.

## Fitted per-vehicle corrections

Mean over the five leave-one-take-out folds of the vehicle-frame correction, in
the LiDAR frame:

| Vehicle | Translation | Fold spread | Rotation roll / pitch / yaw | Fold spread |
|---|---|---|---|---|
| 2 | (0.27, 0.37, −0.37) m | ≤7.5 cm | (−0.13, 0.23, −0.85)° | ≤0.10° |
| 3 | (0.05, −0.37, −0.53) m | ≤1 cm | (−1.00, 1.21, 0.17)° | ≤0.03° |

The vertical components, −0.37 m and −0.53 m, match the raw geodetic height
excess. That is consistent with the GNSS position referring to a point above the
LiDAR, such as the antenna. Vehicle 3's roughly 1° roll and pitch suggest a
mounting-angle error.

## Limits

- **Vehicle 2's two corrections each win somewhere.** The world-frame
  correction is better on road height and steadier within takes. The
  vehicle-frame correction is better for vehicle-to-vehicle agreement and
  similar on walls. Neither is decisive yet.
- **Walls stop near 0.13–0.22 m.** That is close to the floor set by the
  GSplat surface, which is about 0.35 m thick.
- **Some take ends drop sharply for every pose.** Vehicle 3 at the end of
  takes 1 and 2 is an example. Because every pose drops together there, it
  likely reflects the local map surface rather than the vehicle pose.
- **Vehicle 1 is not covered.** Its vehicle-frame correction comes out at
  about (1.9, 2.2, 3.1)°, close to its full mounting rotation of
  (2.5, 2.4, 3.3)°. That suggests vehicle 1's mounting rotation is wrong. Its
  take 3 is RTK-float, and its road-height p90 stays at 0.72 m.
- **Assumptions remain unverified.** The GNSS reference point, the INS axis
  convention and the altitude datum are untested physical assumptions.

## Recommendation and new evidence to gather

- **Candidate reference.** Use the geodetic pose plus the per-vehicle
  correction for vehicles 2 and 3, labelled as a conditional reference.
  Replace `released @ D` for those vehicles only after an independent check.
- **Measure the GNSS antenna position relative to the LiDAR** on vehicles 2
  and 3 with a tape measure or total station. The fitted lever arms predict
  (0.27, 0.37, −0.37) m and (0.05, −0.37, −0.53) m. A match within a few
  centimetres would independently confirm the tie. A mismatch would show the
  correction is absorbing map or datum error. This is quick in-person work and
  the most decisive evidence available.
- **Confirm the Xsens configuration.** Check whether output position is at the
  antenna or the IMU origin, and whether altitude is orthometric. The logged
  heights already agree with orthometric elevation to within decimetres.
- **Vehicle 1.** Recheck its mounting calibration orientation before using it.

## Artifacts and reproduction

Everything is in `artifacts/reference-refinement/geodetic-vehicle-map-tie/`.
`results.json` holds the checks, RTK status, scores, pair agreement, corrections
and per-frame rows. `figures/road-height-over-time.png` plots road height per
take and vehicle for every pose family. `run1-world-correction/` holds the first
run, without the vehicle-frame variant.

```bash
OMP_NUM_THREADS=4 /home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python \
  scripts/investigation/geodetic_vehicle_map_tie.py
```

It reads the raw release logs, the calibration pickles, the map georeference and
the 2 Hz frame cache from the trajectory refinement. Sources are read-only.

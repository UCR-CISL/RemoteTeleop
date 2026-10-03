# What makes released vehicle poses drift: timing, drift or scan distortion

## Answer

The time-varying vehicle pose error has three parts. None of them is scan
distortion.

1. **A timing gap, mainly in yaw.** Each scan is stamped 0–90 ms after the pose
   sample it is paired with. That gap is nearly constant within a take and
   vehicle. Interpolating poses to scan time with no fitted parameters predicts
   the measured yaw error well, with Spearman correlation 0.89. It removes about
   two thirds of the yaw error variance. Overall it improves walls by about 9%
   and leaves road height unchanged. It is real but not the main cause.
2. **Slow drift.** Height and horizontal error grow with time from the release's
   ICP fitting frame. Even after the timing correction, wall error correlates
   with that time at 0.49. On consecutive scans, height changes smoothly.
3. **Scan-to-scan tilt jitter.** Measured roll and pitch error changes by
   0.24–0.31° from one 10 Hz scan to the next for vehicles 1 and 2, and by
   0.87° for vehicle 3. The same measurement on stationary infrastructure scans
   gives 0.005°, so the jitter is in the poses, not the measurement. At 30 m,
   0.3° is about 0.16 m.

The smooth parts could be modelled over time. The tilt jitter cannot, because
it changes faster than any smooth model allows. With these released vehicle
poses, a reference accurate to under 0.1 m everywhere would need per-scan
registration. That conflicts with the rule against per-frame corrections, so it
is your decision.

Released poses, production defaults, evaluation queues and `/mnt` sources are
unchanged. Everything is geometric consistency against stationary
infrastructure, not ground truth.

## Method

- **Inputs.** The same 584 vehicle frames as the per-vehicle correction
  experiment. Archive `metadata.json` supplies GNSS and LiDAR timestamps, and
  YAML poses for all 6,705 vehicle frames.
- **Zero-parameter timing model.** `S(a) @ inverse(I(a)) @ I(f)`. `I` is the
  released pose interpolated to the scan's LiDAR time, and `a` is the release
  ICP fitting frame. It equals the released pose at `a` to 7e-15.
- **Clock offset.** One optional global clock offset, chosen only within
  leave-one-take-out folds.
- **Scoring.** Every model is scored on one shared point set per frame, with
  residuals capped at 1 m.
- **Per-frame measurement.** A diagnostic rigid fit of each frame to the
  infrastructure, never applied. It is kept for 257 well-constrained frames.
- **Decision rules.** Written to `settings.json` before scoring.

A first pass kept no frames for the per-frame check because of a conditioning
filter in mixed units. It also stopped on a NaN before saving. It is preserved
as `run1-nan-crash.log` and superseded.

## Results

**Timing gap per take and vehicle.** Medians range from 0.2 ms to 90 ms. Within
each take and vehicle, the 5–95% range spans about 4 ms or less. The vehicles reach 10–13 m/s.

**Pose families on all frames.** 562 frames with shared support, median over
frames.

| Pose | Road height | Wall separation |
|---|---:|---:|
| Released | 0.075 m | 0.272 m |
| Timing-anchored, zero parameters | 0.076 m | 0.247 m |
| Timing-anchored, fold-fitted clock offset | 0.075 m | 0.261 m |

Vehicle 1 walls improve the most, from 0.277 m to 0.216 m. Interpolating without
the fitting-frame anchor changes little, at 0.071 m and 0.269 m against released
0.075 m and 0.265 m, scored as a separate pair.

**Clock offset.** The leave-one-take-out choice is between −20 and +10 ms. The
all-take residual curve is flat between −20 and +10 ms and minimal at −10 ms.
The LiDAR timestamps need no extra shift.

**Predeclared rules.**

| Hypothesis | Measured | Rule | Verdict |
|---|---|---|---|
| Timing is the main cause | Ground +1%, walls −9%; horizontal slope 1.26, R² 0.23 | ≥20% on both; slope 0.5–1.5 with R² ≥ 0.3 | Not met |
| Residual drift | Wall Spearman 0.49 with time, p < 1e-30; ground 0.19 | ≥ 0.3, p < 0.01 | Supported for walls |
| Scan distortion | Non-rigid residual versus turn rate −0.32 | ≥ +0.3 | Not supported, opposite sign |

**Per-frame measured error**, 257 frames:

| Time from ICP fitting frame | Median tilt | Median height |
|---|---:|---:|
| Under 3 s | 0.49° | 0.07 m |
| 3–10 s | 0.73° | 0.20 m |
| Over 10 s | 0.76° | 0.25 m |

Median tilt by vehicle is 0.75° for vehicle 1, 0.54° for vehicle 2 and 1.02° for
vehicle 3. Measured yaw is about 1.5 times the timing prediction but follows it
closely. Yaw error RMS falls from 0.81° to 0.46° after subtracting the
prediction.

**Smoothness on consecutive 10 Hz scans.** Each value is the scan-to-scan change
against the spread over about 4 s, as RMS.

| Window | Height | Horizontal | Tilt | Yaw |
|---|---|---|---|---|
| Take 1, vehicle 1 | 0.034 / 0.108 m | 0.065 / 0.075 m | 0.24 / 0.31° | 0.12 / 0.15° |
| Take 2, vehicle 2 | 0.022 / 0.018 m | 0.071 / 0.202 m | 0.31 / 0.32° | 0.14 / 0.42° |
| Take 2, vehicle 3 | 0.25 / 0.20 m | 0.13 / 0.18 m | 0.87 / 1.08° | 0.24 / 0.39° |

When the scan-to-scan change is much smaller than the spread, the error is
smooth, as for vehicle 1 height and vehicle 2 horizontal and yaw. When they are
similar, the error is jitter, as for tilt everywhere and all of vehicle 3. On
stationary infrastructure scans the same measurement gives 0.005° median tilt
and millimetres of translation (`measurement-noise-floor.json`). Vehicle
viewpoints make their floor somewhat higher, but not by a factor of fifty.

**Downstream effects.** The timing correction does not change the spread
between sensors on shared road tiles. It lowers the shared map transform's
leave-one-sensor-out sensitivity from 0.81 m to 0.60 m at 60 m. Held-out map
residuals change little for ground, 0.204 m against 0.200 m, and improve for
walls, 0.208 m against 0.236 m.

## Recommendation

Choose what the evaluation reference should be, because the evidence now limits
the options.

- **Keep released poses, restrict where they are trusted.** Within about 3 s of
  each take's ICP fitting frame, and within about 30 m of the infrastructure,
  vehicles 1 and 2 agree with the infrastructure to 1–2 cm in road height.
  Evaluating localization only on such frames is honest but covers few frames.
  It does not cover vehicle 3.
- **Smooth trajectory refinement.** Combine the zero-parameter timing correction
  with a smooth spline over time per take and vehicle, fitted to infrastructure
  and vehicle-to-vehicle overlap and validated on held-out windows. This would
  remove drift and most yaw error, but leave about 0.25–0.3° of tilt jitter,
  roughly 0.1–0.15 m at 30 m. It will not reach 0.1 m everywhere.
- **Per-scan registration.** Register every vehicle scan to the infrastructure
  and to the other vehicles. Only this addresses the tilt jitter. It is
  per-frame correction, it would be correlated with LiDAR-based localization
  under evaluation, and vehicle 3 rarely has enough overlap.

If the target stays under 0.1 m, the second option is the most defensible next
build, with its remaining error stated explicitly. The timing interpolation is
safe to include: it has no parameters, keeps the release anchor exactly, and
improved walls in every all-frame comparison here.

## Artifacts and reproduction

Everything is in `artifacts/reference-refinement/vehicle-pose-timing/`.

- `results.json` holds checks, gaps, clock-offset curves, the comparisons, the
  per-frame measurements, the drift and distortion tests, tile spread and the
  map stage.
- `settings.json` holds the predeclared rules, `measurement-noise-floor.json`
  the stationary-scan control, and `timelines.json` the cached poses and
  timestamps read from the archives.
- Figures in `figures/`:
  - `measured-vs-predicted.png` compares measured and timing-predicted corrections.
  - `error-vs-time-timing.png` shows error against time from the fitting frame.
  - `clock-offset-curve.png` shows the residual against an added clock offset.
  - `structure-crops-timing.png` shows fixed façade and road patches.

```bash
PY=/home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python
OMP_NUM_THREADS=4 $PY scripts/investigation/diagnose_vehicle_pose_timing.py
```

It requires `vehicle-sensor-corrections/frames.npz` from
`fit_vehicle_sensor_corrections.py`. The stationary-scan control is the
script's `measurement_noise_floor` method and writes `measurement-noise-floor.json`.

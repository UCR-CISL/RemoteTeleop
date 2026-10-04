# Smooth time-varying vehicle trajectory refinement

## Result

The refinement fails the adoption rule declared before fitting, so it is not
adopted. It works where the stationary infrastructure can anchor it and hurts
where it cannot.

- **Vehicle 1, takes 1–2, well anchored.** On held-out time blocks, road
  height improves from 0.075 m to 0.014 m and wall separation from 0.30 m to
  0.105 m. That is close to the 0.1 m goal.
- **Vehicle 3, never anchored.** At 2 Hz, no vehicle 3 frame has enough
  infrastructure overlap in any take. Vehicle-to-vehicle constraints alone make
  it worse: held-out road height goes from 0.144 m to 0.192 m.
- **Near the release fitting frame.** Released poses are already 0.004 m there
  for vehicle 1. A 10 s spline fitted mostly to distant frames degrades those
  frames to 0.109 m.

Released poses, production defaults, evaluation queues and `/mnt` sources are
unchanged. All numbers are consistency with stationary infrastructure, not
ground truth.

## Method

- **Base pose.** The timing-anchored pose: released poses interpolated to LiDAR
  time, exactly equal to released at each take's ICP fitting frame.
- **Correction.** Per take and vehicle, a cubic B-spline over time of a rotation
  about the vehicle position plus a translation. A weak size prior and a
  second-difference smoothness penalty are applied.
- **Constraints.** Point-to-plane against the stationary infrastructure. In the
  joint variant, also vehicle against vehicle at the same frame ID.
- **Frames.** 1,247 training frames at 2 Hz. The 30 reserved scans and 123
  time-window frames are held out, with a 0.5 s exclusion zone around each.
- **Knot spacing.** Constant, 10 s, 5 s or 2.5 s, chosen by five-fold blocked
  cross-validation over time within each take. The validation scored only the
  joint variant.
- **Adoption rule.** At least 10% improvement in both road height and walls
  versus released and timing-anchored poses, on held-out blocks and on reserved
  and time-window frames, without worse vehicle-pair separation.

Before the run completed, the `/tmp` worktree and its git-ignored inputs were
lost in a reboot. They were rebuilt from the archives and committed records and
checked against committed results. The shared `D` matches to 6.7e-16, and all
15 overlap-selection winners match.

## Results

**Cross-validation, held-out time blocks, all vehicles.** Medians in meters.

| Knot spacing | Road height | Walls | Vehicle pairs |
|---|---:|---:|---:|
| Released | 0.088 | 0.303 | 0.345 |
| Timing-anchored | 0.092 | 0.290 | 0.368 |
| Constant | 0.068 | 0.279 | 0.282 |
| 10 s (chosen) | 0.055 | 0.279 | 0.285 |
| 5 s | 0.064 | 0.273 | 0.299 |
| 2.5 s | 0.069 | 0.268 | 0.311 |

At 10 s, the change per vehicle is:

| Vehicle | Road height | Walls |
|---|---|---|
| 1 | 0.075 → 0.014 m | 0.300 → 0.105 m |
| 2 | 0.055 → 0.032 m | 0.276 → 0.287 m |
| 3 | 0.144 → 0.192 m | 0.431 → 0.323 m |

**Reserved and time-window frames.** 153 frames, never fitted.

| Pose | Road height | Walls | Vehicle pairs |
|---|---:|---:|---:|
| Released | 0.016 m | 0.227 m | 0.251 m |
| Timing-anchored | 0.018 m | 0.219 m | 0.246 m |
| 10 s, joint | 0.096 m | 0.225 m | 0.291 m |
| 10 s, infrastructure only | 0.034 m | 0.226 m | 0.244 m |
| 2.5 s, joint | 0.034 m | 0.218 m | 0.214 m |
| 2.5 s, infrastructure only | 0.016 m | 0.216 m | 0.249 m |

These frames mostly lie within 2 s of the fitting frame, where released poses are
already accurate. On vehicle 1's 10 mid-take reserved scans, road height improves from 0.096 m
to 0.021–0.034 m in every refined variant.

**Adoption rule.** On held-out blocks, road height gains 37–40% but walls gain
only 4–8%. On reserved and time-window frames, road height worsens five- to
six-fold. The rule is not met.

**Support at 2 Hz.** Frames with at least 150 infrastructure correspondences:

| Take | Vehicle 1 | Vehicle 2 | Vehicle 3 | Vehicle pairs |
|---|---:|---:|---:|---:|
| 1 | 73 | 0 | 0 | 84 |
| 2 | 158 | 140 | 0 | 307 |
| 3 | 0 | 10 | 0 | 279 |
| 4 | 9 | 6 | 0 | 55 |
| 5 | 6 | 0 | 0 | 94 |

**Shared map transform.** Sensitivity to leaving one sensor out, at 60 m:

| Pose | Displacement |
|---|---:|
| Released | 0.86 m |
| Timing-anchored | 0.44 m |
| 10 s joint refinement | 0.82 m |

## Why it fails

- **No absolute anchor for most data.** Vehicle 3 never sees the
  infrastructure enough to be anchored, and vehicle 2 rarely does. Vehicle-pair
  constraints only fix relative geometry, so errors move between vehicles
  instead of disappearing.
- **The release anchor is sharper than the spline.** Released poses are nearly
  exact at the fitting frame and degrade within seconds. A smooth curve fitted
  mostly away from that frame does not stay exact there.
- **Tilt jitter remains.** Scan-to-scan tilt changes of 0.24–0.87°, measured
  earlier, are faster than any smooth model.

## Recommendation

- **Do not adopt the refinement as a reference.**
- **The timing interpolation alone stays a safe improvement.** It halves the
  shared map transform's sensitivity to leaving out one sensor and slightly
  improves walls everywhere.
- **The only defensible refined reference covers anchored segments.** That is
  the infrastructure-only refinement, applied where frames have infrastructure
  support. This currently means vehicle 1 in takes 1–2 and vehicle 2 in take 2.
  Everywhere else the timing-anchored pose would apply, labelled as unrefined.
  Those segments would need their own validation before use. That is a
  post-hoc proposal, so it needs a fresh predeclared test, not reuse of these
  numbers.
- **Vehicle 3 and most of vehicle 2 need new evidence.** Neither can reach
  0.1 m from this data. The recommendations from the earlier reports still
  apply: static captures beside the infrastructure, or surveyed control points.

## Artifacts and reproduction

Everything is in `artifacts/reference-refinement/vehicle-trajectory-refinement/`
of the worktree at `~/Documents/CISL-Projects/RemoteTeleop-reference-refinement`.

- `results.json` holds the cross-validation, validation, adoption check,
  controls, support and map stage.
- `poses.json` holds every refined pose, and `controls.json` the spline
  coefficients.
- Figures in `figures/`:
  - `cross-validation.png` shows held-out scores by knot spacing.
  - `heldout-error-vs-time.png` shows held-out error against time from the
    fitting frame.
  - `correction-trajectories.png` shows the fitted corrections over time.

```bash
PY=/home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python
OMP_NUM_THREADS=2 $PY scripts/investigation/refine_vehicle_trajectories.py
```

It needs the rebuilt shared calibration, overlap selection, time-window manifest
and `vehicle-pose-timing/timelines.json`. Runtime is about 20 minutes with two
fitting workers.

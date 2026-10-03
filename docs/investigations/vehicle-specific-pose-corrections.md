# Vehicle-specific versus take-and-vehicle pose corrections

## Answer

Neither constant correction works. On frames they were not fitted on, both a
per-vehicle correction and a per-take-and-vehicle correction do worse than the
released poses. This holds in the world frame and in the sensor frame. The error
is time-varying within each take. Near the frame where the release fitted its
ICP correction, vehicle poses already agree with the stationary infrastructure
to about 1–2 cm in ground height. Away from that frame, disagreement grows to
0.1–0.5 m. A model that could fix this needs a correction that changes over time
within each take and vehicle.

No released pose, production default, evaluation queue or `/mnt` source was
changed. Corrections exist only inside this experiment. All numbers measure
LiDAR-to-LiDAR consistency against the stationary infrastructure, not ground truth.

## Protocol

- **Reference.** The union of stationary infrastructure scans. Take-to-take
  infrastructure separation has a median of 1.6 cm. GSplat is not used to fit
  any vehicle correction.
- **Frames.** 584 vehicle frames read read-only from the archives:
  - 431 trajectory frames, evenly spaced through each take, used for fitting.
  - The 30 reserved scans and the 123 earlier time-window frames, used only for
    validation. Trajectory frames within 5 frames of any of them are excluded.
- **Models.**
  - **World frame.** `delta @ W @ N`, one correction per vehicle or per take and vehicle.
  - **Sensor frame.** `W @ N @ delta`, the shape a mounting error would have,
    one correction per vehicle or per take and vehicle.
  - **Audit alternative poses.** The release audit's ICP-anchored reordering
    candidate and its uncorrected sensor baseline, alone and with per-vehicle
    corrections added.
- **Fit.** Robust point-to-plane fit against the infrastructure. Each frame and
  each surface class (road or wall) gets equal weight. A weak prior of about 1°
  and 0.3 m keeps corrections in a plausible mounting range.
- **Scoring.** One fixed point set per frame, shared by every model: points
  within 1 m of the infrastructure under any model. Points a correction pushes
  away are penalized at 1 m, not dropped.

A first run let frames drop out during fitting, so many fits collapsed onto a few
frames. Some returned implausible values, such as 1.0 m and 1.2° fitted on 23 of
144 frames. It is preserved in `vehicle-sensor-corrections-run1-unregularized/`
and superseded.

## Results

**Leave-one-take-out.** Each correction is fitted on four takes and scored on
the fifth. 545 frames, median over frames, in meters.

| Model | Road height | Wall separation |
|---|---:|---:|
| Released | 0.089 | 0.31 |
| Per-vehicle, world frame | 0.100 | 0.41 |
| Per-vehicle, sensor frame | 0.104 | 0.41 |
| Audit reordering candidate | 0.125 | 0.34 |
| Audit candidate + per-vehicle sensor frame | 0.106 | 0.50 |

**Same takes, unseen frames.** Corrections are fitted on trajectory frames of
the same takes and scored on the reserved and time-window frames. 96 frames.

| Model | Road height | Wall separation |
|---|---:|---:|
| Released | 0.013 | 0.16 |
| Per-vehicle, world frame | 0.011 | 0.35 |
| Per-take-and-vehicle, world frame | 0.028 | 0.25 |
| Per-take-and-vehicle, sensor frame | 0.045 | 0.28 |

**Corrections are not stable across takes.** Fitted separately per take, the
same vehicle's sensor-frame correction varies by up to 0.8° in rotation and
0.15–0.37 m in translation, measured as a standard deviation. A fixed mounting
error would give the same correction every take.

**The error grows with time from the release's ICP fitting frame.** This holds
at a fixed distance from the infrastructure. Released poses, vehicles 15–30 m
from the infrastructure, median over frames:

| Time from ICP fitting frame | Road height | Wall separation |
|---|---:|---:|
| Under 3 s | 0.010 m | 0.15 m |
| 3–10 s | 0.017 m | 0.29 m |
| Over 10 s | 0.075 m | 0.31 m |

Frames more than 30 m from the infrastructure are worse at every time, which is
how attitude error appears at longer range. A constant correction fitted across
takes reduces error more than 10 s from the fitting frame. It also breaks the
frames near the fitting frame, from 0.15 m to 0.51 m on walls for the
sensor-frame model. That trade-off is the signature of time-varying error.

**Support is uneven.** Only 5–6 of vehicle 3's 144 trajectory frames overlap the
infrastructure enough to fit. Vehicles 1 and 2 have 75 and 51. Vehicle 3 cannot
be corrected against the infrastructure alone.

**Shared map transform.** Refitting the shared map transform with corrected
vehicles does not stabilize it. Leaving one sensor out still moves it 0.46–1.49 m
at 60 m, against 0.81 m with released poses.

## Recommendation

- **Do not adopt a per-vehicle or per-take-and-vehicle constant correction.**
  Neither generalizes to unseen frames.
- **Do not adopt the audit's reordering candidate.** It is worse than the
  released poses on every validation set.
- **Next experiment: time-varying correction.** Fit a smooth correction over
  time for each take and vehicle, such as a low-order spline in rotation and
  translation. Fit it to infrastructure overlap at frames spread through the
  take, plus vehicle-to-vehicle overlap where the infrastructure is out of view.
  Validate on held-out time windows. This generalizes what the release ICP did
  at a single frame. It is a trajectory refinement, so it must be validated on
  held-out windows to avoid hiding the disagreement it removes.
- **Vehicle 3 needs extra constraints.** Its infrastructure overlap is too thin.
  Vehicle-to-vehicle overlap, or the static captures recommended earlier, would
  supply them.
- **Keep the earlier shared map tie provisional** until vehicle trajectories are
  consistent. The map tie cannot be pinned below the vehicle disagreement.

## Artifacts and reproduction

Everything is in `artifacts/reference-refinement/vehicle-sensor-corrections/`.

- `results.json` holds all fits, validation rows, shared-point-set scores, the
  time analysis, tile spread and map refits. `corrections.json` holds every
  fitted matrix. `frames.npz` caches the frames read from the archives.
- Figures in `figures/`:
  - `error-vs-icp-frame-time.png` shows error against time from the ICP fitting frame.
  - `model-comparison.png` compares models per vehicle on validation frames.
    It scores each model only on points that stay within 1 m of the
    infrastructure under that model, so it favours corrections that push
    points away. Use the tables above, which share one point set per frame.
  - `ground-height-vs-heading.png` plots ground height against vehicle heading.
  - `structure-crops.png` shows fixed façade and road patches.

```bash
PY=/home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python
OMP_NUM_THREADS=4 $PY scripts/investigation/fit_vehicle_sensor_corrections.py
OMP_NUM_THREADS=4 $PY scripts/investigation/fit_vehicle_sensor_corrections.py --stage evaluate
```

The first command fits and scores everything. The second reuses the saved
corrections to redo the time analysis, tile spread, map refits and figures.

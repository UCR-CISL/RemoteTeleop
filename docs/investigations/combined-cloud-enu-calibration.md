# Combined-cloud design of one shared ENU-to-GSplat transform

## Decision

No better-supported single transform was found. Retain the earlier shared tie
(`shared-enu-calibration/calibration.json`) as the provisional comparator, keep
the overlap-selected fit as a diagnostic, and do not adopt the new candidates.

The limiting factor is not sampling, weighting, frame selection or GSplat
surface choice. Released vehicle scans disagree with each other, and with the
stationary infrastructure, by 0.1–0.8 m on the same physical surfaces. A shared
rigid transform cannot remove those differences. The fitted transform therefore
depends on which vehicles and takes are included, by up to about 0.9 m at 60 m.

All released poses, ICP corrections, production defaults, evaluation queues and
`/mnt` sources are unchanged. No teleoperation batch was launched. Every number
below is geometric consistency with the GSplat map or between LiDARs, not
independent ground-truth accuracy.

## Protocol

Composition: `map_T_lidar = D @ original_W @ released_ENU_T_lidar`. Source clouds
already contain `original_W @ released_ENU`; only `D` is fitted. Stored matrices
are read from artifacts.

Declared before fitting (`config.json`):

- **Spatial split.** W-frame 20 m checkerboard tiles. Parity 0 calibrates;
  parity 1 is untouched validation. A swapped-parity fit is reported separately.
- **Validation frames.** The 40 previously reserved scans: two per take and unit,
  all five takes, never used by any fit. A leakage check enforces this.
- **Calibration pool.** 36 items: one infrastructure scan, five infrastructure
  three-frame aggregates, the 15 per-take overlap winners and 15 older
  three-frame vehicle aggregates.
- **Classes from source geometry.** Ground is a near-horizontal normal near the
  local minimum. Vertical has a near-horizontal normal direction. Everything
  else (vegetation, clutter) is evaluated but never fitted.
- **Weighting.** Ground and vertical get equal total weight. Within a class each
  sensor gets equal weight. Within a sensor each 20 m tile and wall-azimuth
  quadrant gets equal weight. Weights are recomputed on surviving
  correspondences every iteration.
- **GSplat surface.** Primary target is opacity > 0.05, a local-centroid plane,
  and local normal spread under 0.10 m. Opacity is treated as a filter, never as
  a weight or a confidence.

A first run balanced weights only before correspondence gating. The
infrastructure then held 44% of the weight instead of 25%. It is preserved in
`combined-cloud-calibration-run1-design-weights/` and superseded.

## Findings

**1. The starting experiment reproduces exactly.** Re-running the prior solver
on the same four clouds gives the stored matrix to 8.9e-16. Before/after top views
and road-band side slabs are in `before-after-starting-set-top.png` and
`-side.png`. The released ENU frame is tilted about 2.5° relative to the map.

**2. Maximum infrastructure overlap is not a complementary-coverage selection.**
Among all 6,000 one-item-per-sensor combinations, the prior set ranks below the
first percentile on unique ground and wall-orientation footprint. Coverage-designed
sets cover two to three times more, with better conditioning. They still do not
validate better, so more coverage alone does not fix the fit.

| Set | Footprint keys | Min. eigenvalue |
|---|---:|---:|
| Prior overlap set | 3,247 | 0.42 |
| Designed, single scans | 5,097 | 0.76 |
| Designed, aggregates allowed | 10,254 | 0.89 |

**3. Balancing matters for walls; it does not fix roads.** Uniform weighting lets
dense ground dominate. Held-out vertical residuals in validation tiles rise to
0.299 m, against 0.216 m when balanced.

**4. The GSplat surface is a thick layer.** The median local normal spread is
0.11 m. Only 2% of neighbourhoods are under 0.05 m. Nearest-center matching
biases residuals toward the near face of that layer. Side slabs also show
stacked ground layers near the west strip. Changing the surface definition
moves the transform by at most 0.21 m, far less than changing the frames.

**5. Held-out validation shows no reliable improvement.** The table uses held-out
scans in untouched tiles and a common support set. It reports the median
absolute point-to-surface residual in meters.

| Transform | Ground | Vertical | Ground signed |
|---|---:|---:|---:|
| Earlier shared tie | 0.171 | 0.182 | −0.067 |
| Overlap-selected | 0.164 | 0.206 | −0.041 |
| Prior 4 clouds, balanced | 0.143 | 0.175 | +0.013 |
| Coverage-designed, aggregates | 0.189 | 0.215 | −0.062 |
| All-pool balanced (predeclared) | 0.145 | 0.216 | +0.042 |

The differences are 2–4 cm. They are small relative to the transform variation
in finding 6 and the sensor disagreement in finding 7.

**6. The transform is unstable under alternative selections.** The table gives
the maximum displacement relative to the candidate over a 120 m box.

| Alternative | Displacement |
|---|---:|
| Leave out vehicle 1 / 2 / 3 | 0.88 / 0.82 / 0.11 m |
| Leave out take 3 / 5 | 0.51 / 0.60 m |
| Leave out take 1, 2 or 4 | about 0.12 m |
| Bootstrap over items, median / p90 | 0.19 / 0.82 m |
| One sensor only | 0.49–1.22 m |
| Earlier shared tie / overlap-selected | 0.74 / 1.07 m |

**7. The observations conflict, and the conflict is in the sources.**

- **Stationary infrastructure is self-consistent.** Take-to-take infrastructure
  separation has a median of 1.6 cm.
- **Vehicle scans are not.** Ground height against that infrastructure ranges
  from −0.38 to +0.26 m per scan. Vehicle 3 scans match the infrastructure
  locally but sit about 0.7 m above the map farther away. That pattern
  indicates per-frame attitude error.
- **Same tile, different heights.** In 38 shared 10 m road tiles, the median
  spread of per-sensor ground height is 0.26 m. 95% of tiles exceed 0.1 m and
  45% exceed 0.3 m. Any shared rigid `D` leaves this unchanged.
- **Same façade, different planes.** On the predeclared north-west upper façade,
  held-out vehicle 1 and vehicle 2 sit about 0.7 m apart under every transform.
- **Different preferred tilts.** Ground-only fits imply tilts that differ by
  sensor, from +0.6° pitch for infrastructure to −1.1° pitch for vehicle 3.
  Wall-only tilts change by more than 1° between the calibration tiles and all
  tiles, so they are weakly supported.

**Attribution.** The causes rank as follows.

1. **Inconsistent source geometry** is the dominant, transform-invariant conflict.
2. **Ambiguous map surfaces** add a residual floor of roughly 0.1 m. Map
   thickness is nearly uniform, so a per-cell ambiguity label is not
   discriminative.
3. **Insufficient support** affects many individual 20 m sensor cells, and
   wall-only tilt. It does not affect the pooled six-degree-of-freedom fit, whose
   scaled condition number is 2.5.

## Recommendation

- Keep the earlier shared tie as the provisional reference. Do not adopt the
  overlap-selected fit or the new candidates. Their held-out gains are within
  the selection sensitivity.
- Do not tune `D` further against the GSplat map. The sources disagree beyond
  the 0.1 m goal before the map enters.
- Gather evidence that separates per-frame vehicle pose error from the map tie.
  - **Static vehicle captures** beside the infrastructure, using fixed surfaces
    such as the north-west façade and the central road. These would test
    released vehicle attitude without motion.
  - **Surveyed control points** visible in both LiDAR and the GSplat imagery.
    These would fix tilt and height independently of vehicle poses.
  - **Per-frame attitude checks** of vehicle 3 against infrastructure ground and
    walls over its full trajectory.

## Artifacts and reproduction

Everything is in `artifacts/reference-refinement/combined-cloud-calibration/`.
`index.html` shows all figures, and `figures/` holds the PNGs. `results.json`
holds every metric, `transforms.json` every matrix, and `tilt-conflict.json`
and `inter-sensor-ground-spread.json` the conflict analyses. The four headline
matrices are also in `scripts/investigation/cooperscene_combined_enu_candidates.json`.

```bash
PY=/home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python
OMP_NUM_THREADS=8 $PY scripts/investigation/design_combined_enu_calibration.py
OMP_NUM_THREADS=8 $PY scripts/investigation/design_combined_enu_calibration.py --stage tilt
OMP_NUM_THREADS=4 $PY scripts/investigation/render_combined_enu_calibration.py
```

Inputs are the cached selected and reserved clouds from earlier investigations.
No archive is reread.

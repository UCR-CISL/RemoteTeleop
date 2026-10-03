# Shared CooperScene ENU / GSplat calibration

One full rigid transform was fitted from the released ICP-refined CooperScene
frame to the existing Riverside GSplat. It uses all three vehicles plus
infrastructure across takes 1–5: 60 calibration scans and 40 separate held-out
scans, sampled chronologically from the original archives. Eight previously
inspected frames are excluded from fitting and checked separately.

## Saved transform

[Compact calibration profile](../../scripts/investigation/cooperscene_shared_enu_map.json)
records the map SHA256, original tie, correction `D`, and final
`map_T_released_enu`. Use column vectors and meters:

```text
map_T_lidar = map_T_released_enu @ released_enu_T_lidar
map_T_released_enu = D @ original_W
```

Do not additionally apply `original_W` when using the final matrix. Raw INS or
LiDAR-local coordinates first require their appropriate released pose composition.
The correction's Euler XYZ rotation is approximately
`[-0.814729, 2.662098, 0.398855]` degrees, and its map-origin translation is
`[0.295009, 0.599425, -1.614461]` meters. This is not a constant local road-height
adjustment. Production configuration is unchanged.

## Results

| Geometry consistency statistic | Before | After |
|---|---:|---:|
| Median scan NN median, 40 held-out scans | 0.482 m | 0.261 m |
| Median road-normal disagreement, previous eight frames | 3.258° | 1.093° |
| Median absolute road-height separation, previous eight frames | 0.631 m | 0.328 m |

All 40 scan medians improved. Seven of eight road-height comparisons improved;
take 5 / vehicle 3 / frame 71 retains +0.647 m separation, and take 1 / vehicle 1 /
frame 0 worsens from −0.356 to −0.405 m. These are geometry residuals, not ATE.

Leaving out one take changes sampled positions by up to 0.196 m at the five
tested locations around the map origin. Leaving out one vehicle changes rotation
by up to 0.415°. A shared rigid calibration helps substantially but cannot resolve
all per-agent or time-dependent disagreement. This is a map-relative calibration
candidate, not independent ground truth. Acceptance gates remain unchanged.

## Reproduction and evidence

Run `calibrate_shared_enu_map.py`, then `validate_shared_enu_map.py` and
`render_shared_enu_calibration.py` with the generated `calibration.json`, followed
by `report_shared_enu_calibration.py`, all under `scripts/investigation/`.
Source archives stay read-only; outputs are separate from earlier experiments.

- [Detailed report](../../artifacts/reference-refinement/shared-enu-calibration/report.md)
- [Cloud and camera gallery](../../artifacts/reference-refinement/shared-enu-calibration/index.html)
- [Fixed-frame validation](../../artifacts/reference-refinement/shared-enu-validation/report.md)

Fitting balances take/unit contributions and near-horizontal versus other
normal categories, with voxelization, vehicle masks, and robust point-to-plane
residuals. Gaussian opacity is not calibrated physical confidence. Original
unit-specific annotation masks are used in the 60/40-scan experiment; the eight
cached cases have extra fused boxes. Every before/after pair uses identical points.
Repeated infrastructure observations are not independent sensors or independent
validation of the released ICP refinements.

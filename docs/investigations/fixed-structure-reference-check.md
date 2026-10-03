# CooperScene: fixed physical structures and temporal consistency

## Decision

The expected shared ENU frame is reproduced, but the selected LiDAR observations do not agree uniformly at the sub-0.1 m level. The same central road patch agrees closely for infrastructure and vehicles 1/3; multiple facade panels show larger vehicle 3 offsets. A new single rigid structure fit improves training patches but substantially worsens withheld roads. Do not promote that transform or re-rank localization as ground-truth accuracy.

All released sensor poses and intentional per-take ICP corrections remain fixed. No production defaults, localization acceptance thresholds, mounted sources or evaluation queues were changed. All new outputs live under `artifacts/reference-refinement/structure-experiment/`.

## Method

Use the prior infrastructure-overlap selection: take 1/vehicle 1 offset417, take 2/vehicle 2 offset835, take 2/vehicle 3 offset834, plus stationary take1/unit0/frame481510. Points already contain `original_W @ released_enu`; the common map correction is `D`, yielding `D @ original_W @ released_enu_T_lidar`.

Manually inspect distinctive corners, stepped facade outlines and separated height bands in top/side cloud views, alongside GSplat RGB centers. Select identical physical panels and road footprints. This substantiates geometric identity, not surveyed semantics or camera-confirmed material. Reject the west raised strip as ambiguous road/curb. Record absent visibility explicitly. Freeze the nine-region manifest before fitting; do not replace failed withheld surfaces with new ones.

Plane comparisons require two-dimensional support, bounded scatter, hull containment and nearby samples at common anchors. Direct return comparisons match tangent coordinates within0.30m and measure separation along the mean surface normal. These diagnostic patch gates are separate from unchanged runtime registration gates. Distances here are surface agreement, not trajectory ATE.

## LiDAR-to-LiDAR results at selected frames

Median absolute orthogonal separation from infrastructure, meters; parentheses contain plane normal disagreement in degrees:

| Physical patch | Vehicle 1 | Vehicle 2 | Vehicle 3 |
|---|---:|---:|---:|
| NW upper facade | 0.024 (1.05) | 0.078 (1.43) | 0.287 (2.91) |
| NE lower facade | 0.066 (2.80) | 0.140 (3.07) | 0.051 (4.16) |
| SW boundary wall | 0.025 (2.71) | 0.024 (2.58) | 0.230 (1.16) |
| NW lower west panel | 0.100 (0.89) | 0.192 (1.17) | 0.334 (1.43) |
| Central road | 0.017 (0.79) | Unsupported: five returns | 0.024 (0.03) |

The vehicle3 facade result is corroborated directly by nearby raw returns: medians0.286m (NW upper),0.209m (SW),0.346m (NW lower). It is not only a fitted-plane intersection effect. The lower panel has only10 nearby infrastructure/vehicle3 return pairs, so its support is weaker. Infrastructure does not support the withheld south/east roads; vehicle-to-vehicle comparisons there show additional gaps, including vehicle1/3 direct-return medians0.218m and0.249m.

A same-take control replaces the take1 infrastructure reference with the existing take2 infrastructure aggregate. Infrastructure take1/take2 differs by median0.004–0.018m and maximum0.039m on the five commonly supported surfaces. Vehicle3 still differs by0.293m (NW upper),0.217m (SW),0.336m (NW lower). Thus the sampled cross-take infrastructure discrepancy does not explain the vehicle3 result.

## One common map transform

Fit only NW upper facade, NE lower facade and central road. Equal total weight per structure, equal sensor weight within each structure. No per-vehicle correction or broad nearest-neighbor matching. Primary map opacity>0.05; retain opacity>0 as a fixed-transform surface sensitivity. The west ambiguous strip never enters the fit.

The solver is numerically rank6 with scaled condition46.2. Numerical solvability does not establish a valid global tie. On identical supported source/map pairs:

| Patch partition | Prior overlap-selected transform | New structure fit |
|---|---:|---:|
| Training facades, median absolute separation | 0.123m | 0.042m |
| Training central road, median absolute separation | 0.0145m | 0.0110m |
| Withheld lower facade, median absolute separation | 0.230m | 0.118m |
| Withheld roads, median absolute separation | 0.042m | 0.648m |
| Withheld roads, maximum absolute separation | 0.201m | 0.936m |

The candidate also loses support at some fixed anchors. Detailed metrics retain those failures; the table uses common supported pairs, not zeros for missing data. Positive-opacity map extraction also shows withheld road degradation (~0.615m median), so the effect is not removed by that opacity change. The transform is a rejected diagnostic candidate. The prior transform is a comparator, not certified truth. Withheld means excluded from this new structure fit: the previous broad fit already used the source clouds, and the NW withheld bay shares the training building.

## Nearby frames, unchanged poses

Read123 original scans,41 per vehicle, nominal±2seconds. Evaluate a single unchanged center anchor per physical patch against stationary infrastructure. Include hull/nearby-sample gates on every frame; gaps in plots mean unsupported.

Vehicle3 NW facade signed separation changes from approximately−0.411m to+0.781m over the window; SW spans−0.722m to+0.197m. Central road is close at the chosen frame but reaches−0.590m at+1.6seconds. Expanded point crops show coherent displaced surfaces, not just plane crossings. Vehicle3 changes heading by about55degrees in these four seconds. Vehicle1 is more stable on the SW wall (−0.083..+0.072m); vehicle2 also changes on multiple facades.

Timestamp skew, speed, heading and anchor sensor azimuth are logged. These observations make time/heading-dependent pose composition, synchronization or scan distortion worth testing. They do not identify a clock correction or prove a specific cause. Azimuth is not measured point acquisition time; no deskew has been performed. A common rigid map transform preserves LiDAR-to-LiDAR discrepancies and therefore cannot remove them.

## Recommendation

Keep the released corrections and current evaluation references explicitly provisional. Next isolate temporal pose composition on the same fixed structures: compare the released timestamp association with carefully interpolated INS orientation/position and documented sensor lever arms. Establish scan timestamp/rotation conventions before attempting deskew. Fit any timing/calibration hypothesis on designated structures/windows and validate on different structures/windows; do not use GSplat or evaluation reference error to tune that correction. A successful hypothesis must improve multiple surfaces simultaneously without per-frame pose fitting.

First retain and inspect these artifacts:

- `lidar/report.md`, `same-surface-overlays.png`, `pair-measurements.json` (including direct-return checks)
- `map/report.md`, `metrics.json`, `calibration.json`, per-surface and identity views
- `temporal/report.md`, `fixed-anchor-time-windows.png`, `temporal-extreme-crops.png`, `measurements.json`
- `index.html` and `gallery.png` for the combined visual report.

## Reproduction

Run with the repository virtual environment and numerical threads limited to one:

```bash
python scripts/investigation/compare_selected_lidar_structures.py --stage inspect
python scripts/investigation/compare_selected_lidar_structures.py --stage measure
python scripts/investigation/compare_selected_lidar_structures.py --stage take_control
python scripts/investigation/fit_verified_structure_map.py
python scripts/investigation/audit_structure_time_windows.py --prepare
```

The frozen configuration is `scripts/investigation/cooperscene_structure_rois.json`. Existing selected-cloud and original archive inputs are prerequisites. The temporal `--prepare` step reads archives without modifying them and writes only separate investigation outputs. See each script's `--help` for output controls.

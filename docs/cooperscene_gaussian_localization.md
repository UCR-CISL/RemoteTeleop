# CooperScene Gaussian-splat localization

This is an offline LiDAR-to-Gaussian-map experiment.  It does not start any
vehicle-side or remote-side process.

## Reproduce the mini result

From the repository root, run:

```bash
.venv/bin/python scripts/localize_cooperscene.py \
  --output-dir artifacts/cooperscene_localization \
  --render --render-device cpu
```

The defaults select CooperScene `data/mini`, split `test`, scenario `1`, and
vehicle agent `1`.  They align the first timestamp synchronized across agents
`0`–`3` (`481410` in the mini set), then propagate the final transform over
all 30 agent-1 frames.

## Deployment with one vehicle

Use `--coarse-agents selected` when other agents are unavailable. This loads
only the selected agent, chooses its requested frame (or its first frame), and
still runs the global coarse registration on that LiDAR scan before refinement:

```bash
.venv/bin/python scripts/localize_cooperscene.py \
  --agent 1 --coarse-agents selected \
  --output-dir artifacts/cooperscene_single_agent_coarse_cuda \
  --render --render-device cuda
```

`--coarse-agents all` is the default and combines synchronized scans from
agents 0–3. `localization.json` records the choice as `coarse_mode` and keeps
the coarse registration metrics for either mode. Use `--skip-coarse` only to
omit that stage entirely (`"coarse_mode": "skipped"` and
`"coarse_registration": null`). Without a coarse stage or
`--initial-transform`, refinement starts from the identity
`gaussian_T_cooperscene` assumption. That is appropriate only when the
CooperScene and Gaussian-map coordinates are already approximately aligned;
it permits a wider first correction (up to 5 m / 10 degrees) while keeping the
normal correspondence-quality gates. An explicit prior retains the normal 1 m
/ 3 degree correction gate and can be supplied with
`--initial-transform previous_localization.json` when it is known to be
compatible with the current scan.

Outputs in the chosen directory are:

- `localization.json`: configuration, transforms, coarse and final metrics.
- `localized_trajectory.npz`: `frame_ids` and Gaussian-map `map_T_lidar`.
- `trajectory.png`: XY pose plot.
- `camera_render.png`: the selected localized front-camera view when `--render`
  is supplied.

LiDAR returns inside the frame's GT vehicle boxes are excluded by default
because the Riverside GSplat contains only static background. The default box
padding is 0.2 m. Use `--keep-dynamic-points` only for an ablation, or adjust
the padding with `--dynamic-box-padding-m`.

## Optional shared-ENU prior fusion

The Riverside PLY has a georeferenced ENU origin and CooperScene's historical
metadata establishes a separate ENU origin. Their common physical intersection
defines the horizontal prior in:

```text
config/cooperscene_riverside_iowa_enu_prior.json
```

Use it as an explicit GNSS/ENU prior after LiDAR ICP:

```bash
.venv/bin/python scripts/localize_cooperscene.py \
  --data-root data/mini --split train --scenario 1 --agent 1 \
  --frame-id 481260 \
  --splat /path/to/riverside_iowa.ply \
  --output-dir artifacts/cooperscene_train1_agent1_enu_prior_map \
  --skip-coarse \
  --initial-transform artifacts/cooperscene_single_agent_coarse_cuda/localization.json \
  --enu-prior-transform config/cooperscene_riverside_iowa_enu_prior.json
```

This deterministic MAP stage keeps ICP's roll, pitch, and vertical translation,
then robustly reoptimizes east, north, and yaw with the ENU pose prior. The JSON
preserves both `scan_registration` and the fused `final_registration` metrics.
For take 1, agent 1, frame 481260, the default prior weight produces a 0.0938 m
BEV discrepancy while the fused point-cloud registration remains accepted.

This is GNSS/ENU + LiDAR fusion, not evidence of sub-0.1 m LiDAR-only accuracy:
the same ENU pose contributes to both the prior and the reported discrepancy.
Use an independent survey or held-out control points for an unbiased accuracy
claim.

The fixed `--enu-prior-weight 0.14` remains the default. An experimental,
uncertainty-parameterized alternative is available when the GNSS/INS position
and heading uncertainties are known:

```bash
.venv/bin/python scripts/localize_cooperscene.py \
  ... \
  --enu-prior-weighting-mode uncertainty \
  --enu-prior-translation-std-m 0.05 \
  --enu-prior-heading-std-deg 1.0
```

Uncertainty mode estimates directional scan-transform covariance from the
LiDAR/Gaussian correspondence Jacobians. It uses 2 m spatial blocks both for a
cluster-sandwich covariance and for a heuristic block-heterogeneity inflation
from the block-local transform corrections; the latter prevents thousands of
correlated points from creating false millimeter-level confidence. It then
combines that covariance with the supplied horizontal-position and heading
standard deviations. These should come from the GNSS/INS solution and map
datum/extrinsic uncertainty in deployment, rather than being tuned against the
reported CooperScene pose.

`enu_prior_fusion` records the scan, prior, and posterior 3x3 covariance
matrices, total/eligible spatial-block counts, data-Hessian spectrum, robust
scan residual scale, and the innovation NIS. The covariance axes are east
translation, north translation, and yaw;
scan/posterior matrices describe `gaussian_T_cooperscene`, while the prior
matrix describes the LiDAR pose. `innovation_exceeds_nominal_chi2_95pct`
compares NIS to the usual three-degree-of-freedom threshold, but is only a
diagnostic: the scan and prior can share datum errors, and the heuristic
inflation has not been calibrated by bootstrap/NEES coverage. An exceeded
threshold warrants investigation, not a stronger prior.

On take 1 / agent 1 / frame 481260, a declared 5 cm horizontal prior and
1 degree heading prior produced a 0.0356 m BEV discrepancy **to that same
prior**. Declared 10 cm and 20 cm horizontal priors produced 0.1218 m and
0.3083 m respectively. This sensitivity result demonstrates automatic
weighting, not independent sub-0.1 m accuracy: the recorded ENU pose is both
the input prior and the discrepancy reference.

### Optional vertical fusion

BEV position fusion remains the default and preserves ICP's vertical
translation. To include height while still leaving roll and pitch under ICP
control, select the separate XYZ mode:

```bash
.venv/bin/python scripts/localize_cooperscene.py \
  ... \
  --enu-prior-position-mode xyz \
  --enu-prior-weighting-mode uncertainty \
  --enu-prior-translation-std-m 0.05 \
  --enu-prior-vertical-std-m 0.15 \
  --enu-prior-heading-std-deg 1.0
```

Fixed weighting also supports `--enu-prior-position-mode xyz`; it applies the
same dimensionless `--enu-prior-weight` to east, north, up, and lever-arm-scaled
yaw. Uncertainty mode instead expands its transform and pose covariance from
3x3 `[east, north, yaw]` to 4x4 `[east, north, up, yaw]` and uses the separate
vertical standard deviation.

The current vertical transform is `273.00 - 273.43 = -0.43 m`, using the
historical CooperScene `lla_origin` numeric elevation and the GSplat's
documented orthometric Z reference. CooperScene does not explicitly label its
LLA height datum, so exact datum equivalence remains unproven. A static-ground
lower-envelope check independently
estimated that the BEV-localized LiDAR should move downward by approximately
0.31–0.36 m across frames 481260, 481320, and 481379. This is close to the
0.291 m selected-pose discrepancy implied by the geodetic transform, suggesting
that the origin conversion is reasonable and the residual comes mainly from
ICP Z/tilt and map-surface correspondence.

This does not make Z survey-grade. The GSplat reports roughly 0.15 m p90
terrain-height residual from Google photometa, CooperScene's GNSS altitude
uncertainty is unavailable, and the ground-envelope estimate has broad local
dispersion. On frame 481260, fixed XYZ fusion at weight 0.14 reduced the 3D
discrepancy from about 0.293 m to 0.162 m while retaining an accepted
point-cloud fit. Treat XYZ mode as an explicit experiment until a surveyed
height control point or trustworthy GNSS vertical covariance is available.

## Frames and calibration

Transforms use `target_T_source` naming.  CooperScene YAML `lidar_pose` is
called `cooperscene_T_lidar`.  Registration estimates
`gaussian_T_cooperscene`; trajectory poses are:

```text
gaussian_T_lidar = gaussian_T_cooperscene @ cooperscene_T_lidar
```

These Gaussian-map poses are saved under the public `map_T_lidar` output name.
For agents 1–3, the documented CooperScene camera calibration is
`camera_T_lidar` in CV camera coordinates.  The render pose is:

```text
gaussian_T_camera = gaussian_T_lidar @ inverse(camera_T_lidar)
```

## Initialization and metrics

The coarse stage registers either one selected-agent scan or (the default) one
synchronized scan from each of agents 0–3 to an opacity-filtered, strided
Gaussian-center proxy. Its RANSAC/FPFH fitness is always recorded. A zero
coarse fitness is **not** silently treated as successful feature matching: the
report preserves it.

At this site, the CooperScene coordinate system and Gaussian map are already
approximately metric and aligned.  Consequently an identity-like coarse ICP
pose is a valid prior for the target-agent refinement.  A different prior can
be supplied with `--initial-transform FILE.npz` or `FILE.json`; accepted keys
are `gaussian_T_cooperscene`, `map_T_source`, or `transform`, each a 4x4
matrix.

The second stage uses the selected vehicle's full synchronized scan and a dense
opacity-positive Gaussian-center proxy, initialized from the coarse result. It
uses 0.25 m and 0.10 m voxels and accepts only when all gates pass: inlier ratio
at least 0.35, median residual at most 0.20 m, p95 residual at most 0.60 m,
and correction at most 1 m / 3 degrees.

`inlier_ratio` is the fraction of **all source scan points** within the final
ICP distance threshold (0.20 m).  The gate metrics `median_residual`,
`p90_residual`, and `p95_residual` are computed over accepted
correspondences only.  `all_point_median_residual`, `all_point_p90_residual`,
and `all_point_p95_residual` explicitly summarize every source point,
including unmatched/dynamic points.  `accepted` means the stated gates all
passed.

## Current mini result

The CPU reproduction above produced:

- coarse FPFH/RANSAC fitness `0.0` (reported, not hidden);
- final inlier ratio `0.619284`; accepted-correspondence median `0.114248 m`,
  p90 `0.179147 m`, and p95 `0.189345 m`;
- all-point median `0.161955 m`, p90 `0.370838 m`, and p95 `0.477536 m`;
- final correction `0.7358 m` and `0.9928 degrees` from coarse;
- agent-1 end-to-end trajectory displacement `0.007941 m` across 30 frames,
  consistent with the stationary mini scene.

`--render-device cpu` uses the deterministic projective-splat fallback and is
the recommended reproduction choice while CUDA/gsplat is unstable.  Use
`--render-device cuda` to request CUDA; the renderer falls back to CPU if
initialization or rasterization fails.  Leave the default `auto` to select CUDA
when available.  Renders use the 1920x1200 CooperScene camera calibration at
downsample factor 2 by default.

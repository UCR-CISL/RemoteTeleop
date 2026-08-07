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

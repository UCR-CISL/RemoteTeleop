# GSV_LOCALIZATION_PLAN.md

## Goal

Localize a robot equipped with an Ouster LiDAR in a prebuilt Google-Street-View-derived Gaussian Splat (3DGS) map.

The 3DGS map is built from COLMAP using street-view cameras. The reconstruction is already approximately metric because COLMAP bundle adjustment used GPS soft constraints upstream. At the Irvine site, prior Google-Earth alignment reported:

- COLMAP scale: approximately `0.9992 m / COLMAP unit`
- fitted Sim(3) scale: `1.00085`
- mean camera-center alignment residual: `0.08 m`
- yaw offset: approximately `0 deg`

The localization system must **not depend on Google Street View ENU coordinates at runtime**. Instead, LiDAR geometry is used to bridge the Ouster/LIO frame to the Gaussian-splat frame.

The map is assumed to be mostly static and cleaned of dynamic objects. It contains buildings, road, sidewalks, curbs, poles/signs, and other persistent landmarks. Some sparse 3DGS artifacts ("floaters") may remain. Runtime LiDAR scans may contain dynamic cars, cyclists, pedestrians, and other transient objects.

---

## Core Design

Use a two-stage system:

1. **Offline / initialization alignment**
   - Build a metric local LiDAR map using Ouster LiDAR + IMU and LiDAR-inertial odometry (LIO).
   - Register the accumulated LiDAR submap against the 3DGS map.
   - Estimate a one-time transform from the LiDAR-map frame to the 3DGS/COLMAP frame.
   - Optionally estimate a one-time Sim(3) scale, but initialize scale at 1.0 and expect the final scale to remain very close to 1.
   - Freeze scale after initialization.

2. **Online localization**
   - Use LIO to predict the robot pose.
   - Crop the 3DGS localization map around the predicted pose.
   - Register the current LiDAR scan or short local LiDAR submap to the 3DGS map.
   - Use robust outlier rejection so dynamic vehicles and residual floaters do not dominate.
   - Fuse the registration correction with the LIO estimate.
   - Detect and reject bad registration updates.

Online tracking should be an **SE(3)** problem, not Sim(3).

---

# 1. Coordinate Frames

Use explicit frame names throughout the codebase.

Recommended names:

- `G`: Gaussian-splat / COLMAP map frame
- `L`: LIO odometry frame
- `B`: robot base frame
- `S`: Ouster sensor frame

Required static calibration:

```text
T_B_S
```

Required one-time global alignment:

```text
T_G_L
```

Online LIO provides:

```text
T_L_B(t)
```

Therefore the predicted robot pose in the Gaussian map is:

```text
T_G_B(t) = T_G_L * T_L_B(t)
```

If a scan-matching correction is estimated:

```text
Delta_T_G_B
```

then the corrected pose is approximately:

```text
T_G_B_corrected = Delta_T_G_B * T_G_B_predicted
```

Be consistent about left-vs-right multiplication and frame direction.

---

# 2. Map Representation

Do not assume raw Gaussian centers are an ideal geometric surface representation.

Implement at least two backends so they can be compared experimentally.

## Backend A — Gaussian centers / surfels

For each Gaussian retain:

- mean `mu`
- covariance / scale and orientation
- opacity
- optional color
- optional semantic label
- neighborhood support score

Construct a localization subset by rejecting:

- very low-opacity Gaussians
- isolated Gaussians
- extreme-size Gaussians
- Gaussians inconsistent with neighboring geometry

Prefer stable surfaces over appearance-only splats.

## Backend B — extracted geometric proxy

Create a point-cloud or surfel representation derived from the 3DGS.

Possible methods:

- depth-render the 3DGS from many map viewpoints and back-project
- convert stable Gaussians to surface samples
- optionally reconstruct a mesh and re-sample the mesh

This backend is likely the simplest robust baseline for GICP.

---

# 3. Floater Rejection

Floaters are 3DGS artifacts that do not correspond to physical surfaces.

Implement map-side filters before registration.

Suggested filters:

## 3.1 Local support

For Gaussian `i`:

```text
N_i(r) = number of neighboring Gaussian centers within radius r
```

Reject or strongly downweight isolated points.

Initial values to test:

- neighborhood radius: `0.20–0.50 m`
- minimum neighbors: `5–20`

Tune according to map density.

## 3.2 Scale / covariance filter

Reject Gaussians whose scale is far outside the local distribution.

Examples:

- extremely large Gaussians spanning open space
- tiny isolated Gaussians with no supporting neighborhood

## 3.3 Optional multi-view depth consistency

Render the Gaussian from multiple nearby views.

A physically plausible surface should produce a spatially consistent depth layer from multiple viewpoints.

This can be added later if simple support filtering is insufficient.

---

# 4. Initial LiDAR-to-3DGS Alignment

The initial alignment should use an **accumulated LIO submap**, not one raw Ouster scan.

Recommended sequence length:

- minimum: `20–30 m` of vehicle motion
- preferred: `50–100 m`
- include turns if possible
- include multiple vertical structures such as buildings, curbs, poles, and signs

Build:

```text
P_L = accumulated metric LiDAR submap in the LIO frame
```

Register against the cleaned 3DGS geometric map.

Initially solve:

```text
p_G = s * R * p_L + t
```

Use a robust Sim(3) or staged alignment:

1. rough manual / GNSS / map-based initialization if available
2. coarse feature or voxel registration
3. robust GICP / point-to-plane refinement
4. optional scale refinement

Expected scale:

```text
s approximately 1.0
```

After this step:

```text
freeze s
```

and store the resulting `T_G_L`.

---

# 5. Recommended Online Registration

Preferred initial baseline:

```text
LIO pose prediction
      |
      v
local map crop
      |
      v
current LiDAR scan / short submap
      |
      v
robust GICP / point-to-plane registration
      |
      v
pose correction
```

Use a short local LiDAR submap if one raw scan is too sparse.

Example time window:

- `0.3–1.0 s`, depending on vehicle speed and Ouster scan rate

Deskew every scan using IMU/LIO motion estimates.

---

# 6. Dynamic Object Robustness

The Gaussian map is static, but the runtime LiDAR cloud may contain cars, pedestrians, and cyclists.

Do not require the full scan to match.

Use at least:

1. maximum correspondence-distance gating
2. robust loss
3. trimmed correspondence selection
4. minimum inlier-ratio checks

Recommended robust losses:

- Huber
- Cauchy
- Tukey

Recommended trimmed-registration experiments:

- retain best `60%`
- retain best `70%`
- retain best `80%`
- retain best `90%`

Select the setting that best tolerates traffic without discarding too much static geometry.

Semantic dynamic-object removal is optional and should initially be treated as an enhancement rather than a hard dependency.

---

# 7. Geometry Weighting

Different map structures contribute differently to pose observability.

## High-value geometry

Prefer:

- building facades
- building corners
- poles
- signs
- barriers
- curbs
- sidewalk edges
- other permanent vertical structures

## Lower-value geometry

Road planes are useful for:

- `z`
- roll
- pitch

but are weak for:

- translation along the road
- lateral translation on a large flat surface
- yaw

Avoid allowing road points to dominate registration.

Possible weighting scheme:

```text
vertical / edge / corner geometry: weight 1.0
curbs and sidewalk edges:          weight 1.0
general building surface:          weight 0.8–1.0
road plane:                        weight 0.2–0.5
suspected vegetation:              weight 0.3–0.7
```

These values are initial tuning suggestions, not fixed requirements.

---

# 8. Concrete Accuracy Metrics

The implementation should report metrics at several levels.

## 8.1 Map metric-scale error

Compare distances between corresponding static landmarks in:

- LiDAR submap
- 3DGS map

For landmark pairs:

```text
scale_error = |d_GS / d_LiDAR - 1|
```

Targets:

### Preferred
```text
median relative scale error < 0.2%
```

### Acceptable
```text
median relative scale error < 0.5%
```

### Fail / investigate
```text
median relative scale error > 1.0%
```

For a 100 m map segment:

- `0.2%` = `0.20 m`
- `0.5%` = `0.50 m`
- `1.0%` = `1.00 m`

Given the Irvine alignment result, expect the global scale estimate to be much better than the failure threshold.

---

## 8.2 Static-surface map accuracy

After a good manual or optimized alignment, evaluate LiDAR points belonging to static structures against the 3DGS-derived geometry.

Report point-to-map residuals separately for:

- buildings
- curbs
- poles/signs
- sidewalks
- road

Metrics:

```text
median point-to-map distance
P90 point-to-map distance
P95 point-to-map distance
```

Initial targets:

### Strong map
```text
median < 0.05 m
P90    < 0.10 m
P95    < 0.15 m
```

### Usable map
```text
median < 0.10 m
P90    < 0.20 m
P95    < 0.30 m
```

If static building residuals are consistently above `0.20–0.30 m`, investigate the map geometry before spending significant time tuning the localization optimizer.

---

# 9. Localization Accuracy Targets

Ground-truth evaluation is strongly recommended.

Possible ground truth:

- RTK-GNSS / GNSS-INS
- survey-grade total station
- high-quality offline LiDAR SLAM trajectory
- precisely surveyed landmarks

Report Absolute Trajectory Error (ATE) and Relative Pose Error (RPE).

## 9.1 Translational ATE

Primary target:

```text
RMSE ATE translation <= 0.10 m
```

Stretch target:

```text
RMSE ATE translation <= 0.05 m
```

Minimum acceptable for early prototype:

```text
RMSE ATE translation <= 0.20 m
```

Also report:

```text
median ATE
P95 ATE
maximum ATE
```

Preferred:

```text
median ATE <= 0.08 m
P95 ATE    <= 0.20 m
```

---

## 9.2 Yaw accuracy

Road localization is particularly sensitive to yaw.

Primary target:

```text
yaw RMSE <= 0.5 deg
```

Stretch target:

```text
yaw RMSE <= 0.25 deg
```

Early-prototype acceptable:

```text
yaw RMSE <= 1.0 deg
```

Large yaw errors can create substantial lateral position error over distance.

---

## 9.3 Roll / pitch accuracy

Because the road plane constrains orientation well:

```text
roll RMSE  <= 0.25 deg
pitch RMSE <= 0.25 deg
```

Prototype acceptable:

```text
<= 0.5 deg
```

---

## 9.4 Relative drift

Measure RPE over fixed travel distances.

Recommended intervals:

- `10 m`
- `50 m`
- `100 m`

Targets after GS correction:

```text
10 m translational RPE  <= 0.05 m
50 m translational RPE  <= 0.10 m
100 m translational RPE <= 0.20 m
```

Also report yaw RPE.

---

# 10. Registration Quality Metrics

Every registration update should expose diagnostics.

## 10.1 Inlier ratio

```text
inlier_ratio = valid_correspondences / candidate_correspondences
```

Initial thresholds:

```text
good:      >= 0.60
marginal:  0.35–0.60
reject:    < 0.35
```

Tune empirically.

---

## 10.2 Median registration residual

For accepted correspondences:

```text
good:      median residual < 0.08 m
marginal:  0.08–0.15 m
reject:    > 0.15–0.20 m
```

Thresholds should eventually be category-aware because road points and facade points may have different error distributions.

---

## 10.3 P90 / P95 residual

Track tails, not only means.

Preferred:

```text
P90 < 0.15 m
P95 < 0.25 m
```

Large tails can indicate:

- dynamic cars
- floaters
- bad initialization
- local map geometry errors
- incorrect correspondences

---

## 10.4 Pose correction magnitude

Since LIO gives the initial guess, scan-to-map registration should usually produce a modest correction.

Example sanity gates per update:

```text
translation correction <= 0.5 m
rotation correction    <= 3 deg
```

Values above these limits should be considered suspicious unless the system is explicitly performing relocalization.

For high-rate normal tracking, tighter typical corrections are expected.

---

# 11. Failure Detection

A GS registration update should be rejected if several indicators fail.

Example rule:

Reject update if any two of the following occur:

```text
inlier ratio < 0.35
median residual > 0.15 m
P95 residual > 0.30 m
translation correction > 0.5 m
rotation correction > 3 deg
registration Hessian is poorly conditioned
insufficient non-road geometry
```

If rejected:

```text
fall back to LIO-only prediction
```

Do not allow one bad GS match to corrupt the state estimate.

---

# 12. Degeneracy / Observability Checks

A road can contain long planar or repetitive geometry.

Inspect the registration Hessian / information matrix.

Detect weakly constrained directions.

Typical degeneracy examples:

- long flat road
- long featureless wall
- tunnel-like geometry
- repeated building facades
- temporary occlusion by traffic

Compute eigenvalues of the approximate pose Hessian.

Track:

```text
lambda_min
lambda_max
condition_number = lambda_max / lambda_min
```

Do not trust a pose update when one or more motion directions are weakly observable.

Tune the exact condition-number threshold experimentally.

---

# 13. Traffic Robustness Benchmarks

Create explicit traffic-density tests.

Suggested test bins:

```text
0–10% dynamic LiDAR points
10–25%
25–40%
40–60%
>60%
```

Estimate the dynamic fraction using manual labels or a semantic detector for evaluation only.

Success criterion:

```text
localization ATE degradation < 2x
up to at least 40% dynamic points
```

Preferred:

```text
P95 ATE remains < 0.30 m
with 40% dynamic returns
```

Also test complete occlusion of one side of the street.

---

# 14. Floater Robustness Benchmark

Artificially inject map outliers if natural floaters are insufficient for controlled evaluation.

Test:

```text
0%
1%
2%
5%
10%
```

of local map samples as isolated false geometry.

Success target:

```text
<= 5% floaters should cause < 20% increase in translation ATE
```

The exact percentages are synthetic stress tests, not assumptions about the real map.

---

# 15. Initialization / Relocalization Metrics

Separate global initialization from normal tracking.

For initial alignment, test perturbations around ground truth.

Suggested perturbation grid:

```text
translation:
0.5 m
1 m
2 m
5 m
10 m

yaw:
2 deg
5 deg
10 deg
20 deg
45 deg
```

Measure capture rate.

Desired tracking-stage capture basin:

```text
>= 95% success for:
translation <= 1 m
yaw <= 5 deg
```

Desired coarse relocalization target:

```text
>= 90% success for:
translation <= 5 m
yaw <= 20 deg
```

If global relocalization is needed from arbitrary poses, implement a separate coarse localization module rather than forcing local GICP to solve it.

---

# 16. Latency / Runtime Targets

For online navigation, localization should operate faster than the vehicle state changes meaningfully.

Recommended target:

```text
GS registration update rate >= 10 Hz
```

Preferred:

```text
>= 20 Hz
```

If full scan-to-map registration cannot meet this:

- keep LIO at high rate
- run GS corrections at lower rate
- fuse asynchronously

Suggested latency target:

```text
median GS correction latency < 50 ms
P95 latency < 100 ms
```

Hardware-dependent, so measure before optimizing.

---

# 17. Evaluation Dataset

Record synchronized:

- raw Ouster packets / point clouds
- IMU
- timestamps
- LIO output
- localization corrections
- final fused pose
- ground truth if available

Include varied scenes:

1. empty road
2. light traffic
3. heavy traffic
4. parked vehicles
5. moving vehicles close to sensor
6. intersections
7. long straight road
8. building-rich area
9. sparse-landmark area
10. partial map artifacts / floaters

---

# 18. Recommended Development Milestones

## Milestone 0 — Verify metric map

Deliverables:

- sampled GS geometry
- known landmark-distance comparison
- estimated GS/LiDAR scale

Pass criteria:

```text
scale error < 0.5%
```

---

## Milestone 1 — Static offline registration

Input:

- accumulated LIO submap
- cleaned GS map

Deliverable:

```text
T_G_L
```

Pass criteria:

```text
static-map median residual < 0.10 m
P90 residual < 0.20 m
```

Stretch:

```text
median < 0.05 m
P90 < 0.10 m
```

---

## Milestone 2 — Online scan-to-map tracking

Start with traffic-free data.

Pass criteria:

```text
ATE RMSE < 0.20 m
yaw RMSE < 1.0 deg
registration failure rate < 1%
```

---

## Milestone 3 — Robust traffic handling

Enable:

- robust loss
- trimmed correspondences
- residual gates

Pass criteria:

```text
ATE RMSE < 0.20 m under moderate traffic
P95 ATE < 0.30 m
no catastrophic pose jumps
```

---

## Milestone 4 — Floater robustness

Enable map filtering.

Pass criteria:

```text
isolated map artifacts do not produce accepted false corrections
```

and:

```text
<= 5% synthetic floater contamination
causes < 20% ATE degradation
```

---

## Milestone 5 — Production-quality fusion

Fuse:

```text
LIO + GS corrections
```

with an EKF or factor graph.

Primary target:

```text
ATE RMSE <= 0.10 m
yaw RMSE <= 0.5 deg
P95 ATE <= 0.20 m
```

Stretch:

```text
ATE RMSE <= 0.05 m
yaw RMSE <= 0.25 deg
```

---

# 19. Suggested Ablations

Run these experiments before committing to a sophisticated custom Gaussian registration implementation.

## Map representation

Compare:

1. raw Gaussian centers
2. filtered Gaussian centers
3. GS-derived point cloud
4. GS-derived surfels
5. direct covariance-aware Gaussian registration

## Registration

Compare:

1. ICP point-to-point
2. ICP point-to-plane
3. GICP
4. trimmed GICP
5. covariance-aware GS-GICP

## Geometry selection

Compare:

1. all static map geometry
2. no road
3. downweighted road
4. buildings + curbs + poles only

## Dynamic handling

Compare:

1. no special handling
2. robust loss only
3. trimmed registration
4. semantic dynamic-object removal
5. semantic removal + robust registration

---

# 20. Recommended MVP

Do not begin with a custom differentiable Gaussian-registration method.

Start with:

```text
cleaned GS
   |
   v
extract static point/surfel map
   |
   v
voxel downsample
   |
   v
KD-tree / voxel index
```

Online:

```text
Ouster + IMU
   |
   v
LIO
   |
   v
deskewed scan
   |
   v
local GS-map crop
   |
   v
trimmed GICP
   |
   v
quality gates
   |
   v
LIO + GS fusion
```

This establishes whether the underlying GS geometry is good enough.

Only after the baseline works should direct Gaussian covariance information be incorporated.

---

# 21. Key Research Question

The most important empirical question is:

> How metrically accurate are the physical surfaces represented by the COLMAP-derived 3DGS?

The prior `0.08 m` camera-center residual is encouraging but does **not** directly measure Gaussian surface accuracy.

Before aggressively optimizing the localization algorithm, measure static LiDAR-to-map distances for buildings, curbs, poles, and road surfaces.

If the stable structures themselves are displaced by `0.20–0.30 m`, that error will impose a localization floor.

---

# 22. Suggested Logging Schema

For every registration update log:

```text
timestamp
predicted_pose
corrected_pose
delta_translation_m
delta_rotation_deg
num_scan_points
num_candidate_correspondences
num_inliers
inlier_ratio
mean_residual_m
median_residual_m
p90_residual_m
p95_residual_m
robust_cost
hessian_eigenvalues
hessian_condition_number
map_crop_size
registration_runtime_ms
accepted_or_rejected
rejection_reason
```

These logs are essential for debugging traffic, degeneracy, map artifacts, and bad pose corrections.

---

# 23. Definition of Success

A strong final system should satisfy approximately:

```text
Metric map scale error:
    < 0.2% preferred

Static surface residual:
    median < 0.05–0.10 m
    P90    < 0.10–0.20 m

Online localization:
    translation ATE RMSE <= 0.10 m
    yaw RMSE <= 0.5 deg
    P95 translation ATE <= 0.20 m

Robustness:
    stable with >= 40% dynamic LiDAR returns
    <= 5% map floater contamination causes < 20% ATE degradation

Runtime:
    >= 10 Hz GS correction
    >= 20 Hz preferred

Safety:
    bad registration updates are detected and rejected
    localization can fall back to LIO without catastrophic jumps
```

These should be treated as **engineering targets to validate**, not guarantees implied by the existing 3DGS.

The first go/no-go decision is the measured static LiDAR-to-GS surface residual.

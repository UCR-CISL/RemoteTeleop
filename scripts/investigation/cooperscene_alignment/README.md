# CooperScene recipe application and localization evaluation

The retained calibration lives in
`config/cooperscene_geodetic_alignment_recipe.json`. The reusable application
code is `src/localization/cooperscene_geodetic.py`:

```python
from src.localization.cooperscene_geodetic import FrozenProfile, GeodeticPoses

trajectory = GeodeticPoses(take, agent, georeference)
profile = FrozenProfile()  # Defaults to the repository's retained recipe.
map_T_lidar = profile.map_T_lidar(
    trajectory.lidar_pose(lidar_timestamp_ns, antenna_up_m=0.03),
    take=take, agent=agent,
)
```

Raw GNSS positions are converted directly into the map's georeferenced ENU;
orientation is interpolated at LiDAR capture time. Agent 1 uses the recorded
Rx(180 degree) device normalization. Sensor calibration and the rotated antenna
lever arm are applied before the shared map transform G. Vehicle 1 additionally
receives its retained mounting delta, and vehicle 1/take 3 receives its saved
map-frame position bias. Vehicles 2/3 receive G only. Timestamp extrapolation is
rejected. The 3 cm antenna offset remains an unverified assumption.

This is a conditional, map-assisted reference. The recipe is also initialization
and the ENU fusion prior in the evaluation below, making initial reference error
zero. Agreement with this reference does not establish independent GT accuracy.
The recipe is not automatically substituted into existing replay localization.

## Frame search

```sh
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python \
  scripts/investigation/cooperscene_alignment/frame_search.py \
  --output artifacts/recipe-frame-search-NEW
```

The default `--prior` is the recipe JSON in `config/`. An explicit JSON path or
historical profile directory is accepted. `--takes`, `--agents`, and `--indices`
allow a small pilot. The default full search covers all 14 takes and vehicles
1–3, every native frame, with no percentage cutoff or warm start. Mounted
archives, GNSS logs, and calibration files are opened read-only. The data/map
paths retain the local investigation defaults.

The map target retains all finite Gaussian centers with opacity >0. Source
clouds retain full-resolution finite points outside annotated vehicle boxes,
with 0.2 m padding and production box-center composition. Each independent frame
reuses the existing ICP and ENU fusion implementations. Cached map voxels,
normals, and nearest-neighbor trees reproduce the original numerical results.

A frame must pass upstream scan registration, fusion convergence, and final
registration gates. The original minimum 35% inliers, residual limits, 1 m
transform correction and 3 degree rotation correction gates remain intact.
Strict reference-error qualification is XY <0.1 m for BEV and XYZ <0.1 m for XYZ.
Each method records the best accepted frame, the earliest qualifying frame,
frame 0, and BEV's best full-XYZ candidate. No qualifying frame after complete
exhaustion retains the labelled `frame0_recipe_prior_not_qualified` fallback.
A conservative displacement certificate skips optimization only when passing
the gates is mathematically impossible; no optimized error is invented.

Signed XYZ and orientation disagreement, raw scan errors, fusion diagnostics,
inlier support, residuals, capture elapsed time, and percentage are saved in
per-frame JSON. Resuming requires identical code, recipe/map hashes and source
provenance. Migration changes those hashes, so use a fresh output directory
rather than resuming older worktree outputs.

## Concurrent search and report

```sh
.venv/bin/python scripts/investigation/cooperscene_alignment/search_queue.py \
  --output artifacts/recipe-frame-search-full-NEW --workers 8
.venv/bin/python scripts/investigation/cooperscene_alignment/search_report.py \
  artifacts/recipe-frame-search-full-NEW
```

The queue runs independent takes, checkpoints results and refreshes combined
`status.json`, `summary.json`, and `report.md` every 30 seconds. Failures are
explicit. These commands evaluate offline point clouds; they do not launch
teleoperation replay.

## Import validation

The main-repository implementation was checked on exactly the earlier frames:
take 1/V1/frame 250, take 3/V1/frame 0, and take 4/V2/frame 150. Initial,
scan-only, BEV and XYZ results, rejection certificates, and acceptance were
compared against saved full-search records with numeric tolerance 1e-10,
excluding runtime. Evidence: `artifacts/alignment-import-smoke-20261011/`.

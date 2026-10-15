# Vehicle shape/color proxy evaluation (offline, single frame)

`evaluate.py` scores the shape-fitted, color-tinted vehicle proxies on fixed CooperScene frames
(`eval_frames.json`: 48 frames, 14 takes, agents 1-3). It does not use MCAP or split-system replay.
Each frame is rendered through `PoseOnlyCompositedCameraBackend` at the frame's GT-registered splat
pose. The pose comes from the accepted per-frame `xyz` alignment in
`artifacts/worktree-archive-20261011/alignment_artifacts/recipe-frame-search-full-20261004`.

For every GT vehicle in view (3-60 m), the harness runs these steps on the previous K native frames
(K defaults to 10, about 1 s; the window is filled from following frames at a sequence start):

1. **Crop and snap.** In-box LiDAR returns are taken in the object frame, using native `map_T_lidar` and
   `map_T_object` with the GT box bottom snapped onto the LiDAR ground by `LidarGroundSnapper`.
   Returns inside a neighbor's box inflated by 0.2 m are dropped.
2. **Accumulate and refit.** Returns go into `ObjectPointAccumulator`. `VehicleShapeFitter` refits after
   every scan, warm-started from the last accepted fit; like the live estimator it refits only the
   incumbent body type except every `--sweep-every`-th fit (default 5), which reconsiders all types.
   The last accepted fit is published; before it, the dimension-prior type mean ("prior only").
   A cold one-shot fit at the end of the window is also recorded.
3. **Color.** `VehicleColorEstimator.observe` runs on each frame's native 1920x1200 image with the
   refined calibration, a full-scan `SparseDepthBuffer`, and nearer GT boxes as occluders.

Frame t is held out and scored. Its in-box points give true point-to-triangle distances to the legacy
primitive, the published type's mean and the published fit. The frame is rendered three times with the
same `body_color_rgb`/`color_confidence` tint: legacy primitive, type mean, fitted `vehicle_shape`.

```bash
# from the worktree root (about 10 min on an RTX 4090; evidence building dominates)
PYTHONPATH=. ../RemoteTeleop/.venv/bin/python scripts/investigation/vehicle_shape/evaluate.py
# metric-only variants (no GPU), e.g. a library-config comparison:
PYTHONPATH=. ../RemoteTeleop/.venv/bin/python scripts/investigation/vehicle_shape/evaluate.py --no-render \
    --output /tmp/variant --default-incumbent --color '{"chromatic_min_fraction": 0.35}'
```

Outputs (default `../RemoteTeleop/artifacts/pcd-vehicle-assets-20261015/`):

- `REVIEW.md` (open it in the VS Code Markdown preview). It holds the aggregate table per range
  bucket, the worst cases, and per take -> agent -> frame sections.
- `frames/<key>/compare.png`: real camera | legacy primitive | type mean | fitted shape (960x600 each, same tint).
- `frames/<key>/vehicles.png`: one row per vehicle. It shows a real crop with belt-band LiDAR
  samples, render crops, a color swatch, BEV and side views of the accumulated points against the
  fitted (green) and default (dashed) outlines, and the metrics.
- `summary.json` (settings, aggregates, frames, vehicles, color observation logs) and `summary.csv`
  (one row per vehicle).

`--fitter`, `--accumulator` and `--color` take JSON keyword overrides for `VehicleShapeFitter`,
`ObjectPointAccumulator` and `VehicleColorConfig` (e.g. `--fitter '{"alternatives": 6}'` refines every type in a sweep,
`'{"inflation_lwh_m": [0, 0, 0]}'` treats boxes as tight). `--default-incumbent` is an experimental flag: the
first fit treats the sedan type mean as the incumbent.

`--baseline <earlier output>` adds a "Changes since" section to REVIEW.md: a before/after aggregate table
(including body-type accuracy against `body_labels.json`: three labelers over every v2 vehicle row; high/medium
confidence rows score exact type and car/SUV/van/pickup group) and, for the vehicles listed
in `--changes` (see `v2_changes.json`), real | before | after crops with color swatches. The v2 review was made
with:

```bash
PYTHONPATH=. ../RemoteTeleop/.venv/bin/python scripts/investigation/vehicle_shape/evaluate.py \
    --output ../RemoteTeleop/artifacts/pcd-vehicle-assets-20261015-v2 \
    --baseline ../RemoteTeleop/artifacts/pcd-vehicle-assets-20261015 \
    --changes scripts/investigation/vehicle_shape/v2_changes.json
```

To compare variants, copy a variant's `summary.json` (and optionally `frames/*/compare.png`) to
`<output>/variants/<name>/`, then rewrite REVIEW.md:

```bash
PYTHONPATH=. ../RemoteTeleop/.venv/bin/python scripts/investigation/vehicle_shape/evaluate.py --rebuild-review
```

The universal mesh review (v3) runs on the 12 dev frames:

```bash
PYTHONPATH=. ../RemoteTeleop/.venv/bin/python scripts/investigation/vehicle_shape/evaluate.py --only \
    take05_a1_524085 take08_a2_537370 take01_a1_481325 take07_a2_534390 take09_a2_540610 take06_a2_527890 \
    take13_a3_552390 take06_a1_527905 take03_a2_497165 take12_a3_549200 take10_a3_543585 take03_a3_497215 \
    --output ../RemoteTeleop/artifacts/pcd-vehicle-assets-20261015-v3 \
    --baseline ../RemoteTeleop/artifacts/pcd-vehicle-assets-20261015-v2 \
    --changes scripts/investigation/vehicle_shape/v3_changes.json
# type-mean gallery (archetype sizes, or normalized into one detector box with --box "[5.1,2.22,1.9]")
PYTHONPATH=. ../RemoteTeleop/.venv/bin/python scripts/investigation/vehicle_shape/preset_gallery.py \
    --output ../RemoteTeleop/artifacts/pcd-vehicle-assets-20261015-v3/presets.png
```

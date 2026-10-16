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
2. **Accumulate, refit and publish.** Returns go into `ObjectPointAccumulator`. `VehicleShapeFitter` refits
   after every scan, warm-started from the last accepted fit; like the live estimator it refits only the
   incumbent body type except every `--sweep-every`-th fit (default 5), which reconsiders all types, and a
   per-track `TypePosterior` decides the body type. The publication policy is the live one
   (`VehicleAppearanceConfig`):
   - **Range gate** (`--max-range`, default 25 m LiDAR range; `--range-hysteresis` 5 m): a vehicle beyond it
     at frame t gets neither shape nor color and renders the default asset; once in range it keeps its hints
     out to 30 m. Out-of-range scans are accumulated but not fit (`--fit-out-of-range` fits them too) and not
     colored.
   - **Evidence** (`--min-publish-evidence`, default 100 fit evidence voxels): weaker fits publish nothing
     (legacy primitive); nothing is published before the first fit. `0` restores the v4 policy (last accepted
     fit, dimension-prior type mean before it).
   - **Rejection** (`--unpublish-after-rejections`, default 2): that many consecutive rejected fits
     (rms > 0.25 m) unpublish the shape; a single rejected fit keeps the last one.
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

`--fitter`, `--accumulator`, `--color` and `--posterior` take JSON keyword overrides for `VehicleShapeFitter`,
`ObjectPointAccumulator`, `VehicleColorConfig` and `TypePosterior` (e.g. `--fitter '{"alternatives": 6}'` refines
every type in a sweep, `'{"inflation_lwh_m": [0, 0, 0]}'` treats boxes as tight,
`--posterior '{"switch_margin_nats": 2}'`). `--default-incumbent` is an experimental flag: the
first fit treats the sedan type mean as the incumbent.

- `--no-color` skips color estimation (shape-only passes; color does not affect shape metrics).
- `--live-schedule` replays the past frames through `VehicleAppearanceEstimator` itself instead of refitting
  every scan: refit cadence, per-update fit budget and priority, sweep cadence, freezing, range gate and
  publication (synchronous fits, no future frames at a sequence start, no color).
  `publication_delay_scans` (scans from a track's first scan to its first published shape) and the shapes
  published by frame t measure the live lag. `--appearance` takes `VehicleAppearanceConfig` overrides (JSON),
  e.g. `--appearance '{"max_fits_per_update": 4}'`.
- `--occluder-mask <npz>` passes a `scripts/carve_occluder_mask.py` sidecar to the renders' proxy occluder
  depth (`OpacityThresholdDepth(occluder_mask=...)`); default: no mask.
- `--beats-legacy 0.9` (offline variant) publishes a fit only when its robust cost on the accumulated voxels
  (Huber point-to-triangle) is at most 0.9 x the legacy primitive's, and records `legacy_cost_ratio`. On dev+val it
  mostly withheld fits that beat legacy on the held-out frame, so the live estimator has no such gate.

Metric-only dev/val passes (the frame lists are kept outside the repo):

```bash
PYTHONPATH=. ../RemoteTeleop/.venv/bin/python scripts/investigation/vehicle_shape/evaluate.py --no-render \
    --no-color --only <frame keys> --output /tmp/w2/dev            # every-scan refits
PYTHONPATH=. ../RemoteTeleop/.venv/bin/python scripts/investigation/vehicle_shape/evaluate.py --no-render \
    --no-color --only <frame keys> --output /tmp/w2/dev-live --live-schedule --max-range 30
```

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

The v4 review (end-face registration, calibrated type posterior, visible-end features, road-layer removal;
the type-mean column is the box-dimension-only type) on the same 12 dev frames:

```bash
PYTHONPATH=. ../RemoteTeleop/.venv/bin/python scripts/investigation/vehicle_shape/evaluate.py --only \
    take05_a1_524085 take08_a2_537370 take01_a1_481325 take07_a2_534390 take09_a2_540610 take06_a2_527890 \
    take13_a3_552390 take06_a1_527905 take03_a2_497165 take12_a3_549200 take10_a3_543585 take03_a3_497215 \
    --output ../RemoteTeleop/artifacts/pcd-vehicle-assets-20261015-v4 \
    --baseline ../RemoteTeleop/artifacts/pcd-vehicle-assets-20261015-v3 \
    --changes scripts/investigation/vehicle_shape/v4_changes.json
```

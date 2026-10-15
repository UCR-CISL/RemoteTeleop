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
2. **Accumulate and refit.** Returns go into `ObjectPointAccumulator`. `CarShapeFitter` refits after
   every scan, warm-started from the last accepted fit (a live per-poll refit), and the last
   accepted fit is published. A cold one-shot fit at the end of the window is also recorded.
3. **Color.** `VehicleColorEstimator.observe` runs on each frame's native 1920x1200 image with the
   refined calibration, a full-scan `SparseDepthBuffer`, and nearer GT boxes as occluders.

Frame t is held out and scored. Its in-box points give one-sided point-to-surface distances for the
default sedan and for the published fit. The frame is rendered twice: once with the default primitive,
and once with `vehicle_shape` plus `body_color_rgb`/`color_confidence` hints.

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
- `frames/<key>/compare.png`: real camera | default primitive | fitted+tinted (960x600 each).
- `frames/<key>/vehicles.png`: one row per vehicle. It shows a real crop with belt-band LiDAR
  samples, render crops, a color swatch, BEV and side views of the accumulated points against the
  fitted (green) and default (dashed) outlines, and the metrics.
- `summary.json` (settings, aggregates, frames, vehicles, color observation logs) and `summary.csv`
  (one row per vehicle).

`--fitter`, `--accumulator` and `--color` take JSON keyword overrides for `CarShapeFitter`,
`ObjectPointAccumulator` and `VehicleColorConfig`. Three experimental flags are implemented as
in-script subclasses or fitter inputs, so the library stays unchanged:

- `--default-incumbent`: the first fit treats the default sedan (+x forward) as the incumbent.
- `--fixed-forward-sign`: fits only `forward_sign=+1`.
- `--bright-majority-color`: a colored minority cluster wins only over a dark achromatic majority.

To compare variants, copy a variant's `summary.json` (and optionally `frames/*/compare.png`) to
`<output>/variants/<name>/`, then rewrite REVIEW.md:

```bash
PYTHONPATH=. ../RemoteTeleop/.venv/bin/python scripts/investigation/vehicle_shape/evaluate.py --rebuild-review
```

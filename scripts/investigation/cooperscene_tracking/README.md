# Offline CooperScene detection and track evaluation

Evaluate a fixed, timestamped frame JSONL independently of inference or tracking:

```bash
python -m scripts.investigation.cooperscene_tracking.evaluate frames.jsonl \
  --output-dir artifacts/tracking-evaluation-new --center-threshold-m 2
```

Each row has `sequence_id` (take and agent), `frame_id`, integer `timestamp_us`,
and `detections`, `tracks`, `gt` arrays. Every box has `track_id`, a rigid 4×4
`world_T_object`, positive `dimensions_lwh`, and `label` (`car` by default).
Optional `score` is retained by the producer; filtering must happen before
evaluation and be declared in the producer's manifest. Detector identities may
be frame-specific; track identities must persist. Keep empty frames in the file.

The producer must declare coverage, score threshold, localization reference,
start frame, timestamp policy, and GT visibility/range policy. The evaluator
compares exactly those supplied boxes and cannot certify the producer's frame
transforms. GT is solely an evaluation input. Rendering ground placement must
not alter scored poses. Retained recipe alignment applies to raw geodetic
GNSS/INS and sensor composition, never directly to released ICP-refined poses.

`pilot.py prepare --payload-compression-level 1` selects faster lossless NPZ
storage and records the level in the manifest. Decoded BGR/XYZI arrays, poses
and timestamps are unchanged. Omitting the option retains NumPy's existing
compression behavior. Two real payload controls verified identical decoded
bytes and byte-exact default files; this is a storage optimization, not an
inference or tracking change.

Frame association is class-aware Hungarian assignment, maximizing valid pair
count before minimizing XYZ center distance, with an inclusive 2 m default
gate. Precision/recall include duplicate predictions as false positives. Center
RMSE, LWH absolute errors, and length-axis angular errors modulo 180° are
reported over matched boxes only; these are object geometry errors, not ego
localization accuracy. This center protocol does not measure box overlap.

`geometric_idf1` uses a separate sequence-wide one-to-one assignment of GT and
predicted identities that maximizes the number of geometrically eligible
co-occurrences. IDTP is that maximum, IDFN and IDFP are the remaining GT and
prediction occurrences, and IDF1 is `2*IDTP/(2*IDTP+IDFN+IDFP)`. In ambiguous
crowds this global assignment can differ from local frame matches. It is
explicitly a geometric alternative, not an official IoU MOTChallenge or HOTA
score. ID switches compare each GT identity's current match with its last
matched prediction, including gaps. Fragmentation counts recovery after a
previously matched GT has an unmatched annotated frame. GT absence alone is
not fragmentation. All identities are namespaced by `sequence_id`.

Native YAML IDs are taken from the `vehicles` dictionary. The loader uses
`location + rotation @ center` and twice `extent`; its Euler conversion matches
CooperScene/CARLA's degree roll/yaw/pitch convention. Shared IDs across agents
describe the same global object (initial take 1 ID 103 has identical positions
in agents 1 and 2). IDs recur across takes, so never combine take identity
namespaces. Annotation absence is not proof of physical object death. Full
continuity and possible ID reuse within a take require a dataset audit before
claiming physical identity accuracy.

A read-only audit of the first 100 frames of takes 1, 3, and 7 across agents
1–3 found 452, 1,233, and 620 shared-agent identity occurrences, respectively,
all with identical global locations. However, take 1/agent 1 ID 55 jumps
49.075 m between frames 481290 and 481291, changes heading from −4.42° to
87.94°, and changes length from 4.317 m to 5.335 m. Treat annotation identity
metrics as provisional and flag this discontinuity; never silently reinterpret
GT as physically continuous. `audit_gt.py` reports adjacent-frame jumps above
a declared threshold (10 m by default), without changing identities. Other
inspected sequences had maximum adjacent displacements of 1.50–3.19 m and
annotation dimensions can vary by almost a metre between frames.

Cached detection comparisons can run `retrack.py frames.jsonl --output-dir NEW`
and evaluate the resulting `frames.jsonl`. GT passes through unchanged; the
tracker consumes detector boxes only. The default 0.5 s coast limit expires
identities before association, including when detector frames are sparse.

`--split-discontinuous-gt` adds separate sensitivity metrics using GT identity
segments at consecutive-frame center jumps exceeding `max(10 m, 50 m/s * dt +
2 m)`. Canonical metrics and IDs remain unchanged. `gt_continuity.json` records
every flag and the full policy.

`--front-camera-only` adds front-camera metrics alongside all supplied LiDAR
range boxes. It requires each row's `world_T_camera`, `camera_intrinsic`,
`image_width`, and `image_height`. Both GT and predictions must have projected
cuboid image intersection area at least 16 pixels². The shared projection
helper rejects the entire box if any corner has camera depth ≤0.1 m; it does
not clip boxes at the near plane. This means geometric image visibility, not
actual visibility through occlusion. Per GT ID the report includes visible,
detected, traced frames, recall, ever-detected status, longest track miss run,
and ID switches. Acceptance requires at least 90% detection and track recall
for every ID with at least five visible frames, zero never-detected IDs among
all visible IDs, and at most one switch per 100 visible GT observations.
Zero switches is separately reported. An empty eligible set does not pass.
An optional `--precision-target 0.9` adds a declared minimum for both detection
and track precision, so broad false-positive output cannot satisfy this target
merely by covering GT IDs. Coverage, identity and precision checks remain
separately reported; omitted precision targets are explicitly null.

The active sidecar uses a nuScenes checkpoint with one front image and one
current LiDAR frame, while upstream evaluation loads multiple camera views and
nine sweeps. It keeps only class index 0 (`car`) above the sidecar confidence
threshold; other nuScenes classes are discarded before evaluation. MMDet
`gravity_center` supplies the box center. Axis dimensions are reordered to
semantic length/width, adding 90° yaw when Y is longer than X; the renderer
therefore receives length-aligned X axes.

Read-only inspection of Lambda's `remote-teleop-bevfusion:latest` confirmed
upstream image loading uses RGB and its preprocessor has `bgr_to_rgb=False`.
The sidecar's OpenCV BGR-to-RGB conversion is correct. Its upstream LiDAR
loader has `norm_intensity=False`. Three sampled native frames have intensity
medians 20–22, 99th percentiles 208–497, and maxima 2,830–2,878; one sampled
nuScenes sweep has median 15, 99th percentile 100, maximum 255. This is a
measured input distribution difference, not evidence for a particular
normalization. Calibration, single-frame/single-view input differences, and
confidence settings require controlled comparisons on identical cached inputs.

`refilter.py frames.jsonl --raw-directory RAW --classes car truck bus trailer
construction_vehicle --score-threshold 0.2 --output-dir NEW` reuses cached model
outputs for a declared motor-vehicle ontology trial. Native YAML `vehicles`
does not supply detailed categories; the trial groups GT's generic category
as `vehicle`, preserves `annotation_class_name`, and retains detector class
names and IDs separately. Only the listed motor-vehicle classes enter the
tracker with grouped label `vehicle`; pedestrian, bicycle, cone, and barrier
outputs are excluded. Original GT geometry and timestamps are unchanged.
Exact request/frame/timestamp/ordinal provenance, calibration, and image sizes
must match all cached frames before an output directory is created. Use the
same command with `--classes car` as the controlled car-only tracker baseline.
Optional `--minimum-confirmation-hits 2` requires consecutive distinct captures
before emitting a new track. Optional `--continuation-score-threshold .1` keeps
primary detections and births at `--score-threshold .2`, then associates weaker
raw boxes only to unmatched confirmed tracks. Weaker boxes cannot create or
confirm tracks. Both are controlled trials; baseline defaults remain unchanged.
Native detector classes also remain on track snapshots for primitive rendering.
This evaluation grouping does not change deployment class defaults. Renderers
should choose primitive asset type from retained native detector categories,
since `vehicle` is an evaluation ontology label.

The Foxglove reader exposes the global annotation union as `frame.vehicles`;
`metadata['boxes']` contains camera-projected boxes only. The native selected
agent replay uses that agent's annotations instead. Declare which source is
used. The deployment adapter joins normalized timestamps exactly; the Foxglove
reader first joins asynchronous streams by ordinal frame, checks record skew
against one nominal frame period, and assigns synthetic 10 Hz timestamps.
Report original timestamps/skew separately when interpreting motion latency.

`assignments.jsonl` preserves frame provenance and separate detection/track
matches, misses, false positives and identity switches for overlay production.
`metrics.json` records both metric sets and the matching protocol. Output
directories must be new so previous results remain intact.

For crowded overlays, `render.py --compact-labels` displays known take-local
GT IDs as `GT66`, BEVFusion vehicle tracks as `P14`, and frame-local proposals
as `D259.3`. The sequence remains in the title; saved assignments retain full
IDs. Unknown ID formats retain their full labels. Scores and failure markers
remain visible, and rendering does not alter poses or scores.

## Batch detector benchmark (`batch_detect.py`, `eval_detbench.py`)

Fast replacement for the per-frame ZMQ pilot when only single-frame detector
quality is needed. `batch_detect.py` runs inside the mmdet3d image, builds each
checkpoint's embedded training config (`checkpoint-embedded-config.py`) and its
unchanged `CooperSceneDataset` test pipeline (pkl paths, `LoadPointsFromFile`
4-D XYZI — the released `.bin` intensity is already /65535 —, `ImageAug3D`
crop .3→256×512, ±72 m filter). The installed `projects.BEVFusion` replaces
CooperScene's `models.bevfusion` (source-identical except KITTI-only loader
branches); spconv-layout sparse weights are permuted to the mmcv layout in
memory before a strict load. Several `--variant NAME:CONFIG:CKPT:{baseline,benchmark}`
share one DataLoader pass: each file is read once per sample (the sshfs dataset
mount, ~12 MB/s, is the bottleneck: ~0.5 s/sample). `benchmark` replaces
camera0 `lidar2cam`/`cam2img` by `calibration_results_all.json`
(`scene{take}_agent{agent}`). Output per variant and split:
`predictions.npz` (keys, boxes `[x,y,z_gravity,l,w,h,yaw]` LiDAR frame with
score ≥ 0.05, per-sample synchronized forward ms, evaluation-only LiDAR point
counts inside each pkl GT box) and `manifest.json` (hashes, pipeline, latency,
wall time). `--fp16` autocasts the forward (no measurable speed-up observed).

```bash
artifacts/detbench-20261012/run_batch.sh smoke test --limit 50   # A/B/C smoke
artifacts/detbench-20261012/run_batch.sh full test               # ~35 min (IO bound)
artifacts/detbench-20261012/run_batch.sh full val                # ~20 min
.venv/bin/python -m scripts.investigation.cooperscene_tracking.eval_detbench \
  artifacts/detbench-20261012/full --official-check
```

`eval_detbench.py` uses `src/evaluation/cooperscene_detection.py`
(`DetectionBenchmark`). Primary metrics are BEV: rotated BEV IoU AP@0.3/0.5/0.7
with the CooperScene/OpenCOOD protocol (greedy best-remaining GT per frame,
global score sort, VOC2010; GT filtered to the dataset's ±72 m/[-5,3] m range)
and inclusive 2 m BEV center Hungarian matching with P/R/F1 at scores
0.1–0.5, plus BEV center RMSE and yaw error of matched pairs at the best-F1
threshold. 3D IoU AP, z (center and bottom) and size errors are secondary
diagnostics only. Subsets — front-camera FOV (benchmark calibration, any box
corner/center in 1920×1200 at depth > 0.1), BEV range 0–30/30–50/50–72 m and
GT with ≥ 5 LiDAR points — use ignore semantics: matching is over all GT;
predictions matched to excluded GT are dropped, unmatched predictions count
only inside the subset. `--official-check` recomputes AP with CooperScene's
own `calculate_ap.py` (identical to 4 decimals on the smoke set). Frames keep
`ego_pose` and `iter_sequences()` yields time-ordered (take, agent)
sequences for a later world-frame tracking evaluation; note the info pkls carry
no GT track IDs (native YAML IDs would be needed).

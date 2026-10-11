# CooperScene tracking pilot, 2026-10-11

Only take 1 / vehicle 1 was evaluated: native frames **481260–481760**, 501
consecutive frames (50.00 s between first and last captures). The first 60
frames were development inputs; frames 60–500 were held out from parameter
selection. Other sequences remain on hold. Initial input preparation also cached take 3/V1 and take 4/V2;
those caches were not inferred after the user requested one sequence first.

## Acceptance and protocol

The minimum requested condition is that every front-camera GT annotation ID is
detected at least once. The stronger pilot target is at least 90% detection and
track recall for each ID visible in at least five frames, no never-detected ID,
and at most one identity switch per 100 visible GT observations. Here
“visible” in automated metrics means geometric front FOV; actual optical
visibility requires the camera/occlusion audit below. An optional
90% precision target prevents passing coverage by emitting excessive boxes.
Passing the minimum does not establish that the stronger target passes.

Geometric camera visibility requires the projected cuboid to intersect at least
16 pixels² of the original image, with all corners ahead of the 0.1 m near
plane. It does not test occlusion. Matching uses class-aware, maximum-cardinality
then minimum-distance Hungarian assignment with an inclusive **2 m XYZ center
gate**. Predictions and GT receive the same visibility filter. Reported
geometric IDF1 is a supported center-distance alternative, not official HOTA or
IoU MOTChallenge. Coverage is 283 projected GT observations, 11 annotation IDs,
nine IDs meeting the five-frame minimum. See the evaluation tool
[protocol](../scripts/investigation/cooperscene_tracking/README.md).

## Initial 60-frame development localization and GT

The start is archived XYZ's earliest strictly qualified frame, index 0, with
0.03555 m conditional reference disagreement. Its XYZ registration correction
is frozen across all trials. Each capture uses the retained raw GNSS/INS ENU
recipe with sensor extrinsics, the rotated 3 cm antenna lever arm, V1 mounting
correction, and V1/take-3-only position correction when applicable. G is never
applied directly to released ICP poses. No acceptance gate was changed.

For evaluation, released GT boxes are first transformed into the released
LiDAR frame, then placed with the retained reference LiDAR pose. Predictions
use the frozen XYZ-localized LiDAR pose. Conditional localization disagreement
in this window is 0.03555–0.03725 m, mean 0.03694 m. The retained recipe is also
initialization and fusion prior; this is reference consistency, not independent
GT localization accuracy. Original image captures lag LiDAR by 17.83–18.92 ms;
the pilot uses paired native images and the LiDAR-time camera pose and reports
that skew rather than inventing synchronized exposure times.

GT IDs agree across agents in inspected frames and recur across takes. One
take-1 ID, 55, jumps 49.075 m at 481290→481291. Canonical IDs are preserved;
separate discontinuity-split identity sensitivity is available. Neither GT
boxes nor identities enter inference or tracking.

## Controlled comparisons

All trials use identical images, native LiDAR geometry, timestamps, poses, GT
geometry, start index, and 2 m matching. The checkpoint and Docker image remain
fixed. The sidecar initially keeps only nuScenes class `car` at score ≥0.2.

| Input / selection | Detection precision | Detection recall | Track precision | Track recall |
|---|---:|---:|---:|---:|
| Original, score 0.2 | 41.7% | 5.3% | 43.9% | 17.7% |
| Original, score 0.05 | 32.6% | 14.8% | 22.7% | 28.6% |
| Intensity clipped to 255, original crop | 91.2% | 76.7% | 79.3% | 90.5% |
| Crop adjustment, score 0.2 | 46.4% | 9.2% | 47.5% | 23.3% |
| Crop + intensity clipped to 255, car only | 87.8% | 84.1% | 76.4% | 92.6% |
| Same clipped raw outputs, motor-vehicle grouping | 83.8% | 89.4% | 75.5% | 97.9% |

The fixed resize factor 0.48 cuts a CooperScene 1920×1200 image to approximately
original Y=667–1200. The explicit experimental config
`docker/bevfusion/cooperscene_front.py` uses 0.36, matching the upstream resized
height of 432 pixels and retaining more of the horizon. Shared MMDet image
augmentation updates the corresponding intrinsic transform.

Intensity is the dominant improvement: native sampled maxima exceed 2,800,
whereas the sampled nuScenes sweep tops out at 255. The optional pilot clamp
preserves XYZ and on-disk inputs; it is a controlled model-input adaptation,
not a calibrated physical reflectivity conversion. Clipping alone still misses
GT ID 58; the crop adjustment is needed for the minimum ID coverage. Both nearby sedans are
detected and tracked in all 60 clipped frames. RGB conversion and the decoder's
bottom-center→gravity-center conversion were checked in the actual image and
are correct. No blanket height correction was fitted to GT. Matched boxes still
show vertical error; explicit render grounding does not alter evaluation poses.

Native YAML labels are generic `vehicles`, with no detailed category. The
cached motor-vehicle trial declares a coarse `vehicle` group for car, truck,
bus, trailer and construction vehicle, preserving original detector classes
and the annotation label in separate fields. Pedestrians, bicycles, cones and
barriers are excluded. This is an explicit ontology comparison; it changes no
raw model output. The matched tracker comparison uses the corrected tracker
for both cached car-only and motor-vehicle runs. Motor-vehicle grouping has
geometric IDF1 **0.852**, **0 ID switches**, **1 fragmentation**; car-only
control has IDF1 0.815, 3 switches and 4 fragmentations.

Every projected GT ID is detected at least once in the clipped crop trial and
in the motor-vehicle trial. The stronger per-ID detection target and 90%
precision target still fail. Keep other takes on hold. Default sidecar
confidence, default camera config, and production class selection remain
unchanged; trial policies are explicit and results retain their manifests.

The tracker already uses a 3D constant-velocity Kalman filter and class-aware
Hungarian association, immediate births, a 0.5 s coast limit, and distance plus
Mahalanobis gates with dimension/heading costs. The expiry bug is fixed: old
tracks expire before association. Sparse detections now correctly create a new
ID after expiry; the old implementation's apparent continuity could revive an
expired ID. The live adapter has one in-flight and one replaceable pending
request, so it can drop intermediate detector frames. The offline pilot waits
for every frame and persists raw detections separately from tracks. The clipped
crop trial measured end-to-end client latency of 120.3 ms mean, 113.8 ms median,
134.3 ms p95 and 367.9 ms maximum. These include 100 ms client polling and
transport; they are not isolated GPU inference times. The mean exceeds the
roughly 100 ms capture interval, so live adapter frame dropping remains a
throughput concern. Offline metrics use original capture timestamps.

## Hosts and deployment

Verified SSH aliases: `lambda-justin` → CISL-lambda, 100.97.168.98, two RTX 4090s;
`gamma-justin` → gamma.engr.ucr.edu, 100.70.20.114, one RTX 5080.
`gamma-cisl` reaches the same Gamma under another user. Gamma is not Alien3.

Both isolated repositories are
`/home/justin/Documents/CISL-Projects/RemoteTeleop-tracking-20261011`, branch
`split-systems`, cloned from a Git bundle. The old repositories have unrelated
changes and were preserved: Lambda `main` at 8654a6e; Gamma
`vehicle-isaacteleop-local-integration` at e2c6d1f. Temporary integration patches
in our isolated clones are retained in named stashes before fast-forwarding.
The four expected commits were already attached locally before this work.

Lambda borrows the original `.venv` via a symlink, with protobuf/Foxglove
dependencies in isolated `.tracking-site`; imports verify torch 2.8.0+cu128,
NumPy 1.26.4, SciPy 1.17.1, OpenCV 4.11, gsplat, PyTorch3D and the compositor.
Gamma uses `.venv-render/run-python`, inheriting the existing torch
2.13.0+cu130 environment while installing rendering/MCAP dependencies in the
isolated environment. PyTorch3D is compiled there with an isolated NVIDIA
CUDA toolkit; the original environment is unchanged. Environment manifests,
compiler logs and pinned package lists live under each deployment's artifacts.

Mounted datasets were not present on either remote host at inspection. Lambda
already had offline MCAPs for takes 1–5 in the older proxy-batch repository.
This pilot stages native cached payloads from the local read-only source rather
than changing mounts or reusing old MCAP timestamp/localization policies.
Gamma's Riverside map is staged under `artifacts/reference-map-20261011`;
SHA256 is `f8731969529063e37a42f92ff3a9dc75b713bc1dc792bef41b0cd88f0701262d`.
No existing replay queue was active. Gamma's existing CloudXR service and
unrelated Lambda services remained running. Only our uniquely named offline
detector containers were started and stopped. No vehicle control was launched.

## Reproduce and inspect

Prepare a fresh single-sequence input directory locally:

```sh
.venv/bin/python scripts/investigation/cooperscene_tracking/pilot.py prepare \
  --inputs artifacts/tracking-inputs-NEW --sequence take01-vehicle1 --count 60
```

On Lambda, start the existing BEVFusion image with the staged `docker/bevfusion`
directory and original checkpoint directory mounted read-only, a new writable
raw-output directory, localhost endpoint 18766, score 0.2 and explicit
`--config /workspace/cooperscene_front.py --raw-predictions-dir /raw`.
The checkpoint mount is `/checkpoints` and the CLI must explicitly specify
`--checkpoint /checkpoints/bevfusion_converted.pth`.

```sh
PYTHONPATH=.tracking-site:$PWD .venv/bin/python \
  scripts/investigation/cooperscene_tracking/pilot.py infer \
  --inputs artifacts/tracking-pilot-inputs-20261011 \
  --sequence take01-vehicle1 --intensity-clip 255 --output artifacts/tracking-NEW

.venv/bin/python -m scripts.investigation.cooperscene_tracking.evaluate \
  artifacts/tracking-NEW/frames.jsonl --output-dir artifacts/metrics-NEW \
  --front-camera-only --split-discontinuous-gt --precision-target .9

.venv/bin/python -m scripts.investigation.cooperscene_tracking.render \
  artifacts/tracking-NEW/frames.jsonl \
  --inputs artifacts/tracking-pilot-inputs-20261011 \
  --assignments artifacts/metrics-NEW/assignments.jsonl \
  --output artifacts/overlays-NEW --front-camera-only
```

The labelled camera/LiDAR videos and PNGs are under
`artifacts/tracking-take1-motor-vehicle-front-overlays-20261011/`;
`tracking-take1-baseline-front-overlays-20261011/` is the baseline.
Wireframe diagnostics intentionally show boxes without depth occlusion and
are labelled accordingly. Primitive asset depth-buffer validation is separate,
using real GPU rasterization and shared asset/scene-depth tests. Corrected real-map
panels are in `artifacts/tracking-take1-gamma-primitives-front-20261011/`: frame
30 GT 31 is fully hidden by map and nearer-asset depth; frame 10 GT 9 is partly
hidden by a nearer asset. Height is
preserved by default in both rendering backends; ground placement is explicit.

Metrics and association records are in
`artifacts/tracking-take1-motor-vehicle-metrics-front-20261011/` and the individual
baseline/trial metric directories. Per-ID recall, misses, false positives,
identity switches, GT discontinuities, class/confidence diagnostics and cached
refilter provenance are retained. The initial mislabelled low-confidence run
reached the old detector before restart and is marked discarded; only the
`low-confidence-retry2` records were evaluated. A raw-logging Docker request
compatibility error was fixed and regression-tested before collecting them.

## Follow-up: lifecycle and failure audit

The first 60 frames remain the development window. A reproducible failure audit
and six labelled camera PNGs are in
`artifacts/tracking-take1-failure-audit-20261011/`. Of 30 detection misses,
27 have a geometrically eligible raw motor-vehicle proposal below score 0.2.
Lowering the threshold globally to 0.1 improves recall but reduces detection
precision to 67.5% and tracking precision to 48.4%. No tested scalar threshold
(0.1, 0.2, 0.3, 0.5) passes the declared acceptance checks.

Thirty-one of 49 unmatched detections contain no native LiDAR points inside
the predicted cuboid, compared with five of 253 matched detections. This is
a sensor-support diagnostic, not an automatic rejection rule: genuinely
occluded or distant vehicles can have sparse or absent returns. Checking the
native annotation union of agents 0–3 resolves none of the unmatched boxes.
GT absence alone does not establish that a physical vehicle is absent. GT 55's
jump and the distant pickup's uncertain camera correspondence remain flagged.
The published camera extrinsic exactly matches the raw calibration pickle;
that equality verifies provenance, not physical projection accuracy.

Optional tracker trials preserve primary detections at score 0.2 and use the
same 0.5 s coast, geometry, timestamps and GT. Confirmation requires consecutive
distinct captures; unconfirmed tracks are not emitted. Two-stage association
first matches score ≥0.2, then lets score 0.1–0.2 observations continue only
unmatched confirmed tracks. Weaker observations cannot create or confirm tracks.
Native detector class names are retained on grouped track snapshots for assets.

| Tracker policy | Track precision | Track recall | Geometric IDF1 | ID switches |
|---|---:|---:|---:|---:|
| Original lifecycle | 75.5% | 97.9% | 0.852 | 0 |
| Two-hit confirmation | 79.4% | 95.4% | 0.867 | 0 |
| Low-score continuation | 71.0% | 98.6% | 0.825 | 0 |
| Confirmation + continuation | 75.5% | 95.8% | 0.844 | 0 |

These are development comparisons. Two-hit confirmation removes 20 false
positive observations but loses seven true positive observations and worsens
the two weak IDs. Low-score continuation extends false tracks as well as true
tracks. Neither experiment is promoted as an overall solution; defaults remain
unchanged. Exact baseline replay compatibility and GT-input isolation are tested.

All 501 native take-1/V1 frames (481260–481760, 50.00 s) are prepared under the
same initialization and trajectory policy. First-60 image/LiDAR arrays and
existing metadata exactly match the pilot; explicit image dimensions are added.
The fixed XYZ initialization correction has conditional recipe disagreement of
0.0356–0.4364 m over the complete window. This frozen correction plus recipe
trajectory is not continuous per-frame XYZ registration, and the short-pilot
strict XYZ qualification must not be extrapolated to the whole sequence.
Candidate policies and the held-out frame range 60–500 are frozen in
`artifacts/tracking-take1-full-validation-20261011/frozen-policy.json` before
held-out inference. Full-sequence model evaluation is complete below; other takes remain on hold. Payload hashes are in the new input directory's
`payload-verification.json`.

### Corrected XYZ baseline before full-sequence inference

Archive inspection found the actual independently initialized per-frame XYZ
fusion outputs. The full comparison therefore uses
`artifacts/tracking-take1-full-xyz-inputs-20261011/`, superseding the frozen
initial-correction trajectory for this validation. The earlier 60-frame metrics
remain labelled development diagnostics under their original pose policy.

`pilot.py rebase-xyz` validates recipe/map hashes, exact frame identity/capture
time and retained raw-reference poses. Accepted archived fused estimates are
used directly. Rejected optimizer proposals never become estimator poses:
the last accepted left correction is propagated on the current raw GNSS/INS
recipe reference. All source arrays, GT, reference poses and capture times are
unchanged. The new metadata uses relative paths to the existing NPZ files.

There are 476 accepted XYZ frames and 25 explicitly propagated frames. Rejected
indices are 116, 393, 421 and 479–500. Maximum conditional reference disagreement
is 0.1201 m on accepted frames and 0.1619 m on propagated frames; the final
propagation age is 2.20 s. This is an archived independent-frame policy with
explicit fallback, not a continuous XYZ estimator. The detector/tracker trial
settings are unchanged and the localization correction was recorded before
any full-sequence model outputs were collected.

```sh
.venv/bin/python scripts/investigation/cooperscene_tracking/pilot.py rebase-xyz \
  --inputs artifacts/tracking-take1-full-inputs-20261011 \
  --output artifacts/tracking-take1-full-xyz-inputs-NEW \
  --map /path/to/riverside_iowa.ply
```

The depth-buffered optical audit is in
`artifacts/tracking-take1-depth-calibration-audit-20261011/`. Direct local
LiDAR point projections also show the distant pickup correspondence offset;
no ENU/world pose enters that projection. Six audited camera/PCD files exactly
match their raw backup sources, so release-to-native conversion did not rotate
points, resize images or rectify images. Both metadata sensor timestamps have
the same +37 s offset from raw filenames (clock conversion reason unverified),
preserving the 18–19 ms camera/LiDAR skew. The published extrinsic matches the
raw calibration. Those facts do not certify physical camera alignment or
surface identity; no speculative extrinsic or height correction was applied.


## Full-sequence validation with archived per-frame XYZ

Both frozen model trials ran on all 501 frames with identical source payloads,
GT, timestamps, intrinsics and actual XYZ poses. The model-input trial changes
camera crop from .48 to .36 and clips intensity at 255. Both sides group the
native car/truck/bus/trailer/construction-vehicle classes for generic vehicle
GT, retain native class metadata, and use a .2 primary score threshold. These
paired changes establish an input-recipe improvement, not isolated attribution
to either change. No checkpoint weights were trained or changed.

Full geometric front-FOV coverage is 1,664 GT observations and 29 annotation
IDs, 28 present for at least five frames. All numbers below use the declared
2 m XYZ center matching gate; predictions receive the same FOV filter as GT.

| Input/tracker | Detection precision / recall | Track precision / recall | ID switches | Fragmentations | Geometric IDF1 |
|---|---:|---:|---:|---:|---:|
| Original / control | 31.7% / 8.8% | 36.5% / 20.0% | 26 | 29 | .134 |
| Corrected inputs / control | 74.4% / 79.0% | 67.1% / 85.3% | 8 | 10 | .734 |
| Corrected / two-hit confirmation | same | 73.2% / 83.4% | 2 | 6 | .770 |
| Corrected / low-score continuation | same | 61.5% / 87.0% | 9 | 9 | .705 |
| Corrected / both | same | 72.1% / 84.6% | 3 | 5 | .767 |

Held-out frames 60–500 use tracker state warmed from frame zero, with evaluation
identity history starting at frame 60. Original detection precision/recall is
30.8%/9.3%; corrected is 72.4%/76.8%. Corrected control track precision/recall is
65.4%/82.8%, geometric IDF1 .709, eight switches and nine fragmentations.
Two-hit confirmation gives 71.8%/81.0%, IDF1 .750, two switches and five
fragmentations. Low-score continuation adds false positives; no optional policy
is promoted and defaults remain unchanged. Held-out results are reported once;
no parameters were selected from these results.

The corrected control's matched front-FOV detection center error is mean
0.794 m, RMSE 0.839 m; dimension absolute errors (length, width, height) average
(0.373, 0.227, 0.122) m; length-axis yaw error averages 11.75 degrees modulo 180.
Track center RMSE is 0.868 m. These are conditional reference errors of matched
objects, not localization-independent accuracy or errors of missed objects.

The minimum geometric coverage condition fails: GT 69 (24 FOV frames), 68
(27) and 182 (12) never match a primary detection. None has an above-.2 motor
proposal within 2 m during its FOV window. Center rays intersect nearer GT
cuboids in 24/24, 18/27 and 9/12 frames, respectively. GT 69 also has very sparse
LiDAR support. Such intersections indicate possible occlusion, not a certified
visible-surface mask. Camera/LiDAR projections still have unresolved physical
correspondence offsets. Therefore neither zero geometric misses nor coverage
of every physically visible vehicle is established. Increasing tracker coast
cannot recover missing detector evidence. Keep all other takes on hold.

Exact reports and labelled artifacts:

- `artifacts/tracking-take1-full-clipped-policy-comparison-20261011/`: all four
  frozen policies, per-ID coverage, exact equal-input checks, independent XYZ audit.
- `artifacts/tracking-take1-full-original-baseline-comparison-20261011/`: paired
  original comparison, separate development and held-out metrics.
- `artifacts/tracking-take1-full-front-overlays-20261011/`: full camera+LiDAR
  BEV detection and track videos; wireframes intentionally have no occlusion.
- `artifacts/tracking-take1-full-visuals-20261011/coverage.png`: per-ID timeline.
- `artifacts/tracking-take1-full-failure-audit-20261011/`: camera/BEV examples,
  raw proposal scores, local LiDAR support and cuboid ray intersections.
- `artifacts/tracking-take1-full-gamma-primitives-labelled-20261011/`: sixteen real-map
  depth-buffered panels at eight indices using native primitive kinds and
  preserved XYZ height. Generic vehicle GT uses explicitly labelled car-shaped
  primitives scaled to its annotated dimensions, a rendering-only assumption.
  Rendering ground placement is disabled.
- `artifacts/tracking-take1-full-validation-20261011/actual-execution-provenance.json`:
  executed commit and corrected input hashes; earlier provenance is retained.

The independent XYZ audit confirms 476 registration/fusion-accepted estimates,
including 465 below the strict .1 m conditional recipe-error threshold. Eleven
accepted estimates exceed that qualification threshold. The 25 rejected frames
use explicit propagation; acceptance is not synonymous with strict reference
qualification. Retained localization uses GT vehicle masks with .2 m padding,
and V1 mounting calibration includes take 1. GT never enters detector or tracker,
but is not evaluation-only throughout upstream localization preparation. The
holdout validates tracking within this sequence, not independent localization
calibration. Published/raw-pickle extrinsic agreement establishes provenance,
not physical calibration accuracy; the 3 cm antenna lever arm remains assumed.

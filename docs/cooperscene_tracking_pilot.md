# CooperScene tracking pilot, 2026-10-11

Only take 1 / vehicle 1 was evaluated: native frames **481260–481760**, 501
consecutive frames (50.00 s between first and last captures). The first 60
frames were development inputs; frames 60–500 were held out from the initial
nuScenes input-parameter selection. Subsequent native/checkpoint and tracker
comparisons are explicitly exploratory or training-overlapping below. Other
sequences remain on hold. Initial input preparation also cached take 3/V1 and take 4/V2;
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
IoU MOTChallenge. Initial development coverage is 283 projected GT observations,
11 annotation IDs, nine IDs meeting the five-frame minimum; full-sequence coverage
is 1,664 observations, 29 IDs and 28 eligible IDs. See the evaluation tool
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

## Follow-up development input and provenance audits

The model-input ablation uses only original development frames 0–59 with the
same archived XYZ poses, GT, timestamps and image/LiDAR source arrays. Primary
score .2, motor grouping and the default tracker are fixed. It isolates the
paired input changes without retuning on held-out frames.

| Input recipe | Detection precision / recall | Track precision / recall |
|---|---:|---:|
| Crop .36, clip255, single scan (reference) | 83.8% / 89.4% | 75.5% / 97.9% |
| Crop .48, clip255, single scan | 86.6% / 80.2% | 76.5% / 90.8% |
| Crop .36, unclipped, single scan | 44.6% / 10.2% | 44.8% / 24.4% |
| Crop .36, clip255, past-only sweeps within .5 s | 88.6% / 66.1% | 78.6% / 79.2% |

Intensity clipping accounts for most of the development improvement; the crop
adds recall with some precision cost. The installed nuScenes config expects
nine past sweeps plus a current scan, with XYZI/time-lag features. The explicit
sweep trial uses available native 10 Hz scans, positive lag in seconds and
archived XYZ ego-motion compensation. It supplies roughly five past scans
within .5 s, not exact nuScenes sampling, and does not compensate moving objects.
Its coverage is worse; it is not promoted or run on held-out frames. Single-scan
wire messages and tracker defaults remain unchanged. New outputs and exact
reference-field checks are in
`artifacts/tracking-take1-development-ablation-20261011/`.

The box-convention audit verifies every raw gravity center equals bottom center
plus half-height and verifies semantic length/width yaw conversion preserves
cuboid corners. Matched front-FOV detections have a signed native-LiDAR Z residual
averaging +.709 m. Subtracting half-height would artificially reduce this error,
but independent SUSTech cube construction and annotation export code confirm
geometric GT centers. No bottom-as-center shift or localization height correction
is justified. Source center conventions are verified; physical vertical accuracy
remains unresolved. See `artifacts/tracking-take1-box-convention-audit-20261011/`.

The full native/refined annotation join finds 8,186 same-frame/ID counterparts
among 8,195 native observations. All seven inspected low-coverage IDs match
refined geometry exactly. Literal source `Car` labels carry a hardcoded-export
initialization caveat and do not independently prove detailed physical categories.
The shared +37-second timestamp shift is an intentional operation in the source
snapshot; this proves pairing preservation, not the physical clock convention.
No distortion coefficients, calibration capture dimensions or rectification
report were found. See `artifacts/tracking-take1-optical-model-audit-20261011/`.

An identity-event audit finds seven of eight control front-FOV switches follow
4–18-frame match gaps; the remaining event loses the previous track on adjacent
frames. Low-score continuation also introduces ambiguous switches while old
tracks remain nearby. This does not justify a larger global coast interval.
After its first request, the corrected detector's client latency averages
108.12 ms, median109.00 ms, p95 116.79 ms over500frames; the first request waited
7.25 s despite a103.46 ms server report. Sequential offline inference retains
allcaptures; latency alone does not demonstrate10Hz throughput. Exact events
and startup/steady timing are in
`artifacts/tracking-take1-full-identity-audit-20261011/`.

### Native CooperScene checkpoint preparation

The [official model repository](https://huggingface.co/cisl-hf/CooperScene)
provides a single-agent LiDAR-camera checkpoint, pinned at revision
`2e7c7080a5061e28a9f3a32fbda075c7359eb815`, SHA256
`f06cdc3c78610b882663adbe68b6621a2a1c6fd0bff119e985f7219be335ed8d`.
Its input contract is four point features, generic vehicle output, crop .3 to
256×512, and published converter intensity division by65535. It must not reuse
the nuScenes five-feature/sweep/clip255 adapter. Native single-agent architecture
and CUDA sources are byte-identical to the installed upstream implementation;
only loading and cooperative extensions differ. The prepared runtime preserves
the embedded checkpoint model config and native augmentation, supplies in-memory
four-feature inputs and skips redundant backbone initialization downloads.

The native checkpoint's training metadata points at the released training data;
take1 results are adaptation/in-training diagnostics, not an unbiased checkpoint
holdout. Its ±72m output range also exceeds this evaluation's ±54m GT cache scope;
extra out-of-scope proposals require separate accounting. Preparation evidence,
embedded config and source-equivalence hashes are in
`artifacts/tracking-cooperscene-checkpoint-audit-20261011/`.

The native checkpoint has now completed the same60-frame development comparison
with four-feature inputs and intensity division by65535. Detector front-FOV
precision/recall is93.8%/96.8%, matched center RMSE .348m (corrected nuScenes
.827m); track precision/recall89.5%/98.9%, IDF1 .940, zero switches and zero
fragmentations. All11 geometric front-FOV GT IDs are detected at least once,
but GT114512 detection recall78.3% and GT58 detection/track recall77.8%
still fail the stronger per-ID90% target. This remains an in-training/adaptation
diagnostic, not independent model generalization or certified RGB visibility.

Both candidate and baseline were explicitly scoped to ±54m in current retained
reference-LiDAR XY for evaluation; no GT, detections or tracks were excluded in
this60-frame comparison. The native tracker consumed all original detector
outputs, independent of evaluation scoping. Exact common poses, GT, timestamps,
intrinsics and payload paths are verified. The initial native attempt failed
during warmup because LiDARPoints was still declared5D; its logs remain in
`artifacts/tracking-take1-native-development-20261011/`. The fixed successful
run and camera/BEV videos are in
`artifacts/tracking-take1-native-development-v2-20261011/`.

The runtime config uses an empty lazy NuScenesDataset factory solely because
the installed init_model queries a dataset palette. It loads no annotations;
checkpoint metadata continues to declare classvehicle and controls model labels.
The embedded native model config and camera augmentation remain unchanged.

### Native checkpoint: full single-sequence result

The native checkpoint completed all 501 take1/V1 frames with the same archived
XYZ localization, timestamps, GT and default tracker. No other sequence has been
expanded. Independent replay verified all input poses, camera/LiDAR payload
hashes, four features, normalized intensity, checkpoint/config hashes, class
mapping and tracker outputs. Both models use the declared symmetric ±54 m
reference-LiDAR XY evaluation scope, inclusive 2 m XYZ center association and
geometric front-camera cuboid filtering. The tracker sees unscoped detections.

| Front-camera metric | Corrected nuScenes | Native CooperScene |
|---|---:|---:|
| Detection precision / recall | 74.4% / 79.0% | 96.8% / 92.2% |
| Track precision / recall | 67.2% / 85.3% | 92.9% / 95.9% |
| ID switches / fragmentation | 8 / 10 | 11 / 6 |
| Geometric IDF1 | .734 | .918 |

Native detection center RMSE is .406 m, mean dimension absolute errors
(.114, .057, .062) m for length/width/height, and mean length-axis yaw error
4.21 degrees. There are 1,664 geometric front-FOV GT observations and 29 IDs;
**all 29 receive at least one detection**. This meets the user's minimum
geometric track-coverage condition. The stronger 90% per-ID detection and track
coverage criterion still fails; GT130 receives only 3/23 detections and 15/23
track matches. Precision and switch-rate gates pass. These are conditional
reference-agreement metrics, not independently certified GT or optical accuracy.

The native model trained on take1. Its improvement is an adaptation diagnostic,
not a model-generalization result. GT69's seven matched detections trace correctly
to raw proposals, but four contain zero current LiDAR points; all its center rays
intersect nearer annotated vehicles. Literal camera visibility cannot be inferred
from a projected box alone. Source lens/rectification metadata remain unavailable.
GT is excluded from detector/tracker input; the retained upstream localization
preparation still carries the previously disclosed GT vehicle-mask dependency.

An evaluation-only stratification of all1,664 observations finds462 camera-to-GT
center rays intersect nearer supplied GT cuboids: detection/track recall is
78.1%/89.4%. For the remaining1,202 observations without a tested foreground
intersection, recall is97.7%/98.4%. GT69 andGT130 are entirely in the obstructed
proxy group. Some per-ID gaps remain in the other group. A center ray does not
model the whole visible object, unannotated occluders or calibration uncertainty;
this proxy does not replace the primary metrics or acceptance gates. Exact
per-observation evidence is in the failure audit's obstruction stratification.

Eleven front-FOV ID switches were reproduced independently. Three duplicate
proposal episodes create parallel IDs and switch back; other switches follow
observation gaps or competing/coasted duplicates. All switch frames have accepted
XYZ error .018–.091 m, with no localization spike. Native GT55's off-camera
discontinuity affects global-ID sensitivity, but none of the front-FOV switches.

An optional two-hit confirmation trial improves development track precision
89.5%→93.5% and geometric IDF1 .940→.951 while reducing recall 98.9%→96.8%.
The policy was frozen before replaying all501 cached native outputs. In the full
same-sequence comparison, track precision/recall becomes95.3%/93.4%, switches5,
fragmentations5 and IDF1 .922. Detections and every non-track input are exact.
This is exploratory: baseline full-sequence failures were already inspected.
The extra birth delay harms short-lived coverage, so tracker defaults remain
unchanged and the stronger per-ID gate still fails.

The reproducible raw-XYZI server adapter is
`docker/bevfusion/cooperscene_native_front.py`. Mount the pinned checkpoint audit
directory read-only at `/native`, use this adapter as `--config`, the pinned
`/native/bevfusion_single_lidarcam.pth` checkpoint and score threshold .2.
**Clients must send raw intensity: do not also pass pilot `--intensity-scale`.**
The adapter divides intensity by65535 and declares four features. A real GPU
two-frame parity check gives exactly identical predictions to the earlier
host-scaled native run. The updated code is deployed to both isolated Lambda
and Gamma repositories; normalization tests pass on both. Only owned offline
containers were started and stopped. Original worktrees and queues are preserved.

Native detector client latency after the first request averages 109.16 ms (p95 123.86 ms); its first request waits7.39s for startup. Server processing after the first request averages97.46ms. These are serialized offline detector round trips, excluding tracking/rendering, not a verified real-time10Hz throughput claim. Exact timing is in the full native artifact's `latency.json`.

Artifacts:

- `artifacts/tracking-take1-native-full-20261011/`: exact run script, raw outputs,
  scoped metrics, per-ID coverage chart and 501-frame detection/track videos.
- `artifacts/tracking-take1-native-full-identity-audit-20261011/`: independent
  full-input validation, reproducible tracker replay and ID-switch evidence.
- `artifacts/tracking-take1-native-full-failure-audit-20261011/`: 60 camera/LiDAR
  PNGs, confidence, point-support, competing-GT and occlusion-proxy analysis.
- `artifacts/tracking-take1-native-full-gamma-primitives-20261011/`: 16 labelled
  real-GPU map/primitive panels. Shared scene/asset depth correctly removes eight
  fully occluded sampled assets and partly occludes94; all localized heights are
  preserved, with no rendering ground placement or unsupported asset categories.
- `artifacts/tracking-take1-native-server-scale-smoke-20261011/`: canonical raw
  input deployment script, logs and exact two-frame prediction parity proof.
- `artifacts/tracking-take1-native-confirmation-trial-20261011/`: frozen optional
  two-hit tracker trial, common-input validation and identical-scope metrics.

Keep the one-sequence pilot until the visibility limitations and remaining
per-object gaps are resolved or explicitly scoped. Expansion should use a
declared checkpoint-independent sequence and frozen input/track policies.

### Duplicate-birth guard and declared-test preflight

An optional `--suppress-duplicate-births` cached-trial flag prevents an unmatched
primary detection from starting a second track when it substantially overlaps
the current primary detection assigned to an already-confirmed track. It uses
same-class cuboid IoU >=.3 and length-axis agreement within30 degrees modulo180.
Overlap is calculated in the matched detection's axes, preserving arbitrary
world rotations and object heights; nonparallel vertical axes are skipped.
Association, correction, coast duration, confidence and detector outputs are
unchanged. Coasting ghosts, newly born/tentative tracks and low-score continuation
detections cannot serve as suppression references. Tracker defaults stay disabled.

The first60 development frames are unchanged: there are no qualifying duplicate
births. The full501 comparison is exploratory because its earlier failure audit
motivated this mechanism. The optional guard with immediate confirmation yields
track precision95.3%, recall95.9%,5switches,6fragmentations and geometricIDF1 .930.
Default precision/recall was92.9%/95.9%,11switches and IDF1 .918. False-positive
front tracks fall122→79. Independent audit finds14 suppressed birth pairs, all
nearest the same supplied GT object, with zero distinct-GT suppression flags.
Exactly six old switches from the GT38/65/142 duplicate episodes disappear;
there are no new front switches and no gained/lost matched GT observations in
either front FOV or the full ±54m scope. Every non-track field and detector
metric is exact. A fresh default replay remains byte-identical across all501
frames. Independent convex-polyhedron volume checks match the overlap helper
across100 rotated cuboid pairs to4e-16 IoU error.

Adding two-hit confirmation to the guard yields96.6% precision,93.4% recall,
3switches,5fragmentations and IDF1 .929. It loses41 matched front observations.
Some switch reductions reflect delayed/omitted earlier identities rather than
repairing detection gaps, so immediate-confirmation guard is the better coverage
candidate. The stronger per-ID coverage gate still fails. Related84 tests pass
locally;46 focused tracker/trial tests pass on both updated remote repositories.

Separate confidence-only development trials retain all inputs and use default
tracking: score.15 gives detection90.8%/98.2% and tracks87.3%/99.6%; score.10 gives
detection88.6%/98.6% and tracks82.9%/99.6%. Lowering the global threshold adds
false positives and identity instability; neither is promoted or run on the
remaining441 frames. Primary score.20 remains fixed.

Evidence is in `artifacts/tracking-take1-native-duplicate-birth-trial-20261011/`:
frozen policy, overlap verifier, default replay, unchanged-input comparison,
separate birth diagnostics, causal audit, paired camera/LiDAR PNGs and labelled
guard track video. Confidence trials are in
`artifacts/tracking-take1-native-confidence-development-20261011/`.

The one-sequence pipeline now has independent input, tracker and metric replay
checks sufficient for a narrowly scoped declared-test comparison, with optical
and conditional-localization limitations retained. Take2/V1 preparation targets
300 frames from its earliest strictXYZ-qualified start index17/frame487867
through index316/frame488166, without moving the start to a GT-rich window.
This includes451 geometric front-GT observations and11 eligible IDs; its first60
alone would contain only one ID. All881 archived frame records and remaining
source metadata were checked. The current exact checkpoint-declared train
annotation file excludes take2 and the test file includes it. Historical training
file hashes are not embedded in the checkpoint, and take2 contributed to the
retained V1 mounting calibration, so this is a declared detector-test split,
not independent localization/calibration GT. Preflight evidence is in
`artifacts/tracking-heldout-preflight-20261011/`; detector/tracker policies were
frozen before preparing the new inputs. The completed comparison is below; no
all-take expansion is launched.


### Take2/V1 declared detector-test comparison

The frozen 300-frame comparison completed on Lambda at commit aa0dc01, using
four-feature native BEVFusion, raw XYZI with server-only intensity /65535,
score .20, no temporal sweeps, and identical archived XYZ inputs for all trials.
An independent source audit verifies all 300 raw payload hashes, recipe poses,
GT transformations, camera composition and timestamps. There are 298 accepted
XYZ poses and two flagged propagated poses; maximum reference error is .09046m.
No consecutive native GT-ID discontinuity was found within this window. IDs are
take-scoped and evaluated for V1 alone; shared cross-agent IDs are not merged.

Matching remains class-aware inclusive 2m XYZ center distance, maximum
cardinality/minimum distance Hungarian, symmetric reference-LiDAR XY ±54m scope,
and projected-cuboid front FOV area >=16 native pixels² with all corners >.1m.
451 front observations span 11 eligible GT IDs. Every ID is detected at least
once, but the stronger per-ID >=90% detection and tracking coverage criteria
fail. Geometric FOV is not certified optical visibility; occlusion and native
camera calibration limits remain explicit. GT is evaluation-only for detection
and tracking; upstream archived localization used a GT vehicle mask, as already
noted. Take2 is excluded from the checkpoint's currently available declared
training annotation file, but checkpoint historical training hashes are absent
and retained V1 mounting calibration includes take2.

| Frozen variant | Track precision | Track recall | ID switches | Fragmentation | Geometric IDF1 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Default immediate birth | 59.4% | 85.4% | 14 | 7 | .542 |
| Optional duplicate-birth guard | 62.7% | 85.4% | 6 | 7 | .607 |
| Two-hit confirmation | 64.3% | 79.8% | 9 | 4 | .580 |
| Guard plus two-hit confirmation | 66.9% | 79.8% | 3 | 4 | .645 |

Detections are identical in all four trials: 337TP, 123FP, 114FN, precision73.3%
and recall74.7%. Matched center RMSE is .829m, dimension MAE L/W/H is
.087/.034/.051m, and length-axis yaw error is5.32 degrees modulo180. These are
conditional retained-reference errors, not independent physical GT accuracy.
The guard retains aggregate track TP385/FN66 while reducing FP263→229.
Confirmation loses25 matched front observations and is not selected as a
coverage repair. Neither default nor guard meets the precision90% target or
switch-rate target; only guard plus confirmation meets the switch-rate target,
while still failing coverage and precision. Defaults remain unchanged; no heldout
threshold or parameter retuning was performed.

For the guard, GT84 is detected7/70 and tracked21/70 eligible frames; GT92 is
50/79 and68/79. Detection coverage also falls below90% for GT48(18/21) and
GT96(43/52); tracking coverage fails for GT114(6/7). These failures must be
inspected before any broader evaluation. Evidence, frozen scripts, all raw
predictions, metrics and labelled camera/LiDAR videos are under
`artifacts/tracking-take2-native-heldout-pilot-20261011/`. Separate real-map
primitive depth panels are under
`artifacts/tracking-take2-native-heldout-gamma-primitives-20261011/`; all object
heights are preserved, ground placement is None, and no SAM assets are used.

Warm client latency is111.43ms mean/117.61ms p95 over299 requests; first request
including startup is7.13s. Warm server processing mean is101.07ms. Serial offline
replay preserves captures and is not proof of realtime10Hz throughput. This
small declared-test comparison exposes a detector/coverage gap despite the
tracker duplicate improvement; all-take expansion remains deferred.


Independent metric replay reproduces every match and scalar with a separately
implemented geometric projection, Hungarian matching, continuity counts and
geometric IDF1 calculation. It verifies source/wire payload hashes, native
four-feature class mapping and all300 error-free requests. The guard suppresses
seven same-nearest-GT birth pairs, with no other GT within2m of either proposal.
It removes eight old front ID switches without new switches or any gained/lost
matched GT-frame observations, globally or in front FOV. Thus its duplicate
repair transfers to this declared test window, although overall targets fail.

The two worst IDs84/92 lie about47/46m away and have sparse current LiDAR support
(median5/0 points, maxima15/1). Their projected boxes sit above the cars in the
native camera panels, so actual optical visibility/correspondence cannot be
certified. Lower-score proposals account for54/63 misses on84 and21/29 on92;
the remainder have no retained post-head proposal within2m. Lowering confidence
is therefore not established as a safe repair. A center-ray GT-vehicle proxy
marks109 observations obstructed (detector recall39.4%, track61.5%) and342
without tested intersections (86.0%,93.0%). This proxy omits non-GT occluders and
does not resolve camera calibration or optical visibility; primary metrics stay
unchanged. Native image/LiDAR failure panels are in the separate failure audit.

All three diagnostic videos were decoded as300 frames at1872×796. Twelve fixed
uniform-sample Gamma panels use actual CUDA Gaussian scene depth and shared
PyTorch3D primitive depth. Among sampled asset instances,41 are partially
occluded, one visible and one fully occluded. At frame299 the predicted track
84 has311 projected mesh pixels:75 blocked by scene depth and236 by nearer
assets, leaving zero visible pixels. Object poses and heights were unchanged;
this demonstrates the shared depth buffer, not correctness of map geometry or
optical visibility of every GT target.


The failure audit also identifies six front-camera scope losses with valid
all-LiDAR matches: GT37 tracking at488095, GT96 detection/tracking at488115,
GT108 detection/tracking at488140, and GT114 tracking at488160. Predictions fall
outside the symmetric geometric camera filter although GT passes. In particular,
GT114's6/7 camera track coverage does not mean its track was absent in LiDAR.
These boundary effects are reported separately; primary matching is unchanged.


### Exact camera timestamps and separate validity sensitivity

The original take2 source export marks frame488157 (pilot ordinal290) with
image_ns=0 and copies the previous PNG. Native and released488156/488157 images
share SHA087eabd2181e581706c85aa182644b2347bc067c7e118bfdaf7b295c810643fe.
Earlier source-consistency checks reproduced this timestamp exactly but did not
establish capture validity. Future prepared inputs retain exact integer
image_timestamp_ns. The optional evaluation flag
`--front-camera-only --require-valid-image-timestamp` rejects absent/malformed
metadata and skips nonpositive camera timestamps for front scoring only. It
preserves identity continuity, reports skipped frame/GT provenance, and applies
no skew gate or inferred capture time. Positive timestamp alone does not certify
calibration, optical visibility or freshness.

The primary300-frame benchmark remains unchanged. A separately labelled source-
timestamp sensitivity evaluates299 camera frames/446 geometric GT observations;
all300 global LiDAR metrics and split-ID sensitivity metrics are exactly unchanged.
Detection precision/recall becomes72.97%/74.44%; guard tracks62.40%/85.20%,6switches,
7fragmentations. The later low-continuation exploratory candidate gives62.96%/
92.60%,4switches,3fragmentations. All11 IDs remain ever detected and all variants
still fail the full coverage/precision criteria. No source, detector request,
tracker input, localization or GT geometry was changed. Evidence and reproduction
scripts are in `artifacts/tracking-take2-camera-timestamp-sensitivity-20261011/`;
exact source proof is in
`artifacts/tracking-take2-camera-correspondence-audit-20261011/`.

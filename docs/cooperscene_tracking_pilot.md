# CooperScene tracking pilot, 2026-10-11

The latest same-sequence run covers all 864 Take 2/V1 frames from the fixed
qualified start. Native-export front detection precision/recall are 80.1%/74.8%;
tracking achieves 74.8%/92.1%, with 39 of 40 geometric-front GT IDs ever matched.
GT208 remains missed. Matched detection ENU horizontal RMSE is 0.824m (0.294m
within 20m); vertical RMSE is 0.407m. XYZ localization reference RMSE is 0.037m,
but three tail frames slightly exceed 0.1m. See the full-sequence section below.
These results do not meet the coverage/precision conditions or establish good
horizontal accuracy throughout the evaluated range. Grounding remains an
explicit rendering option that preserves scored poses.

A controlled 300-frame Take 2/V2 pilot now follows the verified source, model,
matching and replay checks. It detects all 19 geometric-front GT IDs at least
once, with detection precision/recall 83.5%/89.4%; tracking recall is 96.9% and
geometric IDF1 0.805. All XYZ translation reference errors are below 0.1m.
Stronger accuracy targets remain unmet; V2 does not resolve the V1 GT208 miss.
Take 2/V3 now covers its 134 available frames after the same start policy:
14 of 15 geometric-front GT IDs are detected, with GT130 remaining missed.
All three agent probes preserve calibration, scoring heights and tracker defaults.

The next controlled 300-frame Take 3/V1 probe detects all 16 geometric-front GT
IDs at least once, with detection precision/recall 98.1%/92.8% and tracking
93.4%/96.4%. Detection horizontal/vertical RMSE is 0.406/0.131m. Its XYZ reference
errors are all below 0.1m. Take 3 overlaps the current declared training split;
this better window does not resolve Take 2's misses or establish held-out accuracy.

Take-2 results remain provisional: the native and released refined GT
exports disagree on the same 300 frames. Existing native-export scores are
preserved, but do not establish complete front-camera GT coverage. A separate
unfiltered refined-export comparison now finds 17 never-matched geometric-front
IDs; neither coverage nor precision targets pass. Camera correspondence and
annotation authority remain caveats. The working coverage condition uses projected
bbox intersection, including partly off-screen boxes; physical visibility is not
an additional gate. Controlled expansion uses the verified protocol while
retaining these limitations; failing accuracy targets is reported separately.

The initial pilot evaluated take 1 / vehicle 1: native frames **481260–481760**, 501
consecutive frames (50.00 s between first and last captures). The first 60
frames were development inputs; frames 60–500 were held out from the initial
nuScenes input-parameter selection. Subsequent native/checkpoint and tracker
comparisons are explicitly exploratory or training-overlapping below. At that
initial stage other sequences were held. Preparation also cached take 3/V1 and take 4/V2;
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

The working criterion follows the earlier bbox-proxy logic: a GT box qualifies
when its projected bounding rectangle overlaps the camera image by at least
16 pixels². Full containment is unnecessary. This same shared projector gates
predictions and GT; its segmentation-prompt default of 64 pixels² is a separate
consumer policy. Existing calibration/XYZ values remain fixed. Calibration and
GT-source uncertainties are reported alongside the geometric scores; they do
not prevent continuing detector/tracker comparisons on this sequence.

Geometric camera visibility requires the projected cuboid bounding rectangle to
intersect at least 16 pixels² of the original image, with all corners ahead of the 0.1 m near
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


### Confirmed-track continuation and residual source audit

One development candidate retains primary detections and births at.20 while
allowing.10–.20 proposals to update unmatched previously confirmed tracks. It
uses immediate confirmation, the optional duplicate-birth guard and unchanged
.5s coast. No weak proposal can create a track. The first60 take1 development
frames gain two matches and twoFP; full501 guard recall95.91%→97.12%, precision
95.28%→95.00%, switches5→2, fragmentation6→3, geometricIDF1 .9302→.9522. Independent
replay finds24 gained and4 lost GT-frame matches (two each on GT85/130), net20.
Detections and every non-track field are unchanged. Take1 overlaps training.

The single candidate was then run on the already-inspected take2 window as an
explicitly exploratory followup, not untouched heldout validation. On the original
300 frames, guard recall85.37%→92.68%, precision62.70%→63.24%, switches6→4,
fragmentation7→3, geometricIDF1 .6066→.6763. GT84 tracking21→49/70 and GT92
68→73/79;33 matched GT-frame observations are gained with no losses. All primary
detections remain unchanged. Independent replay confirms511/589 low updates in
take1/take2 and zero low births. In take2,369 weak updates are farther than2m from
any suppliedGT, so continuation also sustains unsupported proposals. Strong per-ID
coverage and precision still fail; defaults are not promoted. Cross-host detection
pose reconstruction differed only by1–2ULP; metadata is exact, poses are checked
within1e-12, and archived canonical detections are preserved in evaluation copies.
Evidence and labelled video/coverage plot are in
`artifacts/tracking-take1-native-continuation-development-20261011/` and
`artifacts/tracking-take2-native-continuation-exploratory-20261011/`.

Residual guard229FP comprise116 detector-updated and113 coasted outputs; coasting
also contributes50TP. Stable track50 accounts for82FP before GT92's first supplied
annotation. Original unfiltered native/refined labels confirm those82 absences,
but source camera/point views cannot prove that the persistent proposal is a real
vehicle. Canonical counts remain unchanged; do not suppress this track or shorten
coasts merely to remove those scores. Four of six guard switches are expiry/rebirth,
one is identity migration through a finite4.454m association, and one is a related
rebirth. All six have accepted XYZ errors .0194–.0258m; no localization change is
justified. Lifecycle/cost/weak-update evidence and13 camera/LiDAR panels are in
`artifacts/tracking-take2-residual-tracker-audit-20261011/`.

Original take2 ROS SQLite recordings provide additional independent source evidence:
three selected1920×1200 RGB images equal native BGR pixels exactly, and every native
XYZ point uniquely pairs with original os_sensor PointCloud2 within1.43µm, with exact
intensity. Both original sensor headers differ from export metadata by+37s, preserving
pair skew; this is not a37s inter-sensor offset. Front-sector per-point times lie about
40–59ms into100ms scans. Images follow those returns by about5–23ms before
the copied-image event, but followup exports after the loss use images about76–94ms
older than front-sector returns. This affects moving-object correspondence. Late
raw-GNSS ego motion bounds (<.14px rotation/.022px translation at45m) do not explain
persistent fixed-scene projection offsets. Export cropping and a systematic rigid
point conversion are ruled out in the checked frames. Actual calibration capture
ROI/binning/rectification/distortion, exposure/PTP and deployed driver configuration
remain unavailable; source consistency is not physical calibration accuracy. No GT
fit, height shift, source edit or XYZ refinement was applied. Reproducible original-
bag, point-time, clock, annotation-birth and direct projection evidence is in
`artifacts/tracking-take2-camera-correspondence-audit-20261011/`.

### Controlled front-scan camera-pairing followup

A fresh exploratory take2/V1 rerun preserves the same 300 LiDAR samples, XYZ/reference
poses, GT, start frame, camera K/extrinsics and raw intensity policy. Its frozen,
GT-independent rule selects the positive published camera timestamp nearest the
LiDAR header +50 ms, with absolute residual ≤40 ms. This midpoint comes from measured
front-sector acquisition times. Exactly 10 images change (488157–488166 use captures
488158–488167); 290 canonical payloads are reused. All 300 selected native BGR images
and original/native XYZI arrays pass exact verification. Maximum residual is
14.739072 ms. Camera world poses intentionally remain original; this isolates image
selection without exposure-pose interpolation or a calibration fit.

Lambda GPU 1 completes all 300 requests with the exact original Docker image,
checkpoint, native configuration/server and four-feature normalization. Its own
container stops afterward; existing work is preserved. Independent comparison
finds all 200 raw proposals bit-exact on each of the 290 unchanged inputs. The 10
changed images affect predictions: all 151 primary boxes scoring ≥0.20 have
GT-independent center correspondences, with maximum changes of 2.375 mm in center,
1.436 mm in dimensions, 0.0244° in heading and 0.0004801 in confidence. Large ordered
weak-proposal differences are ranking changes rather than primary displacement.
The image path responds, but this small intervention does not establish how strongly
the camera branch contributes or resolve optical correspondence.

All five frozen tracker variants reproduce primary precision/recall, identities,
fragmentation, geometric IDF1 and per-ID coverage counts. This holds for all 300
frames (451 geometric FOV observations), the common 299 excluding the original
invalid capture (446 observations), and the declared unchanged 290/changed 10
partitions. Subset scoring uses full-window tracker outputs and never resets tracking
at a subset boundary. Small geometric error changes remain explicit. Detection stays
337 TP/123 FP/114 FN (precision 73.26%, recall 74.72%). Guard stays 385 TP/229 FP/66 FN,
with six switches. Continuation stays 418 TP/243 FP/33 FN (precision 63.24%, recall
92.68%), four switches, three fragmentations and geometric IDF1 0.6763. All 11 geometric
FOV IDs are ever detected, but the stronger per-ID coverage and precision targets
still fail. Production defaults and the canonical benchmark are unchanged; there
is no expansion to all takes.

Frozen inputs, reproduction scripts, execution receipts and 20 paired camera/BEV
PNGs plus two 10-frame videos are in
`artifacts/tracking-take2-front-scan-camera-pairing-inputs-20261011/` and
`artifacts/tracking-take2-camera-pairing-trial-20261011/`. Independent source selection,
wire/raw parity and four-window metric audits are in
`artifacts/tracking-take2-camera-pairing-independent-audit-20261011/`.
The overlays label GT/predicted IDs, misses, FP and association errors. Their
wireframes do not perform depth occlusion. Separate real-map primitive diagnostics
cover scene/asset occlusion. Timing repair does not resolve persistent fixed-scene
projection offsets. Camera calibration provenance and camera-branch influence are
the next concrete checks, with localization held fixed.

### Camera influence and rejected development changes

The camera influence diagnostic uses six fixed, uniformly sampled take2 frames,
each with its original image, a black image and the first sample's image. All 18
requests retain identical LiDAR, camera matrices and XYZ poses. Original predictions
are bit-exact against the paired-camera cache. The camera branch is active: black
and mismatched images substantially change image-neck features, but change camera
BEV features by only 2.20–7.28% and 1.79–3.42% relative RMS. Primary box center changes
are at most 24.3 mm and 50.4 mm, respectively. Camera BEV also consumes unchanged
sparse LiDAR depth; these aggregate ratios do not isolate learned image importance.
Synthetic image trials are diagnostics, not physical detection benchmarks, and
profiling hooks invalidate latency comparisons. Inputs, execution receipts and the
independent source/geometry audit are in
`artifacts/tracking-take2-camera-influence-inputs-20261011/`,
`artifacts/tracking-take2-camera-influence-trial-20261011/` and
`artifacts/tracking-take2-camera-influence-independent-audit-20261011/`.

The native camera voxel pool truncates negative fractional indices toward zero,
retaining ray samples below its intended z=-10 m boundary. A separately frozen
six-sample take1 development diagnostic changes only this operation to floor.
All original proposals reproduce the development cache exactly, and actual geometry
and lifted features match across modes before pooling. Floor removes about 51,577
invalid ray samples (9.06–11.07% of absolute lifted-feature mass), but both modes
score 18 TP/1 FP/2 FN in geometric front FOV. Center RMSE changes only 0.44147 to
0.44120 m. This inherited training behavior is not silently fixed or promoted;
there is no evidence of recovered detection coverage. Sampled frames do not support
tracking metrics. Evidence is in
`artifacts/tracking-camera-voxel-floor-inputs-20261011/` and
`artifacts/tracking-camera-voxel-floor-trial-20261011/`.

A separate tracker prototype permits unmatched assignments with a fixed heuristic
penalty, keeping the existing gates and confirmed low-score continuation. Its full
501-frame take1 development trial recovers no additional GT observations, adds
25 FP (85 to 110), and raises ID switches from two to six. Most new FP duplicate
already matched objects. The prototype is rejected without a take2 trial or parameter
search; production association is unchanged. Reproduction, decision and independent
audit are in `artifacts/tracking-association-abstention-development-20261011/`.

All 18,492 mounted motion-info records have camera matrices exactly matching the
retained replay constants for all three agents. They provide no distortion or
rectification metadata and are not the standard training-info files named by the
checkpoint. The documented `Downloads/tower-params` package is absent under Justin
on both Lambda and Gamma; protected Cisl paths remain uninspected. These findings
are in `artifacts/training-camera-info-audit-20261011/`,
`artifacts/tracking-camera-calibration-provenance-search-20261011/` and
`artifacts/tracking-camera-calibration-remote-search-20261011/`. Physical camera
correspondence remains unresolved. The single take2 benchmark, retained XYZ policy,
acceptance thresholds and production defaults remain unchanged; no all-take
expansion is justified by these diagnostics.

### Crop, sparse support and temporal LiDAR diagnosis

The frozen take2 benchmark is also audited against the actual native camera crop:
1920×1200 →576×360 →crop[32,104,544,360] →512×256. Of 451 geometric front GT
observations, 433 intersect this crop and 18 do not. Of 114 detection misses,
108 are fully inside, three partially inside and three outside. All 70 ID84 and
79 ID92 observations are fully inside. Crop exclusion does not explain the main
coverage failures. The full native-camera benchmark remains unchanged; cropping
is not grounds to exclude a user-visible object.

Original single-scan LiDAR points transformed by archived XYZ yield median five
points inside ID84 boxes at 47–49 m, and median zero for ID92 at 45.6–46.7 m.
All 451 point counts are independently reproduced. Zero support does not prove
occlusion or invalid GT: 44 of 50 matched ID92 detections also have zero nominal
GT-box points. GT is used only for downstream diagnosis. Ten labelled native/crop
PNGs, source hashes and exact support reports are in
`artifacts/tracking-take2-camera-crop-audit-20261011/`; the independent audit is in
`artifacts/tracking-take2-camera-crop-independent-audit-20261011/`.

An independent raw-proposal audit classifies the 114 canonical misses: 56 have a
vehicle proposal within the fixed 2 m gate scoring 0.1–0.2, 33 below 0.1, 23 have
none within 2 m, and two have ≥0.2 proposals failing predicted geometric FOV while
their GT passes. These are candidate diagnostics, not automatically recoverable
one-to-one matches. The raw audit reproduces every original canonical assignment
and does not change thresholds. See `artifacts/tracking-take2-raw-miss-audit-20261011/`.

One frozen temporal-input development trial tests whether additional LiDAR support
helps: current scan plus up to two immediately preceding scans (≤250 ms), transformed
using archived XYZ into current LiDAR coordinates. The current image, calibration,
GT, reference and native model remain fixed. Original intensities are preserved and
normalized once on the server. Six take1 samples compare the original and candidate;
the first has no history and is an exact control. Source ordinal500 retains the
explicit baseline propagated-pose policy. There is no GT-based aggregation, motion
compensation, voxel thinning or threshold search. Moving-object smear and the
different training distribution are declared limitations.

The 12-request trial completes on Lambda. Both variants score 18 TP/2 FN in front
geometric FOV; the candidate gains one GT37 observation but loses one GT77 observation.
FP increases from one to three, and center RMSE rises from 0.44147 to 0.61739 m.
The candidate is rejected without a full development or take2 rerun. These gapped
samples do not support tracking metrics or generalization claims. Frozen inputs,
execution, decision and independent verification are in
`artifacts/tracking-lidar-three-scan-inputs-20261011/`,
`artifacts/tracking-lidar-three-scan-trial-20261011/`,
`artifacts/tracking-lidar-three-scan-independent-audit-20261011/` and
`artifacts/tracking-lidar-three-scan-completed-audit-20261011/`.
No detector/tracker default, localization or acceptance target is changed.

The mounted `cooperscene_motion_infos_val_visible.pkl` is also checked rather than
assuming its filename supplies camera visibility labels. It contains only takes13/14,
not take2; 12,362 instances are removed relative to ordinary validation infos, with
all non-instance fields unchanged. Its instances provide only boxes, class labels
and velocities, with no optical visibility, occlusion, image boxes or point counts.
This undocumented filter cannot certify front-camera GT visibility or replace the
canonical benchmark: 7,810 retained object centers are behind the front camera.
Filtering also leaves the original ID/motion arrays unfiltered in 2,206 rows, so
the file must not be used directly as tracking annotations. Provenance evidence is in
`artifacts/tracking-visible-info-provenance-audit-20261011/`.

### Static camera correspondence and confidence capability

GT-independent native-cloud projections find two plausible reflective sign
correspondences persistently above their RGB footprints. The Iowa sign's six
image-minus-projected footprint-midpoint differences are 94.96–98.36 px; a second
keep-right sign gives 87.44 px. Independent extraction reproduces source hashes,
point indices and arithmetic, while reporting broader-cluster ambiguities in two
frames. These are unsurveyed identity hypotheses and partial sampled footprints,
not exact homologous point errors or a unique calibration solution. No GT, ENU,
model boxes, pose optimization or fitted image shift enter extraction. Native
panels and limitations are in `artifacts/tracking-take2-static-landmark-audit-20261011/`
and `artifacts/tracking-take2-static-landmark-independent-20261011/`.

One labelled nonunique sensitivity probe changes model Kcy by +95 px on six frozen
Take1 development samples. All original controls are bit-exact; source images,
XYZI, extrinsics, archived XYZ, GT and thresholds are unchanged. Canonical evaluation
restores original K after preserving native model-K provenance, so both variants
score identical FOV membership. Both yield 18 TP/1 FP/2 FN front-camera and
84 TP/4 FP/9 FN LiDAR; neither gains or loses a GT observation. Maximum primary
center change is 8.97 mm. Reject without expansion or calibration promotion. Inputs,
execution, K-only evaluation receipts and independent review are in
`artifacts/tracking-camera-cy-sensitivity-{inputs,trial,independent-audit,completed-audit}-20261011/`.
Gapped development samples do not support tracking or generalization claims.

An exact confidence capability audit then rematches all 60,000 immutable Take2 raw
proposals at 59,982 descending unique score events, batching ties and recomputing
maximum-cardinality/minimum-distance Hungarian assignments in affected frames.
Original class mapping, XYZ, symmetric 54 m scope, native camera FOV and inclusive
2 m XYZ gate remain fixed. Canonical score0.2 exactly reproduces 337 TP/123 FP/114 FN.
At ≥90% precision, maximum confidence-curve recall is only 307/451=68.07%
(34 FP/144 FN); all 11 IDs appear, but ID84 has only 1/70 matched observations and
ID92 only 40/79. Retaining every scoped raw proposal still leaves 25 observations
unmatched. Confidence adjustment alone cannot meet simultaneous 90% precision and
90% coverage on this benchmark. No operating threshold is selected or deployed.
Custom all-points precision-envelope AP is 0.840307 for this center-distance/FOV
protocol, not official IoU AP. Figure, full curve, per-ID counts, source hashes and
four full evaluator controls are in `artifacts/tracking-take2-detector-pr-audit-20261011/`.
Optical visibility remains uncertified; these results are conditional geometric
reference comparisons. The one-sequence benchmark and stronger targets remain in force.

### Native LiDAR-only checkpoint comparison

A pinned native single-agent LiDAR-only checkpoint is retrieved and audited before
use (115,390,760 bytes, SHA256
`d71292a772a3f422eb818ab5012a893ee325bffef572524f9c89681eea50f761`).
Its embedded model equals the published and artifact runtime model: epoch30,
298 tensors, one vehicle class, native four-feature points and ±72m preprocessing.
A CPU runtime preflight confirms all state keys/shapes and points-only packing,
once-only intensity scaling, unchanged source points and exclusion of images and
camera metadata from model input. The native palette-only lazy dataset factory
loads no GT. Frozen preparation receipts and later resolved execution policy remain
separate. This is a distinct trained checkpoint comparison, not a controlled causal
camera-branch removal or proof of independent training coverage.

Twelve Lambda requests compare it with the current checkpoint on six fixed Take2
ordinals [0,60,120,180,240,299]. All200 original proposals are bit-exact against the
canonical300-frame cache, including original ordinal299 rather than its later
paired-image diagnostic. Both variants detect the same 7/12 front-FOV observations,
with zero front FP. Front center RMSE worsens 0.72521→0.80162m. In symmetric reference
LiDAR scope, TP/FP/FN worsen 60/4/16→50/9/26; one observation is gained and eleven lost.
Reject without full-sequence expansion or promotion. The retained XYZ/reference,
source images/clouds/timestamps, GT, original evaluation K, score0.2 and matching
thresholds remain fixed. Gapped samples support no tracking metrics.

Matching-pair provenance, frozen inputs, execution, twelve labelled camera/LiDAR
PNGs and independent preflight/completed reviews are in
`artifacts/tracking-native-lidar-only-{checkpoint,inputs,trial,preflight-independent,completed-independent}-20261011/`.
PNG wireframes preserve localized object height and use no ground placement; they
are explicitly labelled without depth occlusion and do not replace primitive-asset
occlusion validation. Camera FOV still does not certify actual optical visibility.
The current detector, tracker and primary one-sequence benchmark remain unchanged.

### Confirmation/continuation tradeoff and replay integration

One fixed exploratory candidate combines already-established two-hit strong
confirmation with0.1 continuation for previously confirmed tracks, primary/birth0.2,
existing duplicate-birth guard and0.5s coast. The three-way cached300-frame Take2
comparison reproduces both prior controls. No detector inputs, primary detections,
XYZ/reference/GT/timestamps/K or matching gates change; no new parameter search is
performed. Compared with one-hit low continuation, front TP/FP/FN change
418/243/33→397/233/54, precision63.24%→63.02%, recall92.68%→88.03%, and geometric
IDF1 .67626→.66050. Switches/fragmentations stay4/3. It loses21 front observations
(GT84 eleven, GT92 nine, GT0 one), with none gained. Wider precision improves
75.62%→79.32%, but75 matched observations are lost. Reject for the front-camera
coverage objective; do not promote based only on wider precision.

Independent raw-only replay reproduces all identities and both-scope metrics.
The candidate's479 weak updates produce no weak births, tentative updates or
confirmations. Native capture-time birth-to-confirm delay has median100.037ms,
p95 815.482ms and maximum1000.192ms; this is observation waiting time rather than
network/inference latency. Frozen protocol, lifecycle events and decision are in
`artifacts/tracking-take2-confirmed-continuation-trial-20261011/`, with separate
`artifacts/tracking-take2-confirmed-continuation-independent-20261011/` review.

The real replay adapter also needed integration changes before optional continuation
could reach rendering: it previously let weak server responses create hidden tracks,
and discarded rendered boxes whenever the latest score fell below the primary
threshold. It now gates births inside the existing tracker, exposes explicit
confirmation/continuation/birth-guard options, and retains confirmed-track prompts
with honest latest confidence. Ordinary primary-filtered responses retain the
same defaults; sub-primary responses no longer create hidden identities. Optional
continuation remains off by default. Source changes are confined to the adapter,
its lifecycle/CLI tests and replay documentation; detector/tracker algorithms,
localization, object heights and rendering ground placement remain unchanged.

A cached public-adapter replay joins image/metadata/LiDAR in three arrival orders
for all300 frames, validating canonical BGR, raw XYZI and calibration before each
cached native response. No GT enters adapter metadata. Tracker outputs and all665
projected prompts reproduce the frozen one-hit continuation run with zero observed
pose difference;203 prompts across163 frames retain an honest latest score below0.2.
These are confirmed-track prompts, not203 new true detections. An independent fresh
run reproduces outputs and receipts byte-for-byte. Evidence is in
`artifacts/tracking-take2-offline-adapter-parity{,-independent}-20261011/`.
This test uses area16 rather than CLI default64, full native range rather than the
XY54m evaluation scope, and serial cached responses. It establishes neither NN
accuracy, asynchronous drop/latency behavior nor actual optical visibility.
Adapter/tracker/CooperScene/Foxglove replay regression checks pass:89 tests locally.
The stronger front-camera coverage and precision targets remain unmet.

### Delayed results through the actual offline publisher

A fixed cached Take2/V1 trial now uses actual `RosFrameAdapter.receive_metadata`
publication, following Foxglove replay's image→LiDAR→metadata order. Published
boxes are sampled before submitting current metadata, unlike the earlier serial
post-join parity test. Three prespecified virtual service delays50/150/300ms retain
all300 camera evaluation frames, original capture times, raw bytes, calibration,
XYZ/reference and GT; GT never enters adapter metadata. Delays are simulations,
not measured GPU/network latency or throughput. Detector fields remain the frozen
current-frame reference and must not be presented as newly delivered detections.

| Virtual service delay | Served / replaced requests after tail polling | Front TP / FP / FN | Track precision / recall | Geometric IDF1 | Switches / fragments |
| --- | --- | --- | --- | --- | --- |
|50ms|300 /0|410 /243 /41|62.79% /90.91%|.66848|4 /5|
|150ms|151 /149|343 /252 /108|57.65% /76.05%|.57744|3 /8|
|300ms|88 /212|95 /74 /356|56.21% /21.06%|.26452|4 /78|

Latest-result capture age at publication has median100/300/600ms, including public
poll timing and time waiting for submission. At300ms,212 camera frames have empty
prompts as observations age beyond0.5s coast. Tail polling needs1/2/2 calls and
adds no evaluated camera frames or rewritten durable snapshots. The stronger
coverage/precision targets remain unmet; no coast, score or queue defaults change.
Frozen evidence is in `artifacts/tracking-take2-delayed-adapter-trial-20261011/`.
Its execution receipt binds the pre-fix adapter SHA to both execution start and end.

A separate original-K projection sensitivity check confirms that area16 and the
CLI default64 retain identical full-range counts in this window:451 GT,460 primary
detections and665 serial low-continuation track prompts, with zero removals.
Evidence is in `artifacts/tracking-take2-adapter-area-audit-20261011/`; this does not
certify optical visibility or other sequences.

An independent recount verifies delayed matches, switches, fragmentation,
geometric IDF1, capture clocks, coast limits and pending/drain accounting in
`artifacts/tracking-take2-delayed-adapter-independent-20261011/`. A labelled plot
is in `artifacts/tracking-take2-delayed-adapter-visuals-20261011/`.

The production idle loop now polls ready BEVFusion results after MCAP input ends,
so the latest pending request can execute while services remain active. This is
an EOF lifecycle fix rather than a detection-quality improvement or lossless mode.
A real-main mocked replay regression verifies late response processing, unchanged
capture timestamps and exactly the original durable snapshots. Adapter, tracker,
CooperScene and Foxglove replay checks pass:90 tests locally. The delayed benchmark
above exercises the pre-fix source recorded in its execution receipt; it does not
claim a full post-fix NN or asynchronous-performance evaluation.

### Native BEV reflection diagnostic

A single fixed horizontal reflection follows native training augmentation:
normalized LiDAR Y negates and `lidar_aug_matrix=diag(1,-1,1,1)` left-composes;
RGB, original K and physical extrinsics remain unchanged. Native single-pipeline
prediction does not undo augmentation, so raw center Y and yaw are inversely mapped
before normal sidecar length-axis normalization and physical world composition.
Box rotations stay proper; reflection is never used as a world pose. Synthetic
cuboid/projection and actual native CPU packing checks pass. Directed vehicle
front/back is not certified by the cuboid/modulo180-degree heading protocol.

The six prespecified Take2/V1 ordinals0/60/120/180/240/299 reuse the canonical
payloads, XYZ/reference, timestamps and GT. All200 proposals of each original NN
control match canonical outputs exactly before reflected requests are permitted.
Twelve serial requests and fixed detection evaluation complete successfully.
Front TP/FP/FN change7/0/5→7/3/5: recall58.33% unchanged, precision100%→70%,
center RMSE .72521→.83813m. No original front GT observation is lost or gained;
GT84,92 and96 remain undetected in these samples. Wider54m TP/FP/FN change
60/4/16→57/3/19, losing IDs44/45 at487927 and52 at488107 with no gains.
False positives here mean unmatched supplied annotations; they do not certify
that an apparent physical vehicle is nonexistent. Optical visibility remains
unverified. The frozen expansion rule fails, so reject without full300 inference,
ensemble promotion or any detector/tracker/localization/default changes.

Independent GT-free comparison finds57 primary proposal pairs within2m among
64 original and60 reflected predictions, with7 original/3 reflected unpaired.
Raw augmented↔physical inversion, normalized point hashes, original controls and
independent fixed-GT metric recount all pass. Six gapped frames support no tracking
metrics. Twelve labelled camera/LiDAR wireframe PNGs preserve localized heights
and use no ground placement; their explicit no-depth-occlusion labels distinguish
them from the separately verified primitive-asset compositor.
Evidence: `artifacts/tracking-take2-bev-reflection-{inputs,trial}-20261011/` and
`artifacts/tracking-take2-reflection-{preflight-independent,independent}-20261011/`.

Instrumented original submit-to-reply times after the startup request span
93.63–116.04ms; reflected requests span110.21–123.57ms. The first original request
waits7.139s while server initialization/warmup precedes endpoint availability;
its server processing is100.87ms. Reflected point-copy/hash/extra raw writing adds
diagnostic overhead. These serial samples are neither10Hz throughput evidence
nor a production latency comparison, and no replay rate is selected from them.
They justify measuring real queue delivery and startup readiness on the same
fixed sequence before relying on simulated service delays. The owned probe
container is stopped; inference exit0 is distinct from explicit-stop exit137.

### Native delivery control stop and startup readiness

The fixed300-frame Take2/V1 actual-client delivery trial stopped at its frozen
exact-output gate. Of153 served camera requests,152 reproduced all200 canonical
proposals exactly; ordinal284/frame488151 differed by at most6.42mm in gravity
center and0.000119 in confidence. LiDAR wire bytes and recorded physical
calibration metadata match. Independent review confirms unchanged proposal
membership at scores0.1/0.2, but the cause remains unproven. No tolerance was
introduced retrospectively, no delivery metrics are reported, and the planned
0.5× condition was not launched. All300 camera callbacks and the EOF drain ran,
but publication traces were lost because control validation preceded their
serialization. A future attempt must preserve unvalidated traces before the
gate while withholding metrics on failure. Evidence is in
`artifacts/tracking-take2-real-delivery-{trial,independent}-20261011/`.

A separate CPU-only Docker probe demonstrated that a published TCP port can
accept connections before the backend ZeroMQ socket binds. The launcher now
requires `EVENT_HANDSHAKE_SUCCEEDED` using a temporary DEALER monitor, without
sending an inference request. The existing container check and overall startup
deadline remain in effect. All19 launcher tests pass, including rejection of a
TCP-only listener and acceptance of a real ROUTER with no application request.
Evidence is in `artifacts/tracking-bevfusion-readiness-audit-20261011/`.
This fixes startup sequencing; it does not establish detector quality or queue
performance. Further evaluation remains confined to the same Take2/V1 window;
the stronger front-camera coverage and precision targets remain unmet.

### Same-input native output repeatability

Eight prespecified serial requests alternate ordinals282/284 four times each.
Actual decoded image/XYZI bytes, typed packed tensors and camera/augmentation
metadata are identical across same-frame repeats. All sidecar responses exactly
normalize their own native outputs. Ordinal282 matches the archived200 proposals
on all four repeats; ordinal284 matches once and produces a second mutually
identical variant three times. The second variant also matches the prior failed
delivery batch. Its maximum center/box-tensor/score differences remain
6.42mm/0.022686/0.000119, with no class or0.1/0.2 membership changes observed.
Independent inspection verifies source/model pins and all stored byte/tensor
copies. Evidence is in
`artifacts/tracking-take2-delivery-control-repeat{,-independent}-20261011/`.

This demonstrates output variation downstream of equal observed packed inputs,
which precede the model data preprocessor. It does not identify a particular
kernel, establish universal numerical bounds or certify determinism. The frozen
repeat gate fails as declared; all forensic traces survive. The earlier300-frame
attempt remains failed and unscored. A new delivery protocol will retain exact
source/model/wire controls, persist traces before validation and report native
output variation separately, evaluating actual outputs rather than requiring
universal bitwise equality to cached proposals. No accuracy target, matching gate,
localization policy or production default changes follow from this diagnostic.

### Actual client delivery on the fixed Take2/V1 window

A new prespecified retry passed exact source, model, decoded wire and lifecycle
controls at both wall replay rates. All300 captures, archived XYZ poses, camera
calibration, GT, confidence thresholds and association gates remain fixed.
The instrumented native client uses the real adapter/tracker with an in-memory
publisher; this is not live ROS/network/compositor throughput. Unvalidated traces
are persisted before gates. Native proposal drift is descriptive; cached outputs
are a separate reference. Independent reconstruction agrees on all seven metric
views and per-ID counts.

| Wall rate | Served captures | Published precision | Published recall | Geometric IDF1 | ID switches | Fragmentations |
|---|---:|---:|---:|---:|---:|---:|
| 1.0 | 151/300 | 57.65% | 76.05% | 0.5774 | 3 | 8 |
| 0.5 | 300/300 | 62.79% | 90.91% | 0.6685 | 4 | 5 |

All11 geometric front-FOV IDs match at least once. Neither rate meets90% precision
and90% coverage for each eligible ID. GT84 published coverage is4/70 versus45/70;
GT92 is61/79 versus70/79. At rate0.5 the fresh detector still matches only7/70 for
GT84 and50/79 for GT92. Source-capture detector precision/recall is73.39%/75.00%
over151 served captures versus73.26%/74.72% over300, with different exposure.
Matching remains class-aware inclusive2m XYZ, maximum cardinality then minimum
distance, symmetric reference XY54 and original geometric camera FOV with16px
minimum projected area. IDF1 uses this center association, not standard IoU HOTA.
Optical visibility remains unverified: projected boxes visibly float above physical
cars, so geometric FOV alone cannot certify camera visibility.

Two rate1.0 results and one rate0.5 result arrive after the final publication;
they are scored only at their original captures, never as extra camera frames.
Median latest-result source age at publication is300.367ms versus99.997ms.
Held primary boxes at current camera captures have recall20.84% versus66.08%,
preserving original result world poses. Slower replay improves availability but
does not establish detector accuracy or meet the user target.

Three rate0.5 requests differ from cached outputs. Proposal row permutations
explain large index-wise center differences; GT-free geometric correspondence
shows maximum primary-center differences below0.9mm for these requests. Selected
boxes are not bitwise identical; these observations are not universal drift bounds
or accuracy tolerances. Raw proposals and exact wire receipts survive. The earlier
failed delivery trial remains failed and unscored.

Evidence: `artifacts/tracking-take2-real-delivery-retry-20261011/`,
`artifacts/tracking-take2-real-delivery-retry-evaluation-20261011/` and
`artifacts/tracking-take2-real-delivery-retry-completed-independent-20261011/`.
Evaluator `render-rate-{1.0,0.5}/` directories contain labelled300-frame10FPS
source-timeline videos and camera/LiDAR PNGs showing GT/predicted IDs, misses,
false positives and ID switches. Wireframes explicitly have no depth occlusion;
localized height is preserved without ground placement. Primitive compositor
occlusion evidence is separate. Work remains on this same sequence.

A separate Gamma primitive render uses the actual published rate0.5 frame299,
original XYZ poses and no ground placement. Asset47 at45.69m projects307 pixels:
72 are hidden by Gaussian scene depth and235 by nearer assets, leaving zero
visible. This verifies shared asset/scene depth buffering for this distant case,
not optical camera calibration. The existing CloudXR processes remain untouched.
Evidence: `artifacts/tracking-take2-actual-delivery-gamma-primitives-20261011/`.

### Production wait polling: actual full-rate rerun

The scheduled-wait fix services the nonblocking detector poll between capture
callbacks, using sleeps of at most10ms during positive waits. Both MCAP paths
share the helper. Late captures remain immediate, no-detector pacing retains its
single sleep, and EOF/publication/timestamp policies are unchanged. All109 relevant
adapter, tracker and MCAP tests pass locally and on both isolated hosts. Blocking
callbacks still delay service;10ms is a requested sleep bound, not real-time QoS.

A separately frozen one-condition actual rerun calls the production helper on
the same300 predecoded Take2/V1 captures at wallrate1.0. Source/model/wire/own
native-to-sidecar and lifecycle controls pass, with independent seven-view recount.
The original capture epoch, XYZ/start policy, GT and thresholds remain fixed.

| Actual condition | Served captures | Published precision | Published recall | Geometric IDF1 | Switches | Fragmentations |
|---|---:|---:|---:|---:|---:|---:|
| Original1.0 | 151/300 | 57.65% | 76.05% | 0.5774 | 3 | 8 |
| Wait polling1.0 | 251/300 | 62.16% | 86.70% | 0.6481 | 5 | 10 |
| Original0.5 | 300/300 | 62.79% | 90.91% | 0.6685 | 4 | 5 |

Candidate published GT84 coverage is36/70 and GT92 is67/79. All11 IDs match at
least once, but precision, per-ID90% coverage and at-most-one-switch per100 GT
observations remain unmet. Detector source-capture precision/recall is73.91%/
74.10% on251 served captures (390 GT observations):289TP,102FP,101FN. Matched
center RMSE is0.831m for detections and0.951m for published tracks. This is a
served subset, not the identical exposure of the original151 or300 results.
Held latest primary recall at current captures is44.79%, separate from tracking.
Median latest-result source age is200.084ms versus300.367ms originally. Two
candidate responses arrive after the final publication; no synthetic frames are
added to track coverage.

250 native batches match the cached proposals bitwise; one differs. At source
ordinal233, one weak proposal crosses the0.1 continuation threshold (actual
0.100710 versus cached0.098854); primary0.2 membership is unchanged. Thus tracker
inputs are not perfectly identical between executions. The observed delivery and
coverage gains are not a replicated causal estimate or a universal NN determinism
claim. The paired model-free simulation supports earlier dispatch but models no
CPU/network/OS costs. ID switches and fragmentation increase in the actual rerun;
there is no tracker-parameter or detector-quality promotion from these results.

Evidence: `artifacts/tracking-take2-scheduled-wait-native-trial-20261011/`,
`artifacts/tracking-take2-scheduled-wait-native-evaluation-20261011/` and the
corresponding prelaunch/completed independent audits. The new evaluator includes
one labelled300-frame10FPS source-timeline video and eight camera/LiDAR PNGs.
Predecoded/mock-publication, calibration uncertainty and no wireframe depth
occlusion remain explicit. Other sequences remain on hold; optical calibration
and the GT84/92 detection/identity failures still require work.

### Same-sequence miss and association diagnosis

The requested minimum remains: every front-visible GT ID must be detected at
least once. These Take2 trials use stronger frozen promotion targets:90%
coverage for every ID present in at least five frames,90% precision and at most
one ID switch per100 GT observations before expanding beyond this sequence. Current
counts use geometric camera FOV; unresolved physical calibration prevents treating
them as certified optical visibility. All11 geometric IDs are detected at least
once, but sustained coverage and precision still fail.

The source-capture miss audit finds no primary match changes across130 paired
GT84/92 observations between the wait-poll run and the full300 slower run. In the
latter, GT84 has7/70 primary detections:35 misses have a nearby0.1–0.2 proposal,
19 a proposal below0.1 and nine no retained proposal within2m. GT92 has50/79:
eight weak,13 below0.1 and eight absent nearby proposals. Raw200 proposals have
already passed model-head filtering. Nearby low-confidence geometry is not a
valid detection. Sparse distant LiDAR support and roughly0.5m conditional vertical
discrepancies are diagnostic evidence, not justification to change localization,
height, calibration or confidence gates. Camera crops still show uncertain
physical correspondence. Evidence:
`artifacts/tracking-take2-detector-miss-mechanisms-20261011/`.

Exact reconstruction of the wait-poll publications identifies weak updates with
3–4m innovations that impart large velocities to established tracks. Subsequent
coasting and overlapping2m GT association neighborhoods contribute to switches;
birth/death alone does not explain them. A single artifact-only CPU probe retains
predicted velocity during weak position correction, using the corresponding
restricted-gain Joseph covariance update. Strong updates, inputs, clocks,
thresholds and gate code remain fixed. Independent recount confirms:

| Recorded timeline | Baseline TP/FP/FN | Probe TP/FP/FN | Baseline→probe IDF1 | Switches | GT84 coverage |
|---|---:|---:|---:|---:|---:|
| Wait polling1.0 |391/238/60|400/229/51|0.6481→0.6667|5→4|36→43/70|
| Original0.5 |410/243/41|413/240/38|0.6685→0.6739|4→4|45→47/70|

These are counterfactual projected states from recorded detector responses,
not fresh detector runs or actual new adapter publications. Candidate center
RMSE worsens0.9514→0.9676m and one GT92 observation is lost; slower-run RMSE
improves0.9084→0.9001m. Both variants fail precision and per-ID coverage targets.
The probe is rejected for promotion; production tracking remains unchanged.
Evidence: `artifacts/tracking-take2-candidate-association-failure-audit-20261011/`,
`artifacts/tracking-take2-weak-position-only-probe-20261011/` and its separate
prelaunch/completed independent audits.

A read-only auxiliary-input audit finds qualified V2 XYZ available only from
primary ordinal107; V3 has no qualified pose within this pilot. Latest-past V2
headers precede the primary by about57ms, but original per-point scan times show
the four fixed V2 scans complete about42ms after the primary header and57ms
before primary scan completion. This supports a complete-scan diagnostic policy,
not availability at the primary header instant or hardware synchronization.
Future best poses are excluded. Raw recipe sensor composition and rejected-frame
propagation must remain explicit; released ICP poses cannot receive G directly.
The retained search explicitly passes the assumed0.03m rotated antenna offset
for every agent; the geodetic API's default is0. V1-only axis/mount corrections
are separate. The frozen feasibility report's V1-only antenna statement is
incorrect and is superseded by the auxiliary preflight composition audit.
Cross-agent concatenation changes the native model input distribution and has
not been evaluated or promoted. Evidence:
`artifacts/tracking-take2-cooperative-input-feasibility-20261011/` and
`artifacts/tracking-take2-auxiliary-scan-timing-audit-20261011/`.
The CPU-only V2 preflight prepares four fixed pairs at ordinals120,180,240,299,
preserving the original V1 point prefix, camera and raw intensities. All193
sequential V2 raw-recipe references match the archive to1.43e-14; rejected XYZ
frames use the retained propagation policy. Peer points are transformed by
inverse(primary XYZ pose) times peer XYZ pose. There is no new GT masking,
deskew, dynamic-object time compensation or NN run. Point/voxel budgets are CPU
estimates, not GPU latency/memory validation. Prepared inputs and provenance:
`artifacts/tracking-take2-auxiliary-v2-input-preflight-20261011/`.

### Fixed auxiliary-LiDAR detector comparison

Eight fresh native requests compare original V1 inputs against V1 plus transformed
V2 points at the four preselected ordinals120/180/240/299. Camera, capture poses,
XYZ localization, checkpoint, raw intensity handling and thresholds remain fixed.
GT is used only downstream for evaluation; no tracker runs on these four gapped
samples. Association remains class-aware inclusive2m XYZ, maximum cardinality
then minimum distance, with the original reference XY±54m and camera projection.
The native single-camera model was not trained here for concatenated peer clouds;
this diagnostic changes its point distribution, without deskew or dynamic-object
time compensation. Fresh original controls are the comparator, not cached results.

| Primary score≥0.2 scope | Original TP/FP/FN | Combined TP/FP/FN | Precision | Recall | Center RMSE |
|---|---:|---:|---:|---:|---:|
| Geometric front |6/0/5|6/2/5|100%→75%|54.55%→54.55%|0.783→1.046m|
| Reference XY±54m |46/3/14|45/7/15|93.88%→86.54%|76.67%→75%|0.651→0.719m|

The combined input gains GT84 at299 (score0.190524→0.200349, crossing the fixed
0.2 threshold) but loses front GT48 at120 (0.293851→0.154865). Across the larger
scope it also gains GT106 at299 and loses GT88 at180/240. GT92 remains unmatched
in both annotated samples. The diagnostic score≥0.1 view has unchanged front
matches8/11 but FP2→5; broader matches remain50/60 with FP11→14. It does not
justify lowering the primary threshold. Dimensions and heading errors also
worsen. Independently recounted results fail the frozen no-loss and precision
gates: **stop this variant; no larger auxiliary NN window or threshold search**.
Neither production inputs nor tracking parameters are promoted.

The optional GT-box point audit finds no added peer points inside GT84 at240 or
299, despite its299 match gain. GT92 receives eight and three exact-box peer
points respectively without gaining a match. These source-derived counts do not
establish optical visibility or a causal explanation. GT84/92 annotations are
absent at120/180; no missing boxes are invented or false positives discarded.
All16 camera/LiDAR PNGs preserve localized height. The projected boxes still
float above physical cars, so geometric front eligibility cannot certify the
user's optical-visibility requirement. Prediction labels are frame-local indices,
not persistent IDs; these samples provide no IDF1 or ID-switch measurement.

Two earlier executions are frozen **failed/unscored**. The first exposed a
tuple-versus-JSON-list receipt bug; the second exposed host OpenBLAS versus pinned
runtime MKL differences in float32 calibration reconstruction. A CPU-only audit
inside the unchanged pinned runtime reproduces all64 stored matrices exactly.
Before the final fresh execution, source-derived calibration expectations were
frozen and exact dtype, shape, array and byte-hash checks replaced the incorrect
host reconstruction. A rehashed one-ULP corruption is rejected; no tolerance,
camera calibration or localization correction was changed. The final eight
requests pass source, wire, own raw-proposal normalization and typed packed-input
checks. Owned-container cleanup, no OOM, free GPU/port and source preservation
are verified. Existing queues, original repositories and mounted data are intact.

Evidence: `artifacts/tracking-take2-auxiliary-v2-native-canonical-20261011/`,
`artifacts/tracking-take2-auxiliary-v2-native-evaluation-20261011/`,
`artifacts/tracking-take2-auxiliary-v2-calibration-verifier-audit-20261011/` and the
canonical prelaunch/completed independent audits. This is one exploratory
four-pair experiment, not a replicated detector-effect estimate, tracking pass,
independent localization accuracy result or reason to expand to other takes.

### Prediction point support and camera evidence

A frozen sensor-only pass counts LiDAR points inside all 3,705 native prediction
cuboids from the actual 251 served captures. Raw response, source NPZ and wire
receipt hashes are checked before counting; GT is loaded only after the count
file is frozen. An independent full recount reproduces every count and the
subsequent fixed-threshold assignments. Exact boxes and boxes padded by 0.25 m
are counted separately, in full raw clouds and the native strict model range
(-72,-72,-5) < XYZ < (72,72,3). These are pre-voxel counts, not learned feature
contributions. Both cloud selections give identical counts for these boxes.

At score >=0.2, source-capture geometric-front detections remain 289 TP, 102 FP,
101 FN: precision 73.91%, recall 74.10%, matched center RMSE 0.831m. Exact-box
support medians are 85 for TP and 5.5 for FP, but distant 40–54m TP span 1–82 points
and FP span 0–25. In the wider 54m scope, 283 matched TP and 44 FP have zero exact
support. A point minimum would remove valid geometric matches and can retain
erroneous boxes supported by other surfaces. **No point-count filter, threshold
search or tracker change is promoted.** Three front FP match GT in the wider
scope but fail GT camera-projection eligibility, illustrating another limitation
of treating geometric-front FP as proven false physical objects.

The prespecified static-pole search yields ambiguous facade candidates, not a
certified camera/LiDAR correspondence; no reprojection correction is fitted.
Bounded read-only recovery of a malformed February2026 calibration SQLite bag
recovers one complete 1920x1200 RGB checkerboard image and its adjacent LiDAR
message, reading 19.53MB of source pages. Their sensor headers differ 32.24ms.
Metric board dimensions, motion, target correspondence, and linkage to the
September 2025 CooperScene camera/lens/rig remain unverified. This is diagnostic
evidence, not a replacement intrinsic/extrinsic calibration. Mounted data and
localized object heights remain unchanged.

Evidence: `artifacts/tracking-take2-prediction-point-support-20261011/`, its
prelaunch/completed independent audits,
`artifacts/tracking-native-range-boundary-source-20261011/`,
`artifacts/tracking-take2-static-pole-correspondence-20261011/`,
`artifacts/tracking-camera-calibration-structural-recovery-20261011/`,
`artifacts/tracking-camera-calibration-btree-prefix-20261011/` and
`artifacts/tracking-camera-calibration-single-pair-recovery-20261011/`.

### Native versus refined GT coverage on the same 300 frames

The preparation step filters native GT to reference LiDAR XY+/-54m, whereas
the native model point range extends to 72m. A read-only audit of all supplied
native annotations at 487867–488166 finds 3,825 observations, 39 IDs, and no
annotation outside 54m (maximum coordinate magnitude 50.333m). Every retained
box matches the canonical benchmark exactly. Thus the preparation filter loses
no supplied native annotation here; it does not prove the export is complete.

The released `250928/2/1/labels_refined` export contains 5,875 observations for
these identical frame names. Only 3,128 observations join by textual ID;
2,747 occur only in refined and 697 only in native. ID sets differ in all 300
frames. Of the refined-only observations, 1,639 are within 54m, 389 between 54–72m,
and 719 beyond 72m; 236, 274 and 567 respectively pass the original geometric-front
projection. These counts are source discrepancies, not additional certified
visible GT or boxes added to scoring.

Independent manual reconstruction verifies native location+R*center/2*extent,
refined ENU quaternion wxyz/LWH, all 600 YAML hashes and the frozen reference
mapping. Of the 3,128 joins, 2,689 geometries agree at the declared tolerances;
80 additional joins agree when 180-degree heading reversals are treated as
equivalent, and 359 still differ. ID29 at 488034 has a 14.769m center difference
and 43.846-degree heading difference, despite a zero native center offset and
only 0.030m maximum dimension difference. This cannot be explained by a harmless
heading reversal or omitted center composition. Neither export is silently
preferred, combined, relabelled or used as tracker input. Existing scores remain
valid only for their declared native-export benchmark; the all-visible-GT
condition remains unestablished pending export/identity provenance.

The retained SensorPlatform exporter source provides a possible scope mechanism:
`late_fusion/gen_local_labels.py` selects annotations by ego provenance or planar
distance <=50m, using `VEHICLE_LIDAR_RANGE`. The inspected OPV2V exporter selects
`LABEL_FOLDER='labels_tracked'`, rather than `labels_refined`, and preserves
IDs/ENU geometry without an additional 50m filter.
This source inspection does not identify the exact version that generated the
mounted native export; range selection also cannot explain the changed geometry
of shared IDs. A future refined-export comparison must therefore be declared
separately, with its own continuity audit, rather than replacing historical GT.

The same 300 native/release metadata rows and clocks agree exactly. All 5,875
refined ENU centers transformed by inverse native LiDAR pose agree with their
own stored LiDAR centers to 9.75e-13m. Thus this transfer is supported without
changing localization. Neither export triggers the declared adjacent-appearance
jump gate max(10m, 50m/s*dt+2m); absence of such jumps does not prove physical
identity continuity or resolve differences between exports.
Historical local-quaternion variable names conflict with SciPy's xyzw ordering;
the audited transfer uses the declared ENU wxyz quaternion and native ego pose,
rather than assuming that stored local quaternion tuple has the same ordering.

Evidence: `artifacts/tracking-take2-range-coverage-audit-20261011/`,
`artifacts/tracking-take2-native-refined-gt-coverage-20261011/` and
`artifacts/tracking-take2-native-refined-gt-coverage-independent-20261011/` and
`artifacts/tracking-take2-gt-export-provenance-audit-20261011/`.

### GT-source sensitivity with unchanged detections and published tracks

Two preregistered CPU comparisons use the same 251 actually served primary
detection captures and 300 actually published tracking snapshots. Original
native scores reproduce exactly before refined scoring. The first comparison
keeps reference LiDAR XY+/-54m; the second reconstructs unfiltered source traces
and verifies that their 54m subsets reproduce the original frames exactly.
Neither runs inference or tracking. Refined ENU wxyz poses pass through inverse
native ego pose and retained reference pose; GT remains evaluation-only.

| Geometric-front view / GT scope | TP / FP / FN | Precision / recall | Center RMSE | Geometric IDF1 / switches / fragments |
|---|---:|---:|---:|---:|
| Served detections / native54 |289 / 102 / 101|73.91% / 74.10%|0.831m|Not tracking|
| Served detections / refined54 |283 / 108 / 214|72.38% / 56.94%|0.834m|Not tracking|
| Served detections / refined full front |283 / 108 / 912|72.38% / 23.68%|0.834m|Not tracking|
| Published tracks / native54 |391 / 238 / 60|62.16% / 86.70%|0.951m|0.6481 / 5 / 10|
| Published tracks / refined54 |399 / 230 / 177|63.43% / 69.27%|0.972m|0.6390 / 5 / 16|
| Published tracks / refined full front |404 / 229 / 1013|63.82% / 28.51%|0.970m|0.3805 / 5 / 16|

Full-front refined GT has 30 IDs, with 17 never matched in either view. Among
IDs exposed for at least five frames, 15 of 28 are never detected and 16 of 29
are never tracked. Thus even the minimum geometric coverage condition fails;
this is not proof that every unmatched annotation is optically visible. Within
54m the never-matched refined IDs are 71,75,100,103,105. Each export retains its
own IDs; no cross-export identity mapping, union, exclusion or GT extension is
introduced. Refined IDs71/85/95 have 41-frame annotation gaps; permissive jump
checks find no flagged jumps, without establishing identity through gaps.

Global matching precedes range stratification, so a prediction cannot be reused
across bands. Published refined GT/match counts are 576/399 within54m,
274/5 at54–72m, and 567/0 beyond72m. Served counts are 497/283,229/0,469/0.
Bands use max(|x|,|y|) in reference LiDAR coordinates, not Euclidean radius.
All 2,955 source detections were already within54m; unfiltered publication adds
four prompts, yielding 633. The model's configured point range is a separate
input constraint. Detection captures include two results received after the
last publication; these are not 300 fresh camera-time detections. Published
prompts retain actual weak-confidence continuations and capture-derived poses.

Both comparisons receive independent complete recounts, including source hashes,
manual quaternion transforms, assignments, per-ID coverage, geometry errors and
identity metrics. No detector/tracker improvement follows from changing GT
source. Precision and per-ID recall gates fail; other sequences remain on hold.

Separate 54m and full-front paired videos each contain 300 frames at 10FPS,
with 12 PNG comparisons per scope. Native/refined panels use identical camera,
LiDAR and predictions, with persistent track IDs distinguished from frame-local
proposal indices. Full-front BEV extent 180m is display-only and contains all
eligible GT boxes; camera/evaluation boxes are never cropped to that extent.
Five additional postscore PNGs inspect the missed refined IDs. At150, near
GT71 projects across curb/road/sky without a confidently identifiable vehicle;
other boxes remain above physical cars. These observations preserve all scored
annotations and identify unresolved correspondence, not invalid GT or a height
correction. All original heights, K, poses, thresholds and source bytes remain
unchanged. Wireframes do not certify optical visibility or primitive occlusion;
the earlier unchanged primitive-depth test remains separate evidence.

Evidence: `artifacts/tracking-take2-refined-gt-sensitivity-20261011/`,
`artifacts/tracking-take2-refined-gt-full-front-sensitivity-20261011/`, their
prelaunch/completed independent audits and rendering roots, and
`artifacts/tracking-take2-refined-missed-track-review-20261011/`.


### Missed-GT provenance and recorded LiDAR support

A bounded source audit traces the same five missed refined IDs to the mounted
original and corrected-orientation editor JSON and global tracked labels. Their
local refined geometry agrees with corrected editor PSR and global tracked
records. Editor `Car` is not a detailed physical category: the inspected
historical forward converter hardcodes it, while the reverse converter preserves
`obj_type`. That historical forward script is restricted to another take/frame
window, so it does not prove the invocation that produced these Take2 files.
At frame488017, the global tracked ID71 record has type4 (`trailer` in the
historical class list),
but across its 78 source appearances its type changes among 0/4/2. Its recorded
score is zero; score semantics and human review are not established. No origin
agent field or verified ego-vehicle identity was found. These records therefore
cannot establish that GT71 is physically a trailer, invalid, or optically visible.

A separate fixed five-frame diagnostic uses the full oriented 3D cuboids, raw
cached LiDAR and unchanged localized poses. Inclusive half-LWH containment has
no padding, height shift or source filtering. Exact in-box counts are zero for
GT71@150, GT100@265, GT103@270 and GT105@290; GT75@174 contains 34 points in
31 fixed 0.25m voxels. Projection overlap in BEV/side views is not 3D support.
Neither an empty cuboid nor a nonempty one certifies optical visibility, object
semantics or annotation validity. Ten PNGs show original camera projection and
LiDAR context. GT105@290 retains the previously documented invalid image clock
(`image_ns=0`) and duplicated preceding image; its camera panel cannot certify
LiDAR-time correspondence. Its LiDAR count remains valid. Historical scores and
all replay rows remain unchanged. All source hashes/arrays are preserved; no detector, tracker,
localization, calibration, score or GT exclusion changes follow from this audit.
GT review/version provenance remains necessary before these misses can support
an optical-coverage claim or a justified detector correction.

Evidence: `artifacts/tracking-take2-missed-gt-ontology-provenance-20261011/`,
`artifacts/tracking-take2-missed-gt-lidar-support-20261011/`, and
`artifacts/tracking-take2-missed-gt-diagnostics-independent-20261011/`.


### Four-capture benchmark-extrinsics detector check, 2026-10-12

The baseline LiDAR–camera extrinsic remains in ROS-converted MCAP replay.
The benchmark-adjusted extrinsic is a separate detector-input probe; the user
prefers deferring a replay-calibration switch until tracking is improved, unless
BEVFusion detections become substantially better. No switch follows from this
four-capture result.

Capture ordinals120/180/240/299 were selected before outcomes. Eight actual native
inference requests use identical images, XYZI, intrinsics, timestamps, localized
LiDAR poses, checkpoint and Docker image. Only candidate camera pose changes to
the mounted benchmark `scene2_agent1` extrinsic. Both variants are evaluated and
rendered with the original K/T, GT reference, inclusive2m matching and clipped
projected-bbox area16px²; partly off-screen boxes qualify. No GT enters inference.

| Native evaluation scope | Original TP / FP / FN | Adjusted TP / FP / FN | Original / adjusted center RMSE |
|---|---:|---:|---:|
| Projected front camera |6 / 0 / 5|6 / 0 / 5|0.782853 / 0.783740m|
| Reference LiDAR XY square54m |46 / 3 / 14|46 / 3 / 14|0.650949 / 0.650825m|

Fresh baseline output reproduces all200 raw proposals for each archived capture
exactly, and historical counts and GT/proposal pairs pass before candidate
scoring. Candidate native matched GT observations are unchanged, with zero gains
or losses. The separately labelled refined full-front sensitivity is also
unchanged at6TP/0FP/19FN. Small mixed center-error changes do not establish a
benefit. The predeclared front-TP gain requirement fails: no capture expansion,
threshold/calibration search or production promotion follows from this probe.
Four gapped frames support detection comparison, not tracking ID metrics.

Actual inference/source/wire/calibration-packing controls and independent complete
recounts pass. All eight responses were retained before validation; the owned
container stopped and existing repositories/processes were preserved. The
backend records one internal initialization warmup and eight requested forwards,
with no application prime request. Eight paired PNGs show native/refined GT,
misses and frame-local proposal IDs. Original heights are preserved; no SAM,
ground placement or optical/depth-occlusion certification is involved.

A subsequent read-only audit of the unchanged300-frame tracking baseline finds
that its five switches include expiry/rebirth, publication coast expiry, recovery
onto another track and two evaluation reassociations between live tracks. Weak
corrections at source258/275 inject large velocity changes. This identifies a
motion-estimation issue to investigate; it does not justify rerunning rejected
confirmation, position-only or abstention variants. Tracker defaults remain
unchanged while a separate confidence-noise prototype is prepared for review.

Evidence: `artifacts/tracking-take2-benchmark-extrinsics-probe-plan-20261012/`,
`artifacts/tracking-take2-benchmark-extrinsics-native-trial-20261012/nn-execution/`,
`artifacts/tracking-take2-benchmark-extrinsics-native-evaluation-20261012/`,
`artifacts/tracking-take2-benchmark-extrinsics-completed-independent-20261012/`,
`artifacts/tracking-take2-benchmark-extrinsics-native-rendering-20261012/`, and
`artifacts/tracking-take2-track-id-failure-audit-20261012/`.

### Cached tracker probes and nuScenes control, 2026-10-12

Two additional single-condition probes use the same 251 recorded detector
responses and 300 publication timestamps. Both reproduce baseline identities,
scores and dimensions exactly, with pose roundoff below 1e-12. GT enters only
the downstream evaluator; these are counterfactual sampled tracks, not new
detector responses or actual adapter publications.

| Native geometric front | TP / FP / FN | IDF1 | Switches / fragmentation | Center RMSE |
|---|---:|---:|---:|---:|
| Baseline |391 / 238 / 60|0.6481|5 / 10|0.9514m|
| Confidence-scaled weak correction noise |400 / 229 / 51|0.6667|5 / 7|0.9668m|
| Three-hit weak-only tentative births |396 / 288 / 55|0.5903|19 / 8|0.9530m|

The confidence-noise probe gains nine GT observations with none lost, but fails
the predeclared overall-RMSE gate. The 391 common observations improve from
0.9514m to 0.9380m RMSE; the nine gained observations have 1.8198m RMSE. This
breakdown explains the aggregate increase without replacing the acceptance gate.
The weak-birth probe keeps strong births immediate and requires three consecutive
served matches before promoting an unmatched weak tracklet. It gains eight GT
observations and loses three, increases false positives and switches, and is
rejected. Neither mechanism changes production defaults or justifies a sweep.

Evidence: `artifacts/tracking-take2-confidence-noise-probe-20261012/`,
`artifacts/tracking-take2-confidence-noise-visual-comparison-20261012/`,
`artifacts/tracking-take2-confidence-noise-availability-audit-20261012/`, and
`artifacts/tracking-take2-weak-three-hit-birth-probe-20261012/`.

The next detector control uses one nuScenes scene, scene-0103 (40 keyframes),
with recorded calibration and separate sensor exposure poses. The intended
comparison is the upstream nuScenes checkpoint with native six-camera/multi-sweep
inputs versus one front camera with identical LiDAR sweeps. Detection scoring
uses explicit nuScenes classes, score >=0.2, class-aware inclusive 2m XYZ center
association, and separately reports model-range and geometric-front coverage,
dimensions and heading errors. For the front vehicle classes, precision and
recall >=80% and matched center RMSE <=1m are prospective sanity targets, not
official nuScenes benchmark scores or proof of CooperScene calibration accuracy.

Source inspection exposed a loader bug: reverse sample links could select
intermediate sweeps before keyframe records. Keyframe-only indexing now prevents
that choice; all 40 scene-0103 LiDAR/front-camera links are verified keyframes,
with sample timestamps equal to LiDAR timestamps. Twenty focused loader/export/
adapter/MCAP tests pass. Inference and its accuracy remain to be measured.

### nuScenes checkpoint control results, 2026-10-12

The same 40 scene-0103 keyframes were evaluated with the legacy converted
nuScenes checkpoint and an official MMDetection3D checkpoint for the native
six-camera BEVFusion configuration. Each condition also evaluates one front
camera with identical native multi-sweep points. GT, calibration, exposure
poses, score >=0.2, inclusive 2m XYZ matching and geometric-front policy remain
fixed; neither height nor heading is fitted to GT.

| Front vehicles | Precision / recall | XYZ RMSE | Directed yaw MAE |
|---|---:|---:|---:|
| Legacy converted, six cameras |90.14% / 65.92%|0.8756m|160.40deg under cuboid-equivalent +90deg axes|
| Official compatible, six cameras |89.14% / 67.41%|0.3667m|4.35deg|
| Official compatible, one front camera |88.78% / 66.92%|0.3511m|4.59deg|

Official outputs use their native gravity center, LWH and directed yaw without
axis changes. Their mean front-vehicle LiDAR-Z errors are +0.0327m and +0.0215m,
compared with approximately +0.8m for the legacy weights. Recall still fails the
prospective 80% target. All 402 geometric-front GT observations remain included,
including 59 with zero LiDAR and radar points; physical occlusion is not filtered.
These are fixed-score one-scene diagnostics, not official nuScenes AP/NDS or
paper reproduction. The legacy run used Lambda's pinned image; the official run
used the local runtime, so this is not a replicated weight-only causal estimate.

The first official run is invalid because tolerant initialization skipped 21
incompatible sparse-convolution kernels. Its outputs are preserved separately
and excluded from accuracy conclusions. A source-supported kernel-axis
permutation produces a separate compatible checkpoint; all 582 parameter keys,
shapes and copied tensor values are verified before the valid 80 forwards. The
original checkpoint remains unchanged. An independent recount reproduces all
80 prediction sets, geometric matches, counts and pose errors.

The sidecar now enforces strict checkpoint loading before constructing its
pipeline or serving detections. CPU checks pass for legacy and compatible
nuScenes weights, reject incompatible official weights, and verify the retained
CooperScene checkpoint in its pinned runtime (574 keys). This four-line source
change was rsynced to Lambda and Gamma without moving their Git HEADs or
restarting services. CooperScene weights, calibration and XYZ policy are unchanged.

Evidence: `artifacts/tracking-nuscenes-scene0103-native-trial-20261012/`,
`artifacts/tracking-nuscenes-official-native-checkpoint-20261012/`,
`artifacts/tracking-nuscenes-scene0103-official-native-retry-20261012/`,
`artifacts/tracking-nuscenes-scene0103-official-recount-20261012/`,
`artifacts/tracking-bevfusion-strict-load-cpu-20261012/`,
`artifacts/tracking-bevfusion-native-strict-load-cpu-20261012/`, and
`artifacts/tracking-bevfusion-strict-constructor-rsync-20261012/`.

### Take-2 localization recheck and remaining tracking errors

The independent raw-GNSS/INS recipe recheck covers all 300 pilot poses: XYZ
translation RMSE is 0.031700m, maximum 0.090459m, and all satisfy strict error
<0.1m. Stored reference matrices and the archived accepted/propagated pose policy
reproduce exactly. There are 298 accepted poses and two propagated poses. This
is agreement with the retained ENU recipe, not independent absolute GT accuracy.

For the unchanged 289 matched source front-camera detections, true ENU horizontal
RMSE is 0.767680m and vertical RMSE is 0.317188m. Horizontal RMSE within 20m is
0.354m. Published tracks have ENU horizontal RMSE 0.887949m and vertical RMSE
0.341680m. Replacing estimated sensor poses with recipe poses only in an
evaluation attribution diagnostic does not improve source horizontal error
(0.773155m); no prediction or localization pose was replaced in the pipeline.
Thus localization passes the requested translation check, but horizontal
detection accuracy across the full front-camera range remains unresolved.
Ground placement remains explicit and rendering-only, preserving scored poses.

A fixed-pair geometry diagnostic finds matched LWH absolute errors of
0.085/0.034/0.051m and mean length-axis yaw error 5.08 degrees modulo180.
Converter, native head and physical cuboid conventions are consistent. Overall
horizontal-error correlations with shape error and yaw error are weak (0.25 and
0.22); these conditional matched-pair statistics do not justify a geometry
correction. See `artifacts/tracking-take2-shape-yaw-error-diagnostic-20261012/`.

Three additional isolated cached tracker trials retain the same 251 responses
and 300 publication times. All reproduce the baseline before candidate scoring;
none changes production defaults or qualifies for promotion.

| Native geometric front | TP / FP / FN | IDF1 | Switches / fragmentation | XYZ RMSE |
|---|---:|---:|---:|---:|
| Baseline |391 / 238 / 60|0.6481|5 / 10|0.9514m|
| Soft lateral velocity constraint |383 / 245 / 68|0.6358|2 / 6|1.0809m|
| Constant acceleration |390 / 243 / 61|0.6439|7 / 8|1.0280m|
| BEV footprint IoU association |390 / 228 / 61|0.6268|4 / 9|0.9490m|

The IoU trial slightly improves positional error and false positives but loses
seven previously matched GT observations and lowers IDF1. The other two trials
worsen position error and coverage. No parameter sweep follows these failures.

Evidence: `artifacts/tracking-take2-xyz-translation-recheck-20261012/`,
`artifacts/tracking-take2-enu-error-decomposition-20261012/`,
`artifacts/tracking-take2-horizontal-error-contributors-20261012/`,
`artifacts/tracking-take2-soft-nonholonomic-probe-20261012/`,
`artifacts/tracking-take2-constant-acceleration-probe-20261012/`, and
`artifacts/tracking-take2-bev-iou-association-probe-20261012/`.

### Optional identity reacquisition

An identity-only registry retains confirmed retired identity candidates for up
to two seconds after their last observation. It requires the original finite
association gates, matching class, predicted center distance <=3m, length-axis
agreement within30 degrees modulo180, dimension ratios within[0.8,1.25], and a
mutually unique old/new pair. Expired Kalman filters remain expired; new filters
start fresh, and no boxes are published during gaps. GT is evaluation-only.

| Fixed 300-frame native scope | Baseline / enabled IDF1 | Baseline / enabled switches | Unchanged TP / FP / FN |
|---|---:|---:|---:|
| Geometric front camera |0.6481 / 0.7222|5 / 4|391 / 238 / 60|
| Full tracker state, reference LiDAR XY square54m |0.7578 / 0.8016|12 / 4|3210 / 964 / 615|

All positions, rotations, dimensions, scores, matching indices, misses and false
positives are unchanged. Front XYZ RMSE remains0.951419m and ENU horizontal RMSE
0.887949m. The full-state scope above includes off-camera tracks and differs
from earlier all54 diagnostics based on published camera-prompt subsets.
Nine annotated reuse links agree at their old-last/new-first GT association;
one link remains unverified. Existing GT84/92 association ambiguity persists.
This is an identity improvement, not a detection or coverage improvement.

The production option is disabled by default. Enable it for offline comparisons
with `--bevfusion-maximum-reacquisition-seconds 2.0`. The production tracker
matches the frozen prototype bit-for-bit for all300 frames, with both the
option disabled and enabled. Focused tracker and adapter tests cover expiry,
fresh filter state, ambiguous candidates, active identities, bounded retention,
rigid rotations and argument validation.

A separate baseline FP audit finds199 of238 front FP publications on track10
before GT92's annotations begin; it subsequently matches GT92 for20 frames.
This suggests early annotation censoring, without proving physical correspondence.
All199 remain counted as FP. Only6 FP coast without a latest-response update,
whereas16 TP also coast; no coast-only suppression is justified.

Evidence: `artifacts/tracking-take2-identity-reacquisition-probe-20261012/`,
`artifacts/tracking-take2-identity-reacquisition-production-parity-20261012/`, and
`artifacts/tracking-take2-wait-baseline-fp-attribution-20261012/`.

The Gamma primitive-asset check also passes on cached production frames243/299:
unlabelled pixels, geometry and scene/shared-asset occlusion statistics are
identical between baseline and enabled identities. At frame299 the asset45/10
case at45.7m projects304 pixels:59 are scene-occluded and245 hidden by nearer
assets, leaving zero visible. This verifies compositor behavior for the current
wait1.0 inputs, not physical camera correspondence. Original K/T and heights are
preserved without grounding or SAM; existing CloudXR processes are unchanged.
Paired PNGs and receipt:
`artifacts/tracking-take2-identity-reacquisition-gamma-primitives-20261012/`.

### Supplemental native IoU diagnostic

The CooperScene paper reports BEV/3D IoU AP, not translation or yaw error.
The same251 cached detector responses now have a supplemental IoU evaluation
using original single-agent test-PKL centered LWH/yaw boxes and the official
polygon-IoU/VOC2010 functions. Original reference-LiDAR54m and camera16px
eligibility remain fixed. The original289/102/101 front center-matching counts
and all289 pairs reproduce before scoring. Native upright7D benchmark boxes
are a separate representation; no scored world pose or object height changes.

| Available proposals, confidence >=0.1 | BEV AP at0.3 / 0.5 / 0.7 | 3D AP at0.3 / 0.5 / 0.7 |
|---|---:|---:|
| Geometric front camera |0.7043 / 0.4680 / 0.2126|0.6235 / 0.2883 / 0.0879|
| Reference LiDAR XY square54m |0.8004 / 0.5905 / 0.3199|0.7143 / 0.3533 / 0.2138|

These are truncated-proposal, 251-capture subset AP diagnostics, not full
benchmark reproduction. The broader-view BEV values are close to the paper's
single-vehicle LiDAR-camera0.80/0.57/0.33, but coverage, range and proposal
support differ. Published3D values are0.72/0.42/0.23. No numeric center-RMSE
target can be inferred from those AP values.

At the unchanged primary confidence0.2, GT92 matches40/69 observations at
BEV IoU0.5, but only8/69 at3D IoU0.5. Its original center-matched median BEV/3D
IoUs are0.692/0.435, supporting vertical error as a limitation for this ID.
GT84 has zero IoU0.5 matches in either representation; grounding cannot recover
its missed detections or horizontal overlap. No GT-conditioned tracking rule
or grounding is applied by this diagnostic.

Evidence: `artifacts/tracking-take2-native-iou-diagnostic-20261012/` and
`artifacts/tracking-cooperscene-paper-error-comparison-20261012/`. Published
reference: [CooperScene paper, Section4 and Table4](https://arxiv.org/html/2606.31219v1).

### Explicit display grounding and camera-resolution probe

An explicit rendering-only comparison uses the same enabled-identity frames
243/299. Gaussian-map depth in a fixed lower-central asphalt region seeds the
existing bounded road-surface implementation; thin upward-facing surfels and
connected-road defaults are fixed before rendering, with no GT lookup or
parameter sweep. The resulting profile supports29 of32 input boxes; three
unsupported boxes retain their poses exactly. All eight originally front-eligible
rendered boxes have support. Scored poses, original visibility eligibility,
calibration, identities, dimensions and scores remain unchanged. Placement acts
on display copies and can change XY/rotation along the local road tilt. Map
geometry and plausible connectivity do not establish surveyed road semantics.
Grounding remains opt-in, with original-height panels retained for comparison.

The separate2x camera-resolution probe runs four fresh forwards at captures
120/180/240/299. It doubles resolution while preserving crop field of view,
calibration and all learned weights. All574 checkpoint tensors load strictly
first; only the source-generated, nonlearned frustum grid is then regenerated
for512x1024 images and64x128 features. The other573 state tensors remain exact.
The initial unchanged-grid strict-load incompatibility is retained as evidence;
it is not bypassed with partial loading. No checkpoint or production server
is edited.

Front counts remain6/0/5 and all54 counts46/3/14, with zero GT observation gains
or losses. Front XYZ RMSE worsens0.78310 to0.86894m; all54 worsens0.65114 to
0.66311m. The probe fails the declared gain/error gates, so native resolution
remains unchanged. There are four candidate forwards, zero warmups, and no
further resolution search. Peak allocated memory is1.32GB; three steady forwards
take69.74–70.76ms excluding preprocessing, not a replay-throughput measurement.

Evidence: `artifacts/tracking-take2-render-only-grounding-20261012/`,
`artifacts/tracking-take2-double-camera-resolution-probe-20261012/`, and
`artifacts/tracking-take2-double-camera-resolution-regenerated-frustum-20261012/`.

### Full same-sequence run and XYZ recheck

The fixed start remains native index 17/frame487867. All 864 frames through
index 880/frame488730 now have fresh native-checkpoint inference, retaining all
200 raw proposals. Original 300 sensor arrays, timestamps, calibration, GT and
retained XYZ poses reproduce exactly. This run processes every capture serially
offline; it differs from the previous 251-served/300-publication queue cadence
and establishes no real-time throughput or latency result. GT enters evaluation
only. Baseline calibration, native resolution, classes, confidence thresholds,
reference-LiDAR 54m square, partial-image 16px eligibility and inclusive 2m XYZ
matching remain fixed.

| Front-camera output | TP / FP / FN | Precision / recall | XYZ RMSE |
|---|---|---|---:|
| Detections, confidence >=0.2 |1707 / 425 / 576|80.07% / 74.77%|0.9187m|
| Tracks, either identity option |2102 / 710 / 181|74.75% / 92.07%|0.9593m|

Optional 2s identity reacquisition improves geometric IDF1 from 0.6375 to 0.7242
and switches from 20 to 14; fragmentation remains 13. All geometry, dimensions,
scores and ordering are identical. Across all 54m tracks, switches fall 58 to 29
and IDF1 rises 0.7385 to 0.7835. The option remains disabled by default. Of 40
geometrically front-eligible GT IDs, 39 are ever matched; GT208 has zero matches
in 16 observations. The unchanged per-ID 90% recall and 90% precision targets fail.
Image intersection does not certify optical visibility.

The independent raw-GNSS/INS recipe recheck reproduces all stored references
and retained accepted/propagated poses exactly: 856 accepted and 8 propagated.
Full XYZ translation RMSE is 0.036967m, maximum 0.109072m; 861/864 satisfy strict
error <0.1m. Frames 488724/488725/488726 have errors 0.100294/0.109072/0.102848m
and remain included without gate changes. The original 300 all pass, with
maximum 0.090459m. This is conditional agreement with the retained ENU recipe,
not independent absolute GT accuracy; G is never applied directly to ICP poses.

Existing matched pairs give detection ENU horizontal/vertical RMSE
0.823797/0.406715m, and track RMSE 0.862508/0.419956m. Detection horizontal
RMSE is 0.293752m within 20m, 0.716129m at 20–40m and 0.980532m at >=40m.
Far matched detections have mean vertical error -0.473980m. Horizontal error
therefore remains material despite good localization reference agreement;
ground placement cannot address these errors or missed detections. These are
matched-only statistics, with no reassociation or pose corrections.

GT208 occurs at frames 488652–488667, 43–49m from LiDAR. Its original-camera
box lies wholly inside the model crop for 14 frames and partly inside for two.
All 16 oriented GT boxes contain zero original LiDAR returns. Eight frames have
raw proposals within 2m, but their highest confidence is 0.00583–0.06854; eight
have no close proposal. Primary detections are at least 8.03m away. XYZ errors
here are 0.043–0.063m, unrelated to the three tail exceptions. This supports a
detector-support limitation, without proving occlusion or invalid annotations.
Its horizontal/vertical matched errors are unavailable, not zero.

Native GT continuity was checked across 3456 annotation files from four agents:
30,413 shared observations have identical literal source geometry. V1 has 82
IDs before camera/range eligibility, two annotation gaps and a near 180-degree
yaw flip. Canonical IDs remain unchanged; absent annotations do not establish
death. Cached public-adapter and real FrameSnapshot codec replay also reproduces
all original 300 output geometries and matching pairs. It does not reenact the
49 unserved callbacks or certify queue performance.

Evidence: `artifacts/tracking-take2-full-sequence-native-20261012/` (results,
fixed-pair ENU errors and GT208 camera/LiDAR PNGs),
`artifacts/tracking-take2-full-sequence-xyz-recheck-20261012/`,
`artifacts/tracking-take2-full-sequence-gt-continuity-20261012/`, and
`artifacts/tracking-take2-identity-adapter-wire-replay-20261012/`.
Labelled camera/LiDAR comparison video and representative PNGs are in
`artifacts/tracking-take2-full864-labelled-20261012/`; wireframes preserve
original heights and do not perform depth occlusion.

A single cached 864-frame range-noise probe keeps optional 2s reacquisition and
uses sensor-derived isotropic measurement covariance
`I * clip((detection_range / 20m)^2, 1, 4)` in association and correction.
Baseline geometry and metrics reproduce exactly before the candidate runs.
The candidate worsens TP/FP/FN from 2102/710/181 to 2074/729/209, ENU horizontal
RMSE from 0.862508 to 0.869601m and IDF1 from 0.724239 to 0.646874. There are 45
lost matched observations and 17 gained; common-pair errors also worsen.
The frozen improvement gates reject it, with production unchanged and no NN
rerun. Evidence: `artifacts/tracking-take2-full864-range-noise-probe-20261012/`.

Original-resolution GT208 camera crops at frames 488652/488659/488667 do not
identify a particular physical vehicle as that ID. Its projected rectangle lies
over background above prominent car bodies; nearer GT rectangles overlap it,
but rectangle overlap is not optical occlusion proof. At 488652, projecting all
original LiDAR points with unchanged K/T places foreground returns on visible
car bodies and road, so a universal calibration pixel offset is not established.
The camera/annotation correspondence remains uncertain, with GT208 still counted
as missed and no GT exclusions or pose adjustments. Raw/overlay pairs and exact
projection provenance: `artifacts/tracking-take2-GT208-camera-context-20261012/`.

The one-frame source check at 488652 confirms the native PKL's named cloud XYZ
and camera pixels equal the replay inputs exactly; binary intensity equals raw
intensity/65535. PKL K/T agrees with runtime K/T, and model payload hashes bind
the same sensor arrays. Selected GT centers/dimensions agree with native YAML.
Raw oriented-box point counts for GT208/84/188/237 are 0/57/14/39. No wrong-file,
cloud or calibration linkage is established; annotation exposure remains
unknown. Evidence: `artifacts/tracking-take2-frame488652-source-correspondence-20261012/`.

One cached 864-frame detector-postprocessing trial fits the sensor-facing box
face to raw LiDAR support, requiring point count and tangential-span support.
Median face residuals are bounded to 0.25m, then projected onto the retained
ENU horizontal plane so localized physical height is preserved. Unsupported
boxes, dimensions, yaw, confidence and order remain unchanged; GT enters only
evaluation. This does not refine localization or perform ground placement.
Independent synthetic controls verify face signs, exact prefilter/fallback and
height preservation under tilted sensor poses and rotated world coordinates.

The baseline reproduces all saved 864-frame geometry and metrics exactly.
The candidate adjusts 5910 of 14122 boxes but worsens detection ENU horizontal
RMSE from 0.823797 to 0.826153m and XYZ RMSE from 0.918727 to 0.921007m.
It loses five previously matched detection observations and gains one.
Tracking TP/FP/FN changes from 2102/710/181 to 2100/711/183; horizontal and
XYZ errors also worsen, including on common matched observations. The frozen
gates reject the trial; production remains unchanged. Source hashes and ENU
height preservation pass. Evidence:
`artifacts/tracking-take2-full864-visible-face-probe-20261012/`.

### Full-proposal detection scoring

All 172800 cached raw proposals (200 per capture) now have native upright-box
IoU scoring on the same 864 frames, including confidence below 0.1. Original
primary center counts 1707/425/576 and every matching pair remain exact; 48
representative checks agree with the source evaluator. No NN, calibration,
height, tracker or confidence default changes occur.

| Scope, all proposals | BEV AP at 0.3 / 0.5 / 0.7 | 3D AP at 0.3 / 0.5 / 0.7 |
|---|---:|---:|
| Geometric front camera |0.7304 / 0.4253 / 0.1974|0.5198 / 0.2441 / 0.0996|
| Reference LiDAR XY square 54m |0.8445 / 0.5067 / 0.2250|0.7505 / 0.2753 / 0.1525|

At confidence 0.05, center matching detects all 40 front GT IDs at least once,
including GT208, with precision 65.05% and recall 90.49%. Confidence 0.2 gives
39 IDs, precision 80.07% and recall 74.77%; 0.1 still misses GT208. These fixed
confidence diagnostics do not change the replay pipeline. GT208's best raw
proposal reaches BEV/3D IoU 0.85665/0.55719 at confidence 0.04877, below replay
continuation and birth thresholds. Low confidence explains some lost coverage;
lowering it does not fix strict pose overlap and substantially increases FP.

The Luna paper check confirms single-vehicle LiDAR-camera BEV AP 0.80/0.57/0.33
and 3D AP 0.72/0.42/0.23. Our broader-view AP is near/slightly above at 0.3 but
lower at 0.5/0.7. This remains one test-split sequence and ego, with fixed 54m
scope; the paper averages the full test evaluation across three egos. Neither
paper reproduction nor being within a reported error interval is established.
The paper reports no detector center/yaw-error target; its separate roughly
0.2m spatial-alignment RMSE concerns LiDAR point correspondences.

Evidence: `artifacts/tracking-take2-full864-all-proposal-iou-20261012/` and
`artifacts/tracking-cooperscene-full864-paper-comparison-20261012/`.
Published comparison: [CooperScene paper, Section 4 and Table 4](https://arxiv.org/html/2606.31219v1).

A targeted benchmark-extrinsics probe uses four GT208 development captures
785/788/790/800 with eight fresh baseline/candidate forwards. The provided
`scene2_agent1` camera transform has the verified left correction
`Rz(-2 degrees) @ Rx(-2.5 degrees)`; only the inference camera pose changes.
All sensor bytes, K, XYZ poses, weights and original evaluation-camera gates
remain fixed. All 574 tensors load strictly with the original frustum; fresh
baseline primary membership/counts/matching pairs reproduce cached results.

Both cases give front 21/5/4 and all54 83/10/12, with zero GT gains or losses.
Front horizontal RMSE changes 0.487501 to 0.489014m and XYZ 0.514817 to 0.515292m.
GT208 remains missed at 0.2. At capture 790 its best BEV/3D overlap improves
slightly, but confidence falls 0.048767 to 0.046905; at 800 it falls 0.066236 to
0.064835. The acceptance gates reject a calibration switch or expansion.
These GT-selected development cases do not establish full-sequence behavior.
Replay calibration remains baseline; source hashes and input-matrix review
pass, and the owned inference container exits 0 without OOM. Evidence:
`artifacts/tracking-take2-GT208-benchmark-extrinsics-probe-20261012/`.

A separate cached 864-frame weak-track branch leaves all primary filters and
published IDs/geometry/scores unchanged. It uses only raw boxes with
`0.05 <= score < 0.2`, three-hit confirmation, duplicate guards against strong tracks
and high-confidence detections, and current-observation-only publication.
GT enters evaluation after combining the outputs; no GT-dependent selection,
pose correction, calibration change or new inference occurs.

The union changes front TP/FP/FN from 2102/710/181 to 2149/1193/134: 47 additional
matches and 483 additional FP. Precision falls 0.7475 to 0.6430 and IDF1 falls
0.724239 to0.656; switches increase 14 to 151 and fragmentation 13 to 18.
GT208 remains unmatched, so ever-matched coverage stays 39/40. Added weak boxes
replace primary matched identities in 138 existing GT matches despite the
unchanged original outputs. Lower aggregate position RMSE therefore does not
justify this branch. Frozen cost/identity gates reject it; production remains
unchanged. Source hashes, all 864 primary-output parity and focused branch
controls pass. Evidence:
`artifacts/tracking-take2-full864-secondary-weak-probe-20261012/`.
Its `diagnostic-PNGs/` shows the first two replacement frames251/252: GT84's
match changes from primary42 to weak182 while all primary box geometry remains
exact. Candidate panels are explicitly labelled rejected, with original heights.

The exact earlier weak-correction confidence-noise policy was also rechecked
on all 864 frames with optional identity reacquisition. It improves common-pair
XY/XYZ errors slightly and overall horizontal RMSE from 0.862508 to 0.853216m,
but loses ten existing matches, gains five and increases FP 710 to 715.
Some aggregate improvement comes from dropping harder matches. The fixed
coverage/FP gates reject the policy; no production update occurs. Evidence:
`artifacts/tracking-take2-full864-confidence-noise-probe-20261012/`.

### Controlled Take 2/V2 pilot

The next agent uses the same strict-qualified-start and archived XYZ policy:
native indices 123–422, frames 487973–488272, 300 consecutive captures. V2 uses
its own raw GNSS/INS sensor composition and baseline K/T, with no V1 mounting
correction. All 300 references and accepted/propagated poses reproduce exactly:
295 accepted, five propagated, translation RMSE 0.037299m/max 0.058905m, zero
errors >=0.1m. This remains conditional recipe agreement, not absolute GT.

All 300 fresh serial forwards strictly load the same 574 native-checkpoint
tensors and original frustum, preserving single raw XYZI/intensity once.
All 60000 raw proposals and source payloads pass controls. GT is evaluation-only;
there is no SAM, grounding, calibration/default change or new localization.

| Geometric front output | TP / FP / FN | Precision / recall | XYZ RMSE |
|---|---|---|---:|
| Detections >=0.2 |933 / 185 / 111|83.45% / 89.37%|0.749077m|
| Tracks, either identity mode |1012 / 390 / 32|72.18% / 96.93%|0.761604m|

All 19 GT IDs are detected and tracked at least once, meeting the minimum here.
Stronger per-ID 90% coverage and 90% precision fail: GT111 tracks 6/15 and GT135
35/46. Front IDF1 is 0.804579 with nine switches/eight fragments in both modes;
IDs can differ while geometry and these aggregate metrics agree. All54 IDF1
improves 0.819809 to 0.825298 and switches 12 to 10 with optional reacquisition.
This does not resolve V1's miss. V2 starts cold at its own earliest-qualified
frame; its sensor inputs and window differ from V1, so this is generalization,
not a paired causal comparison of agents or a real-time latency test.

Existing matched pairs give detection ENU horizontal/vertical RMSE
0.656074/0.361502m and track RMSE 0.656469/0.386121m. Track horizontal RMSE
within 20m is 0.360m but increases to 0.813m at 20–40m and 0.938m at >=40m.
Good translation agreement therefore does not establish uniformly good
horizontal detection accuracy or justify changing scored heights.

A fixed-pair evaluation-only attribution finds orientation disagreement with
the recipe of 0.706188 degrees RMS/max 0.761562. Reexpressing the same 933 detector
matches with reference poses only for diagnostics changes horizontal/vertical
RMSE from 0.656074/0.361502 to 0.645325/0.297916m; signed vertical mean changes
-0.214674 to -0.031969m. Orientation contributes more at distance: far vertical
RMSE 0.710762 becomes 0.418196m. Thus vertical scoring error is not solely a
detector effect. This neither proves a physical localization defect nor changes
matching, FOV, tracker inputs, rendered/scored poses or mounting calibration.

Evidence: `artifacts/tracking-take2-vehicle2-native-pilot-20261012/`, including
`pose-attribution/` and `visualization/` (30-frame GT111 camera/LiDAR video and
two GT111/GT135 PNGs, original heights and explicit visibility uncertainty).

### Take 2/V3 coverage and display-ground verification

V3's earliest strict-qualified start is native index 747/frame 488597. Of 881
native frames, only 134 remain through index 880/frame 488730; the requested 300
was clamped at the recording end without moving the start. All 134 references
and archived poses reproduce exactly: 126 accepted/eight propagated, translation
RMSE 0.036913m/max 0.066262m, zero >=0.1m. Orientation disagreement is larger:
1.643586 degrees RMS/max 2.104005, still conditional recipe agreement.

All 134 actual serial forwards retain 26800 proposals and strictly load the
same 574 weights/original frustum. Source/payload/calibration controls pass.
Front detections give 343/40/100 (precision 89.56%, recall 77.43%); tracks give
356/84/87 (precision 80.91%, recall 80.36%), geometric IDF1 0.795017, six switches
and five fragments. Front aggregate metrics agree across identity options.
Only 14/15 GT IDs are ever detected: GT130 remains 0/6. Minimum and stronger
coverage targets fail. Cold starts/windows differ across agents; this is not a
paired causal comparison or a real-time queue/latency benchmark.

Matched detection ENU horizontal/vertical RMSE is 0.729765/0.653443m; tracks
0.741360/0.640341m. Track horizontal RMSE within 20m is 0.304m, but increases to
0.841m at 20–40m and 0.967m at >=40m. On the same 343 detector matches, reference-pose
reexpression solely for diagnostics changes vertical RMSE 0.653443 to 0.326780m
while horizontal error slightly worsens. Far vertical RMSE 0.845586 becomes
0.396710m. Orientation contributes to scoring residuals; this does not prove
physical localization error or change poses, matching, calibration or heights.

GT130's first five eligible frames have no raw proposal within 2m under either
original or diagnostic reference reexpression. Its sixth has only a diagnostic
weak 0.050916-score proposal within 1.876m; primary detections still miss all six.
Thus reference orientation does not recover the primary detection failure.
The labelled 20-frame camera/LiDAR clip preserves original scoring poses.
Evidence: `artifacts/tracking-take2-vehicle3-native-pilot-20261012/`, including
`pose-attribution/` and `visualization/`.

Separately, V2 real-map primitive panels 258/263 compare original height against
explicit opt-in placement using the existing road profile unchanged. Both
panels select the same original camera-eligible boxes before placement (7/18
and 8/20); all selected boxes have support. Display-up shifts range 0.024–0.867m
and 0.029–0.673m. Unsupported full-list objects retain their original poses.
There is no GT lookup, new profile fit, extrapolation, SAM or scoring correction.
The initial all54 render included a camera-crossing ineligible primitive and
is preserved; final panels use the declared original eligibility.

Scene/shared-asset depth buffering hides V2 track 20 at 50.87m: all 27 projected
pixels are scene-occluded. At 50.31m, track 30's 545 pixels split into 127 scene,
128 nearer-asset occlusions and 290 visible. This verifies compositor behavior,
not optical ground truth. Source/profile/map hashes and Gamma CloudXR state
remain unchanged. Evidence:
`artifacts/tracking-take2-vehicle2-explicit-ground-render-20261012/`.

### Controlled Take 3/V1 pilot

Native indices 0–299, frames 496760–497059, use the same earliest-qualified-start,
archived accepted/propagated policy and baseline calibration. All 300 are accepted:
XYZ translation reference RMSE 0.025217m/max 0.042865m, zero >=0.1m; orientation
RMS 0.251445 degrees/max 0.367467. Direct raw-GNSS quaternion/F1, rotated 0.03m
antenna and recorded/new sensor-extrinsic composition reproduces the retained
V1 mounting plus Take-3-only left position correction. Stored references and
archived poses reproduce exactly. The position bias was map-assisted and fitted
on part of this take; these are conditional reference errors, not independent
absolute localization accuracy.

The current checkpoint-declared training PKL contains all 501 Take-3 frames for
each of V1/V2/V3. This probe is training-split-overlapping; the immutable
checkpoint's historical annotation hash remains unknown. Its better scores are
not evidence of held-out generalization.

All 300 fresh serial forwards retain 60000 proposals and strictly load the same
574 tensors/original frustum. Source/calibration/payload controls pass. Geometric
front detections >=0.2 give 461/9/36 TP/FP/FN (98.09% precision, 92.76% recall);
tracks give 479/34/18 (93.37%/96.38%). All 16 native GT IDs are detected at least
once, passing the minimum detection condition. GT108 has one detected observation
but no track; GT73 tracks 20/28 and GT120 8/10, so the stronger per-ID 90% coverage
target fails. Optional 2s identity reacquisition changes only IDs: geometric IDF1
0.916832 to 0.948515, switches two to zero, fragmentation one unchanged.

Matched detection ENU XY/Z RMSE is 0.405642/0.131279m, tracks 0.423356/0.123788m.
Track XY/Z by <20/20–40/>=40m is 0.214/0.098, 0.425/0.110, 0.455/0.158m. These
matched-only errors preserve original heights; grounding is unnecessary for
scoring and remains an explicit display option. Logged image–LiDAR skew is
23.992–26.866ms, ego speed times skew at most 0.027177m; this is not exposure or
object-motion synchronization certification. Different windows/cold starts do
not support causal agent comparisons or real-time latency claims.

Evidence: `artifacts/tracking-take3-vehicle1-native-pilot-20261012/`, including
source/output controls, XYZ recheck, fixed-pair ENU errors and capture timing.
The decoded 20-frame paired camera/LiDAR clip at ordinals 65–84 shows GT73's
six-frame gap and baseline ID switch 22 to 27 at ordinal 80; optional reacquisition
retains 22. Labels preserve original calibration/heights and explicitly describe
geometric visibility. See `visualization/` for the clip and PNGs 74/80.

A cached residual audit reproduces all 300 tracker frames exactly. GT73's
coasted predictions already exceed the 2m matching gate at ages 0.3–0.5s; five
gap frames lack usable detections and the next strong proposal is offset 3.313m.
GT120's first two close proposals score 0.138/0.181, below the birth threshold.
GT108 is associated to an updated track, but its smoothed box falls outside the
camera intersection filter while the raw detection and GT qualify at the edge.
These cases do not justify global coast extension or weak births. No tracker
change follows. Evidence: `artifacts/tracking-take3-residual-coverage-audit-20261012/`.

### Take 4/V2 baseline and recorded-time camera probe

The earliest qualified native start is index 128/frame 515548. Only 43 of 171
frames remain, through index 170/frame 515590; the start is preserved. XYZ has
42 accepted and one propagated pose, conditional translation RMSE 0.069430m/max
0.101312m. Ordinal 1/frame 515549 is the one >=0.1m exception and remains scored.
Orientation RMS/max is 0.447794/0.770081 degrees. Direct raw-GNSS/sensor composition
and archived poses reproduce exactly, with no V1 mounting or Take-3 bias.
This window overlaps the current declared training PKL.

All 43 fresh serial forwards strictly load the same 574 tensors/original frustum
and retain 8600 raw proposals. Front detections give 201/7/55 TP/FP/FN
(96.63% precision, 78.52% recall); tracks give 222/6/34 (97.37%/86.72%). Both ID
options agree: geometric IDF1 0.917355, zero switches/fragments. Only 14/15 GT IDs
are ever detected; GT77 remains 0/6, while GT47 detects 2/11 and tracks 5/11.
Minimum and stronger coverage conditions fail. Matched detection ENU XY/Z RMSE
is 0.661451/0.365212m; tracks 0.612658/0.370274m. Original heights remain.

Each named native PNG matches its cached pixels and logged camera timestamp;
all camera times have raw-GNSS interpolation support. Camera precedes LiDAR by
72.815–74.359ms while ego speed is 8.661–9.715m/s, about 0.638–0.708m travel.
This motivates a physical recorded-time probe, without claiming true exposure
synchronization. Its 43 fresh forwards change only inference camera poses:
world_L @ inverse(raw_L) @ raw_image_time_L @ fixed_L_camera. Current XYZ
correction is held at LiDAR time; static extrinsics, pixels, points, K, timestamps,
weights, scoring camera/FOV, GT and matching remain fixed.

The candidate is rejected: detections 200/6/56 lose one match with no gains;
XY/XYZ RMSE worsens 0.661451/0.755577 to 0.663349/0.757758m. Tracks 217/5/39 lose
five with no gains, IDF1 falls to 0.903766 and switches/fragments become one/two.
Its aggregate track error appears slightly better by dropping harder matches;
on the same 217 pairs XY worsens 0.604458 to 0.608592m and XYZ 0.708236 to 0.711578m.
GT77 remains missed. No time-policy, calibration, tracker or grounding default
changes follow, and there is no parameter sweep.

Evidence: `artifacts/tracking-take4-vehicle2-native-pilot-20261012/` includes
original-height labelled GT77 camera/LiDAR video (15 decoded frames) and PNGs 4/9.
`artifacts/tracking-take4-vehicle2-camera-motion-probe-20261012/` preserves the
candidate inputs, source controls and paired common-error/coverage results.
Current training metadata contains takes 1,3,4,5,6,7,9,10,11,12; takes 2,8,13,14
are absent. The historical checkpoint training annotation hash remains unknown;
this membership guides subsequent evaluation without proving immutable holdout.

### Take 8/V1 test-metadata probe

The next 300 consecutive native captures are indices 0–299, frames 537330–537629.
All are accepted under the same archived XYZ policy: translation reference RMSE
0.045407m/max 0.058247m, zero >=0.1m; orientation RMS/max 0.203697/0.320731 degrees.
Direct raw F1/antenna/sensor/V1-mount composition reproduces references and poses
exactly, with no Take-3 bias. Current declared train/validation metadata excludes
Take 8 and test metadata contains all 431 frames per agent. Exact historical
checkpoint annotation identity remains unknown; recipe agreement remains
conditional. Logged camera lag is 20.647–23.318ms, ego speed times lag at most
0.023807m; no camera-time or calibration change is made.

All 300 fresh serial forwards retain 60000 raw proposals and strictly load the
same 574 tensors/original frustum; source controls pass. Front detections >=0.2
give 765/118/523 TP/FP/FN (86.64% precision, 59.39% recall); tracks give
1043/198/245 (84.04%/80.98%). Optional identity reacquisition improves geometric
IDF1 0.61289 to 0.62950 and switches 20 to 18; fragmentation 19 and geometry
are unchanged. Only 16/19 GT IDs are ever detected: GT11/13/16 have three/two/one
eligible observations and remain missed. The minimum criterion fails without
excluding short-lived IDs. GT40 detects 21/213 and tracks 111/213; GT25 detects
63/285 and tracks 186/285. Stronger per-ID coverage and precision targets fail.

Matched detection ENU XY/Z RMSE is 0.806631/0.198914m, tracks 0.870283/0.188665m.
Good XYZ translation agreement therefore does not establish good horizontal
object accuracy. Vertical error is relatively small; original heights remain
and grounding cannot fix the missing evidence or horizontal errors.

GT40's raw audit finds a proposal within 2m in every eligible frame, but only
21 strong frames, 103 weak-only (0.1–0.2) and 89 without a score >=0.1. Its 102
track misses comprise 47 weak-only and 55 without continuation evidence, with
no missed strong-source frame. Three representative boxes have only 7–9 raw
LiDAR points at about 48m. The gap at ordinals 270–278 exposes expiry followed
by weak detections that cannot restart an identity before the strong return.
This motivates a separate bounded cached weak-reacquisition prototype, pending
scored gates; it does not justify new weak births, larger coasts or defaults.
Raw proposal/support evidence: `artifacts/tracking-take8-GT40-source-audit-20261012/`.

Evidence: `artifacts/tracking-take8-vehicle1-native-pilot-20261012/`. Its decoded
20-frame clip at ordinals 269–288 shows GT40 recovery: baseline 39 to 44 at 279,
optional reacquisition retains 39. PNG 4 labels the never-detected short-lived
IDs. Original K/T, heights, assignment indices and source hashes are preserved;
camera correspondence and optical visibility remain caveats.

A consolidated six-probe table, exact JSON/CSV and hash-bound extraction are in
`artifacts/tracking-native-probe-summary-20261012/`. Different windows/cold starts
remain descriptive comparisons, with no pooled accuracy or real-time claim.

### Recent confirmed identity weak-recovery probe

A single cached prototype lets an unmatched 0.1–0.2 detection initialize a fresh
filter only for a mutually unique retired confirmed identity under the existing
2s age, class, finite association, 3m center, 30-degree axis and dimension gates.
It preserves true weak confidence, creates no new weak identities, resets velocity
and covariance, and cannot renew the recovery anchor through weak recoveries.
Strong birth/continuation/coast settings, detection inputs, heights and GT stay fixed.

Review found the first artifact omitted the original duplicate-birth guard for
appended weak recoveries. Original outputs/code remain byte-exact; a separate
corrected implementation protects only actual HIGH observations associated with
already-confirmed tracks before correction, preserving the production guard's
exclusions. Independent nonvacuous guard-on/off, ambiguity, successive-anchor,
coasting/weak/tentative/newborn and unique-ID controls pass. Corrected Take-8 and
Take-2 replays reproduce every original output/assignment/metric byte exactly;
the omission did not affect any observed recovery. Production source is unchanged.

Take 8 improves tracks 1043/198/245 to 1092/202/196 TP/FP/FN: 49 matched GT
observations gained, none lost; GT40 coverage 111 to 149/213. But FP rises by four,
fragmentation 19 to 23, and common-pair Z error slightly worsens. Frozen adoption
gates reject it. An explicitly authorized independent exploratory Take-2 full864
check keeps that rejection and the same policy: tracks 2102/710/181 become
2125/757/158, 23 gains without losses but 47 additional FP. IDF1 drops 0.724239
to 0.66060, switches 14 to 15; GT208 stays missed. Slight common-pair pose gains
do not justify these costs. Stop this policy, with no production promotion or
further tuning/cross-checks.

Final evidence binds corrected runs in
`artifacts/tracking-recent-confirmed-weak-reacquisition-probe-20261012/corrected-duplicate-guard/`;
the parent preserves the original experiment and independent cross-check history.

### Take 8/V2 fixed-policy probe

The next 300 native captures start at earliest-qualified index 1/frame 537331,
ending index 300/frame 537630. XYZ uses 198 accepted and 102 propagated poses:
conditional reference RMSE 0.087488m/max 0.107980m. All 36 >=0.1m exceptions
(six accepted/30 propagated) remain scored; orientation RMS/max is
0.665905/0.775325 degrees. Direct raw V2 sensor composition and stored references/
archived poses reproduce exactly, without V1 corrections or Take-3 bias.

All 300 fresh serial forwards retain 60000 proposals and strictly load the same
574 tensors/original frustum. Front detections give 1444/61/104 TP/FP/FN
(95.95% precision, 93.28% recall), tracks 1470/80/78 (94.84%/94.96%). Both identity
options agree: geometric IDF1 0.936088, one switch/seven fragments. Only 16/21
GT IDs are ever detected. GT11/13/16 have three/two/one observations, GT54 six,
and GT27 24; all remain undetected. GT27 tracks only 3/24. Aggregate precision
passes, but minimum and stronger per-ID coverage fail without dropping these IDs.

Matched detection ENU XY/Z RMSE is 0.533222/0.122307m, tracks 0.576559/0.126720m.
Track XY/Z by <20/20–40/>=40m is 0.272/0.082, 1.182/0.217, 1.339/0.318m. Near-field
dominance must not hide poor distant accuracy. Original heights remain; grounding
does not resolve the coverage/horizontal failures. Logged image precedes LiDAR
by 72.955–75.825ms, ego speed times skew at most 0.300145m/median 0.000842m;
baseline calibration/time policy is unchanged and exposure timing remains uncertain.

Evidence: `artifacts/tracking-take8-vehicle2-native-pilot-20261012/`; current declared
train/validation exclusion and test membership retain the historical-checkpoint
and conditional-reference caveats. The consolidated summary now contains seven
fixed-policy probes.

On the same 1444 detector matches, reference-pose reexpression solely for
diagnostics changes XY/Z RMSE 0.533222/0.122307 to 0.561162/0.123547m. Far XY
1.344864 becomes 1.415503m; reference-induced far XY shift RMS is only 0.086946m
against original far mean north-axis error -1.119644m. Reference orientation does
not remove the horizontal residual. This does not prove independent physical
localization accuracy or change matching, heights, poses, calibration or scoring.
See `pose-attribution/` for separate orientation/translation effects and pins.

The decoded 20-frame GT27 camera/LiDAR clip at ordinals 265–284 and PNGs 265/282
preserve original poses; PNG 296 shows never-detected GT54. GT27 has no proposal
within 2m at score >=0.1 in all 24 eligible frames. Representative 261/273/282
nearest scores are 0.00161/0.04385/0.02582, with 2/0/6 raw points inside the
conditional GT cuboid. Those projected rectangles are fully inside the camera,
about 9k–16k pixels², so partial off-screen geometry does not explain these cases.
Optical correspondence/visibility remains uncertain. Its only three matches
282–284 are zero-velocity coasts of a track born at 280, not fresh nearby detector
observations. No weak-recovery or grounding correction follows this evidence.

### Take 8/V3 and separate height attribution

The final Take-8 agent probe uses native indices 108–407, frames 537438–537737,
from the earliest strict-qualified start. All 300 captures remain: 299 accepted,
one propagated, XYZ translation reference RMSE 0.033451m/max 0.065195m, zero
>=0.1m. Orientation disagreement is much larger: RMS 1.755287 degrees. Direct
raw V3 composition and archived/reference poses reproduce exactly; there are no
V1 corrections, Take-3 bias, recipe changes or G applications to released ICP poses.

All 300 fresh forwards retain 60000 raw proposals and strictly load the same
574 tensors/original frustum. Front detections give 733/291/513 TP/FP/FN
(71.58% precision, 58.83% recall), tracks 968/531/278 (64.58%/77.69%). Optional
identity reacquisition improves geometric IDF1 0.64408 to 0.65209, switches ten
to nine, fragmentation 32 unchanged. Only 12/13 GT IDs are detected at least
once; GT67 is 0/2 despite two geometrically matched tracked observations. All
13 are tracked at least once, which does not satisfy the minimum detector target.
GT50 detects 178/300 and tracks 169/300; GT49 detects 89/297 and tracks 243/297.
Precision and stronger per-ID coverage fail. All scored heights remain original.

On the same 733 detector matches, original ENU XY/Z RMSE 0.874433/1.051320m
becomes 0.866468/0.180584m under reference-pose reexpression solely for diagnostic
attribution. Orientation-only Z is 0.188992m, translation-only 1.030503m. Far Z
1.319432 becomes 0.216188m; reference-induced far Z shift RMS is 1.245329m.
Native LiDAR-frame residuals on those same pairs are XY/Z 0.866787/0.179046m.
Thus sub-0.1m position agreement does not guarantee reference-orientation
agreement, and the large world-height residual is mostly an orientation
contribution here. This neither establishes which orientation is physically
correct nor changes scored poses, matching, GT, renderer heights or tracker
inputs. It is not evidence that BEVFusion itself has a 1m height defect.
Horizontal errors remain substantial; blanket grounding is not a tracking fix.

Evidence: `artifacts/tracking-take8-vehicle3-native-pilot-20261012/`. The decoded
20-frame original-height GT49 clip at ordinals 278–297 shows baseline ID16 to 54
at 288, while reacquisition retains 16. PNG40 shows GT50's switch in both modes;
PNG283 shows a gap. `pose-attribution/` binds reference and sensor residuals.
The consolidated summary now includes eight probes and orientation RMS alongside
translation errors, so good positional agreement cannot hide this distinction.

### Take 8 recorded clocks and static benchmark calibration check

Across all 431 same native frame IDs, vehicle LiDAR header phases relative to V2
are V1 median -9.769ms and V3 +11.664ms; infrastructure is +18.233ms, with maximum
all-agent phase 28.769ms. V1/V2 camera headers differ about 86.980ms. At 33 selected
frames, 1692 shared GT observations across six agent pairs have exactly equal
literal global centers/dimensions despite those phases. Native YAML records no
annotation capture epoch; historical generation assumes synchronized filenames,
and converters/grouping use filenames. Shared IDs/content do not prove a shared
capture time. Measured LiDAR-phase times conditional GT speed cannot alone explain
V2's far mean north-axis error -1.1196m. Annotation/exposure epoch remains unknown;
no fitted offset, GT time warp, reassociation or replay time-policy change follows.
Evidence: `artifacts/tracking-take8-cross-agent-clock-audit-20261012/`.

The supplied `scene8_agent2` benchmark extrinsic uses the same K/translation and
relative camera Rx -3 degrees. A targeted four-capture detector-only probe at
ordinals 261/273/282/296 uses eight fresh paired forwards (baseline/candidate),
strict 574 tensors/original frustum, zero warmups and unchanged payloads/XYZ.
Only inference camera poses change; evaluation camera, GT and FOV remain original.
Both give 17TP/0FP/7FN with no gains/losses. XY/XYZ point estimates change slightly
from 0.853942/0.880238 to 0.854045/0.881268m; GT27/54 confidence remains far below
0.2. Fresh baseline assignment/miss/FP indices reproduce exactly. Reject the
calibration switch and further expansion of this probe; this is targeted,
GT-informed diagnostic selection, not an independent held-out calibration test.
Evidence: `artifacts/tracking-take8-vehicle2-benchmark-extrinsics-probe-20261012/`.

A complementary cached detector-only evaluation separates ego/reference-frame
disagreement from source detector geometry. GT is recovered into native LiDAR
coordinates through inverse reference pose, predictions use actual model-native
coordinates, and the virtual native camera uses the original fixed extrinsic.
K, score 0.2, class-aware 2m XYZ, 16px and symmetric native54 scope remain declared;
camera eligibility and matching can differ from the primary world evaluation.
There is no new inference, tracker, GT time warp, pose change or primary-score
replacement. Original world assignments/counts and source hashes reproduce exactly.

Take8/V3 native-front gives 830/194/417 TP/FP/FN, with 1247 eligible observations
versus 1246 in the primary world score; native XY/Z RMSE is 0.9941/0.2096m. Its
12/13 ever-detected condition still fails. All54 gives 4058/373/729 on the same
4787 GT observations, native XYZ 0.6820m versus primary world 0.8761m. Take2/V3
native-front gives 350/33/94 on 444 GT observations versus 443 world, native XYZ
0.8408m, with the same 14/15 ever-detected failure. Fixed original pairs are the
cleaner height comparison: Take8's 733 pairs have native Z 0.1790m versus ENU
1.0513m; Take2's 343 pairs have native Z 0.3297m versus ENU 0.6534m. These are
coordinate/evaluation diagnostics, not improvements to inference or tracking,
independent physical GT, or permission to replace scored heights.
Evidence: `artifacts/tracking-v3-native-detector-only-evaluation-20261012/`, including
all eligibility/matching differences, rigid recomposition guards and source pins.

### Take 13/V1: position check and remaining detector misses

The 300 fresh forwards cover native indices 85–384, frames 552085–552384,
from the earliest strict-qualified start. All 296 accepted and four propagated
poses are below 0.1m: XYZ reference RMSE 0.040580m, maximum 0.068021m.
Independent raw GNSS/sensor composition reproduces the retained reference and
archived poses, including V1 mounting correction and excluding Take-3 bias.
Orientation RMS is 0.310993 degrees. This verifies retained recipe agreement,
not independent absolute accuracy. Current declared annotations place Take13
in validation; historical checkpoint training membership remains unverified.

Front detections give 397/134/399 TP/FP/FN (74.76% precision, 49.87% recall),
tracks 553/304/243 (64.53%/69.47%). Both identity modes give IDF1 0.515426,
12 switches and 18 fragments. Only 15/17 eligible GT IDs are ever detected:
GT76 is 0/20 and GT97 is 0/43. Detector ENU XY/Z RMSE is 0.984594/0.443426m;
tracks give 1.026816/0.518420m. Horizontal accuracy is still inadequate, so
position agreement does not justify treating the remaining error as vertical
or changing scored heights. Ground placement remains explicit, display-only.

The labelled GT76 ordinals 104–123 video and PNG223 for GT97 retain original
poses. Fixed diagnostic captures 113/223 have fully on-screen GT rectangles,
9/0 LiDAR points inside their GT cuboids, and nearest raw proposal confidence
0.0375/0.000636. These examples are not explained by partial off-screen boxes.
Evidence: `artifacts/tracking-take13-vehicle1-native-pilot-20261012/`.

### Camera boundary and cooperative source controls

Take8/V3 GT67 has strong proposals at both eligible captures 164/165: scores
0.776/0.743 and center errors 0.415/0.494m before camera filtering. Their boxes
project fully left of the image, whereas GT has a thin left-edge intersection.
All corner depths exceed 6.95m; this is a projection-boundary disagreement,
not a near-plane failure or absent proposal. Partial off-screen boxes remain
eligible under the original clipped-area gate. Take8/V2 GT11/54 examples instead
have no primary proposal within 2m despite fully on-screen GT rectangles.
Evidence: `artifacts/tracking-take8-front-boundary-audit-20261012/`.

A rejected cached three-agent source prototype uses 192 common Take8 frames
537438–537629 with identical cold tracker starts and unchanged host evaluation.
It preserves whole source poses/dimensions/confidence and uses GT only downstream.
Front detection recall rises 60.77% to 98.95%, but FP rises 38 to 427; track FP
rises 78 to 739, switches 14 to 85, and IDF1 falls 0.727067 to 0.610208.
Common-match horizontal errors also worsen. Extra sensor observations change
observability; this is not a single-detector improvement or a comparison with
the full 300-frame pilot. Peer headers are ahead by up to 22.330ms, requiring
offline batch waiting. Reject adoption under the frozen loss/FP/identity/error
gates. No production settings, calibration or heights change.
Evidence: `artifacts/tracking-take8-cooperative-source-probe-20261012/`.

Its downstream failure audit reproduces all 7988 source observations and 119
ambiguous components exactly, with current source-pose matrices unchanged and
inverse-coordinate roundtrip error below 2.2e-14m. Of 427 detector FP assignments,
192 are within 2m of an already matched GT object, 225 lack any host-native GT
within 2m even before FOV filtering, and ten lie near GT outside the front scope.
For tracks the corresponding counts are 433/290/16. Missing annotation, capture
epoch and optical visibility remain uncertain; these labels do not establish
physical object absence. Weak contamination prevents 29 otherwise valid strong
cliques from deduplicating (30 excess strong observations), but 71 ambiguous
components contain only strong observations. Weak contamination therefore cannot
explain the full FP increase. Detailed provenance and categories are retained in
`failure-audit/`; original outputs and rejection remain unchanged.

A separately frozen consensus contrast preserves all 2911 host observations
exactly and admits only 14 peer extras from unambiguous strong peer-agent pairs.
Weak peer observations, peer singletons and ambiguous peer components cannot
introduce extras. GT-free source provenance and functional controls pass.
On the same cold 192-frame window, detections change 581/38/375 to 587/42/369
TP/FP/FN; tracks change 732/78/224 to 735/92/221. No matched GT observations
are lost, and common-match XY/XYZ errors do not worsen, but switches rise
14 to 19 and FP increases in both stages. Reject this variant under the
unchanged gates; no adoption, further fusion expansion or production change.
Evidence: `artifacts/tracking-take8-cooperative-consensus-probe-20261012/`.

The source-bound coverage ledger now records nine completed comparable sequences,
2841 evaluated frames, 30 qualified sequences still pending and 959 unscored
tails within completed sequences. Across all 42 sequences, 39 have strict
qualified starts; Take4/V3 and Take9/V1 have no accepted frame, and Take7/V3's
best accepted XYZ error is 0.116225m. Gates remain unchanged. Legacy Take1
results are listed separately because their replay policy differs.
Evidence: `artifacts/tracking-sequence-coverage-ledger-20261012/` and the updated
`artifacts/tracking-native-probe-summary-20261012/` nine-sequence comparison.

### Take 13/V2: unchanged replay and full-window XYZ check

The next native pilot covers 300 fresh forwards at indices 29–328, frames
552029–552328, leaving 72 of 401 native frames after this window. Independent
V2 raw composition/reference/archive and strict 574-tensor/source-normalization
controls pass; all 60000 raw proposals are retained. XYZ reference RMSE is
0.046114m, maximum 0.087014m, with zero errors >=0.1m across 297 accepted and
three propagated frames. Orientation RMS is 0.488298 degrees. No V1 correction,
Take-3 bias, height correction or calibration change is applied.

Front detections give 250/22/121 TP/FP/FN (91.91% precision, 67.39% recall),
tracks 264/34/107 (88.59%/71.16%). Both identity modes are identical: IDF1
0.789238, zero switches and zero fragments. Ten of 12 eligible GT IDs are ever
detected/tracked; GT20 is 0/1 and GT37 is 0/75. Detector ENU XY/Z RMSE is
0.733293/0.290107m, tracks 0.782318/0.282532m. Track signed vertical mean is
-0.01454m. Horizontal error and missed detections remain material despite good
XYZ position agreement, so grounding is not adopted as a tracking correction.
Logged camera-minus-LiDAR phase is -0.208 to +3.719ms, with ego-speed-times-skew
maximum 0.005293m; this does not certify exposure or annotation capture epochs.
Take13 is in the current declared validation split, with historical checkpoint
annotation membership unknown. GT remains downstream evaluation only.
Evidence: `artifacts/tracking-take13-vehicle2-native-pilot-20261012/`.

With this terminal result, the ledger and consolidated primary comparison now
contain ten completed sequences and 3141 frames. There are 29 qualified sequences
pending and 1031 unscored tails within completed windows (959 plus this
sequence's 72). The valid nine-sequence ledger snapshot
is preserved byte-for-byte; the new row is counted only after its terminal
inference/source/scoring receipt passed. This remains partial 42-sequence coverage.

The subsequent GT37 camera audit changes the failure interpretation: all 75
originally eligible observations have a strong proposal within 2m before FOV
filtering and a nearby track, but both are rejected because at least one box
corner crosses the 0.1m near plane. At ordinal 239 the GT minimum depth is
0.166m, the prediction minimum is -0.551m, confidence is 0.647 and center error
0.424m. There are 8891 raw LiDAR points inside the GT box. The GT rectangle
partly intersects the lower-right image; this is not detector absence or sparse
support. Original statistics remain bound to their declared rejection policy.
Evidence: `visualization/GT37-boundary-report.json` within the Take13/V2 root.

### Take 13/V3: short remaining window

The requested 300-frame pilot clamps to the 44 available captures at indices
357–400, frames 552357–552400. All 44 fresh forwards/8800 raw proposals and
strict-load/source controls pass. XYZ reference RMSE/max is 0.042708/0.063512m,
zero >=0.1m, across 40 accepted and four propagated frames; orientation RMS is
1.773661 degrees. Front detections give 99/43/66 TP/FP/FN, tracks 106/59/59;
IDF1 is 0.587879 with one switch and three fragments in both identity modes.
All seven eligible GT IDs are detected at least once in this short window,
but GT94 detects only 3/39 and tracks 6/39. Stronger accuracy targets fail.

Detector ENU XY/Z RMSE is 0.740440/0.957066m; tracks give 0.779762/0.966338m.
On the same original pairs, native sensor Z residuals are 0.347744/0.341181m,
showing a substantial ego/reference contribution to world height disagreement.
This is an evaluation diagnostic, not a pose/height correction or detector
improvement. Original poses, timestamps, heights and calibration remain unchanged.
Evidence: `artifacts/tracking-take13-vehicle3-native-pilot-20261012/`.

Current source-bound ledger and summary coverage is 11 completed sequences /
3185 frames, with 28 qualified sequences pending and 1031 unscored tails in
completed windows. The valid ten-sequence snapshot is preserved unchanged.

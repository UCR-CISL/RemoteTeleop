# CooperScene tracking pilot

Current coverage is19 completed cases/6716 captures, with20 qualified sequences
pending and241 remaining tail frames. All three Take14 agent windows now use
the fixed earliest-qualified policy. V2's401 and V3's305 XYZ position errors
are all below0.1m; V1 retains81 exceptions. Detection/tracking coverage targets
remain unmet. On V3's same651 matched detector observations, nativeXY/Z error
is0.935/0.245m versus worldENU XY/Up0.936/0.804m: height disagreement includes
substantial ego/reference rotation effects. Sensor-relative scores are a separate
diagnostic, not an algorithm gain or a new localization baseline. Original
scored heights remain unchanged; grounding is explicit and rendering-only.
The current19-case partial-camera comparison is
`artifacts/tracking-native-probe-summary-current19-6716-clipped-20261012/`:
detection precision/recall87.25%/80.93%, tracking81.96%/87.80% across29861
eligible GT observations. Counts are pooled; IDF1 remains per case. The prior
18-case and19-case/full501/full758 snapshots are preserved unchanged. This uses
near-plane clipping and permits partly offscreen boxes. The historical unclipped
comparison is preserved at
`artifacts/tracking-native-probe-summary-current19-6716-20261012/`.
These populations differ; the scoring change is not a model improvement.
Take3/V1 now covers501 frames: every XYZ reference error is below0.043m,
detection horizontal/vertical RMSE0.381/0.150m, and35/36 eligible IDs ever
detected. GT105 remains missed despite a five-frame full-width input probe.
The new full132-frame Take3/V2 window has detector/track horizontal RMSE
0.442/0.406m, but retains22 XYZ reference errors >=0.1m, mainly during its final
registration-rejected propagation stretch. GT89 remains missed0/11. It is a
declared training-split diagnostic, not a held-out accuracy result.
Take3/V3's full461-frame window detects all31 eligible IDs at least once, with
detector/track horizontal RMSE0.322/0.324m and every XYZ error below0.1m. Native
same-pair vertical error is0.097m; larger worldUp error includes reference-pose
orientation disagreement. An explicit frozen-profile display comparison raises
cars without establishing visual improvement, so no grounding default changes.

The full Take2/V1 run covers all 864 frames from the fixed
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

The earlier controlled 300-frame Take 3/V1 probe detected all 16 geometric-front GT
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
intersect at least16 pixels² of the original image. HistoricalFalse requires
all corners ahead of the0.1m near plane; separately labelled explicit-clipping
scores clip edges at that plane. Both allow partly off-screen boxes and do not
test optical occlusion. Matching uses class-aware, maximum-cardinality
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

### Explicit near-plane clipping and tracked render prompts

`project_vehicle_box(..., clip_near_plane=True)` clips the 12 cuboid edges to
the positive 0.1m near plane before projecting their retained vertices. The
evaluator exposes the same explicit option and records it in visibility policy;
its default remains the historical rejection policy. ROS BEVFusion tracked
render prompts explicitly enable clipping, so partially visible predictions
reach the renderer with unchanged track IDs, poses, dimensions and confidence.
No grounding, detector/tracker update or source-calibration change is involved.
Analytic projection/evaluator/render tests pass locally, and new clipping checks
plus the full 56-test ROS adapter suite pass on Lambda and Gamma after rsync.

One cached Take13/V2 comparison uses the same 300 frame records and retains
the literal original assignments/metrics under the default policy. On the fixed
371 original GT observations, clipping recovers all 75 GT37 matches: detections
250 to 325, tracks 264 to 339, with no previous match loss or ID replacement.
The full clipped scope also admits 214 additional GT observations, giving 585
total. Its detections are 468/116/117 TP/FP/FN (80.14% precision, 80.00% recall),
tracks 501/238/84 (67.79%/85.64%), IDF1 0.7568, zero switches/fragments.
These totals have a different eligibility denominator and do not establish a
detector or tracker accuracy improvement. GT37 is now detected/tracked 209/211;
GT20 remains the only never-detected ID at 0/1. Newly eligible GT100 and GT46
have poor detection coverage, 1/40 and 8/38. The paired ordinal-239 PNG shows
the original filtering miss alongside the same unchanged detection now included.
Evidence: `artifacts/tracking-take13-vehicle2-nearplane-clipped-evaluation-20261012/`.

The actual Take13/V2 ordinal-239 prediction also passes a headless primitive
render on Gamma: 1503 asset pixels, all with finite positive depths
3.475322–4.499870m. A fully behind-camera control yields zero pixels. The PNG
shows a partial lower-right asset without a screen-filling artifact. Original
pose, dimensions, height and K are preserved; no map, ground placement or SAM
is involved. This is one rendering case, not optical visibility certification
or a scene-depth occlusion test. Existing CloudXR processes remain unchanged.
Evidence: `artifacts/tracking-take13-vehicle2-nearplane-primitive-render-20261012/`.

### Same clipped camera policy across completed recordings

A cached comparison now covers all 11 completed recordings / 3185 frames.
For every recording, the original policy reproduces literal assignments and
core metrics; stored frames remain byte-identical under both policies. The
clipped policy loses no previous matched GT observations and replaces no
previous matched prediction IDs in either detection or tracking. GT eligibility
also expands, so each report separates fixed original observations from new
ones. No inference, tracker replay, pose/height change or GT input occurs.

|Take/vehicle|Original / clipped GT observations|Clipped detection TP/FP/FN|Clipped track TP/FP/FN|
|---|---:|---:|---:|
|2/1|2283 / 3110|2513 / 455 / 597|2908 / 747 / 202|
|2/2|1044 / 1344|1080 / 185 / 264|1233 / 390 / 111|
|2/3|443 / 519|416 / 64 / 103|430 / 109 / 89|
|3/1|497 / 891|851 / 146 / 40|868 / 192 / 23|
|4/2|256 / 299|242 / 7 / 57|264 / 6 / 35|
|8/1|1288 / 1691|1137 / 248 / 554|1428 / 327 / 263|
|8/2|1548 / 1809|1471 / 65 / 338|1502 / 84 / 307|
|8/3|1246 / 1558|1045 / 315 / 513|1279 / 555 / 279|
|13/1|796 / 1092|692 / 139 / 400|849 / 308 / 243|
|13/2|371 / 585|468 / 116 / 117|501 / 238 / 84|
|13/3|165 / 213|140 / 47 / 73|145 / 66 / 68|

Existing never-detected IDs such as Take2 GT208/130, Take4 GT77 and Take13
GT76/97 remain; newly eligible IDs also expose failures. Take8/V3 GT67 remains
fully off-screen in its predictions rather than crossing the near plane, so
clipping does not recover it. This corrects the camera filtering scope, not
model accuracy, and does not satisfy the full tracking target. Per-case IDF1,
switches, fragmentation, coverage and ENU errors on original/common/new matches
are retained in the batch reports. Comparisons across different sensor windows
remain conditional; overlapping agent observations are not independent samples.
Evidence: `artifacts/tracking-completed-probes-nearplane-clipped-evaluation-20261012/`
and `artifacts/tracking-completed-probes-nearplane-clipped-independent-20261012/`.

### Cross-agent GT geometry limits cooperative interpretation

Over the same 192 Take8 captures, shared GT IDs in the original prepared lists
have different world geometry after each agent's own reference mapping:

|Agent pair|Shared observations|ENU XY/Z RMS|XYZ RMS / maximum|XYZ >2m|
|---|---:|---:|---:|---:|
|V1–V2|1868|0.476 / 0.094m|0.485 / 0.763m|0|
|V1–V3|1748|1.275 / 0.790m|1.500 / 1.942m|0|
|V2–V3|1011|1.601 / 0.987m|1.881 / 2.516m|340|

LWH differences are zero. Twelve selected native YAML files have exactly equal
shared global object geometry; their prepared GT matrices reproduce
`reference_W_lidar @ inverse(native_lidar_pose) @ native_global_GT` exactly.
This confirms that own-reference mapping introduces the observed discrepancies.
All-192-frame literal YAML equality and annotation capture epochs are not
assumed. The discrepancy can exceed the 2m association gate before any detector
error, limiting cooperative comparisons against one host's reference GT. It
does not explain all FP assignments or establish which geometry is physically
correct. No common GT frame is fitted or substituted, and GT never enters
model/tracker inputs. Retained localization, poses and original scores remain
unchanged; further work continues on single-agent evaluation.
Evidence: `artifacts/tracking-take8-cross-agent-GT-geometry-audit-20261012/`.

### Take 14/V1: full remaining window and independent XYZ recheck

The 235 available captures at indices 166–400, frames 556226–556460, complete
235 fresh forwards/47000 raw proposals with strict 574-tensor/source controls.
No earlier start or later-frame filtering is used. All 216 accepted and 19
propagated poses remain. An independent raw-GNSS/F1/antenna/recorded-extrinsic/
V1-mount composition and archived-pose reconstruction reproduces the exports:
XYZ RMSE 0.087123m, maximum 0.114592m, with 81 errors >=0.1m (69 accepted,
12 propagated). ENU horizontal/Up RMSE is 0.051721/0.070109m; orientation RMS
is 0.282526 degrees. A qualified start does not guarantee sub-0.1m agreement
throughout replay. This is retained-recipe agreement, not independent absolute GT.

|Camera policy|GT observations|Detection TP/FP/FN|Track TP/FP/FN|IDF1 reacquisition 0→2s|Switches 0→2s|
|---|---:|---:|---:|---:|---:|
|Historical rejection|1088|860/48/228|919/116/169|0.7763→0.8130|6→4|
|Explicit clipping|1323|1077/54/246|1141/129/182|0.8068→0.8369|6→4|

Clipping adds only 235 GT38 observations; all original matches remain exact.
Clipped detector precision/recall is 95.23%/81.41%, tracks 89.84%/86.24%.
Detector ENU XY/Z RMSE is 0.843847/0.191955m historically and
0.967945/0.177471m with clipping. Matched sets differ, not detector parameters.
GT79 remains never detected in five observations; clipped coverage is 21/22
ever-detected IDs. At ordinal 19 no raw proposal is within 2m, and its exact
GT box contains zero raw points; padding by 0.25m includes nearby points.
This does not establish optical visibility or annotation validity.

The decoded paired 20-frame clip at ordinals 11–30 and PNGs 18/49 preserve
original geometry/heights: GT49 changes baseline ID11→13→18, while the registry
retains ID11. Logged camera-minus-LiDAR phase is +47.088–49.317ms; ego-speed-
times-skew reaches 0.494m, without certifying exposure/annotation epochs.
Grounding remains explicit and display-only; horizontal detection errors and
full-window XYZ exceptions remain unresolved.
Evidence: `artifacts/tracking-take14-vehicle1-native-pilot-20261012/`, independent
`artifacts/tracking-take14-vehicle1-xyz-independent-recheck-20261012/`, and the
standalone PNG/PDF in `artifacts/tracking-take14-vehicle1-localization-error-plot-20261012/`.

The ledger now records 12 completed cases /3420 frames, 27 qualified sequences
pending and 1031 unscored tails. The previous ledger snapshot and frozen
11-case summary remain unchanged. The new historical-policy comparison is
`artifacts/tracking-native-probe-summary-current12-20261012/`; clipped scopes
are separately labelled rather than mixed into that table.

### Remaining detection failures and GT identity cautions

The frozen eleven-case clipped catalogue has 18 never-detected take/agent-ID
cases over 180 observations: 23 have strong nearby proposals fully outside the
camera, three have nearby scores 0.1–0.2, 68 only scores below 0.1, and 86 have
no raw proposal within the declared world-reference 2m gate. There are no strong
camera-eligible association-conflict cases. World-gate failure is not proof of
physical detector absence; reference orientation and annotation timing matter.
Evidence: `artifacts/tracking-clipped-never-detection-catalogue-20261012/`.

Take8 IDs11/13/16 share complete, nonzero geometry in overlapping early captures
upstream in tracked, refined and native labels; the preparer preserves it exactly.
Source type6 maps to motorcycle but becomes Car in refined/native records. No
invalid-placeholder convention or distinct physical identity is established;
global and per-agent numeric ID namespaces also differ. Separately, sampled
GT38 (Take14/V1), GT29 (Take8/V2) and GT28 (Take2/V2) contain the host sensor's
horizontal footprint, suggesting host-vehicle labels without explicit ID proof.
These ambiguities limit physical interpretation of raw per-ID coverage. No GT
IDs are deduplicated, excluded, relabelled or supplied to model/tracker inputs.
Evidence: `artifacts/tracking-take8-early-GT-identity-audit-20261012/` and
`artifacts/tracking-nearplane-GT-agent-identity-audit-20261012/`.

A single cached Take14/V1 two-hit confirmation contrast reproduces the existing
one-hit registry baseline exactly before scoring. Track TP/FP/FN changes
1141/129/182 to 1123/101/200: precision rises 89.84%→91.75%, switches 4→1,
but 18 matched GT observations are lost. Common-pair XY/XYZ errors worsen slightly,
despite an improved aggregate error after dropping harder matches. Reject under
the unchanged coverage/FP/identity/error gates; confirmation defaults remain.
Evidence: `artifacts/tracking-take14-vehicle1-confirmation-two-hits-probe-20261012/`.

### Take14/V2: localized window passes, persistent detection misses remain

All 300 fresh forwards at indices 0–299/frame556060–556359 complete with strict
574-tensor/source controls and 60000 raw proposals. An independent raw V2
composition and manual accepted/propagated reconstruction verifies all 298
accepted and two propagated poses: XYZ RMSE 0.051328m, maximum 0.070917m,
zero >=0.1m; ENU horizontal/Up RMSE 0.046457/0.021825m. No V1 correction or
take-3 bias applies. This remains conditional retained-recipe agreement.

Historical detector TP/FP/FN is 689/173/199 (precision/recall 79.93%/77.59%);
tracks 690/302/198 (69.56%/77.70%), IDF1 0.628723, zero switches and four
fragmentations. Reacquisition 0/2s has identical geometry and metrics here.
All 15 eligible IDs are detected at least once, but GT69 has only 1/39 detections
and GT74 4/78 (tracks 6/78); stronger per-ID coverage remains unmet.
Detector ENU XY/Z RMSE is 0.750926/0.117071m; under 20m it is 0.160/0.051m,
while 20–40m and >=40m horizontal errors are 1.079/1.263m. Grounding cannot
resolve these horizontal errors and remains display-only.

Explicit clipping retains all original 689/690 matched pairs and adds three
GT49 observations. Detector counts become 692/207/199, tracks 693/445/198,
IDF1 0.585510. Added unmatched predictions reflect the expanded scope, not a
changed model. A GT-free exact tracker replay identifies 355 current-observation
and 90 coasted unmatched track observations; all lie outside 2m of supplied GT.
The ordinal-0 camera overlay nevertheless shows a real foreground sedan at one
high-confidence unmatched prediction. Counts describe the supplied-GT protocol;
annotation completeness is under investigation before calling them hallucinations.
GT, source poses, heights and calibration remain unchanged.

Evidence: `artifacts/tracking-take14-vehicle2-native-pilot-20261012/` (including
labelled PNGs and decoded 20-frame video),
`artifacts/tracking-take14-vehicle2-xyz-independent-recheck-20261012/`, and
`artifacts/tracking-take14-vehicle2-fp-lifecycle-audit-20261012/`.
The ledger and new `artifacts/tracking-native-probe-summary-current13-20261012/`
record 13 completed cases/3720 frames, 26 qualified sequences pending and 1132
unscored tails. Previous summaries and the twelve-case ledger snapshot are preserved.

The bounded miss audit finds no strong-camera exclusion or association-conflict
cases for GT69/74. GT69 has one match, ten weak-only (0.1–0.2), fifteen below-0.1
and thirteen outside-world-2m observations; GT74 has four matches, eleven
weak-only, thirty-five below-0.1 and twenty-eight outside-world-2m. Representative
missed boxes have LiDAR support, so absence of points does not explain every miss.
Ordinal0 foreground proposal0 is within 0.081906m of an upstream own-agent
annotation omitted from global/native labels. Native IDs2/17 are preserved exactly
by preparation. Across eight selected captures, persistent own trackedID1 is
initially omitted, then lies within 0.022m of nativeGT34 and laterGT84. This
suggests annotation omission/identity fragmentation, without proving physical
continuity or exact transition times. No GT is supplemented or remapped.
Evidence: `artifacts/tracking-take14-vehicle2-targeted-miss-audit-20261012/`.

A four-capture paired detector-input test uses the actual supplied
`scene14_agent2` benchmark extrinsic (relative Rx -3 degrees), eight fresh
forwards, strict574 and unchanged RGB/XYZI/K/localization/scoring camera.
Cached baseline assignments reproduce exactly. Front TP/FP/FN stays9/2/3,
all54 stays31/9/12, with no gained/lost matches and slightly worse position
errors. GT74@239 and GT69@280 confidence remains below the admission thresholds.
Reject this candidate; calibration defaults remain unchanged. These targeted
development captures are not held-out or a full benchmark reproduction.
Evidence: `artifacts/tracking-take14-vehicle2-benchmark-extrinsics-probe-20261012/`.

The pinned runtime's actual CPU-only ImageAug3D transform confirms native resize
0.3 and crop `[32,104,544,360]` of the resized576x360 image, producing256x512.
Thus about107 original pixels per horizontal edge fall outside model RGB.
Using continuous pixel-cell extents, eight of39 full-front-eligible GT69 boxes
(seven misses, one source match) and41 of78 GT74 boxes (all misses) lie wholly
outside the model crop. Other misses remain inside. The checkpoint-consistent
crop is not an integration bug, and LiDAR may detect objects outside RGB.
The report distinguishes pixel-center extents from subpixel edge overlap and
scales the original16px area gate to1.44 model pixels. No scores/poses/GT change.
Evidence: `artifacts/tracking-take14-vehicle2-model-image-crop-audit-20261012/`.

A subsequent four-capture/eight-forward full-width preprocessing contrast changes
only native ImageAug3D resize0.3→512/1920, preserving256x512 final dimensions,
weights, frustum, calibration, source poses, points/images and original scoring.
The actual operator brings GT69@261 x1914–1919 into model x510.400–511.733.
Both cached baseline assignment scopes reproduce exactly. Front TP/FP/FN stays
8/2/4, all54 stays31/8/12; no matches are gained or lost. Common front XYZ RMSE
changes0.445297→0.444154m, but target confidence stays below admission thresholds.
Reject augmentation changes or expansion; RGB cropping is not the sole failure.
This is a targeted development contrast outside native checkpoint augmentation,
not a new production baseline. Evidence:
`artifacts/tracking-take14-vehicle2-fullwidth-camera-probe-20261012/`.

### Take14/V2: filter-output lag contrasts

Two cached300-frame GT-free output contrasts retain exact latent Kalman states,
covariance, association, lifecycle, dimensions, rotations, scores and identities.
Both0/2s registry baseline controls reproduce every original output and clipped
assignment before downstream scoring. No inference or production changes occur.

Current-measurement XYZ output changes tracks693/445/198→710/435/181, gains29
GT observations but loses12, and raises fragmentation4→19. Common681-pair ENU
XY improves0.746998→0.705317m; aggregateXYZ worsens0.782847→0.785336m.
An independently verified second contrast applies only one fixed half-horizontal
innovation using explicit retained-ENU up `G.R[:,2]`, preserving filtered Up
exactly and leaving coast extrapolation unchanged. It changes counts to705/438/186,
gains14 but loses2 (GT30@83, GT74@255), fragmentation4→6. Common691-pair XY
improves0.768348→0.743092m, but aggregateXYZ becomes0.787977m.
On the same653 GT/prediction triplets and actual capture intervals, display
acceleration RMS rises8.297→22.349m/s²; GT-residual acceleration rises
23.710→31.729m/s². These describe supplied-GT-conditioned motion, not pure
sensor jitter or physical GT acceleration. Neither contrast is promoted:
smoothing adds some avoidable lag, but these output corrections lose coverage
and reduce continuity/smoothness. Original filtered centers remain the baseline.
Evidence: `artifacts/tracking-take14-vehicle2-observed-center-probe-20261012/`,
`artifacts/tracking-take14-vehicle2-observed-center-independent-20261012/`,
`artifacts/tracking-take14-vehicle2-half-horizontal-innovation-probe-20261012/`,
and `artifacts/tracking-take14-vehicle2-half-horizontal-independent-20261012/`.

### Take14/V2: complete401-capture fixed-policy sequence

The new full401 artifact reuses the exact original300 metadata/payloads,
response/raw bytes and continuous tracker state, preparing only101 new captures
and running101 fresh forwards/20200 raw proposals. The combined sequence spans
indices0–400/frames556060–556460, with no tail. Original300 output fields and
both camera-scope assignment/core-metric prefixes reproduce exactly. No rejected
calibration, crop or filter-output contrast is used.

Independent raw V2 recipe composition and manual archived reconstruction confirm
398 accepted and three propagated poses: XYZ RMSE0.052512m, maximum0.073359m,
zero >=0.1m; ENU horizontal/Up RMSE0.047676/0.022011m. Maximum propagation
age0.100114s; orientation RMS0.499001 degrees. All101 tail poses remain below
0.1m. These are conditional retained-recipe errors, not independent physical GT.

|Camera scope|Detection TP/FP/FN|Track TP/FP/FN|Detection P/R|Track P/R|IDF1|
|---|---:|---:|---:|---:|---:|
|HistoricalFalse|1174/236/329|1197/385/306|83.26%/78.11%|75.66%/79.64%|0.680713|
|Explicit clipping|1177/270/390|1200/528/367|81.34%/75.11%|69.44%/76.58%|0.639150|

Both registry settings retain identical metrics here: zero switches and18
fragmentations. Clipped detector ENU XY/Z RMSE0.786423/0.156913m; tracks
0.844363/0.162190m. Historical detector horizontal errors are0.292/1.036/1.285m
in <20/20–40/>=40m bands. Full-sequence clipped coverage is24/25 ever-detected
IDs; stronger per-ID targets remain unmet. No source or scoring height is grounded.

NewGT126 is missed in all61 clipped-eligible observations and is absent from
historicalFalse scope. A bounded own-raw catalogue finds60 only-below0.1 nearby
proposals and one outside the world2m gate. Its native reference XY range is
0.171–0.677m. In three checked native captures, the camera lies inside its3D
annotation box, giving full-image clipped rectangles; this does not establish
ordinary frontal-target optical visibility or a verified self-ID convention.
GT and nearest proposals contain zero exact/padded scan points at two sampled
captures. No GT is excluded, unioned, remapped or supplied to tracking inputs.
Evidence: `artifacts/tracking-take14-vehicle2-full401-native-20261012/`,
`artifacts/tracking-take14-vehicle2-full401-xyz-independent-20261012/`, and
`artifacts/tracking-take14-vehicle2-GT126-miss-audit-20261012/`.

The ledger now contains13 cases/3821 captures,26 qualified sequences pending,
1031 tails, with the previous13-case/3720 snapshot preserved byte-exact.
`artifacts/tracking-native-probe-summary-current13-full401-20261012/` replaces
only the Take14/V2 comparison row; the other12 rows and older summary roots remain
unchanged. This completes this sequence, not the full tracking goal.

### Primitive raster failure and car-only back-face correction

An actual Gamma20-frame clip at Take14/V2 indices0–19 preserves original
localized object poses/heights, calibration, predicted IDs and GT wireframes,
with SAM and grounding off. It compares camera/no-scene-depth composition with
Gaussian scene depth and shared asset depth. Ordinal0 fartrack5 (~50m) has314
mesh pixels:207 scene-occluded and107 visible. At ordinal19 it has304 pixels,
200 hidden by scene and104 incorrectly hidden behind a near-camera primitive.
Track8 at ~1.1m fills144000 pixels at12 and108107 at19, making the clip unusable.
The depth arithmetic passes, but that does not establish correct physical
occlusion. The original clip remains preserved as failure evidence.

Isolated same-input raster controls find actual perspective-camera `znear=None`.
Explicit0.1m near-plane clipping alone does not resolve the failure. Back-face
culling does: track8 contributes zero pixels at12/19 while foreground/farther
cars recover. The camera lies outside the largest closed car-body component by
solid-angle test; whole primitive geometry contains open surface components.
No self-GT convention or detection exclusion is inferred from these controls.
PyTorch3D culling depends on exterior triangle winding; see its
[rasterizer contract](https://pytorch3d.readthedocs.io/en/latest/_modules/pytorch3d/renderer/mesh/rasterizer.html).

Actual GPU exterior controls cover all10 supported kinds from six directions.
Car alpha/depth/RGB is byte-exact in all six, including normal Take14/V2 cars
and the prior Take13/V2 near-plane case. Blanket culling changes some other
templates' open details (construction underside loses5.91% of sampled pixels),
so it is not adopted. Commit5103645 adds an optional default-False raster
argument and enables it only for primitive `car`; other templates and general
mesh rendering retain two-sided behavior. Focused40 tests pass locally, Lambda
and Gamma. Both remote isolated worktrees have identical source/test hashes;
remote HEADs and original CloudXR processes are unchanged.
Evidence: `artifacts/tracking-take14-vehicle2-gamma-primitives-20261012/`,
`artifacts/tracking-take14-vehicle2-gamma-raster-controls-20261012/`, and
`artifacts/tracking-primitive-backface-regression-20261012/`.

The final car-only production path renders and decodes a new20-frame clip in
`artifacts/tracking-take14-vehicle2-gamma-primitives-corrected-20261012/`.
All original frames, RGB, poses/heights, calibration and GT labels remain exact;
old failure artifacts remain unchanged. Full-screen fill disappears at12/19,
but front-facing near-track8 geometry still covers3057/3953/49807/16307 pixels
at15/16/17/18. This remaining partial coverage is retained and labelled, not
hidden by grounding or object filtering. A root review also decodes all20 and
inspects frame17. The fix addresses back-face artifacts, not detector accuracy
or certified optical occlusion. Deployment receipts and source hashes are in
`artifacts/tracking-primitive-car-backface-deployment-20261012/`.

### Take14/V3: complete305-frame window and separated detector errors

All305 available captures from index96/frame556156 through400/frame556460
complete305 fresh forwards/61000 raw proposals/zero warmups with strict574
original native frustum and baseline calibration/augmentation. Independent raw
V3 quaternion/antenna/recordedE/G composition and archived reconstruction match
exactly:305 accepted, no propagated, XYZ RMSE0.032746m/max0.058454m,
zero >=0.1m. ENU horizontal/Up localization RMSE0.020113/0.025841m;
orientation RMS1.754216 degrees. No V1 mounting or take3 bias applies.

|World-reference camera scope|Detection TP/FP/FN|Track TP/FP/FN|IDF1 0→2s|Switches 0→2s|Fragmentations|
|---|---:|---:|---:|---:|---:|
|HistoricalFalse|460/525/836|521/819/775|0.353566→0.358877|12→10|27|
|Explicit clipping|651/896/836|712/1205/775|0.341363→0.345476|12→10|27|

Clipped coverage is25/26 ever-detected IDs, withGT79 missed0/15. WorldENU
detector XY/Up RMSE0.936376/0.803596m; tracks0.951977/0.842840m.
Logged camera leads LiDAR40.668–42.150ms, without verified annotation epoch.
Source/control receipts: `artifacts/tracking-take14-vehicle3-native-pilot-20261012/`
and `artifacts/tracking-take14-vehicle3-xyz-independent-20261012/`.

An exact fixed-pair decomposition separates pose-rotation, translation and
sensor-relative geometry contributions. For651 original detector pairs, Up RMS
components are0.632221/0.025963/0.243354m; they have cross-terms and are NOT
additive RMS quantities. Native detectorZ RMS is0.245292m. For712 track pairs,
rotation/translation/sensor Up components are0.667392/0.025849/0.240821m;
nativeZ RMS0.243568m. All vectors sum to the original scored errors within
1.6e-14m. Horizontal error remains about0.936/0.952m after the diagnostic
reference substitution. No matches, output poses, heights or scores change.
Evidence: `artifacts/tracking-take14-vehicle3-fixed-pair-error-decomposition-20261012/`
and standalonePNG/PDF `artifacts/tracking-take14-vehicle3-error-separation-plot-20261012/`.

An independently verified sensor-relative supplement uses actual raw native
detections and `inverse(XYZ_W_lidar) @ world_track`, against published native
LiDAR GT geometry. Both GT/prediction camera projections use original baseline
LiDAR-camera extrinsics, with native54m scope and unchanged class/2m/16px gates.
GT never enters tracker/model inputs; all original world tracker controls match
exactly before scoring. Clipped counts become detector909/638/582 and
tracks1099/818/392, IDF1 0.455399→0.466549, switches28→25, fragmentation38.
This changes evaluation representation, eligibility and matched pairs, not the
model or localization. Original651/712 pairs retain nativeXY/Z0.935464/0.245292
and0.951296/0.243568m; all-newly-matched-set errors differ and are separately
reported. Primary world-reference metrics remain unchanged. Evidence:
`artifacts/tracking-take14-vehicle3-native-sensor-relative-evaluation-20261012/`
and `artifacts/tracking-take14-vehicle3-native-sensor-relative-independent-20261012/`.

### Take14 cross-agent GT representation and remainingGT79 caveat

The same235 frames556226–556460 are compared using preparedall54 GT before
FOV. Shared world-reference GT pair XYZ RMS/maximum is1.313/2.312m (V1–V2),
1.346/2.229m (V1–V3),2.119/2.909m (V2–V3). Pair counts exceeding2m are
163/2231,11/3061,977/2544. LWH differences are zero. Nine sampled native YAMLs
share identical global object geometry and reconstruct preparation exactly;
these discrepancies concern own-reference representations, not verified physical
GT error. Agent2→3 LiDAR phase is59.905–61.821ms, with unknown object-label epoch.
V1's81 XYZ exceptions remain. No common reference is fitted or substituted.
Evidence: `artifacts/tracking-take14-cross-agent-GT-geometry-20261012/`.

GT79's15 V3 misses remain under both native/world gates: four have only raw
scores below0.1 nearby, eleven have no raw proposal within2m. Three native-frame
camera/LiDAR PNGs preserve original calibration and show no recoverable strong
proposal. Exact/padded GT scan points are0/0,0/0,34/54 at first/middle/last
sample, so lack of points is not a complete explanation. Optical identity and
occlusion remain uncertain near the station/background image edge. Same-frame
V2 native geometry agrees but is not camera-eligible there. No GT is excluded.
Evidence: `artifacts/tracking-take14-vehicle3-GT79-miss-audit-20261012/`.

The continuity supplement finds nearly linear published global positions over
these15 frames (maximum positional second difference1.18e-5m), constant yaw and
dimensions, and no explicit observed/coasted/interpolated state. The available
historical exporter has a different schema from the mounted records, so it does
not establish their generation provenance. This does not prove invalid GT or
justify excludingGT79; its misses remain in every applicable score.

### Take14/V3 supplied calibration contrasts: no switch

Four fixed captures (ordinals0/87/94/101) receive baseline and supplied-extrinsic
forwards, followed by four fresh full-package forwards using both supplied K and
extrinsic. Baseline outputs are reused byte-exact for the second contrast.
The supplied focal lengths are5% lower. Evaluation camera calibration, GT,
localization, confidence thresholds and association gates remain fixed; only
model calibration inputs change. Strict574 checkpoint loading and native sparse
convolution compatibility checks pass for every run.

At confidence0.2, baseline, extrinsic-only and full-package results all have
native-front TP/FP/FN11/11/12 and all54m52/13/21; confidence0.1 diagnostics also
have unchanged counts. Full-package common front horizontal/XYZ RMSE changes
1.106982/1.125307m to1.101603/1.120447m, about5mm improvement without a recovered
GT observation. GT79 remains missed; its first nearby raw proposal's confidence
decreases, and the middle/last captures have no proposal within2m.
The independently checked full-package contrast does not meet the required
substantial detection improvement, so baseline calibration remains active.
These are targeted four-capture contrasts, not full-sequence calibration claims.
Evidence: `artifacts/tracking-take14-vehicle3-benchmark-extrinsics-probe-20261012/`,
`artifacts/tracking-take14-vehicle3-benchmark-full-calibration-probe-20261012/`,
and `artifacts/tracking-take14-vehicle3-benchmark-full-calibration-independent-20261012/`.

V3 therefore passes the independent retained-recipe XYZ<0.1m check, but detector
horizontal RMSE remains about0.94m on the original fixed pairs. Its large world
Up error also includes the reference-orientation lever arm; native detector Z
RMSE is about0.245m. The condition for adopting grounding has not been met.
Scored poses and heights remain unchanged; any future rendering ground placement
must remain explicit and separate from localization and detection scoring.

The new14-case comparison preserves the previous13 rows and every older summary
root. The ledger's prior13/full401 snapshot remains byte-exact. These checks
separate geometry/reference limits; stronger tracking conditions remain unmet.

### Take14/V3 cached box-overlap and horizontal-error diagnostics

All305 captures/61000 cached raw proposals receive a native sensor-relative
upright-box IoU supplement using the published CooperScene corner/IoU/VOC2010
AP utilities. AP uses per-frame confidence-ranked greedy IoU association and
global confidence ranking; it differs from the primary Hungarian center metric.
Original native54m scope and clipped16px camera eligibility remain fixed.
The front/all54 populations contain1491/5491 GT observations. Independent
checks verify all61000 bottom-center-to-gravity-center conversions exactly,
GT dimensions exactly, analytic BEV/height controls and60 official matching
fixtures. Upright PKL yaw differs from full native-pose heading by at most
0.0593 degrees; this representation distinction is retained, not corrected.

|All-proposal scope|BEV AP .3/.5/.7|3D AP .3/.5/.7|
|---|---:|---:|
|Front camera|30.00% /14.28% /7.67%|23.90% /10.98% /5.02%|
|All native54m|69.65% /51.79% /40.15%|64.53% /45.93% /25.03%|

At the retained confidence0.2, front BEV AP is25.75% /13.07% /7.55% and
3D AP21.21% /10.66% /5.01%. Original651 center-matched detector pairs have
median BEV/3D IoU0.573/0.471; only295 also meet3D IoU0.5. GT79 has raw
overlap-qualified proposals atIoU0.3, but their highest confidence is0.00156,
well below0.1/0.2. No threshold change is adopted. These one-agent subset metrics
are not a reproduction of the full paper benchmark; GT incompleteness and
unverified optical visibility remain limitations.

The same651 pairs have horizontal median0.751m/p95 1.601m and native heading
longitudinal/lateral RMS0.630/0.692m. LWH mean absolute error is0.157/0.059/0.073m;
median heading disagreement is about2.2 degrees. No systematic90-degree axis or
dimension swap is established. Published adjacent-frame GT velocity implies a
41ms constant-velocity horizontal displacement RMS0.294m;501/651 errors exceed
that diagnostic estimate. This is not a rigorous acceleration bound. Unknown
label epoch prevents a timing correction or physical GT claim. The original2m
matching gate also censors the fixed-pair error population.
An outcome-selected, LiDAR-supported example (ordinal180, GT35) has1.946m
horizontal error and16 points inside each original GT/prediction box. Its PNG
labels native coordinates and uncertain optical correspondence explicitly.

GT-free exact tracker replay partitions the original712 matched track pairs:
674 updated at the current capture have horizontal/Z RMS0.937/0.241m;38 coasted
pairs have1.172/0.289m. Different target/range distributions prevent a causal
comparison, but current-observation errors remain substantial. Original local
serial model calls take78.49ms median/86.51ms p95, with367.99ms maximum; this is
not asynchronous replay or end-to-end latency. No poses, GT, calibration,
association, heights, source files or production defaults change.
Evidence: `artifacts/tracking-take14-vehicle3-native-iou-diagnostic-20261012/`,
`artifacts/tracking-take14-vehicle3-horizontal-error-audit-20261012/`,
`artifacts/tracking-take14-vehicle3-track-age-error-audit-20261012/`, and
standalonePNG/PDF `artifacts/tracking-take14-vehicle3-detection-quality-figure-20261012/`.
Independent convention review:
`artifacts/tracking-take14-vehicle3-box-convention-review-20261012/`.

### Take14/V3 fixed weak-confidence correction trial

The exact earlier confidence-scaled weak-correction policy is tested on all305
cached responses, without tuning. Both0s/2s identity controls reproduce all
baseline outputs and assignments; an independent recount verifies all12
scope/variant views. Raw detections and localization remain unchanged; estimated
track Z may change through filtering, with no grounding.

With2s reacquisition, clipped world TP/FP/FN changes712/1205/775 to715/1200/772,
switches10→9 and fragmentation27→24. These aggregate gains hide six lost and
nine gained GT observations; common706-pair native horizontal RMSE worsens
0.947108→0.948092m. Native-relative TP/FP/FN changes1099/818/392 to1100/815/391,
but IDF1 falls0.466549→0.461538, fragmentation38→39, and seven previous matches
are lost. Common1092-pair horizontal RMSE worsens1.238036→1.239416m.
Only30 of674 current-observation world-matched tracks had weak last updates,
so this policy addresses a small part of the measured detector error.
The fixed coverage/common-error gates reject it; production is unchanged.
Evidence: `artifacts/tracking-take14-vehicle3-confidence-noise-probe-20261012/`
and `artifacts/tracking-take14-vehicle3-confidence-noise-independent-20261012/`.

A new per-GT PNG/PDF shows every eligible ID over the same305 captures, with
fresh detector matches, track-only matches, misses and out-of-scope periods
separate. Original world and native-relative panels reproduce651/712/1487 and
909/1099/1491 detector matches/track matches/GT observations exactly.
GT79 remains0/15 in both; the native-relative panel is an evaluation supplement,
not improved model output. Evidence:
`artifacts/tracking-take14-vehicle3-per-GT-coverage-figure-20261012/`.

### Take14/V3 cached offline publication timing

Two305-capture virtual-service traces use the actual public adapter and original
per-call serial model durations, frozen XYZ/calibration/images/points and
registry0/.2birth/.1continuation/1hit/duplicate guard/.5s coast. Snapshot prompts
are evaluated before submitting current metadata, in image→LiDAR→metadata order.
The capture-tick condition polls only at callbacks. The paced condition calls
the production `McapFrameReplay._wait_until`305 times with injected clocks and
3190 sleeps no longer than10ms. It tests this method on a constructorless
instance, not the MCAP parser, network codec, GPU service or full replay launcher.

|Same world camera population|Served/replaced|Track TP/FP/FN|IDF1|Switches/fragments|Median result age|
|---|---:|---:|---:|---:|---:|
|Serial same-capture post-result baseline|305/0|712/1205/775|0.341363|12/27|Not a delayed-publication measurement|
|Capture-tick virtual polling|300/5|697/1151/790|0.338231|14/37|200.001ms|
|Production wait-method virtual polling|303/2|694/1190/793|0.335805|14/29|100.001ms|

Both traces have zero service errors and terminal submitted/result counts agree;
EOF polling publishes no additional evaluated snapshots. Published full-range
prompt totals1850/1886 become1848/1884 after unchanged54m scope. Faster virtual
polling improves freshness and request coverage, but does not improve accuracy.
Maximum virtual result age is400.333ms; saved serial call duration median/p95 is
78.490/86.511ms. These are simulations using measured serial durations, not
measured asynchronous throughput or latency. Empty detector fields yield
placeholder evaluator zeros and are explicitly NOT_EVALUATED; only published
track accuracy is reported. No production or queue setting changes.
Evidence: `artifacts/tracking-take14-vehicle3-offline-adapter-trace-20261012/`
and `artifacts/tracking-take14-vehicle3-offline-adapter-paced-trace-20261012/`.
Independent prompt/source, queue-closure and metric recount:
`artifacts/tracking-take14-vehicle3-offline-adapter-independent-20261012/`.
It verifies both305-frame conditions without a new tracker replay or inference;
neither trace establishes protocol-codec round-trip or network performance.

### Take3/V2 complete qualified132-frame window

All132 available captures from fixed earliest index369/frame497129 through
500/frame497260 complete132 fresh forwards/26400 raw proposals/zero warmups.
Strict574 loading, sparse-convolution compatibility, native frustum and baseline
calibration/augmentation remain fixed. The native501-frame prefix369 is outside
the qualified policy, not discarded based on detection results. Current declared
training includesTake3; historical checkpoint training-file identity is unknown.

|Camera scope|Detection TP/FP/FN|Track TP/FP/FN|IDF1 0→2s|Switches 0→2s|Fragments|
|---|---:|---:|---:|---:|---:|
|HistoricalFalse|581/16/81|617/53/45|0.923423→0.926426|2→1|1|
|Explicit clipping|711/17/82|748/54/45|0.935423→0.937931|2→1|1|

Clipped detector precision/recall is97.66%/89.66%; tracks93.27%/94.33%.
Detector horizontal/Up RMSE is0.442209/0.222047m; tracks0.406336/0.225047m.
These results demonstrate substantially better accuracy in this particular
window, without establishing performance on other takes. Twenty-one of22
eligible GT IDs are ever detected/tracked;GT89 remains0/11 at ordinals0–10,
before the late propagated localization failures. The minimum all-ID criterion
still fails, and camera eligibility is geometric rather than verified optical
visibility. No GT is excluded or used as tracker input.

Independent raw quaternion/GNSS/.03m antenna/inverse recordedtower2 extrinsic/G
checks verify all132 archived XYZ poses. No V1 mounting or take3-only bias is
applied toV2. There are105 accepted/27 propagated poses. XYZ reference RMSE is
0.092550m/max0.209663m, with22 >=0.1m: one accepted at frame497210 and21 propagated
at497240–497260. Maximum propagation age is2.400315s. ENU horizontal/Up pose
RMSE is0.067999/0.062783m; orientation RMS0.506242 degrees. These are retained
recipe agreement, not independently certified physical GT accuracy. All frames
and pose heights remain in scoring; neither registration gates nor start policy
changes. Camera leads LiDAR45.667–47.658ms (median46.922ms), with unknown exposure
and annotation epoch. Rendering ground placement remains explicit and unused.

A new20-frame10FPS camera/LiDAR video compares detections, baseline persistent
IDs and2s identity-registry IDs, with three labelled miss/FP/switch PNGs. Source
pose/K/extrinsic/payload hashes pass; root and producer inspect decoded frames.
These wireframe diagnostics provide no scene/asset depth-occlusion certification.
Source results: `artifacts/tracking-take3-vehicle2-native-pilot-20261012/`;
independent XYZ: `artifacts/tracking-take3-vehicle2-xyz-independent-20261012/`;
overlays: `artifacts/tracking-take3-vehicle2-tracking-overlays-20261012/`.

The bounded GT89 audit checks all200 raw proposals at each of its11 eligible
captures. All11 XYZ errors are below0.1m (maximum0.083132m), and the GT projection
is entirely inside the network image crop. Six frames have any raw proposal
within2m, but maximum confidence is0.02351; none reach0.1/0.2. GT native range
is41.48–48.87m. Exact and0.25m-padded GT cuboid LiDAR support is zero in every
capture, both under original evaluation and native-reference transforms.
Camera views suggest foreground-pickup occlusion, but do not certify optical
visibility or invalidate the annotation. No recoverable existing >=0.1 proposal
discarded by the tracker is demonstrated. GT89 stays in the original scores.
First/middle/last camera/LiDAR PNGs preserve original calibration and native
geometry; the network crop is labelled separately from object boxes.
Evidence: `artifacts/tracking-take3-vehicle2-GT89-miss-audit-20261012/`.

A separate11-frame camera-depth diagnostic projects actual raw LiDAR samples
into the GT convex silhouette, retaining minimum measured depth in fixed5px
cells without interpolation. Native-reference silhouettes contain592 samples;
583 are at least1m in front of the nearest GT corner. Observed cell coverage is
12.9–20.6%, so most cells remain unknown and whole-object camera occlusion is
not certified. Original world-reference silhouettes contain365 samples,319
foreground, with5.6–13.6% cell coverage. First/middle/last PNGs label measured
foreground points and separate both GT representations. This supports partial
foreground-depth evidence conditional on baseline calibration/timing, without
changing GT eligibility or any detection/tracking metric. Evidence:
`artifacts/tracking-take3-vehicle2-GT89-camera-depth-diagnostic-20261012/`.

The new15-case summary/ledger preserve previous14 roots and rows exactly:
4258 captures completed,24 qualified sequences pending,1031 unscored tails.
Reports: `artifacts/tracking-native-probe-summary-current15-20261012/` and
`artifacts/tracking-sequence-coverage-ledger-current15-20261012/`.

### Take3/V3 full461-frame window: good horizontal accuracy and native height

All461 available captures from earliest qualified index40/frame496800 through
500/frame497260 complete461 fresh forwards/92200 raw proposals/zero warmups.
Checkpoint574 strict loading/sparse compatibility, native frustum and baseline
calibration/augmentation are unchanged. Take3 is in the declared training split;
this is not held-out detector generalization. The first40 native captures remain
outside the fixed qualified policy. No detector/model/tracker variant is added.

|Camera scope|Detection TP/FP/FN|Track TP/FP/FN|IDF1 0→2s|Switches 0→2s|Fragments|
|---|---:|---:|---:|---:|---:|
|HistoricalFalse|2020/37/164|2085/57/99|0.948220→0.958391|3→1|7|
|Explicit clipping|2362/55/165|2430/74/97|0.952495→0.961240|3→1|7|

Clipped detection precision/recall is97.72%/93.47%; tracking97.04%/96.16%.
All31 eligible GT IDs are detected and tracked at least once, meeting the
minimum per-ID condition in this window. The stronger90% per-ID target still
fails: eleven IDs with at least five eligible observations have track recall
below90%. Clipped detector horizontal/Up RMSE is0.322225/0.686799m; tracks
0.324289/0.708315m. Eligibility remains geometric; optical visibility is not
certified by these scores. Logged camera capture trails LiDAR13.572–16.379ms,
median14.950ms, with exposure/annotation epoch unverified.

Independent rawINS quaternion/.03m antenna/inverse recordedtower3 extrinsic/G
checks verify all461 poses and payload identities. No V1 mounting or take3-only
position bias applies. There are457 accepted/four propagated poses; maximum
propagation age0.199914s. XYZ reference RMSE0.051284m/max0.076275m, zero>=0.1m;
ENU horizontal/Up RMSE0.031046/0.040819m. Orientation RMS2.044991 degrees/max
2.215372 degrees remains a meaningful disagreement despite small translation.
Agreement is conditional on the retained raw recipe, not independent physical GT.

A cached native sensor-relative supplement reconstructs the same GT-free
tracker states and verifies original world outputs before scoring. On the
ORIGINAL2362 detector pairs, native horizontal/Z RMSE is0.329591/0.096624m;
on2430 track pairs,0.329966/0.096652m. These same-pair checks show that worldUp
error cannot be treated as an intrinsic BEVFusion height defect. No scoring
poses or heights change. Separately matched native-clipped detector counts are
2371/46/153; tracks2441/63/83,2s IDF1 0.966189/switches1/fragments6. This changes
evaluation representation and three GT eligibility observations, not model
accuracy. Primary world metrics remain unchanged. An inherited nonsemantic
report key named`all305` is documented: actual loops/controls/receipt cover461.
Independent source/assignment checks reproduce all four native views. Exact
fixed-pair vector decomposition gives detector/track rotation Up RMS0.721/0.743m,
translation0.040/0.040m and sensor geometry0.096/0.096m. Components sum to the
original error vectors within2e-14m; component RMS values are not additive.
This establishes reference-orientation contribution, not physical GT truth.
Review: `artifacts/tracking-take3-vehicle3-native-sensor-relative-independent-20261012/`.

Twenty consecutive camera/LiDAR video frames and three outcome-selected PNGs
label detections, persistent IDs, misses, protocol FPs and switches; original
poses/heights/K/extrinsics remain frozen. Wireframes have no depth-occlusion
certificate. Evidence: `artifacts/tracking-take3-vehicle3-native-pilot-20261012/`,
`artifacts/tracking-take3-vehicle3-xyz-independent-20261012/`,
`artifacts/tracking-take3-vehicle3-native-sensor-relative-evaluation-20261012/`,
and `artifacts/tracking-take3-vehicle3-tracking-overlays-20261012/`.

### Take3/V3 explicit display-ground comparison: no adoption

The existing GT-free frozen map ground profile is applied only to copies of
first20 saved track poses:152 supported placements and77 unsupported poses
preserved exactly. No GT, new plane/height fitting, calibration or tracker input
is used. Supported display shifts along ENUUp are+0.240..+0.908m, median+0.437m.
Actual Gamma primitive renders compare original versus explicit ground placement
at ordinals0/10/19, each with camera-no-scene-depth and Gaussian/shared-asset-depth
controls. Car-only back-face culling is active and SAM is off.

Grounding raises displayed cars and does not establish optical improvement;
native matched Z error is already about0.097m. The profile is not adopted for
this sequence and no production/default/scoring-height change occurs. Shared
depth records partial far-car occlusion: ordinal19 original track13 has907
projected pixels,422 scene-hidden/314 nearer-asset-hidden/171 visible; grounded
has915/124/662/129. No fully hidden case is invented, and Gaussian expected depth
is not a certificate of physical visibility. All alpha-supported depths are
finite and positive. Gamma's original CloudXR processes remain running and GPU
returns to baseline after the owned render completes. Paired PNGs, placement,
source-preservation and execution receipts:
`artifacts/tracking-take3-vehicle3-explicit-ground-render-20261012/`.

New16-case reporting preserves all prior15 roots and rows exactly:4719 captures,
23 qualified sequences pending,1031 unscored tails. Reports:
`artifacts/tracking-native-probe-summary-current16-20261012/` and
`artifacts/tracking-sequence-coverage-ledger-current16-20261012/`.


### Full Take3/V1 localization check and detection miss

The fixed start remains index0/frame496760, ending index500/frame497260.
`artifacts/tracking-take3-vehicle1-full501-native-20261012/` reuses the original
300 inference records exactly and adds201 fresh forwards, with zero warmups.
Both tracker options replay continuously from the same start, without a reset
at index300. Independent source-composition and scoring checks are in
`artifacts/tracking-take3-vehicle1-full501-xyz-independent-20261012/`.
Raw GNSS/INS composition includes V1's mounting correction and the additional
Take3-only position correction; the retained recipe is not applied to released
ICP-refined poses.

All501 frames are registration accepted. XYZ reference RMSE is0.021565m,
maximum0.042865m, with zero errors at or above0.1m. Orientation RMSE is0.267°.
These checks establish agreement with the retained recipe reference, rather
than independent absolute accuracy. Detection ENU horizontal/vertical RMSE is
0.381487/0.150438m; tracking is0.389473/0.145833m. Matched-only errors do not
assign zero error to missed objects.

| Partial-camera result | Detections | Tracks |
|---|---:|---:|
| TP/FP/FN |1721/296/129|1799/380/51|
| Precision/recall |85.32%/93.03%|82.56%/97.24%|
| Ever matched / eligible GT IDs |35/36|36/36|

Baseline IDF1 is0.871680; optional2s reacquisition gives0.885580 and reduces
ID switches3→1, with six fragmentations in both. Defaults remain unchanged.
Every ID being tracked geometrically does not satisfy the requirement that every
eligible GT ID be detected: GT105 has zero primary detections in five eligible
frames. The boundary video and labelled miss/false-positive PNGs are in
`artifacts/tracking-take3-vehicle1-full501-tracking-overlays-20261012/`.
Camera timestamps follow LiDAR by23.992–28.106ms; exposure and annotation epoch
remain unverified. Take3 is declared training-overlapping, not held out.

The GT105 audit at
`artifacts/tracking-take3-vehicle1-full501-GT105-miss-audit-20261012/` finds all
five projected regions outside the baseline network image crop. Native GT-box
point counts are30/20/4/3/1. Track35 is updated by stronger neighboring
proposals on all five frames, rather than coasting; its first two filtered
positions happen to match GT105 within2m. This is not physical identity proof.

A bounded five-forward full-width experiment restored all five regions to the
network input but recovered zero GT105 primary detections. Primary front
TP/FP/FN stayed30/7/8; common horizontal error improved only about5mm. The
predeclared recovery condition failed, so no uniform501-frame variant rerun or
default change was made. Frozen calibration, thresholds, source heights and
scoring stayed fixed. Results and terminal receipts are in
`artifacts/tracking-take3-vehicle1-full501-GT105-fullwidth-probe-20261012/`.

All three qualified Take3 windows are now complete:501/132/461 captures for
V1/V2/V3, respectively, totaling1094. This excludes fixed unselected prefixes
and does not claim all1503 native agent frames were evaluated. V2 still has22
XYZ exceptions, so the V1 check must not be generalized to every agent.
For V3, native matched vertical RMSE is already0.097m; world vertical mismatch
includes reference orientation effects. The earlier explicit rendering-only
ground comparison did not establish visual improvement. Grounding stays
optional and must preserve every pose and height used for scoring.

### Partial-camera summary and cross-agent GT89 observation audit

The current16-case partial-camera summary verifies every source projection and
assignment against the frozen geometry. Detection counts are16903/3229/4864
TP/FP/FN; tracking18569/5200/3198. Historical unclipped controls reproduce
exactly. The2720 eligible observations with the camera inside an annotated box
remain included and explicitly uncertain; no GT is excluded to improve scores.
Geometric eligibility is not a certificate of optical visibility.

`artifacts/tracking-take3-GT89-cross-agent-observation-audit-20261012/` checks
six same-named V2/V3 frames497129–497134. Their native global GT89 cuboids are
identical. V2 has zero exact/padded GT-box LiDAR points; V3 has10–65 exact
points and primary detections on all six, with confidence0.365–0.565. V3's
LiDAR timestamp follows V2 by38.029–38.449ms; shared frame names do not establish
simultaneous availability. This supports differing recorded observations,
without certifying physical identity or complete optical occlusion. Peer
observations were not fused, and GT never entered either tracker.


### Take3 horizontal association diagnostic and actual MCAP replay gap

`artifacts/tracking-take3-horizontal-association-diagnostic-20261012/` preserves
current partial-camera source eligibility and reproduces primary XYZ matching
before separately evaluating ENU-horizontal2m association. V1 and V2 detector
counts remain unchanged, with zero eligible misses attributable only to the
vertical gate. GT105 and GT89 still have no eligible horizontal2m proposal.
Ground placement therefore cannot recover these missing detections.

V3's separate horizontal diagnostic adds ten matches without losing a primary
GT observation (2362/55/165→2372/45/155 TP/FP/FN). Those ten pairs have native
vertical RMSE0.104580m versus ENU Up1.602031m. Exact Up decomposition gives
reference-rotation component RMSE1.614335m, translation0.035008m and native
sensor geometry0.106704m. Components sum as vectors; RMS values are not additive.
Horizontal RMSE on these pairs remains1.394m. This is a conditional reference
attribution, not a detector improvement, physical GT adjudication or evidence
that grounding fixes the poses. Primary metrics and original heights are frozen.

Actual recording evidence is now available at
`artifacts/tracking-mcap-offline-integration-20261012/`. The local ROS-converted
Take1/V2 MCAP contains501 images and501 metadata records, with no LiDAR stream.
Indexed records200/201 (frames481460/481461) decode images exactly matching
native PNGs and the new probe NPZs. Camera K and the relative camera/ego
extrinsic match the baseline. Native LiDAR is an explicitly external supplement;
historical absolute MCAP poses do not establish verified XYZ provenance.

Separately, Lambda's actual split-system Take2 Foxglove recording contains881
camera and LiDAR samples per agent. A read-only two-frame parse succeeds; its
reader obtains calibration from the baseline code rather than file metadata.
The local ROS file's missing LiDAR must not be generalized to this recording.
A bounded real-record parser→explicit artifact XYZ join→public adapter test also
passes, with GT removed and two synthetic vehicle responses used only to check
persistent-ID publication. It is not inference or tracking accuracy evidence.

At that bounded audit, production MCAP replay lacked a verified per-frame XYZ
pose consumer and rejected a nonzero ROS start. The opt-in implementation and
actual-record validation below now address this gap; older static-transform
manifests still do not reproduce the scored raw-GNSS/INS policy.


### Full Take1/V2 fixed-start evaluation

The next completed case is301 captures at indices200–500/frame481460–481760,
starting at the previously archived earliest strict-qualified frame. All301
forwards are fresh, with60200 raw proposals and zero warmups. Original strict574
weights, sparse-convolution fix, frustum, baseline calibration, single-scan XYZI,
confidence0.2 and inclusive class-aware2m XYZ matching remain fixed. The owned
container stopped successfully; no queue or production defaults changed.
Evidence: `artifacts/tracking-take1-vehicle2-native-pilot-20261012/` and
`artifacts/tracking-take1-vehicle2-xyz-independent-20261012/`.

Current partial-camera detections give1726/56/226 TP/FP/FN (96.86% precision,
88.42% recall); tracks1775/113/177 (94.01%/90.93%). All projected IDs count20,
with19 ever detected/tracked: GT146 is missed in all11 eligible frames. GT85's
tracking coverage is73/225, including a152-frame longest miss. The minimum
all-ID and stronger per-ID90% conditions therefore fail despite good aggregate
precision. Both0/2s tracker options have IDF1 0.915625, three switches and one
fragmentation; optional identity changes do not improve these scores.

Matched detection ENU horizontal/Up RMSE is0.399452/0.288198m; tracks are
0.440553/0.293349m. Raw V2 quaternion/0.03m antenna/inverse recorded extrinsic
composition with the retained recipe is independently verified, without V1 or
Take3 corrections. All301 XYZ errors are below0.1m: RMSE0.020694m, maximum
0.045902m, with245 accepted and56 propagated frames. Orientation RMSE is0.612°;
all propagated frames are retained. Camera timestamps precede LiDAR by a median
21.734ms; optical/annotation timing remains conditional. Take1 appears in the
current declared training metadata, so this is not held-out generalization.

The labelled first20-frame video and PNGs for GT146, GT85, an ID switch and
unmatched tracks are in
`artifacts/tracking-take1-vehicle2-tracking-overlays-20261012/`. They preserve
original scored poses/heights and show camera plus LiDAR diagnostics; wireframe
projection does not certify optical visibility or scene-depth occlusion.


### Verified XYZ sidecar replay and actual detector transport

Offline MCAP replay now accepts `--xyz-pose-sidecar`, implemented by
`src/data/xyz_pose_sidecar.py` and the existing `McapFrameReplay`. It selects a
strict contiguous source-index/frame-ID range for ROS-CDR or Foxglove, replaces
recorded poses with verified XYZ poses before detector delivery, and preserves
original LiDAR/image capture timestamps separately from playback pacing.
Baseline K/extrinsics, image dimensions/dtype and RGB/XYZI hashes are checked;
GT/reference metadata is excluded. Missing frames and incompatible static
relocalization/self-scoring combinations fail rather than silently falling back.
The local Take1/V2 ROS file's native LiDAR supplement is explicitly declared;
Foxglove replay requires the recorded cloud. Heights remain unchanged.

`artifacts/tracking-take1-vehicle2-xyz-mcap-replay-20261012/` supplies GT-free
2/20/301-frame sidecars, the source builder and actual-record replay receipts.
On stable source, actual MCAP parser/public-adapter replay of all301 frames in
both tracker modes matches saved physical requests and tracker states, including
IDs, dimensions, scores and poses. Cached responses make this integration
validation, not new accuracy or throughput evidence.

A separate actual MCAP→public client→private IPC→native GPU detector test runs
two fresh forwards. Both frames481460/481461 reproduce all200 ordered proposals
per frame bitexact, including centers, dimensions, yaw and confidence. Strict574
weights, the sparse-convolution fix, original frustum, baseline calibration and
single-scan intensity handling are preserved. The successful container exits0;
a failed diagnostic logger attempt is retained separately (one forward), for
three total forwards across attempts and zero warmups.

Both camera publications in this short live detector check precede any completed
response; replies arrive during EOF drain, with no invented future publication.
This check preceded the selected-clock correction: prefix reading consumed
playback time and could burst the selected ROS frames. Playback now starts at
the first selected record, matching the existing Foxglove policy.
It verifies actual sensor/model transport, but does not establish timely rendered
tracks or real-time throughput. A longer capture-paced check remains necessary.
Defaults and primary detection/tracking scores are unchanged. The optional
`--bevfusion-minimum-area-px 16` reproduces the declared coverage area; default64
remains unchanged. All79 focused replay/sidecar tests pass locally, on Lambda and
on Gamma. Code and docs are rsynced to isolated repositories; active queues and
remote HEADs remain unchanged.

For reproducibility, the native scored pilot and its cached/live IPC fixture use
birth confidence0.2, continuation0.1, duplicate-birth guard enabled, one-hit
confirmation and0.5s coast; baseline/reacquisition labels select0/2s identity
reuse. These are explicit pilot controls, distinct from untouched production
defaults (continuation0.2, guard off). Scores remain unchanged. The pacing
correction has80 passing focused tests; a20-frame capture-paced live detector
check is being prepared without changing model, inputs or tracker parameters.


### Actual20-frame capture-paced publication check

`artifacts/tracking-take1-vehicle2-xyz-mcap-paced20-20261012/` reruns the same
first20 fixed-start captures through the real MCAP parser/public adapter/private
native detector socket after the selected-clock correction. All20 input callbacks
are retained; only15 requests are served because the normal latest-only pending
queue replaces five. Zero warmups and model errors; the owned container and
scoring finish successfully. All15 served physical requests and own raw200
normalization controls pass. Eight ordered raw outputs are bitexact; seven have
small numerical/reordering differences, with GT-free paired-center maximum
6.21mm. This is descriptive drift, not adoption of a new accuracy tolerance.

| Same20 source captures | Published asynchronous tracks | Cached serial tracks |
|---|---:|---:|
| TP/FP/FN |61/2/46|83/1/24|
| Precision/recall |96.83%/57.01%|98.81%/77.57%|
| Geometric IDF1 |0.71765|0.86911|
| Switches/fragmentations |0/4|0/0|
| Matched XYZ RMSE |0.29505m|0.29280m|

Median callback interval is100.198ms (92.642–106.231ms), with selected-record
callback lateness14.9–22.3ms. Cold first inference takes341.371ms, versus steady
median55.121ms. Submit-to-poll median is115.799ms; published source age is
median200.139ms, maximum500.423ms. Captures0/1/2/3/5 publish empty boxes; replies
for18/19 arrive only during EOF drain, without invented future publications.
These are observed bounded cold-start/pilot-policy timings, not a deployment
throughput guarantee. The mocked analysis sink excludes actual JPEG/spool cost.

This identifies a concrete publication-availability failure independent of
localization/height: prediction geometry stays close to the serial reference,
but missing/stale results reduce recall and add fragmentations. The next offline
improvement is an explicit mode that awaits each joined capture's detector result
before publishing that capture. Nonblocking production defaults remain separate;
model, inputs, GT matching and scored heights must remain fixed in comparison.


### Offline current-capture publication improvement

The new opt-in `--mcap-await-detections` addresses the measured availability
loss. It requires verified XYZ MCAP replay with BEVFusion and submits each joined
capture once before normal snapshot publication. A bounded wait polls for the
current result; detector errors or a30s timeout abort. Image shape is retained
before submission consumes the sensor buffer, so current-result projection and
real analysis-spool delivery remain valid. Default live/nonblocking behavior is
unchanged. No model, calibration, source pose, height or GT threshold changes.

`artifacts/tracking-take1-vehicle2-xyz-mcap-awaited20-20261012/` reruns exactly
the same20 captures with this flag and fresh native inference. All20 requests
are served, with zero replacements, warmups or errors. Every publication carries
its current result (source age0ms); tracks give83/1/24 TP/FP/FN, precision98.81%,
recall77.57%, IDF1 0.869110 and zero switches/fragmentations. These match serial
reference counts/IDs, versus asynchronous61/2/46, recall57.01%, IDF1 0.717647
and four fragmentations. XYZ RMSE is0.292791m versus serial0.292799m. The gain is
publication availability, not new detector accuracy; underlying misses persist.

Wall-clock selected-record to final publication is3.143s for1.900s of captures;
median callback interval145.695ms and blocking129.626ms. Cold inference335.811ms
and steady median54.960ms are reported separately. It is an offline correctness
policy, not a10Hz guarantee. All physical requests/own raw normalization pass;
nine of20 ordered raw outputs are bitexact, with remaining numerical/reordering
drift reported separately rather than hidden by an adopted tolerance. The owned
container and scoring exit0; earlier asynchronous and failed-attempt artifacts
remain unchanged. Independent code/receipt review passes. All88 focused tests
pass locally; deployment verification uses the same focused suite on both hosts.


A labelled actual-publication before/after video and PNGs0/5/19 are available at
`artifacts/tracking-take1-vehicle2-mcap-publication-comparison-20261012/`.
They reuse saved assignments and unchanged source images/GT/poses, showing
asynchronous source age versus current-capture offline results. The video plays
source captures at10FPS; this is not the measured wall-clock delivery rate.

### Full301 awaited replay and localization check before grounding

`artifacts/tracking-take1-vehicle2-xyz-mcap-awaited301-20261012/` completes
301 fresh forwards through the public MCAP adapter with real frame/analysis
spools and 301 decoded JPEGs. Every publication uses its current capture's
result; no requests are replaced. Detection counts remain1726/56/226 and
published track counts1775/113/177, exactly matching the serial reference.
Track IDF1 remains0.915625 with three switches and one fragmentation.
The instrumented replay takes46.239s for29.999s of source captures. This
reruns an existing case; coverage stays17 cases/5221 captures.

The independent raw GNSS/INS, antenna and recorded-extrinsic composition
check covers all301 localized poses against the retained ENU recipe:
XYZ RMSE0.020694m, maximum0.045902m, zero errors >=0.1m, including all56
propagated poses. This establishes recipe agreement, not independent absolute
GT accuracy. Matched detection horizontal/Up RMSE remains approximately
0.399/0.288m; vertical error does not dominate this case.

`artifacts/tracking-take1-vehicle2-GT146-GT85-miss-audit-20261012/` separates
the two persistent coverage failures. GT146 has five weak proposals and six
below0.1 across11 eligible observations, with3–23 LiDAR returns and four
observations outside the learned image crop. Direct raw-recipe checks on these
11 poses have maximum XYZ error0.025063m. GT85 has the same nearby master
track6 throughout its152-frame scored gap, with current strong-score witnesses,
but its projected box lies beyond the camera boundary. GT still has a partial
intersection. Original symmetric camera eligibility remains fixed; these
predictions are shown before filtering only in labelled diagnostic overlays.

Neither failure is recovered by correcting vertical placement. No tracker
threshold or grounding change follows this audit. Any subsequent grounding
must be explicit and rendering-only, preserve scored XYZ poses and heights,
and demonstrate visual improvement against an original-height comparison.

Independent actual-wire/spool verification is recorded in
`artifacts/tracking-take1-vehicle2-mcap-awaited301-independent-20261012/`.
The labelled actual-publication video and four fixed cases are available in
`artifacts/tracking-take1-vehicle2-mcap-awaited301-compact-overlays-20261012/`:
`GT66` means `take1:66`, and `P14` means `bevfusion:vehicle:14`. Full IDs remain
in saved results and the ID mapping. Compact labels are optional; the original
full-label overlays remain intact. Two focused visualization tests pass locally
and on Lambda/Gamma after rsync, without restarting services or queues.

The uniform cached image-boundary diagnostic at
`artifacts/tracking-camera-canvas-boundary-diagnostic-20261012/` compares
continuous canvas bounds [0,w]×[0,h] with the current pixel-center bounds
[0,w-1]×[0,h-1], preserving16px area and all geometry/matching rules. It adds
five detector and four track matches for GT85;148 of the original152-frame
track gap remain missed. The frozen Take2/V2 control adds one false positive
and no matches. This is a scoring/projection convention diagnostic, not a model
gain; current production bounds and authoritative scores remain unchanged.

### Take1/V3: full405 and height-error attribution

`artifacts/tracking-take1-vehicle3-native-pilot-20261012/` evaluates all405
captures from archived start96/frame481356 through500/481760, using405 fresh
forwards and81000 raw proposals. Strict574 weights, sparse-convolution fix,
original frustum, baseline calibration, source heights and pilot tracker settings
remain fixed. Current partial-camera D TP/FP/FN is1135/53/126 (95.54% precision,
90.01% recall); T is1189/107/72 (91.74%/94.29%). IDF1 is0.923739, with three
switches and one fragmentation; registry0/2 geometry and metrics are identical.
Only24/27 eligible IDs are ever detected/tracked. GT103(1 observation),
GT189(10) and GT220(2) remain missed; none is excluded to pass coverage.

The independent raw V3 quaternion/antenna/inverse tower3 extrinsic/G check at
`artifacts/tracking-take1-vehicle3-xyz-independent-20261012/` reproduces all405
poses and four scoring views. All374 accepted poses are below0.1m (maximum
0.069682m), but20 propagated tail poses exceed0.1m, reaching0.425500m at2.700s
since acceptance. All405 remain scored. Overall XYZ RMSE is0.070108m and
orientation RMS1.722 degrees; no V1 mounting or Take3 correction is applied.
This verifies retained-reference agreement, not independent physical accuracy.

Matched detection ENU horizontal/Up RMSE is0.467978/0.471195m; track errors
are0.459595/0.504577m. The cached fixed-pair diagnostic at
`artifacts/tracking-take1-vehicle3-pose-tail-error-diagnostic-20261012/`
shows native detector Z RMSE0.207m. On accepted frames, total detector Up RMS
0.468m has vector components with RMS0.015m translation,0.526m relative
rotation at the GT center and0.201m sensor geometry. Their vectors reconstruct
the original errors within2.25e-14m; RMS values are not additive. Thus small
XYZ translation error does not establish that world height error is entirely
BEVFusion's intrinsic vertical error. The last20 poses also have large native
horizontal residuals; correlation with propagation does not prove causality.
No localization refit, grounding, calibration or tracker change follows this
diagnostic. Original17 cases and41 other sequence ledger entries are preserved
in the new18-case snapshots; coverage and accuracy goals remain incomplete.

The compact labelled first20 video and miss/switch/FP cases are in
`artifacts/tracking-take1-vehicle3-tracking-overlays-20261012/`; the FP case
retains its failing propagated pose. The13-observation never-ID audit at
`artifacts/tracking-take1-vehicle3-never-ID-audit-20261012/` finds a strong
but camera-excluded proposal for GT103, near-zero native LiDAR support and
weak proposals for GT189, and crop exclusion plus weak/inaccurate proposals
for GT220. No horizontal-only or tracker association fix recovers these cases.
Wireframe eligibility remains distinct from certified optical visibility.

The frozen42-forward full-width detector probe at
`artifacts/tracking-take1-vehicle3-fullwidth-detector-probe-20261012/`
restores GT220's model-input ROI using the previously tested resize512/1920
and the same256x512 tensor/frustum, without changing K/extrinsics/poses.
GT220 remains0/2, with no raw native or world-center proposal within2m.
V3 first20 front counts remain94/3/10; its all54 control adds one FP. Take2/V2
first20 front improves32/2/20 to32/0/20, but target recovery fails. The probe
is rejected: no default change or full405 rerun. Source/checkpoint controls
pass,42 forwards and8400 raw proposals finish with zero warmups; cached
baselines are reused. This targeted development trial does not add coverage.

### Take1/V1: full501 meets minimum geometric ID coverage

`artifacts/tracking-take1-vehicle1-full501-native-20261012/` evaluates501
fresh captures at archived start0/frame481260 through500/481760. All501
native RGB/XYZI arrays are compared with cached NPZs before reuse; source
metadata and archived XYZ poses are regenerated, not inherited model results.
The first container attempt fails before any forward because that cache lacks
a mount. Its artifacts remain preserved; the read-only mount correction allows
retry23948 to complete501 forwards/100200 raw proposals, with strict574,
original frustum, baseline calibration and pilot tracker settings unchanged.

Current clipped detections give2009/51/131 TP/FP/FN (97.52% precision,
93.88% recall); tracks2088/94/52 (95.69%/97.57%), IDF1 0.959741, two switches
and three fragmentations. All31 geometric-front IDs are detected and tracked
at least once, meeting the minimum condition for this case. Stronger per-ID
90% coverage still fails: GT130 detections3/23, GT69 tracks18/24, GT37 tracks
34/45 and GT85 tracks30/38. Registry0/2 geometry and aggregate metrics agree.
Take1 overlaps declared training metadata; no held-out performance claim.

Independent raw V1 quaternion×F1,0.03m antenna, inverse corrected tower1
extrinsic and G composition agrees with the retained mounting-delta recipe,
without the Take3 position bias. All501 XYZ RMSE is0.048006m, maximum0.161854m;
29 errors >=0.1m include11 accepted and18 propagated poses. Those remain
scored. Orientation RMS is0.290336 degrees. Recorded camera capture times
follow LiDAR by a median19.432ms; true exposure/object-motion alignment is
not certified. Evidence and four-view recount are at
`artifacts/tracking-take1-vehicle1-full501-independent-20261012/`.

Matched detector ENU horizontal/Up RMSE is0.327045/0.186890m; tracks are
0.371112/0.188539m. The fixed-pair diagnostic at
`artifacts/tracking-take1-vehicle1-pose-error-diagnostic-20261012/` finds
native detector XY/Z0.322/0.168m. Up vector components have RMS0.0208m
translation,0.0783m relative rotation and0.1657m sensor geometry; these are
not additive and reconstruct original errors within1.96e-14m. The29 failing
source poses contain53 GT observations, with52 detector matches and53 track
matches; they do not establish a localization-caused tracking miss. Scene
exposure differs from the other472 captures. No localization refit or change
to scored poses/heights follows the check.

Compact labelled first20 video and GT69 miss/ID-switch/unmatched-track PNGs
are in `artifacts/tracking-take1-vehicle1-full501-tracking-overlays-20261012/`.
They retain source poses and saved assignments; wireframes do not certify
optical visibility or depth occlusion. The19-case ledger adds only this full
sequence, preserving18 previous rows/cases and41 other sequence entries.

### Take2/V2: full758 and localization cross-check

`artifacts/tracking-take2-vehicle2-full758-native-20261012/` extends the fixed
start123/frame487973 to880/488730. The exact300-frame prefix is reused;
458 fresh forwards produce91600 additional raw proposals. Continuous trackers
do not reset at the boundary. Independent source, raw-recipe, model-input,
prefix and four-view checks pass in
`artifacts/tracking-take2-vehicle2-full758-independent-20261012/`.

Current clipped detections are2914/311/448 TP/FP/FN (90.36% precision,
86.67% recall); tracks3152/608/210 (83.83%/93.75%). Two-second identity
reacquisition reduces switches15 to13 and raises IDF1 from0.825892 to0.832350
without changing geometry; fragmentation stays14. Detections cover38/39 IDs,
tracks39/39. GT192 has no primary matched detection in20 eligible observations,
so the minimum detector condition remains unmet.

Raw V2 recipe composition uses no V1 mounting or Take3 correction. XYZ RMSE
is0.047314m, maximum0.134839m;34 errors >=0.1m include31 accepted and3 propagated
frames. All are retained. GT192's20 frames all pass, maximum0.048799m.
Matched detector ENU horizontal/Up RMSE is0.657742/0.423209m, native XY/Z
0.654372/0.381247m. The fixed-pair check in
`artifacts/tracking-take2-vehicle2-pose-error-diagnostic-20261012/`
reconstructs Up using translation, orientation lever-arm and sensor residual
components (RMS0.034860/0.182790/0.382035m, not additive). Small translation
alone cannot establish irreducible detector height error or justify changing
scored heights. Camera precedes LiDAR by median84.360ms; exposure alignment
and physical GT completeness remain unverified.

The cached GT192/FP lifecycle audit in
`artifacts/tracking-take2-vehicle2-GT192-tracker-FP-audit-20261012/`
reproduces all758 tracker outputs exactly. GT192's15 track matches arise from
four weak updates and11 coasting observations. Of608 unmatched tracks,
287 have primary measurements,182 weak measurements and139 coast; shortening
coasting alone would lose valid coverage and leave most unmatched observations.
One GT192 primary proposal has horizontal1.856m/Up-1.092m, outside the fixed
2m 3D gate. No horizontal-only rematching, grounding or tracker change is adopted.

Compact labelled first20 video and GT192/ID-switch/unmatched-track PNGs are in
`artifacts/tracking-take2-vehicle2-full758-tracking-overlays-20261012/`.
The persistent-FP source views in
`artifacts/tracking-take2-vehicle2-persistent-FP-source-evidence-20261012/`
verify native annotations equal prepared GT, but cannot certify physical absence:
the sparse far proposals project into thin right-edge/background regions, and
track8's later match to moving GT52 does not prove physical continuity.
The prospective two-second primary-refresh diagnostic in
`artifacts/tracking-primary-refresh-budget-probe-20261012/` reproduces the
baseline exactly and finds no scored TP or FP older than two seconds since a
strong update. Some unscored filters exceed it. No variant/control run or rule
change follows because no direct benefit is established.

### Take8/V1: full431, strong source boxes excluded by camera projection

`artifacts/tracking-take8-vehicle1-full431-native-20261012/` covers start0 through
430/frame537760 with300 exact reused results and131 fresh forwards. Independent
source/raw-recipe/model/prefix/four-view checks pass in
`artifacts/tracking-take8-vehicle1-full431-independent-20261012/`.
All431 XYZ poses are accepted: RMSE0.042804m, maximum0.058247m, orientation
RMS0.211159 degrees. V1 mounting composition is retained without Take3 bias.

Current clipped detections give1695/264/719 TP/FP/FN (86.52%/70.22%); tracks
2105/367/309 (85.15%/87.20%). Registry0/2 IDF1 improves0.717560 to0.752354,
switches28 to24, fragmentation stays25 and geometry is unchanged. Both detect
or track22/27 eligible IDs. Matched detector horizontal/Up RMSE is
0.842162/0.326777m; good source-position agreement does not resolve coverage.

The27-observation never-ID audit in
`artifacts/tracking-take8-vehicle1-full431-never-ID-audit-20261012/`
finds strong raw proposals within2m for every GT3/GT49 eligible observation,
but none survives prediction camera eligibility. GT projections survive as
near-plane-clipped right-edge slivers. These are benchmark misses despite
existing strong source boxes, not evidence of detector absence. GT11/GT13 have
only one eligible weak observation each; GT16 has no raw proposal within2m.
GT11/13/16 share identical geometry at overlapping frames, and GT3/49 centers
are0.023–0.270m apart where both occur. IDs and annotations remain unchanged;
their physical interpretation and optical visibility are not certified.
Compact first20 video and GT3/ID-switch/FP PNGs are in
`artifacts/tracking-take8-vehicle1-full431-tracking-overlays-20261012/`.
No score, calibration, crop, grounding or tracker default changes follow.

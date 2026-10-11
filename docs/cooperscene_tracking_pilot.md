# CooperScene tracking pilot, 2026-10-11

Only take 1 / vehicle 1 was evaluated: native frames **481260–481319**, 60
consecutive frames (5.90 s between first and last captures). Other sequences
remain on hold. Initial input preparation also cached take 3/V1 and take 4/V2;
those caches were not inferred after the user requested one sequence first.

## Acceptance and protocol

The minimum requested condition is that every front-camera GT annotation ID is
detected at least once. The stronger pilot target is at least 90% detection and
track recall for each ID visible in at least five frames, no never-detected ID,
and at most one identity switch per 100 visible GT observations. An optional
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

## Fixed localization and GT

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
for every frame and persists raw detections separately from tracks.

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

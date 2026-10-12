# Offline MCAP replay and asset rendering

The replay queue uses recorded MCAP inputs, procedural vehicle assets and the
existing remote compositor. It supports CooperScene agents 1–3 and nuScenes,
selected start-frame offsets, isolated attempts, retries, and resume. Input
readiness checks verify a completed MCAP footer/summary, camera counts and map
files. No SAM3/SAM3D workers are needed in asset-proxy mode.

```sh
.venv/bin/python scripts/deployment/run_remote_teleop_batch.py \
  --manifest path/to/manifest.json --output-root artifacts/replay-NEW --dry-run
```

Remove `--dry-run` only when intentionally launching data playback on prepared
hosts. `prepare_remote_teleop_batch.py` discovers and validates MCAP/map inputs;
`stage_remote_teleop_batch.py` stages isolated code/dependencies and has its own
`--dry-run`. Neither is automatically run by importing these modules.

Each nuScenes manifest entry must name its scene-specific map. CooperScene entries
must name the intended Riverside map and explicit cached localization transform.
The manifest preparer's recorded-pose reference defaults are historical; they do
not automatically apply the new geodetic calibration recipe. The retained recipe
operates on raw-GNSS/INS poses and must not be applied directly to released
ICP-refined coordinates. Localization metrics label the supplied reference.

Vehicle meshes and wireframe fallbacks share one object depth buffer as well as
Gaussian scene-depth tests. Near assets occlude farther assets regardless of
iteration order. Render grounding is explicit: `--ground-plane` accepts a plane
or bounded road-surface JSON. `--ground-z` on the compositor accepts a fixed map Z;
it is mutually exclusive with `--ground-plane`. Omitted grounding preserves the
localized object height. Ground placement modifies rendered boxes, not localized
sensor poses or trajectory-error measurements. Unsupported bounded ground
queries preserve the incoming pose and emit per-track placement diagnostics.

A bounded, map-only profile tested on Take2/V1 frames243/299 is available at
`artifacts/tracking-take2-render-only-grounding-20261012/ground-profile.json`.
To compare display grounding, explicitly pass that path with `--ground-plane`
to the compositor. The paired diagnostic supports29 of32 boxes and preserves
the other three exactly. Local road tilt can change displayed XY and rotation,
as well as height; tracker and localization poses remain unchanged. The profile
uses Gaussian-map road support rather than GT, and is not surveyed road truth.
Omit the flag to keep the original localized object heights.

Validation used simulated launchers/readers and local tests only; no replay or
real vehicle process was launched during integration.

For the later deterministic detector/tracker evaluation on Lambda and Gamma,
see [CooperScene tracking pilot](cooperscene_tracking_pilot.md). It uses native
capture timestamps, the retained raw-GNSS/INS XYZ policy, recorded detector
outputs, and explicit front-camera coverage checks. It does not launch vehicle
control or change the localization policy of older MCAP replay manifests.

The verified native CooperScene detector adapter is
`docker/bevfusion/cooperscene_native_front.py`. It requires the pinned model
config and checkpoint described in the pilot report, mounted read-only at
`/native`, and uses four-feature raw XYZI with intensity normalized inside the
server by65535. Do not combine this adapter with client-side intensity scaling,
nuScenes intensity clipping or past sweeps. A real GPU two-frame check reproduces
the earlier host-normalized native predictions exactly. Detailed run scripts
and hashes are in the pilot artifacts; the take1 result overlaps model training.

This detector verification does not validate an arbitrary MCAP manifest's
localization. The tracking pilot uses the fixed archived per-frame XYZ export,
including flagged propagation on rejected registration frames. Preserve that
policy for comparisons; an old cached transform alone does not reproduce it.
Keep render ground placement omitted unless explicitly requested, and keep GT
out of detector/tracker inputs. The optional two-hit tracker confirmation trial
remains separate; production tracker defaults were not changed.


Tracking input preparation now retains exact `image_timestamp_ns`. For a separate
camera-only timestamp sensitivity, evaluate with `--front-camera-only
--require-valid-image-timestamp`: zero/negative timestamps skip camera scoring,
missing metadata is an error, and all LiDAR scoring and tracker inputs remain
unchanged. This does not certify optical visibility or replace the fixed primary
benchmark. Do not reconstruct missing capture timestamps from floating-point skew.

The replay adapter now exposes optional tracker controls:
`--bevfusion-continuation-score-threshold`,
`--bevfusion-minimum-confirmation-hits` and
`--bevfusion-duplicate-birth-guard`. Optional identity reacquisition is enabled by
`--bevfusion-maximum-reacquisition-seconds 2.0`; it is disabled by default. It
matches a new track to a mutually unambiguous retired identity using class,
predicted position, heading and dimensions. Expired filters remain expired;
new filters start fresh and no extra boxes appear during gaps. The Take2/V1
cached pilot improves identity metrics while preserving all box estimates,
but does not improve detection accuracy or certify unannotated identity links.
By default, continuation uses the primary
`--bevfusion-score-threshold`, confirmation requires one observation, and the
birth guard is off. The primary threshold gates track creation; a weak server
proposal cannot create a hidden track. When explicitly enabled, low-score updates
can maintain already-confirmed tracks and their rendered IDs. Box prompts retain
the actual latest confidence rather than inventing a high score or hiding a valid
confirmed track solely because its latest update is weak.

The previously evaluated exploratory guard/continuation policy uses primary0.2,
continuation0.1, one confirmation hit and the birth guard. To reproduce it in a
separate recorded-only adapter run, pass:

```sh
--bevfusion-score-threshold 0.2 \
--bevfusion-continuation-score-threshold 0.1 \
--bevfusion-minimum-confirmation-hits 1 \
--bevfusion-duplicate-birth-guard
```

The pinned native detector server must also emit scores down to0.1; a server
filtering at0.2 cannot supply continuation proposals. Lower server filtering does
not lower the adapter's0.2 birth threshold. Do not enable this policy in existing
queues automatically: primary benchmark precision and per-ID coverage still fail.
Combining continuation with two-hit confirmation lost21 front-camera GT observations
on the fixed Take2 window and is rejected. The deterministic cached sensor-join
check is distinct from full asynchronous MCAP throughput or optical visibility.

During scheduled MCAP pacing waits, the replay services the detector through
its nonblocking public poll and sleeps in intervals of at most10ms. This lets
ready results dispatch pending requests between captures. Record order, capture
deadlines, timestamps and publication counts remain unchanged; overdue records
proceed immediately. Without a detector, pacing retains its single sleep.
Decoding and callbacks can still delay polling; this is not a real-time bound
or a lossless inference guarantee.

After MCAP input ends, the running adapter continues polling BEVFusion in its
existing idle service loop. This consumes a ready response and dispatches the
latest pending request without requiring another image/metadata callback. It
preserves each request's capture timestamp and does not rewrite durable snapshots
or publish synthetic camera frames. Shutdown still cancels outstanding work;
this is not a blocking drain or a lossless inference policy.

The replay queue keeps only the latest pending request when inference is busy.
A fixed Take2/V1 cached simulation using actual Foxglove publication order shows
front tracking recall90.91%/76.05%/21.06% at virtual50/150/300ms service delays.
These are simulated workload results, not measured GPU latency. At300ms, stale
observations often exceed the existing0.5s coast limit. No automatic queue, coast,
replay-rate or detector-threshold changes are applied. See the tracking pilot
report and `artifacts/tracking-take2-delayed-adapter-trial-20261011/`.

Verified per-frame XYZ replay is opt-in through `--xyz-pose-sidecar path.json`
with `--mcap`. The version1 sidecar declares take/agent, MCAP name/size/mtime
(optional streaming SHA256), recipe/map/input provenance, exact BGR image width/height, baseline K, image dimensions and
camera-from-LiDAR extrinsic, and a contiguous original source-index/frame-ID
range. Each row supplies a rigid `world_T_lidar`, exact original LiDAR/image
capture timestamps, recorded replay timestamp, and decoded BGR/XYZI SHA256.
It contains no GT or reference poses. Camera pose is XYZ LiDAR pose composed
with the inverse baseline extrinsic; ego remains explicitly the LiDAR alias.

The sidecar range selects the start/end for ROS-CDR and Foxglove recordings;
no missing-frame, nearest-frame or released-pose fallback is allowed. Recorded
MCAP times pace playback, while detector/tracker/publications preserve the
original LiDAR capture clock and scored XYZ heights. Actual decoded K/extrinsic
and sensor byte hashes must match before delivery. ROS MCAPs lacking LiDAR need
an explicit per-row `points_npz` path/checksum supplement; this is native source
LiDAR, not a claim that the recording contained it. Foxglove replay requires
and validates the contained point cloud. Static relocalization, localization
metrics and online localizers cannot be combined with sidecar replay.

The sidecar loader is `src/data/xyz_pose_sidecar.py`; a verified GT-free example
builder is in `artifacts/tracking-take1-vehicle2-xyz-mcap-replay-20261012/`.
For the evaluated geometric coverage policy, explicitly pass
`--bevfusion-minimum-area-px 16 --bevfusion-score-threshold 0.2` with
`--bbox-source bevfusion` and the intended detector endpoint. The scored native
pilot uses `--bevfusion-continuation-score-threshold 0.1` and
`--bevfusion-duplicate-birth-guard`, one-hit confirmation and 0/2s identity
reacquisition. Its “baseline” label means zero reacquisition, not unmodified
production tracker settings. Omit these controls to use production defaults
(continuation0.2, guard off); the area default remains64 pixels squared. No rendering grounding is
implicitly enabled. This opt-in path does not validate arbitrary historical
MCAP calibration or prove independent absolute localization accuracy.

The ROS sidecar path starts its playback clock at the first selected record,
after reading the skipped prefix. Prefix I/O cannot force the selected window
into catch-up playback. This pacing rule has a regression test; capture timestamps
used for tracker motion remain the original LiDAR timestamps.

For deterministic offline comparison, additionally pass
`--mcap-await-detections`. This requires MCAP + verified XYZ sidecar +
`--bbox-source bevfusion`. Each fully joined capture is submitted once; the
adapter polls until that capture's result arrives before publishing its normal
snapshot and analysis frame. A detector error or30-second timeout aborts the
run. Original capture timestamps, poses and heights remain unchanged; delayed
wall-clock delivery can exceed the recorded cadence. This is an explicit
lossless comparison policy, not a10Hz or real-time guarantee. Omitting the flag
retains normal nonblocking latest-pending replay and live callbacks unchanged.

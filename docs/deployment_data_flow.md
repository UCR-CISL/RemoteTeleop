# Deployment data flow

The current deployment path has two host roles, loaded from `cfg/alien4_alien3.yaml`
with `RemoteTeleopConfig.from_yaml()`. `alien4` is the local `vehicle` host;
`alien3` is the remote-ops host. The `vehicle` role owns camera ingestion,
stable-track admission, SAM3 masking, sequential SAM3D reconstruction, and mesh
publication. The `remote-ops` role owns the remote mesh cache and GSplat
composition. Large mesh payloads are versioned and SHA-256 verified; camera and
control publication do not wait for reconstruction.

The authoritative render path and the slow analysis path are independent.
Every pose+bbox `FrameSnapshot` is durably spooled and receiver-pulled by the
remote compositor; a mesh is never required to render or acknowledge a frame.
The image and metadata records are joined by exact dataset timestamp into a
separate, vehicle-local `FrameDetections` spool. SAM3 and SAM3D each pull and
ACK every analysis frame at their own pace, so slow inference cannot overflow
a PUB/SUB queue and cannot delay remote proxy rendering.

`BoxPrompt.xyxy` is the projected 2D extent of the authoritative 3D box.
`BoxPrompt.text_label` is sent to SAM3 along with that geometric prompt.
CooperScene currently emits `car` for every box.

The default stable-track admission window is 2 seconds of continuous
visibility. During that window the admission policy retains the best eligible
keyframe by projected area, visibility, confidence, and image-border clearance.
After two seconds, its projected 2D bbox and `car` label prompt SAM3; an
accepted mask plus the same local JPEG is passed to sequential SAM3D object
reconstruction. Until a verified mesh reaches the remote cache, the compositor
continues rendering that track as its bbox proxy.
Per-frame analysis records maintain visibility and select the keyframe; they do
not trigger per-frame masks. With the default `new_tracks` policy, each track
produces at most one SAM3 mask request and one deduplicated SAM3D reconstruction
within a scene generation.

## CooperScene MCAP conversion

`CooperSceneMcapProcessor` converts one selected take and agent without loading
PCD files. It writes a ROS 2-profile MCAP with two synchronized topics:

- `/camera/image_raw`: `sensor_msgs/msg/Image` (`bgr8`, CDR)
- `/camera/frame_detections`: `std_msgs/msg/String` containing versioned JSON
  for the frame ID, source timestamp, camera intrinsics, world poses, and
  projected labelled boxes

The processor uses the fixed `gaussian_T_cooperscene` localization transform
when supplied. For example:

```python
from pathlib import Path

from src.data import CooperSceneMcapConfig, CooperSceneMcapProcessor

# Load the 4x4 matrix from the accepted localization artifact.
localization_transform = ...
config = CooperSceneMcapConfig(
    data_root=Path("/path/to/CooperScene"),
    output_path=Path("artifacts/cooperscene_take_1_agent_1.mcap"),
    scenario="1",
    agent="1",
    scene_generation="cooperscene:train:1:1",
    gaussian_T_cooperscene=localization_transform,
)
frame_count = CooperSceneMcapProcessor(config).process()
```

The timestamps begin at zero and advance at the configured dataset cadence.
They represent dataset time, not the wall clock used to measure subscriber and
render latency.

For the SAM launcher, one direct MCAP reader consumes image and metadata records
in chronological record order at the recorded 10 Hz cadence. It bypasses the
offline rosbag/DDS delivery hop, which dropped small metadata callbacks under
raw-image load, but it does not preload future poses. Each current metadata
record is immediately committed to the render spool; its matching image is
independently committed to the analysis spool. The launcher requires exact
501-record durable end markers for both streams.

`src.deployment.ros_frame_adapter` is the vehicle-side source adapter.
It validates `/camera/frame_detections` and durably appends one image-free
`FrameSnapshot` containing sequence, timestamp, generation/frame identifiers,
`world_T_ego`, and boxes. A ROUTER serves those snapshots only when the remote
DEALER requests its next durable cursor; the compositor ACKs only after the
rendered PNG and cursor are committed. With `--analysis-endpoint`, the adapter
also subscribes to `/camera/image_raw`, joins it to metadata locally, and
durably writes image-bearing `FrameDetections` for SAM. Live mode requires a
sourced ROS 2 environment. Playback mode uses `--mcap` from the project Python
environment and keeps the live ROS behavior unchanged.

```bash
PYTHONPATH=/workspace${PYTHONPATH:+:$PYTHONPATH} /usr/bin/python3 \
  src/deployment/ros_frame_adapter.py \
  --frames-endpoint tcp://0.0.0.0:8768 \
  --analysis-endpoint tcp://0.0.0.0:8771 \
  --queue-depth 2048 --high-water-mark 2048 \
  --scene-generation cooperscene:train:1:1 \
  --metrics-path artifacts/realtime/metrics/ros_ego_pose_adapter.jsonl
```

The remote compositor connects its `--frames-endpoint` to that address and uses
the stable, locally configured agent-1 `ego_T_camera`, intrinsic matrix, and
render dimensions. It derives `world_T_camera = world_T_ego @ ego_T_camera`
locally; no camera extrinsics or intrinsics traverse the ZMQ pose stream.

```bash
.venv/bin/python \
  -m src.realtime.composited_camera_process \
  --frames-endpoint tcp://100.101.224.74:8768 \
  --assets-endpoint tcp://100.101.224.74:8769 \
  --health-endpoint tcp://<health-address>:<port> \
  --splat data/riverside_r3.spz \
  --output-dir artifacts/split_system_pose_only_20260819/composited \
  --metrics-path artifacts/split_system_pose_only_20260819/metrics/compositor.jsonl \
  --agent 1 --render-width 480 --render-height 300
```

The authoritative path is receiver-pulled with a window of one and durable
cursor/hash acknowledgements; it does not depend on PUB/SUB startup timing or
HWM capacity. Camera video remains on its dedicated video transport, while
full images and the analysis spool stay local to the vehicle. The compositor
projects and renders every received box immediately as a proxy, replacing it
on later frames only after the corresponding verified mesh reaches its cache.

## Configured split-system launcher

Use the launcher for the configured two-host replay instead of manually
monitoring vehicle and remote processes. It loads host addresses and repository
paths from `cfg/alien4_alien3.yaml`. The `local: true` vehicle entry runs its
ROS sidecar commands locally (no SSH back into alien4); only alien3 is reached
over SSH. The ROS replay sidecar is image `remote_teleop_ros_mcap:humble`,
container `remote_teleop_ros_mcap`.

The launcher currently starts only the ROS frame adapter and the alien3 remote
compositor. Although the alien4 GPU/SAM environment is ready, SAM3/SAM3D
workers remain separate orchestration and are not started by this launcher.

```bash
./.venv/bin/python scripts/deployment/run_remote_teleop_simulation.py \
  --config cfg/alien4_alien3.yaml \
  --output-root artifacts/split_system_bbox_launcher_20260819_retry2
```

For asynchronous mesh reconstruction, use the separate SAM launcher. It keeps
the same lossless render contract, enables the local analysis spool on port
8771, prewarms SAM3 in FP16 and SAM3D with selective NF4 before playback, and
keeps both CUDA-resident. One shared execution lease serializes their inference
without CPU-offloading either model between jobs. The launcher waits for both
startup markers and SAM3D's durable completion marker, and allows mesh assets to finish
synchronizing before stopping the compositor:

```bash
./.venv/bin/python scripts/deployment/run_remote_teleop_sam_simulation.py \
  --config cfg/alien4_alien3.yaml \
  --output-root artifacts/split_system_sam_simulation \
  --stable-seconds 2
```

## Procedural vehicle proxy mode

Add `--asset-proxy` to `scripts/deployment/run_remote_teleop_sam_simulation.py`
to render supported vehicles from their first detection without SAM3 or SAM3D.
The remote compositor builds one template per vehicle kind on demand, saves it
under `composited/vehicle-proxies/`, and caches its topology/colors on the GPU.
For each bbox it updates body vertices and wheel positions in memory, then
applies the normal rigid object pose and optional `--ground-z` placement.
There are no per-frame GLB writes or asset transfers. Successful vehicles count
as `mesh` in existing render metrics; unsupported labels retain bbox proxies.

Templates cover `car`, `truck`, `bus`, `articulated_bus`, `trailer`,
`construction_vehicle`, `motorcycle`, `bicycle`, `ambulance`, and `police`.
The shared label mapping accepts both full nuScenes categories and detector
shorthand. nuScenes export and adapters preserve vehicle labels instead of
filtering everything except cars. Existing MCAPs exported with the old car-only
filter must be regenerated to contain the additional GT tracks; the renderer
cannot recover discarded annotations. BEVFusion exposes its available vehicle
classes but does not distinguish emergency vehicles or bus subtypes.

Bodies use white vertex colors, lighter neutral-grey continuous side glazing,
and soft panel shading. The car cabin is one curved shell: its A- and C-pillars
share edges and smooth normals with the roof rails, with no separate roof slab.
Construction vehicles retain their muted equipment paint. Cars retain red rear lamps.
Wheels keep a fixed metric radius and width per vehicle kind, moving with axle
positions as the body changes size. Axles retain enough separation to prevent
adjacent tires intersecting on shortened trucks and trailers. Long and tall cars therefore keep circular
wheels. Low car bodies preserve clearance above the wheel tops instead of
compressing their hood through the tires. Only a bbox too small to contain the chosen wheels reduces each wheel
assembly uniformly. These are generic class silhouettes: they do not infer
specific models, riders, articulated joint angles, or construction attachments.
Exact body resizing has no temporal smoothing and may show bbox size jitter.
As in the existing compositor, object overlays depth-test against the Gaussian
scene; overlapping mesh objects are composited in track order.

This mode disables SAM startup/completion checks and remote asset reception.
The MCAP adapter still writes its existing analysis spool; BEVFusion still runs
when selected. Remote rendering requires torch, PyTorch3D, trimesh, and the
existing Gaussian renderer. Omitting the flag retains the SAM workflow. Asset
load/render failures propagate explicitly rather than silently showing boxes.

Render local preview grids without starting either deployment side:

```bash
python -m src.viz.preview_car_proxy --device cpu --output-dir artifacts/vehicle-proxies-v3
```

Use `--device cuda` when available. `vehicles.png` shows all ten vehicle kinds;
`proportions.png` shows standard, limo, tall, wide, compact, low, and extreme
car proportions. Each also has a `-views.png` grid containing front, side, and
rear views. `rear.png` and `rear-views.png` emphasize the car C-pillars and
tail lights, including straight rear views. `edge-cases.png` additionally exercises a tiny box that triggers
the wheel fit cap, a stretched bicycle, a compact truck, and a wide loader.
Images use the same proxy deformation and mesh renderer as the compositor,
fitted independently to show shape rather than relative size.
The output also contains individual PNGs and nominal metric GLBs in `meshes/`.

## Per-vehicle shape and paint hints

With `--bbox-source bevfusion`, `src.deployment.ros_frame_adapter` can decorate each tracked
car's `BoxPrompt` with compact hints from `VehicleAppearanceEstimator`
(`--vehicle-shape-fit`, `--vehicle-color`; both default off). They are computed on
the vehicle from the BEVFusion request's LiDAR scan and full-resolution image (NumPy only), and
only these fields leave the vehicle:

- `vehicle_shape`: a `UniversalVehicleShape` vector (body type and box-normalized profile
  parameters) fitted to the track's accumulated LiDAR returns.
- `body_color_rgb` (sRGB 0-1) and `color_confidence`: the fused paint color.
- `road_rgb` (sRGB 0-1, only with `body_color_rgb`): the camera's road ring next to the car. The
  remote scales the paint so paint/road luminance matches the camera (`ProxyPaintExposure`).

A missing field renders the default: the legacy primitive car and its untinted paint. The remote
`--asset-proxy` compositor applies the hints; until they arrive, prompts are byte-identical to an
adapter without the estimator.

Publication policy (`VehicleAppearanceConfig`):

- **Range gate.** Neither shape nor color is published beyond `max_publish_range_m` (25 m
  horizontal LiDAR range, `--vehicle-appearance-max-range-m`). A published track keeps its hints
  out to 30 m (`publish_range_hysteresis_m`), then drops back to the default asset. Out-of-range
  tracks keep accumulating LiDAR but are neither fit nor colored. LiDAR at 25-45 m gives too little
  shape evidence: on the dev/val frames, most body-type errors were beyond 25 m.
- **Evidence.** A fit is published only from `min_publish_evidence` (100) evidence voxels on
  (`--vehicle-shape-min-publish-evidence`). Nothing is published before the first fit.
- **Rejection.** Two consecutive rejected fits (rms > 0.25 m) unpublish the shape. A single
  rejected fit keeps the last one.
- **Type.** A per-track `TypePosterior` over periodic type sweeps decides the body type. Rejected
  sweeps do not vote. A confident type is latched.
- **Registration.** Scans are registered along the track onto a locked end-face reference. A
  lasting box-bias change (more than 0.15 m for 10 scans) restarts the track's accumulation.

Live cadence. Fits run on one background thread. Each BEVFusion result starts at most
`max_fits_per_update` (2) fits. In-range tracks waiting for a first shape go first (most
evidence first), then the oldest fit. Tracks far below the publish evidence are skipped. One fit
costs about 10 ms for an incumbent-only refit and about 60 ms for a full type sweep (p95, one
core). Replaying 1 s of CooperScene history per vehicle (`evaluate.py --live-schedule`, dev+val):

- about 1 fit per track per second;
- for cars within 25 m, a first shape 1 scan after the track's first scan (median; mean 1.4, v5 2.4);
- 60/66 vehicles within 25 m show a shape at frame t.

To fit the vehicle CPU budget, change `--vehicle-shape-max-fits-per-update`. Use 1 to halve the
fit load, 0 to stop fitting (no shapes), or 4 to roughly double the refits. Then check
`vehicle_appearance_update_ms` and `vehicle_shape_fit_count` in the adapter's BEVFusion metrics.

Occlusion. With `--asset-proxy`, the remote compositor hides proxies behind the splat scene. By
default (`--proxy-occlusion-depth occluder`) the occluder is the depth where accumulated
non-ground splat opacity reaches `--occluder-threshold` (0.5), so road fuzz and faint floaters do
not cut into the cars. `--occluder-mask` adds a per-splat mask from
`scripts/carve_occluder_mask.py` that leaves LiDAR-carved floaters out of the occluder pass;
`expected` restores the legacy expected-depth test.

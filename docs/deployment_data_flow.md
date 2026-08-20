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

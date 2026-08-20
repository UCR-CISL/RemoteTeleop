# Initial CooperScene video handoff

Status: the lossless split-system SAM replay completed successfully on 2026-08-19.
The latest validated output is
`artifacts/split_system_sam_alien4_alien3_20260819_retry11/composited/composited.mp4`
(SHA-256 `de043f53ff1231c566c85e1cdbd34256f54e94d4f698d0f230c36050ab9a3f46`).

## Current alien4 → alien3 replay configuration

`cfg/alien4_alien3.yaml` is the current host pairing. `alien4` is this local
workstation and performs the vehicle role; its `local: true` entry means the
launcher does not SSH back into alien4. `alien3` remains the remote-ops host
and is reached over SSH. The basic proxy-only launcher can use the ROS sidecar.
The SAM launcher now reads the MCAP directly from alien4's project environment,
so its offline replay does not depend on ROS/DDS delivery.

The replay input is
`artifacts/cooperscene_take_1_agent_1.mcap`: 3,464,357,142 bytes, SHA-256
`00d4c86fd7b8f7fbfe631987138eec0cb3fc728b134bb775b94b524c140ad367`.
It contains 1,002 ROS messages: 501 `/camera/image_raw` and 501
`/camera/frame_detections`. ROS bag timestamps span 0 through 50,000,000,000 ns;
the embedded dataset timestamps span 0 through 50,000,000 us.

Alien4's GPU/SAM environment is ready. The basic launcher starts only the ROS
frame adapter and alien3 compositor. The separate SAM launcher starts those
services plus SAM3 and SAM3D.

The SAM-enabled launcher is
`scripts/deployment/run_remote_teleop_sam_simulation.py`. It adds a durable,
vehicle-local image analysis stream without changing the image-free remote
render contract. Both SAM consumers pull and ACK the full ordered analysis
sequence independently. Stable tracks are admitted after 2 seconds of
continuous visibility; the best eligible projected bbox is sent to SAM3 with
the label `car`, and accepted masks are passed to sequential SAM3D. Remote
frames always render immediately with bbox proxies and switch to verified
meshes only when those assets become available.

Retry 11 validated the real-time concurrent SAM path with 501 render snapshots,
501 local analysis frames, and zero sequence or timestamp gaps. A single direct
MCAP reader delivered both record types chronologically at 10 Hz while alien3
rendered and alien4 ran SAM3/SAM3D concurrently. Both durable spools reached
their exact end markers. SAM3 and SAM3D use
one cross-process GPU residency lock, offloading between turns rather than
holding both models on the 24 GiB GPU. Tracks `12` and `104` produced verified
GLB assets, which were transferred to alien3's mesh cache. The output is
480x300 at 10 FPS, 501 frames, and 50.1 seconds.

The retry-11 MP4 contains proxies throughout because the compositor completed
the render pass before the asynchronous meshes arrived. The meshes are valid
and transferred, but a future proxy-to-mesh demonstration should pace or defer
remote rendering so their arrival occurs before the final source frame.

## Successful split-system bbox-proxy validation (2026-08-19)

In this historical validation, `lambda` performed the vehicle role: ROS 2 MCAP playback and the adapter bound
the paired `frame_object_detections` and `ego_pose_sample` stream at
`tcp://0.0.0.0:8768`. `alien3` performed the remote-ops role: its compositor connected to
`tcp://100.97.168.98:8768` and produced the MP4 above locally in the
repository artifact path.

The MP4 is 480x300 at 10 FPS, with 501 frames and a duration of 50.1 seconds.
The adapter and compositor both recorded sequences 0 through 500 and source
timestamps 0 through 50,000,000 us. Every compositor metric has
`sequence_gap: 0` and `timestamp_gap_us: 0`. All 1002 messages were processed
in ordered detection-before-pose pairs with HWM 2048, not latest-value draining.
The compositor rendered bbox proxies in 491 frames, with 1,663 proxies total
and at most 8 in one frame.

For each sequence, the vehicle first sends `FrameObjectDetections` with the
source timestamp, scene/frame identifiers, and bbox prompts, then the matching
`EgoPoseSample` with `world_T_ego`. `alien3` stores the stable agent-1
`ego_T_camera`, camera intrinsic matrix, and 480x300 render dimensions, then
derives `world_T_camera = world_T_ego @ ego_T_camera` before GSplat rendering.
Camera video remains on its dedicated video transport; full images remain local
to the vehicle for segmentation and reconstruction. Only bbox proxies are
projected and visualized in this validation; no SAM/PyTorch3D mesh transport
was added.

Start the remote compositor before the vehicle publisher and allow the PUB/SUB
subscription to settle before replay. HWM alone cannot recover samples sent
before a subscriber has connected.

## Hosts

For the current pairing, only the remote host needs SSH:

```bash
ssh alien3-cooperslam   # remote-ops / GSplat role
```

Host details and repository paths remain authoritative in
`cfg/alien4_alien3.yaml`. The launcher executes alien4 vehicle commands locally.

Verified on `lambda` on 2026-08-19:

- `/mnt/bcc-data/data/CooperScene` is mounted;
- the repository venv imports `mcap`, OpenCV, and NumPy;
- two RTX 4090 GPUs are visible;
- `ros2` was not found on `PATH` during the probe;
- the localization JSON and Riverside SPZ were not yet present in the repo.

The current workspace has the two missing artifacts:

```text
artifacts/cooperscene_single_agent_coarse_cuda/localization.json
data/riverside_r3.spz
```

## Required order

1. Sync the current implementation to both host repositories without
   overwriting unrelated host changes.
2. Keep the accepted localization JSON on `alien4`.
3. Generate or verify the MCAP on local `alien4` before starting any replay.
4. Confirm the MCAP has both expected topics.
5. Start the `remote_teleop_ros_mcap:humble` image as container
   `remote_teleop_ros_mcap`, with the repository mounted at `/workspace`;
   it supplies ROS 2 plus the MCAP rosbag storage plugin.
6. Copy the Riverside SPZ to `alien3` and verify its venv/render dependencies.
7. Start the remote receiver/compositor independently.
8. Start the configured vehicle adapter on `alien4`. For the SAM simulation it
   reads the MCAP directly with `--mcap`; for live deployment it uses incoming
   ROS callbacks. Collect the composited MP4 from `alien3`.

Use the configured launcher below for the approved split-system simulation;
it coordinates the two independent host processes and collects results without
manual process monitoring.

## MCAP generation on alien4

Run from the local `alien4` repository after syncing the processor and localization
artifact. This records training take 1, agent 1 at 10 Hz:

```bash
./.venv/bin/python - <<'PY'
import json
from pathlib import Path

import numpy as np

from src.data import CooperSceneMcapConfig, CooperSceneMcapProcessor

root = Path.cwd()
localization = json.loads(
    (root / "artifacts/cooperscene_single_agent_coarse_cuda/localization.json")
    .read_text(encoding="utf-8")
)
config = CooperSceneMcapConfig(
    data_root=Path("/mnt/bcc-data/data/CooperScene"),
    output_path=root / "artifacts/cooperscene_take_1_agent_1.mcap",
    split="train",
    scenario="1",
    agent="1",
    frames_per_second=10.0,
    scene_generation="cooperscene:train:1:1",
    gaussian_T_cooperscene=np.asarray(
        localization["gaussian_T_cooperscene"], dtype=np.float64
    ),
)
print(CooperSceneMcapProcessor(config).process())
PY
```

Expected output is 501 frames. Expected topics are:

```text
/camera/image_raw              sensor_msgs/msg/Image
/camera/frame_detections       std_msgs/msg/String
```

After ROS is available, inspect before replay:

```bash
ros2 bag info -s mcap artifacts/cooperscene_take_1_agent_1.mcap
```

## Implementation entry points

- `src/data/data_processor.py`: abstract processor contract.
- `src/data/cooperscene_mcap.py`: CooperScene-to-ROS 2 MCAP processor.
- `src/deployment/vehicle.py`: bounded Torch-spawned SAM3/SAM3D graph and
  transferable mesh publication.
- `src/deployment/remote.py`: hash-verified remote mesh cache.
- `src/realtime/composited_camera_process.py`: pose-only GSplat compositor.
- `docs/deployment_data_flow.md`: protocol and role overview.

## ROS replay adapter

`src.deployment.ros_frame_adapter` subscribes to replay metadata and durably
spools image-free `FrameSnapshot` records for remote rendering. When an
analysis endpoint is configured, it also subscribes to images, joins them to
metadata by exact source timestamp, and writes a separate local JPEG-bearing
`FrameDetections` spool for SAM. Images never enter the remote frame stream.
Start it in a sourced ROS 2 shell before playback.

On the Humble Python 3.10 sidecar, direct adapter invocation must prepend
`/workspace` to the already sourced ROS `PYTHONPATH`, rather than replacing it:

```bash
PYTHONPATH=/workspace${PYTHONPATH:+:$PYTHONPATH} /usr/bin/python3 \
  src/deployment/ros_frame_adapter.py \
  --frames-endpoint tcp://0.0.0.0:8768 \
  --queue-depth 2048 --high-water-mark 2048 \
  --scene-generation cooperscene:train:1:1 \
  --metrics-path artifacts/split_system_pose_only_20260819/alien4/ros_ego_pose_adapter.jsonl
```

On `alien3`, the corresponding compositor command shape is:

```bash
/home/coop3r-slam/miniconda3/bin/conda run -n coop3r-slam python \
  -m src.realtime.composited_camera_process \
  --frames-endpoint tcp://100.101.224.74:8768 \
  --assets-endpoint tcp://100.101.224.74:8769 \
  --health-endpoint tcp://<health-address>:<port> \
  --splat data/riverside_r3.spz \
  --output-dir artifacts/split_system_pose_only_20260819/composited \
  --metrics-path artifacts/split_system_pose_only_20260819/metrics/compositor.jsonl \
  --agent 1 --render-width 480 --render-height 300
```

## Configured launcher

The simulation launcher reads `cfg/alien4_alien3.yaml` by default. It does not
SSH local alien4; it runs the vehicle ROS sidecar there and SSHes only alien3.

```bash
./.venv/bin/python scripts/deployment/run_remote_teleop_simulation.py \
  --config cfg/alien4_alien3.yaml \
  --output-root artifacts/split_system_bbox_launcher_20260819_retry2
```

Use the SAM-enabled launcher for masking, reconstruction, durable mesh
transfer, and GPU take-turn residency:

```bash
./.venv/bin/python scripts/deployment/run_remote_teleop_sam_simulation.py \
  --config cfg/alien4_alien3.yaml \
  --output-root artifacts/split_system_sam_alien4_alien3_20260819_retry11 \
  --stable-seconds 2
```

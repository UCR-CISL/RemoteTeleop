# BasicTeleop
Just for testing purposes


pygobject is pinned because of the ubuntu version (22.04)


# Carla Teleop
```bash
# Spawn a vehicle
python3 -m src.carla.carla_spawn --keep-alive

# Stream camera
python3 -m src.carla.launch_carla_streamer
python3 -m src.carla.launch_carla_stream_receiver
```
Frame rate for streaming CARLA sensor data is slow. Because the gstream sending and receiving code works well, the bug must lie on the CARLA side.



# ZED/Lucid Streaming
```bash
# receiver
uv run python -m src.streaming.gstream_zed_receiver --timestamp-host=100.70.20.114
# timestamp data 
uv run python -m src.streaming.gstream_zed_receiver --timestamp-host=100.70.20.114 > run.log
```

# CloudXR Workflow

This workflow reuses the existing Lucid RTP/H.264 sender and uses IsaacTeleop
`camera_viz` as the receive/decode/render side. CloudXR/OpenXR then delivers the
rendered XR session to the headset/client.

Camera side:

```bash
uv run python -m src.streaming.arena_sender --stream-host <cloudxr-workstation-ip> --stream-port 5000
```

Workstation side:

```bash
./scripts/run_cloudxr_streamer.sh
```

The launcher uses `config/lucid_cloudxr_streamer.yaml`, which listens on RTP
port `5000` and runs `camera_viz` in XR mode. If IsaacTeleop `camera_viz` has not
been set up yet, run:

```bash
cd thirdparty/IsaacTeleop/examples/camera_viz
./camera_viz.sh setup
```

If CloudXR is the active OpenXR runtime, source its environment before launching
the streamer, following the IsaacTeleop CloudXR setup docs.

`cloudxr_streamer` does not replace `arena_sender.py`, and it is not another
GTK/GStreamer receiver. The receive path is IsaacTeleop `camera_viz`; the
headset/client transport is CloudXR.

# ZMQ Vehicle Control MVP
Remote side with a steering wheel:
```bash
./scripts/run_remote_steering_worker.sh --bind "tcp://*:5555" --verbose --log-mcap logs/vehicle_control.mcap
```

Remote side with a steering wheel, Isaac Teleop integration:
```bash
# Terminal 1
source .venv/bin/activate
python3 -m isaacteleop.cloudxr

# Terminal 2
source .venv/bin/activate
source /home/justin/.cloudxr/run/cloudxr.env
./scripts/run_isaac_remote_steering_worker.sh --verbose
```

The steering wheel axis mapping lives in `config/steering_wheel_config.yaml`.

Remote side with keyboard fallback:
```bash
./scripts/run_keyboard_control_worker.sh --bind "tcp://*:5555" --verbose
```

Remote side with keyboard fallback through Isaac Teleop retargeting:
```bash
./scripts/run_isaac_keyboard_control_worker.sh --bind "tcp://*:5555" --verbose
```

Keyboard controls follow the simple kia-opendbc joystick example:

- `W` / `S`: increment gas/brake axis
- `A` / `D`: increment steering axis
- `R`: reset axes to neutral
- `C`: publish neutral
- `Q` or `Esc`: quit

Vehicle side:
```bash
./scripts/run_panda_worker.sh --connect "tcp://<remote-ip>:5555"
```

Use `--dry-run` on the vehicle side to validate ZMQ transport without opening the Panda device.
Replay a command log:
```bash
uv run python -m src.replay_command_mcap logs/vehicle_control.mcap
```

# NuScenes offline teleoperation data

`NuScenesSequenceDataset` keeps every drive separate: each dataset item is one
NuScenes scene, and iterating it yields ordered keyframes. It defaults to
`CAM_FRONT`, `LIDAR_TOP`, and `v1.0-mini`.

```python
from src.data import NuScenesSequenceDataset

dataset = NuScenesSequenceDataset("/path/to/nuscenes", future_steps=12)
for sequence in dataset:
    for frame in sequence:
        # frame.image: RGB HxWx3; frame.lidar_points: Nx5 float32
        # frame.camera_intrinsic: 3x3; frame.lidar_to_camera: 4x4
        # frame.vehicle_boxes; frame.trajectory: (<=12, 3), LiDAR coordinates
        pass
```

The trajectory includes the current ego pose followed by future keyframe poses,
all expressed in the current LiDAR frame. Vehicle boxes use the same frame and
include all `vehicle.*` NuScenes categories (cars, trucks, buses, and more).

Open a Rerun timeline with the front camera, projected 3D scene, LiDAR, vehicle
boxes, calibration, and ego trajectory:

```bash
python -m src.viz.nuscenes_visualizer /path/to/nuscenes --scene 0
```

## SAM 3D vehicle reconstruction MVP

Initialize the pinned upstream checkouts, then create the single CUDA
environment used by replay, online SAM 3.1 masking, SAM 3D Objects, and the
viewer. All dependencies are declared and locked by the root `pyproject.toml`.
Build concurrency must remain at two (and never exceed four) on this host.

```bash
git submodule update --init --recursive thirdparty/sam-3d-objects thirdparty/drivestudio
uv venv .venv --python 3.11
scripts/run_uv.sh
```

SAM 3.1 and SAM 3D Objects weights both require approved Hugging Face access.
Authenticate once with `hf auth login`. SAM 3.1 is fetched into the Hugging
Face cache by the resident mask worker. Download SAM 3D Objects weights into
the pinned submodule:

```bash
hf download facebook/sam-3d-objects \
  --revision 97f96a08e7e261512724a60d7cd6dbb14a2c7cde \
  --include 'checkpoints/*' --max-workers 1 \
  --local-dir thirdparty/sam-3d-objects/checkpoints/hf-download
mv thirdparty/sam-3d-objects/checkpoints/hf-download/checkpoints \
  thirdparty/sam-3d-objects/checkpoints/hf
```

The SAM 3D pipeline config must then exist at
`thirdparty/sam-3d-objects/checkpoints/hf/pipeline.yaml`.

Run the full isolated-process MVP in a host tmux session:

```bash
tmux new-session -d -s remote-teleop-realtime -c "$(pwd)"
tmux send-keys -t remote-teleop-realtime \
  "MAX_JOBS=2 .venv/bin/python -m src.realtime.supervisor \
  --dataroot /path/to/nuscenes --scene scene-0061 --fp16" C-m
```

`--fp16` uses FP16 inference for both resident models. The supervisor also uses
CUDA expandable segments to avoid fragmentation while their allocations
overlap. It loads and warms SAM 3.1, then loads SAM 3D Objects while both are
resident. nuScenes replay starts only after both workers report `READY`.
Synthetic SAM3D warmup is deliberately avoided: unlike a discriminative model,
its dummy input can generate unpredictably dense geometry. On a CUDA OOM the
supervisor records `admission.json`, stops the run, and does not silently
offload or reduce model quality.
During replay:

- nuScenes GT `vehicle.car` tracks provide stable IDs, metric pose, and size;
- Rerun displays one metric box proxy for every live track immediately;
- online SAM 3.1 generates masks from projected boxes on current camera frames;
- one acknowledged reconstruction request is sent per new track;
- SAM 3D Objects generates an object-local mesh asynchronously;
- metric alignment uses the GT `[length, width, height]` box, then Rerun replaces
  only that track's proxy;
- JSONL metrics record replay FPS, dropped frames, mask latency/FPS, queue time,
  reconstruction throughput, and CUDA memory.

Offline masks remain available only as an explicit fallback with
`--offline-mask-root`; no legacy SAM v1 path is supported. Outputs default to
`artifacts/realtime/`, with one mesh directory per scene generation and track.

For CooperScene, the same isolated workers can replay a localized vehicle,
show projected GT box proxies immediately, admit continuously visible tracks
after two seconds in waves of up to five, and replace proxies with depth-aware
SAM3D mesh overlays in the GSplat camera render:

```bash
MAX_JOBS=2 .venv/bin/python -m src.realtime.supervisor \
  --source cooperscene \
  --dataroot /mnt/bcc-data/data/CooperScene \
  --split train --scene 1 --agent 1 \
  --splat data/riverside_r3.spz \
  --localization-transform artifacts/cooperscene_single_agent_coarse_cuda/localization.json \
  --output-root artifacts/cooperscene_sam3d_agent1 \
  --sam3d-config checkpoints/hf/pipeline.yaml \
  --fp16
```

The CooperScene defaults group same-frame box prompts under one SAM 3.1 image
encoding and cache up to five same-frame SAM3D point maps. SAM3D itself remains
single-object and runs admitted requests sequentially. The compositor writes numbered PNGs and
`composited.mp4`. CooperScene replay and standalone rendering process the full
take by default. To truncate at the overlap diagnostic's cutoff, pass both
`--overlap-manifest PATH` and `--stop-at-overlap`.

After localization, replay and default sequence rendering load only each
frame's YAML pose/boxes and camera image. They do not open the PCD or run the
LiDAR overlap prepass. Passing `--stop-at-overlap` explicitly enables the
LiDAR-backed overlap diagnostic/cutoff path.

For a 24 GiB GPU, the measured memory/latency tradeoff keeps SAM3 in FP16 and
uses selective NF4 weights for SAM3D, while retaining FP16 compute for its
sparse operators:

```bash
MAX_JOBS=2 .venv/bin/python -m src.realtime.supervisor \
  --source cooperscene \
  --dataroot /mnt/bcc-data/data/CooperScene \
  --split train --scene 1 --agent 1 \
  --splat data/riverside_r3.spz \
  --localization-transform artifacts/cooperscene_single_agent_coarse_cuda/localization.json \
  --output-root artifacts/cooperscene_sam3d_agent1_nf4 \
  --sam3d-config checkpoints/hf/pipeline.yaml \
  --sam3-precision fp16 --sam3d-precision nf4 \
  --sam3d-stage1-inference-steps 12 \
  --sam3d-stage2-inference-steps 12 \
  --render-downsample 4
```

This remains an asynchronous geometry overlay, not a teleoperation video
path: the measured CooperScene replay produced about 1.43 composited frames/s
and new meshes took about 6.57 seconds. Keep camera/control transport on a
separate real-time path and show projected boxes until meshes arrive.

DriveStudio is pinned only to define the future static-scene renderer boundary.
Its preprocessing, GSplat training, checkpoint loading, nuScenes-to-map
registration, and combined renderer are intentionally deferred.

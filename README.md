# BasicTeleop
Just for testing purposes


pygobject is pinned because of the ubuntu version (22.04)


# Carla Teleop
```bash
# Spawn a vehicle
python3 -m src.carla.carla_spawn --keep-alive

# Launch holoscan distributed app for keyboard control
source /opt/ros/humble/setup.bash
python3 distributed_carla_teleop_app.py --driver --worker --address 127.0.0.1:10000 --fragments RemoteWorkstationFragment
python3 distributed_carla_teleop_app.py --worker --address 127.0.0.1:10000 --fragments VehicleFragment

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

# ZMQ Kia Control MVP
Remote side with a steering wheel:
```bash
./scripts/run_remote_steering_worker.sh --bind "tcp://*:5555" --verbose --log-mcap logs/kia_control.mcap
```

The steering wheel axis mapping lives in `config/steering_wheel_config.yaml`.

Remote side with keyboard fallback:
```bash
./scripts/run_keyboard_control_worker.sh --bind "tcp://*:5555" --verbose
```

Keyboard controls follow the simple kia-opendbc joystick example:

- `W` / `S`: increment gas/brake axis
- `A` / `D`: increment steering axis
- `R`: reset axes to neutral
- `C`: publish neutral
- `Q` or `Esc`: quit

Vehicle side:
```bash
./scripts/run_kia_panda_worker.sh --connect "tcp://<remote-ip>:5555"
```

Use `--dry-run` on the vehicle side to validate ZMQ transport without opening the Panda device.
Replay a command log:
```bash
uv run python -m src.replay_command_mcap logs/kia_control.mcap
```

# Deployment
## Remote-side
```bash
source .venv/bin/activate
export PYTHONPATH="$(pwd)"
python3 distributed_kia_teleop_app.py --driver --worker --address 100.70.20.114 --fragments SteeringWheelFragment
```

## Car-side
```bash
source ~/opendbc/.venv/bin/activate
export PYTHONPATH="$(pwd)"
python3 distributed_kia_teleop_app.py --worker --address 100.70.20.114 --fragments PandaFragment
```

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
# reciever
uv run python -m src.streaming.gstream_zed_receiver --timestamp-host=100.70.20.114
# timestamp data 
uv run python -m src.streaming.gstream_zed_receiver --timestamp-host=100.70.20.114 > run.log
```

CloudXR viewer path for the existing Lucid RTP/H.264 sender:
```bash
# Camera side: send RTP/H.264 to the CloudXR workstation.
uv run python -m src.streaming.arena_sender --stream-host <cloudxr-workstation-ip> --stream-port 5000

# Workstation side: run IsaacTeleop camera_viz in XR mode.
./run_cloudxr_streamer.sh
```

`cloudxr_streamer` does not replace `arena_sender.py`. It launches IsaacTeleop's
`camera_viz` receiver with `config/lucid_cloudxr_streamer.yaml`; CloudXR/OpenXR
then handles the rendered XR session delivery to the headset/client.

# ZMQ Kia Control MVP
Remote side:
```bash
./run_remote_steering_worker.sh --bind "tcp://*:5555" --verbose --log-mcap logs/kia_control.mcap
```

Vehicle side:
```bash
./run_kia_panda_worker.sh --connect "tcp://<remote-ip>:5555"
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

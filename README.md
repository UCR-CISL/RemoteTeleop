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



# ZED Streaming
```bash
# reciever
uv run python -m src.streaming.gstream_zed_receiver --timestamp-host=100.70.20.114
# timestamp data 
uv run python -m src.streaming.gstream_zed_receiver --timestamp-host=100.70.20.114 > run.log
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
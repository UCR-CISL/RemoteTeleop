# BasicTeleop
Just for testing purposes


pygobject is pinned because of the ubuntu version (22.04)


# Carla Teleop
```bash
# Spawn a vehicle
python3 python/carla_spawn.py --keep-alive

# Launch holoscan distributed app for keyboard control
source /opt/ros/humble/setup.bash
python3 python/distributed_launch.py --driver --worker --address 127.0.0.1:10000 --fragments RemoteWorkstationFragment
python3 python/distributed_launch.py --worker --address 127.0.0.1:10000 --fragments VehicleFragment

# Stream camera
python3 python/launch_carla_streamer.py
python3 python/launch_carla_stream_receiver.py
```
Frame rate for streaming CARLA sensor data is slow. Because the gstream sending and receiving code works well, the bug must lie on the CARLA side.

# ZED Streaming
```bash
python3 python/streaming/gstream_zed_sender.py
python3 python/streaming/gstream_zed_receiver.py
```
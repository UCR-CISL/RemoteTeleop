# v2 Teleop Apps

Run robot teleop from the `v2` directory in the ROS1 conda environment:

```bash
cd /home/cisl/Documents/RemoteTeleop/v2
conda activate ros_env
```

`ros_env` should report ROS1 Noetic:

```bash
python -c "import os; print(os.environ.get('ROS_DISTRO'))"
```

If you previously sourced ROS2 Humble in the same terminal, clear the ROS2 Python paths before running the robot app:

```bash
unset PYTHONPATH
conda activate ros_env
```

## Lubao Robot Teleop

Both robot teleop apps publish `geometry_msgs/Twist` to `/control_api` from the `RobotFragment`.

### Steering Wheel

Use this version when the steering wheel hardware is connected to the operator machine.

```bash
# Operator machine, also acting as the Holoscan driver
python robot_teleop_app.py --driver --worker \
    --fragments SteeringWheelFragment \
    --address <operator_ip>:8765

# Robot machine
python robot_teleop_app.py --worker \
    --fragments RobotFragment \
    --driver-address <operator_ip>:8765
```

### WASD Keyboard

Use this version when driving from the operator machine keyboard.
Run `keyboard_robot_teleop_app.py` on both machines; `robot_teleop_app.py` only contains `SteeringWheelFragment`.

Controls:

- `W`: forward
- `S`: reverse
- `A`: turn left
- `D`: turn right

```bash
# Operator machine, also acting as the Holoscan driver
python keyboard_robot_teleop_app.py --driver --worker \
    --fragments KeyboardFragment \
    --address <operator_ip>:8765

# Robot machine
python keyboard_robot_teleop_app.py --worker \
    --fragments RobotFragment \
    --driver-address <operator_ip>:8765
```

For single-machine smoke tests, run either app without distributed Holoscan flags:

```bash
python robot_teleop_app.py
python keyboard_robot_teleop_app.py
```

Both apps accept ROS network overrides:

```bash
python keyboard_robot_teleop_app.py \
    --ros-master-uri http://10.42.0.1:11311 \
    --ros-hostname 10.42.0.254
```

## Carla Teleop

```bash
# Spawn a vehicle
python carla/carla_spawn.py --keep-alive

# Launch Holoscan distributed app
source /opt/ros/humble/setup.bash

python distributed_carla_teleop_app.py --driver --worker --address 127.0.0.1:10000 --fragments RemoteWorkstationFragment
python distributed_carla_teleop_app.py --worker --address 127.0.0.1:10000 --fragments VehicleFragment

# Stream camera
python carla/launch_carla_streamer.py
python carla/launch_carla_stream_receiver.py
```

Frame rate for streaming CARLA sensor data is slow. Because the gstream sending and receiving code works well, the bug must lie on the CARLA side.

## ZED Streaming

```bash
python streaming/gstream_zed_sender.py
python streaming/gstream_zed_receiver.py
```

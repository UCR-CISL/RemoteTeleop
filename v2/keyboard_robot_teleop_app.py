#!/usr/bin/env python3
"""
Distributed Holoscan teleop application for the Lubao delivery robot using WASD.

Two fragments, each run on a separate machine:

  KeyboardFragment       - operator PC
    Operator: KeyboardTeleopOperator
      Reads WASD keyboard state with pynput when available, or terminal
      polling otherwise, and emits:
        throttle       float  -1.0 (reverse) to +1.0 (forward)
        steering_angle float  -0.5 (left) to +0.5 (right)

  RobotFragment          - robot PC (ROS1 / ros_env)
    Operator: RobotTeleopOp
      Receives throttle + steering_angle, publishes geometry_msgs/Twist
      to /control_api at the arrival rate of the keyboard operator.
      ROS1 modules are imported lazily inside start() and never loaded
      on the operator machine.

Controls
--------
  W  forward
  S  reverse
  A  turn left
  D  turn right

Usage - distributed
-------------------
  Operator machine (also acts as driver):
    python keyboard_robot_teleop_app.py --driver --worker \\
        --fragments KeyboardFragment \\
        --address <operator_ip>:8765

  Robot machine:
    python keyboard_robot_teleop_app.py --worker \\
        --fragments RobotFragment \\
        --driver-address <operator_ip>:8765

Usage - single machine
----------------------
  python keyboard_robot_teleop_app.py
"""

from holoscan.core import Application

from fragments.keyboard_fragment import KeyboardFragment
from fragments.robot_fragment import RobotFragment


class KeyboardRobotTeleopApp(Application):
    def __init__(
        self,
        *args,
        ros_master_uri="http://10.42.0.1:11311",
        ros_hostname="10.42.0.254",
        **kwargs,
    ):
        self._ros_master_uri = ros_master_uri
        self._ros_hostname = ros_hostname
        super().__init__(*args, **kwargs)

    def compose(self):
        keyboard_fragment = KeyboardFragment(self, name="KeyboardFragment")
        robot_fragment = RobotFragment(
            self,
            name="RobotFragment",
            ros_master_uri=self._ros_master_uri,
            ros_hostname=self._ros_hostname,
        )

        self.add_fragment(keyboard_fragment)
        self.add_fragment(robot_fragment)

        self.add_flow(
            keyboard_fragment,
            robot_fragment,
            {
                ("keyboard.throttle", "robot_teleop.throttle"),
                ("keyboard.steering_angle", "robot_teleop.steering_angle"),
            },
        )


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Lubao robot WASD keyboard teleop")
    parser.add_argument("--ros-master-uri", default="http://10.42.0.1:11311")
    parser.add_argument("--ros-hostname", default="10.42.0.254")
    # Pass remaining args through to Holoscan (--driver, --worker, --fragments, etc.)
    args, _ = parser.parse_known_args()

    app = KeyboardRobotTeleopApp(
        ros_master_uri=args.ros_master_uri,
        ros_hostname=args.ros_hostname,
    )
    app.run()

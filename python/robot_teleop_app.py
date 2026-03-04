#!/usr/bin/env python3
"""
Distributed Holoscan teleop application for the Lubao delivery robot.

Two fragments, each run on a separate machine:

  SteeringWheelFragment  — steering wheel PC
    Operator: SteeringWheelTxOp
      Reads joystick via SteeringWheelOperator, emits:
        accel          float  -1 (full brake) → +1 (full throttle)
        steering_angle float  -0.5 (full left) → +0.5 (full right)

  RobotFragment          — robot PC (ROS1 / ros_env)
    Operator: RobotTeleopOp
      Receives accel + steering_angle, publishes geometry_msgs/Twist
      to /control_api at the arrival rate of the steering operator.
      ROS1 modules are imported lazily inside start() and never loaded
      on the steering machine.

Velocity mapping
----------------
  linear.x  = clamp(net_linear * MAX_LINEAR,  -1.2, +1.2)  m/s
  angular.z = clamp(-steering_angle / 0.5 * MAX_ANGULAR, -3.14, +3.14)  rad/s

  where net_linear = throttle - brake
        throttle   = (1 - accel) / 2        # 0 when pedal up, 1 when floored
        brake      = (1 - brake_raw) / 2    # 0 when pedal up, 1 when floored

Usage — distributed
-------------------
  Steering machine (also acts as driver):
    python robot_teleop_app.py --driver --worker \\
        --fragments SteeringWheelFragment \\
        --address <steering_ip>:8765

  Robot machine:
    python robot_teleop_app.py --worker \\
        --fragments RobotFragment \\
        --driver-address <steering_ip>:8765

Usage — single machine (local test, no hardware)
-------------------------------------------------
  python robot_teleop_app.py

Note
----
  SteeringwheelController reads its config from a hardcoded path inside
  steering_wheel_controller.py. Ensure that path points to
  /home/cisl/BasicTeleop/config/steering_wheel_config.ini on the steering machine.
"""

import os
from holoscan.conditions import PeriodicCondition
from holoscan.core import Application, Fragment, Operator, OperatorSpec

from python.steering_wheel_operator import SteeringWheelOperator

# Robot velocity limits (from teleop.py / robot manual)
MAX_LINEAR  = 0.2   # m/s
MAX_ANGULAR = 3.14  # rad/s

class SteeringWheelFragment(Fragment):
    def compose(self):
        steering_op = SteeringWheelOperator(
            self,
            PeriodicCondition(self, recess_period=10_000_000),  # 10 ms → 100 Hz
            name="steering_wheel",
        )
        self.add_operator(steering_op)


# ── Robot fragment ─────────────────────────────────────────────────────────────

class RobotTeleopOp(Operator):
    """Translates accel + steering_angle into ROS1 Twist and publishes to /control_api.

    ROS1 imports (rospy, geometry_msgs) are deferred to start() so that this
    module can be imported on the steering machine without a ROS1 environment.
    """

    def __init__(self, fragment, *args,
                 ros_master_uri="http://10.42.0.1:11311",
                 ros_hostname="10.42.0.254",
                 **kwargs):
        self._ros_master_uri = ros_master_uri
        self._ros_hostname   = ros_hostname
        super().__init__(fragment, *args, **kwargs)

    def setup(self, spec: OperatorSpec):
        spec.input("throttle")
        spec.input("steering_angle")

    def start(self):
        # Lazy ROS1 imports — only executed on the robot machine.
        os.environ.setdefault("ROS_MASTER_URI", self._ros_master_uri)
        os.environ.setdefault("ROS_HOSTNAME",   self._ros_hostname)

        import rospy
        from geometry_msgs.msg import Twist

        rospy.init_node("holoscan_teleop", anonymous=True, disable_signals=True)
        self._pub   = rospy.Publisher("/control_api", Twist, queue_size=1)
        self._Twist = Twist
        print(f"[RobotTeleopOp] ROS1 publisher ready — "
              f"master={self._ros_master_uri}, hostname={self._ros_hostname}")

    def stop(self):
        # Send a zero-velocity command so the robot stops immediately.
        self._pub.publish(self._Twist())
        print("[RobotTeleopOp] Zero-velocity stop sent.")

    def compute(self, op_input, op_output, context):
        accel          = float(op_input.receive("throttle"))
        steering_angle = float(op_input.receive("steering_angle"))

        msg = self._Twist()

        # Linear: net pedal signal [-1, 1] scaled to robot's velocity range.
        msg.linear.x = max(-MAX_LINEAR, min(MAX_LINEAR,
                           accel * MAX_LINEAR))

        # Angular: steering wheel ±0.5 mapped to full robot angular range.
        # Negated so that right-turn (positive steering_angle) → negative angular.z
        # (ROS convention: positive z = counter-clockwise / left turn).
        msg.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR,
                            -steering_angle / 0.5 * MAX_ANGULAR))

        self._pub.publish(msg)


class RobotFragment(Fragment):
    def __init__(self, app, *args,
                 ros_master_uri="http://10.42.0.1:11311",
                 ros_hostname="10.42.0.254",
                 **kwargs):
        self._ros_master_uri = ros_master_uri
        self._ros_hostname   = ros_hostname
        super().__init__(app, *args, **kwargs)

    def compose(self):
        robot_op = RobotTeleopOp(
            self,
            name="robot_teleop",
            ros_master_uri=self._ros_master_uri,
            ros_hostname=self._ros_hostname,
        )
        self.add_operator(robot_op)


# ── Application ────────────────────────────────────────────────────────────────

class RobotTeleopApp(Application):
    def __init__(self, *args,
                 ros_master_uri="http://10.42.0.1:11311",
                 ros_hostname="10.42.0.254",
                 **kwargs):
        self._ros_master_uri = ros_master_uri
        self._ros_hostname   = ros_hostname
        super().__init__(*args, **kwargs)

    def compose(self):
        steering_fragment = SteeringWheelFragment(self, name="SteeringWheelFragment")
        robot_fragment    = RobotFragment(
            self,
            name="RobotFragment",
            ros_master_uri=self._ros_master_uri,
            ros_hostname=self._ros_hostname,
        )

        self.add_fragment(steering_fragment)
        self.add_fragment(robot_fragment)

        self.add_flow(
            steering_fragment,
            robot_fragment,
            {
                ("steering_wheel.throttle",          "robot_teleop.throttle"),
                ("steering_wheel.steering_angle",  "robot_teleop.steering_angle"),
            },
        )


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Lubao robot steering wheel teleop")
    parser.add_argument("--ros-master-uri", default="http://10.42.0.1:11311")
    parser.add_argument("--ros-hostname",   default="10.42.0.254")
    # Pass remaining args through to Holoscan (--driver, --worker, --fragments, etc.)
    args, _ = parser.parse_known_args()

    app = RobotTeleopApp(
        ros_master_uri=args.ros_master_uri,
        ros_hostname=args.ros_hostname,
    )
    app.run()

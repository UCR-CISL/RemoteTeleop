import os

from holoscan.core import Fragment, Operator, OperatorSpec

# Robot velocity limits (from teleop.py / robot manual)
MAX_LINEAR = 0.5   # m/s
MAX_ANGULAR = 3.14  # rad/s


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
        self._ros_hostname = ros_hostname
        super().__init__(fragment, *args, **kwargs)

    def setup(self, spec: OperatorSpec):
        spec.input("throttle")
        spec.input("steering_angle")

    def start(self):
        # Lazy ROS1 imports — only executed on the robot machine.
        os.environ.setdefault("ROS_MASTER_URI", self._ros_master_uri)
        os.environ.setdefault("ROS_HOSTNAME", self._ros_hostname)

        import rospy
        from geometry_msgs.msg import Twist

        rospy.init_node("holoscan_teleop", anonymous=True, disable_signals=True)
        self._pub = rospy.Publisher("/control_api", Twist, queue_size=1)
        self._Twist = Twist
        print(f"[RobotTeleopOp] ROS1 publisher ready — "
              f"master={self._ros_master_uri}, hostname={self._ros_hostname}")

    def stop(self):
        # Send a zero-velocity command so the robot stops immediately.
        self._pub.publish(self._Twist())
        print("[RobotTeleopOp] Zero-velocity stop sent.")

    def compute(self, op_input, op_output, context):
        accel = float(op_input.receive("throttle"))
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
        self._ros_hostname = ros_hostname
        super().__init__(app, *args, **kwargs)

    def compose(self):
        robot_op = RobotTeleopOp(
            self,
            name="robot_teleop",
            ros_master_uri=self._ros_master_uri,
            ros_hostname=self._ros_hostname,
        )
        self.add_operator(robot_op)

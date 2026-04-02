import threading

import msgpack
import msgpack_numpy as mnp
import rclpy
from pynput import keyboard
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray

from holoscan.core import Fragment, Operator, OperatorSpec


class CarlaKeyboardControllerOp(Operator):
    def __init__(self, fragment: Fragment, name: str):
        super().__init__(fragment, name=name)
        self._accel = 0.0
        self._steer = 0.0

    def start(self):
        self._listener = keyboard.Listener(
            on_press=lambda key: self._on_key_press(key),
        )
        self._listener.start()

    def stop(self):
        self._listener.stop()

    def setup(self, spec: OperatorSpec):
        spec.output("accel")
        spec.output("steer")

    def compute(self, op_input, op_output, context):
        op_output.emit(self._accel, "accel")
        op_output.emit(self._steer, "steer")

    def _on_key_press(self, key):
        if key == keyboard.Key.up:
            self._accel = min(self._accel + 0.1, 1.0)
        elif key == keyboard.Key.down:
            self._accel = max(self._accel - 0.1, 0.0)
        elif key == keyboard.Key.left:
            self._steer = max(self._steer - 0.1, -1.0)
        elif key == keyboard.Key.right:
            self._steer = min(self._steer + 0.1, 1.0)


class XRControllerOp(Operator):
    """Holoscan operator that subscribes to /xr_teleop/controller_data
    and emits accel/steer from XR controller thumbsticks."""

    def __init__(self, fragment: Fragment, name: str):
        super().__init__(fragment, name=name)
        self._node = None
        self._spin_thread = None
        self._shutdown = False
        self._lock = threading.Lock()
        self._accel = 0.0
        self._steer = 0.0

    def setup(self, spec: OperatorSpec):
        spec.output("accel")
        spec.output("steer")

    def start(self):
        self._node = Node("holoscan_xr_controller")
        self._node.create_subscription(
            ByteMultiArray,
            "/xr_teleop/controller_data",
            self._on_controller_data,
            10,
        )
        self._shutdown = False
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self):
        try:
            while not self._shutdown and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.1)
        except Exception as e:
            print(f"ROS2 spin error: {e}")

    def _on_controller_data(self, msg: ByteMultiArray):
        data = msgpack.unpackb(
            bytes([ab for a in msg.data for ab in a]),
            object_hook=mnp.decode,
        )
        left_thumbstick = data.get("left_thumbstick", [0.0, 0.0])
        right_thumbstick = data.get("right_thumbstick", [0.0, 0.0])

        with self._lock:
            self._accel = float(left_thumbstick[1])
            self._steer = float(right_thumbstick[0])

    def compute(self, op_input, op_output, context):
        with self._lock:
            accel = self._accel
            steer = self._steer
        op_output.emit(accel, "accel")
        op_output.emit(steer, "steer")

    def stop(self):
        self._shutdown = True
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
        if self._node is not None:
            self._node.destroy_node()


class RemoteWorkstationFragment(Fragment):
    def __init__(self, app, name):
        super().__init__(app, name=name)

    def compose(self):
        # carla_controller = CarlaKeyboardControllerOp(self, name="carla_controller")
        carla_controller = XRControllerOp(self, name="carla_controller")
        self.add_operator(carla_controller)

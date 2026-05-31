#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import msgpack
import msgpack_numpy as mnp
from std_msgs.msg import ByteMultiArray


class ControllerListener(Node):
    def __init__(self):
        super().__init__("controller_listener")
        self.create_subscription(
            ByteMultiArray,
            "/xr_teleop/controller_data",
            self.controller_callback,
            10,
        )
        self.get_logger().info("Listening on /xr_teleop/controller_data ...")

    def controller_callback(self, msg: ByteMultiArray):
        data = msgpack.unpackb(
            bytes([ab for a in msg.data for ab in a]),
            object_hook=mnp.decode,
        )

        print(f"Left thumbstick: {data['left_thumbstick']}")
        print(f"Right thumbstick: {data['right_thumbstick']}")
        # print(f"Right trigger: {data['right_trigger_value']}")
        # print(f"A button pressed: {data.get('right_a_click', False)}")


def main(args=None):
    rclpy.init(args=args)
    node = ControllerListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

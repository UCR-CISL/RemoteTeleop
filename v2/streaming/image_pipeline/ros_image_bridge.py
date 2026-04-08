"""
ROS2 image subscriber that forwards raw BGR frames to a Unix socket.
Runs OUTSIDE the venv, in the ROS2 environment.

Connects to the Unix socket server started by gstreamer_sender.py.

Usage:
    python3 ros_image_bridge.py [--socket /tmp/ros_frames.sock] [--topic /lucid/image_raw]

On alienware:
    topic is /lucid/image_raw
"""

import argparse
import socket
import struct
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


def imgmsg_to_bgr(img_msg) -> np.ndarray:
    if img_msg.encoding == "bgr8":
        dtype, n_channels = np.uint8, 3
    elif img_msg.encoding == "rgb8":
        dtype, n_channels = np.uint8, 3
    elif img_msg.encoding == "mono8":
        dtype, n_channels = np.uint8, 1
    else:
        raise ValueError(f"Unsupported encoding: {img_msg.encoding}")

    grid = np.frombuffer(img_msg.data, dtype=np.dtype(dtype))
    frame = grid.reshape((img_msg.height, img_msg.width, n_channels))

    if img_msg.encoding == "rgb8":
        frame = frame[:, :, ::-1]  # RGB → BGR

    return frame


class ImageBridge(Node):
    """Subscribes to a ROS image topic and forwards frames over a Unix socket."""

    def __init__(self, topic: str, socket_path: str):
        super().__init__('ros_image_bridge')

        self._socket_path = socket_path
        self._sock: socket.socket | None = None
        self._connect()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.img_subscriber = self.create_subscription(
            Image, topic, self._image_callback, qos_profile
        )
        self.get_logger().info(f"Subscribed to {topic}, forwarding to {socket_path}")

    def _connect(self):
        """Connect (or reconnect) to the gstreamer_sender socket server."""
        while True:
            try:
                sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                sock.connect(self._socket_path)
                self._sock = sock
                self.get_logger().info(f"Connected to {self._socket_path}")
                return
            except (ConnectionRefusedError, FileNotFoundError):
                self.get_logger().warn(
                    f"Cannot connect to {self._socket_path}, retrying in 2s..."
                )
                time.sleep(2)

    def _send_frame(self, frame_bgr: np.ndarray):
        h, w = frame_bgr.shape[:2]
        data = frame_bgr.tobytes()
        # Header: width (4B) | height (4B) | data_len (4B)
        header = struct.pack('>III', w, h, len(data))
        try:
            self._sock.sendall(header + data)
        except (BrokenPipeError, OSError):
            self.get_logger().warn("Socket disconnected, reconnecting...")
            self._connect()

    def _image_callback(self, msg: Image):
        frame = imgmsg_to_bgr(msg)

        # Resize to half resolution for streaming (e.g. 1440p → 720p)
        h, w = frame.shape[:2]
        frame = cv2.resize(frame, (w // 2, h // 2))

        self._send_frame(frame)


def main():
    parser = argparse.ArgumentParser(description="ROS2 → Unix socket image bridge")
    parser.add_argument("--topic", type=str, default="/lucid/image_raw", help="ROS image topic")
    parser.add_argument("--socket", type=str, default="/tmp/ros_frames.sock", help="Unix socket path")
    args = parser.parse_args()

    rclpy.init()
    node = ImageBridge(args.topic, args.socket)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

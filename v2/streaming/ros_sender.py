"""
on alienware: 
use `        self.img_subscriber = self.create_subscription(Image, "/lucid/image_raw", self.image_callback, qos_profile)`
to subscribe to the raw image topic from the ZED camera node, then convert the ROS Image message to a BGR byte array and send it to the GStreamer pipeline.
"""
from dataclasses import fields
import sys

import argparse

import cv2
import gi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

gi.require_version('Gst', '1.0')
from gi.repository import Gst

import numpy as np

import faulthandler
import typing
import inspect
import signal
faulthandler.enable()


def imgmsg_to_bgr(img_msg) -> np.ndarray:
    if img_msg.encoding == "bgr8":
        dtype, n_channels = np.uint8, 3
    elif img_msg.encoding == "rgb8":
        dtype, n_channels = np.uint8, 3
    elif img_msg.encoding == "mono8":
        dtype, n_channels = np.uint8, 1
    else:
        raise ValueError(f"Unsupported encoding: {img_msg.encoding}")

    row_width = img_msg.width * n_channels
    grid = np.frombuffer(img_msg.data, dtype=np.dtype(dtype)).reshape((img_msg.height, img_msg.step))
    frame = grid[:, :row_width].reshape((img_msg.height, img_msg.width, n_channels))

    if img_msg.encoding == "rgb8":
        frame = frame[:, :, ::-1]  # RGB → BGR
    elif img_msg.encoding == "mono8":
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    return frame
class GStreamerStreamer:
    """GStreamer RTP H.264 streamer with raw video input."""
    
    def __init__(self, host: str, port: int, width: int, height: int, fps: int):
        Gst.init(None)
        self.width = width
        self.height = height
        
        pipeline_desc = (
            f"appsrc name=src is-live=true do-timestamp=true format=time "
            f"caps=video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 ! "
            f"videoconvert ! "
            f"video/x-raw,format=I420 ! "
            f"x264enc bitrate=4000 ! "
            f"h264parse config-interval=1 ! "
            f"rtph264pay pt=96 mtu=1400 config-interval=1 ! "
            f"udpsink host={host} port={port} sync=false async=false"
        )
        
        self.pipeline = Gst.parse_launch(pipeline_desc)
        self.appsrc = self.pipeline.get_by_name('src')
        self.pipeline.set_state(Gst.State.PLAYING)
        self.frame_count = 0
        self.fps = fps
        
        print(f"GStreamer RTP streaming to {host}:{port}")
    
    def send_frame(self, frame_bgr: np.ndarray):
        """Send BGR frame data to GStreamer pipeline."""
        if frame_bgr is None or frame_bgr.size == 0:
            return

        if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            print(f"Dropping non-BGR frame with shape={frame_bgr.shape}")
            return

        if frame_bgr.shape[1] != self.width or frame_bgr.shape[0] != self.height:
            print(
                "Dropping frame with unexpected size: "
                f"got={frame_bgr.shape[1]}x{frame_bgr.shape[0]}, "
                f"expected={self.width}x{self.height}"
            )
            return

        frame_bytes = frame_bgr.tobytes()
        expected_len = self.width * self.height * 3
        if len(frame_bytes) != expected_len:
            print(f"Dropping frame with unexpected byte length: got={len(frame_bytes)} expected={expected_len}")
            return
        
        # Create GStreamer buffer
        buf = Gst.Buffer.new_allocate(None, len(frame_bytes), None)
        buf.fill(0, frame_bytes)
        buf.pts = self.frame_count * (1_000_000_000 // self.fps)  # nanoseconds
        buf.duration = 1_000_000_000 // self.fps

        # Push buffer
        retval = self.appsrc.emit('push-buffer', buf)
        if retval != Gst.FlowReturn.OK:
            print(f"Error pushing buffer: {retval}")
        
        if self.frame_count % (self.fps * 10) == 0:
            print(f"Streamed {self.frame_count} frames")
            print(f"streamed {len(frame_bytes)} byte long frame")
        self.frame_count += 1
        

    
    def stop(self):
        """Stop GStreamer pipeline."""
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)



class ImageSender(Node):
    def __init__(self):
        super().__init__('image_sender')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )   

        self.streamer = None
        self._logged_first_frame = False
        # Subscribe to the raw image topic from the ZED camera node
        self.img_subscriber = self.create_subscription(Image, "/lucid/image_raw", self.image_callback, qos_profile)

    def set_streamer(self, streamer: GStreamerStreamer):
        self.streamer = streamer

    def image_callback(self, msg):
        if self.streamer is None:
            return
        frame = imgmsg_to_bgr(msg)

        # Ensure outgoing frame dimensions always match appsrc caps.
        target_size = (self.streamer.width, self.streamer.height)
        if (frame.shape[1], frame.shape[0]) != target_size:
            frame = cv2.resize(frame, target_size, interpolation=cv2.INTER_AREA)

        if not self._logged_first_frame:
            print(
                "First frame info: "
                f"encoding={msg.encoding}, step={msg.step}, "
                f"source={msg.width}x{msg.height}, sent={frame.shape[1]}x{frame.shape[0]}"
            )
            self._logged_first_frame = True

        self.streamer.send_frame(frame)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="ZED camera with H.264 RTP streaming")
    parser.add_argument("--width", type=int, default=1280, help="Frame width")
    parser.add_argument("--height", type=int, default=720, help="Frame height")
    parser.add_argument("--fps", type=int, default=30, help="Frame rate")
    parser.add_argument("--stream-host", type=str, default="127.0.0.1", help="RTP destination host")
    parser.add_argument("--stream-port", type=int, default=5000, help="RTP destination port")
    # parser.add_argument("--retry-interval", type=int, default=10, help="Camera retry interval (seconds)")
    args = parser.parse_args()
    streamer = GStreamerStreamer(args.stream_host, args.stream_port, args.width, args.height, args.fps)

    

    def signal_handler(sig, frame):
        print("Interrupt received, shutting down...")
        streamer.stop()
        sender.destroy_node()

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.init()
        sender = ImageSender()
        sender.set_streamer(streamer)
        rclpy.spin(sender)

        streamer.stop()
        sender.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(e)

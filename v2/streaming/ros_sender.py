"""
on alienware: 
use `        self.img_subscriber = self.create_subscription(Image, "/lucid/image_raw", self.image_callback, qos_profile)`
to subscribe to the raw image topic from the ZED camera node, then convert the ROS Image message to a BGR byte array and send it to the GStreamer pipeline.
"""

import argparse

import cv2
import gi
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

gi.require_version('Gst', '1.0')
from gi.repository import Gst

class GStreamerStreamer:
    """GStreamer RTP H.264 streamer with raw video input."""
    
    def __init__(self, host: str, port: int, width: int, height: int, fps: int):
        Gst.init(None)
        
        pipeline_desc = (
            f"appsrc name=src is-live=true do-timestamp=true format=time "
            f"caps=video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 ! "
            f"videoconvert ! "
            f"video/x-raw,format=I420 ! "
            f"nvh264enc bitrate=4000 ! "
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
    
    def send_frame(self, frame_bgr: bytes):
        """Send BGR frame data to GStreamer pipeline."""
        if not frame_bgr or len(frame_bgr) == 0:
            return
        
        # Create GStreamer buffer
        buf = Gst.Buffer.new_allocate(None, len(frame_bgr), None)
        buf.fill(0, frame_bgr)
        buf.pts = self.frame_count * (1000000000 // self.fps)  # nanoseconds
        buf.duration = 1000000000 // self.fps
        
        # Push buffer
        retval = self.appsrc.emit('push-buffer', buf)
        if retval != Gst.FlowReturn.OK:
            print(f"Error pushing buffer: {retval}")
        
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
        self.bridge = CvBridge()
        self.streamer = None
        # Subscribe to the raw image topic from the ZED camera node
        self.img_subscriber = self.create_subscription(Image, "/lucid/image_raw", self.image_callback, qos_profile)

    def set_streamer(self, streamer: GStreamerStreamer):
        self.streamer = streamer

    def image_callback(self, msg):
        if self.streamer is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        bgr_frame = frame[:,:,::-1]
        bgr_frame_shape = bgr_frame.shape[0:2]
        # Resize frame to half resolution for streaming (eg 1440p → 720p)
        reshaped_bgr_frame = cv2.resize(bgr_frame, (bgr_frame_shape[1]//2, bgr_frame_shape[0]//2)) # todo needed or no?
        
        self.streamer.send_frame(reshaped_bgr_frame.tobytes())



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


    rclpy.init()
    sender = ImageSender()
    sender.set_streamer(streamer)
    rclpy.spin(sender)

    streamer.stop()
    sender.destroy_node()
    rclpy.shutdown()

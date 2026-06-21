"""
Arena API Direct Sender with GStreamer RTP Streaming

This sender captures images directly from Lucid Vision cameras using arena_api
and streams them via GStreamer RTP H.264 encoding to a configurable destination.
"""
import ctypes

import argparse
import signal
import sys
import time

import cv2
import gi
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst

from arena_api.buffer import BufferFactory
from arena_api.system import system
from arena_api.enums import PixelFormat

from src.streaming.timestamp_sender import UDPTimestampSender


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
            f"nvh264enc bitrate=4000 ! "
            f"h264parse config-interval=1 ! "
            f"rtph264pay pt=96 mtu=1400 config-interval=1 ! "
            f"udpsink name=sink host={host} port={port} sync=false async=false"
        )
        
        self.pipeline = Gst.parse_launch(pipeline_desc)
        self.appsrc = self.pipeline.get_by_name('src')
        self.pipeline.set_state(Gst.State.PLAYING)
        self.frame_count = 0
        self.fps = fps

        udpsink = self.pipeline.get_by_name('sink')
        udp_pad = udpsink.get_static_pad("sink")
        udp_pad.add_probe(Gst.PadProbeType.BUFFER, self._on_udp_probe)
        
        print(f"GStreamer RTP streaming to {host}:{port}")

    def _on_udp_probe(self, pad, info):
        buffer = info.get_buffer()
        if buffer is not None:
            timestamp = f"SEND:frame={self.frame_count-1},pts={buffer.pts},duration={buffer.duration},localtime={time.perf_counter_ns()},len={buffer.get_size()}"
            if hasattr(self, 'timestamp_sender'):
                self.timestamp_sender.send_timestamp(timestamp)
        return Gst.PadProbeReturn.OK

    def set_timestamp_sender(self, timestamp_sender: UDPTimestampSender):
        self.timestamp_sender = timestamp_sender

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
        if hasattr(self, 'timestamp_sender'):
            timestamp = f"SEND:frame={self.frame_count},pts={buf.pts},duration={buf.duration},localtime={time.perf_counter_ns()},len={len(frame_bytes)}"
            self.timestamp_sender.send_timestamp(timestamp)

        retval = self.appsrc.emit('push-buffer', buf)

        if retval != Gst.FlowReturn.OK:
            print(f"Error pushing buffer: {retval}")

        if self.frame_count % (self.fps * 10) == 0:
            print(f"Streamed {self.frame_count} frames ({len(frame_bytes)} bytes per frame)")
        
        self.frame_count += 1

    def stop(self):
        """Stop GStreamer pipeline."""
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)


class ArenaCameraSender:
    """Captures images from Arena API camera and sends to GStreamer."""
    
    def __init__(self, streamer: GStreamerStreamer, timestamp_sender=None):
        self.streamer = streamer
        self.timestamp_sender = timestamp_sender
        self.frame_count = 0
        self._logged_first_frame = False
        self.device = None
        self.running = True

    def configure_device(self, device, width=None, height=None, fps=None):
        """Configure camera settings for streaming."""
        nodemap = device.nodemap
        tl_stream_nodemap = device.tl_stream_nodemap

        # Store initial settings
        self.initial_acquisition_mode = nodemap.get_node("AcquisitionMode").value
        
        # Set acquisition mode
        nodemap.get_node("AcquisitionMode").value = "Continuous"
        
        # Set buffer handling mode to get newest frames
        tl_stream_nodemap["StreamBufferHandlingMode"].value = "NewestOnly"
        
        # Enable stream auto negotiate packet size
        tl_stream_nodemap['StreamAutoNegotiatePacketSize'].value = True
        
        # Enable stream packet resend
        tl_stream_nodemap['StreamPacketResendEnable'].value = True

        # Get current device dimensions
        nodes = nodemap.get_node(['Width', 'Height', 'PixelFormat'])

        # Set pixel format to BGR8 for direct BGR output
        if nodes['PixelFormat'].is_writable:
            nodes['PixelFormat'].value = PixelFormat.BGR8

        # Set width and height if provided
        if width and nodes['Width'].is_writable:
            nodes['Width'].value = min(width, nodes['Width'].max)

        if height and nodes['Height'].is_writable:
            nodes['Height'].value = min(height, nodes['Height'].max)

        print(f"Device configured: {nodes['Width'].value}x{nodes['Height'].value}")
        self.device = device
        return device

    def start_acquisition(self, num_buffers=10):
        """Start image acquisition from camera."""
        if not self.device:
            raise RuntimeError("Device not configured. Call configure_device first.")
        
        self.device.start_stream(num_buffers)
        print(f"Stream started with {num_buffers} buffers")

    def stop_acquisition(self):
        """Stop image acquisition from camera."""
        if self.device:
            self.device.stop_stream()
            print("Stream stopped")

    def acquisition_loop(self, num_frames=None):
        """Main acquisition loop - capture and stream images."""
        try:
            frame_num = 0
            while self.running and (num_frames is None or frame_num < num_frames):
                # Get buffer from device
                buffer = self.device.get_buffer()
                
                
                if buffer is None:
                    continue

                try:
                    # Send capture timestamp
                    if self.timestamp_sender is not None:
                        timestamp_msg = f"CAPTURE:frame={self.frame_count},pts=0,localtime={time.perf_counter_ns()}"
                        self.timestamp_sender.send_timestamp(timestamp_msg)

                    # Convert buffer to numpy array (BGR)
                    frame = self._buffer_to_bgr_frame(buffer)

                    # Resize if necessary to match streamer requirements
                    target_size = (self.streamer.width, self.streamer.height)
                    # print("[dbg] frame.shape = ", frame.shape)
                    if (frame.shape[1], frame.shape[0]) != target_size:
                        frame = cv2.resize(frame, target_size, interpolation=cv2.INTER_AREA)

                    if not self._logged_first_frame:
                        print(
                            f"First frame info: "
                            f"source={buffer.width}x{buffer.height}, "
                            f"format={buffer.pixel_format.name}, "
                            f"sent={frame.shape[1]}x{frame.shape[0]}"
                        )
                        self._logged_first_frame = True

                    # Send frame to GStreamer
                    self.streamer.send_frame(frame)
                    self.frame_count += 1
                    frame_num += 1

                finally:
                    # Always requeue the buffer
                    self.device.requeue_buffer(buffer)

        except Exception as e:
            print(f"Error in acquisition loop: {e}")
            raise

    def _buffer_to_bgr_frame(self, buffer) -> np.ndarray:
        """Convert Arena buffer to BGR numpy array."""
        item = BufferFactory.copy(buffer)
        try:
            
            buffer_bytes_per_pixel = int(len(item.data) / (item.width * item.height))
            array = (ctypes.c_ubyte * buffer_bytes_per_pixel * item.width * item.height).from_address(ctypes.addressof(item.pbytes))
            frame = np.ndarray(buffer=array, dtype=np.uint8, shape=(item.height, item.width, buffer_bytes_per_pixel))
            if buffer_bytes_per_pixel == 1:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        finally:
            BufferFactory.destroy(item) #no need as we requeue later?
            pass
        return frame

    def stop(self):
        """Stop the sender."""
        self.running = False
        self.stop_acquisition()


def create_devices_with_tries(max_tries=6, sleep_time_secs=10):
    """Wait for device connection."""
    for try_num in range(max_tries):
        devices = system.create_device()
        if devices:
            print(f"Created {len(devices)} device(s)")
            return devices
        
        print(
            f"Try {try_num + 1} of {max_tries}: waiting for {sleep_time_secs} "
            f"secs for a device to be connected..."
        )
        for sec_count in range(sleep_time_secs):
            time.sleep(1)
            print(f"{sec_count + 1} seconds passed", end='\r')
        print()
    
    raise Exception("No device found! Please connect a device and run again.")


def main():
    parser = argparse.ArgumentParser(
        description="Arena API Camera with H.264 RTP GStreamer Streaming"
    )
    parser.add_argument("--width", type=int, default=1280, help="Frame width")
    parser.add_argument("--height", type=int, default=720, help="Frame height")
    parser.add_argument("--fps", type=int, default=30, help="Frame rate")
    parser.add_argument("--stream-host", type=str, default="127.0.0.1", help="RTP destination host")
    parser.add_argument("--stream-port", type=int, default=5000, help="RTP destination port")
    parser.add_argument("--timestamp-host", type=str, default="127.0.0.1", help="UDP timestamp destination host")
    parser.add_argument("--timestamp-port", type=int, default=22102, help="UDP timestamp destination port")
    parser.add_argument("--num-frames", type=int, default=None, help="Number of frames to capture (None for infinite)")
    parser.add_argument("--num-buffers", type=int, default=10, help="Number of buffers for stream")

    args = parser.parse_args()

    # Initialize GStreamer streamer
    streamer = GStreamerStreamer(
        args.stream_host, 
        args.stream_port, 
        args.width, 
        args.height, 
        args.fps
    )

    # Initialize timestamp sender
    timestamp_sender = UDPTimestampSender(args.timestamp_host, args.timestamp_port)
    streamer.set_timestamp_sender(timestamp_sender)

    # Initialize camera sender
    sender = ArenaCameraSender(streamer, timestamp_sender)

    def signal_handler(sig, frame):
        print("\nInterrupt received, shutting down...")
        sender.stop()
        streamer.stop()
        timestamp_sender.sock.close()
        system.destroy_device()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        # Connect to camera
        print("Connecting to camera...")
        devices = create_devices_with_tries()
        device = system.select_device(devices)

        # Configure and start
        print("Configuring device...")
        sender.configure_device(device, args.width, args.height, args.fps)
        
        print("Starting acquisition...")
        sender.start_acquisition(args.num_buffers)

        print("Starting streaming...")
        sender.acquisition_loop(args.num_frames)

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Cleaning up...")
        sender.stop()
        streamer.stop()
        timestamp_sender.sock.close()
        system.destroy_device()
        print("Done")


if __name__ == '__main__':
    print('Arena API GStreamer Sender started\n')
    main()
    print('\nSender finished')

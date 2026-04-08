"""
GStreamer RTP H.264 streamer that reads BGR frames from a Unix socket.
Runs INSIDE the venv.

Acts as a server — start this before ros_image_bridge.py.

Usage:
    python3 gstreamer_sender.py [--socket /tmp/ros_frames.sock]
                                [--stream-host 127.0.0.1] [--stream-port 5000]
                                [--fps 30] [--bitrate 4000]
"""

import argparse
import os
import socket
import struct

import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst


class GStreamerStreamer:
    """GStreamer RTP H.264 streamer with dynamic resolution from incoming frames."""

    def __init__(self, host: str, port: int, fps: int, bitrate: int):
        Gst.init(None)
        self._host = host
        self._port = port
        self._fps = fps
        self._bitrate = bitrate
        self._pipeline = None
        self._appsrc = None
        self._frame_count = 0
        self._width = None
        self._height = None

    def _build_pipeline(self, width: int, height: int):
        pipeline_desc = (
            f"appsrc name=src is-live=true do-timestamp=true format=time "
            f"caps=video/x-raw,format=BGR,width={width},height={height},framerate={self._fps}/1 ! "
            f"videoconvert ! "
            f"video/x-raw,format=I420 ! "
            f"x264enc bitrate={self._bitrate} ! "
            f"h264parse config-interval=1 ! "
            f"rtph264pay pt=96 mtu=1400 config-interval=1 ! "
            f"udpsink host={self._host} port={self._port} sync=false async=false"
        )
        self._pipeline = Gst.parse_launch(pipeline_desc)
        self._appsrc = self._pipeline.get_by_name('src')
        self._pipeline.set_state(Gst.State.PLAYING)
        self._width = width
        self._height = height
        print(f"GStreamer pipeline started: {width}x{height} @ {self._fps}fps → {self._host}:{self._port}")

    def send_frame(self, frame_data: bytes, width: int, height: int):
        if self._pipeline is None:
            self._build_pipeline(width, height)
        elif (width, height) != (self._width, self._height):
            # Resolution changed — rebuild pipeline
            self._pipeline.set_state(Gst.State.NULL)
            self._frame_count = 0
            self._build_pipeline(width, height)

        buf = Gst.Buffer.new_allocate(None, len(frame_data), None)
        buf.fill(0, frame_data)
        buf.pts = self._frame_count * (1_000_000_000 // self._fps)
        buf.duration = 1_000_000_000 // self._fps

        retval = self._appsrc.emit('push-buffer', buf)
        if retval != Gst.FlowReturn.OK:
            print(f"Error pushing buffer: {retval}")

        self._frame_count += 1
        if self._frame_count % (self._fps * 10) == 0:
            print(f"Streamed {self._frame_count} frames")

    def stop(self):
        if self._pipeline:
            self._pipeline.set_state(Gst.State.NULL)


def recv_exact(conn: socket.socket, n: int) -> bytes | None:
    """Read exactly n bytes from socket, returns None if connection closed."""
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def serve(socket_path: str, streamer: GStreamerStreamer):
    if os.path.exists(socket_path):
        os.unlink(socket_path)

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(socket_path)
    server.listen(1)
    print(f"Listening on {socket_path} ...")

    try:
        while True:
            conn, _ = server.accept()
            print("Client connected")
            try:
                while True:
                    # Header: width (4B) | height (4B) | data_len (4B)
                    header = recv_exact(conn, 12)
                    if header is None:
                        print("Client disconnected")
                        break
                    width, height, data_len = struct.unpack('>III', header)
                    frame_data = recv_exact(conn, data_len)
                    if frame_data is None:
                        print("Client disconnected mid-frame")
                        break
                    streamer.send_frame(frame_data, width, height)
            finally:
                conn.close()
    finally:
        server.close()
        streamer.stop()
        if os.path.exists(socket_path):
            os.unlink(socket_path)


def main():
    parser = argparse.ArgumentParser(description="Unix socket → GStreamer RTP H.264 sender")
    parser.add_argument("--socket", type=str, default="/tmp/ros_frames.sock", help="Unix socket path")
    parser.add_argument("--stream-host", type=str, default="127.0.0.1", help="RTP destination host")
    parser.add_argument("--stream-port", type=int, default=5000, help="RTP destination port")
    parser.add_argument("--fps", type=int, default=30, help="Frame rate")
    parser.add_argument("--bitrate", type=int, default=4000, help="x264enc bitrate (kbps)")
    args = parser.parse_args()

    streamer = GStreamerStreamer(args.stream_host, args.stream_port, args.fps, args.bitrate)
    serve(args.socket, streamer)


if __name__ == '__main__':
    main()

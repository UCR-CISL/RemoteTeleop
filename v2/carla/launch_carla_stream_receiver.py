import signal

import gi

from python.streaming.gstream_zed_receiver import GStreamerReceiver

gi.require_version('Gst', '1.0')
gi.require_version('Gtk', '3.0')
from gi.repository import GLib, Gst, Gtk

if __name__ == "__main__":
    print("Starting RTP H.264 receiver for ZED camera...")
    receiver = GStreamerReceiver(5000, True)
    print(f"Listening for RTP stream on port {5000}")
    
    def signal_handler(sig, frame):
        print("\nInterrupted by user")
        receiver.stop()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        Gtk.main()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        receiver.stop()
        print("Cleanup complete")
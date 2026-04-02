import carla
import numpy as np
import signal
import threading

from python.streaming.gstream_zed_sender import GStreamerStreamer


class CarlaCameraSensorOp:
    """Camera sensor operator - attaches to vehicle and outputs images."""

    def __init__(
        self,
        name: str,
        host: str = "localhost",
        port: int = 2000,
        width: int = 1280,
        height: int = 720,
    ):
        self._host = host
        self._port = port
        self._width = width
        self._height = height
        self._client = None
        self._world = None
        self._vehicle = None
        self._camera = None
        self._running = threading.Event()
        self._fps = 30
        self._streamer = GStreamerStreamer("127.0.0.1", 5000, self._width, self._height, self._fps)

    def start(self):
        # Connect to CARLA
        self._client = carla.Client(self._host, self._port)
        self._client.set_timeout(10.0)
        self._world = self._client.get_world()

        # Find the hero vehicle
        actors = self._world.get_actors().filter("vehicle.*")
        for actor in actors:
            if actor.attributes.get("role_name") == "hero":
                self._vehicle = actor
                break

        if self._vehicle is None and len(actors) > 0:
            self._vehicle = actors[0]

        if self._vehicle is None:
            raise RuntimeError("No vehicle found in CARLA. Run the spawn script first.")

        print(f"Camera sensor connected to vehicle: {self._vehicle.type_id}")

        # Create and attach camera sensor
        blueprint_library = self._world.get_blueprint_library()
        camera_bp = blueprint_library.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", str(self._width))
        camera_bp.set_attribute("image_size_y", str(self._height))
        camera_bp.set_attribute("fov", "90")

        # Attach camera behind and above the vehicle (chase cam)
        camera_transform = carla.Transform(
            carla.Location(x=-8.0, z=4.0),
            carla.Rotation(pitch=-15.0)
        )
        self._camera = self._world.spawn_actor(
            camera_bp, camera_transform, attach_to=self._vehicle
        )

        # Register callback to receive camera images
        self._running.set()
        self._camera.listen(self._on_camera_image)
        print("Camera sensor attached and listening")

    def _on_camera_image(self, image):
        if not self._running.is_set():
            return
        # Convert CARLA BGRA to BGR bytes and send directly to GStreamer
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((self._height, self._width, 4))
        frame_bgr = np.ascontiguousarray(array[:, :, :3])
        self._streamer.send_frame(frame_bgr.tobytes())

    def stop(self):
        self._running.clear()
        if self._camera is not None:
            self._camera.stop()
            self._camera.destroy()
            print("Camera sensor destroyed")
        self._streamer.stop()

if __name__ == "__main__":
    carla_streamer = CarlaCameraSensorOp("streamer")
    carla_streamer.start()

    def signal_handler(sig, frame):
        print("\nInterrupted by user")
        carla_streamer.stop()

    signal.signal(signal.SIGINT, signal_handler)

    try:
        # Main thread just waits; streaming happens in CARLA's callback thread
        threading.Event().wait()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        carla_streamer.stop()
        print("Cleanup complete")

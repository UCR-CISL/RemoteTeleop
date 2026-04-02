import numpy as np

from holoscan.core import Fragment, Operator, OperatorSpec


class CarlaCameraSensorOp(Operator):
    """Camera sensor operator - attaches to vehicle and outputs images."""

    def __init__(
        self,
        fragment: Fragment,
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
        self._latest_image = None
        super().__init__(fragment, name=name)

    def start(self):
        import carla

        self._client = carla.Client(self._host, self._port)
        self._client.set_timeout(10.0)
        self._world = self._client.get_world()

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

        blueprint_library = self._world.get_blueprint_library()
        camera_bp = blueprint_library.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", str(self._width))
        camera_bp.set_attribute("image_size_y", str(self._height))
        camera_bp.set_attribute("fov", "90")

        camera_transform = carla.Transform(
            carla.Location(x=-8.0, z=4.0),
            carla.Rotation(pitch=-15.0)
        )
        self._camera = self._world.spawn_actor(
            camera_bp, camera_transform, attach_to=self._vehicle
        )

        self._camera.listen(self._on_camera_image)
        print("Camera sensor attached and listening")

    def _on_camera_image(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((self._height, self._width, 4))
        self._latest_image = array[:, :, :3].copy()

    def stop(self):
        if self._camera is not None:
            self._camera.stop()
            self._camera.destroy()
            print("Camera sensor destroyed")

    def setup(self, spec: OperatorSpec):
        pass

    def compute(self, op_input, op_output, context):
        if self._latest_image is not None:
            self._streamer.send_frame(self._latest_image.tobytes())


class CarlaDriveControllerOp(Operator):
    def __init__(self, fragment: Fragment, name: str, host: str = "localhost", port: int = 2000):
        self._host = host
        self._port = port
        self._vehicle = None
        self._world = None
        self._client = None
        super().__init__(fragment, name=name)

    def start(self):
        import carla
        self._client = carla.Client(self._host, self._port)
        self._client.set_timeout(10.0)
        self._world = self._client.get_world()

        settings = self._world.get_settings()
        settings.synchronous_mode = False
        self._world.apply_settings(settings)

        actors = self._world.get_actors().filter("vehicle.*")
        for actor in actors:
            if actor.attributes.get("role_name") == "hero":
                self._vehicle = actor
                break

        if self._vehicle is None and len(actors) > 0:
            self._vehicle = actors[0]

        if self._vehicle is None:
            raise RuntimeError("No vehicle found in Carla. Run the spawn script first.")

        print(f"Connected to vehicle: {self._vehicle.type_id}")

    def stop(self):
        if self._vehicle is not None:
            import carla
            self._vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))

    def setup(self, spec: OperatorSpec):
        spec.input("accel")
        spec.input("steer")

    def compute(self, op_input, op_output, context):
        import carla

        accel = op_input.receive("accel")
        steer = op_input.receive("steer")

        if self._vehicle is None:
            return

        if accel >= 0:
            throttle = accel
            brake = 0.0
        else:
            throttle = 0.0
            brake = abs(accel)

        control = carla.VehicleControl(
            throttle=float(throttle),
            steer=float(steer),
            brake=float(brake),
            hand_brake=False,
            reverse=False,
        )
        self._vehicle.apply_control(control)
        print(f"Control: throttle={throttle:.2f}, steer={steer:.2f}, brake={brake:.2f}")


class KiaDriveControllerOp(Operator):
    def __init__(self, fragment: Fragment, name: str):
        super().__init__(fragment, name=name)

    def setup(self, spec: OperatorSpec):
        spec.input("accel")
        spec.input("steer")

    def compute(self, op_input, op_output, context):
        accel = op_input.receive("accel")
        steer = op_input.receive("steer")
        print(f"Kia Control: accel={accel:.2f}, steer={steer:.2f}")


class VehicleFragment(Fragment):
    def __init__(self, app, name, carla_host="localhost", carla_port=2000):
        self._carla_host = carla_host
        self._carla_port = carla_port
        super().__init__(app, name=name)

    def compose(self):
        drive_controller = CarlaDriveControllerOp(
            self,
            name="drive_controller",
            host=self._carla_host,
            port=self._carla_port,
        )
        # camera_sensor = CarlaCameraSensorOp(
        #     self,
        #     name="camera_sensor",
        #     host=self._carla_host,
        #     port=self._carla_port,
        # )
        self.add_operator(drive_controller)
        # self.add_operator(camera_sensor)

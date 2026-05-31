import rclpy
from holoscan.core import Application

from fragments.remote_workstation_fragment import RemoteWorkstationFragment
from fragments.vehicle_fragment import VehicleFragment

class TeleopApp(Application):
    def __init__(self, carla_host="localhost", carla_port=2000):
        self._carla_host = carla_host
        self._carla_port = carla_port
        super().__init__()

    def compose(self):
        remote_fragment = RemoteWorkstationFragment(
            self,
            name="RemoteWorkstationFragment",
        )
        vehicle_fragment = VehicleFragment(
            self,
            name="VehicleFragment",
            carla_host=self._carla_host,
            carla_port=self._carla_port,
        )

        self.add_fragment(remote_fragment)
        self.add_fragment(vehicle_fragment)

        # Connect keyboard controller outputs to drive controller inputs
        self.add_flow(
            remote_fragment,
            vehicle_fragment,
            {("carla_controller.accel", "drive_controller.accel"),
             ("carla_controller.steer", "drive_controller.steer")},
        )


def main():
    rclpy.init()
    try:
        app = TeleopApp()
        app.run()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
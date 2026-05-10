from holoscan.core import Application

from fragments.steering_wheel_fragment import SteeringWheelFragment
from fragments.vehicle_fragment import PandaFragment


class TeleopApp(Application):
    def compose(self):
        steering_fragment = SteeringWheelFragment(
            self,
            name="SteeringWheelFragment",
        )
        vehicle_fragment = PandaFragment(
            self,
            name="PandaFragment",
        )

        self.add_fragment(steering_fragment)
        self.add_fragment(vehicle_fragment)

        self.add_flow(
            steering_fragment,
            vehicle_fragment,
            {("steering_wheel.throttle", "drive_controller.accel"),
             ("steering_wheel.steering_angle", "drive_controller.steer")},
        )


def main():
    app = TeleopApp()
    app.run()


if __name__ == "__main__":
    main()

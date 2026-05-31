from holoscan.core import Application
from holoscan.schedulers import GreedyScheduler

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
# ypeError: __init__(): incompatible constructor arguments. The following argument types are supported:
#     1. holoscan.schedulers._schedulers.GreedyScheduler(fragment: holoscan.core._core.Fragment, *, clock: holoscan.resources._resources.GXFClock = None, stop_on_deadlock: bool = True, max_duration_ms: int = -1, check_recession_period_ms: float = 0.0, stop_on_deadlock_timeout: int = 0, name: str = 'greedy_scheduler')

# Invoked with: kwargs: stop_on_deadlock_timeout=20000
        self.scheduler(GreedyScheduler(self, stop_on_deadlock_timeout=20000)) # 20 second timeout for deadlock detection


#  python3 distributed_kia_teleop_app.py --driver --worker --address 100.70.20.114 --fragments SteeringWheelFragment
#  python3 distributed_kia_teleop_app.py --driver --master --address 100.70.20.114 --fragments PandaFragment
def main():
    app = TeleopApp()
    app.run()


if __name__ == "__main__":
    main()

from holoscan.core import Application, Fragment, Operator, OperatorSpec
from holoscan.conditions import PeriodicCondition
import pygame
from python.control.steering_wheel_controller import SteeringwheelController

class SteeringWheelOperator(Operator):
    def __init__(self, fragment: Fragment, *args, **kwargs):
        super().__init__(fragment, *args, **kwargs)

    def start(self):
        pygame.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            raise RuntimeError("No joystick detected. Please connect a joystick and try again.")
        elif joystick_count > 1:
            raise RuntimeError(f"Multiple joysticks detected ({joystick_count}). Please connect only one joystick and try again.")
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        self._controller = SteeringwheelController(joystick)

    def setup(self, spec: OperatorSpec):
        # spec.output("accel")
        # spec.output("steering_angle")
        pass

    def compute(self, op_input, op_output, context):
        steering_angle, brake, accel = self._controller.parse_events()
        print("Steering angle:", steering_angle) # -1 to 1, start at 0
        print("Brake:", brake) # 1 to -1, start at 1
        print("Accel:", accel) # -1 to 1, start at -1
        # op_output.emit(throttle, "accel")
        # op_output.emit(steering_angle, "steering_angle")


class SteeringWheelApp(Application):
    def __init__(self):
        super().__init__()

    def compose(self):
        periodic_condition = PeriodicCondition(self, recess_period=10_000_000)
        operator = SteeringWheelOperator(self, periodic_condition, name="steering_wheel_operator")
        self.add_operator(operator)

if __name__ == "__main__":
    app = SteeringWheelApp()
    app.run()
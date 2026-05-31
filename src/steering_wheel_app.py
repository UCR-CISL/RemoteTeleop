from holoscan.core import Application, Fragment, Operator, OperatorSpec
from holoscan.conditions import PeriodicCondition
import pygame
from src.control.steering_wheel_controller import SteeringwheelController

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
        spec.output("throttle")
        spec.output("steering_angle")

    def compute(self, op_input, op_output, context):
        print("Reading joystick input for Steering Wheel Operator...")
        steering_angle, brake, accel = self._controller.parse_events()
        brake = (-brake + 1)/2
        throttle = (-accel + 1)/2
        throttle = throttle - brake
        print(f"Throtte: {throttle} | Steering: {steering_angle}", end="\r")
        op_output.emit(throttle, "throttle")
        op_output.emit(steering_angle, "steering_angle")

class PrintSinkOperator(Operator):
    def setup(self, spec: OperatorSpec):
        spec.input("throttle")
        spec.input("steering_angle")

    def compute(self, op_input, op_output, context):
        throttle = op_input.receive("throttle")
        steering_angle = op_input.receive("steering_angle")
        print(f"[sink] throttle={throttle}, steering={steering_angle}")


class SteeringWheelFragment(Fragment):
    def compose(self):
        steering_wheel_operator = SteeringWheelOperator(
            self,
            PeriodicCondition(self, recess_period=10_000_000),
            name="steering_wheel_operator",
        )
        self.add_operator(steering_wheel_operator)


class PrintSinkFragment(Fragment):
    def compose(self):
        sink = PrintSinkOperator(
            self,
            name="print_sink",
        )
        self.add_operator(sink)


class SteeringWheelApp(Application):
    def __init__(self):
        super().__init__()

    def compose(self):
        steering_wheel_fragment = SteeringWheelFragment(
            self,
            name="SteeringWheelFragment",
        )
        print_sink_fragment = PrintSinkFragment(
            self,
            name="PrintSinkFragment",
        )

        self.add_fragment(steering_wheel_fragment)
        self.add_fragment(print_sink_fragment)

        self.add_flow(
            steering_wheel_fragment,
            print_sink_fragment,
            {
                ("steering_wheel_operator.throttle", "print_sink.throttle"),
                ("steering_wheel_operator.steering_angle", "print_sink.steering_angle"),
            },
        )

if __name__ == "__main__":
    app = SteeringWheelApp()
    app.run()

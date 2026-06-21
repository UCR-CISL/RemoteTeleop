import select
import sys
import termios
import time
import tty

from holoscan.conditions import PeriodicCondition
from holoscan.core import Fragment, Operator, OperatorSpec

from control.keyboard_controller import RobotKeyboardController


class TerminalKeyboardReader:
    KEY_TIMEOUT_SECONDS = 0.15

    def __init__(self):
        self._fd = None
        self._old_settings = None
        self._key_times = {}

    def start(self):
        if not sys.stdin.isatty():
            raise RuntimeError("Terminal keyboard fallback requires an interactive TTY.")
        self._fd = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)

    def stop(self):
        if self._fd is not None and self._old_settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def pressed_keys(self):
        now = time.monotonic()
        while select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key:
                self._key_times[key] = now

        return {
            key
            for key, key_time in self._key_times.items()
            if now - key_time <= self.KEY_TIMEOUT_SECONDS
        }


class KeyboardTeleopOperator(Operator):
    def __init__(self, fragment: Fragment, *args, **kwargs):
        self._controller = RobotKeyboardController()
        self._listener = None
        self._terminal_reader = None
        super().__init__(fragment, *args, **kwargs)

    def setup(self, spec: OperatorSpec):
        spec.output("throttle")
        spec.output("steering_angle")

    def start(self):
        try:
            from pynput import keyboard
        except ImportError:
            self._terminal_reader = TerminalKeyboardReader()
            self._terminal_reader.start()
            print("[KeyboardTeleopOperator] WASD controls ready using terminal input.")
        else:
            self._listener = keyboard.Listener(
                on_press=self._controller.press,
                on_release=self._controller.release,
            )
            self._listener.start()
            print("[KeyboardTeleopOperator] WASD controls ready using pynput.")

    def stop(self):
        if self._listener is not None:
            self._listener.stop()
        if self._terminal_reader is not None:
            self._terminal_reader.stop()

    def compute(self, op_input, op_output, context):
        if self._terminal_reader is not None:
            self._controller.set_pressed_keys(self._terminal_reader.pressed_keys())
        throttle, steering_angle = self._controller.controls()
        print(f"Keyboard throttle={throttle:.2f} | steering={steering_angle:.2f}", end="\r")
        op_output.emit(throttle, "throttle")
        op_output.emit(steering_angle, "steering_angle")


class KeyboardFragment(Fragment):
    def compose(self):
        keyboard_op = KeyboardTeleopOperator(
            self,
            PeriodicCondition(self, recess_period=10_000_000),  # 10 ms -> 100 Hz
            name="keyboard",
        )
        self.add_operator(keyboard_op)

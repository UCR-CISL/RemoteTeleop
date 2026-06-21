import sys
import unittest
from pathlib import Path


V2_DIR = Path(__file__).resolve().parents[1] / "v2"
sys.path.insert(0, str(V2_DIR))

from control.keyboard_controller import RobotKeyboardController


class RobotKeyboardControllerTest(unittest.TestCase):
    def test_wasd_press_and_release_updates_robot_control_axes(self):
        controller = RobotKeyboardController()

        controller.press("w")
        controller.press("d")

        throttle, steering_angle = controller.controls()
        self.assertEqual(throttle, 1.0)
        self.assertEqual(steering_angle, 0.5)

        controller.release("w")
        controller.release("d")

        throttle, steering_angle = controller.controls()
        self.assertEqual(throttle, 0.0)
        self.assertEqual(steering_angle, 0.0)

    def test_opposing_keys_cancel_each_axis(self):
        controller = RobotKeyboardController()

        controller.press("w")
        controller.press("s")
        controller.press("a")
        controller.press("d")

        throttle, steering_angle = controller.controls()
        self.assertEqual(throttle, 0.0)
        self.assertEqual(steering_angle, 0.0)

    def test_uppercase_keys_are_treated_like_lowercase(self):
        controller = RobotKeyboardController()

        controller.press("W")
        controller.press("A")

        throttle, steering_angle = controller.controls()
        self.assertEqual(throttle, 1.0)
        self.assertEqual(steering_angle, -0.5)

    def test_set_pressed_keys_replaces_previous_state(self):
        controller = RobotKeyboardController()

        controller.press("w")
        controller.set_pressed_keys({"s", "a"})

        throttle, steering_angle = controller.controls()
        self.assertEqual(throttle, -1.0)
        self.assertEqual(steering_angle, -0.5)


if __name__ == "__main__":
    unittest.main()

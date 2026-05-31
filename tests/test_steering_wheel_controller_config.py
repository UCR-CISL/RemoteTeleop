from pathlib import Path

import pytest

from src.control.steering_wheel_controller import SteeringWheelConfig, SteeringwheelController


class FakeJoystick:
    def get_numaxes(self) -> int:
        return 4

    def get_axis(self, index: int) -> float:
        values = {
            0: 0.25,
            1: 0.0,
            2: -1.0,
            3: 1.0,
        }
        return values[index]

    def get_numbuttons(self) -> int:
        return 11

    def get_button(self, _index: int) -> int:
        return 0


def write_config(path: Path) -> None:
    path.write_text(
        """
g920_racing_wheel:
  steering_wheel: 0
  throttle: 2
  brake: 3
  clutch: 1
  handbrake: 4
  reverse: 10
sensitivity:
  mode: 0
  min: 0.5
  max: 0.5
""".lstrip(),
        encoding="utf-8",
    )


def test_steering_wheel_controller_reads_yaml_config(tmp_path: Path) -> None:
    config_path = tmp_path / "steering_wheel_config.yaml"
    write_config(config_path)

    controller = SteeringwheelController(FakeJoystick(), config_path=config_path)

    assert controller._steer_idx == 0
    assert controller._throttle_idx == 2
    assert controller._brake_idx == 3
    assert controller._reverse_idx == 10
    assert controller._handbrake_idx == 4
    assert controller.steering_mode == 0
    assert controller.steering_sensitivity_min == 0.5
    assert controller.steering_sensitivity_max == 0.5


def test_steering_wheel_config_updates_and_saves_yaml(tmp_path: Path) -> None:
    config_path = tmp_path / "steering_wheel_config.yaml"
    output_path = tmp_path / "saved.yaml"
    write_config(config_path)
    config = SteeringWheelConfig(config_path)

    config.update_sensitivity(mode=1, minimum=0.25, maximum=0.75)
    config.save(output_path)

    saved = SteeringWheelConfig(output_path)
    assert saved.sensitivity_int("mode") == 1
    assert saved.sensitivity_float("min") == 0.25
    assert saved.sensitivity_float("max") == 0.75


def test_steering_wheel_config_rejects_missing_file(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        SteeringWheelConfig(tmp_path / "missing.yaml")

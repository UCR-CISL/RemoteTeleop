import math
import os
from pathlib import Path
from typing import Any, Tuple

import yaml

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame
from pygame.locals import KMOD_CTRL
from pygame.locals import KMOD_SHIFT
from pygame.locals import K_0
from pygame.locals import K_9
from pygame.locals import K_BACKQUOTE
from pygame.locals import K_BACKSPACE
from pygame.locals import K_COMMA
from pygame.locals import K_DOWN
from pygame.locals import K_ESCAPE
from pygame.locals import K_F1
from pygame.locals import K_LEFT
from pygame.locals import K_PERIOD
from pygame.locals import K_RIGHT
from pygame.locals import K_SLASH
from pygame.locals import K_SPACE
from pygame.locals import K_TAB
from pygame.locals import K_UP
from pygame.locals import K_a
from pygame.locals import K_b
from pygame.locals import K_c
from pygame.locals import K_d
from pygame.locals import K_f
from pygame.locals import K_g
from pygame.locals import K_h
from pygame.locals import K_i
from pygame.locals import K_l
from pygame.locals import K_m
from pygame.locals import K_n
from pygame.locals import K_o
from pygame.locals import K_p
from pygame.locals import K_q
from pygame.locals import K_r
from pygame.locals import K_s
from pygame.locals import K_t
from pygame.locals import K_v
from pygame.locals import K_w
from pygame.locals import K_x
from pygame.locals import K_z
from pygame.locals import K_MINUS
from pygame.locals import K_EQUALS

from src.control import joystick_constants as js


class SteeringWheelConfig:
    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)
        if not self._path.is_file():
            raise FileNotFoundError(f"Steering wheel config not found: {self._path}")
        with self._path.open("r", encoding="utf-8") as file:
            data = yaml.safe_load(file)
        if not isinstance(data, dict):
            raise ValueError(f"Steering wheel config must be a YAML mapping: {self._path}")
        self._data = data

    @property
    def path(self) -> Path:
        return self._path

    @property
    def data(self) -> dict[str, Any]:
        return self._data

    def wheel_int(self, key: str) -> int:
        return int(self._wheel_section()[key])

    def sensitivity_int(self, key: str) -> int:
        return int(self._sensitivity_section()[key])

    def sensitivity_float(self, key: str) -> float:
        return float(self._sensitivity_section()[key])

    def update_sensitivity(self, *, mode: int, minimum: float, maximum: float) -> None:
        sensitivity = self._sensitivity_section()
        sensitivity["mode"] = int(mode)
        sensitivity["min"] = float(minimum)
        sensitivity["max"] = float(maximum)

    def save(self, path: str | Path | None = None) -> None:
        target = Path(path) if path is not None else self._path
        with target.open("w", encoding="utf-8") as file:
            yaml.safe_dump(self._data, file, sort_keys=False)

    def _wheel_section(self) -> dict[str, Any]:
        section = self._data.get("g920_racing_wheel")
        if not isinstance(section, dict):
            raise KeyError("Steering wheel config missing g920_racing_wheel section")
        return section

    def _sensitivity_section(self) -> dict[str, Any]:
        section = self._data.get("sensitivity")
        if not isinstance(section, dict):
            raise KeyError("Steering wheel config missing sensitivity section")
        return section

class SteeringwheelController(object):
    def __init__(self, joystick, config_path=None):
        self._steer_cache = 0.0

        self._joystick = joystick

        if config_path is None:
            config_path = Path(__file__).resolve().parents[2] / "config" / "steering_wheel_config.yaml"
        self._config = SteeringWheelConfig(config_path)
        self._steer_idx = self._config.wheel_int("steering_wheel")
        self._throttle_idx = self._config.wheel_int("throttle")
        self._brake_idx = self._config.wheel_int("brake")
        self._reverse_idx = self._config.wheel_int("reverse")
        self._handbrake_idx = self._config.wheel_int("handbrake")

        self.steering_mode = self._config.sensitivity_int("mode")
        self.steering_sensitivity_min = self._config.sensitivity_float("min")
        self.steering_sensitivity_max = self._config.sensitivity_float("max")

        self._mph = 0
        self._accel = 0.0
        self._brake = 0.0
        self._steering_angle = 0.0

    def parse_events(self) -> Tuple[float, float, float]:
        pygame.event.pump()

        self._parse_vehicle_wheel()
        # Currently, the Pandarunner's reverse gear is not available
        # self._control.reverse = self._control.gear < 0

        return self._steering_angle, self._brake, self._accel

    def _parse_vehicle_keys(self, keys, milliseconds):
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    def _parse_vehicle_wheel(self):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        jsButtons = [float(self._joystick.get_button(i)) for i in
                     range(self._joystick.get_numbuttons())]

        steerCmd = jsInputs[self._steer_idx]


        K2 = 1.6
        x = jsInputs[self._throttle_idx]

        # Original nonlinear computation
        y = K2 + (2.05 * math.log10(-0.7 * x + 1.4) - 1.2) / 0.92

        # Determine original output range (can be computed from min/max of x)
        y_min =-0.049509802142144954
        y_max = 1.0136408197875375

        # Scale to 0-0.75
        throttleCmd = (y - y_min) * 0.75 / (y_max - y_min)

        #Speed limit
        if self._mph >=45 :
            throttleCmd = 0 

        brakeCmd = 1.6 + (2.05 * math.log10(

            -0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1

        self._steering_angle = steerCmd
        self._brake = jsInputs[self._brake_idx] 
        self._accel = jsInputs[self._throttle_idx] 

    def update_steering_config(self, steering_config):
        self.steering_mode = steering_config[0]
        self.steering_sensitivity_min = steering_config[1]
        self.steering_sensitivity_max = steering_config[2]
        self._config.update_sensitivity(
            mode=self.steering_mode,
            minimum=self.steering_sensitivity_min,
            maximum=self.steering_sensitivity_max,
        )

    def save_config_file(self):
        self._config.save("wheel_config.yaml")

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)

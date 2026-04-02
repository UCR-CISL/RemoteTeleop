import os
os.environ["SDL_VIDEODRIVER"] = "dummy"


import pygame
import math
from configparser import ConfigParser
from typing import Tuple

import math
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

from python.control import joystick_constants as js

class SteeringwheelController(object):
    def __init__(self, joystick):
        self._steer_cache = 0.0

        self._joystick = joystick

        self._parser = ConfigParser()
        self._parser.read('/home/justin/Documents/CISL-Projects/BasicTeleop/config/steering_wheel_config.ini')
        self._steer_idx = int(
            self._parser.get('G920 Racing Wheel', 'steering_wheel'))
        self._throttle_idx = int(
            self._parser.get('G920 Racing Wheel', 'throttle'))
        self._brake_idx = int(self._parser.get('G920 Racing Wheel', 'brake'))
        self._reverse_idx = int(self._parser.get('G920 Racing Wheel', 'reverse'))
        self._handbrake_idx = int(self._parser.get('G920 Racing Wheel', 'handbrake'))

        self.steering_mode = int(self._parser.get('Sensitivity', 'mode'))
        self.steering_sensitivity_min = float(self._parser.get('Sensitivity', 'min'))
        self.steering_sensitivity_max = float(self._parser.get('Sensitivity', 'max'))

        self._mph = 0
        self._accel = 0.0
        self._brake = 0.0
        self._steering_angle = 0.0

    def parse_events(self) -> Tuple[float, float]:
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

        steerCmd = jsInputs[self._steer_idx] * 0.5


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
        self._parser.set('Sensitivity', 'mode', str(self.steering_mode))
        self._parser.set('Sensitivity', 'min', str(self.steering_sensitivity_min))
        self._parser.set('Sensitivity', 'max', str(self.steering_sensitivity_max))

    def save_config_file(self):
        with open('wheel_config.ini', 'w') as config_file:  # save
            self._parser.write(config_file)

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)
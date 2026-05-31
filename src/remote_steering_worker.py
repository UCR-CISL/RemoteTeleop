#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import signal
import time

import pygame
import zmq

from src.control.steering_wheel_controller import SteeringwheelController
from src.control.teleop_command import TeleopCommand


DEFAULT_BIND = "tcp://*:5555"
DEFAULT_TOPIC = "kia_control"


class RemoteSteeringWorker:
    def __init__(self, args: argparse.Namespace):
        self._bind = args.bind
        self._topic = args.topic
        self._rate_hz = args.rate_hz
        self._config = args.config
        self._verbose = args.verbose
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.PUB)
        self._controller = None
        self._steering_neutral = 0.0
        self._sequence = 0
        self._running = True

    def run(self) -> None:
        self._register_signal_handlers()
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.bind(self._bind)

        joystick = self._open_single_joystick()
        self._controller = SteeringwheelController(joystick, config_path=self._config)
        self._capture_steering_neutral()
        period_s = 1.0 / self._rate_hz

        print(f"Publishing steering commands on {self._bind} topic={self._topic!r} at {self._rate_hz:.1f} Hz")
        try:
            while self._running:
                started = time.monotonic()
                self._publish_next_command()
                time.sleep(max(0.0, period_s - (time.monotonic() - started)))
        finally:
            self._publish_neutral()
            self._socket.close()
            self._context.term()
            pygame.quit()
            print("\nRemote steering worker stopped.")

    def stop(self, _signum=None, _frame=None) -> None:
        self._running = False

    def _register_signal_handlers(self) -> None:
        signal.signal(signal.SIGINT, self.stop)
        signal.signal(signal.SIGTERM, self.stop)

    def _open_single_joystick(self) -> pygame.joystick.Joystick:
        pygame.init()
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            raise RuntimeError("No joystick detected. Connect the steering wheel and try again.")
        if joystick_count > 1:
            raise RuntimeError(
                f"Multiple joysticks detected ({joystick_count}). Connect only the steering wheel."
            )
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Using joystick: {joystick.get_name()}")
        return joystick

    def _publish_next_command(self) -> None:
        steering_angle, raw_brake_axis, raw_accel_axis = self._controller.parse_events()
        command = TeleopCommand.from_wheel_axes(
            sequence=self._sequence,
            steering_angle=steering_angle - self._steering_neutral,
            raw_accel_axis=raw_accel_axis,
            raw_brake_axis=raw_brake_axis,
        )
        self._publish(command)
        if self._verbose:
            print(
                f"seq={command.sequence} accel={command.accel:+.3f} "
                f"steer={command.steer:+.3f} throttle={command.throttle:.3f} brake={command.brake:.3f}",
                end="\r",
                flush=True,
            )
        self._sequence += 1

    def _publish_neutral(self) -> None:
        self._publish(
            TeleopCommand(
                sequence=self._sequence,
                timestamp_ns=time.time_ns(),
                accel=0.0,
                steer=0.0,
                throttle=0.0,
                brake=0.0,
            )
        )

    def _publish(self, command: TeleopCommand) -> None:
        payload = json.dumps(command.to_dict(), separators=(",", ":"))
        self._socket.send_string(f"{self._topic} {payload}")

    def _capture_steering_neutral(self) -> None:
        steering_angle, _raw_brake_axis, _raw_accel_axis = self._controller.parse_events()
        self._steering_neutral = steering_angle
        print(f"Steering neutral offset: {self._steering_neutral:+.3f}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Read steering wheel input and publish Kia teleop commands over ZMQ.")
    parser.add_argument("--bind", default=DEFAULT_BIND, help="ZMQ PUB bind address.")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="ZMQ topic prefix.")
    parser.add_argument("--rate-hz", type=float, default=100.0, help="Publish rate.")
    parser.add_argument("--config", default=None, help="Steering wheel config path.")
    parser.add_argument("--verbose", action="store_true", help="Print live control values.")
    return parser


def main() -> None:
    RemoteSteeringWorker(build_parser().parse_args()).run()


if __name__ == "__main__":
    main()

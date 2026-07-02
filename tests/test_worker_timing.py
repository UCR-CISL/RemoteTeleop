from src.control.vehicle_command import VehicleControlCommand
from src.keyboard_control_worker import build_parser as build_keyboard_parser
from src.panda_worker import PandaWorker
from src.remote_steering_worker import build_parser as build_steering_parser


def test_keyboard_worker_defaults_to_50_hz() -> None:
    args = build_keyboard_parser().parse_args([])

    assert args.rate_hz == 50.0


def test_remote_steering_worker_defaults_to_50_hz() -> None:
    args = build_steering_parser().parse_args([])

    assert args.rate_hz == 50.0


def test_panda_worker_applies_only_freshly_received_commands() -> None:
    worker = PandaWorker.__new__(PandaWorker)
    command = VehicleControlCommand(sequence=3, timestamp_ns=10, steer=0.25, accel=0.1, throttle=0.1, brake=0.0)
    worker._last_command = command
    worker._neutral_sent_after_timeout = False
    worker._poll_once = lambda: True

    assert worker._next_command_to_apply() is command


def test_panda_worker_does_not_repeat_current_command_without_message(monkeypatch) -> None:
    worker = PandaWorker.__new__(PandaWorker)
    worker._last_command = VehicleControlCommand(sequence=3, timestamp_ns=10, steer=0.25, accel=0.1, throttle=0.1, brake=0.0)
    worker._last_received = 1.0
    worker._command_timeout = 10.0
    worker._neutral_sent_after_timeout = False
    worker._poll_once = lambda: False
    monkeypatch.setattr("src.panda_worker.time.monotonic", lambda: 2.0)

    assert worker._next_command_to_apply() is None


def test_panda_worker_sends_one_neutral_after_timeout(monkeypatch) -> None:
    worker = PandaWorker.__new__(PandaWorker)
    worker._last_command = VehicleControlCommand(sequence=4, timestamp_ns=10, steer=0.25, accel=0.1, throttle=0.1, brake=0.0)
    worker._last_received = 1.0
    worker._command_timeout = 0.25
    worker._neutral_sent_after_timeout = False
    worker._poll_once = lambda: False
    monkeypatch.setattr("src.panda_worker.time.monotonic", lambda: 2.0)

    neutral = worker._next_command_to_apply()

    assert neutral is not None
    assert neutral.sequence == 4
    assert neutral.steer == 0.0
    assert neutral.accel == 0.0
    assert neutral.throttle == 0.0
    assert neutral.brake == 0.0
    assert worker._next_command_to_apply() is None

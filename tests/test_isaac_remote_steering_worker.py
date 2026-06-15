import time
from pathlib import Path

from isaacteleop.retargeters import VehicleControlRetargeter, VehicleControlRetargeterConfig
from isaacteleop.schema import SteeringWheelOutput, VehicleControlCommand as IsaacVehicleControlCommand

from src.control.vehicle_retargeter import SteeringWheelSample
from src.isaac_remote_steering_worker import (
    DEFAULT_COLLECTION_ID,
    DEFAULT_CONFIG_PATH,
    DEFAULT_DEVICE_PATH,
    DEFAULT_AXIS_MAPPING,
    NativeSteeringWheelPlugin,
    build_parser,
    isaac_command_to_wire_command,
    resolve_axis_mapping,
    wire_sample_from_isaac_sample,
)


def test_isaac_remote_steering_worker_defaults_to_50_hz() -> None:
    args = build_parser().parse_args([])

    assert args.rate_hz == 50.0
    assert args.device == DEFAULT_DEVICE_PATH
    assert args.config == str(DEFAULT_CONFIG_PATH)
    assert args.collection_id == DEFAULT_COLLECTION_ID
    assert not args.no_start_plugin


def test_isaac_remote_steering_worker_uses_repo_axis_config_by_default() -> None:
    args = build_parser().parse_args([])

    assert resolve_axis_mapping(args) == {
        "steering_axis": 0,
        "throttle_axis": 2,
        "brake_axis": 3,
        "clutch_axis": 1,
    }


def test_isaac_remote_steering_worker_axis_overrides_win_over_config() -> None:
    args = build_parser().parse_args(
        [
            "--steering-axis",
            "4",
            "--throttle-axis",
            "5",
            "--brake-axis",
            "6",
            "--clutch-axis",
            "-1",
        ]
    )

    assert resolve_axis_mapping(args) == {
        "steering_axis": 4,
        "throttle_axis": 5,
        "brake_axis": 6,
        "clutch_axis": -1,
    }


def test_isaac_remote_steering_worker_can_use_plugin_default_axis_mapping() -> None:
    args = build_parser().parse_args(["--config", ""])

    assert resolve_axis_mapping(args) == DEFAULT_AXIS_MAPPING


def test_native_plugin_command_uses_device_collection_and_axes() -> None:
    plugin = NativeSteeringWheelPlugin(
        binary=Path("/tmp/steering_wheel_plugin"),
        device_path="/dev/input/js2",
        collection_id="wheel",
        steering_axis=3,
        throttle_axis=4,
        brake_axis=5,
        clutch_axis=-1,
    )

    assert plugin.command == [
        "/tmp/steering_wheel_plugin",
        "/dev/input/js2",
        "wheel",
        "3",
        "4",
        "5",
        "-1",
    ]


def test_isaac_command_converts_to_existing_wire_command() -> None:
    isaac_command = IsaacVehicleControlCommand(7, -0.25, 0.5, 0.5, 0.0)

    command = isaac_command_to_wire_command(isaac_command, timestamp_ns=123)

    assert command.to_dict() == {
        "sequence": 7,
        "timestamp_ns": 123,
        "steer": -0.25,
        "accel": 0.5,
        "throttle": 0.5,
        "brake": 0.0,
    }


def test_isaac_retargeter_outputs_vehicle_wire_shape() -> None:
    retargeter = VehicleControlRetargeter(
        VehicleControlRetargeterConfig(steer_scale=-1.0),
        steering_neutral=0.0,
    )
    sample = SteeringWheelOutput(0.25, -1.0, 1.0, 0.0)

    command = isaac_command_to_wire_command(
        retargeter.retarget(sample, sequence=2),
        timestamp_ns=time.time_ns(),
    )

    assert command.sequence == 2
    assert command.steer == -0.25
    assert command.accel == 1.0
    assert command.throttle == 1.0
    assert command.brake == 0.0


def test_isaac_raw_axis_flow_uses_inverted_full_range_pedal_axes() -> None:
    isaac_retargeter = VehicleControlRetargeter(
        VehicleControlRetargeterConfig(steer_scale=-1.0),
        steering_neutral=0.0,
    )

    isaac_command = isaac_command_to_wire_command(
        isaac_retargeter.retarget(
            SteeringWheelOutput(0.25, -1.0, 1.0, 0.0),
            sequence=2,
        ),
        timestamp_ns=123,
    )

    assert isaac_command.to_dict() == {
        "sequence": 2,
        "timestamp_ns": 123,
        "steer": -0.25,
        "accel": 1.0,
        "throttle": 1.0,
        "brake": 0.0,
    }


def test_isaac_sample_converts_to_existing_mcap_sample_shape() -> None:
    sample = SteeringWheelOutput(-0.5, -1.0, 1.0, 0.0)

    wire_sample = wire_sample_from_isaac_sample(sample, timestamp_ns=456)

    assert wire_sample.to_dict() == {
        "steering": -0.5,
        "accel_axis": -1.0,
        "brake_axis": 1.0,
        "timestamp_ns": 456,
    }

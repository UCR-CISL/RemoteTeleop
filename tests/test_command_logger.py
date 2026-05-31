import json

import pytest

from src.control.command_logger import JsonlCommandLogger
from src.control.vehicle_command import VehicleControlCommand
from src.control.vehicle_retargeter import SteeringWheelSample


def test_jsonl_command_logger_writes_sample_and_command(tmp_path):
    path = tmp_path / "commands.jsonl"
    sample = SteeringWheelSample(steering=-0.5, accel_axis=-1.0, brake_axis=-1.0, timestamp_ns=10)
    command = VehicleControlCommand.neutral(sequence=4, timestamp_ns=11)

    with JsonlCommandLogger(path) as logger:
        logger.write(sample=sample, command=command)

    records = [json.loads(line) for line in path.read_text().splitlines()]
    assert records == [
        {
            "sample": {
                "steering": -0.5,
                "accel_axis": -1.0,
                "brake_axis": -1.0,
                "timestamp_ns": 10,
            },
            "command": {
                "sequence": 4,
                "timestamp_ns": 11,
                "steer": 0.0,
                "accel": 0.0,
                "throttle": 0.0,
                "brake": 0.0,
            },
        }
    ]


def test_jsonl_command_logger_requires_context(tmp_path):
    logger = JsonlCommandLogger(tmp_path / "commands.jsonl")

    with pytest.raises(RuntimeError):
        logger.write(
            sample=SteeringWheelSample(steering=0.0, accel_axis=0.0, brake_axis=0.0, timestamp_ns=1),
            command=VehicleControlCommand.neutral(),
        )


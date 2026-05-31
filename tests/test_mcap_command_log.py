import pytest

from src.control.mcap_command_log import McapCommandLogReader, McapCommandLogger
from src.control.vehicle_command import VehicleControlCommand
from src.control.vehicle_retargeter import SteeringWheelSample


def test_mcap_command_logger_writes_sample_and_command(tmp_path):
    path = tmp_path / "commands.mcap"
    sample = SteeringWheelSample(steering=-0.5, accel_axis=-1.0, brake_axis=-1.0, timestamp_ns=10)
    command = VehicleControlCommand.neutral(sequence=4, timestamp_ns=11)

    with McapCommandLogger(path) as logger:
        logger.write(sample=sample, command=command)

    readback = list(McapCommandLogReader(path).records())
    assert len(readback) == 1
    assert readback[0].sample == sample
    assert readback[0].command == command


def test_mcap_command_logger_requires_context(tmp_path):
    logger = McapCommandLogger(tmp_path / "commands.mcap")

    with pytest.raises(RuntimeError):
        logger.write(
            sample=SteeringWheelSample(steering=0.0, accel_axis=0.0, brake_axis=0.0, timestamp_ns=1),
            command=VehicleControlCommand.neutral(),
        )

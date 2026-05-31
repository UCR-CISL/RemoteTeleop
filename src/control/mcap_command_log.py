from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import BinaryIO, Iterable

from mcap.reader import make_reader
from mcap.writer import CompressionType, Writer

from src.control.vehicle_command import VehicleControlCommand
from src.control.vehicle_retargeter import SteeringWheelSample


VEHICLE_COMMAND_TOPIC = "vehicle_control"
VEHICLE_COMMAND_SCHEMA = {
    "type": "object",
    "properties": {
        "sample": {
            "type": "object",
            "properties": {
                "steering": {"type": "number"},
                "accel_axis": {"type": "number"},
                "brake_axis": {"type": "number"},
                "timestamp_ns": {"type": "integer"},
            },
            "required": ["steering", "accel_axis", "brake_axis", "timestamp_ns"],
        },
        "command": {
            "type": "object",
            "properties": {
                "sequence": {"type": "integer"},
                "timestamp_ns": {"type": "integer"},
                "steer": {"type": "number"},
                "accel": {"type": "number"},
                "throttle": {"type": "number"},
                "brake": {"type": "number"},
            },
            "required": ["sequence", "timestamp_ns", "steer", "accel", "throttle", "brake"],
        },
    },
    "required": ["sample", "command"],
}


@dataclass(frozen=True)
class CommandLogRecord:
    sample: SteeringWheelSample
    command: VehicleControlCommand

    @classmethod
    def from_dict(cls, value: dict) -> "CommandLogRecord":
        sample = value["sample"]
        return cls(
            sample=SteeringWheelSample(
                steering=float(sample["steering"]),
                accel_axis=float(sample["accel_axis"]),
                brake_axis=float(sample["brake_axis"]),
                timestamp_ns=int(sample["timestamp_ns"]),
            ),
            command=VehicleControlCommand.from_dict(value["command"]),
        )

    def to_dict(self) -> dict:
        return {
            "sample": self.sample.to_dict(),
            "command": self.command.to_dict(),
        }

    def to_json(self) -> bytes:
        return json.dumps(self.to_dict(), separators=(",", ":")).encode("utf-8")


class McapCommandLogger:
    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)
        self._file: BinaryIO | None = None
        self._writer: Writer | None = None
        self._channel_id: int | None = None

    def __enter__(self) -> "McapCommandLogger":
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self._path.open("wb")
        self._writer = Writer(self._file, compression=CompressionType.NONE)
        self._writer.start()
        schema_id = self._writer.register_schema(
            name="remote_teleop.VehicleControlCommandRecord",
            encoding="jsonschema",
            data=json.dumps(VEHICLE_COMMAND_SCHEMA, separators=(",", ":")).encode("utf-8"),
        )
        self._channel_id = self._writer.register_channel(
            topic=VEHICLE_COMMAND_TOPIC,
            message_encoding="json",
            schema_id=schema_id,
        )
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        if self._writer is not None:
            self._writer.finish()
            self._writer = None
            self._channel_id = None
        if self._file is not None:
            self._file.close()
            self._file = None

    def write(self, *, sample: SteeringWheelSample, command: VehicleControlCommand) -> None:
        if self._writer is None or self._channel_id is None:
            raise RuntimeError("McapCommandLogger must be opened before writing")
        record = CommandLogRecord(sample=sample, command=command)
        self._writer.add_message(
            channel_id=self._channel_id,
            log_time=command.timestamp_ns,
            publish_time=command.timestamp_ns,
            sequence=command.sequence,
            data=record.to_json(),
        )


class McapCommandLogReader:
    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)

    def records(self) -> Iterable[CommandLogRecord]:
        with self._path.open("rb") as file:
            reader = make_reader(file)
            for _schema, _channel, message in reader.iter_messages(topics=[VEHICLE_COMMAND_TOPIC]):
                yield CommandLogRecord.from_dict(json.loads(message.data.decode("utf-8")))


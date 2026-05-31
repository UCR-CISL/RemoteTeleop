from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Iterable

from src.control.vehicle_command import VehicleControlCommand
from src.control.vehicle_retargeter import SteeringWheelSample


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


class JsonlCommandLogReader:
    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)

    def records(self) -> Iterable[CommandLogRecord]:
        with self._path.open("r", encoding="utf-8") as file:
            for line in file:
                if not line.strip():
                    continue
                yield CommandLogRecord.from_dict(json.loads(line))

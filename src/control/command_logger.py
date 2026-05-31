from __future__ import annotations

import json
from pathlib import Path
from typing import TextIO

from src.control.vehicle_command import VehicleControlCommand
from src.control.vehicle_retargeter import SteeringWheelSample


class JsonlCommandLogger:
    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)
        self._file: TextIO | None = None

    def __enter__(self) -> "JsonlCommandLogger":
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self._path.open("a", encoding="utf-8")
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None

    def write(self, *, sample: SteeringWheelSample, command: VehicleControlCommand) -> None:
        if self._file is None:
            raise RuntimeError("JsonlCommandLogger must be opened before writing")
        record = {
            "sample": sample.to_dict(),
            "command": command.to_dict(),
        }
        self._file.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._file.flush()


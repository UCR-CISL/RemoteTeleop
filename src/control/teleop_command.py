from __future__ import annotations

from dataclasses import asdict, dataclass
import time
from typing import Any


@dataclass(frozen=True)
class TeleopCommand:
    sequence: int
    timestamp_ns: int
    accel: float
    steer: float
    throttle: float
    brake: float

    @classmethod
    def from_wheel_axes(
        cls,
        *,
        sequence: int,
        steering_angle: float,
        raw_accel_axis: float,
        raw_brake_axis: float,
    ) -> "TeleopCommand":
        pedal_accel = axis_to_pedal(raw_accel_axis)
        pedal_brake = axis_to_pedal(raw_brake_axis)
        net_accel = clamp(pedal_accel - pedal_brake, -1.0, 1.0)
        return cls(
            sequence=sequence,
            timestamp_ns=time.time_ns(),
            accel=net_accel,
            steer=clamp(steering_angle, -1.0, 1.0),
            throttle=net_accel if net_accel > 0.0 else 0.0,
            brake=-net_accel if net_accel < 0.0 else 0.0,
        )

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "TeleopCommand":
        return cls(
            sequence=int(value["sequence"]),
            timestamp_ns=int(value["timestamp_ns"]),
            accel=float(value["accel"]),
            steer=float(value["steer"]),
            throttle=float(value.get("throttle", 0.0)),
            brake=float(value.get("brake", 0.0)),
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def axis_to_pedal(axis_value: float) -> float:
    return clamp((-float(axis_value) + 1.0) / 2.0, 0.0, 1.0)


def clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, float(value)))

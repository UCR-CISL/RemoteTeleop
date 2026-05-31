from __future__ import annotations

from dataclasses import dataclass
import time

from src.control.vehicle_command import VehicleControlCommand, clamp


@dataclass(frozen=True)
class SteeringWheelSample:
    steering: float
    accel_axis: float
    brake_axis: float
    timestamp_ns: int

    @classmethod
    def now(cls, *, steering: float, accel_axis: float, brake_axis: float) -> "SteeringWheelSample":
        return cls(
            steering=steering,
            accel_axis=accel_axis,
            brake_axis=brake_axis,
            timestamp_ns=time.time_ns(),
        )

    def to_dict(self) -> dict[str, float | int]:
        return {
            "steering": self.steering,
            "accel_axis": self.accel_axis,
            "brake_axis": self.brake_axis,
            "timestamp_ns": self.timestamp_ns,
        }


@dataclass(frozen=True)
class VehicleControlRetargeterConfig:
    steering_deadzone: float = 0.01
    pedal_deadzone: float = 0.01
    steer_scale: float = 1.0
    accel_scale: float = 1.0
    brake_scale: float = 1.0


class VehicleControlRetargeter:
    def __init__(
        self,
        config: VehicleControlRetargeterConfig | None = None,
        *,
        steering_neutral: float = 0.0,
    ) -> None:
        self._config = config or VehicleControlRetargeterConfig()
        self._steering_neutral = steering_neutral

    @property
    def steering_neutral(self) -> float:
        return self._steering_neutral

    def calibrate_neutral(self, sample: SteeringWheelSample) -> None:
        self._steering_neutral = sample.steering

    def retarget(self, sample: SteeringWheelSample, *, sequence: int) -> VehicleControlCommand:
        steer = self._apply_deadzone(
            (sample.steering - self._steering_neutral) * self._config.steer_scale,
            self._config.steering_deadzone,
        )
        throttle = self._apply_deadzone(
            axis_to_pedal(sample.accel_axis) * self._config.accel_scale,
            self._config.pedal_deadzone,
        )
        brake = self._apply_deadzone(
            axis_to_pedal(sample.brake_axis) * self._config.brake_scale,
            self._config.pedal_deadzone,
        )
        accel = clamp(throttle - brake, -1.0, 1.0)

        return VehicleControlCommand(
            sequence=sequence,
            timestamp_ns=sample.timestamp_ns,
            steer=clamp(steer, -1.0, 1.0),
            accel=accel,
            throttle=accel if accel > 0.0 else 0.0,
            brake=-accel if accel < 0.0 else 0.0,
        )

    @staticmethod
    def _apply_deadzone(value: float, threshold: float) -> float:
        return 0.0 if abs(value) <= threshold else value


def axis_to_pedal(axis_value: float) -> float:
    return clamp((-float(axis_value) + 1.0) / 2.0, 0.0, 1.0)


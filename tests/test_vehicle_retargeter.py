from src.control.vehicle_retargeter import (
    SteeringWheelSample,
    VehicleControlRetargeter,
    VehicleControlRetargeterConfig,
    axis_to_pedal,
)


def sample(steering=0.0, accel_axis=-1.0, brake_axis=-1.0):
    return SteeringWheelSample(
        steering=steering,
        accel_axis=accel_axis,
        brake_axis=brake_axis,
        timestamp_ns=123,
    )


def test_axis_to_pedal_maps_g923_idle_and_pressed_values():
    assert axis_to_pedal(-1.0) == 1.0
    assert axis_to_pedal(1.0) == 0.0
    assert axis_to_pedal(0.0) == 0.5


def test_idle_throttle_and_brake_cancel_to_neutral_command():
    retargeter = VehicleControlRetargeter(steering_neutral=-0.5)

    command = retargeter.retarget(
        sample(steering=-0.5, accel_axis=-1.0, brake_axis=-1.0),
        sequence=7,
    )

    assert command.sequence == 7
    assert command.timestamp_ns == 123
    assert command.steer == 0.0
    assert command.accel == 0.0
    assert command.throttle == 0.0
    assert command.brake == 0.0


def test_throttle_only_outputs_positive_accel():
    retargeter = VehicleControlRetargeter(steering_neutral=0.0)

    command = retargeter.retarget(
        sample(accel_axis=-1.0, brake_axis=1.0),
        sequence=1,
    )

    assert command.accel == 1.0
    assert command.throttle == 1.0
    assert command.brake == 0.0


def test_brake_only_outputs_negative_accel_and_positive_brake():
    retargeter = VehicleControlRetargeter(steering_neutral=0.0)

    command = retargeter.retarget(
        sample(accel_axis=1.0, brake_axis=-1.0),
        sequence=1,
    )

    assert command.accel == -1.0
    assert command.throttle == 0.0
    assert command.brake == 1.0


def test_steering_neutral_and_deadzone_are_applied():
    retargeter = VehicleControlRetargeter(
        VehicleControlRetargeterConfig(steering_deadzone=0.05),
        steering_neutral=-0.5,
    )

    assert retargeter.retarget(sample(steering=-0.47), sequence=1).steer == 0.0
    assert retargeter.retarget(sample(steering=-0.40), sequence=1).steer == 0.09999999999999998


def test_negative_steer_scale_inverts_steering_after_neutral():
    retargeter = VehicleControlRetargeter(
        VehicleControlRetargeterConfig(steer_scale=-1.0),
        steering_neutral=0.0,
    )

    assert retargeter.retarget(sample(steering=1.0), sequence=1).steer == -1.0
    assert retargeter.retarget(sample(steering=-1.0), sequence=1).steer == 1.0


def test_calibrate_neutral_updates_steering_offset():
    retargeter = VehicleControlRetargeter()
    retargeter.calibrate_neutral(sample(steering=-0.5))

    command = retargeter.retarget(sample(steering=-0.5), sequence=1)

    assert retargeter.steering_neutral == -0.5
    assert command.steer == 0.0

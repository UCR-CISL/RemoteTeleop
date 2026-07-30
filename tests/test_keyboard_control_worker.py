from src.keyboard_control_worker import KeyboardControlState


def test_wasd_updates_persistent_axes_like_vehicle_example() -> None:
    state = KeyboardControlState(axis_increment=0.05)

    assert state.apply_key("w")
    assert state.apply_key("a")
    assert state.gas_brake == 0.05
    assert state.steer == 0.05

    assert state.apply_key("s")
    assert state.apply_key("d")
    assert state.gas_brake == 0.0
    assert state.steer == 0.0


def test_keyboard_state_clamps_axes() -> None:
    state = KeyboardControlState(axis_increment=0.2)

    for _ in range(10):
        state.apply_key("w")
        state.apply_key("a")

    assert state.gas_brake == 1.0
    assert state.steer == 1.0


def test_keyboard_command_splits_throttle_and_brake() -> None:
    state = KeyboardControlState(axis_increment=0.25)
    state.apply_key("s")

    command = state.to_command(sequence=7)

    assert command.sequence == 7
    assert command.accel == -0.25
    assert command.throttle == 0.0
    assert command.brake == 0.25


def test_reset_and_quit_keys() -> None:
    state = KeyboardControlState(axis_increment=0.5)
    state.apply_key("w")
    state.apply_key("a")

    assert state.apply_key("r")
    assert state.gas_brake == 0.0
    assert state.steer == 0.0

    assert state.apply_key("q")
    assert state.quit_requested

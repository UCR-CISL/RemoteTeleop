from src.isaac_keyboard_control_worker import IsaacKeyboardControlState, build_parser


def test_isaac_keyboard_worker_defaults_to_50_hz() -> None:
    args = build_parser().parse_args([])

    assert args.rate_hz == 50.0


def test_isaac_keyboard_state_outputs_isaac_steering_sample() -> None:
    state = IsaacKeyboardControlState(axis_increment=0.25)
    state.apply_key("w")
    state.apply_key("a")

    sample = state.to_isaac_sample()

    assert sample.steering == 0.25
    assert sample.throttle == 0.5
    assert sample.brake == 1.0
    assert sample.clutch == 0.0

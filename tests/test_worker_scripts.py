from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def test_plain_keyboard_runner_keeps_non_isaac_worker_path() -> None:
    script = (REPO_ROOT / "scripts" / "run_keyboard_control_worker.sh").read_text()

    assert 'repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"' in script
    assert 'export PYTHONPATH="${repo_root}${PYTHONPATH:+:${PYTHONPATH}}"' in script
    assert "exec uv run python -m src.keyboard_control_worker" in script
    assert "src.isaac_keyboard_control_worker" not in script


def test_isaac_keyboard_runner_uses_separate_isaac_worker() -> None:
    script = (REPO_ROOT / "scripts" / "run_isaac_keyboard_control_worker.sh").read_text()

    assert 'REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"' in script
    assert 'export PYTHONPATH="${REPO_ROOT}:${PYTHONPATH:-}"' in script
    assert 'exec "${REPO_ROOT}/.venv/bin/python" -m src.isaac_keyboard_control_worker --rate-hz 50 "$@"' in script


def test_isaac_teleop_keyboard_example_runner_exists() -> None:
    script_path = (
        REPO_ROOT
        / "thirdparty"
        / "IsaacTeleop"
        / "examples"
        / "vehicle_teleop"
        / "scripts"
        / "run_isaac_keyboard_control_worker.sh"
    )

    script = script_path.read_text()
    assert 'EXAMPLE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"' in script
    assert 'export PYTHONPATH="${EXAMPLE_ROOT}/python${PYTHONPATH:+:${PYTHONPATH}}"' in script
    assert "python -m vehicle_teleop.isaac_keyboard_control_worker" in script
    assert "--rate-hz 50" in script

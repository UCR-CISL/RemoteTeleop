#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}:${PYTHONPATH:-}"

exec "${REPO_ROOT}/.venv/bin/python" -m src.isaac_keyboard_control_worker --rate-hz 50 "$@"

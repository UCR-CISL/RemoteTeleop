#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="${repo_root}${PYTHONPATH:+:${PYTHONPATH}}"

exec uv run python -m src.streaming.cloudxr_streamer "$@"

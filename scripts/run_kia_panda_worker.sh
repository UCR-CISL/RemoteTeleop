#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}:${REPO_ROOT}/thirdparty/kia-opendbc:${REPO_ROOT}/thirdparty/panda:${PYTHONPATH:-}"

exec "${REPO_ROOT}/.venv/bin/python" -m src.kia_panda_worker "$@"

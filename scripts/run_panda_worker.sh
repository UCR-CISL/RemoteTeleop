#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OPENDBC_ROOT="${OPENDBC_ROOT:-}"
if [[ -z "${OPENDBC_ROOT}" ]]; then
    for candidate in "${REPO_ROOT}"/thirdparty/*opendbc; do
        if [[ -d "${candidate}" ]]; then
            OPENDBC_ROOT="${candidate}"
            break
        fi
    done
fi
export PYTHONPATH="${REPO_ROOT}:${OPENDBC_ROOT}:${REPO_ROOT}/thirdparty/panda:${PYTHONPATH:-}"

exec "${REPO_ROOT}/.venv/bin/python" -m src.panda_worker "$@"

#!/usr/bin/env bash
set -euo pipefail

# Reusable unattended uv command. Codex may replace only the command below.
export UV_CACHE_DIR=/tmp/remote-teleop-uv-cache
export UV_CONCURRENT_DOWNLOADS=4
export UV_CONCURRENT_BUILDS=2
export MAX_JOBS=2
export UV_PROJECT_ENVIRONMENT=.venv

uv pip install --python .venv/bin/python \
  --index-strategy unsafe-best-match \
  --extra-index-url https://download.pytorch.org/whl/cu121 \
  setuptools==80.9.0 wheel ninja numpy==1.26.4 torch==2.5.1
uv sync --group dev --group reconstruction

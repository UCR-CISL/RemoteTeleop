#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${repo_root}${PYTHONPATH:+:${PYTHONPATH}}"

exec env \
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  LD_PRELOAD=/lib/x86_64-linux-gnu/libavutil.so.56 \
  GST_DEBUG="GST_TRACER:7" \
  GST_TRACERS="interlatency" \
  python3 -m src.streaming.ros_sender --width 1280 --height 720 --fps 30 "$@"

"""Shared process health and metrics helpers."""

from __future__ import annotations

import json
import os
from pathlib import Path
from time import time_ns
from typing import Any, Mapping

from src.realtime.protocol import WorkerHealth, WorkerState
from src.realtime.transport import PublisherTransport


class HealthReporter:
    """Publish a worker snapshot; callers control heartbeat cadence."""

    def __init__(
        self,
        publisher: PublisherTransport,
        *,
        worker_id: str,
        device: str,
    ) -> None:
        self._publisher = publisher
        self.worker_id = worker_id
        self.device = device

    def publish(
        self,
        state: WorkerState,
        *,
        queue_depth: int = 0,
        detail: str | None = None,
    ) -> None:
        allocated, reserved = _cuda_memory_mib()
        self._publisher.send(
            WorkerHealth(
                worker_id=self.worker_id,
                state=state,
                timestamp_us=time_ns() // 1_000,
                pid=os.getpid(),
                device=self.device,
                queue_depth=queue_depth,
                cuda_allocated_mib=allocated,
                cuda_reserved_mib=reserved,
                detail=detail,
            )
        )


class JsonlMetrics:
    """Append crash-tolerant, process-local metric events."""

    def __init__(self, path: str | Path | None) -> None:
        self.path = None if path is None else Path(path)
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, event: str, values: Mapping[str, Any] | None = None) -> None:
        if self.path is None:
            return
        payload = {
            "event": event,
            "recorded_at_us": time_ns() // 1_000,
            **dict(values or {}),
        }
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, allow_nan=False, default=str) + "\n")


def _cuda_memory_mib() -> tuple[float, float]:
    try:
        import torch

        if torch.cuda.is_available():
            mib = 1024.0 * 1024.0
            return (
                torch.cuda.memory_allocated() / mib,
                torch.cuda.memory_reserved() / mib,
            )
    except (ImportError, RuntimeError):
        pass
    return 0.0, 0.0

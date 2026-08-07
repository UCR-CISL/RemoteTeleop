"""Small process-runtime utilities shared by the real-time executables."""

from __future__ import annotations

from threading import Event, Lock, Thread
from typing import Any, Mapping

import zmq

from src.realtime.process_support import HealthReporter, JsonlMetrics
from src.realtime.protocol import WorkerState
from src.realtime.transport import PublisherTransport


class JsonlMetricsWriter(JsonlMetrics):
    """Compatibility adapter for callers that construct a complete event."""

    def append(self, values: Mapping[str, Any]) -> None:
        record = dict(values)
        event = str(record.pop("event"))
        self.write(event, record)


class HealthHeartbeat:
    """Publish health from a socket owned solely by a background thread."""

    def __init__(
        self,
        *,
        endpoint: str,
        worker_id: str,
        device: str,
        interval_seconds: float = 1.0,
    ) -> None:
        if interval_seconds <= 0:
            raise ValueError("interval_seconds must be positive")
        self.endpoint = endpoint
        self.worker_id = worker_id
        self.device = device
        self.interval_seconds = interval_seconds
        self._lock = Lock()
        self._state = WorkerState.LOADING
        self._queue_depth = 0
        self._detail: str | None = None
        self._stop = Event()
        self._thread: Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = Thread(
            target=self._run,
            name=f"{self.worker_id}-health",
            daemon=True,
        )
        self._thread.start()

    def update(
        self,
        state: WorkerState,
        *,
        queue_depth: int = 0,
        detail: str | None = None,
    ) -> None:
        with self._lock:
            self._state = WorkerState(state)
            self._queue_depth = queue_depth
            self._detail = detail

    def stop(self) -> None:
        with self._lock:
            if self._state is not WorkerState.FAILED:
                self._state = WorkerState.STOPPING
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(2.0, self.interval_seconds * 2))
            self._thread = None

    def _snapshot(self) -> tuple[WorkerState, int, str | None]:
        with self._lock:
            state = self._state
            queue_depth = self._queue_depth
            detail = self._detail
        return state, queue_depth, detail

    def _run(self) -> None:
        context = zmq.Context()
        publisher = PublisherTransport.open(
            context,
            self.endpoint,
            bind=False,
            high_water_mark=10,
        )
        reporter = HealthReporter(
            publisher, worker_id=self.worker_id, device=self.device
        )
        try:
            while True:
                state, queue_depth, detail = self._snapshot()
                reporter.publish(state, queue_depth=queue_depth, detail=detail)
                if self._stop.wait(self.interval_seconds):
                    state, queue_depth, detail = self._snapshot()
                    reporter.publish(state, queue_depth=queue_depth, detail=detail)
                    break
        finally:
            publisher.close()
            context.term()

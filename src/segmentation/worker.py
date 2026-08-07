"""Transport-independent lifecycle for a dedicated segmentation process."""

from __future__ import annotations

from .backends import MaskBackend
from .models import FramePrompt, MaskBatch, WorkerState


class MaskWorker:
    """Own one mask backend for the lifetime of a spawned worker process."""

    def __init__(self, backend: MaskBackend) -> None:
        self.backend = backend

    @property
    def state(self) -> WorkerState:
        return self.backend.state

    @property
    def ready(self) -> bool:
        return self.state is WorkerState.READY

    def start(
        self, warmup_frame: FramePrompt | None = None, *, warmup_iterations: int = 3
    ) -> None:
        self.backend.load()
        self.backend.warmup(warmup_frame, iterations=warmup_iterations)

    def process(self, frame: FramePrompt) -> MaskBatch:
        if not self.ready:
            raise RuntimeError("mask worker must finish load and warmup before processing")
        return self.backend.predict(frame)

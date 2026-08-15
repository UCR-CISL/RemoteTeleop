"""Cross-process CUDA model residency coordination."""

from __future__ import annotations

from dataclasses import dataclass
import fcntl
import os
from pathlib import Path
import time


@dataclass(frozen=True)
class ResidencyAcquisition:
    owner: str
    wait_seconds: float


class GpuResidencyLease:
    """A nonblocking, process-scoped exclusive lease backed by ``flock``."""

    def __init__(self, path: str | Path, *, owner: str) -> None:
        if not owner:
            raise ValueError("owner must not be empty")
        self.path = Path(path)
        self.owner = owner
        self._stream: object | None = None
        self._waiting_since: float | None = None

    @property
    def held(self) -> bool:
        return self._stream is not None

    def try_acquire(self) -> ResidencyAcquisition | None:
        if self._stream is not None:
            return ResidencyAcquisition(self.owner, 0.0)
        now = time.monotonic()
        if self._waiting_since is None:
            self._waiting_since = now
        self.path.parent.mkdir(parents=True, exist_ok=True)
        stream = self.path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            stream.close()
            return None
        wait_seconds = now - self._waiting_since
        self._waiting_since = None
        stream.seek(0)
        stream.truncate()
        stream.write(f"{self.owner} pid={os.getpid()}\n")
        stream.flush()
        self._stream = stream
        return ResidencyAcquisition(self.owner, wait_seconds)

    def release(self) -> None:
        stream = self._stream
        if stream is None:
            return
        self._stream = None
        fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
        stream.close()

    def close(self) -> None:
        self.release()

    def __enter__(self) -> "GpuResidencyLease":
        acquisition = self.try_acquire()
        if acquisition is None:
            raise RuntimeError(f"GPU residency is held by another process: {self.path}")
        return self

    def __exit__(self, *_: object) -> None:
        self.release()


def release_cuda_memory() -> None:
    """Collect Python objects, then return unused Torch blocks to CUDA."""

    import gc

    gc.collect()
    try:
        import torch

        if torch.cuda.is_available():
            torch.cuda.synchronize()
            torch.cuda.empty_cache()
            torch.cuda.ipc_collect()
    except (ImportError, RuntimeError):
        pass

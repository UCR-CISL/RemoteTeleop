"""Non-blocking reconstruction lifecycle and duplicate suppression."""

from __future__ import annotations

from collections import deque
import json
from pathlib import Path
from threading import Lock

from .mesh_alignment import MeshAligner
from .models import (
    ReconstructedAsset,
    ReconstructionJob,
    ReconstructionStatus,
)
from .sam3d import ObjectReconstructor


class ReconstructionCoordinator:
    """Own one reconstruction state per track and a FIFO work queue.

    Calling ``submit`` never runs model inference.  A worker process/thread calls
    ``run_next``; the renderer only reads immutable snapshots through ``get``.
    """

    def __init__(self, reconstructor: ObjectReconstructor, aligner: MeshAligner) -> None:
        self._reconstructor = reconstructor
        self._aligner = aligner
        self._queue: deque[ReconstructionJob] = deque()
        self._states: dict[str, ReconstructedAsset] = {}
        self._lock = Lock()

    def submit(self, job: ReconstructionJob, *, retry_failed: bool = False) -> ReconstructedAsset:
        with self._lock:
            existing = self._states.get(job.track_id)
            if existing is not None and (
                existing.status is not ReconstructionStatus.FAILED or not retry_failed
            ):
                return existing
            queued = ReconstructedAsset(
                request_id=job.request_id,
                track_id=job.track_id,
                status=ReconstructionStatus.QUEUED,
                measured_dimensions=job.dimensions,
                world_T_object=job.world_T_object,
                metadata=dict(job.metadata),
            )
            self._states[job.track_id] = queued
            self._queue.append(job)
            return queued

    def get(self, track_id: str) -> ReconstructedAsset | None:
        with self._lock:
            return self._states.get(track_id)

    @property
    def pending_count(self) -> int:
        with self._lock:
            return len(self._queue)

    def run_next(self) -> ReconstructedAsset | None:
        with self._lock:
            if not self._queue:
                return None
            job = self._queue.popleft()
            self._states[job.track_id] = ReconstructedAsset(
                request_id=job.request_id,
                track_id=job.track_id,
                status=ReconstructionStatus.RUNNING,
                measured_dimensions=job.dimensions,
                world_T_object=job.world_T_object,
                metadata=dict(job.metadata),
            )
        try:
            raw_path = self._reconstructor.reconstruct(job)
            aligned_path = _aligned_path(job.output_path)
            alignment = self._aligner.align(raw_path, aligned_path, job.dimensions)
            backend_metadata = _backend_metadata(raw_path)
            result = ReconstructedAsset(
                request_id=job.request_id,
                track_id=job.track_id,
                status=ReconstructionStatus.READY,
                raw_mesh_path=raw_path,
                aligned_mesh_path=alignment.output_path,
                object_T_raw_mesh=alignment.object_T_raw_mesh,
                uniform_scale=alignment.uniform_scale,
                measured_dimensions=job.dimensions,
                aligned_dimensions=alignment.aligned_dimensions,
                world_T_object=job.world_T_object,
                metadata={
                    **job.metadata,
                    **backend_metadata,
                    "dimension_residual_m": alignment.dimension_residual.tolist(),
                    "ground_offset_m": alignment.ground_offset,
                    "center_error_m": alignment.center_error,
                    "ground_error_m": alignment.ground_error,
                },
            )
        except Exception as exc:  # A model failure must not kill the worker loop.
            result = ReconstructedAsset(
                request_id=job.request_id,
                track_id=job.track_id,
                status=ReconstructionStatus.FAILED,
                measured_dimensions=job.dimensions,
                world_T_object=job.world_T_object,
                error=f"{type(exc).__name__}: {exc}",
                metadata=dict(job.metadata),
            )
        with self._lock:
            self._states[job.track_id] = result
        return result


def _aligned_path(raw_path: Path) -> Path:
    return raw_path.with_name(f"{raw_path.stem}.aligned{raw_path.suffix}")


def _backend_metadata(raw_path: Path) -> dict:
    sidecar = raw_path.with_suffix(".sam3d.json")
    if not sidecar.is_file():
        return {}
    try:
        return {"reconstruction": json.loads(sidecar.read_text(encoding="utf-8"))}
    except (OSError, json.JSONDecodeError):
        return {"reconstruction_metadata_error": f"could not read {sidecar}"}

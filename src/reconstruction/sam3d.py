"""SAM 3D Objects adapter with no import-time CUDA/model dependency."""

from __future__ import annotations

from collections.abc import Callable
from importlib import import_module
from pathlib import Path
from typing import Any, Mapping, Protocol

from .models import ReconstructionJob


class ObjectReconstructor(Protocol):
    def load(self) -> None:
        """Load model weights and allocate persistent inference state."""

    def reconstruct(self, job: ReconstructionJob) -> Path:
        """Reconstruct ``job`` and return a renderer-readable mesh path."""

    def unload(self) -> None:
        """Release resident accelerator state while retaining service state."""

    def suspend(self) -> Mapping[str, Any]:
        """Offload accelerator state while retaining initialized host state."""

    def resume(self) -> Mapping[str, Any]:
        """Restore retained state to its inference device."""


class SAM3DObjectReconstructor:
    """Lazy adapter around a checkout-specific SAM 3D Objects entrypoint.

    Upstream SAM 3D Objects is not a stable Python package.  The configured
    factory must return an object with ``reconstruct(image=..., mask=...,
    output_path=..., seed=...)`` or a callable with the same keyword arguments.
    This narrow seam keeps upstream-specific setup out of the coordinator.
    """

    def __init__(
        self,
        *,
        factory: Callable[..., Any] | None = None,
        factory_path: str | None = None,
        factory_kwargs: Mapping[str, Any] | None = None,
    ) -> None:
        if factory is None and factory_path is None:
            raise ValueError("factory or factory_path is required")
        self._factory = factory
        self._factory_path = factory_path
        self._factory_kwargs = dict(factory_kwargs or {})
        self._backend: Any | None = None

    def _load_backend(self) -> Any:
        if self._backend is not None:
            return self._backend
        factory = self._factory
        if factory is None:
            assert self._factory_path is not None
            module_name, separator, attribute = self._factory_path.partition(":")
            if not separator or not module_name or not attribute:
                raise ValueError("factory_path must have the form 'module.path:factory'")
            try:
                factory = getattr(import_module(module_name), attribute)
            except (ImportError, AttributeError) as exc:
                raise RuntimeError(f"cannot load SAM 3D factory {self._factory_path!r}") from exc
        self._backend = factory(**self._factory_kwargs)
        return self._backend

    def load(self) -> None:
        """Eagerly construct and load the backend before accepting work."""

        backend = self._load_backend()
        load = getattr(backend, "load", None)
        if load is not None:
            load()

    def resume(self) -> Mapping[str, Any]:
        """Resume a retained backend, or lazily construct it on first use."""

        backend = self._load_backend()
        resume = getattr(backend, "resume", None)
        if resume is not None:
            return dict(resume() or {})
        load = getattr(backend, "load", None)
        if load is not None:
            load()
        return {"residency_action": "load_fallback"}

    def suspend(self) -> Mapping[str, Any]:
        """Prefer CPU offload; destroy only backends without that lifecycle."""

        backend = self._backend
        if backend is None:
            return {"residency_action": "not_loaded"}
        suspend = getattr(backend, "suspend", None)
        if suspend is not None:
            return dict(suspend() or {})
        self.unload()
        return {"residency_action": "destroy_fallback"}

    def reconstruct(self, job: ReconstructionJob) -> Path:
        backend = self._load_backend()
        job.output_path.parent.mkdir(parents=True, exist_ok=True)
        kwargs = {
            "job": job,
            "image": job.observation.image,
            "mask": job.mask,
            "output_path": job.output_path,
            "seed": job.seed,
        }
        result = backend.reconstruct(**kwargs) if hasattr(backend, "reconstruct") else backend(**kwargs)
        path = Path(result) if isinstance(result, (str, Path)) else job.output_path
        if not path.is_file() or path.stat().st_size == 0:
            raise RuntimeError(f"SAM 3D did not produce a non-empty mesh at {path}")
        return path

    def unload(self) -> None:
        backend = self._backend
        self._backend = None
        if backend is not None:
            unload = getattr(backend, "unload", None)
            if unload is not None:
                unload()
        from src.realtime.gpu_residency import release_cuda_memory

        release_cuda_memory()

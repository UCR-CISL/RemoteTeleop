"""Concrete adapter for the pinned Meta SAM 3D Objects submodule."""

from __future__ import annotations

from collections import OrderedDict
import hashlib
import os
import json
from pathlib import Path
import sys
import time
from types import MethodType
from typing import Any

import numpy as np

from .models import ReconstructionJob


SAM3D_CPU_UNSUPPORTED_REASON = (
    "the pinned SAM3D Objects backend is CUDA-only: its upstream pipeline uses "
    "CUDA autocast, CUDA sparse operators, and hard-coded CUDA tensor placement"
)


def validate_sam3d_device(device: str) -> str:
    """Reject devices the pinned upstream implementation cannot execute on."""

    normalized = str(device).strip().casefold()
    is_cuda = normalized == "cuda" or (
        normalized.startswith("cuda:") and normalized.removeprefix("cuda:").isdigit()
    )
    if not is_cuda:
        raise ValueError(f"SAM3D device {device!r} is unsupported; {SAM3D_CPU_UNSUPPORTED_REASON}")
    return str(device)


def _sam3d_nf4_roots(inference: Any) -> tuple[Any, ...]:
    """Return independently-owned torch modules from the upstream container."""

    roots = [
        *inference.models.values(),
        *inference.condition_embedders.values(),
    ]
    # The MoGe pipeline wrapper is not itself an nn.Module.
    depth_model = getattr(inference.depth_model, "model", inference.depth_model)
    roots.append(depth_model)
    return tuple(
        root
        for root in roots
        if hasattr(root, "parameters") and hasattr(root, "modules")
    )


def _sam3d_transfer_roots(inference: Any) -> tuple[Any, ...]:
    """Enumerate each independently-owned stateful module exactly once."""

    candidates: list[Any] = []
    models = getattr(inference, "models", {})
    candidates.extend(models.values() if hasattr(models, "values") else (models,))
    embedders = getattr(inference, "condition_embedders", {})
    candidates.extend(
        embedders.values() if hasattr(embedders, "values") else (embedders,)
    )
    depth = getattr(inference, "depth_model", None)
    delegate = getattr(depth, "_delegate", depth)
    candidates.append(getattr(delegate, "model", delegate))
    candidates.extend(
        getattr(inference, name, None)
        for name in ("pose_decoder", "ss_preprocessor", "slat_preprocessor")
    )
    roots: list[Any] = []
    seen: set[int] = set()
    for candidate in candidates:
        if candidate is None or not callable(getattr(candidate, "to", None)):
            continue
        identity = id(candidate)
        if identity not in seen:
            seen.add(identity)
            roots.append(candidate)
    return tuple(roots)


class UpstreamSAM3DBackend:
    """Load Meta's public inference wrapper once and export its GLB result."""

    def __init__(
        self,
        *,
        repository: str | Path,
        config_path: str | Path,
        compile_model: bool = False,
        precision: str = "default",
        stage1_inference_steps: int | None = None,
        stage2_inference_steps: int | None = None,
        pointmap_cache_size: int = 0,
        device: str = "cuda:0",
    ) -> None:
        if precision not in {"default", "fp16", "nf4"}:
            raise ValueError("precision must be 'default', 'fp16', or 'nf4'")
        if precision == "nf4" and compile_model:
            raise ValueError("NF4 inference does not support model compilation")
        for name, value in (
            ("stage1_inference_steps", stage1_inference_steps),
            ("stage2_inference_steps", stage2_inference_steps),
        ):
            if value is not None and value <= 0:
                raise ValueError(f"{name} must be positive when provided")
        if pointmap_cache_size < 0:
            raise ValueError("pointmap_cache_size must be non-negative")
        self.repository = Path(repository).resolve()
        config = Path(config_path)
        self.config_path = (self.repository / config).resolve() if not config.is_absolute() else config
        self.compile_model = compile_model
        self.precision = precision
        self.stage1_inference_steps = stage1_inference_steps
        self.stage2_inference_steps = stage2_inference_steps
        self.pointmap_cache_size = pointmap_cache_size
        self.device = validate_sam3d_device(device)
        self._inference: Any | None = None
        self._depth_cache: _SameFrameDepthCache | None = None
        self._load_metrics: dict[str, Any] = {}
        self._suspended = False
        self._last_transfer_metrics: dict[str, Any] = {}

    def _load(self) -> Any:
        if self._inference is not None:
            return self._inference
        if not self.config_path.is_file():
            raise FileNotFoundError(f"SAM3D pipeline config not found: {self.config_path}")
        package_path = self.repository / "sam3d_objects"
        if not package_path.is_dir():
            raise FileNotFoundError(f"SAM3D checkout is incomplete: {self.repository}")

        os.environ.setdefault("LIDRA_SKIP_INIT", "true")
        repository_path = str(self.repository)
        if repository_path not in sys.path:
            sys.path.insert(0, repository_path)
        try:
            from hydra.utils import instantiate
            from omegaconf import OmegaConf
            import sam3d_objects  # noqa: F401 - registers Hydra targets
        except ImportError as exc:
            raise RuntimeError(
                "SAM3D inference dependencies are incomplete; sync the reconstruction group"
            ) from exc
        config = OmegaConf.load(self.config_path)
        config.rendering_engine = "pytorch3d"
        config.compile_model = self.compile_model
        config.device = self.device
        if self.precision == "fp16":
            config.dtype = "float16"
            config.shape_model_dtype = "float16"
        effective_dtype = str(config.dtype)
        effective_shape_dtype = str(config.get("shape_model_dtype", config.dtype))
        # The MVP exports mesh vertex colors, so neither Gaussian decoder is
        # needed. Avoid their weights and decode buffers on 24 GiB GPUs.
        config.decode_formats = ["mesh"]
        config.slat_decoder_gs_config_path = None
        config.slat_decoder_gs_ckpt_path = None
        config.slat_decoder_gs_4_config_path = None
        config.slat_decoder_gs_4_ckpt_path = None
        config.workspace_dir = str(self.config_path.parent)
        load_started = time.perf_counter()
        _reset_cuda_peaks()
        self._inference = instantiate(config)
        quantization_metrics: dict[str, int | float] = {}
        if self.precision == "nf4":
            from src.common.nf4 import quantize_dense_linear_nf4

            quantization_metrics = quantize_dense_linear_nf4(
                _sam3d_nf4_roots(self._inference), device=self.device
            ).as_dict()
        if self.pointmap_cache_size > 0:
            self._depth_cache = _SameFrameDepthCache(
                self._inference.depth_model,
                capacity=self.pointmap_cache_size,
            )
            self._inference.depth_model = self._depth_cache
        self._inference.postprocess_slat_output = MethodType(
            _mesh_only_postprocess, self._inference
        )
        _synchronize_cuda()
        self._load_metrics = {
            "model_load_seconds": time.perf_counter() - load_started,
            "requested_precision": self.precision,
            "effective_dtype": effective_dtype,
            "effective_shape_model_dtype": effective_shape_dtype,
            "pointmap_cache_size": self.pointmap_cache_size,
            **quantization_metrics,
            **_cuda_peak_metrics("model_load_"),
        }
        return self._inference

    def load(self) -> None:
        """Eagerly load the resident upstream pipeline."""

        if self._inference is None:
            self._load()
        elif self._suspended:
            self.resume()

    @property
    def last_transfer_metrics(self) -> dict[str, Any]:
        return dict(self._last_transfer_metrics)

    def suspend(self) -> dict[str, Any]:
        """Move persistent model state to CPU without rebuilding the pipeline."""

        inference = self._inference
        if inference is None or self._suspended:
            return {"residency_action": "already_suspended"}
        started = time.perf_counter()
        if self._depth_cache is not None:
            self._depth_cache.clear()
        roots = _sam3d_transfer_roots(inference)
        for module in roots:
            module.to("cpu")
        _set_pipeline_device(inference, "cpu")
        from src.realtime.gpu_residency import release_cuda_memory

        release_cuda_memory()
        self._suspended = True
        self._last_transfer_metrics = {
            "residency_action": "cpu_offload",
            "offload_transfer_seconds": time.perf_counter() - started,
            "transferred_module_roots": len(roots),
        }
        return self.last_transfer_metrics

    def resume(self) -> dict[str, Any]:
        """Move the retained pipeline back to its configured CUDA device."""

        if self._inference is None:
            started = time.perf_counter()
            self._load()
            self._last_transfer_metrics = {
                "residency_action": "initial_load",
                "initial_load_seconds": time.perf_counter() - started,
            }
            return self.last_transfer_metrics
        if not self._suspended:
            return {"residency_action": "already_resident"}
        started = time.perf_counter()
        roots = _sam3d_transfer_roots(self._inference)
        for module in roots:
            module.to(self.device)
        _set_pipeline_device(self._inference, self.device)
        _synchronize_cuda()
        self._suspended = False
        self._last_transfer_metrics = {
            "residency_action": "cuda_resume",
            "resume_transfer_seconds": time.perf_counter() - started,
            "transferred_module_roots": len(roots),
        }
        return self.last_transfer_metrics

    def unload(self) -> None:
        """Release the upstream pipeline and cached CUDA tensors."""

        self._inference = None
        self._depth_cache = None
        self._load_metrics = {}
        self._suspended = False
        from src.realtime.gpu_residency import release_cuda_memory

        release_cuda_memory()

    def reconstruct(
        self,
        *,
        job: ReconstructionJob,
        image: np.ndarray,
        mask: np.ndarray,
        output_path: Path,
        seed: int,
    ) -> Path:
        if self._suspended:
            self.resume()
        inference = self._load()
        rgba = np.concatenate(
            (image[..., :3], (mask.astype(np.uint8) * 255)[..., None]), axis=-1
        )
        _reset_cuda_peaks()
        _synchronize_cuda()
        started = time.perf_counter()
        output = inference.run(
            rgba,
            None,
            seed,
            stage1_only=False,
            with_mesh_postprocess=False,
            with_texture_baking=False,
            with_layout_postprocess=False,
            use_vertex_color=True,
            stage1_inference_steps=self.stage1_inference_steps,
            stage2_inference_steps=self.stage2_inference_steps,
        )
        _synchronize_cuda()
        inference_seconds = time.perf_counter() - started
        glb = output.get("glb")
        if glb is None:
            raise RuntimeError("SAM3D output did not contain a mesh/GLB")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        glb.export(str(output_path))
        if not output_path.is_file() or output_path.stat().st_size == 0:
            raise RuntimeError(f"SAM3D failed to export {output_path}")
        metadata = {
            "backend": "facebookresearch/sam-3d-objects",
            "repository": str(self.repository),
            "repository_commit": _git_commit(self.repository),
            "config_path": str(self.config_path),
            "seed": seed,
            **self._load_metrics,
            "inference_seconds": inference_seconds,
            "stage1_inference_steps": self.stage1_inference_steps or 25,
            "stage2_inference_steps": self.stage2_inference_steps or 25,
            **(self._depth_cache.last_call_metrics if self._depth_cache else {}),
            "inference_objects_per_second": 1.0 / inference_seconds,
            **_cuda_peak_metrics("inference_"),
            "predicted_rotation_wxyz": _to_list(output.get("rotation")),
            "predicted_translation": _to_list(output.get("translation")),
            "predicted_scale": _to_list(output.get("scale")),
        }
        output_path.with_suffix(".sam3d.json").write_text(
            json.dumps(metadata, indent=2), encoding="utf-8"
        )
        return output_path


class _SameFrameDepthCache:
    """Bounded exact cache for mask-independent MoGe RGB depth outputs."""

    def __init__(self, delegate: Any, *, capacity: int) -> None:
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        self._delegate = delegate
        self._capacity = capacity
        self._cache: OrderedDict[bytes, dict[str, Any]] = OrderedDict()
        self._hits = 0
        self._misses = 0
        self._evictions = 0
        self._last_hit = False
        self._last_seconds = 0.0

    def __call__(self, image: Any) -> dict[str, Any]:
        array = image.detach().cpu().contiguous().numpy()
        digest = hashlib.blake2b(digest_size=16)
        digest.update(str(array.shape).encode("ascii"))
        digest.update(array.dtype.str.encode("ascii"))
        digest.update(memoryview(array))
        key = digest.digest()
        started = time.perf_counter()
        cached = self._cache.get(key)
        if cached is not None:
            self._cache.move_to_end(key)
            self._hits += 1
            self._last_hit = True
            self._last_seconds = time.perf_counter() - started
            return dict(cached)
        output = self._delegate(image)
        if getattr(output.get("pointmaps"), "is_cuda", False):
            _synchronize_cuda()
        cached = {
            name: value.detach() if hasattr(value, "detach") else value
            for name, value in output.items()
            if name in {"pointmaps", "intrinsics"}
        }
        if "pointmaps" not in cached:
            raise RuntimeError("MoGe depth model output did not contain pointmaps")
        self._cache[key] = cached
        self._cache.move_to_end(key)
        while len(self._cache) > self._capacity:
            self._cache.popitem(last=False)
            self._evictions += 1
        self._misses += 1
        self._last_hit = False
        self._last_seconds = time.perf_counter() - started
        return output

    def clear(self) -> None:
        """Release cached pointmaps, which may otherwise retain CUDA tensors."""

        self._cache.clear()
        self._last_hit = False
        self._last_seconds = 0.0

    @property
    def last_call_metrics(self) -> dict[str, object]:
        return {
            "pointmap_cache_hit": self._last_hit,
            "pointmap_cache_hits": self._hits,
            "pointmap_cache_misses": self._misses,
            "pointmap_cache_evictions": self._evictions,
            "depth_model_call_seconds": self._last_seconds,
        }

    def __getattr__(self, name: str) -> Any:
        return getattr(self._delegate, name)


def _set_pipeline_device(inference: Any, device: str) -> None:
    import torch

    resolved = torch.device(device)
    inference.device = resolved
    depth = getattr(inference, "depth_model", None)
    delegate = getattr(depth, "_delegate", depth)
    if delegate is not None and hasattr(delegate, "device"):
        delegate.device = resolved


def _to_list(value: Any) -> Any:
    if value is None:
        return None
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value).tolist()


def _git_commit(repository: Path) -> str | None:
    git_entry = repository / ".git"
    if git_entry.is_file():
        text = git_entry.read_text(encoding="utf-8").strip()
        if text.startswith("gitdir:"):
            git_entry = (repository / text.split(":", 1)[1].strip()).resolve()
    if not git_entry.is_dir() or not (git_entry / "HEAD").is_file():
        return None
    head = (git_entry / "HEAD").read_text(encoding="utf-8").strip()
    if head.startswith("ref:"):
        reference = git_entry / head.split(":", 1)[1].strip()
        return reference.read_text(encoding="utf-8").strip() if reference.is_file() else None
    return head


def _reset_cuda_peaks() -> None:
    try:
        import torch

        if torch.cuda.is_available():
            torch.cuda.reset_peak_memory_stats()
    except (ImportError, RuntimeError):
        pass


def _synchronize_cuda() -> None:
    try:
        import torch

        if torch.cuda.is_available():
            torch.cuda.synchronize()
    except (ImportError, RuntimeError):
        pass


def _cuda_peak_metrics(prefix: str) -> dict[str, float]:
    try:
        import torch

        if torch.cuda.is_available():
            mib = 1024.0 * 1024.0
            return {
                f"{prefix}gpu_peak_allocated_mib": torch.cuda.max_memory_allocated() / mib,
                f"{prefix}gpu_peak_reserved_mib": torch.cuda.max_memory_reserved() / mib,
            }
    except (ImportError, RuntimeError):
        pass
    return {}


def _mesh_only_postprocess(
    _pipeline: Any,
    outputs: dict[str, Any],
    with_mesh_postprocess: bool,
    _with_texture_baking: bool,
    _use_vertex_color: bool,
) -> dict[str, Any]:
    """Build a vertex-colored GLB without the unused Gaussian appearance model."""
    from sam3d_objects.model.backbone.tdfy_dit.utils import postprocessing_utils

    meshes = outputs.get("mesh")
    if not meshes:
        outputs["glb"] = None
        return outputs
    outputs["glb"] = postprocessing_utils.to_glb(
        None,
        meshes[0],
        with_mesh_postprocess=with_mesh_postprocess,
        with_texture_baking=False,
        use_vertex_color=True,
    )
    return outputs

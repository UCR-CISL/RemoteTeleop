"""Lifecycle-managed box-prompted mask backends.

The official SAM 3 image processor exposes a frame-level image encoding but
only a single-box geometric prompt method.  This adapter therefore encodes
each frame once, resets prompt state between tracks, and performs one decode
per box.  The public backend API remains frame-batched so a future upstream
batched-box API can replace the decode loop without changing callers.
"""

from __future__ import annotations

from collections.abc import Callable
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter
from types import MethodType
from typing import Any, Mapping, Protocol

import numpy as np
from numpy.typing import NDArray

from .models import (
    BoxPrompt,
    FramePrompt,
    MaskBatch,
    MaskResult,
    MaskWorkerMetrics,
    WorkerState,
)


class MaskBackend(Protocol):
    @property
    def state(self) -> WorkerState: ...

    def load(self) -> None: ...

    def warmup(self, frame: FramePrompt | None = None, *, iterations: int = 3) -> None: ...

    def predict(self, frame: FramePrompt) -> MaskBatch: ...


class SAM3ImagePredictor(Protocol):
    """Small seam around the unstable upstream SAM 3 processor API."""

    def set_image(self, image: NDArray[np.uint8]) -> object: ...

    def predict_box(
        self, state: object, normalized_cxcywh: tuple[float, float, float, float]
    ) -> Mapping[str, Any]: ...

    def predict_boxes(
        self,
        state: object,
        boxes_xyxy: tuple[tuple[float, float, float, float], ...],
    ) -> tuple[Mapping[str, Any], ...]: ...


@dataclass(frozen=True)
class SAM3MaskBackendConfig:
    checkpoint: Path | None = None
    device: str = "cuda"
    confidence_threshold: float = 0.5
    minimum_mask_in_box_fraction: float = 0.5
    minimum_box_coverage: float = 0.05
    compile: bool = False
    precision: str = "default"
    prompt_mode: str = "serial_grounding"

    def __post_init__(self) -> None:
        if self.precision not in {"default", "fp16"}:
            raise ValueError("precision must be 'default' or 'fp16'")
        if self.prompt_mode not in {"serial_grounding", "interactive_batch"}:
            raise ValueError(
                "prompt_mode must be 'serial_grounding' or 'interactive_batch'"
            )
        for name in (
            "confidence_threshold",
            "minimum_mask_in_box_fraction",
            "minimum_box_coverage",
        ):
            value = getattr(self, name)
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be between zero and one")


class _OfficialSAM3ImagePredictor:
    def __init__(self, config: SAM3MaskBackendConfig) -> None:
        try:
            from sam3.model.sam3_image_processor import Sam3Processor
            from sam3.model_builder import (
                build_sam3_image_model,
                download_ckpt_from_hf,
            )
            from PIL import Image
        except ImportError as exc:
            raise RuntimeError(
                "SAM 3 is not installed; sync the repository's sam3 dependency group"
            ) from exc

        kwargs: dict[str, Any] = {
            "device": config.device,
            "compile": config.compile,
        }
        # build_sam3_image_model otherwise defaults to the older SAM 3 weights.
        # The official loader extracts the detector weights from the 3.1
        # multiplex checkpoint for use by the image architecture.
        checkpoint = config.checkpoint or Path(download_ckpt_from_hf(version="sam3.1"))
        kwargs["checkpoint_path"] = str(checkpoint)
        model = build_sam3_image_model(**kwargs)
        if config.precision == "fp16":
            model = model.half()
            _patch_sam3_fp16_ffn(model)
        self._processor = Sam3Processor(
            model,
            device=config.device,
            confidence_threshold=config.confidence_threshold,
        )
        self._image_type = Image
        self._device = config.device
        self._precision = config.precision

    def set_image(self, image: NDArray[np.uint8]) -> object:
        # Upstream infers ndarray dimensions from shape[-2:], which is wrong
        # for HWC arrays. PIL preserves the original height and width.
        with self._autocast():
            return self._processor.set_image(
                self._image_type.fromarray(image, mode="RGB")
            )

    def predict_box(
        self, state: object, normalized_cxcywh: tuple[float, float, float, float]
    ) -> Mapping[str, Any]:
        if not isinstance(state, dict):
            raise TypeError("official SAM 3 processor state must be a dictionary")
        self._processor.reset_all_prompts(state)
        with self._autocast():
            return self._processor.add_geometric_prompt(
                box=list(normalized_cxcywh), label=True, state=state
            )

    def predict_boxes(
        self,
        state: object,
        boxes_xyxy: tuple[tuple[float, float, float, float], ...],
    ) -> tuple[Mapping[str, Any], ...]:
        if not isinstance(state, dict):
            raise TypeError("official SAM 3 processor state must be a dictionary")
        if not boxes_xyxy:
            return ()
        boxes = np.asarray(boxes_xyxy, dtype=np.float32)
        with self._autocast():
            masks, scores, _low_resolution_masks = self._processor.model.predict_inst(
                state,
                box=boxes,
                multimask_output=False,
            )
        masks = np.asarray(masks)
        scores = np.asarray(scores)
        if len(boxes_xyxy) == 1:
            masks = masks[None, ...]
            scores = scores[None, ...]
        if masks.shape[0] != len(boxes_xyxy) or scores.shape[0] != len(boxes_xyxy):
            raise RuntimeError("SAM 3 interactive decoder returned the wrong batch size")
        return tuple(
            {"masks": masks[index], "scores": scores[index]}
            for index in range(len(boxes_xyxy))
        )

    def _autocast(self) -> Any:
        if not self._device.startswith("cuda"):
            return nullcontext()
        import torch

        # SAM 3's fused MLP emits BF16 activations. Newer recommended PyTorch
        # releases autocast the following Linear automatically; make that
        # contract explicit for the unified Torch 2.5 SAM3D environment.
        dtype = torch.float16 if self._precision == "fp16" else torch.bfloat16
        return torch.autocast(device_type="cuda", dtype=dtype)


class SAM3MaskBackend:
    """Resident SAM 3 image backend with explicit load and warmup phases."""

    name = "sam3-online"

    def __init__(
        self,
        config: SAM3MaskBackendConfig,
        *,
        predictor_factory: Callable[[SAM3MaskBackendConfig], SAM3ImagePredictor]
        | None = None,
    ) -> None:
        self.config = config
        self._factory = predictor_factory or _OfficialSAM3ImagePredictor
        self._predictor: SAM3ImagePredictor | None = None
        self._state = WorkerState.CREATED
        self._load_seconds = 0.0
        self._warmup_seconds = 0.0

    @property
    def state(self) -> WorkerState:
        return self._state

    def load(self) -> None:
        if self._predictor is not None:
            return
        self._state = WorkerState.LOADING
        started = perf_counter()
        try:
            self._predictor = self._factory(self.config)
            _synchronize_cuda(self.config.device)
            self._load_seconds = perf_counter() - started
            self._state = WorkerState.LOADED
        except Exception:
            self._state = WorkerState.FAILED
            raise

    def warmup(self, frame: FramePrompt | None = None, *, iterations: int = 3) -> None:
        if iterations < 1:
            raise ValueError("warmup iterations must be positive")
        self.load()
        if frame is None:
            prompt_count = 4 if self.config.prompt_mode == "interactive_batch" else 1
            box_width = 756.0 / prompt_count
            frame = FramePrompt(
                frame_id="sam3-warmup",
                timestamp_us=0,
                image=np.zeros((1008, 1008, 3), dtype=np.uint8),
                boxes=tuple(
                    BoxPrompt(
                        f"warmup-{index}",
                        (
                            126.0 + index * box_width,
                            252.0,
                            126.0 + (index + 1) * box_width,
                            756.0,
                        ),
                    )
                    for index in range(prompt_count)
                ),
            )
        if not frame.boxes:
            raise ValueError("warmup frame must contain at least one box")
        self._state = WorkerState.WARMING
        started = perf_counter()
        try:
            for _ in range(iterations):
                self._predict_frame(frame, collect_gpu_metrics=False)
            _synchronize_cuda(self.config.device)
            self._warmup_seconds = perf_counter() - started
            self._state = WorkerState.READY
        except Exception:
            self._state = WorkerState.FAILED
            raise

    def predict(self, frame: FramePrompt) -> MaskBatch:
        if self._predictor is None:
            raise RuntimeError("SAM 3 backend must be loaded and warmed before inference")
        if self._state is not WorkerState.READY:
            raise RuntimeError(f"SAM 3 backend is not ready: {self._state.value}")
        self._state = WorkerState.BUSY
        try:
            result = self._predict_frame(frame, collect_gpu_metrics=True)
            self._state = WorkerState.READY
            return result
        except Exception:
            self._state = WorkerState.FAILED
            raise

    def _predict_frame(
        self, frame: FramePrompt, *, collect_gpu_metrics: bool
    ) -> MaskBatch:
        assert self._predictor is not None
        if collect_gpu_metrics:
            _reset_peak_cuda(self.config.device)
        total_started = perf_counter()
        encode_started = perf_counter()
        predictor_state = self._predictor.set_image(frame.image)
        _synchronize_cuda(self.config.device)
        encode_seconds = perf_counter() - encode_started

        decoded_seconds = 0.0
        masks: dict[str, MaskResult] = {}
        rejected: dict[str, str] = {}
        raw_results: tuple[Mapping[str, Any], ...] | None = None
        if self.config.prompt_mode == "interactive_batch" and frame.boxes:
            started = perf_counter()
            raw_results = self._predictor.predict_boxes(
                predictor_state,
                tuple(box.xyxy for box in frame.boxes),
            )
            _synchronize_cuda(self.config.device)
            decoded_seconds += perf_counter() - started
        for index, box in enumerate(frame.boxes):
            if raw_results is None:
                started = perf_counter()
                raw = self._predictor.predict_box(
                    predictor_state, _normalize_box(box.xyxy, frame.image.shape[:2])
                )
                _synchronize_cuda(self.config.device)
                decoded_seconds += perf_counter() - started
            else:
                raw = raw_results[index]
            try:
                masks[box.track_id] = self._select_mask(frame.image.shape[:2], box, raw)
            except (RuntimeError, ValueError) as exc:
                rejected[box.track_id] = str(exc)

        _synchronize_cuda(self.config.device)
        total_seconds = perf_counter() - total_started
        allocated, reserved = _peak_cuda_mib(self.config.device)
        metrics = MaskWorkerMetrics(
            backend=self.name,
            model_load_seconds=self._load_seconds,
            warmup_seconds=self._warmup_seconds,
            image_encode_seconds=encode_seconds,
            prompt_decode_seconds=decoded_seconds,
            total_seconds=total_seconds,
            prompts=len(frame.boxes),
            accepted_masks=len(masks),
            gpu_peak_allocated_mib=allocated,
            gpu_peak_reserved_mib=reserved,
        )
        return MaskBatch(
            frame_id=frame.frame_id,
            timestamp_us=frame.timestamp_us,
            masks=masks,
            rejected_tracks=rejected,
            metrics=metrics,
        )

    def _select_mask(
        self,
        image_size: tuple[int, int],
        box: BoxPrompt,
        raw: Mapping[str, Any],
    ) -> MaskResult:
        masks = _as_numpy(raw.get("masks"))
        scores = _as_numpy(raw.get("scores"))
        if masks.ndim == 4 and masks.shape[1] == 1:
            masks = masks[:, 0]
        if masks.ndim == 2:
            masks = masks[None, ...]
        scores = scores.reshape(-1)
        if masks.ndim != 3 or masks.shape[0] == 0 or scores.shape != (masks.shape[0],):
            raise RuntimeError("SAM 3 returned malformed masks or scores")
        if masks.shape[1:] != image_size or not np.all(np.isfinite(scores)):
            raise RuntimeError("SAM 3 returned incorrectly sized masks or non-finite scores")

        height, width = image_size
        x0, y0, x1, y1 = _clipped_integer_box(box.xyxy, width, height)
        box_area = max(1, (x1 - x0) * (y1 - y0))
        candidates: list[tuple[float, int, float, float]] = []
        for index, candidate in enumerate(masks):
            binary = np.asarray(candidate > 0, dtype=np.bool_)
            foreground = int(binary.sum())
            intersection = int(binary[y0:y1, x0:x1].sum())
            in_box_fraction = intersection / max(1, foreground)
            box_coverage = intersection / box_area
            score = float(scores[index])
            if (
                score >= self.config.confidence_threshold
                and in_box_fraction >= self.config.minimum_mask_in_box_fraction
                and box_coverage >= self.config.minimum_box_coverage
            ):
                candidates.append(
                    (score * in_box_fraction, index, in_box_fraction, box_coverage)
                )
        if not candidates:
            raise RuntimeError("SAM 3 masks failed confidence or projected-box validation")
        _, index, in_box_fraction, box_coverage = max(candidates)
        return MaskResult(
            track_id=box.track_id,
            mask=np.asarray(masks[index] > 0, dtype=np.bool_),
            confidence=float(scores[index]),
            in_box_fraction=in_box_fraction,
            box_coverage=box_coverage,
        )


class OfflineMaskBackend:
    """Explicit fallback that obtains a prepared mask from a caller-owned source."""

    name = "offline"

    def __init__(
        self,
        loader: Callable[[FramePrompt, BoxPrompt], NDArray[np.bool_] | None],
    ) -> None:
        self._loader = loader
        self._state = WorkerState.CREATED

    @property
    def state(self) -> WorkerState:
        return self._state

    def load(self) -> None:
        self._state = WorkerState.READY

    def warmup(self, frame: FramePrompt | None = None, *, iterations: int = 3) -> None:
        if iterations < 1:
            raise ValueError("warmup iterations must be positive")
        self.load()

    def predict(self, frame: FramePrompt) -> MaskBatch:
        if self._state is not WorkerState.READY:
            raise RuntimeError("offline backend must be explicitly loaded before inference")
        started = perf_counter()
        masks: dict[str, MaskResult] = {}
        rejected: dict[str, str] = {}
        height, width = frame.image.shape[:2]
        for box in frame.boxes:
            mask = self._loader(frame, box)
            if mask is None:
                rejected[box.track_id] = "offline mask not found"
                continue
            binary = np.asarray(mask, dtype=np.bool_)
            if binary.shape != (height, width) or not np.any(binary):
                rejected[box.track_id] = "offline mask is empty or incorrectly sized"
                continue
            x0, y0, x1, y1 = _clipped_integer_box(box.xyxy, width, height)
            intersection = int(binary[y0:y1, x0:x1].sum())
            masks[box.track_id] = MaskResult(
                track_id=box.track_id,
                mask=binary,
                confidence=1.0,
                in_box_fraction=intersection / max(1, int(binary.sum())),
                box_coverage=intersection / max(1, (x1 - x0) * (y1 - y0)),
            )
        elapsed = perf_counter() - started
        return MaskBatch(
            frame_id=frame.frame_id,
            timestamp_us=frame.timestamp_us,
            masks=masks,
            rejected_tracks=rejected,
            metrics=MaskWorkerMetrics(
                backend=self.name,
                model_load_seconds=0.0,
                warmup_seconds=0.0,
                image_encode_seconds=0.0,
                prompt_decode_seconds=elapsed,
                total_seconds=elapsed,
                prompts=len(frame.boxes),
                accepted_masks=len(masks),
            ),
        )


def _normalize_box(
    xyxy: tuple[float, float, float, float], image_size: tuple[int, int]
) -> tuple[float, float, float, float]:
    height, width = image_size
    x0, y0, x1, y1 = xyxy
    return (
        ((x0 + x1) / 2.0) / width,
        ((y0 + y1) / 2.0) / height,
        (x1 - x0) / width,
        (y1 - y0) / height,
    )


def _clipped_integer_box(
    xyxy: tuple[float, float, float, float], width: int, height: int
) -> tuple[int, int, int, int]:
    x0, y0, x1, y1 = np.rint(xyxy).astype(int)
    return max(0, x0), max(0, y0), min(width, x1), min(height, y1)


def _as_numpy(value: Any) -> NDArray[Any]:
    if value is None:
        return np.asarray([])
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    try:
        return np.asarray(value)
    except TypeError:
        # NumPy has no BF16 dtype. Scores may remain BF16 after CUDA autocast.
        if hasattr(value, "float"):
            return np.asarray(value.float())
        raise


def _patch_sam3_fp16_ffn(model: Any) -> None:
    """Preserve upstream's FP32 residual while allowing FP16 FFN weights."""

    import torch

    def forward_ffn(module: Any, target: Any) -> Any:
        with torch.amp.autocast(device_type="cuda", enabled=False):
            working = target.to(dtype=module.linear1.weight.dtype)
            update = module.linear2(
                module.dropout3(module.activation(module.linear1(working)))
            )
        target = target + module.dropout4(update.to(dtype=target.dtype))
        return module.norm3(target)

    for module in model.modules():
        if (
            type(module).__module__ == "sam3.model.decoder"
            and hasattr(module, "forward_ffn")
            and hasattr(module, "linear1")
        ):
            module.forward_ffn = MethodType(forward_ffn, module)


def _torch_cuda(device: str) -> Any | None:
    if not device.startswith("cuda"):
        return None
    try:
        import torch
    except ImportError:
        return None
    return torch.cuda if torch.cuda.is_available() else None


def _synchronize_cuda(device: str) -> None:
    cuda = _torch_cuda(device)
    if cuda is not None:
        cuda.synchronize(device)


def _reset_peak_cuda(device: str) -> None:
    cuda = _torch_cuda(device)
    if cuda is not None:
        cuda.reset_peak_memory_stats(device)


def _peak_cuda_mib(device: str) -> tuple[float | None, float | None]:
    cuda = _torch_cuda(device)
    if cuda is None:
        return None, None
    scale = 1024.0**2
    return (
        cuda.max_memory_allocated(device) / scale,
        cuda.max_memory_reserved(device) / scale,
    )

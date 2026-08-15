from __future__ import annotations

import pickle
from types import SimpleNamespace

import numpy as np
import pytest

from src.segmentation import (
    BoxPrompt,
    FramePrompt,
    MaskWorker,
    OfflineMaskBackend,
    SAM3MaskBackend,
    SAM3MaskBackendConfig,
    WorkerState,
)
from src.segmentation.backends import _OfficialSAM3ImagePredictor, _move_processor_state


def _frame() -> FramePrompt:
    return FramePrompt(
        frame_id="scene:0",
        timestamp_us=123,
        image=np.zeros((10, 20, 3), dtype=np.uint8),
        boxes=(
            BoxPrompt("car-a", (2.0, 2.0, 10.0, 8.0)),
            BoxPrompt("car-b", (12.0, 2.0, 19.0, 8.0)),
        ),
    )


def test_sam3_worker_loads_warms_and_encodes_once_per_frame():
    calls: list[object] = []

    class Predictor:
        def set_image(self, image):
            calls.append(("set_image", image.shape))
            return {"shared_embedding": True}

        def predict_box(self, state, box):
            calls.append(("predict_box", state, box))
            mask = np.zeros((1, 1, 10, 20), dtype=bool)
            if box[0] < 0.5:
                mask[0, 0, 2:8, 2:10] = True
            else:
                mask[0, 0, 2:8, 12:19] = True
            return {"masks": mask, "scores": np.asarray([0.9])}

    backend = SAM3MaskBackend(
        SAM3MaskBackendConfig(device="cpu"),
        predictor_factory=lambda _: Predictor(),
    )
    worker = MaskWorker(backend)
    assert worker.state is WorkerState.CREATED
    worker.start(_frame(), warmup_iterations=1)
    assert worker.ready
    calls.clear()

    result = worker.process(_frame())

    assert worker.ready
    assert set(result.masks) == {"car-a", "car-b"}
    assert [call[0] for call in calls].count("set_image") == 1
    assert [call[0] for call in calls].count("predict_box") == 2
    assert result.metrics.prompts == result.metrics.accepted_masks == 2
    assert result.metrics.backend == "sam3-online"
    # xyxy (2, 2, 10, 8) -> normalized cxcywh for a 20x10 image.
    np.testing.assert_allclose(calls[1][2], (0.3, 0.5, 0.4, 0.6))


def test_sam3_interactive_mode_batches_all_boxes_in_one_decode():
    calls: list[object] = []

    class Predictor:
        def set_image(self, image):
            calls.append(("set_image", image.shape))
            return {}

        def predict_boxes(self, state, boxes):
            calls.append(("predict_boxes", boxes))
            results = []
            for x0, y0, x1, y1 in boxes:
                mask = np.zeros((1, 10, 20), dtype=bool)
                mask[0, int(y0) : int(y1), int(x0) : int(x1)] = True
                results.append({"masks": mask, "scores": np.asarray([0.9])})
            return tuple(results)

    backend = SAM3MaskBackend(
        SAM3MaskBackendConfig(device="cpu", prompt_mode="interactive_batch"),
        predictor_factory=lambda _: Predictor(),
    )
    worker = MaskWorker(backend)
    worker.start(_frame(), warmup_iterations=1)
    calls.clear()

    result = worker.process(_frame())

    assert set(result.masks) == {"car-a", "car-b"}
    assert [call[0] for call in calls] == ["set_image", "predict_boxes"]
    assert calls[1][1] == tuple(box.xyxy for box in _frame().boxes)


def test_sam3_interactive_dummy_warmup_uses_multi_prompt_batch():
    batch_sizes = []

    class Predictor:
        def set_image(self, image):
            self.shape = image.shape[:2]
            return {}

        def predict_boxes(self, state, boxes):
            batch_sizes.append(len(boxes))
            return tuple(
                {
                    "masks": np.ones((1, *self.shape), dtype=bool),
                    "scores": np.asarray([0.9]),
                }
                for _box in boxes
            )

    backend = SAM3MaskBackend(
        SAM3MaskBackendConfig(device="cpu", prompt_mode="interactive_batch"),
        predictor_factory=lambda _: Predictor(),
    )

    backend.warmup(iterations=1)

    assert batch_sizes == [4]
    assert backend.state is WorkerState.READY


def test_sam3_rejects_mask_that_belongs_to_neighbor():
    class Predictor:
        def set_image(self, image):
            return {}

        def predict_box(self, state, box):
            mask = np.zeros((1, 10, 20), dtype=bool)
            mask[0, 2:8, :2] = True
            return {"masks": mask, "scores": np.asarray([0.99])}

    backend = SAM3MaskBackend(
        SAM3MaskBackendConfig(device="cpu"),
        predictor_factory=lambda _: Predictor(),
    )
    backend.load()
    assert backend.state is WorkerState.LOADED
    with pytest.raises(RuntimeError, match="not ready"):
        backend.predict(_frame())
    backend.warmup(_frame(), iterations=1)

    result = backend.predict(_frame())

    assert not result.masks
    assert set(result.rejected_tracks) == {"car-a", "car-b"}


def test_sam3_failure_moves_worker_to_failed_state():
    backend = SAM3MaskBackend(
        SAM3MaskBackendConfig(device="cpu"),
        predictor_factory=lambda _: (_ for _ in ()).throw(RuntimeError("load failed")),
    )
    with pytest.raises(RuntimeError, match="load failed"):
        backend.load()
    assert backend.state is WorkerState.FAILED


def test_sam3_worker_can_unload_and_reload_without_replacing_worker():
    created: list[object] = []

    class Predictor:
        def set_image(self, image):
            return {}

        def predict_box(self, state, box):
            return {
                "masks": np.ones((1, 10, 20), dtype=bool),
                "scores": np.asarray([0.9]),
            }

    backend = SAM3MaskBackend(
        SAM3MaskBackendConfig(device="cpu"),
        predictor_factory=lambda _: created.append(Predictor()) or created[-1],
    )
    worker = MaskWorker(backend)
    worker.start(_frame(), warmup_iterations=1)

    worker.unload()
    assert worker.state is WorkerState.CREATED
    worker.start(_frame(), warmup_iterations=1)

    assert worker.ready
    assert len(created) == 2


def test_sam3_worker_suspends_and_resumes_persistent_predictor():
    created = []

    class Predictor:
        def __init__(self):
            self.suspends = 0
            self.resumes = 0

        def set_image(self, image):
            self.shape = image.shape[:2]
            return {}

        def predict_box(self, state, box):
            return {
                "masks": np.ones((1, *self.shape), dtype=bool),
                "scores": np.asarray([0.9]),
            }

        def suspend(self):
            self.suspends += 1
            return {"residency_action": "cpu_offload", "offload_transfer_seconds": 0.3}

        def resume(self):
            self.resumes += 1
            return {"residency_action": "cuda_resume", "resume_transfer_seconds": 0.2}

    backend = SAM3MaskBackend(
        SAM3MaskBackendConfig(device="cpu"),
        predictor_factory=lambda _: created.append(Predictor()) or created[-1],
    )
    worker = MaskWorker(backend)
    first = worker.start(_frame(), warmup_iterations=1)
    offload = worker.unload()
    resumed = worker.start(_frame(), warmup_iterations=1)

    assert first["residency_action"] == "initial_load"
    assert offload["offload_transfer_seconds"] == 0.3
    assert resumed["resume_transfer_seconds"] == 0.2
    assert len(created) == 1
    assert created[0].suspends == created[0].resumes == 1


def test_official_sam3_predictor_moves_processor_and_clears_image_state(monkeypatch):
    transfers = []

    class Model:
        def to(self, device):
            transfers.append(str(device))
            return self

    predictor = _OfficialSAM3ImagePredictor.__new__(_OfficialSAM3ImagePredictor)
    predictor._processor = SimpleNamespace(model=Model(), device="cuda:0", find_stage=None)
    predictor._device = "cuda:0"
    predictor._configured_device = "cuda:0"
    predictor._active_state = {"backbone_out": {"gpu": object()}}
    predictor._suspended = False
    monkeypatch.setattr(
        "src.realtime.gpu_residency.release_cuda_memory", lambda: None
    )
    monkeypatch.setattr("src.segmentation.backends._synchronize_cuda", lambda _: None)

    offload = predictor.suspend()
    resume = predictor.resume()

    assert predictor._active_state is None
    assert transfers == ["cpu", "cuda:0"]
    assert predictor._processor.device == "cuda:0"
    assert predictor._device == "cuda:0"
    assert offload["residency_action"] == "cpu_offload"
    assert resume["residency_action"] == "cuda_resume"


def test_processor_state_moves_persistent_find_stage_tensors():
    processor = SimpleNamespace(
        device="cuda",
        find_stage=SimpleNamespace(
            img_ids=np.asarray([1]),
            text_ids=__import__("torch").tensor([2]),
            nested=[__import__("torch").tensor([3])],
        ),
    )

    _move_processor_state(processor, "cpu")

    assert processor.device == "cpu"
    assert processor.find_stage.text_ids.device.type == "cpu"
    assert processor.find_stage.nested[0].device.type == "cpu"


def test_offline_backend_is_explicit_and_reports_missing_masks():
    prepared = np.zeros((10, 20), dtype=bool)
    prepared[2:8, 2:10] = True
    backend = OfflineMaskBackend(
        lambda _frame, box: prepared if box.track_id == "car-a" else None
    )
    worker = MaskWorker(backend)
    with pytest.raises(RuntimeError, match="finish load"):
        worker.process(_frame())

    worker.start()
    result = worker.process(_frame())

    assert set(result.masks) == {"car-a"}
    assert result.rejected_tracks == {"car-b": "offline mask not found"}
    assert result.metrics.backend == "offline"


def test_frame_rejects_duplicate_track_prompts():
    with pytest.raises(ValueError, match="unique"):
        FramePrompt(
            frame_id="frame",
            timestamp_us=0,
            image=np.zeros((4, 4, 3), dtype=np.uint8),
            boxes=(
                BoxPrompt("same", (0, 0, 2, 2)),
                BoxPrompt("same", (1, 1, 3, 3)),
            ),
        )


def test_mask_batch_is_process_serializable():
    backend = OfflineMaskBackend(
        lambda frame, _box: np.ones(frame.image.shape[:2], dtype=bool)
    )
    backend.load()
    restored = pickle.loads(pickle.dumps(backend.predict(_frame())))
    assert set(restored.masks) == {"car-a", "car-b"}

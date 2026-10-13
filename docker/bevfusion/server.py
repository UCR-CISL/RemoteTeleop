#!/usr/bin/env python3
"""Persistent front-camera BEVFusion server run inside ``mmdet3d-bevfusion``.

This file is deliberately mounted from ``docker/bevfusion`` read-only.  It
does not require the RemoteTeleop repository to be visible inside Docker.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
import traceback

import cv2
import numpy as np
import zmq

sys.path.insert(0, "/workspace")
from bevfusion_wire import BevFusionBox, BevFusionCodec, BevFusionRequest, BevFusionResult  # type: ignore[import-not-found]
from raw_predictions import RawPredictionWriter


def _register_runtime_transform() -> None:
    """Register the no-filesystem front-camera transform used by nuscenes_front."""
    from mmcv.transforms import BaseTransform
    from mmdet3d.registry import TRANSFORMS
    from mmdet3d.structures.points import LiDARPoints

    @TRANSFORMS.register_module(force=True)
    class RuntimeFrontCameraInputs(BaseTransform):
        def __init__(self, point_feature_count=5, intensity_scale=1.0):
            if point_feature_count not in (4, 5):
                raise ValueError("model point feature count must be 4 or 5")
            self.point_feature_count = point_feature_count
            if not np.isfinite(intensity_scale) or intensity_scale <= 0:
                raise ValueError("model intensity scale must be finite and positive")
            self.intensity_scale = intensity_scale

        def transform(self, results):
            bgr = results.pop("bevfusion_image_bgr")
            points = results.pop("bevfusion_points")
            if bgr.ndim != 3 or bgr.shape[2] != 3:
                raise ValueError("BEVFusion image must be HxWx3 BGR")
            if points.ndim != 2 or points.shape[1] not in (4, 5):
                raise ValueError("BEVFusion points must be Nx4 or explicit Nx5 XYZI/time-lag")
            if not np.isfinite(points).all() or (points.shape[1] == 5 and np.any(points[:, 4] < 0)):
                raise ValueError("BEVFusion points must be finite with nonnegative time lag")
            if points.shape[1] > self.point_feature_count:
                raise ValueError("explicit sweep features cannot be sent to a four-feature model")
            if self.intensity_scale != 1.0:
                points = points.copy()
                points[:, 3] *= self.intensity_scale
            # MMDet's image conventions are RGB; transport remains BGR so it
            # matches OpenCV/ROS without an encode-decode round trip.
            results["img"] = [cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)]
            # The upstream nuScenes checkpoint consumes XYZI/time-lag; its
            # single-scan path adds zero lag. The native CooperScene model
            # consumes four features, selected explicitly by its config.
            if points.shape[1] == 4 and self.point_feature_count == 5:
                points = np.concatenate((points, np.zeros((len(points), 1), dtype=np.float32)), axis=1)
            if not points.flags.writeable:
                points = points.copy()
            results["points"] = LiDARPoints(points, points_dim=self.point_feature_count, attribute_dims=None)
            results["img_shape"] = bgr.shape[:2]
            results["ori_shape"] = bgr.shape[:2]
            return results


_register_runtime_transform()


class BevFusionModel:
    """Owns the MMDetection3D model lifecycle and single-frame inference."""

    def __init__(self, config: Path, checkpoint: Path, device: str, score_threshold: float, raw_predictions_dir: Path | None = None) -> None:
        from mmdet3d.apis import init_model
        from mmengine.dataset import Compose
        # init_model tolerates missing or incompatible parameters. A sidecar
        # must fail before serving detections from partially loaded weights,
        # so weights are loaded once, strictly, below.
        self.model = init_model(str(config), None, device=device)
        self._load_strict(checkpoint)
        self.pipeline = Compose(self.model.cfg.test_dataloader.dataset.pipeline)
        self.device = device
        self.score_threshold = score_threshold
        self._warmed = False
        self.raw_predictions = RawPredictionWriter(raw_predictions_dir) if raw_predictions_dir is not None else None

    def _load_strict(self, checkpoint: Path) -> None:
        """Strict load; spconv2 (O,D,H,W,I) sparse weights -> mmcv (D,H,W,I,O) in memory.

        Native CooperScene checkpoints were saved with spconv; this image uses
        mmcv sparse ops. The permutation is exact and only applies to 5-D
        tensors whose shape differs from the model's.
        """
        import torch
        loaded = torch.load(str(checkpoint), map_location="cpu")
        state = loaded.get("state_dict", loaded)
        target = self.model.state_dict()
        for name, value in list(state.items()):
            if value.ndim == 5 and name in target and value.shape != target[name].shape:
                state[name] = value.permute(1, 2, 3, 4, 0).contiguous()
        self.model.load_state_dict(state, strict=True)
        meta = loaded.get("meta", {})
        # Same precedence as mmdet3d.apis.init_model.
        if "dataset_meta" in meta:
            self.model.dataset_meta = meta["dataset_meta"]
        else:
            self.model.dataset_meta = {"classes": meta.get("CLASSES", self.model.cfg.get("class_names"))}

    def warmup(self) -> None:
        import torch
        if self._warmed:
            return
        image = np.zeros((900, 1600, 3), dtype=np.uint8)
        points = np.zeros((256, 4), dtype="<f4")
        points[:, 0] = np.linspace(1.0, 50.0, len(points))
        points[:, 1] = np.linspace(-20.0, 20.0, len(points))
        points[:, 3] = 0.5
        identity = tuple(float(value) for value in np.eye(4).reshape(-1))
        self.infer(BevFusionRequest(
            request_id="warmup", scene_generation="warmup", sequence=0,
            frame_id="warmup", timestamp_us=0, config_id="nuscenes",
            camera_intrinsic=(1266.4, 0.0, 816.3, 0.0, 1266.4, 491.5,
                              0.0, 0.0, 1.0),
            world_T_camera=identity, world_T_lidar=identity,
            image_width=1600, image_height=900, image_bgr=image.tobytes(),
            lidar_point_count=len(points), lidar_f32=points.tobytes(),
        ))
        if self.device.startswith("cuda"):
            torch.cuda.synchronize()
        self._warmed = True

    def infer(
        self, request: BevFusionRequest
    ) -> tuple[tuple[BevFusionBox, ...], dict[str, float]]:
        """Materialize one request privately and use MMDet3D's public demo API.

        The configured front-camera adapter must supply the matching annotation
        template/calibration expected by the selected MMDet3D config.
        """
        # ``nuscenes_front.py`` supplies a pipeline whose first transform
        # accepts these in-memory fields.  No host file or repository mount is
        # involved in the request path.
        from mmengine.dataset import pseudo_collate
        from mmdet3d.structures import get_box_type
        import torch
        preprocess_started = time.perf_counter()
        box_type_3d, box_mode_3d = get_box_type("LiDAR")
        intrinsic = np.asarray(request.camera_intrinsic, dtype=np.float32).reshape(3, 3)
        cam2img = np.eye(4, dtype=np.float32)
        cam2img[:3, :3] = intrinsic
        lidar_to_camera = np.linalg.inv(
            np.asarray(request.world_T_camera, dtype=np.float32).reshape(4, 4)
        ) @ np.asarray(request.world_T_lidar, dtype=np.float32).reshape(4, 4)
        camera_to_lidar = np.linalg.inv(lidar_to_camera)
        data = self.pipeline({
            "bevfusion_image_bgr": np.frombuffer(request.image_bgr, dtype=np.uint8).reshape(
                request.image_height, request.image_width, 3),
            "bevfusion_points": np.frombuffer(request.lidar_f32, dtype="<f4").reshape(
                request.lidar_point_count, request.lidar_feature_count),
            "cam2img": cam2img[None, ...], "lidar2cam": lidar_to_camera[None, ...],
            "cam2lidar": camera_to_lidar[None, ...],
            "lidar2img": (cam2img @ lidar_to_camera)[None, ...],
            "dataset": "nuscenes", "sample_idx": request.frame_id,
            "box_type_3d": box_type_3d, "box_mode_3d": box_mode_3d,
        })
        inference_started = time.perf_counter()
        cuda = self.device.startswith("cuda")
        with torch.inference_mode():
            # Equivalent to BaseModel.test_step, split for stage timing.
            batch = self.model.data_preprocessor(pseudo_collate([data]), False)
            if cuda:
                torch.cuda.synchronize()
            forward_started = time.perf_counter()
            prediction = self.model._run_forward(batch, mode="predict")[0]
            if cuda:
                torch.cuda.synchronize()
        postprocess_started = time.perf_counter()
        instances = prediction.pred_instances_3d
        boxes = instances.bboxes_3d.tensor.detach().cpu().numpy()
        centers = instances.bboxes_3d.gravity_center.detach().cpu().numpy()
        scores = instances.scores_3d.detach().cpu().numpy()
        labels = instances.labels_3d.detach().cpu().numpy()
        class_names = self.model.dataset_meta.get("classes") or ("car",)
        if self.raw_predictions is not None:
            self.raw_predictions.write(request, boxes=boxes, gravity_centers=centers, scores=scores,
                labels=labels, class_names=self.model.dataset_meta.get("classes", ()))
        detections = []
        for box, center, score, label in zip(boxes, centers, scores, labels):
            if int(label) != 0 or float(score) < self.score_threshold:
                continue
            x_size, y_size, height = map(float, box[3:6])
            yaw = float(box[6])
            # MMDet boxes expose axis sizes, while the teleop protocol requires
            # semantic length/width with the object x-axis along its length.
            if y_size > x_size:
                length, width = y_size, x_size
                yaw += math.pi / 2
            else:
                length, width = x_size, y_size
            yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
            detections.append(BevFusionBox(
                tuple(map(float, center)), (length, width, height), yaw,
                float(score), int(label), str(class_names[int(label)]),
            ))
        completed = time.perf_counter()
        return tuple(detections), {
            "preprocess": (inference_started - preprocess_started) * 1000,
            "inference": (postprocess_started - inference_started) * 1000,
            "data_preprocessor": (forward_started - inference_started) * 1000,
            "forward": (postprocess_started - forward_started) * 1000,
            "postprocess": (completed - postprocess_started) * 1000,
        }


class BevFusionServer:
    def __init__(self, model: BevFusionModel, socket: zmq.Socket, metrics_path: str, codec: BevFusionCodec | None = None) -> None:
        self.model, self.socket, self.metrics_path, self.codec = model, socket, metrics_path, codec or BevFusionCodec()

    def _metric(self, record: dict) -> None:
        line = json.dumps(record, separators=(",", ":"), allow_nan=False)
        if self.metrics_path in ("-", "/dev/stdout"):
            print(line, flush=True)
        else:
            with open(self.metrics_path, "a", encoding="utf-8") as stream:
                stream.write(line + "\n")

    def serve_forever(self) -> None:
        self.model.warmup()
        while True:
            identity, *parts = self.socket.recv_multipart()
            started = time.perf_counter()
            request = None
            try:
                request = self.codec.decode_request(parts)
                boxes, timings = self.model.infer(request)
                timings["server_total"] = (time.perf_counter() - started) * 1000
                result = BevFusionResult(request.request_id, request.scene_generation, request.sequence,
                    request.frame_id, request.timestamp_us, request.config_id, boxes=boxes,
                    timings_ms=timings)
            except Exception as error:
                traceback.print_exc()
                if request is None:
                    self._metric({
                        "event": "bevfusion_protocol_error",
                        "error": f"{type(error).__name__}: {error}",
                    })
                    continue
                result = BevFusionResult(request.request_id, request.scene_generation, request.sequence,
                    request.frame_id, request.timestamp_us, request.config_id,
                    timings_ms={"server_total": (time.perf_counter() - started) * 1000},
                    error=f"{type(error).__name__}: {error}")
            self.socket.send_multipart([identity, *self.codec.encode_result(result)])
            self._metric({"event": "bevfusion_result", "request_id": result.request_id,
                "sequence": result.sequence, "frame_id": result.frame_id,
                "box_count": len(result.boxes), "error": result.error,
                **{f"{key}_ms": value for key, value in (result.timings_ms or {}).items()}})


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="tcp://0.0.0.0:8766")
    parser.add_argument("--config", type=Path, default=Path("/workspace/nuscenes_front.py"))
    parser.add_argument("--checkpoint", type=Path, default=Path("/workspace/checkpoints/bevfusion_converted.pth"))
    parser.add_argument("--metrics-path", default="-")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--score-threshold", type=float, default=0.2)
    parser.add_argument("--raw-predictions-dir", type=Path, help="optional writable diagnostic directory; includes all model-output classes before sidecar filtering")
    args = parser.parse_args()
    if not args.config.is_file() or not args.checkpoint.is_file():
        parser.error("--config and --checkpoint must name readable files in /workspace")
    model = BevFusionModel(args.config, args.checkpoint, args.device, args.score_threshold, args.raw_predictions_dir)
    model.warmup()
    context = zmq.Context(); socket = context.socket(zmq.ROUTER)
    socket.setsockopt(zmq.SNDHWM, 1); socket.setsockopt(zmq.RCVHWM, 1); socket.setsockopt(zmq.ROUTER_MANDATORY, 1)
    socket.bind(args.endpoint)
    try:
        BevFusionServer(model, socket, args.metrics_path).serve_forever()
    finally:
        socket.close(linger=0); context.term()


if __name__ == "__main__":
    main()

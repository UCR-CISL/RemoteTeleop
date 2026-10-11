"""Optional offline diagnostics for model outputs before sidecar filtering."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


class RawPredictionWriter:
    """Write one immutable JSON record per inference request."""

    def __init__(self, directory: Path) -> None:
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)

    def write(self, request, *, boxes, gravity_centers, scores, labels, class_names) -> Path:
        count = len(scores)
        if any(len(values) != count for values in (boxes, gravity_centers, labels)):
            raise ValueError("raw prediction arrays must have identical counts")
        classes = list(class_names)
        provenance = {name: getattr(request, name) for name in (
            "request_id", "scene_generation", "sequence", "frame_id", "timestamp_us", "config_id",
            "camera_intrinsic", "world_T_camera", "world_T_lidar", "image_width", "image_height", "lidar_point_count")}
        payload = dict(provenance, schema_version=1,
            stage="model pred_instances_3d before sidecar car/confidence filtering; model head filters already applied",
            class_names=classes, coordinate_frame="request LiDAR",
            tensor_origin="LiDAR bottom center", tensor_layout="x,y,z,x_size,y_size,height,yaw,optional_vx,optional_vy",
            predictions=[dict(box_tensor=box.tolist(), gravity_center=center.tolist(), score=float(score),
                              class_id=int(label), class_name=classes[int(label)] if 0 <= int(label) < len(classes) else None)
                         for box, center, score, label in zip(boxes, gravity_centers, scores, labels)])
        digest = hashlib.sha256(request.request_id.encode()).hexdigest()[:16]
        path = self.directory / f"{request.sequence:06d}-{digest}.json"
        with path.open("x", encoding="utf-8") as stream:
            json.dump(payload, stream, separators=(",", ":"), allow_nan=False)
            stream.write("\n")
        return path

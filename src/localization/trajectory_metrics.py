"""Per-frame pose disagreement against an explicitly named map-frame reference."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np


class TrajectoryMetrics:
    """Persist all samples and summarize XYZ errors without fitting the trajectory."""

    def __init__(self, path: Path, *, reference_kind: str, expected_samples: int | None = None,
                 position_mode: str = "xyz"):
        if position_mode not in {"xyz", "xy"}:
            raise ValueError("position_mode must be xyz or xy")
        self.position_mode = position_mode
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.exists():
            raise ValueError(f"trajectory metrics already exist: {self.path}")
        self.reference_kind = reference_kind
        self.expected_samples = expected_samples
        self._rows: list[dict] = []

    def record(self, *, sequence: int, timestamp_us: int, frame_id: str,
               estimated_pose: np.ndarray, reference_pose: np.ndarray, map_R_dataset: np.ndarray | None = None) -> dict:
        estimate, reference = (np.asarray(p, dtype=float).reshape(4, 4)
                               for p in (estimated_pose, reference_pose))
        if not np.isfinite(estimate).all() or not np.isfinite(reference).all():
            raise ValueError("trajectory poses must be finite")
        if sequence != len(self._rows):
            raise ValueError("trajectory metric sequences must be contiguous from zero")
        delta = estimate[:3, 3] - reference[:3, 3]
        # NuScenes splats use camera-oriented map axes. Resolve signed XYZ and
        # horizontal/vertical error in the recorded dataset frame (Z up).
        if map_R_dataset is not None:
            delta = np.asarray(map_R_dataset, dtype=float).reshape(3, 3).T @ delta
        rotation = estimate[:3, :3] @ reference[:3, :3].T
        row = dict(sequence=sequence, timestamp_us=timestamp_us, frame_id=frame_id,
                   reference_kind=self.reference_kind, error_axes="dataset" if map_R_dataset is not None else "map", dx_m=float(delta[0]),
                   dy_m=float(delta[1]), dz_m=float(delta[2]),
                   translation_m=float(np.linalg.norm(delta)),
                   horizontal_translation_m=float(np.linalg.norm(delta[:2])),
                   vertical_translation_m=float(abs(delta[2])),
                   orientation_deg=float(np.degrees(np.arccos(np.clip((np.trace(rotation)-1)/2, -1, 1)))),
                   estimated_map_T_lidar=estimate.tolist(), reference_map_T_lidar=reference.tolist())
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(row, allow_nan=False) + "\n")
        self._rows.append(row)
        if self.expected_samples is not None and len(self._rows) == self.expected_samples:
            self.finish()
        return row

    def finish(self) -> dict:
        summary = dict(reference_kind=self.reference_kind, frame_count=len(self._rows),
                       trajectory_evaluated=bool(self._rows), position_mode=self.position_mode,
                       expected_samples=self.expected_samples,
                       complete=(self.expected_samples is not None and len(self._rows) == self.expected_samples),
                       trajectory_alignment_applied=False,
                       independent_ground_truth=False)
        for key in ("translation_m", "horizontal_translation_m", "vertical_translation_m", "orientation_deg"):
            values = np.asarray([row[key] for row in self._rows])
            summary[key] = (dict(mean=float(values.mean()), rmse=float(np.sqrt(np.mean(values**2))),
                                 p95=float(np.percentile(values, 95)), maximum=float(values.max()))
                            if len(values) else None)
        destination = self.path.with_suffix(".summary.json")
        temporary = destination.with_suffix(".tmp")
        temporary.write_text(json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8")
        temporary.replace(destination)
        return summary

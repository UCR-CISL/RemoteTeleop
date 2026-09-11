#!/usr/bin/env python3
"""Create a timestamp-checked NuScenes source/composited comparison video."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re

import cv2
import numpy as np


_FRAME_NAME = re.compile(r"^\d+_(\d+)\.png$")


@dataclass(frozen=True)
class ComparisonConfig:
    composited_video: Path
    compositor_metrics: Path
    scene_root: Path
    localization: Path
    output_video: Path
    panel_height: int = 540


@dataclass(frozen=True)
class TimelineFrame:
    sequence: int
    frame_id: str
    timestamp_us: int
    source_image: Path
    global_T_camera: np.ndarray


class NuScenesComparisonVideo:
    """Validate two 10 Hz streams, compose them, and report pose deviation."""

    def __init__(self, config: ComparisonConfig) -> None:
        if config.panel_height <= 0:
            raise ValueError("panel_height must be positive")
        self.config = config

    def run(self) -> dict:
        timeline = self._load_timeline()
        localized_T_global = self._load_localization()
        legacy_T_global = np.linalg.inv(timeline[0].global_T_camera)
        errors = [
            self._pose_error(frame.global_T_camera, legacy_T_global, localized_T_global)
            for frame in timeline
        ]

        capture = cv2.VideoCapture(str(self.config.composited_video))
        if not capture.isOpened():
            raise ValueError(f"failed to open video: {self.config.composited_video}")
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        frame_count = int(round(capture.get(cv2.CAP_PROP_FRAME_COUNT)))
        expected_fps = self._validate_timestamps(timeline)
        if frame_count != len(timeline):
            raise ValueError(
                f"video has {frame_count} frames but metrics contain {len(timeline)}"
            )
        if not math.isclose(fps, expected_fps, rel_tol=0.0, abs_tol=1e-3):
            raise ValueError(f"video is {fps:g} FPS but timestamps imply {expected_fps:g} FPS")

        panel_width = round(self.config.panel_height * 16 / 9)
        self.config.output_video.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(
            str(self.config.output_video),
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps,
            (panel_width * 2, self.config.panel_height),
        )
        if not writer.isOpened():
            capture.release()
            raise ValueError(f"failed to open video writer: {self.config.output_video}")

        try:
            for index, (frame, (translation_m, orientation_deg)) in enumerate(
                zip(timeline, errors, strict=True)
            ):
                ok, composited = capture.read()
                if not ok:
                    raise ValueError(f"failed to decode composited video frame {index}")
                source = cv2.imread(str(frame.source_image), cv2.IMREAD_COLOR)
                if source is None:
                    raise ValueError(f"failed to decode source image: {frame.source_image}")
                source = cv2.resize(source, (panel_width, self.config.panel_height))
                composited = cv2.resize(
                    composited, (panel_width, self.config.panel_height)
                )
                timestamp = f"t = {frame.timestamp_us / 1_000_000:.1f} s"
                self._label(source, "Original NuScenes CAM_FRONT", timestamp)
                self._label(composited, "Localized GSplat composite", timestamp)
                self._error_box(composited, translation_m, orientation_deg)
                writer.write(np.concatenate((source, composited), axis=1))
        finally:
            capture.release()
            writer.release()

        translations = np.asarray([value[0] for value in errors])
        orientations = np.asarray([value[1] for value in errors])
        summary = {
            "frame_count": len(timeline),
            "fps": fps,
            "start_timestamp_us": timeline[0].timestamp_us,
            "end_timestamp_us": timeline[-1].timestamp_us,
            "translation_error_m": self._statistics(translations),
            "orientation_error_deg": self._statistics(orientations),
            "frames": [
                {
                    "sequence": frame.sequence,
                    "frame_id": frame.frame_id,
                    "timestamp_us": frame.timestamp_us,
                    "translation_error_m": translation_m,
                    "orientation_error_deg": orientation_deg,
                }
                for frame, (translation_m, orientation_deg) in zip(
                    timeline, errors, strict=True
                )
            ],
        }
        summary_path = self.config.output_video.with_suffix(".json")
        summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        return summary

    def _load_timeline(self) -> tuple[TimelineFrame, ...]:
        records = [
            json.loads(line)
            for line in self.config.compositor_metrics.read_text(encoding="utf-8").splitlines()
            if line
        ]
        records = [record for record in records if record.get("event") == "compositor_frame"]
        records.sort(key=lambda record: record["sequence"])
        if not records:
            raise ValueError("compositor metrics contain no frames")
        expected_sequences = list(range(len(records)))
        if [record.get("sequence") for record in records] != expected_sequences:
            raise ValueError("compositor metric sequences must be contiguous and start at zero")

        frames = []
        for record in records:
            output_name = Path(str(record.get("output_path", ""))).name
            match = _FRAME_NAME.fullmatch(output_name)
            if match is None:
                raise ValueError(f"cannot recover frame ID from output path: {output_name}")
            frame_id = match.group(1)
            source_image = self.config.scene_root / "images" / f"{frame_id}_0.jpg"
            extrinsic = self.config.scene_root / "extrinsics" / f"{frame_id}_0.txt"
            if not source_image.is_file() or not extrinsic.is_file():
                raise FileNotFoundError(f"missing CAM_FRONT data for frame {frame_id}")
            frames.append(TimelineFrame(
                sequence=int(record["sequence"]),
                frame_id=frame_id,
                timestamp_us=int(record["source_timestamp_us"]),
                source_image=source_image,
                global_T_camera=self._transform(np.loadtxt(extrinsic), str(extrinsic)),
            ))
        return tuple(frames)

    @staticmethod
    def _validate_timestamps(timeline: tuple[TimelineFrame, ...]) -> float:
        timestamps = np.asarray([frame.timestamp_us for frame in timeline], dtype=np.int64)
        if len(timestamps) < 2:
            raise ValueError("at least two timestamped frames are required")
        deltas = np.diff(timestamps)
        if np.any(deltas <= 0) or not np.all(deltas == deltas[0]):
            raise ValueError("source timestamps must have one positive, constant cadence")
        return 1_000_000.0 / float(deltas[0])

    def _load_localization(self) -> np.ndarray:
        payload = json.loads(self.config.localization.read_text(encoding="utf-8"))
        return self._transform(
            payload.get("gaussian_T_nuscenes_global"),
            "gaussian_T_nuscenes_global",
        )

    @classmethod
    def _pose_error(
        cls,
        global_T_camera: np.ndarray,
        legacy_T_global: np.ndarray,
        localized_T_global: np.ndarray,
    ) -> tuple[float, float]:
        legacy = legacy_T_global @ global_T_camera
        localized = localized_T_global @ global_T_camera
        translation = float(np.linalg.norm(localized[:3, 3] - legacy[:3, 3]))
        delta_rotation = localized[:3, :3] @ legacy[:3, :3].T
        cosine = np.clip((np.trace(delta_rotation) - 1.0) / 2.0, -1.0, 1.0)
        return translation, float(np.degrees(np.arccos(cosine)))

    @staticmethod
    def _transform(value: object, label: str) -> np.ndarray:
        matrix = np.asarray(value, dtype=np.float64)
        if matrix.size != 16 or not np.isfinite(matrix).all():
            raise ValueError(f"{label} must contain a finite 4x4 transform")
        matrix = matrix.reshape(4, 4)
        if not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1e-7):
            raise ValueError(f"{label} must be homogeneous")
        return matrix

    @staticmethod
    def _statistics(values: np.ndarray) -> dict[str, float]:
        return {
            "initial": float(values[0]),
            "mean": float(np.mean(values)),
            "median": float(np.median(values)),
            "p95": float(np.percentile(values, 95)),
            "max": float(np.max(values)),
        }

    @staticmethod
    def _label(image: np.ndarray, title: str, timestamp: str) -> None:
        overlay = image.copy()
        cv2.rectangle(overlay, (12, 10), (410, 76), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.65, image, 0.35, 0.0, image)
        cv2.putText(image, title, (24, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(image, timestamp, (24, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (230, 230, 230), 1)

    @staticmethod
    def _error_box(image: np.ndarray, translation_m: float, orientation_deg: float) -> None:
        width = image.shape[1]
        left = width - 330
        overlay = image.copy()
        cv2.rectangle(overlay, (left, 10), (width - 12, 76), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.70, image, 0.30, 0.0, image)
        cv2.putText(
            image, f"Translation error: {translation_m:.3f} m", (left + 12, 37),
            cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2,
        )
        cv2.putText(
            image, f"Orientation error: {orientation_deg:.3f} deg", (left + 12, 65),
            cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2,
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--composited-video", type=Path, required=True)
    parser.add_argument("--compositor-metrics", type=Path, required=True)
    parser.add_argument("--scene-root", type=Path, required=True)
    parser.add_argument("--localization", type=Path, required=True)
    parser.add_argument("--output-video", type=Path, required=True)
    parser.add_argument("--panel-height", type=int, default=540)
    args = parser.parse_args()
    config = ComparisonConfig(**vars(args))
    summary = NuScenesComparisonVideo(config).run()
    print(json.dumps({key: value for key, value in summary.items() if key != "frames"}, indent=2))


if __name__ == "__main__":
    main()

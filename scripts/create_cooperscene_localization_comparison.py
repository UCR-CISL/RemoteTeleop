#!/usr/bin/env python3
"""Create a timestamp-checked CooperScene source/composited comparison video."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import sys

import cv2
import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


_FRAME_NAME = re.compile(r"^\d+_(\d+)\.png$")


@dataclass(frozen=True)
class ComparisonConfig:
    composited_video: Path
    compositor_metrics: Path
    sequence_root: Path | None
    mcap: Path | None
    localization: Path
    output_video: Path
    panel_height: int = 540


@dataclass(frozen=True)
class TimelineFrame:
    sequence: int
    frame_id: str
    timestamp_us: int
    source_image: Path


class CooperSceneComparisonVideo:
    """Validate the replay timeline and show its initial ENU localization error."""

    def __init__(self, config: ComparisonConfig) -> None:
        if config.panel_height <= 0:
            raise ValueError("panel_height must be positive")
        if (config.sequence_root is None) == (config.mcap is None):
            raise ValueError("provide exactly one of sequence_root or mcap")
        self.config = config

    def run(self) -> dict:
        timeline = self._load_timeline()
        initial_error = self._load_initial_error()
        expected_fps = self._validate_timestamps(timeline)
        capture = cv2.VideoCapture(str(self.config.composited_video))
        if not capture.isOpened():
            raise ValueError(f"failed to open video: {self.config.composited_video}")
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        frame_count = int(round(capture.get(cv2.CAP_PROP_FRAME_COUNT)))
        if frame_count != len(timeline):
            raise ValueError(
                f"video has {frame_count} frames but metrics contain {len(timeline)}"
            )
        if not math.isclose(fps, expected_fps, rel_tol=0.0, abs_tol=1e-3):
            raise ValueError(f"video is {fps:g} FPS but timestamps imply {expected_fps:g} FPS")

        panel_width = round(self.config.panel_height * 16 / 9)
        self.config.output_video.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(
            str(self.config.output_video), cv2.VideoWriter_fourcc(*"mp4v"), fps,
            (panel_width * 2, self.config.panel_height),
        )
        if not writer.isOpened():
            capture.release()
            raise ValueError(f"failed to open video writer: {self.config.output_video}")
        try:
            source_frames = self._source_frames(timeline)
            for index, (frame, source) in enumerate(zip(timeline, source_frames, strict=True)):
                ok, composited = capture.read()
                if not ok:
                    raise ValueError(f"failed to decode composited video frame {index}")
                source = cv2.resize(source, (panel_width, self.config.panel_height))
                composited = cv2.resize(composited, (panel_width, self.config.panel_height))
                timestamp = f"t = {frame.timestamp_us / 1_000_000:.1f} s"
                self._label(source, "Original CooperScene camera0", timestamp)
                self._label(composited, "Localized GSplat composite", timestamp)
                self._error_box(composited, initial_error)
                writer.write(np.concatenate((source, composited), axis=1))
        finally:
            capture.release()
            writer.release()

        summary = {
            "frame_count": len(timeline),
            "fps": fps,
            "start_timestamp_us": timeline[0].timestamp_us,
            "end_timestamp_us": timeline[-1].timestamp_us,
            "initial_localization_error": initial_error,
            "trajectory_evaluated": False,
        }
        self.config.output_video.with_suffix(".json").write_text(
            json.dumps(summary, indent=2), encoding="utf-8"
        )
        return summary

    def _load_timeline(self) -> tuple[TimelineFrame, ...]:
        records = [
            json.loads(line)
            for line in self.config.compositor_metrics.read_text(encoding="utf-8").splitlines()
            if line
        ]
        records = sorted(
            (record for record in records if record.get("event") == "compositor_frame"),
            key=lambda record: record["sequence"],
        )
        if not records:
            raise ValueError("compositor metrics contain no frames")
        if [record.get("sequence") for record in records] != list(range(len(records))):
            raise ValueError("compositor metric sequences must be contiguous and start at zero")
        frames = []
        for record in records:
            output_name = Path(str(record.get("output_path", ""))).name
            match = _FRAME_NAME.fullmatch(output_name)
            if match is None:
                raise ValueError(f"cannot recover frame ID from output path: {output_name}")
            frame_id = match.group(1)
            source_image = (
                self._source_image(frame_id) if self.config.sequence_root is not None
                else Path(frame_id)
            )
            frames.append(TimelineFrame(
                sequence=int(record["sequence"]), frame_id=frame_id,
                timestamp_us=int(record["source_timestamp_us"]), source_image=source_image,
            ))
        return tuple(frames)

    def _source_image(self, frame_id: str) -> Path:
        assert self.config.sequence_root is not None
        for suffix in (".png", ".jpg", ".jpeg"):
            path = self.config.sequence_root / f"{frame_id}_camera0{suffix}"
            if path.is_file():
                return path
        raise FileNotFoundError(f"missing camera0 image for CooperScene frame {frame_id}")

    def _source_frames(self, timeline: tuple[TimelineFrame, ...]):
        if self.config.sequence_root is not None:
            for frame in timeline:
                image = cv2.imread(str(frame.source_image), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError(f"failed to decode source image: {frame.source_image}")
                yield image
            return

        from mcap.reader import make_reader
        from src.deployment.ros_frame_adapter import (
            _decode_mcap_image,
            _ros_image_to_bgr,
            _timestamp_us,
        )

        assert self.config.mcap is not None
        with self.config.mcap.open("rb") as stream:
            records = make_reader(stream).iter_messages(
                topics=("/camera/image_raw",), log_time_order=False
            )
            count = 0
            for frame, (_schema, _channel, message) in zip(timeline, records, strict=True):
                decoded = _decode_mcap_image(message.data)
                timestamp_us = _timestamp_us(decoded)
                if timestamp_us != frame.timestamp_us:
                    raise ValueError(
                        f"MCAP image timestamp {timestamp_us} does not match compositor "
                        f"timestamp {frame.timestamp_us} at sequence {frame.sequence}"
                    )
                count += 1
                yield _ros_image_to_bgr(decoded)
            if count != len(timeline):
                raise ValueError(
                    f"MCAP contains {count} source images but timeline has {len(timeline)} frames"
                )

    def _load_initial_error(self) -> dict:
        payload = json.loads(self.config.localization.read_text(encoding="utf-8"))
        error = payload.get("initial_localization_error")
        if not isinstance(error, dict):
            raise ValueError("localization must contain initial_localization_error")
        return {
            "frame_id": str(error["frame_id"]),
            "reference_frame": str(error["reference_frame"]),
            "translation_m": float(error.get(
                "horizontal_translation_m", error["translation_m"]
            )),
            "orientation_deg": float(error["orientation_deg"]),
        }

    @staticmethod
    def _validate_timestamps(timeline: tuple[TimelineFrame, ...]) -> float:
        timestamps = np.asarray([frame.timestamp_us for frame in timeline], dtype=np.int64)
        if len(timestamps) < 2:
            raise ValueError("at least two timestamped frames are required")
        deltas = np.diff(timestamps)
        if np.any(deltas <= 0) or not np.all(deltas == deltas[0]):
            raise ValueError("source timestamps must have one positive, constant cadence")
        return 1_000_000.0 / float(deltas[0])

    @staticmethod
    def _label(image: np.ndarray, title: str, timestamp: str) -> None:
        overlay = image.copy()
        cv2.rectangle(overlay, (12, 10), (410, 76), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.65, image, 0.35, 0.0, image)
        cv2.putText(image, title, (24, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(image, timestamp, (24, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (230, 230, 230), 1)

    @staticmethod
    def _error_box(image: np.ndarray, error: dict) -> None:
        width = image.shape[1]
        left = width - 350
        overlay = image.copy()
        cv2.rectangle(overlay, (left, 10), (width - 12, 96), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.70, image, 0.30, 0.0, image)
        lines = (
            f"Initial ENU error (frame {error['frame_id']})",
            f"BEV translation: {error['translation_m']:.3f} m",
            f"Orientation: {error['orientation_deg']:.3f} deg",
        )
        for index, line in enumerate(lines):
            cv2.putText(
                image, line, (left + 12, 34 + index * 25), cv2.FONT_HERSHEY_SIMPLEX,
                0.52, (255, 255, 255), 1 if index == 0 else 2,
            )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--composited-video", type=Path, required=True)
    parser.add_argument("--compositor-metrics", type=Path, required=True)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--sequence-root", type=Path)
    source.add_argument("--mcap", type=Path)
    parser.add_argument("--localization", type=Path, required=True)
    parser.add_argument("--output-video", type=Path, required=True)
    parser.add_argument("--panel-height", type=int, default=540)
    args = parser.parse_args()
    summary = CooperSceneComparisonVideo(ComparisonConfig(**vars(args))).run()
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()

"""Durably spool remote render state and vehicle-local SAM analysis frames.

The replay MCAP may still contain camera images for the dedicated video path,
The remote stream remains image-free. When configured, camera images are
joined locally with metadata and written to a separate receiver-pulled spool.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import signal
from pathlib import Path
from queue import Empty, Full, Queue
import time
from threading import Event, Lock, Thread
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any, Mapping, Protocol

import cv2
import numpy as np
import zmq

from src.common.vehicle_categories import vehicle_proxy_kind
from src.localization.cooperscene_calibration import CALIBRATIONS, DEFAULT_CALIBRATION, camera_metadata
from src.realtime.process_support import JsonlMetrics
from src.realtime.protocol import BoxPrompt, EgoPoseSample, FrameDetections, FrameEnd, FrameObjectDetections, FrameSnapshot
from src.realtime.bevfusion_tracker import BevFusionKalmanTracker, WorldDetection
from src.realtime.transport import RouterTransport
from src.deployment.lidar_ground_snap import LidarGroundSnapper
from src.deployment.vehicle import VehicleAnalysisSpool, VehicleAssetSession, VehicleAssetStore, VehicleFrameSession, VehicleFrameSpool
from src.reconstruction.models import VehicleDimensions
from src.reconstruction.projection import project_vehicle_box

if TYPE_CHECKING:
    from src.deployment.vehicle_appearance import VehicleAppearanceEstimator


class PosePublisher(Protocol):
    def send(
        self, message: FrameObjectDetections | EgoPoseSample, *, topic: str | None = None
    ) -> None: ...


class SnapshotSink(Protocol):
    def append(self, snapshot: FrameSnapshot) -> None: ...


class BevFusionClient(Protocol):
    """Non-blocking detector transport used by the MCAP adapter."""

    def submit(self, request: Mapping[str, Any]) -> bool: ...
    def poll(self) -> Any | None: ...


class LatestBevFusionBoxes:
    """Join local sensor records and retain the newest completed detection batch.

    One request is allowed in flight.  When inference is slower than playback,
    later complete sensor triplets replace the single pending request instead
    of stalling the durable 10 Hz stream.

    ``request_calibration`` selects the CooperScene camera calibration sent to
    the detector, independently of the rendering calibration in the metadata:
    the CooperScene checkpoint was trained with the ``baseline`` lidar2cam.
    It applies only to metadata tagged with ``camera_calibration``/``camera_agent``
    (CooperScene); ``None`` sends the metadata camera unchanged.

    ``appearance`` optionally fits per-track shape/paint hints from each request's
    LiDAR and image. Color projects with the metadata (rendering) camera, not the
    detector's ``request_calibration``, so samples match the composited view.
    """

    def __init__(self, client: BevFusionClient, *, score_threshold: float, minimum_area_px: float = 64.0,
                 continuation_score_threshold: float | None = None, minimum_confirmation_hits: int = 1,
                 suppress_duplicate_births: bool = False,
                 maximum_reacquisition_seconds: float = 0.0,
                 maximum_coast_seconds: float = 0.5,
                 request_calibration: str | None = "baseline",
                 track_log: JsonlMetrics | None = None,
                 ground_snapper: LidarGroundSnapper | None = None,
                 appearance: VehicleAppearanceEstimator | None = None) -> None:
        if not 0.0 <= score_threshold <= 1.0:
            raise ValueError("bevfusion score threshold must be between zero and one")
        self.client = client
        # Optional offline record of every live track per published frame
        # (including tracks outside the camera), for tracking evaluation.
        self.track_log = track_log
        self.request_calibration = request_calibration
        # Optional: snap detection heights onto the request's LiDAR ground before tracking.
        self.ground_snapper = ground_snapper
        self.appearance = appearance
        self._inflight_lidar_points: dict[str, np.ndarray] = {}
        # request_id -> (full-res BGR, K, camera_T_lidar) for in-flight/pending color work.
        self._request_cameras: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
        self.score_threshold = score_threshold
        self.minimum_area_px = minimum_area_px
        self._metadata: dict[int, Mapping[str, Any]] = {}
        self._images: dict[int, np.ndarray] = {}
        self._lidar: dict[int, np.ndarray] = {}
        self._in_flight = False
        self._inflight_lidar_poses: dict[str, np.ndarray] = {}
        self._pending: Any | None = None
        self._latest_sequence: int | None = None
        self._tracker = BevFusionKalmanTracker(
            birth_score_threshold=score_threshold,
            continuation_score_threshold=score_threshold if continuation_score_threshold is None else continuation_score_threshold,
            minimum_confirmation_hits=minimum_confirmation_hits,
            suppress_duplicate_births=suppress_duplicate_births,
            maximum_reacquisition_seconds=maximum_reacquisition_seconds,
            maximum_coast_seconds=maximum_coast_seconds,
        )
        self.submitted_count = 0
        self.replaced_pending_count = 0
        self.result_count = 0
        self.error_count = 0

    @property
    def latest_sequence(self) -> int | None:
        return self._latest_sequence

    def receive_metadata(self, payload: Mapping[str, Any], sequence: int) -> tuple[int, int] | None:
        self._poll()
        value = dict(payload)
        value["source_sequence"] = sequence
        timestamp = _non_negative_int(value.get("timestamp_us"), "timestamp_us")
        self._metadata[timestamp] = value
        image = self._images.get(timestamp)
        image_shape = None if image is None else image.shape[:2]
        self._try_submit(timestamp)
        return image_shape

    def receive_image(self, message: Any) -> None:
        timestamp = _timestamp_us(message)
        self._images[timestamp] = _ros_image_to_bgr(message)
        self._try_submit(timestamp)

    def receive_lidar(self, message: Any) -> None:
        timestamp = _timestamp_us(message)
        self._lidar[timestamp] = np.asarray(message.points, dtype=np.float32)
        self._try_submit(timestamp)

    def boxes_for(self, payload: Mapping[str, Any], image_shape: tuple[int, int] | None = None) -> tuple[BoxPrompt, ...]:
        self._poll()
        intrinsic = np.asarray(payload["camera_intrinsic"], dtype=np.float64).reshape(3, 3)
        world_T_camera = np.asarray(payload["world_T_camera"], dtype=np.float64).reshape(4, 4)
        if image_shape is None:
            image = self._images.get(_non_negative_int(payload["timestamp_us"], "timestamp_us"))
            if image is None:
                return ()
            image_shape = image.shape[:2]
        boxes = []
        if self._latest_sequence is None:
            return ()
        timestamp_us = _non_negative_int(payload["timestamp_us"], "timestamp_us")
        tracks = self._tracker.boxes_at(timestamp_us)
        if self.track_log is not None:
            self.track_log.write("bevfusion_tracks", {
                "sequence": payload.get("sequence"), "frame_id": payload.get("frame_id"),
                "timestamp_us": timestamp_us, "bevfusion_result_sequence": self._latest_sequence,
                "tracks": [dict(track_id=t.track_id, label=t.label, score=float(t.score),
                                dimensions_lwh=[float(v) for v in t.dimensions_lwh],
                                world_T_object=[float(v) for v in np.asarray(t.world_T_object).reshape(-1)])
                           for t in tracks]})
        for tracked in tracks:
            world_T_object = tracked.world_T_object
            dimensions = tracked.dimensions_lwh
            label = tracked.label
            score = tracked.score
            if vehicle_proxy_kind(label) is None:
                continue
            projected = project_vehicle_box(
                world_T_object=world_T_object, world_T_camera=world_T_camera,
                intrinsics=intrinsic, dimensions=VehicleDimensions(*dimensions),
                image_size=image_shape, minimum_area_px=self.minimum_area_px,
                clip_near_plane=True,  # Retain the in-frame portion of tracked render prompts.
            )
            if projected is None:
                continue
            boxes.append(BoxPrompt(
                track_id=tracked.track_id,
                xyxy=tuple(float(value) for value in projected.xyxy), dimensions_lwh=dimensions,
                world_T_object=tuple(float(value) for value in world_T_object.reshape(-1)),
                text_label=label, visibility=projected.visible_fraction,
                detection_confidence=score,
                **({} if self.appearance is None else self.appearance.prompt_fields(tracked.track_id)),
            ))
        return tuple(boxes)

    def _try_submit(self, timestamp: int) -> None:
        metadata = self._metadata.get(timestamp)
        image = self._images.get(timestamp)
        points = self._lidar.get(timestamp)
        if metadata is None or image is None or points is None:
            return
        from src.realtime.bevfusion import BevFusionRequest
        camera = metadata
        if self.request_calibration is not None and "camera_calibration" in metadata:
            camera = camera_metadata(metadata["camera_agent"], _rigid_matrix(
                metadata.get("map_T_lidar", metadata["world_T_ego"]), "map_T_lidar"), self.request_calibration)
        request = BevFusionRequest(
            request_id=f"{metadata['scene_generation']}:{metadata['source_sequence']}",
            scene_generation=str(metadata["scene_generation"]), sequence=int(metadata["source_sequence"]),
            frame_id=str(metadata["frame_id"]), timestamp_us=timestamp, config_id="nuscenes",
            camera_intrinsic=tuple(float(value) for value in camera["camera_intrinsic"]),
            world_T_camera=tuple(float(value) for value in camera["world_T_camera"]),
            world_T_lidar=tuple(float(value) for value in metadata.get(
                "map_T_lidar", metadata["world_T_ego"]
            )),
            image_width=int(image.shape[1]), image_height=int(image.shape[0]),
            image_bgr=np.ascontiguousarray(image).tobytes(), lidar_point_count=len(points),
            lidar_f32=np.ascontiguousarray(points, dtype="<f4").tobytes(),
        )
        del self._metadata[timestamp], self._images[timestamp], self._lidar[timestamp]
        if self.appearance is not None and self.appearance.config.color:
            # Rendering camera from the same metadata: camera_T_lidar is invariant to relocalization.
            self._request_cameras[request.request_id] = (
                image, np.asarray(metadata["camera_intrinsic"], dtype=np.float64).reshape(3, 3),
                np.linalg.inv(_rigid_matrix(metadata["world_T_camera"], "world_T_camera"))
                @ _rigid_matrix(request.world_T_lidar, "map_T_lidar"))
        if self._in_flight:
            if self._pending is not None:
                self.replaced_pending_count += 1
                self._request_cameras.pop(self._pending.request_id, None)
            self._pending = request
        elif self.client.submit(request):
            self._in_flight = True
            self.submitted_count += 1
            self._inflight_lidar_poses[request.request_id] = np.asarray(request.world_T_lidar, dtype=np.float64).reshape(4, 4)
            self._remember_lidar_points(request)
        else:
            if self._pending is not None:
                self._request_cameras.pop(self._pending.request_id, None)
            self._pending = request

    def _remember_lidar_points(self, request: Any) -> None:
        if self.ground_snapper is not None or self.appearance is not None:
            self._inflight_lidar_points[request.request_id] = np.frombuffer(
                request.lidar_f32, dtype="<f4").reshape(request.lidar_point_count, -1)

    def poll(self) -> None:
        """Process ready inference and latest pending work without publishing frames."""
        self._poll()

    def idle(self, seconds: float, sleep: Any = time.sleep) -> None:
        """Sleep up to ``seconds``, returning early when an in-flight reply is readable."""
        wait = getattr(self.client, "wait", None)
        if wait is None or not self._in_flight:
            sleep(seconds)
        else:
            wait(max(1, int(seconds * 1000)))

    def wait_for_sequence(self, sequence: int, *, timeout_seconds: float = 30.0,
                          monotonic: Any = time.monotonic, sleep: Any = time.sleep) -> None:
        """Bounded offline wait for this capture; polling never publishes frames."""
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
            raise ValueError("detector sequence must be a nonnegative integer")
        if not np.isfinite(timeout_seconds) or timeout_seconds <= 0:
            raise ValueError("detector wait timeout must be finite and positive")
        deadline = monotonic() + timeout_seconds
        while True:
            if self.error_count:
                raise RuntimeError("BEVFusion failed while awaiting offline capture")
            if self.latest_sequence == sequence:
                return
            if self.latest_sequence is not None and self.latest_sequence > sequence:
                raise RuntimeError("BEVFusion skipped the awaited capture")
            self.poll()
            if self.error_count:
                raise RuntimeError("BEVFusion failed while awaiting offline capture")
            if self.latest_sequence == sequence:
                return
            remaining = deadline - monotonic()
            if remaining <= 0:
                raise TimeoutError(f"BEVFusion capture {sequence} did not complete")
            self.idle(min(remaining, .01), sleep)

    def _poll(self) -> None:
        result = self.client.poll()
        if result is None:
            return
        self._in_flight = False
        self.result_count += 1
        request_id = _result_value(result, "request_id")
        world_T_lidar = self._inflight_lidar_poses.pop(request_id, None)
        lidar_points = self._inflight_lidar_points.pop(request_id, None)
        request_camera = self._request_cameras.pop(request_id, None)
        if world_T_lidar is not None and _result_value(result, "error") is None:
            detections = tuple(
                WorldDetection(world_T_object, dimensions, label, score)
                for world_T_object, dimensions, label, score
                in _bevfusion_world_boxes(result, world_T_lidar, self.ground_snapper, lidar_points)
            )
            timestamp_us = int(_result_value(result, "timestamp_us"))
            self._tracker.update(detections, timestamp_us=timestamp_us)
            if self.appearance is not None and lidar_points is not None:
                from src.deployment.vehicle_appearance import AppearanceCamera
                self.appearance.update(
                    self._tracker.last_matches, self._tracker.boxes_at(timestamp_us), timestamp_us=timestamp_us,
                    world_T_lidar=world_T_lidar, lidar_points=lidar_points,
                    camera=None if request_camera is None else AppearanceCamera(*request_camera))
            self._latest_sequence = int(_result_value(result, "sequence"))
        elif _result_value(result, "error") is not None:
            self.error_count += 1
        if self._pending is not None:
            pending, self._pending = self._pending, None
            if self.client.submit(pending):
                self._in_flight = True
                self.submitted_count += 1
                self._inflight_lidar_poses[pending.request_id] = np.asarray(pending.world_T_lidar, dtype=np.float64).reshape(4, 4)
                self._remember_lidar_points(pending)
            else:
                self._pending = pending

    def close(self) -> None:
        if self.appearance is not None:
            self.appearance.close()
        if self._metadata or self._images or self._lidar:
            raise RuntimeError("BEVFusion replay ended with unmatched image, metadata, or LiDAR timestamps")
        close = getattr(self.client, "close", None)
        if close is not None:
            close()


class RosAnalysisAssembler:
    """Exactly join ROS images and metadata into durable local analysis frames."""

    def __init__(
        self,
        sink: VehicleAnalysisSpool,
        *,
        expected_samples: int | None = None,
        maximum_pending: int = 2_048,
        jpeg_quality: int = 90,
    ) -> None:
        if maximum_pending <= 0:
            raise ValueError("maximum_pending must be positive")
        if not 1 <= jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")
        self.sink = sink
        self.expected_samples = expected_samples
        self.maximum_pending = maximum_pending
        self.jpeg_quality = jpeg_quality
        self._metadata: dict[int, tuple[Mapping[str, Any], int]] = {}
        self._images: dict[int, Any] = {}
        self._written = 0
        self._lock = Lock()
        self._pairs: Queue[tuple[Mapping[str, Any], int, Any] | None] = Queue(
            maxsize=maximum_pending
        )
        self._failure: BaseException | None = None
        self._closed = False
        self._thread = Thread(
            target=self._run, name="ros-analysis-spool-writer", daemon=True
        )
        self._thread.start()

    def receive_metadata(self, payload: Mapping[str, Any], sequence: int) -> None:
        timestamp_us = _non_negative_int(payload.get("timestamp_us"), "timestamp_us")
        with self._lock:
            self.raise_if_failed()
            if timestamp_us in self._metadata:
                raise ValueError(f"duplicate analysis metadata timestamp {timestamp_us}")
            self._metadata[timestamp_us] = (dict(payload), sequence)
            self._check_bound()
            self._enqueue_pair(timestamp_us)

    def receive_image(self, message: Any) -> None:
        stamp = message.header.stamp
        timestamp_us = int(stamp.sec) * 1_000_000 + int(stamp.nanosec) // 1_000
        with self._lock:
            self.raise_if_failed()
            if timestamp_us in self._images:
                raise ValueError(f"duplicate analysis image timestamp {timestamp_us}")
            self._images[timestamp_us] = message
            self._check_bound()
            self._enqueue_pair(timestamp_us)

    def _check_bound(self) -> None:
        if len(self._metadata) > self.maximum_pending or len(self._images) > self.maximum_pending:
            raise RuntimeError("analysis image/metadata join exceeded its lossless pending bound")

    def _enqueue_pair(self, timestamp_us: int) -> None:
        metadata_item = self._metadata.get(timestamp_us)
        image = self._images.get(timestamp_us)
        if metadata_item is None or image is None:
            return
        payload, sequence = metadata_item
        del self._metadata[timestamp_us]
        del self._images[timestamp_us]
        self._pairs.put_nowait((payload, sequence, image))

    def _write_pair(
        self, payload: Mapping[str, Any], sequence: int, image: Any
    ) -> None:
        timestamp_us = _non_negative_int(payload.get("timestamp_us"), "timestamp_us")
        bgr = _ros_image_to_bgr(image)
        ok, encoded = cv2.imencode(
            ".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not ok:
            raise ValueError("failed to encode analysis image as JPEG")
        frame = FrameDetections(
            frame_id=_non_empty_string(payload.get("frame_id"), "frame_id"),
            request_id=_non_empty_string(payload.get("request_id"), "request_id"),
            scene_generation=_non_empty_string(
                payload.get("scene_generation"), "scene_generation"
            ),
            timestamp_us=timestamp_us,
            image_jpeg=encoded.tobytes(),
            camera_intrinsic=payload.get("camera_intrinsic"),
            world_T_camera=payload.get("world_T_camera"),
            world_T_ego=payload.get("world_T_ego"),
            boxes=_box_prompts(payload.get("boxes", ())),
            end_of_scene=bool(payload.get("end_of_scene", False)),
            source_sequence=sequence,
        )
        self.sink.append(frame)
        self._written += 1
        if self.expected_samples is not None and self._written == self.expected_samples:
            self.sink.finalize(FrameEnd(frame.scene_generation, sequence, timestamp_us))
        elif self.expected_samples is not None and self._written > self.expected_samples:
            raise RuntimeError("received more analysis frames than expected_samples")

    def _run(self) -> None:
        try:
            while True:
                item = self._pairs.get()
                try:
                    if item is None:
                        return
                    self._write_pair(*item)
                finally:
                    self._pairs.task_done()
        except BaseException as error:
            self._failure = error
            while True:
                try:
                    self._pairs.get_nowait()
                except Empty:
                    break
                else:
                    self._pairs.task_done()

    def raise_if_failed(self) -> None:
        if self._failure is not None:
            raise RuntimeError("durable analysis spool writer failed") from self._failure

    def close(self) -> None:
        if self._closed:
            self.raise_if_failed()
            return
        self._closed = True
        self._pairs.join()
        self._pairs.put(None)
        self._thread.join()
        self.raise_if_failed()
        if self._metadata or self._images:
            raise RuntimeError(
                "analysis replay ended with unmatched image or metadata timestamps"
            )
        if self.expected_samples is not None and self._written != self.expected_samples:
            raise RuntimeError(
                f"expected {self.expected_samples} durable analysis frames, got {self._written}"
            )


@dataclass(frozen=True)
class _McapImage:
    """The small ``sensor_msgs/Image`` surface consumed by the analysis spool."""

    header: Any
    height: int
    width: int
    step: int
    encoding: str
    data: bytes


@dataclass(frozen=True)
class _McapPointCloud:
    header: Any
    points: np.ndarray


class _PrefetchedFrames:
    """Read/decode the next recorded frames on a worker thread, in source order.

    Decoding then overlaps detector waits and pacing sleeps instead of
    delaying them. Errors are re-raised in the consumer at the failing frame.
    """

    _END = object()

    def __init__(self, frames: Any, depth: int = 2) -> None:
        self._queue: Queue[Any] = Queue(maxsize=depth)
        self._stop = Event()
        self._thread = Thread(target=self._run, args=(frames,), name="mcap-frame-prefetch", daemon=True)
        self._thread.start()

    def _put(self, item: Any) -> bool:
        while not self._stop.is_set():
            try:
                self._queue.put(item, timeout=0.1)
                return True
            except Full:
                continue
        return False

    def _run(self, frames: Any) -> None:
        try:
            for frame in frames:
                if not self._put(frame):
                    return
            self._put(self._END)
        except BaseException as error:
            self._put(error)

    def __iter__(self) -> Any:
        try:
            while True:
                item = self._queue.get()
                if item is self._END:
                    return
                if isinstance(item, BaseException):
                    raise item
                yield item
        finally:
            self._stop.set()


class McapFrameReplay:
    """Replay ROS2 or Foxglove MCAP recordings without ROS/DDS delivery.

    This is deliberately playback-only: the live vehicle continues to use the
    ROS subscriptions below.  The reader keeps MCAP record order (rather than
    sorting equal timestamps) and paces selected records using their log time.
    """

    def __init__(
        self,
        adapter: "RosFrameAdapter",
        *,
        image_topic: str,
        metadata_topic: str,
        lidar_topic: str | None = None,
        expected_samples: int | None,
        rate: float = 1.0,
        sleep: Any = time.sleep,
        monotonic: Any = time.monotonic,
        reader_factory: Any = None,
        agent: str = "1",
        start_sequence: int = 0,
        xyz_pose_sidecar: Any = None,
        await_detections: bool = False,
        camera_calibration: str | None = DEFAULT_CALIBRATION,
    ) -> None:
        if isinstance(start_sequence, bool) or not isinstance(start_sequence, int) or start_sequence < 0:
            raise ValueError("start_sequence must be a non-negative integer")
        self.await_detections = await_detections
        if await_detections and (adapter.bevfusion is None or adapter.config.bbox_source != "bevfusion"):
            raise ValueError("await detections requires BEVFusion")
        self.xyz_pose_sidecar = xyz_pose_sidecar
        if xyz_pose_sidecar is not None:
            if start_sequence not in (0, xyz_pose_sidecar.start_index):
                raise ValueError("XYZ sidecar range conflicts with start_sequence")
            if expected_samples is not None and expected_samples != len(xyz_pose_sidecar.frames):
                raise ValueError("XYZ sidecar expected_samples/range mismatch")
            config = adapter.config
            if (config.gaussian_T_cooperscene is not None or config.gaussian_T_nuscenes_global is not None
                    or adapter.localization_metrics is not None):
                raise ValueError("XYZ sidecar cannot combine with static relocalization/metrics")
            if getattr(adapter.analysis_assembler, "localizer", None) is not None:
                raise ValueError("XYZ sidecar cannot combine with an online localizer")
            start_sequence = xyz_pose_sidecar.start_index
        self.start_sequence = start_sequence
        if rate <= 0:
            raise ValueError("MCAP replay rate must be positive")
        if not image_topic.startswith("/") or not metadata_topic.startswith("/"):
            raise ValueError("MCAP topics must be absolute")
        if image_topic == metadata_topic or lidar_topic in {image_topic, metadata_topic}:
            raise ValueError("MCAP image and metadata topics must differ")
        if adapter.analysis_assembler is None:
            raise ValueError("direct MCAP replay requires an analysis assembler")
        self.adapter = adapter
        self.image_topic = image_topic
        self.metadata_topic = metadata_topic
        self.lidar_topic = lidar_topic
        self.expected_samples = expected_samples
        self.rate = rate
        self.sleep = sleep
        self.monotonic = monotonic
        self.reader_factory = reader_factory or _mcap_reader
        self.agent = agent
        # CooperScene camera used for replay projection/rendering (Foxglove and
        # XYZ-sidecar sources); None keeps the sidecar-bound camera. Detector
        # inputs are chosen separately by LatestBevFusionBoxes.
        self.camera_calibration = camera_calibration

    def _wait_until(self, target: float) -> None:
        """Service pending detector work while preserving the capture deadline."""
        remaining = target - self.monotonic()
        if self.adapter.bevfusion is None:
            if remaining > 0:
                self.sleep(remaining)
            return
        idle = getattr(self.adapter.bevfusion, "idle", lambda seconds, sleep: sleep(seconds))
        while remaining > 0:
            self.adapter.bevfusion.poll()
            remaining = target - self.monotonic()
            if remaining > 0:
                idle(min(remaining, 0.01), self.sleep)
            remaining = target - self.monotonic()

    def replay(self, path: Path) -> None:
        if self.xyz_pose_sidecar is not None:
            self.xyz_pose_sidecar.validate_mcap(path, self.agent)
        metadata_count = 0
        image_count = 0
        lidar_count = 0
        first_log_time: int | None = None
        previous_log_time: int | None = None
        replay_started = self.monotonic()
        with path.open("rb") as stream:
            reader = self.reader_factory(stream)
            summary = reader.get_summary() if hasattr(reader, "get_summary") else None
            if summary is not None and any(
                schema.name == "foxglove.CompressedImage" for schema in summary.schemas.values()
            ):
                self._replay_foxglove(path)
                return
            if self.xyz_pose_sidecar is not None:
                self._replay_xyz_ros(reader, summary)
                return
            if self.await_detections:
                raise ValueError("await detections requires a Foxglove MCAP or XYZ sidecar")
            if self.start_sequence:
                raise ValueError("nonzero start_sequence requires a Foxglove MCAP")
            for schema, channel, message in reader.iter_messages(
                topics=tuple(topic for topic in (self.image_topic, self.metadata_topic, self.lidar_topic) if topic), log_time_order=False
            ):
                _validate_mcap_record(
                    channel.topic,
                    channel,
                    schema,
                    image_topic=self.image_topic,
                    metadata_topic=self.metadata_topic,
                    lidar_topic=self.lidar_topic,
                )
                if previous_log_time is not None and message.log_time < previous_log_time:
                    raise ValueError("MCAP source records are not in nondecreasing log-time order")
                if first_log_time is None:
                    first_log_time = message.log_time
                target = replay_started + (
                    (message.log_time - first_log_time) / 1_000_000_000.0 / self.rate
                )
                self._wait_until(target)
                previous_log_time = message.log_time
                if channel.topic == self.image_topic:
                    self.adapter.analysis_assembler.receive_image(_decode_mcap_image(message.data))
                    if self.adapter.bevfusion is not None:
                        self.adapter.bevfusion.receive_image(_decode_mcap_image(message.data))
                    image_count += 1
                elif channel.topic == self.metadata_topic:
                    self.adapter.receive_metadata(_decode_mcap_string(message.data))
                    metadata_count += 1
                elif channel.topic == self.lidar_topic:
                    if self.adapter.bevfusion is not None:
                        self.adapter.bevfusion.receive_lidar(_decode_mcap_point_cloud2(message.data))
                    lidar_count += 1
        if self.expected_samples is not None:
            if metadata_count != self.expected_samples:
                raise RuntimeError(
                    f"expected {self.expected_samples} MCAP metadata records, got {metadata_count}"
                )
            if image_count != self.expected_samples:
                raise RuntimeError(
                    f"expected {self.expected_samples} MCAP image records, got {image_count}"
                )
            if self.lidar_topic is not None and lidar_count != self.expected_samples:
                raise RuntimeError(
                    f"expected {self.expected_samples} MCAP LiDAR records, got {lidar_count}"
                )
        if self.adapter.analysis_assembler is not None:
            self.adapter.analysis_assembler.raise_if_failed()


    def _deliver_xyz(self, index, metadata, image_bgr, lidar_points):
        metadata, points = self.xyz_pose_sidecar.bind(index, metadata, image_bgr, lidar_points)
        if self.camera_calibration is not None:
            metadata.update(camera_metadata(self.agent, np.asarray(metadata["map_T_lidar"]).reshape(4, 4),
                                            self.camera_calibration))
        timestamp_ns = metadata["timestamp_us"] * 1000
        header = SimpleNamespace(stamp=SimpleNamespace(sec=timestamp_ns // 10**9,
            nanosec=timestamp_ns % 10**9), frame_id=metadata["frame_id"])
        h, w = image_bgr.shape[:2]
        image = _McapImage(header, h, w, w * 3, "bgr8", image_bgr.tobytes())
        self.adapter.analysis_assembler.receive_image(image)
        if self.adapter.bevfusion is not None:
            self.adapter.bevfusion.receive_image(image)
            self.adapter.bevfusion.receive_lidar(_McapPointCloud(header, points))
        self.adapter.receive_metadata(metadata, wait_for_bevfusion=self.await_detections)

    def _replay_xyz_ros(self, reader, summary):
        """Join exact original indices; supplemental LiDAR is explicit in the sidecar."""
        sidecar = self.xyz_pose_sidecar
        topics = [self.image_topic, self.metadata_topic]
        contained_lidar = self.lidar_topic is not None and summary is not None and any(
            c.topic == self.lidar_topic for c in summary.channels.values())
        if contained_lidar:
            topics.append(self.lidar_topic)
        pending = {}
        count = 0
        started = self.monotonic()
        first_log_time = None
        previous_log_time = None
        for schema, channel, message in reader.iter_messages(topics=topics, log_time_order=False):
            index = message.sequence
            if index < sidecar.start_index:
                continue
            if index > sidecar.end_index:
                break
            _validate_mcap_record(channel.topic, channel, schema, image_topic=self.image_topic,
                                  metadata_topic=self.metadata_topic, lidar_topic=self.lidar_topic)
            if previous_log_time is not None and message.log_time < previous_log_time:
                raise ValueError("XYZ MCAP records are not in nondecreasing log-time order")
            previous_log_time = message.log_time
            row = pending.setdefault(index, {})
            if len(pending) > 8:
                raise ValueError("unbalanced XYZ source frame streams")
            key = ("image" if channel.topic == self.image_topic else
                   "metadata" if channel.topic == self.metadata_topic else "lidar")
            if key in row:
                raise ValueError("duplicate XYZ source sensor record")
            if key == "image":
                image = _decode_mcap_image(message.data)
                row[key] = _ros_image_to_bgr(image)
                row["image_timestamp_us"] = _timestamp_us(image)
            elif key == "metadata":
                row[key] = json.loads(_decode_mcap_string(message.data))
            else:
                cloud = _decode_mcap_point_cloud2(message.data)
                row[key] = cloud.points
                row["lidar_timestamp_us"] = _timestamp_us(cloud)
            if first_log_time is None:
                first_log_time = message.log_time
                # Reading the skipped prefix is outside selected-frame playback.
                started = self.monotonic()
            self._wait_until(started + (message.log_time - first_log_time) / 1e9 / self.rate)
            if "image" in row and "metadata" in row and (not contained_lidar or "lidar" in row):
                if row["image_timestamp_us"] != row["metadata"]["timestamp_us"] or (contained_lidar and
                        row["lidar_timestamp_us"] != row["metadata"]["timestamp_us"]):
                    raise ValueError("XYZ ROS recorded sensor clock mismatch")
                self._deliver_xyz(index, row["metadata"], row["image"], row.get("lidar"))
                del pending[index]
                count += 1
        sidecar.finish()
        if pending or (self.expected_samples is not None and count != self.expected_samples):
            raise RuntimeError("XYZ replay range/count incomplete")
        self.adapter.analysis_assembler.raise_if_failed()

    def _replay_foxglove(self, path: Path) -> None:
        from src.data.foxglove_mcap import FoxgloveMcapReader

        frames = FoxgloveMcapReader(
            path, agent=self.agent, scene_generation=self.adapter.scene_generation,
            include_lidar=self.lidar_topic is not None or self.xyz_pose_sidecar is not None,
            # XYZ sidecars record and validate the baseline camera; the replay
            # calibration is re-applied after binding (see _deliver_xyz).
            calibration=("baseline" if self.xyz_pose_sidecar is not None
                         else self.camera_calibration or DEFAULT_CALIBRATION),
        )
        started = self.monotonic()
        first_timestamp = None
        first_metadata_timestamp_us = None
        count = 0
        for frame in _PrefetchedFrames(frames):
            if frame.sequence < self.start_sequence:
                continue
            if self.xyz_pose_sidecar is not None and frame.sequence > self.xyz_pose_sidecar.end_index:
                break
            if first_timestamp is None:
                first_timestamp = frame.timestamp_ns
                first_metadata_timestamp_us = int(frame.metadata["timestamp_us"])
                started = self.monotonic()
            target = started + (frame.timestamp_ns - first_timestamp) / 1e9 / self.rate
            self._wait_until(target)
            if self.xyz_pose_sidecar is not None:
                if frame.lidar_points is None:
                    raise ValueError("XYZ Foxglove replay requires contained LiDAR")
                self._deliver_xyz(frame.sequence, frame.metadata, frame.image_bgr, frame.lidar_points)
                count += 1
                continue
            metadata = dict(frame.metadata)
            metadata["recorded_source_sequence"] = frame.sequence
            metadata["recorded_timestamp_us"] = metadata["timestamp_us"]
            metadata["source_sequence"] = count
            metadata["timestamp_us"] = int(metadata["timestamp_us"]) - first_metadata_timestamp_us
            timestamp_ns = metadata["timestamp_us"] * 1000
            header = SimpleNamespace(
                stamp=SimpleNamespace(sec=timestamp_ns // 10**9, nanosec=timestamp_ns % 10**9),
                frame_id=str(frame.metadata["frame_id"]),
            )
            height, width = frame.image_bgr.shape[:2]
            image = _McapImage(header, height, width, width * 3, "bgr8", frame.image_bgr.tobytes())
            self.adapter.analysis_assembler.receive_image(image)
            if self.adapter.bevfusion is not None:
                self.adapter.bevfusion.receive_image(image)
                if frame.lidar_points is not None:
                    self.adapter.bevfusion.receive_lidar(_McapPointCloud(header, frame.lidar_points))
            if self.await_detections:
                self.adapter.receive_metadata(metadata, wait_for_bevfusion=True)
            else:
                self.adapter.receive_metadata(metadata)
            count += 1
        if self.xyz_pose_sidecar is not None:
            self.xyz_pose_sidecar.finish()
        if self.expected_samples is not None and count != self.expected_samples:
            raise RuntimeError(f"expected {self.expected_samples} MCAP frames, got {count}")
        self.adapter.analysis_assembler.raise_if_failed()


def _mcap_reader(stream: Any) -> Any:
    try:
        from mcap.reader import make_reader
    except ImportError as error:  # pragma: no cover - deployment environment dependent
        raise RuntimeError(
            "direct MCAP replay requires the Python 'mcap' package"
        ) from error
    return make_reader(stream)


def _validate_mcap_record(
    topic: str,
    channel: Any,
    schema: Any,
    *,
    image_topic: str,
    metadata_topic: str,
    lidar_topic: str | None = None,
) -> None:
    if topic == image_topic:
        expected_schema = "sensor_msgs/msg/Image"
    elif topic == metadata_topic:
        expected_schema = "std_msgs/msg/String"
    elif topic == lidar_topic:
        expected_schema = "sensor_msgs/msg/PointCloud2"
    else:
        raise ValueError(f"unexpected direct-MCAP topic: {topic}")
    if channel.message_encoding != "cdr":
        raise ValueError(f"MCAP topic {topic} must use CDR encoding")
    if schema is None or schema.name != expected_schema:
        actual = None if schema is None else schema.name
        raise ValueError(f"MCAP topic {topic} has schema {actual!r}, expected {expected_schema!r}")


def _decode_mcap_string(data: bytes) -> str:
    reader = _CdrReader(data)
    return reader.string()


def _decode_mcap_image(data: bytes) -> _McapImage:
    reader = _CdrReader(data)
    seconds = reader.i32()
    nanoseconds = reader.u32()
    frame_id = reader.string()
    height = reader.u32()
    width = reader.u32()
    encoding = reader.string()
    reader.u8()
    reader.align(4)
    step = reader.u32()
    pixels = reader.bytes()
    if reader.remaining:
        raise ValueError("unexpected trailing bytes in MCAP Image CDR payload")
    return _McapImage(
        header=type("Header", (), {
            "stamp": type("Time", (), {"sec": seconds, "nanosec": nanoseconds})(),
            "frame_id": frame_id,
        })(),
        height=height,
        width=width,
        step=step,
        encoding=encoding,
        data=pixels,
    )


def _decode_mcap_point_cloud2(data: bytes) -> _McapPointCloud:
    """Decode this project's compact XYZ-intensity PointCloud2 encoding."""
    reader = _CdrReader(data)
    seconds, nanoseconds, frame_id = reader.i32(), reader.u32(), reader.string()
    height, width, field_count = reader.u32(), reader.u32(), reader.u32()
    fields = []
    for _ in range(field_count):
        fields.append((reader.string(), reader.u32(), reader.u8(), reader.u32()))
    is_bigendian = reader.u8()
    reader.align(4)
    point_step, row_step = reader.u32(), reader.u32()
    raw = reader.bytes()
    is_dense = reader.u8()
    if reader.remaining or height != 1 or is_bigendian or point_step != 16 or row_step != width * 16 or is_dense:
        raise ValueError("unsupported PointCloud2 layout")
    if fields != [("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1), ("intensity", 12, 7, 1)]:
        raise ValueError("PointCloud2 must contain packed float32 x/y/z/intensity")
    points = np.frombuffer(raw, dtype="<f4").reshape(width, 4).copy()
    return _McapPointCloud(
        header=type("Header", (), {"stamp": type("Time", (), {"sec": seconds, "nanosec": nanoseconds})(), "frame_id": frame_id})(),
        points=points,
    )


class _CdrReader:
    """Minimal little-endian CDR reader for our generated Image/String schemas."""

    def __init__(self, data: bytes) -> None:
        if data[:4] != b"\x00\x01\x00\x00":
            raise ValueError("only little-endian CDR MCAP records are supported")
        self.data = data
        self.offset = 4

    @property
    def remaining(self) -> int:
        return len(self.data) - self.offset

    def align(self, alignment: int) -> None:
        self.offset = (self.offset + alignment - 1) & -alignment
        if self.offset > len(self.data):
            raise ValueError("truncated CDR record")

    def i32(self) -> int:
        import struct
        self.align(4)
        if self.offset + 4 > len(self.data):
            raise ValueError("truncated CDR int32")
        value = struct.unpack_from("<i", self.data, self.offset)[0]
        self.offset += 4
        return value

    def u32(self) -> int:
        import struct
        self.align(4)
        if self.offset + 4 > len(self.data):
            raise ValueError("truncated CDR uint32")
        value = struct.unpack_from("<I", self.data, self.offset)[0]
        self.offset += 4
        return value

    def u8(self) -> int:
        if self.offset >= len(self.data):
            raise ValueError("truncated CDR uint8")
        value = self.data[self.offset]
        self.offset += 1
        return value

    def bytes(self) -> bytes:
        length = self.u32()
        if self.offset + length > len(self.data):
            raise ValueError("truncated CDR byte array")
        value = self.data[self.offset:self.offset + length]
        self.offset += length
        return value

    def string(self) -> str:
        value = self.bytes()
        if not value or value[-1] != 0:
            raise ValueError("CDR string is missing its NUL terminator")
        try:
            return value[:-1].decode("utf-8")
        except UnicodeDecodeError as error:
            raise ValueError("invalid UTF-8 CDR string") from error


class _LocalPosePublisher:
    """Compatibility sink for the local-only pair used by legacy tests/tools."""

    def send(self, message: FrameObjectDetections | EgoPoseSample, *, topic: str | None = None) -> None:
        del message, topic


class FrameRouterService:
    """Own the blocking ROUTER loop separately from ROS callback execution."""

    def __init__(self, session: VehicleFrameSession | VehicleAssetSession) -> None:
        self.session = session
        self._stop = Event()
        self._thread: Thread | None = None
        self._failure: BaseException | None = None

    def start(self) -> None:
        if self._thread is None:
            self._thread = Thread(target=self._run, name="vehicle-frame-router", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def raise_if_failed(self) -> None:
        if self._failure is not None:
            raise RuntimeError("vehicle frame router failed") from self._failure

    def _run(self) -> None:
        self.session.router.socket.setsockopt(zmq.RCVTIMEO, 100)
        while not self._stop.is_set():
            try:
                self.session.serve_once()
            except zmq.Again:
                continue
            except BaseException as error:
                self._failure = error
                self._stop.set()
                return


@dataclass(frozen=True)
class RosFrameAdapterConfig:
    """Validation policy for metadata replayed by the pose adapter."""

    expected_scene_generation: str | None = None
    expected_samples: int | None = None
    bbox_source: str = "gt"
    gaussian_T_nuscenes_global: np.ndarray | None = None
    gaussian_T_cooperscene: np.ndarray | None = None
    stored_gaussian_T_cooperscene: np.ndarray | None = None

    def __post_init__(self) -> None:
        if self.expected_scene_generation == "":
            raise ValueError("expected_scene_generation must be non-empty when provided")
        if self.expected_samples is not None and self.expected_samples <= 0:
            raise ValueError("expected_samples must be positive when provided")
        if self.bbox_source not in {"gt", "bevfusion"}:
            raise ValueError("bbox_source must be gt or bevfusion")
        if self.gaussian_T_nuscenes_global is not None:
            object.__setattr__(
                self,
                "gaussian_T_nuscenes_global",
                _rigid_matrix(self.gaussian_T_nuscenes_global, "gaussian_T_nuscenes_global"),
            )
        if (self.gaussian_T_cooperscene is None) != (
            self.stored_gaussian_T_cooperscene is None
        ):
            raise ValueError(
                "CooperScene relocalization requires both target and stored alignments"
            )
        for name in ("gaussian_T_cooperscene", "stored_gaussian_T_cooperscene"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, _rigid_matrix(value, name))


class RosFrameAdapter:
    """Convert ordered ``std_msgs/String`` frame metadata to ego-pose samples."""

    def __init__(
        self,
        publisher: PosePublisher,
        config: RosFrameAdapterConfig | None = None,
        *,
        metrics: JsonlMetrics | None = None,
        snapshot_sink: SnapshotSink | None = None,
        analysis_assembler: RosAnalysisAssembler | None = None,
        bevfusion: LatestBevFusionBoxes | None = None,
        localization_metrics: Any = None,
        localization_reference_transform: np.ndarray | None = None,
    ) -> None:
        if (localization_metrics is None) != (localization_reference_transform is None):
            raise ValueError("localization metrics require a reference transform")
        self.localization_metrics = localization_metrics
        self.localization_reference_transform = (
            None if localization_reference_transform is None
            else _rigid_matrix(localization_reference_transform, "localization reference transform")
        )
        self.publisher = publisher
        self.config = config or RosFrameAdapterConfig()
        self.metrics = metrics or JsonlMetrics(None)
        self.snapshot_sink = snapshot_sink
        self.analysis_assembler = analysis_assembler
        if self.config.bbox_source == "bevfusion" and bevfusion is None:
            raise ValueError("bevfusion bbox source requires a detector client")
        self.bevfusion = bevfusion
        self._next_sequence = 0
        self._scene_generation = self.config.expected_scene_generation

    @property
    def scene_generation(self) -> str | None:
        return self._scene_generation

    def receive_metadata(
        self, message: str | Mapping[str, Any] | Any, *, arrival_ns: int | None = None,
        wait_for_bevfusion: bool = False,
    ) -> EgoPoseSample:
        """Publish one sample in ROS callback order.

        Image bytes and intrinsics remain vehicle-local. The optional exact
        camera pose is retained for datasets whose camera is not fixed to ego.
        """
        arrival_ns = time.monotonic_ns() if arrival_ns is None else arrival_ns
        payload = _metadata_payload(message)
        original_payload = payload
        if self.config.gaussian_T_nuscenes_global is not None:
            payload = _relocalize_nuscenes_metadata(
                payload, self.config.gaussian_T_nuscenes_global
            )
        if self.config.gaussian_T_cooperscene is not None:
            assert self.config.stored_gaussian_T_cooperscene is not None
            payload = _relocalize_cooperscene_metadata(
                payload,
                self.config.gaussian_T_cooperscene,
                self.config.stored_gaussian_T_cooperscene,
            )
        generation = _non_empty_string(payload.get("scene_generation"), "scene_generation")
        if self._scene_generation is None:
            self._scene_generation = generation
        elif generation != self._scene_generation:
            raise ValueError(
                "metadata scene_generation does not match adapter scene generation: "
                f"{generation!r} != {self._scene_generation!r}"
            )
        callback_completed_ns = time.monotonic_ns()
        common = {
            "sequence": self._next_sequence,
            "timestamp_us": _non_negative_int(payload.get("timestamp_us"), "timestamp_us"),
            "scene_generation": generation,
            "frame_id": _non_empty_string(payload.get("frame_id"), "frame_id"),
        }
        if self.localization_metrics is not None:
            stored = original_payload.get("gaussian_T_nuscenes_global", original_payload.get("map_T_nuscenes_global"))
            if generation.startswith("cooperscene:"):
                stored = self.config.stored_gaussian_T_cooperscene
                if stored is None:
                    stored = original_payload.get("gaussian_T_cooperscene", np.eye(4))
            if stored is None:
                raise ValueError("localization metrics require the stored dataset-to-map alignment")
            original_pose = _rigid_matrix(original_payload.get("map_T_lidar", original_payload.get("world_T_ego")), "original map_T_lidar")
            reference_pose = self.localization_reference_transform @ np.linalg.inv(_rigid_matrix(stored, "stored alignment")) @ original_pose
            self.localization_metrics.record(
                sequence=common["sequence"], timestamp_us=common["timestamp_us"], frame_id=common["frame_id"],
                estimated_pose=_rigid_matrix(payload.get("map_T_lidar", payload.get("world_T_ego")), "estimated map_T_lidar"),
                reference_pose=reference_pose,
                map_R_dataset=self.localization_reference_transform[:3, :3],
            )
        image_shape = None
        # Validate the capture poses before submitting any deterministic offline work.
        if wait_for_bevfusion:
            if self.bevfusion is None or self.config.bbox_source != "bevfusion":
                raise ValueError("offline detector wait requires BEVFusion")
            _rigid_matrix(payload.get("map_T_lidar", payload.get("world_T_ego")), "map_T_lidar")
            _rigid_matrix(payload.get("world_T_camera"), "world_T_camera")
            image_shape = self.bevfusion.receive_metadata(payload, self._next_sequence)
            if image_shape is None:
                raise ValueError("offline detector wait requires a joined image")
            self.bevfusion.wait_for_sequence(self._next_sequence)
        boxes = (
            self.bevfusion.boxes_for(dict(payload, **common), image_shape=image_shape)
            if self.bevfusion is not None else _box_prompts(payload.get("boxes", ()))
        )
        detections = FrameObjectDetections(**common, boxes=boxes)
        pose = EgoPoseSample(**common, world_T_ego=payload.get("world_T_ego"))
        snapshot = FrameSnapshot(
            **common,
            world_T_ego=pose.world_T_ego,
            world_T_camera=payload.get("world_T_camera"),
            boxes=detections.boxes,
        )
        # The spool is the durability boundary: remote delivery is allowed only
        # after this append returns successfully.
        if self.snapshot_sink is not None:
            self.snapshot_sink.append(snapshot)
        analysis_payload = dict(payload)
        analysis_payload["boxes"] = [box.to_dict() for box in boxes]
        if self.analysis_assembler is not None:
            self.analysis_assembler.receive_metadata(analysis_payload, self._next_sequence)
        if self.bevfusion is not None and not wait_for_bevfusion:
            self.bevfusion.receive_metadata(payload, self._next_sequence)
        self.publisher.send(detections)
        self.publisher.send(pose)
        published_ns = time.monotonic_ns()
        self.metrics.write(
            "ros_frame_snapshot_spooled",
            {
                "sequence": pose.sequence,
                "frame_id": pose.frame_id,
                "scene_generation": pose.scene_generation,
                "source_timestamp_us": pose.timestamp_us,
                "recorded_source_sequence": payload.get("recorded_source_sequence"),
                "recorded_timestamp_us": payload.get("recorded_timestamp_us"),
                "source_timestamp_ns": payload.get("source_timestamp_ns"),
                "object_detection_count": len(detections.boxes),
                "transport_contract": "durable_frame_snapshot",
                "snapshot_durably_spooled": self.snapshot_sink is not None,
                "metadata_callback_arrival_ns": arrival_ns,
                "callback_completed_monotonic_ns": callback_completed_ns,
                "publication_monotonic_ns": published_ns,
                "callback_to_publication_ms": (published_ns - arrival_ns) / 1_000_000.0,
                **(
                    {
                        "bbox_source": "bevfusion",
                        "bevfusion_result_sequence": self.bevfusion.latest_sequence,
                        "bevfusion_result_age_frames": (
                            None if self.bevfusion.latest_sequence is None
                            else pose.sequence - self.bevfusion.latest_sequence
                        ),
                        "bevfusion_submitted_count": self.bevfusion.submitted_count,
                        "bevfusion_result_count": self.bevfusion.result_count,
                        "bevfusion_replaced_pending_count": self.bevfusion.replaced_pending_count,
                        "bevfusion_error_count": self.bevfusion.error_count,
                        **({} if self.bevfusion.appearance is None else {
                            "vehicle_appearance_update_ms": self.bevfusion.appearance.last_update_ms,
                            "vehicle_shape_fit_count": self.bevfusion.appearance.fit_count,
                            "vehicle_shape_fit_error_count": self.bevfusion.appearance.fit_error_count,
                        }),
                    }
                    if self.bevfusion is not None else {"bbox_source": "gt"}
                ),
            },
        )
        self._next_sequence += 1
        if self.config.expected_samples is not None and self._next_sequence == self.config.expected_samples:
            finalize = getattr(self.snapshot_sink, "finalize", None)
            if finalize is None:
                raise RuntimeError("expected_samples requires a durable frame spool")
            finalize(FrameEnd(generation, snapshot.sequence, snapshot.timestamp_us))
        elif self.config.expected_samples is not None and self._next_sequence > self.config.expected_samples:
            raise RuntimeError("received more metadata samples than expected_samples")
        return pose


def _metadata_payload(message: str | Mapping[str, Any] | Any) -> Mapping[str, Any]:
    raw = message.data if hasattr(message, "data") else message
    if isinstance(raw, str):
        payload = json.loads(raw)
    elif isinstance(raw, Mapping):
        payload = dict(raw)
    else:
        raise TypeError("metadata must be a JSON string, mapping, or ROS String message")
    if not isinstance(payload, Mapping):
        raise ValueError("metadata JSON must contain an object")
    if payload.get("version") != 1:
        raise ValueError("metadata version must be 1")
    return payload


def _load_gaussian_T_nuscenes_global(
    path: Path, *, expected_scene_generation: str | None = None
) -> np.ndarray:
    """Load the fixed NuScenes-global to Gaussian-map localization result."""

    if path.suffix == ".json":
        payload = json.loads(path.read_text(encoding="utf-8"))
        value = payload.get("gaussian_T_nuscenes_global")
        scene_id = payload.get("scene_id")
    elif path.suffix == ".npz":
        with np.load(path) as archive:
            value = archive["gaussian_T_nuscenes_global"] if "gaussian_T_nuscenes_global" in archive else None
            scene_id = str(archive["scene_id"]) if "scene_id" in archive else None
    else:
        raise ValueError("localization_transform must end in .json or .npz")
    if value is None:
        raise ValueError(
            f"localization transform {path} must contain gaussian_T_nuscenes_global"
        )
    if (
        scene_id
        and expected_scene_generation is not None
        and f"nuscenes:{scene_id}" != expected_scene_generation
    ):
        raise ValueError(
            "localization transform scene does not match --scene-generation: "
            f"nuscenes:{scene_id} != {expected_scene_generation}"
        )
    return _rigid_matrix(value, f"gaussian_T_nuscenes_global in {path}")


def _load_gaussian_T_cooperscene(path: Path) -> np.ndarray:
    """Load a CooperScene-global to Gaussian-map localization result."""

    if path.suffix == ".json":
        value = json.loads(path.read_text(encoding="utf-8")).get(
            "gaussian_T_cooperscene"
        )
    elif path.suffix == ".npz":
        with np.load(path) as archive:
            value = archive["gaussian_T_cooperscene"] if "gaussian_T_cooperscene" in archive else None
    else:
        raise ValueError("localization_transform must end in .json or .npz")
    if value is None:
        raise ValueError(
            f"localization transform {path} must contain gaussian_T_cooperscene"
        )
    return _rigid_matrix(value, f"gaussian_T_cooperscene in {path}")


def _relocalize_cooperscene_metadata(
    payload: Mapping[str, Any],
    gaussian_T_cooperscene: np.ndarray,
    stored_gaussian_T_cooperscene: np.ndarray,
) -> dict[str, Any]:
    """Rebase CooperScene metadata from its baked map into a new Gaussian map."""

    map_rebase = gaussian_T_cooperscene @ np.linalg.inv(stored_gaussian_T_cooperscene)
    result = dict(payload)
    for name in ("world_T_camera", "world_T_ego", "map_T_lidar"):
        if name in result:
            result[name] = tuple(float(value) for value in (
                map_rebase @ _rigid_matrix(result[name], name)
            ).reshape(-1))
    rebased_boxes = []
    for box in result.get("boxes", ()):
        rebased = dict(box)
        if "world_T_object" in rebased:
            rebased["world_T_object"] = tuple(float(value) for value in (
                map_rebase @ _rigid_matrix(rebased["world_T_object"], "world_T_object")
            ).reshape(-1))
        rebased_boxes.append(rebased)
    result["boxes"] = rebased_boxes
    result["gaussian_T_cooperscene"] = tuple(
        float(value) for value in gaussian_T_cooperscene.reshape(-1)
    )
    return result


def _relocalize_nuscenes_metadata(
    payload: Mapping[str, Any], gaussian_T_nuscenes_global: np.ndarray
) -> dict[str, Any]:
    """Rebase a legacy NuScenes MCAP from its stored map frame to a localized one."""

    old_alignment = payload.get(
        "gaussian_T_nuscenes_global", payload.get("map_T_nuscenes_global")
    )
    if old_alignment is None:
        raise ValueError(
            "NuScenes localization requires MCAP metadata with map_T_nuscenes_global"
        )
    old_gaussian_T_nuscenes_global = _rigid_matrix(
        old_alignment, "stored gaussian_T_nuscenes_global"
    )
    map_rebase = gaussian_T_nuscenes_global @ np.linalg.inv(old_gaussian_T_nuscenes_global)

    result = dict(payload)
    for name in ("world_T_camera", "world_T_ego", "map_T_lidar"):
        if name in result:
            result[name] = tuple(float(value) for value in (
                map_rebase @ _rigid_matrix(result[name], name)
            ).reshape(-1))
    rebased_boxes = []
    for box in result.get("boxes", ()):
        rebased = dict(box)
        if "world_T_object" in rebased:
            rebased["world_T_object"] = tuple(float(value) for value in (
                map_rebase @ _rigid_matrix(rebased["world_T_object"], "world_T_object")
            ).reshape(-1))
        rebased_boxes.append(rebased)
    result["boxes"] = rebased_boxes
    alignment = tuple(float(value) for value in gaussian_T_nuscenes_global.reshape(-1))
    result["gaussian_T_nuscenes_global"] = alignment
    # Preserve the old field as an alias for existing MCAP consumers.
    result["map_T_nuscenes_global"] = alignment
    return result


def _rigid_matrix(value: Any, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.size != 16 or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must contain a finite 4x4 transform")
    matrix = matrix.reshape(4, 4)
    if not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1e-7):
        raise ValueError(f"{name} must be homogeneous")
    return matrix


def _non_empty_string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be a non-empty string")
    return value


def _non_negative_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _timestamp_us(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000 + int(stamp.nanosec) // 1_000


def _result_value(value: Any, name: str, default: Any = None) -> Any:
    return value.get(name, default) if isinstance(value, Mapping) else getattr(value, name, default)


def _bevfusion_world_boxes(result: Any, world_T_lidar: np.ndarray, ground_snapper: LidarGroundSnapper | None = None,
                           lidar_points: np.ndarray | None = None) -> tuple[tuple[np.ndarray, tuple[float, float, float], str, float], ...]:
    """Normalize a detector result whose boxes are expressed in LiDAR coordinates.

    With ``ground_snapper`` and the request's ``lidar_points``, box heights are first
    snapped onto the local LiDAR ground (in the LiDAR frame).
    """
    raw_boxes = _result_value(result, "boxes", ())
    normalized = []
    for box in raw_boxes:
        item = (lambda name, default=None: box.get(name, default)) if isinstance(box, Mapping) else (lambda name, default=None: getattr(box, name, default))
        lidar_T_object = np.eye(4, dtype=np.float64)
        lidar_T_object[:3, 3] = np.asarray(item("center_xyz"), dtype=np.float64)
        yaw = float(item("yaw_rad", 0.0))
        lidar_T_object[:3, :3] = ((np.cos(yaw), -np.sin(yaw), 0.0), (np.sin(yaw), np.cos(yaw), 0.0), (0.0, 0.0, 1.0))
        dimensions = tuple(float(value) for value in item("dimensions_lwh"))
        if len(dimensions) != 3 or min(dimensions) <= 0:
            raise ValueError("BEVFusion dimensions_lwh must be positive")
        label, score = str(item("class_name", item("label", ""))), float(item("score", 0.0))
        if not np.isfinite(score) or not 0 <= score <= 1:
            raise ValueError("BEVFusion score must be between zero and one")
        normalized.append((lidar_T_object, dimensions, label, score))
    if ground_snapper is not None and lidar_points is not None and normalized:
        poses = np.stack([item[0] for item in normalized])
        shifts = ground_snapper.offsets(lidar_points, poses[:, :3, 3], np.arctan2(poses[:, 1, 0], poses[:, 0, 0]),
                                        np.array([item[1] for item in normalized]))
        for (lidar_T_object, *_), shift in zip(normalized, shifts):
            lidar_T_object[2, 3] += shift
    return tuple((world_T_lidar @ lidar_T_object, *rest) for lidar_T_object, *rest in normalized)


def _box_prompts(value: Any) -> tuple[BoxPrompt, ...]:
    if not isinstance(value, (list, tuple)):
        raise ValueError("boxes must be an array")
    try:
        return tuple(BoxPrompt.from_dict(item) for item in value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"invalid boxes: {error}") from error


def _ros_image_to_bgr(message: Any) -> np.ndarray:
    """Decode supported ROS Image layouts while honoring per-row padding."""

    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    if height <= 0 or width <= 0 or step <= 0:
        raise ValueError("image height, width, and step must be positive")
    encoding = str(message.encoding).lower()
    channels = {"bgr8": 3, "rgb8": 3, "mono8": 1}.get(encoding)
    if channels is None:
        raise ValueError(f"unsupported ROS image encoding: {message.encoding!r}")
    packed_step = width * channels
    if step < packed_step:
        raise ValueError("ROS image step is smaller than its packed row width")
    data = np.frombuffer(bytes(message.data), dtype=np.uint8)
    if data.size != height * step:
        raise ValueError("ROS image data length does not match height * step")
    packed = data.reshape(height, step)[:, :packed_step]
    if channels == 1:
        image = cv2.cvtColor(packed.reshape(height, width), cv2.COLOR_GRAY2BGR)
    else:
        image = packed.reshape(height, width, channels)
        if encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return np.ascontiguousarray(image)


def _create_ros_node(
    adapter: RosFrameAdapter,
    *,
    metadata_topic: str,
    image_topic: str,
    queue_depth: int,
) -> Any:
    try:
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
    except ImportError as error:  # pragma: no cover - requires a sourced ROS environment
        raise RuntimeError(
            "ROS pose replay requires a sourced ROS 2 environment with rclpy and std_msgs"
        ) from error

    class PoseAdapterNode(Node):
        def __init__(self) -> None:
            super().__init__("remote_teleop_pose_adapter")
            self.create_subscription(String, metadata_topic, self._on_metadata, queue_depth)
            if adapter.analysis_assembler is not None:
                self.create_subscription(Image, image_topic, self._on_image, queue_depth)

        def _on_metadata(self, message: Any) -> None:
            try:
                adapter.receive_metadata(message)
            except Exception as error:
                # A skipped source record would make the durable sequence lie.
                # Escalate to the supervisor instead of logging and continuing.
                self.get_logger().fatal(f"invalid replay metadata: {error}")
                raise RuntimeError("fatal frame metadata validation failure") from error

        def _on_image(self, message: Any) -> None:
            try:
                assert adapter.analysis_assembler is not None
                adapter.analysis_assembler.receive_image(message)
                if adapter.bevfusion is not None:
                    adapter.bevfusion.receive_image(message)
            except Exception as error:
                self.get_logger().fatal(f"invalid replay image: {error}")
                raise RuntimeError("fatal frame image validation failure") from error

    return PoseAdapterNode()


def _vehicle_appearance(*, shape: bool, color: bool, background: bool = True) -> VehicleAppearanceEstimator | None:
    """Build the optional shape/color hint estimator only when a feature is enabled.

    ``background`` moves shape fits off the publishing thread; deterministic
    offline replay keeps them synchronous.
    """
    if not (shape or color):
        return None
    from src.deployment.vehicle_appearance import VehicleAppearanceConfig, VehicleAppearanceEstimator
    return VehicleAppearanceEstimator(VehicleAppearanceConfig(shape=shape, color=color, fit_in_background=background))


def _open_bevfusion_client(endpoint: str | None) -> BevFusionClient:
    """Keep the optional Docker transport outside the normal replay import path."""
    if not endpoint:
        raise ValueError("BEVFusion endpoint is required")
    try:
        from src.realtime.bevfusion import BevFusionClient
    except ImportError as error:  # pragma: no cover - optional service deployment
        raise RuntimeError("BEVFusion client support is not installed") from error
    context = zmq.Context.instance()
    return BevFusionClient.open(context, endpoint, identity=f"teleop-adapter-{time.monotonic_ns()}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metadata-topic", default="/camera/frame_detections")
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--lidar-topic", default="/lidar/points")
    parser.add_argument("--mcap", type=Path)
    parser.add_argument("--mcap-await-detections", action="store_true", help="Offline Foxglove/XYZ replay: wait for each current capture before publishing")
    parser.add_argument("--xyz-pose-sidecar", type=Path, help="Offline-only verified per-frame XYZ poses and sensor identity")
    parser.add_argument("--mcap-rate", type=float, default=1.0)
    parser.add_argument("--mcap-agent", default="1", help="Camera/LiDAR agent in Foxglove MCAPs")
    parser.add_argument("--mcap-start-sequence", type=int, default=0)
    parser.add_argument("--mcap-camera-calibration", choices=CALIBRATIONS, default=DEFAULT_CALIBRATION,
                        help="CooperScene camera calibration for replay projection/rendering")
    parser.add_argument("--localization-transform", type=Path)
    parser.add_argument("--stored-localization-transform", type=Path)
    parser.add_argument("--bbox-source", choices=("gt", "bevfusion"), default="gt")
    parser.add_argument("--bevfusion-endpoint")
    parser.add_argument("--bevfusion-score-threshold", type=float, default=0.2)
    parser.add_argument("--bevfusion-minimum-area-px", type=float, default=64.0)
    parser.add_argument("--bevfusion-camera-calibration", choices=(*CALIBRATIONS, "metadata"),
                        default="baseline", help="CooperScene camera sent to BEVFusion (checkpoint trained "
                        "with baseline); 'metadata' sends the replay camera unchanged")
    parser.add_argument("--bevfusion-continuation-score-threshold", type=float,
                        help="Minimum score for confirmed-track updates; defaults to the birth score")
    parser.add_argument("--bevfusion-minimum-confirmation-hits", type=int, default=1)
    parser.add_argument("--bevfusion-duplicate-birth-guard", action="store_true")
    parser.add_argument("--bevfusion-maximum-reacquisition-seconds", type=float, default=0.0,
                        help="Reuse identities after unambiguous short gaps; zero disables reacquisition")
    parser.add_argument("--bevfusion-maximum-coast-seconds", type=float, default=0.5,
                        help="Drop tracks this long after their last associated detection")
    parser.add_argument("--bevfusion-lidar-ground-snap", action=argparse.BooleanOptionalAction, default=True,
                        help="Shift detection heights onto the local LiDAR ground before tracking (default: on)")
    parser.add_argument("--vehicle-shape-fit", action=argparse.BooleanOptionalAction, default=False,
                        help="Fit per-track car shape hints from accumulated LiDAR (BEVFusion only; default: off)")
    parser.add_argument("--vehicle-color", action=argparse.BooleanOptionalAction, default=False,
                        help="Estimate per-track paint color from LiDAR-supported image samples "
                        "(BEVFusion only; default: off)")
    parser.add_argument("--bevfusion-tracks-path", type=Path,
                        help="Optional JSONL of all live tracker boxes per published frame (offline evaluation)")
    parser.add_argument("--frames-endpoint", default="tcp://0.0.0.0:5555")
    parser.add_argument("--queue-depth", type=int, default=2_048)
    parser.add_argument("--high-water-mark", type=int, default=2_048)
    parser.add_argument("--scene-generation")
    parser.add_argument("--expected-samples", type=int)
    parser.add_argument("--metrics-path", type=Path)
    parser.add_argument("--localization-reference-transform", type=Path)
    parser.add_argument("--localization-reference-kind", choices=("shared_enu_prior", "recorded_pose_alignment_baseline"))
    parser.add_argument("--localization-metrics-path", type=Path)
    parser.add_argument("--localization-position-mode", choices=("xyz", "xy"), default="xyz")

    parser.add_argument("--spool-root", type=Path, default=Path("artifacts/vehicle_frame_spool"))
    parser.add_argument("--assets-endpoint")
    parser.add_argument("--asset-store-root", type=Path, default=Path("artifacts/vehicle_assets"))
    parser.add_argument("--analysis-endpoint")
    parser.add_argument(
        "--analysis-spool-root", type=Path, default=Path("artifacts/vehicle_analysis")
    )
    args = parser.parse_args()
    if not np.isfinite(args.bevfusion_minimum_area_px) or args.bevfusion_minimum_area_px <= 0:
        parser.error("--bevfusion-minimum-area-px must be finite and positive")
    if args.mcap_await_detections and (args.mcap is None or args.bbox_source != "bevfusion"):
        parser.error("--mcap-await-detections requires MCAP and BEVFusion")
    if args.xyz_pose_sidecar is not None:
        if args.mcap is None or any((args.localization_transform, args.stored_localization_transform,
                args.localization_metrics_path, args.localization_reference_transform)):
            parser.error("--xyz-pose-sidecar requires offline MCAP without static relocalization/metrics")
        from src.data.xyz_pose_sidecar import VerifiedXYZPoseSidecar
        xyz_pose_sidecar = VerifiedXYZPoseSidecar(args.xyz_pose_sidecar)
    else:
        xyz_pose_sidecar = None
    if args.queue_depth <= 0:
        parser.error("--queue-depth must be positive")
    if args.high_water_mark <= 0:
        parser.error("--high-water-mark must be positive")
    if args.mcap_rate <= 0:
        parser.error("--mcap-rate must be positive")
    if args.mcap_start_sequence < 0 or (args.mcap_start_sequence and args.mcap is None):
        parser.error("--mcap-start-sequence requires MCAP playback and must be non-negative")
    if args.mcap is not None and not args.mcap.is_file():
        parser.error(f"--mcap does not exist: {args.mcap}")
    if args.localization_transform is not None and not args.localization_transform.is_file():
        parser.error(f"--localization-transform does not exist: {args.localization_transform}")
    if args.stored_localization_transform is not None and not args.stored_localization_transform.is_file():
        parser.error(
            f"--stored-localization-transform does not exist: {args.stored_localization_transform}"
        )
    if (args.localization_transform is None) != (args.stored_localization_transform is None):
        if args.scene_generation is None or args.scene_generation.startswith("cooperscene:"):
            parser.error(
                "CooperScene relocalization requires both --localization-transform and "
                "--stored-localization-transform"
            )
    if not 0 <= args.bevfusion_score_threshold <= 1:
        parser.error("--bevfusion-score-threshold must be between zero and one")
    if (args.bevfusion_continuation_score_threshold is not None
            and not 0 <= args.bevfusion_continuation_score_threshold <= args.bevfusion_score_threshold):
        parser.error("--bevfusion-continuation-score-threshold must be between zero and the birth score")
    if args.bevfusion_minimum_confirmation_hits < 1:
        parser.error("--bevfusion-minimum-confirmation-hits must be positive")
    if (not np.isfinite(args.bevfusion_maximum_reacquisition_seconds)
            or args.bevfusion_maximum_reacquisition_seconds < 0):
        parser.error("--bevfusion-maximum-reacquisition-seconds must be finite and non-negative")
    if not np.isfinite(args.bevfusion_maximum_coast_seconds) or args.bevfusion_maximum_coast_seconds <= 0:
        parser.error("--bevfusion-maximum-coast-seconds must be finite and positive")
    if args.bbox_source == "bevfusion" and not args.bevfusion_endpoint:
        parser.error("--bbox-source bevfusion requires --bevfusion-endpoint")
    if (args.vehicle_shape_fit or args.vehicle_color) and args.bbox_source != "bevfusion":
        parser.error("--vehicle-shape-fit/--vehicle-color require --bbox-source bevfusion")

    if args.mcap is not None:
        # A detached background shell may pass SIGINT as ignored. Restore it
        # so the launcher's normal shutdown reaches the cleanup below.
        signal.signal(signal.SIGINT, signal.default_int_handler)

    context = zmq.Context()
    router = RouterTransport.open(
        context, args.frames_endpoint, bind=True, high_water_mark=args.high_water_mark
    )
    spool = VehicleFrameSpool(args.spool_root)
    service = FrameRouterService(VehicleFrameSession(router, spool))
    assets_router = None
    assets_service = None
    if args.assets_endpoint:
        assets_router = RouterTransport.open(
            context, args.assets_endpoint, bind=True, high_water_mark=args.high_water_mark
        )
        assets_service = FrameRouterService(
            VehicleAssetSession(assets_router, VehicleAssetStore(args.asset_store_root))
        )
    analysis_router = None
    analysis_service = None
    analysis_assembler = None
    if args.analysis_endpoint:
        analysis_router = RouterTransport.open(
            context, args.analysis_endpoint, bind=True, high_water_mark=args.high_water_mark
        )
        analysis_spool = VehicleAnalysisSpool(args.analysis_spool_root)
        analysis_service = FrameRouterService(
            VehicleFrameSession(analysis_router, analysis_spool)
        )
        analysis_assembler = RosAnalysisAssembler(
            analysis_spool, expected_samples=args.expected_samples
        )
    bevfusion = None
    if args.bbox_source == "bevfusion":
        bevfusion = LatestBevFusionBoxes(
            _open_bevfusion_client(args.bevfusion_endpoint),
            score_threshold=args.bevfusion_score_threshold,
            minimum_area_px=args.bevfusion_minimum_area_px,
            continuation_score_threshold=args.bevfusion_continuation_score_threshold,
            minimum_confirmation_hits=args.bevfusion_minimum_confirmation_hits,
            suppress_duplicate_births=args.bevfusion_duplicate_birth_guard,
            maximum_reacquisition_seconds=args.bevfusion_maximum_reacquisition_seconds,
            maximum_coast_seconds=args.bevfusion_maximum_coast_seconds,
            request_calibration=(None if args.bevfusion_camera_calibration == "metadata"
                                 else args.bevfusion_camera_calibration),
            track_log=None if args.bevfusion_tracks_path is None else JsonlMetrics(args.bevfusion_tracks_path),
            ground_snapper=LidarGroundSnapper() if args.bevfusion_lidar_ground_snap else None,
            appearance=_vehicle_appearance(shape=args.vehicle_shape_fit, color=args.vehicle_color,
                                           background=not args.mcap_await_detections),
        )
    localization_metrics = None
    localization_reference = None
    if any((args.localization_reference_transform, args.localization_reference_kind, args.localization_metrics_path)):
        if not all((args.localization_reference_transform, args.localization_reference_kind, args.localization_metrics_path)):
            parser.error("localization metrics require reference transform, reference kind, and output path")
        from src.localization.trajectory_metrics import TrajectoryMetrics
        reference_payload = json.loads(args.localization_reference_transform.read_text())
        for key in ("reference_T_dataset", "gaussian_T_cooperscene", "gaussian_T_nuscenes_global", "map_T_nuscenes_global"):
            if key in reference_payload:
                localization_reference = _rigid_matrix(reference_payload[key], key)
                break
        if localization_reference is None:
            parser.error("reference transform must contain reference_T_dataset or a dataset alignment")
        localization_metrics = TrajectoryMetrics(args.localization_metrics_path,
            reference_kind=args.localization_reference_kind, expected_samples=args.expected_samples,
            position_mode=args.localization_position_mode)
    adapter = RosFrameAdapter(
        _LocalPosePublisher(),
        RosFrameAdapterConfig(
            expected_scene_generation=args.scene_generation, expected_samples=args.expected_samples,
            bbox_source=args.bbox_source,
            gaussian_T_nuscenes_global=(
                None if args.localization_transform is None or not args.scene_generation.startswith("nuscenes:")
                else _load_gaussian_T_nuscenes_global(
                    args.localization_transform,
                    expected_scene_generation=args.scene_generation,
                )
            ),
            gaussian_T_cooperscene=(
                _load_gaussian_T_cooperscene(args.localization_transform)
                if args.localization_transform is not None
                and args.scene_generation.startswith("cooperscene:") else None
            ),
            stored_gaussian_T_cooperscene=(
                _load_gaussian_T_cooperscene(args.stored_localization_transform)
                if args.stored_localization_transform is not None else None
            ),
        ),
        metrics=JsonlMetrics(args.metrics_path),
        snapshot_sink=spool,
        analysis_assembler=analysis_assembler,
        bevfusion=bevfusion,
        localization_metrics=localization_metrics,
        localization_reference_transform=localization_reference,
    )
    rclpy = None
    node = None
    if args.mcap is None:
        try:
            import rclpy as imported_rclpy
        except ImportError as error:  # pragma: no cover - requires a sourced ROS environment
            raise RuntimeError("source ROS 2 before starting the pose adapter") from error
        rclpy = imported_rclpy
        rclpy.init()
        node = _create_ros_node(
            adapter,
            metadata_topic=args.metadata_topic,
            image_topic=args.image_topic,
            queue_depth=args.queue_depth,
        )
    try:
        service.start()
        if assets_service is not None:
            assets_service.start()
        if analysis_service is not None:
            analysis_service.start()
        if args.mcap is not None:
            if analysis_assembler is None:
                raise RuntimeError("direct MCAP replay requires --analysis-endpoint")
            McapFrameReplay(
                adapter,
                image_topic=args.image_topic,
                metadata_topic=args.metadata_topic,
                lidar_topic=args.lidar_topic if args.bbox_source == "bevfusion" else None,
                expected_samples=args.expected_samples,
                rate=args.mcap_rate,
                agent=args.mcap_agent,
                start_sequence=args.mcap_start_sequence,
                xyz_pose_sidecar=xyz_pose_sidecar,
                await_detections=args.mcap_await_detections,
                camera_calibration=args.mcap_camera_calibration,
            ).replay(args.mcap)
        while rclpy is None or rclpy.ok():
            if bevfusion is not None:
                bevfusion.poll()
            if rclpy is not None:
                assert node is not None
                rclpy.spin_once(node, timeout_sec=0.1)
            else:
                time.sleep(0.1)
            service.raise_if_failed()
            if assets_service is not None:
                assets_service.raise_if_failed()
            if analysis_service is not None:
                analysis_service.raise_if_failed()
            if analysis_assembler is not None:
                analysis_assembler.raise_if_failed()
    finally:
        if localization_metrics is not None:
            localization_metrics.finish()
        if analysis_assembler is not None:
            analysis_assembler.close()
        if bevfusion is not None:
            bevfusion.close()
        service.stop()
        if assets_service is not None:
            assets_service.stop()
        if analysis_service is not None:
            analysis_service.stop()
        if node is not None:
            node.destroy_node()
        if rclpy is not None:
            rclpy.shutdown()
        router.close()
        if assets_router is not None:
            assets_router.close()
        if analysis_router is not None:
            analysis_router.close()
        context.term()


if __name__ == "__main__":
    main()

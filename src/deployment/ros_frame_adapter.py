"""Durably spool remote render state and vehicle-local SAM analysis frames.

The replay MCAP may still contain camera images for the dedicated video path,
The remote stream remains image-free. When configured, camera images are
joined locally with metadata and written to a separate receiver-pulled spool.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
from queue import Empty, Queue
import time
from threading import Event, Lock, Thread
from typing import Any, Mapping, Protocol

import cv2
import numpy as np
import zmq

from src.realtime.process_support import JsonlMetrics
from src.realtime.protocol import BoxPrompt, EgoPoseSample, FrameDetections, FrameEnd, FrameObjectDetections, FrameSnapshot
from src.realtime.transport import RouterTransport
from src.deployment.vehicle import VehicleAnalysisSpool, VehicleAssetSession, VehicleAssetStore, VehicleFrameSession, VehicleFrameSpool


class PosePublisher(Protocol):
    def send(
        self, message: FrameObjectDetections | EgoPoseSample, *, topic: str | None = None
    ) -> None: ...


class SnapshotSink(Protocol):
    def append(self, snapshot: FrameSnapshot) -> None: ...


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


class McapFrameReplay:
    """Replay the two known CooperScene MCAP records without ROS/DDS delivery.

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
        expected_samples: int | None,
        rate: float = 1.0,
        sleep: Any = time.sleep,
        reader_factory: Any = None,
    ) -> None:
        if rate <= 0:
            raise ValueError("MCAP replay rate must be positive")
        if not image_topic.startswith("/") or not metadata_topic.startswith("/"):
            raise ValueError("MCAP topics must be absolute")
        if image_topic == metadata_topic:
            raise ValueError("MCAP image and metadata topics must differ")
        if adapter.analysis_assembler is None:
            raise ValueError("direct MCAP replay requires an analysis assembler")
        self.adapter = adapter
        self.image_topic = image_topic
        self.metadata_topic = metadata_topic
        self.expected_samples = expected_samples
        self.rate = rate
        self.sleep = sleep
        self.reader_factory = reader_factory or _mcap_reader

    def replay(self, path: Path) -> None:
        metadata_count = 0
        image_count = 0
        previous_log_time: int | None = None
        with path.open("rb") as stream:
            reader = self.reader_factory(stream)
            for schema, channel, message in reader.iter_messages(
                topics=(self.image_topic, self.metadata_topic), log_time_order=False
            ):
                _validate_mcap_record(
                    channel.topic,
                    channel,
                    schema,
                    image_topic=self.image_topic,
                    metadata_topic=self.metadata_topic,
                )
                if previous_log_time is not None:
                    delta_ns = message.log_time - previous_log_time
                    if delta_ns < 0:
                        raise ValueError("MCAP source records are not in nondecreasing log-time order")
                    if delta_ns:
                        self.sleep(delta_ns / 1_000_000_000.0 / self.rate)
                previous_log_time = message.log_time
                if channel.topic == self.image_topic:
                    self.adapter.analysis_assembler.receive_image(_decode_mcap_image(message.data))
                    image_count += 1
                elif channel.topic == self.metadata_topic:
                    self.adapter.receive_metadata(_decode_mcap_string(message.data))
                    metadata_count += 1
        if self.expected_samples is not None:
            if metadata_count != self.expected_samples:
                raise RuntimeError(
                    f"expected {self.expected_samples} MCAP metadata records, got {metadata_count}"
                )
            if image_count != self.expected_samples:
                raise RuntimeError(
                    f"expected {self.expected_samples} MCAP image records, got {image_count}"
                )
        if self.adapter.analysis_assembler is not None:
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
) -> None:
    if topic == image_topic:
        expected_schema = "sensor_msgs/msg/Image"
    elif topic == metadata_topic:
        expected_schema = "std_msgs/msg/String"
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

    def __post_init__(self) -> None:
        if self.expected_scene_generation == "":
            raise ValueError("expected_scene_generation must be non-empty when provided")
        if self.expected_samples is not None and self.expected_samples <= 0:
            raise ValueError("expected_samples must be positive when provided")


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
    ) -> None:
        self.publisher = publisher
        self.config = config or RosFrameAdapterConfig()
        self.metrics = metrics or JsonlMetrics(None)
        self.snapshot_sink = snapshot_sink
        self.analysis_assembler = analysis_assembler
        self._next_sequence = 0
        self._scene_generation = self.config.expected_scene_generation

    @property
    def scene_generation(self) -> str | None:
        return self._scene_generation

    def receive_metadata(
        self, message: str | Mapping[str, Any] | Any, *, arrival_ns: int | None = None
    ) -> EgoPoseSample:
        """Publish one sample in ROS callback order.

        The existing MCAP metadata includes image and camera fields; they are
        deliberately ignored so the ZMQ wire message stays pose-only.
        """
        arrival_ns = time.monotonic_ns() if arrival_ns is None else arrival_ns
        payload = _metadata_payload(message)
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
        detections = FrameObjectDetections(
            **common, boxes=_box_prompts(payload.get("boxes", ()))
        )
        pose = EgoPoseSample(**common, world_T_ego=payload.get("world_T_ego"))
        snapshot = FrameSnapshot(**common, world_T_ego=pose.world_T_ego, boxes=detections.boxes)
        # The spool is the durability boundary: remote delivery is allowed only
        # after this append returns successfully.
        if self.snapshot_sink is not None:
            self.snapshot_sink.append(snapshot)
        if self.analysis_assembler is not None:
            self.analysis_assembler.receive_metadata(payload, self._next_sequence)
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
                "object_detection_count": len(detections.boxes),
                "transport_contract": "durable_frame_snapshot",
                "snapshot_durably_spooled": self.snapshot_sink is not None,
                "metadata_callback_arrival_ns": arrival_ns,
                "callback_completed_monotonic_ns": callback_completed_ns,
                "publication_monotonic_ns": published_ns,
                "callback_to_publication_ms": (published_ns - arrival_ns) / 1_000_000.0,
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


def _non_empty_string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be a non-empty string")
    return value


def _non_negative_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


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
            except Exception as error:
                self.get_logger().fatal(f"invalid replay image: {error}")
                raise RuntimeError("fatal frame image validation failure") from error

    return PoseAdapterNode()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metadata-topic", default="/camera/frame_detections")
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--mcap", type=Path)
    parser.add_argument("--mcap-rate", type=float, default=1.0)
    parser.add_argument("--frames-endpoint", default="tcp://0.0.0.0:5555")
    parser.add_argument("--queue-depth", type=int, default=2_048)
    parser.add_argument("--high-water-mark", type=int, default=2_048)
    parser.add_argument("--scene-generation")
    parser.add_argument("--expected-samples", type=int)
    parser.add_argument("--metrics-path", type=Path)
    parser.add_argument("--spool-root", type=Path, default=Path("artifacts/vehicle_frame_spool"))
    parser.add_argument("--assets-endpoint")
    parser.add_argument("--asset-store-root", type=Path, default=Path("artifacts/vehicle_assets"))
    parser.add_argument("--analysis-endpoint")
    parser.add_argument(
        "--analysis-spool-root", type=Path, default=Path("artifacts/vehicle_analysis")
    )
    args = parser.parse_args()
    if args.queue_depth <= 0:
        parser.error("--queue-depth must be positive")
    if args.high_water_mark <= 0:
        parser.error("--high-water-mark must be positive")
    if args.mcap_rate <= 0:
        parser.error("--mcap-rate must be positive")
    if args.mcap is not None and not args.mcap.is_file():
        parser.error(f"--mcap does not exist: {args.mcap}")

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
    adapter = RosFrameAdapter(
        _LocalPosePublisher(),
        RosFrameAdapterConfig(
            expected_scene_generation=args.scene_generation, expected_samples=args.expected_samples
        ),
        metrics=JsonlMetrics(args.metrics_path),
        snapshot_sink=spool,
        analysis_assembler=analysis_assembler,
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
                expected_samples=args.expected_samples,
                rate=args.mcap_rate,
            ).replay(args.mcap)
        while rclpy is None or rclpy.ok():
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
        if analysis_assembler is not None:
            analysis_assembler.close()
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

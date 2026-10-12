from __future__ import annotations

import json
from types import SimpleNamespace

import cv2
import numpy as np

import pytest

from src.deployment.ros_frame_adapter import LatestBevFusionBoxes, McapFrameReplay, RosAnalysisAssembler, RosFrameAdapter, RosFrameAdapterConfig, _load_gaussian_T_nuscenes_global
from src.realtime.bevfusion import BevFusionBox, BevFusionResult
from src.deployment.vehicle import VehicleAnalysisSpool, VehicleFrameSpool
from src.realtime.process_support import JsonlMetrics
from src.realtime.protocol import FrameDetections, ProtocolCodec
from src.realtime import EgoPoseSample, FrameObjectDetections
from src.data.cooperscene_mcap import _serialize_image, _serialize_metadata


class _Publisher:
    def __init__(self) -> None:
        self.messages = []

    def send(self, message, *, topic=None):
        self.messages.append((topic, message))


def _metadata(timestamp_us=100_000, frame_id="100", generation="cooperscene:train:1:1"):
    # Images/intrinsics remain local; exact camera pose is retained in snapshots.
    return {
        "version": 1,
        "frame_id": frame_id,
        "request_id": f"take:{frame_id}",
        "scene_generation": generation,
        "timestamp_us": timestamp_us,
        "camera_intrinsic": [1.0] * 9,
        "world_T_camera": [
            1.0, 0.0, 0.0, 0.5,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 1.5,
            0.0, 0.0, 0.0, 1.0,
        ],
        "world_T_ego": [
            1.0, 0.0, 0.0, 2.0,
            0.0, 1.0, 0.0, 3.0,
            0.0, 0.0, 1.0, 4.0,
            0.0, 0.0, 0.0, 1.0,
        ],
        "boxes": [{
            "track_id": "car-1",
            "xyxy": [10.0, 20.0, 110.0, 80.0],
            "dimensions_lwh": [4.5, 1.8, 1.5],
            "world_T_object": [
                1.0, 0.0, 0.0, 2.0,
                0.0, 1.0, 0.0, 3.0,
                0.0, 0.0, 1.0, 4.0,
                0.0, 0.0, 0.0, 1.0,
            ],
            "text_label": "car",
        }],
        "end_of_scene": False,
    }


def test_adapter_publishes_ordered_pose_only_samples_and_metrics(tmp_path):
    publisher = _Publisher()
    metrics_path = tmp_path / "adapter.jsonl"
    adapter = RosFrameAdapter(publisher, metrics=JsonlMetrics(metrics_path))

    first = adapter.receive_metadata(json.dumps(_metadata(100_000, "100")), arrival_ns=20)
    second = adapter.receive_metadata(_metadata(200_000, "101"), arrival_ns=30)

    assert [type(message) for _, message in publisher.messages] == [
        FrameObjectDetections, EgoPoseSample, FrameObjectDetections, EgoPoseSample
    ]
    assert [message.sequence for _, message in publisher.messages] == [0, 0, 1, 1]
    detections, first_pose, second_detections, second_pose = [
        message for _, message in publisher.messages
    ]
    assert detections.sequence == first_pose.sequence == 0
    assert detections.frame_id == first_pose.frame_id == "100"
    assert second_detections.timestamp_us == second_pose.timestamp_us == 200_000
    assert len(detections.boxes) == 1
    assert first.frame_id == "100"
    assert second.timestamp_us == 200_000
    assert set(first._metadata()) == {
        "sequence", "timestamp_us", "scene_generation", "frame_id", "world_T_ego"
    }
    events = [json.loads(line) for line in metrics_path.read_text().splitlines()]
    assert [event["event"] for event in events] == ["ros_frame_snapshot_spooled"] * 2
    assert [event["sequence"] for event in events] == [0, 1]
    assert [event["metadata_callback_arrival_ns"] for event in events] == [20, 30]
    assert all(event["publication_monotonic_ns"] >= event["callback_completed_monotonic_ns"] for event in events)
    assert [event["object_detection_count"] for event in events] == [1, 1]
    assert all(event["transport_contract"] == "durable_frame_snapshot" for event in events)


def test_adapter_relocalizes_legacy_nuscenes_mcap_metadata():
    payload = _metadata(generation="nuscenes:152")
    stored_alignment = np.eye(4)
    stored_alignment[:3, 3] = [5, 7, 11]
    payload["map_T_nuscenes_global"] = tuple(stored_alignment.reshape(-1))
    payload["map_T_lidar"] = payload["world_T_ego"]
    alignment = np.eye(4)
    alignment[:3, 3] = [10, 20, 30]
    publisher = _Publisher()
    adapter = RosFrameAdapter(
        publisher, RosFrameAdapterConfig(gaussian_T_nuscenes_global=alignment)
    )

    pose = adapter.receive_metadata(payload)
    detections = publisher.messages[0][1]

    np.testing.assert_allclose(np.asarray(pose.world_T_ego).reshape(4, 4)[:3, 3], [7, 16, 23])
    np.testing.assert_allclose(
        np.asarray(detections.boxes[0].world_T_object).reshape(4, 4)[:3, 3], [7, 16, 23]
    )


def test_adapter_relocalizes_legacy_cooperscene_mcap_metadata():
    payload = _metadata()
    stored_alignment = np.eye(4)
    stored_alignment[:3, 3] = [5, 7, 11]
    target_alignment = np.eye(4)
    target_alignment[:3, 3] = [10, 20, 30]
    publisher = _Publisher()
    adapter = RosFrameAdapter(
        publisher,
        RosFrameAdapterConfig(
            gaussian_T_cooperscene=target_alignment,
            stored_gaussian_T_cooperscene=stored_alignment,
        ),
    )

    pose = adapter.receive_metadata(payload)
    detections = publisher.messages[0][1]

    np.testing.assert_allclose(
        np.asarray(pose.world_T_ego).reshape(4, 4)[:3, 3], [7, 16, 23]
    )
    np.testing.assert_allclose(
        np.asarray(detections.boxes[0].world_T_object).reshape(4, 4)[:3, 3],
        [7, 16, 23],
    )


def test_localization_artifact_rejects_another_scene_generation(tmp_path):
    artifact = tmp_path / "localization.json"
    artifact.write_text(json.dumps({
        "scene_id": "171",
        "gaussian_T_nuscenes_global": np.eye(4).tolist(),
    }))

    with pytest.raises(ValueError, match="nuscenes:171 != nuscenes:152"):
        _load_gaussian_T_nuscenes_global(
            artifact, expected_scene_generation="nuscenes:152"
        )


def test_adapter_output_round_trips_without_binary_image_parts():
    adapter = RosFrameAdapter(_Publisher())
    pose = adapter.receive_metadata(_metadata())

    encoded = ProtocolCodec().encode(pose)
    assert len(encoded) == 2
    topic, decoded = ProtocolCodec().decode(encoded)
    assert topic == "ego_pose_sample"
    assert decoded == pose


def test_adapter_rejects_malformed_or_duplicate_detection_prompts():
    duplicate = _metadata()
    duplicate["boxes"] *= 2
    with pytest.raises(ValueError, match="one prompt per track_id"):
        RosFrameAdapter(_Publisher()).receive_metadata(duplicate)
    malformed = _metadata()
    malformed["boxes"] = {"not": "an array"}
    with pytest.raises(ValueError, match="boxes must be an array"):
        RosFrameAdapter(_Publisher()).receive_metadata(malformed)


def test_adapter_latches_and_rejects_mismatched_scene_generation():
    adapter = RosFrameAdapter(_Publisher(), RosFrameAdapterConfig(expected_scene_generation="scene-a"))

    with pytest.raises(ValueError, match="does not match"):
        adapter.receive_metadata(_metadata(generation="scene-b"))
    assert adapter.scene_generation == "scene-a"
    assert adapter.receive_metadata(_metadata(generation="scene-a")).sequence == 0


@pytest.mark.parametrize("payload, error", [
    ({"version": 2}, "version must be 1"),
    (_metadata(timestamp_us=-1), "timestamp_us must be a non-negative integer"),
    (_metadata(generation=""), "scene_generation must be a non-empty string"),
])
def test_adapter_rejects_invalid_metadata(payload, error):
    with pytest.raises(ValueError, match=error):
        RosFrameAdapter(_Publisher()).receive_metadata(payload)


def _image(timestamp_us, *, encoding="bgr8", padding=0):
    bgr = np.zeros((4, 6, 3), dtype=np.uint8)
    bgr[:, :] = (10, 30, 220)
    pixels = bgr if encoding == "bgr8" else cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    packed = pixels.reshape(4, 18)
    rows = np.pad(packed, ((0, 0), (0, padding)))
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=timestamp_us // 1_000_000, nanosec=(timestamp_us % 1_000_000) * 1_000)
        ),
        height=4, width=6, step=18 + padding, encoding=encoding, data=rows.tobytes(),
    )


@pytest.mark.parametrize("image_first", [False, True])
def test_analysis_assembler_durably_joins_image_and_metadata_in_either_order(tmp_path, image_first):
    spool = VehicleAnalysisSpool(tmp_path / "analysis")
    assembler = RosAnalysisAssembler(spool, expected_samples=1)
    metadata = _metadata(timestamp_us=400_000)
    image = _image(400_000, encoding="rgb8", padding=5)

    if image_first:
        assembler.receive_image(image)
        assembler.receive_metadata(metadata, 0)
    else:
        assembler.receive_metadata(metadata, 0)
        assembler.receive_image(image)
    assembler.close()

    frame = spool.get(metadata["scene_generation"], 0)
    assert frame is not None
    assert frame.source_sequence == 0
    assert frame.boxes[0].text_label == "car"
    decoded = cv2.imdecode(np.frombuffer(frame.image_jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (4, 6, 3)
    assert decoded[0, 0, 2] > decoded[0, 0, 0]
    assert spool.end(metadata["scene_generation"]) is not None


def test_direct_mcap_replay_preserves_record_order_pacing_and_durable_spools(tmp_path):
    scene = "cooperscene:train:1:1"
    snapshots = VehicleFrameSpool(tmp_path / "frames")
    analysis = VehicleAnalysisSpool(tmp_path / "analysis")
    assembler = RosAnalysisAssembler(analysis, expected_samples=2)
    adapter = RosFrameAdapter(
        _Publisher(),
        RosFrameAdapterConfig(expected_samples=2),
        snapshot_sink=snapshots,
        analysis_assembler=assembler,
    )
    records = []
    for sequence, timestamp_us in enumerate((0, 100_000)):
        payload = _metadata(timestamp_us=timestamp_us, frame_id=str(sequence), generation=scene)
        detections = FrameDetections(
            frame_id=payload["frame_id"], request_id=payload["request_id"],
            scene_generation=scene, timestamp_us=timestamp_us, image_jpeg=b"x",
            camera_intrinsic=payload["camera_intrinsic"], world_T_camera=payload["world_T_camera"],
            world_T_ego=payload["world_T_ego"], boxes=(),
        )
        image = _serialize_image(
            timestamp_ns=timestamp_us * 1_000, frame_id="camera0", height=1, width=1,
            data=bytes((1, 2, 3)),
        )
        log_time = timestamp_us * 1_000
        records.extend((
            ("/camera/image_raw", image, log_time),
            ("/camera/frame_detections", _serialize_metadata(detections), log_time),
        ))

    class Reader:
        def iter_messages(self, **kwargs):
            assert kwargs == {
                "topics": ("/camera/image_raw", "/camera/frame_detections"),
                "log_time_order": False,
            }
            for topic, data, log_time in records:
                schema_name = "sensor_msgs/msg/Image" if topic.endswith("image_raw") else "std_msgs/msg/String"
                yield SimpleNamespace(name=schema_name), SimpleNamespace(topic=topic, message_encoding="cdr"), SimpleNamespace(data=data, log_time=log_time)

    waits = []
    now = [0.0]
    def sleep(seconds):
        waits.append(seconds)
        now[0] += seconds
    path = tmp_path / "input.mcap"
    path.write_bytes(b"test")
    McapFrameReplay(
        adapter,
        image_topic="/camera/image_raw",
        metadata_topic="/camera/frame_detections",
        expected_samples=2,
        sleep=sleep,
        monotonic=lambda: now[0],
        reader_factory=lambda stream: Reader(),
    ).replay(path)
    assembler.close()

    assert waits == [0.1]
    assert snapshots.end(scene) is not None
    np.testing.assert_allclose(
        np.asarray(snapshots.get(scene, 0).world_T_camera).reshape(4, 4)[:3, 3],
        [0.5, 0.0, 1.5],
    )
    assert analysis.end(scene) is not None
    assert [analysis.get(scene, sequence).timestamp_us for sequence in range(2)] == [0, 100_000]


def test_direct_mcap_replay_rejects_incomplete_authoritative_stream(tmp_path):
    adapter = RosFrameAdapter(
        _Publisher(), RosFrameAdapterConfig(expected_samples=1),
        snapshot_sink=VehicleFrameSpool(tmp_path / "frames"),
        analysis_assembler=RosAnalysisAssembler(VehicleAnalysisSpool(tmp_path / "analysis"), expected_samples=1),
    )

    class Reader:
        def iter_messages(self, **kwargs):
            del kwargs
            return iter(())

    path = tmp_path / "empty.mcap"
    path.write_bytes(b"test")
    with pytest.raises(RuntimeError, match="expected 1 MCAP metadata records, got 0"):
        McapFrameReplay(
            adapter, image_topic="/camera/image_raw", metadata_topic="/camera/frame_detections",
            expected_samples=1, reader_factory=lambda stream: Reader(),
        ).replay(path)


def test_direct_mcap_replay_reads_generated_cdr_records(tmp_path):
    from mcap.writer import Writer

    scene = "cooperscene:train:1:1"
    payload = _metadata(timestamp_us=0, frame_id="0", generation=scene)
    detections = FrameDetections(
        frame_id=payload["frame_id"], request_id=payload["request_id"],
        scene_generation=scene, timestamp_us=0, image_jpeg=b"x",
        camera_intrinsic=payload["camera_intrinsic"], world_T_camera=payload["world_T_camera"],
        world_T_ego=payload["world_T_ego"], boxes=(),
    )
    path = tmp_path / "generated.mcap"
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile="ros2")
        image_schema = writer.register_schema("sensor_msgs/msg/Image", "ros2msg", b"")
        metadata_schema = writer.register_schema("std_msgs/msg/String", "ros2msg", b"")
        image_channel = writer.register_channel("/camera/image_raw", "cdr", image_schema)
        metadata_channel = writer.register_channel("/camera/frame_detections", "cdr", metadata_schema)
        writer.add_message(
            image_channel, 0,
            _serialize_image(timestamp_ns=0, frame_id="camera0", height=1, width=1, data=b"\x01\x02\x03"),
            0,
        )
        writer.add_message(metadata_channel, 0, _serialize_metadata(detections), 0)
        writer.finish()

    analysis = VehicleAnalysisSpool(tmp_path / "analysis")
    assembler = RosAnalysisAssembler(analysis, expected_samples=1)
    adapter = RosFrameAdapter(
        _Publisher(), RosFrameAdapterConfig(expected_samples=1),
        snapshot_sink=VehicleFrameSpool(tmp_path / "frames"), analysis_assembler=assembler,
    )
    McapFrameReplay(
        adapter, image_topic="/camera/image_raw", metadata_topic="/camera/frame_detections",
        expected_samples=1,
    ).replay(path)
    assembler.close()

    assert analysis.get(scene, 0).timestamp_us == 0


@pytest.mark.parametrize("label", ["car", "truck", "bus", "trailer", "construction_vehicle", "motorcycle", "bicycle", "vehicle.emergency.ambulance", "pedestrian", "traffic_cone"])
def test_bevfusion_latest_only_reprojects_lidar_boxes_and_drops_gt(label):
    class Client:
        def __init__(self):
            self.requests = []
            self.result = None
        def submit(self, request):
            self.requests.append(request)
            return True
        def poll(self):
            result, self.result = self.result, None
            return result

    client = Client()
    detector = LatestBevFusionBoxes(client, score_threshold=0.2, minimum_area_px=1)
    payload = _metadata(timestamp_us=0, frame_id="0", generation="nuscenes:152")
    payload["camera_intrinsic"] = [50.0, 0.0, 50.0, 0.0, 50.0, 50.0, 0.0, 0.0, 1.0]
    payload["world_T_camera"] = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    payload["world_T_ego"] = payload["world_T_camera"]
    detector.receive_image(_image(0))
    detector.receive_metadata(payload, 0)
    detector.receive_lidar(SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)), points=np.zeros((1, 4), dtype=np.float32)))
    assert len(client.requests) == 1
    request = client.requests[0]
    client.result = BevFusionResult(
        request_id=request.request_id, scene_generation=request.scene_generation,
        sequence=request.sequence, frame_id=request.frame_id, timestamp_us=request.timestamp_us,
        config_id=request.config_id,
        boxes=(BevFusionBox((0, 0, 10), (4, 2, 2), 0, 0.9, class_name=label),),
    )
    next_payload = dict(payload, timestamp_us=100_000, frame_id="1", source_sequence=1)
    boxes = detector.boxes_for(next_payload, (100, 100))
    if label in {"pedestrian", "traffic_cone"}:
        assert boxes == ()
    else:
        track_label = label.replace(".", "_")
        assert [box.track_id for box in boxes] == [f"bevfusion:{track_label}:0"]
        assert boxes[0].detection_confidence == 0.9
        assert boxes[0].text_label == label


def test_bevfusion_latest_only_replaces_an_older_pending_request():
    class Client:
        def __init__(self): self.requests = []
        def submit(self, request): self.requests.append(request); return True
        def poll(self): return None

    client = Client()
    detector = LatestBevFusionBoxes(client, score_threshold=0.2)
    identity = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    for sequence in range(3):
        timestamp_us = sequence * 100_000
        payload = _metadata(
            timestamp_us=timestamp_us,
            frame_id=str(sequence),
            generation="nuscenes:152",
        )
        payload.update(
            camera_intrinsic=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            world_T_camera=identity,
            world_T_ego=identity,
        )
        detector.receive_image(_image(timestamp_us))
        detector.receive_metadata(payload, sequence)
        detector.receive_lidar(SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(
                sec=timestamp_us // 1_000_000,
                nanosec=(timestamp_us % 1_000_000) * 1_000,
            )),
            points=np.zeros((1, 4), dtype=np.float32),
        ))

    assert [request.sequence for request in client.requests] == [0]
    assert detector.replaced_pending_count == 1


def test_bevfusion_boxes_are_identical_in_snapshot_and_analysis_spools(tmp_path):
    class Client:
        def __init__(self): self.requests = []; self.result = None
        def submit(self, request): self.requests.append(request); return True
        def poll(self): result, self.result = self.result, None; return result

    client = Client()
    detector = LatestBevFusionBoxes(client, score_threshold=0.2, minimum_area_px=1)
    base = _metadata(timestamp_us=0, frame_id="0", generation="nuscenes:152")
    identity = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    base.update(camera_intrinsic=[10., 0., 3., 0., 10., 2., 0., 0., 1.], world_T_camera=identity, world_T_ego=identity)
    detector.receive_image(_image(0)); detector.receive_metadata(base, 0)
    detector.receive_lidar(SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)), points=np.zeros((1, 4), dtype=np.float32)))
    request = client.requests[0]
    client.result = BevFusionResult(request.request_id, request.scene_generation, request.sequence, request.frame_id, request.timestamp_us, request.config_id, (BevFusionBox((0, 0, 10), (4, 2, 2), 0, .9),))
    frames, analysis = VehicleFrameSpool(tmp_path / "frames"), VehicleAnalysisSpool(tmp_path / "analysis")
    assembler = RosAnalysisAssembler(analysis, expected_samples=1)
    adapter = RosFrameAdapter(_Publisher(), RosFrameAdapterConfig(expected_samples=1, bbox_source="bevfusion"), snapshot_sink=frames, analysis_assembler=assembler, bevfusion=detector)
    current = dict(base, timestamp_us=100_000, frame_id="1")
    image = _image(100_000)
    assembler.receive_image(image); detector.receive_image(image)
    adapter.receive_metadata(current)
    assembler.close()
    snapshot = frames.get("nuscenes:152", 0)
    detected = analysis.get("nuscenes:152", 0)
    assert snapshot is not None and detected is not None
    assert snapshot.boxes == detected.boxes
    assert snapshot.boxes[0].track_id == "bevfusion:car:0"


def test_foxglove_selected_start_rebases_and_preserves_provenance(tmp_path, monkeypatch):
    from src.data.foxglove_mcap import FoxgloveFrame
    import src.data.foxglove_mcap as foxglove

    scene = 'cooperscene:train:1:1'
    frames = []
    for i in range(4):
        metadata = _metadata(timestamp_us=i * 100000, frame_id=str(100 + i), generation=scene)
        metadata['source_sequence'] = i
        metadata['source_timestamp_ns'] = 1000000000 + i * 100000000
        frames.append(FoxgloveFrame(i, metadata['source_timestamp_ns'], np.zeros((1, 1, 3), dtype=np.uint8), metadata, np.zeros((2, 4), dtype=np.float32), ()))
    monkeypatch.setattr(foxglove, 'FoxgloveMcapReader', lambda *args, **kwargs: frames)
    images, metadata_rows, clouds = [], [], []
    adapter = SimpleNamespace(scene_generation=scene,
        analysis_assembler=SimpleNamespace(receive_image=images.append, raise_if_failed=lambda: None),
        bevfusion=SimpleNamespace(receive_image=lambda x: None, receive_lidar=clouds.append, poll=lambda: None),
        receive_metadata=metadata_rows.append)
    waits, now = [], [0.0]
    def sleep(seconds):
        waits.append(seconds)
        now[0] += seconds
    replay = McapFrameReplay(adapter, image_topic='/image', metadata_topic='/metadata',
        lidar_topic='/lidar', expected_samples=2, start_sequence=2,
        sleep=sleep, monotonic=lambda: now[0])
    replay._replay_foxglove(tmp_path / 'unused.mcap')
    assert sum(waits) == pytest.approx(.1)
    assert max(waits) <= .01
    assert [row['frame_id'] for row in metadata_rows] == ['102', '103']
    assert [row['timestamp_us'] for row in metadata_rows] == [0, 100000]
    assert [row['recorded_source_sequence'] for row in metadata_rows] == [2, 3]
    assert [row['recorded_timestamp_us'] for row in metadata_rows] == [200000, 300000]
    assert [row['source_sequence'] for row in metadata_rows] == [0, 1]
    assert [image.header.stamp.nanosec for image in images] == [0, 100000000]
    assert len(clouds) == 2
    assert frames[2].metadata['timestamp_us'] == 200000


@pytest.mark.parametrize("foxglove", [False, True])
def test_mcap_wait_dispatches_pending_detection_without_extra_publications(tmp_path, monkeypatch, foxglove):
    import src.deployment.ros_frame_adapter as module
    import src.data.foxglove_mcap as foxglove_module
    from src.data.foxglove_mcap import FoxgloveFrame

    now, waits = [0.0], []

    class DelayedClient:
        def __init__(self):
            self.requests = []
            self.pending = None

        def submit(self, request):
            assert self.pending is None
            self.requests.append((request, now[0]))
            self.pending = (request, now[0] + .035)
            return True

        def poll(self):
            if self.pending is None or now[0] < self.pending[1]:
                return None
            request, _ = self.pending
            self.pending = None
            return BevFusionResult(request.request_id, request.scene_generation, request.sequence,
                                   request.frame_id, request.timestamp_us, request.config_id, ())

    def sleep(seconds):
        waits.append(seconds)
        now[0] += seconds

    timestamps = (0, 10_000, 100_000)
    scene = "cooperscene:train:1:1"
    client = DelayedClient()
    detector = LatestBevFusionBoxes(client, score_threshold=.2)
    snapshots = VehicleFrameSpool(tmp_path / "frames")
    assembler = RosAnalysisAssembler(VehicleAnalysisSpool(tmp_path / "analysis"), expected_samples=3)
    publisher = _Publisher()
    adapter = RosFrameAdapter(publisher, RosFrameAdapterConfig(expected_samples=3, bbox_source="bevfusion"),
                              snapshot_sink=snapshots, analysis_assembler=assembler, bevfusion=detector)
    records, frames = [], []
    for sequence, timestamp in enumerate(timestamps):
        metadata = _metadata(timestamp_us=timestamp, frame_id=str(sequence), generation=scene)
        frames.append(FoxgloveFrame(sequence, timestamp * 1000, np.zeros((1, 1, 3), np.uint8),
                                   metadata, np.zeros((2, 4), np.float32), ()))
        detections = FrameDetections(frame_id=str(sequence), request_id=metadata["request_id"],
            scene_generation=scene, timestamp_us=timestamp, image_jpeg=b"x",
            camera_intrinsic=metadata["camera_intrinsic"], world_T_camera=metadata["world_T_camera"],
            world_T_ego=metadata["world_T_ego"], boxes=())
        for topic, schema, data in (
            ("/image", "sensor_msgs/msg/Image", _serialize_image(timestamp_ns=timestamp * 1000,
                frame_id="camera0", height=1, width=1, data=b"\x01\x02\x03")),
            ("/lidar", "sensor_msgs/msg/PointCloud2", str(timestamp).encode()),
            ("/metadata", "std_msgs/msg/String", _serialize_metadata(detections)),
        ):
            records.append((SimpleNamespace(name=schema), SimpleNamespace(topic=topic, message_encoding="cdr"),
                            SimpleNamespace(log_time=timestamp * 1000, data=data)))

    class Reader:
        def iter_messages(self, **kwargs):
            return iter(records)

    def cloud(data):
        timestamp = int(data)
        return SimpleNamespace(header=_image(timestamp).header, points=np.zeros((2, 4), np.float32))

    monkeypatch.setattr(module, "_decode_mcap_point_cloud2", cloud)
    monkeypatch.setattr(foxglove_module, "FoxgloveMcapReader", lambda *a, **k: frames)
    path = tmp_path / "input.mcap"
    path.write_bytes(b"test")
    replay = McapFrameReplay(adapter, image_topic="/image", metadata_topic="/metadata", lidar_topic="/lidar",
        expected_samples=3, sleep=sleep, monotonic=lambda: now[0], reader_factory=lambda _: Reader())
    if foxglove:
        replay._replay_foxglove(path)
    else:
        replay.replay(path)
    assembler.close()

    assert [r.timestamp_us for r, _ in client.requests] == list(timestamps)
    assert [r.sequence for r, _ in client.requests] == [0, 1, 2]
    assert client.requests[1][1] == pytest.approx(.04)  # Pending frame dispatched before next capture.
    assert client.requests[2][1] == pytest.approx(.1)
    assert sum(waits) == pytest.approx(.1)
    assert max(waits) <= .01
    assert len(publisher.messages) == 6
    assert [snapshots.get(scene, i).timestamp_us for i in range(3)] == list(timestamps)
    assert detector.result_count == 2
    assert detector.latest_sequence == 1  # Replay EOF still leaves final request in flight.


def test_mcap_wait_keeps_late_frames_immediate_and_no_detector_sleep_unchanged():
    now, waits, polls = [1.0], [], []
    def sleep(seconds):
        waits.append(seconds)
        now[0] += seconds
    adapter = SimpleNamespace(analysis_assembler=object(), bevfusion=None)
    replay = McapFrameReplay(adapter, image_topic="/image", metadata_topic="/metadata", expected_samples=None,
                             sleep=sleep, monotonic=lambda: now[0])
    replay._wait_until(1.1)
    assert waits == pytest.approx([.1])
    adapter.bevfusion = SimpleNamespace(poll=lambda: polls.append(now[0]))
    replay._wait_until(1.0)
    assert polls == []
    assert waits == pytest.approx([.1])


class _LifecycleBevFusionClient:
    def __init__(self):
        self.requests = []
        self.result = None

    def submit(self, request):
        self.requests.append(request)
        return True

    def poll(self):
        result, self.result = self.result, None
        return result


def _complete_lifecycle_detection(detector, client, timestamp_us, sequence, score, extra_boxes=()):
    payload = _metadata(timestamp_us=timestamp_us, frame_id=str(sequence), generation="offline-lifecycle")
    payload["camera_intrinsic"] = [50., 0., 50., 0., 50., 50., 0., 0., 1.]
    payload["world_T_camera"] = np.eye(4).reshape(-1).tolist()
    payload["world_T_ego"] = payload["world_T_camera"]
    detector.receive_image(_image(timestamp_us))
    detector.receive_metadata(payload, sequence)
    detector.receive_lidar(SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(
            sec=timestamp_us // 1_000_000, nanosec=(timestamp_us % 1_000_000) * 1000)),
        points=np.zeros((1, 4), dtype=np.float32),
    ))
    request = client.requests[-1]
    client.result = BevFusionResult(
        request_id=request.request_id, scene_generation=request.scene_generation,
        sequence=request.sequence, frame_id=request.frame_id, timestamp_us=request.timestamp_us,
        config_id=request.config_id,
        boxes=(BevFusionBox((0, 0, 10), (4, 2, 2), 0, score, class_name="car"), *extra_boxes),
    )
    return detector.boxes_for(payload, (100, 100))


@pytest.mark.parametrize("continuation", [None, .1])
def test_bevfusion_weak_server_proposal_cannot_birth_a_track(continuation):
    client = _LifecycleBevFusionClient()
    detector = LatestBevFusionBoxes(client, score_threshold=.2, minimum_area_px=1,
                                  continuation_score_threshold=continuation)
    assert _complete_lifecycle_detection(detector, client, 0, 0, .15) == ()
    assert _complete_lifecycle_detection(detector, client, 100_000, 1, .15) == ()
    strong = _complete_lifecycle_detection(detector, client, 200_000, 2, .9)
    assert strong[0].track_id == "bevfusion:car:0"


def test_bevfusion_confirmed_weak_update_remains_visible_with_honest_confidence():
    client = _LifecycleBevFusionClient()
    detector = LatestBevFusionBoxes(client, score_threshold=.2, minimum_area_px=1,
                                  continuation_score_threshold=.1)
    strong = _complete_lifecycle_detection(detector, client, 0, 0, .9)
    weak = _complete_lifecycle_detection(detector, client, 100_000, 1, .15)
    assert len(weak) == 1
    assert weak[0].track_id == strong[0].track_id
    assert weak[0].detection_confidence == .15


def test_bevfusion_default_primary_only_ignores_weak_updates_then_expires():
    client = _LifecycleBevFusionClient()
    detector = LatestBevFusionBoxes(client, score_threshold=.2, minimum_area_px=1)
    strong = _complete_lifecycle_detection(detector, client, 0, 0, .9)
    coast = _complete_lifecycle_detection(detector, client, 100_000, 1, .15)
    assert coast[0].track_id == strong[0].track_id
    assert coast[0].detection_confidence == .9
    assert _complete_lifecycle_detection(detector, client, 500_001, 2, .15) == ()


@pytest.mark.parametrize("reacquisition_seconds, same_identity", [(0.0, False), (2.0, True)])
def test_bevfusion_adapter_optional_identity_reacquisition(reacquisition_seconds, same_identity):
    client = _LifecycleBevFusionClient()
    detector = LatestBevFusionBoxes(
        client, score_threshold=.2, minimum_area_px=1,
        maximum_reacquisition_seconds=reacquisition_seconds,
    )
    original = _complete_lifecycle_detection(detector, client, 0, 0, .9)
    reacquired = _complete_lifecycle_detection(detector, client, 600_001, 1, .9)
    assert len(original) == len(reacquired) == 1
    assert (original[0].track_id == reacquired[0].track_id) == same_identity
    assert reacquired[0].detection_confidence == .9


def test_bevfusion_weak_updates_cannot_confirm_tentative_tracks():
    client = _LifecycleBevFusionClient()
    detector = LatestBevFusionBoxes(client, score_threshold=.2, minimum_area_px=1,
                                  continuation_score_threshold=.1, minimum_confirmation_hits=2)
    assert _complete_lifecycle_detection(detector, client, 0, 0, .9) == ()
    assert _complete_lifecycle_detection(detector, client, 100_000, 1, .15) == ()
    assert _complete_lifecycle_detection(detector, client, 200_000, 2, .9) == ()
    confirmed = _complete_lifecycle_detection(detector, client, 300_000, 3, .9)
    assert len(confirmed) == 1
    assert confirmed[0].track_id == "bevfusion:car:0"


@pytest.mark.parametrize("settings", [
    {"continuation_score_threshold": -.1},
    {"continuation_score_threshold": .3},
    {"continuation_score_threshold": float("nan")},
    {"minimum_confirmation_hits": 0},
    {"minimum_confirmation_hits": True},
    {"maximum_reacquisition_seconds": -.1},
    {"maximum_reacquisition_seconds": float("nan")},
    {"maximum_reacquisition_seconds": float("inf")},
])
def test_bevfusion_adapter_rejects_invalid_tracker_settings(settings):
    with pytest.raises(ValueError):
        LatestBevFusionBoxes(_LifecycleBevFusionClient(), score_threshold=.2, **settings)


@pytest.mark.parametrize("arguments, expected", [
    (["--bevfusion-continuation-score-threshold", ".3"], "between zero and the birth score"),
    (["--bevfusion-maximum-reacquisition-seconds", "-1"], "finite and non-negative"),
    (["--bevfusion-maximum-reacquisition-seconds", "nan"], "finite and non-negative"),
    (["--bevfusion-maximum-reacquisition-seconds", "inf"], "finite and non-negative"),
    (["--bevfusion-continuation-score-threshold", "nan"], "between zero and the birth score"),
    (["--bevfusion-minimum-confirmation-hits", "0"], "must be positive"),
])
def test_bevfusion_cli_rejects_invalid_tracker_settings_before_transport(monkeypatch, capsys, arguments, expected):
    from src.deployment import ros_frame_adapter
    monkeypatch.setattr("sys.argv", ["ros_frame_adapter", *arguments])
    with pytest.raises(SystemExit) as error:
        ros_frame_adapter.main()
    assert error.value.code == 2
    assert expected in capsys.readouterr().err


@pytest.mark.parametrize("guard, expected_count", [(False, 2), (True, 1)])
def test_bevfusion_adapter_optional_duplicate_birth_guard_preserves_continuation(guard, expected_count):
    client = _LifecycleBevFusionClient()
    detector = LatestBevFusionBoxes(client, score_threshold=.2, minimum_area_px=1,
                                  suppress_duplicate_births=guard)
    strong = _complete_lifecycle_detection(detector, client, 0, 0, .9)
    boxes = _complete_lifecycle_detection(
        detector, client, 100_000, 1, .9,
        extra_boxes=(BevFusionBox((.2, 0, 10), (4, 2, 2), 0, .8, class_name="car"),),
    )
    assert len(boxes) == expected_count
    assert boxes[0].track_id == strong[0].track_id
    assert boxes[0].detection_confidence == .9


def test_main_idle_loop_advances_final_bevfusion_pending_request(monkeypatch, tmp_path):
    from src.deployment import ros_frame_adapter as module

    spools = []
    adapters = []

    class Client(_LifecycleBevFusionClient):
        def complete(self, request):
            self.result = BevFusionResult(
                request_id=request.request_id, scene_generation=request.scene_generation,
                sequence=request.sequence, frame_id=request.frame_id, timestamp_us=request.timestamp_us,
                config_id=request.config_id,
                boxes=(BevFusionBox((0, 0, 10), (4, 2, 2), 0, .9, class_name="car"),),
            )

        def submit(self, request):
            accepted = super().submit(request)
            if request.sequence == 2:
                self.complete(request)
            return accepted

    class Router:
        def close(self): pass

    class Spool:
        def __init__(self, *args):
            self.snapshots = []
            spools.append(self)
        def append(self, snapshot): self.snapshots.append(snapshot)

    class Assembler:
        def __init__(self, *args, **kwargs): pass
        def receive_image(self, image): pass
        def receive_metadata(self, payload, sequence): pass
        def raise_if_failed(self): pass
        def close(self): pass

    class Service:
        def __init__(self, *args): self.checks = 0
        def start(self): pass
        def stop(self): pass
        def raise_if_failed(self):
            self.checks += 1
            if self.checks == 2:
                raise RuntimeError("stop fake idle loop")

    class Replay:
        def __init__(self, adapter, **kwargs):
            self.adapter = adapter
            adapters.append(adapter)
        def replay(self, path):
            for i in range(3):
                timestamp = i * 100_000
                self.adapter.bevfusion.receive_image(_image(timestamp))
                self.adapter.bevfusion.receive_lidar(SimpleNamespace(
                    header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=timestamp * 1000)),
                    points=np.zeros((1, 4), np.float32),
                ))
                self.adapter.receive_metadata(_metadata(timestamp_us=timestamp, frame_id=str(i)))
            # Response0 arrives only after EOF; no further image/metadata callback.
            assert [r.sequence for r in client.requests] == [0]
            client.complete(client.requests[0])

    client = Client()
    mcap = tmp_path / "offline.mcap"
    mcap.touch()
    monkeypatch.setattr("sys.argv", ["ros_frame_adapter", "--mcap", str(mcap),
        "--bbox-source", "bevfusion", "--bevfusion-endpoint", "fake",
        "--analysis-endpoint", "fake"])
    monkeypatch.setattr(module, "_open_bevfusion_client", lambda endpoint: client)
    monkeypatch.setattr(module.zmq, "Context", lambda: SimpleNamespace(term=lambda: None))
    monkeypatch.setattr(module.RouterTransport, "open", lambda *args, **kwargs: Router())
    monkeypatch.setattr(module, "VehicleFrameSpool", Spool)
    monkeypatch.setattr(module, "VehicleAnalysisSpool", Spool)
    monkeypatch.setattr(module, "VehicleFrameSession", lambda *args: None)
    monkeypatch.setattr(module, "FrameRouterService", Service)
    monkeypatch.setattr(module, "RosAnalysisAssembler", Assembler)
    monkeypatch.setattr(module, "McapFrameReplay", Replay)
    monkeypatch.setattr(module.time, "sleep", lambda seconds: None)
    with pytest.raises(RuntimeError, match="stop fake idle loop"):
        module.main()
    assert [r.sequence for r in client.requests] == [0, 2]
    assert [r.timestamp_us for r in client.requests] == [0, 200_000]
    assert client.result is None
    assert adapters[0].bevfusion.result_count == 2
    assert adapters[0].bevfusion.latest_sequence == 2
    # Polling advances inference only; published source frames remain immutable.
    assert [s.timestamp_us for s in spools[0].snapshots] == [0, 100_000, 200_000]
    assert [len(s.boxes) for s in spools[0].snapshots] == [0, 0, 0]

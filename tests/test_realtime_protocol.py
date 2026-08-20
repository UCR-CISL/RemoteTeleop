import hashlib
import json

import pytest
import zmq

from src.realtime import (
    Acknowledgement,
    AssetEvent,
    AssetState,
    BoxPrompt,
    DealerTransport,
    EgoPoseSample,
    FrameObjectDetections,
    FrameDetections,
    LatestValueSubscriber,
    MaskBatch,
    MaskResult,
    ProtocolCodec,
    ProtocolError,
    ReconstructionEnd,
    ReconstructionRequest,
    RouterTransport,
    WorkerHealth,
    WorkerState,
)


IDENTITY_3 = tuple(float(value) for value in (1, 0, 0, 0, 1, 0, 0, 0, 1))
IDENTITY_4 = tuple(
    float(value) for value in (1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)
)


def frame(frame_id: str = "frame-1") -> FrameDetections:
    return FrameDetections(
        frame_id=frame_id,
        request_id=f"mask-{frame_id}",
        scene_generation="scene-0061:1",
        timestamp_us=123,
        image_jpeg=b"\xff\xd8jpeg",
        camera_intrinsic=IDENTITY_3,
        world_T_camera=IDENTITY_4,
        world_T_ego=IDENTITY_4,
        boxes=(
            BoxPrompt(
                track_id="car-1",
                xyxy=(10, 20, 110, 80),
                dimensions_lwh=(4.5, 1.8, 1.5),
                world_T_object=IDENTITY_4,
                text_label="car",
                visibility=0.9,
            ),
        ),
    )


def pose(sequence: int = 0) -> EgoPoseSample:
    return EgoPoseSample(
        sequence=sequence,
        timestamp_us=123 + sequence,
        scene_generation="scene-0061:1",
        frame_id=f"frame-{sequence}",
        world_T_ego=IDENTITY_4,
    )


def object_detections(sequence: int = 0) -> FrameObjectDetections:
    return FrameObjectDetections(
        sequence=sequence,
        timestamp_us=123 + sequence,
        scene_generation="scene-0061:1",
        frame_id=f"frame-{sequence}",
        boxes=frame().boxes,
    )


@pytest.mark.parametrize(
    "message",
    [
        pose(),
        object_detections(),
        frame(),
        MaskBatch(
            frame_id="frame-1",
            request_id="mask-frame-1",
            scene_generation="scene-0061:1",
            timestamp_us=123,
            masks=(
                MaskResult(
                    track_id="car-1",
                    confidence=0.95,
                    coverage=0.4,
                    box_overlap=0.8,
                    latency_ms=23,
                    mask_png=b"\x89PNG",
                ),
            ),
            batch_latency_ms=25,
        ),
        ReconstructionRequest(
            request_id="reconstruct-car-1",
            scene_generation="scene-0061:1",
            track_id="car-1",
            frame_id="frame-1",
            timestamp_us=123,
            image_jpeg=b"\xff\xd8crop",
            mask_png=b"\x89PNG",
            camera_intrinsic=IDENTITY_3,
            dimensions_lwh=(4.5, 1.8, 1.5),
            world_T_object=IDENTITY_4,
            world_T_camera=IDENTITY_4,
            quality_score=0.75,
        ),
        AssetEvent(
            request_id="reconstruct-car-1",
            scene_generation="scene-0061:1",
            track_id="car-1",
            state=AssetState.READY,
            aligned_mesh_path="/tmp/car-1.glb",
            metrics={"inference_seconds": 6.4},
        ),
        AssetEvent(
            request_id="reconstruct-car-1",
            scene_generation="scene-0061:1",
            track_id="car-1",
            state=AssetState.READY,
            asset_id="scene-0061:1:car-1",
            asset_version=2,
            content_sha256=hashlib.sha256(b"mesh-payload").hexdigest(),
            mesh_payload=b"mesh-payload",
            mesh_suffix=".ply",
        ),
        WorkerHealth(
            worker_id="sam3",
            state=WorkerState.READY,
            timestamp_us=123,
            pid=42,
            device="cuda:0",
            cuda_allocated_mib=1024,
        ),
        Acknowledgement(request_id="reconstruct-car-1", accepted=True),
    ],
)
def test_protocol_round_trip(message):
    codec = ProtocolCodec()

    topic, decoded = codec.decode(codec.encode(message))

    assert topic == message.message_type
    assert decoded == message


def test_codec_rejects_unknown_version_and_wrong_binary_count():
    codec = ProtocolCodec()
    parts = codec.encode(frame())
    envelope = json.loads(parts[1])
    envelope["protocol_version"] = 3
    parts[1] = json.dumps(envelope).encode()

    with pytest.raises(ProtocolError, match="unsupported protocol version"):
        codec.decode(parts)
    with pytest.raises(ProtocolError, match="expects 1 binary parts"):
        codec.decode(codec.encode(frame())[:-1])


def test_ego_pose_wire_contract_has_no_binary_or_camera_image_fields():
    codec = ProtocolCodec()
    encoded = codec.encode(pose(7))

    assert len(encoded) == 2
    payload = json.loads(encoded[1])["payload"]
    assert set(payload) == {
        "sequence",
        "timestamp_us",
        "scene_generation",
        "frame_id",
        "world_T_ego",
    }
    topic, decoded = codec.decode(encoded)
    assert topic == "ego_pose_sample"
    assert decoded == pose(7)


def test_object_detection_wire_contract_has_no_binary_camera_or_image_fields():
    codec = ProtocolCodec()
    encoded = codec.encode(object_detections(7))

    assert len(encoded) == 2
    payload = json.loads(encoded[1])["payload"]
    assert set(payload) == {
        "sequence", "timestamp_us", "scene_generation", "frame_id", "boxes"
    }
    assert not {"image_jpeg", "camera_intrinsic", "world_T_camera"} & set(payload)
    topic, decoded = codec.decode(encoded)
    assert topic == "frame_object_detections"
    assert decoded == object_detections(7)


@pytest.mark.parametrize(
    ("kwargs", "error"),
    [
        ({"sequence": -1}, "sequence must be non-negative"),
        ({"sequence": True}, "sequence must be an integer"),
        ({"timestamp_us": -1}, "timestamp_us must be non-negative"),
        ({"timestamp_us": 1.5}, "timestamp_us must be an integer"),
        ({"frame_id": ""}, "frame_id must be non-empty"),
        ({"scene_generation": 1}, "scene_generation must be a string"),
        ({"world_T_ego": (*IDENTITY_4[:12], 1.0, 0.0, 0.0, 1.0)}, "homogeneous"),
        ({"world_T_ego": (2.0, *IDENTITY_4[1:])}, "orthonormal"),
    ],
)
def test_ego_pose_rejects_invalid_identifiers_or_transforms(kwargs, error):
    values = {
        "sequence": 0,
        "timestamp_us": 123,
        "scene_generation": "scene-0061:1",
        "frame_id": "frame-0",
        "world_T_ego": IDENTITY_4,
    }
    values.update(kwargs)
    with pytest.raises(ValueError, match=error):
        EgoPoseSample(**values)


def test_contract_validation_rejects_duplicate_tracks_and_invalid_asset():
    prompt = frame().boxes[0]
    with pytest.raises(ValueError, match="one prompt per track_id"):
        FrameDetections(
            frame_id="frame",
            request_id="request",
            scene_generation="scene:1",
            timestamp_us=0,
            image_jpeg=b"jpeg",
            camera_intrinsic=IDENTITY_3,
            world_T_camera=IDENTITY_4,
            world_T_ego=IDENTITY_4,
            boxes=(prompt, prompt),
        )
    with pytest.raises(ValueError, match="one prompt per track_id"):
        FrameObjectDetections(
            sequence=0,
            timestamp_us=0,
            scene_generation="scene:1",
            frame_id="frame",
            boxes=(prompt, prompt),
        )
    with pytest.raises(ValueError, match="aligned_mesh_path"):
        AssetEvent(
            request_id="request",
            scene_generation="scene:1",
            track_id="car",
            state=AssetState.READY,
        )
    with pytest.raises(ValueError, match="mesh_suffix"):
        AssetEvent(
            request_id="request",
            scene_generation="scene:1",
            track_id="car",
            state=AssetState.READY,
            aligned_mesh_path="/tmp/car.ply",
            mesh_suffix="./../../mesh",
        )


def test_latest_value_subscriber_discards_queued_frames():
    context = zmq.Context()
    sender = context.socket(zmq.PAIR)
    receiver = context.socket(zmq.PAIR)
    endpoint = "inproc://latest-value-test"
    sender.bind(endpoint)
    receiver.connect(endpoint)
    codec = ProtocolCodec()
    subscriber = LatestValueSubscriber(receiver, codec=codec)
    try:
        sender.send_multipart(codec.encode(frame("frame-1")))
        sender.send_multipart(codec.encode(frame("frame-2")))
        sender.send_multipart(codec.encode(frame("frame-3")))

        _, result, discarded = subscriber.receive_latest()

        assert isinstance(result, FrameDetections)
        assert result.frame_id == "frame-3"
        assert discarded == 2
    finally:
        receiver.close(0)
        sender.close(0)
        context.term()


def test_router_dealer_preserves_identity_and_typed_ack():
    context = zmq.Context()
    endpoint = "inproc://router-dealer-test"
    router = RouterTransport.open(context, endpoint, bind=True)
    dealer = DealerTransport.open(context, endpoint, identity="sam3-worker")
    try:
        dealer.send(frame())
        identity, _, request = router.receive()
        assert identity == b"sam3-worker"
        assert isinstance(request, FrameDetections)

        router.send(
            identity,
            Acknowledgement(request_id=request.request_id, accepted=True),
        )
        _, reply = dealer.receive()
        assert reply == Acknowledgement(request_id="mask-frame-1", accepted=True)
    finally:
        dealer.close()
        router.close()
        context.term()


def test_reconstruction_end_round_trips_without_binary_parts():
    message = ReconstructionEnd("take:end", "take", 500)
    encoded = ProtocolCodec().encode(message)

    assert len(encoded) == 2
    assert ProtocolCodec().decode(encoded) == ("reconstruction_end", message)

import json

import pytest
import zmq

from src.realtime import (
    Acknowledgement,
    AssetEvent,
    AssetState,
    BoxPrompt,
    DealerTransport,
    FrameDetections,
    LatestValueSubscriber,
    MaskBatch,
    MaskResult,
    ProtocolCodec,
    ProtocolError,
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
                visibility=0.9,
            ),
        ),
    )


@pytest.mark.parametrize(
    "message",
    [
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
    envelope["protocol_version"] = 2
    parts[1] = json.dumps(envelope).encode()

    with pytest.raises(ProtocolError, match="unsupported protocol version"):
        codec.decode(parts)
    with pytest.raises(ProtocolError, match="expects 1 binary parts"):
        codec.decode(codec.encode(frame())[:-1])


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
    with pytest.raises(ValueError, match="aligned_mesh_path"):
        AssetEvent(
            request_id="request",
            scene_generation="scene:1",
            track_id="car",
            state=AssetState.READY,
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

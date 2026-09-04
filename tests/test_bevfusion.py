from __future__ import annotations

from threading import Thread
import importlib.util
from pathlib import Path

import pytest
import zmq

from src.realtime.bevfusion import (
    BevFusionBox, BevFusionClient, BevFusionCodec, BevFusionProtocolError,
    BevFusionRequest, BevFusionResult,
)


IDENTITY = (1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)


def request() -> BevFusionRequest:
    return BevFusionRequest("scene:3:3", "scene", 3, "003", 300_000, "nuscenes-front",
        (1, 0, 0, 0, 1, 0, 0, 0, 1), IDENTITY, IDENTITY, 2, 1, b"\x01\x02\x03\x04\x05\x06", 2,
        b"\x00" * 32)


def test_request_and_result_multipart_round_trip():
    codec = BevFusionCodec()
    value = request()
    assert codec.decode_request(codec.encode_request(value)) == value
    result = BevFusionResult(value.request_id, value.scene_generation, value.sequence, value.frame_id,
        value.timestamp_us, value.config_id, (BevFusionBox((1, 2, 3), (4, 2, 1.5), 0.2, 0.9),), {"inference": 9.1})
    assert codec.decode_result(codec.encode_result(result)) == result


def test_docker_wire_decoder_accepts_the_host_codec():
    wire_path = Path(__file__).parents[1] / "docker" / "bevfusion" / "bevfusion_wire.py"
    spec = importlib.util.spec_from_file_location("docker_bevfusion_wire", wire_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    decoded = module.BevFusionCodec().decode_request(
        BevFusionCodec().encode_request(request())
    )

    assert decoded.request_id == request().request_id
    assert decoded.image_bgr == request().image_bgr
    assert decoded.lidar_f32 == request().lidar_f32


def test_request_rejects_non_raw_bgr_and_wrong_lidar_shape():
    with pytest.raises(ValueError, match="width"):
        BevFusionRequest("id", "scene", 0, "frame", 0, "cfg", (1,) * 9, IDENTITY, IDENTITY, 2, 1, b"bad", 0, b"")
    with pytest.raises(ValueError, match="lidar_f32"):
        BevFusionRequest("id", "scene", 0, "frame", 0, "cfg", (1,) * 9, IDENTITY, IDENTITY, 1, 1, b"\0\0\0", 1, b"\0" * 15)


def test_client_requires_exact_result_provenance():
    context = zmq.Context(); endpoint = "inproc://bevfusion-client"
    router = context.socket(zmq.ROUTER); router.bind(endpoint)
    client = BevFusionClient.open(context, endpoint, identity="test", timeout_ms=200)
    codec = BevFusionCodec()
    def reply() -> None:
        identity, *parts = router.recv_multipart(); sent = codec.decode_request(parts)
        wrong = BevFusionResult("wrong", sent.scene_generation, sent.sequence, sent.frame_id, sent.timestamp_us, sent.config_id)
        router.send_multipart([identity, *codec.encode_result(wrong)])
    thread = Thread(target=reply); thread.start()
    try:
        with pytest.raises(BevFusionProtocolError, match="provenance"):
            client.infer(request())
    finally:
        thread.join(); client.close(); router.close(linger=0); context.term()


def test_client_submit_and_zero_timeout_poll():
    context = zmq.Context(); endpoint = "inproc://bevfusion-submit-poll"
    router = context.socket(zmq.ROUTER); router.bind(endpoint)
    client = BevFusionClient.open(context, endpoint, identity="test", timeout_ms=200)
    try:
        assert client.submit(request()) is True
        assert client.poll(timeout_ms=0) is None
        assert client.poll() is None
    finally:
        client.close(); router.close(linger=0); context.term()


def test_client_infer_reports_timeout():
    context = zmq.Context(); endpoint = "inproc://bevfusion-timeout"
    router = context.socket(zmq.ROUTER); router.bind(endpoint)
    client = BevFusionClient.open(context, endpoint, identity="test", timeout_ms=1)
    try:
        with pytest.raises(TimeoutError, match="scene:3:3"):
            client.infer(request())
    finally:
        client.close(); router.close(linger=0); context.term()

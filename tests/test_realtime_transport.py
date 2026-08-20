"""Integration coverage for the authoritative receiver-pulled frame stream."""

from __future__ import annotations

from threading import Event, Thread
import time

import zmq

from src.deployment.vehicle import VehicleFrameSession, VehicleFrameSpool
from src.realtime import (
    BoxPrompt,
    DealerTransport,
    FrameAck,
    FrameHello,
    FrameSnapshot,
    FrameDetections,
    PublisherTransport,
    RouterTransport,
    SubscriberTransport,
)


IDENTITY_4 = tuple(
    float(value) for value in (1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)
)


def _snapshot(sequence: int) -> FrameSnapshot:
    return FrameSnapshot(
        sequence=sequence,
        timestamp_us=1_000_000 + sequence * 100_000,
        scene_generation="scene-0061:1",
        frame_id=f"frame-{sequence}",
        world_T_ego=IDENTITY_4,
        boxes=(BoxPrompt(
            track_id=f"car-{sequence}",
            xyxy=(float(sequence), 2.0, float(sequence + 20), 30.0),
            dimensions_lwh=(4.5, 1.8, 1.5),
            world_T_object=(
                1, 0, 0, float(sequence), 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,
            ),
            text_label="car",
        ),),
    )


def _analysis_frame(sequence: int) -> FrameDetections:
    snapshot = _snapshot(sequence)
    return FrameDetections(
        frame_id=snapshot.frame_id,
        request_id=f"request-{sequence}",
        scene_generation=snapshot.scene_generation,
        timestamp_us=snapshot.timestamp_us,
        image_jpeg=b"synthetic-jpeg",
        camera_intrinsic=(1, 0, 0, 0, 1, 0, 0, 0, 1),
        world_T_camera=IDENTITY_4,
        world_T_ego=IDENTITY_4,
        boxes=snapshot.boxes,
        source_sequence=sequence,
    )


def test_router_dealer_strict_pull_renders_all_501_snapshots_with_exact_boxes(tmp_path):
    """A slow remote cannot make the vehicle skip, coalesce, or mutate a frame."""

    context = zmq.Context()
    endpoint = "inproc://strict-pull-501-frames"
    router = RouterTransport.open(context, endpoint, bind=True, high_water_mark=1)
    spool = VehicleFrameSpool(tmp_path / "spool")
    expected = [_snapshot(sequence) for sequence in range(501)]
    for snapshot in expected:
        spool.append(snapshot)
    session = VehicleFrameSession(router, spool)
    stopping = Event()

    def serve() -> None:
        router.socket.setsockopt(zmq.RCVTIMEO, 20)
        while not stopping.is_set():
            try:
                session.serve_once()
            except zmq.Again:
                continue

    server = Thread(target=serve, daemon=True)
    server.start()
    remote = DealerTransport.open(
        context, endpoint, identity="strict-renderer", high_water_mark=1
    )
    remote.socket.setsockopt(zmq.RCVTIMEO, 2_000)
    acknowledged: list[FrameAck] = []
    received: list[FrameSnapshot] = []
    try:
        for sequence, wanted in enumerate(expected):
            remote.send(FrameHello(wanted.scene_generation, "strict-renderer", sequence))
            _, snapshot = remote.receive()
            assert isinstance(snapshot, FrameSnapshot)
            # Make the consumer slower than the server. Window-one delivery
            # must apply backpressure instead of filling/dropping a queue.
            time.sleep(0.001)
            assert snapshot == wanted
            received.append(snapshot)
            ack = FrameAck(snapshot.scene_generation, snapshot.sequence, snapshot.frame_id, snapshot.sha256)
            remote.send(ack)
            acknowledged.append(ack)

        deadline = time.monotonic() + 2.0
        while session._in_flight and time.monotonic() < deadline:
            time.sleep(0.005)

        assert [snapshot.sequence for snapshot in received] == list(range(501))
        assert [snapshot.boxes for snapshot in received] == [snapshot.boxes for snapshot in expected]
        assert [ack.sequence for ack in acknowledged] == list(range(501))
        assert not session._in_flight
    finally:
        remote.close()
        stopping.set()
        server.join(timeout=2.0)
        router.close()
        context.term()


def test_ordered_local_analysis_subscriber_receives_501_slow_frames_without_drain():
    """SAM/SAM3D inputs retain every bounded queued frame in source order."""

    context = zmq.Context()
    endpoint = "inproc://ordered-local-analysis-501-frames"
    publisher = PublisherTransport.open(context, endpoint, bind=True, high_water_mark=2_048)
    subscriber = SubscriberTransport.open(
        context, endpoint, bind=False, topics=("frame_detections",), high_water_mark=2_048
    )
    subscriber.socket.setsockopt(zmq.RCVTIMEO, 2_000)
    expected = [_analysis_frame(sequence) for sequence in range(501)]
    try:
        # SUB subscription establishment is asynchronous. This is not part of
        # the authoritative cross-host path; it models the bounded local work
        # queues after both workers have started.
        time.sleep(0.05)
        for frame in expected:
            publisher.send(frame)

        received: list[FrameDetections] = []
        for _ in expected:
            _, frame = subscriber.receive()
            assert isinstance(frame, FrameDetections)
            time.sleep(0.001)
            received.append(frame)

        assert [frame.source_sequence for frame in received] == list(range(501))
        assert [frame.frame_id for frame in received] == [frame.frame_id for frame in expected]
        assert [frame.boxes for frame in received] == [frame.boxes for frame in expected]
    finally:
        subscriber.close()
        publisher.close()
        context.term()

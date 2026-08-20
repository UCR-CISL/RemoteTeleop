from __future__ import annotations

import cv2
import numpy as np

from src.realtime.durable_analysis import DurableAnalysisSubscriber
from src.realtime.protocol import FrameAck, FrameDetections, FrameEnd, FrameHello, FramePending


class _Socket:
    def __init__(self, owner):
        self.owner = owner

    def poll(self, timeout=0):
        del timeout
        return bool(self.owner.incoming)


class _Dealer:
    def __init__(self):
        self.incoming = []
        self.sent = []
        self.socket = _Socket(self)

    def send(self, message, *, topic=None):
        self.sent.append(message)

    def receive(self, *, flags=0):
        del flags
        return "", self.incoming.pop(0)

    def close(self):
        pass


def _frame(sequence=0):
    ok, jpeg = cv2.imencode(".jpg", np.zeros((2, 3, 3), dtype=np.uint8))
    assert ok
    identity = (1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)
    return FrameDetections(
        frame_id=f"f{sequence}", request_id=f"r{sequence}", scene_generation="take",
        timestamp_us=sequence * 100_000, image_jpeg=jpeg.tobytes(),
        camera_intrinsic=(1, 0, 1, 0, 1, 1, 0, 0, 1),
        world_T_camera=identity, world_T_ego=identity, source_sequence=sequence,
    )


def test_durable_analysis_subscriber_retries_pending_and_acks_after_processing():
    dealer = _Dealer()
    source = DurableAnalysisSubscriber(
        dealer, scene_generation="take", receiver_id="sam3-mask"
    )

    assert not source.poll()
    assert dealer.sent == [FrameHello("take", "sam3-mask", 0)]
    dealer.incoming.append(FramePending("take", 0))
    assert not source.poll()

    frame = _frame()
    dealer.incoming.append(frame)
    assert source.poll()
    assert source.receive()[1] == frame
    assert not any(isinstance(message, FrameAck) for message in dealer.sent)
    source.acknowledge(frame)
    assert dealer.sent[-1] == FrameAck("take", 0, "f0", frame.sha256)

    dealer.incoming.append(FrameEnd("take", 0, 0))
    assert source.poll()
    end = source.receive()[1]
    source.acknowledge(end)
    assert not source.poll()

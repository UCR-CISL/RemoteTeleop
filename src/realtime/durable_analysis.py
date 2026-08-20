"""Receiver-pulled local analysis frames for slow SAM consumers."""

from __future__ import annotations

from typing import Protocol

import zmq

from src.realtime.protocol import (
    FrameAck,
    FrameDetections,
    FrameEnd,
    FrameHello,
    FramePending,
    WireMessage,
)
from src.realtime.transport import DealerTransport


class DurableAnalysisError(RuntimeError):
    """Raised when the durable local analysis sequence is inconsistent."""


class AnalysisFrameSource(Protocol):
    socket: object

    def poll(self, timeout: int) -> bool: ...

    def receive(self) -> tuple[str, WireMessage]: ...

    def acknowledge(self, message: WireMessage) -> None: ...

    def close(self) -> None: ...


class DurableAnalysisSubscriber:
    """Pull exactly one durable image-bearing frame and ACK after processing."""

    def __init__(
        self,
        transport: DealerTransport,
        *,
        scene_generation: str,
        receiver_id: str,
        next_sequence: int = 0,
    ) -> None:
        if not scene_generation or not receiver_id:
            raise ValueError("scene_generation and receiver_id must be non-empty")
        if next_sequence < 0:
            raise ValueError("next_sequence must be non-negative")
        self.transport = transport
        self.scene_generation = scene_generation
        self.receiver_id = receiver_id
        self.next_sequence = next_sequence
        self._request_outstanding = False
        self._received: tuple[str, WireMessage] | None = None
        self._complete = False

    @property
    def socket(self) -> object:
        return self.transport.socket

    @classmethod
    def open(
        cls,
        context: zmq.Context,
        endpoint: str,
        *,
        scene_generation: str,
        receiver_id: str,
        high_water_mark: int = 1,
    ) -> "DurableAnalysisSubscriber":
        return cls(
            DealerTransport.open(
                context,
                endpoint,
                identity=receiver_id,
                high_water_mark=high_water_mark,
            ),
            scene_generation=scene_generation,
            receiver_id=receiver_id,
        )

    def poll(self, timeout: int = 0) -> bool:
        if self._received is not None:
            return True
        if self._complete:
            return False
        if not self._request_outstanding:
            self.transport.send(
                FrameHello(self.scene_generation, self.receiver_id, self.next_sequence)
            )
            self._request_outstanding = True
        if not self.transport.socket.poll(timeout=timeout):
            return False
        topic, message = self.transport.receive()
        self._request_outstanding = False
        if isinstance(message, FramePending):
            if (
                message.scene_generation != self.scene_generation
                or message.next_sequence != self.next_sequence
            ):
                raise DurableAnalysisError("pending response does not match requested frame")
            return False
        if isinstance(message, FrameEnd):
            if (
                message.scene_generation != self.scene_generation
                or message.final_sequence + 1 != self.next_sequence
            ):
                raise DurableAnalysisError("frame end does not match analysis cursor")
            self._received = (topic, message)
            return True
        if not isinstance(message, FrameDetections):
            raise DurableAnalysisError(
                f"expected durable FrameDetections, got {type(message).__name__}"
            )
        if (
            message.scene_generation != self.scene_generation
            or message.source_sequence != self.next_sequence
        ):
            raise DurableAnalysisError("analysis frame does not match requested cursor")
        self._received = (topic, message)
        return True

    def receive(self) -> tuple[str, WireMessage]:
        if self._received is None:
            raise DurableAnalysisError("receive called without a ready analysis frame")
        result = self._received
        self._received = None
        return result

    def acknowledge(self, message: WireMessage) -> None:
        if isinstance(message, FrameEnd):
            self._complete = True
            return
        if not isinstance(message, FrameDetections):
            raise DurableAnalysisError("only analysis frames can be acknowledged")
        sequence = message.source_sequence
        if sequence != self.next_sequence:
            raise DurableAnalysisError("analysis acknowledgement is out of order")
        self.transport.send(
            FrameAck(
                message.scene_generation,
                sequence,
                message.frame_id,
                message.sha256,
            )
        )
        self.next_sequence += 1

    def close(self) -> None:
        self.transport.close()

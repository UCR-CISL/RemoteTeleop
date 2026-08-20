"""Small ZeroMQ transports built around :mod:`src.realtime.protocol`."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import zmq

from src.realtime.protocol import ProtocolCodec, WireMessage


class _SocketOwner:
    def __init__(self, socket: zmq.Socket[Any], *, owns_socket: bool) -> None:
        self.socket = socket
        self._owns_socket = owns_socket

    def close(self, *, linger_ms: int = 0) -> None:
        if self._owns_socket:
            self.socket.close(linger=linger_ms)

    def __enter__(self) -> "_SocketOwner":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def _configured_socket(
    context: zmq.Context[Any],
    socket_type: int,
    endpoint: str,
    *,
    bind: bool,
    high_water_mark: int,
) -> zmq.Socket[Any]:
    if high_water_mark <= 0:
        raise ValueError("high_water_mark must be positive")
    socket = context.socket(socket_type)
    socket.setsockopt(zmq.SNDHWM, high_water_mark)
    socket.setsockopt(zmq.RCVHWM, high_water_mark)
    if bind:
        socket.bind(endpoint)
    else:
        socket.connect(endpoint)
    return socket


class PublisherTransport(_SocketOwner):
    """Publish typed latest-value events over a PUB socket."""

    def __init__(
        self,
        socket: zmq.Socket[Any],
        *,
        codec: ProtocolCodec | None = None,
        owns_socket: bool = False,
    ) -> None:
        super().__init__(socket, owns_socket=owns_socket)
        self.codec = codec or ProtocolCodec()

    @classmethod
    def open(
        cls,
        context: zmq.Context[Any],
        endpoint: str,
        *,
        bind: bool,
        high_water_mark: int = 10,
    ) -> "PublisherTransport":
        return cls(
            _configured_socket(
                context, zmq.PUB, endpoint, bind=bind, high_water_mark=high_water_mark
            ),
            owns_socket=True,
        )

    def send(self, message: WireMessage, *, topic: str | None = None) -> None:
        self.socket.send_multipart(self.codec.encode(message, topic=topic))


class LatestValueSubscriber(_SocketOwner):
    """Receive the newest available event and discard older queued values."""

    def __init__(
        self,
        socket: zmq.Socket[Any],
        *,
        codec: ProtocolCodec | None = None,
        owns_socket: bool = False,
    ) -> None:
        super().__init__(socket, owns_socket=owns_socket)
        self.codec = codec or ProtocolCodec()

    @classmethod
    def open(
        cls,
        context: zmq.Context[Any],
        endpoint: str,
        *,
        bind: bool,
        topics: Sequence[str] = ("",),
        high_water_mark: int = 10,
    ) -> "LatestValueSubscriber":
        socket = _configured_socket(
            context, zmq.SUB, endpoint, bind=bind, high_water_mark=high_water_mark
        )
        for topic in topics:
            socket.setsockopt(zmq.SUBSCRIBE, topic.encode("utf-8"))
        return cls(socket, owns_socket=True)

    def receive_latest(
        self, *, flags: int = 0
    ) -> tuple[str, WireMessage, int]:
        parts = self.socket.recv_multipart(flags=flags)
        discarded = 0
        while True:
            try:
                parts = self.socket.recv_multipart(flags=zmq.NOBLOCK)
                discarded += 1
            except zmq.Again:
                break
        topic, message = self.codec.decode(parts)
        return topic, message, discarded


class SubscriberTransport(_SocketOwner):
    """Receive every published event without latest-value coalescing."""

    def __init__(
        self,
        socket: zmq.Socket[Any],
        *,
        codec: ProtocolCodec | None = None,
        owns_socket: bool = False,
    ) -> None:
        super().__init__(socket, owns_socket=owns_socket)
        self.codec = codec or ProtocolCodec()

    @classmethod
    def open(
        cls,
        context: zmq.Context[Any],
        endpoint: str,
        *,
        bind: bool,
        topics: Sequence[str] = ("",),
        high_water_mark: int = 100,
    ) -> "SubscriberTransport":
        socket = _configured_socket(
            context, zmq.SUB, endpoint, bind=bind, high_water_mark=high_water_mark
        )
        for topic in topics:
            socket.setsockopt(zmq.SUBSCRIBE, topic.encode("utf-8"))
        return cls(socket, owns_socket=True)

    def receive(self, *, flags: int = 0) -> tuple[str, WireMessage]:
        return self.codec.decode(self.socket.recv_multipart(flags=flags))

    def poll(self, timeout: int = 0) -> bool:
        return bool(self.socket.poll(timeout=timeout))

    def acknowledge(self, message: WireMessage) -> None:
        """Legacy PUB/SUB sources have no acknowledgement channel."""

        del message


class DealerTransport(_SocketOwner):
    """Send typed requests and receive acknowledgements/results."""

    def __init__(
        self,
        socket: zmq.Socket[Any],
        *,
        codec: ProtocolCodec | None = None,
        owns_socket: bool = False,
    ) -> None:
        super().__init__(socket, owns_socket=owns_socket)
        self.codec = codec or ProtocolCodec()

    @classmethod
    def open(
        cls,
        context: zmq.Context[Any],
        endpoint: str,
        *,
        identity: str,
        bind: bool = False,
        high_water_mark: int = 10,
    ) -> "DealerTransport":
        if high_water_mark <= 0:
            raise ValueError("high_water_mark must be positive")
        socket = context.socket(zmq.DEALER)
        socket.setsockopt(zmq.SNDHWM, high_water_mark)
        socket.setsockopt(zmq.RCVHWM, high_water_mark)
        socket.setsockopt(zmq.IDENTITY, identity.encode("utf-8"))
        if bind:
            socket.bind(endpoint)
        else:
            socket.connect(endpoint)
        return cls(socket, owns_socket=True)

    def send(self, message: WireMessage, *, topic: str | None = None) -> None:
        self.socket.send_multipart(self.codec.encode(message, topic=topic))

    def receive(self, *, flags: int = 0) -> tuple[str, WireMessage]:
        return self.codec.decode(self.socket.recv_multipart(flags=flags))


class RouterTransport(_SocketOwner):
    """Receive routed requests and reply to their stable ZMQ identities."""

    def __init__(
        self,
        socket: zmq.Socket[Any],
        *,
        codec: ProtocolCodec | None = None,
        owns_socket: bool = False,
    ) -> None:
        super().__init__(socket, owns_socket=owns_socket)
        self.codec = codec or ProtocolCodec()

    @classmethod
    def open(
        cls,
        context: zmq.Context[Any],
        endpoint: str,
        *,
        bind: bool = True,
        high_water_mark: int = 10,
    ) -> "RouterTransport":
        socket = _configured_socket(
            context, zmq.ROUTER, endpoint, bind=bind, high_water_mark=high_water_mark
        )
        # Do not silently queue a reply for a receiver that has disconnected.
        # The caller must retain the durable source and wait for a new hello.
        socket.setsockopt(zmq.ROUTER_MANDATORY, 1)
        return cls(socket, owns_socket=True)

    def receive(self, *, flags: int = 0) -> tuple[bytes, str, WireMessage]:
        parts = self.socket.recv_multipart(flags=flags)
        if len(parts) < 3:
            raise ValueError("routed message requires identity, topic, and metadata")
        topic, message = self.codec.decode(parts[1:])
        return bytes(parts[0]), topic, message

    def send(
        self,
        identity: bytes,
        message: WireMessage,
        *,
        topic: str | None = None,
    ) -> None:
        self.socket.send_multipart([bytes(identity), *self.codec.encode(message, topic=topic)])

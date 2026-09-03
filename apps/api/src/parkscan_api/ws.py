"""WebSocket fan-out for ``/ws/live``.

The interesting part is not that this broadcasts — it is what it does when a
browser stops reading.

Each connected client gets **its own bounded queue** (default 100). ``broadcast``
is synchronous and never awaits, so the MQTT ingest task cannot be blocked by a
slow consumer. On overflow the *oldest* frame is discarded, not the newest: a
live map wants the most recent pose, and dropping the newest would freeze the
display at the moment of congestion. Dropped frames are counted and reported to
that client as ``{"type":"lag","dropped":N}`` before the next real frame, so data
loss is visible rather than silent (docs/API_CONTRACT.md).

Each client is drained by its **own task**, and every send is wrapped in
``asyncio.wait_for``. One stalled tab therefore fails alone: it is disconnected
after the send timeout while every other client keeps receiving.

No Redis. At three robots and a single uvicorn worker that would be architecture
theatre; the fan-out is in-process and the README says so.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
from collections.abc import Iterable, Mapping
from typing import Any, Protocol, runtime_checkable

import structlog

LAG_TYPE = "lag"
DEFAULT_MAXSIZE = 100
DEFAULT_SEND_TIMEOUT_S = 5.0

log: structlog.stdlib.BoundLogger = structlog.get_logger("parkscan.ws")


@runtime_checkable
class WebSocketSink(Protocol):
    """The only thing this module needs from a WebSocket.

    Narrow on purpose: the queueing behaviour is testable with a plain object
    and no server, which is the point of the tests in ``tests/test_ws.py``.
    """

    async def send_text(self, data: str) -> None: ...


def encode(envelope: Mapping[str, Any]) -> str:
    """Serialise one envelope. Newline-free, per the API contract."""
    return json.dumps(envelope, separators=(",", ":"), default=str)


class ClientChannel:
    """One connected client: a bounded queue, a drop counter, a topic filter."""

    __slots__ = ("_dropped", "_maxsize", "_topics", "queue", "sink")

    def __init__(self, sink: WebSocketSink, *, maxsize: int = DEFAULT_MAXSIZE) -> None:
        self.sink = sink
        self._maxsize = maxsize
        self.queue: asyncio.Queue[str] = asyncio.Queue(maxsize=maxsize)
        self._dropped = 0
        self._topics: frozenset[str] | None = None

    @property
    def maxsize(self) -> int:
        return self._maxsize

    @property
    def dropped(self) -> int:
        """Frames discarded since the last ``lag`` notice was emitted."""
        return self._dropped

    @property
    def topics(self) -> frozenset[str] | None:
        """``None`` means "everything" — the default until the client subscribes."""
        return self._topics

    def subscribe(self, topics: Iterable[str] | None) -> None:
        self._topics = None if topics is None else frozenset(topics)

    def wants(self, envelope_type: str) -> bool:
        return self._topics is None or envelope_type in self._topics

    def offer(self, message: str) -> None:
        """Enqueue without ever blocking, discarding the oldest frame on overflow."""
        try:
            self.queue.put_nowait(message)
            return
        except asyncio.QueueFull:
            pass
        try:
            self.queue.get_nowait()
        except asyncio.QueueEmpty:  # pragma: no cover - only under a concurrent drain
            pass
        else:
            self._dropped += 1
        try:
            self.queue.put_nowait(message)
        except asyncio.QueueFull:  # pragma: no cover - only under a concurrent producer
            self._dropped += 1

    def take_dropped(self) -> int:
        """Read and reset the drop counter."""
        dropped, self._dropped = self._dropped, 0
        return dropped


class ConnectionManager:
    """In-process fan-out to every open ``/ws/live`` socket."""

    def __init__(
        self,
        *,
        maxsize: int = DEFAULT_MAXSIZE,
        send_timeout_s: float = DEFAULT_SEND_TIMEOUT_S,
    ) -> None:
        self._maxsize = maxsize
        self._send_timeout_s = send_timeout_s
        self._writers: dict[ClientChannel, asyncio.Task[None]] = {}

    @property
    def client_count(self) -> int:
        return len(self._writers)

    def connect(self, sink: WebSocketSink) -> ClientChannel:
        """Register a client and start its private writer task."""
        channel = ClientChannel(sink, maxsize=self._maxsize)
        self._writers[channel] = asyncio.create_task(
            self._drain(channel), name=f"ws-writer-{id(channel):x}"
        )
        return channel

    async def disconnect(self, channel: ClientChannel) -> None:
        """Stop and await that client's writer. Safe to call twice."""
        task = self._writers.pop(channel, None)
        if task is None or task.done():
            return
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task

    def broadcast(self, envelope: Mapping[str, Any]) -> None:
        """Queue one envelope for every interested client. Never blocks."""
        envelope_type = str(envelope.get("type", ""))
        message = encode(envelope)
        for channel in list(self._writers):
            if channel.wants(envelope_type):
                channel.offer(message)

    async def close(self) -> None:
        """Tear down every writer. Used by the lifespan on shutdown."""
        for channel in list(self._writers):
            await self.disconnect(channel)

    async def _drain(self, channel: ClientChannel) -> None:
        """Per-client writer. A failure here removes only this client."""
        try:
            while True:
                message = await channel.queue.get()
                dropped = channel.take_dropped()
                if dropped:
                    await self._send(channel, encode({"type": LAG_TYPE, "dropped": dropped}))
                await self._send(channel, message)
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # includes TimeoutError from _send
            log.info("ws_client_dropped", reason=type(exc).__name__, detail=str(exc)[:200])
            self._writers.pop(channel, None)

    async def _send(self, channel: ClientChannel, message: str) -> None:
        await asyncio.wait_for(channel.sink.send_text(message), timeout=self._send_timeout_s)

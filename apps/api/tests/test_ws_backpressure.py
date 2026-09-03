"""The WebSocket fan-out's backpressure behaviour.

This is the part of the WebSocket layer worth testing hardest, and it needs no
services at all: one slow browser tab must not be able to stall the broadcast to
every other viewer, and an overflowing client must lose the OLDEST frames (stale
robot poses) rather than the newest, and must be told how many it lost.
"""

from __future__ import annotations

import asyncio

import pytest

from parkscan_api.ws import ClientChannel, ConnectionManager, encode


class RecordingSink:
    """A sink that records what it was sent."""

    def __init__(self) -> None:
        self.sent: list[str] = []

    async def send_text(self, data: str) -> None:
        self.sent.append(data)


class StalledSink:
    """A browser tab that has stopped reading. Never completes a send."""

    def __init__(self) -> None:
        self.attempts = 0

    async def send_text(self, data: str) -> None:
        self.attempts += 1
        await asyncio.Event().wait()  # never returns


def test_offer_never_blocks_and_drops_oldest_first() -> None:
    channel = ClientChannel(RecordingSink(), maxsize=3)

    for i in range(5):
        channel.offer(f"msg-{i}")

    assert channel.queue.qsize() == 3
    drained = [channel.queue.get_nowait() for _ in range(3)]

    # The two OLDEST were discarded, not the newest. A live dashboard wants the
    # freshest pose; keeping stale frames and dropping current ones would be worse
    # than useless.
    assert drained == ["msg-2", "msg-3", "msg-4"]
    assert channel.dropped == 2


def test_dropped_counter_is_read_and_reset() -> None:
    channel = ClientChannel(RecordingSink(), maxsize=1)
    channel.offer("a")
    channel.offer("b")
    channel.offer("c")

    assert channel.take_dropped() == 2
    # Reset, so the next lag notice reports only new drops rather than a running total.
    assert channel.take_dropped() == 0


def test_queue_bound_is_respected_under_a_large_burst() -> None:
    channel = ClientChannel(RecordingSink(), maxsize=10)
    for i in range(1_000):
        channel.offer(f"m{i}")

    assert channel.queue.qsize() == 10
    assert channel.dropped == 990


async def test_one_stalled_client_does_not_block_the_others() -> None:
    """The whole point of a task-per-client with a timeout."""
    manager = ConnectionManager(maxsize=8, send_timeout_s=0.05)
    stalled = StalledSink()
    healthy = RecordingSink()

    manager.connect(stalled)
    manager.connect(healthy)

    for i in range(3):
        manager.broadcast({"type": "telemetry", "payload": {"i": i}})

    await asyncio.sleep(0.3)

    # The healthy client received everything despite the other tab being wedged.
    assert len(healthy.sent) == 3, f"healthy client starved: {healthy.sent}"
    await manager.close()


async def test_subscription_filter_only_delivers_requested_types() -> None:
    manager = ConnectionManager(maxsize=8)
    sink = RecordingSink()
    channel = manager.connect(sink)
    channel.subscribe(["alerts"])

    manager.broadcast({"type": "telemetry", "payload": {}})
    manager.broadcast({"type": "alerts", "payload": {"kind": "battery_critical"}})
    await asyncio.sleep(0.1)

    assert len(sink.sent) == 1
    assert "battery_critical" in sink.sent[0]
    await manager.close()


def test_encode_emits_newline_free_json() -> None:
    # The wire format is one JSON object per frame; an embedded newline would break
    # any line-oriented consumer downstream.
    out = encode({"type": "telemetry", "payload": {"note": "a\nb"}})
    assert "\n" not in out


@pytest.mark.parametrize("maxsize", [1, 2, 100])
def test_channel_reports_its_bound(maxsize: int) -> None:
    channel = ClientChannel(RecordingSink(), maxsize=maxsize)
    assert channel.maxsize == maxsize

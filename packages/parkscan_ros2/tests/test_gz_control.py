"""``GzControl``'s request-building and polling logic, tested without a real Gazebo.

The service-call semantics this class encodes (``multi_step`` must pair with ``pause:
true``; the reply is an accept, not a completion signal) were themselves found only by
running a real ``gz sim`` -- see ADR-0008 §3 and this package's git history. These tests
do not re-verify Gazebo's behaviour; they verify this class asks for the right thing and
waits correctly for confirmation, using a fake subprocess runner and a fake background
updater in place of the real stats-tailing thread.
"""

from __future__ import annotations

import threading
import time

import pytest
from parkscan_ros2.bridge.gz_control import GzControl, GzControlError


def _ok_runner(argv: list[str], timeout_s: float) -> tuple[int, str]:
    return 0, "data: true\n"


def _refusing_runner(argv: list[str], timeout_s: float) -> tuple[int, str]:
    return 0, "data: false\n"


def _failing_runner(argv: list[str], timeout_s: float) -> tuple[int, str]:
    return 1, "Service call timed out\n"


def test_step_sends_multi_step_paired_with_pause_true() -> None:
    """The exact bug ADR-0008 found: multi_step alone does not repause."""
    seen: list[list[str]] = []

    def capturing_runner(argv: list[str], timeout_s: float) -> tuple[int, str]:
        seen.append(argv)
        return 0, "data: true\n"

    gc = GzControl("test_world", runner=capturing_runner)
    gc._iterations = 100

    def delayed_bump() -> None:
        time.sleep(0.02)
        _bump(gc, 100 + 7)

    threading.Thread(target=delayed_bump).start()
    gc.step(7, poll_interval_s=0.005)

    assert len(seen) == 1
    req = seen[0][seen[0].index("--req") + 1]
    assert "multi_step: 7" in req
    assert "pause: true" in req


def test_step_blocks_until_iterations_actually_reaches_the_target() -> None:
    """This is the fix for the reply being an accept, not a completion signal: step()
    must not return the instant the service call comes back."""
    gc = GzControl("test_world", runner=_ok_runner)
    gc._iterations = 0
    returned_at: list[float] = []

    def delayed_bump() -> None:
        time.sleep(0.05)
        _bump(gc, 4)

    threading.Thread(target=delayed_bump).start()
    start = time.monotonic()
    gc.step(4, poll_interval_s=0.005)
    returned_at.append(time.monotonic() - start)

    assert returned_at[0] >= 0.04, "step() returned before iterations reached the target"


def test_step_raises_rather_than_hanging_forever_if_the_target_is_never_reached() -> None:
    """The lesson this project already paid for once: a blocking wait with no deadline
    is how a scripting mistake became a 3-hour hung Docker container. step() must fail
    loudly, bounded, rather than hang."""
    gc = GzControl("test_world", runner=_ok_runner, service_timeout_s=0.05)
    gc._iterations = 0
    with pytest.raises(GzControlError, match="never reached iteration"):
        gc.step(2, poll_interval_s=0.01)


def test_step_raises_if_the_service_call_itself_is_refused() -> None:
    gc = GzControl("test_world", runner=_refusing_runner)
    with pytest.raises(GzControlError, match="refused"):
        gc.step(1)


def test_step_raises_if_the_service_call_fails_outright() -> None:
    gc = GzControl("test_world", runner=_failing_runner)
    with pytest.raises(GzControlError, match="refused"):
        gc.step(1)


def test_step_rejects_a_non_positive_iteration_count() -> None:
    gc = GzControl("test_world", runner=_ok_runner)
    with pytest.raises(GzControlError, match="positive"):
        gc.step(0)


def test_reset_sends_reset_all_paired_with_pause_true_and_zeroes_iterations() -> None:
    seen: list[list[str]] = []

    def capturing_runner(argv: list[str], timeout_s: float) -> tuple[int, str]:
        seen.append(argv)
        return 0, "data: true\n"

    gc = GzControl("test_world", runner=capturing_runner)
    gc._iterations = 500
    gc.reset()

    req = seen[0][seen[0].index("--req") + 1]
    assert "reset: {all: true}" in req
    assert "pause: true" in req
    assert gc._current_iterations() == 0


def test_reset_raises_if_the_service_call_is_refused() -> None:
    gc = GzControl("test_world", runner=_refusing_runner)
    with pytest.raises(GzControlError, match="refused"):
        gc.reset()


def test_control_and_stats_topic_names_are_world_scoped() -> None:
    gc = GzControl("ParkScan_synthetic_lot_A", runner=_ok_runner)
    assert gc.control_topic == "/world/ParkScan_synthetic_lot_A/control"
    assert gc.stats_topic == "/world/ParkScan_synthetic_lot_A/stats"


def _bump(gc: GzControl, value: int) -> None:
    with gc._lock:
        gc._iterations = value

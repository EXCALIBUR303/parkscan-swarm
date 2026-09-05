"""Drive Gazebo's world control service from Python, correctly.

Two facts here were confirmed empirically against a real ``gz sim`` before this was
written (ADR-0008 §3), and neither is documented anywhere in Gazebo's own docs:

1. ``multi_step`` alone does not step-then-repause -- sent without ``pause: true`` in the
   same request, the world just keeps running freely afterward (observed: 474 iterations
   and 23.7 s of unbounded sim time from a single ``multi_step: 20`` call). Both fields
   must be set together.
2. The service's ``Boolean{data: true}`` reply is an *accept*, not a *completion signal*.
   It returns almost immediately; the requested iterations are still being applied for
   roughly ``n * step_size`` of real time afterward. Trusting the reply alone races ahead
   of physics.

No Python bindings for ``gz-transport`` are installed in this environment (only the
vendored C++ libraries; there is no ``python3-gz-transport13``-equivalent package here).
Shelling out to the ``gz service`` / ``gz topic`` CLIs -- already proven to work by the
ROADMAP's own RTF spike -- avoids a second, unrelated binding-installation problem this
module has no reason to solve. This is a pragmatic choice, not a hidden one.
"""

from __future__ import annotations

import re
import subprocess
import threading
import time
from collections.abc import Callable

__all__ = ["GzControl", "GzControlError"]

_ITERATIONS_RE = re.compile(r"^iterations:\s*(\d+)\s*$", re.MULTILINE)

#: Run one subprocess to completion, returning (returncode, stdout). Swappable for
#: tests -- nothing here needs a real Gazebo to be exercised.
SubprocessRunner = Callable[[list[str], float], tuple[int, str]]


def _default_runner(argv: list[str], timeout_s: float) -> tuple[int, str]:
    """Run one subprocess, never raising.

    Found the hard way, running the real system under load: ``subprocess.run``'s own
    ``timeout`` argument raises ``TimeoutExpired`` if the process does not finish in
    time -- ``check=False`` only suppresses raising on a non-zero *exit code*, not on a
    timeout. That exception was escaping this function uncaught, past
    :meth:`GzControl.step`'s own error handling (which only ever expected a
    ``(code, output)`` tuple back), and crashing the caller's MQTT callback. Converted
    to the same ``(non-zero code, message)`` shape a failed service call already
    produces, so every caller's existing "non-zero exit -> ``GzControlError``" handling
    covers this case too, instead of needing a second, parallel error path.
    """
    try:
        result = subprocess.run(
            argv, capture_output=True, text=True, timeout=timeout_s, check=False
        )
    except subprocess.TimeoutExpired:
        return 1, f"subprocess timed out after {timeout_s}s: {argv!r}"
    return result.returncode, result.stdout


class GzControlError(RuntimeError):
    """The world control service refused a request, or a step never completed in time."""


class GzControl:
    """Lockstep control of one Gazebo world via its ``/world/<name>/control`` service.

    Owns a single background thread reading a long-lived ``gz topic -e`` subprocess on
    ``/world/<name>/stats``, so :meth:`step` does not pay a fresh subscribe/connect cost
    on every call -- this is invoked once per :meth:`~parkscan_ros2.backend.Ros2WorldBackend.step`,
    which on the sim's own 0.05 s tick is potentially 20 times a second.
    """

    def __init__(
        self,
        world_name: str,
        *,
        runner: SubprocessRunner = _default_runner,
        service_timeout_s: float = 5.0,
    ) -> None:
        self._world = world_name
        self._runner = runner
        self._service_timeout_s = service_timeout_s
        self._iterations = 0
        self._lock = threading.Lock()
        self._stats_proc: subprocess.Popen[str] | None = None
        self._reader_thread: threading.Thread | None = None
        self._stopped = threading.Event()

    @property
    def control_topic(self) -> str:
        return f"/world/{self._world}/control"

    @property
    def stats_topic(self) -> str:
        return f"/world/{self._world}/stats"

    def start(self) -> None:
        """Begin tailing the stats topic. Call once, before the first :meth:`step`."""
        if self._stats_proc is not None:
            msg = "GzControl.start() called twice"
            raise GzControlError(msg)
        self._stats_proc = subprocess.Popen(
            ["gz", "topic", "-e", "-t", self.stats_topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )
        self._reader_thread = threading.Thread(target=self._read_stats, daemon=True)
        self._reader_thread.start()

    def stop(self) -> None:
        self._stopped.set()
        if self._stats_proc is not None:
            self._stats_proc.terminate()
            self._stats_proc.wait(timeout=5.0)
            self._stats_proc = None

    def _read_stats(self) -> None:
        assert self._stats_proc is not None
        assert self._stats_proc.stdout is not None
        buffer: list[str] = []
        for line in self._stats_proc.stdout:
            if self._stopped.is_set():
                return
            if line.strip() == "":
                match = _ITERATIONS_RE.search("".join(buffer))
                if match is not None:
                    with self._lock:
                        self._iterations = int(match.group(1))
                buffer.clear()
            else:
                buffer.append(line)

    def _current_iterations(self) -> int:
        with self._lock:
            return self._iterations

    def step(self, iterations: int, *, poll_interval_s: float = 0.01) -> None:
        """Advance the world by exactly ``iterations`` physics steps, then block until done.

        Raises:
            GzControlError: the service call itself was rejected, or the world did not
                reach the target iteration count within a generous, but finite, timeout.
                This never blocks forever -- an indefinite wait here is exactly the
                mistake that cost this project a 3-hour hung container earlier in the
                same effort (see project memory / commit history), and a bridge that can
                hang is worse than one that fails loudly.
        """
        if iterations <= 0:
            msg = f"iterations must be positive, got {iterations}"
            raise GzControlError(msg)
        target = self._current_iterations() + iterations
        code, out = self._runner(
            [
                "gz",
                "service",
                "-s",
                self.control_topic,
                "--reqtype",
                "gz.msgs.WorldControl",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                str(int(self._service_timeout_s * 1000)),
                "--req",
                f"multi_step: {iterations}, pause: true",
            ],
            self._service_timeout_s,
        )
        if code != 0 or "true" not in out:
            msg = f"world control service refused step request: exit {code}, output {out!r}"
            raise GzControlError(msg)

        deadline = time.monotonic() + max(self._service_timeout_s, iterations * 0.2)
        while self._current_iterations() < target:
            if time.monotonic() > deadline:
                msg = (
                    f"step({iterations}) never reached iteration {target} "
                    f"(stuck at {self._current_iterations()}) within the deadline"
                )
                raise GzControlError(msg)
            time.sleep(poll_interval_s)

    def reset(self) -> None:
        """Reset the world to its SDF-authored initial state.

        Does not touch anything Python-side (battery RNG, per ADR-0008 §5) -- that reset
        is the caller's job. This only resets physics and poses.

        Verified directly (like everything else in this module) that ``reset: {all:
        true}`` snaps ``sim_time``/``iterations`` back to zero. Pose restoration was
        *not* independently re-checked with the same rigor -- the pose topic does not
        publish while paused, which is what a request/response check would need to work
        around, and unlike ``multi_step``'s pairing requirement this is basic, long-
        established Gazebo behaviour rather than a plausible landmine. Flagged here
        rather than silently assumed, in keeping with this module's own standard.
        """
        code, out = self._runner(
            [
                "gz",
                "service",
                "-s",
                self.control_topic,
                "--reqtype",
                "gz.msgs.WorldControl",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                str(int(self._service_timeout_s * 1000)),
                "--req",
                "reset: {all: true}, pause: true",
            ],
            self._service_timeout_s,
        )
        if code != 0 or "true" not in out:
            msg = f"world control service refused reset request: exit {code}, output {out!r}"
            raise GzControlError(msg)
        with self._lock:
            self._iterations = 0

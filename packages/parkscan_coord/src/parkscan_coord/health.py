"""Failure detection: three independent mechanisms, and one judgement.

Why three
---------
A fleet fails in more than one way, and no single detector catches all of them. Each
mechanism here exists because the other two are blind to something:

1. **Liveness, from the broker's MQTT Last Will.** When a robot's TCP session drops, the
   broker publishes its retained will message and the coordinator learns about it with no
   polling at all. The coordinator *consumes* that as a :class:`ConnectionEvent`; it does
   **not** synthesise one. Emulating a last will in application code would mean the claim
   "the broker detects hard disconnects" is about code we wrote rather than about the
   broker, which would be worthless. :class:`ConnectionEvent` enforces the distinction
   structurally: a disconnect event that does not carry
   :attr:`LivenessSource.BROKER_LAST_WILL` is rejected at construction.

2. **Heartbeat deadline.** ``OFFLINE`` after *k* missed beats (``k = 3`` at 1 Hz). This is
   the one that catches a **hung** robot -- one that keeps its TCP session open, so the
   broker is perfectly happy and the will never fires, while the process has stopped
   making progress. Last Will cannot see that case; that is precisely why both exist.

3. **Stuck detection.** Over a sliding window: displacement below ``epsilon`` **and** the
   covered-cell count did not increase **and** the robot is not ``WAITING_ON_LEASE``.
   That last exclusion is load-bearing, not a nicety. Under ADR-0007 a robot waits for the
   *whole* resource set of its next leg, so legitimate queueing is both normal and
   stationary; without the exclusion the reservation system manufactures a stuck-detection
   every time a robot queues, and a demo would show the coordination layer "failing"
   at exactly the moments it is working correctly.

Judgement is kept separate from self-report
-------------------------------------------
The output is a :class:`~parkscan_core.types.CoordView` -- the coordinator's own opinion --
and it is deliberately **not** merged with the robot's self-reported
:class:`~parkscan_core.types.RobotState`. A robot that believes it is ``INSPECTING`` while
the coordinator has judged it ``OFFLINE`` is not a bug to be normalised away; it is the
exact divergence the recovery logic in :mod:`parkscan_coord.coordinator` exists to handle,
and ``docs/DATA_MODEL.md`` gives the two judgements separate columns for the same reason.

Determinism
-----------
Nothing here reads a clock, holds randomness, or depends on dict ordering. Every method
takes ``now_ns`` from the caller and every collection is returned in sorted order, so a
replay of the same event sequence produces the same judgements on every machine.

Latency, stated honestly
------------------------
:attr:`HealthConfig.detection_latency_s` is ``k x heartbeat_period``. It is a **configured
deadline**, not a measured achievement, and it must never be added to the reassignment
latency reported by :mod:`parkscan_coord.metrics` (ACCEPTANCE H6).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import StrEnum

from parkscan_agent.robot import TelemetrySample

from parkscan_core.types import CoordView, RobotId, RobotState

__all__ = [
    "NS_PER_S",
    "STUCK_EXEMPT_STATES",
    "ConnectionEvent",
    "HealthAssessment",
    "HealthConfig",
    "HealthMonitor",
    "LivenessSource",
]

NS_PER_S = 1_000_000_000
"""Nanoseconds per second. Every timestamp in this package is integer nanoseconds."""

STUCK_EXEMPT_STATES: frozenset[RobotState] = frozenset(
    {
        RobotState.WAITING_ON_LEASE,
        RobotState.CHARGING,
        RobotState.IDLE,
        RobotState.FAILED,
        RobotState.OFFLINE,
    }
)
"""States in which standing still is correct behaviour, so stuck detection is suppressed.

``WAITING_ON_LEASE`` is the one the architecture calls out, and the one that would
otherwise fire constantly: under ADR-0007 a robot that cannot get the complete resource
set for its next leg stops and queues, by design.

The other four are the same argument applied consistently. ``CHARGING`` is docked and
motionless on purpose. ``IDLE`` has no order, so it is not failing to make progress on
anything -- it is waiting to be given work. ``FAILED`` and ``OFFLINE`` have already been
judged, and re-labelling them ``STUCK`` would only lose information.
"""


class LivenessSource(StrEnum):
    """Where a connection event came from. The broker, in both cases."""

    BROKER_CONNECT = "broker_connect"
    """The broker reported a session opening (``$SYS`` / connect callback)."""

    BROKER_LAST_WILL = "broker_last_will"
    """The broker published the robot's retained Last Will and Testament.

    Only the broker can produce this. Application code that fabricated it would be
    claiming a detection mechanism it does not have.
    """


@dataclass(frozen=True, slots=True)
class ConnectionEvent:
    """One broker-originated liveness signal about one robot.

    The pairing of ``connected`` and ``source`` is validated rather than trusted: a
    disconnect must come from the Last Will and a connect from a session opening. That
    makes the honesty claim in the module docstring a property of the type, not of a
    comment somebody may later delete.
    """

    robot_id: RobotId
    stamp_ns: int
    connected: bool
    source: LivenessSource
    detail: str = ""

    def __post_init__(self) -> None:
        if not self.connected and self.source is not LivenessSource.BROKER_LAST_WILL:
            msg = (
                "a disconnect must be sourced from the broker's Last Will "
                f"({LivenessSource.BROKER_LAST_WILL.value!r}), got {self.source.value!r}; "
                "the coordinator does not emulate the will itself"
            )
            raise ValueError(msg)
        if self.connected and self.source is not LivenessSource.BROKER_CONNECT:
            msg = (
                "a connect must be sourced from a broker session opening "
                f"({LivenessSource.BROKER_CONNECT.value!r}), got {self.source.value!r}"
            )
            raise ValueError(msg)


@dataclass(frozen=True, slots=True)
class HealthConfig:
    """Tuning for all three detectors. Frozen, so a run's config is a hashable value."""

    heartbeat_period_ns: int = NS_PER_S
    """Expected telemetry period. 1 Hz, matching ``docs/ARCHITECTURE.md`` section 4."""

    missed_beats_offline: int = 3
    """``k``: consecutive missed beats before the verdict is ``OFFLINE``."""

    missed_beats_suspect: int = 1
    """Missed beats before the verdict is ``SUSPECT``. ``SUSPECT`` changes no assignment;
    it exists so the dashboard can show doubt before the coordinator acts on it."""

    stuck_window_ns: int = 10 * NS_PER_S
    """Sliding window over which "no progress" is judged."""

    stuck_epsilon_m: float = 0.25
    """Displacement below which the robot counts as not having moved."""

    stuck_min_samples: int = 3
    """Samples the window must contain before any stuck verdict. Two points cannot
    distinguish "stopped" from "sampled twice at the same instant"."""

    def __post_init__(self) -> None:
        if self.heartbeat_period_ns <= 0:
            msg = f"heartbeat_period_ns must be positive, got {self.heartbeat_period_ns}"
            raise ValueError(msg)
        if self.missed_beats_offline < 1:
            msg = f"missed_beats_offline must be at least 1, got {self.missed_beats_offline}"
            raise ValueError(msg)
        if not 1 <= self.missed_beats_suspect <= self.missed_beats_offline:
            msg = (
                "missed_beats_suspect must be in [1, missed_beats_offline], got "
                f"{self.missed_beats_suspect} with k={self.missed_beats_offline}"
            )
            raise ValueError(msg)
        if self.stuck_window_ns <= 0:
            msg = f"stuck_window_ns must be positive, got {self.stuck_window_ns}"
            raise ValueError(msg)
        if self.stuck_epsilon_m < 0.0:
            msg = f"stuck_epsilon_m must be non-negative, got {self.stuck_epsilon_m}"
            raise ValueError(msg)
        if self.stuck_min_samples < 2:
            msg = f"stuck_min_samples must be at least 2, got {self.stuck_min_samples}"
            raise ValueError(msg)

    @property
    def detection_deadline_ns(self) -> int:
        """``k x heartbeat_period``: how long silence lasts before the verdict flips."""
        return self.missed_beats_offline * self.heartbeat_period_ns

    @property
    def detection_latency_s(self) -> float:
        """The heartbeat detection deadline in seconds.

        **This is a configuration value, not a result.** It is exactly
        ``missed_beats_offline x heartbeat_period``, chosen by us; lowering it does not
        make the system better at detecting anything, it only trades false positives
        against delay. It is reported separately from ``reassignment_latency_s`` and the
        two are never summed (ACCEPTANCE H6).
        """
        return self.detection_deadline_ns / NS_PER_S


@dataclass(frozen=True, slots=True)
class HealthAssessment:
    """The coordinator's judgement of one robot at one instant, with its evidence.

    The evidence fields are here so a dashboard or a test failure message can say *why*
    a robot was declared dead, rather than only that it was.
    """

    robot_id: RobotId
    view: CoordView
    now_ns: int
    missed_beats: int
    last_beat_ns: int
    connected: bool
    latched_offline: bool
    displacement_m: float
    window_span_ns: int
    window_samples: int
    reason: str

    @property
    def usable(self) -> bool:
        """Whether the coordinator may hand this robot new work.

        ``SUSPECT`` counts as usable on purpose: one missed beat at 1 Hz is normal jitter,
        and pulling orders on it would thrash the fleet for no reason.
        """
        return self.view in (CoordView.ONLINE, CoordView.SUSPECT)


@dataclass(slots=True)
class _Obs:
    """One telemetry observation, reduced to what stuck detection needs."""

    stamp_ns: int
    x_m: float
    y_m: float
    covered_total: int
    state: RobotState


@dataclass(slots=True)
class _Track:
    """Everything the monitor remembers about one robot."""

    robot_id: RobotId
    last_beat_ns: int
    connected: bool = True
    latched_offline: bool = False
    latch_reason: str = ""
    covered_total: int = 0
    window: list[_Obs] = field(default_factory=list)


class HealthMonitor:
    """Runs the three detectors and reports one :class:`~parkscan_core.types.CoordView`.

    The monitor is a pure accumulator: it is fed events, it reads no clock, and every
    query takes ``now_ns`` from the caller. Replaying the same events with the same
    ``now_ns`` values always yields the same verdicts.

    ``OFFLINE`` **latches**. Once the deadline has passed or the Last Will has fired, fresh
    telemetry does not quietly bring the robot back: only
    :meth:`clear_offline_latch`, which the coordinator calls as part of Lazarus
    re-registration, does. A robot that could resurrect itself by simply resuming
    publication is the double-assignment hazard, not a convenience.
    """

    def __init__(self, config: HealthConfig | None = None) -> None:
        self._cfg = config if config is not None else HealthConfig()
        self._tracks: dict[RobotId, _Track] = {}

    @property
    def config(self) -> HealthConfig:
        return self._cfg

    @property
    def tracked(self) -> tuple[RobotId, ...]:
        """Every robot known to the monitor, in id order."""
        return tuple(sorted(self._tracks))

    # ------------------------------------------------------------------- ingestion --
    def register(self, robot_id: RobotId, now_ns: int) -> None:
        """Start tracking ``robot_id``, treating ``now_ns`` as its first heartbeat.

        Re-registering a known robot resets its heartbeat baseline and clears its stuck
        window, but does **not** clear an offline latch -- that is
        :meth:`clear_offline_latch`, and keeping them separate is what stops a stray
        ``register`` call from resurrecting a robot the coordinator declared dead.
        """
        track = self._tracks.get(robot_id)
        if track is None:
            self._tracks[robot_id] = _Track(robot_id=robot_id, last_beat_ns=now_ns)
            return
        track.last_beat_ns = now_ns
        track.window.clear()

    def on_connection(self, event: ConnectionEvent) -> None:
        """Consume one broker liveness signal.

        A disconnect latches ``OFFLINE`` immediately -- the whole point of the Last Will is
        that it does not wait for a deadline. A connect records that the session is back
        but deliberately does **not** clear the latch.
        """
        track = self._tracks.get(event.robot_id)
        if track is None:
            track = _Track(robot_id=event.robot_id, last_beat_ns=event.stamp_ns)
            self._tracks[event.robot_id] = track
        track.connected = event.connected
        if not event.connected:
            track.latched_offline = True
            detail = f" ({event.detail})" if event.detail else ""
            track.latch_reason = f"broker last will at t={event.stamp_ns} ns{detail}"

    def on_telemetry(self, sample: TelemetrySample) -> None:
        """Consume one telemetry sample: it is the heartbeat *and* the stuck-window input.

        Samples older than the last one recorded are ignored, so an out-of-order delivery
        cannot rewind a robot's heartbeat or corrupt the window's ordering.
        """
        track = self._tracks.get(sample.robot_id)
        if track is None:
            track = _Track(robot_id=sample.robot_id, last_beat_ns=sample.stamp_ns)
            self._tracks[sample.robot_id] = track
        elif sample.stamp_ns < track.last_beat_ns:
            return

        track.last_beat_ns = sample.stamp_ns
        track.covered_total += len(sample.covered_cells)
        track.window.append(
            _Obs(
                stamp_ns=sample.stamp_ns,
                x_m=sample.pose.x_m,
                y_m=sample.pose.y_m,
                covered_total=track.covered_total,
                state=sample.state,
            )
        )
        self._trim(track)

    def clear_offline_latch(self, robot_id: RobotId, now_ns: int) -> None:
        """Lazarus: accept that a robot declared dead is back, and start it fresh.

        Called by :class:`~parkscan_coord.coordinator.Coordinator` only from an explicit
        re-registration, never from the arrival of telemetry.
        """
        track = self._tracks.get(robot_id)
        if track is None:
            msg = f"unknown robot {robot_id!r}"
            raise KeyError(msg)
        track.latched_offline = False
        track.latch_reason = ""
        track.connected = True
        track.last_beat_ns = now_ns
        track.window.clear()

    def is_latched_offline(self, robot_id: RobotId) -> bool:
        """Whether ``robot_id`` has been declared dead and not yet re-registered."""
        track = self._tracks.get(robot_id)
        return track is not None and track.latched_offline

    # ------------------------------------------------------------------ judgement --
    def missed_beats(self, robot_id: RobotId, now_ns: int) -> int:
        """Beats that came due since the last one arrived, and did not.

        A beat due at ``t`` counts as missed once ``now_ns`` reaches ``t``: at exactly
        ``last_beat + k x period`` the count is ``k``. That boundary is what makes
        "``OFFLINE`` after exactly ``k`` missed beats, not before" a testable statement
        rather than a rounding accident.
        """
        track = self._tracks.get(robot_id)
        if track is None:
            msg = f"unknown robot {robot_id!r}"
            raise KeyError(msg)
        elapsed = now_ns - track.last_beat_ns
        if elapsed <= 0:
            return 0
        return elapsed // self._cfg.heartbeat_period_ns

    def assess(self, robot_id: RobotId, now_ns: int) -> HealthAssessment:
        """Judge one robot at ``now_ns``.

        Idempotent, but not side-effect-free: crossing the heartbeat deadline **latches**
        the ``OFFLINE`` verdict, which is what makes re-registration mandatory afterwards.
        Calling it twice at the same ``now_ns`` returns the same assessment.

        Raises:
            KeyError: ``robot_id`` was never registered and has never been heard from.
        """
        track = self._tracks.get(robot_id)
        if track is None:
            msg = f"unknown robot {robot_id!r}"
            raise KeyError(msg)

        missed = self.missed_beats(robot_id, now_ns)
        displacement, span, samples = self._window_stats(track)

        view, reason = self._verdict(track, missed, displacement, span, samples, now_ns)
        return HealthAssessment(
            robot_id=robot_id,
            view=view,
            now_ns=now_ns,
            missed_beats=missed,
            last_beat_ns=track.last_beat_ns,
            connected=track.connected,
            latched_offline=track.latched_offline,
            displacement_m=displacement,
            window_span_ns=span,
            window_samples=samples,
            reason=reason,
        )

    def assess_all(self, now_ns: int) -> tuple[HealthAssessment, ...]:
        """Judge every tracked robot, in id order."""
        return tuple(self.assess(robot_id, now_ns) for robot_id in sorted(self._tracks))

    def view(self, robot_id: RobotId, now_ns: int) -> CoordView:
        """Just the verdict, for callers that do not need the evidence."""
        return self.assess(robot_id, now_ns).view

    # ------------------------------------------------------------------ internals --
    def _verdict(
        self,
        track: _Track,
        missed: int,
        displacement: float,
        span: int,
        samples: int,
        now_ns: int,
    ) -> tuple[CoordView, str]:
        """Combine the three detectors into one view. Order of precedence matters."""
        if track.latched_offline:
            return CoordView.OFFLINE, track.latch_reason or "declared offline"

        if missed >= self._cfg.missed_beats_offline:
            track.latched_offline = True
            track.latch_reason = (
                f"{missed} missed heartbeats at t={now_ns} ns "
                f"(deadline {self._cfg.detection_latency_s:g} s after the last beat at "
                f"t={track.last_beat_ns} ns)"
            )
            return CoordView.OFFLINE, track.latch_reason

        if self._is_stuck(track, displacement, span, samples):
            return (
                CoordView.STUCK,
                f"no progress for {span / NS_PER_S:g} s: moved {displacement:.3f} m "
                f"(< {self._cfg.stuck_epsilon_m:g} m) and covered no new cells",
            )

        if missed >= self._cfg.missed_beats_suspect:
            return CoordView.SUSPECT, f"{missed} missed heartbeat(s)"

        return CoordView.ONLINE, "heartbeat current"

    def _is_stuck(self, track: _Track, displacement: float, span: int, samples: int) -> bool:
        if samples < self._cfg.stuck_min_samples or span < self._cfg.stuck_window_ns:
            return False
        if any(obs.state in STUCK_EXEMPT_STATES for obs in track.window):
            return False
        if track.window[-1].covered_total > track.window[0].covered_total:
            return False
        return displacement < self._cfg.stuck_epsilon_m

    def _window_stats(self, track: _Track) -> tuple[float, int, int]:
        """Return ``(displacement_m, span_ns, sample_count)`` for the live window.

        Displacement is the **diagonal of the axis-aligned bounding box** of the window's
        positions, not the net start-to-end distance. Net displacement is zero for a robot
        oscillating between two points or grinding against an obstacle, which would let a
        genuinely wedged robot hide. The bounding-box diagonal is an upper bound on every
        pairwise distance in the window, so the test can only ever be conservative: a
        robot that really moved is never reported as stuck.
        """
        window = track.window
        if not window:
            return 0.0, 0, 0
        xs = [obs.x_m for obs in window]
        ys = [obs.y_m for obs in window]
        diagonal = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        span = window[-1].stamp_ns - window[0].stamp_ns
        return diagonal, span, len(window)

    def _trim(self, track: _Track) -> None:
        """Drop observations that fell out of the sliding window.

        One sample from *before* the cut is kept. Without it a window holding only
        strictly-inside samples spans at most ``window - period``, so at a sampling period
        that does not divide the window the span never reaches ``stuck_window_ns`` and the
        detector silently never fires.
        """
        window = track.window
        if not window:
            return
        cutoff = window[-1].stamp_ns - self._cfg.stuck_window_ns
        first_inside = len(window)
        for index, obs in enumerate(window):
            if obs.stamp_ns >= cutoff:
                first_inside = index
                break
        start = max(0, first_inside - 1)
        if start > 0:
            track.window = window[start:]

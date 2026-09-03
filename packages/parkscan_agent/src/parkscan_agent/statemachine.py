"""The robot task state machine, as one table.

Why a table and not ``if`` statements
-------------------------------------
Scattered conditionals encode the transition relation implicitly: the set of legal
(state, event) pairs is whatever falls out of the branch structure, so nobody can read it
off, and the *illegal* pairs are invisible. Acceptance criterion C7 requires a test over
**all** (state, event) pairs including the illegal ones, and that test can only be written
against an explicit relation. :data:`TRANSITIONS` is that relation; :func:`step` is the
only way to move, and anything not in the table raises :class:`IllegalTransition` rather
than silently doing nothing.

The states are :class:`~parkscan_core.types.RobotState` -- the robot's **own** view of
itself, which is what it reports in telemetry. The coordinator keeps a separate
:class:`~parkscan_core.types.CoordView` judgement; the two are deliberately not merged,
because a robot that believes it is ``INSPECTING`` while the coordinator has declared it
``OFFLINE`` is precisely the case the Phase-4 recovery logic exists to handle.

Design notes on three transitions that look odd until you know why
------------------------------------------------------------------
* **``WAITING_ON_LEASE`` resumes via two different events.** A pure ``(state, event) ->
  state`` map cannot remember where the wait began, and "grant" means two different
  things: a robot that queued for a lane segment on the way to its zone must resume its
  mission leg, while a robot that queued for a charging bay must resume its return leg.
  Collapsing them into one event would send a nearly-flat robot back to work. So the
  caller -- which does know its own intent -- picks :attr:`Event.LEASE_GRANTED`
  (-> ``TRANSIT``) or :attr:`Event.BAY_LEASE_GRANTED` (-> ``RETURNING``). Nothing about
  leases themselves lives in this package; the agent only reacts to the outcome.
* **Resuming a mission always lands in ``TRANSIT``, never straight back in
  ``INSPECTING``.** A robot that was interrupted mid-sweep is already inside its zone, so
  :class:`~parkscan_agent.robot.RobotAgent` raises :attr:`Event.ZONE_REACHED` as soon as it
  sees a sweep already in progress -- within the same tick, so the detour is free at
  runtime. Paying for it in the table buys a relation that stays a pure function of
  ``(state, event)`` and can therefore be exhaustively tested.
* **``ABORT`` is illegal from ``RETURNING`` and ``CHARGING``.** The coordinator may cancel
  an *order*; it may not cancel a robot's attempt to reach power. Making that an
  ``IllegalTransition`` rather than a comment means a future coordinator bug fails loudly.

``FAILED`` and ``OFFLINE`` are left only by :attr:`Event.RECOVERED`, which the coordinator
issues after a Lazarus robot has re-registered with a fresh epoch and explicitly abandoned
its old order. Recovery lands in ``IDLE``, never back in the interrupted task: a silent
resume is exactly the double-assignment hazard the epoch mechanism exists to stop.
"""

from __future__ import annotations

from collections.abc import Mapping
from enum import StrEnum
from types import MappingProxyType

from parkscan_core.types import RobotState

__all__ = [
    "TRANSITIONS",
    "Event",
    "IllegalTransition",
    "is_legal",
    "legal_events",
    "step",
]


class Event(StrEnum):
    """Everything that can move a robot between task states.

    Events are named for *what happened*, not for the state they lead to, so one event can
    mean different things from different states and the table stays the single place where
    meaning is assigned.
    """

    ASSIGNED = "assigned"
    """The coordinator handed this robot a zone (carrying an assignment epoch)."""

    ROUTE_STARTED = "route_started"
    """A lane-graph route to the zone entry was planned and the robot began driving it."""

    ZONE_REACHED = "zone_reached"
    """The robot arrived at its zone entry waypoint (or is already inside after a wait)."""

    SWEEP_COMPLETE = "sweep_complete"
    """The in-zone back-and-forth sweep finished at the zone exit waypoint."""

    LEASE_DENIED = "lease_denied"
    """A shared resource (lane segment or bay) is held by someone else; queue for it."""

    LEASE_GRANTED = "lease_granted"
    """A queued **mission** lease came through; resume driving the route."""

    BAY_LEASE_GRANTED = "bay_lease_granted"
    """A queued **charging-bay** lease came through; resume the return-to-charge leg."""

    BATTERY_CRITICAL = "battery_critical"
    """The feasibility check says the charge left barely covers the drive to a bay."""

    BAY_REACHED = "bay_reached"
    """The docking controller reported a successful dock inside the bay."""

    DOCK_FAILED = "dock_failed"
    """The docking controller gave up: tolerance not met within its tick budget."""

    CHARGED = "charged"
    """State of charge reached the configured full level."""

    ABORT = "abort"
    """The coordinator cancelled the current order (reassignment, zone requeue)."""

    FAULT = "fault"
    """An unrecoverable local fault; the robot stops and waits to be recovered."""

    COMMS_LOST = "comms_lost"
    """The robot lost the broker, or the coordinator declared it gone."""

    RECOVERED = "recovered"
    """Lazarus re-registration accepted: fresh epoch, old order explicitly abandoned."""


class IllegalTransition(RuntimeError):
    """Raised by :func:`step` for a ``(state, event)`` pair that is not in the table."""

    def __init__(self, state: RobotState, event: Event) -> None:
        self.state = state
        self.event = event
        allowed = ", ".join(sorted(e.value for e in legal_events(state))) or "(none)"
        super().__init__(
            f"illegal transition: event {event.value!r} is not accepted in state "
            f"{state.value!r}; legal events there are: {allowed}"
        )


_TABLE: dict[tuple[RobotState, Event], RobotState] = {
    # -- IDLE: no order, sitting on the lane graph -----------------------------------
    (RobotState.IDLE, Event.ASSIGNED): RobotState.ASSIGNED,
    (RobotState.IDLE, Event.BATTERY_CRITICAL): RobotState.RETURNING,
    (RobotState.IDLE, Event.FAULT): RobotState.FAILED,
    (RobotState.IDLE, Event.COMMS_LOST): RobotState.OFFLINE,
    # -- ASSIGNED: order accepted, route not yet running -----------------------------
    (RobotState.ASSIGNED, Event.ROUTE_STARTED): RobotState.TRANSIT,
    (RobotState.ASSIGNED, Event.LEASE_DENIED): RobotState.WAITING_ON_LEASE,
    (RobotState.ASSIGNED, Event.BATTERY_CRITICAL): RobotState.RETURNING,
    (RobotState.ASSIGNED, Event.ABORT): RobotState.IDLE,
    (RobotState.ASSIGNED, Event.FAULT): RobotState.FAILED,
    (RobotState.ASSIGNED, Event.COMMS_LOST): RobotState.OFFLINE,
    # -- TRANSIT: driving the lane route to the zone entry ---------------------------
    (RobotState.TRANSIT, Event.ZONE_REACHED): RobotState.INSPECTING,
    (RobotState.TRANSIT, Event.LEASE_DENIED): RobotState.WAITING_ON_LEASE,
    (RobotState.TRANSIT, Event.BATTERY_CRITICAL): RobotState.RETURNING,
    (RobotState.TRANSIT, Event.ABORT): RobotState.IDLE,
    (RobotState.TRANSIT, Event.FAULT): RobotState.FAILED,
    (RobotState.TRANSIT, Event.COMMS_LOST): RobotState.OFFLINE,
    # -- INSPECTING: sweeping the zone -----------------------------------------------
    (RobotState.INSPECTING, Event.SWEEP_COMPLETE): RobotState.IDLE,
    (RobotState.INSPECTING, Event.LEASE_DENIED): RobotState.WAITING_ON_LEASE,
    (RobotState.INSPECTING, Event.BATTERY_CRITICAL): RobotState.RETURNING,
    (RobotState.INSPECTING, Event.ABORT): RobotState.IDLE,
    (RobotState.INSPECTING, Event.FAULT): RobotState.FAILED,
    (RobotState.INSPECTING, Event.COMMS_LOST): RobotState.OFFLINE,
    # -- WAITING_ON_LEASE: stopped, queued for a shared resource ---------------------
    (RobotState.WAITING_ON_LEASE, Event.LEASE_GRANTED): RobotState.TRANSIT,
    (RobotState.WAITING_ON_LEASE, Event.BAY_LEASE_GRANTED): RobotState.RETURNING,
    (RobotState.WAITING_ON_LEASE, Event.BATTERY_CRITICAL): RobotState.RETURNING,
    (RobotState.WAITING_ON_LEASE, Event.ABORT): RobotState.IDLE,
    (RobotState.WAITING_ON_LEASE, Event.FAULT): RobotState.FAILED,
    (RobotState.WAITING_ON_LEASE, Event.COMMS_LOST): RobotState.OFFLINE,
    # -- RETURNING: driving to a bay and docking -------------------------------------
    (RobotState.RETURNING, Event.BAY_REACHED): RobotState.CHARGING,
    (RobotState.RETURNING, Event.DOCK_FAILED): RobotState.FAILED,
    (RobotState.RETURNING, Event.LEASE_DENIED): RobotState.WAITING_ON_LEASE,
    (RobotState.RETURNING, Event.FAULT): RobotState.FAILED,
    (RobotState.RETURNING, Event.COMMS_LOST): RobotState.OFFLINE,
    # -- CHARGING: docked, drawing power ---------------------------------------------
    (RobotState.CHARGING, Event.CHARGED): RobotState.IDLE,
    (RobotState.CHARGING, Event.FAULT): RobotState.FAILED,
    (RobotState.CHARGING, Event.COMMS_LOST): RobotState.OFFLINE,
    # -- FAILED / OFFLINE: only re-registration gets you out -------------------------
    (RobotState.FAILED, Event.RECOVERED): RobotState.IDLE,
    (RobotState.FAILED, Event.COMMS_LOST): RobotState.OFFLINE,
    (RobotState.OFFLINE, Event.RECOVERED): RobotState.IDLE,
}

TRANSITIONS: Mapping[tuple[RobotState, Event], RobotState] = MappingProxyType(_TABLE)
"""The whole transition relation. Read-only: a caller cannot bend the rules at runtime."""


def step(state: RobotState, event: Event) -> RobotState:
    """Apply ``event`` in ``state`` and return the new state.

    Raises:
        IllegalTransition: ``(state, event)`` is not in :data:`TRANSITIONS`. This is a
            programming error in the caller, not a runtime condition to swallow -- a robot
            that quietly ignores an event it cannot handle is how a fleet ends up in a
            state nobody modelled.
    """
    try:
        return TRANSITIONS[(state, event)]
    except KeyError:
        raise IllegalTransition(state, event) from None


def is_legal(state: RobotState, event: Event) -> bool:
    """Whether ``event`` is accepted in ``state``. Never raises."""
    return (state, event) in TRANSITIONS


def legal_events(state: RobotState) -> frozenset[Event]:
    """Every event ``state`` accepts, for error messages and for UI affordances."""
    return frozenset(event for (s, event) in TRANSITIONS if s == state)

"""Recovery, fencing and the four anti-thrash guards, on a deliberately tiny lot.

The lot here is built in the test rather than loaded from ``sim/lot/lot.yaml``: two zones,
two lane segments and one bay, so a sweep is a few metres and a scenario is a handful of
events. The committed lot is exercised by the autonomy and allocation suites; what this
file needs is control over *when* a robot fails, not a realistic drive.

Robots are real :class:`~parkscan_agent.robot.RobotAgent` objects over a
:class:`~parkscan_core.testing.FakeBackend`, because the coordinator drives them through
the real state machine and an ``IllegalTransition`` it would raise in a run must be raised
here too. Telemetry is hand-built so that each test states exactly the situation it means.

The allocator is a scripted double. That is the point: the guards must hold against an
allocator that proposes a swap on every single round, which is precisely what a greedy
strategy over a noisy cost signal does.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Final

import pytest
from parkscan_agent.robot import RobotAgent, TelemetrySample
from parkscan_coord.coordinator import (
    AllocationRequest,
    Assignment,
    Coordinator,
    CoordinatorConfig,
    CostFn,
    DecisionKind,
    IngestResult,
    Trigger,
)
from parkscan_coord.cost import RobotSnapshot
from parkscan_coord.health import NS_PER_S, ConnectionEvent, HealthConfig, LivenessSource
from parkscan_coord.metrics import reassignment_churn

from parkscan_core.lot import (
    ChargingBay,
    LaneEdge,
    LaneGraph,
    LaneNode,
    LotOrigin,
    LotSize,
    LotSpec,
    Zone,
)
from parkscan_core.testing import FakeBackend
from parkscan_core.types import CoordView, Pose2D, RobotId, RobotState, TwistCmd, ZoneState

STILL = TwistCmd(v_mps=0.0, omega_radps=0.0)

START_POSES: Final[dict[RobotId, Pose2D]] = {
    "robot_1": Pose2D(x_m=1.0, y_m=5.0, theta_rad=0.0),
    "robot_2": Pose2D(x_m=11.0, y_m=5.0, theta_rad=0.0),
}

_JITTER_M: Final[tuple[float, ...]] = (
    0.0,
    1.2,
    -0.9,
    0.4,
    -1.2,
    0.7,
    -0.3,
    1.1,
    -1.0,
    0.2,
    -0.6,
)
"""A fixed jitter table. A table rather than an RNG so the "noisy" cost signal in the
thrash test is bit-reproducible and carries no global random state.

The length is ODD on purpose. Each allocation round consumes exactly two entries (the
incumbent, then the challenger), so an even-length table would pin the incumbent to even
indices forever and the challenger could never come out cheaper — the thrash test would
still pass, but vacuously, because there was never a swap to refuse. An odd length
rotates the parity every round. Every magnitude stays well under the 5 m hysteresis
delta, so hysteresis still refuses all fifty rounds."""


def _toy_lot() -> LotSpec:
    """Two 8 x 10 m zones on a four-segment lane, with one charging bay."""
    nodes = (
        LaneNode(id="bay_n", x_m=1.0, y_m=1.0),
        LaneNode(id="za_e", x_m=1.0, y_m=5.0),
        LaneNode(id="za_x", x_m=7.0, y_m=5.0),
        LaneNode(id="zb_e", x_m=11.0, y_m=5.0),
        LaneNode(id="zb_x", x_m=17.0, y_m=5.0),
    )
    edges = (
        LaneEdge(id="ln_bay_spur", a="bay_n", b="za_e"),
        LaneEdge(id="ln_a", a="za_e", b="za_x"),
        LaneEdge(id="ln_link", a="za_x", b="zb_e"),
        LaneEdge(id="ln_b", a="zb_e", b="zb_x"),
    )
    zones = (
        Zone(
            id="zone_a",
            polygon=((0.0, 0.0), (8.0, 0.0), (8.0, 10.0), (0.0, 10.0)),
            entry_waypoint="za_e",
            exit_waypoint="za_x",
            sweep_length_m=21.0,
        ),
        Zone(
            id="zone_b",
            polygon=((10.0, 0.0), (18.0, 0.0), (18.0, 10.0), (10.0, 10.0)),
            entry_waypoint="zb_e",
            exit_waypoint="zb_x",
            sweep_length_m=21.0,
        ),
    )
    return LotSpec(
        name="toy",
        origin=LotOrigin(lat=0.0, lon=0.0),
        size_m=LotSize(w=20.0, h=10.0),
        zones=zones,
        parking_spaces=(),
        charging_bays=(ChargingBay(id="bay_1", node="bay_n", position=(1.0, 1.0)),),
        lane_graph=LaneGraph(nodes=nodes, edges=edges),
    )


# ------------------------------------------------------------------ allocator doubles --
class _ScriptedAllocator:
    """Returns whatever the test scripted, and remembers every request it was given."""

    def __init__(self, plan: Callable[[AllocationRequest], list[Assignment]]) -> None:
        self._plan = plan
        self.requests: list[AllocationRequest] = []

    def __call__(self, request: AllocationRequest) -> Sequence[Assignment]:
        self.requests.append(request)
        return self._plan(request)


def _first_come(request: AllocationRequest) -> list[Assignment]:
    """Pair each free robot with the next unassigned zone, both in id order."""
    free = [robot.robot_id for robot in request.robots]
    out: list[Assignment] = []
    for demand in request.demands:
        if demand.assigned_robot is not None or not free:
            continue
        out.append(Assignment(robot_id=free.pop(0), zone_id=demand.zone_id))
    return out


def _only(zone_id: str) -> Callable[[AllocationRequest], list[Assignment]]:
    """Offer one zone, to the first free robot, and nothing else."""

    def plan(request: AllocationRequest) -> list[Assignment]:
        demand = next((d for d in request.demands if d.zone_id == zone_id), None)
        if demand is None or demand.assigned_robot is not None or not request.robots:
            return []
        return [Assignment(robot_id=request.robots[0].robot_id, zone_id=zone_id)]

    return plan


def _seed_then_swap(
    initial: RobotId, challenger: RobotId, zone_id: str
) -> Callable[[AllocationRequest], list[Assignment]]:
    """Give the zone to ``initial`` on the first round, then propose ``challenger`` forever.

    The swap guards can only be tested against a zone that is actually held, and a plan
    that only ever names the challenger never establishes that precondition -- the zone
    simply stays unassigned and the "swap" is a first assignment wearing a disguise.
    """

    def plan(request: AllocationRequest) -> list[Assignment]:
        demand = next((d for d in request.demands if d.zone_id == zone_id), None)
        if demand is None:
            return []
        if demand.assigned_robot is None:
            if any(robot.robot_id == initial for robot in request.robots):
                return [Assignment(robot_id=initial, zone_id=zone_id)]
            return []
        if any(robot.robot_id == challenger for robot in request.robots):
            return [Assignment(robot_id=challenger, zone_id=zone_id)]
        return []

    return plan


def _keep_alive(rig: _Rig, robot_id: RobotId, upto_ns: int, *, every_ns: int = NS_PER_S) -> None:
    """Heartbeat a robot up to ``upto_ns``.

    The whole fleet is registered by ``start``, so a robot that never reports is correctly
    declared OFFLINE after k missed beats and latched. A test that wants a robot merely
    *idle* rather than *dead* has to say so by letting it speak.
    """
    now = every_ns
    while now <= upto_ns:
        _report(rig, robot_id, now, state=RobotState.IDLE)
        now += every_ns


def _always_swap(
    robot_id: RobotId, zone_id: str
) -> Callable[[AllocationRequest], list[Assignment]]:
    """Propose the same swap on every single round -- what a noisy greedy allocator does."""

    def plan(request: AllocationRequest) -> list[Assignment]:
        if any(robot.robot_id == robot_id for robot in request.robots):
            return [Assignment(robot_id=robot_id, zone_id=zone_id)]
        return []

    return plan


# ----------------------------------------------------------------------- cost doubles --
class _JitteryCost:
    """A cost signal that wobbles by less than the hysteresis delta and never settles."""

    def __init__(self) -> None:
        self.calls: list[tuple[RobotId, float]] = []

    def __call__(self, lot: LotSpec, robot: RobotSnapshot, zone: Zone) -> float:
        value = 100.0 + _JITTER_M[len(self.calls) % len(_JITTER_M)]
        self.calls.append((robot.robot_id, value))
        return value

    def best_seen_improvement_m(self, incumbent: RobotId) -> float:
        """Largest amount by which a challenger ever undercut the incumbent.

        Walks ADJACENT pairs without assuming which of the two was priced first: the
        coordinator is free to price incumbent-then-challenger or the reverse, and an
        instrument that depends on that order measures the coordinator's iteration order
        rather than the cost signal.
        """
        best = 0.0
        for (robot_a, cost_a), (robot_b, cost_b) in zip(self.calls, self.calls[1:], strict=False):
            if robot_a == incumbent and robot_b != incumbent:
                best = max(best, cost_a - cost_b)
            elif robot_b == incumbent and robot_a != incumbent:
                best = max(best, cost_b - cost_a)
        return best


def _robot_2_is_far_cheaper(lot: LotSpec, robot: RobotSnapshot, zone: Zone) -> float:
    return 10.0 if robot.robot_id == "robot_2" else 100.0


# ------------------------------------------------------------------------------- rig --
@dataclass(frozen=True, slots=True)
class _Rig:
    lot: LotSpec
    backend: FakeBackend
    agents: dict[RobotId, RobotAgent]
    allocator: _ScriptedAllocator
    coord: Coordinator


def _rig(
    *,
    plan: Callable[[AllocationRequest], list[Assignment]] = _first_come,
    config: CoordinatorConfig | None = None,
    cost_fn: CostFn | None = None,
    seeded: tuple[RobotId, ...] = ("robot_1", "robot_2"),
    start_ns: int = 0,
) -> _Rig:
    lot = _toy_lot()
    backend = FakeBackend(START_POSES)
    agents = {robot_id: RobotAgent(robot_id, lot, backend) for robot_id in START_POSES}
    allocator = _ScriptedAllocator(plan)
    kwargs: dict[str, object] = {}
    if cost_fn is not None:
        kwargs["cost_fn"] = cost_fn
    coord = Coordinator(
        lot,
        agents,
        allocator=allocator,
        config=config if config is not None else CoordinatorConfig(),
        **kwargs,  # type: ignore[arg-type]
    )
    coord.start(start_ns, poses={r: START_POSES[r] for r in seeded})
    return _Rig(lot=lot, backend=backend, agents=agents, allocator=allocator, coord=coord)


def _report(
    rig: _Rig,
    robot_id: RobotId,
    now_ns: int,
    *,
    state: RobotState = RobotState.INSPECTING,
    zone_id: str | None = None,
    epoch: int | None = None,
    covered: tuple[str, ...] = (),
    pose: Pose2D | None = None,
    moving: bool = True,
) -> IngestResult:
    """Hand one telemetry sample to the coordinator, defaulting the epoch to the live one.

    ``moving`` defaults to True and drifts the reported pose with the clock, because a
    healthy robot is one that is *getting somewhere*. Reporting the start pose forever
    would be telemetry from a robot that has not moved in ten seconds, and the stuck
    detector would be right to say so — which is what made the earlier version of these
    fixtures fail. Tests that mean to simulate a genuinely stuck robot pass
    ``moving=False`` and say so.
    """
    if epoch is None:
        epoch = 0 if zone_id is None else rig.coord.zone_status(zone_id).epoch
    start = START_POSES[robot_id]
    if pose is not None:
        at = pose
    elif moving:
        # Well clear of stuck_epsilon_m (0.25 m) per second of simulated time.
        at = Pose2D(
            x_m=start.x_m + 1.0 * (now_ns / NS_PER_S),
            y_m=start.y_m,
            theta_rad=start.theta_rad,
        )
    else:
        at = start
    return rig.coord.ingest(
        TelemetrySample(
            robot_id=robot_id,
            stamp_ns=now_ns,
            pose=at,
            twist=STILL,
            command=STILL,
            battery_soc=0.9,
            state=state,
            zone_id=zone_id,
            epoch=epoch,
            covered_cells=covered,
            detail="test",
        )
    )


def _planned(rig: _Rig, zone_id: str) -> tuple[str, ...]:
    return rig.coord.zone_status(zone_id).cells_total


def _event_count(rig: _Rig) -> int:
    """Number of recovery events so far.

    Wrapped in a function on purpose. Asserting ``len(rig.coord.reassignment_events) == 0``
    inline makes mypy narrow that member expression to ``tuple[()]`` and keep the narrowing
    for the rest of the test, so every later use of the property is reported unreachable.
    """
    return len(rig.coord.reassignment_events)


def _kinds(decisions: Sequence[object]) -> list[DecisionKind]:
    return [d.kind for d in decisions]  # type: ignore[attr-defined]


# ------------------------------------------------------------------ initial allocation --
def test_the_first_allocation_goes_out_through_the_ordinary_event_path() -> None:
    rig = _rig()
    assert rig.allocator.requests == []  # start() queues; it does not allocate

    decisions = rig.coord.tick(0)
    assert [d.kind for d in decisions] == [DecisionKind.ASSIGNED, DecisionKind.ASSIGNED]
    assert rig.allocator.requests[0].triggers == (Trigger.ZONE_ADDED,)
    assert rig.coord.zone_status("zone_a").assigned_robot == "robot_1"
    assert rig.coord.zone_status("zone_b").assigned_robot == "robot_2"
    assert rig.agents["robot_1"].zone_id == "zone_a"
    assert rig.coord.assignments_total == 2


def test_no_event_means_no_resolve() -> None:
    # Anti-thrash guard 3. Twenty ticks with healthy telemetry and nothing changing must
    # not produce a second allocation round; a periodic re-solve is the biggest churn
    # source in this design, so its absence is asserted rather than assumed.
    rig = _rig()
    rig.coord.tick(0)
    assert len(rig.allocator.requests) == 1

    for i in range(1, 21):
        now = i * NS_PER_S
        _report(rig, "robot_1", now, zone_id="zone_a", state=RobotState.TRANSIT)
        _report(rig, "robot_2", now, zone_id="zone_b", state=RobotState.TRANSIT)
        assert rig.coord.tick(now) == ()

    assert len(rig.allocator.requests) == 1
    assert rig.coord.assignments_total == 2


# ---------------------------------------------------------------------- epoch fencing --
def test_a_stale_epoch_report_is_dropped_and_a_current_one_is_accepted() -> None:
    rig = _rig(plan=_only("zone_a"))
    rig.coord.tick(0)
    live_epoch = rig.coord.zone_status("zone_a").epoch
    assert live_epoch == 1

    cell = _planned(rig, "zone_a")[0]
    stale = _report(
        rig, "robot_1", NS_PER_S, zone_id="zone_a", epoch=live_epoch - 1, covered=(cell,)
    )
    assert not stale.accepted
    assert stale.stale_epoch
    assert "stale epoch" in stale.reason
    assert rig.coord.zone_status("zone_a").cells_done == ()
    assert rig.coord.stale_reports_dropped == 1

    current = _report(
        rig, "robot_1", 2 * NS_PER_S, zone_id="zone_a", epoch=live_epoch, covered=(cell,)
    )
    assert current.accepted
    assert rig.coord.zone_status("zone_a").cells_done == (cell,)
    assert rig.coord.stale_reports_dropped == 1


def test_a_stale_report_still_counts_as_a_heartbeat() -> None:
    # The fence guards progress, not liveness: a robot sending stale reports is wrong
    # about the zone but demonstrably alive, and declaring it dead would be a second bug.
    rig = _rig(plan=_only("zone_a"))
    rig.coord.tick(0)
    for i in range(1, 8):
        now = i * NS_PER_S
        result = _report(rig, "robot_1", now, zone_id="zone_a", epoch=0)
        assert result.stale_epoch
        assert rig.coord.monitor.assess("robot_1", now).view is CoordView.ONLINE


def test_a_report_about_a_zone_the_robot_no_longer_holds_is_dropped() -> None:
    rig = _rig(plan=_only("zone_a"))
    rig.coord.tick(0)
    epoch = rig.coord.zone_status("zone_a").epoch
    result = _report(rig, "robot_2", NS_PER_S, zone_id="zone_a", epoch=epoch)
    assert not result.accepted
    assert result.stale_epoch


# ------------------------------------------------------------- requeue with remainder --
def test_requeue_carries_the_remaining_cells_instead_of_restarting_the_zone() -> None:
    rig = _rig(plan=_only("zone_a"), seeded=("robot_1",))
    rig.coord.tick(0)
    planned = _planned(rig, "zone_a")
    assert len(planned) >= 3

    done = planned[:2]
    _report(rig, "robot_1", NS_PER_S, zone_id="zone_a", covered=done)
    assert rig.coord.zone_status("zone_a").cells_done == done

    # Silence: three missed beats at 1 Hz from the last report at t = 1 s.
    decisions = rig.coord.tick(4 * NS_PER_S)
    requeue = next(d for d in decisions if d.kind is DecisionKind.REQUEUED)

    assert set(requeue.remaining_cells) == set(planned) - set(done)
    assert len(requeue.remaining_cells) == len(planned) - 2

    status = rig.coord.zone_status("zone_a")
    assert status.state is ZoneState.UNASSIGNED
    assert status.cells_done == done  # the partial work survived the robot that did it
    assert status.cells_remaining == requeue.remaining_cells
    assert status.fraction == pytest.approx(2 / len(planned))


def test_the_remainder_survives_the_reassignment_too() -> None:
    rig = _rig(plan=_only("zone_a"), seeded=("robot_1",))
    rig.coord.tick(0)
    planned = _planned(rig, "zone_a")
    done = planned[:2]
    _report(rig, "robot_1", NS_PER_S, zone_id="zone_a", covered=done)

    # robot_2 is powered on and idle throughout — the whole fleet is registered at start,
    # so a robot that never heartbeats is (correctly) declared offline and latched, and
    # would not be available to take the work over.
    for i in range(1, 5):
        _report(rig, "robot_2", i * NS_PER_S, state=RobotState.IDLE)

    rig.coord.tick(4 * NS_PER_S)  # robot_1 declared offline, zone requeued
    _report(rig, "robot_2", 5 * NS_PER_S, state=RobotState.IDLE)
    rig.coord.tick(5 * NS_PER_S)

    status = rig.coord.zone_status("zone_a")
    assert status.assigned_robot == "robot_2"
    assert status.cells_done == done
    assert len(status.cells_remaining) == len(planned) - 2
    assert status.epoch == 2  # a fresh epoch fences out anything robot_1 still sends


# ------------------------------------------------------------------------- offline --
def test_a_silent_robot_is_declared_offline_and_loses_its_zone() -> None:
    rig = _rig(plan=_only("zone_a"), seeded=("robot_1",))
    rig.coord.tick(0)

    assert rig.coord.tick(2 * NS_PER_S) == ()  # two missed beats: suspect, not acted on
    assert rig.coord.monitor.assess("robot_1", 2 * NS_PER_S).view is CoordView.SUSPECT
    assert rig.coord.zone_status("zone_a").assigned_robot == "robot_1"

    decisions = rig.coord.tick(3 * NS_PER_S)
    assert DecisionKind.ROBOT_OFFLINE in _kinds(decisions)
    assert DecisionKind.REQUEUED in _kinds(decisions)
    assert rig.coord.zone_status("zone_a").assigned_robot is None
    assert rig.agents["robot_1"].state is RobotState.OFFLINE
    assert rig.coord.forced_requeues == 1


def test_a_broker_last_will_acts_without_waiting_for_the_heartbeat_deadline() -> None:
    rig = _rig(plan=_only("zone_a"), seeded=("robot_1",))
    rig.coord.tick(0)

    decisions = rig.coord.on_connection(
        ConnectionEvent(
            robot_id="robot_1",
            stamp_ns=NS_PER_S // 2,
            connected=False,
            source=LivenessSource.BROKER_LAST_WILL,
        )
    )
    assert DecisionKind.ROBOT_OFFLINE in _kinds(decisions)
    assert rig.coord.zone_status("zone_a").assigned_robot is None


def test_a_robot_that_breaks_off_to_charge_gives_its_zone_back() -> None:
    rig = _rig(plan=_only("zone_a"), seeded=("robot_1",))
    rig.coord.tick(0)
    result = _report(rig, "robot_1", NS_PER_S, zone_id="zone_a", state=RobotState.RETURNING)
    assert result.accepted
    assert DecisionKind.REQUEUED in _kinds(result.decisions)
    assert rig.coord.zone_status("zone_a").assigned_robot is None
    assert rig.coord.snapshot(NS_PER_S).pending_triggers == (Trigger.BATTERY_LOW,)


def test_a_completed_zone_is_banked_and_triggers_a_re_solve() -> None:
    rig = _rig(plan=_only("zone_a"), seeded=("robot_1",))
    rig.coord.tick(0)
    planned = _planned(rig, "zone_a")
    result = _report(
        rig, "robot_1", 5 * NS_PER_S, zone_id="zone_a", state=RobotState.IDLE, covered=planned
    )
    assert DecisionKind.ZONE_COMPLETE in _kinds(result.decisions)

    status = rig.coord.zone_status("zone_a")
    assert status.state is ZoneState.COMPLETE
    assert status.fraction == pytest.approx(1.0)
    assert rig.coord.zone_completions[0].stamp_ns == 5 * NS_PER_S
    assert Trigger.ZONE_COMPLETE in rig.coord.snapshot(5 * NS_PER_S).pending_triggers


# ------------------------------------------------------------------------- stuck --
def test_a_stuck_robot_loses_its_zone_and_becomes_available_again() -> None:
    rig = _rig(plan=_only("zone_a"), seeded=("robot_1",))
    rig.coord.tick(0)
    for i in range(11):
        # moving=False is the whole point here: the robot reports in, so it is not
        # OFFLINE, but it never gets anywhere.
        _report(
            rig,
            "robot_1",
            i * NS_PER_S,
            zone_id="zone_a",
            state=RobotState.TRANSIT,
            moving=False,
        )

    decisions = rig.coord.tick(10 * NS_PER_S)
    assert DecisionKind.ROBOT_STUCK in _kinds(decisions)
    assert DecisionKind.REQUEUED in _kinds(decisions)
    assert rig.agents["robot_1"].state is RobotState.IDLE  # aborted, not declared dead
    assert not rig.coord.monitor.is_latched_offline("robot_1")


# ------------------------------------------------------- anti-thrash: the four guards --
def test_a_noisy_cost_signal_produces_zero_churn() -> None:
    # The honest proof that hysteresis works. The allocator proposes the same swap on
    # every one of fifty rounds, the cost signal genuinely favours the challenger on some
    # of them, and the clock advances well past the cooldown each round so that hysteresis
    # is the only guard doing any work. Nothing moves.
    cost = _JitteryCost()
    rig = _rig(plan=_seed_then_swap("robot_1", "robot_2", "zone_a"), cost_fn=cost)
    rig.coord.tick(0)
    assert rig.coord.zone_status("zone_a").assigned_robot == "robot_1"

    cooldown = rig.coord.config.reassign_cooldown_ns
    previous = 0
    for round_index in range(1, 51):
        now = round_index * 2 * cooldown
        # The gap between rounds is 2 x cooldown = 60 s, far longer than the heartbeat
        # deadline. Without this both robots would be declared offline between rounds and
        # the allocator would never see a challenger to refuse.
        for beat in range(previous + NS_PER_S, now, NS_PER_S):
            _report(rig, "robot_1", beat, zone_id="zone_a", state=RobotState.TRANSIT)
            _report(rig, "robot_2", beat, state=RobotState.IDLE)
        previous = now
        _report(rig, "robot_1", now, zone_id="zone_a", state=RobotState.TRANSIT)
        _report(rig, "robot_2", now, state=RobotState.IDLE)
        rig.coord.resolve(now, trigger=Trigger.ZONE_ADDED)

    assert len(rig.allocator.requests) == 51
    assert rig.coord.voluntary_reassignments == 0
    assert reassignment_churn(
        rig.coord.voluntary_reassignments, rig.coord.assignments_total
    ) == pytest.approx(0.0)
    assert rig.coord.zone_status("zone_a").assigned_robot == "robot_1"
    assert rig.coord.zone_status("zone_a").reassignments == 0

    held = [d for d in rig.coord.decisions if d.kind is DecisionKind.HELD]
    assert len(held) == 50
    assert all("hysteresis" in d.reason for d in held)

    # And the test is not vacuous: the challenger really did price the zone lower on some
    # rounds, just never by the delta. A strict-improvement rule would have swapped.
    improvement = cost.best_seen_improvement_m("robot_1")
    assert improvement > 0.0
    assert improvement < rig.coord.config.hysteresis_delta_m


def test_a_real_improvement_does_pass_hysteresis() -> None:
    # Positive control for the test above: the guard refuses noise, not everything.
    rig = _rig(
        plan=_seed_then_swap("robot_1", "robot_2", "zone_a"),
        cost_fn=_robot_2_is_far_cheaper,
    )
    rig.coord.tick(0)
    now = 10 * rig.coord.config.reassign_cooldown_ns
    _keep_alive(rig, "robot_1", now)
    _keep_alive(rig, "robot_2", now)
    _report(rig, "robot_2", now, state=RobotState.IDLE)

    decisions = rig.coord.resolve(now, trigger=Trigger.ZONE_ADDED)
    assert _kinds(decisions) == [DecisionKind.REQUEUED, DecisionKind.REASSIGNED]
    assert rig.coord.zone_status("zone_a").assigned_robot == "robot_2"
    assert rig.coord.zone_status("zone_a").epoch == 2
    assert rig.coord.voluntary_reassignments == 1
    assert rig.coord.forced_requeues == 0  # discretionary, so not a recovery
    assert rig.coord.reassignment_events == ()  # and not a recovery latency either


def test_the_cooldown_blocks_a_swap_hysteresis_would_have_allowed() -> None:
    rig = _rig(
        plan=_seed_then_swap("robot_1", "robot_2", "zone_a"),
        cost_fn=_robot_2_is_far_cheaper,
    )
    rig.coord.tick(0)
    now = rig.coord.config.reassign_cooldown_ns // 2
    _keep_alive(rig, "robot_1", now)
    _keep_alive(rig, "robot_2", now)
    _report(rig, "robot_2", now, state=RobotState.IDLE)

    decisions = rig.coord.resolve(now, trigger=Trigger.ZONE_ADDED)
    assert _kinds(decisions) == [DecisionKind.HELD]
    assert "cooldown" in decisions[0].reason
    assert rig.coord.zone_status("zone_a").assigned_robot == "robot_1"


def test_the_cap_flags_a_zone_unrecoverable_instead_of_retrying_forever() -> None:
    config = CoordinatorConfig(max_reassignments_per_zone=2, reassign_cooldown_ns=0)
    rig = _rig(plan=_only("zone_a"), config=config, seeded=("robot_1",))

    for attempt in range(4):
        now = (attempt * 10 + 1) * NS_PER_S
        # Faulting is not dying: keep the heartbeat up across the ten-second gap, or the
        # robot is declared OFFLINE and the test stops exercising the reassignment cap.
        _keep_alive(rig, "robot_1", now - NS_PER_S)
        rig.coord.tick(now)
        if rig.coord.zone_status("zone_a").state is ZoneState.UNRECOVERABLE:
            break
        assert rig.coord.zone_status("zone_a").assigned_robot == "robot_1"
        # The robot faults: the coordinator hears about it, and the robot's own local
        # fault handling puts it back in IDLE, ready to be given work again.
        _report(rig, "robot_1", now + NS_PER_S, zone_id="zone_a", state=RobotState.FAILED)
        rig.agents["robot_1"].fail("injected fault")
        rig.agents["robot_1"].recover()

    status = rig.coord.zone_status("zone_a")
    assert status.state is ZoneState.UNRECOVERABLE
    assert status.reassignments == config.max_reassignments_per_zone + 1
    assert DecisionKind.ZONE_UNRECOVERABLE in _kinds(rig.coord.decisions)

    # And it stays put: further rounds never offer it again.
    assignments_before = rig.coord.assignments_total
    # Only requests made from HERE on are evidence; slicing the tail would reach back into
    # rounds from before the cap fired, which legitimately did offer zone_a. And the robot
    # has to stay alive, or resolve() short-circuits with no candidates and makes no
    # request at all — an absence that would pass a naive assertion for the wrong reason.
    first_new = len(rig.allocator.requests)
    _keep_alive(rig, "robot_1", 105 * NS_PER_S)
    for i in range(5):
        rig.coord.resolve((100 + i) * NS_PER_S, trigger=Trigger.ZONE_ADDED)
    assert rig.coord.assignments_total == assignments_before
    fresh = rig.allocator.requests[first_new:]
    assert fresh, "no allocation round ran, so this proves nothing"
    # `request.zones` is the lot's full zone list and always contains zone_a; `demands`
    # is what the allocator is actually invited to fill, so that is what must exclude it.
    assert all(
        "zone_a" not in tuple(demand.zone_id for demand in request.demands) for request in fresh
    )


def test_a_capped_zone_also_refuses_discretionary_swaps() -> None:
    config = CoordinatorConfig(max_reassignments_per_zone=0, reassign_cooldown_ns=0)
    rig = _rig(
        plan=_seed_then_swap("robot_1", "robot_2", "zone_a"),
        config=config,
        cost_fn=_robot_2_is_far_cheaper,
    )
    rig.coord.tick(0)
    _report(rig, "robot_2", NS_PER_S, state=RobotState.IDLE)
    decisions = rig.coord.resolve(NS_PER_S, trigger=Trigger.ZONE_ADDED)
    assert _kinds(decisions) == [DecisionKind.HELD]
    assert "cap" in decisions[0].reason


def test_hysteresis_delta_of_zero_is_rejected_outright() -> None:
    with pytest.raises(ValueError, match="not a guard"):
        CoordinatorConfig(hysteresis_delta_m=0.0)


# -------------------------------------------------------------------------- Lazarus --
def test_a_robot_declared_dead_may_not_silently_resume() -> None:
    rig = _rig(plan=_only("zone_a"), seeded=("robot_1",))
    rig.coord.tick(0)
    rig.coord.tick(3 * NS_PER_S)
    assert rig.coord.monitor.is_latched_offline("robot_1")

    # It starts talking again. That is not enough, and must not be.
    refused = _report(rig, "robot_1", 4 * NS_PER_S, state=RobotState.IDLE)
    assert not refused.accepted
    assert "re-register" in refused.reason
    assert rig.coord.monitor.assess("robot_1", 4 * NS_PER_S).view is CoordView.OFFLINE

    # Nor is it offered work while it is dead. Asserted as "no dead robot was ever
    # offered as a candidate", not as "the allocator was called with an empty fleet":
    # resolve() short-circuits when nobody is available rather than asking an allocator
    # to allocate to no one, so requiring a fresh request here would be testing the
    # optimisation instead of the rule.
    before = len(rig.allocator.requests)
    rig.coord.resolve(5 * NS_PER_S, trigger=Trigger.ZONE_ADDED)
    for request in rig.allocator.requests[before:]:
        assert "robot_1" not in [robot.robot_id for robot in request.robots]
    assert rig.coord.zone_status("zone_a").assigned_robot is None


def test_re_registration_abandons_the_old_order_and_takes_a_fresh_epoch() -> None:
    rig = _rig(plan=_only("zone_a"), seeded=("robot_1",))
    rig.coord.tick(0)
    first_epoch = rig.coord.zone_status("zone_a").epoch
    rig.coord.tick(3 * NS_PER_S)

    decisions = rig.coord.re_register("robot_1", 4 * NS_PER_S)
    assert _kinds(decisions) == [DecisionKind.ROBOT_RECOVERED]
    agent = rig.agents["robot_1"]
    assert agent.state is RobotState.IDLE
    assert agent.zone_id is None  # the old order is gone, not resumed

    _report(rig, "robot_1", 5 * NS_PER_S, state=RobotState.IDLE)
    rig.coord.tick(5 * NS_PER_S)
    status = rig.coord.zone_status("zone_a")
    assert status.assigned_robot == "robot_1"
    assert status.epoch > first_epoch

    # A report carrying the pre-death epoch is now fenced out.
    stale = _report(rig, "robot_1", 6 * NS_PER_S, zone_id="zone_a", epoch=first_epoch)
    assert stale.stale_epoch


def test_re_registration_is_not_a_general_purpose_reset() -> None:
    rig = _rig(plan=_only("zone_a"), seeded=("robot_1",))
    rig.coord.tick(0)
    with pytest.raises(ValueError, match="has not been declared offline"):
        rig.coord.re_register("robot_1", NS_PER_S)
    with pytest.raises(KeyError):
        rig.coord.re_register("robot_9", NS_PER_S)


# --------------------------------------------------------------- reassignment latency --
def test_reassignment_latency_runs_from_detection_to_the_replacement_taking_over() -> None:
    # robot_2 has not been heard from yet, so at the moment robot_1 is declared dead there
    # is nobody to hand the zone to. The replacement arrives two seconds later, and that
    # gap -- not the three seconds spent noticing -- is the reassignment latency.
    rig = _rig(plan=_only("zone_a"), seeded=("robot_1",))
    rig.coord.tick(0)

    rig.coord.tick(3 * NS_PER_S)
    assert rig.coord.zone_status("zone_a").assigned_robot is None
    assert _event_count(rig) == 0

    # robot_2 was registered by start() and then never spoke, so it too was declared dead.
    # Coming back therefore goes through the Lazarus path — which is the point: a robot the
    # coordinator has written off may not simply start talking again (see
    # test_a_robot_declared_dead_may_not_silently_resume).
    rig.coord.re_register("robot_2", 5 * NS_PER_S)
    _report(rig, "robot_2", 5 * NS_PER_S, state=RobotState.IDLE)
    rig.coord.tick(5 * NS_PER_S)

    events = rig.coord.reassignment_events
    assert len(events) == 1
    event = events[0]
    # Compared field by field rather than as a tuple: `RobotId` is a PEP 695 alias and
    # mypy --strict treats the tuple form as a non-overlapping equality, which marks
    # everything after it unreachable.
    assert event.failed_robot == "robot_1"
    assert event.replacement_robot == "robot_2"
    assert event.detected_ns == 3 * NS_PER_S
    assert event.reassigned_ns == 5 * NS_PER_S
    assert event.latency_s == pytest.approx(2.0)

    # Reported separately from the detection deadline, and deliberately not added to it:
    # the 3 s it took to notice is a config value (k x heartbeat period), not a result.
    assert HealthConfig().detection_latency_s == pytest.approx(3.0)
    assert event.latency_s != pytest.approx(5.0)


# ---------------------------------------------------------------------- determinism --
def _scenario() -> tuple[object, ...]:
    rig = _rig(plan=_first_come)
    rig.coord.tick(0)
    for i in range(1, 4):
        now = i * NS_PER_S
        _report(rig, "robot_1", now, zone_id="zone_a", state=RobotState.INSPECTING)
        _report(rig, "robot_2", now, zone_id="zone_b", state=RobotState.INSPECTING)
        rig.coord.tick(now)
    rig.coord.tick(8 * NS_PER_S)  # both robots go silent and are declared offline
    _report(rig, "robot_1", 9 * NS_PER_S, state=RobotState.IDLE)
    rig.coord.tick(9 * NS_PER_S)
    return rig.coord.decisions


def test_the_same_event_sequence_produces_the_same_decisions() -> None:
    assert _scenario() == _scenario()


# ------------------------------------------------------------------- proposal hygiene --
def test_a_proposal_naming_a_robot_twice_is_refused_rather_than_half_applied() -> None:
    def double_book(request: AllocationRequest) -> list[Assignment]:
        return [
            Assignment(robot_id="robot_1", zone_id="zone_a"),
            Assignment(robot_id="robot_1", zone_id="zone_b"),
        ]

    rig = _rig(plan=double_book)
    decisions = rig.coord.tick(0)
    assert _kinds(decisions) == [DecisionKind.ASSIGNED, DecisionKind.HELD]
    assert rig.coord.zone_status("zone_a").assigned_robot == "robot_1"
    assert rig.coord.zone_status("zone_b").assigned_robot is None


def test_a_proposal_for_an_unknown_zone_is_refused() -> None:
    def bogus(request: AllocationRequest) -> list[Assignment]:
        return [Assignment(robot_id="robot_1", zone_id="zone_nowhere")]

    rig = _rig(plan=bogus)
    decisions = rig.coord.tick(0)
    assert _kinds(decisions) == [DecisionKind.HELD]
    assert "not a zone of this lot" in decisions[0].reason


def test_a_robot_without_an_observed_pose_is_not_offered_work() -> None:
    rig = _rig(plan=_first_come, seeded=("robot_1",))
    rig.coord.tick(0)
    assert tuple(r.robot_id for r in rig.allocator.requests[0].robots) == ("robot_1",)
    assert rig.coord.zone_status("zone_b").assigned_robot is None


# ---------------------------------------------------------------------- bookkeeping --
def test_zone_outcomes_and_visits_feed_the_metrics_module() -> None:
    rig = _rig(plan=_only("zone_a"), seeded=("robot_1",))
    rig.coord.tick(0)
    planned = _planned(rig, "zone_a")
    _report(rig, "robot_1", NS_PER_S, zone_id="zone_a", covered=planned[:2])

    outcomes = {outcome.zone_id: outcome for outcome in rig.coord.zone_outcomes()}
    assert outcomes["zone_a"].cells_done == 2
    assert outcomes["zone_a"].cells_total == len(planned)
    assert outcomes["zone_b"].cells_done == 0

    assert rig.coord.cell_visits == tuple(
        visit
        for visit in rig.coord.cell_visits
        if visit.robot_id == "robot_1" and visit.cell_id in planned[:2]
    )
    assert len(rig.coord.cell_visits) == 2


def test_a_fleet_with_mismatched_agent_configs_is_rejected() -> None:
    from parkscan_agent.robot import AgentConfig

    lot = _toy_lot()
    backend = FakeBackend(START_POSES)
    agents = {
        "robot_1": RobotAgent("robot_1", lot, backend),
        "robot_2": RobotAgent("robot_2", lot, backend, AgentConfig(cell_size_m=1.0)),
    }
    with pytest.raises(ValueError, match="different AgentConfig"):
        Coordinator(lot, agents, allocator=_ScriptedAllocator(_first_come))

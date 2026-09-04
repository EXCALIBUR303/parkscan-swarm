"""The coordinator: one central allocator, and the recovery logic around it.

What this is, stated plainly
----------------------------
Three robots driven by one central process. That is a **coordinated fleet**, not a swarm
(ACCEPTANCE H11). There is no consensus here, no decentralised bidding and no
inconsistent situational awareness to reconcile -- ADR-0005 rejects CBBA for exactly that
reason. The coordinator holds the whole picture, decides, and pushes orders out.

How it drives robots
--------------------
Only through the public :class:`~parkscan_agent.robot.RobotAgent` surface --
:meth:`~parkscan_agent.robot.RobotAgent.assign`, ``abort``, ``hold``, ``resume``,
``fail``, ``mark_offline``, ``recover`` -- and it reads only
:class:`~parkscan_agent.robot.TelemetrySample`. It never reaches into an agent's internals
and never touches a backend. Every one of those calls is a state-machine event, so an
order a robot cannot legally accept raises
:class:`~parkscan_agent.statemachine.IllegalTransition` rather than being silently
swallowed; where the coordinator legitimately does not know a robot's state (it has, after
all, just declared it dead) it checks :func:`~parkscan_agent.statemachine.is_legal` first.

Fencing: one mechanism, two uses
--------------------------------
Every zone carries a **monotonic assignment epoch**. A progress report is accepted only if
``report.epoch == zone.epoch``; anything older is dropped silently. That is the same
fencing mechanism as the bay lease's token, applied to a different resource: a monotonic
counter whose holder must present the current value, and a resource that rejects anything
lower (ADR-0007; VDA 5050 pairs ``orderId`` / ``orderUpdateId`` the same way). It is what
stops a revived robot and its replacement from both reporting work on the same zone.

The four anti-thrash guards, all four
-------------------------------------
Reassignment is where a coordinator like this normally falls apart -- a cost function with
any noise in it will happily move zones back and forth forever. Four independent guards
stack:

1. **Hysteresis.** A zone moves only if ``new_cost < current_cost - delta``, with ``delta``
   the named constant :data:`HYSTERESIS_DELTA_M`. Strict improvement (``delta = 0``) is
   *not* enough: any noise at all in the cost signal crosses zero infinitely often.
2. **Per-zone cooldown.** A zone that just changed hands cannot change hands again for
   :data:`REASSIGN_COOLDOWN_NS`.
3. **Event-triggered re-solve only.** :meth:`Coordinator.tick` re-solves only when a
   :class:`Trigger` is pending -- a robot went offline or stuck, a zone completed, a robot
   broke off to charge, or zones were added. There is no periodic re-solve, because a
   timer that re-solves an already-good assignment is the single biggest source of churn
   in this design.
4. **Hard cap.** After :data:`MAX_REASSIGNMENTS_PER_ZONE` requeues, a zone is flagged
   ``UNRECOVERABLE`` and left alone. A zone that has defeated four robots is a zone with a
   problem the allocator cannot solve, and retrying it forever converts one stuck zone into
   a fleet-wide livelock.

Guard 3 is why ``tick`` is cheap to call every step and guard 1 is why calling it often
changes nothing.

Recovery hazards
----------------
* **Double-assignment** -- the epoch fence above.
* **Lost work** -- requeue carries :attr:`~parkscan_agent.robot.ZoneProgress.cells_remaining`
  rather than restarting the zone. See the honest limit below.
* **Thrashing** -- the four guards.
* **Lazarus** -- a robot the coordinator declared dead is *latched* dead. Its telemetry is
  refused, it is never offered work, and only an explicit :meth:`Coordinator.re_register`
  brings it back: fresh epoch, order explicitly abandoned, never a silent resume.

Honest limit on "requeue with remainder"
----------------------------------------
The coordinator preserves the remainder: ``cells_done`` accumulates on the **zone**, across
epochs and across robots, so a failed robot's coverage is not discarded, the dashboard's
coverage does not jump backwards, and ``coverage_pct`` counts the work once.

What it does **not** do is make the replacement robot skip that work.
:class:`~parkscan_agent.robot.RobotAgent` has no partial-sweep entry point -- ``assign``
takes a zone and plans the whole lawnmower path -- so the replacement re-drives the zone
from its entry waypoint. The saving is in accounting and in detections already banked, not
in metres driven, and :func:`parkscan_coord.metrics.duplicate_visit_rate` is where the
re-driven ground shows up rather than being hidden.

Determinism
-----------
No clock is read, no global RNG is touched, and no decision depends on dict iteration
order: robots and zones are offered to the allocator in sorted id order, and the
allocator's proposals are re-sorted before they are applied. Given the same event
sequence, the coordinator produces the same decisions on every machine.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from enum import StrEnum
from typing import Final, Protocol

from parkscan_agent.battery_policy import BatteryParams, ranked_bays, required_soc
from parkscan_agent.robot import AgentConfig, RobotAgent, TelemetrySample, planned_cells
from parkscan_agent.statemachine import Event, is_legal

from parkscan_coord.cost import RobotSnapshot, assignment_cost
from parkscan_coord.health import (
    NS_PER_S,
    ConnectionEvent,
    HealthAssessment,
    HealthMonitor,
)
from parkscan_coord.leases import Grant, LeaseManager
from parkscan_coord.metrics import CellVisit, ReassignmentEvent, ZoneCompletion, ZoneOutcome
from parkscan_core.lot import LotSpec, Zone
from parkscan_core.types import CoordView, Pose2D, RobotId, RobotState, ZoneState

__all__ = [
    "HYSTERESIS_DELTA_M",
    "MAX_REASSIGNMENTS_PER_ZONE",
    "REASSIGN_COOLDOWN_NS",
    "AllocationRequest",
    "Allocator",
    "Assignment",
    "AssignmentLike",
    "Coordinator",
    "CoordinatorConfig",
    "CoordinatorSnapshot",
    "CostFn",
    "Decision",
    "DecisionKind",
    "IngestResult",
    "Trigger",
    "ZoneDemand",
    "ZoneStatus",
]

HYSTERESIS_DELTA_M: Final[float] = 5.0
"""Cost improvement, in metres, a reassignment must beat before it is allowed.

Named, because the number is the guard. ``delta = 0`` -- plain strict improvement -- is
not a guard at all: any noise in the cost signal crosses zero in both directions
arbitrarily often, and the fleet spends the run swapping zones instead of sweeping them.
5 m is about 4% of a zone's 124 m authored sweep, so a swap has to be worth something
before it is taken.
"""

REASSIGN_COOLDOWN_NS: Final[int] = 30 * NS_PER_S
"""Minimum interval between two changes of hands for one zone."""

MAX_REASSIGNMENTS_PER_ZONE: Final[int] = 3
"""Requeues a zone may survive. The next one flags it ``UNRECOVERABLE`` instead."""


class Trigger(StrEnum):
    """The events -- and only these -- that cause a re-solve.

    ``docs/ARCHITECTURE.md`` names four: a robot went offline, a zone completed, a robot's
    battery took it out of service, and zones were added. ``ROBOT_STUCK`` and
    ``ROBOT_RECOVERED`` are the same family (a robot became unusable; a robot became
    usable) and are listed separately only so the audit trail says which happened. What
    matters is what is absent: there is no timer here.
    """

    ROBOT_OFFLINE = "robot_offline"
    ROBOT_STUCK = "robot_stuck"
    ROBOT_RECOVERED = "robot_recovered"
    ROBOT_AVAILABLE = "robot_available"
    """A robot reported ``IDLE`` without completing a zone -- it finished charging, was
    recovered, or has just come online. Without this, a robot that leaves a charging bay
    while no other event is pending would sit idle until something unrelated happened,
    and closing that gap with a periodic re-solve is exactly what guard 3 forbids."""

    ZONE_COMPLETE = "zone_complete"
    BATTERY_LOW = "battery_low"
    ZONE_ADDED = "zone_added"


class DecisionKind(StrEnum):
    """What the coordinator did. One record per state change, for the audit trail."""

    ASSIGNED = "assigned"
    """A zone with no holder was given to a robot."""

    REASSIGNED = "reassigned"
    """A zone was taken from one robot and given to another because it priced better.
    Discretionary; this is what :func:`parkscan_coord.metrics.reassignment_churn` counts."""

    REQUEUED = "requeued"
    """A zone came back to the pool, carrying its remaining cells."""

    ZONE_COMPLETE = "zone_complete"
    ZONE_UNRECOVERABLE = "zone_unrecoverable"
    """The reassignment cap was reached. The zone is left alone, not retried."""

    ROBOT_OFFLINE = "robot_offline"
    ROBOT_STUCK = "robot_stuck"
    ROBOT_RECOVERED = "robot_recovered"
    """Lazarus re-registration accepted."""

    HELD = "held"
    """A proposed reassignment was refused by one of the anti-thrash guards. Recorded
    rather than dropped, because "why did it not move the zone" is the question a reviewer
    actually asks."""


@dataclass(frozen=True, slots=True)
class Decision:
    """One coordinator action, with the reason attached."""

    kind: DecisionKind
    stamp_ns: int
    reason: str
    robot_id: RobotId | None = None
    zone_id: str | None = None
    epoch: int | None = None
    remaining_cells: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class IngestResult:
    """What happened to one telemetry sample.

    A stale-epoch report is *dropped silently* -- no exception, no decision record, no
    state change. ``accepted=False`` with :attr:`stale_epoch` set is how a caller or a test
    observes that, and :attr:`CoordinatorSnapshot.stale_reports_dropped` counts them.
    """

    accepted: bool
    reason: str
    zone_id: str | None = None
    stale_epoch: bool = False
    decisions: tuple[Decision, ...] = ()


@dataclass(frozen=True, slots=True)
class ZoneDemand:
    """One zone as the coordinator's bookkeeping sees it, for the allocator's context."""

    zone_id: str
    state: ZoneState
    epoch: int
    assigned_robot: RobotId | None
    cells_total: tuple[str, ...]
    cells_done: tuple[str, ...]
    cells_remaining: tuple[str, ...]
    reassignments: int


@dataclass(frozen=True, slots=True)
class ZoneStatus:
    """Public view of one zone's bookkeeping."""

    zone_id: str
    state: ZoneState
    epoch: int
    assigned_robot: RobotId | None
    cells_total: tuple[str, ...]
    cells_done: tuple[str, ...]
    reassignments: int
    last_change_ns: int | None
    completed_ns: int | None

    @property
    def cells_remaining(self) -> tuple[str, ...]:
        """Planned cells still outstanding -- the requeue payload."""
        done = set(self.cells_done)
        return tuple(cell for cell in self.cells_total if cell not in done)

    @property
    def fraction(self) -> float:
        """Covered fraction of the **planned** cells, in ``[0, 1]``."""
        if not self.cells_total:
            return 1.0
        planned = set(self.cells_total)
        return len(planned.intersection(self.cells_done)) / len(planned)


@dataclass(frozen=True, slots=True)
class AllocationRequest:
    """Everything the allocator needs, as one immutable value.

    ``robots`` holds only robots that are **available**: alive by the coordinator's
    judgement, not latched offline, and reporting ``IDLE``. ``zones`` holds every zone that
    still needs work, *including* zones a robot currently holds -- offering those is what
    makes a cost-improving swap possible at all, and the coordinator (not the allocator)
    then decides whether the swap survives the anti-thrash guards.

    Both tuples are sorted by id, so a strategy's tie-breaking is reproducible.
    ``demands[i]`` describes ``zones[i]``.
    """

    now_ns: int
    triggers: tuple[Trigger, ...]
    lot: LotSpec
    params: BatteryParams
    robots: tuple[RobotSnapshot, ...]
    zones: tuple[Zone, ...]
    demands: tuple[ZoneDemand, ...]

    @property
    def trigger(self) -> Trigger:
        """The first trigger of this round. ``triggers`` carries the rest."""
        return self.triggers[0]

    def demand_for(self, zone_id: str) -> ZoneDemand:
        for demand in self.demands:
            if demand.zone_id == zone_id:
                return demand
        msg = f"{zone_id!r} is not in this allocation request"
        raise KeyError(msg)


class AssignmentLike(Protocol):
    """Structural type of one allocator proposal: a robot and the zone it should take.

    A ``Protocol`` rather than a concrete class so that
    :class:`parkscan_coord.allocation.Assignment` satisfies it without this module
    importing the allocator, and the two halves of the package stay independently
    testable.
    """

    @property
    def robot_id(self) -> RobotId: ...

    @property
    def zone_id(self) -> str: ...


@dataclass(frozen=True, slots=True)
class Assignment:
    """A concrete :class:`AssignmentLike`, for tests and for callers without an allocator."""

    robot_id: RobotId
    zone_id: str


class Allocator(Protocol):
    """Anything that turns an :class:`AllocationRequest` into proposals.

    ``parkscan_coord.allocation.allocate`` is wired in by the runner behind a one-line
    adapter. Injecting it rather than importing it keeps the coordinator's recovery logic
    testable against a deliberately hostile stub allocator -- which is how the anti-thrash
    guards get proved.

    A proposal is a *request*, not an order: the coordinator applies the four guards and
    may refuse it.
    """

    def __call__(self, request: AllocationRequest) -> Sequence[AssignmentLike]: ...


class CostFn(Protocol):
    """Prices one ``(robot, zone)`` pair. Defaults to
    :func:`parkscan_coord.cost.assignment_cost`, the ADR-0005 cost every strategy shares."""

    def __call__(self, lot: LotSpec, robot: RobotSnapshot, zone: Zone) -> float: ...


#: How long a bay grant lives before it must be renewed. Renewed every tick the holder
#: is RETURNING or CHARGING (docs/adr/0007), so the exact value barely matters as long as
#: it comfortably exceeds one ingest cycle; it exists to reclaim a bay from a robot that
#: stops reporting (the "dead holder" case ADR-0007's lease TTL is for).
BAY_LEASE_TTL_NS: Final[int] = 10 * NS_PER_S


def _bay_resource(bay_id: str) -> str:
    """The lease resource name for one charging bay."""
    return f"bay:{bay_id}"


@dataclass(frozen=True, slots=True)
class CoordinatorConfig:
    """The four anti-thrash guards' tuning, in one frozen value."""

    hysteresis_delta_m: float = HYSTERESIS_DELTA_M
    reassign_cooldown_ns: int = REASSIGN_COOLDOWN_NS
    max_reassignments_per_zone: int = MAX_REASSIGNMENTS_PER_ZONE
    bay_lease_ttl_ns: int = BAY_LEASE_TTL_NS

    def __post_init__(self) -> None:
        if self.hysteresis_delta_m <= 0.0:
            msg = (
                "hysteresis_delta_m must be positive; a delta of zero is plain strict "
                f"improvement, which is not a guard against a noisy cost signal "
                f"(got {self.hysteresis_delta_m})"
            )
            raise ValueError(msg)
        if self.reassign_cooldown_ns < 0:
            msg = f"reassign_cooldown_ns must be non-negative, got {self.reassign_cooldown_ns}"
            raise ValueError(msg)
        if self.max_reassignments_per_zone < 0:
            msg = (
                "max_reassignments_per_zone must be non-negative, got "
                f"{self.max_reassignments_per_zone}"
            )
            raise ValueError(msg)
        if self.bay_lease_ttl_ns <= 0:
            msg = f"bay_lease_ttl_ns must be positive, got {self.bay_lease_ttl_ns}"
            raise ValueError(msg)


@dataclass(frozen=True, slots=True)
class CoordinatorSnapshot:
    """The whole coordinator as one value, for the dashboard and for assertions."""

    now_ns: int
    zones: tuple[ZoneStatus, ...]
    health: tuple[HealthAssessment, ...]
    assignments_total: int
    voluntary_reassignments: int
    forced_requeues: int
    stale_reports_dropped: int
    pending_triggers: tuple[Trigger, ...]


@dataclass(slots=True)
class _Recovery:
    """Why a zone is back in the pool, so the reassignment latency can be measured."""

    detected_ns: int
    failed_robot: RobotId
    cause: str


@dataclass(slots=True)
class _ZoneRecord:
    zone: Zone
    cells_total: tuple[str, ...]
    state: ZoneState = ZoneState.UNASSIGNED
    epoch: int = 0
    assigned_robot: RobotId | None = None
    cells_done: set[str] = field(default_factory=set)
    reassignments: int = 0
    last_change_ns: int | None = None
    completed_ns: int | None = None
    recovery: _Recovery | None = None


class Coordinator:
    """Central allocation, failure detection and recovery for one fleet.

    Usage is a tick loop the runner owns::

        coordinator.start(now_ns)               # register robots, queue ZONE_ADDED
        while running:
            for agent in agents.values():
                coordinator.ingest(agent.tick())
            coordinator.tick(backend.now_ns())  # health sweep + event-triggered re-solve

    :meth:`tick` is safe to call every step: the health sweep is cheap and the re-solve
    only happens when something actually changed (anti-thrash guard 3).
    """

    def __init__(
        self,
        lot: LotSpec,
        agents: Mapping[RobotId, RobotAgent],
        *,
        allocator: Allocator,
        config: CoordinatorConfig | None = None,
        battery: BatteryParams | None = None,
        agent_config: AgentConfig | None = None,
        cost_fn: CostFn = assignment_cost,
        monitor: HealthMonitor | None = None,
    ) -> None:
        if not agents:
            msg = "a coordinator needs at least one robot"
            raise ValueError(msg)

        self._lot = lot
        self._agents = dict(agents)
        self._allocator = allocator
        self._cfg = config if config is not None else CoordinatorConfig()
        self._cost_fn = cost_fn
        self._monitor = monitor if monitor is not None else HealthMonitor()
        self._agent_cfg = self._resolve_agent_config(agent_config)
        self._battery = battery if battery is not None else self._agent_cfg.battery

        self._zones: dict[str, _ZoneRecord] = {
            zone.id: _ZoneRecord(zone=zone, cells_total=planned_cells(zone, lot, self._agent_cfg))
            for zone in lot.zones
        }
        self._bay_leases = LeaseManager()
        self._targeted_bay: dict[RobotId, str] = {}
        """Which bay each returning/waiting/charging robot is currently pointed at.

        Tracked independently of the lease grant so :meth:`_manage_bay_lease` can tell
        "already pursuing this bay, nothing to do" from "just granted a DIFFERENT bay
        than the one already in flight, must redirect" without reaching into the agent's
        private state.
        """

        self._views: dict[RobotId, CoordView] = {}
        self._reported: dict[RobotId, RobotState] = {}
        self._poses: dict[RobotId, Pose2D] = {}
        self._soc: dict[RobotId, float] = {}
        self._pending: list[Trigger] = []

        self._decisions: list[Decision] = []
        self._reassignments: list[ReassignmentEvent] = []
        self._completions: list[ZoneCompletion] = []
        self._visits: set[tuple[RobotId, str]] = set()

        self._assignments_total = 0
        self._voluntary = 0
        self._forced = 0
        self._stale_dropped = 0

    def _resolve_agent_config(self, supplied: AgentConfig | None) -> AgentConfig:
        """The config the coordinator plans cells with.

        It must match the fleet's, or the coordinator's idea of "all cells of zone Z"
        differs from the robots' and coverage never reaches 100% for reasons no test would
        explain. So when it is not supplied it is taken from the agents, and disagreement
        is an error rather than a silent choice of one robot's opinion.
        """
        if supplied is not None:
            return supplied
        configs = [self._agents[robot_id].config for robot_id in sorted(self._agents)]
        first = configs[0]
        for robot_id, cfg in zip(sorted(self._agents), configs, strict=True):
            if cfg != first:
                msg = (
                    f"robot {robot_id!r} has a different AgentConfig from the rest of the "
                    "fleet; pass agent_config= explicitly so the coordinator's cell "
                    "accounting matches the robots'"
                )
                raise ValueError(msg)
        return first

    # ----------------------------------------------------------------- properties --
    @property
    def lot(self) -> LotSpec:
        return self._lot

    @property
    def config(self) -> CoordinatorConfig:
        return self._cfg

    @property
    def monitor(self) -> HealthMonitor:
        return self._monitor

    @property
    def decisions(self) -> tuple[Decision, ...]:
        """Every action taken, in order. The audit trail."""
        return tuple(self._decisions)

    @property
    def reassignment_events(self) -> tuple[ReassignmentEvent, ...]:
        """Recovery events, for :func:`parkscan_coord.metrics.reassignment_latency_s`."""
        return tuple(self._reassignments)

    @property
    def zone_completions(self) -> tuple[ZoneCompletion, ...]:
        return tuple(self._completions)

    @property
    def cell_visits(self) -> tuple[CellVisit, ...]:
        """Distinct ``(robot, cell)`` pairs, for ``duplicate_visit_rate``."""
        return tuple(
            CellVisit(robot_id=robot_id, cell_id=cell) for robot_id, cell in sorted(self._visits)
        )

    @property
    def assignments_total(self) -> int:
        return self._assignments_total

    @property
    def voluntary_reassignments(self) -> int:
        """Cost-improving swaps. The numerator of ``reassignment_churn``."""
        return self._voluntary

    @property
    def forced_requeues(self) -> int:
        """Requeues caused by a failure or a battery break-off. Not churn."""
        return self._forced

    @property
    def stale_reports_dropped(self) -> int:
        """Progress reports refused by the epoch fence."""
        return self._stale_dropped

    def zone_status(self, zone_id: str) -> ZoneStatus:
        record = self._zones.get(zone_id)
        if record is None:
            msg = f"{zone_id!r} is not a zone of lot {self._lot.name!r}"
            raise KeyError(msg)
        return self._status(record)

    def zone_outcomes(self) -> tuple[ZoneOutcome, ...]:
        """Final per-zone coverage, for :func:`parkscan_coord.metrics.coverage_pct`."""
        return tuple(
            ZoneOutcome(
                zone_id=record.zone.id,
                cells_total=len(record.cells_total),
                cells_done=len(set(record.cells_total).intersection(record.cells_done)),
                state=record.state,
            )
            for _, record in sorted(self._zones.items())
        )

    def snapshot(self, now_ns: int) -> CoordinatorSnapshot:
        return CoordinatorSnapshot(
            now_ns=now_ns,
            zones=tuple(self._status(record) for _, record in sorted(self._zones.items())),
            health=self._monitor.assess_all(now_ns),
            assignments_total=self._assignments_total,
            voluntary_reassignments=self._voluntary,
            forced_requeues=self._forced,
            stale_reports_dropped=self._stale_dropped,
            pending_triggers=tuple(self._pending),
        )

    # ------------------------------------------------------------------- lifecycle --
    def register(
        self,
        robot_id: RobotId,
        now_ns: int,
        *,
        pose: Pose2D | None = None,
        battery_soc: float = 1.0,
    ) -> None:
        """Begin tracking one robot's health, optionally seeding its pose and charge.

        The pose is optional because the natural runner loop ingests telemetry before the
        first re-solve, which supplies it. Seeding matters only if the first
        :meth:`resolve` happens before any telemetry has arrived: a robot whose position
        the coordinator has never observed cannot be priced, so it is not offered work.
        """
        if robot_id not in self._agents:
            msg = f"{robot_id!r} is not a robot of this fleet"
            raise KeyError(msg)
        self._monitor.register(robot_id, now_ns)
        self._views[robot_id] = CoordView.ONLINE
        if pose is not None:
            self._poses[robot_id] = pose
            self._soc[robot_id] = battery_soc

    def start(self, now_ns: int, *, poses: Mapping[RobotId, Pose2D] | None = None) -> None:
        """Register the whole fleet and queue the initial ``ZONE_ADDED`` allocation.

        Queues rather than allocates: the first assignment goes out on the next
        :meth:`tick`, through exactly the same event-triggered path as every later one.
        There is no separate bootstrap code path to get wrong.
        """
        for robot_id in sorted(self._agents):
            pose = None if poses is None else poses.get(robot_id)
            self.register(robot_id, now_ns, pose=pose)
        self._queue(Trigger.ZONE_ADDED)

    # -------------------------------------------------------------------- ingestion --
    def ingest(self, sample: TelemetrySample) -> IngestResult:
        """Consume one telemetry sample: heartbeat, progress, and state transitions.

        Refusals, in order:

        1. An unknown robot.
        2. A robot **latched offline**. Its telemetry is not even counted as a heartbeat --
           if it were, a dead robot could resurrect itself simply by resuming publication,
           which is the Lazarus hazard. :meth:`re_register` is the only way back.
        3. A stale **epoch**. Dropped silently: the sample still counts as a heartbeat,
           because a stale report proves the robot is alive even though it proves nothing
           about the zone. The fence guards progress, not liveness.
        """
        robot_id = sample.robot_id
        if robot_id not in self._agents:
            return IngestResult(accepted=False, reason=f"{robot_id!r} is not a robot of this fleet")

        if self._monitor.is_latched_offline(robot_id):
            return IngestResult(
                accepted=False,
                reason=(
                    f"{robot_id!r} was declared offline; it must re-register before its "
                    "reports are accepted again"
                ),
            )

        self._monitor.on_telemetry(sample)
        self._poses[robot_id] = sample.pose
        self._soc[robot_id] = sample.battery_soc
        previous_state = self._reported.get(robot_id)
        self._reported[robot_id] = sample.state
        if sample.state is RobotState.IDLE and previous_state is not RobotState.IDLE:
            self._queue(Trigger.ROBOT_AVAILABLE)

        self._manage_bay_lease(sample)

        if sample.zone_id is None:
            return IngestResult(accepted=True, reason="telemetry only, no assignment")

        record = self._zones.get(sample.zone_id)
        if record is None:
            return IngestResult(
                accepted=False,
                reason=f"{sample.zone_id!r} is not a zone of lot {self._lot.name!r}",
                zone_id=sample.zone_id,
            )

        if sample.epoch != record.epoch:
            self._stale_dropped += 1
            return IngestResult(
                accepted=False,
                reason=(
                    f"stale epoch: report {sample.epoch} against zone epoch {record.epoch}; dropped"
                ),
                zone_id=sample.zone_id,
                stale_epoch=True,
            )

        if record.assigned_robot != robot_id:
            self._stale_dropped += 1
            return IngestResult(
                accepted=False,
                reason=(
                    f"{sample.zone_id!r} is held by {record.assigned_robot!r}, not "
                    f"{robot_id!r}; dropped"
                ),
                zone_id=sample.zone_id,
                stale_epoch=True,
            )

        record.cells_done.update(sample.covered_cells)
        for cell in sample.covered_cells:
            self._visits.add((robot_id, cell))

        decisions = self._on_reported_state(record, sample)
        return IngestResult(
            accepted=True,
            reason="progress accepted",
            zone_id=sample.zone_id,
            decisions=decisions,
        )

    def on_connection(self, event: ConnectionEvent) -> tuple[Decision, ...]:
        """Consume one broker liveness signal and act on it immediately.

        A Last Will does not wait for a heartbeat deadline -- that is the whole reason it is
        worth having -- so the verdict is applied here rather than on the next tick. The
        re-solve it triggers still waits for :meth:`tick`, like every other trigger.
        """
        self._monitor.on_connection(event)
        return self._sweep_health(event.stamp_ns)

    def re_register(self, robot_id: RobotId, now_ns: int) -> tuple[Decision, ...]:
        """Lazarus: accept a robot the coordinator declared dead, on explicit terms.

        The robot is forced to ``OFFLINE`` and then recovered, which makes
        :class:`~parkscan_agent.robot.RobotAgent` drop its route, its sweep and its order.
        It comes back ``IDLE`` with no assignment and receives a **fresh epoch** when it is
        next given work, so any report still in flight from its previous life is fenced
        out. There is no path here that lets it resume what it was doing.

        Raises:
            KeyError: unknown robot.
            ValueError: the robot was never declared dead. Re-registration is not a
                general-purpose reset.
        """
        agent = self._agents.get(robot_id)
        if agent is None:
            msg = f"{robot_id!r} is not a robot of this fleet"
            raise KeyError(msg)
        if not self._monitor.is_latched_offline(robot_id):
            msg = (
                f"{robot_id!r} has not been declared offline; re-registration exists to "
                "revive a robot the coordinator judged dead, not to reset a live one"
            )
            raise ValueError(msg)

        if is_legal(agent.state, Event.COMMS_LOST):
            agent.mark_offline("coordinator declared this robot dead")
        agent.recover()
        # Lazarus abandons the old order outright (module docstring); any bay lease or
        # queue position from the previous life goes with it, not carried into the fresh
        # epoch.
        self._forfeit_bay(robot_id)

        self._monitor.clear_offline_latch(robot_id, now_ns)
        self._views[robot_id] = CoordView.ONLINE
        self._queue(Trigger.ROBOT_RECOVERED)
        return (
            self._record(
                DecisionKind.ROBOT_RECOVERED,
                now_ns,
                f"{robot_id!r} re-registered; previous order abandoned, awaiting a fresh epoch",
                robot_id=robot_id,
            ),
        )

    # ------------------------------------------------------------------- the loop --
    def tick(self, now_ns: int) -> tuple[Decision, ...]:
        """Judge every robot, then re-solve **only if** something triggered one.

        Anti-thrash guard 3 lives in the second half of that sentence: there is no periodic
        re-solve, so calling ``tick`` at 20 Hz costs three health assessments and changes
        nothing unless an event occurred.
        """
        decisions = list(self._sweep_health(now_ns))
        if self._pending:
            decisions.extend(self.resolve(now_ns))
        return tuple(decisions)

    def resolve(self, now_ns: int, *, trigger: Trigger | None = None) -> tuple[Decision, ...]:
        """Run one allocation round and apply what survives the guards.

        Normally driven by :meth:`tick` from queued triggers. ``trigger=`` forces a round
        with no event pending, which is what the runner uses when zones are added
        mid-run -- and what the thrash test uses to prove that even 50 forced rounds over a
        noisy cost signal move nothing.
        """
        if trigger is not None:
            self._queue(trigger)
        triggers = tuple(self._pending)
        self._pending.clear()
        if not triggers:
            return ()

        request = self._build_request(now_ns, triggers)
        if not request.robots or not request.zones:
            return ()
        return self._apply(self._allocator(request), request)

    # ------------------------------------------------------- health -> decisions --
    def _sweep_health(self, now_ns: int) -> tuple[Decision, ...]:
        """Turn changes in the coordinator's judgement into recovery actions."""
        decisions: list[Decision] = []
        for assessment in self._monitor.assess_all(now_ns):
            robot_id = assessment.robot_id
            if robot_id not in self._agents:
                continue
            previous = self._views.get(robot_id, CoordView.ONLINE)
            self._views[robot_id] = assessment.view
            if assessment.view is CoordView.OFFLINE and previous is not CoordView.OFFLINE:
                decisions.extend(self._declare_offline(assessment, now_ns))
            elif assessment.view is CoordView.STUCK and previous is not CoordView.STUCK:
                decisions.extend(self._declare_stuck(assessment, now_ns))
        return tuple(decisions)

    def _declare_offline(self, assessment: HealthAssessment, now_ns: int) -> tuple[Decision, ...]:
        robot_id = assessment.robot_id
        agent = self._agents[robot_id]
        decisions = [
            self._record(
                DecisionKind.ROBOT_OFFLINE,
                now_ns,
                assessment.reason,
                robot_id=robot_id,
            )
        ]
        if is_legal(agent.state, Event.COMMS_LOST):
            agent.mark_offline(assessment.reason)
        self._forfeit_bay(robot_id)
        decisions.extend(
            self._release(robot_id, now_ns, cause="robot declared offline", detected_ns=now_ns)
        )
        self._queue(Trigger.ROBOT_OFFLINE)
        return tuple(decisions)

    def _declare_stuck(self, assessment: HealthAssessment, now_ns: int) -> tuple[Decision, ...]:
        """Take the order off a robot that has stopped making progress.

        ``STUCK`` is not latched, unlike ``OFFLINE``: the robot is still talking, so it is
        still a robot. Once its order is aborted it reports ``IDLE``, which
        :data:`~parkscan_coord.health.STUCK_EXEMPT_STATES` excludes, so it becomes
        available again and may be given different work. If it is *physically* wedged it
        will stick again, and the per-zone reassignment cap is what stops that from
        becoming a livelock -- the zone is flagged ``UNRECOVERABLE`` rather than retried
        forever. Quarantining the robot itself is not implemented.
        """
        robot_id = assessment.robot_id
        agent = self._agents[robot_id]
        decisions = [
            self._record(DecisionKind.ROBOT_STUCK, now_ns, assessment.reason, robot_id=robot_id)
        ]
        if is_legal(agent.state, Event.ABORT):
            agent.abort()
        self._forfeit_bay(robot_id)
        decisions.extend(
            self._release(robot_id, now_ns, cause="robot declared stuck", detected_ns=now_ns)
        )
        self._queue(Trigger.ROBOT_STUCK)
        return tuple(decisions)

    # --------------------------------------------------------------- bay arbitration --
    def _manage_bay_lease(self, sample: TelemetrySample) -> None:
        """Arbitrate which robot may approach which charging bay, right now.

        Every robot picks its own nearest bay independently
        (:func:`~parkscan_agent.battery_policy.nearest_bay`, a pure function of position)
        with no coordination at all. A Phase 6 review found that this lets two robots
        converge on the SAME bay and collide during the final approach, where local
        avoidance is deliberately switched off (docs/adr/0007 -- reproduced there as two
        robots overlapping by 0.005 m). The state machine already carries the vocabulary
        arbitration needs -- ``Event.LEASE_DENIED`` / ``Event.BAY_LEASE_GRANTED`` and
        :meth:`~parkscan_agent.robot.RobotAgent.hold` / ``.resume`` exist specifically for
        a caller that arbitrates a shared resource -- and this method is that caller.

        Three fixes, layered on top of each other, each found by running the previous one
        and watching what broke next:

        1. **Arbitration.** At most one robot is ever told to proceed toward a given bay;
           the :class:`~parkscan_coord.leases.LeaseManager` grant decides who.
        2. **Physical handoff.** The lease outlives ``CHARGING``. Releasing it the instant
           the holder reports ``IDLE`` is logically correct and physically wrong -- a
           robot that finishes charging does not teleport away, it sits exactly where it
           was until given a new zone. Releasing on ``IDLE`` let the very next waiter be
           granted a bay whose spur was still physically blocked, and the approaching
           robot got wedged behind a stationary body it could slow for but never get
           around, running out ``DockConfig.max_ticks`` and draining to 0% -- a criterion
           C2 violation, not an inefficiency. So the lease is held until the former
           occupant is off the working states AND at least
           ``AgentConfig.dock.approach_radius_m`` from the bay -- the same radius the
           docking controller itself uses, reused rather than inventing a second one.
        3. **Try every REACHABLE bay before queueing at all.** Even with (1) and (2), a
           robot denied its single nearest bay simply queued -- and queueing has NO upper
           bound: it can take as long as the current holder's entire charge cycle
           (measured here at roughly 540 s). A robot's own return-trigger
           (:func:`~parkscan_agent.battery_policy.should_return`) budgets travel, docking
           and a fixed reserve for the NEAREST bay only, never a wait and never a farther
           alternative, so a robot that queued for its nearest bay while a FARTHER bay sat
           completely free could -- and, on this two-bay lot with three robots, reliably
           did, even from a comfortable 35% starting charge, once two robots happened to
           return around the same time -- drain to empty waiting for a bay it never needed
           to wait for. So every bay is tried, nearest first
           (:func:`~parkscan_agent.battery_policy.ranked_bays`), and only a robot denied
           every REACHABLE one ever queues.

           Reachable, not merely tried, is the load-bearing word, and it was not in the
           first version of this fix. Blindly redirecting to whatever bay is free is
           worse than queueing when that bay is farther than the robot's remaining charge
           can cover: queueing at least holds position and spends nothing, where a doomed
           detour actively burns what is left. Measured directly -- a robot with just
           enough charge for its 12 m nearest bay, redirected on that basis to a 62 m
           alternate, drained to exactly 0.0 four thousand ticks later having covered
           under a fifth of the distance. So a bay only counts as a candidate at all if
           ``sample.battery_soc`` clears
           :func:`~parkscan_agent.battery_policy.required_soc` for ITS distance, not the
           nearest one's; a robot with insufficient charge for every bay falls back to
           trying the nearest anyway, since refusing to act at all helps nothing further.

           A robot the coordinator moves onto a bay other than the one it started toward
           is redirected via :meth:`~parkscan_agent.robot.RobotAgent.retarget_bay`, which
           swaps the docking target without touching ``RobotState`` -- the return itself
           was already decided by the robot; only which bay serves it is this method's
           business.

        Scope, stated plainly. This still does not wire a
        :class:`~parkscan_coord.leases.ResourceController` or fencing tokens into the
        physical docking action, so the "resurrected holder" hazard ADR-0007 already
        documents remains -- unchanged, only narrower. And with only two bays, a genuine
        THREE-way simultaneous return still queues one robot with the same no-upper-bound
        wait (3) describes; trying every bay only helps while at least one is free. That
        residual is not eliminated, only made much rarer -- two-way contention, the
        common case demonstrated above, no longer queues at all. Nor does this cover a
        robot that dies or is declared stuck *while parked at the bay*: forfeiting the
        lease then (see :meth:`_forfeit_bay`) is unconditional, because a dead robot may
        never move away to satisfy a clearance check, and refusing to ever reclaim the bay
        would be worse -- a second robot could still be sent into a spur blocked by a
        corpse. There is no coordination fix for a robot that cannot move; that is a
        robotics problem, not an allocation one.
        """
        robot_id = sample.robot_id
        agent = self._agents[robot_id]
        grant = self._bay_leases.grant_of(robot_id)

        if sample.state not in (
            RobotState.RETURNING,
            RobotState.WAITING_ON_LEASE,
            RobotState.CHARGING,
        ):
            # Anything else -- IDLE, FAILED, INSPECTING, TRANSIT, ASSIGNED -- means this
            # robot is no longer working the bay. But "no longer working it" and "no
            # longer standing in it" are different facts (fix 2 above), so the lease is
            # held until the robot has actually put distance behind it.
            if grant is not None:
                if self._clear_of_bay(sample.pose, grant):
                    self._bay_leases.release_all(robot_id)
                    self._targeted_bay.pop(robot_id, None)
                else:
                    self._bay_leases.renew(robot_id, sample.stamp_ns, self._cfg.bay_lease_ttl_ns)
            return

        if sample.state is RobotState.CHARGING:
            # Already holds the bay it is sitting in; keep the lease from expiring under
            # it mid-charge. Renewed every tick rather than TTL-sized to the charge
            # duration, which this layer has no honest way to predict.
            if grant is not None:
                self._bay_leases.renew(robot_id, sample.stamp_ns, self._cfg.bay_lease_ttl_ns)
            return

        # RETURNING or WAITING_ON_LEASE: heading for, or queued for, a bay.
        if grant is not None:
            self._bay_leases.renew(robot_id, sample.stamp_ns, self._cfg.bay_lease_ttl_ns)
            return

        candidates = ranked_bays(self._lot, sample.pose.x_m, sample.pose.y_m)

        # Reachability, not just distance order. The robot's own return-trigger
        # (`should_return`) budgets exactly enough charge for the NEAREST bay plus a
        # fixed margin -- it has no reason to budget for anything farther. Redirecting it
        # to a farther bay it cannot actually reach on that budget is not a fallback, it
        # is a worse failure than queueing: queueing at least holds position and spends
        # nothing, where a doomed detour actively burns the little charge left. Measured
        # here directly -- a robot with just enough charge for its 12 m nearest bay,
        # redirected to a 62 m alternate, drained to exactly 0.0 four thousand ticks later
        # having covered less than a fifth of the distance. So only a bay this robot can
        # actually finish reaching is ever attempted or redirected to.
        reachable = [
            bay
            for bay in candidates
            if sample.battery_soc >= required_soc(bay.distance_m, self._battery)
        ]
        # A robot with insufficient charge for EVERY bay is already lost regardless of
        # what this method does; falling back to the full list rather than an empty one
        # at least tries the nearest option instead of refusing to act at all.
        targets = reachable if reachable else candidates

        if robot_id not in self._targeted_bay:
            # First sighting of this return. The robot's OWN `_begin_return` already
            # picked `nearest_bay` -- exactly `candidates[0]` -- on its own initiative,
            # before this method ever ran, and by construction it was reachable when
            # chosen (that is what made `should_return` fire). Seeding the bookkeeping
            # with that fact, rather than leaving it unset, is what lets the "did the
            # grant change anything" check below tell a genuine redirect apart from the
            # ordinary case where the nearest bay was free and the robot is already
            # correctly pointed at it.
            self._targeted_bay[robot_id] = candidates[0].bay_id

        for bay in targets:
            resource = _bay_resource(bay.bay_id)
            new_grant = self._bay_leases.acquire_all(
                robot_id, (resource,), sample.stamp_ns, self._cfg.bay_lease_ttl_ns
            )
            if new_grant is None:
                continue

            if self._targeted_bay[robot_id] != bay.bay_id and agent.state in (
                RobotState.RETURNING,
                RobotState.WAITING_ON_LEASE,
            ):
                agent.retarget_bay(bay.bay_id, sample.pose)
            self._targeted_bay[robot_id] = bay.bay_id

            if sample.state is RobotState.WAITING_ON_LEASE and is_legal(
                agent.state, Event.BAY_LEASE_GRANTED
            ):
                agent.resume()
            return

        # Every REACHABLE bay is occupied (or none is reachable at all). Queue for the
        # nearest of `targets` -- the aging FIFO means this is not a permanent
        # commitment, and the very next tick re-tries every reachable bay in order again,
        # so a nearer one freeing up first is taken immediately rather than waited past.
        nearest = targets[0]
        if sample.state is RobotState.RETURNING and is_legal(agent.state, Event.LEASE_DENIED):
            resource = _bay_resource(nearest.bay_id)
            holder = self._bay_leases.holder(resource)
            self._targeted_bay[robot_id] = nearest.bay_id
            agent.hold(f"{resource} held by {holder!r}; queued")
        # else: already WAITING_ON_LEASE and still denied everywhere -- stay queued.

    def _clear_of_bay(self, pose: Pose2D, grant: Grant) -> bool:
        """Has the grant's holder put the docking radius behind it, physically?

        ``grant.resources`` carries the resource name (``"bay:<id>"``); the id is looked
        up against the lot's own charging-bay list rather than trusting anything derived
        from it, so a malformed resource name fails loudly (a crash) instead of silently
        treating every bay as vacated.
        """
        bay_id = grant.resources[0].removeprefix("bay:")
        bay = next(b for b in self._lot.charging_bays if b.id == bay_id)
        distance = math.hypot(pose.x_m - bay.position[0], pose.y_m - bay.position[1])
        return distance >= self._agent_cfg.dock.approach_radius_m

    def bay_lease_holder(self, bay_id: str) -> RobotId | None:
        """Which robot currently holds ``bay_id``'s lease, if any. For tests and telemetry."""
        return self._bay_leases.holder(_bay_resource(bay_id))

    def _forfeit_bay(self, robot_id: RobotId) -> None:
        """Give up anything ``robot_id`` holds or is queued for.

        Two calls, not one: :meth:`~parkscan_coord.leases.LeaseManager.release_all` only
        clears a resource the robot actually HOLDS, and a robot merely queued for a bay
        holds nothing by construction (that is the whole point of ADR-0007's hold-and-wait
        break). Without also calling
        :meth:`~parkscan_coord.leases.LeaseManager.cancel_wait` -- which its own
        docstring says to call exactly here, "when you declare it OFFLINE" -- a robot
        declared offline, stuck, or Lazarus-recovered while queued would sit in the wait
        list forever, since ``QueueParams.abandon_after_ns`` is disabled by default.
        """
        self._bay_leases.release_all(robot_id)
        self._bay_leases.cancel_wait(robot_id)
        self._targeted_bay.pop(robot_id, None)

    def _on_reported_state(
        self, record: _ZoneRecord, sample: TelemetrySample
    ) -> tuple[Decision, ...]:
        """React to what an accepted, current-epoch report says the robot is doing."""
        state = sample.state
        if state is RobotState.IDLE:
            return (self._complete(record, sample.stamp_ns),)

        if state is RobotState.RETURNING:
            decisions = self._release(
                sample.robot_id,
                sample.stamp_ns,
                cause="robot broke off to charge",
                detected_ns=sample.stamp_ns,
            )
            self._queue(Trigger.BATTERY_LOW)
            return decisions

        if state is RobotState.FAILED:
            decisions = self._release(
                sample.robot_id,
                sample.stamp_ns,
                cause="robot reported a local fault",
                detected_ns=sample.stamp_ns,
            )
            self._queue(Trigger.ROBOT_OFFLINE)
            return decisions

        if state is RobotState.INSPECTING and record.state is ZoneState.ASSIGNED:
            record.state = ZoneState.IN_PROGRESS
        return ()

    def _complete(self, record: _ZoneRecord, stamp_ns: int) -> Decision:
        """Bank a finished zone. The completing tick still names its zone, by design."""
        robot_id = record.assigned_robot
        if robot_id is not None:
            self._absorb_progress(record, robot_id)
        record.state = ZoneState.COMPLETE
        record.assigned_robot = None
        record.completed_ns = stamp_ns
        record.last_change_ns = stamp_ns
        record.recovery = None
        self._completions.append(ZoneCompletion(zone_id=record.zone.id, stamp_ns=stamp_ns))
        self._queue(Trigger.ZONE_COMPLETE)
        return self._record(
            DecisionKind.ZONE_COMPLETE,
            stamp_ns,
            f"{record.zone.id!r} swept by {robot_id!r}",
            robot_id=robot_id,
            zone_id=record.zone.id,
            epoch=record.epoch,
        )

    # --------------------------------------------------------------- requeue path --
    def _release(
        self, robot_id: RobotId, now_ns: int, *, cause: str, detected_ns: int
    ) -> tuple[Decision, ...]:
        """Take whatever zone ``robot_id`` holds back into the pool, with its remainder."""
        held = [
            record for _, record in sorted(self._zones.items()) if record.assigned_robot == robot_id
        ]
        return tuple(
            self._requeue(record, now_ns, cause=cause, detected_ns=detected_ns, forced=True)
            for record in held
        )

    def _requeue(
        self,
        record: _ZoneRecord,
        now_ns: int,
        *,
        cause: str,
        detected_ns: int,
        forced: bool,
    ) -> Decision:
        """Return one zone to the pool, preserving the cells already covered.

        The remainder is preserved on the **zone**, so it survives the robot that produced
        it. The replacement still re-drives the zone from its entry waypoint -- see the
        module docstring's honest limit -- but the coverage accounting, and any detections
        already banked, are not thrown away.
        """
        robot_id = record.assigned_robot
        if robot_id is not None:
            self._absorb_progress(record, robot_id)

        record.assigned_robot = None
        record.reassignments += 1
        record.last_change_ns = now_ns
        if forced:
            self._forced += 1
        remaining = self._remaining(record)

        if record.reassignments > self._cfg.max_reassignments_per_zone:
            record.state = ZoneState.UNRECOVERABLE
            record.recovery = None
            return self._record(
                DecisionKind.ZONE_UNRECOVERABLE,
                now_ns,
                (
                    f"{cause}; {record.zone.id!r} has now been requeued "
                    f"{record.reassignments} times, over the cap of "
                    f"{self._cfg.max_reassignments_per_zone}. Flagged unrecoverable rather "
                    "than retried"
                ),
                robot_id=robot_id,
                zone_id=record.zone.id,
                epoch=record.epoch,
                remaining_cells=remaining,
            )

        record.state = ZoneState.UNASSIGNED
        if forced and robot_id is not None:
            record.recovery = _Recovery(detected_ns=detected_ns, failed_robot=robot_id, cause=cause)
        return self._record(
            DecisionKind.REQUEUED,
            now_ns,
            (
                f"{cause}; {record.zone.id!r} requeued with {len(remaining)} of "
                f"{len(record.cells_total)} cells outstanding"
            ),
            robot_id=robot_id,
            zone_id=record.zone.id,
            epoch=record.epoch,
            remaining_cells=remaining,
        )

    def _absorb_progress(self, record: _ZoneRecord, robot_id: RobotId) -> None:
        """Copy the robot's cell coverage for this epoch onto the zone record.

        Both the live and the stashed progress are checked: a robot that was aborted or
        broke off to charge has already released its assignment and stashed it in
        ``last_progress``, while one being marked offline still holds it. The epoch is
        matched in either case, so progress from a previous life never lands here.
        """
        agent = self._agents.get(robot_id)
        if agent is None:
            return
        for progress in (agent.progress, agent.last_progress):
            if (
                progress is not None
                and progress.zone_id == record.zone.id
                and progress.epoch == record.epoch
            ):
                record.cells_done.update(progress.cells_done)

    # -------------------------------------------------------------- allocation --
    def _build_request(self, now_ns: int, triggers: tuple[Trigger, ...]) -> AllocationRequest:
        candidates: list[RobotSnapshot] = []
        for robot_id in sorted(self._agents):
            if not self._available(robot_id, now_ns):
                continue
            pose = self._poses.get(robot_id)
            if pose is None:
                continue
            candidates.append(
                RobotSnapshot(
                    robot_id=robot_id,
                    pose=pose,
                    battery_soc=self._soc.get(robot_id, 1.0),
                    available=True,
                )
            )

        zones: list[Zone] = []
        demands: list[ZoneDemand] = []
        for _, record in sorted(self._zones.items()):
            if record.state in (ZoneState.COMPLETE, ZoneState.UNRECOVERABLE):
                continue
            zones.append(record.zone)
            demands.append(
                ZoneDemand(
                    zone_id=record.zone.id,
                    state=record.state,
                    epoch=record.epoch,
                    assigned_robot=record.assigned_robot,
                    cells_total=record.cells_total,
                    cells_done=tuple(sorted(record.cells_done)),
                    cells_remaining=self._remaining(record),
                    reassignments=record.reassignments,
                )
            )

        return AllocationRequest(
            now_ns=now_ns,
            triggers=triggers,
            lot=self._lot,
            params=self._battery,
            robots=tuple(candidates),
            zones=tuple(zones),
            demands=tuple(demands),
        )

    def _available(self, robot_id: RobotId, now_ns: int) -> bool:
        """Whether a robot may be given work: alive, not latched dead, and idle."""
        if self._monitor.is_latched_offline(robot_id):
            return False
        if not self._monitor.assess(robot_id, now_ns).usable:
            return False
        return self._agents[robot_id].state is RobotState.IDLE

    def _apply(
        self, proposals: Sequence[AssignmentLike], request: AllocationRequest
    ) -> tuple[Decision, ...]:
        """Apply the allocator's proposals, in id order, through the four guards.

        Proposals are re-sorted here so the outcome does not depend on the order a strategy
        happened to emit them in. A proposal naming a robot or a zone already used in this
        round is refused: an allocator returning two zones for one robot is a bug, and
        applying the first of them silently would hide it.
        """
        now_ns = request.now_ns
        decisions: list[Decision] = []
        used_robots: set[RobotId] = set()
        used_zones: set[str] = set()

        for proposal in sorted(proposals, key=lambda p: (p.zone_id, p.robot_id)):
            zone_id, robot_id = proposal.zone_id, proposal.robot_id
            record = self._zones.get(zone_id)
            if record is None:
                decisions.append(
                    self._record(
                        DecisionKind.HELD,
                        now_ns,
                        f"proposed zone {zone_id!r} is not a zone of this lot",
                        robot_id=robot_id,
                        zone_id=zone_id,
                    )
                )
                continue
            if zone_id in used_zones or robot_id in used_robots:
                decisions.append(
                    self._record(
                        DecisionKind.HELD,
                        now_ns,
                        "duplicate proposal: this robot or zone was already used this round",
                        robot_id=robot_id,
                        zone_id=zone_id,
                    )
                )
                continue
            if record.state in (ZoneState.COMPLETE, ZoneState.UNRECOVERABLE):
                decisions.append(
                    self._record(
                        DecisionKind.HELD,
                        now_ns,
                        f"{zone_id!r} is {record.state.value} and takes no more assignments",
                        robot_id=robot_id,
                        zone_id=zone_id,
                    )
                )
                continue
            if robot_id not in self._agents or not self._available(robot_id, now_ns):
                decisions.append(
                    self._record(
                        DecisionKind.HELD,
                        now_ns,
                        f"{robot_id!r} is not available for work",
                        robot_id=robot_id,
                        zone_id=zone_id,
                    )
                )
                continue

            if record.assigned_robot == robot_id:
                continue

            if record.assigned_robot is None:
                decisions.append(self._assign(record, robot_id, now_ns, voluntary=False))
                used_robots.add(robot_id)
                used_zones.add(zone_id)
                continue

            held, decision = self._consider_swap(record, robot_id, now_ns)
            decisions.extend(decision)
            if not held:
                used_robots.add(robot_id)
                used_zones.add(zone_id)

        return tuple(decisions)

    def _consider_swap(
        self, record: _ZoneRecord, robot_id: RobotId, now_ns: int
    ) -> tuple[bool, tuple[Decision, ...]]:
        """The three value-based guards, in order. Returns ``(held, decisions)``."""
        zone_id = record.zone.id
        incumbent = record.assigned_robot
        if incumbent is None:  # pragma: no cover - callers check this first
            return True, ()

        if record.reassignments >= self._cfg.max_reassignments_per_zone:
            return True, (
                self._record(
                    DecisionKind.HELD,
                    now_ns,
                    (
                        f"{zone_id!r} has been reassigned {record.reassignments} times, at "
                        f"the cap of {self._cfg.max_reassignments_per_zone}; "
                        "discretionary swaps are refused"
                    ),
                    robot_id=robot_id,
                    zone_id=zone_id,
                ),
            )

        last = record.last_change_ns
        if last is not None and now_ns - last < self._cfg.reassign_cooldown_ns:
            return True, (
                self._record(
                    DecisionKind.HELD,
                    now_ns,
                    (
                        f"{zone_id!r} changed hands {(now_ns - last) / NS_PER_S:g} s ago, "
                        f"inside the {self._cfg.reassign_cooldown_ns / NS_PER_S:g} s cooldown"
                    ),
                    robot_id=robot_id,
                    zone_id=zone_id,
                ),
            )

        current = self._price(incumbent, record.zone)
        proposed = self._price(robot_id, record.zone)
        if current is None or proposed is None:
            return True, (
                self._record(
                    DecisionKind.HELD,
                    now_ns,
                    "cannot price the swap: a pose has not been observed for both robots",
                    robot_id=robot_id,
                    zone_id=zone_id,
                ),
            )

        delta = self._cfg.hysteresis_delta_m
        if not proposed < current - delta:
            return True, (
                self._record(
                    DecisionKind.HELD,
                    now_ns,
                    (
                        f"hysteresis: {robot_id!r} prices {zone_id!r} at {proposed:.2f} m "
                        f"against {incumbent!r}'s {current:.2f} m, which does not beat it "
                        f"by the {delta:g} m delta"
                    ),
                    robot_id=robot_id,
                    zone_id=zone_id,
                ),
            )

        agent = self._agents[incumbent]
        if is_legal(agent.state, Event.ABORT):
            agent.abort()
        decisions = [
            self._requeue(
                record,
                now_ns,
                cause=(
                    f"{robot_id!r} prices {zone_id!r} {current - proposed:.2f} m better "
                    f"than {incumbent!r}, beating the {delta:g} m hysteresis delta"
                ),
                detected_ns=now_ns,
                forced=False,
            ),
            self._assign(record, robot_id, now_ns, voluntary=True),
        ]
        return False, tuple(decisions)

    def _assign(
        self, record: _ZoneRecord, robot_id: RobotId, now_ns: int, *, voluntary: bool
    ) -> Decision:
        """Hand a zone to a robot on a **fresh epoch**.

        The epoch increments here and nowhere else. Every order therefore carries a number
        strictly greater than any report still in flight from a previous holder, which is
        what makes the fence in :meth:`ingest` able to drop those reports.
        """
        record.epoch += 1
        self._agents[robot_id].assign(record.zone.id, epoch=record.epoch)
        record.assigned_robot = robot_id
        record.state = ZoneState.ASSIGNED
        record.last_change_ns = now_ns
        self._assignments_total += 1
        if voluntary:
            self._voluntary += 1

        recovery = record.recovery
        if recovery is not None:
            record.recovery = None
            if recovery.failed_robot != robot_id:
                self._reassignments.append(
                    ReassignmentEvent(
                        zone_id=record.zone.id,
                        failed_robot=recovery.failed_robot,
                        replacement_robot=robot_id,
                        detected_ns=recovery.detected_ns,
                        reassigned_ns=now_ns,
                        cause=recovery.cause,
                    )
                )

        kind = DecisionKind.REASSIGNED if voluntary else DecisionKind.ASSIGNED
        return self._record(
            kind,
            now_ns,
            f"{record.zone.id!r} assigned to {robot_id!r} at epoch {record.epoch}",
            robot_id=robot_id,
            zone_id=record.zone.id,
            epoch=record.epoch,
            remaining_cells=self._remaining(record),
        )

    # ------------------------------------------------------------------ internals --
    def _price(self, robot_id: RobotId, zone: Zone) -> float | None:
        pose = self._poses.get(robot_id)
        if pose is None:
            return None
        snapshot = RobotSnapshot(
            robot_id=robot_id,
            pose=pose,
            battery_soc=self._soc.get(robot_id, 1.0),
            available=True,
        )
        return self._cost_fn(self._lot, snapshot, zone)

    def _remaining(self, record: _ZoneRecord) -> tuple[str, ...]:
        return tuple(cell for cell in record.cells_total if cell not in record.cells_done)

    def _status(self, record: _ZoneRecord) -> ZoneStatus:
        return ZoneStatus(
            zone_id=record.zone.id,
            state=record.state,
            epoch=record.epoch,
            assigned_robot=record.assigned_robot,
            cells_total=record.cells_total,
            cells_done=tuple(sorted(record.cells_done)),
            reassignments=record.reassignments,
            last_change_ns=record.last_change_ns,
            completed_ns=record.completed_ns,
        )

    def _queue(self, trigger: Trigger) -> None:
        """Queue a re-solve trigger, deduplicated, preserving the order they arrived in."""
        if trigger not in self._pending:
            self._pending.append(trigger)

    def _record(
        self,
        kind: DecisionKind,
        stamp_ns: int,
        reason: str,
        *,
        robot_id: RobotId | None = None,
        zone_id: str | None = None,
        epoch: int | None = None,
        remaining_cells: tuple[str, ...] = (),
    ) -> Decision:
        decision = Decision(
            kind=kind,
            stamp_ns=stamp_ns,
            reason=reason,
            robot_id=robot_id,
            zone_id=zone_id,
            epoch=epoch,
            remaining_cells=remaining_cells,
        )
        self._decisions.append(decision)
        return decision

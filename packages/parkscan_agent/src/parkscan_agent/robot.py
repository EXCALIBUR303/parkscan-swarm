"""``RobotAgent``: the autonomy of one robot, tied together.

What it is
----------
:class:`RobotAgent` owns one robot's behaviour and nothing else. It reads a
:class:`~parkscan_core.world.SensorFrame` from a
:class:`~parkscan_core.world.WorldBackend`, decides, actuates, and returns a
:class:`TelemetrySample`. It does **no I/O**: no MQTT, no HTTP, no logging to disk, no
clock. The runner that owns the tick loop serialises the returned sample onto
``parkscan/v1/{run_id}/robot/{id}/telemetry`` and ``.../progress``.

That split is not tidiness. It is what makes the autonomy testable: every test in this
package drives a real agent against a ten-line fake backend, with no broker, no database
and no event loop. It is also what keeps the agent honest about provenance -- an agent that
cannot see the backend's identity cannot claim to know whether its data is simulated, so
``data_source`` is stamped by the runner, from the backend it chose, and never guessed
here.

The agent depends only on :mod:`parkscan_core`. It never imports ``parkscan_sim``, and a
test asserts that: the same autonomy has to run unchanged against a future ROS 2 backend,
which is the entire claim ADR-0001 makes.

The sweep
---------
On assignment the robot routes over the **lane graph** to its zone's entry waypoint, then
walks a back-and-forth path along the zone's aisle at
:attr:`AgentConfig.sweep_spacing_m` lateral offsets, and finishes at the exit waypoint.

Stated plainly, per ACCEPTANCE H8: **this is a lawnmower path over an authored rectangle,
not a cellular decomposition.** The zones are hand-authored in ``sim/lot/lot.yaml``, the
aisle is obstacle-free by construction, and a boustrophedon decomposition degenerates to
exactly this on such a cell. Nothing here computes a decomposition, chooses a sweep
direction, or optimises anything, and the code should not be described as if it did.

Progress reporting is deliberately **finer than the assignment granularity**: the robot
reports covered *cell* ids (a :attr:`AgentConfig.cell_size_m` grid over the zone) on the
tick it enters each one. That is what lets the coordinator requeue "zone Z, remaining
cells {...}" when a robot breaks off for charge, instead of throwing a half-swept zone away.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from itertools import pairwise

from parkscan_agent.battery_policy import BatteryParams, ReturnDecision, should_return
from parkscan_agent.docking import DockConfig, DockController, DockPhase
from parkscan_agent.lanegraph import nearest_node, node_xy, route, route_points
from parkscan_agent.navigation import STOP, NavConfig, avoid, follow_route
from parkscan_agent.statemachine import Event, step
from parkscan_core.lot import LotSpec, Zone
from parkscan_core.types import Pose2D, RobotId, RobotState, SensorFrame, TwistCmd
from parkscan_core.world import WorldBackend

__all__ = [
    "AgentConfig",
    "RobotAgent",
    "TelemetrySample",
    "ZoneProgress",
    "cell_at",
    "cells_covered_from",
    "planned_cells",
    "sweep_waypoints",
    "waypoint_sequence",
]

type Point2D = tuple[float, float]

_BATTERY_WATCHED: frozenset[RobotState] = frozenset(
    {
        RobotState.IDLE,
        RobotState.ASSIGNED,
        RobotState.TRANSIT,
        RobotState.INSPECTING,
        RobotState.WAITING_ON_LEASE,
    }
)
"""States in which the return-to-charge feasibility check runs.

Not ``RETURNING`` or ``CHARGING`` (already handled), and not ``FAILED``/``OFFLINE``
(a robot that is not driving cannot act on the answer).
"""


# ------------------------------------------------------------------------- config --
@dataclass(frozen=True, slots=True)
class AgentConfig:
    """Everything that tunes one robot. Frozen: a run's config is a value you can hash."""

    nav: NavConfig = field(default_factory=NavConfig)
    battery: BatteryParams = field(default_factory=BatteryParams)
    dock: DockConfig = field(default_factory=DockConfig)

    sweep_passes: int = 3
    """Number of along-aisle passes per zone.

    Three matches the authored ``sweep_length_m = 3 * 40 + 2 * 2 = 124`` in
    ``sim/lot/lot.yaml``. An odd count finishes at the exit waypoint rather than back at
    the entry, which is why the lot authors one.
    """

    sweep_spacing_m: float = 2.0
    """Lateral offset between passes. Passes sit symmetrically about the aisle centreline,
    so 3 passes at 2.0 m span 4 m of the 6 m aisle and stay off the kerb."""

    cell_size_m: float = 2.5
    """Coverage-cell edge, in metres. 2.5 m is the lot's stall pitch, so a covered cell
    lines up with a countable piece of the lot rather than an arbitrary grid."""

    coverage_radius_m: float = 1.0
    """Half-width of the swath a robot is credited with inspecting as it drives.

    A cell counts as covered when the robot's axis-aligned footprint box of this
    half-width overlaps it. Two reasons it is not zero, and both are worth stating rather
    than burying:

    * A point-coverage model is wrong in the robot's favour *and* against it. The camera
      sees a strip of ground to either side, not the single point under the axle.
    * The controller rounds corners within
      :attr:`~parkscan_agent.navigation.NavConfig.waypoint_tolerance_m`, so a point model
      can miss the cell at the very end of a pass and make 100% coverage unreachable --
      an accounting artefact that would look exactly like a coverage bug.

    1.0 m is a modelling choice sized to the controller's own corner tolerance and to the
    6 m aisle, not a measured sensor footprint. Nothing here claims to know what a real
    camera would resolve.
    """

    def __post_init__(self) -> None:
        if self.sweep_passes < 1:
            msg = f"sweep_passes must be at least 1, got {self.sweep_passes}"
            raise ValueError(msg)
        if self.sweep_spacing_m < 0.0:
            msg = f"sweep_spacing_m must be non-negative, got {self.sweep_spacing_m}"
            raise ValueError(msg)
        if self.cell_size_m <= 0.0:
            msg = f"cell_size_m must be positive, got {self.cell_size_m}"
            raise ValueError(msg)
        if self.coverage_radius_m < 0.0:
            msg = f"coverage_radius_m must be non-negative, got {self.coverage_radius_m}"
            raise ValueError(msg)


# ------------------------------------------------------------------------ outputs --
@dataclass(frozen=True, slots=True)
class ZoneProgress:
    """How much of one zone assignment has been swept.

    Retained after the assignment ends -- completed *or* abandoned -- because the
    coordinator's requeue needs the remainder, not a fresh start.
    """

    zone_id: str
    epoch: int
    cells_done: tuple[str, ...]
    cells_total: tuple[str, ...]

    @property
    def fraction(self) -> float:
        """Covered fraction in ``[0, 1]``. ``1.0`` for a zone with no planned cells.

        Only *planned* cells count toward the numerator. The sensor footprint sometimes
        reaches a cell the planned path never touches -- the kerb strip beside the aisle,
        say -- and letting those inflate the number would report more than 100% coverage
        of a zone, which is worse than useless.
        """
        if not self.cells_total:
            return 1.0
        planned = set(self.cells_total)
        return len(planned.intersection(self.cells_done)) / len(planned)

    @property
    def cells_remaining(self) -> tuple[str, ...]:
        """Planned cells not yet covered -- the payload of a requeue."""
        done = set(self.cells_done)
        return tuple(cell for cell in self.cells_total if cell not in done)


@dataclass(frozen=True, slots=True)
class TelemetrySample:
    """One tick of one robot, as a value. The runner serialises this; it is not published
    from here.

    ``zone_id`` and ``epoch`` describe the assignment that was live **during** this tick,
    which is the assignment ``covered_cells`` belongs to. ``state`` is the state the robot
    is in **after** the tick's transitions. On the tick where a robot breaks off a sweep,
    that means ``zone_id`` still names the abandoned zone while ``state`` already reads
    ``returning`` -- deliberately, so the final cells of the abandoned work are attributed
    to the right zone and epoch instead of being dropped.

    There is no ``data_source`` field. The agent cannot honestly know whether it is driving
    a simulator or hardware, so it does not claim to; the runner stamps provenance from the
    backend it constructed (ACCEPTANCE H1).
    """

    robot_id: RobotId
    stamp_ns: int
    pose: Pose2D
    twist: TwistCmd
    """The **measured** body twist reported by the backend, not the command."""
    command: TwistCmd
    """What this tick actuated."""
    battery_soc: float
    state: RobotState
    zone_id: str | None
    epoch: int
    covered_cells: tuple[str, ...]
    """Cells entered for the first time on this tick. Usually empty or a single id."""
    detail: str
    """Short human-readable reason for what the robot is doing. For the dashboard and for
    test failure messages; never parsed."""


# ------------------------------------------------------------------- zone geometry --
def _bounds(zone: Zone) -> tuple[float, float, float, float]:
    xs = [p[0] for p in zone.polygon]
    ys = [p[1] for p in zone.polygon]
    return min(xs), min(ys), max(xs), max(ys)


def cell_at(zone: Zone, x_m: float, y_m: float, cell_size_m: float) -> str | None:
    """Id of the coverage cell containing ``(x_m, y_m)``, or ``None`` if outside ``zone``.

    Cells tile the zone's bounding rectangle from its south-west corner. The id is
    ``"{zone_id}:c{col:02d}r{row:02d}"``: stable, sortable, and readable in a log without
    a lookup table.
    """
    if cell_size_m <= 0.0:
        msg = f"cell_size_m must be positive, got {cell_size_m}"
        raise ValueError(msg)
    x0, y0, x1, y1 = _bounds(zone)
    if not (x0 <= x_m <= x1 and y0 <= y_m <= y1):
        return None
    cols = max(1, math.ceil((x1 - x0) / cell_size_m))
    rows = max(1, math.ceil((y1 - y0) / cell_size_m))
    col = min(cols - 1, int((x_m - x0) // cell_size_m))
    row = min(rows - 1, int((y_m - y0) // cell_size_m))
    return f"{zone.id}:c{col:02d}r{row:02d}"


def cells_covered_from(
    zone: Zone, x_m: float, y_m: float, cell_size_m: float, radius_m: float
) -> tuple[str, ...]:
    """Every cell of ``zone`` overlapped by the footprint box centred on ``(x_m, y_m)``.

    The footprint is the axis-aligned square of half-width ``radius_m``; see
    :attr:`AgentConfig.coverage_radius_m` for why coverage is a swath rather than a point.
    ``radius_m == 0`` degenerates to :func:`cell_at`. The result is sorted, so a telemetry
    sample's ``covered_cells`` is reproducible.
    """
    if cell_size_m <= 0.0:
        msg = f"cell_size_m must be positive, got {cell_size_m}"
        raise ValueError(msg)
    if radius_m < 0.0:
        msg = f"radius_m must be non-negative, got {radius_m}"
        raise ValueError(msg)
    x0, y0, x1, y1 = _bounds(zone)
    lo_x, hi_x = max(x0, x_m - radius_m), min(x1, x_m + radius_m)
    lo_y, hi_y = max(y0, y_m - radius_m), min(y1, y_m + radius_m)
    if lo_x > hi_x or lo_y > hi_y:
        return ()
    cols = max(1, math.ceil((x1 - x0) / cell_size_m))
    rows = max(1, math.ceil((y1 - y0) / cell_size_m))
    col_lo = min(cols - 1, int((lo_x - x0) // cell_size_m))
    col_hi = min(cols - 1, int((hi_x - x0) // cell_size_m))
    row_lo = min(rows - 1, int((lo_y - y0) // cell_size_m))
    row_hi = min(rows - 1, int((hi_y - y0) // cell_size_m))
    return tuple(
        f"{zone.id}:c{col:02d}r{row:02d}"
        for col in range(col_lo, col_hi + 1)
        for row in range(row_lo, row_hi + 1)
    )


def sweep_waypoints(zone: Zone, lot: LotSpec, cfg: AgentConfig) -> tuple[Point2D, ...]:
    """The back-and-forth path through ``zone``, from its entry waypoint to its exit.

    Passes run along the entry-to-exit axis at lateral offsets symmetric about it; odd
    numbers of passes therefore end on the exit side, and a final leg closes onto the exit
    waypoint itself so the robot leaves the zone where the lane graph expects it.

    On the shipped lot this yields, for ``zone_01`` (entry ``(5, 5)``, exit ``(45, 5)``,
    3 passes at 2 m)::

        (5, 3) (45, 3) (45, 5) (5, 5) (5, 7) (45, 7) (45, 5)

    which is 128 m of driving against the lot's authored ``sweep_length_m = 124``. The
    authored figure is the *allocator's cost estimate*, not a promise about this path; the
    4 m difference is the two lateral legs onto and off the outermost passes. Nothing here
    reads ``sweep_length_m``.
    """
    entry = node_xy(lot, zone.entry_waypoint)
    exit_ = node_xy(lot, zone.exit_waypoint)
    dx, dy = exit_[0] - entry[0], exit_[1] - entry[1]
    span = math.hypot(dx, dy)
    if span == 0.0:  # pragma: no cover - LotSpec forbids entry == exit
        msg = f"zone {zone.id!r}: entry and exit waypoints coincide"
        raise ValueError(msg)
    perp = (-dy / span, dx / span)

    n = cfg.sweep_passes
    points: list[Point2D] = []
    for i in range(n):
        offset = (i - (n - 1) / 2.0) * cfg.sweep_spacing_m
        shift = (perp[0] * offset, perp[1] * offset)
        near, far = (entry, exit_) if i % 2 == 0 else (exit_, entry)
        points.append((near[0] + shift[0], near[1] + shift[1]))
        points.append((far[0] + shift[0], far[1] + shift[1]))
    points.append(exit_)
    return tuple(points)


def planned_cells(zone: Zone, lot: LotSpec, cfg: AgentConfig) -> tuple[str, ...]:
    """Every coverage cell the planned sweep path passes through, sorted.

    This -- not the whole zone rectangle -- is the denominator of coverage. A robot drives
    the aisle; it never enters the stall rows or the landscaping, so counting those cells
    would make 100% coverage unreachable and acceptance criterion C4 unsatisfiable by
    construction. The path is sampled at a quarter of a cell so no cell it crosses is
    missed.
    """
    path = sweep_waypoints(zone, lot, cfg)
    step_m = cfg.cell_size_m / 4.0
    found: set[str] = set()
    for a, b in pairwise(path):
        length = math.dist(a, b)
        samples = max(1, math.ceil(length / step_m))
        for i in range(samples + 1):
            t = i / samples
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            cell = cell_at(zone, x, y, cfg.cell_size_m)
            if cell is not None:
                found.add(cell)
    return tuple(sorted(found))


# ---------------------------------------------------------------------- the agent --
class RobotAgent:
    """Autonomy for one robot, driving one :class:`~parkscan_core.world.WorldBackend`.

    Construct one per robot, call :meth:`assign` when the coordinator hands out a zone, and
    call :meth:`tick` once per simulation step. Everything else -- :meth:`abort`,
    :meth:`hold`, :meth:`resume`, :meth:`fail`, :meth:`mark_offline`, :meth:`recover` -- is
    the coordinator pushing an external event in, and each is a thin wrapper over
    :func:`parkscan_agent.statemachine.step`, so an event the robot cannot accept raises
    :class:`~parkscan_agent.statemachine.IllegalTransition` instead of being ignored.

    The agent holds **no** randomness and reads **no** clock: every time value it reports
    comes from the backend's ``SensorFrame.stamp_ns``. Two runs of the same backend with
    the same seed drive the agent through the same states in the same order.
    """

    def __init__(
        self,
        robot_id: RobotId,
        lot: LotSpec,
        backend: WorldBackend,
        config: AgentConfig | None = None,
    ) -> None:
        self._robot_id = robot_id
        self._lot = lot
        self._backend = backend
        self._cfg = config if config is not None else AgentConfig()
        self._zones: dict[str, Zone] = {z.id: z for z in lot.zones}

        self._state = RobotState.IDLE
        self._zone: Zone | None = None
        self._epoch = 0
        self._detail = "idle"

        self._route: tuple[Point2D, ...] = ()
        self._route_index = 0
        self._sweep: tuple[Point2D, ...] = ()
        self._sweep_index = 0

        self._covered: set[str] = set()
        self._planned: tuple[str, ...] = ()
        self._last_progress: ZoneProgress | None = None

        self._dock: DockController | None = None
        self._return_committed = False
        self._resume_event = Event.LEASE_GRANTED

    # ------------------------------------------------------------------ properties --
    @property
    def robot_id(self) -> RobotId:
        return self._robot_id

    @property
    def state(self) -> RobotState:
        """The robot's own view of itself. The coordinator keeps a separate judgement."""
        return self._state

    @property
    def zone_id(self) -> str | None:
        """The zone currently assigned, or ``None``."""
        return self._zone.id if self._zone is not None else None

    @property
    def epoch(self) -> int:
        """Assignment epoch. A progress report is only valid while this matches the zone's."""
        return self._epoch

    @property
    def config(self) -> AgentConfig:
        return self._cfg

    @property
    def progress(self) -> ZoneProgress | None:
        """Live progress on the current assignment, or ``None`` when there is none."""
        if self._zone is None:
            return None
        return ZoneProgress(
            zone_id=self._zone.id,
            epoch=self._epoch,
            cells_done=tuple(sorted(self._covered)),
            cells_total=self._planned,
        )

    @property
    def last_progress(self) -> ZoneProgress | None:
        """The most recent assignment's progress, kept after it completed or was abandoned.

        This is what the coordinator requeues from: ``cells_remaining`` on an abandoned
        zone is the work still outstanding, so a robot that breaks off for charge costs the
        fleet the remainder of one zone rather than the whole of it.
        """
        return self._last_progress

    # ------------------------------------------------- coordinator-driven events --
    def assign(self, zone_id: str, *, epoch: int | None = None) -> None:
        """Accept an order for ``zone_id``.

        ``epoch`` is the coordinator's per-zone monotonic assignment epoch, echoed back in
        every telemetry and progress message so a stale report from a revived robot can be
        dropped. When omitted the agent increments its own counter, which is only useful in
        tests -- in a run the coordinator owns the number.

        Raises:
            ValueError: ``zone_id`` is not a zone of this lot.
            ~parkscan_agent.statemachine.IllegalTransition: the robot is not ``IDLE``.
        """
        zone = self._zones.get(zone_id)
        if zone is None:
            known = ", ".join(sorted(self._zones)[:4])
            msg = f"{zone_id!r} is not a zone of lot {self._lot.name!r} (have {known}, ...)"
            raise ValueError(msg)
        self._apply(Event.ASSIGNED)
        self._zone = zone
        self._epoch = self._epoch + 1 if epoch is None else epoch
        self._covered = set()
        self._planned = planned_cells(zone, self._lot, self._cfg)
        self._route, self._route_index = (), 0
        self._sweep, self._sweep_index = (), 0
        self._detail = f"assigned {zone_id} at epoch {self._epoch}"

    def abort(self) -> None:
        """Coordinator cancelled the current order. Progress is stashed, not discarded."""
        self._apply(Event.ABORT)
        self._return_committed = False
        self._dock = None
        self._release_assignment("order aborted by the coordinator")

    def hold(self, reason: str = "waiting on a lease") -> None:
        """A shared resource is held by someone else: stop and queue.

        The agent knows nothing about leases; it only reacts. Which resume event
        :meth:`resume` will use is decided here, from the state the robot was actually in,
        so a robot queueing for a bay resumes its return leg rather than going back to work.
        """
        self._resume_event = (
            Event.BAY_LEASE_GRANTED if self._state is RobotState.RETURNING else Event.LEASE_GRANTED
        )
        self._apply(Event.LEASE_DENIED)
        self._detail = reason

    def resume(self) -> None:
        """The queued lease was granted: carry on with whatever was interrupted."""
        self._apply(self._resume_event)
        self._detail = "lease granted, resuming"

    def fail(self, reason: str = "fault") -> None:
        """Local unrecoverable fault. Only :meth:`recover` gets the robot out of this."""
        self._apply(Event.FAULT)
        self._detail = f"failed: {reason}"

    def mark_offline(self, reason: str = "comms lost") -> None:
        """The broker connection died, or the coordinator declared this robot gone."""
        self._apply(Event.COMMS_LOST)
        self._detail = f"offline: {reason}"

    def recover(self) -> None:
        """Lazarus re-registration: return to ``IDLE`` with the old order abandoned.

        Never a silent resume. The robot drops its route, its sweep and its assignment, and
        waits for a fresh :meth:`assign` carrying a new epoch -- which is what stops a
        revived robot and its replacement both working the same zone.
        """
        self._apply(Event.RECOVERED)
        self._return_committed = False
        self._dock = None
        self._release_assignment("recovered; awaiting a fresh assignment")

    # ------------------------------------------------------------------- the loop --
    def tick(self) -> TelemetrySample:
        """Sense, decide, actuate. Returns the telemetry for this tick.

        Order inside a tick, and it matters:

        1. **Record coverage** for the state the robot was in when the tick began, so a
           cell entered on the same tick that the robot breaks off for charge is still
           attributed to the zone it belongs to.
        2. **Evaluate battery feasibility**, every tick, in every working state.
        3. **Drive**: run the state's controller, applying local avoidance to the result.
        4. **Actuate** exactly once.
        """
        frame = self._backend.sense(self._robot_id)
        zone_at_tick = self.zone_id
        epoch_at_tick = self._epoch

        covered = self._record_coverage(frame)
        self._check_battery(frame)
        command = self._drive(frame)
        self._backend.actuate(self._robot_id, command)

        return TelemetrySample(
            robot_id=self._robot_id,
            stamp_ns=frame.stamp_ns,
            pose=frame.pose,
            twist=frame.twist,
            command=command,
            battery_soc=frame.battery_soc,
            state=self._state,
            zone_id=zone_at_tick,
            epoch=epoch_at_tick,
            covered_cells=covered,
            detail=self._detail,
        )

    # ------------------------------------------------------------------ internals --
    def _apply(self, event: Event) -> None:
        self._state = step(self._state, event)

    def _record_coverage(self, frame: SensorFrame) -> tuple[str, ...]:
        """Mark the cell under the robot, if it is sweeping a zone it is actually inside."""
        if self._state is not RobotState.INSPECTING or self._zone is None:
            return ()
        seen = cells_covered_from(
            self._zone,
            frame.pose.x_m,
            frame.pose.y_m,
            self._cfg.cell_size_m,
            self._cfg.coverage_radius_m,
        )
        fresh = tuple(cell for cell in seen if cell not in self._covered)
        self._covered.update(fresh)
        return fresh

    def _check_battery(self, frame: SensorFrame) -> None:
        # ``_return_committed`` matters more than it looks. A robot that has been told to
        # queue for an occupied bay sits in WAITING_ON_LEASE with a flat battery and a
        # failing feasibility check; without this guard it would re-fire BATTERY_CRITICAL
        # on every tick, jump straight back to RETURNING and walk out of the queue it was
        # just put in. The commitment is cleared when it finishes charging, or when the
        # coordinator explicitly cancels the attempt.
        if self._state not in _BATTERY_WATCHED or self._return_committed:
            return
        decision = should_return(frame.battery_soc, frame.pose, self._lot, self._cfg.battery)
        if decision.must_return:
            self._begin_return(decision, frame.pose)

    def _begin_return(self, decision: ReturnDecision, pose: Pose2D) -> None:
        """Break off and drive to a bay. Called from the continuous feasibility check."""
        bay = next(b for b in self._lot.charging_bays if b.id == decision.bay.bay_id)
        controller = DockController(bay, self._lot, self._cfg.nav, self._cfg.dock)
        controller.start(pose)
        self._dock = controller
        self._return_committed = True
        self._apply(Event.BATTERY_CRITICAL)
        self._release_assignment(
            f"battery {decision.soc:.3f} below the {decision.required_soc:.3f} needed for "
            f"{decision.bay.distance_m:.1f} m of lane back to {decision.bay.bay_id}"
        )

    def retarget_bay(self, bay_id: str, pose: Pose2D) -> None:
        """Redirect an in-progress or queued return to a DIFFERENT bay.

        Called by the coordinator when it grants a bay other than the one this robot's
        own :meth:`_begin_return` picked on its own initiative -- normally because the
        nearest bay was occupied and a farther one was free. Queueing for one specific
        bay has no upper bound on wait time; driving a little farther does, which is why
        the coordinator tries every bay before it ever calls :meth:`hold` for this
        resource (see :mod:`parkscan_coord.coordinator`).

        This does not change ``state`` and is not routed through the state machine: it
        redirects an ALREADY-DECIDED return (``RETURNING`` or ``WAITING_ON_LEASE``), it
        never starts or ends one -- those stay exactly where they always were,
        :meth:`_begin_return` and the state machine respectively.

        ``pose`` is the caller's own telemetry pose for this tick, passed in rather than
        re-sensed, matching how :meth:`tick` hands :meth:`_begin_return` the frame it
        already has instead of asking the backend again.

        Raises:
            ValueError: called while not ``RETURNING`` or ``WAITING_ON_LEASE`` -- a
                misuse this method reports rather than silently ignores, symmetric with
                how :meth:`hold` and :meth:`resume` raise through the state machine for
                the same kind of mistake. The caller is expected to check first, as it
                already must before calling those.
        """
        if self._state not in (RobotState.RETURNING, RobotState.WAITING_ON_LEASE):
            msg = (
                f"cannot retarget a bay from {self._state.value!r}; only legal from "
                "RETURNING or WAITING_ON_LEASE"
            )
            raise ValueError(msg)
        bay = next(b for b in self._lot.charging_bays if b.id == bay_id)
        controller = DockController(bay, self._lot, self._cfg.nav, self._cfg.dock)
        controller.start(pose)
        self._dock = controller
        self._detail = f"redirected to {bay_id}"

    def _release_assignment(self, reason: str) -> None:
        """Stash progress and drop the order. The coordinator requeues the remainder."""
        current = self.progress
        if current is not None:
            self._last_progress = current
        self._zone = None
        self._route, self._route_index = (), 0
        self._sweep, self._sweep_index = (), 0
        self._detail = reason

    def _plan_transit(self, pose: Pose2D) -> None:
        if self._zone is None:  # pragma: no cover - only reachable from ASSIGNED
            msg = "cannot plan a transit without an assigned zone"
            raise RuntimeError(msg)
        origin = nearest_node(self._lot, pose.x_m, pose.y_m)
        path = route(self._lot, origin, self._zone.entry_waypoint)
        self._route = tuple(route_points(self._lot, path))
        self._route_index = 0
        self._detail = f"routing to {self._zone.entry_waypoint} via {len(path)} lane nodes"

    def _drive(self, frame: SensorFrame) -> TwistCmd:
        nav = self._cfg.nav
        pose = frame.pose

        if self._state is RobotState.ASSIGNED:
            self._plan_transit(pose)
            self._apply(Event.ROUTE_STARTED)

        if self._state is RobotState.TRANSIT:
            if self._sweep:
                # Interrupted mid-sweep and resumed: already inside the zone, so the
                # arrival event fires immediately rather than re-driving the transit route.
                self._apply(Event.ZONE_REACHED)
            elif self._zone is None:
                # TRANSIT with no order. Reachable only if the coordinator resumed a lease
                # it had also aborted; stop rather than invent a destination.
                self._detail = "transit without an assignment; holding position"
                return STOP
            else:
                if not self._route:
                    # Held before the route was planned (LEASE_DENIED arrived in ASSIGNED).
                    self._plan_transit(pose)
                leg = follow_route(pose, self._route, self._route_index, nav)
                self._route_index = leg.index
                if not leg.finished:
                    self._detail = f"transit, waypoint {leg.index + 1}/{len(self._route)}"
                    return avoid(leg.cmd, frame.ranges, nav)
                self._apply(Event.ZONE_REACHED)
                self._begin_sweep()

        if self._state is RobotState.INSPECTING:
            leg = follow_route(pose, self._sweep, self._sweep_index, nav)
            self._sweep_index = leg.index
            if not leg.finished:
                self._detail = (
                    f"sweeping {self.zone_id}, leg {leg.index + 1}/{len(self._sweep)}, "
                    f"{len(self._covered)}/{len(self._planned)} cells"
                )
                return avoid(leg.cmd, frame.ranges, nav)
            self._apply(Event.SWEEP_COMPLETE)
            self._release_assignment(f"sweep of {self._zone.id if self._zone else '?'} complete")
            return STOP

        if self._state is RobotState.RETURNING:
            return self._drive_dock(frame)

        if self._state is RobotState.CHARGING:
            if frame.battery_soc >= self._cfg.battery.charged_soc:
                self._apply(Event.CHARGED)
                self._dock = None
                self._return_committed = False
                self._detail = f"charged to {frame.battery_soc:.3f}, idle"
            else:
                self._detail = f"charging, {frame.battery_soc:.3f}"
            return STOP

        return STOP

    def _drive_dock(self, frame: SensorFrame) -> TwistCmd:
        if self._dock is None:  # pragma: no cover - RETURNING always has a controller
            msg = "RETURNING without a docking controller"
            raise RuntimeError(msg)
        status = self._dock.update(frame.pose)
        self._detail = f"{self._dock.bay_id}: {status.detail}"
        if status.docked:
            self._apply(Event.BAY_REACHED)
            return STOP
        if status.failed:
            self._apply(Event.DOCK_FAILED)
            return STOP
        if status.phase is DockPhase.ROUTING:
            return avoid(status.cmd, frame.ranges, self._cfg.nav)
        # Final approach and alignment run without the reactive layer. The bay structure
        # is by definition inside the avoidance envelope, so leaving avoidance switched on
        # would scale the approach speed to zero a few centimetres short of the contact
        # and the robot would never dock. This is a bounded, deliberate exception: it
        # applies only within DockConfig.approach_radius_m of the bay.
        return status.cmd

    def _begin_sweep(self) -> None:
        if self._zone is None:  # pragma: no cover - only reachable from TRANSIT
            msg = "cannot begin a sweep without an assigned zone"
            raise RuntimeError(msg)
        self._sweep = sweep_waypoints(self._zone, self._lot, self._cfg)
        self._sweep_index = 0
        self._detail = f"sweeping {self._zone.id}, {len(self._planned)} cells planned"


def waypoint_sequence(
    lot: LotSpec, zone_id: str, cfg: AgentConfig | None = None
) -> tuple[Point2D, ...]:
    """The full inspection waypoint sequence for ``zone_id``: entry, then the sweep.

    Exposed for the acceptance evidence (A1, ``pytest -k waypoint_sequence``) and for the
    dashboard's planned-path overlay, so both read the same geometry the robot drives
    rather than a second copy that can drift out of step with it.
    """
    zones = {z.id: z for z in lot.zones}
    zone = zones.get(zone_id)
    if zone is None:
        msg = f"{zone_id!r} is not a zone of lot {lot.name!r}"
        raise ValueError(msg)
    config = cfg if cfg is not None else AgentConfig()
    return (node_xy(lot, zone.entry_waypoint), *sweep_waypoints(zone, lot, config))

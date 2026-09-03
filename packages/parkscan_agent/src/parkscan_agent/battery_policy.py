"""When to abandon the sweep and drive to a charging bay.

Why this is a feasibility check and not a percentage
----------------------------------------------------
A fixed "return at 20%" threshold is wrong in both directions on this lot, and both
failures are real:

* **Wasteful near a bay.** A robot sweeping zone 4, whose entry is 12 m of lane from
  ``bay_1``, gives up a third of its useful range for a return leg that costs almost
  nothing.
* **Fatal far from one.** A robot in zone 12 is ~200 m of lane from the nearest bay. At
  20% it may already be unable to get back, and acceptance criterion C2 says
  ``depletion_events == 0`` -- a robot that reaches 0% before a bay is a build failure,
  not a bad chart.

So the trigger is a predicate over the *actual* return cost::

    soc < k * d_lane(pose -> nearest bay) * (1 + margin) + soc_dock + soc_reserve

where ``d_lane`` is lane-graph distance (:func:`parkscan_agent.lanegraph.lane_distance_m`),
never straight-line: on this lot the two differ by more than a factor of two for a robot
in the east columns, and using the Euclidean figure would produce exactly the confident,
plausible under-estimate that strands a robot.

Why it is evaluated every tick
------------------------------
Checking only at zone boundaries means committing to a 124 m sweep on whatever charge is
left when the zone starts. The whole point of a feasibility test is that it tracks the
return cost as the robot moves, so it fires *during* the sweep, at the first tick where
the remaining charge stops covering the drive home. The partially-swept zone is not lost:
the robot reports covered cells continuously, so the coordinator requeues the remainder.

Everything here is a pure function of ``(soc, pose, lot, params)``. No simulator, no
clock, no I/O -- so the thresholds can be unit-tested directly, and the coordinator can
reuse :func:`sortie_feasible` as its allocation feasibility predicate without instantiating
a robot.

Honest limits of the model
--------------------------
``soc_per_metre`` is a **linear** distance model. Real batteries are not linear in
distance, temperature or load, and this one knows nothing about turning cost, idle draw,
or the charge curve. It is calibrated against the simulator's own battery model and is
used to make a decision with a safety margin, not to predict remaining range. Nothing in
this project should be read as a claim about hardware endurance.
"""

from __future__ import annotations

from dataclasses import dataclass

from parkscan_agent.lanegraph import lane_distance_m, nearest_node
from parkscan_core.lot import LotSpec
from parkscan_core.types import Pose2D

__all__ = [
    "BatteryParams",
    "BayChoice",
    "ReturnDecision",
    "nearest_bay",
    "required_soc",
    "should_return",
    "sortie_feasible",
]


@dataclass(frozen=True, slots=True)
class BatteryParams:
    """Battery model and safety policy. Frozen, so a run's policy is a loggable value."""

    soc_per_metre: float = 1.0 / 2500.0
    """``k``: state of charge consumed per metre driven.

    The default gives a full battery ~2500 m of range, which is about six 124 m zone
    sweeps plus the transits between them on the shipped 150 x 100 m lot -- enough that a
    run has to make real return-to-charge decisions rather than never needing one, and
    little enough that ``make bench`` exercises bay contention with 2 bays and 3 robots.
    """

    margin: float = 0.25
    """Fractional padding on the estimated return distance.

    Covers what the linear model omits: turning, the docking manoeuvre's creep, avoidance
    detours, and the fact that the robot is somewhere between lane nodes rather than on
    one. 0.25 is a policy choice, not a measurement.
    """

    soc_reserve: float = 0.05
    """Absolute floor kept in the bank after arriving at the bay.

    A robot that arrives at exactly 0% has already failed: it cannot re-position if it
    docks badly, and it has no margin for the queue if the bay is occupied.
    """

    soc_dock: float = 0.01
    """Charge budgeted for the final approach and alignment inside the bay."""

    charged_soc: float = 0.95
    """State of charge at which a charging robot is released back to work."""

    def __post_init__(self) -> None:
        if self.soc_per_metre <= 0.0:
            msg = f"soc_per_metre must be positive, got {self.soc_per_metre}"
            raise ValueError(msg)
        if self.margin < 0.0:
            msg = f"margin must be non-negative, got {self.margin}"
            raise ValueError(msg)
        for name, value in (
            ("soc_reserve", self.soc_reserve),
            ("soc_dock", self.soc_dock),
            ("charged_soc", self.charged_soc),
        ):
            if not 0.0 <= value <= 1.0:
                msg = f"{name} must be in [0, 1], got {value}"
                raise ValueError(msg)


@dataclass(frozen=True, slots=True)
class BayChoice:
    """The cheapest charging bay to run for, and what it costs to get there."""

    bay_id: str
    node: str
    """The bay's lane node -- a spur off the perimeter lane, so a docked robot does not
    block the through lane."""
    distance_m: float
    """Lane-graph distance from the robot's snapped position to :attr:`node`."""


@dataclass(frozen=True, slots=True)
class ReturnDecision:
    """The full result of one feasibility evaluation, not just a boolean.

    Carrying the numbers alongside the verdict is what lets the dashboard explain *why* a
    robot broke off a sweep, and lets a test assert on the threshold rather than on the
    single bit that came out of it.
    """

    must_return: bool
    bay: BayChoice
    soc: float
    required_soc: float

    @property
    def slack(self) -> float:
        """Charge above the requirement. Negative exactly when :attr:`must_return`."""
        return self.soc - self.required_soc


def nearest_bay(spec: LotSpec, x_m: float, y_m: float) -> BayChoice:
    """The charging bay with the smallest **lane-graph** distance from ``(x_m, y_m)``.

    The pose is snapped onto the graph with :func:`~parkscan_agent.lanegraph.nearest_node`
    first; from there every candidate bay is costed by driving distance. Ties are broken
    by bay id so the choice is reproducible.

    Raises:
        ValueError: the lot declares no charging bays.
        ~parkscan_agent.lanegraph.NoRouteError: no bay is reachable from the robot.
    """
    if not spec.charging_bays:
        msg = f"lot {spec.name!r} declares no charging bays"
        raise ValueError(msg)
    origin = nearest_node(spec, x_m, y_m)
    candidates = [
        BayChoice(bay_id=bay.id, node=bay.node, distance_m=lane_distance_m(spec, origin, bay.node))
        for bay in sorted(spec.charging_bays, key=lambda b: b.id)
    ]
    return min(candidates, key=lambda c: (c.distance_m, c.bay_id))


def required_soc(distance_m: float, params: BatteryParams) -> float:
    """Charge needed to drive ``distance_m`` home, dock, and still hold the reserve."""
    if distance_m < 0.0:
        msg = f"distance_m must be non-negative, got {distance_m}"
        raise ValueError(msg)
    travel = params.soc_per_metre * distance_m * (1.0 + params.margin)
    return travel + params.soc_dock + params.soc_reserve


def should_return(soc: float, pose: Pose2D, lot: LotSpec, params: BatteryParams) -> ReturnDecision:
    """Evaluate the return-to-charge feasibility check for one robot, right now.

    Call this on **every** tick while the robot is working. It is a few microseconds of
    Dijkstra on a 44-node graph, and the alternative -- checking at zone boundaries -- is
    what strands a robot mid-sweep.
    """
    if not 0.0 <= soc <= 1.0:
        msg = f"soc must be in [0, 1], got {soc}"
        raise ValueError(msg)
    bay = nearest_bay(lot, pose.x_m, pose.y_m)
    needed = required_soc(bay.distance_m, params)
    return ReturnDecision(must_return=soc < needed, bay=bay, soc=soc, required_soc=needed)


def sortie_feasible(
    soc: float,
    lot: LotSpec,
    params: BatteryParams,
    *,
    work_distance_m: float,
    finish_node: str,
) -> bool:
    """Whether ``soc`` covers ``work_distance_m`` of work **and** the drive home after it.

    This is the allocator's feasibility predicate, kept here so there is exactly one
    battery model in the system: the coordinator costs a candidate assignment as
    ``d_lane(robot -> zone entry) + sweep_len(zone)`` and asks whether the robot could
    still reach a bay from the zone's exit. Assigning work a robot cannot finish and
    survive is how ``depletion_events`` stops being zero.
    """
    if work_distance_m < 0.0:
        msg = f"work_distance_m must be non-negative, got {work_distance_m}"
        raise ValueError(msg)
    nodes = lot.lane_graph.node_map()
    if finish_node not in nodes:
        msg = f"{finish_node!r} is not a lane node of lot {lot.name!r}"
        raise ValueError(msg)
    finish = nodes[finish_node]
    home = nearest_bay(lot, finish.x_m, finish.y_m)
    needed = required_soc(work_distance_m + home.distance_m, params)
    return soc >= needed

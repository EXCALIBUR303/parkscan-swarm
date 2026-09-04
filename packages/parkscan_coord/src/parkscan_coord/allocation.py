"""Three interchangeable allocation strategies over one shared cost function.

Assigning 12 zones to 3 robots is ST-SR-TA (Gerkey & Mataric), scheduling class
``R || sum w_j C_j``, strongly NP-hard; lane and bay contention makes utilities
interdependent, so the iTax label is XD [ST-SR-TA] (Korsah et al. 2013). The coordinator
decouples that cross-schedule dependency into the reservation layer
(:mod:`parkscan_coord.leases`) and approximates the rest as **iterated ST-SR-IA**:
re-solve an instantaneous assignment whenever an event frees a robot. ADR-0005 records
the decision and the rejection of CBBA.

This module is a **pure function**. :func:`allocate` reads an :class:`AllocatorState`, a
:class:`~parkscan_core.lot.LotSpec` and a
:class:`~parkscan_agent.battery_policy.BatteryParams` and returns assignments. No clock,
no I/O, no RNG, no dependence on dictionary order -- so the whole allocator is testable
without a simulator, and ``make bench``'s 30 seeds are reproducible.

The three strategies
--------------------
``GREEDY`` (default)
    Repeated argmin over feasible ``(idle robot, unassigned zone)`` pairs. Ties are
    broken by ``(cost, robot_id, zone_id)``, which is a total order, so the result is
    byte-reproducible.

``HUNGARIAN``
    ``scipy.optimize.linear_sum_assignment`` on the rectangular matrix -- the **optimal
    per-round reference**. It exists so greedy's gap is measured rather than asserted
    (acceptance C9).

``SSI``
    Sequential single-item auction with marginal-cost (``BidSumPath``) bids, Lagoudakis
    et al., RSS 2005. Each round every robot bids ``RPC(r, bundle + {z}) - RPC(r,
    bundle)`` with cheapest insertion into its current bundle; the single globally lowest
    bid wins that zone; repeat until no feasible bid remains.

What "greedy is 2-competitive" actually says
--------------------------------------------
That result is for **maximising** utility -- greedy achieves at least half the optimum
when you are adding value. Under **cost minimisation**, which is what this allocator
does, greedy's approximation ratio is **unbounded**: one cheap cell can lure a robot away
from the only zone it could serve affordably, and the price of the resulting forced pair
is limited only by the size of the matrix entry. ``test_allocation.py`` measures both a
pinned realistic gap and an adversarial matrix where greedy is ~5e5 times worse, rather
than quoting the maximisation theorem at a minimisation problem.

What SSI's guarantee applies to
-------------------------------
SSI's "within 2x optimal on MiniSum" theorem is about the **bundles** the auction builds.
The coordinator dispatches only the *head* of each bundle -- one zone per robot, because
a robot executes one zone at a time and the allocator is re-run on the next event -- so
the guarantee describes the plan, not the sequence of zones a robot ends up driving.
:func:`plan_bundles` exposes the full tours for inspection; :func:`allocate` returns the
heads.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum
from typing import Final

import numpy as np
from parkscan_agent.battery_policy import BatteryParams

# scipy 1.18 ships no py.typed marker and `scipy-stubs` is not a dependency of this
# workspace (adding one is out of scope), so this import is the single untyped edge in
# the package. `_hungarian` converts everything it returns to built-in ints immediately.
from scipy.optimize import linear_sum_assignment  # type: ignore[import-untyped]

from parkscan_coord.cost import INFEASIBLE_COST, CostModel, RobotSnapshot, build_cost_model
from parkscan_core.lot import LotSpec, Zone
from parkscan_core.types import RobotId, ZoneState

__all__ = [
    "AllocationError",
    "AllocatorState",
    "Assignment",
    "Bundle",
    "Strategy",
    "allocate",
    "allocate_model",
    "plan_bundles",
    "total_cost_m",
]


class AllocationError(Exception):
    """An allocation was asked for, or produced, something structurally invalid."""


class Strategy(StrEnum):
    """Which search to run over the shared cost model."""

    GREEDY = "greedy"
    HUNGARIAN = "hungarian"
    SSI = "ssi"


@dataclass(frozen=True, slots=True, order=True)
class Assignment:
    """One robot is to work one zone. The unit of the allocator's output."""

    robot_id: RobotId
    zone_id: str


@dataclass(frozen=True, slots=True)
class Bundle:
    """One robot's SSI tour: the zones it won, in the order the auction inserted them."""

    robot_id: RobotId
    zone_ids: tuple[str, ...]
    path_cost_m: float
    """``RPC(robot, zone_ids)`` -- approach + sweeps + exit-to-entry links."""


@dataclass(frozen=True, slots=True)
class AllocatorState:
    """The coordinator's world, reduced to exactly what allocation depends on.

    ``robots`` carries position, charge and availability;
    :attr:`~parkscan_coord.cost.RobotSnapshot.available` is the coordinator's judgement
    (from :class:`~parkscan_core.types.RobotState` and
    :class:`~parkscan_core.types.CoordView`), kept out here so the allocator holds no
    policy of its own.

    ``zones`` maps zone id to :class:`~parkscan_core.types.ZoneState`. Only
    ``UNASSIGNED`` zones are allocatable: ``ASSIGNED`` and ``IN_PROGRESS`` belong to a
    robot already, ``COMPLETE`` is done, and ``UNRECOVERABLE`` is the hard cap that stops
    a zone being retried forever. Requeueing a partially swept zone is the coordinator's
    job -- it sets the state back to ``UNASSIGNED`` -- and the zone is then priced at its
    full sweep length again (see :mod:`parkscan_coord.cost`).
    """

    robots: tuple[RobotSnapshot, ...]
    zones: Mapping[str, ZoneState]

    def available_robots(self) -> tuple[RobotSnapshot, ...]:
        """Available robots, sorted by id. Sorting is what makes ties reproducible."""
        return tuple(sorted((r for r in self.robots if r.available), key=lambda r: r.robot_id))

    def allocatable_zone_ids(self) -> tuple[str, ...]:
        """Ids of ``UNASSIGNED`` zones, sorted -- never in dictionary order."""
        return tuple(sorted(z for z, state in self.zones.items() if state is ZoneState.UNASSIGNED))


# --------------------------------------------------------------------- strategies --
def _greedy(model: CostModel) -> list[Assignment]:
    """Repeated argmin over feasible pairs, ties broken by ``(cost, robot, zone)``.

    Sorting every feasible pair once and walking the list is exactly repeated argmin:
    after a robot or a zone is consumed, the next still-available entry in cost order is
    the argmin of what remains. It is also a total order, so two runs cannot disagree.

    Greedy is the default because it is explainable and instant. It is **not** near
    optimal in general: under cost minimisation its ratio is unbounded (see the module
    docstring), which is precisely why :class:`Strategy.HUNGARIAN` is shipped alongside
    it as the reference.
    """
    costs = model.pair_costs()
    candidates = sorted(
        (float(costs[i, j]), model.robot_ids[i], model.zone_ids[j])
        for i in range(model.n_robots)
        for j in range(model.n_zones)
        if bool(model.feasible[i, j])
    )
    taken_robots: set[RobotId] = set()
    taken_zones: set[str] = set()
    chosen: list[Assignment] = []
    for _cost, robot_id, zone_id in candidates:
        if robot_id in taken_robots or zone_id in taken_zones:
            continue
        taken_robots.add(robot_id)
        taken_zones.add(zone_id)
        chosen.append(Assignment(robot_id=robot_id, zone_id=zone_id))
        if len(chosen) == min(model.n_robots, model.n_zones):
            break
    return chosen


def _hungarian(model: CostModel) -> list[Assignment]:
    """Optimal per-round assignment via ``scipy.optimize.linear_sum_assignment``.

    ``linear_sum_assignment`` handles the rectangular 3x12 matrix natively and returns
    ``min(n_robots, n_zones)`` pairs, one per row when robots are the scarce side.

    Infeasible pairs are priced at the large **finite**
    :data:`~parkscan_coord.cost.INFEASIBLE_COST` sentinel rather than ``np.inf``: an
    all-``inf`` row makes scipy raise, and the sentinel is far enough above any real cost
    that a sentinel cell is never chosen while a feasible completion exists.

    It is chosen anyway when a robot has *no* feasible zone at all -- the solver still has
    to put that row somewhere -- so those pairs are dropped here, and
    :func:`_validate_assignments` then re-checks every surviving pair against the model's
    own feasibility mask. The check is deliberately downstream of the drop: if a
    sentinel-priced pair ever reached the output it would be a silent battery violation,
    not a rounding detail.
    """
    masked = model.masked_costs(INFEASIBLE_COST)
    rows, cols = linear_sum_assignment(masked)
    chosen: list[Assignment] = []
    for raw_row, raw_col in zip(rows, cols, strict=True):
        i, j = int(raw_row), int(raw_col)
        if not bool(model.feasible[i, j]):
            continue
        chosen.append(Assignment(robot_id=model.robot_ids[i], zone_id=model.zone_ids[j]))
    return chosen


def _ssi_bundles(model: CostModel) -> list[Bundle]:
    """Run the sequential single-item auction to exhaustion and return the tours.

    One item is auctioned per round. Every robot bids its **marginal** path cost for
    every still-unsold zone, using cheapest insertion into its current bundle:

    .. code-block:: text

        bid(r, z) = min over insertion positions p of
                    RPC(r, bundle_r with z at p) - RPC(r, bundle_r)

    The globally lowest bid wins that one zone, and the auction repeats. That is
    ``BidSumPath`` from Lagoudakis et al. (RSS 2005); bidding the *pair* cost instead
    would make this an expensive re-implementation of greedy.

    A robot only bids on zones it is battery-feasible for individually. A multi-zone
    bundle is a plan, not a promise that the whole tour runs on one charge -- only its
    head is dispatched, and feasibility is re-evaluated on the next allocation round.

    Ties are broken by ``(bid, robot_id, zone_id, position)``, a total order.
    """
    bundles: dict[RobotId, tuple[str, ...]] = {robot_id: () for robot_id in model.robot_ids}
    remaining: list[str] = list(model.zone_ids)

    while remaining:
        best: tuple[float, RobotId, str, int] | None = None
        for robot_id in model.robot_ids:
            tour = bundles[robot_id]
            base = model.tour_cost(robot_id, tour)
            for zone_id in remaining:
                if not model.is_feasible(robot_id, zone_id):
                    continue
                for position in range(len(tour) + 1):
                    candidate = (*tour[:position], zone_id, *tour[position:])
                    bid = model.tour_cost(robot_id, candidate) - base
                    key = (bid, robot_id, zone_id, position)
                    if best is None or key < best:
                        best = key
        if best is None:
            break  # nothing left that anyone can afford
        _bid, winner, won_zone, at = best
        tour = bundles[winner]
        bundles[winner] = (*tour[:at], won_zone, *tour[at:])
        remaining.remove(won_zone)

    return [
        Bundle(
            robot_id=robot_id,
            zone_ids=bundles[robot_id],
            path_cost_m=model.tour_cost(robot_id, bundles[robot_id]),
        )
        for robot_id in model.robot_ids
    ]


def _ssi(model: CostModel) -> list[Assignment]:
    """The dispatchable head of each SSI bundle: one zone per robot, this round."""
    return [
        Assignment(robot_id=bundle.robot_id, zone_id=bundle.zone_ids[0])
        for bundle in _ssi_bundles(model)
        if bundle.zone_ids
    ]


_SOLVERS: Final[Mapping[Strategy, Callable[[CostModel], list[Assignment]]]] = {
    Strategy.GREEDY: _greedy,
    Strategy.HUNGARIAN: _hungarian,
    Strategy.SSI: _ssi,
}


# ------------------------------------------------------------------------ entry --
def _validate_assignments(model: CostModel, chosen: Sequence[Assignment]) -> None:
    """Reject a round that is not a valid matching, or that ignores the battery.

    Every strategy passes through here, so "no zone twice, no robot twice, nothing a
    robot cannot survive" is a property of the module rather than of three separate
    implementations that each remembered to check.
    """
    robots = [a.robot_id for a in chosen]
    zones = [a.zone_id for a in chosen]
    if len(set(robots)) != len(robots):
        msg = f"a robot was assigned twice in one round: {robots}"
        raise AllocationError(msg)
    if len(set(zones)) != len(zones):
        msg = f"a zone was assigned twice in one round: {zones}"
        raise AllocationError(msg)
    for assignment in chosen:
        if not model.is_feasible(assignment.robot_id, assignment.zone_id):
            msg = (
                f"{assignment.robot_id!r} was assigned {assignment.zone_id!r}, which it "
                f"has not the charge to sweep and return from "
                f"(cost {model.pair_cost(assignment.robot_id, assignment.zone_id):.3f} m, "
                f"sentinel {INFEASIBLE_COST})"
            )
            raise AllocationError(msg)


def allocate_model(model: CostModel, *, strategy: Strategy = Strategy.GREEDY) -> list[Assignment]:
    """Run ``strategy`` over an already-built :class:`~parkscan_coord.cost.CostModel`.

    The matrix-level entry point: the property test drives 1000 random cost matrices
    through here without a lot, a robot or a simulator anywhere in sight.

    Returns assignments sorted by robot id, at most ``min(n_robots, n_zones)`` of them,
    and fewer when battery feasibility rules pairs out.
    """
    chosen = _SOLVERS[Strategy(strategy)](model) if model.n_robots and model.n_zones else []
    _validate_assignments(model, chosen)
    return sorted(chosen)


def allocate(
    state: AllocatorState,
    lot: LotSpec,
    params: BatteryParams,
    *,
    strategy: Strategy = Strategy.GREEDY,
) -> list[Assignment]:
    """Assign unassigned zones to available robots. Pure function; no I/O, no clock.

    Raises:
        AllocationError: ``state`` names a zone that is not in ``lot``.
        ValueError: ``strategy`` is not a :class:`Strategy`.
    """
    robots = state.available_robots()
    zone_ids = state.allocatable_zone_ids()
    if not robots or not zone_ids:
        return []

    by_id: dict[str, Zone] = {zone.id: zone for zone in lot.zones}
    unknown = [zone_id for zone_id in zone_ids if zone_id not in by_id]
    if unknown:
        msg = f"zone id(s) {unknown} are not in lot {lot.name!r}"
        raise AllocationError(msg)

    model = build_cost_model(lot, robots, [by_id[zone_id] for zone_id in zone_ids], params)
    return allocate_model(model, strategy=strategy)


def plan_bundles(model: CostModel) -> list[Bundle]:
    """The full SSI tours behind :func:`allocate`'s heads, for inspection and tests."""
    return _ssi_bundles(model)


def total_cost_m(model: CostModel, assignments: Sequence[Assignment]) -> float:
    """Sum of ``c(i, z)`` over ``assignments`` -- the quantity C9 compares."""
    return float(np.sum([model.pair_cost(a.robot_id, a.zone_id) for a in assignments]))

"""The one cost function every allocation strategy shares.

Why one function and not three
------------------------------
``greedy``, ``hungarian`` and ``ssi`` exist so that greedy's gap can be *measured*
against an optimal per-round reference instead of asserted (ADR-0005). That comparison
is only meaningful if all three price the same assignment identically, so the cost lives
here, once, and the strategies in :mod:`parkscan_coord.allocation` are pure search
procedures over the numbers this module produces.

The cost of putting robot ``i`` on zone ``z`` is

.. code-block:: text

    c(i, z) = d_lane(nearest_node(pose_i) -> entry(z)) + sweep_length_m(z)

``d_lane`` is :func:`parkscan_agent.lanegraph.lane_distance_m` -- Dijkstra over the lane
graph -- and **never** ``math.dist``. On this lot a robot in the east columns is more
than twice as far from a bay by road as by straight line, and a Euclidean cost would rank
robots by a distance none of them can drive. The single legitimate use of Euclidean
distance is snapping the free-space pose onto the graph (:func:`robot_node`), which is a
projection, not a cost.

Feasibility includes the return leg
-----------------------------------
A pair is *feasible* only if the robot's charge covers the approach, the sweep, **and**
the drive from the zone's exit waypoint to the nearest charging bay:

.. code-block:: text

    soc >= required_soc(approach + sweep + d_lane(exit(z) -> nearest bay), params)

Dropping the return leg is the mistake that makes ``depletion_events == 0`` (acceptance
C2) unachievable: the robot finishes the zone and then discovers it cannot get home. The
arithmetic is :func:`parkscan_agent.battery_policy.required_soc`, reused rather than
re-derived, so the coordinator and the robot cannot drift apart about what "enough
charge" means. ``test_allocation.py`` pins that equivalence against
:func:`~parkscan_agent.battery_policy.sortie_feasible` directly.

Purity and determinism
----------------------
Everything here is a pure function of ``(state, lot, params)``. No clock, no I/O, no
global RNG, no dependence on dict iteration order: :class:`CostModel` rows and columns
are built from explicitly sorted id tuples, and the arrays it holds are copied and made
read-only, so a model is a value that two strategies can share without either being able
to perturb the other's input.

Honest limit
------------
A zone that was half swept and requeued is still priced at its **full** authored
``sweep_length_m``. The allocator sees zone ids and states, not cell coverage, so the
remaining work is over-estimated for a requeued zone. That is deliberately conservative
-- it never promises a robot a sortie shorter than the one it will actually drive -- but
it is an approximation, not a measurement.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Final, Self

import numpy as np
import numpy.typing as npt
from parkscan_agent.battery_policy import BatteryParams, nearest_bay, required_soc
from parkscan_agent.lanegraph import lane_distance_m, nearest_node, node_xy

from parkscan_core.lot import LotSpec, Zone
from parkscan_core.types import Pose2D, RobotId

__all__ = [
    "INFEASIBLE_COST",
    "CostModel",
    "PairCost",
    "RobotSnapshot",
    "approach_distance_m",
    "assignment_cost",
    "battery_feasible",
    "build_cost_model",
    "evaluate_pair",
    "robot_node",
]

INFEASIBLE_COST: Final[float] = 1.0e9
"""Sentinel price of a battery-infeasible pair, in metres. Large, but **finite**.

``scipy.optimize.linear_sum_assignment`` raises ``ValueError: cost matrix is infeasible``
when a row is all ``inf``, and ``np.inf - np.inf`` is ``nan`` in the marginal-cost
arithmetic SSI does. A finite sentinel keeps both well-posed. It is nine orders of
magnitude above the largest real cost on a 150 x 100 m lot (a few hundred metres), so no
feasible completion of an assignment can ever be out-priced by taking a sentinel cell,
and a chosen cell at or above the sentinel is unambiguously a bug -- which is exactly how
:func:`parkscan_coord.allocation.allocate_model` treats it.
"""


@dataclass(frozen=True, slots=True)
class RobotSnapshot:
    """One robot as the allocator sees it: where it is, how charged, and is it free.

    ``available`` is supplied by the coordinator, which owns the policy of turning a
    :class:`~parkscan_core.types.RobotState` and a :class:`~parkscan_core.types.CoordView`
    into "may be given work". Keeping that decision outside the allocator is what lets
    ``allocate`` stay a pure function of numbers.
    """

    robot_id: RobotId
    pose: Pose2D
    battery_soc: float
    available: bool = True

    def __post_init__(self) -> None:
        if not self.robot_id:
            msg = "robot_id must be a non-empty string"
            raise ValueError(msg)
        if not 0.0 <= self.battery_soc <= 1.0:
            msg = f"battery_soc must be in [0, 1], got {self.battery_soc}"
            raise ValueError(msg)


@dataclass(frozen=True, slots=True)
class PairCost:
    """The full pricing of one ``(robot, zone)`` pair, not just the scalar.

    The components are kept because they are what a reviewer (and the dashboard) needs to
    see *why* a pair was rejected: a pair can be infeasible because the zone is far, or
    because its exit is far from a bay, and those are different problems.
    """

    robot_id: RobotId
    zone_id: str
    approach_m: float
    """``d_lane(robot -> entry(zone))``."""
    sweep_m: float
    """The zone's authored back-and-forth sweep length."""
    return_m: float
    """``d_lane(exit(zone) -> nearest bay)`` -- the leg feasibility must not forget."""
    return_bay_id: str
    required_soc: float
    """Charge needed for approach + sweep + return, including margin and reserve."""
    battery_soc: float

    @property
    def cost_m(self) -> float:
        """``c(i, z)`` -- what every strategy minimises."""
        return self.approach_m + self.sweep_m

    @property
    def feasible(self) -> bool:
        """Whether the robot can do this zone *and still reach a bay afterwards*."""
        return self.battery_soc >= self.required_soc

    @property
    def soc_slack(self) -> float:
        """Charge above the requirement; negative exactly when infeasible."""
        return self.battery_soc - self.required_soc


def robot_node(lot: LotSpec, robot: RobotSnapshot) -> str:
    """Snap a robot's free-space pose onto the lane graph.

    The only Euclidean measurement in the costing path, and a projection rather than a
    cost -- see :func:`parkscan_agent.lanegraph.nearest_node`.
    """
    return nearest_node(lot, robot.pose.x_m, robot.pose.y_m)


def approach_distance_m(lot: LotSpec, robot: RobotSnapshot, zone: Zone) -> float:
    """Lane-graph distance from the robot to the zone's entry waypoint, in metres."""
    return lane_distance_m(lot, robot_node(lot, robot), zone.entry_waypoint)


def evaluate_pair(
    lot: LotSpec, robot: RobotSnapshot, zone: Zone, params: BatteryParams
) -> PairCost:
    """Price and battery-check one ``(robot, zone)`` pair."""
    approach = approach_distance_m(lot, robot, zone)
    exit_x, exit_y = node_xy(lot, zone.exit_waypoint)
    home = nearest_bay(lot, exit_x, exit_y)
    work_m = approach + zone.sweep_length_m
    return PairCost(
        robot_id=robot.robot_id,
        zone_id=zone.id,
        approach_m=approach,
        sweep_m=zone.sweep_length_m,
        return_m=home.distance_m,
        return_bay_id=home.bay_id,
        required_soc=required_soc(work_m + home.distance_m, params),
        battery_soc=robot.battery_soc,
    )


def assignment_cost(lot: LotSpec, robot: RobotSnapshot, zone: Zone) -> float:
    """``c(i, z) = d_lane(robot -> entry(z)) + sweep_length_m(z)``, in metres."""
    return approach_distance_m(lot, robot, zone) + zone.sweep_length_m


def battery_feasible(lot: LotSpec, robot: RobotSnapshot, zone: Zone, params: BatteryParams) -> bool:
    """Whether ``robot`` can sweep ``zone`` and still reach a charging bay after it."""
    return evaluate_pair(lot, robot, zone, params).feasible


def _frozen_f64(values: npt.ArrayLike) -> npt.NDArray[np.float64]:
    array = np.array(values, dtype=np.float64, copy=True)
    array.setflags(write=False)
    return array


def _frozen_bool(values: npt.ArrayLike) -> npt.NDArray[np.bool_]:
    array = np.array(values, dtype=np.bool_, copy=True)
    array.setflags(write=False)
    return array


@dataclass(frozen=True, slots=True, eq=False)
class CostModel:
    """Everything the three strategies need, as arrays over sorted id tuples.

    Rows are robots and columns are zones, both in the order given by ``robot_ids`` and
    ``zone_ids``. :func:`build_cost_model` sorts those, so the model -- and therefore
    every strategy's tie-breaking -- is independent of the order the coordinator happened
    to iterate its dictionaries in.

    ``link_m[a, b] = d_lane(exit(zone a) -> entry(zone b))`` is what makes SSI's
    marginal-cost bids possible: a sequential single-item auction prices a zone by what
    it adds to a robot's *path*, which needs zone-to-zone distances that a plain
    ``robot x zone`` matrix cannot express. ``greedy`` and ``hungarian`` never read it.

    ``eq=False`` because comparing two models would compare numpy arrays, which yields an
    array rather than a bool. The arrays are copied on construction and marked read-only,
    so a model is an immutable value in practice as well as in intent.
    """

    robot_ids: tuple[RobotId, ...]
    zone_ids: tuple[str, ...]
    approach_m: npt.NDArray[np.float64]
    """``(n_robots, n_zones)`` lane distance to each zone entry."""
    sweep_m: npt.NDArray[np.float64]
    """``(n_zones,)`` authored sweep length per zone."""
    link_m: npt.NDArray[np.float64]
    """``(n_zones, n_zones)`` exit-to-entry lane distance; the diagonal is unused."""
    feasible: npt.NDArray[np.bool_]
    """``(n_robots, n_zones)`` battery feasibility, return leg included."""

    def __post_init__(self) -> None:
        n, m = len(self.robot_ids), len(self.zone_ids)
        if len(set(self.robot_ids)) != n:
            msg = f"robot_ids contains duplicates: {self.robot_ids}"
            raise ValueError(msg)
        if len(set(self.zone_ids)) != m:
            msg = f"zone_ids contains duplicates: {self.zone_ids}"
            raise ValueError(msg)

        object.__setattr__(self, "approach_m", _frozen_f64(self.approach_m))
        object.__setattr__(self, "sweep_m", _frozen_f64(self.sweep_m))
        object.__setattr__(self, "link_m", _frozen_f64(self.link_m))
        object.__setattr__(self, "feasible", _frozen_bool(self.feasible))

        expected: tuple[tuple[str, tuple[int, ...], tuple[int, ...]], ...] = (
            ("approach_m", self.approach_m.shape, (n, m)),
            ("sweep_m", self.sweep_m.shape, (m,)),
            ("link_m", self.link_m.shape, (m, m)),
            ("feasible", self.feasible.shape, (n, m)),
        )
        for name, actual, want in expected:
            if actual != want:
                msg = f"{name} must have shape {want}, got {actual}"
                raise ValueError(msg)

        for name, array in (
            ("approach_m", self.approach_m),
            ("sweep_m", self.sweep_m),
            ("link_m", self.link_m),
        ):
            if not bool(np.all(np.isfinite(array))):
                msg = f"{name} must be finite; got {array!r}"
                raise ValueError(msg)
            if bool(np.any(array < 0.0)):
                msg = f"{name} must be non-negative; got {array!r}"
                raise ValueError(msg)

        if n and m and float(np.max(self.pair_costs())) >= INFEASIBLE_COST:
            msg = (
                f"a real pair cost reached the infeasibility sentinel "
                f"({INFEASIBLE_COST}); the sentinel must stay unreachable by real costs"
            )
            raise ValueError(msg)

    # ------------------------------------------------------------------ shape --
    @property
    def n_robots(self) -> int:
        return len(self.robot_ids)

    @property
    def n_zones(self) -> int:
        return len(self.zone_ids)

    def robot_index(self, robot_id: RobotId) -> int:
        try:
            return self.robot_ids.index(robot_id)
        except ValueError:
            msg = f"{robot_id!r} is not a robot of this cost model {self.robot_ids}"
            raise KeyError(msg) from None

    def zone_index(self, zone_id: str) -> int:
        try:
            return self.zone_ids.index(zone_id)
        except ValueError:
            msg = f"{zone_id!r} is not a zone of this cost model {self.zone_ids}"
            raise KeyError(msg) from None

    # ------------------------------------------------------------------ costs --
    def pair_costs(self) -> npt.NDArray[np.float64]:
        """``(n_robots, n_zones)`` matrix of ``c(i, z)``, ignoring feasibility."""
        return self.approach_m + self.sweep_m[np.newaxis, :]

    def masked_costs(self, sentinel: float = INFEASIBLE_COST) -> npt.NDArray[np.float64]:
        """``pair_costs`` with infeasible cells replaced by a large finite sentinel."""
        return np.where(self.feasible, self.pair_costs(), sentinel)

    def pair_cost(self, robot_id: RobotId, zone_id: str) -> float:
        i, j = self.robot_index(robot_id), self.zone_index(zone_id)
        return float(self.approach_m[i, j]) + float(self.sweep_m[j])

    def is_feasible(self, robot_id: RobotId, zone_id: str) -> bool:
        return bool(self.feasible[self.robot_index(robot_id), self.zone_index(zone_id)])

    def tour_cost(self, robot_id: RobotId, zone_ids: Sequence[str]) -> float:
        """Robot path cost (RPC) of visiting ``zone_ids`` in order, in metres.

        ``approach(first) + sum(sweep) + sum(exit -> entry links)``. This is the
        ``BidSumPath`` objective of Lagoudakis et al. (RSS 2005): SSI bids the increase
        in *this* number, which is why it can beat a myopic per-pair argmin.
        """
        if not zone_ids:
            return 0.0
        i = self.robot_index(robot_id)
        first = self.zone_index(zone_ids[0])
        total = float(self.approach_m[i, first]) + float(self.sweep_m[first])
        previous = first
        for zone_id in zone_ids[1:]:
            current = self.zone_index(zone_id)
            total += float(self.link_m[previous, current]) + float(self.sweep_m[current])
            previous = current
        return total

    # ----------------------------------------------------------- construction --
    @classmethod
    def from_pair_costs(
        cls,
        robot_ids: Sequence[RobotId],
        zone_ids: Sequence[str],
        costs: npt.ArrayLike,
        *,
        feasible: npt.ArrayLike | None = None,
        link_m: npt.ArrayLike | None = None,
    ) -> Self:
        """Build a model straight from a ``robot x zone`` matrix.

        For tests and for the property test over random matrices, where there is no lot:
        the whole cost lands in ``approach_m`` and every sweep length is zero, which
        leaves ``pair_costs() == costs`` exactly. ``link_m`` defaults to zeros, so a model
        built this way is meaningful for ``greedy`` and ``hungarian`` and degenerate (but
        still valid) for ``ssi``, which prices paths rather than pairs.
        """
        n, m = len(robot_ids), len(zone_ids)
        return cls(
            robot_ids=tuple(robot_ids),
            zone_ids=tuple(zone_ids),
            approach_m=_frozen_f64(costs),
            sweep_m=np.zeros(m, dtype=np.float64),
            link_m=np.zeros((m, m), dtype=np.float64) if link_m is None else _frozen_f64(link_m),
            feasible=(
                np.ones((n, m), dtype=np.bool_) if feasible is None else _frozen_bool(feasible)
            ),
        )


def build_cost_model(
    lot: LotSpec,
    robots: Sequence[RobotSnapshot],
    zones: Sequence[Zone],
    params: BatteryParams,
) -> CostModel:
    """Price every ``(robot, zone)`` pair on ``lot`` into a :class:`CostModel`.

    Robots and zones are sorted by id first: the model is the allocator's whole input, so
    sorting here is what makes every downstream tie-break reproducible.
    """
    ordered_robots = sorted(robots, key=lambda r: r.robot_id)
    ordered_zones = sorted(zones, key=lambda z: z.id)
    n, m = len(ordered_robots), len(ordered_zones)

    approach = np.zeros((n, m), dtype=np.float64)
    sweep = np.zeros(m, dtype=np.float64)
    feasible = np.zeros((n, m), dtype=np.bool_)
    for j, zone in enumerate(ordered_zones):
        sweep[j] = zone.sweep_length_m
    for i, robot in enumerate(ordered_robots):
        for j, zone in enumerate(ordered_zones):
            pair = evaluate_pair(lot, robot, zone, params)
            approach[i, j] = pair.approach_m
            feasible[i, j] = pair.feasible

    link = np.zeros((m, m), dtype=np.float64)
    for a, zone_a in enumerate(ordered_zones):
        for b, zone_b in enumerate(ordered_zones):
            if a != b:
                link[a, b] = lane_distance_m(lot, zone_a.exit_waypoint, zone_b.entry_waypoint)

    return CostModel(
        robot_ids=tuple(r.robot_id for r in ordered_robots),
        zone_ids=tuple(z.id for z in ordered_zones),
        approach_m=approach,
        sweep_m=sweep,
        link_m=link,
        feasible=feasible,
    )

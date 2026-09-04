"""Tests for the shared cost function and the three allocation strategies.

The load-bearing ones are the two that turn a claim into a measurement:

* :func:`test_greedy_total_never_beats_hungarian_over_1000_matrices` -- acceptance C9.
* :func:`test_pinned_regression_greedy_is_6_29_times_optimal` -- the exact ratio the
  README and ADR-0005 quote, pinned so it cannot drift.

Every random matrix here comes from an explicitly seeded ``numpy.random.Generator``.
There is no global RNG anywhere in this file, so a failure is reproducible from the seed
printed in the parametrisation.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from parkscan_agent.battery_policy import BatteryParams, sortie_feasible
from parkscan_agent.lanegraph import lane_distance_m, nearest_node
from parkscan_coord.allocation import (
    AllocationError,
    AllocatorState,
    Assignment,
    Strategy,
    _validate_assignments,
    allocate,
    allocate_model,
    plan_bundles,
    total_cost_m,
)
from parkscan_coord.cost import (
    INFEASIBLE_COST,
    CostModel,
    RobotSnapshot,
    build_cost_model,
    evaluate_pair,
)

from parkscan_core.lot import LotSpec, load_lot
from parkscan_core.types import Pose2D, ZoneState

REPO_ROOT = Path(__file__).resolve().parents[3]
LOT_YAML = REPO_ROOT / "sim" / "lot" / "lot.yaml"

STRATEGIES = tuple(Strategy)


@pytest.fixture(scope="module")
def lot() -> LotSpec:
    return load_lot(LOT_YAML)


@pytest.fixture(scope="module")
def params() -> BatteryParams:
    return BatteryParams()


def make_robot(robot_id: str, x_m: float, y_m: float, soc: float = 1.0) -> RobotSnapshot:
    return RobotSnapshot(
        robot_id=robot_id,
        pose=Pose2D(x_m=x_m, y_m=y_m, theta_rad=0.0),
        battery_soc=soc,
    )


def fleet(soc: float = 1.0) -> tuple[RobotSnapshot, ...]:
    """Three robots spread across the lot: south-west corner, middle, north-east."""
    return (
        make_robot("robot_1", 5.0, 5.0, soc),
        make_robot("robot_2", 75.0, 50.0, soc),
        make_robot("robot_3", 140.0, 90.0, soc),
    )


def all_zones_unassigned(lot: LotSpec) -> dict[str, ZoneState]:
    return {zone.id: ZoneState.UNASSIGNED for zone in lot.zones}


# ------------------------------------------------------------------ cost function --
def test_cost_is_lane_distance_plus_sweep_never_euclidean(
    lot: LotSpec, params: BatteryParams
) -> None:
    """``c(i, z) = d_lane(robot -> entry(z)) + sweep_length_m``.

    The straight-line figure is checked too, and asserted to be *different*: if the two
    ever agreed for this pair the test would pass while proving nothing.
    """
    robot = make_robot("robot_1", 5.0, 5.0)
    zone = next(z for z in lot.zones if z.id == "zone_12")

    pair = evaluate_pair(lot, robot, zone, params)
    start = nearest_node(lot, robot.pose.x_m, robot.pose.y_m)
    expected = lane_distance_m(lot, start, zone.entry_waypoint)

    assert pair.approach_m == pytest.approx(expected)
    assert pair.cost_m == pytest.approx(expected + zone.sweep_length_m)

    entry_x, entry_y = next(
        (n.x_m, n.y_m) for n in lot.lane_graph.nodes if n.id == zone.entry_waypoint
    )
    euclidean = float(np.hypot(entry_x - robot.pose.x_m, entry_y - robot.pose.y_m))
    assert pair.approach_m > euclidean + 1.0


def test_feasibility_agrees_with_the_robots_own_battery_model(
    lot: LotSpec, params: BatteryParams
) -> None:
    """One battery model, not two.

    The allocator's predicate must be exactly
    :func:`parkscan_agent.battery_policy.sortie_feasible` -- same margin, same reserve,
    same return leg. A second implementation that drifted by a percent would let the
    coordinator hand out a sortie the robot's own check would refuse mid-drive.
    """
    for robot in fleet(soc=0.55):
        for zone in lot.zones:
            pair = evaluate_pair(lot, robot, zone, params)
            expected = sortie_feasible(
                robot.battery_soc,
                lot,
                params,
                work_distance_m=pair.approach_m + pair.sweep_m,
                finish_node=zone.exit_waypoint,
            )
            assert pair.feasible is expected, f"{robot.robot_id} / {zone.id}"


def test_feasibility_accounts_for_the_return_leg(lot: LotSpec, params: BatteryParams) -> None:
    """A charge that covers approach + sweep but not the drive home is infeasible."""
    zone = next(z for z in lot.zones if z.id == "zone_12")
    robot = make_robot("robot_1", 5.0, 5.0)
    pair = evaluate_pair(lot, robot, zone, params)
    assert pair.return_m > 0.0

    one_way_only = params.soc_per_metre * pair.cost_m * (1.0 + params.margin)
    stranded = make_robot("robot_1", 5.0, 5.0, soc=one_way_only)
    assert evaluate_pair(lot, stranded, zone, params).feasible is False


def test_cost_model_arrays_are_read_only(lot: LotSpec, params: BatteryParams) -> None:
    model = build_cost_model(lot, fleet(), list(lot.zones), params)
    with pytest.raises(ValueError, match="read-only"):
        model.approach_m[0, 0] = 0.0


def test_cost_model_rejects_a_cost_at_the_sentinel() -> None:
    """Real costs must stay far below the sentinel, or "infeasible" stops meaning it."""
    with pytest.raises(ValueError, match="sentinel"):
        CostModel.from_pair_costs(("r1",), ("z1",), [[INFEASIBLE_COST]])


# ------------------------------------------------------- C9: greedy vs. hungarian --
def test_greedy_total_never_beats_hungarian_over_1000_matrices() -> None:
    """Acceptance C9, as a property test over 1000 seeded random cost matrices.

    ``linear_sum_assignment`` is optimal for the linear sum assignment problem, so the
    only possible outcomes are "greedy ties it" or "greedy is worse". A single matrix
    where greedy came out cheaper would mean the reference is not actually optimal, and
    every comparative number in the README would be worthless.

    Shapes are randomised too (1-5 robots, 1-8 zones), because a bug in the rectangular
    case is exactly the bug that would not show up on square matrices.
    """
    losses = 0
    for seed in range(1000):
        rng = np.random.default_rng(seed)
        n = int(rng.integers(1, 6))
        m = int(rng.integers(1, 9))
        costs = rng.uniform(0.0, 500.0, size=(n, m))
        model = CostModel.from_pair_costs(
            tuple(f"robot_{i}" for i in range(n)),
            tuple(f"zone_{j}" for j in range(m)),
            costs,
        )
        greedy = total_cost_m(model, allocate_model(model, strategy=Strategy.GREEDY))
        optimal = total_cost_m(model, allocate_model(model, strategy=Strategy.HUNGARIAN))
        assert greedy >= optimal - 1e-9, f"seed {seed}: greedy {greedy} < optimal {optimal}"
        losses += greedy > optimal + 1e-9
    # Not an assertion about quality, a guard against a vacuous test: if greedy tied on
    # all 1000 matrices the property above would be true for uninteresting reasons.
    assert losses > 0


PINNED_ROBOTS = ("robot_1", "robot_2", "robot_3")
PINNED_ZONES = ("zone_a", "zone_b", "zone_c")
PINNED_COSTS = (
    (10.0, 40.0, 900.0),
    (20.0, 526.1, 526.1),
    (35.0, 526.1, 30.0),
)


def test_pinned_regression_greedy_is_6_29_times_optimal() -> None:
    """The exact number ADR-0005 and the README quote, pinned to this matrix.

    Greedy takes the globally cheapest cell first -- ``robot_1`` on ``zone_a`` at 10 m --
    and in doing so spends the only robot that was cheap on ``zone_b``. ``robot_2`` is
    then forced onto ``zone_b`` at 526.1 m. The optimal assignment gives ``zone_a`` to
    ``robot_2`` for 10 m more and saves 486 m elsewhere::

        greedy  : r1->zone_a 10.0 + r3->zone_c 30.0 + r2->zone_b 526.1 = 566.1 m
        optimal : r2->zone_a 20.0 + r1->zone_b  40.0 + r3->zone_c  30.0 =  90.0 m

    This is a synthetic matrix, not one measured off the lot: it is here to pin the
    *ratio* the write-up claims, so that "greedy's gap is proved rather than asserted"
    stays true if either implementation changes.
    """
    model = CostModel.from_pair_costs(PINNED_ROBOTS, PINNED_ZONES, PINNED_COSTS)

    greedy = allocate_model(model, strategy=Strategy.GREEDY)
    optimal = allocate_model(model, strategy=Strategy.HUNGARIAN)

    assert greedy == [
        Assignment("robot_1", "zone_a"),
        Assignment("robot_2", "zone_b"),
        Assignment("robot_3", "zone_c"),
    ]
    assert optimal == [
        Assignment("robot_1", "zone_b"),
        Assignment("robot_2", "zone_a"),
        Assignment("robot_3", "zone_c"),
    ]

    greedy_total = total_cost_m(model, greedy)
    optimal_total = total_cost_m(model, optimal)
    assert greedy_total == pytest.approx(566.1)
    assert optimal_total == pytest.approx(90.0)
    assert greedy_total / optimal_total == pytest.approx(6.29, rel=1e-12)


@pytest.mark.parametrize("expensive_m", [1.0e2, 1.0e4, 1.0e6])
def test_greedy_ratio_under_cost_minimisation_is_unbounded(expensive_m: float) -> None:
    """The correction most write-ups get wrong, as an executable demonstration.

    The famous **"greedy is 2-competitive"** result is about **maximising** utility: a
    greedy matching collects at least half the optimal value. Under **cost
    minimisation**, which is what this allocator does, there is no such bound and the
    ratio is **unbounded** -- it grows linearly with a single matrix entry:

    .. code-block:: text

        robot_1: [0.9, 1.0]        greedy  : r1->zone_a 0.9 + r2->zone_b M
        robot_2: [1.0,   M]        optimal : r1->zone_b 1.0 + r2->zone_a 1.0 = 2.0
                                   ratio   : (0.9 + M) / 2  ->  unbounded in M

    At ``M = 1e6`` that is ~5.0e5, the figure ADR-0005 cites. The parametrisation shows
    the ratio rising by a factor of 100 for each factor of 100 in ``M``, which is the
    part a fixed competitive ratio would forbid. Quoting the maximisation theorem here
    would understate greedy's worst case by five orders of magnitude.
    """
    model = CostModel.from_pair_costs(
        ("robot_1", "robot_2"),
        ("zone_a", "zone_b"),
        ((0.9, 1.0), (1.0, expensive_m)),
    )
    greedy_total = total_cost_m(model, allocate_model(model, strategy=Strategy.GREEDY))
    optimal_total = total_cost_m(model, allocate_model(model, strategy=Strategy.HUNGARIAN))

    assert optimal_total == pytest.approx(2.0)
    assert greedy_total == pytest.approx(0.9 + expensive_m)
    assert greedy_total / optimal_total == pytest.approx((0.9 + expensive_m) / 2.0)


# ------------------------------------------------------------- validity of a round --
@pytest.mark.parametrize("strategy", STRATEGIES)
def test_every_strategy_returns_a_valid_matching(
    strategy: Strategy, lot: LotSpec, params: BatteryParams
) -> None:
    """No zone twice, no robot twice, nothing invented, at most one round's worth."""
    state = AllocatorState(robots=fleet(), zones=all_zones_unassigned(lot))
    chosen = allocate(state, lot, params, strategy=strategy)

    robot_ids = [a.robot_id for a in chosen]
    zone_ids = [a.zone_id for a in chosen]
    assert len(robot_ids) == len(set(robot_ids))
    assert len(zone_ids) == len(set(zone_ids))
    assert set(robot_ids) <= {r.robot_id for r in fleet()}
    assert set(zone_ids) <= {z.id for z in lot.zones}
    assert len(chosen) == min(len(fleet()), len(lot.zones))


@pytest.mark.parametrize("strategy", STRATEGIES)
def test_no_strategy_picks_a_battery_infeasible_pair(
    strategy: Strategy, lot: LotSpec, params: BatteryParams
) -> None:
    """A charge that rules some zones out must rule them out for all three strategies.

    The state of charge is chosen so the feasibility mask is genuinely mixed -- the test
    asserts that first, because a mask that was all-``True`` would make the real
    assertion pass without exercising anything.
    """
    robots = fleet(soc=0.20)
    zones = list(lot.zones)
    model = build_cost_model(lot, robots, zones, params)
    assert bool(np.any(~model.feasible)), "fixture no longer produces infeasible pairs"
    assert bool(np.any(model.feasible)), "fixture no longer produces feasible pairs"

    state = AllocatorState(robots=robots, zones=all_zones_unassigned(lot))
    for assignment in allocate(state, lot, params, strategy=strategy):
        assert model.is_feasible(assignment.robot_id, assignment.zone_id)


@pytest.mark.parametrize("strategy", STRATEGIES)
def test_a_robot_with_no_feasible_zone_is_left_idle(strategy: Strategy) -> None:
    """Better an idle robot than one sent on a sortie it cannot come back from.

    This is the case that makes the Hungarian sentinel subtle: ``linear_sum_assignment``
    must place every row somewhere, so ``robot_2``'s row lands on a sentinel cell and has
    to be dropped afterwards rather than trusted.
    """
    model = CostModel.from_pair_costs(
        ("robot_1", "robot_2"),
        ("zone_a", "zone_b"),
        ((10.0, 20.0), (30.0, 40.0)),
        feasible=((True, True), (False, False)),
    )
    chosen = allocate_model(model, strategy=strategy)
    assert [a.robot_id for a in chosen] == ["robot_1"]
    assert chosen[0].zone_id == "zone_a"


def test_the_validator_rejects_an_infeasible_pair_whatever_produced_it() -> None:
    """The guard behind the sentinel, driven directly rather than trusted.

    Every strategy returns through :func:`_validate_assignments`, so this is the last
    line of defence against a battery violation reaching a robot. It is exercised with a
    hand-made assignment because no current strategy will produce one -- which is exactly
    why a future one must still be caught.
    """
    model = CostModel.from_pair_costs(("robot_1",), ("zone_a",), ((5.0,),), feasible=((False,),))
    assert allocate_model(model, strategy=Strategy.GREEDY) == []
    with pytest.raises(AllocationError, match="charge"):
        _validate_assignments(model, [Assignment("robot_1", "zone_a")])


# ------------------------------------------------------------------- determinism --
@pytest.mark.parametrize("strategy", STRATEGIES)
def test_same_input_gives_the_same_output(
    strategy: Strategy, lot: LotSpec, params: BatteryParams
) -> None:
    state = AllocatorState(robots=fleet(soc=0.6), zones=all_zones_unassigned(lot))
    first = allocate(state, lot, params, strategy=strategy)
    for _ in range(5):
        assert allocate(state, lot, params, strategy=strategy) == first


@pytest.mark.parametrize("strategy", STRATEGIES)
def test_output_does_not_depend_on_input_ordering(
    strategy: Strategy, lot: LotSpec, params: BatteryParams
) -> None:
    """``make bench`` replays 30 seeds; a dict-order-sensitive allocator would ruin it.

    The same fleet and the same zones are presented in reversed order. Because the
    allocator sorts both before costing anything, the assignments must be identical --
    not merely equivalent in total cost.
    """
    zones = all_zones_unassigned(lot)
    forward = AllocatorState(robots=fleet(soc=0.6), zones=zones)
    backward = AllocatorState(
        robots=tuple(reversed(fleet(soc=0.6))),
        zones=dict(reversed(list(zones.items()))),
    )
    assert allocate(forward, lot, params, strategy=strategy) == allocate(
        backward, lot, params, strategy=strategy
    )


# -------------------------------------------------------------------------- SSI --
def test_ssi_bids_marginal_path_cost_with_cheapest_insertion() -> None:
    """SSI must insert into the middle of a tour, not append to the end of it.

    Pinned so the mechanism is visible. One robot, three zones, zero sweep lengths, and
    an ``exit -> entry`` link matrix that makes ``zone_c`` cheap between ``zone_a`` and
    ``zone_b`` but expensive after ``zone_b``::

        round 1: bids are the pair costs        -> zone_a wins (0.0)
        round 2: zone_b after zone_a bids 1.0,
                 zone_c after zone_a bids 1.0   -> tie, broken by zone id -> zone_b
        round 3: zone_c between a and b bids 1.0,
                 zone_c appended after b bids 40.0

    So the tour is ``(a, c, b)`` costing 2.0 m. An append-only auction -- the mistake
    that makes SSI a slower greedy -- would have produced ``(a, b, c)`` at 41.0 m.
    """
    zones = ("zone_a", "zone_b", "zone_c")
    links = (
        (0.0, 1.0, 1.0),  # a -> b, a -> c
        (1.0, 0.0, 40.0),  # b -> a, b -> c
        (50.0, 1.0, 0.0),  # c -> a, c -> b
    )
    model = CostModel.from_pair_costs(("robot_1",), zones, ((0.0, 100.0, 3.0),), link_m=links)

    bundles = plan_bundles(model)
    assert len(bundles) == 1
    assert bundles[0].zone_ids == ("zone_a", "zone_c", "zone_b")
    assert bundles[0].path_cost_m == pytest.approx(2.0)
    assert model.tour_cost("robot_1", ("zone_a", "zone_b", "zone_c")) == pytest.approx(41.0)

    # Only the head of the tour is dispatched; the rest is re-auctioned next round.
    assert allocate_model(model, strategy=Strategy.SSI) == [Assignment("robot_1", "zone_a")]


def test_ssi_bundles_partition_the_zones(lot: LotSpec, params: BatteryParams) -> None:
    """Every zone is won exactly once, and each tour costs what the model says it does."""
    model = build_cost_model(lot, fleet(), list(lot.zones), params)
    bundles = plan_bundles(model)

    won = [zone_id for bundle in bundles for zone_id in bundle.zone_ids]
    assert sorted(won) == sorted(z.id for z in lot.zones)
    assert len(won) == len(set(won))
    for bundle in bundles:
        assert bundle.path_cost_m == pytest.approx(
            model.tour_cost(bundle.robot_id, bundle.zone_ids)
        )


def test_ssi_awards_nothing_when_no_robot_can_afford_anything(
    lot: LotSpec, params: BatteryParams
) -> None:
    state = AllocatorState(robots=fleet(soc=0.02), zones=all_zones_unassigned(lot))
    assert allocate(state, lot, params, strategy=Strategy.SSI) == []


# ------------------------------------------------------------- state filtering --
@pytest.mark.parametrize("strategy", STRATEGIES)
def test_only_unassigned_zones_are_allocated(
    strategy: Strategy, lot: LotSpec, params: BatteryParams
) -> None:
    zones = dict.fromkeys((z.id for z in lot.zones), ZoneState.COMPLETE)
    zones["zone_05"] = ZoneState.UNASSIGNED
    zones["zone_06"] = ZoneState.IN_PROGRESS
    zones["zone_07"] = ZoneState.UNRECOVERABLE

    chosen = allocate(AllocatorState(robots=fleet(), zones=zones), lot, params, strategy=strategy)
    assert [a.zone_id for a in chosen] == ["zone_05"]


@pytest.mark.parametrize("strategy", STRATEGIES)
def test_unavailable_robots_are_never_given_work(
    strategy: Strategy, lot: LotSpec, params: BatteryParams
) -> None:
    robots = (
        RobotSnapshot("robot_1", Pose2D(5.0, 5.0, 0.0), 1.0, available=False),
        make_robot("robot_2", 75.0, 50.0),
    )
    chosen = allocate(
        AllocatorState(robots=robots, zones=all_zones_unassigned(lot)),
        lot,
        params,
        strategy=strategy,
    )
    assert [a.robot_id for a in chosen] == ["robot_2"]


@pytest.mark.parametrize("strategy", STRATEGIES)
def test_nothing_to_do_returns_an_empty_list(
    strategy: Strategy, lot: LotSpec, params: BatteryParams
) -> None:
    assert allocate(AllocatorState((), {}), lot, params, strategy=strategy) == []
    assert (
        allocate(
            AllocatorState(fleet(), dict.fromkeys(("zone_01",), ZoneState.COMPLETE)),
            lot,
            params,
            strategy=strategy,
        )
        == []
    )


def test_a_zone_that_is_not_in_the_lot_is_an_error(lot: LotSpec, params: BatteryParams) -> None:
    state = AllocatorState(fleet(), {"zone_99": ZoneState.UNASSIGNED})
    with pytest.raises(AllocationError, match="zone_99"):
        allocate(state, lot, params)


def test_strategy_accepts_its_string_name(lot: LotSpec, params: BatteryParams) -> None:
    """The runner reads the strategy from config, so a plain string has to work."""
    state = AllocatorState(robots=fleet(), zones=all_zones_unassigned(lot))
    assert allocate(state, lot, params, strategy=Strategy("hungarian")) == allocate(
        state, lot, params, strategy=Strategy.HUNGARIAN
    )
    with pytest.raises(ValueError, match="ssi-plus"):
        Strategy("ssi-plus")

"""Metric definitions, each pinned against a fixture whose answer was worked out by hand.

A benchmark is only as honest as its definitions, so every test here computes the expected
value independently of the implementation -- by counting, not by re-running the function --
and the awkward cases (no events at all, a clean hand-off, the same robot covering a cell
twice) get their own tests because they are where a definition quietly becomes a claim it
should not make.
"""

from __future__ import annotations

import math

import pytest
from parkscan_coord.health import NS_PER_S, HealthConfig
from parkscan_coord.metrics import (
    METRIC_NAMES,
    BatterySample,
    CellVisit,
    DockEvent,
    Occupancy,
    ReassignmentEvent,
    RunObservations,
    TrackPoint,
    ZoneCompletion,
    ZoneOutcome,
    build_report,
    conflict_count,
    coverage_pct,
    depletion_events,
    detection_latency_s,
    duplicate_visit_rate,
    makespan_s,
    min_battery_at_dock,
    paired_bootstrap,
    reassignment_churn,
    reassignment_latency_s,
    total_distance_m,
)

from parkscan_core.types import ZoneState


# ------------------------------------------------------------------ coverage_pct --
def test_coverage_is_cell_weighted_not_zone_weighted() -> None:
    # 90 of 100 planned cells covered, but spread so that a zone-weighted average would
    # say 50%: one small zone finished, one large zone barely started.
    zones = (
        ZoneOutcome(zone_id="zone_a", cells_total=10, cells_done=10, state=ZoneState.COMPLETE),
        ZoneOutcome(zone_id="zone_b", cells_total=90, cells_done=80, state=ZoneState.UNASSIGNED),
    )
    assert coverage_pct(zones) == pytest.approx(90.0)


def test_coverage_of_an_empty_lot_is_complete() -> None:
    assert coverage_pct(()) == pytest.approx(100.0)


def test_a_zone_cannot_report_more_cells_than_it_planned() -> None:
    with pytest.raises(ValueError, match="exceeds"):
        ZoneOutcome(zone_id="zone_a", cells_total=4, cells_done=5, state=ZoneState.COMPLETE)


# -------------------------------------------------------------------- makespan_s --
def test_makespan_is_the_last_completion_not_the_sum() -> None:
    completions = (
        ZoneCompletion(zone_id="zone_a", stamp_ns=10 * NS_PER_S),
        ZoneCompletion(zone_id="zone_b", stamp_ns=42 * NS_PER_S),
        ZoneCompletion(zone_id="zone_c", stamp_ns=17 * NS_PER_S),
    )
    assert makespan_s(2 * NS_PER_S, completions) == pytest.approx(40.0)


def test_makespan_without_completions_is_zero() -> None:
    assert makespan_s(0, ()) == pytest.approx(0.0)


# --------------------------------------------------------------- total_distance_m --
def test_distance_groups_by_robot_and_orders_by_time() -> None:
    # robot_1 walks a 3-4-5 triangle's legs (3 + 4 = 7 m); robot_2 walks 2 m.
    # Deliberately interleaved and out of order, to prove neither matters.
    track = (
        TrackPoint(robot_id="robot_1", stamp_ns=2, x_m=3.0, y_m=0.0),
        TrackPoint(robot_id="robot_2", stamp_ns=1, x_m=0.0, y_m=0.0),
        TrackPoint(robot_id="robot_1", stamp_ns=1, x_m=0.0, y_m=0.0),
        TrackPoint(robot_id="robot_2", stamp_ns=2, x_m=0.0, y_m=2.0),
        TrackPoint(robot_id="robot_1", stamp_ns=3, x_m=3.0, y_m=4.0),
    )
    assert total_distance_m(track) == pytest.approx(9.0)


def test_a_single_point_has_no_distance() -> None:
    assert total_distance_m((TrackPoint("robot_1", 0, 1.0, 1.0),)) == pytest.approx(0.0)


# ---------------------------------------------------------- duplicate_visit_rate --
def test_only_two_different_robots_make_a_duplicate() -> None:
    visits = (
        CellVisit(robot_id="robot_1", cell_id="zone_a:c00r00"),
        CellVisit(robot_id="robot_1", cell_id="zone_a:c00r00"),  # same robot, same cell
        CellVisit(robot_id="robot_1", cell_id="zone_a:c01r00"),
        CellVisit(robot_id="robot_2", cell_id="zone_a:c01r00"),  # the only duplicate
        CellVisit(robot_id="robot_2", cell_id="zone_b:c00r00"),
        CellVisit(robot_id="robot_3", cell_id="zone_c:c00r00"),
    )
    # 4 distinct cells, 1 of them covered by two different robots.
    assert duplicate_visit_rate(visits) == pytest.approx(0.25)


def test_duplicate_rate_of_a_run_that_covered_nothing_is_zero() -> None:
    assert duplicate_visit_rate(()) == pytest.approx(0.0)


# ------------------------------------------------------------------- latencies --
def test_reassignment_latency_is_the_mean_over_recovery_events() -> None:
    events = (
        ReassignmentEvent(
            zone_id="zone_a",
            failed_robot="robot_1",
            replacement_robot="robot_2",
            detected_ns=3 * NS_PER_S,
            reassigned_ns=5 * NS_PER_S,
            cause="robot declared offline",
        ),
        ReassignmentEvent(
            zone_id="zone_b",
            failed_robot="robot_2",
            replacement_robot="robot_3",
            detected_ns=10 * NS_PER_S,
            reassigned_ns=14 * NS_PER_S,
            cause="robot declared stuck",
        ),
    )
    assert reassignment_latency_s(events) == pytest.approx(3.0)


def test_no_reassignments_is_nan_not_zero() -> None:
    # "nothing was reassigned" and "reassignment took no time" are different statements,
    # and 0.0 would report the second while meaning the first.
    assert math.isnan(reassignment_latency_s(()))


def test_a_reassignment_cannot_precede_its_own_detection() -> None:
    with pytest.raises(ValueError, match="precedes"):
        ReassignmentEvent(
            zone_id="zone_a",
            failed_robot="robot_1",
            replacement_robot="robot_2",
            detected_ns=5 * NS_PER_S,
            reassigned_ns=4 * NS_PER_S,
            cause="impossible",
        )


@pytest.mark.parametrize(
    ("period_ns", "k", "expected_s"),
    [(NS_PER_S, 3, 3.0), (NS_PER_S // 2, 3, 1.5), (2 * NS_PER_S, 1, 2.0)],
)
def test_detection_latency_is_exactly_the_configured_deadline(
    period_ns: int, k: int, expected_s: float
) -> None:
    # It is k x heartbeat_period and nothing else -- a number we chose, not one we earned.
    config = HealthConfig(heartbeat_period_ns=period_ns, missed_beats_offline=k)
    assert detection_latency_s(config) == pytest.approx(expected_s)


def test_the_two_latencies_are_separate_rows_and_are_never_summed() -> None:
    config = HealthConfig(heartbeat_period_ns=NS_PER_S, missed_beats_offline=3)
    events = (
        ReassignmentEvent(
            zone_id="zone_a",
            failed_robot="robot_1",
            replacement_robot="robot_2",
            detected_ns=3 * NS_PER_S,
            reassigned_ns=5 * NS_PER_S,
            cause="robot declared offline",
        ),
    )
    report = build_report(
        RunObservations(start_ns=0, health=config, reassignments=events, assignments_total=2)
    )

    assert report.reassignment_latency_s == pytest.approx(2.0)
    assert report.detection_latency_s == pytest.approx(3.0)

    rows = {sample.name: sample.value for sample in report.samples()}
    assert rows["reassignment_latency_s"] == pytest.approx(2.0)
    assert rows["detection_latency_s"] == pytest.approx(3.0)

    # The tempting headline -- "recovered in 5.0 s" -- is not produced anywhere. ACCEPTANCE
    # H6: the two are reported separately and never added.
    assert all(sample.value != pytest.approx(5.0) for sample in report.samples())
    assert report.reassignment_events_n == 1


# -------------------------------------------------------------------- churn --
@pytest.mark.parametrize(
    ("voluntary", "total", "expected"),
    [(0, 12, 0.0), (3, 12, 0.25), (0, 0, 0.0), (1, 1, 1.0)],
)
def test_churn_is_the_discretionary_share_of_assignments(
    voluntary: int, total: int, expected: float
) -> None:
    assert reassignment_churn(voluntary, total) == pytest.approx(expected)


# ------------------------------------------------------------------- battery --
def test_min_battery_at_dock_is_the_worst_arrival() -> None:
    docks = (
        DockEvent(robot_id="robot_1", stamp_ns=1, battery_soc=0.31),
        DockEvent(robot_id="robot_2", stamp_ns=2, battery_soc=0.12),
        DockEvent(robot_id="robot_1", stamp_ns=3, battery_soc=0.28),
    )
    assert min_battery_at_dock(docks) == pytest.approx(0.12)


def test_no_docks_is_nan_not_a_full_battery() -> None:
    assert math.isnan(min_battery_at_dock(()))


def test_a_flat_robot_that_stays_flat_is_one_depletion_episode() -> None:
    samples = tuple(
        BatterySample(robot_id="robot_1", stamp_ns=i * NS_PER_S, soc=0.0 if i >= 3 else 0.4)
        for i in range(10)
    )
    assert depletion_events(samples) == 1


def test_recharging_and_depleting_again_counts_twice() -> None:
    socs = [0.4, 0.0, 0.0, 0.5, 0.9, 0.0, 0.0]
    samples = tuple(
        BatterySample(robot_id="robot_1", stamp_ns=i * NS_PER_S, soc=soc)
        for i, soc in enumerate(socs)
    )
    assert depletion_events(samples) == 2


def test_a_healthy_fleet_has_no_depletion_events() -> None:
    samples = tuple(
        BatterySample(robot_id=f"robot_{r}", stamp_ns=i * NS_PER_S, soc=0.2 + 0.1 * i)
        for r in (1, 2, 3)
        for i in range(5)
    )
    assert depletion_events(samples) == 0


# ------------------------------------------------------------------ conflicts --
def test_a_clean_handoff_is_not_a_conflict() -> None:
    # robot_1 leaves the spur at exactly the instant robot_2 enters it. Intervals are
    # half-open, so this is a hand-off, not two robots in one cell.
    holds = (
        Occupancy(resource_id="ln_bay_1_spur", robot_id="robot_1", start_ns=0, end_ns=100),
        Occupancy(resource_id="ln_bay_1_spur", robot_id="robot_2", start_ns=100, end_ns=200),
    )
    assert conflict_count(holds) == 0


def test_two_holders_of_one_resource_is_one_conflict() -> None:
    holds = (
        Occupancy(resource_id="ln_bay_1_spur", robot_id="robot_1", start_ns=0, end_ns=150),
        Occupancy(resource_id="ln_bay_1_spur", robot_id="robot_2", start_ns=100, end_ns=200),
        Occupancy(resource_id="ln_bay_2_spur", robot_id="robot_3", start_ns=100, end_ns=200),
    )
    assert conflict_count(holds) == 1


def test_three_overlapping_holders_are_three_pairs() -> None:
    holds = tuple(
        Occupancy(resource_id="bay_1", robot_id=f"robot_{i}", start_ns=0, end_ns=100)
        for i in (1, 2, 3)
    )
    assert conflict_count(holds) == 3


def test_the_same_robot_re_entering_a_resource_is_not_a_conflict() -> None:
    holds = (
        Occupancy(resource_id="bay_1", robot_id="robot_1", start_ns=0, end_ns=100),
        Occupancy(resource_id="bay_1", robot_id="robot_1", start_ns=50, end_ns=150),
    )
    assert conflict_count(holds) == 0


# --------------------------------------------------------------------- report --
def test_report_emits_the_ten_fixed_names_in_order() -> None:
    report = build_report(RunObservations(start_ns=0, health=HealthConfig()))
    assert tuple(sample.name for sample in report.samples()) == METRIC_NAMES
    assert len(METRIC_NAMES) == 10


def test_a_nominal_run_report_reads_the_way_acceptance_expects() -> None:
    # The C1-C4 shape: complete coverage, no duplication, no churn, no conflicts.
    config = HealthConfig()
    observations = RunObservations(
        start_ns=0,
        health=config,
        zones=(
            ZoneOutcome(zone_id="zone_a", cells_total=6, cells_done=6, state=ZoneState.COMPLETE),
            ZoneOutcome(zone_id="zone_b", cells_total=6, cells_done=6, state=ZoneState.COMPLETE),
        ),
        completions=(
            ZoneCompletion(zone_id="zone_a", stamp_ns=30 * NS_PER_S),
            ZoneCompletion(zone_id="zone_b", stamp_ns=45 * NS_PER_S),
        ),
        visits=(
            CellVisit(robot_id="robot_1", cell_id="zone_a:c00r00"),
            CellVisit(robot_id="robot_2", cell_id="zone_b:c00r00"),
        ),
        docks=(DockEvent(robot_id="robot_1", stamp_ns=50 * NS_PER_S, battery_soc=0.33),),
        battery=(BatterySample(robot_id="robot_1", stamp_ns=0, soc=1.0),),
        assignments_total=2,
        voluntary_reassignments=0,
    )
    report = build_report(observations)

    assert report.coverage_pct == pytest.approx(100.0)
    assert report.makespan_s == pytest.approx(45.0)
    assert report.duplicate_visit_rate == pytest.approx(0.0)
    assert report.reassignment_churn == pytest.approx(0.0)
    assert report.conflict_count == 0
    assert report.depletion_events == 0
    assert report.min_battery_at_dock == pytest.approx(0.33)
    assert math.isnan(report.reassignment_latency_s)
    assert report.reassignment_events_n == 0


# ---------------------------------------------------------- paired bootstrap --
def _paired_fixture() -> tuple[list[float], list[float]]:
    """Ten paired seeds where ``a`` wins on eight, loses on one and ties on one."""
    a = [10.0, 11.0, 9.0, 12.0, 10.5, 11.5, 9.5, 13.0, 10.0, 11.0]
    b = [12.0, 13.0, 11.0, 14.0, 12.5, 13.5, 11.5, 12.0, 10.0, 13.0]
    return a, b


def test_bootstrap_is_reproducible_for_a_fixed_seed() -> None:
    a, b = _paired_fixture()
    first = paired_bootstrap(a, b, seed=20260904)
    second = paired_bootstrap(a, b, seed=20260904)
    assert first == second
    assert first.ci_low == second.ci_low
    assert first.ci_high == second.ci_high


def test_the_point_estimate_does_not_depend_on_the_seed() -> None:
    # Only the interval is resampled; the median difference is a statistic of the data.
    # (The interval itself may legitimately coincide across seeds on coarse data like
    # this, so asserting that it moves would be testing the fixture, not the code.)
    a, b = _paired_fixture()
    first = paired_bootstrap(a, b, seed=1)
    second = paired_bootstrap(a, b, seed=2)
    assert first.median_difference == pytest.approx(second.median_difference)
    assert first.wins == second.wins


def test_win_loss_tie_counts_are_counted_by_hand() -> None:
    a, b = _paired_fixture()
    result = paired_bootstrap(a, b, seed=7, lower_is_better=True)
    assert (result.wins, result.losses, result.ties) == (8, 1, 1)
    assert result.wins + result.losses + result.ties == result.n_pairs == 10
    assert result.median_difference < 0.0  # lower is better, so a is ahead


def test_higher_is_better_flips_wins_and_losses() -> None:
    a, b = _paired_fixture()
    result = paired_bootstrap(a, b, seed=7, lower_is_better=False)
    assert (result.wins, result.losses, result.ties) == (1, 8, 1)


def test_an_indistinguishable_pair_is_reported_as_not_significant() -> None:
    # Differences straddle zero, so the interval covers it and the summary must say so
    # out loud rather than quoting a bare median (ACCEPTANCE H7).
    a = [10.0, 11.0, 9.0, 12.0, 10.0, 11.0, 9.0, 12.0]
    b = [11.0, 10.0, 10.0, 11.0, 9.0, 12.0, 10.0, 11.0]
    result = paired_bootstrap(
        a, b, seed=3, metric="makespan_s", strategy_a="ssi", strategy_b="greedy"
    )
    assert not result.significant
    assert result.ci_low <= 0.0 <= result.ci_high
    summary = result.summary()
    assert "not significant" in summary
    assert "95% CI" in summary
    assert "wins" in summary
    assert "n=8 paired seeds" in summary


def test_a_clear_win_is_reported_as_significant() -> None:
    a = [1.0, 1.1, 0.9, 1.0, 1.2, 0.8, 1.0, 1.1]
    b = [9.0, 9.1, 8.9, 9.0, 9.2, 8.8, 9.0, 9.1]
    result = paired_bootstrap(a, b, seed=11)
    assert result.significant
    assert result.ci_high < 0.0
    assert "not significant" not in result.summary()


@pytest.mark.parametrize(
    ("a", "b", "kwargs", "match"),
    [
        ([1.0, 2.0], [1.0], {}, "equal length"),
        ([1.0], [2.0], {}, "at least 2 pairs"),
        ([1.0, 2.0], [1.0, 2.0], {"resamples": 0}, "resamples must be positive"),
        ([1.0, 2.0], [1.0, 2.0], {"confidence": 1.0}, "confidence"),
        ([1.0, math.nan], [1.0, 2.0], {}, "finite"),
    ],
)
def test_bootstrap_rejects_inputs_it_cannot_honestly_summarise(
    a: list[float], b: list[float], kwargs: dict[str, float], match: str
) -> None:
    with pytest.raises(ValueError, match=match):
        paired_bootstrap(a, b, seed=1, **kwargs)  # type: ignore[arg-type]

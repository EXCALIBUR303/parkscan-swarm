"""Unit tests for the pure metric functions.

Every case here is hand-written data, not a simulation. That is the point: if a metric
needs a running simulator to be checked, the metric is not a function.
"""

from __future__ import annotations

import math

import pytest
from parkscan_eval.metrics import (
    DEFAULT_RANGE_BIN_EDGES,
    ContractOutcome,
    ErrorSummary,
    GeoPair,
    RejectionRecord,
    detector_contract_summary,
    geolocation_error_stats,
    horizon_rejection_rate,
    percentile,
    pipeline_latency_stats,
)


def _pair(range_m: float, error_m: float) -> GeoPair:
    """A pair whose truth-to-estimate distance is exactly ``error_m``, along +x."""
    return GeoPair(truth=(0.0, 0.0), estimate=(error_m, 0.0), range_m=range_m)


# --------------------------------------------------------------------- percentile --
@pytest.mark.parametrize(
    ("values", "q", "expected"),
    [
        ([5.0], 0.0, 5.0),
        ([5.0], 1.0, 5.0),
        ([1.0, 2.0, 3.0, 4.0], 0.5, 2.5),
        ([1.0, 2.0, 3.0, 4.0, 5.0], 0.5, 3.0),
        # R type 7: position = q * (n - 1) = 0.95 * 3 = 2.85, between 3.0 and 4.0.
        ([1.0, 2.0, 3.0, 4.0], 0.95, 3.85),
        ([4.0, 1.0, 3.0, 2.0], 0.0, 1.0),
        ([4.0, 1.0, 3.0, 2.0], 1.0, 4.0),
    ],
)
def test_percentile_matches_linear_interpolation(
    values: list[float], q: float, expected: float
) -> None:
    assert percentile(values, q) == pytest.approx(expected)


def test_percentile_does_not_care_about_input_order() -> None:
    forward = [float(i) for i in range(20)]
    assert percentile(forward, 0.95) == percentile(list(reversed(forward)), 0.95)


def test_percentile_rejects_empty_and_out_of_range_q() -> None:
    with pytest.raises(ValueError, match="empty sequence"):
        percentile([], 0.5)
    with pytest.raises(ValueError, match=r"q must lie in"):
        percentile([1.0], 1.5)


# ------------------------------------------------------------------ ErrorSummary --
def test_error_summary_of_empty_is_a_zero_count_nan_summary() -> None:
    summary = ErrorSummary.of([])
    assert summary.count == 0
    assert math.isnan(summary.mean_m)
    assert math.isnan(summary.median_m)
    assert math.isnan(summary.p95_m)


def test_error_summary_statistics() -> None:
    summary = ErrorSummary.of([1.0, 2.0, 3.0, 4.0])
    assert summary.count == 4
    assert summary.mean_m == pytest.approx(2.5)
    assert summary.median_m == pytest.approx(2.5)
    assert summary.p95_m == pytest.approx(3.85)


# ------------------------------------------------------------- geolocation error --
def test_geo_pair_error_is_euclidean() -> None:
    pair = GeoPair(truth=(1.0, 2.0), estimate=(4.0, 6.0), range_m=5.0)
    assert pair.error_m == pytest.approx(5.0)


@pytest.mark.parametrize(
    ("truth", "estimate", "range_m"),
    [
        ((0.0, 0.0), (0.0, 0.0), -1.0),
        ((0.0, 0.0), (0.0, 0.0), math.inf),
        ((math.nan, 0.0), (0.0, 0.0), 1.0),
        ((0.0, 0.0), (0.0, math.inf), 1.0),
    ],
)
def test_geo_pair_rejects_nonsense(
    truth: tuple[float, float], estimate: tuple[float, float], range_m: float
) -> None:
    with pytest.raises(ValueError):
        GeoPair(truth=truth, estimate=estimate, range_m=range_m)


def test_geolocation_error_stats_bins_by_range_and_counts_each_bin() -> None:
    pairs = [
        _pair(0.5, 0.01),
        _pair(1.5, 0.03),
        _pair(2.5, 0.10),
        _pair(5.0, 0.40),
        _pair(5.5, 0.60),
        _pair(9.0, 2.00),
    ]
    stats = geolocation_error_stats(pairs)

    assert stats.overall.count == 6
    assert [b.summary.count for b in stats.bins] == [2, 1, 2, 0, 1]
    assert [b.label for b in stats.bins] == ["0-2 m", "2-4 m", "4-6 m", "6-8 m", "8+ m"]
    # The empty 6-8 m bin is reported, not omitted.
    assert math.isnan(stats.bins[3].summary.mean_m)
    assert stats.bins[2].summary.mean_m == pytest.approx(0.5)


def test_geolocation_error_stats_exposes_growth_with_range() -> None:
    """The headline finding must be visible in the binned output, not just the overall."""
    near = [_pair(1.0, 0.05), _pair(1.5, 0.06)]
    far = [_pair(9.0, 3.10), _pair(11.0, 4.40)]
    stats = geolocation_error_stats([*near, *far])

    assert stats.bins[0].summary.mean_m < stats.bins[-1].summary.mean_m
    # ... and the single overall mean sits between them, describing neither.
    assert stats.bins[0].summary.mean_m < stats.overall.mean_m < stats.bins[-1].summary.mean_m


def test_geolocation_error_stats_rejects_empty_input() -> None:
    with pytest.raises(ValueError, match="at least one pair"):
        geolocation_error_stats([])


def test_geolocation_error_stats_refuses_to_silently_drop_out_of_range_samples() -> None:
    with pytest.raises(ValueError, match="falls outside the bin edges"):
        geolocation_error_stats([_pair(9.0, 1.0)], bin_edges=(0.0, 2.0, 6.0))


@pytest.mark.parametrize(
    "edges",
    [
        (1.0,),
        (2.0, 1.0),
        (0.0, 2.0, 2.0),
        (-1.0, 2.0),
        (math.inf, 2.0),
        (0.0, math.nan, 3.0),
    ],
)
def test_geolocation_error_stats_validates_bin_edges(edges: tuple[float, ...]) -> None:
    with pytest.raises(ValueError):
        geolocation_error_stats([_pair(1.0, 0.1)], bin_edges=edges)


def test_default_bin_edges_straddle_the_documented_six_metre_gate() -> None:
    assert 6.0 in DEFAULT_RANGE_BIN_EDGES
    assert DEFAULT_RANGE_BIN_EDGES[-1] == math.inf


# ------------------------------------------------------------- horizon rejection --
def test_horizon_rejection_rate_breaks_out_reasons() -> None:
    results = [
        RejectionRecord(valid=True, reason="accepted"),
        RejectionRecord(valid=True, reason="accepted"),
        RejectionRecord(valid=False, reason="above_horizon"),
        RejectionRecord(valid=False, reason="above_horizon"),
        RejectionRecord(valid=False, reason="range_gate"),
    ]
    stats = horizon_rejection_rate(results)

    assert stats.total == 5
    assert stats.rejected == 3
    assert stats.rejection_rate == pytest.approx(0.6)
    assert stats.reasons == (("above_horizon", 2), ("accepted", 2), ("range_gate", 1))


def test_horizon_rejection_reason_order_is_deterministic_not_insertion_order() -> None:
    a = [
        RejectionRecord(valid=False, reason="range_gate"),
        RejectionRecord(valid=False, reason="above_horizon"),
        RejectionRecord(valid=False, reason="above_horizon"),
    ]
    b = [
        RejectionRecord(valid=False, reason="above_horizon"),
        RejectionRecord(valid=False, reason="above_horizon"),
        RejectionRecord(valid=False, reason="range_gate"),
    ]
    assert horizon_rejection_rate(a).reasons == horizon_rejection_rate(b).reasons


def test_horizon_rejection_rate_of_nothing_is_zero_with_a_visible_total() -> None:
    stats = horizon_rejection_rate([])
    assert stats.total == 0
    assert stats.rejected == 0
    assert stats.rejection_rate == 0.0
    assert stats.reasons == ()


@pytest.mark.parametrize(
    ("valid", "reason"),
    [(True, "above_horizon"), (False, "accepted"), (True, "")],
)
def test_rejection_record_keeps_valid_and_reason_consistent(valid: bool, reason: str) -> None:
    with pytest.raises(ValueError):
        RejectionRecord(valid=valid, reason=reason)


# ------------------------------------------------------------------ latency stats --
def test_pipeline_latency_stats_converts_seconds_to_milliseconds() -> None:
    stats = pipeline_latency_stats([0.001, 0.002, 0.003, 0.004])
    assert stats.count == 4
    assert stats.p50_ms == pytest.approx(2.5)
    assert stats.p95_ms == pytest.approx(3.85)


def test_pipeline_latency_stats_of_nothing_is_empty_not_zero() -> None:
    stats = pipeline_latency_stats([])
    assert stats.count == 0
    assert math.isnan(stats.p50_ms)
    assert math.isnan(stats.p95_ms)


@pytest.mark.parametrize("bad", [-0.001, math.inf, math.nan])
def test_pipeline_latency_stats_rejects_impossible_durations(bad: float) -> None:
    with pytest.raises(ValueError, match="finite and non-negative"):
        pipeline_latency_stats([0.001, bad])


# --------------------------------------------------------------- contract summary --
def test_detector_contract_summary_counts_and_keeps_failure_detail() -> None:
    summary = detector_contract_summary(
        [
            ContractOutcome(detector_name="SimulatedDetector", passed=True),
            ContractOutcome(
                detector_name="BrokenDetector",
                passed=False,
                detail="raising the threshold added a detection",
            ),
        ]
    )
    assert summary.total == 2
    assert summary.passed == 1
    assert summary.failed == 1
    assert summary.failures == (("BrokenDetector", "raising the threshold added a detection"),)


def test_detector_contract_summary_of_nothing_is_all_zero() -> None:
    summary = detector_contract_summary([])
    assert (summary.total, summary.passed, summary.failed, summary.failures) == (0, 0, 0, ())


def test_contract_outcome_rejects_a_passing_result_that_carries_a_complaint() -> None:
    with pytest.raises(ValueError, match="passing outcome carries no detail"):
        ContractOutcome(detector_name="X", passed=True, detail="but actually it failed")

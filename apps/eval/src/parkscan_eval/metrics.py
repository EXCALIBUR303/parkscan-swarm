"""Pure metric functions for the evaluation harness.

Nothing in this module reads a file, a clock, an environment variable or a random
number. Every function is a deterministic transformation of its arguments, which is what
makes ``results.json`` reproducible and what makes these numbers testable without running
a simulation at all.

What these metrics are, and are not
-----------------------------------
Every function here is **Tier 1** in the sense of ``apps/eval/EVAL.md``: it summarises the
behaviour of *our pipeline* against *known simulated ground truth*. None of it is a
statement about detection accuracy in a real car park, and no name in this module may be
re-used to imply one (``docs/ACCEPTANCE.md`` H3, H4). There is deliberately no ``map50``,
no ``precision`` and no ``recall`` here: those require a real model on a named public test
split, and inventing them from a simulated detector would be the exact overclaim the
acceptance criteria forbid.

The input records
-----------------
:class:`GeoPair`, :class:`RejectionRecord` and :class:`ContractOutcome` are defined here
rather than imported from ``parkscan_perception``. That is on purpose: the metric layer
must be unit-testable with three lines of hand-written data and no detector, no camera and
no simulator. ``parkscan_eval.run_eval`` is the single place that adapts perception's
result objects into these records.

Percentile definition
---------------------
``median`` and ``p95`` use linear interpolation between the two nearest order statistics
(the ``numpy`` default, R type 7, NIST "linear"): for ``n`` sorted samples the ``q``
quantile sits at position ``q * (n - 1)``. It is spelled out here because "p95" alone is
ambiguous across libraries by up to a whole sample at small ``n``, and these tables are
small.
"""

from __future__ import annotations

import itertools
import math
from collections import Counter
from collections.abc import Sequence
from dataclasses import dataclass

__all__ = [
    "DEFAULT_RANGE_BIN_EDGES",
    "ContractOutcome",
    "ContractSummary",
    "ErrorSummary",
    "GeoPair",
    "GeolocationErrorStats",
    "LatencyStats",
    "RangeBin",
    "RejectionRecord",
    "RejectionStats",
    "detector_contract_summary",
    "geolocation_error_stats",
    "horizon_rejection_rate",
    "percentile",
    "pipeline_latency_stats",
]

DEFAULT_RANGE_BIN_EDGES: tuple[float, ...] = (0.0, 2.0, 4.0, 6.0, 8.0, math.inf)
"""Range bins in metres, in the units the error budget is argued in.

The 6 m edge is where the documented valid range sits (``docs/ARCHITECTURE.md`` section 5),
so the bins straddle it: two bins below it, one containing it, and two above so that the
growth *past* the gate is visible in the same table that justifies the gate. Bins are
half-open ``[lo, hi)`` and the last edge is ``inf`` so nothing can silently fall off the
top of a headline table.
"""

_ACCEPTED = "accepted"
"""Canonical reason string for a geolocation that was not rejected."""


# ------------------------------------------------------------------ input records --
@dataclass(frozen=True, slots=True)
class GeoPair:
    """One geolocated detection paired with the ground truth it corresponds to.

    ``truth`` and ``estimate`` are ``(x_m, y_m)`` in the lot's metric frame. ``range_m`` is
    the range this pair is **binned at**, and the choice matters enough to state:

    ``parkscan_eval.run_eval`` supplies the ground range from the camera to the *known
    truth*, so the table answers "how far off is the answer, given how far away the thing
    actually was". Binning by the *estimated* range instead contaminates the far bins in
    exactly the direction that flatters the result -- a sample whose true range is 18 m and
    whose estimate lands at 40 m leaves the 16-20 m bin and joins the 20+ one, so every bin
    below reports the errors that happened to stay small. The estimated range is still
    recorded per detection in ``results.csv`` and is what the runtime range gate acts on;
    it is just not the axis an error-versus-range characterisation belongs on.
    """

    truth: tuple[float, float]
    estimate: tuple[float, float]
    range_m: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.range_m) or self.range_m < 0.0:
            msg = f"range_m must be finite and non-negative, got {self.range_m}"
            raise ValueError(msg)
        for name, point in (("truth", self.truth), ("estimate", self.estimate)):
            if not all(math.isfinite(v) for v in point):
                msg = f"{name} must be finite, got {point!r}"
                raise ValueError(msg)

    @property
    def error_m(self) -> float:
        """Euclidean distance between the back-projected point and the known truth."""
        return math.dist(self.truth, self.estimate)


@dataclass(frozen=True, slots=True)
class RejectionRecord:
    """The outcome of one geolocation attempt, reduced to what the metric needs.

    ``reason`` is a short stable slug (``"above_horizon"``, ``"range_gate"``, ...). It is
    required even when ``valid`` is true, where it must be ``"accepted"`` -- an optional
    reason field invites ``None`` into the breakdown table, and a breakdown with a ``None``
    row is a breakdown nobody can read.
    """

    valid: bool
    reason: str

    def __post_init__(self) -> None:
        if not self.reason:
            msg = "reason must be a non-empty slug"
            raise ValueError(msg)
        if self.valid and self.reason != _ACCEPTED:
            msg = f"a valid result must carry reason {_ACCEPTED!r}, got {self.reason!r}"
            raise ValueError(msg)
        if not self.valid and self.reason == _ACCEPTED:
            msg = "a rejected result must carry a reason other than 'accepted'"
            raise ValueError(msg)


@dataclass(frozen=True, slots=True)
class ContractOutcome:
    """Whether one detector passed ``assert_detector_contract``.

    ``detail`` is the assertion message when ``passed`` is false and ``""`` when it is
    true. It is carried into the summary so a failure is reported with its cause rather
    than as a bare count.
    """

    detector_name: str
    passed: bool
    detail: str = ""

    def __post_init__(self) -> None:
        if not self.detector_name:
            msg = "detector_name must be non-empty"
            raise ValueError(msg)
        if self.passed and self.detail:
            msg = f"a passing outcome carries no detail, got {self.detail!r}"
            raise ValueError(msg)


# ------------------------------------------------------------------ output records --
@dataclass(frozen=True, slots=True)
class ErrorSummary:
    """Mean / median / p95 of a set of errors in metres, plus how many there were.

    An empty set yields ``count == 0`` and ``nan`` statistics rather than raising: an
    empty *bin* is a legitimate observation about where the samples fell, and the count is
    reported next to the statistic precisely so that a ``nan`` (or a mean over two
    samples) cannot be mistaken for a result.
    """

    count: int
    mean_m: float
    median_m: float
    p95_m: float

    @classmethod
    def of(cls, errors: Sequence[float]) -> ErrorSummary:
        """Summarise ``errors``; an empty sequence gives the empty summary."""
        if not errors:
            return cls(count=0, mean_m=math.nan, median_m=math.nan, p95_m=math.nan)
        ordered = sorted(errors)
        return cls(
            count=len(ordered),
            mean_m=math.fsum(ordered) / len(ordered),
            median_m=percentile(ordered, 0.50),
            p95_m=percentile(ordered, 0.95),
        )


@dataclass(frozen=True, slots=True)
class RangeBin:
    """One half-open range bin ``[lo_m, hi_m)`` and the errors that fell into it."""

    lo_m: float
    hi_m: float
    summary: ErrorSummary

    @property
    def label(self) -> str:
        """Human-readable bin label, e.g. ``"2-4 m"`` or ``"8+ m"``."""
        if math.isinf(self.hi_m):
            return f"{self.lo_m:g}+ m"
        return f"{self.lo_m:g}-{self.hi_m:g} m"


@dataclass(frozen=True, slots=True)
class GeolocationErrorStats:
    """Geolocation error overall **and** binned by range.

    Both are reported because only the binned form is honest. Back-projection error grows
    roughly with the square of range, so a single mean is a statement about where the
    targets happened to be as much as about the pipeline: move the targets closer and the
    same code reports a better number. The bins are the finding; the overall figure is
    context.
    """

    overall: ErrorSummary
    bins: tuple[RangeBin, ...]


@dataclass(frozen=True, slots=True)
class RejectionStats:
    """How many geolocation attempts were rejected, and why."""

    total: int
    rejected: int
    rejection_rate: float
    reasons: tuple[tuple[str, int], ...]
    """``(reason, count)`` for every reason seen, including ``"accepted"``. Ordered by
    descending count then by reason, so the ordering never depends on dict insertion
    order and the JSON diffs cleanly between runs."""


@dataclass(frozen=True, slots=True)
class LatencyStats:
    """p50 / p95 over durations the caller measured. This module measures nothing."""

    count: int
    p50_ms: float
    p95_ms: float


@dataclass(frozen=True, slots=True)
class ContractSummary:
    """Pass/fail counts for ``assert_detector_contract`` across detectors."""

    total: int
    passed: int
    failed: int
    failures: tuple[tuple[str, str], ...]
    """``(detector_name, detail)`` for each failure, in the order the outcomes arrived."""


# ----------------------------------------------------------------------- functions --
def percentile(values: Sequence[float], q: float) -> float:
    """The ``q`` quantile of ``values`` by linear interpolation (R type 7).

    ``values`` need not be sorted. ``q`` is a fraction in ``[0, 1]``. Raises on an empty
    sequence: there is no defensible median of nothing, and returning ``nan`` here would
    push the decision to whichever call site forgot to check.
    """
    if not values:
        msg = "percentile of an empty sequence is undefined"
        raise ValueError(msg)
    if not 0.0 <= q <= 1.0:
        msg = f"q must lie in [0, 1], got {q}"
        raise ValueError(msg)
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = q * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def geolocation_error_stats(
    pairs: Sequence[GeoPair],
    *,
    bin_edges: Sequence[float] = DEFAULT_RANGE_BIN_EDGES,
) -> GeolocationErrorStats:
    """Euclidean geolocation error, overall and binned by range.

    Raises ``ValueError`` on an empty ``pairs``. A harness that geolocated nothing has a
    bug, and a summary of nothing is a ``nan`` waiting to be pasted into a README as a
    number. Empty *bins*, by contrast, are reported with ``count == 0``.

    Bins are half-open ``[lo, hi)``. A pair whose range falls outside ``bin_edges``
    raises rather than being dropped -- silently discarding the far samples would flatter
    exactly the trend this table exists to expose.
    """
    if not pairs:
        msg = "geolocation_error_stats needs at least one pair"
        raise ValueError(msg)
    edges = _validate_edges(bin_edges)

    buckets: list[list[float]] = [[] for _ in range(len(edges) - 1)]
    for pair in pairs:
        index = _bin_index(pair.range_m, edges)
        if index is None:
            msg = (
                f"range {pair.range_m} m falls outside the bin edges "
                f"[{edges[0]}, {edges[-1]}); widen bin_edges rather than dropping samples"
            )
            raise ValueError(msg)
        buckets[index].append(pair.error_m)

    return GeolocationErrorStats(
        overall=ErrorSummary.of([pair.error_m for pair in pairs]),
        bins=tuple(
            RangeBin(lo_m=edges[i], hi_m=edges[i + 1], summary=ErrorSummary.of(buckets[i]))
            for i in range(len(buckets))
        ),
    )


def horizon_rejection_rate(results: Sequence[RejectionRecord]) -> RejectionStats:
    """Fraction of geolocation attempts that were rejected, with the reasons broken out.

    The single rate is not the interesting part. "12% rejected" reads as a defect; "12%
    rejected, of which 9% ``above_horizon`` and 3% ``range_gate``" says the pipeline is
    refusing to invent points for rays that never meet the ground, which is the designed
    behaviour (``docs/ACCEPTANCE.md`` C11). Both are returned so a caller cannot report
    the first without the second being available.

    An empty ``results`` gives ``total == 0`` and ``rejection_rate == 0.0``: no attempt
    was made, so no attempt was rejected. That is a statement about the run, and the
    caller has ``total`` to see it.
    """
    counts = Counter(record.reason for record in results)
    total = len(results)
    rejected = sum(1 for record in results if not record.valid)
    return RejectionStats(
        total=total,
        rejected=rejected,
        rejection_rate=(rejected / total) if total else 0.0,
        reasons=tuple(sorted(counts.items(), key=lambda item: (-item[1], item[0]))),
    )


def pipeline_latency_stats(samples: Sequence[float]) -> LatencyStats:
    """p50 / p95 over ``samples``, which are **durations in seconds** the caller measured.

    This function does not time anything. It cannot: a metric module that reached for a
    clock would make ``results.json`` depend on how loaded the machine was, and the
    harness would stop being reproducible. The harness measures, this summarises.

    The reported figures are the wall-clock cost of *our* per-frame pipeline on whatever
    machine produced them. They are not a throughput claim, and they say nothing about a
    real detector: ``SimulatedDetector`` does no inference.
    """
    for sample in samples:
        if not math.isfinite(sample) or sample < 0.0:
            msg = f"durations must be finite and non-negative seconds, got {sample}"
            raise ValueError(msg)
    if not samples:
        return LatencyStats(count=0, p50_ms=math.nan, p95_ms=math.nan)
    return LatencyStats(
        count=len(samples),
        p50_ms=percentile(samples, 0.50) * 1000.0,
        p95_ms=percentile(samples, 0.95) * 1000.0,
    )


def detector_contract_summary(results: Sequence[ContractOutcome]) -> ContractSummary:
    """Pass/fail counts for the shared detector contract (``docs/ACCEPTANCE.md`` C12).

    Failures are carried through with their assertion text, because "2 of 3 passed" is not
    an actionable result and a contract failure is a build failure, not a chart.
    """
    passed = sum(1 for outcome in results if outcome.passed)
    return ContractSummary(
        total=len(results),
        passed=passed,
        failed=len(results) - passed,
        failures=tuple(
            (outcome.detector_name, outcome.detail) for outcome in results if not outcome.passed
        ),
    )


# ------------------------------------------------------------------------ private --
def _validate_edges(bin_edges: Sequence[float]) -> tuple[float, ...]:
    edges = tuple(float(edge) for edge in bin_edges)
    if len(edges) < 2:
        msg = f"bin_edges needs at least two edges, got {edges!r}"
        raise ValueError(msg)
    if edges[0] < 0.0 or math.isinf(edges[0]):
        msg = f"the first bin edge must be finite and non-negative, got {edges[0]}"
        raise ValueError(msg)
    if any(math.isnan(edge) for edge in edges):
        msg = f"bin_edges must not contain nan: {edges!r}"
        raise ValueError(msg)
    if any(lo >= hi for lo, hi in itertools.pairwise(edges)):
        msg = f"bin_edges must be strictly increasing, got {edges!r}"
        raise ValueError(msg)
    return edges


def _bin_index(range_m: float, edges: tuple[float, ...]) -> int | None:
    """Index of the half-open bin containing ``range_m``, or ``None`` if it is outside."""
    for index in range(len(edges) - 1):
        if edges[index] <= range_m < edges[index + 1]:
            return index
    return None

"""The ten benchmark metrics, each with one exact definition, and the paired bootstrap.

Why definitions live here and not in the runner
-----------------------------------------------
``docs/DATA_MODEL.md`` fixes the ten metric *names* that ``make bench`` writes to
``metric_sample``. Names are cheap; the definitions are where a benchmark is honest or
isn't. So every metric here is a **pure function of explicit observation records** -- no
clock, no I/O, no global RNG, no dependence on dict ordering -- which means a reviewer can
read the definition, and a test can pin it against a hand-built fixture with an answer
worked out by hand. ``apps/eval`` collects the records; it does not get to decide what
they mean.

The two latencies are separate, and that is the point
-----------------------------------------------------
``reassignment_latency_s`` and ``detection_latency_s`` measure different things and are
**never summed** (ACCEPTANCE H6, ``docs/DATA_MODEL.md``):

* :func:`detection_latency_s` is ``k x heartbeat_period``. It is a **configuration value**
  we chose, not a result we achieved. Halving the heartbeat period halves it, and buys
  nothing but false positives.
* :func:`reassignment_latency_s` measures the coordinator's own work: the interval from the
  instant its judgement of a robot flipped to the instant a replacement accepted the
  requeued zone. That number is affected by the allocator, the cooldown and the hysteresis
  delta, so it is a real result.

Adding them yields a plausible-sounding "recovered in 3.2 s" headline that is mostly the
heartbeat constant we picked, and that conflation is the standard overclaim in this
space. :class:`MetricsReport` therefore keeps them in separate fields, and
:meth:`MetricsReport.samples` emits them as separate rows.

Reporting a difference
----------------------
:func:`paired_bootstrap` compares two strategies over the *same* seeds. It returns a
median difference, a percentile 95% CI, win/loss/tie counts and an explicit
:attr:`PairedComparison.significant` flag, and :meth:`PairedComparison.summary` writes the
sentence ACCEPTANCE H7 demands -- including the words "not significant" when the interval
covers zero. It is seeded through ``numpy.random.default_rng`` and touches no global RNG,
so the same inputs and seed give the same interval on every machine.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from itertools import pairwise
from typing import Final

import numpy as np

from parkscan_coord.health import NS_PER_S, HealthConfig
from parkscan_core.types import RobotId, ZoneState

__all__ = [
    "METRIC_NAMES",
    "BatterySample",
    "CellVisit",
    "DockEvent",
    "MetricSample",
    "MetricsReport",
    "Occupancy",
    "PairedComparison",
    "ReassignmentEvent",
    "RunObservations",
    "TrackPoint",
    "ZoneCompletion",
    "ZoneOutcome",
    "build_report",
    "conflict_count",
    "coverage_pct",
    "depletion_events",
    "detection_latency_s",
    "duplicate_visit_rate",
    "makespan_s",
    "min_battery_at_dock",
    "paired_bootstrap",
    "reassignment_churn",
    "reassignment_latency_s",
    "total_distance_m",
]

METRIC_NAMES: Final[tuple[str, ...]] = (
    "coverage_pct",
    "makespan_s",
    "total_distance_m",
    "duplicate_visit_rate",
    "reassignment_latency_s",
    "detection_latency_s",
    "reassignment_churn",
    "min_battery_at_dock",
    "depletion_events",
    "conflict_count",
)
"""The fixed metric names from ``docs/DATA_MODEL.md``, in report order.

``reassignment_latency_s`` and ``detection_latency_s`` are adjacent in this tuple and
still separate rows. Nothing in this module ever adds them.
"""


# ------------------------------------------------------------------ observations --
@dataclass(frozen=True, slots=True)
class ZoneOutcome:
    """How one zone ended: how much of it was covered, and in what state it finished."""

    zone_id: str
    cells_total: int
    cells_done: int
    state: ZoneState

    def __post_init__(self) -> None:
        if self.cells_total < 0 or self.cells_done < 0:
            msg = f"zone {self.zone_id!r}: cell counts must be non-negative"
            raise ValueError(msg)
        if self.cells_done > self.cells_total:
            msg = (
                f"zone {self.zone_id!r}: cells_done={self.cells_done} exceeds "
                f"cells_total={self.cells_total}; only planned cells count toward coverage"
            )
            raise ValueError(msg)


@dataclass(frozen=True, slots=True)
class ZoneCompletion:
    """The instant one zone reached ``COMPLETE``. Makespan is built from these."""

    zone_id: str
    stamp_ns: int


@dataclass(frozen=True, slots=True)
class TrackPoint:
    """One robot's position at one instant. Distance is the polyline through these."""

    robot_id: RobotId
    stamp_ns: int
    x_m: float
    y_m: float


@dataclass(frozen=True, slots=True)
class CellVisit:
    """One robot covered one coverage cell. Repeats by the same robot are one visit."""

    robot_id: RobotId
    cell_id: str


@dataclass(frozen=True, slots=True)
class ReassignmentEvent:
    """One recovery: a zone taken from a failed robot and given to a replacement.

    ``detected_ns`` is the instant the **coordinator's judgement flipped** -- not the
    instant the robot actually failed. The interval between those two is the detection
    latency, which is a configured deadline (:func:`detection_latency_s`) rather than
    anything this event measures. The two are reported separately and never summed
    (ACCEPTANCE H6).
    """

    zone_id: str
    failed_robot: RobotId
    replacement_robot: RobotId
    detected_ns: int
    reassigned_ns: int
    cause: str

    def __post_init__(self) -> None:
        if self.reassigned_ns < self.detected_ns:
            msg = (
                f"zone {self.zone_id!r}: reassigned_ns={self.reassigned_ns} precedes "
                f"detected_ns={self.detected_ns}"
            )
            raise ValueError(msg)

    @property
    def latency_s(self) -> float:
        """Detection-to-reassignment, in seconds. Detection latency is *not* in here."""
        return (self.reassigned_ns - self.detected_ns) / NS_PER_S


@dataclass(frozen=True, slots=True)
class DockEvent:
    """A robot reached a charging bay, with the charge it had left when it got there."""

    robot_id: RobotId
    stamp_ns: int
    battery_soc: float

    def __post_init__(self) -> None:
        if not 0.0 <= self.battery_soc <= 1.0:
            msg = f"battery_soc must be in [0, 1], got {self.battery_soc}"
            raise ValueError(msg)


@dataclass(frozen=True, slots=True)
class BatterySample:
    """One state-of-charge reading. Depletion episodes are counted from these."""

    robot_id: RobotId
    stamp_ns: int
    soc: float


@dataclass(frozen=True, slots=True)
class Occupancy:
    """One robot held one single-occupancy resource over ``[start_ns, end_ns)``.

    Half-open on purpose: a robot leaving a lane cell at exactly the instant another
    enters it is a hand-off, not a collision, and closed intervals would score it as one.
    """

    resource_id: str
    robot_id: RobotId
    start_ns: int
    end_ns: int

    def __post_init__(self) -> None:
        if self.end_ns < self.start_ns:
            msg = (
                f"{self.resource_id!r}/{self.robot_id!r}: end_ns={self.end_ns} precedes "
                f"start_ns={self.start_ns}"
            )
            raise ValueError(msg)


# ----------------------------------------------------------------- the ten metrics --
def coverage_pct(zones: Iterable[ZoneOutcome]) -> float:
    """Percentage of **planned** cells that were covered, over the whole lot.

    Cell-weighted, not zone-weighted: a fleet that finishes eleven small zones and
    abandons one large one has not covered 91.7%. Cells the sensor footprint happened to
    touch outside the planned set do not count toward the numerator -- letting them would
    report more than 100% coverage of a zone, which is worse than useless
    (``ZoneOutcome`` rejects ``cells_done > cells_total`` at construction).

    Returns ``100.0`` for an empty lot: nothing was planned, so nothing was missed.
    """
    outcomes = list(zones)
    total = sum(outcome.cells_total for outcome in outcomes)
    if total == 0:
        return 100.0
    done = sum(outcome.cells_done for outcome in outcomes)
    return 100.0 * done / total


def makespan_s(start_ns: int, completions: Iterable[ZoneCompletion]) -> float:
    """Time from run start to the **last** zone completing, in seconds.

    ``C_max``, the standard makespan -- not the sum of completion times, and not the mean.
    Zones that never completed contribute nothing, so a run that abandons work looks
    *faster* by this metric alone; that is why makespan is never reported without
    ``coverage_pct`` beside it (ADR-0005 requires the pair).

    Returns ``0.0`` when nothing completed.
    """
    stamps = [completion.stamp_ns for completion in completions]
    if not stamps:
        return 0.0
    return max(0, max(stamps) - start_ns) / NS_PER_S


def total_distance_m(track: Iterable[TrackPoint]) -> float:
    """Summed polyline length of every robot's path, in metres.

    Points are grouped by robot and ordered by timestamp before measuring, so the caller
    may pass an interleaved fleet-wide stream in any order and get the same answer.
    """
    by_robot: dict[RobotId, list[TrackPoint]] = {}
    for point in track:
        by_robot.setdefault(point.robot_id, []).append(point)

    total = 0.0
    for robot_id in sorted(by_robot):
        points = sorted(by_robot[robot_id], key=lambda p: p.stamp_ns)
        for previous, current in pairwise(points):
            total += math.hypot(current.x_m - previous.x_m, current.y_m - previous.y_m)
    return total


def duplicate_visit_rate(visits: Iterable[CellVisit]) -> float:
    """Fraction of covered cells that **more than one robot** covered.

    A cell re-covered by the same robot on a later sweep pass is not duplicated work --
    that is one robot's own swath overlapping itself, which the sweep geometry guarantees.
    Duplication means two *different* robots drove the same ground, which is what
    acceptance A6 ("3 robots receive different zones, no intentional duplication") is
    about.

    Non-zero after a recovery is correct and expected, not a bug: a replacement robot
    re-sweeps the zone from its entry, so it re-covers whatever the failed robot had
    already done. Recovery costs duplicated work, and this metric is where that cost shows
    up instead of being hidden.

    Returns ``0.0`` when no cells were covered at all.
    """
    robots_by_cell: dict[str, set[RobotId]] = {}
    for visit in visits:
        robots_by_cell.setdefault(visit.cell_id, set()).add(visit.robot_id)
    if not robots_by_cell:
        return 0.0
    duplicated = sum(1 for robots in robots_by_cell.values() if len(robots) > 1)
    return duplicated / len(robots_by_cell)


def reassignment_latency_s(events: Iterable[ReassignmentEvent]) -> float:
    """Mean detection-to-reassignment interval over recovery events, in seconds.

    Measured from the instant the coordinator's judgement of a robot flipped to the
    instant a **different** robot accepted the requeued zone. It therefore prices the
    allocator, the per-zone cooldown and the hysteresis delta -- the parts we designed.

    It does **not** include the time taken to reach that judgement. That is
    :func:`detection_latency_s`, a configured deadline, and the two must never be summed
    (ACCEPTANCE H6).

    Returns ``nan`` when there were no reassignments, because "no events" is not "fast".
    :attr:`MetricsReport.reassignment_events_n` carries the count so a reader can tell the
    two apart.
    """
    latencies = [event.latency_s for event in events]
    if not latencies:
        return math.nan
    return sum(latencies) / len(latencies)


def detection_latency_s(config: HealthConfig) -> float:
    """``k x heartbeat_period`` -- **a configuration value, not an achievement.**

    This is the heartbeat deadline we chose (``missed_beats_offline`` beats of
    ``heartbeat_period_ns``), reported so that ``reassignment_latency_s`` can be read in
    context. It is not measured, it does not vary between strategies or seeds, and
    lowering it does not improve the system -- it only trades false positives for delay.

    Two things it is deliberately not:

    * It is **not** the whole recovery time. Never add it to
      :func:`reassignment_latency_s`; the sum is the standard overclaim this project
      forbids (ACCEPTANCE H6).
    * It is **not** the latency of the Last Will path. A hard disconnect is detected by the
      broker in whatever time the broker takes, which this number says nothing about. It
      bounds the *heartbeat* path -- the hung-robot case the will never fires for.
    """
    return config.detection_latency_s


def reassignment_churn(voluntary_reassignments: int, assignments_total: int) -> float:
    """Share of assignments that were **discretionary** re-allocations.

    Numerator: reassignments the coordinator chose to make because another robot priced
    the zone better -- the ones the four anti-thrash guards exist to suppress. Forced
    requeues (the holder died, went stuck, or broke off to charge) are excluded, because
    counting them would make correct recovery look like thrash and would push the
    honest reading of acceptance C3 in exactly the wrong direction.

    Denominator: every assignment the coordinator made, initial ones included.

    ``0.0`` in the nominal run is what proves hysteresis works (C3). Returns ``0.0`` when
    nothing was ever assigned.
    """
    if assignments_total <= 0:
        return 0.0
    return voluntary_reassignments / assignments_total


def min_battery_at_dock(docks: Iterable[DockEvent]) -> float:
    """Lowest state of charge any robot had on reaching a bay, in ``[0, 1]``.

    The margin the battery policy actually left, as opposed to the margin it intended.
    Reported next to ``depletion_events``: a run with zero depletions and a
    ``min_battery_at_dock`` of 0.01 got lucky rather than good.

    Returns ``nan`` when nothing docked -- ``1.0`` would read as "everything arrived
    full", which is the opposite of what no data means.
    """
    values = [dock.battery_soc for dock in docks]
    if not values:
        return math.nan
    return min(values)


def depletion_events(samples: Iterable[BatterySample], *, depleted_at_soc: float = 0.0) -> int:
    """Number of episodes in which a robot's charge reached ``depleted_at_soc`` or below.

    Consecutive depleted readings from one robot are one episode, not one per sample: a
    robot sitting flat for 200 ticks failed once. A robot that recharges and depletes
    again counts twice.

    Acceptance C2 requires this to be ``0``. It is a build failure, not a chart.
    """
    by_robot: dict[RobotId, list[BatterySample]] = {}
    for sample in samples:
        by_robot.setdefault(sample.robot_id, []).append(sample)

    episodes = 0
    for robot_id in sorted(by_robot):
        depleted = False
        for sample in sorted(by_robot[robot_id], key=lambda s: s.stamp_ns):
            if sample.soc <= depleted_at_soc:
                if not depleted:
                    episodes += 1
                depleted = True
            else:
                depleted = False
    return episodes


def conflict_count(occupancies: Iterable[Occupancy]) -> int:
    """Number of overlapping holder pairs on single-occupancy resources.

    Two robots holding the same resource over intervals that intersect is one conflict per
    pair. Intervals are half-open, so a clean hand-off at a shared instant is not counted.
    Acceptance C1 requires ``0``: never two robots in one single-occupancy lane cell,
    never two holders of one bay lease.

    Counting *pairs* rather than instants is deliberate. An instant-based count depends on
    the sampling rate, so it would report a bigger number for the same bug simply because
    the run logged more often.
    """
    by_resource: dict[str, list[Occupancy]] = {}
    for occupancy in occupancies:
        by_resource.setdefault(occupancy.resource_id, []).append(occupancy)

    conflicts = 0
    for resource_id in sorted(by_resource):
        holds = sorted(by_resource[resource_id], key=lambda o: (o.start_ns, o.end_ns, o.robot_id))
        for index, first in enumerate(holds):
            for second in holds[index + 1 :]:
                if second.start_ns >= first.end_ns:
                    break
                if second.robot_id != first.robot_id:
                    conflicts += 1
    return conflicts


# ---------------------------------------------------------------------- reporting --
@dataclass(frozen=True, slots=True)
class MetricSample:
    """One row of ``metric_sample``: exactly what ``make bench`` persists."""

    name: str
    value: float
    unit: str


@dataclass(frozen=True, slots=True)
class RunObservations:
    """Everything one run produced, in the form the metric functions consume.

    Assembled by ``apps/eval`` from the coordinator's records and the run's telemetry.
    Passing one bundle rather than ten arguments keeps :func:`build_report` from growing a
    positional-argument order that is easy to get wrong and impossible to see at a glance.
    """

    start_ns: int
    health: HealthConfig
    zones: tuple[ZoneOutcome, ...] = ()
    completions: tuple[ZoneCompletion, ...] = ()
    track: tuple[TrackPoint, ...] = ()
    visits: tuple[CellVisit, ...] = ()
    reassignments: tuple[ReassignmentEvent, ...] = ()
    docks: tuple[DockEvent, ...] = ()
    battery: tuple[BatterySample, ...] = ()
    occupancies: tuple[Occupancy, ...] = ()
    assignments_total: int = 0
    voluntary_reassignments: int = 0


@dataclass(frozen=True, slots=True)
class MetricsReport:
    """The ten metrics of one run, plus the counts needed to read them honestly.

    ``reassignment_latency_s`` and ``min_battery_at_dock`` are ``nan`` when their event
    set was empty. The companion ``*_n`` fields are what tell a reader that ``nan`` means
    "this did not happen" rather than "this failed to be measured".
    """

    coverage_pct: float
    makespan_s: float
    total_distance_m: float
    duplicate_visit_rate: float
    reassignment_latency_s: float
    detection_latency_s: float
    reassignment_churn: float
    min_battery_at_dock: float
    depletion_events: int
    conflict_count: int
    reassignment_events_n: int
    dock_events_n: int
    assignments_total: int
    voluntary_reassignments: int

    def samples(self) -> tuple[MetricSample, ...]:
        """The ten rows, in ``METRIC_NAMES`` order, with units.

        ``reassignment_latency_s`` and ``detection_latency_s`` are two separate rows and
        are never combined into a third.
        """
        units: dict[str, tuple[float, str]] = {
            "coverage_pct": (self.coverage_pct, "percent"),
            "makespan_s": (self.makespan_s, "s"),
            "total_distance_m": (self.total_distance_m, "m"),
            "duplicate_visit_rate": (self.duplicate_visit_rate, "ratio"),
            "reassignment_latency_s": (self.reassignment_latency_s, "s"),
            "detection_latency_s": (self.detection_latency_s, "s"),
            "reassignment_churn": (self.reassignment_churn, "ratio"),
            "min_battery_at_dock": (self.min_battery_at_dock, "soc"),
            "depletion_events": (float(self.depletion_events), "count"),
            "conflict_count": (float(self.conflict_count), "count"),
        }
        return tuple(
            MetricSample(name=name, value=units[name][0], unit=units[name][1])
            for name in METRIC_NAMES
        )


def build_report(observations: RunObservations) -> MetricsReport:
    """Compute all ten metrics from one run's observations. Pure."""
    return MetricsReport(
        coverage_pct=coverage_pct(observations.zones),
        makespan_s=makespan_s(observations.start_ns, observations.completions),
        total_distance_m=total_distance_m(observations.track),
        duplicate_visit_rate=duplicate_visit_rate(observations.visits),
        reassignment_latency_s=reassignment_latency_s(observations.reassignments),
        detection_latency_s=detection_latency_s(observations.health),
        reassignment_churn=reassignment_churn(
            observations.voluntary_reassignments, observations.assignments_total
        ),
        min_battery_at_dock=min_battery_at_dock(observations.docks),
        depletion_events=depletion_events(observations.battery),
        conflict_count=conflict_count(observations.occupancies),
        reassignment_events_n=len(observations.reassignments),
        dock_events_n=len(observations.docks),
        assignments_total=observations.assignments_total,
        voluntary_reassignments=observations.voluntary_reassignments,
    )


# ------------------------------------------------------------- paired comparison --
@dataclass(frozen=True, slots=True)
class PairedComparison:
    """A paired comparison of two strategies over the same seeds.

    Paired, because the seeds are shared: every seed produces one run of each strategy on
    the identical lot, fleet and failure schedule, so the difference on a seed removes the
    seed's own difficulty from the comparison. Comparing unpaired means across 30 seeds at
    a fleet size of three would mostly measure which seeds each strategy happened to get.
    """

    metric: str
    strategy_a: str
    strategy_b: str
    n_pairs: int
    median_difference: float
    """Median of ``a - b`` across seeds, in the metric's own unit."""
    ci_low: float
    ci_high: float
    confidence: float
    wins: int
    """Seeds on which ``a`` beat ``b``, under :attr:`lower_is_better`."""
    losses: int
    ties: int
    resamples: int
    seed: int
    lower_is_better: bool

    @property
    def significant(self) -> bool:
        """Whether the CI excludes zero.

        A percentile bootstrap interval, not a hypothesis test. "Significant" here means
        only "the interval does not cover zero at this confidence and this many seeds".
        """
        return not (self.ci_low <= 0.0 <= self.ci_high)

    def summary(self) -> str:
        """One sentence, in the form ACCEPTANCE H7 requires.

        Always carries the CI, the win/loss/tie counts and the fleet-size caveat, and says
        "not significant" out loud when the interval covers zero. There is no code path
        here that produces a bare number.
        """
        verdict = "significant" if self.significant else "not significant"
        direction = "lower is better" if self.lower_is_better else "higher is better"
        return (
            f"{self.metric}: {self.strategy_a} vs {self.strategy_b}, median difference "
            f"{self.median_difference:+.4g} (95% CI {self.ci_low:+.4g} to "
            f"{self.ci_high:+.4g}, {self.wins}/{self.n_pairs} wins, {self.losses} losses, "
            f"{self.ties} ties, {direction}), {verdict} at this fleet size "
            f"(n={self.n_pairs} paired seeds, {self.resamples} bootstrap resamples, "
            f"seed={self.seed})"
        )


def paired_bootstrap(
    a: Sequence[float],
    b: Sequence[float],
    *,
    seed: int,
    metric: str = "metric",
    strategy_a: str = "a",
    strategy_b: str = "b",
    resamples: int = 10_000,
    confidence: float = 0.95,
    lower_is_better: bool = True,
) -> PairedComparison:
    """Percentile bootstrap CI for the median paired difference ``a - b``.

    ``a[i]`` and ``b[i]`` must be the two strategies' results on the **same seed**; that
    pairing is the whole point, so the lengths must match.

    Reproducible by construction: the only randomness is a
    ``numpy.random.default_rng(seed)`` created here and used nowhere else. No global RNG
    is touched, so the interval does not depend on what else the process did first, and
    two calls with the same arguments return equal results.

    Raises:
        ValueError: mismatched lengths, fewer than two pairs, a non-positive resample
            count, or a confidence outside ``(0, 1)``.
    """
    if len(a) != len(b):
        msg = f"paired samples must have equal length, got {len(a)} and {len(b)}"
        raise ValueError(msg)
    if len(a) < 2:
        msg = f"a paired bootstrap needs at least 2 pairs, got {len(a)}"
        raise ValueError(msg)
    if resamples < 1:
        msg = f"resamples must be positive, got {resamples}"
        raise ValueError(msg)
    if not 0.0 < confidence < 1.0:
        msg = f"confidence must be in (0, 1), got {confidence}"
        raise ValueError(msg)

    first = np.asarray(a, dtype=np.float64)
    second = np.asarray(b, dtype=np.float64)
    if not bool(np.all(np.isfinite(first))) or not bool(np.all(np.isfinite(second))):
        msg = "paired bootstrap inputs must be finite; nan marks a metric that has no value"
        raise ValueError(msg)

    differences = first - second
    n = differences.size

    rng = np.random.default_rng(seed)
    indices = rng.integers(0, n, size=(resamples, n))
    medians = np.median(differences[indices], axis=1)

    tail = (1.0 - confidence) / 2.0
    low, high = np.percentile(medians, [100.0 * tail, 100.0 * (1.0 - tail)])

    better = differences < 0.0 if lower_is_better else differences > 0.0
    worse = differences > 0.0 if lower_is_better else differences < 0.0

    return PairedComparison(
        metric=metric,
        strategy_a=strategy_a,
        strategy_b=strategy_b,
        n_pairs=n,
        median_difference=float(np.median(differences)),
        ci_low=float(low),
        ci_high=float(high),
        confidence=confidence,
        wins=int(np.count_nonzero(better)),
        losses=int(np.count_nonzero(worse)),
        ties=int(np.count_nonzero(differences == 0.0)),
        resamples=resamples,
        seed=seed,
        lower_is_better=lower_is_better,
    )

"""The reusable detector contract -- acceptance criterion C12.

:func:`assert_detector_contract` is a **function, not a test**. That is deliberate. If the
contract lived inside a ``test_*`` function it could only ever be applied to the detectors
this package happens to know about; as a function it can be pointed at the simulated
detector here, at a future RF-DETR wrapper, at a ground-truth passthrough, and at anything
a reviewer writes in a scratch file, all with one import and one call. ``docs/ACCEPTANCE.md``
C12 is "``assert_detector_contract`` passes for **every** ``Detector`` implementation", and
that sentence is only enforceable if the contract is a thing you can hand a detector to.

It raises :class:`AssertionError` with a message naming the frame, the detection and the
observed value, and it does so through explicit ``raise`` statements rather than ``assert``
statements. ``assert`` is removed by ``python -O``; a contract that silently stops checking
under an optimisation flag is worse than no contract, because it still looks like one.

What is checked, and why each check earns its place:

``determinism``
    Two identical calls return identical detections, and a third call after
    :meth:`~parkscan_perception.detector.Detector.warmup` still returns them. This is the
    property that makes the seeded benchmark and offline re-scoring meaningful.
``declared classes``
    Every emitted ``kind`` is in ``det.classes``. A detector that emits an undeclared class
    breaks the dashboard and the schema at once.
``threshold respected``
    Every score is at least the threshold that was asked for.
``bbox inside the frame``
    ``0 <= x1 <= x2 <= width`` and ``0 <= y1 <= y2 <= height``. An out-of-frame box
    back-projects to a confident, wrong, plausible-looking world point.
``source matches``
    ``detection.det_source == det.source`` on every detection, so the SIMULATED badge
    cannot be defeated by a detector that mislabels its own output (H1).
``monotone in the threshold``
    Raising the threshold never *adds* detections. Optionally (default on) the stronger
    property that the higher-threshold result is a literal subset of the lower-threshold
    one, which is what lets an evaluation harness call ``detect`` once at a low threshold
    and derive every operating point above it without re-running the detector.

Also checked, because they cost nothing and catch real mistakes: ``detector_name`` matches
``det.name``; the detection's ``stamp_ns`` matches its frame's; and only potholes carry a
``severity`` (``docs/DATA_MODEL.md``: the column is potholes-only).
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from itertools import pairwise
from typing import Final, NoReturn

from parkscan_core.types import Detection, DetKind, Frame
from parkscan_perception.detector import DEFAULT_SCORE_THRESHOLD, Detector

__all__ = ["DEFAULT_CONTRACT_THRESHOLDS", "assert_detector_contract"]

DEFAULT_CONTRACT_THRESHOLDS: Final[tuple[float, ...]] = (0.05, DEFAULT_SCORE_THRESHOLD, 0.6, 0.95)
"""Ascending sweep. The lowest is the one every per-detection check runs at, and the span
has to be wide enough that a detector with a realistic score distribution actually loses
detections along the way -- otherwise the monotonicity check passes without testing."""


def _fail(msg: str) -> NoReturn:
    """Every clause fails through here, so the contract cannot be disabled by ``python -O``."""
    raise AssertionError(msg)


def _check_thresholds(thresholds: Sequence[float]) -> None:
    if len(thresholds) < 2:
        _fail(f"contract needs at least two thresholds to test monotonicity, got {thresholds!r}")
    for lo, hi in pairwise(thresholds):
        if lo >= hi:
            _fail(f"thresholds must be strictly ascending, got {thresholds!r}")
    for t in thresholds:
        if not 0.0 <= t <= 1.0:
            _fail(f"thresholds must lie in [0, 1], got {thresholds!r}")


def _check_detection(det: Detector, frame: Frame, d: Detection, threshold: float) -> None:
    where = f"{det.name} frame_id={frame.frame_id}"
    if d.kind not in det.classes:
        _fail(f"{where}: emitted undeclared class {d.kind!r}; declared {sorted(det.classes)}")
    if d.det_source is not det.source:
        _fail(f"{where}: det_source {d.det_source!r} does not match declared {det.source!r}")
    if d.detector_name != det.name:
        _fail(f"{where}: detector_name {d.detector_name!r} does not match name {det.name!r}")
    if d.stamp_ns != frame.stamp_ns:
        _fail(f"{where}: stamp_ns {d.stamp_ns} does not match frame stamp {frame.stamp_ns}")
    if d.confidence < threshold:
        _fail(f"{where}: confidence {d.confidence!r} is below the requested {threshold!r}")
    b = d.bbox
    if not (0.0 <= b.x1 <= b.x2 <= frame.width):
        _fail(f"{where}: bbox x span ({b.x1}, {b.x2}) outside [0, {frame.width}]")
    if not (0.0 <= b.y1 <= b.y2 <= frame.height):
        _fail(f"{where}: bbox y span ({b.y1}, {b.y2}) outside [0, {frame.height}]")
    if d.kind is not DetKind.POTHOLE and d.severity is not None:
        _fail(
            f"{where}: severity {d.severity!r} on a {d.kind!r} detection; severity is pothole-only"
        )


def assert_detector_contract(
    det: Detector,
    frames: Iterable[Frame],
    *,
    thresholds: Sequence[float] = DEFAULT_CONTRACT_THRESHOLDS,
    subset_monotonic: bool = True,
    require_nonempty: bool = True,
) -> None:
    """Assert that ``det`` honours the detector contract over ``frames``.

    Raises :class:`AssertionError` on the first violation, with a message identifying the
    detector, the frame and the offending value. Returns ``None`` on success.

    ``frames`` is consumed once and materialised, so a generator is fine.

    ``subset_monotonic`` additionally requires that the detections surviving a higher
    threshold are literally the same objects as at a lower one. Leave it on unless a
    detector genuinely cannot provide it -- an in-graph-NMS model that is *given* the score
    threshold may legitimately return a different, not merely smaller, set -- and say so at
    the call site, because switching it off weakens the guarantee the eval harness relies
    on.

    ``require_nonempty`` fails a run in which the detector produced nothing at all. Without
    it, a detector that always returns ``[]`` passes every other check vacuously, and the
    contract would certify a detector that does nothing.
    """
    frame_list = tuple(frames)
    if not frame_list:
        _fail("assert_detector_contract needs at least one frame")
    _check_thresholds(thresholds)

    if not det.name:
        _fail("detector name must be non-empty")
    if not det.classes:
        _fail(f"{det.name}: classes must declare at least one DetKind")

    base = thresholds[0]
    total = 0

    for frame in frame_list:
        first = tuple(det.detect(frame, score_threshold=base))
        second = tuple(det.detect(frame, score_threshold=base))
        if first != second:
            _fail(
                f"{det.name} frame_id={frame.frame_id}: detect() is not deterministic\n"
                f"  call 1: {first!r}\n  call 2: {second!r}"
            )

        # warmup() may load weights or compile kernels; it may not change the answer, and
        # it must survive being called again after the detector has already been used.
        det.warmup()
        det.warmup()
        after_warmup = tuple(det.detect(frame, score_threshold=base))
        if after_warmup != first:
            _fail(
                f"{det.name} frame_id={frame.frame_id}: warmup() changed detect() output\n"
                f"  before: {first!r}\n  after:  {after_warmup!r}"
            )

        for d in first:
            _check_detection(det, frame, d, base)
        total += len(first)

        previous = first
        for threshold in thresholds[1:]:
            current = tuple(det.detect(frame, score_threshold=threshold))
            for d in current:
                _check_detection(det, frame, d, threshold)
            if len(current) > len(previous):
                _fail(
                    f"{det.name} frame_id={frame.frame_id}: raising the threshold added "
                    f"detections ({len(previous)} at {base!r} .. {len(current)} at "
                    f"{threshold!r})"
                )
            if subset_monotonic and not set(current) <= set(previous):
                added = sorted(set(current) - set(previous), key=repr)
                _fail(
                    f"{det.name} frame_id={frame.frame_id}: threshold {threshold!r} produced "
                    f"detections absent at the lower threshold: {added!r}"
                )
            previous = current

    if require_nonempty and total == 0:
        _fail(
            f"{det.name}: produced no detections on any of the {len(frame_list)} frames at "
            f"threshold {base!r}. Every other check passes vacuously on an empty result, so "
            f"the contract refuses to certify this run; pass frames the detector can fire on, "
            f"or set require_nonempty=False and say why."
        )

"""The contract has to fail. These are the detectors that break each rule on purpose.

A contract nobody has watched reject anything is a contract nobody knows is wired up. Each
class below violates exactly one clause and is asserted to be caught by it, so the checks
in :func:`~parkscan_perception.contract.assert_detector_contract` are themselves tested
rather than trusted.
"""

from __future__ import annotations

import inspect
import itertools
from collections.abc import Callable, Sequence

import pytest
from parkscan_perception.contract import (
    DEFAULT_CONTRACT_THRESHOLDS,
    assert_detector_contract,
)
from parkscan_perception.detector import DEFAULT_SCORE_THRESHOLD, Detector
from parkscan_perception.simulated import DEFAULT_CLASSES

from parkscan_core.types import BBox, Detection, DetKind, DetSource, Frame, Severity

FrameFactory = Callable[[int], Frame]


def _detection(
    frame: Frame,
    confidence: float,
    *,
    name: str = "broken",
    kind: DetKind = DetKind.POTHOLE,
    source: DetSource = DetSource.SIMULATED,
    bbox: BBox | None = None,
    severity: Severity | None = Severity.LOW,
    stamp_ns: int | None = None,
) -> Detection:
    return Detection(
        kind=kind,
        det_source=source,
        detector_name=name,
        confidence=confidence,
        severity=severity,
        bbox=bbox if bbox is not None else BBox(10.0, 300.0, 60.0, 340.0),
        stamp_ns=frame.stamp_ns if stamp_ns is None else stamp_ns,
    )


class _Base:
    """A detector that passes the contract. Every class below is this, minus one rule."""

    name = "broken"
    source = DetSource.SIMULATED
    classes = DEFAULT_CLASSES

    def warmup(self) -> None:
        return None

    def detect(
        self, frame: Frame, *, score_threshold: float = DEFAULT_SCORE_THRESHOLD
    ) -> Sequence[Detection]:
        made = [_detection(frame, 0.9, name=self.name), _detection(frame, 0.4, name=self.name)]
        return [d for d in made if d.confidence >= score_threshold]


class Nondeterministic(_Base):
    def __init__(self) -> None:
        self._counter = itertools.count()

    def detect(
        self, frame: Frame, *, score_threshold: float = DEFAULT_SCORE_THRESHOLD
    ) -> Sequence[Detection]:
        drift = 0.9 - 0.01 * next(self._counter)
        return [_detection(frame, drift, name=self.name)]


class UndeclaredClass(_Base):
    classes = frozenset({DetKind.POTHOLE})

    def detect(
        self, frame: Frame, *, score_threshold: float = DEFAULT_SCORE_THRESHOLD
    ) -> Sequence[Detection]:
        return [_detection(frame, 0.9, name=self.name, kind=DetKind.SPACE_OCCUPANCY, severity=None)]


class MislabelledSource(_Base):
    source = DetSource.MODEL

    def detect(
        self, frame: Frame, *, score_threshold: float = DEFAULT_SCORE_THRESHOLD
    ) -> Sequence[Detection]:
        return [_detection(frame, 0.9, name=self.name, source=DetSource.SIMULATED)]


class WrongDetectorName(_Base):
    def detect(
        self, frame: Frame, *, score_threshold: float = DEFAULT_SCORE_THRESHOLD
    ) -> Sequence[Detection]:
        return [_detection(frame, 0.9, name="somebody-else")]


class WrongStamp(_Base):
    def detect(
        self, frame: Frame, *, score_threshold: float = DEFAULT_SCORE_THRESHOLD
    ) -> Sequence[Detection]:
        return [_detection(frame, 0.9, name=self.name, stamp_ns=frame.stamp_ns + 1)]


class BelowThreshold(_Base):
    def detect(
        self, frame: Frame, *, score_threshold: float = DEFAULT_SCORE_THRESHOLD
    ) -> Sequence[Detection]:
        return [_detection(frame, 0.01, name=self.name)]


class OutOfFrame(_Base):
    def detect(
        self, frame: Frame, *, score_threshold: float = DEFAULT_SCORE_THRESHOLD
    ) -> Sequence[Detection]:
        wide = BBox(10.0, 300.0, frame.width + 5.0, 340.0)
        return [_detection(frame, 0.9, name=self.name, bbox=wide)]


class TallerThanTheFrame(_Base):
    def detect(
        self, frame: Frame, *, score_threshold: float = DEFAULT_SCORE_THRESHOLD
    ) -> Sequence[Detection]:
        tall = BBox(10.0, 300.0, 60.0, frame.height + 2.0)
        return [_detection(frame, 0.9, name=self.name, bbox=tall)]


class SeverityOnOccupancy(_Base):
    def detect(
        self, frame: Frame, *, score_threshold: float = DEFAULT_SCORE_THRESHOLD
    ) -> Sequence[Detection]:
        return [
            _detection(
                frame,
                0.9,
                name=self.name,
                kind=DetKind.SPACE_OCCUPANCY,
                severity=Severity.HIGH,
            )
        ]


class ThresholdAddsDetections(_Base):
    """The failure mode the monotonicity clause exists for: a detector whose 'confident'
    branch produces *more* boxes, so a stricter operating point reports more damage."""

    def detect(
        self, frame: Frame, *, score_threshold: float = DEFAULT_SCORE_THRESHOLD
    ) -> Sequence[Detection]:
        count = 1 if score_threshold < 0.5 else 4
        return [_detection(frame, 0.99, name=self.name) for _ in range(count)]


class ThresholdSwapsDetections(_Base):
    """Same count at every threshold, different boxes. Only the subset clause sees it."""

    def detect(
        self, frame: Frame, *, score_threshold: float = DEFAULT_SCORE_THRESHOLD
    ) -> Sequence[Detection]:
        shift = 10.0 * score_threshold
        return [
            _detection(frame, 0.99, name=self.name, bbox=BBox(shift, 300.0, shift + 50.0, 340.0))
        ]


class WarmupChangesOutput(_Base):
    def __init__(self) -> None:
        self._warm = False

    def warmup(self) -> None:
        self._warm = True

    def detect(
        self, frame: Frame, *, score_threshold: float = DEFAULT_SCORE_THRESHOLD
    ) -> Sequence[Detection]:
        return [_detection(frame, 0.95 if self._warm else 0.9, name=self.name)]


class Silent(_Base):
    def detect(
        self, frame: Frame, *, score_threshold: float = DEFAULT_SCORE_THRESHOLD
    ) -> Sequence[Detection]:
        return []


class Nameless(_Base):
    name = ""


class Classless(_Base):
    classes = frozenset()


# ---------------------------------------------------------------------- the tests --
def test_the_baseline_detector_passes(frames: tuple[Frame, ...]) -> None:
    """Otherwise every failure below could be an artefact of the fixture, not the clause."""
    assert_detector_contract(_Base(), frames)


@pytest.mark.parametrize(
    ("detector", "expected"),
    [
        (Nondeterministic(), "not deterministic"),
        (UndeclaredClass(), "undeclared class"),
        (MislabelledSource(), "does not match declared"),
        (WrongDetectorName(), "does not match name"),
        (WrongStamp(), "does not match frame stamp"),
        (BelowThreshold(), "below the requested"),
        (OutOfFrame(), "bbox x span"),
        (TallerThanTheFrame(), "bbox y span"),
        (SeverityOnOccupancy(), "pothole-only"),
        (ThresholdAddsDetections(), "added"),
        (ThresholdSwapsDetections(), "absent at the lower threshold"),
        (WarmupChangesOutput(), "warmup"),
        (Silent(), "produced no detections"),
        (Nameless(), "name must be non-empty"),
        (Classless(), "at least one DetKind"),
    ],
)
def test_each_clause_rejects_its_own_violation(
    detector: Detector, expected: str, frames: tuple[Frame, ...]
) -> None:
    with pytest.raises(AssertionError, match=expected):
        assert_detector_contract(detector, frames)


def test_the_subset_clause_can_be_switched_off_but_the_count_clause_cannot(
    frames: tuple[Frame, ...],
) -> None:
    """``subset_monotonic=False`` is the documented escape hatch for an in-graph-NMS model.
    It weakens exactly one clause and leaves 'never adds detections' fully in force."""
    assert_detector_contract(ThresholdSwapsDetections(), frames, subset_monotonic=False)
    with pytest.raises(AssertionError, match="added"):
        assert_detector_contract(ThresholdAddsDetections(), frames, subset_monotonic=False)


def test_a_silent_detector_can_only_pass_when_the_caller_says_so(
    frames: tuple[Frame, ...],
) -> None:
    assert_detector_contract(Silent(), frames, require_nonempty=False)


def test_the_contract_needs_frames() -> None:
    with pytest.raises(AssertionError, match="at least one frame"):
        assert_detector_contract(_Base(), [])


@pytest.mark.parametrize(
    ("thresholds", "expected"),
    [
        ((0.25,), "at least two thresholds"),
        ((0.5, 0.25), "strictly ascending"),
        ((0.25, 0.25), "strictly ascending"),
        ((-0.1, 0.5), "lie in .0, 1."),
        ((0.5, 1.5), "lie in .0, 1."),
    ],
)
def test_bad_threshold_sweeps_are_rejected(
    thresholds: tuple[float, ...], expected: str, frames: tuple[Frame, ...]
) -> None:
    with pytest.raises(AssertionError, match=expected):
        assert_detector_contract(_Base(), frames, thresholds=thresholds)


def test_the_default_sweep_is_ascending_and_spans_a_useful_range() -> None:
    assert list(DEFAULT_CONTRACT_THRESHOLDS) == sorted(DEFAULT_CONTRACT_THRESHOLDS)
    assert DEFAULT_SCORE_THRESHOLD in DEFAULT_CONTRACT_THRESHOLDS
    assert DEFAULT_CONTRACT_THRESHOLDS[0] < 0.1
    assert DEFAULT_CONTRACT_THRESHOLDS[-1] > 0.9


def test_the_contract_accepts_a_generator_of_frames(make_frame: FrameFactory) -> None:
    """It materialises its input, so a one-shot iterable is not silently consumed by the
    first determinism check and then found empty by the next."""
    assert_detector_contract(_Base(), (make_frame(i) for i in range(3)))


def test_the_contract_does_not_rely_on_the_assert_statement() -> None:
    """``python -O`` strips ``assert`` statements.

    A contract that silently stops checking under an optimisation flag is worse than none,
    because it still looks like one -- so every clause raises ``AssertionError`` explicitly
    through ``_fail`` and this test reads the source to prove it.
    """
    source = inspect.getsource(assert_detector_contract)
    assert "_fail(" in source
    assert not [line for line in source.splitlines() if line.strip().startswith("assert ")]

"""SimulatedDetector: determinism, the contract (C12), and the limits of what it means.

Nothing measured here is a detection-accuracy result. These tests check that a
detection-shaped record generator is reproducible and well-formed, which is what the rest
of the pipeline needs from it (``docs/ACCEPTANCE.md`` H3).
"""

from __future__ import annotations

import inspect
import math
from collections.abc import Callable

import pytest
from parkscan_perception.contract import assert_detector_contract
from parkscan_perception.detector import Detector
from parkscan_perception.simulated import (
    DEFAULT_CLASSES,
    GroundTruthTarget,
    NoiseModel,
    SimulatedDetector,
    StaticTargets,
)

from parkscan_core.types import BBox, DetKind, DetSource, Frame, Severity

_KINDS = (DetKind.POTHOLE, DetKind.SPACE_OCCUPANCY)
_SEVERITY = (Severity.LOW, Severity.MEDIUM, Severity.HIGH)

FrameFactory = Callable[[int], Frame]


def targets_for(frame: Frame) -> tuple[GroundTruthTarget, ...]:
    """A pure ground-truth provider: what the simulator 'knows' about this frame.

    Derived from ``frame_id`` alone, so the provider itself cannot become a hidden source
    of nondeterminism -- the detector's guarantee is only as strong as its input's.
    """
    out: list[GroundTruthTarget] = []
    for k in range(1 + frame.frame_id % 3):
        kind = _KINDS[k % len(_KINDS)]
        x1 = 40.0 + 30.0 * k + (frame.frame_id % 7) * 11.0
        y1 = 300.0 + 12.0 * k
        out.append(
            GroundTruthTarget(
                bbox=BBox(x1, y1, x1 + 55.0, y1 + 40.0),
                kind=kind,
                severity=_SEVERITY[(frame.frame_id + k) % 3] if kind is DetKind.POTHOLE else None,
            )
        )
    return tuple(out)


def make_detector(seed: int = 42, noise: NoiseModel | None = None) -> SimulatedDetector:
    if noise is None:
        return SimulatedDetector(targets_for, seed=seed)
    return SimulatedDetector(targets_for, seed=seed, noise=noise)


# ----------------------------------------------------------------- the interface --
def test_simulated_detector_is_a_detector_at_runtime() -> None:
    assert isinstance(make_detector(), Detector)


def test_simulated_detector_is_a_detector_statically() -> None:
    # mypy --strict checks this annotation; it is the compile-time half of the isinstance
    # check above, and the two together are what "satisfies Detector" means.
    detector: Detector = make_detector()
    assert detector.source is DetSource.SIMULATED
    assert detector.classes == DEFAULT_CLASSES
    assert detector.name == "SimulatedDetector"


def test_source_is_simulated_and_there_is_no_way_to_configure_it(
    make_frame: FrameFactory,
) -> None:
    """A constructor argument for ``source`` would be a one-line route around every honesty
    guarantee in the project, so there isn't one (H1)."""
    assert "source" not in inspect.signature(SimulatedDetector.__init__).parameters
    det = make_detector()
    assert det.source is DetSource.SIMULATED
    produced = det.detect(make_frame(3), score_threshold=0.0)
    assert produced
    assert all(d.det_source is DetSource.SIMULATED for d in produced)


def test_contract_passes(frames: tuple[Frame, ...]) -> None:
    """Acceptance criterion C12, for this implementation."""
    assert_detector_contract(make_detector(), frames)


def test_contract_passes_for_several_seeds_and_noise_models(frames: tuple[Frame, ...]) -> None:
    for seed in (0, 1, 7, 2**40 + 3, -19):
        assert_detector_contract(SimulatedDetector(targets_for, seed=seed), frames)
    loud = NoiseModel(jitter_px=6.0, miss_rate=0.3, false_positive_rate=0.8)
    assert_detector_contract(SimulatedDetector(targets_for, seed=5, noise=loud), frames)


# -------------------------------------------------------------------- determinism --
def test_same_seed_and_frame_give_identical_output(frames: tuple[Frame, ...]) -> None:
    a, b = make_detector(seed=11), make_detector(seed=11)
    for frame in frames:
        assert tuple(a.detect(frame)) == tuple(b.detect(frame))


def test_output_does_not_depend_on_processing_order(frames: tuple[Frame, ...]) -> None:
    """The per-frame stream comes from ``(seed, frame_id)``, never from a running counter.

    A detector holding one advancing generator would pass 'same frame twice' and still fail
    this, and would then produce different detections whenever a frame was dropped -- which
    is exactly what happens under load.
    """
    forward, backward = make_detector(seed=3), make_detector(seed=3)
    expected = {frame.frame_id: tuple(forward.detect(frame)) for frame in frames}
    for frame in reversed(frames):
        assert tuple(backward.detect(frame)) == expected[frame.frame_id]


def test_a_fresh_detector_reproduces_an_old_one(frames: tuple[Frame, ...]) -> None:
    first = tuple(tuple(make_detector(seed=9).detect(f)) for f in frames)
    second = tuple(tuple(make_detector(seed=9).detect(f)) for f in frames)
    assert first == second


def test_different_seeds_give_different_output(frames: tuple[Frame, ...]) -> None:
    a, b = make_detector(seed=11), make_detector(seed=12)
    assert any(tuple(a.detect(f)) != tuple(b.detect(f)) for f in frames)


def test_the_threshold_only_filters(frames: tuple[Frame, ...]) -> None:
    """Raising the threshold removes detections; it never reshuffles or re-draws them."""
    det = make_detector(seed=4)
    for frame in frames:
        everything = set(det.detect(frame, score_threshold=0.0))
        for threshold in (0.1, 0.25, 0.5, 0.8, 0.95, 1.0):
            kept = set(det.detect(frame, score_threshold=threshold))
            assert kept == {d for d in everything if d.confidence >= threshold}


# -------------------------------------------------------------------- noise model --
def test_a_certain_miss_rate_suppresses_every_true_detection(frames: tuple[Frame, ...]) -> None:
    det = make_detector(seed=1, noise=NoiseModel(miss_rate=1.0, false_positive_rate=0.0))
    assert all(det.detect(f, score_threshold=0.0) == () for f in frames)


def test_a_zero_noise_detector_reproduces_the_ground_truth_boxes_exactly(
    make_frame: FrameFactory,
) -> None:
    det = make_detector(
        seed=1, noise=NoiseModel(jitter_px=0.0, miss_rate=0.0, false_positive_rate=0.0)
    )
    frame = make_frame(5)
    produced = det.detect(frame, score_threshold=0.0)
    truth = targets_for(frame)
    assert len(produced) == len(truth)
    for got, want in zip(produced, truth, strict=True):
        assert got.bbox == want.bbox
        assert got.kind is want.kind
        assert got.severity is want.severity
        assert got.stamp_ns == frame.stamp_ns
        assert got.detector_name == det.name


def test_false_positives_appear_and_are_well_formed(frames: tuple[Frame, ...]) -> None:
    det = make_detector(seed=2, noise=NoiseModel(miss_rate=1.0, false_positive_rate=2.0))
    produced = [d for f in frames for d in det.detect(f, score_threshold=0.0)]
    assert produced, "a false-positive rate of 2.0 per frame produced nothing over 24 frames"
    for d in produced:
        assert d.kind in DEFAULT_CLASSES
        assert (d.severity is None) is (d.kind is not DetKind.POTHOLE)
        assert 0.0 <= d.confidence <= 1.0


def test_confidence_stays_inside_the_unit_interval(frames: tuple[Frame, ...]) -> None:
    """Beta support is exactly [0, 1], which is exactly what Detection requires. A clipped
    normal would have piled probability mass on the endpoints instead."""
    det = make_detector(seed=6, noise=NoiseModel(confidence_alpha=0.4, confidence_beta=0.4))
    values = [d.confidence for f in frames for d in det.detect(f, score_threshold=0.0)]
    assert values
    assert all(0.0 <= c <= 1.0 and math.isfinite(c) for c in values)


def test_jitter_never_pushes_a_box_outside_the_frame(make_frame: FrameFactory) -> None:
    """A target flush against the image edge, with jitter far larger than the margin."""
    edge = GroundTruthTarget(BBox(0.0, 440.0, 40.0, 480.0), DetKind.POTHOLE, Severity.LOW)
    det = SimulatedDetector(
        StaticTargets({i: (edge,) for i in range(60)}),
        seed=8,
        noise=NoiseModel(jitter_px=25.0, miss_rate=0.0, false_positive_rate=0.0),
    )
    seen = 0
    for i in range(60):
        frame = make_frame(i)
        for d in det.detect(frame, score_threshold=0.0):
            assert 0.0 <= d.bbox.x1 <= d.bbox.x2 <= frame.width
            assert 0.0 <= d.bbox.y1 <= d.bbox.y2 <= frame.height
            seen += 1
    assert seen > 0


# ------------------------------------------------------------------- input rules --
def test_static_targets_returns_nothing_for_an_unknown_frame(make_frame: FrameFactory) -> None:
    provider = StaticTargets({0: (GroundTruthTarget(BBox(1.0, 1.0, 2.0, 2.0), DetKind.POTHOLE),)})
    assert provider(make_frame(1)) == ()
    assert len(provider(make_frame(0))) == 1


def test_severity_on_a_non_pothole_target_is_rejected() -> None:
    with pytest.raises(ValueError, match="pothole-only"):
        GroundTruthTarget(BBox(0.0, 0.0, 1.0, 1.0), DetKind.SPACE_OCCUPANCY, Severity.HIGH)


def test_a_target_outside_the_declared_classes_is_rejected(make_frame: FrameFactory) -> None:
    det = SimulatedDetector(
        targets_for, seed=1, classes=frozenset({DetKind.SPACE_OCCUPANCY}), name="occupancy-only"
    )
    with pytest.raises(ValueError, match="not in the declared classes"):
        det.detect(make_frame(0), score_threshold=0.0)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"jitter_px": -1.0},
        {"confidence_alpha": 0.0},
        {"confidence_beta": -2.0},
        {"miss_rate": 1.5},
        {"false_positive_rate": -0.1},
        {"fp_min_size_px": 0.0},
        {"fp_min_size_px": 90.0, "fp_max_size_px": 20.0},
    ],
)
def test_invalid_noise_model_rejected(kwargs: dict[str, float]) -> None:
    with pytest.raises(ValueError, match="must"):
        NoiseModel(**kwargs)


def test_empty_name_or_classes_rejected() -> None:
    with pytest.raises(ValueError, match="name"):
        SimulatedDetector(targets_for, seed=1, name="")
    with pytest.raises(ValueError, match="classes"):
        SimulatedDetector(targets_for, seed=1, classes=frozenset())


def test_warmup_is_a_documented_no_op(frames: tuple[Frame, ...]) -> None:
    det = make_detector(seed=13)
    before = tuple(det.detect(frames[0]))
    det.warmup()
    det.warmup()
    assert tuple(det.detect(frames[0])) == before
    assert det.seed == 13

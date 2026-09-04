"""The interface itself: seed derivation, and what the Protocol does and does not accept."""

from __future__ import annotations

import numpy as np
import pytest
from parkscan_perception.detector import (
    DEFAULT_SCORE_THRESHOLD,
    Detector,
    frame_rng,
    frame_seed,
)

from parkscan_core.types import DetKind, DetSource

_UINT64_MAX = 2**64 - 1


def test_the_default_threshold_matches_the_architecture_document() -> None:
    assert DEFAULT_SCORE_THRESHOLD == 0.25


@pytest.mark.parametrize(
    ("seed", "frame_id"), [(0, 0), (1, 0), (0, 1), (42, 7), (-3, 9), (2**63, 5)]
)
def test_frame_seed_is_a_pure_64_bit_function(seed: int, frame_id: int) -> None:
    value = frame_seed(seed, frame_id)
    assert value == frame_seed(seed, frame_id)
    assert 0 <= value <= _UINT64_MAX


def test_frame_seed_separates_frames_and_seeds() -> None:
    """Neighbouring frames must not get neighbouring streams, or a run's frames would be
    correlated in a way nobody chose."""
    by_frame = [frame_seed(7, i) for i in range(256)]
    by_seed = [frame_seed(i, 7) for i in range(256)]
    assert len(set(by_frame)) == len(by_frame)
    assert len(set(by_seed)) == len(by_seed)
    assert frame_seed(7, 0) != frame_seed(0, 7)


def test_frame_seed_is_not_python_hash() -> None:
    """``hash`` is only stable within one process (PYTHONHASHSEED randomises strings, and
    the int guarantee is per-run). A per-frame seed has to mean the same thing tomorrow."""
    assert frame_seed(3, 4) != hash((3, 4))


def test_frame_rng_is_reproducible_and_frame_scoped() -> None:
    a = frame_rng(11, 5).random(8)
    b = frame_rng(11, 5).random(8)
    c = frame_rng(11, 6).random(8)
    assert np.array_equal(a, b)
    assert not np.array_equal(a, c)


def test_frame_rng_is_a_numpy_generator_not_the_global_one() -> None:
    """No global RNG anywhere: seeding numpy's legacy global must not change a thing."""
    np.random.seed(1234)  # deliberately poking numpy's legacy global state
    first = frame_rng(2, 2).random(4)
    np.random.seed(9999)
    second = frame_rng(2, 2).random(4)
    assert isinstance(frame_rng(2, 2), np.random.Generator)
    assert np.array_equal(first, second)


def test_protocol_rejects_an_object_that_is_not_a_detector() -> None:
    class NotADetector:
        name = "nope"

    assert not isinstance(NotADetector(), Detector)
    assert not isinstance(object(), Detector)


def test_protocol_accepts_any_structurally_complete_object() -> None:
    """Structural, not nominal: a future RF-DETR wrapper never imports this Protocol."""

    class Minimal:
        name = "minimal"
        source = DetSource.GROUND_TRUTH
        classes = frozenset({DetKind.POTHOLE})

        def warmup(self) -> None:
            return None

        def detect(
            self, frame: object, *, score_threshold: float = DEFAULT_SCORE_THRESHOLD
        ) -> list[object]:
            return []

    assert isinstance(Minimal(), Detector)

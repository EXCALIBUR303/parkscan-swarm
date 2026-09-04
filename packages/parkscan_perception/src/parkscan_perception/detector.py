"""The detector interface every implementation satisfies, and the seed derivation it uses.

Three design decisions carry this whole package. They are stated here, once, because each
one is load-bearing and each one is easy to undo by accident.

**(a) ``detect()`` is pure with respect to the frame.**
The same :class:`~parkscan_core.types.Frame` handed to the same detector must produce the
same detections, byte for byte, on every call, in every process, on every run. So a
detector owns a *constructor* seed and derives a per-frame generator from
``(seed, frame.frame_id)`` via :func:`frame_rng`. There is no module-level generator, no
``random`` module, no ``time.time()`` and no dependence on how many frames have already
been processed -- a detector that advanced one shared stream would give different answers
depending on what it had been shown earlier, and the 30-seed benchmark would stop being
reproducible the moment a frame was dropped. Purity is also what makes offline evaluation
meaningful: re-scoring a recorded run cannot silently produce different detections from
the run that was recorded.

**(b) A detector never receives robot pose and never emits world coordinates.**
:class:`~parkscan_core.types.Frame` carries an image, a frame id, a stamp and a camera id,
and nothing else; :class:`~parkscan_core.types.Detection` is expressed in image pixels.
Geolocation lives in :mod:`parkscan_perception.geolocation` as a separate stage that takes
the pose. Two things follow. A detector that knew the map could *cheat* -- it could emit
the known position of a pothole it never saw, and every metric downstream would look
excellent. And because the two stages share no state, each is testable on its own: the
detector against frames, the geolocation against known geometry with no detector in sight.

**(c) Every :class:`~parkscan_core.types.Detection` carries ``det_source``.**
It is a required field with no default, it is a NOT NULL column
(``docs/DATA_MODEL.md``), and it is what the dashboard renders the SIMULATED badge from.
The badge is therefore a property of the data rather than a disclaimer somebody remembered
to write, which is the difference between a structural honesty guarantee and a cosmetic
label (``docs/ACCEPTANCE.md`` H1, H2). The contract in
:mod:`parkscan_perception.contract` asserts that ``detection.det_source`` equals the
detector's declared ``source`` on every detection, so a detector cannot mislabel its own
output.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Final, Protocol, runtime_checkable

import numpy as np

from parkscan_core.types import Detection, DetKind, DetSource, Frame

__all__ = [
    "DEFAULT_SCORE_THRESHOLD",
    "Detector",
    "frame_rng",
    "frame_seed",
]

DEFAULT_SCORE_THRESHOLD: Final = 0.25
"""Default confidence floor. Matches the Protocol default so callers and implementations
cannot drift apart, and matches ``docs/ARCHITECTURE.md`` section 5."""

_MASK64: Final = (1 << 64) - 1
_GOLDEN64: Final = 0x9E3779B97F4A7C15
_MIX_A: Final = 0xBF58476D1CE4E5B9
_MIX_B: Final = 0x94D049BB133111EB


def _splitmix64(x: int) -> int:
    """One SplitMix64 round: avalanche a 64-bit integer into a well-spread 64-bit integer.

    Written out rather than reached for via :func:`hash`, because ``hash`` is not a stable
    function: ``PYTHONHASHSEED`` randomises string hashing per process, and even for ints
    the guarantee is only "consistent within a run". A per-frame seed has to be the same
    number tomorrow, in CI, and inside a re-scored replay, so it is computed from named
    arithmetic constants that cannot change under anyone's interpreter settings.
    """
    z = (x + _GOLDEN64) & _MASK64
    z = ((z ^ (z >> 30)) * _MIX_A) & _MASK64
    z = ((z ^ (z >> 27)) * _MIX_B) & _MASK64
    return (z ^ (z >> 31)) & _MASK64


def frame_seed(seed: int, frame_id: int) -> int:
    """Derive a stable 64-bit seed from a detector seed and a frame id.

    Two frames of the same run get uncorrelated streams, and the same frame gets the same
    stream no matter when -- or whether -- its neighbours were processed. Negative inputs
    are folded into the 64-bit range rather than rejected, so the function is total.
    """
    return _splitmix64(_splitmix64(seed & _MASK64) ^ (frame_id & _MASK64))


def frame_rng(seed: int, frame_id: int) -> np.random.Generator:
    """The per-frame generator. **The only source of randomness a detector may use.**"""
    return np.random.default_rng(frame_seed(seed, frame_id))


@runtime_checkable
class Detector(Protocol):
    """What every detector -- simulated, real model, or ground truth -- must provide.

    ``classes`` is what the detector is *allowed* to emit, declared up front so the
    contract can reject a detector that quietly starts producing a class nothing
    downstream knows how to render or store.

    ``warmup()`` exists for the real-model implementations: RF-DETR's first inference pays
    for weight loading and kernel compilation, and a benchmark that includes that cost in
    its first sample reports a latency distribution with an invented tail. It must be safe
    to call more than once, and it must not change what :meth:`detect` returns -- the
    contract checks both.
    """

    name: str
    source: DetSource
    classes: frozenset[DetKind]

    def warmup(self) -> None:
        """Do any one-off work now. Idempotent; never changes :meth:`detect` output."""
        ...

    def detect(
        self,
        frame: Frame,
        *,
        score_threshold: float = DEFAULT_SCORE_THRESHOLD,
    ) -> Sequence[Detection]:
        """Detections for ``frame``, in image space, every one scoring at least
        ``score_threshold``. Pure with respect to ``frame``: same input, same output."""
        ...

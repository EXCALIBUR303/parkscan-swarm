"""``SimulatedDetector`` -- detection-shaped records from known geometry. **Not vision.**

This detector does not look at pixels. It never touches ``frame.image``, it contains no
model, no features, no thresholding of anything visual, and it would behave identically if
every frame were pure black. What it does is take ground-truth geometry the simulator
already knows -- "there is a HIGH-severity pothole occupying this box in this frame" -- and
turn it into :class:`~parkscan_core.types.Detection` records with a seeded, configurable
noise model on top: box jitter, a drawn confidence, a miss rate and a false-positive rate.

**What that measures, and what it does not.** Running this detector exercises the pipeline:
detector interface, geolocation, MQTT envelope, schema, WebSocket fan-out, dashboard pins,
severity rendering, and the SIMULATED badge. Every number produced from it is a statement
about *the pipeline*. It says **nothing whatsoever** about how well any real detector finds
real potholes in real images, and no metric derived from it may be named, labelled or
described as detection accuracy (``docs/ACCEPTANCE.md`` H3, H4). Its ``det_source`` is
:attr:`~parkscan_core.types.DetSource.SIMULATED` and that value is persisted, so the claim
is enforced by the data rather than by a footnote. Real-model accuracy belongs in a
separate ``EVAL.md``, measured on a named test split, and is never blended with these.

The noise model is a caricature, and an honest one: real detectors miss in *correlated*
ways -- distance, occlusion, motion blur, class confusion -- while this one misses
independently at a rate somebody typed into a config. It exists to make the pipeline handle
absent detections, spurious detections and imperfect boxes at all, not to model a detector.

Determinism
-----------
Everything stochastic comes from :func:`~parkscan_perception.detector.frame_rng`, built
fresh inside every :meth:`SimulatedDetector.detect` call from ``(seed, frame.frame_id)``.
Two consequences are worth stating because both are easy to break:

* The number of random draws never depends on the score threshold or on which branch was
  taken. Every target draws its miss coin, its jitter and its confidence *before* anything
  is decided, and thresholding happens last, on the finished list. That is what makes
  raising the threshold strictly filter the result instead of reshuffling it -- the
  monotonicity half of the contract.
* Class choice for false positives iterates a **sorted tuple**, never the ``frozenset``.
  ``StrEnum`` members hash by name, string hashing is randomised per process by
  ``PYTHONHASHSEED``, and so ``frozenset`` iteration order is not stable across runs. A
  detector seeded identically would otherwise emit different classes in a different
  process, which is precisely the kind of bug a determinism test only catches on a bad day.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Final, Protocol

import numpy as np

from parkscan_core.types import BBox, Detection, DetKind, DetSource, Frame, Severity
from parkscan_perception.detector import DEFAULT_SCORE_THRESHOLD, frame_rng

__all__ = [
    "DEFAULT_CLASSES",
    "DEFAULT_NOISE",
    "GroundTruthTarget",
    "NoiseModel",
    "SimulatedDetector",
    "StaticTargets",
    "TargetProvider",
]

DEFAULT_CLASSES: Final = frozenset({DetKind.POTHOLE, DetKind.SPACE_OCCUPANCY})
"""Both kinds in the data model. A module constant rather than a call in a default
argument, which ruff's bugbear rules rightly refuse."""

_SEVERITIES: Final[tuple[Severity, ...]] = (Severity.LOW, Severity.MEDIUM, Severity.HIGH)
"""Fixed order, so a seeded severity draw means the same thing in every process."""


@dataclass(frozen=True, slots=True)
class GroundTruthTarget:
    """One thing the simulator knows is in the frame, in image pixels.

    This is the detector's *input*, and giving it an explicit named type is the point: the
    runner has to hand the detector exactly what the simulator knows, in image space, and
    cannot hand it a pose, a world coordinate or a map. The detector's ignorance of the
    world is enforced by the shape of this type rather than by convention.

    ``severity`` is pothole-only, matching the nullable column in ``docs/DATA_MODEL.md``;
    supplying one for another kind is rejected here rather than downstream.
    """

    bbox: BBox
    kind: DetKind
    severity: Severity | None = None

    def __post_init__(self) -> None:
        if self.kind is not DetKind.POTHOLE and self.severity is not None:
            msg = f"severity is pothole-only; got {self.severity!r} on kind {self.kind!r}"
            raise ValueError(msg)


class TargetProvider(Protocol):
    """Supplies the ground truth for one frame.

    A callable rather than a stored list because the runner computes what is visible from
    the robot's current pose and the lot geometry, per frame. It is called exactly once per
    :meth:`SimulatedDetector.detect` call and must be a pure function of the frame -- a
    provider that returned different targets for the same frame would defeat the
    determinism guarantee no matter how carefully the detector itself is seeded.
    """

    def __call__(self, frame: Frame) -> Sequence[GroundTruthTarget]: ...


@dataclass(frozen=True, slots=True, eq=False)
class StaticTargets:
    """A :class:`TargetProvider` backed by a fixed ``frame_id -> targets`` mapping.

    The obvious provider for tests and for replay: unknown frame ids yield no targets,
    which is the honest answer for "the simulator knows nothing about this frame".
    """

    by_frame: Mapping[int, Sequence[GroundTruthTarget]]

    def __call__(self, frame: Frame) -> Sequence[GroundTruthTarget]:
        return self.by_frame.get(frame.frame_id, ())


@dataclass(frozen=True, slots=True)
class NoiseModel:
    """Seeded, configurable imperfection. Every default is a knob, not a measurement.

    ``jitter_px``
        Half-width of a uniform perturbation applied independently to each of the four box
        edges. Uniform, not Gaussian, so the perturbation is *bounded*: an unbounded tail
        would occasionally produce a box that survives clipping but is nowhere near the
        target, and the resulting geolocation outlier would look like a pipeline bug.
    ``confidence_alpha`` / ``confidence_beta``
        Confidence is drawn from ``Beta(alpha, beta)``, whose support is exactly ``[0, 1]``
        -- the range :class:`~parkscan_core.types.Detection` requires. Clipping a normal
        would have piled probability mass on 0.0 and 1.0, which then shows up as an
        implausible spike at the ends of any confidence histogram. The default
        ``Beta(8, 2)`` has mean 0.8 and standard deviation about 0.12.
    ``miss_rate``
        Probability that a known target produces no detection at all.
    ``false_positive_rate``
        Mean of a Poisson draw for the number of spurious detections per frame, so a frame
        may occasionally produce two. Spurious boxes are placed in the lower part of the
        image, where the ground plane is, since a false positive above the horizon would be
        thrown away by geolocation and would never exercise anything.
    """

    jitter_px: float = 2.0
    confidence_alpha: float = 8.0
    confidence_beta: float = 2.0
    miss_rate: float = 0.05
    false_positive_rate: float = 0.02
    fp_min_size_px: float = 12.0
    fp_max_size_px: float = 64.0

    def __post_init__(self) -> None:
        if self.jitter_px < 0.0:
            msg = f"jitter_px must be non-negative, got {self.jitter_px}"
            raise ValueError(msg)
        if self.confidence_alpha <= 0.0 or self.confidence_beta <= 0.0:
            msg = (
                "confidence_alpha and confidence_beta must be positive, got "
                f"{self.confidence_alpha} and {self.confidence_beta}"
            )
            raise ValueError(msg)
        if not 0.0 <= self.miss_rate <= 1.0:
            msg = f"miss_rate must lie in [0, 1], got {self.miss_rate}"
            raise ValueError(msg)
        if self.false_positive_rate < 0.0:
            msg = f"false_positive_rate must be non-negative, got {self.false_positive_rate}"
            raise ValueError(msg)
        if not 0.0 < self.fp_min_size_px <= self.fp_max_size_px:
            msg = (
                "false-positive box sizes must satisfy 0 < min <= max, got "
                f"{self.fp_min_size_px} and {self.fp_max_size_px}"
            )
            raise ValueError(msg)


DEFAULT_NOISE: Final = NoiseModel()
"""Shared immutable default, so ``NoiseModel()`` never appears in an argument default."""


def _clip_span(lo: float, hi: float, limit: float) -> tuple[float, float]:
    """Order and clip one axis of a jittered box into ``[0, limit]``.

    Ordering happens first: independent jitter on both edges can cross them over, and
    :class:`~parkscan_core.types.BBox` rejects a crossed box outright. Clipping second
    guarantees the contract's in-frame invariant. A box clipped to zero width or height is
    dropped by the caller rather than nudged back to a minimum size, because inventing a
    box for a target that has left the frame would be inventing a detection.
    """
    if lo > hi:
        lo, hi = hi, lo
    return (min(max(lo, 0.0), limit), min(max(hi, 0.0), limit))


class SimulatedDetector:
    """A :class:`~parkscan_perception.detector.Detector` over simulator ground truth.

    ``source`` is always :attr:`~parkscan_core.types.DetSource.SIMULATED` and is not
    configurable. A constructor argument that let this class claim to be a model would be a
    one-line route around every honesty guarantee in the project.

    Example::

        det = SimulatedDetector(
            StaticTargets({7: (GroundTruthTarget(BBox(100, 300, 160, 360), DetKind.POTHOLE,
                                                 Severity.HIGH),)}),
            seed=42,
        )
        det.detect(Frame(7, 1_000, "cam_front", 640, 480, None))
    """

    __slots__ = ("_class_order", "_noise", "_seed", "_targets", "classes", "name", "source")

    name: str
    source: DetSource
    classes: frozenset[DetKind]

    def __init__(
        self,
        targets: TargetProvider,
        *,
        seed: int,
        noise: NoiseModel = DEFAULT_NOISE,
        classes: frozenset[DetKind] = DEFAULT_CLASSES,
        name: str = "SimulatedDetector",
    ) -> None:
        if not name:
            msg = "name must be non-empty"
            raise ValueError(msg)
        if not classes:
            msg = "classes must declare at least one DetKind"
            raise ValueError(msg)
        self.name = name
        self.source = DetSource.SIMULATED
        self.classes = classes
        self._targets = targets
        self._seed = int(seed)
        self._noise = noise
        self._class_order: tuple[DetKind, ...] = tuple(sorted(classes, key=lambda k: k.value))

    @property
    def seed(self) -> int:
        """The constructor seed. Every draw in every frame descends from this and nothing else."""
        return self._seed

    def warmup(self) -> None:
        """Nothing to warm up: there is no model, no weights and no kernels to compile.

        The method still exists, and is still called by the contract, because the interface
        has to be the same one a real detector implements -- and because a no-op that is
        proven not to change :meth:`detect` output is a stronger statement than an absent
        method.
        """

    def detect(
        self,
        frame: Frame,
        *,
        score_threshold: float = DEFAULT_SCORE_THRESHOLD,
    ) -> tuple[Detection, ...]:
        """Detection-shaped records for ``frame``. Pure: same frame, same output, always."""
        rng = frame_rng(self._seed, frame.frame_id)
        noise = self._noise
        out: list[Detection] = []

        for target in self._targets(frame):
            if target.kind not in self.classes:
                msg = (
                    f"{self.name}: ground-truth target of kind {target.kind!r} is not in the "
                    f"declared classes {sorted(self.classes)}"
                )
                raise ValueError(msg)
            # Every draw happens before any decision, so the stream depends on the frame
            # alone -- never on the threshold, and never on whether this target was missed.
            missed = float(rng.random()) < noise.miss_rate
            jitter = rng.uniform(-noise.jitter_px, noise.jitter_px, size=4)
            confidence = float(rng.beta(noise.confidence_alpha, noise.confidence_beta))
            if missed:
                continue
            box = target.bbox
            x1, x2 = _clip_span(box.x1 + float(jitter[0]), box.x2 + float(jitter[2]), frame.width)
            y1, y2 = _clip_span(box.y1 + float(jitter[1]), box.y2 + float(jitter[3]), frame.height)
            if x2 <= x1 or y2 <= y1:
                continue
            out.append(
                Detection(
                    kind=target.kind,
                    det_source=DetSource.SIMULATED,
                    detector_name=self.name,
                    confidence=confidence,
                    severity=target.severity,
                    bbox=BBox(x1, y1, x2, y2),
                    stamp_ns=frame.stamp_ns,
                )
            )

        out.extend(self._false_positives(frame, rng))
        return tuple(d for d in out if d.confidence >= score_threshold)

    def _false_positives(self, frame: Frame, rng: np.random.Generator) -> list[Detection]:
        """Spurious detections: Poisson count, uniform placement in the lower image half."""
        noise = self._noise
        count = int(rng.poisson(noise.false_positive_rate))
        made: list[Detection] = []
        for _ in range(count):
            w = float(rng.uniform(noise.fp_min_size_px, noise.fp_max_size_px))
            h = float(rng.uniform(noise.fp_min_size_px, noise.fp_max_size_px))
            cx = float(rng.uniform(0.0, frame.width))
            cy = float(rng.uniform(frame.height / 2.0, frame.height))
            confidence = float(rng.beta(noise.confidence_alpha, noise.confidence_beta))
            kind = self._class_order[int(rng.integers(0, len(self._class_order)))]
            severity_index = int(rng.integers(0, len(_SEVERITIES)))
            x1, x2 = _clip_span(cx - w / 2.0, cx + w / 2.0, frame.width)
            y1, y2 = _clip_span(cy - h / 2.0, cy + h / 2.0, frame.height)
            if x2 <= x1 or y2 <= y1:
                continue
            made.append(
                Detection(
                    kind=kind,
                    det_source=DetSource.SIMULATED,
                    detector_name=self.name,
                    confidence=confidence,
                    severity=_SEVERITIES[severity_index] if kind is DetKind.POTHOLE else None,
                    bbox=BBox(x1, y1, x2, y2),
                    stamp_ns=frame.stamp_ns,
                )
            )
        return made

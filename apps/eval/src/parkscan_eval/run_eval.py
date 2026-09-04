"""``python -m parkscan_eval.run_eval --seed 42 --out <dir>`` -- the evaluation harness.

What it does, in order
----------------------
1. Loads the committed lot and builds the camera from the *geolocation stage's own*
   intrinsics and mount, so both directions of the projection describe one camera.
2. **Self-checks the geometry.** Forward-projects a grid of known ground points with
   :mod:`parkscan_eval.camera` and back-projects them with
   :mod:`parkscan_perception.geolocation`, with no error injected and the range gate
   effectively off. If the round trip is not exact the run aborts. A convention drift
   between the two modules -- a pitch sign, an axis permutation -- does not crash on its
   own; it produces confident, plausible, wrong points, and without this check the harness
   would faithfully report their error as a pipeline result.
3. Drives a robot down one zone's aisle in ``SimLite`` past targets at known positions and
   records the frames a camera would have produced.
4. Runs ``assert_detector_contract`` over those frames (acceptance criterion C12).
5. For each frame: detect, then geolocate every detection **twice** -- once at the
   production range gate and once with the gate relaxed -- from a *perturbed* pose and
   mount, and match detections back to the targets that produced them.
6. Writes ``results.csv`` and ``results.json`` and prints a short summary.

Why the pose and mount are perturbed
------------------------------------
With a perfect pose and a perfect mount, back-projection inverts forward projection to
about ``1e-13`` m at every range, and the harness would report a geolocation accuracy of
zero. That number would be true and completely worthless: it measures floating-point
arithmetic, not the pipeline. So the harness gives the geolocation stage the pose and mount
a real robot would have -- the true ones plus a **stated** attitude, height and position
error budget (:class:`ErrorBudget`) -- and reports the resulting position error against the
known truth. Every number this produces is therefore a statement of the form "given this
error budget, in this simulation, the geolocation stage lands this far off at this range".
It is a Tier 1 result in the sense of ``EVAL.md``, and it is not, and cannot be turned into,
a claim about detection accuracy anywhere (``docs/ACCEPTANCE.md`` H3, H4).

Reproducibility
---------------
``results.csv`` is byte-identical between two runs with the same seed, on the same lot, with
the same configuration. Everything that feeds it is deterministic: the scene geometry comes
from the lot alone, ``SimLite`` is seeded, the detector derives its per-frame stream from
``(seed, frame_id)``, and the error budget is drawn from a single generator seeded from the
run seed in one fixed-shape call. Floats are written with :func:`repr`, which is the
shortest exactly round-trippable form, so re-reading the CSV recovers the values the metrics
were computed from.

``results.json`` is **not** byte-identical, and says so in the file: its ``latency`` block
contains wall-clock measurements. Every other block in it is reproducible.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from parkscan_perception.contract import assert_detector_contract
from parkscan_perception.detector import DEFAULT_SCORE_THRESHOLD
from parkscan_perception.geolocation import (
    DEFAULT_MAX_RANGE_M,
    CameraIntrinsics,
    CameraMount,
    GeolocResult,
    backproject_pixel,
    geolocate_detection,
)
from parkscan_perception.simulated import (
    DEFAULT_NOISE,
    NoiseModel,
    SimulatedDetector,
    StaticTargets,
)
from parkscan_sim.config import SimConfig
from parkscan_sim.world import SimLite

from parkscan_core.lot import LotSpec, load_lot
from parkscan_core.types import Detection, DetKind, Pose2D
from parkscan_eval.camera import CameraModel, project_ground_point
from parkscan_eval.metrics import (
    DEFAULT_RANGE_BIN_EDGES,
    ContractOutcome,
    ContractSummary,
    ErrorSummary,
    GeolocationErrorStats,
    GeoPair,
    LatencyStats,
    RejectionRecord,
    RejectionStats,
    detector_contract_summary,
    geolocation_error_stats,
    horizon_rejection_rate,
    pipeline_latency_stats,
)
from parkscan_eval.provenance import Provenance, describe
from parkscan_eval.scene import RecordedFrame, SceneConfig, Sighting, drive_and_record

__all__ = [
    "CSV_COLUMNS",
    "RELAXED_RANGE_BIN_EDGES",
    "RELAXED_RANGE_GATE_M",
    "DetectionRow",
    "ErrorBudget",
    "EvalConfig",
    "EvalResult",
    "GeometryMismatchError",
    "assert_geometry_agrees",
    "format_summary",
    "main",
    "match_detections",
    "results_json",
    "run",
    "write_csv",
    "write_json",
]

REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_LOT = REPO_ROOT / "sim" / "lot" / "lot.yaml"

RELAXED_RANGE_GATE_M: float = 1000.0
"""The gate used for the second, clearly-labelled geolocation pass.

Large enough that nothing is range-rejected, so the error-versus-range table extends past
the production gate. That table is what *justifies* the 6 m gate; a table that stopped at
6 m could only assert it."""

RELAXED_RANGE_BIN_EDGES: tuple[float, ...] = (0.0, 2.0, 4.0, 6.0, 8.0, 12.0, 16.0, 20.0, math.inf)
"""Bins for the gate-relaxed pass: the production bins, continued out to where the targets
stop being resolvable at all."""

_BUDGET_STREAM: int = 0x5EED_B0D6
"""Stream id for the error-budget generator, so its draws can never collide with the
detector's ``(seed, frame_id)`` streams even at the same seed."""

_DRAW_COLUMNS: int = 5
"""Per frame, in this fixed order: pitch, yaw, camera height, robot x, robot y."""


# ------------------------------------------------------------------------- config --
@dataclass(frozen=True, slots=True)
class ErrorBudget:
    """The estimation error handed to the geolocation stage. Authored, not measured.

    These are the magnitudes a plausible small ground robot's state estimator might carry.
    **None of them is a measurement of any hardware**, and the numbers this harness reports
    are only meaningful relative to them -- halve ``pitch_sigma_rad`` and every geolocation
    error in the output roughly halves. They are hashed into ``results.json`` so a figure
    can never be quoted without the budget that produced it.

    Draws are Gaussian and therefore unbounded. That is deliberate: the interesting failure
    at long range is precisely the tail, where an attitude error larger than the depression
    angle tips the ray *above the horizon* and geolocation refuses to answer at all.
    """

    pitch_sigma_rad: float = 0.008726646259971648
    """0.5 degrees. The dominant term: it moves the point by about
    ``sigma * (R^2 + h^2) / h``, which is quadratic in range."""

    yaw_sigma_rad: float = 0.008726646259971648
    """0.5 degrees. Moves the point *across* the ray, so it grows only linearly with range."""

    height_sigma_m: float = 0.005
    """5 mm of mount-height error. Scales the whole ray, so its effect is linear in range."""

    position_sigma_m: float = 0.02
    """20 mm of pose error per axis, applied as a translation of the camera centre. It moves
    the answer by the same amount at every range -- which is why it ends up near the bottom
    of the ranked error sources rather than the top."""

    def __post_init__(self) -> None:
        for name in ("pitch_sigma_rad", "yaw_sigma_rad", "height_sigma_m", "position_sigma_m"):
            value: float = getattr(self, name)
            if not math.isfinite(value) or value < 0.0:
                msg = f"{name} must be finite and non-negative, got {value}"
                raise ValueError(msg)

    def as_dict(self) -> dict[str, float]:
        return {
            "pitch_sigma_rad": self.pitch_sigma_rad,
            "yaw_sigma_rad": self.yaw_sigma_rad,
            "height_sigma_m": self.height_sigma_m,
            "position_sigma_m": self.position_sigma_m,
        }


@dataclass(frozen=True, slots=True)
class EvalConfig:
    """Everything that determines the contents of ``results.csv``, in one hashable object."""

    seed: int = 42
    scene: SceneConfig = field(default_factory=SceneConfig)
    noise: NoiseModel = DEFAULT_NOISE
    budget: ErrorBudget = field(default_factory=ErrorBudget)
    intrinsics: CameraIntrinsics = field(default_factory=CameraIntrinsics)
    mount: CameraMount = field(default_factory=CameraMount)
    max_range_m: float = DEFAULT_MAX_RANGE_M
    score_threshold: float = DEFAULT_SCORE_THRESHOLD

    @property
    def match_gate_px(self) -> float:
        """How close a detection's bottom-centre must be to a target's to count as that target.

        Derived, not chosen. The detector perturbs each box edge by at most ``jitter_px``, so
        the bottom-centre moves by at most ``hypot(jitter_px, jitter_px)`` -- about 2.8 px at
        the default. Twice ``jitter_px`` clears that bound with headroom while staying far
        smaller than the image, so a spurious detection has a vanishing chance of landing
        inside the gate of a real target.
        """
        return 2.0 * self.noise.jitter_px

    def camera_model(self) -> CameraModel:
        """The forward-projection twin of ``(intrinsics, mount)``. One set of numbers."""
        return CameraModel(
            width=self.intrinsics.width,
            height=self.intrinsics.height,
            fx=self.intrinsics.fx,
            fy=self.intrinsics.fy,
            cx=self.intrinsics.cx,
            cy=self.intrinsics.cy,
            height_m=self.mount.height_m,
            pitch_rad=self.mount.pitch_rad,
        )

    def as_dict(self) -> dict[str, object]:
        """The exact structure that gets hashed into ``results.json``."""
        return {
            "seed": self.seed,
            "max_range_m": self.max_range_m,
            "relaxed_max_range_m": RELAXED_RANGE_GATE_M,
            "score_threshold": self.score_threshold,
            "match_gate_px": self.match_gate_px,
            "scene": {
                "zone_id": self.scene.zone_id,
                "speed_mps": self.scene.speed_mps,
                "camera_period_ticks": self.scene.camera_period_ticks,
                "camera_id": self.scene.camera_id,
                "lead_in_m": self.scene.lead_in_m,
                "tail_m": self.scene.tail_m,
                "target_spacing_m": self.scene.target_spacing_m,
                "lateral_offsets_m": list(self.scene.lateral_offsets_m),
                "target_size_m": self.scene.target_size_m,
                "min_box_px": self.scene.min_box_px,
                "frame_margin_px": self.scene.frame_margin_px,
                "max_steps": self.scene.max_steps,
            },
            "detector_noise": {
                "jitter_px": self.noise.jitter_px,
                "confidence_alpha": self.noise.confidence_alpha,
                "confidence_beta": self.noise.confidence_beta,
                "miss_rate": self.noise.miss_rate,
                "false_positive_rate": self.noise.false_positive_rate,
                "fp_min_size_px": self.noise.fp_min_size_px,
                "fp_max_size_px": self.noise.fp_max_size_px,
            },
            "error_budget": self.budget.as_dict(),
            "camera": {
                "width": self.intrinsics.width,
                "height": self.intrinsics.height,
                "hfov_rad": self.intrinsics.hfov_rad,
                "fx": self.intrinsics.fx,
                "fy": self.intrinsics.fy,
                "cx": self.intrinsics.cx,
                "cy": self.intrinsics.cy,
                "mount_height_m": self.mount.height_m,
                "mount_pitch_rad": self.mount.pitch_rad,
            },
        }


# --------------------------------------------------------------------------- rows --
@dataclass(frozen=True, slots=True)
class DetectionRow:
    """One row of ``results.csv``: one detection, its truth, and both geolocation passes.

    ``valid`` / ``reason`` / ``range_m`` / ``error_m`` describe the **production** pass, at
    the real range gate -- what the running system would have stored. The ``relaxed_``
    columns describe the same detection back-projected with the gate effectively off, which
    is what the error-versus-range table past 6 m is built from. ``est_x_m`` / ``est_y_m``
    are the back-projected point; the two passes compute it identically, so one pair covers
    both, and it is empty exactly when the ray never met the ground.
    """

    frame_id: int
    stamp_ns: int
    matched: bool
    target_id: str
    kind: str
    severity: str
    confidence: float
    det_source: str
    detector_name: str
    robot_x_m: float
    robot_y_m: float
    truth_x_m: float | None
    truth_y_m: float | None
    true_range_m: float | None
    est_x_m: float | None
    est_y_m: float | None
    range_m: float
    error_m: float | None
    valid: bool
    reason: str
    relaxed_range_m: float
    relaxed_error_m: float | None


CSV_COLUMNS: tuple[str, ...] = (
    "frame_id",
    "stamp_ns",
    "matched",
    "target_id",
    "kind",
    "severity",
    "confidence",
    "det_source",
    "detector_name",
    "robot_x_m",
    "robot_y_m",
    "truth_x_m",
    "truth_y_m",
    "true_range_m",
    "est_x_m",
    "est_y_m",
    "range_m",
    "error_m",
    "valid",
    "reason",
    "relaxed_range_m",
    "relaxed_error_m",
)


@dataclass(frozen=True, slots=True)
class EvalResult:
    """Everything one run produced."""

    config: EvalConfig
    provenance: Provenance
    rows: tuple[DetectionRow, ...]
    frame_count: int
    target_count: int
    matched_count: int
    unmatched_count: int
    missed_count: int
    geolocation: GeolocationErrorStats
    geolocation_relaxed: GeolocationErrorStats
    rejection: RejectionStats
    contract: ContractSummary
    detect_latency: LatencyStats
    geolocate_latency: LatencyStats
    frame_latency: LatencyStats


# ---------------------------------------------------------------------- self-check --
class GeometryMismatchError(RuntimeError):
    """The harness's forward projection and the geolocation stage disagree."""


_SELF_CHECK_POSES: tuple[Pose2D, ...] = (
    Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0),
    Pose2D(x_m=12.5, y_m=5.0, theta_rad=0.7),
    Pose2D(x_m=40.0, y_m=30.0, theta_rad=-2.1),
)
_SELF_CHECK_OFFSETS: tuple[tuple[float, float], ...] = (
    (1.0, 0.0),
    (2.5, 0.8),
    (5.0, -1.2),
    (9.0, 1.5),
)
_SELF_CHECK_TOLERANCE_M: float = 1e-6


def assert_geometry_agrees(
    camera: CameraModel,
    intrinsics: CameraIntrinsics,
    mount: CameraMount,
) -> float:
    """Round-trip known ground points through both projection stages; return the worst error.

    ``camera`` is the harness's forward model; ``intrinsics`` and ``mount`` are what the
    geolocation stage will be given. In a real run they describe the same camera by
    construction (:meth:`EvalConfig.camera_model`), and this function is what makes that a
    checked fact rather than an assumption -- it takes them separately precisely so a test
    can hand it two cameras that disagree and watch it complain.

    Raises :class:`GeometryMismatchError` if any point fails to come back within
    :data:`_SELF_CHECK_TOLERANCE_M`. It runs before every evaluation because the failure it
    guards against is silent: two modules that disagree about a pitch sign or an axis
    permutation still produce points on the ground, still produce a plausible-looking error
    distribution, and would let the harness publish a number that describes nothing.
    """
    worst = 0.0
    for pose in _SELF_CHECK_POSES:
        cos_t, sin_t = math.cos(pose.theta_rad), math.sin(pose.theta_rad)
        for forward_m, lateral_m in _SELF_CHECK_OFFSETS:
            truth = (
                pose.x_m + forward_m * cos_t - lateral_m * sin_t,
                pose.y_m + forward_m * sin_t + lateral_m * cos_t,
            )
            projection = project_ground_point(truth, pose, camera)
            if not projection.in_front:
                msg = (
                    f"self-check: point {truth!r} at range {projection.ground_range_m:.2f} m is "
                    f"not in front of the camera from pose {pose!r}"
                )
                raise GeometryMismatchError(msg)
            result = backproject_pixel(
                projection.u,
                projection.v,
                pose,
                intrinsics,
                mount,
                max_range_m=RELAXED_RANGE_GATE_M,
            )
            if not result.valid or result.point is None:
                msg = (
                    "self-check: parkscan_perception rejected a pixel that "
                    "parkscan_eval.camera produced from a ground point "
                    f"({result.reason!r} at range {result.range_m}). The most likely cause is "
                    "a pitch sign or axis convention that has drifted between the two modules."
                )
                raise GeometryMismatchError(msg)
            worst = max(worst, math.dist(result.point, truth))
    if worst > _SELF_CHECK_TOLERANCE_M:
        msg = (
            f"self-check: forward projection and back-projection disagree by {worst:.6g} m, "
            f"tolerance {_SELF_CHECK_TOLERANCE_M:g} m. The two stages are describing different "
            "cameras; fix that before trusting any number this harness produces."
        )
        raise GeometryMismatchError(msg)
    return worst


# ------------------------------------------------------------------------ matching --
def match_detections(
    detections: Sequence[Detection],
    sightings: Sequence[Sighting],
    gate_px: float,
) -> tuple[Sighting | None, ...]:
    """Associate each detection with the target that produced it, or ``None``.

    Greedy, nearest-first, one-to-one on the bbox **bottom-centre** -- the pixel geolocation
    actually uses. Ties are broken by index so the association is deterministic. A detection
    with no match is a false positive; it is kept in the output and counted, because a
    harness that quietly dropped the detector's spurious output would be measuring a
    detector that does not exist.
    """
    candidates: list[tuple[float, int, int]] = []
    for det_index, detection in enumerate(detections):
        du, dv = detection.bbox.bottom_centre
        for sight_index, sighting in enumerate(sightings):
            if detection.kind is not sighting.target.kind:
                continue
            su, sv = sighting.bbox.bottom_centre
            distance = math.hypot(du - su, dv - sv)
            if distance <= gate_px:
                candidates.append((distance, det_index, sight_index))
    candidates.sort()

    matched: list[Sighting | None] = [None] * len(detections)
    used_detections: set[int] = set()
    used_sightings: set[int] = set()
    for _, det_index, sight_index in candidates:
        if det_index in used_detections or sight_index in used_sightings:
            continue
        matched[det_index] = sightings[sight_index]
        used_detections.add(det_index)
        used_sightings.add(sight_index)
    return tuple(matched)


# ----------------------------------------------------------------------------- run --
def _perturbed(
    pose: Pose2D,
    mount: CameraMount,
    draws: Sequence[float],
) -> tuple[Pose2D, CameraMount]:
    """Apply one frame's error-budget draws. Column order is fixed by ``_DRAW_COLUMNS``."""
    pitch, yaw, height, dx, dy = draws
    return (
        Pose2D(x_m=pose.x_m + dx, y_m=pose.y_m + dy, theta_rad=pose.theta_rad + yaw),
        CameraMount(height_m=mount.height_m + height, pitch_rad=mount.pitch_rad + pitch),
    )


def _row(
    frame: RecordedFrame,
    detection: Detection,
    sighting: Sighting | None,
    production: GeolocResult,
    relaxed: GeolocResult,
) -> DetectionRow:
    truth = sighting.target.position if sighting is not None else None
    point = relaxed.point if relaxed.point is not None else production.point
    error = math.dist(truth, point) if (truth is not None and point is not None) else None
    return DetectionRow(
        frame_id=frame.frame.frame_id,
        stamp_ns=detection.stamp_ns,
        matched=sighting is not None,
        target_id=sighting.target.id if sighting is not None else "",
        kind=str(detection.kind),
        severity=str(detection.severity) if detection.severity is not None else "",
        confidence=detection.confidence,
        det_source=str(detection.det_source),
        detector_name=detection.detector_name,
        robot_x_m=frame.pose.x_m,
        robot_y_m=frame.pose.y_m,
        truth_x_m=truth[0] if truth is not None else None,
        truth_y_m=truth[1] if truth is not None else None,
        true_range_m=sighting.true_range_m if sighting is not None else None,
        est_x_m=point[0] if point is not None else None,
        est_y_m=point[1] if point is not None else None,
        range_m=production.range_m,
        error_m=error if production.valid else None,
        valid=production.valid,
        reason=production.reason,
        relaxed_range_m=relaxed.range_m,
        relaxed_error_m=error if relaxed.valid else None,
    )


def run(config: EvalConfig, *, lot_path: Path = DEFAULT_LOT) -> EvalResult:
    """Execute one evaluation and return everything it produced."""
    camera = config.camera_model()
    assert_geometry_agrees(camera, config.intrinsics, config.mount)

    lot: LotSpec = load_lot(lot_path)
    sim = SimLite(SimConfig())
    sim.reset(config.seed, lot)
    frames = drive_and_record(lot, sim, camera, config.scene)
    if not frames:
        msg = f"the drive down zone {config.scene.zone_id!r} produced no camera frames"
        raise RuntimeError(msg)

    detector = SimulatedDetector(
        StaticTargets({f.frame.frame_id: f.ground_truth() for f in frames}),
        seed=config.seed,
        noise=config.noise,
        classes=frozenset({DetKind.POTHOLE}),
    )

    contract = _run_contract(detector, frames)

    # One fixed-shape draw for the whole run: the values a frame gets never depend on how
    # many detections any earlier frame happened to produce.
    rng = np.random.default_rng([config.seed, _BUDGET_STREAM])
    draws = rng.normal(size=(len(frames), _DRAW_COLUMNS)) * np.array(
        [
            config.budget.pitch_sigma_rad,
            config.budget.yaw_sigma_rad,
            config.budget.height_sigma_m,
            config.budget.position_sigma_m,
            config.budget.position_sigma_m,
        ]
    )

    rows: list[DetectionRow] = []
    pairs: list[GeoPair] = []
    relaxed_pairs: list[GeoPair] = []
    rejections: list[RejectionRecord] = []
    detect_s: list[float] = []
    geolocate_s: list[float] = []
    frame_s: list[float] = []
    target_count = 0
    matched_count = 0

    for index, recorded in enumerate(frames):
        target_count += len(recorded.sightings)
        est_pose, est_mount = _perturbed(
            recorded.pose, config.mount, [float(d) for d in draws[index]]
        )

        frame_start = time.perf_counter()
        detections = tuple(detector.detect(recorded.frame, score_threshold=config.score_threshold))
        after_detect = time.perf_counter()
        production_results = [
            geolocate_detection(
                d, est_pose, config.intrinsics, est_mount, max_range_m=config.max_range_m
            )
            for d in detections
        ]
        frame_end = time.perf_counter()

        # The second, gate-relaxed pass is deliberately OUTSIDE the timed region. It is a
        # sensitivity analysis this harness performs; the running system does not do it, and
        # charging its cost to the pipeline would misreport the pipeline by a factor of two.
        relaxed_results = [
            geolocate_detection(
                d, est_pose, config.intrinsics, est_mount, max_range_m=RELAXED_RANGE_GATE_M
            )
            for d in detections
        ]
        results = list(zip(production_results, relaxed_results, strict=True))

        detect_s.append(after_detect - frame_start)
        geolocate_s.append(frame_end - after_detect)
        frame_s.append(frame_end - frame_start)

        matches = match_detections(detections, recorded.sightings, config.match_gate_px)
        for detection, sighting, (production, relaxed) in zip(
            detections, matches, results, strict=True
        ):
            rows.append(_row(recorded, detection, sighting, production, relaxed))
            rejections.append(
                RejectionRecord(
                    valid=production.valid,
                    reason="accepted" if production.valid else production.reason,
                )
            )
            if sighting is None:
                continue
            matched_count += 1
            # Binned at the TRUE range, not the estimated one -- see GeoPair's docstring.
            if production.valid and production.point is not None:
                pairs.append(
                    GeoPair(
                        truth=sighting.target.position,
                        estimate=production.point,
                        range_m=sighting.true_range_m,
                    )
                )
            if relaxed.valid and relaxed.point is not None:
                relaxed_pairs.append(
                    GeoPair(
                        truth=sighting.target.position,
                        estimate=relaxed.point,
                        range_m=sighting.true_range_m,
                    )
                )

    return EvalResult(
        config=config,
        provenance=describe(REPO_ROOT, config.as_dict()),
        rows=tuple(rows),
        frame_count=len(frames),
        target_count=target_count,
        matched_count=matched_count,
        unmatched_count=len(rows) - matched_count,
        missed_count=target_count - matched_count,
        geolocation=geolocation_error_stats(pairs, bin_edges=DEFAULT_RANGE_BIN_EDGES),
        geolocation_relaxed=geolocation_error_stats(
            relaxed_pairs, bin_edges=RELAXED_RANGE_BIN_EDGES
        ),
        rejection=horizon_rejection_rate(rejections),
        contract=contract,
        detect_latency=pipeline_latency_stats(detect_s),
        geolocate_latency=pipeline_latency_stats(geolocate_s),
        frame_latency=pipeline_latency_stats(frame_s),
    )


def _run_contract(detector: SimulatedDetector, frames: Sequence[RecordedFrame]) -> ContractSummary:
    """Run the shared detector contract and reduce it to a summary (C12)."""
    try:
        assert_detector_contract(detector, [f.frame for f in frames])
    except AssertionError as exc:
        return detector_contract_summary(
            [ContractOutcome(detector_name=detector.name, passed=False, detail=str(exc))]
        )
    return detector_contract_summary([ContractOutcome(detector_name=detector.name, passed=True)])


# ------------------------------------------------------------------------- output --
def _cell(value: object) -> str:
    """One CSV cell. Floats use ``repr`` -- shortest exactly round-trippable form."""
    if value is None:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return repr(value)
    return str(value)


def write_csv(rows: Sequence[DetectionRow], path: Path) -> None:
    """Write one row per detection.

    ``lineterminator="\\n"`` is not cosmetic: :mod:`csv` defaults to CRLF, and a file whose
    line endings depend on nothing at all is still a file whose bytes differ from the one
    everybody else generated.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(CSV_COLUMNS)
        for row in rows:
            writer.writerow([_cell(getattr(row, column)) for column in CSV_COLUMNS])


def _summary_dict(summary: ErrorSummary) -> dict[str, object]:
    return {
        "count": summary.count,
        "mean_m": summary.mean_m,
        "median_m": summary.median_m,
        "p95_m": summary.p95_m,
    }


def _error_dict(stats: GeolocationErrorStats) -> dict[str, object]:
    return {
        "overall": _summary_dict(stats.overall),
        "by_range": [
            {
                "lo_m": b.lo_m,
                "hi_m": b.hi_m,
                "label": b.label,
                **_summary_dict(b.summary),
            }
            for b in stats.bins
        ],
    }


def _latency_dict(stats: LatencyStats) -> dict[str, object]:
    return {"count": stats.count, "p50_ms": stats.p50_ms, "p95_ms": stats.p95_ms}


def results_json(result: EvalResult) -> dict[str, object]:
    """The ``results.json`` payload, as a plain dictionary."""
    return {
        "schema": "parkscan-eval/results/v1",
        "tier": 1,
        "tier_note": (
            "Every figure in this file is Tier 1: the behaviour of THIS pipeline against "
            "known simulated ground truth, under the stated error budget. Nothing here is "
            "a measurement of real-world detection accuracy, and no mAP, precision or "
            "recall appears because a simulated detector cannot produce one. See EVAL.md."
        ),
        "run": {
            "seed": result.config.seed,
            **result.provenance.as_dict(),
        },
        "config": result.config.as_dict(),
        "counts": {
            "frames": result.frame_count,
            "target_sightings": result.target_count,
            "detections": len(result.rows),
            "matched_detections": result.matched_count,
            "unmatched_detections": result.unmatched_count,
            "unreported_sightings": result.missed_count,
        },
        "geolocation_error": _error_dict(result.geolocation),
        "geolocation_error_gate_relaxed": {
            "note": (
                f"Same detections and the same perturbed pose, re-geolocated with the range "
                f"gate raised to {RELAXED_RANGE_GATE_M:g} m. This is a sensitivity analysis "
                f"that justifies the {result.config.max_range_m:g} m production gate; it is "
                "not what the running system stores."
            ),
            "max_range_m": RELAXED_RANGE_GATE_M,
            **_error_dict(result.geolocation_relaxed),
        },
        "rejection": {
            "total": result.rejection.total,
            "rejected": result.rejection.rejected,
            "rejection_rate": result.rejection.rejection_rate,
            "reasons": dict(result.rejection.reasons),
        },
        "detector_contract": {
            "total": result.contract.total,
            "passed": result.contract.passed,
            "failed": result.contract.failed,
            "failures": [
                {"detector_name": name, "detail": detail}
                for name, detail in result.contract.failures
            ],
        },
        "latency": {
            "reproducible": False,
            "note": (
                "Wall-clock measurements of this machine. They are the only part of this "
                "file that changes between two runs of the same seed, and they say nothing "
                "about a real detector: SimulatedDetector performs no inference."
            ),
            "unit": "milliseconds per camera frame, not per detection",
            "detect": _latency_dict(result.detect_latency),
            "geolocate": _latency_dict(result.geolocate_latency),
            "frame_total": _latency_dict(result.frame_latency),
        },
    }


def write_json(result: EvalResult, path: Path) -> None:
    """Write ``results.json``. Sorted keys and a trailing newline, so it diffs cleanly."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(results_json(result), handle, indent=2, sort_keys=True)
        handle.write("\n")


def _fmt(value: float) -> str:
    return "     -" if math.isnan(value) else f"{value:6.3f}"


def format_summary(result: EvalResult, out_dir: Path) -> str:
    """A short human-readable report. Every line names what it is and is not."""
    lines: list[str] = []
    dirty = " (WORKING TREE DIRTY)" if result.provenance.git_dirty else ""
    lines.append(
        f"ParkScan eval  seed={result.config.seed}  "
        f"git={result.provenance.git_sha[:12]}{dirty}  "
        f"config={result.provenance.config_hash[:12]}"
    )
    lines.append(
        f"  frames {result.frame_count}   target sightings {result.target_count}   "
        f"detections {len(result.rows)} "
        f"({result.matched_count} matched, {result.unmatched_count} unmatched)"
    )
    lines.append("")
    lines.append(
        f"  Geolocation error by TRUE range, gate relaxed to {RELAXED_RANGE_GATE_M:g} m "
        "(simulated truth, stated error budget):"
    )
    lines.append(f"    {'range':>10}  {'n':>6}  {'mean':>6}  {'p50':>6}  {'p95':>6}   [metres]")
    for rbin in result.geolocation_relaxed.bins:
        lines.append(
            f"    {rbin.label:>10}  {rbin.summary.count:>6}  {_fmt(rbin.summary.mean_m)}  "
            f"{_fmt(rbin.summary.median_m)}  {_fmt(rbin.summary.p95_m)}"
        )
    prod = result.geolocation.overall
    lines.append(
        f"  Inside the {result.config.max_range_m:g} m production gate: n={prod.count}  "
        f"mean {_fmt(prod.mean_m)}  p50 {_fmt(prod.median_m)}  p95 {_fmt(prod.p95_m)} m"
    )
    lines.append("")
    total = result.rejection.total or 1
    reasons = ", ".join(
        f"{name} {count} ({100.0 * count / total:.1f}%)" for name, count in result.rejection.reasons
    )
    lines.append(f"  Geolocation outcome over {result.rejection.total} attempts: {reasons}")
    lines.append(
        "    (the range-gate share reflects where the targets were placed - most sightings "
        "are beyond"
    )
    lines.append(
        f"     {result.config.max_range_m:g} m by design - and is not a defect rate. The "
        "above-horizon share is the C11 behaviour.)"
    )
    lines.append(
        f"  Detector contract (C12): {result.contract.passed}/{result.contract.total} passed"
    )
    for name, detail in result.contract.failures:
        lines.append(f"    FAILED {name}: {detail.splitlines()[0]}")
    lines.append(
        f"  Latency p50/p95 ms - detect {result.detect_latency.p50_ms:.3f}/"
        f"{result.detect_latency.p95_ms:.3f}  geolocate {result.geolocate_latency.p50_ms:.3f}/"
        f"{result.geolocate_latency.p95_ms:.3f}  frame {result.frame_latency.p50_ms:.3f}/"
        f"{result.frame_latency.p95_ms:.3f}   (not reproducible; no inference performed)"
    )
    lines.append("")
    lines.append(f"  Wrote {out_dir / 'results.csv'} ({len(result.rows)} rows)")
    lines.append(f"  Wrote {out_dir / 'results.json'}")
    lines.append(
        "  TIER 1 ONLY. mAP@50, mAP@50-95 and precision/recall are NOT measured here and "
        "cannot be: see EVAL.md."
    )
    return "\n".join(lines)


# ---------------------------------------------------------------------------- CLI --
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m parkscan_eval.run_eval",
        description=(
            "Evaluate the perception pipeline against known simulated ground truth. "
            "Produces Tier 1 metrics only (see apps/eval/EVAL.md)."
        ),
    )
    parser.add_argument("--seed", type=int, default=42, help="run seed (default: 42)")
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("apps/eval/out"),
        help="directory for results.csv and results.json (default: apps/eval/out)",
    )
    parser.add_argument(
        "--lot", type=Path, default=DEFAULT_LOT, help="lot spec YAML (default: sim/lot/lot.yaml)"
    )
    parser.add_argument("--zone", default=SceneConfig().zone_id, help="which zone's aisle to drive")
    parser.add_argument("--quiet", action="store_true", help="write the files, print nothing")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config = EvalConfig(seed=args.seed, scene=SceneConfig(zone_id=args.zone))

    result = run(config, lot_path=args.lot)
    out_dir: Path = args.out
    write_csv(result.rows, out_dir / "results.csv")
    write_json(result, out_dir / "results.json")

    if not args.quiet:
        print(format_summary(result, out_dir))
    # A contract failure is a build failure, not a chart (docs/ACCEPTANCE.md C12).
    return 1 if result.contract.failed else 0


if __name__ == "__main__":
    sys.exit(main())

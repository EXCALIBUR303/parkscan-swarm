"""Reproducibility is the product, and honesty is the other one.

The headline case here is byte-identity of ``results.csv`` across two runs of the same seed.
Everything else in this file guards a way that could quietly stop being true: the self-check
that catches a projection convention drift, the association that keeps false positives out
of the error table, and the absence of any accuracy metric from the output.
"""

from __future__ import annotations

import dataclasses
import json
import math
from pathlib import Path

import pytest
from parkscan_eval.camera import CameraModel
from parkscan_eval.run_eval import (
    CSV_COLUMNS,
    RELAXED_RANGE_GATE_M,
    ErrorBudget,
    EvalConfig,
    GeometryMismatchError,
    assert_geometry_agrees,
    format_summary,
    main,
    match_detections,
    results_json,
    run,
)
from parkscan_eval.scene import SceneConfig, Sighting, Target
from parkscan_perception.geolocation import CameraIntrinsics, CameraMount
from parkscan_perception.simulated import NoiseModel

from parkscan_core.types import BBox, Detection, DetKind, DetSource, Severity


# ------------------------------------------------------------------- reproducible --
def test_results_csv_is_byte_identical_for_a_fixed_seed(tmp_path: Path) -> None:
    first, second = tmp_path / "a", tmp_path / "b"
    assert main(["--seed", "42", "--out", str(first), "--quiet"]) == 0
    assert main(["--seed", "42", "--out", str(second), "--quiet"]) == 0

    a = (first / "results.csv").read_bytes()
    b = (second / "results.csv").read_bytes()
    assert a == b
    assert len(a.splitlines()) > 100


def test_a_different_seed_produces_a_different_csv(tmp_path: Path) -> None:
    first, second = tmp_path / "a", tmp_path / "b"
    main(["--seed", "42", "--out", str(first), "--quiet"])
    main(["--seed", "7", "--out", str(second), "--quiet"])
    assert (first / "results.csv").read_bytes() != (second / "results.csv").read_bytes()


def test_results_json_is_identical_apart_from_the_declared_latency_block(
    tmp_path: Path,
) -> None:
    """Timings are the only non-reproducible thing in the file, and the file says so."""
    first, second = tmp_path / "a", tmp_path / "b"
    main(["--seed", "42", "--out", str(first), "--quiet"])
    main(["--seed", "42", "--out", str(second), "--quiet"])

    a = json.loads((first / "results.json").read_text(encoding="utf-8"))
    b = json.loads((second / "results.json").read_text(encoding="utf-8"))
    assert a["latency"]["reproducible"] is False
    assert a.pop("latency") is not None
    b.pop("latency")
    assert a == b


def test_the_csv_header_is_the_declared_column_list(tmp_path: Path) -> None:
    main(["--seed", "42", "--out", str(tmp_path), "--quiet"])
    header = (tmp_path / "results.csv").read_text(encoding="utf-8").splitlines()[0]
    assert header.split(",") == list(CSV_COLUMNS)
    for required in ("range_m", "error_m", "valid", "reason", "confidence", "severity"):
        assert required in CSV_COLUMNS
    assert "det_source" in CSV_COLUMNS


def test_the_csv_uses_lf_line_endings(tmp_path: Path) -> None:
    """A CRLF file is a different file, and csv.writer defaults to CRLF."""
    main(["--seed", "42", "--out", str(tmp_path), "--quiet"])
    assert b"\r\n" not in (tmp_path / "results.csv").read_bytes()


def test_floats_in_the_csv_round_trip_exactly(tmp_path: Path) -> None:
    result = run(EvalConfig(seed=42))
    main(["--seed", "42", "--out", str(tmp_path), "--quiet"])
    lines = (tmp_path / "results.csv").read_text(encoding="utf-8").splitlines()
    index = CSV_COLUMNS.index("confidence")
    written = [float(line.split(",")[index]) for line in lines[1:]]
    assert written == [row.confidence for row in result.rows]


# --------------------------------------------------------------------- self-check --
def _camera_from(intrinsics: CameraIntrinsics, mount: CameraMount) -> CameraModel:
    return CameraModel(
        width=intrinsics.width,
        height=intrinsics.height,
        fx=intrinsics.fx,
        fy=intrinsics.fy,
        cx=intrinsics.cx,
        cy=intrinsics.cy,
        height_m=mount.height_m,
        pitch_rad=mount.pitch_rad,
    )


def test_the_two_projection_stages_agree_to_floating_point() -> None:
    config = EvalConfig()
    worst = assert_geometry_agrees(config.camera_model(), config.intrinsics, config.mount)
    assert worst < 1e-9


def test_the_self_check_catches_a_flipped_pitch_sign() -> None:
    """The failure this guards against never crashes on its own; it just lies quietly."""
    intrinsics, mount = CameraIntrinsics(), CameraMount()
    flipped = _camera_from(intrinsics, dataclasses.replace(mount, pitch_rad=-mount.pitch_rad))
    with pytest.raises(GeometryMismatchError):
        assert_geometry_agrees(flipped, intrinsics, mount)


def test_the_self_check_catches_a_focal_length_that_has_drifted() -> None:
    intrinsics, mount = CameraIntrinsics(), CameraMount()
    wrong = dataclasses.replace(_camera_from(intrinsics, mount), fx=intrinsics.fx * 1.05)
    with pytest.raises(GeometryMismatchError, match="disagree by"):
        assert_geometry_agrees(wrong, intrinsics, mount)


def test_the_self_check_catches_a_mount_height_that_has_drifted() -> None:
    intrinsics, mount = CameraIntrinsics(), CameraMount()
    wrong = dataclasses.replace(_camera_from(intrinsics, mount), height_m=mount.height_m + 0.1)
    with pytest.raises(GeometryMismatchError, match="disagree by"):
        assert_geometry_agrees(wrong, intrinsics, mount)


# ---------------------------------------------------------------------- matching --
def _detection(u: float, v: float, kind: DetKind = DetKind.POTHOLE) -> Detection:
    return Detection(
        kind=kind,
        det_source=DetSource.SIMULATED,
        detector_name="SimulatedDetector",
        confidence=0.9,
        severity=Severity.HIGH if kind is DetKind.POTHOLE else None,
        bbox=BBox(u - 10.0, v - 10.0, u + 10.0, v),
        stamp_ns=0,
    )


def _sighting(u: float, v: float, target_id: str = "pothole_01") -> Sighting:
    return Sighting(
        target=Target(
            id=target_id,
            position=(1.0, 2.0),
            size_m=0.45,
            kind=DetKind.POTHOLE,
            severity=Severity.HIGH,
        ),
        bbox=BBox(u - 10.0, v - 10.0, u + 10.0, v),
        true_range_m=3.0,
    )


def test_matching_associates_the_nearest_bottom_centre() -> None:
    detections = [_detection(100.0, 200.0), _detection(300.0, 200.0)]
    sightings = [_sighting(301.0, 200.5, "far"), _sighting(100.5, 199.0, "near")]
    matched = match_detections(detections, sightings, gate_px=4.0)
    assert matched[0] is not None
    assert matched[1] is not None
    assert matched[0].target.id == "near"
    assert matched[1].target.id == "far"


def test_matching_is_one_to_one() -> None:
    """Two detections on one target cannot both claim it: one is a duplicate."""
    detections = [_detection(100.0, 200.0), _detection(101.0, 200.0)]
    matched = match_detections(detections, [_sighting(100.0, 200.0)], gate_px=4.0)
    assert sum(1 for m in matched if m is not None) == 1


def test_a_detection_outside_the_gate_is_unmatched() -> None:
    matched = match_detections([_detection(100.0, 200.0)], [_sighting(140.0, 260.0)], 4.0)
    assert matched == (None,)


def test_the_match_gate_is_derived_from_the_detector_jitter() -> None:
    config = EvalConfig(noise=NoiseModel(jitter_px=3.0))
    assert config.match_gate_px == 6.0
    # ... and it clears the worst displacement a bounded jitter can cause.
    assert config.match_gate_px > math.hypot(3.0, 3.0)


# ----------------------------------------------------------------------- honesty --
def test_the_json_reports_no_accuracy_metric_of_any_kind() -> None:
    """ACCEPTANCE H4: mAP / precision / recall belong to a real model on a named split."""
    payload = json.dumps(results_json(run(EvalConfig(seed=42)))).lower()
    for banned in ('"map', '"map50', '"precision"', '"recall"', '"f1"', '"accuracy"'):
        assert banned not in payload


def test_every_detection_row_is_labelled_simulated() -> None:
    """H1: the SIMULATED provenance travels with the data, not with a footnote."""
    result = run(EvalConfig(seed=42))
    assert result.rows
    assert {row.det_source for row in result.rows} == {"simulated"}


def test_the_printed_summary_states_the_tier_and_what_is_absent(tmp_path: Path) -> None:
    summary = format_summary(run(EvalConfig(seed=42)), tmp_path)
    assert "TIER 1" in summary
    assert "NOT measured here" in summary
    assert "mAP@50" in summary


# -------------------------------------------------------------------- the results --
def test_error_grows_with_range_across_the_relaxed_bins() -> None:
    """The headline finding: a single number would hide this."""
    result = run(EvalConfig(seed=42))
    means = [b.summary.mean_m for b in result.geolocation_relaxed.bins if b.summary.count > 0]
    assert len(means) >= 6
    assert means == sorted(means)
    assert means[-1] > 20.0 * means[0]


def test_the_production_gate_keeps_the_stored_error_small() -> None:
    result = run(EvalConfig(seed=42))
    assert result.geolocation.overall.count > 100
    assert result.geolocation.overall.p95_m < 2.0


ZERO_BUDGET = ErrorBudget(
    pitch_sigma_rad=0.0, yaw_sigma_rad=0.0, height_sigma_m=0.0, position_sigma_m=0.0
)


def test_zeroing_every_error_source_makes_geolocation_exact() -> None:
    """Proof that what the harness reports is the injected budget, not a pipeline defect.

    With no attitude, pose or height error *and* no box jitter, forward projection and
    back-projection are exact inverses, and the reported error collapses to floating point.
    Any residual would mean the pipeline itself was losing the answer.
    """
    result = run(
        EvalConfig(seed=42, budget=ZERO_BUDGET, noise=NoiseModel(jitter_px=0.0)),
    )
    assert result.geolocation_relaxed.overall.count > 500
    assert result.geolocation_relaxed.overall.p95_m < 1e-9


def test_box_jitter_alone_is_a_real_and_range_dependent_error_source() -> None:
    """The second-ranked source in EVAL.md: bbox bottom-edge localisation, on its own."""
    jitter_only = run(EvalConfig(seed=42, budget=ZERO_BUDGET)).geolocation_relaxed
    full = run(EvalConfig(seed=42)).geolocation_relaxed

    assert jitter_only.overall.p95_m > 1e-3
    # It matters, and it is still smaller than the attitude budget layered on top of it.
    assert jitter_only.overall.p95_m < full.overall.p95_m
    means = [b.summary.mean_m for b in jitter_only.bins if b.summary.count > 0]
    assert means == sorted(means)


def test_the_detector_contract_passes_and_a_failure_would_fail_the_run(
    tmp_path: Path,
) -> None:
    result = run(EvalConfig(seed=42))
    assert result.contract.total == 1
    assert result.contract.failed == 0
    assert main(["--seed", "42", "--out", str(tmp_path), "--quiet"]) == 0


def test_unmatched_detections_are_kept_and_counted_not_dropped() -> None:
    """A harness that hid the detector's false positives would measure a different detector."""
    result = run(EvalConfig(seed=42, noise=NoiseModel(false_positive_rate=0.5)))
    assert result.unmatched_count > 0
    assert result.unmatched_count == sum(1 for row in result.rows if not row.matched)
    assert all(row.target_id == "" for row in result.rows if not row.matched)
    assert all(row.error_m is None for row in result.rows if not row.matched)


def test_rejection_reasons_cover_every_attempt() -> None:
    result = run(EvalConfig(seed=42))
    assert result.rejection.total == len(result.rows)
    assert sum(count for _, count in result.rejection.reasons) == result.rejection.total
    reasons = dict(result.rejection.reasons)
    assert reasons["accepted"] + result.rejection.rejected == result.rejection.total
    # C11: rays at or above the horizon are rejected, never clamped to a plausible point.
    assert reasons.get("above_horizon", 0) > 0


def test_no_row_carries_a_point_it_was_not_allowed_to_have() -> None:
    result = run(EvalConfig(seed=42))
    for row in result.rows:
        if row.reason == "above_horizon":
            assert row.est_x_m is None
            assert math.isinf(row.range_m)
        if row.valid:
            assert row.range_m <= EvalConfig().max_range_m
            assert row.est_x_m is not None


def test_the_relaxed_pass_is_the_same_geometry_with_the_gate_moved() -> None:
    result = run(EvalConfig(seed=42))
    both = [r for r in result.rows if r.error_m is not None]
    assert both
    for row in both:
        assert row.relaxed_error_m == row.error_m
        assert row.relaxed_range_m == row.range_m
    assert EvalConfig().max_range_m < RELAXED_RANGE_GATE_M


def test_a_scene_nothing_is_visible_in_is_an_error_not_a_zero_row_csv() -> None:
    """A run that produced nothing must say so rather than write an empty, passing file."""
    config = EvalConfig(seed=42, scene=SceneConfig(min_box_px=100_000.0))
    with pytest.raises(ValueError, match="at least one pair"):
        run(config)

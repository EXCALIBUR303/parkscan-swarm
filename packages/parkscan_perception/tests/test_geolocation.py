"""Geolocation: exact round trip, horizon rejection, the range gate, and the error budget.

The error-budget test is the important one. It is the only test here that measures a
*property* rather than a value, and it is the property the README is allowed to document:
flat-ground monocular error grows with the square of ground range, which is why a range
gate exists at all and why it sits at 6 m rather than wherever the map still looked
plausible.
"""

from __future__ import annotations

import math

import pytest
from parkscan_perception.geolocation import (
    DEFAULT_MAX_RANGE_M,
    REASON_ABOVE_HORIZON,
    REASON_BEYOND_MAX_RANGE,
    REASON_NON_FINITE,
    REASON_OK,
    CameraIntrinsics,
    CameraMount,
    GeolocResult,
    backproject_pixel,
    camera_ray,
    geolocate_detection,
    project_ground_point,
)

from parkscan_core.types import BBox, Detection, DetKind, DetSource, Pose2D, Severity

EXACT = 1e-9
"""This is closed-form arithmetic on doubles, not an approximation. A loose tolerance here
would hide a genuine sign or convention error inside 'close enough'."""

ONE_DEGREE = math.radians(1.0)


# ------------------------------------------------------------------ configuration --
def test_intrinsics_focal_length_matches_the_documented_formula() -> None:
    cam = CameraIntrinsics(width=640, height=480, hfov_rad=math.radians(70.0))
    assert cam.fx == pytest.approx(320.0 / math.tan(math.radians(35.0)), abs=EXACT)
    assert cam.fy == cam.fx
    assert (cam.cx, cam.cy) == (320.0, 240.0)
    assert cam.vfov_rad == pytest.approx(2.0 * math.atan(240.0 / cam.fx), abs=EXACT)


def test_default_mount_looks_below_the_horizon() -> None:
    # The sign convention is load-bearing: a positive pitch_rad stares at the sky and
    # every single detection would be rejected, silently, forever.
    mount = CameraMount()
    assert mount.looks_below_horizon
    assert mount.pitch_rad == pytest.approx(math.radians(-15.0), abs=EXACT)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"height_m": 0.0},
        {"height_m": -0.4},
        {"height_m": math.inf},
        {"pitch_rad": math.pi / 2.0},
        {"pitch_rad": -math.pi},
    ],
)
def test_invalid_mount_rejected(kwargs: dict[str, float]) -> None:
    with pytest.raises(ValueError, match=r"height_m|pitch_rad"):
        CameraMount(**kwargs)


@pytest.mark.parametrize("bad", [0.0, -1.0, math.inf, math.nan])
def test_invalid_max_range_rejected(
    bad: float, intrinsics: CameraIntrinsics, mount: CameraMount
) -> None:
    with pytest.raises(ValueError, match="max_range_m"):
        backproject_pixel(320.0, 400.0, Pose2D(0.0, 0.0, 0.0), intrinsics, mount, max_range_m=bad)


# --------------------------------------------------------------------- round trip --
@pytest.mark.parametrize(
    ("pose", "target"),
    [
        (Pose2D(0.0, 0.0, 0.0), (3.0, 0.0)),
        (Pose2D(0.0, 0.0, 0.0), (1.0, 0.4)),
        (Pose2D(0.0, 0.0, 0.0), (5.5, -1.2)),
        (Pose2D(12.5, -30.25, 0.0), (14.0, -30.25)),
        (Pose2D(12.5, -30.25, math.pi / 2.0), (12.5, -27.0)),
        (Pose2D(-4.0, 7.0, -2.4), (-6.1, 4.9)),
        (Pose2D(75.0, 50.0, 3.0), (72.4, 50.9)),
    ],
)
def test_round_trip_recovers_a_known_ground_point(
    pose: Pose2D,
    target: tuple[float, float],
    intrinsics: CameraIntrinsics,
    mount: CameraMount,
) -> None:
    """World point -> pixel -> world point, with no error worth measuring."""
    pixel = project_ground_point(target[0], target[1], pose, intrinsics, mount)
    assert pixel is not None
    result = backproject_pixel(*pixel, pose, intrinsics, mount)
    assert result.valid, result.reason
    assert result.point is not None
    assert result.point[0] == pytest.approx(target[0], abs=EXACT)
    assert result.point[1] == pytest.approx(target[1], abs=EXACT)
    assert result.reason == REASON_OK
    assert result.range_m == pytest.approx(
        math.hypot(target[0] - pose.x_m, target[1] - pose.y_m), abs=EXACT
    )


def test_round_trip_error_is_below_one_nanometre_across_the_valid_range(
    intrinsics: CameraIntrinsics, mount: CameraMount
) -> None:
    pose = Pose2D(3.0, -8.0, 0.7)
    worst = 0.0
    # Along-track and lateral offsets chosen so every target stays inside the 6 m gate:
    # hypot(5.5, 0.8) = 5.56 m. This test is about arithmetic exactness, not the gate.
    for range_m in (0.6, 1.0, 2.0, 3.0, 4.0, 5.0, 5.5):
        for lateral in (-0.8, 0.0, 0.8):
            x = pose.x_m + range_m * math.cos(pose.theta_rad) - lateral * math.sin(pose.theta_rad)
            y = pose.y_m + range_m * math.sin(pose.theta_rad) + lateral * math.cos(pose.theta_rad)
            pixel = project_ground_point(x, y, pose, intrinsics, mount)
            assert pixel is not None
            result = backproject_pixel(*pixel, pose, intrinsics, mount)
            assert result.valid, f"{range_m} m / {lateral} m: {result.reason}"
            assert result.point is not None
            worst = max(worst, math.hypot(result.point[0] - x, result.point[1] - y))
    assert worst < EXACT, f"worst round-trip error {worst!r} m"


def test_projection_of_a_point_behind_the_camera_returns_none(
    intrinsics: CameraIntrinsics, mount: CameraMount
) -> None:
    assert project_ground_point(-3.0, 0.0, Pose2D(0.0, 0.0, 0.0), intrinsics, mount) is None


# ---------------------------------------------------------------- horizon reject --
def _horizon_row(intrinsics: CameraIntrinsics, mount: CameraMount) -> float:
    """Image row where the ray through the image centre column is exactly horizontal.

    From ``d_w[z] = sin(pitch) - y * cos(pitch)`` with ``y = (v - cy)/fy``: zero at
    ``y = tan(pitch)``.
    """
    return intrinsics.cy + intrinsics.fy * math.tan(mount.pitch_rad)


def test_rays_above_the_horizon_are_rejected_and_carry_no_point(
    intrinsics: CameraIntrinsics, mount: CameraMount
) -> None:
    pose = Pose2D(10.0, 10.0, 0.4)
    horizon = _horizon_row(intrinsics, mount)
    rows = [0.0, 20.0, horizon - 40.0, horizon - 5.0, horizon - 0.5]
    for v in rows:
        assert camera_ray(intrinsics.cx, v, pose, intrinsics, mount)[2] >= 0.0
        result = backproject_pixel(intrinsics.cx, v, pose, intrinsics, mount)
        assert not result.valid
        assert result.reason == REASON_ABOVE_HORIZON
        assert result.point is None
        assert result.range_m == math.inf


def test_the_horizon_row_itself_is_rejected(
    intrinsics: CameraIntrinsics, mount: CameraMount
) -> None:
    """``d_w[z] == 0`` is on the reject side of the test, and floating-point residue near
    it cannot slip through either: a ray a hair below the horizon lands kilometres away and
    is stopped by the range gate."""
    pose = Pose2D(0.0, 0.0, 0.0)
    result = backproject_pixel(
        intrinsics.cx, _horizon_row(intrinsics, mount), pose, intrinsics, mount
    )
    assert not result.valid
    assert result.point is None
    assert result.reason in {REASON_ABOVE_HORIZON, REASON_BEYOND_MAX_RANGE}


def test_a_near_horizon_ray_is_rejected_rather_than_clamped(
    intrinsics: CameraIntrinsics, mount: CameraMount
) -> None:
    """The failure C11 exists to prevent: a plausible-looking point far away.

    A clamping implementation would return ``valid=True`` with a point at exactly the gate
    distance. This one returns the *true* enormous range and no point at all, so the
    rejection is visible in the data instead of being disguised as a measurement.
    """
    pose = Pose2D(0.0, 0.0, 0.0)
    v = _horizon_row(intrinsics, mount) + 1e-3
    result = backproject_pixel(intrinsics.cx, v, pose, intrinsics, mount)
    assert not result.valid
    assert result.point is None
    assert result.reason == REASON_BEYOND_MAX_RANGE
    assert result.range_m > 1_000.0, result.range_m
    assert math.isfinite(result.range_m)


def test_result_type_refuses_to_carry_a_point_it_should_not() -> None:
    with pytest.raises(ValueError, match="must carry a point"):
        GeolocResult(True, None, 1.0, REASON_OK)
    with pytest.raises(ValueError, match="must not carry a point"):
        GeolocResult(False, (1.0, 2.0), 1.0, REASON_ABOVE_HORIZON)


@pytest.mark.parametrize(("u", "v"), [(math.nan, 400.0), (320.0, math.inf), (math.inf, math.nan)])
def test_non_finite_pixels_are_rejected(
    u: float, v: float, intrinsics: CameraIntrinsics, mount: CameraMount
) -> None:
    result = backproject_pixel(u, v, Pose2D(0.0, 0.0, 0.0), intrinsics, mount)
    assert not result.valid
    assert result.reason == REASON_NON_FINITE
    assert result.point is None


# ------------------------------------------------------------------- range gate --
@pytest.mark.parametrize("range_m", [6.5, 8.0, 12.0, 40.0])
def test_detections_beyond_max_range_are_invalid_with_a_clear_reason(
    range_m: float, intrinsics: CameraIntrinsics, mount: CameraMount
) -> None:
    pose = Pose2D(0.0, 0.0, 0.0)
    pixel = project_ground_point(range_m, 0.0, pose, intrinsics, mount)
    assert pixel is not None
    result = backproject_pixel(*pixel, pose, intrinsics, mount)
    assert not result.valid
    assert result.reason == REASON_BEYOND_MAX_RANGE
    assert result.point is None
    # The range that failed the gate is still reported: docs/DATA_MODEL.md stores range_m
    # NOT NULL beside geoloc_valid, and a rejected detection at 6.5 m is a different fact
    # from a rejected detection at 40 m.
    assert result.range_m == pytest.approx(range_m, abs=EXACT)


def test_the_gate_is_configurable_and_the_default_is_six_metres(
    intrinsics: CameraIntrinsics, mount: CameraMount
) -> None:
    assert DEFAULT_MAX_RANGE_M == 6.0
    pose = Pose2D(0.0, 0.0, 0.0)
    pixel = project_ground_point(9.0, 0.0, pose, intrinsics, mount)
    assert pixel is not None
    assert not backproject_pixel(*pixel, pose, intrinsics, mount).valid
    assert backproject_pixel(*pixel, pose, intrinsics, mount, max_range_m=10.0).valid


# ------------------------------------------------------------- bottom-centre rule --
def test_geolocate_detection_uses_the_bbox_bottom_centre(
    intrinsics: CameraIntrinsics, mount: CameraMount
) -> None:
    """Not the box centre. The centre puts a pothole about half a box height too far away,
    and the wrong answer still looks like a point on the ground."""
    pose = Pose2D(2.0, 2.0, 0.3)
    bbox = BBox(300.0, 330.0, 380.0, 410.0)
    detection = Detection(
        kind=DetKind.POTHOLE,
        det_source=DetSource.SIMULATED,
        detector_name="fixture",
        confidence=0.9,
        severity=Severity.HIGH,
        bbox=bbox,
        stamp_ns=1_000,
    )
    from_detection = geolocate_detection(detection, pose, intrinsics, mount)
    from_bottom = backproject_pixel(*bbox.bottom_centre, pose, intrinsics, mount)
    centre_v = (bbox.y1 + bbox.y2) / 2.0
    from_centre = backproject_pixel(340.0, centre_v, pose, intrinsics, mount)

    assert from_detection == from_bottom
    assert from_detection.point is not None
    assert from_centre.point is not None
    assert from_centre.range_m > from_detection.range_m + 0.05


# ------------------------------------------------------------------ error budget --
def _recovered_point(
    range_m: float,
    truth: CameraMount,
    used: CameraMount,
    intrinsics: CameraIntrinsics,
    *,
    pose_truth: Pose2D,
    pose_used: Pose2D | None = None,
    pixel_dv: float = 0.0,
) -> tuple[float, float]:
    """Place a target at an exact ground range, then back-project it with wrong parameters.

    ``max_range_m`` is opened right up here on purpose: this measures how large the error
    *is*, and the 6 m gate exists precisely because that number stops being tolerable.
    """
    pixel = project_ground_point(range_m, 0.0, pose_truth, intrinsics, truth)
    assert pixel is not None
    u, v = pixel
    result = backproject_pixel(
        u,
        v + pixel_dv,
        pose_used if pose_used is not None else pose_truth,
        intrinsics,
        used,
        max_range_m=10_000.0,
    )
    assert result.valid, result.reason
    assert result.point is not None
    return result.point


def test_error_grows_quadratically_with_ground_range(
    intrinsics: CameraIntrinsics, mount: CameraMount
) -> None:
    """A fixed 1-degree pitch error, swept over range: ``error / range^2`` stays put.

    This is the documented property, measured against the implementation rather than
    asserted from the closed form. Measured here: 0.177 m at 2 m rising to 1.830 m at 6 m,
    a ratio of 0.0442 to 0.0508 -- a drift of about 15% across the whole valid range, while
    the error itself grows more than tenfold. The drift is upward because tilting the ray
    toward the horizon makes the plane intersection more sensitive, so the far side of the
    error is a little worse than a pure square law rather than a little better.
    """
    pose = Pose2D(0.0, 0.0, 0.0)
    wrong = CameraMount(height_m=mount.height_m, pitch_rad=mount.pitch_rad + ONE_DEGREE)

    ranges = (2.0, 3.0, 4.0, 5.0, 6.0)
    errors: list[float] = []
    ratios: list[float] = []
    for range_m in ranges:
        x, y = _recovered_point(range_m, mount, wrong, intrinsics, pose_truth=pose)
        error = math.hypot(x - range_m, y - 0.0)
        errors.append(error)
        ratios.append(error / range_m**2)

    # Constant-ish ratio is the quadratic claim.
    assert max(ratios) / min(ratios) < 1.30, dict(zip(ranges, ratios, strict=True))
    # And it is genuinely quadratic, not linear: doubling the range roughly quadruples the
    # error. A linear error law would give 2.0 here.
    assert errors[-1] / errors[1] > 3.0, dict(zip(ranges, errors, strict=True))
    # Monotone, and large enough at the gate to justify the gate.
    assert errors == sorted(errors)
    assert errors[-1] > 1.0, errors


def test_pitch_error_dominates_the_other_error_sources(
    intrinsics: CameraIntrinsics, mount: CameraMount
) -> None:
    """Ranked at a fixed 5 m, one plausible unit of each source.

    Each perturbation is a realistic single unit of its own kind -- 1 degree of pitch,
    2 px of bbox bottom edge, 2 cm of mount height, 1 degree of yaw -- so the comparison is
    'which of these hurts most', not 'which knob did I turn hardest'. Pitch wins by a
    factor of three or more, which is why attitude heads the documented ranking and why the
    valid range is short.
    """
    range_m = 5.0
    pose = Pose2D(0.0, 0.0, 0.0)
    truth = (range_m, 0.0)

    def err(point: tuple[float, float]) -> float:
        return math.hypot(point[0] - truth[0], point[1] - truth[1])

    pitch = err(
        _recovered_point(
            range_m,
            mount,
            CameraMount(mount.height_m, mount.pitch_rad + ONE_DEGREE),
            intrinsics,
            pose_truth=pose,
        )
    )
    bbox_px = err(
        _recovered_point(range_m, mount, mount, intrinsics, pose_truth=pose, pixel_dv=2.0)
    )
    height = err(
        _recovered_point(
            range_m,
            mount,
            CameraMount(mount.height_m + 0.02, mount.pitch_rad),
            intrinsics,
            pose_truth=pose,
        )
    )
    yaw = err(
        _recovered_point(
            range_m,
            mount,
            mount,
            intrinsics,
            pose_truth=pose,
            pose_used=Pose2D(pose.x_m, pose.y_m, pose.theta_rad + ONE_DEGREE),
        )
    )

    ranked = {"pitch_1deg": pitch, "bbox_2px": bbox_px, "height_2cm": height, "yaw_1deg": yaw}
    assert pitch > 3.0 * bbox_px, ranked
    assert pitch > 3.0 * height, ranked
    assert pitch > 3.0 * yaw, ranked
    assert min(ranked.values()) > 0.0, ranked


def test_a_lower_mounted_camera_has_a_worse_error_budget(intrinsics: CameraIntrinsics) -> None:
    """Halving the mount height roughly doubles the error for the same pitch error.

    Stated because it is the one design lever available: the gate distance is not a free
    choice, it follows from where the camera sits.
    """
    pose = Pose2D(0.0, 0.0, 0.0)
    range_m = 4.0
    errors: dict[float, float] = {}
    for height in (0.9, 0.45):
        truth = CameraMount(height_m=height, pitch_rad=math.radians(-15.0))
        wrong = CameraMount(height_m=height, pitch_rad=truth.pitch_rad + ONE_DEGREE)
        x, y = _recovered_point(range_m, truth, wrong, intrinsics, pose_truth=pose)
        errors[height] = math.hypot(x - range_m, y)
    assert errors[0.45] > 1.7 * errors[0.9], errors

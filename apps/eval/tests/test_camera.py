"""The forward projection must be the exact inverse of the documented back-projection.

The oracle below is the back-projection formula from ``docs/ARCHITECTURE.md`` section 5,
transcribed here and nowhere else. It lives in the test tree on purpose: the harness must
not ship its own copy of the stage it is grading, but a test needs an independent way to
check that ``parkscan_eval.camera`` speaks the same geometry the perception package does.
If the two ever diverge, this test fails before ``run_eval`` can report a flattering zero.
"""

from __future__ import annotations

import math

import pytest
from parkscan_eval.camera import (
    CameraModel,
    footprint_bbox,
    ground_range_m,
    project_ground_point,
)

from parkscan_core.types import Pose2D


def back_project(u: float, v: float, pose: Pose2D, camera: CameraModel) -> tuple[float, float]:
    """docs/ARCHITECTURE.md section 5, verbatim. Raises if the ray is at/above the horizon."""
    dx_opt = (u - camera.cx) / camera.fx
    dy_opt = (v - camera.cy) / camera.fy
    dz_opt = 1.0

    # R_c_opt: optical -> body (x forward, y left, z up).
    x_body, y_body, z_body = dz_opt, -dx_opt, -dy_opt

    # Ry(-pitch).
    cos_p, sin_p = math.cos(-camera.pitch_rad), math.sin(-camera.pitch_rad)
    x_r = x_body * cos_p + z_body * sin_p
    y_r = y_body
    z_r = -x_body * sin_p + z_body * cos_p

    # Rz(theta).
    cos_t, sin_t = math.cos(pose.theta_rad), math.sin(pose.theta_rad)
    dw_x = x_r * cos_t - y_r * sin_t
    dw_y = x_r * sin_t + y_r * cos_t
    dw_z = z_r

    if dw_z >= 0.0:
        msg = "ray at or above the horizon"
        raise ValueError(msg)

    t = -camera.height_m / dw_z
    return (pose.x_m + t * dw_x, pose.y_m + t * dw_y)


def camera(
    *,
    width: int = 640,
    height: int = 480,
    fx: float = 457.0073621574767,
    fy: float = 457.0073621574767,
    cx: float = 320.0,
    cy: float = 240.0,
    height_m: float = 0.45,
    pitch_rad: float = -0.2617993877991494,
) -> CameraModel:
    """The camera the perception package's own defaults describe.

    640x480, a 70-degree horizontal FOV, 0.45 m above the ground, tilted 15 degrees down.
    Written out here rather than imported from ``parkscan_perception`` so that this test
    checks the harness's geometry against a transcription of the *document*, not against
    whatever the perception package happens to default to today.
    """
    return CameraModel(
        width=width,
        height=height,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        height_m=height_m,
        pitch_rad=pitch_rad,
    )


CAMERA = camera()

POSES = [
    Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0),
    Pose2D(x_m=12.5, y_m=5.0, theta_rad=0.0),
    Pose2D(x_m=12.5, y_m=5.0, theta_rad=math.pi / 2),
    Pose2D(x_m=40.0, y_m=30.0, theta_rad=-2.3),
    Pose2D(x_m=7.25, y_m=61.5, theta_rad=3.0),
]


@pytest.mark.parametrize("pose", POSES)
@pytest.mark.parametrize("forward_m", [0.9, 1.6, 3.0, 5.5, 9.0, 14.0])
@pytest.mark.parametrize("lateral_m", [-1.5, 0.0, 0.75])
def test_forward_projection_round_trips_through_the_architecture_formula(
    pose: Pose2D, forward_m: float, lateral_m: float
) -> None:
    # A point placed in the robot's own frame, then expressed in the world.
    cos_t, sin_t = math.cos(pose.theta_rad), math.sin(pose.theta_rad)
    truth = (
        pose.x_m + forward_m * cos_t - lateral_m * sin_t,
        pose.y_m + forward_m * sin_t + lateral_m * cos_t,
    )

    projection = project_ground_point(truth, pose, CAMERA)
    assert projection.in_front

    recovered = back_project(projection.u, projection.v, pose, CAMERA)
    assert math.dist(recovered, truth) < 1e-9


def test_a_point_straight_ahead_lands_on_the_vertical_centre_line() -> None:
    pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    projection = project_ground_point((4.0, 0.0), pose, CAMERA)
    assert projection.u == pytest.approx(CAMERA.cx)
    assert projection.in_image


def test_the_optical_axis_meets_the_ground_at_the_image_centre() -> None:
    """A negative pitch is a downward tilt; ``-h / tan(pitch)`` is where the axis lands."""
    pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    axis_range_m = -CAMERA.height_m / math.tan(CAMERA.pitch_rad)
    assert axis_range_m == pytest.approx(1.6791, abs=1e-3)

    on_axis = project_ground_point((axis_range_m, 0.0), pose, CAMERA)
    assert on_axis.v == pytest.approx(CAMERA.cy)

    # Nearer than the axis intersection is below the centre; further is above it.
    assert project_ground_point((1.0, 0.0), pose, CAMERA).v > CAMERA.cy
    assert project_ground_point((6.0, 0.0), pose, CAMERA).v < CAMERA.cy


def test_a_point_further_away_projects_higher_in_the_image() -> None:
    pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    near = project_ground_point((2.0, 0.0), pose, CAMERA)
    far = project_ground_point((8.0, 0.0), pose, CAMERA)
    assert far.v < near.v


def test_a_point_to_the_left_projects_left_of_centre() -> None:
    pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    projection = project_ground_point((4.0, 1.0), pose, CAMERA)
    assert projection.u < CAMERA.cx


def test_a_point_behind_the_camera_has_no_pixel() -> None:
    pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    projection = project_ground_point((-4.0, 0.0), pose, CAMERA)
    assert not projection.in_front
    assert not projection.visible
    assert math.isnan(projection.u)


def test_points_beyond_the_horizon_leave_the_top_of_the_image() -> None:
    """The horizon is a hard limit: nothing on the ground projects above it."""
    pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    # v of the horizon for this mounting: the limit of v as the range goes to infinity.
    horizon_v = CAMERA.cy + CAMERA.fy * math.tan(CAMERA.pitch_rad)
    for range_m in (5.0, 50.0, 500.0, 5000.0):
        projection = project_ground_point((range_m, 0.0), pose, CAMERA)
        assert projection.v > horizon_v


def test_ground_range_is_horizontal_distance_only() -> None:
    pose = Pose2D(x_m=1.0, y_m=2.0, theta_rad=1.1)
    assert ground_range_m(pose, (4.0, 6.0)) == pytest.approx(5.0)


def test_footprint_bbox_bottom_centre_is_the_projected_truth() -> None:
    pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    truth = (3.5, 0.4)
    projection = project_ground_point(truth, pose, CAMERA)
    bbox = footprint_bbox(truth, 0.4, pose, CAMERA)
    assert bbox is not None
    u, v = bbox.bottom_centre
    assert u == pytest.approx(projection.u)
    assert v == pytest.approx(projection.v)


def test_footprint_bbox_shrinks_with_range() -> None:
    pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    near = footprint_bbox((2.0, 0.0), 0.4, pose, CAMERA)
    far = footprint_bbox((8.0, 0.0), 0.4, pose, CAMERA)
    assert near is not None
    assert far is not None
    assert far.width < near.width
    assert far.height < near.height


def test_footprint_bbox_is_none_behind_the_camera() -> None:
    pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    assert footprint_bbox((-2.0, 0.0), 0.4, pose, CAMERA) is None


def test_camera_model_validates_its_parameters() -> None:
    with pytest.raises(ValueError, match="resolution must be positive"):
        camera(width=0)
    with pytest.raises(ValueError, match="resolution must be positive"):
        camera(height=-1)
    with pytest.raises(ValueError, match="focal lengths must be positive"):
        camera(fx=0.0)
    with pytest.raises(ValueError, match="focal lengths must be positive"):
        camera(fy=-1.0)
    with pytest.raises(ValueError, match="height_m must be finite and positive"):
        camera(height_m=0.0)
    with pytest.raises(ValueError, match=r"pitch_rad must be finite"):
        camera(pitch_rad=math.pi)


def test_footprint_bbox_rejects_a_non_positive_size() -> None:
    with pytest.raises(ValueError, match="size_m must be positive"):
        footprint_bbox((3.0, 0.0), 0.0, Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0), CAMERA)

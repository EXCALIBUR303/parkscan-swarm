"""Kinematics: the integrator is checked against closed-form solutions, not against itself."""

from __future__ import annotations

import math

import pytest
from parkscan_sim.kinematics import TwistLimits, clamp_twist, integrate_pose, wrap_angle

from parkscan_core.types import Pose2D, TwistCmd

DT = 0.01
V = 1.0
OMEGA = 0.5
RADIUS = V / OMEGA  # 2.0 m


def _euler(pose: Pose2D, cmd: TwistCmd, dt: float) -> Pose2D:
    """Explicit Euler, for comparison only. Never used by the simulator."""
    return Pose2D(
        x_m=pose.x_m + cmd.v_mps * math.cos(pose.theta_rad) * dt,
        y_m=pose.y_m + cmd.v_mps * math.sin(pose.theta_rad) * dt,
        theta_rad=wrap_angle(pose.theta_rad + cmd.omega_radps * dt),
    )


def _closed_form_arc(steps: int) -> tuple[float, float, float]:
    """Exact unicycle solution from (0, 0, 0) under constant (V, OMEGA)."""
    t = steps * DT
    angle = OMEGA * t
    return (RADIUS * math.sin(angle), RADIUS - RADIUS * math.cos(angle), angle)


# ------------------------------------------------------------------------- basics --
def test_zero_command_leaves_the_pose_bit_identical() -> None:
    pose = Pose2D(x_m=12.25, y_m=-3.5, theta_rad=0.7)
    result = integrate_pose(pose, TwistCmd(v_mps=0.0, omega_radps=0.0), 0.05)
    assert result.x_m == pose.x_m
    assert result.y_m == pose.y_m
    assert result.theta_rad == pose.theta_rad


def test_straight_line_along_x_never_leaves_the_axis() -> None:
    pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    cmd = TwistCmd(v_mps=1.2, omega_radps=0.0)
    for _ in range(100):
        pose = integrate_pose(pose, cmd, 0.05)
    # sin(0.0) is exactly 0.0, so y accumulates exactly nothing: this is an equality,
    # not an approximation, and it would break the moment a heading term crept in.
    assert pose.y_m == 0.0
    assert pose.theta_rad == 0.0
    assert pose.x_m == pytest.approx(6.0, abs=1e-12)


def test_straight_line_on_a_diagonal_splits_evenly() -> None:
    pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=math.pi / 4.0)
    cmd = TwistCmd(v_mps=1.0, omega_radps=0.0)
    for _ in range(50):
        pose = integrate_pose(pose, cmd, 0.02)
    assert pose.x_m == pytest.approx(math.cos(math.pi / 4.0), abs=1e-12)
    assert pose.y_m == pytest.approx(math.sin(math.pi / 4.0), abs=1e-12)


def test_pure_rotation_does_not_translate() -> None:
    pose = Pose2D(x_m=5.0, y_m=5.0, theta_rad=0.0)
    cmd = TwistCmd(v_mps=0.0, omega_radps=1.0)
    for _ in range(200):
        pose = integrate_pose(pose, cmd, 0.01)
    assert pose.x_m == 5.0
    assert pose.y_m == 5.0
    assert pose.theta_rad == pytest.approx(2.0, abs=1e-12)


# ---------------------------------------------------------------------- the arc --
def test_constant_omega_arc_matches_the_closed_form_circle() -> None:
    """A quarter turn of a 2 m circle, against the analytic solution."""
    steps = round((math.pi / 2.0 / OMEGA) / DT)
    pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    cmd = TwistCmd(v_mps=V, omega_radps=OMEGA)
    for _ in range(steps):
        pose = integrate_pose(pose, cmd, DT)

    exact_x, exact_y, exact_theta = _closed_form_arc(steps)
    assert math.dist((pose.x_m, pose.y_m), (exact_x, exact_y)) < 1e-5
    assert pose.theta_rad == pytest.approx(exact_theta, abs=1e-12)


def test_arc_stays_on_the_circle_within_the_predicted_second_order_bias() -> None:
    """Midpoint traces an exact regular polygon, so its radius error is a bounded bias.

    The predicted circumradius is ``R * (w*dt/2) / sin(w*dt/2)``, i.e. a relative bias of
    ``(w*dt)^2 / 24`` with no secular drift. Samples therefore sit between ``R`` and
    ``R + 2 * bias``. The bound below is that prediction, not a tolerance tuned to pass.
    """
    steps = round((2.0 * math.pi / OMEGA) / DT)  # a full circle
    bound = RADIUS * (OMEGA * DT) ** 2 / 12.0 * 1.01
    centre = (0.0, RADIUS)

    pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    cmd = TwistCmd(v_mps=V, omega_radps=OMEGA)
    worst = 0.0
    for _ in range(steps):
        pose = integrate_pose(pose, cmd, DT)
        worst = max(worst, abs(math.dist((pose.x_m, pose.y_m), centre) - RADIUS))

    assert worst < bound
    # And the bias is real, not zero -- this is a second-order method, not an exact one.
    assert worst > RADIUS * (OMEGA * DT) ** 2 / 24.0


def test_midpoint_is_orders_of_magnitude_better_than_euler_on_the_same_arc() -> None:
    """The reason integrate_pose is RK2. Euler spirals outward; midpoint does not."""
    steps = round((math.pi / 2.0 / OMEGA) / DT)
    cmd = TwistCmd(v_mps=V, omega_radps=OMEGA)
    exact_x, exact_y, _ = _closed_form_arc(steps)

    mid = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    euler = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    for _ in range(steps):
        mid = integrate_pose(mid, cmd, DT)
        euler = _euler(euler, cmd, DT)

    mid_err = math.dist((mid.x_m, mid.y_m), (exact_x, exact_y))
    euler_err = math.dist((euler.x_m, euler.y_m), (exact_x, exact_y))
    assert euler_err > 1e-3  # ~7 mm at dt = 0.01 -- visible, and it grows with dt
    assert euler_err > 100.0 * mid_err


# ------------------------------------------------------------------------- wrap --
@pytest.mark.parametrize(
    ("raw", "expected"),
    [
        (0.0, 0.0),
        (math.pi, math.pi),
        (-math.pi, -math.pi),
        (2.0 * math.pi, 0.0),
        (3.0 * math.pi, -math.pi),
        (5.0, 5.0 - 2.0 * math.pi),
        (-5.0, 2.0 * math.pi - 5.0),
    ],
)
def test_wrap_angle(raw: float, expected: float) -> None:
    assert wrap_angle(raw) == pytest.approx(expected, abs=1e-15)


def test_wrap_angle_range() -> None:
    for i in range(-2000, 2000):
        wrapped = wrap_angle(i * 0.01)
        assert -math.pi <= wrapped <= math.pi


# ------------------------------------------------------------------------ clamp --
def test_clamp_twist_saturates_both_components_including_reverse() -> None:
    limits = TwistLimits(max_v_mps=1.2, max_omega_radps=1.5)
    assert clamp_twist(TwistCmd(v_mps=9.0, omega_radps=9.0), limits) == TwistCmd(1.2, 1.5)
    assert clamp_twist(TwistCmd(v_mps=-9.0, omega_radps=-9.0), limits) == TwistCmd(-1.2, -1.5)


def test_clamp_twist_passes_an_in_range_command_through_unchanged() -> None:
    limits = TwistLimits()
    cmd = TwistCmd(v_mps=0.31, omega_radps=-0.77)
    assert clamp_twist(cmd, limits) == cmd


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_clamp_twist_rejects_non_finite_commands(bad: float) -> None:
    with pytest.raises(ValueError, match="finite"):
        clamp_twist(TwistCmd(v_mps=bad, omega_radps=0.0), TwistLimits())
    with pytest.raises(ValueError, match="finite"):
        clamp_twist(TwistCmd(v_mps=0.0, omega_radps=bad), TwistLimits())


@pytest.mark.parametrize("bad", [0.0, -1.0, math.nan, math.inf])
def test_twist_limits_reject_nonsense(bad: float) -> None:
    with pytest.raises(ValueError, match=r"max_v_mps|max_omega_radps"):
        TwistLimits(max_v_mps=bad)
    with pytest.raises(ValueError, match=r"max_v_mps|max_omega_radps"):
        TwistLimits(max_omega_radps=bad)


def test_integrate_pose_rejects_a_negative_or_non_finite_dt() -> None:
    pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    cmd = TwistCmd(v_mps=1.0, omega_radps=0.0)
    for bad in (-0.01, math.nan, math.inf):
        with pytest.raises(ValueError, match="dt_s"):
            integrate_pose(pose, cmd, bad)

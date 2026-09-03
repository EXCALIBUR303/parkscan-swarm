"""Differential-drive (unicycle) kinematics.

What this is, plainly
---------------------
Planar kinematics on a flat ground plane, integrated one fixed tick at a time. There is
no mass, no inertia, no wheel slip, no motor model, no suspension and no contact physics
here. A commanded twist is realised immediately, saturated at the configured limits.
That is the honest description of ``sim-lite`` (ADR-0001): it exists to be fast and
bit-reproducible so the 30-seed metrics harness can run, not to predict how a real robot
would move.

The state is REP-103 throughout: ``x`` forward/east, ``y`` left/north, ``theta``
counter-clockwise from ``+x``, metres and radians.

The continuous model is the unicycle::

    x'     = v * cos(theta)
    y'     = v * sin(theta)
    theta' = omega

Every function here is pure: same arguments, same result, no state, no RNG, no clock.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from parkscan_core.types import Pose2D, TwistCmd

__all__ = ["TwistLimits", "clamp_twist", "integrate_pose", "wrap_angle"]

_TWO_PI = 2.0 * math.pi


@dataclass(frozen=True, slots=True)
class TwistLimits:
    """Saturation applied to a commanded twist before it is integrated.

    Reverse is allowed: ``v`` is clamped to ``[-max_v_mps, +max_v_mps]``. The limits are
    hard saturation, not an acceleration model -- ``sim-lite`` has no acceleration model,
    and pretending otherwise would be an overclaim.
    """

    max_v_mps: float = 1.2
    max_omega_radps: float = 1.5

    def __post_init__(self) -> None:
        if not math.isfinite(self.max_v_mps) or self.max_v_mps <= 0.0:
            msg = f"max_v_mps must be finite and positive, got {self.max_v_mps}"
            raise ValueError(msg)
        if not math.isfinite(self.max_omega_radps) or self.max_omega_radps <= 0.0:
            msg = f"max_omega_radps must be finite and positive, got {self.max_omega_radps}"
            raise ValueError(msg)


def wrap_angle(theta_rad: float) -> float:
    """Wrap an angle into ``[-pi, +pi]``.

    ``math.remainder`` is used rather than a ``fmod``/``if`` chain because it is
    correctly rounded and exact for the boundary cases, so wrapping never introduces a
    rounding difference that a determinism test would then have to tolerate.
    """
    return math.remainder(theta_rad, _TWO_PI)


def clamp_twist(cmd: TwistCmd, limits: TwistLimits) -> TwistCmd:
    """Saturate ``cmd`` to ``limits``. Non-finite components are rejected, not clamped.

    A ``nan`` command would silently poison the pose for the rest of the run and every
    downstream comparison against it would quietly succeed, so it raises instead.
    """
    if not math.isfinite(cmd.v_mps) or not math.isfinite(cmd.omega_radps):
        msg = f"twist command must be finite, got {cmd!r}"
        raise ValueError(msg)
    v = max(-limits.max_v_mps, min(limits.max_v_mps, cmd.v_mps))
    omega = max(-limits.max_omega_radps, min(limits.max_omega_radps, cmd.omega_radps))
    return TwistCmd(v_mps=v, omega_radps=omega)


def integrate_pose(pose: Pose2D, cmd: TwistCmd, dt_s: float) -> Pose2D:
    """Advance ``pose`` by ``dt_s`` under a constant ``cmd``, using midpoint (RK2).

    Why midpoint and not Euler
    --------------------------
    Explicit Euler evaluates ``cos``/``sin`` at the heading *at the start* of the tick,
    so every step leaves along the tangent of the arc and lands outside it. The error is
    first order and it does not cancel: on a constant-``omega`` arc the integrated path
    visibly spirals outward. Measured here, ``v = 1.0 m/s``, ``omega = 0.5 rad/s``
    (a 2 m circle), quarter turn:

    ==========  ==============  ==============
    ``dt``      Euler error     midpoint error
    ==========  ==============  ==============
    0.05 s      35.4 mm         0.074 mm
    0.01 s      7.07 mm         0.0029 mm
    ==========  ==============  ==============

    A 35 mm cross-track error per quarter turn is larger than the tolerance the sweep
    planner and the collision check care about, so Euler would be showing us an artefact
    of the integrator rather than a property of the controller.

    Midpoint evaluates the heading at the half tick. For a constant ``(v, omega)`` the
    samples it produces are *exactly* the vertices of a regular polygon: each step is a
    chord of length ``v * dt`` whose bisector is the mid-heading, and the heading advances
    by exactly ``omega * dt``. The circumradius of that polygon is

        ``R_poly = R * (omega * dt / 2) / sin(omega * dt / 2) = R * (1 + (omega*dt)^2/24 + ...)``

    -- a second-order *bias* with no secular drift, rather than Euler's accumulating
    outward spiral. ``tests/test_kinematics.py`` asserts exactly this against the
    closed-form circle.

    The pure-rotation case (``v == 0``) and the straight-line case (``omega == 0``) both
    fall out of the same expression with no special-casing, which is why there is none.
    """
    if not math.isfinite(dt_s) or dt_s < 0.0:
        msg = f"dt_s must be finite and non-negative, got {dt_s}"
        raise ValueError(msg)
    theta_mid = pose.theta_rad + 0.5 * cmd.omega_radps * dt_s
    return Pose2D(
        x_m=pose.x_m + cmd.v_mps * math.cos(theta_mid) * dt_s,
        y_m=pose.y_m + cmd.v_mps * math.sin(theta_mid) * dt_s,
        theta_rad=wrap_angle(pose.theta_rad + cmd.omega_radps * dt_s),
    )

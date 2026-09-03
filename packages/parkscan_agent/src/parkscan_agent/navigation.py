"""Waypoint following and local obstacle avoidance.

The layering, stated once
-------------------------
Three layers, and the boundary between them is the whole point:

1. **The planner** (:mod:`parkscan_agent.lanegraph`) decides *where to go*: an ordered
   list of lane nodes. It owns the route.
2. **The controller** (:func:`waypoint_command` here) turns "my pose, my next point" into
   a :class:`~parkscan_core.types.TwistCmd`. It has no memory and no map.
3. **Local avoidance** (:func:`avoid` here) looks at the planar range scan and may
   **only slow the robot down or bend its yaw rate**. It can never pick a different
   waypoint, drop one, or re-order the route.

That last restriction is deliberate and is asserted by a test rather than promised in a
comment. A reactive layer allowed to re-plan is how a fleet silently stops covering the
zones the coordinator believes it is covering: the map, the coverage accounting and the
allocator would all keep reasoning about a route the robot abandoned. When avoidance is
not enough, the honest outcome is that the robot slows to a stop and the *planner* is told
-- which in this system surfaces as the coordinator's stuck detection.

What this is, precisely
-----------------------
:func:`waypoint_command` is a pure-pursuit-style controller: the steering term is the
classic pure-pursuit curvature ``2 sin(alpha) / L`` toward a lookahead point, plus a
proportional heading term so the robot still rotates when the speed profile has driven it
to a standstill. Textbook pure pursuit has no heading term and cannot turn on the spot;
saying "pure pursuit" without that caveat would be an overclaim.

There is **no dynamics model here**. This layer emits a desired body twist; the backend
applies its own acceleration and saturation limits, and ``sense()`` reports the twist that
actually resulted. Nothing in this module simulates a robot.

The range-scan layout
---------------------
:class:`~parkscan_core.types.SensorFrame` fixes the range count and says the layout is
constant for a run, but the *angular* layout is a backend detail, so it is configuration
here rather than an assumption baked into the maths. :attr:`NavConfig.scan_fov_rad`
describes a scan that is symmetric about the robot's ``+x`` axis with beams at
``-fov/2 + i * fov/(n-1)``. If a backend uses another layout, set this to match; if it
were guessed wrong, avoidance would deflect the wrong way, which is exactly the sort of
silent, plausible-looking failure worth one config field.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from parkscan_core.types import Pose2D, TwistCmd

__all__ = [
    "NavConfig",
    "RouteProgress",
    "avoid",
    "follow_route",
    "has_arrived",
    "scan_bearings",
    "waypoint_command",
    "wrap_angle",
]

type Point2D = tuple[float, float]

STOP = TwistCmd(v_mps=0.0, omega_radps=0.0)
"""The command a robot issues when it has nothing to do. Named so it reads as intent."""


@dataclass(frozen=True, slots=True)
class NavConfig:
    """Controller tuning. Frozen, so a run's behaviour is a value you can log and diff.

    Defaults are sized for the shipped lot: 6 m aisles, 2.5 m stall pitch, a small
    differential-drive inspection robot at walking pace.
    """

    max_v_mps: float = 1.2
    """Top forward speed. The speed profile scales this down; nothing scales it up."""

    max_omega_radps: float = 1.5
    """Yaw-rate saturation, applied to the planner term and again after avoidance."""

    lookahead_m: float = 2.5
    """Pure-pursuit lookahead ``L``. Larger cuts corners more and oscillates less."""

    min_lookahead_m: float = 0.5
    """Floor on ``L`` so curvature stays finite as the goal is approached."""

    arrival_tolerance_m: float = 0.25
    """A waypoint counts as reached inside this radius.

    This is the *final* tolerance, used for the last point of a route and by
    :func:`has_arrived`. It is deliberately looser than the docking tolerance in
    :mod:`parkscan_agent.docking`: stopping a sweep pass within 25 cm is fine, parking on
    a charging contact is not.
    """

    waypoint_tolerance_m: float = 0.6
    """Acceptance radius for *intermediate* waypoints.

    Looser than :attr:`arrival_tolerance_m` on purpose: an intermediate lane node is a
    corner to round, not a place to stop, and demanding the tight radius at every corner
    makes the robot brake and pirouette at each one.
    """

    slow_radius_m: float = 1.5
    """Distance at which the speed profile begins ramping down linearly to the goal."""

    heading_gain: float = 1.6
    """Proportional gain on heading error, in rad/s per rad."""

    turn_in_place_rad: float = 1.0
    """Above this heading error the robot stops and rotates instead of driving an arc."""

    scan_fov_rad: float = math.pi
    """Angular span of the planar range scan, centred on ``+x``. See the module docstring."""

    avoid_sector_rad: float = 1.05
    """Half-angle of the forward sector avoidance looks at (~60 degrees each side)."""

    avoid_stop_m: float = 0.45
    """Forward clearance below which commanded speed is forced to zero."""

    avoid_slow_m: float = 1.6
    """Forward clearance below which speed is scaled down and deflection kicks in."""

    avoid_gain: float = 1.2
    """Strength of the yaw deflection away from the nearest obstacle, in rad/s."""

    def __post_init__(self) -> None:
        if self.max_v_mps <= 0.0 or self.max_omega_radps <= 0.0:
            msg = "max_v_mps and max_omega_radps must be positive"
            raise ValueError(msg)
        if not 0.0 < self.min_lookahead_m <= self.lookahead_m:
            msg = "need 0 < min_lookahead_m <= lookahead_m"
            raise ValueError(msg)
        if self.arrival_tolerance_m <= 0.0 or self.waypoint_tolerance_m <= 0.0:
            msg = "tolerances must be positive"
            raise ValueError(msg)
        if self.slow_radius_m <= 0.0:
            msg = "slow_radius_m must be positive"
            raise ValueError(msg)
        if not 0.0 <= self.avoid_stop_m <= self.avoid_slow_m:
            msg = "need 0 <= avoid_stop_m <= avoid_slow_m"
            raise ValueError(msg)
        if self.scan_fov_rad <= 0.0:
            msg = "scan_fov_rad must be positive"
            raise ValueError(msg)


# ------------------------------------------------------------------------- helpers --
def wrap_angle(theta_rad: float) -> float:
    """Wrap an angle to ``(-pi, pi]``."""
    return math.remainder(theta_rad, math.tau)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def has_arrived(pose: Pose2D, target: Point2D, tolerance_m: float) -> bool:
    """Whether ``pose`` is within ``tolerance_m`` of ``target`` in position only.

    Heading is ignored: a sweep waypoint is a place to pass through. Docking, which does
    care about heading, has its own check in :mod:`parkscan_agent.docking`.
    """
    return math.dist((pose.x_m, pose.y_m), target) <= tolerance_m


def scan_bearings(count: int, fov_rad: float) -> tuple[float, ...]:
    """Body-frame bearings of a ``count``-beam scan spanning ``fov_rad`` around ``+x``.

    Beams are evenly spaced with both endpoints included, matching the common
    ``angle_min + i * angle_increment`` convention. A single-beam scan is treated as
    dead ahead.
    """
    if count <= 0:
        return ()
    if count == 1:
        return (0.0,)
    step = fov_rad / (count - 1)
    return tuple(-fov_rad / 2.0 + i * step for i in range(count))


# ---------------------------------------------------------------------- controller --
def waypoint_command(pose: Pose2D, target: Point2D, cfg: NavConfig) -> TwistCmd:
    """Body twist that drives ``pose`` toward ``target``.

    The speed profile has three multiplicative parts, each with a reason:

    * ``min(1, distance / slow_radius_m)`` -- ramp down near the goal so the robot stops
      on it instead of circling it.
    * ``max(0, cos(alpha))`` -- do not drive fast in a direction that is badly wrong; the
      cosine falls to zero at 90 degrees of heading error.
    * a hard cut to zero beyond :attr:`NavConfig.turn_in_place_rad` -- a differential-drive
      robot with a large heading error should pivot, not swing out into the next aisle.

    Returns :data:`STOP` once inside :attr:`NavConfig.arrival_tolerance_m`, so a caller
    that keeps calling after arrival simply holds position.
    """
    dx = target[0] - pose.x_m
    dy = target[1] - pose.y_m
    distance = math.hypot(dx, dy)
    if distance <= cfg.arrival_tolerance_m:
        return STOP

    alpha = wrap_angle(math.atan2(dy, dx) - pose.theta_rad)

    speed = cfg.max_v_mps * min(1.0, distance / cfg.slow_radius_m)
    speed *= max(0.0, math.cos(alpha))
    if abs(alpha) > cfg.turn_in_place_rad:
        speed = 0.0

    lookahead = min(cfg.lookahead_m, max(distance, cfg.min_lookahead_m))
    curvature = 2.0 * math.sin(alpha) / lookahead
    omega = _clamp(curvature * speed + cfg.heading_gain * alpha, cfg.max_omega_radps)
    return TwistCmd(v_mps=speed, omega_radps=omega)


# ----------------------------------------------------------------------- avoidance --
def avoid(cmd: TwistCmd, ranges: Sequence[float], cfg: NavConfig) -> TwistCmd:
    """Apply local avoidance to a planned command.

    Contract, and it is narrow on purpose:

    * ``0 <= v_out <= v_in`` -- avoidance only ever slows the robot, never speeds it up
      and never reverses it. Reversing would take the robot off the planned route, which
      is the planner's decision to make, not this layer's.
    * ``|omega_out| <= max_omega_radps`` -- the deflection is bounded, so a noisy scan
      cannot spin the robot.
    * The route is untouched; this function does not even see it.

    Beams outside the forward sector are ignored (an obstacle behind you is not in the
    way), as are ``inf`` returns. The nearest beam in the sector sets the speed scale; the
    deflection pushes yaw away from every close beam, weighted by how close it is, so a
    wall on the left and a wall on the right partially cancel and the robot threads
    between them rather than committing to one side.
    """
    bearings = scan_bearings(len(ranges), cfg.scan_fov_rad)
    nearest = math.inf
    deflection = 0.0
    for bearing, distance in zip(bearings, ranges, strict=True):
        if abs(bearing) > cfg.avoid_sector_rad or not math.isfinite(distance):
            continue
        if distance < nearest:
            nearest = distance
        if distance < cfg.avoid_slow_m:
            urgency = 1.0 - distance / cfg.avoid_slow_m
            # Push away from the obstacle's side. A beam dead ahead (bearing 0) gives no
            # preferred side, so it contributes speed reduction only -- deliberate: an
            # arbitrary tie-break there would make the choice depend on float noise.
            deflection -= math.copysign(urgency, bearing) if bearing != 0.0 else 0.0

    if not math.isfinite(nearest) or nearest >= cfg.avoid_slow_m:
        return cmd

    if nearest <= cfg.avoid_stop_m:
        scale = 0.0
    else:
        span = cfg.avoid_slow_m - cfg.avoid_stop_m
        scale = (nearest - cfg.avoid_stop_m) / span if span > 0.0 else 0.0

    speed = max(0.0, cmd.v_mps) * scale
    omega = _clamp(cmd.omega_radps + cfg.avoid_gain * deflection, cfg.max_omega_radps)
    return TwistCmd(v_mps=speed, omega_radps=omega)


# ------------------------------------------------------------------ route following --
@dataclass(frozen=True, slots=True)
class RouteProgress:
    """Result of one :func:`follow_route` call.

    ``index`` is the waypoint the robot is now driving toward; hand it back on the next
    call. ``finished`` means the final waypoint was reached, at which point ``cmd`` is
    :data:`STOP`.
    """

    cmd: TwistCmd
    index: int
    finished: bool


def follow_route(
    pose: Pose2D,
    waypoints: Sequence[Point2D],
    index: int,
    cfg: NavConfig,
    *,
    final_tolerance_m: float | None = None,
) -> RouteProgress:
    """Drive along ``waypoints``, advancing the index as each one is reached.

    Intermediate waypoints use :attr:`NavConfig.waypoint_tolerance_m` (a corner to round);
    the last one uses ``final_tolerance_m``, defaulting to
    :attr:`NavConfig.arrival_tolerance_m` (a place to stop). Several waypoints can be
    consumed in a single call when they are bunched closer together than the tolerance,
    which keeps the robot from stalling on a dense polyline.

    This function holds no state -- ``index`` is the entire memory, and it is the caller's.
    That is what makes a route replayable: the same ``(pose, waypoints, index)`` always
    yields the same command.
    """
    if not waypoints:
        msg = "follow_route needs at least one waypoint"
        raise ValueError(msg)
    if index < 0:
        msg = f"waypoint index must be non-negative, got {index}"
        raise ValueError(msg)

    final_tol = cfg.arrival_tolerance_m if final_tolerance_m is None else final_tolerance_m
    if final_tol < cfg.arrival_tolerance_m:
        # waypoint_command stops inside cfg.arrival_tolerance_m, so a tighter final
        # tolerance than that would park the robot just outside the goal and call it
        # "not finished" forever. Tight final approaches belong in
        # parkscan_agent.docking, which drives the last metre itself.
        msg = (
            f"final_tolerance_m={final_tol} is tighter than the controller's own arrival "
            f"tolerance {cfg.arrival_tolerance_m}; follow_route could never report finished"
        )
        raise ValueError(msg)
    cursor = index
    last = len(waypoints) - 1
    while cursor < last and has_arrived(pose, waypoints[cursor], cfg.waypoint_tolerance_m):
        cursor += 1
    if cursor >= last and has_arrived(pose, waypoints[last], final_tol):
        return RouteProgress(cmd=STOP, index=last, finished=True)

    target = waypoints[cursor]
    cmd = waypoint_command(pose, target, cfg)
    if cursor == last and cmd == STOP:
        # waypoint_command's own arrival tolerance fired before ours did.
        return RouteProgress(cmd=STOP, index=last, finished=True)
    return RouteProgress(cmd=cmd, index=cursor, finished=False)

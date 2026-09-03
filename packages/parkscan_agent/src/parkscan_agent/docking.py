"""Final approach into a charging bay.

The geometry the lot gives us
-----------------------------
The two bays on the shipped lot are **spurs**: ``bay_1_node`` at ``(8.0, 17.5)`` hangs off
the west perimeter lane at ``wl_s1 (2.5, 17.5)``, and ``bay_2_node`` likewise off
``wl_s2``. That is not decoration. If the bays sat on the perimeter lane itself, a docked
robot would block the only north-south route on the west side and a second robot could
never reach the other bay -- the lease system would be arbitrating a lane it had already
deadlocked. Because they are spurs, the approach direction is determined by geometry
rather than chosen: a robot enters the bay driving **along the spur**, away from the
junction, and leaves by reversing that.

The controller therefore derives the approach heading from the graph, not from a constant:
it finds the bay node's single lane neighbour and takes the bearing from that neighbour to
the bay. A lot that later gives a bay two neighbours falls back to the last edge of the
planned route, which is the same answer whenever the route came in along the spur.

Phases
------
``ROUTING`` -> ``APPROACH`` -> ``ALIGN`` -> ``DOCKED``, with ``FAILED`` reachable from any
of them.

* **ROUTING** follows the lane route at normal speed, up to the *junction* node -- the
  spur's other end. This is ordinary route following; nothing special happens yet.
* **APPROACH** covers the last few metres onto the bay position at
  :attr:`DockConfig.approach_speed_mps`, a small fraction of cruising speed, with a tight
  position tolerance. Slow matters: the position tolerance is centimetres, and a
  controller running at cruise speed overshoots it every tick and hunts forever.
* **ALIGN** rotates in place until the heading matches the spur direction. Position and
  heading are checked separately, in that order, because a robot that satisfies both at
  once by luck is not evidence that the controller can achieve either.
* **DOCKED** is reported once *both* tolerances hold; **FAILED** once the tick budget runs
  out, which is what turns "docking is taking a while" into an event the state machine can
  act on (:attr:`~parkscan_agent.statemachine.Event.DOCK_FAILED`) instead of a robot that
  silently never finishes.

What this does not do
---------------------
There is no charging-contact model, no IR beacon, no fiducial, and no physical latch. A
"successful dock" here means the robot's pose is inside the configured position and
heading tolerance of the bay's authored coordinates. Nothing about this is a claim that a
real robot would mate with a real charger.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum

from parkscan_agent.lanegraph import adjacency, nearest_node, node_xy, route, route_points
from parkscan_agent.navigation import (
    STOP,
    NavConfig,
    follow_route,
    has_arrived,
    wrap_angle,
)
from parkscan_core.lot import ChargingBay, LotSpec
from parkscan_core.types import Pose2D, TwistCmd

__all__ = ["DockConfig", "DockController", "DockPhase", "DockStatus"]

type Point2D = tuple[float, float]


class DockPhase(StrEnum):
    """Where the docking manoeuvre has got to."""

    ROUTING = "routing"
    APPROACH = "approach"
    ALIGN = "align"
    DOCKED = "docked"
    FAILED = "failed"


@dataclass(frozen=True, slots=True)
class DockConfig:
    """Tolerances and limits for the final approach."""

    approach_speed_mps: float = 0.25
    """Speed on the last leg. Roughly a fifth of cruise; see the ALIGN/APPROACH note."""

    align_omega_radps: float = 0.6
    """Yaw rate cap while rotating in place onto the spur heading."""

    position_tolerance_m: float = 0.12
    """How close the robot's origin must be to the bay's authored position."""

    heading_tolerance_rad: float = 0.10
    """How closely the robot must face along the spur (~5.7 degrees)."""

    approach_radius_m: float = 3.0
    """Distance from the bay at which routing hands over to the slow approach."""

    creep_radius_m: float = 1.0
    """Distance over which approach speed ramps linearly to zero at the bay.

    Without a ramp the controller arrives at full approach speed and steps straight over a
    12 cm tolerance every tick, hunting forever; with one, the last tick before arrival is
    a few millimetres long.
    """

    max_ticks: int = 4000
    """Tick budget for the whole manoeuvre before reporting failure.

    A budget rather than a timeout: the agent runs on the simulation clock, and a wall
    clock would make a docking outcome depend on how fast the machine happened to be.
    """

    def __post_init__(self) -> None:
        if self.approach_speed_mps <= 0.0:
            msg = f"approach_speed_mps must be positive, got {self.approach_speed_mps}"
            raise ValueError(msg)
        if self.align_omega_radps <= 0.0:
            msg = f"align_omega_radps must be positive, got {self.align_omega_radps}"
            raise ValueError(msg)
        if self.position_tolerance_m <= 0.0 or self.heading_tolerance_rad <= 0.0:
            msg = "docking tolerances must be positive"
            raise ValueError(msg)
        if self.approach_radius_m <= self.position_tolerance_m:
            msg = "approach_radius_m must be larger than position_tolerance_m"
            raise ValueError(msg)
        if self.creep_radius_m <= self.position_tolerance_m:
            msg = "creep_radius_m must be larger than position_tolerance_m"
            raise ValueError(msg)
        if self.max_ticks <= 0:
            msg = f"max_ticks must be positive, got {self.max_ticks}"
            raise ValueError(msg)


@dataclass(frozen=True, slots=True)
class DockStatus:
    """One tick of docking: the command to send, and everything needed to explain it."""

    phase: DockPhase
    cmd: TwistCmd
    distance_m: float
    """Straight-line distance from the robot to the bay position, in metres."""
    heading_error_rad: float
    """Signed error between the robot's heading and the spur approach heading."""
    ticks: int
    detail: str

    @property
    def docked(self) -> bool:
        return self.phase is DockPhase.DOCKED

    @property
    def failed(self) -> bool:
        return self.phase is DockPhase.FAILED


class DockController:
    """Drives one robot from wherever it is into one charging bay.

    Stateful by necessity -- a manoeuvre has phases -- but the state is small, explicit and
    entirely local: a route, a waypoint cursor, a phase and a tick count. It performs no
    I/O and touches no clock; :meth:`update` is called once per tick with the robot's
    current pose and returns the command the caller should actuate.
    """

    def __init__(
        self,
        bay: ChargingBay,
        lot: LotSpec,
        nav: NavConfig,
        cfg: DockConfig | None = None,
    ) -> None:
        if all(existing.id != bay.id for existing in lot.charging_bays):
            msg = f"bay {bay.id!r} is not a charging bay of lot {lot.name!r}"
            raise ValueError(msg)
        self._bay = bay
        self._lot = lot
        self._nav = nav
        self._cfg = cfg if cfg is not None else DockConfig()
        self._target: Point2D = (bay.position[0], bay.position[1])
        self._approach_heading = _spur_heading(lot, bay)
        self._waypoints: list[Point2D] = [self._target]
        self._index = 0
        self._phase = DockPhase.ROUTING
        self._ticks = 0

    # ------------------------------------------------------------------ properties --
    @property
    def bay_id(self) -> str:
        return self._bay.id

    @property
    def phase(self) -> DockPhase:
        return self._phase

    @property
    def approach_heading_rad(self) -> float:
        """Heading the robot must hold when docked: along the spur, into the bay."""
        return self._approach_heading

    @property
    def waypoints(self) -> tuple[Point2D, ...]:
        """The planned lane route, in metres. Empty until :meth:`start` is called."""
        return tuple(self._waypoints)

    # ---------------------------------------------------------------------- driving --
    def start(self, pose: Pose2D) -> None:
        """Plan the lane route from ``pose`` to the bay and reset the manoeuvre.

        Call once when the robot enters ``RETURNING``. Re-planning mid-dock is deliberately
        not supported: the reactive layer may slow or deflect the robot, but the route to
        the bay is a planner decision, and silently re-planning it would hide the fact that
        the robot is not making progress.

        Raises:
            ~parkscan_agent.lanegraph.NoRouteError: no lane route reaches the bay.
        """
        origin = nearest_node(self._lot, pose.x_m, pose.y_m)
        path = route(self._lot, origin, self._bay.node)
        points = route_points(self._lot, path)
        if points[-1] != self._target:
            points[-1] = self._target
        self._waypoints = points
        self._index = 0
        self._phase = DockPhase.ROUTING
        self._ticks = 0

    def update(self, pose: Pose2D) -> DockStatus:
        """Advance the manoeuvre by one tick and return the command to actuate.

        Once :attr:`DockPhase.DOCKED` or :attr:`DockPhase.FAILED` is reached the controller
        latches: further calls repeat the terminal status with a :data:`STOP` command, so a
        caller that keeps ticking cannot accidentally drive out of the bay.
        """
        distance = math.dist((pose.x_m, pose.y_m), self._target)
        heading_error = wrap_angle(self._approach_heading - pose.theta_rad)

        if self._phase in (DockPhase.DOCKED, DockPhase.FAILED):
            return self._status(STOP, distance, heading_error, "terminal")

        self._ticks += 1
        if self._ticks > self._cfg.max_ticks:
            self._phase = DockPhase.FAILED
            return self._status(
                STOP,
                distance,
                heading_error,
                f"gave up after {self._cfg.max_ticks} ticks at {distance:.3f} m "
                f"and {heading_error:.3f} rad from bay {self._bay.id}",
            )

        if self._phase is DockPhase.ROUTING:
            if distance <= self._cfg.approach_radius_m:
                self._phase = DockPhase.APPROACH
            else:
                progress = follow_route(pose, self._waypoints, self._index, self._nav)
                self._index = progress.index
                if progress.finished:
                    # The route ended but we are still outside the approach radius: the
                    # lane geometry does not actually reach this bay. Fail loudly.
                    self._phase = DockPhase.FAILED
                    return self._status(
                        STOP,
                        distance,
                        heading_error,
                        f"lane route to bay {self._bay.id} ended {distance:.3f} m short",
                    )
                return self._status(progress.cmd, distance, heading_error, "following lane route")

        if self._phase is DockPhase.APPROACH:
            if has_arrived(pose, self._target, self._cfg.position_tolerance_m):
                self._phase = DockPhase.ALIGN
            else:
                return self._status(
                    self._approach_cmd(pose, distance),
                    distance,
                    heading_error,
                    "slow approach onto the bay",
                )

        # ALIGN: position is inside tolerance; rotate onto the spur heading.
        if abs(heading_error) > self._cfg.heading_tolerance_rad:
            if not has_arrived(pose, self._target, self._cfg.position_tolerance_m):
                # Rotating pushed the origin back out of tolerance (or avoidance nudged
                # it). Go back and re-approach rather than declaring a dock we do not have.
                self._phase = DockPhase.APPROACH
                return self._status(
                    self._approach_cmd(pose, distance),
                    distance,
                    heading_error,
                    "drifted out of position, re-approaching",
                )
            omega = max(
                -self._cfg.align_omega_radps,
                min(self._cfg.align_omega_radps, self._nav.heading_gain * heading_error),
            )
            return self._status(
                TwistCmd(v_mps=0.0, omega_radps=omega),
                distance,
                heading_error,
                "aligning to the spur heading",
            )

        self._phase = DockPhase.DOCKED
        return self._status(
            STOP,
            distance,
            heading_error,
            f"docked in bay {self._bay.id} at {distance:.3f} m, {heading_error:.3f} rad",
        )

    # ------------------------------------------------------------------- internals --
    def _approach_cmd(self, pose: Pose2D, distance: float) -> TwistCmd:
        """Creep toward the bay, steering with the same heading law as cruising.

        Deliberately *not* :func:`~parkscan_agent.navigation.waypoint_command`: that
        function stops inside ``NavConfig.arrival_tolerance_m``, which is twice the docking
        tolerance, so reusing it here would park the robot short of the bay and the
        controller would never converge.
        """
        bearing = math.atan2(
            self._target[1] - pose.y_m,
            self._target[0] - pose.x_m,
        )
        alpha = wrap_angle(bearing - pose.theta_rad)
        speed = self._cfg.approach_speed_mps * max(0.0, math.cos(alpha))
        if abs(alpha) > self._nav.turn_in_place_rad:
            speed = 0.0
        # Ramp down over the last metre so the final tick lands inside the tolerance
        # instead of stepping across it.
        speed *= min(1.0, distance / self._cfg.creep_radius_m)
        omega = max(
            -self._cfg.align_omega_radps,
            min(self._cfg.align_omega_radps, self._nav.heading_gain * alpha),
        )
        return TwistCmd(v_mps=speed, omega_radps=omega)

    def _status(
        self, cmd: TwistCmd, distance: float, heading_error: float, detail: str
    ) -> DockStatus:
        return DockStatus(
            phase=self._phase,
            cmd=cmd,
            distance_m=distance,
            heading_error_rad=heading_error,
            ticks=self._ticks,
            detail=detail,
        )


def _spur_heading(lot: LotSpec, bay: ChargingBay) -> float:
    """Heading a robot must face when docked: from the spur junction into the bay.

    Derived from the lane graph so the lot's geometry, not a constant in this file, decides
    it. A bay node with exactly one neighbour (a true spur, which is how this lot is built)
    gives an unambiguous answer; with several neighbours the smallest node id is used so
    the result is still deterministic, and the caller's route will normally have come in
    along that edge anyway.
    """
    neighbours = adjacency(lot).get(bay.node, ())
    bx, by = bay.position
    if not neighbours:
        # No spur to come in along; face east so the value is defined and reproducible.
        return 0.0
    junction = min(neighbours)[0]
    jx, jy = node_xy(lot, junction)
    if math.isclose(jx, bx, abs_tol=1e-9) and math.isclose(jy, by, abs_tol=1e-9):
        return 0.0
    return math.atan2(by - jy, bx - jx)

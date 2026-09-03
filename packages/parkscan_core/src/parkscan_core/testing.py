"""Test doubles for :class:`~parkscan_core.world.WorldBackend`.

Shipped as part of the package, not hidden in a test directory, for two reasons:

* It implements the *core* protocol, so core is where it belongs. Every consumer of
  ``WorldBackend`` -- the autonomy today, the coordinator in Phase 4 -- needs the same
  double, and a copy per test tree would drift.
* It lets a test exercise autonomy or coordination WITHOUT importing ``parkscan_sim``.
  ADR-0001 requires that separation, and a shared, typed double is what makes obeying it
  the easy path rather than the disciplined one.

:class:`FakeBackend` is a kinematic stub, not a simulator. It has no sensors worth the
name, no obstacles, no battery model and no collision checking. Use ``parkscan_sim`` when
any of that matters; use this when it would only be noise.
"""

from __future__ import annotations

import math
from collections.abc import Mapping

from parkscan_core.lot import LotSpec
from parkscan_core.types import Pose2D, RobotId, SensorFrame, TwistCmd

__all__ = ["FakeBackend"]


def _wrap(theta_rad: float) -> float:
    return math.atan2(math.sin(theta_rad), math.cos(theta_rad))


class FakeBackend:
    """Exact unicycle integration, a constant battery, and clear range readings.

    Deterministic by construction: it holds no randomness and reads no clock, so a test
    that drives it produces the same trajectory on every run and on every machine.
    """

    def __init__(
        self,
        poses: Mapping[RobotId, Pose2D],
        *,
        dt_s: float = 0.05,
        battery_soc: float = 1.0,
        ranges: tuple[float, ...] = (math.inf,) * 9,
    ) -> None:
        if not poses:
            msg = "FakeBackend needs at least one robot"
            raise ValueError(msg)
        self._poses: dict[RobotId, Pose2D] = dict(poses)
        self._cmds: dict[RobotId, TwistCmd] = {
            robot: TwistCmd(v_mps=0.0, omega_radps=0.0) for robot in poses
        }
        self._dt_s = dt_s
        self._soc: dict[RobotId, float] = dict.fromkeys(poses, battery_soc)
        self._ranges = ranges
        self._tick = 0

    # ------------------------------------------------------------- WorldBackend --
    def reset(self, seed: int, lot: LotSpec) -> None:
        """Rewind the clock. Poses are whatever the constructor was given.

        The seed is accepted and ignored on purpose: there is nothing random here, and a
        double that silently re-randomised would defeat the tests that use it.
        """
        self._tick = 0

    def step(self, dt: float) -> None:
        for robot, cmd in self._cmds.items():
            pose = self._poses[robot]
            half_heading = pose.theta_rad + 0.5 * cmd.omega_radps * dt
            self._poses[robot] = Pose2D(
                x_m=pose.x_m + cmd.v_mps * math.cos(half_heading) * dt,
                y_m=pose.y_m + cmd.v_mps * math.sin(half_heading) * dt,
                theta_rad=_wrap(pose.theta_rad + cmd.omega_radps * dt),
            )
        self._tick += 1

    def now_ns(self) -> int:
        return round(self._tick * self._dt_s * 1e9)

    def robots(self) -> tuple[RobotId, ...]:
        return tuple(self._poses)

    def sense(self, robot: RobotId) -> SensorFrame:
        return SensorFrame(
            stamp_ns=self.now_ns(),
            pose=self._poses[robot],
            twist=self._cmds[robot],
            battery_soc=self._soc[robot],
            ranges=self._ranges,
            camera=None,
        )

    def actuate(self, robot: RobotId, cmd: TwistCmd) -> None:
        self._cmds[robot] = cmd

    # ------------------------------------------------------------- test helpers --
    @property
    def dt_s(self) -> float:
        return self._dt_s

    def pose(self, robot: RobotId) -> Pose2D:
        return self._poses[robot]

    def battery_soc(self, robot: RobotId) -> float:
        return self._soc[robot]

    def set_soc(self, robot: RobotId, soc: float) -> None:
        if not 0.0 <= soc <= 1.0:
            msg = f"soc must be in [0, 1], got {soc}"
            raise ValueError(msg)
        self._soc[robot] = soc

    def set_ranges(self, ranges: tuple[float, ...]) -> None:
        self._ranges = ranges

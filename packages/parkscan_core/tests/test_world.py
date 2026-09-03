"""The WorldBackend Protocol is satisfiable and checkable at runtime."""

from __future__ import annotations

from collections.abc import Sequence

from parkscan_core.lot import LotSpec
from parkscan_core.types import Pose2D, RobotId, SensorFrame, TwistCmd
from parkscan_core.world import WorldBackend


class StubBackend:
    """A minimal structural implementation -- no inheritance, that is the point."""

    def __init__(self) -> None:
        self.t_ns = 0
        self.commands: dict[RobotId, TwistCmd] = {}

    def reset(self, seed: int, lot: LotSpec) -> None:
        self.t_ns = 0
        self.seed = seed
        self.lot = lot

    def step(self, dt: float) -> None:
        self.t_ns += int(dt * 1e9)

    def now_ns(self) -> int:
        return self.t_ns

    def robots(self) -> Sequence[RobotId]:
        return ("robot_1",)

    def sense(self, robot: RobotId) -> SensorFrame:
        return SensorFrame(
            stamp_ns=self.t_ns,
            pose=Pose2D(0.0, 0.0, 0.0),
            twist=self.commands.get(robot, TwistCmd(0.0, 0.0)),
            battery_soc=1.0,
            ranges=(float("inf"),) * 8,
            camera=None,
        )

    def actuate(self, robot: RobotId, cmd: TwistCmd) -> None:
        self.commands[robot] = cmd


class NotABackend:
    def step(self, dt: float) -> None:
        return None


def test_a_structural_implementation_satisfies_the_protocol(lot: LotSpec) -> None:
    backend: WorldBackend = StubBackend()
    assert isinstance(backend, WorldBackend)
    backend.reset(seed=7, lot=lot)
    assert backend.now_ns() == 0
    assert list(backend.robots()) == ["robot_1"]
    backend.actuate("robot_1", TwistCmd(0.5, 0.0))
    backend.step(0.1)
    frame = backend.sense("robot_1")
    assert backend.now_ns() == 100_000_000
    assert frame.stamp_ns == 100_000_000
    assert frame.twist == TwistCmd(0.5, 0.0)
    assert frame.camera is None


def test_a_partial_implementation_is_rejected_at_runtime() -> None:
    assert not isinstance(NotABackend(), WorldBackend)

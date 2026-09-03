"""The agent end to end, plus docking, plus the architectural boundary.

Everything here runs against the fake backend from conftest, never against
parkscan_sim. That is not a convenience: ADR-0001 says the autonomy must be
backend-agnostic, and `test_autonomy_never_imports_the_simulator` is the assertion that
keeps it true as the code grows.
"""

from __future__ import annotations

import ast
import math
import subprocess
import sys
from pathlib import Path

import pytest
from parkscan_agent.docking import DockConfig, DockController, DockPhase
from parkscan_agent.lanegraph import node_xy
from parkscan_agent.navigation import NavConfig
from parkscan_agent.robot import AgentConfig, RobotAgent, planned_cells, sweep_waypoints
from parkscan_agent.statemachine import IllegalTransition

from parkscan_core.lot import LotSpec
from parkscan_core.testing import FakeBackend
from parkscan_core.types import Pose2D, RobotState

SRC = Path(__file__).resolve().parents[1] / "src" / "parkscan_agent"


# ----------------------------------------------------- the architectural boundary --
def test_autonomy_never_imports_the_simulator() -> None:
    """ADR-0001: the autonomy must run against any WorldBackend.

    A single `import parkscan_sim` would silently weld the two together and the ROS 2
    backend would stop being possible without a rewrite.
    """
    # Parsed, not grepped: several modules *document* this boundary in their docstrings,
    # and a substring search would flag the documentation as the violation.
    offenders: list[str] = []
    for path in sorted(SRC.glob("*.py")):
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""]
            else:
                continue
            if any(n.split(".")[0] == "parkscan_sim" for n in names):
                offenders.append(f"{path.name}:{node.lineno}")
    assert offenders == [], f"these modules import the simulator: {offenders}"


def test_importing_the_autonomy_does_not_pull_in_the_simulator() -> None:
    """Runtime companion to the static check: import the package in a clean interpreter
    and assert the simulator did not come along as a transitive dependency."""
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import parkscan_agent.robot, sys; print('parkscan_sim' in sys.modules)",
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    assert result.stdout.strip() == "False"


# --------------------------------------------------------------- sweep geometry --
def test_sweep_waypoints_stay_inside_the_zone(lot: LotSpec) -> None:
    cfg = AgentConfig()
    for zone in lot.zones:
        xs = [p[0] for p in zone.polygon]
        ys = [p[1] for p in zone.polygon]
        for x, y in sweep_waypoints(zone, lot, cfg):
            assert min(xs) - 1e-6 <= x <= max(xs) + 1e-6
            assert min(ys) - 1e-6 <= y <= max(ys) + 1e-6


def test_a_zone_plans_a_nonempty_set_of_cells(lot: LotSpec) -> None:
    cfg = AgentConfig()
    for zone in lot.zones:
        cells = planned_cells(zone, lot, cfg)
        assert len(cells) > 0
        assert len(set(cells)) == len(cells), "planned cells must not repeat"


# ------------------------------------------------------------------- the agent --
def test_a_fresh_agent_is_idle_and_reports_telemetry(
    lot: LotSpec, backend_factory: type[FakeBackend]
) -> None:
    backend = backend_factory({"robot_1": _bay_pose(lot)})
    agent = RobotAgent("robot_1", lot, backend)
    assert agent.state is RobotState.IDLE

    sample = agent.tick()
    assert sample.robot_id == "robot_1"
    assert sample.zone_id is None
    assert 0.0 <= sample.battery_soc <= 1.0
    # The agent must not claim provenance it cannot know (H1) — that is the runner's job.
    assert not hasattr(sample, "data_source")


def test_assignment_moves_the_robot_out_of_idle(
    lot: LotSpec, backend_factory: type[FakeBackend]
) -> None:
    backend = backend_factory({"robot_1": _bay_pose(lot)})
    agent = RobotAgent("robot_1", lot, backend)
    agent.assign(lot.zones[0].id)
    assert agent.state is not RobotState.IDLE
    assert agent.zone_id == lot.zones[0].id


def test_an_illegal_external_event_raises_rather_than_being_ignored(
    lot: LotSpec, backend_factory: type[FakeBackend]
) -> None:
    """A coordinator bug must surface, not be swallowed."""
    backend = backend_factory({"robot_1": _bay_pose(lot)})
    agent = RobotAgent("robot_1", lot, backend)
    with pytest.raises(IllegalTransition):
        agent.resume()  # nothing to resume from IDLE


def test_the_agent_drives_a_zone_and_covers_cells(
    lot: LotSpec, backend_factory: type[FakeBackend]
) -> None:
    """A1: navigate to inspection waypoints. Runs the loop for real."""
    zone = lot.zones[0]
    backend = backend_factory({"robot_1": _node_pose(lot, zone.entry_waypoint)})
    agent = RobotAgent("robot_1", lot, backend)
    agent.assign(zone.id)

    covered: set[str] = set()
    states: set[RobotState] = set()
    for _ in range(20_000):
        sample = agent.tick()
        covered.update(sample.covered_cells)
        states.add(sample.state)
        backend.step(backend.dt_s)
        if sample.state is RobotState.IDLE and covered:
            break

    assert RobotState.INSPECTING in states, f"never inspected; saw {states}"
    assert len(covered) >= 3, f"covered only {len(covered)} cells"


def test_a_flat_battery_diverts_the_robot_to_returning(
    lot: LotSpec, backend_factory: type[FakeBackend]
) -> None:
    """A2 precondition: the robot decides to go home on its own."""
    zone = lot.zones[-1]
    backend = backend_factory({"robot_1": _node_pose(lot, zone.entry_waypoint)})
    agent = RobotAgent("robot_1", lot, backend)
    agent.assign(zone.id)
    agent.tick()
    backend.set_soc("robot_1", 0.02)

    for _ in range(500):
        sample = agent.tick()
        backend.step(backend.dt_s)
        if sample.state in (RobotState.RETURNING, RobotState.CHARGING):
            break
    assert agent.state in (RobotState.RETURNING, RobotState.CHARGING)


# ------------------------------------------------------------------- docking ----
@pytest.mark.parametrize("offset", [(2.0, 0.0), (-2.0, 1.5), (0.0, -3.0), (3.0, 3.0)])
def test_docking_succeeds_from_several_start_poses(
    lot: LotSpec, backend_factory: type[FakeBackend], offset: tuple[float, float]
) -> None:
    """A2: autonomous parking at a charging bay."""
    bay = lot.charging_bays[0]
    bx, by = node_xy(lot, bay.node)
    start = Pose2D(x_m=bx + offset[0], y_m=by + offset[1], theta_rad=0.3)
    backend = backend_factory({"r": start})
    controller = DockController(bay, lot, NavConfig(), DockConfig())

    controller.start(backend.pose("r"))
    status = None
    for _ in range(DockConfig().max_ticks):
        status = controller.update(backend.pose("r"))
        if status.phase in (DockPhase.DOCKED, DockPhase.FAILED):
            break
        backend.actuate("r", status.cmd)
        backend.step(backend.dt_s)

    assert status is not None
    assert status.phase is DockPhase.DOCKED, f"{status.phase}: {status.detail}"
    final = backend.pose("r")
    assert math.dist((final.x_m, final.y_m), bay.position) <= DockConfig().position_tolerance_m


def _bay_pose(lot: LotSpec) -> Pose2D:
    x, y = node_xy(lot, lot.charging_bays[0].node)
    return Pose2D(x_m=x, y_m=y, theta_rad=0.0)


def _node_pose(lot: LotSpec, node: str) -> Pose2D:
    x, y = node_xy(lot, node)
    return Pose2D(x_m=x, y_m=y, theta_rad=0.0)

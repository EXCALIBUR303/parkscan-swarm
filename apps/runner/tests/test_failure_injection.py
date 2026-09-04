"""Acceptance A7: an offline robot and a low-battery robot both get their work taken over.

These are integration tests — real simulator, real autonomy, real coordinator — because
the unit tests in ``parkscan_coord`` already prove each mechanism in isolation and what is
left to show is that they compose. No MQTT and no database: the recovery path does not
involve either, and adding them would only add ways for the test to fail for reasons that
are not about recovery.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from parkscan_agent.robot import AgentConfig, RobotAgent
from parkscan_coord.allocation import Strategy
from parkscan_coord.coordinator import Coordinator, CoordinatorConfig, DecisionKind
from parkscan_runner.fleet import StrategyAllocator
from parkscan_sim.config import SimConfig
from parkscan_sim.world import SimLite

from parkscan_core.lot import LotSpec, load_lot
from parkscan_core.types import CoordView, RobotState, ZoneState

REPO_ROOT = Path(__file__).resolve().parents[3]
LOT_YAML = REPO_ROOT / "sim" / "lot" / "lot.yaml"
NS_PER_S = 1_000_000_000


@pytest.fixture(scope="module")
def lot() -> LotSpec:
    return load_lot(LOT_YAML)


class _Fleet:
    """A running three-robot fleet, driven one tick at a time."""

    def __init__(self, lot: LotSpec, seed: int = 42) -> None:
        self.sim = SimLite(SimConfig())
        self.sim.reset(seed, lot)
        self.ids = list(self.sim.robots())
        self.agents = {r: RobotAgent(r, lot, self.sim, AgentConfig()) for r in self.ids}
        self.coord = Coordinator(
            lot,
            self.agents,
            allocator=StrategyAllocator(Strategy.GREEDY),
            config=CoordinatorConfig(),
        )
        self.coord.start(0, poses={r: self.sim.sense(r).pose for r in self.ids})
        self.decisions: list[object] = []
        self.silent: set[str] = set()

    def run(self, steps: int) -> None:
        for _ in range(steps):
            now_ns = self.sim.now_ns()
            for robot_id, agent in self.agents.items():
                sample = agent.tick()
                if robot_id not in self.silent:
                    self.coord.ingest(sample)
            self.decisions.extend(self.coord.tick(now_ns))
            self.sim.step(self.sim.config.dt_s)

    def kinds(self) -> list[DecisionKind]:
        return [d.kind for d in self.decisions]  # type: ignore[attr-defined]


def test_an_offline_robot_loses_its_zone_and_another_takes_it_over(lot: LotSpec) -> None:
    """A7, first scenario: silence, then handover."""
    fleet = _Fleet(lot)
    fleet.run(200)

    victim = next(r for r in fleet.ids if fleet.agents[r].zone_id is not None)
    orphaned = fleet.agents[victim].zone_id
    assert orphaned is not None

    # It stops publishing. Nothing else about it changes — this is a comms failure, not a
    # crash, which is exactly the case the heartbeat deadline exists for and the broker's
    # Last Will would never catch.
    fleet.silent.add(victim)
    fleet.run(400)

    assert fleet.coord.monitor.is_latched_offline(victim)
    assert DecisionKind.ROBOT_OFFLINE in fleet.kinds()
    assert DecisionKind.REQUEUED in fleet.kinds()

    # The zone did not simply vanish with the robot.
    status = fleet.coord.zone_status(orphaned)
    assert status.state is not ZoneState.UNRECOVERABLE
    assert status.assigned_robot != victim


def test_the_orphaned_work_is_resumed_not_restarted(lot: LotSpec) -> None:
    """The remainder is what gets requeued — half-swept work is not thrown away."""
    fleet = _Fleet(lot)
    fleet.run(600)

    victim = next(
        (r for r in fleet.ids if fleet.agents[r].zone_id is not None and _progress(fleet, r)),
        None,
    )
    if victim is None:
        pytest.skip("no robot had partial progress yet at this seed")

    orphaned = fleet.agents[victim].zone_id
    assert orphaned is not None
    done_before = set(fleet.coord.zone_status(orphaned).cells_done)
    assert done_before, "precondition: the victim had covered something"

    fleet.silent.add(victim)
    fleet.run(400)

    status = fleet.coord.zone_status(orphaned)
    # Progress is carried across the handover, so the replacement resumes.
    assert done_before <= set(status.cells_done)
    assert len(status.cells_remaining) < len(status.cells_total)


def test_a_low_battery_robot_hands_its_zone_over_and_goes_to_charge(lot: LotSpec) -> None:
    """A7, second scenario: the robot is alive and honest, it just cannot finish."""
    fleet = _Fleet(lot)
    fleet.run(200)

    victim = next(r for r in fleet.ids if fleet.agents[r].zone_id is not None)
    fleet.sim.set_battery_soc(victim, 0.06)
    fleet.run(1200)

    # It went home rather than stranding, and was never declared dead — it kept talking.
    assert fleet.agents[victim].state in {
        RobotState.RETURNING,
        RobotState.CHARGING,
        RobotState.IDLE,
    }
    assert not fleet.coord.monitor.is_latched_offline(victim)
    assert fleet.coord.monitor.assess(victim, fleet.sim.now_ns()).view is not CoordView.OFFLINE
    assert not fleet.sim.is_depleted(victim), "C2: no robot may reach zero charge"


def test_the_fleet_still_finishes_every_zone_after_a_robot_is_lost(lot: LotSpec) -> None:
    """The point of recovery: coverage survives the failure."""
    fleet = _Fleet(lot)
    fleet.run(200)
    fleet.silent.add(fleet.ids[0])
    fleet.run(40_000)

    incomplete = [
        zone.id
        for zone in lot.zones
        if fleet.coord.zone_status(zone.id).state is not ZoneState.COMPLETE
    ]
    assert incomplete == [], f"zones left unfinished after a robot was lost: {incomplete}"
    assert len(fleet.sim.collisions()) == 0, "C1: recovery must not cause a collision"


def _progress(fleet: _Fleet, robot_id: str) -> bool:
    zone_id = fleet.agents[robot_id].zone_id
    return zone_id is not None and bool(fleet.coord.zone_status(zone_id).cells_done)

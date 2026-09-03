"""Determinism is the product.

Every number in the README is meant to be regenerable by ``make bench`` -- 30 seeds x 3
allocation strategies (ADR-0001, acceptance criterion A11). That claim is only true if a
run is a pure function of ``(seed, lot, SimConfig)`` and the command sequence. These are
the tests that hold it to that, so they assert **exact** float equality rather than
approximate: a trajectory that merely agrees to 1e-9 is not reproducible, it is close.
"""

from __future__ import annotations

import math
import random

import numpy as np
import pytest
from parkscan_sim.config import SimConfig
from parkscan_sim.kinematics import TwistLimits
from parkscan_sim.world import SimLite

from parkscan_core.lot import LotSpec
from parkscan_core.types import TwistCmd

STEPS = 400


#: One robot's (x, y, theta, soc) at a single tick.
RobotState4 = tuple[float, float, float, float]
#: Every robot's state at one tick, in a stable robot order.
FleetState = tuple[RobotState4, ...]
#: (sim_clock_ns, fleet_state) per tick, for the whole run.
Trace = list[tuple[int, FleetState]]


def _command(step_index: int, robot_index: int) -> TwistCmd:
    """A varied but perfectly deterministic command sequence -- no RNG, no clock.

    Sinusoids with mutually irrational-ish periods so the fleet turns, reverses and
    changes speed rather than driving three identical straight lines, which would make
    an integrator bug invisible.
    """
    phase = 0.011 * step_index + 1.7 * robot_index
    return TwistCmd(
        v_mps=0.75 + 0.45 * math.sin(phase),
        omega_radps=0.6 * math.cos(0.023 * step_index + 2.3 * robot_index),
    )


def _run(
    seed: int,
    lot: LotSpec,
    steps: int = STEPS,
    config: SimConfig | None = None,
    sim: SimLite | None = None,
) -> Trace:
    """Drive a fleet for ``steps`` ticks and return the full trajectory trace."""
    cfg = config if config is not None else SimConfig()
    world = sim if sim is not None else SimLite(cfg)
    world.reset(seed, lot)
    robots = list(world.robots())
    trace = []
    for step_index in range(steps):
        for robot_index, robot in enumerate(robots):
            world.actuate(robot, _command(step_index, robot_index))
        world.step(cfg.dt_s)
        trace.append(
            (
                world.now_ns(),
                tuple(
                    (
                        world.pose(robot).x_m,
                        world.pose(robot).y_m,
                        world.pose(robot).theta_rad,
                        world.battery_soc(robot),
                    )
                    for robot in robots
                ),
            )
        )
    return trace


def _poses(trace: Trace) -> list[FleetState]:
    return [row[1] for row in trace]


# ------------------------------------------------------------- the central claim --
def test_same_seed_gives_a_bit_identical_trajectory(lot: LotSpec) -> None:
    """400 steps, twice, same commands: every pose identical to the last bit."""
    first = _run(11, lot)
    second = _run(11, lot)
    assert len(first) == STEPS
    assert first == second  # exact float equality, not approx


def test_same_seed_on_a_reused_instance_gives_the_same_trajectory(lot: LotSpec) -> None:
    """reset() must clear *everything*: a second run on one instance is a fresh run."""
    world = SimLite()
    first = _run(11, lot, sim=world)
    second = _run(11, lot, sim=world)
    assert first == second
    assert world.collision_count() >= 0


def test_a_different_seed_gives_a_different_trajectory(lot: LotSpec) -> None:
    """Otherwise the seed would be decorative and 30 seeds would be one run repeated."""
    first = _poses(_run(11, lot))
    other = _poses(_run(12, lot))
    assert first != other
    divergence = max(
        abs(a - b)
        for row_a, row_b in zip(first, other, strict=True)
        for robot_a, robot_b in zip(row_a, row_b, strict=True)
        for a, b in zip(robot_a, robot_b, strict=True)
    )
    assert divergence > 1e-6


def test_the_seed_changes_stall_occupancy_too(lot: LotSpec) -> None:
    a, b = SimLite(), SimLite()
    a.reset(11, lot)
    b.reset(12, lot)
    assert a.occupied_space_ids() != b.occupied_space_ids()
    c = SimLite()
    c.reset(11, lot)
    assert c.occupied_space_ids() == a.occupied_space_ids()


def test_sensor_output_is_reproducible_too(lot: LotSpec) -> None:
    """Ranges feed the reactive avoidance layer, so they are part of the trajectory."""
    scans = []
    for _ in range(2):
        world = SimLite()
        world.reset(11, lot)
        run = []
        for step_index in range(50):
            for robot_index, robot in enumerate(world.robots()):
                world.actuate(robot, _command(step_index, robot_index))
            world.step(world.config.dt_s)
            run.append(tuple(world.sense(robot).ranges for robot in world.robots()))
        scans.append(run)
    assert scans[0] == scans[1]


def test_no_global_rng_is_involved(lot: LotSpec) -> None:
    """Perturbing every global random stream between runs must change nothing.

    A single ``random.random()`` or ``np.random.rand()` anywhere in the simulator would
    make a run depend on whatever else the process had done first -- test ordering
    included -- and this test would fail.
    """
    random.seed(1)
    np.random.seed(1)
    first = _run(11, lot, steps=120)

    random.seed(999)
    for _ in range(37):
        random.random()
    np.random.seed(999)
    np.random.random(53)

    second = _run(11, lot, steps=120)
    assert first == second


def test_the_sim_clock_is_exactly_the_tick_count(lot: LotSpec) -> None:
    """now_ns() is tick * dt_ns. Never the wall clock, and never drifting."""
    config = SimConfig()
    world = SimLite(config)
    world.reset(3, lot)
    assert world.now_ns() == 0
    dt_ns = round(config.dt_s * 1_000_000_000)
    for tick in range(1, 51):
        world.step(config.dt_s)
        assert world.now_ns() == tick * dt_ns


def test_config_is_part_of_the_run_identity(lot: LotSpec) -> None:
    """Two configs that differ must not silently produce the same trajectory."""
    base = SimConfig()
    slower = SimConfig(limits=TwistLimits(max_v_mps=0.4, max_omega_radps=1.5))
    assert _poses(_run(11, lot, steps=80, config=base)) != _poses(
        _run(11, lot, steps=80, config=slower)
    )


@pytest.mark.parametrize("seed", [0, 1, 7, 2**31 - 1])
def test_reproducibility_holds_across_several_seeds(lot: LotSpec, seed: int) -> None:
    assert _run(seed, lot, steps=60) == _run(seed, lot, steps=60)

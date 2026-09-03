"""SimLite: protocol conformance, the clock, the fleet, charging, and collision reporting."""

from __future__ import annotations

import math

import pytest
from parkscan_sim.battery import BatteryParams
from parkscan_sim.config import SimConfig
from parkscan_sim.obstacles import stall_aabb
from parkscan_sim.world import Collision, CollisionKind, SimLite

from parkscan_core.lot import LotSpec
from parkscan_core.types import BayState, Pose2D, TwistCmd
from parkscan_core.world import WorldBackend

SEED = 11


@pytest.fixture
def sim(lot: LotSpec) -> SimLite:
    world = SimLite()
    world.reset(SEED, lot)
    return world


# ---------------------------------------------------------------- the interface --
def test_simlite_satisfies_the_world_backend_protocol() -> None:
    assert isinstance(SimLite(), WorldBackend)


def test_robots_are_named_robot_1_upwards_in_creation_order(sim: SimLite) -> None:
    assert tuple(sim.robots()) == ("robot_1", "robot_2", "robot_3")
    assert sim.add_robot(Pose2D(x_m=60.0, y_m=45.0, theta_rad=0.0)) == "robot_4"
    assert tuple(sim.robots()) == ("robot_1", "robot_2", "robot_3", "robot_4")


def test_robot_count_is_configurable(lot: LotSpec) -> None:
    world = SimLite(SimConfig(robot_count=1))
    world.reset(SEED, lot)
    assert tuple(world.robots()) == ("robot_1",)


def test_everything_raises_clearly_before_the_first_reset() -> None:
    world = SimLite()
    assert tuple(world.robots()) == ()
    assert world.now_ns() == 0
    for call in (
        lambda: world.step(0.05),
        lambda: world.sense("robot_1"),
        lambda: world.actuate("robot_1", TwistCmd(0.0, 0.0)),
        lambda: world.add_robot(Pose2D(0.0, 0.0, 0.0)),
        lambda: world.seed,
        lambda: world.lot,
    ):
        with pytest.raises(RuntimeError, match="reset"):
            call()


def test_unknown_robot_id_raises_a_useful_key_error(sim: SimLite) -> None:
    with pytest.raises(KeyError, match="robot_99"):
        sim.sense("robot_99")


# --------------------------------------------------------------------- the clock --
def test_now_ns_is_the_sim_clock_and_starts_at_zero(sim: SimLite) -> None:
    assert sim.now_ns() == 0
    for _ in range(10):
        sim.step(sim.config.dt_s)
    assert sim.now_ns() == 10 * 50_000_000  # 0.05 s ticks, exactly


def test_step_rejects_any_tick_other_than_the_configured_one(sim: SimLite) -> None:
    with pytest.raises(ValueError, match="fixed tick"):
        sim.step(0.04)
    for bad in (0.0, -0.05, math.nan, math.inf):
        with pytest.raises(ValueError, match="dt"):
            sim.step(bad)


# ------------------------------------------------------------------ start layout --
def test_start_positions_come_from_the_lot_not_the_seed(lot: LotSpec) -> None:
    """Bay nodes first, then the lane node nearest the first bay -- seed-independent."""
    nodes = lot.lane_graph.node_map()
    expected = [
        (nodes["bay_1_node"].x_m, nodes["bay_1_node"].y_m),
        (nodes["bay_2_node"].x_m, nodes["bay_2_node"].y_m),
        (nodes["wl_s1"].x_m, nodes["wl_s1"].y_m),
    ]
    for seed in (1, 2, 3):
        world = SimLite()
        world.reset(seed, lot)
        got = [(world.pose(r).x_m, world.pose(r).y_m) for r in world.robots()]
        assert got == expected


def test_start_headings_are_lot_derived_plus_a_small_seeded_jitter(lot: LotSpec) -> None:
    """Robot 1 sits on the bay spur and faces its only neighbour, back down the lane."""
    jitter = SimConfig().start_heading_jitter_rad
    headings = []
    for seed in (1, 2, 3):
        world = SimLite()
        world.reset(seed, lot)
        theta = world.pose("robot_1").theta_rad
        assert abs(abs(theta) - math.pi) <= jitter  # facing west, +- the jitter
        headings.append(theta)
    assert len(set(headings)) == 3  # the jitter really is seeded


def test_the_fleet_starts_clear_of_every_obstacle(sim: SimLite) -> None:
    assert sim.collision_count() == 0
    assert sim.active_collision_count() == 0


# ------------------------------------------------------------------------ sense --
def test_sense_reports_the_contract(sim: SimLite) -> None:
    frame = sim.sense("robot_1")
    assert frame.stamp_ns == sim.now_ns()
    assert frame.pose == sim.pose("robot_1")
    assert frame.battery_soc == sim.battery_soc("robot_1")
    assert len(frame.ranges) == sim.config.sensor.n_rays
    assert all(r > 0.0 for r in frame.ranges)
    # Phase 2 renders nothing. Fabricating pixels here would be a lie the perception
    # contract test would then happily pass.
    assert frame.camera is None


def test_measured_twist_is_zero_until_the_first_step(sim: SimLite) -> None:
    sim.actuate("robot_3", TwistCmd(v_mps=1.0, omega_radps=0.0))
    assert sim.sense("robot_3").twist == TwistCmd(0.0, 0.0)
    sim.step(sim.config.dt_s)
    assert sim.sense("robot_3").twist.v_mps == pytest.approx(1.0)


def test_actuate_saturates_the_command(sim: SimLite) -> None:
    sim.actuate("robot_3", TwistCmd(v_mps=99.0, omega_radps=-99.0))
    sim.step(sim.config.dt_s)
    twist = sim.sense("robot_3").twist
    assert twist.v_mps == sim.config.limits.max_v_mps
    assert twist.omega_radps == -sim.config.limits.max_omega_radps


def test_a_latched_command_persists_across_ticks(sim: SimLite) -> None:
    """One actuate() must keep applying until it is replaced.

    Asserted along the robot's OWN heading rather than along +x: start headings are
    derived from the seed and robot_3 does not begin axis-aligned, so an assertion on
    x alone would be measuring the start pose, not the latch.
    """
    start = sim.pose("robot_3")
    sim.actuate("robot_3", TwistCmd(v_mps=0.5, omega_radps=0.0))
    steps = 20
    for _ in range(steps):
        sim.step(sim.config.dt_s)

    end = sim.pose("robot_3")
    expected = 0.5 * steps * sim.config.dt_s  # omega == 0, so the path is a straight line
    travelled = math.hypot(end.x_m - start.x_m, end.y_m - start.y_m)
    assert travelled == pytest.approx(expected, abs=1e-9)
    # Straight ahead: displacement direction matches the (unchanged) heading.
    assert end.theta_rad == pytest.approx(start.theta_rad, abs=1e-12)
    assert end.x_m - start.x_m == pytest.approx(expected * math.cos(start.theta_rad), abs=1e-9)
    assert end.y_m - start.y_m == pytest.approx(expected * math.sin(start.theta_rad), abs=1e-9)


def test_a_forward_ray_sees_the_west_wall_at_the_analytic_distance(lot: LotSpec) -> None:
    """robot_1 starts on the bay spur at x = 8 facing the west wall at x = 0."""
    world = SimLite(SimConfig(start_heading_jitter_rad=1e-12))
    world.reset(SEED, lot)
    params = world.config.sensor
    ranges = world.sense("robot_1").ranges
    assert ranges[params.forward_index] == pytest.approx(8.0, abs=1e-6)


# ---------------------------------------------------------------------- battery --
def test_a_robot_docked_at_a_bay_charges(lot: LotSpec) -> None:
    world = SimLite(SimConfig(initial_soc_min=0.5, initial_soc_max=0.5))
    world.reset(SEED, lot)
    before = world.battery_soc("robot_1")
    world.step(world.config.dt_s)
    assert world.is_charging("robot_1")
    assert world.battery_soc("robot_1") > before


def test_a_robot_driving_away_from_the_bay_drains(sim: SimLite) -> None:
    sim.actuate("robot_1", TwistCmd(v_mps=1.0, omega_radps=0.0))
    for _ in range(40):  # 2 s: far enough out of the dock radius
        sim.step(sim.config.dt_s)
    assert not sim.is_charging("robot_1")
    start = sim.battery_soc("robot_1")
    for _ in range(40):
        sim.step(sim.config.dt_s)
    assert sim.battery_soc("robot_1") < start


def test_a_flat_robot_cannot_move(sim: SimLite) -> None:
    sim.place("robot_3", Pose2D(x_m=70.0, y_m=45.0, theta_rad=0.0), battery_soc=0.0)
    sim.actuate("robot_3", TwistCmd(v_mps=1.2, omega_radps=0.9))
    before = sim.pose("robot_3")
    sim.step(sim.config.dt_s)
    assert sim.is_depleted("robot_3")
    assert sim.pose("robot_3") == before
    assert sim.sense("robot_3").twist == TwistCmd(0.0, 0.0)


def test_battery_parameters_are_configurable(lot: LotSpec) -> None:
    # Derive the horizon instead of guessing it. At v = 1.0 m/s the linear proxy draws
    # idle_w + move_w_per_mps * v; a 1.0 Wh pack lasts capacity_wh / power hours. The
    # original form of this test ran 20 s against a pack good for ~23 s and asserted
    # depletion, so it failed for arithmetic reasons rather than behavioural ones.
    params = BatteryParams(capacity_wh=1.0, idle_w=100.0)
    thirsty = SimConfig(battery=params)
    world = SimLite(thirsty)
    world.reset(SEED, lot)

    speed = 1.0
    power_w = params.idle_w + params.move_w_per_mps * speed
    seconds_to_empty = params.capacity_wh / power_w * 3600.0
    steps = int(seconds_to_empty / world.config.dt_s) + 5  # a few ticks past empty

    world.actuate("robot_3", TwistCmd(v_mps=speed, omega_radps=0.0))
    for _ in range(steps):
        world.step(world.config.dt_s)

    assert world.is_depleted("robot_3")
    assert world.battery_soc("robot_3") == 0.0  # clamped at zero, never negative


# -------------------------------------------------------------------- occupancy --
def test_ground_truth_occupancy_covers_every_stall_and_matches_the_obstacles(
    sim: SimLite, lot: LotSpec
) -> None:
    truth = sim.ground_truth_occupancy()
    assert len(truth) == len(lot.parking_spaces)
    assert set(truth) == {space.id for space in lot.parking_spaces}
    occupied = {sid for sid, state in truth.items() if state is BayState.OCCUPIED}
    assert occupied == set(sim.occupied_space_ids())
    assert BayState.UNKNOWN not in truth.values()  # the sim knows; the fleet does not
    car_ids = {rect.id for rect in sim.obstacles().rects if rect.id.startswith("car_")}
    assert car_ids == {f"car_{space_id}" for space_id in occupied}


def test_occupied_stall_ids_follow_the_lot_order(sim: SimLite, lot: LotSpec) -> None:
    order = [space.id for space in lot.parking_spaces]
    positions = [order.index(space_id) for space_id in sim.occupied_space_ids()]
    assert positions == sorted(positions)


# ------------------------------------------------------------------- collisions --
def test_two_overlapping_robots_are_reported(sim: SimLite) -> None:
    radius = sim.config.robot_radius_m
    sim.place("robot_1", Pose2D(x_m=70.0, y_m=45.0, theta_rad=0.0))
    sim.place("robot_2", Pose2D(x_m=70.0 + radius, y_m=45.0, theta_rad=0.0))
    assert sim.collision_count() == 1
    event = sim.collisions()[0]
    assert isinstance(event, Collision)
    assert event.kind is CollisionKind.ROBOT_ROBOT
    assert {event.robot, event.other} == {"robot_1", "robot_2"}
    assert event.overlap_m == pytest.approx(radius)


def test_an_ongoing_overlap_is_one_episode_not_one_record_per_tick(sim: SimLite) -> None:
    sim.place("robot_1", Pose2D(x_m=70.0, y_m=45.0, theta_rad=0.0))
    sim.place("robot_2", Pose2D(x_m=70.1, y_m=45.0, theta_rad=0.0))
    assert sim.collision_count() == 1
    for _ in range(20):
        sim.step(sim.config.dt_s)
    assert sim.collision_count() == 1
    assert sim.active_collision_count() == 1


def test_separating_and_re_overlapping_is_a_second_episode(sim: SimLite) -> None:
    sim.place("robot_1", Pose2D(x_m=70.0, y_m=45.0, theta_rad=0.0))
    sim.place("robot_2", Pose2D(x_m=70.1, y_m=45.0, theta_rad=0.0))
    sim.place("robot_2", Pose2D(x_m=80.0, y_m=45.0, theta_rad=0.0))
    assert sim.active_collision_count() == 0
    sim.place("robot_2", Pose2D(x_m=70.1, y_m=45.0, theta_rad=0.0))
    assert sim.collision_count() == 2


def test_collisions_are_reported_and_never_resolved(sim: SimLite) -> None:
    """C1 would be vacuous if the simulator quietly pushed overlapping robots apart."""
    sim.place("robot_1", Pose2D(x_m=70.0, y_m=45.0, theta_rad=0.0))
    sim.place("robot_2", Pose2D(x_m=70.1, y_m=45.0, theta_rad=0.0))
    for _ in range(10):
        sim.step(sim.config.dt_s)
    separation = math.dist(
        (sim.pose("robot_1").x_m, sim.pose("robot_1").y_m),
        (sim.pose("robot_2").x_m, sim.pose("robot_2").y_m),
    )
    assert separation == pytest.approx(0.1)
    assert separation < 2.0 * sim.config.robot_radius_m


def test_driving_into_a_parked_car_is_reported(sim: SimLite, lot: LotSpec) -> None:
    space_id = sim.occupied_space_ids()[0]
    space = next(s for s in lot.parking_spaces if s.id == space_id)
    x_min, y_min, x_max, y_max = stall_aabb(space.polygon)
    sim.place(
        "robot_3", Pose2D(x_m=(x_min + x_max) / 2.0, y_m=(y_min + y_max) / 2.0, theta_rad=0.0)
    )
    events = [e for e in sim.collisions() if e.kind is CollisionKind.ROBOT_OBSTACLE]
    assert [e.other for e in events] == [f"car_{space_id}"]
    assert events[0].robot == "robot_3"


def test_leaving_the_lot_hits_a_boundary_wall(sim: SimLite) -> None:
    sim.place("robot_3", Pose2D(x_m=0.1, y_m=50.0, theta_rad=0.0))
    events = sim.collisions()
    assert [(e.kind, e.other) for e in events] == [(CollisionKind.ROBOT_OBSTACLE, "wall_west")]
    assert events[0].overlap_m == pytest.approx(sim.config.robot_radius_m - 0.1)


def test_place_rejects_an_out_of_range_state_of_charge(sim: SimLite) -> None:
    with pytest.raises(ValueError, match="battery_soc"):
        sim.place("robot_1", Pose2D(0.0, 0.0, 0.0), battery_soc=1.5)


# ------------------------------------------------------------------------ reset --
def test_reset_clears_every_trace_of_the_previous_run(lot: LotSpec) -> None:
    world = SimLite()
    world.reset(SEED, lot)
    world.add_robot(Pose2D(x_m=60.0, y_m=45.0, theta_rad=0.0))
    world.place("robot_1", Pose2D(x_m=70.0, y_m=45.0, theta_rad=0.0))
    world.place("robot_2", Pose2D(x_m=70.1, y_m=45.0, theta_rad=0.0))
    for _ in range(25):
        world.step(world.config.dt_s)
    assert world.collision_count() > 0
    assert world.now_ns() > 0

    world.reset(SEED, lot)
    fresh = SimLite()
    fresh.reset(SEED, lot)
    assert tuple(world.robots()) == ("robot_1", "robot_2", "robot_3")
    assert world.now_ns() == 0
    assert world.collision_count() == 0
    assert world.active_collision_count() == 0
    assert world.occupied_space_ids() == fresh.occupied_space_ids()
    for robot in world.robots():
        assert world.pose(robot) == fresh.pose(robot)
        assert world.battery_soc(robot) == fresh.battery_soc(robot)


def test_reset_can_swap_the_lot(lot: LotSpec) -> None:
    world = SimLite()
    world.reset(SEED, lot)
    assert world.lot is lot
    assert world.seed == SEED

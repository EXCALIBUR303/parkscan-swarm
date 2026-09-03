"""Navigation control and the return-to-charge decision."""

from __future__ import annotations

import math

import pytest
from parkscan_agent.battery_policy import (
    BatteryParams,
    nearest_bay,
    required_soc,
    should_return,
)
from parkscan_agent.lanegraph import node_xy
from parkscan_agent.navigation import (
    NavConfig,
    avoid,
    follow_route,
    has_arrived,
    waypoint_command,
)

from parkscan_core.lot import LotSpec
from parkscan_core.testing import FakeBackend
from parkscan_core.types import Pose2D, TwistCmd

CFG = NavConfig()


# ------------------------------------------------------------------- controller --
def test_a_target_dead_ahead_produces_forward_motion_and_no_turn() -> None:
    pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    cmd = waypoint_command(pose, (10.0, 0.0), CFG)
    assert cmd.v_mps > 0.0
    assert cmd.omega_radps == pytest.approx(0.0, abs=1e-9)


def test_a_target_behind_turns_in_place_rather_than_driving_away() -> None:
    """Driving forward at a target behind you is the classic pure-pursuit failure."""
    pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    cmd = waypoint_command(pose, (-10.0, 0.0), CFG)
    assert abs(cmd.omega_radps) > 0.0
    assert cmd.v_mps == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("bearing", [0.4, -0.4, 1.2, -1.2])
def test_turn_direction_always_matches_the_sign_of_the_bearing_error(bearing: float) -> None:
    pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    target = (10.0 * math.cos(bearing), 10.0 * math.sin(bearing))
    cmd = waypoint_command(pose, target, CFG)
    assert math.copysign(1.0, cmd.omega_radps) == math.copysign(1.0, bearing)


def test_commands_never_exceed_the_configured_limits() -> None:
    for theta in [i * math.pi / 8 for i in range(16)]:
        cmd = waypoint_command(Pose2D(x_m=0.0, y_m=0.0, theta_rad=theta), (30.0, -20.0), CFG)
        assert abs(cmd.v_mps) <= CFG.max_v_mps + 1e-12
        assert abs(cmd.omega_radps) <= CFG.max_omega_radps + 1e-12


def test_speed_falls_off_inside_the_slow_radius() -> None:
    pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    far = waypoint_command(pose, (20.0, 0.0), CFG).v_mps
    near = waypoint_command(pose, (CFG.slow_radius_m * 0.4, 0.0), CFG).v_mps
    assert 0.0 < near < far


def test_has_arrived_respects_the_tolerance() -> None:
    pose = Pose2D(x_m=1.0, y_m=1.0, theta_rad=0.0)
    assert has_arrived(pose, (1.0, 1.2), 0.25)
    assert not has_arrived(pose, (1.0, 2.0), 0.25)


# ------------------------------------------------ closed loop, on a fake backend --
def test_the_controller_actually_reaches_a_waypoint(backend_factory: type[FakeBackend]) -> None:
    """Closed loop against the fake backend defined in conftest -- not the simulator."""
    backend = backend_factory({"r": Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)})
    target = (12.0, 7.0)
    for _ in range(4000):
        pose = backend.pose("r")
        if has_arrived(pose, target, CFG.arrival_tolerance_m):
            break
        backend.actuate("r", waypoint_command(pose, target, CFG))
        backend.step(backend.dt_s)
    assert has_arrived(backend.pose("r"), target, CFG.arrival_tolerance_m)


def test_follow_route_walks_every_waypoint_in_order(backend_factory: type[FakeBackend]) -> None:
    backend = backend_factory({"r": Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)})
    waypoints = [(5.0, 0.0), (5.0, 5.0), (0.0, 5.0)]
    index = 0
    seen: list[int] = []
    finished = False
    for _ in range(8000):
        progress = follow_route(backend.pose("r"), waypoints, index, CFG)
        if progress.index != index:
            seen.append(progress.index)
        index = progress.index
        if progress.finished:
            finished = True
            break
        backend.actuate("r", progress.cmd)
        backend.step(backend.dt_s)

    # `index` names the waypoint currently being pursued, not a count of completed ones,
    # so completion is signalled by `finished` rather than by index == len(waypoints).
    assert finished, f"route never completed; stalled at index {index}"
    assert seen == [1, 2], "waypoints must be consumed strictly in order"
    assert has_arrived(backend.pose("r"), waypoints[-1], CFG.arrival_tolerance_m)


# ---------------------------------------------------------------- avoidance ------
def test_avoidance_stops_the_robot_when_something_is_very_close() -> None:
    cmd = TwistCmd(v_mps=1.0, omega_radps=0.0)
    blocked = (CFG.avoid_stop_m * 0.5,) * 9
    assert avoid(cmd, blocked, CFG).v_mps == pytest.approx(0.0)


def test_avoidance_leaves_a_clear_path_untouched() -> None:
    cmd = TwistCmd(v_mps=1.0, omega_radps=0.3)
    assert avoid(cmd, (math.inf,) * 9, CFG) == cmd


def test_avoidance_only_slows_or_deflects_and_never_reverses() -> None:
    """The planner owns the route; avoidance may not decide to go somewhere else."""
    cmd = TwistCmd(v_mps=1.0, omega_radps=0.0)
    for near in (0.5, 0.8, 1.2, 1.5):
        out = avoid(cmd, (near,) * 9, CFG)
        assert 0.0 <= out.v_mps <= cmd.v_mps + 1e-12


# ----------------------------------------------------------- battery policy ------
def test_a_full_battery_never_triggers_a_return(lot: LotSpec) -> None:
    pose = Pose2D(x_m=75.0, y_m=50.0, theta_rad=0.0)
    assert not should_return(1.0, pose, lot, BatteryParams()).must_return


def test_an_empty_battery_always_triggers_a_return(lot: LotSpec) -> None:
    pose = Pose2D(x_m=75.0, y_m=50.0, theta_rad=0.0)
    assert should_return(0.02, pose, lot, BatteryParams()).must_return


def test_the_threshold_is_a_feasibility_check_not_a_fixed_percentage(lot: LotSpec) -> None:
    """The whole point of the policy: far from a bay you must turn back sooner.

    A fixed 20% rule is wasteful next to the bay and fatal at the far corner. This asserts
    the required state of charge actually scales with distance home.
    """
    params = BatteryParams()
    bx, by = node_xy(lot, lot.charging_bays[0].node)
    near_pose = Pose2D(x_m=bx + 2.0, y_m=by, theta_rad=0.0)
    far_pose = Pose2D(x_m=145.0, y_m=95.0, theta_rad=0.0)

    near_req = should_return(1.0, near_pose, lot, params).required_soc
    far_req = should_return(1.0, far_pose, lot, params).required_soc
    assert far_req > near_req, "requirement must scale with distance home"

    # There must EXIST a charge level that is safe next to the bay and unsafe at the far
    # corner. That is precisely what "feasibility check, not fixed percentage" means; a
    # single hardcoded soc would only test one arbitrary point on the curve.
    between = (near_req + far_req) / 2.0
    assert should_return(between, far_pose, lot, params).must_return
    assert not should_return(between, near_pose, lot, params).must_return


def test_required_soc_grows_monotonically_with_distance() -> None:
    params = BatteryParams()
    values = [required_soc(d, params) for d in (0.0, 10.0, 50.0, 200.0)]
    assert values == sorted(values)
    # At zero distance there is still the cost of docking plus the untouchable reserve.
    assert values[0] == pytest.approx(params.soc_dock + params.soc_reserve)


def test_nearest_bay_picks_the_closer_of_the_two(lot: LotSpec) -> None:
    """The lot has exactly two bays, deliberately fewer than the three robots."""
    assert len(lot.charging_bays) == 2
    south = nearest_bay(lot, 8.0, 20.0)
    north = nearest_bay(lot, 8.0, 65.0)
    assert south.bay_id != north.bay_id
    assert south.distance_m >= 0.0 and north.distance_m >= 0.0

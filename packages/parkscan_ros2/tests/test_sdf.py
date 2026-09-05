"""``generate_world_sdf`` is a pure function: given a lot and a fixed occupied-stall set,
it must always render the same well-formed SDF. These tests check structure and the
things a Gazebo load can't cheaply catch in CI (this suite has no Gazebo) -- per-robot
topic uniqueness above all, since that bug shipped once already (found only by loading
the SDF for real; see the git history on this file's commit)."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from parkscan_ros2.sdf import LIDAR_N_RAYS, generate_world_sdf

from parkscan_core.lot import LotSpec, load_lot

REPO_ROOT = Path(__file__).resolve().parents[3]
LOT_YAML = REPO_ROOT / "sim" / "lot" / "lot.yaml"

ROBOT_POSES = (
    ("robot_1", 8.0, 17.5, 0.0),
    ("robot_2", 8.0, 67.5, 0.0),
    ("robot_3", 2.5, 17.5, 0.0),
)


@pytest.fixture(scope="module")
def lot() -> LotSpec:
    return load_lot(LOT_YAML)


def test_the_output_is_well_formed_xml(lot: LotSpec) -> None:
    sdf = generate_world_sdf(lot, occupied_space_ids=(), robot_poses=ROBOT_POSES, dt_s=0.05)
    ET.fromstring(sdf)  # raises ET.ParseError if malformed


def test_every_robot_gets_its_own_scan_odom_and_cmd_vel_topic(lot: LotSpec) -> None:
    """The bug this pins: the lidar/odometry topics do not auto-scope per model the way
    the velocity-control topic does. Without a robot-specific topic string, all three
    robots' scans and odometry collide onto one shared topic -- found only by loading the
    generated SDF in a real Gazebo and listing live topics, not by reading its docs."""
    sdf = generate_world_sdf(lot, occupied_space_ids=(), robot_poses=ROBOT_POSES, dt_s=0.05)
    for robot_id, _, _, _ in ROBOT_POSES:
        assert f"<topic>{robot_id}/scan</topic>" in sdf
        assert f"<odom_topic>{robot_id}/odom</odom_topic>" in sdf
        assert f"<topic>{robot_id}/cmd_vel</topic>" in sdf


def test_the_lidar_matches_sensorparams_defaults_exactly(lot: LotSpec) -> None:
    sdf = generate_world_sdf(lot, occupied_space_ids=(), robot_poses=ROBOT_POSES, dt_s=0.05)
    assert sdf.count(f"<samples>{LIDAR_N_RAYS}</samples>") == len(ROBOT_POSES)


def test_every_robot_has_zero_friction_on_its_collision_surface() -> None:
    """The bug this pins: DART's default friction damped a commanded 0.8 m/s down to a
    steady ~0.31 m/s -- verified directly by commanding a real robot in a real Gazebo
    and measuring its actual velocity, not by reading a physics manual. Zero friction on
    the collision geometry restored velocity tracking to within float precision of
    commanded. SimLite has no friction model either, so this keeps the two backends
    comparable rather than one silently damping motion the other does not."""
    lot = load_lot(LOT_YAML)
    sdf = generate_world_sdf(lot, occupied_space_ids=(), robot_poses=ROBOT_POSES, dt_s=0.05)
    assert sdf.count("<mu>0</mu><mu2>0</mu2>") == len(ROBOT_POSES)


def test_the_step_size_matches_dt_s_exactly(lot: LotSpec) -> None:
    sdf = generate_world_sdf(lot, occupied_space_ids=(), robot_poses=ROBOT_POSES, dt_s=0.05)
    assert "<max_step_size>0.05</max_step_size>" in sdf


def test_only_occupied_spaces_get_a_car_box(lot: LotSpec) -> None:
    occupied = ("space_01_00", "space_03_05")
    sdf = generate_world_sdf(
        lot, occupied_space_ids=occupied, robot_poses=ROBOT_POSES, dt_s=0.05
    )
    assert 'model name="car_space_01_00"' in sdf
    assert 'model name="car_space_03_05"' in sdf
    assert 'model name="car_space_02_00"' not in sdf


def test_the_four_boundary_walls_are_always_present(lot: LotSpec) -> None:
    sdf = generate_world_sdf(lot, occupied_space_ids=(), robot_poses=ROBOT_POSES, dt_s=0.05)
    for wall in ("wall_south", "wall_north", "wall_west", "wall_east"):
        assert f'model name="{wall}"' in sdf


def test_rejects_a_non_positive_dt(lot: LotSpec) -> None:
    with pytest.raises(ValueError, match="dt_s"):
        generate_world_sdf(lot, occupied_space_ids=(), robot_poses=ROBOT_POSES, dt_s=0.0)


def test_rejects_an_unknown_occupied_space_id(lot: LotSpec) -> None:
    with pytest.raises(ValueError, match=r"not in lot\.parking_spaces"):
        generate_world_sdf(
            lot, occupied_space_ids=("not_a_real_space",), robot_poses=ROBOT_POSES, dt_s=0.05
        )

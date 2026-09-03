"""Range sensing: exact analytic distances, and the vectorised cast checked against
the scalar slab reference."""

from __future__ import annotations

import math

import pytest
from parkscan_sim.obstacles import (
    ObstacleField,
    Rect,
    build_obstacle_field,
    point_rect_distance,
    stall_aabb,
)
from parkscan_sim.sensors import SensorParams, cast_rays, ray_aabb_distance, ray_angles

from parkscan_core.lot import LotSpec
from parkscan_core.types import Pose2D

WALL_AHEAD = Rect("wall_ahead", 10.0, -5.0, 11.0, 5.0)
BOX_LEFT = Rect("box_left", -3.0, 2.0, -2.0, 4.0)
BOX_BEHIND = Rect("box_behind", -8.0, -1.0, -7.0, 1.0)
SCENE = ObstacleField((WALL_AHEAD, BOX_LEFT, BOX_BEHIND))


def _scalar_scan(pose: Pose2D, field: ObstacleField, params: SensorParams) -> tuple[float, ...]:
    """Reference scan: the scalar slab method, one ray at a time, min over rects."""
    out = []
    for offset in ray_angles(params):
        angle = pose.theta_rad + offset
        dir_x, dir_y = math.cos(angle), math.sin(angle)
        best = math.inf
        for rect in field.rects:
            best = min(
                best,
                ray_aabb_distance(pose.x_m, pose.y_m, dir_x, dir_y, rect, params.max_range_m),
            )
        out.append(best)
    return tuple(out)


# ------------------------------------------------------------------- ray layout --
def test_odd_ray_count_puts_a_ray_exactly_on_the_heading() -> None:
    params = SensorParams(n_rays=37, fov_rad=1.5 * math.pi)
    offsets = ray_angles(params)
    assert len(offsets) == 37
    # Exactly zero, not 2e-16: this is what makes the forward-distance test an equality.
    assert offsets[params.forward_index] == 0.0
    assert offsets[0] == pytest.approx(-params.fov_rad / 2.0, abs=1e-15)
    assert offsets[-1] == pytest.approx(params.fov_rad / 2.0, abs=1e-15)
    assert offsets[0] == -offsets[-1]
    assert list(offsets) == sorted(offsets)


def test_single_ray_sensor_points_forward() -> None:
    assert ray_angles(SensorParams(n_rays=1)) == (0.0,)


@pytest.mark.parametrize("n_rays", [1, 2, 8, 37, 180])
def test_scan_length_always_equals_n_rays(n_rays: int) -> None:
    params = SensorParams(n_rays=n_rays)
    pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.3)
    assert len(cast_rays(pose, SCENE, params)) == n_rays
    assert len(cast_rays(pose, ObstacleField(()), params)) == n_rays


@pytest.mark.parametrize(
    ("n_rays", "fov", "max_range"),
    [(0, 1.0, 1.0), (4, 0.0, 1.0), (4, 7.0, 1.0), (4, 1.0, 0.0), (4, 1.0, math.nan)],
)
def test_sensor_params_reject_nonsense(n_rays: int, fov: float, max_range: float) -> None:
    with pytest.raises(ValueError, match=r"n_rays|fov_rad|max_range_m"):
        SensorParams(n_rays=n_rays, fov_rad=fov, max_range_m=max_range)


# ------------------------------------------------------- exact analytic distances --
def test_ray_straight_at_a_wall_returns_the_exact_analytic_distance() -> None:
    # From the origin along +x, the near face of WALL_AHEAD is at x = 10.
    assert ray_aabb_distance(0.0, 0.0, 1.0, 0.0, WALL_AHEAD, 50.0) == 10.0


def test_forward_ray_of_a_full_scan_returns_the_exact_analytic_distance() -> None:
    params = SensorParams(n_rays=37, fov_rad=1.5 * math.pi, max_range_m=50.0)
    ranges = cast_rays(
        Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0), ObstacleField((WALL_AHEAD,)), params
    )
    assert ranges[params.forward_index] == 10.0


def test_diagonal_ray_returns_the_exact_analytic_distance() -> None:
    # A 45-degree ray from the origin enters the x-slab at t = 10 / cos(45 deg).
    root_half = math.sqrt(0.5)
    tall = Rect("tall", 10.0, -50.0, 11.0, 50.0)
    got = ray_aabb_distance(0.0, 0.0, root_half, root_half, tall, 50.0)
    assert got == pytest.approx(10.0 / root_half, rel=1e-15)


def test_open_space_returns_inf() -> None:
    far = ObstacleField((Rect("far", 100.0, -5.0, 101.0, 5.0),))
    params = SensorParams(n_rays=37, max_range_m=12.0)
    ranges = cast_rays(Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0), far, params)
    assert all(r == math.inf for r in ranges)


def test_a_hit_beyond_max_range_is_reported_as_no_return_not_as_the_distance() -> None:
    assert ray_aabb_distance(0.0, 0.0, 1.0, 0.0, WALL_AHEAD, 9.9) == math.inf
    assert ray_aabb_distance(0.0, 0.0, 1.0, 0.0, WALL_AHEAD, 10.0) == 10.0


def test_ray_pointing_away_from_the_box_misses() -> None:
    assert ray_aabb_distance(0.0, 0.0, -1.0, 0.0, WALL_AHEAD, 50.0) == math.inf


def test_ray_parallel_to_a_slab_and_outside_it_misses() -> None:
    # Along +x at y = 100: never enters the wall's y-slab of [-5, 5].
    assert ray_aabb_distance(0.0, 100.0, 1.0, 0.0, WALL_AHEAD, 500.0) == math.inf


def test_origin_inside_the_rect_reports_zero_not_the_far_face() -> None:
    assert ray_aabb_distance(10.5, 0.0, 1.0, 0.0, WALL_AHEAD, 50.0) == 0.0


def test_grazing_the_corner_is_a_hit_at_the_corner() -> None:
    # Straight at the wall's south-west corner (10, -5) from (0, -5).
    assert ray_aabb_distance(0.0, -5.0, 1.0, 0.0, WALL_AHEAD, 50.0) == 10.0


# ------------------------------------------------- vectorised vs scalar reference --
@pytest.mark.parametrize(
    "pose",
    [
        Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0),
        Pose2D(x_m=0.0, y_m=0.0, theta_rad=math.pi / 2.0),
        Pose2D(x_m=-1.5, y_m=3.25, theta_rad=-2.1),
        Pose2D(x_m=10.5, y_m=0.0, theta_rad=0.9),  # inside WALL_AHEAD
        Pose2D(x_m=200.0, y_m=200.0, theta_rad=0.0),  # everything out of range
    ],
)
def test_vectorised_cast_agrees_with_the_scalar_slab_reference(pose: Pose2D) -> None:
    params = SensorParams(n_rays=37, fov_rad=1.5 * math.pi, max_range_m=12.0)
    fast = cast_rays(pose, SCENE, params)
    slow = _scalar_scan(pose, SCENE, params)
    assert len(fast) == len(slow)
    for a, b in zip(fast, slow, strict=True):
        if math.isinf(a) or math.isinf(b):
            assert a == b
        else:
            assert a == pytest.approx(b, rel=1e-12, abs=1e-12)


def test_empty_field_is_all_inf() -> None:
    params = SensorParams(n_rays=8)
    ranges = cast_rays(Pose2D(x_m=1.0, y_m=1.0, theta_rad=0.0), ObstacleField(()), params)
    assert ranges == (math.inf,) * 8


# --------------------------------------------------------------- point distances --
@pytest.mark.parametrize(
    ("x", "y", "expected"),
    [
        (10.5, 0.0, 0.0),  # inside
        (9.0, 0.0, 1.0),  # west of the near face
        (12.0, 0.0, 1.0),  # east of the far face
        (9.0, 6.0, math.sqrt(2.0)),  # off the corner
    ],
)
def test_point_rect_distance(x: float, y: float, expected: float) -> None:
    assert point_rect_distance(x, y, WALL_AHEAD) == pytest.approx(expected, abs=1e-15)


def test_vectorised_point_distances_agree_with_the_scalar_reference() -> None:
    for x in (-10.0, 0.0, 9.5, 10.5, 30.0):
        for y in (-6.0, 0.0, 3.0, 12.0):
            fast = SCENE.point_distances(x, y)
            slow = [point_rect_distance(x, y, rect) for rect in SCENE.rects]
            for a, b in zip(fast, slow, strict=True):
                assert float(a) == pytest.approx(b, abs=1e-15)


# -------------------------------------------------------------- obstacle building --
def test_rect_rejects_degenerate_bounds() -> None:
    with pytest.raises(ValueError, match="degenerate"):
        Rect("bad", 1.0, 1.0, 1.0, 2.0)


def test_obstacle_field_rejects_duplicate_ids() -> None:
    with pytest.raises(ValueError, match="duplicate"):
        ObstacleField((WALL_AHEAD, WALL_AHEAD))


def test_build_obstacle_field_makes_four_walls_plus_one_car_per_occupied_stall(
    lot: LotSpec,
) -> None:
    occupied = [space.id for space in lot.parking_spaces[:5]]
    field = build_obstacle_field(lot, occupied, car_inset_m=0.3, wall_thickness_m=0.5)
    ids = field.ids
    assert ids[:4] == ("wall_south", "wall_north", "wall_west", "wall_east")
    assert len(field) == 4 + len(occupied)
    assert set(ids[4:]) == {f"car_{space_id}" for space_id in occupied}


def test_car_rectangle_is_the_stall_inset_on_every_side(lot: LotSpec) -> None:
    space = lot.parking_spaces[0]
    field = build_obstacle_field(lot, [space.id], car_inset_m=0.3, wall_thickness_m=0.5)
    car = next(rect for rect in field.rects if rect.id == f"car_{space.id}")
    x_min, y_min, x_max, y_max = stall_aabb(space.polygon)
    # approx() over the whole tuple, not per element: the elementwise form compares a
    # tuple[float, ...] against a tuple[ApproxBase, ...], which works at runtime but is a
    # type error, and pytest supports the collection form directly.
    assert (car.x_min, car.y_min, car.x_max, car.y_max) == pytest.approx(
        (x_min + 0.3, y_min + 0.3, x_max - 0.3, y_max - 0.3)
    )
    # The authored stalls are 2.5 m x 5.0 m, so the parked car is 1.9 m x 4.4 m.
    assert car.x_max - car.x_min == pytest.approx(1.9)
    assert car.y_max - car.y_min == pytest.approx(4.4)


def test_walls_lie_outside_the_lot_so_the_drivable_area_is_exactly_the_lot(lot: LotSpec) -> None:
    field = build_obstacle_field(lot, [], car_inset_m=0.3, wall_thickness_m=0.5)
    by_id = {rect.id: rect for rect in field.rects}
    assert by_id["wall_south"].y_max == 0.0
    assert by_id["wall_north"].y_min == lot.size_m.h
    assert by_id["wall_west"].x_max == 0.0
    assert by_id["wall_east"].x_min == lot.size_m.w


@pytest.mark.parametrize(("inset", "thickness"), [(0.0, 0.5), (-1.0, 0.5), (0.3, 0.0)])
def test_build_obstacle_field_rejects_nonsense(
    lot: LotSpec, inset: float, thickness: float
) -> None:
    with pytest.raises(ValueError, match=r"car_inset_m|wall_thickness_m"):
        build_obstacle_field(lot, [], car_inset_m=inset, wall_thickness_m=thickness)

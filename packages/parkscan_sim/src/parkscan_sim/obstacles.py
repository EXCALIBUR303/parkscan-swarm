"""The static obstacle set: axis-aligned rectangles derived from the lot.

``sim-lite`` models exactly two kinds of obstacle, and nothing else:

* the four **lot boundary walls**, as thin rectangles lying immediately *outside*
  ``[0, w] x [0, h]``, so a robot that drives out of the lot registers as a collision
  rather than silently leaving the world; and
* one **parked-car rectangle** per stall that this run decided is occupied.

Both are axis-aligned because the authored lot is axis-aligned (stalls are 2.5 m x 5.0 m
boxes on a 2.5 m pitch), and because an axis-aligned box has an exact, branch-free
ray intersection -- the slab method in :mod:`parkscan_sim.sensors` -- with no geometry
library and no floating-point tolerance knob.

Honesty, stated plainly: this is a **static** obstacle set on a flat plane. Cars do not
arrive or leave mid-run, nothing moves, there are no pedestrians, no kerbs, no ramps and
no occlusion beyond straight-line geometry. It is the set the planar range sensor and the
collision check agree on, and it is not a model of a real car park.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

from parkscan_core.lot import LotSpec

__all__ = [
    "ObstacleField",
    "Rect",
    "build_obstacle_field",
    "point_rect_distance",
    "stall_aabb",
]


@dataclass(frozen=True, slots=True)
class Rect:
    """An axis-aligned rectangle in lot metres, with a stable id.

    The id is what a :class:`~parkscan_sim.world.Collision` names, so it has to mean
    something to a human reading the failure: ``wall_north`` or ``car_space_03_07``.
    """

    id: str
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def __post_init__(self) -> None:
        if not self.id:
            msg = "rect id must be non-empty"
            raise ValueError(msg)
        if self.x_min >= self.x_max or self.y_min >= self.y_max:
            msg = (
                f"rect {self.id!r} is degenerate: "
                f"({self.x_min}, {self.y_min}) .. ({self.x_max}, {self.y_max})"
            )
            raise ValueError(msg)


def point_rect_distance(x_m: float, y_m: float, rect: Rect) -> float:
    """Shortest distance from a point to ``rect``; ``0.0`` when the point is inside.

    This is the scalar reference for the disc-versus-rectangle collision test. It returns
    a non-negative distance rather than a signed penetration depth: a robot whose centre
    is *inside* a parked car has already failed, and reporting how far inside would
    invite someone to treat the number as a physical response force. ``sim-lite`` reports
    collisions and never resolves them (acceptance criterion C1).
    """
    dx = max(rect.x_min - x_m, 0.0, x_m - rect.x_max)
    dy = max(rect.y_min - y_m, 0.0, y_m - rect.y_max)
    return math.hypot(dx, dy)


class ObstacleField:
    """An immutable bundle of :class:`Rect` plus the column arrays used to cast rays.

    The arrays exist because ray casting is the hot loop: 37 rays x 3 robots x every
    ``sense()`` over a 30-seed benchmark. They are built once, marked non-writeable, and
    never rebuilt, so the field a run senses against cannot drift out of step with the
    field it collides against -- one source of truth, two consumers.
    """

    __slots__ = ("_ids", "_rects", "_x_max", "_x_min", "_y_max", "_y_min")

    def __init__(self, rects: Iterable[Rect]) -> None:
        self._rects: tuple[Rect, ...] = tuple(rects)
        seen: set[str] = set()
        for rect in self._rects:
            if rect.id in seen:
                msg = f"duplicate obstacle id {rect.id!r}"
                raise ValueError(msg)
            seen.add(rect.id)
        self._ids: tuple[str, ...] = tuple(r.id for r in self._rects)
        self._x_min = self._column([r.x_min for r in self._rects])
        self._y_min = self._column([r.y_min for r in self._rects])
        self._x_max = self._column([r.x_max for r in self._rects])
        self._y_max = self._column([r.y_max for r in self._rects])

    @staticmethod
    def _column(values: Sequence[float]) -> npt.NDArray[np.float64]:
        arr = np.asarray(values, dtype=np.float64)
        arr.setflags(write=False)
        return arr

    def __len__(self) -> int:
        return len(self._rects)

    @property
    def rects(self) -> tuple[Rect, ...]:
        return self._rects

    @property
    def ids(self) -> tuple[str, ...]:
        return self._ids

    @property
    def x_min(self) -> npt.NDArray[np.float64]:
        return self._x_min

    @property
    def y_min(self) -> npt.NDArray[np.float64]:
        return self._y_min

    @property
    def x_max(self) -> npt.NDArray[np.float64]:
        return self._x_max

    @property
    def y_max(self) -> npt.NDArray[np.float64]:
        return self._y_max

    def point_distances(self, x_m: float, y_m: float) -> npt.NDArray[np.float64]:
        """Distance from ``(x_m, y_m)`` to every rect, ``0.0`` where the point is inside.

        Vectorised twin of :func:`point_rect_distance`; ``tests/test_sensors.py`` asserts
        the two agree so the collision check and its scalar reference cannot diverge.
        """
        dx = np.maximum(self._x_min - x_m, np.maximum(0.0, x_m - self._x_max))
        dy = np.maximum(self._y_min - y_m, np.maximum(0.0, y_m - self._y_max))
        return np.hypot(dx, dy)


def stall_aabb(polygon: Sequence[tuple[float, float]]) -> tuple[float, float, float, float]:
    """Axis-aligned bounds ``(x_min, y_min, x_max, y_max)`` of a stall polygon."""
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    return (min(xs), min(ys), max(xs), max(ys))


def build_obstacle_field(
    lot: LotSpec,
    occupied_space_ids: Iterable[str],
    *,
    car_inset_m: float,
    wall_thickness_m: float,
) -> ObstacleField:
    """Assemble the static obstacle set for one run.

    ``car_inset_m`` is the gap left between the stall's painted bounds and the parked
    car's footprint on every side; on the authored 2.5 m x 5.0 m stalls the default
    0.3 m yields a 1.9 m x 4.4 m car, which is a plausible saloon and, more importantly,
    is *derived from the stall geometry* rather than hard-coded, so a lot with different
    stalls stays consistent.

    Walls sit immediately outside the lot rather than on its edge, so the drivable area
    is exactly ``[0, w] x [0, h]`` and a robot only registers a wall collision once part
    of its disc is actually outside the lot.

    Rect order is deterministic: the four walls, then the occupied stalls in
    ``lot.parking_spaces`` order. Nothing here iterates a set or a dict.
    """
    if car_inset_m <= 0.0:
        msg = f"car_inset_m must be positive, got {car_inset_m}"
        raise ValueError(msg)
    if wall_thickness_m <= 0.0:
        msg = f"wall_thickness_m must be positive, got {wall_thickness_m}"
        raise ValueError(msg)

    w = lot.size_m.w
    h = lot.size_m.h
    t = wall_thickness_m
    rects: list[Rect] = [
        Rect("wall_south", -t, -t, w + t, 0.0),
        Rect("wall_north", -t, h, w + t, h + t),
        Rect("wall_west", -t, 0.0, 0.0, h),
        Rect("wall_east", w, 0.0, w + t, h),
    ]

    occupied = set(occupied_space_ids)
    for space in lot.parking_spaces:
        if space.id not in occupied:
            continue
        x_min, y_min, x_max, y_max = stall_aabb(space.polygon)
        car = Rect(
            f"car_{space.id}",
            x_min + car_inset_m,
            y_min + car_inset_m,
            x_max - car_inset_m,
            y_max - car_inset_m,
        )
        rects.append(car)
    return ObstacleField(rects)

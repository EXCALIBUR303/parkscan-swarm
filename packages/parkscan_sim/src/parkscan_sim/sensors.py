"""A planar range sensor: fixed rays, exact geometry, no noise model.

The sensor fires ``n_rays`` rays spread evenly over ``fov_rad``, centred on the robot's
heading, from the robot's origin. Each ray returns the distance in metres to the nearest
static obstacle, or ``math.inf`` when nothing is hit inside ``max_range_m``. The count and
the angular layout are fixed for the whole run, which is what
:class:`~parkscan_core.types.SensorFrame` promises its consumers.

Two deliberate properties
-------------------------
**It is pure.** ``cast_rays(pose, field, params)`` depends on nothing but its arguments --
no clock, no RNG, no simulator state -- so a reactive-avoidance layer can be unit-tested
against a hand-built three-rectangle scene with no simulator at all.

**There is no noise model.** These are exact geometric ranges. ``sim-lite`` does not
pretend to model a real lidar's dropouts, mixed pixels, retroreflector blooming or range
bias, and no number derived from them may be described as sensor accuracy. Adding
Gaussian noise would look more realistic and *be* less honest: it would be a knob whose
value nobody measured, and it would put an RNG draw inside ``sense()``, which is exactly
where determinism goes to die.

Intersection is the **slab method** against axis-aligned boxes, implemented directly:
per axis, the ray's entry and exit parameters into the box's slab are ``t = (bound - o)/d``;
the box is hit when the largest entry is no greater than the smallest exit. Rays exactly
parallel to an axis (``d == 0``) are handled explicitly rather than by adding an epsilon
to the denominator, because an epsilon there silently turns a grazing miss into a hit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

from parkscan_core.types import Pose2D
from parkscan_sim.obstacles import ObstacleField, Rect

__all__ = ["SensorParams", "cast_rays", "ray_aabb_distance", "ray_angles"]

_TWO_PI = 2.0 * math.pi


@dataclass(frozen=True, slots=True)
class SensorParams:
    """Fixed layout of the planar scan.

    The default ``n_rays`` is **odd** on purpose: with an odd count the middle ray,
    index ``n_rays // 2``, points exactly along the robot's heading (offset exactly
    ``0.0``, not ``-2.2e-16``), which makes "what is directly in front of me" an exactly
    testable question rather than an approximately testable one.

    270 degrees at 37 rays is 7.5 degrees of angular resolution -- coarse, and stated as
    such. It is enough for the VFH-style deflection layer described in
    ``docs/ARCHITECTURE.md`` section 3 and it is not a model of any particular device.
    """

    n_rays: int = 37
    fov_rad: float = 1.5 * math.pi
    max_range_m: float = 12.0

    def __post_init__(self) -> None:
        if self.n_rays < 1:
            msg = f"n_rays must be at least 1, got {self.n_rays}"
            raise ValueError(msg)
        if not math.isfinite(self.fov_rad) or not 0.0 < self.fov_rad <= _TWO_PI:
            msg = f"fov_rad must lie in (0, 2*pi], got {self.fov_rad}"
            raise ValueError(msg)
        if not math.isfinite(self.max_range_m) or self.max_range_m <= 0.0:
            msg = f"max_range_m must be finite and positive, got {self.max_range_m}"
            raise ValueError(msg)

    @property
    def forward_index(self) -> int:
        """Index of the ray along the heading. Exactly forward iff ``n_rays`` is odd."""
        return self.n_rays // 2


def ray_angles(params: SensorParams) -> tuple[float, ...]:
    """Ray offsets from the robot heading, from ``-fov/2`` to ``+fov/2`` inclusive.

    Written as ``(i - (n-1)/2) * step`` rather than ``linspace``/``-fov/2 + i*step``
    because only that form makes the centre offset of an odd-count scan come out as
    exactly ``0.0``. The other two forms leave a ``2e-16`` residue there, and a residue is
    enough to turn "the forward ray hits the wall at exactly 10.0 m" into a test that
    needs a tolerance.
    """
    if params.n_rays == 1:
        return (0.0,)
    half = (params.n_rays - 1) / 2.0
    step = params.fov_rad / (params.n_rays - 1)
    return tuple((i - half) * step for i in range(params.n_rays))


def _slab_scalar(o: float, d: float, lo: float, hi: float) -> tuple[float, float]:
    """Entry/exit ray parameters for one axis slab. A miss is ``(+inf, -inf)``."""
    if d == 0.0:
        if o < lo or o > hi:
            return (math.inf, -math.inf)
        return (-math.inf, math.inf)
    t1 = (lo - o) / d
    t2 = (hi - o) / d
    return (t1, t2) if t1 <= t2 else (t2, t1)


def ray_aabb_distance(
    origin_x: float,
    origin_y: float,
    dir_x: float,
    dir_y: float,
    rect: Rect,
    max_range_m: float,
) -> float:
    """Distance along a **unit** ray to ``rect``, or ``math.inf`` for no hit.

    Scalar reference implementation of the slab method. ``cast_rays`` is the vectorised
    twin and ``tests/test_sensors.py`` asserts they agree; this one exists so the method
    can be read and checked against a hand-computed number.

    ``(dir_x, dir_y)`` must be unit length -- ``t`` is measured in units of the direction
    vector, so a non-unit direction silently rescales every range. An origin *inside* the
    rectangle returns ``0.0``: the sensor is inside an obstacle, which is a fact worth
    surfacing, not a hit at the far wall.
    """
    near_x, far_x = _slab_scalar(origin_x, dir_x, rect.x_min, rect.x_max)
    near_y, far_y = _slab_scalar(origin_y, dir_y, rect.y_min, rect.y_max)
    t_near = max(near_x, near_y)
    t_far = min(far_x, far_y)
    if t_far < t_near or t_far < 0.0:
        return math.inf
    t = t_near if t_near >= 0.0 else 0.0
    return t if t <= max_range_m else math.inf


def _slab_vec(
    o: float,
    d: npt.NDArray[np.float64],
    lo: npt.NDArray[np.float64],
    hi: npt.NDArray[np.float64],
) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
    """Vectorised :func:`_slab_scalar`, broadcasting ``(rays, 1)`` against ``(1, rects)``."""
    with np.errstate(divide="ignore", invalid="ignore"):
        # Division by an exactly-zero component yields +-inf (or nan when the numerator
        # is also zero); every such element is overwritten below, so the warnings are
        # suppressed rather than dodged with an epsilon.
        t1 = (lo - o) / d
        t2 = (hi - o) / d
    parallel = d == 0.0
    near = np.where(parallel, -np.inf, np.minimum(t1, t2))
    far = np.where(parallel, np.inf, np.maximum(t1, t2))
    outside = parallel & ((o < lo) | (o > hi))
    return np.where(outside, np.inf, near), np.where(outside, -np.inf, far)


def cast_rays(
    pose: Pose2D,
    field: ObstacleField,
    params: SensorParams,
) -> tuple[float, ...]:
    """Cast the whole scan from ``pose`` against ``field``.

    Pure: no simulator state, no RNG, no clock. Returns exactly ``params.n_rays`` values
    in metres, with ``math.inf`` for "no return within ``max_range_m``".
    """
    offsets = ray_angles(params)
    if len(field) == 0:
        return tuple(math.inf for _ in offsets)

    angles = np.asarray(offsets, dtype=np.float64) + pose.theta_rad
    dir_x = np.cos(angles)[:, None]
    dir_y = np.sin(angles)[:, None]

    near_x, far_x = _slab_vec(pose.x_m, dir_x, field.x_min[None, :], field.x_max[None, :])
    near_y, far_y = _slab_vec(pose.y_m, dir_y, field.y_min[None, :], field.y_max[None, :])

    t_near = np.maximum(near_x, near_y)
    t_far = np.minimum(far_x, far_y)
    hit = (t_far >= t_near) & (t_far >= 0.0)
    t = np.where(t_near >= 0.0, t_near, 0.0)

    per_rect = np.where(hit, t, np.inf)
    nearest = per_rect.min(axis=1)
    ranges = np.where(nearest <= params.max_range_m, nearest, np.inf)
    return tuple(float(r) for r in ranges)

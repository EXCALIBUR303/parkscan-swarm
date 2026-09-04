"""Forward projection: a known ground point -> the pixel a camera would see it at.

This is the **inverse** of the geolocation stage under test, and it is deliberately the
only piece of projection maths this package owns. ``parkscan_perception.geolocation``
back-projects pixels onto the ground plane; the harness must be able to go the other way
to manufacture an input whose correct answer is known exactly. Implementing the forward
direction here, and nothing else, keeps the harness from grading the detector's homework
with the detector's own pen: if this module and the geolocation module ever disagree about
a convention, the round-trip self-check in :mod:`parkscan_eval.run_eval` fails loudly
instead of quietly reporting a beautiful error of zero.

The convention, stated once
---------------------------
Taken literally from ``docs/ARCHITECTURE.md`` section 5, whose back-projection reads::

    d_opt = [(u - cx) / fx, (v - cy) / fy, 1]
    d_w   = Rz(theta) . Ry(-pitch) . R_c_opt . d_opt
    reject if d_w[z] >= 0
    t     = -C_w[z] / d_w[z]
    P     = C_w + t . d_w

* **Optical frame**: ``x`` right, ``y`` down, ``z`` forward. Pixels are ``u`` right and
  ``v`` down from the top-left corner.
* **Body frame**: REP-103, ``x`` forward, ``y`` left, ``z`` up. ``R_c_opt`` is therefore
  the fixed permutation ``x_body = z_opt``, ``y_body = -x_opt``, ``z_body = -y_opt``.
* **World frame**: the lot's metric frame, ``x`` east, ``y`` north, ground plane at
  ``z = 0``. The camera centre is directly above the robot origin at ``height_m``.
* **pitch**: because the formula applies ``Ry(-pitch)``, a **positive** pitch rotates the
  optical axis **up**. A camera aimed at the road ahead therefore carries a *negative*
  pitch. This is stated rather than inferred because a sign error here produces points
  that are still on the ground, still plausible, and completely wrong.

There is no lens distortion model and no mount offset: the camera sits at the robot
origin. Both omissions are error sources in their own right and are ranked as such in
``EVAL.md`` rather than being silently assumed away.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from parkscan_core.types import BBox, Pose2D

__all__ = [
    "CameraModel",
    "Projection",
    "footprint_bbox",
    "ground_range_m",
    "project_ground_point",
]


@dataclass(frozen=True, slots=True)
class CameraModel:
    """Pinhole intrinsics plus the rigid mounting of the camera on the robot.

    **Every field is required.** There are deliberately no defaults: the harness must never
    invent camera parameters, because a stale default here and a changed default in
    ``parkscan_perception.geolocation`` would silently make the forward and backward stages
    describe two different cameras, and the resulting error would look like a pipeline
    result. ``parkscan_eval.run_eval`` builds this object from the very
    :class:`~parkscan_perception.geolocation.CameraIntrinsics` and
    :class:`~parkscan_perception.geolocation.CameraMount` the geolocation stage is given, so
    the numbers have one owner and only the *maths* is independent.
    """

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    height_m: float
    pitch_rad: float

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            msg = f"camera resolution must be positive, got {self.width}x{self.height}"
            raise ValueError(msg)
        if self.fx <= 0.0 or self.fy <= 0.0:
            msg = f"focal lengths must be positive, got fx={self.fx} fy={self.fy}"
            raise ValueError(msg)
        if not math.isfinite(self.height_m) or self.height_m <= 0.0:
            msg = f"height_m must be finite and positive, got {self.height_m}"
            raise ValueError(msg)
        if not math.isfinite(self.pitch_rad) or abs(self.pitch_rad) >= 0.5 * math.pi:
            msg = f"pitch_rad must be finite and within +/- pi/2, got {self.pitch_rad}"
            raise ValueError(msg)

    @property
    def hfov_rad(self) -> float:
        """Horizontal field of view implied by ``fx`` and ``width``."""
        return 2.0 * math.atan(0.5 * self.width / self.fx)

    def contains(self, u: float, v: float, *, margin_px: float = 0.0) -> bool:
        """Whether ``(u, v)`` lies inside the image rectangle, inset by ``margin_px``.

        The rectangle is ``[0, width] x [0, height]`` in continuous pixel coordinates --
        the same convention the detector contract's in-frame check uses, so a box this
        module calls visible is a box that contract will accept.
        """
        return (
            margin_px <= u <= self.width - margin_px and margin_px <= v <= self.height - margin_px
        )


@dataclass(frozen=True, slots=True)
class Projection:
    """Where a world point landed, and how far away it was.

    ``in_front`` is false when the point is behind the image plane, and ``in_image`` false
    when it projects outside the sensor. Both are reported rather than raising, because
    "the target was there but off frame" is a normal, countable outcome of driving past
    something, not an error.
    """

    u: float
    v: float
    depth_m: float
    ground_range_m: float
    in_front: bool
    in_image: bool

    @property
    def visible(self) -> bool:
        return self.in_front and self.in_image


def ground_range_m(pose: Pose2D, point: tuple[float, float]) -> float:
    """Horizontal distance from the robot origin to a ground point, in metres."""
    return math.hypot(point[0] - pose.x_m, point[1] - pose.y_m)


def project_ground_point(
    point: tuple[float, float],
    pose: Pose2D,
    camera: CameraModel,
    *,
    margin_px: float = 0.0,
) -> Projection:
    """Project a ground point ``(x_m, y_m, 0)`` into the image.

    ``margin_px`` insets the rectangle that ``in_image`` is tested against, which is how a
    caller asks for "comfortably inside the frame" rather than "not quite off the edge".

    The algebra is the documented back-projection run backwards. With
    ``D = P - C_w`` in world coordinates::

        (a, b, c) = Rz(-theta) . D                  # into the yawed body frame
        x_body    =  a cos(p) + c sin(p)            # Ry(p), undoing Ry(-p)
        y_body    =  b
        z_body    = -a sin(p) + c cos(p)
        x_opt     = -y_body,  y_opt = -z_body,  z_opt = x_body
        u         = cx + fx . x_opt / z_opt
        v         = cy + fy . y_opt / z_opt

    ``z_opt`` is the depth along the optical axis; a non-positive value means the point is
    at or behind the image plane and no pixel exists for it.
    """
    dx = point[0] - pose.x_m
    dy = point[1] - pose.y_m
    dz = -camera.height_m

    cos_t, sin_t = math.cos(pose.theta_rad), math.sin(pose.theta_rad)
    a = dx * cos_t + dy * sin_t
    b = -dx * sin_t + dy * cos_t
    c = dz

    cos_p, sin_p = math.cos(camera.pitch_rad), math.sin(camera.pitch_rad)
    x_body = a * cos_p + c * sin_p
    y_body = b
    z_body = -a * sin_p + c * cos_p

    x_opt, y_opt, z_opt = -y_body, -z_body, x_body
    horizontal = math.hypot(dx, dy)

    if z_opt <= 0.0:
        return Projection(
            u=math.nan,
            v=math.nan,
            depth_m=z_opt,
            ground_range_m=horizontal,
            in_front=False,
            in_image=False,
        )

    u = camera.cx + camera.fx * x_opt / z_opt
    v = camera.cy + camera.fy * y_opt / z_opt
    return Projection(
        u=u,
        v=v,
        depth_m=z_opt,
        ground_range_m=horizontal,
        in_front=True,
        in_image=camera.contains(u, v, margin_px=margin_px),
    )


def footprint_bbox(
    centre: tuple[float, float],
    size_m: float,
    pose: Pose2D,
    camera: CameraModel,
) -> BBox | None:
    """A bbox whose **bottom-centre is exactly the projection of ``centre``**.

    ``size_m`` is the target's diameter on the ground; it is projected only to give the box
    a plausible width and height in pixels. The box is then constructed *around* the
    projected centre so that ``BBox.bottom_centre`` is the pixel the geolocation stage is
    supposed to recover ``centre`` from.

    That is a deliberate modelling decision and it is worth being explicit about, because
    the alternative is subtly dishonest in the other direction. A real detector's bottom
    edge is the *near rim* of a pothole, so back-projecting it lands roughly half a pothole
    short of the pothole's centre. Baking that bias into the synthetic input would mean the
    harness reported a fixed near-range offset that it had itself invented, and that offset
    would swamp the range-dependent term this experiment exists to expose. So the bias is
    kept out of the input and named as an error source in ``EVAL.md`` instead: what this
    harness measures is the geolocation stage's response to the *attitude, pose and pixel*
    error budget it is given, not a near-rim bias chosen by the harness author.

    Returns ``None`` when the target is not in front of the camera.
    """
    if size_m <= 0.0:
        msg = f"size_m must be positive, got {size_m}"
        raise ValueError(msg)

    centre_projection = project_ground_point(centre, pose, camera)
    if not centre_projection.in_front:
        return None

    # Apparent size: the pinhole scaling of a `size_m` object at this optical depth,
    # floored at one pixel so a distant target still has a box a detector could emit.
    half_width_px = max(0.5, 0.5 * camera.fx * size_m / centre_projection.depth_m)
    box_height_px = max(1.0, camera.fy * size_m / centre_projection.depth_m)

    return BBox(
        x1=centre_projection.u - half_width_px,
        y1=centre_projection.v - box_height_px,
        x2=centre_projection.u + half_width_px,
        y2=centre_projection.v,
    )

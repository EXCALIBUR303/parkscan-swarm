"""Monocular flat-ground back-projection: one pixel, one plane, one world point.

The detector says *where in the image*. This stage says *where in the lot*, using the one
extra assumption a single camera makes possible: the thing it is looking at is standing on
a flat ground plane at ``z = 0``. Given that, the pixel at the bottom edge of a box is a
ray, the ray meets the plane at exactly one point, and that point is the answer.

The pixel used is :attr:`~parkscan_core.types.BBox.bottom_centre`, the ground-contact
point -- never the box centre. Back-projecting the centre of a pothole box places it
roughly half a box height too far away, and the mistake is invisible in the output because
the result is still a plausible point on the ground.

The maths, exactly as implemented
---------------------------------
::

    d_opt   = [(u - cx)/fx, (v - cy)/fy, 1]         # ray in optical axes
    d_w     = Rz(theta) @ Ry(-pitch) @ R_c_opt @ d_opt
    reject if d_w[z] >= 0                            # at or above the horizon
    t       = -C_w[z] / d_w[z]
    P_world = C_w + t * d_w                          # C_w = [robot_x, robot_y, camera_height]

``R_c_opt = [[0,0,1], [-1,0,0], [0,-1,0]]`` converts optical axes (x right, y down, z
forward) into REP-103 body axes (x forward, y left, z up): optical +z becomes body +x,
optical +x becomes body -y, optical +y becomes body -z. It is a rotation (orthonormal,
determinant +1), so its transpose is its inverse -- which is what
:func:`project_ground_point` uses to go the other way.

Sign convention for ``pitch_rad``, stated once and loudly
---------------------------------------------------------
``Ry`` here is the standard right-handed rotation about the body ``+y`` (left) axis::

    Ry(a) = [[cos a, 0, sin a], [0, 1, 0], [-sin a, 0, cos a]]

so ``Ry(a) @ [1,0,0] = [cos a, 0, -sin a]``. The formula applies ``Ry(-pitch)``, whose
effect on the forward axis is ``[cos pitch, 0, +sin pitch]``. **Therefore a positive
``pitch_rad`` tilts the optical axis up, and a camera that can see the ground has a
negative ``pitch_rad``.** The default is ``-15 degrees``. This is spelled out because a
sign error here does not crash: it produces a camera that stares at the sky, every ray is
rejected as above the horizon, and the pipeline goes quietly, permanently blind.

Rejection, never clamping
-------------------------
A ray with ``d_w[z] >= 0`` never meets the ground, and a ray just below the horizon meets
it hundreds of metres away. Both are rejected outright. Clamping the near-horizon case to
some maximum would be worse than useless: it yields a confident, well-formed, plausible
world point that is simply wrong, and it poisons the map silently rather than loudly
(``docs/ACCEPTANCE.md`` C11). The same reasoning drives the range gate: beyond
:data:`DEFAULT_MAX_RANGE_M` the error budget below makes the point meaningless, so it is
returned as invalid *with its range*, not truncated to the limit.

Error budget
------------
Error grows roughly with the **square** of ground range. For a camera at height ``h`` whose
ray meets the ground at range ``R``, the depression angle satisfies ``tan(a) = h/R``, so an
attitude error ``da`` moves the point by about ``da * (R^2 + h^2) / h`` -- quadratic in
``R`` and inversely proportional to mount height. ``tests/test_geolocation.py`` measures
this against the implementation, and ranks the sources at a fixed range: pitch error
dominates bbox bottom-edge error, camera-height error and yaw error. That ranking is why
the gate exists and why it is set where it is.

Two assumptions are not modelled, and both are honest limitations rather than oversights:
the camera sits at the robot's ``(x, y)`` with no lever arm, and the ground is exactly flat
-- which a pothole, being by definition a hole *below* the plane, is not.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Final

import numpy as np
import numpy.typing as npt

from parkscan_core.types import Detection, Pose2D

__all__ = [
    "DEFAULT_MAX_RANGE_M",
    "REASON_ABOVE_HORIZON",
    "REASON_BEYOND_MAX_RANGE",
    "REASON_NON_FINITE",
    "REASON_OK",
    "R_C_OPT",
    "CameraIntrinsics",
    "CameraMount",
    "GeolocResult",
    "backproject_pixel",
    "camera_ray",
    "geolocate_detection",
    "project_ground_point",
]

DEFAULT_MAX_RANGE_M: Final = 6.0
"""Ground range beyond which a back-projected point is rejected. Chosen from the measured
error budget (``docs/RESEARCH_LOG.md`` section 5), not from convenience: at this mount
height a 1-degree pitch error already costs over a metre at 6 m."""

REASON_OK: Final = "ok"
REASON_ABOVE_HORIZON: Final = "above_horizon"
REASON_BEYOND_MAX_RANGE: Final = "beyond_max_range"
REASON_NON_FINITE: Final = "non_finite"


def _frozen(values: list[list[float]]) -> npt.NDArray[np.float64]:
    arr = np.array(values, dtype=np.float64)
    arr.setflags(write=False)
    return arr


R_C_OPT: Final = _frozen([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
"""Optical axes (x right, y down, z forward) to REP-103 body axes (x forward, y left, z up)."""


@dataclass(frozen=True, slots=True)
class CameraIntrinsics:
    """A pinhole camera with square pixels and a centred principal point.

    Defaults describe the forward camera the simulator models: 640x480 at a 70-degree
    horizontal field of view, which gives ``fx = fy = 457.0`` px. There is no distortion
    model -- ``sim-lite`` renders nothing, so there is nothing to undistort, and inventing
    coefficients would be inventing a lens.

    ``cx = width/2`` (not ``(width-1)/2``) so that the principal point is consistent with
    the mandated ``fx = (W/2) / tan(hfov/2)``: both treat the image as spanning ``[0, W]``
    in continuous pixel coordinates, which is also the convention the bbox bounds check in
    :mod:`parkscan_perception.contract` uses.
    """

    width: int = 640
    height: int = 480
    hfov_rad: float = 1.2217304763960306  # 70 degrees

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            msg = f"camera size must be positive, got {self.width}x{self.height}"
            raise ValueError(msg)
        if not math.isfinite(self.hfov_rad) or not 0.0 < self.hfov_rad < math.pi:
            msg = f"hfov_rad must lie in (0, pi), got {self.hfov_rad}"
            raise ValueError(msg)

    @property
    def fx(self) -> float:
        """Focal length in pixels: ``(W/2) / tan(hfov/2)``."""
        return (self.width / 2.0) / math.tan(self.hfov_rad / 2.0)

    @property
    def fy(self) -> float:
        """Equal to :attr:`fx`: square pixels, so the vertical FOV follows from the aspect."""
        return self.fx

    @property
    def cx(self) -> float:
        return self.width / 2.0

    @property
    def cy(self) -> float:
        return self.height / 2.0

    @property
    def vfov_rad(self) -> float:
        """Vertical field of view implied by :attr:`fy` and the image height."""
        return 2.0 * math.atan((self.height / 2.0) / self.fy)


@dataclass(frozen=True, slots=True)
class CameraMount:
    """Where the camera sits on the robot, and how it is aimed.

    ``height_m``
        Optical centre above the ground plane. 0.45 m is a small ground robot's deck. It
        divides the error budget: halving the mount height doubles the position error for
        the same attitude error.
    ``pitch_rad``
        Tilt in the body frame. **Negative is downward** -- see the module docstring; the
        formula applies ``Ry(-pitch)``. The default ``-0.2618`` rad (-15 degrees) puts the
        image-centre ray on the ground at about 1.7 m and keeps the whole 6 m valid range
        inside the lower half of the frame.

    No forward or lateral offset from the robot origin is modelled; the camera is at the
    robot's ``(x, y)``. Adding a lever arm is a two-line change to :func:`backproject_pixel`
    and would need a measured value, which this simulated robot does not have.
    """

    height_m: float = 0.45
    pitch_rad: float = -0.2617993877991494  # -15 degrees: tilted down toward the ground

    def __post_init__(self) -> None:
        if not math.isfinite(self.height_m) or self.height_m <= 0.0:
            msg = f"height_m must be finite and positive, got {self.height_m}"
            raise ValueError(msg)
        if not math.isfinite(self.pitch_rad) or abs(self.pitch_rad) >= math.pi / 2.0:
            msg = f"pitch_rad must lie in (-pi/2, pi/2), got {self.pitch_rad}"
            raise ValueError(msg)

    @property
    def looks_below_horizon(self) -> bool:
        """True when the optical axis is tilted down, i.e. ``pitch_rad < 0``.

        Exists so the sign convention is an assertable property rather than a comment: a
        mount that fails this can only ever see sky, and every detection it feeds to
        :func:`backproject_pixel` will be rejected as above the horizon.
        """
        return self.pitch_rad < 0.0


@dataclass(frozen=True, slots=True)
class GeolocResult:
    """Outcome of one back-projection.

    ``valid``
        Whether ``point`` may be used. False means rejected, never approximated.
    ``point``
        ``(x_m, y_m)`` in the lot frame, or ``None`` whenever ``valid`` is False. The two
        travel together so a caller cannot read a coordinate out of a rejected result.
    ``range_m``
        Ground range from the robot to the point. It is populated **even when the result is
        invalid** -- for a range rejection it is the range that failed the gate, which is
        what ``docs/DATA_MODEL.md`` stores alongside ``geoloc_valid`` and what the error
        budget is plotted against. For a ray at or above the horizon it is
        ``math.inf``: the ray never meets the ground, and any finite number there would be
        a fabrication.
    ``reason``
        One of :data:`REASON_OK`, :data:`REASON_ABOVE_HORIZON`,
        :data:`REASON_BEYOND_MAX_RANGE`, :data:`REASON_NON_FINITE`.
    """

    valid: bool
    point: tuple[float, float] | None
    range_m: float
    reason: str

    def __post_init__(self) -> None:
        if self.valid and self.point is None:
            msg = "a valid GeolocResult must carry a point"
            raise ValueError(msg)
        if not self.valid and self.point is not None:
            msg = f"an invalid GeolocResult must not carry a point, got {self.point!r}"
            raise ValueError(msg)


def _rz(theta: float) -> npt.NDArray[np.float64]:
    """Right-handed rotation about the body ``+z`` (up) axis: the robot's yaw."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _ry(angle: float) -> npt.NDArray[np.float64]:
    """Right-handed rotation about the body ``+y`` (left) axis. See the sign note above."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def _rotation(theta: float, pitch: float) -> npt.NDArray[np.float64]:
    """``Rz(theta) @ Ry(-pitch) @ R_c_opt``: optical axes straight to world axes."""
    return _rz(theta) @ _ry(-pitch) @ R_C_OPT


def camera_ray(
    u: float,
    v: float,
    pose: Pose2D,
    intrinsics: CameraIntrinsics,
    mount: CameraMount,
) -> npt.NDArray[np.float64]:
    """The world-frame direction ``d_w`` of the ray through pixel ``(u, v)``.

    Not normalised: ``t`` in :func:`backproject_pixel` is measured in units of this vector,
    and the ratio ``-C_w[z] / d_w[z]`` is scale-invariant, so normalising would cost a
    square root and buy nothing. Exposed publicly because "is this pixel above the horizon"
    is exactly ``camera_ray(...)[2] >= 0``, and a test should be able to ask that directly.
    """
    d_opt = np.array(
        [
            (u - intrinsics.cx) / intrinsics.fx,
            (v - intrinsics.cy) / intrinsics.fy,
            1.0,
        ],
        dtype=np.float64,
    )
    return _rotation(pose.theta_rad, mount.pitch_rad) @ d_opt


def backproject_pixel(
    u: float,
    v: float,
    pose: Pose2D,
    intrinsics: CameraIntrinsics,
    mount: CameraMount,
    *,
    max_range_m: float = DEFAULT_MAX_RANGE_M,
) -> GeolocResult:
    """Intersect the ray through pixel ``(u, v)`` with the ground plane ``z = 0``.

    Rejects, and never clamps, rays at or above the horizon and points beyond
    ``max_range_m``.
    """
    if not math.isfinite(max_range_m) or max_range_m <= 0.0:
        msg = f"max_range_m must be finite and positive, got {max_range_m}"
        raise ValueError(msg)
    if not (math.isfinite(u) and math.isfinite(v)):
        return GeolocResult(False, None, math.inf, REASON_NON_FINITE)

    d_w = camera_ray(u, v, pose, intrinsics, mount)
    if not bool(np.isfinite(d_w).all()):
        return GeolocResult(False, None, math.inf, REASON_NON_FINITE)

    dz = float(d_w[2])
    if dz >= 0.0:
        # At or above the horizon. The ray never meets the plane; there is no point to
        # return, and inventing one at the range gate would be the exact failure C11 names.
        return GeolocResult(False, None, math.inf, REASON_ABOVE_HORIZON)

    c_w = np.array([pose.x_m, pose.y_m, mount.height_m], dtype=np.float64)
    t = -float(c_w[2]) / dz
    p_world = c_w + t * d_w
    x, y = float(p_world[0]), float(p_world[1])
    range_m = math.hypot(x - pose.x_m, y - pose.y_m)

    if range_m > max_range_m:
        return GeolocResult(False, None, range_m, REASON_BEYOND_MAX_RANGE)
    return GeolocResult(True, (x, y), range_m, REASON_OK)


def geolocate_detection(
    detection: Detection,
    pose: Pose2D,
    intrinsics: CameraIntrinsics,
    mount: CameraMount,
    *,
    max_range_m: float = DEFAULT_MAX_RANGE_M,
) -> GeolocResult:
    """Back-project a detection from its bbox **bottom-centre**, the ground-contact point.

    This is the only place the pose and the detection meet. The detector produced the box
    knowing nothing about where the robot was; the pose arrives here, afterwards, from the
    telemetry that shares the detection's stamp.
    """
    u, v = detection.bbox.bottom_centre
    return backproject_pixel(u, v, pose, intrinsics, mount, max_range_m=max_range_m)


def project_ground_point(
    x_m: float,
    y_m: float,
    pose: Pose2D,
    intrinsics: CameraIntrinsics,
    mount: CameraMount,
) -> tuple[float, float] | None:
    """The exact inverse of :func:`backproject_pixel`: ground point to pixel.

    Returns ``None`` when the point is at or behind the image plane, which is the forward
    twin of the horizon rejection. Not used by the pipeline -- it exists so the round-trip
    test can start from a *known* world point instead of from a pixel somebody chose, and
    so the error-budget sweep can place a target at an exact ground range.
    """
    c_w = np.array([pose.x_m, pose.y_m, mount.height_m], dtype=np.float64)
    v_w = np.array([x_m, y_m, 0.0], dtype=np.float64) - c_w
    # R is orthonormal with determinant +1, so the inverse is the transpose, exactly.
    d_opt = _rotation(pose.theta_rad, mount.pitch_rad).T @ v_w
    depth = float(d_opt[2])
    if depth <= 0.0:
        return None
    u = intrinsics.fx * float(d_opt[0]) / depth + intrinsics.cx
    v = intrinsics.fy * float(d_opt[1]) / depth + intrinsics.cy
    return (u, v)

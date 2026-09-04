"""Wiring the perception stage into the run loop.

The order here is load-bearing and is the whole reason the two stages are separate
packages: **detect, then geolocate, never merged**.

    targets  = what the simulator knows is in view   (world truth, image space only)
    frame    = an image-space record; carries NO pose
    dets     = detector.detect(frame)                 (still image space)
    result   = geolocate_detection(det, pose, ...)    (world space, first time)

The detector is never handed the robot pose. If it were, a "detection" could simply read
the answer out of the map and the geolocation error in ``make eval`` would measure
nothing. Keeping the pose out of the detector is what makes that number real.

The bbox handed to the detector is the pixel AABB of the pothole's footprint, so its
bottom edge is the NEAR rim rather than the centre. Back-projection therefore recovers the
near rim, a systematic bias of roughly one radius — this is error source (3) in
docs/ARCHITECTURE.md, reproduced here on purpose rather than papered over.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from parkscan_perception.geolocation import (
    CameraIntrinsics,
    CameraMount,
    project_ground_point,
)
from parkscan_perception.simulated import GroundTruthTarget
from parkscan_sim.world import Pothole

from parkscan_core.types import BBox, DetKind, Frame, Pose2D


@dataclass(frozen=True, slots=True)
class VisibleTarget:
    """A pothole the camera can see this tick, with its image-space footprint."""

    pothole: Pothole
    bbox: BBox
    true_range_m: float


def _circle_samples(pothole: Pothole, count: int = 12) -> list[tuple[float, float]]:
    """Points around the pothole rim, for building its image footprint."""
    return [
        (
            pothole.x_m + pothole.radius_m * math.cos(2.0 * math.pi * i / count),
            pothole.y_m + pothole.radius_m * math.sin(2.0 * math.pi * i / count),
        )
        for i in range(count)
    ]


def visible_targets(
    potholes: tuple[Pothole, ...],
    pose: Pose2D,
    intrinsics: CameraIntrinsics,
    mount: CameraMount,
    *,
    max_range_m: float,
) -> tuple[VisibleTarget, ...]:
    """Project every pothole in view to an image-space box.

    A pothole counts as visible only if EVERY rim sample projects into the image. A
    partially-clipped footprint would yield a box whose bottom edge is the image border
    rather than the defect, and back-projecting that produces a confident, wrong point —
    the failure mode the range gate and horizon rejection exist to prevent.
    """
    out: list[VisibleTarget] = []
    for pothole in potholes:
        ground_range = math.hypot(pothole.x_m - pose.x_m, pothole.y_m - pose.y_m)
        if ground_range > max_range_m:
            continue

        pixels = [
            project_ground_point(x, y, pose, intrinsics, mount) for x, y in _circle_samples(pothole)
        ]
        if any(pixel is None for pixel in pixels):
            continue
        us = [pixel[0] for pixel in pixels if pixel is not None]
        vs = [pixel[1] for pixel in pixels if pixel is not None]
        if min(us) < 0 or max(us) > intrinsics.width:
            continue
        if min(vs) < 0 or max(vs) > intrinsics.height:
            continue

        out.append(
            VisibleTarget(
                pothole=pothole,
                bbox=BBox(x1=min(us), y1=min(vs), x2=max(us), y2=max(vs)),
                true_range_m=ground_range,
            )
        )
    return tuple(out)


def make_frame(frame_id: int, stamp_ns: int, robot_id: str, intrinsics: CameraIntrinsics) -> Frame:
    """An image-space record with no pixels.

    ``image`` is None because sim-lite renders nothing, and a fabricated array would
    invite someone to believe a detector had looked at it. ``SimulatedDetector`` never
    reads it; a real model would require this field to be populated by a real camera.
    """
    return Frame(
        frame_id=frame_id,
        stamp_ns=stamp_ns,
        camera_id=f"{robot_id}/front",
        width=intrinsics.width,
        height=intrinsics.height,
        image=None,
    )


def targets_for_frame(visible: tuple[VisibleTarget, ...]) -> tuple[GroundTruthTarget, ...]:
    return tuple(
        GroundTruthTarget(bbox=t.bbox, kind=DetKind.POTHOLE, severity=t.pothole.severity)
        for t in visible
    )


class FrameTargets:
    """A ``TargetProvider`` the runner fills one frame ahead of each ``detect`` call.

    The alternative — rebuilding a ``SimulatedDetector`` per frame, or reaching into its
    private target field — would either churn the detector's identity or break its
    encapsulation. This keeps one long-lived detector per robot (so its seed is stable)
    while still feeding it frame-specific truth.

    Determinism is preserved because each ``frame_id`` is issued exactly once and its
    targets are set before the single ``detect`` call that consumes them: the detector's
    output remains a pure function of (seed, frame_id, targets-at-that-frame).
    """

    __slots__ = ("_by_frame",)

    def __init__(self) -> None:
        self._by_frame: dict[int, tuple[GroundTruthTarget, ...]] = {}

    def set(self, frame_id: int, targets: tuple[GroundTruthTarget, ...]) -> None:
        # Only the current frame is retained: keeping every frame would grow without
        # bound over a 20 000-step run for no benefit.
        self._by_frame = {frame_id: targets}

    def __call__(self, frame: Frame) -> tuple[GroundTruthTarget, ...]:
        return self._by_frame.get(frame.frame_id, ())

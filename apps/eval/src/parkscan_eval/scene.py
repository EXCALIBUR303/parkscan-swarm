"""The synthetic evaluation scene: known targets, a real sim drive, recorded frames.

The experiment this module sets up is deliberately narrow. Ground-truth targets are placed
at **exactly known** positions along one zone's drive aisle; a robot is driven down that
aisle by :class:`~parkscan_sim.world.SimLite` itself, so the poses come from the same
kinematic integrator the demo uses rather than from a straight line invented here; and at
each camera exposure every target that would actually be visible is projected into the
image to produce a :class:`~parkscan_perception.simulated.GroundTruthTarget`.

Where the numbers come from
---------------------------
The drive line is **derived from the lot**, not authored here: a zone's ``entry_waypoint``
and ``exit_waypoint`` are lane-graph nodes at the two ends of its aisle, so the robot drives
the aisle the coverage planner would drive, and a change to ``sim/lot/lot.yaml`` moves this
experiment with it. Targets are then laid out along that same segment. Nothing in this
module draws a random number: the *geometry* of the experiment is a function of the lot
alone, exactly as ``SimLite`` derives its start positions from the lot alone, so a row in
``results.csv`` can be traced to the committed lot file plus the seed. The seed controls
what the detector and the error budget *do*, never where anything is.

Why a target may be absent from a frame
---------------------------------------
A target is recorded for a frame only when it is in front of the camera, its whole box lies
inside the image with a margin wider than the detector's jitter, and that box is at least
``min_box_px`` across. The margin matters: a box clipped at the image edge has its
bottom-centre moved by the clipping, and the harness would then score a geolocation against
a pixel the detector was never able to report. The minimum size matters for a different
reason -- at this mount and lens a 0.45 m pothole is under 8 px wide beyond about 25 m, and
feeding boxes that no real detector could resolve would let the harness characterise the
geolocation stage over a range nothing could actually detect at. Both bounds are reported
in ``EVAL.md`` rather than left implicit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from parkscan_perception.simulated import GroundTruthTarget
from parkscan_sim.world import SimLite

from parkscan_core.lot import LotSpec, Zone
from parkscan_core.types import BBox, DetKind, Frame, Pose2D, RobotId, Severity, TwistCmd
from parkscan_eval.camera import CameraModel, footprint_bbox, project_ground_point

__all__ = [
    "RecordedFrame",
    "SceneConfig",
    "Sighting",
    "Target",
    "aisle_segment",
    "build_targets",
    "drive_and_record",
]

_SEVERITY_CYCLE: tuple[Severity, ...] = (Severity.HIGH, Severity.MEDIUM, Severity.LOW)
"""Fixed, index-driven severity assignment. No draw, so severity is reproducible from the
lot alone and a severity column in the CSV means the same thing across every seed."""


@dataclass(frozen=True, slots=True)
class Target:
    """One synthetic ground-truth target at a position the harness knows exactly.

    ``position`` is the point the geolocation stage is expected to recover: the centre of
    the target's footprint on the ground plane ``z = 0``. ``size_m`` is its diameter, used
    only to give the projected box a plausible size in pixels.
    """

    id: str
    position: tuple[float, float]
    size_m: float
    kind: DetKind
    severity: Severity | None


@dataclass(frozen=True, slots=True)
class SceneConfig:
    """Every knob the scene has. Frozen, hashed into ``results.json``, never mutated."""

    zone_id: str = "zone_01"
    """Which zone's aisle to drive. Its entry and exit waypoints define the line."""

    speed_mps: float = 1.0
    """Commanded forward speed. ``SimLite`` clamps it to its own twist limits."""

    camera_period_ticks: int = 4
    """One exposure every N simulator ticks. At ``dt = 0.05 s`` that is 5 Hz."""

    camera_id: str = "cam_front"

    lead_in_m: float = 3.0
    """Distance from the aisle entry to the first target."""

    tail_m: float = 1.0
    """Distance from the last target to the aisle exit."""

    target_spacing_m: float = 2.5
    lateral_offsets_m: tuple[float, ...] = (0.0, 1.1, -0.8, 0.5, -1.4)
    """Offsets perpendicular to the aisle, cycled by target index. They keep targets off a
    single line so the experiment is not purely on the optical axis, and they stay inside
    the 6 m aisle."""

    target_size_m: float = 0.45
    """Footprint diameter of a modelled pothole. An authored choice, not a measurement."""

    min_box_px: float = 8.0
    frame_margin_px: float = 8.0
    max_steps: int = 4000

    def __post_init__(self) -> None:
        if self.speed_mps <= 0.0:
            msg = f"speed_mps must be positive, got {self.speed_mps}"
            raise ValueError(msg)
        if self.camera_period_ticks < 1:
            msg = f"camera_period_ticks must be at least 1, got {self.camera_period_ticks}"
            raise ValueError(msg)
        if self.target_spacing_m <= 0.0:
            msg = f"target_spacing_m must be positive, got {self.target_spacing_m}"
            raise ValueError(msg)
        if not self.lateral_offsets_m:
            msg = "lateral_offsets_m must contain at least one offset"
            raise ValueError(msg)
        if self.target_size_m <= 0.0:
            msg = f"target_size_m must be positive, got {self.target_size_m}"
            raise ValueError(msg)
        if self.min_box_px <= 0.0 or self.frame_margin_px < 0.0:
            msg = (
                "min_box_px must be positive and frame_margin_px non-negative, got "
                f"{self.min_box_px} and {self.frame_margin_px}"
            )
            raise ValueError(msg)
        if self.max_steps < 1:
            msg = f"max_steps must be at least 1, got {self.max_steps}"
            raise ValueError(msg)


@dataclass(frozen=True, slots=True)
class Sighting:
    """One target as it appears in one frame: the box, and the truth behind it."""

    target: Target
    bbox: BBox
    true_range_m: float
    """Ground range from the robot's **true** pose to the target's known position. Kept for
    the record; the range a metric bins on is the one the geolocation stage computed from
    the pose it was actually given."""


@dataclass(frozen=True, slots=True)
class RecordedFrame:
    """One camera exposure: the frame handed to the detector, and what was really there."""

    frame: Frame
    pose: Pose2D
    sightings: tuple[Sighting, ...]

    def ground_truth(self) -> tuple[GroundTruthTarget, ...]:
        """The detector's input for this frame, in image space and nothing else."""
        return tuple(
            GroundTruthTarget(bbox=s.bbox, kind=s.target.kind, severity=s.target.severity)
            for s in self.sightings
        )


def _zone(lot: LotSpec, zone_id: str) -> Zone:
    for zone in lot.zones:
        if zone.id == zone_id:
            return zone
    msg = f"lot {lot.name!r} has no zone {zone_id!r}; it has {[z.id for z in lot.zones]}"
    raise ValueError(msg)


def aisle_segment(lot: LotSpec, zone_id: str) -> tuple[tuple[float, float], tuple[float, float]]:
    """The zone's aisle as ``(entry, exit)`` points in lot metres, read from the lane graph."""
    zone = _zone(lot, zone_id)
    nodes = lot.lane_graph.node_map()
    entry = nodes[zone.entry_waypoint]
    exit_ = nodes[zone.exit_waypoint]
    return ((entry.x_m, entry.y_m), (exit_.x_m, exit_.y_m))


def build_targets(lot: LotSpec, config: SceneConfig) -> tuple[Target, ...]:
    """Lay targets along the zone's aisle. Deterministic, seed-free, lot-derived."""
    (x0, y0), (x1, y1) = aisle_segment(lot, config.zone_id)
    length = math.dist((x0, y0), (x1, y1))
    usable = length - config.lead_in_m - config.tail_m
    if usable <= 0.0:
        msg = (
            f"zone {config.zone_id!r} aisle is {length:.2f} m, too short for a "
            f"{config.lead_in_m} m lead-in and a {config.tail_m} m tail"
        )
        raise ValueError(msg)

    ux, uy = (x1 - x0) / length, (y1 - y0) / length
    # Perpendicular, left of the direction of travel (REP-103: +90 degrees).
    px, py = -uy, ux

    count = int(usable // config.target_spacing_m) + 1
    targets: list[Target] = []
    for index in range(count):
        along = config.lead_in_m + index * config.target_spacing_m
        lateral = config.lateral_offsets_m[index % len(config.lateral_offsets_m)]
        targets.append(
            Target(
                id=f"pothole_{index + 1:02d}",
                position=(x0 + ux * along + px * lateral, y0 + uy * along + py * lateral),
                size_m=config.target_size_m,
                kind=DetKind.POTHOLE,
                severity=_SEVERITY_CYCLE[index % len(_SEVERITY_CYCLE)],
            )
        )
    return tuple(targets)


def _sighting(
    target: Target, pose: Pose2D, camera: CameraModel, config: SceneConfig
) -> Sighting | None:
    """The target's appearance in this frame, or ``None`` when it is not usably visible."""
    projection = project_ground_point(
        target.position, pose, camera, margin_px=config.frame_margin_px
    )
    if not projection.visible:
        return None
    bbox = footprint_bbox(target.position, target.size_m, pose, camera)
    if bbox is None or bbox.width < config.min_box_px:
        return None
    margin = config.frame_margin_px
    inside = (
        margin <= bbox.x1
        and bbox.x2 <= camera.width - margin
        and margin <= bbox.y1
        and bbox.y2 <= camera.height - margin
    )
    if not inside:
        return None
    return Sighting(
        target=target,
        bbox=bbox,
        true_range_m=math.dist((pose.x_m, pose.y_m), target.position),
    )


def drive_and_record(
    lot: LotSpec,
    sim: SimLite,
    camera: CameraModel,
    config: SceneConfig,
    *,
    robot: RobotId = "robot_1",
) -> tuple[RecordedFrame, ...]:
    """Drive ``robot`` down the zone's aisle and record what the camera would have seen.

    ``sim`` must already have been ``reset``. The robot is teleported to the aisle entry
    first -- ``SimLite.place`` exists for exactly this kind of scenario setup -- and then
    driven with a constant forward command until it reaches the exit or ``max_steps`` runs
    out. Frames carry ``image=None``: ``sim-lite`` renders nothing, ``SimulatedDetector``
    never looks at pixels, and fabricating an array here would be a lie that every check
    downstream would happily pass.
    """
    (x0, y0), (x1, y1) = aisle_segment(lot, config.zone_id)
    heading = math.atan2(y1 - y0, x1 - x0)
    sim.place(robot, Pose2D(x_m=x0, y_m=y0, theta_rad=heading))

    targets = build_targets(lot, config)
    dt_s = sim.config.dt_s
    ux, uy = (x1 - x0), (y1 - y0)
    length = math.hypot(ux, uy)
    ux, uy = ux / length, uy / length

    recorded: list[RecordedFrame] = []
    frame_id = 0
    for tick in range(config.max_steps):
        pose = sim.pose(robot)
        travelled = (pose.x_m - x0) * ux + (pose.y_m - y0) * uy
        if travelled >= length:
            break
        if tick % config.camera_period_ticks == 0:
            sightings = tuple(
                s
                for s in (_sighting(target, pose, camera, config) for target in targets)
                if s is not None
            )
            recorded.append(
                RecordedFrame(
                    frame=Frame(
                        frame_id=frame_id,
                        stamp_ns=sim.now_ns(),
                        camera_id=config.camera_id,
                        width=camera.width,
                        height=camera.height,
                        image=None,
                    ),
                    pose=pose,
                    sightings=sightings,
                )
            )
            frame_id += 1
        sim.actuate(robot, TwistCmd(v_mps=config.speed_mps, omega_radps=0.0))
        sim.step(dt_s)

    return tuple(recorded)

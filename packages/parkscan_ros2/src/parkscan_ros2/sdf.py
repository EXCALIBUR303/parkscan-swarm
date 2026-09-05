"""Generate the one committed Gazebo world SDF from ``sim/lot/lot.yaml``.

Per ADR-0008 (`docs/adr/0008-ros2-backend-bridge-design.md`), the SDF's coordinate frame
is the identity of ``lot.yaml``'s own metres: X/Y map straight onto ``Pose2D.x_m/y_m``,
no rotation, no offset. Geometry is derived exactly the way
:func:`parkscan_sim.obstacles.build_obstacle_field` derives ``SimLite``'s obstacle set --
same wall placement, same car-inset formula -- so the two backends present geometrically
comparable worlds, even though (per ADR-0008 §5) this file's occupied-stall pattern is a
single fixed draw rather than re-rolled per seed.

**Deliberately not modelled**, matching ``SimLite``'s own stated honesty
(``parkscan_sim/__init__.py``: *"there is no mass, no inertia"*): each robot is a plain
cylinder, driven directly by a velocity-control plugin rather than simulated wheels and
motors. This project's goal is telemetry/navigation/coordination parity between backends,
not vehicle dynamics realism -- modelling wheels would add detail nothing here measures.
Mass and inertia values exist only because SDF's physics engine requires *some* finite
numbers to integrate; they are round, plausible figures, not a measurement of anything.

Friction on the collision surface is explicitly zeroed (``mu`` / ``mu2``), for a reason
found empirically, not assumed: DART's default friction measurably damps
``VelocityControl``'s commanded velocity against the ground plane's contact patch --
commanding 0.8 m/s and measuring the robot's actual reported velocity a full 10 s later
(steady-state, not a transient) gave ~0.31 m/s, a real and repeatable ~61% loss verified
against a running Gazebo. Zero friction restored tracking to float precision of
commanded. ``SimLite`` has no friction model to lose against in the first place, so this
keeps the two backends comparable rather than one silently damping motion the other does
not -- consistent with this module's stated goal of parity, not dynamics realism.

The lidar sensor is configured to :data:`LIDAR_N_RAYS` / :data:`LIDAR_FOV_RAD` /
:data:`LIDAR_MAX_RANGE_M` -- the exact values of
``parkscan_sim.sensors.SensorParams``'s defaults, duplicated here as plain constants
rather than imported, because importing ``parkscan_sim`` from a peer backend package is
exactly the shape ADR-0008 already rejected for the battery proxy. If ``SensorParams``
changes, this module goes stale silently; there is no test tying the two together.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from xml.sax.saxutils import quoteattr

from parkscan_core.lot import LotSpec

__all__ = [
    "LIDAR_FOV_RAD",
    "LIDAR_MAX_RANGE_M",
    "LIDAR_N_RAYS",
    "ROBOT_RADIUS_M",
    "generate_world_sdf",
]

#: Matches ``parkscan_sim.sensors.SensorParams`` exactly (see module docstring).
LIDAR_N_RAYS: int = 37
LIDAR_FOV_RAD: float = 1.5 * math.pi
LIDAR_MAX_RANGE_M: float = 12.0

#: Matches ``parkscan_sim.config.SimConfig.robot_radius_m`` exactly.
ROBOT_RADIUS_M: float = 0.35
_ROBOT_HEIGHT_M = 0.5
_ROBOT_MASS_KG = 5.0
"""An arbitrary, plausible figure -- see module docstring. Not a measurement."""


def generate_world_sdf(
    lot: LotSpec,
    *,
    occupied_space_ids: Sequence[str],
    robot_poses: Sequence[tuple[str, float, float, float]],
    dt_s: float,
    car_inset_m: float = 0.3,
    wall_thickness_m: float = 0.5,
) -> str:
    """Render the world SDF as a string.

    Args:
        lot: the lot geometry to build walls, occupied stalls, and (informationally,
            since Gazebo does not route over it) charging bays from.
        occupied_space_ids: which ``lot.parking_spaces`` ids get a car box. Not drawn
            here -- callers own the seed/reproducibility decision (ADR-0008 §5); this
            function only renders whatever set it is given.
        robot_poses: ``(robot_id, x_m, y_m, theta_rad)`` for each robot to spawn. Order
            is preserved in the output but is not otherwise significant.
        dt_s: the physics step size. Must equal the value the bridge's ``multi_step``
            calls assume, or the lockstep timing design in ADR-0008 §3 silently drifts.
        car_inset_m: gap between a stall's painted bounds and its car box, each side.
        wall_thickness_m: thickness of the four boundary walls.

    Raises:
        ValueError: ``dt_s``, ``car_inset_m``, or ``wall_thickness_m`` is not positive,
            or ``occupied_space_ids`` names a space not in ``lot.parking_spaces``.
    """
    if dt_s <= 0.0:
        msg = f"dt_s must be positive, got {dt_s}"
        raise ValueError(msg)
    if car_inset_m <= 0.0:
        msg = f"car_inset_m must be positive, got {car_inset_m}"
        raise ValueError(msg)
    if wall_thickness_m <= 0.0:
        msg = f"wall_thickness_m must be positive, got {wall_thickness_m}"
        raise ValueError(msg)

    known_ids = {space.id for space in lot.parking_spaces}
    unknown = sorted(set(occupied_space_ids) - known_ids)
    if unknown:
        msg = f"occupied_space_ids names spaces not in lot.parking_spaces: {unknown}"
        raise ValueError(msg)

    w = lot.size_m.w
    h = lot.size_m.h
    t = wall_thickness_m

    parts: list[str] = []
    parts.append('<?xml version="1.0" ?>')
    parts.append('<sdf version="1.9">')
    parts.append(f'  <world name={quoteattr(lot.name.replace(" ", "_") or "parkscan")}>')
    parts.append("    <physics name=\"parkscan_physics\" type=\"dartsim\">")
    parts.append(f"      <max_step_size>{dt_s}</max_step_size>")
    parts.append("      <real_time_factor>1.0</real_time_factor>")
    parts.append("    </physics>")
    parts.append('    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>')
    parts.append(
        '    <plugin filename="gz-sim-user-commands-system" '
        'name="gz::sim::systems::UserCommands"/>'
    )
    parts.append(
        '    <plugin filename="gz-sim-scene-broadcaster-system" '
        'name="gz::sim::systems::SceneBroadcaster"/>'
    )
    parts.append(
        '    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">\n'
        "      <render_engine>ogre2</render_engine>\n"
        "    </plugin>"
    )

    parts.append(
        '    <light type="directional" name="sun">\n'
        "      <cast_shadows>false</cast_shadows>\n"
        "      <pose>0 0 20 0 0 0</pose>\n"
        "      <diffuse>0.8 0.8 0.8 1</diffuse>\n"
        "      <direction>0.3 0.3 -0.9</direction>\n"
        "    </light>"
    )

    parts.append(_ground_plane(w, h))

    parts.append(_static_box("wall_south", -t, -t, w + t, 0.0))
    parts.append(_static_box("wall_north", -t, h, w + t, h + t))
    parts.append(_static_box("wall_west", -t, 0.0, 0.0, h))
    parts.append(_static_box("wall_east", w, 0.0, w + t, h))

    occupied = set(occupied_space_ids)
    for space in lot.parking_spaces:
        if space.id not in occupied:
            continue
        xs = [p[0] for p in space.polygon]
        ys = [p[1] for p in space.polygon]
        parts.append(
            _static_box(
                f"car_{space.id}",
                min(xs) + car_inset_m,
                min(ys) + car_inset_m,
                max(xs) - car_inset_m,
                max(ys) - car_inset_m,
                height_m=0.4,
            )
        )

    for robot_id, x_m, y_m, theta_rad in robot_poses:
        parts.append(_robot_model(robot_id, x_m, y_m, theta_rad))

    parts.append("  </world>")
    parts.append("</sdf>")
    parts.append("")
    return "\n".join(parts)


def _ground_plane(w: float, h: float) -> str:
    cx, cy = w / 2.0, h / 2.0
    return (
        '    <model name="ground_plane">\n'
        "      <static>true</static>\n"
        f"      <pose>{cx} {cy} 0 0 0 0</pose>\n"
        "      <link name=\"link\">\n"
        "        <collision name=\"collision\">\n"
        "          <geometry>\n"
        f"            <plane><normal>0 0 1</normal><size>{w} {h}</size></plane>\n"
        "          </geometry>\n"
        "        </collision>\n"
        "        <visual name=\"visual\">\n"
        "          <geometry>\n"
        f"            <plane><normal>0 0 1</normal><size>{w} {h}</size></plane>\n"
        "          </geometry>\n"
        "          <material><ambient>0.5 0.5 0.5 1</ambient></material>\n"
        "        </visual>\n"
        "      </link>\n"
        "    </model>"
    )


def _static_box(
    name: str, x_min: float, y_min: float, x_max: float, y_max: float, *, height_m: float = 1.0
) -> str:
    cx = (x_min + x_max) / 2.0
    cy = (y_min + y_max) / 2.0
    sx = x_max - x_min
    sy = y_max - y_min
    return (
        f'    <model name="{name}">\n'
        "      <static>true</static>\n"
        f"      <pose>{cx} {cy} {height_m / 2.0} 0 0 0</pose>\n"
        "      <link name=\"link\">\n"
        "        <collision name=\"collision\">\n"
        f"          <geometry><box><size>{sx} {sy} {height_m}</size></box></geometry>\n"
        "        </collision>\n"
        "        <visual name=\"visual\">\n"
        f"          <geometry><box><size>{sx} {sy} {height_m}</size></box></geometry>\n"
        "        </visual>\n"
        "      </link>\n"
        "    </model>"
    )


def _robot_model(robot_id: str, x_m: float, y_m: float, theta_rad: float) -> str:
    radius = ROBOT_RADIUS_M
    height = _ROBOT_HEIGHT_M
    mass = _ROBOT_MASS_KG
    # Solid-cylinder inertia about its own axis and a diameter axis.
    izz = 0.5 * mass * radius * radius
    ixx = iyy = (mass / 12.0) * (3.0 * radius * radius + height * height)
    half_fov = LIDAR_FOV_RAD / 2.0
    return (
        f'    <model name="{robot_id}">\n'
        f"      <pose>{x_m} {y_m} {height / 2.0} 0 0 {theta_rad}</pose>\n"
        '      <link name="base_link">\n'
        "        <inertial>\n"
        f"          <mass>{mass}</mass>\n"
        f"          <inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>"
        "<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>\n"
        "        </inertial>\n"
        '        <collision name="collision">\n'
        f"          <geometry><cylinder><radius>{radius}</radius>"
        f"<length>{height}</length></cylinder></geometry>\n"
        "          <surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface>\n"
        "        </collision>\n"
        '        <visual name="visual">\n'
        f"          <geometry><cylinder><radius>{radius}</radius>"
        f"<length>{height}</length></cylinder></geometry>\n"
        "          <material><ambient>0.2 0.5 0.8 1</ambient></material>\n"
        "        </visual>\n"
        f'        <sensor name="lidar" type="gpu_lidar">\n'
        f"          <topic>{robot_id}/scan</topic>\n"
        "          <update_rate>20</update_rate>\n"
        f"          <pose>0 0 {-height / 2.0 + 0.05} 0 0 0</pose>\n"
        "          <lidar>\n"
        "            <scan>\n"
        f"              <horizontal><samples>{LIDAR_N_RAYS}</samples>"
        f"<min_angle>{-half_fov}</min_angle><max_angle>{half_fov}</max_angle></horizontal>\n"
        "            </scan>\n"
        f"            <range><min>0.05</min><max>{LIDAR_MAX_RANGE_M}</max>"
        "<resolution>0.01</resolution></range>\n"
        "          </lidar>\n"
        "          <always_on>true</always_on>\n"
        "          <visualize>false</visualize>\n"
        "        </sensor>\n"
        "      </link>\n"
        '      <plugin filename="gz-sim-velocity-control-system"\n'
        '              name="gz::sim::systems::VelocityControl">\n'
        f"        <topic>{robot_id}/cmd_vel</topic>\n"
        "        <link_name>base_link</link_name>\n"
        "      </plugin>\n"
        '      <plugin filename="gz-sim-odometry-publisher-system"\n'
        '              name="gz::sim::systems::OdometryPublisher">\n'
        f"        <odom_topic>{robot_id}/odom</odom_topic>\n"
        "        <odom_frame>odom</odom_frame>\n"
        "        <robot_base_frame>base_link</robot_base_frame>\n"
        "        <dimensions>2</dimensions>\n"
        "      </plugin>\n"
        "    </model>"
    )

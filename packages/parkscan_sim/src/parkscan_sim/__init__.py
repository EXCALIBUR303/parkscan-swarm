"""sim-lite: the deterministic 2D kinematic simulator.

``sim-lite`` is **2D kinematics on a flat ground plane**, not a physics engine. It
integrates a unicycle model at a fixed tick, casts exact geometric rays against
axis-aligned rectangles, and moves a linear battery number. There is no mass, inertia,
friction, contact response, wheel slip, sensor noise or rendering, and nothing it produces
may be described as real-world accuracy or as a physics simulation
(``docs/ACCEPTANCE.md`` H3, H12).

It is the default :class:`~parkscan_core.world.WorldBackend` because the project's
strongest claim -- ``make bench``, 30 seeds x 3 allocation strategies with bootstrap
confidence intervals -- needs a backend that is both fast and bit-reproducible
(ADR-0001). Same seed, same lot, same :class:`SimConfig` gives the same trajectory,
exactly; ``tests/test_determinism.py`` is the test that guards it.

Public surface::

    from parkscan_sim import SimConfig, SimLite

    sim = SimLite(SimConfig(robot_count=3))
    sim.reset(seed=7, lot=load_lot(Path("sim/lot/lot.yaml")))
    sim.actuate("robot_1", TwistCmd(v_mps=0.8, omega_radps=0.0))
    sim.step(sim.config.dt_s)
    frame = sim.sense("robot_1")
"""

from parkscan_sim.battery import (
    BatteryParams,
    charge_soc,
    discharge_power_w,
    drain_soc,
    endurance_s,
)
from parkscan_sim.config import SimConfig
from parkscan_sim.kinematics import TwistLimits, clamp_twist, integrate_pose, wrap_angle
from parkscan_sim.obstacles import (
    ObstacleField,
    Rect,
    build_obstacle_field,
    point_rect_distance,
    stall_aabb,
)
from parkscan_sim.sensors import SensorParams, cast_rays, ray_aabb_distance, ray_angles
from parkscan_sim.world import Collision, CollisionKind, SimLite

__all__ = [
    "BatteryParams",
    "Collision",
    "CollisionKind",
    "ObstacleField",
    "Rect",
    "SensorParams",
    "SimConfig",
    "SimLite",
    "TwistLimits",
    "build_obstacle_field",
    "cast_rays",
    "charge_soc",
    "clamp_twist",
    "discharge_power_w",
    "drain_soc",
    "endurance_s",
    "integrate_pose",
    "point_rect_distance",
    "ray_aabb_distance",
    "ray_angles",
    "stall_aabb",
    "wrap_angle",
]

"""``SimConfig``: every knob ``sim-lite`` has, in one frozen object.

One config object, constructed once and never mutated, is what makes a run describable:
``make bench`` records the config hash alongside the git SHA, so a row in ``results.csv``
can be traced to the exact parameters that produced it. A knob that lived on the
simulator as a mutable attribute could be changed mid-run and the CSV would not know.

Every default below is a *choice*, not a measurement, and the ones that shape the results
say why in their comments.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from parkscan_sim.battery import BatteryParams
from parkscan_sim.kinematics import TwistLimits
from parkscan_sim.sensors import SensorParams

__all__ = ["SimConfig"]


@dataclass(frozen=True, slots=True)
class SimConfig:
    """Configuration for one :class:`~parkscan_sim.world.SimLite` run."""

    dt_s: float = 0.05
    """Fixed tick, 20 Hz. ``step()`` rejects any other value: a variable tick would make
    the trajectory depend on how the caller's loop was scheduled, which is the one thing
    determinism cannot survive."""

    robot_count: int = 3
    """Three robots against two charging bays -- deliberately scarce (ADR-0001)."""

    robot_radius_m: float = 0.35
    """Robots are discs of this radius for collision reporting. A small inspection robot;
    the value is a modelling choice, not a measurement of any hardware."""

    limits: TwistLimits = field(default_factory=TwistLimits)
    battery: BatteryParams = field(default_factory=BatteryParams)
    sensor: SensorParams = field(default_factory=SensorParams)

    stall_occupancy_rate: float = 0.6
    """Fraction of stalls holding a parked car, sampled per stall from the run's seed.
    0.6 leaves plenty of visible free stalls for the occupancy report to be interesting
    while still filling the aisles with obstacles."""

    car_inset_m: float = 0.3
    """Gap between a stall's painted bounds and the parked car inside it, per side."""

    wall_thickness_m: float = 0.5
    """Thickness of the four boundary walls, which sit just *outside* the lot."""

    initial_soc_min: float = 0.85
    initial_soc_max: float = 1.0
    """Robots start nearly-but-not-identically charged, drawn from the run seed, so a
    fleet is never perfectly symmetric and the allocator's battery term does real work
    from the first tick."""

    start_heading_jitter_rad: float = 0.05
    """Start *positions* are derived from the lot (bay nodes first, then the lane nodes
    nearest the first bay) so a run is reproducible from the lot file alone; the start
    *heading* carries a small seeded jitter about the nominal lane direction. About 3
    degrees is enough to make two seeds diverge without starting a robot facing a wall."""

    charge_dock_radius_m: float = 1.0
    """A robot within this distance of a charging bay position **and** stationary is
    treated as docked and charges. Proximity plus stationarity, rather than an explicit
    dock command, keeps the simulator ignorant of the autonomy layer's intent."""

    charge_speed_eps: float = 1e-3

    pothole_count: int = 24
    """How many potholes to scatter on the driving surface.

    Potholes are *simulated ground truth*: sim-lite owns the world, so it owns where the
    damage is. The perception layer never sees this — it receives only image-space boxes —
    which is what keeps the geolocation error an honest measurement rather than a lookup.
    """

    pothole_radius_min_m: float = 0.12
    pothole_radius_max_m: float = 0.45
    """Radius range. A pothole below ~0.1 m is not worth reporting and one above ~0.5 m
    would be a sinkhole, so the band is deliberately narrow and is a modelling choice,
    not a measurement of real road damage."""

    aisle_offset_m: float = 5.0
    """Distance from a zone band's south edge to the drive-aisle centreline. Matches the
    authored lot (2 m kerb + a 6 m aisle spanning y0+2..y0+8)."""

    """"Stationary" tolerance for the docking rule, in m/s and rad/s."""

    def __post_init__(self) -> None:
        if not math.isfinite(self.dt_s) or self.dt_s <= 0.0:
            msg = f"dt_s must be finite and positive, got {self.dt_s}"
            raise ValueError(msg)
        if self.robot_count < 1:
            msg = f"robot_count must be at least 1, got {self.robot_count}"
            raise ValueError(msg)
        for name in (
            "robot_radius_m",
            "car_inset_m",
            "wall_thickness_m",
            "charge_dock_radius_m",
            "charge_speed_eps",
        ):
            value: float = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                msg = f"{name} must be finite and positive, got {value}"
                raise ValueError(msg)
        if not 0.0 <= self.stall_occupancy_rate <= 1.0:
            msg = f"stall_occupancy_rate must lie in [0, 1], got {self.stall_occupancy_rate}"
            raise ValueError(msg)
        if not 0.0 <= self.initial_soc_min <= self.initial_soc_max <= 1.0:
            msg = (
                "initial state of charge must satisfy 0 <= min <= max <= 1, got "
                f"[{self.initial_soc_min}, {self.initial_soc_max}]"
            )
            raise ValueError(msg)
        if not math.isfinite(self.start_heading_jitter_rad) or self.start_heading_jitter_rad < 0.0:
            msg = (
                "start_heading_jitter_rad must be finite and non-negative, got "
                f"{self.start_heading_jitter_rad}"
            )
            raise ValueError(msg)

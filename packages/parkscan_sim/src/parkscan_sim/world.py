"""``SimLite`` -- the deterministic 2D kinematic backend.

What this is
------------
A **2D kinematic simulator on a flat ground plane**. It integrates a unicycle model at a
fixed tick, ray-casts a planar scan against axis-aligned rectangles, and moves a linear
battery number up and down. It is not a physics engine: there is no mass, no inertia, no
friction, no contact response, no wheel slip, no sensor noise and no rendering. Every
number it produces is a property of that model and of the authored lot, and no screenshot
or metric from it may be presented as a physics simulation or as real-world accuracy
(``docs/ACCEPTANCE.md`` H3, H12).

Why it is the default backend
-----------------------------
ADR-0001: the project's strongest claim is ``make bench`` -- 30 seeded runs x 3 allocation
strategies with paired bootstrap confidence intervals, regenerating every README number
from a committed CSV. That experiment is impossible at the 0.2-0.5 real-time factor a
software-rendered Gazebo would give on this machine. A fast, reproducible backend is what
turns a demo into an experiment.

Determinism is the product
--------------------------
Same ``(seed, lot, SimConfig)`` must give a bit-identical trajectory, because every
benchmark number depends on it. The rules this module keeps, without exception:

* **One generator.** ``reset`` builds exactly one ``numpy.random.Generator`` from the
  seed. There is no module-level RNG, no ``random`` module, and no second stream.
* **A fixed draw order.** ``reset`` draws stall occupancy, then start-heading jitter, then
  initial state of charge -- always in that order, always with fixed sizes, so adding a
  future draw at the end cannot silently shift the existing ones.
* **No dict or set iteration drives behaviour.** Robots are held in insertion order,
  obstacles in construction order, lane nodes sorted by id. Sets are used for membership
  only, never to produce a sequence.
* **No wall clock.** ``now_ns`` is ``tick * dt_ns``. ``time`` is not imported here.
* **A fixed tick.** ``step`` rejects any ``dt`` other than ``SimConfig.dt_s``.

Collisions are reported, never resolved
---------------------------------------
Robots are discs. Overlap with another robot or with a static obstacle is recorded as a
:class:`Collision` and nothing else happens: no push-apart, no velocity zeroing, no
sliding. Acceptance criterion C1 asserts ``conflict_count == 0``, and a simulator that
quietly separated overlapping robots would make that assertion vacuous -- the reservation
system could be completely broken and C1 would still pass.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum
from typing import NoReturn

import numpy as np

from parkscan_core.lot import LaneNode, LotSpec
from parkscan_core.types import BayState, Pose2D, RobotId, SensorFrame, TwistCmd
from parkscan_sim.battery import charge_soc, drain_soc
from parkscan_sim.config import SimConfig
from parkscan_sim.kinematics import clamp_twist, integrate_pose, wrap_angle
from parkscan_sim.obstacles import ObstacleField, build_obstacle_field
from parkscan_sim.sensors import cast_rays

__all__ = ["Collision", "CollisionKind", "SimLite"]

_ZERO_TWIST = TwistCmd(v_mps=0.0, omega_radps=0.0)
_NS_PER_S = 1_000_000_000


class CollisionKind(StrEnum):
    """What overlapped what."""

    ROBOT_ROBOT = "robot_robot"
    ROBOT_OBSTACLE = "robot_obstacle"


@dataclass(frozen=True, slots=True)
class Collision:
    """One collision *episode*, recorded at the tick the overlap began.

    An episode, not a per-tick sample: a robot parked inside a parked car would otherwise
    emit one record every tick and drown the run report in a single fault. ``other`` is
    the second robot's id or the obstacle's id, and ``overlap_m`` is how deep the discs
    interpenetrated at onset -- a severity hint, not a physical quantity, since nothing
    here resolves the overlap.
    """

    kind: CollisionKind
    stamp_ns: int
    robot: RobotId
    other: str
    overlap_m: float


@dataclass(slots=True)
class _RobotState:
    """Mutable per-robot state. Private: everything outside reads it through ``sense``."""

    pose: Pose2D
    command: TwistCmd
    measured: TwistCmd
    battery_soc: float
    charging: bool


class SimLite:
    """A deterministic 2D kinematic world implementing
    :class:`~parkscan_core.world.WorldBackend`.

    Typical use::

        sim = SimLite()
        sim.reset(seed=7, lot=load_lot(Path("sim/lot/lot.yaml")))
        for _ in range(400):
            for robot in sim.robots():
                sim.actuate(robot, policy(sim.sense(robot)))
            sim.step(sim.config.dt_s)
    """

    __slots__ = (
        "_active",
        "_bays",
        "_cfg",
        "_collisions",
        "_dt_ns",
        "_field",
        "_lot",
        "_occupied",
        "_order",
        "_rng",
        "_robots",
        "_seed",
        "_tick",
    )

    def __init__(self, config: SimConfig | None = None) -> None:
        self._cfg = config if config is not None else SimConfig()
        self._lot: LotSpec | None = None
        self._rng: np.random.Generator | None = None
        self._seed: int | None = None
        self._robots: dict[RobotId, _RobotState] = {}
        self._order: list[RobotId] = []
        self._field = ObstacleField(())
        self._occupied: tuple[str, ...] = ()
        self._bays: tuple[tuple[str, float, float], ...] = ()
        self._collisions: list[Collision] = []
        self._active: set[tuple[CollisionKind, str, str]] = set()
        self._tick = 0
        # The tick is quantised to whole nanoseconds, and now_ns() is defined as
        # tick * dt_ns -- so the sim clock is exactly consistent with itself even when
        # dt_s is not an exact number of nanoseconds (1/30 s, say).
        self._dt_ns = round(self._cfg.dt_s * _NS_PER_S)

    # ------------------------------------------------------------------ properties --
    @property
    def config(self) -> SimConfig:
        return self._cfg

    @property
    def seed(self) -> int:
        """The seed of the current run. Raises before the first :meth:`reset`."""
        if self._seed is None:
            self._not_ready()
        return self._seed

    @property
    def lot(self) -> LotSpec:
        """The lot of the current run. Raises before the first :meth:`reset`."""
        if self._lot is None:
            self._not_ready()
        return self._lot

    # -------------------------------------------------------------- WorldBackend --
    def reset(self, seed: int, lot: LotSpec) -> None:
        """Rebuild the world from ``lot``, seeding every stochastic choice from ``seed``.

        All prior state is discarded: robots, obstacles, the collision log, the clock and
        the generator. Calling ``reset`` twice with the same arguments yields two worlds
        that are indistinguishable through this interface.

        Start **positions** are derived from the lot alone and are therefore reproducible
        from the committed lot file without knowing the seed: the charging-bay nodes come
        first, in ``lot.charging_bays`` order, then the remaining lane nodes ordered by
        distance to the first bay with the node id breaking ties. With the authored lot
        and three robots that is ``bay_1_node``, ``bay_2_node``, ``wl_s1`` -- the fleet
        starts at the depot, two of them docked.

        Start **headings** are lot-derived too -- each robot faces its start node's
        lowest-id neighbour, i.e. along a real lane -- plus a seeded jitter of at most
        ``SimConfig.start_heading_jitter_rad``. Initial state of charge is drawn from
        ``[initial_soc_min, initial_soc_max]``. Those two draws, together with which
        stalls are occupied, are the whole of what the seed controls.
        """
        self._lot = lot
        self._seed = int(seed)
        self._rng = np.random.default_rng(self._seed)
        self._robots = {}
        self._order = []
        self._collisions = []
        self._active = set()
        self._tick = 0
        self._dt_ns = round(self._cfg.dt_s * _NS_PER_S)

        rng = self._rng
        count = self._cfg.robot_count

        # Draw 1 of 3: stall occupancy, one uniform per stall in lot.parking_spaces order.
        draw = rng.random(len(lot.parking_spaces))
        self._occupied = tuple(
            space.id
            for space, value in zip(lot.parking_spaces, draw, strict=True)
            if float(value) < self._cfg.stall_occupancy_rate
        )
        self._field = build_obstacle_field(
            lot,
            self._occupied,
            car_inset_m=self._cfg.car_inset_m,
            wall_thickness_m=self._cfg.wall_thickness_m,
        )
        self._bays = tuple((b.id, b.position[0], b.position[1]) for b in lot.charging_bays)

        start_nodes = _start_nodes(lot, count)
        headings = _nominal_headings(lot, start_nodes)
        # Draw 2 of 3: heading jitter. Draw 3 of 3: initial state of charge.
        jitter = rng.uniform(
            -self._cfg.start_heading_jitter_rad, self._cfg.start_heading_jitter_rad, size=count
        )
        socs = rng.uniform(self._cfg.initial_soc_min, self._cfg.initial_soc_max, size=count)

        for index, node in enumerate(start_nodes):
            pose = Pose2D(
                x_m=node.x_m,
                y_m=node.y_m,
                theta_rad=wrap_angle(headings[index] + float(jitter[index])),
            )
            self.add_robot(pose, battery_soc=float(socs[index]))

        self._detect_collisions()

    def step(self, dt: float) -> None:
        """Advance the world by exactly one tick.

        Order, fixed and documented because determinism depends on it:

        1. every robot in :meth:`robots` order integrates its latched command with
           midpoint RK2, and records the twist that was actually applied;
        2. every robot's battery charges (docked and stationary) or drains;
        3. the tick counter advances;
        4. collisions are re-detected and any newly-begun overlap is logged.

        ``dt`` must equal ``SimConfig.dt_s``. A variable tick would make the trajectory a
        function of how the caller's loop happened to be scheduled, and the whole
        reproducibility claim would quietly become false.
        """
        self._require_ready()
        if not math.isfinite(dt) or dt <= 0.0:
            msg = f"dt must be finite and positive, got {dt}"
            raise ValueError(msg)
        if not math.isclose(dt, self._cfg.dt_s, rel_tol=1e-12, abs_tol=0.0):
            msg = (
                f"sim-lite runs at a fixed tick: step({dt}) but SimConfig.dt_s is "
                f"{self._cfg.dt_s}. Change the config, not the call."
            )
            raise ValueError(msg)

        for robot_id in self._order:
            state = self._robots[robot_id]
            # A flat battery cannot drive. Without this a depleted robot keeps sweeping
            # and acceptance criterion C2 (depletion_events == 0) becomes unmeasurable.
            applied = _ZERO_TWIST if state.battery_soc <= 0.0 else state.command
            state.pose = integrate_pose(state.pose, applied, dt)
            state.measured = applied

        for robot_id in self._order:
            state = self._robots[robot_id]
            state.charging = self._is_docked(state)
            if state.charging:
                state.battery_soc = charge_soc(state.battery_soc, dt, self._cfg.battery)
            else:
                state.battery_soc = drain_soc(
                    state.battery_soc, state.measured.v_mps, dt, self._cfg.battery
                )

        self._tick += 1
        self._detect_collisions()

    def now_ns(self) -> int:
        """Simulation time in nanoseconds: ``tick * dt_ns``. Never the wall clock."""
        return self._tick * self._dt_ns

    def robots(self) -> Sequence[RobotId]:
        """``("robot_1", "robot_2", "robot_3")`` -- creation order, stable after reset."""
        return tuple(self._order)

    def sense(self, robot: RobotId) -> SensorFrame:
        """The current observation for ``robot``.

        ``camera`` is always ``None`` in this phase. ``sim-lite`` renders nothing, and
        fabricating an array of pixels here would be a lie that the perception contract
        test would then happily pass. Phase 3 adds synthetic frames built from
        ground-truth geometry, and they will carry ``DetSource.SIMULATED`` all the way
        into the database.
        """
        state = self._state(robot)
        return SensorFrame(
            stamp_ns=self.now_ns(),
            pose=state.pose,
            twist=state.measured,
            battery_soc=state.battery_soc,
            ranges=cast_rays(state.pose, self._field, self._cfg.sensor),
            camera=None,
        )

    def actuate(self, robot: RobotId, cmd: TwistCmd) -> None:
        """Latch ``cmd`` (saturated to :class:`~parkscan_sim.kinematics.TwistLimits`).

        The command persists until the next call, so a caller that stops actuating leaves
        the robot driving -- the same failure mode a real velocity interface has.
        """
        self._state(robot).command = clamp_twist(cmd, self._cfg.limits)

    # ------------------------------------------------------------------ fleet API --
    def add_robot(self, pose: Pose2D, *, battery_soc: float = 1.0) -> RobotId:
        """Add a robot at ``pose`` and return its id.

        Ids are assigned in creation order as ``robot_1``, ``robot_2``, ``robot_3``, ...
        and are never reused within a run. :meth:`reset` calls this once per
        ``SimConfig.robot_count``; calling it afterwards is how a scenario adds a fourth
        robot mid-run without consuming any of the run's random draws.
        """
        self._require_ready()
        if not 0.0 <= battery_soc <= 1.0:
            msg = f"battery_soc must lie in [0, 1], got {battery_soc}"
            raise ValueError(msg)
        robot_id: RobotId = f"robot_{len(self._order) + 1}"
        self._robots[robot_id] = _RobotState(
            pose=pose,
            command=_ZERO_TWIST,
            measured=_ZERO_TWIST,
            battery_soc=battery_soc,
            charging=False,
        )
        self._order.append(robot_id)
        return robot_id

    def place(self, robot: RobotId, pose: Pose2D, *, battery_soc: float | None = None) -> None:
        """Teleport ``robot`` -- scenario setup and failure injection only.

        This is not something a robot can do to itself. It exists so a test can construct
        the exact situation it wants to assert on (two robots overlapping, a robot
        stranded far from a bay) without driving there first. It re-runs collision
        detection immediately, and like everything else here it reports rather than
        prevents: placing a robot inside a parked car logs a collision, it does not raise.
        """
        state = self._state(robot)
        state.pose = pose
        if battery_soc is not None:
            if not 0.0 <= battery_soc <= 1.0:
                msg = f"battery_soc must lie in [0, 1], got {battery_soc}"
                raise ValueError(msg)
            state.battery_soc = battery_soc
        self._detect_collisions()

    def pose(self, robot: RobotId) -> Pose2D:
        """Ground-truth pose. ``sim-lite`` has no localisation error, and says so."""
        return self._state(robot).pose

    def battery_soc(self, robot: RobotId) -> float:
        return self._state(robot).battery_soc

    def set_battery_soc(self, robot: RobotId, soc: float) -> None:
        """Force a robot's state of charge. Scenario setup, not part of the physics.

        Exists so a demo or a test can start a robot near-flat and exercise the
        return-to-charge and docking behaviour in seconds rather than waiting out a full
        discharge, and so Phase 4 can inject a low-battery failure at a chosen moment.
        Calling it mid-run does not otherwise perturb the simulation, so a run that uses
        it is still reproducible from (seed, scenario).
        """
        if not 0.0 <= soc <= 1.0:
            msg = f"soc must be in [0, 1], got {soc}"
            raise ValueError(msg)
        self._state(robot).battery_soc = soc

    def is_charging(self, robot: RobotId) -> bool:
        """Whether ``robot`` charged on the most recent :meth:`step`."""
        return self._state(robot).charging

    def is_depleted(self, robot: RobotId) -> bool:
        """Whether ``robot`` is flat, and therefore unable to move. Counted by C2."""
        return self._state(robot).battery_soc <= 0.0

    # ------------------------------------------------------------------- world API --
    def obstacles(self) -> ObstacleField:
        """The static obstacle set this run is using: four walls plus the parked cars."""
        return self._field

    def occupied_space_ids(self) -> tuple[str, ...]:
        """Ids of the stalls this run decided are occupied, in ``lot.parking_spaces`` order."""
        return self._occupied

    def ground_truth_occupancy(self) -> Mapping[str, BayState]:
        """Every stall's true occupancy.

        Ground truth, for scoring and for the Phase 3 simulated detector to derive
        observations from -- never for the autonomy layer to read directly. A robot
        learns occupancy by observing it; the fleet reports
        :attr:`~parkscan_core.types.BayState.UNKNOWN` until it has.
        """
        occupied = set(self._occupied)
        return {
            space.id: (BayState.OCCUPIED if space.id in occupied else BayState.VACANT)
            for space in self.lot.parking_spaces
        }

    def collisions(self) -> tuple[Collision, ...]:
        """Every collision episode since :meth:`reset`, in the order they began."""
        return tuple(self._collisions)

    def collision_count(self) -> int:
        """Number of collision episodes since :meth:`reset`. C1 requires zero."""
        return len(self._collisions)

    def active_collision_count(self) -> int:
        """Number of overlaps currently in progress."""
        return len(self._active)

    # --------------------------------------------------------------------- private --
    def _not_ready(self) -> NoReturn:
        msg = "SimLite.reset(seed, lot) must be called before the world can be used"
        raise RuntimeError(msg)

    def _require_ready(self) -> None:
        if self._lot is None:
            self._not_ready()

    def _state(self, robot: RobotId) -> _RobotState:
        try:
            return self._robots[robot]
        except KeyError:
            self._require_ready()
            msg = f"unknown robot {robot!r}; this world has {tuple(self._order)!r}"
            raise KeyError(msg) from None

    def _is_docked(self, state: _RobotState) -> bool:
        eps = self._cfg.charge_speed_eps
        if abs(state.measured.v_mps) > eps or abs(state.measured.omega_radps) > eps:
            return False
        radius = self._cfg.charge_dock_radius_m
        return any(
            math.dist((state.pose.x_m, state.pose.y_m), (bay_x, bay_y)) <= radius
            for _, bay_x, bay_y in self._bays
        )

    def _detect_collisions(self) -> None:
        """Recompute overlaps and log the ones that have just begun.

        Deliberately O(n^2) over robots and O(n * rects) against obstacles. With three
        robots and roughly eighty rectangles that is a few hundred comparisons per tick,
        and a broad-phase index here would be optimisation with no measurement behind it.
        """
        radius = self._cfg.robot_radius_m
        stamp = self.now_ns()
        current: set[tuple[CollisionKind, str, str]] = set()
        started: list[Collision] = []

        for i, robot_a in enumerate(self._order):
            pose_a = self._robots[robot_a].pose
            for robot_b in self._order[i + 1 :]:
                pose_b = self._robots[robot_b].pose
                separation = math.dist((pose_a.x_m, pose_a.y_m), (pose_b.x_m, pose_b.y_m))
                if separation >= 2.0 * radius:
                    continue
                key = (CollisionKind.ROBOT_ROBOT, robot_a, robot_b)
                current.add(key)
                if key not in self._active:
                    started.append(
                        Collision(
                            kind=CollisionKind.ROBOT_ROBOT,
                            stamp_ns=stamp,
                            robot=robot_a,
                            other=robot_b,
                            overlap_m=2.0 * radius - separation,
                        )
                    )

        if len(self._field) > 0:
            ids = self._field.ids
            for robot_id in self._order:
                pose = self._robots[robot_id].pose
                distances = self._field.point_distances(pose.x_m, pose.y_m)
                for index in np.flatnonzero(distances < radius):
                    obstacle_id = ids[int(index)]
                    key = (CollisionKind.ROBOT_OBSTACLE, robot_id, obstacle_id)
                    current.add(key)
                    if key not in self._active:
                        started.append(
                            Collision(
                                kind=CollisionKind.ROBOT_OBSTACLE,
                                stamp_ns=stamp,
                                robot=robot_id,
                                other=obstacle_id,
                                overlap_m=radius - float(distances[index]),
                            )
                        )

        self._collisions.extend(started)
        self._active = current


# ------------------------------------------------------------------ start placement --
def _start_nodes(lot: LotSpec, count: int) -> tuple[LaneNode, ...]:
    """Deterministic start nodes: charging bays first, then the lane nodes nearest them.

    Seed-independent on purpose. Where a run *starts* is a property of the lot, so two
    seeds differ in what they do rather than in where they begin, and a benchmark row can
    be reproduced from the committed lot file plus the seed alone.
    """
    node_map = lot.lane_graph.node_map()
    chosen: list[LaneNode] = []
    used: set[str] = set()

    for bay in lot.charging_bays:
        if bay.node in used:
            continue
        chosen.append(node_map[bay.node])
        used.add(bay.node)

    if len(chosen) < count:
        anchor = chosen[0] if chosen else min(lot.lane_graph.nodes, key=lambda n: n.id)
        remaining = sorted(
            (n for n in lot.lane_graph.nodes if n.id not in used),
            key=lambda n: (math.dist((n.x_m, n.y_m), (anchor.x_m, anchor.y_m)), n.id),
        )
        for node in remaining:
            if len(chosen) >= count:
                break
            chosen.append(node)
            used.add(node.id)

    if len(chosen) < count:
        msg = (
            f"lot {lot.name!r} has only {len(chosen)} distinct lane nodes to start "
            f"{count} robots from"
        )
        raise ValueError(msg)
    return tuple(chosen[:count])


def _nominal_headings(lot: LotSpec, nodes: Sequence[LaneNode]) -> tuple[float, ...]:
    """Face each start node's lowest-id neighbour, so every robot begins along a lane.

    Lowest id rather than "first edge in the file" so the result survives a reordering of
    the YAML, and rather than "nearest neighbour" so it survives a geometry tweak too.
    """
    adjacency: dict[str, list[str]] = {}
    for edge in lot.lane_graph.edges:
        adjacency.setdefault(edge.a, []).append(edge.b)
        adjacency.setdefault(edge.b, []).append(edge.a)
    node_map = lot.lane_graph.node_map()

    headings: list[float] = []
    for node in nodes:
        neighbours = adjacency.get(node.id)
        if not neighbours:
            headings.append(0.0)
            continue
        target = node_map[min(neighbours)]
        headings.append(math.atan2(target.y_m - node.y_m, target.x_m - node.x_m))
    return tuple(headings)

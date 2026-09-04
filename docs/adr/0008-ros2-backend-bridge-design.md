# ADR-0008 — `Ros2WorldBackend`: namespace scheme, lockstep timing, and the MQTT bridge

**Status:** Proposed, not implemented · **Extends:** ADR-0001, ADR-0002 · **Gated by:** the RTF
spike in `docs/ROADMAP.md` (passed, RTF ≈ 1.0)

## Context

`docs/ROADMAP.md`'s RTF spike has passed, so ADR-0001's Phase S is now viable to attempt. But
"implement `Ros2WorldBackend`" is not a single decision — four things TASKS.md names in one
breath ("`ros:jazzy-ros-base` arm64 Dockerfile", "`rmw_zenoh_cpp` + router; one world, 3
namespaces", "`Ros2WorldBackend` implementing the same Protocol; MQTT bridge") turn out to
depend on a fact none of the existing docs state explicitly:

**`rclpy` cannot run on this host.** ADR-0001 already establishes "ROS 2 does not support
macOS on arm64 at any tier" as the reason the whole `WorldBackend` adapter exists. That fact
doesn't just justify the adapter boundary in the abstract — it means the *process* that calls
`WorldBackend.sense()`/`.actuate()` (today, `apps/runner`'s `Runner`, running in the host's `uv`
venv) can never itself hold an `rclpy` import. `packages/parkscan_core/src/parkscan_core/world.py`'s
own docstring describes the ROS 2 adapter as *"a thin node: subscribe `/<ns>/odom`,
`/<ns>/scan`, `/<ns>/camera/image_raw`; publish `/<ns>/cmd_vel`"* — but a `WorldBackend`
instance that is itself an `rclpy` node cannot be constructed on this machine. Something has to
give, and `docs/ARCHITECTURE.md`'s component map already names the answer in one line without
spelling out how: *"bridges to the same MQTT topics; nothing above the adapter changes."*

This ADR is that spelling-out. It exists because every one of this project's other
multi-week-effort phases got a design review before implementation (Phase 4a preceded 4b for
exactly this reason), and Phase S is the first ROS 2 code in the repository — worth the same
discipline, not less, given how much of it depends on facts (the multi-hour Docker/Gazebo
feedback loop demonstrated by the RTF spike itself) that are expensive to discover by trial.

Three further gaps, found while researching this ADR, that no existing doc addresses:

1. **Timing.** `SimLite.step(dt)` is a synchronous, deterministic advance: call it, and by the
   time it returns every robot has moved by exactly `dt` with no wall-clock dependency. Gazebo,
   run with `-r` (auto-run), advances on its *own* clock regardless of whether anyone calls a
   Python method. A `Ros2WorldBackend.step(dt)` that just `sleep(dt)`s and reads back state
   would make every acceptance number depend on the host's actual RTF holding at exactly 1.0 at
   that instant — precisely the wall-clock coupling the Protocol's own docstring forbids
   (*"Backends must not sub-step adaptively in a way that makes the result depend on wall-clock
   timing"*).
2. **Battery.** Gazebo/`ros_gz` has no battery model. `parkscan_sim.battery`'s linear proxy
   (`idle_w`, `move_w_per_mps`, `capacity_wh`) is pure and backend-agnostic already, but it
   currently lives inside the `parkscan_sim` package, which a second backend package
   importing it would depend on as a library — an odd shape for two peer backends.
3. **Reproducibility.** `SimLite.reset(seed, lot)` draws four seeded quantities: stall
   occupancy, heading jitter, initial SOC, and pothole placement. Three of those are baked into
   Gazebo's *world geometry* (occupied stalls, robot headings) or perception ground truth
   (potholes) — none of which a `.sdf` file can re-roll per seed without regenerating and
   reloading the world, which is out of scope here. Only initial SOC is a plain value a
   Python-side RNG can still seed. `Ros2WorldBackend` will *not* give the same bit-identical
   reproducibility guarantee `SimLite` does, and no existing doc says so.

## Decision

### 1. Namespace scheme

One Gazebo world, one `.sdf` authored from `sim/lot/lot.yaml`'s geometry (150 × 100 m ground
plane, wall/occupied-stall boxes analogous to `parkscan_sim.obstacles.build_obstacle_field`,
three robot models at the same seed-independent start poses `SimLite` already derives from the
lot's charging-bay/lane-node positions). Three ROS 2 namespaces, `/robot_1`, `/robot_2`,
`/robot_3` — matching `SimLite.robots()` and `docs/DATA_MODEL.md`'s `robot.id` values exactly,
so nothing above the backend layer needs to know which backend produced an id. Each namespace
carries the four topics `parkscan_core/world.py` already names: `/<ns>/odom` (`nav_msgs/Odometry`),
`/<ns>/scan` (`sensor_msgs/LaserScan`), `/<ns>/camera/image_raw` (`sensor_msgs/Image`), `/<ns>/cmd_vel`
(`geometry_msgs/Twist`) — wired via `ros_gz_bridge`'s topic-mapping YAML, one entry per topic
per robot.

The lidar sensor plugin in the SDF **must** be configured for 37 rays over a 1.5π field of view
capped at 12.0 m — `parkscan_sim.sensors.SensorParams`'s exact defaults. `SensorFrame.ranges` is
documented as a *"fixed-count planar scan"*; nothing downstream tolerates a different count.

### 2. Coordinate frame: identity, not a transform

The SDF's world origin is `lot.yaml`'s `(0, 0)` with the ground plane in the X/Y plane, Z up.
Gazebo pose `(x, y, yaw)` in metres maps directly onto `Pose2D(x_m=x, y_m=y, theta_rad=yaw)` —
no rotation, no offset, no unit conversion. This is a deliberate simplicity choice, not an
oversight: introducing a transform here would be a second, silent place reproducibility could
drift between backends, on top of the reproducibility gap §4 already states plainly.

### 3. Timing: lockstep via the world control service, not wall-clock sleep

Gazebo is launched **paused** (no `-r`), with a fixed physics `step_size` in the SDF matching
`SimConfig.dt_s = 0.05` (e.g. `step_size: 0.001` with 50 iterations per call, or `step_size:
0.05` with 1 — either is fine as long as it is exact, not approximated). `Ros2WorldBackend.step(dt)`
calls the world's own `/world/<name>/control` service (already confirmed live and serving in
the RTF spike's own log output) with a `multi_step` request for exactly `dt / step_size`
iterations, then blocks on the response before returning. This is synchronous and wall-clock
independent by construction: the call returns only once Gazebo has actually advanced by exactly
`dt`, regardless of host load or actual RTF at that moment. `reset(seed, lot)` uses the same
service's reset field to snap physics and poses back to the SDF's initial state, decomposed
from the *separate* Python-side battery-RNG reset described next.

### 4. Battery: reuse the pure proxy, own it in `Ros2WorldBackend`, propose relocating it

`parkscan_sim.battery`'s functions (`drain_soc`, `charge_soc`, `discharge_power_w`,
`BatteryParams`) take `(soc, v_mps, dt, params)` — no `SimLite` state, no world reference. They
are already exactly as backend-agnostic as `parkscan_core.lot`/`parkscan_core.types`.
**Recommendation for whoever implements this ADR: relocate `battery.py` from `parkscan_sim` to
`parkscan_core`, re-exported from `parkscan_sim` for the existing import sites.** This is a
refactor of tested, committed code and is explicitly *not* being done as part of writing this
ADR — it is a should-do named here so the implementer doesn't have to re-derive the same
architectural observation.

`Ros2WorldBackend` (not the bridge) owns per-robot SOC as a plain Python float, updated every
`step(dt)` from the *measured* twist Gazebo reports back (the real, physics-simulated velocity —
matching `SimLite`'s own "measured twist, not commanded" discipline) and from proximity to a
charging bay (same `lot.charging_bays` positions and the same `charge_dock_radius_m` semantics
`SimLite` uses, so "docked" means the same thing on both backends). The bridge, by contrast,
relays pose/twist/ranges only — it has no battery concept and needs none.

### 5. Reset/seed semantics — the reproducibility gap, stated not hidden

`Ros2WorldBackend.reset(seed, lot)` seeds a local `np.random.default_rng(seed)` for **initial
SOC only** — the one draw with no physical-geometry dependency. It does **not** attempt to
re-roll occupied-stall layout or pothole placement per seed: those are fixed at SDF-authoring
time (a single representative draw, not re-randomized per run). Start poses are seed-independent
already on both backends (derived the same way from `lot.yaml`), so that part transfers with no
gap. **This must be documented plainly wherever Phase S results are reported: `Ros2WorldBackend`
gives the same *initial battery* per seed, not the same *world* per seed. `make bench`'s
bit-identical-trajectory claim is a `SimLite` property and must not be implied for the ROS 2
backend without saying so.**

### 6. The MQTT bridge — concrete wire protocol (currently unspecified anywhere)

A single-purpose `rclpy` node running **inside** the `ros2-gz` container is the only process in
the system that imports `rclpy`. It subscribes to each robot's `/odom` and `/scan` (and
`/camera/image_raw`, published but not yet consumed — see Scope below) and republishes them,
and separately subscribes to an inbound command channel and republishes onto `/<ns>/cmd_vel`.
The channel is MQTT, on the same broker (`mosquitto`, already proven to cross the Docker Desktop
VM boundary for the app's own telemetry) but on **topics distinct from `docs/API_CONTRACT.md`'s
app-level contract** — this is an internal, implementation-detail channel, not something a
dashboard or any other consumer should ever subscribe to:

| Topic | Direction | Payload | QoS |
|---|---|---|---|
| `parkscan/bridge/v1/{robot_id}/state` | bridge → backend | `{"stamp_ns", "pose": {"x_m","y_m","theta_rad"}, "twist": {"v_mps","omega_radps"}, "ranges": [...]}` | 0 |
| `parkscan/bridge/v1/{robot_id}/cmd` | backend → bridge | `{"v_mps", "omega_radps"}` | 0 |

QoS 0, deliberately different from the app-level contract's QoS 1: this is a live-state relay
sampled every physics step — a dropped or superseded message is simply stale data waiting to be
overwritten by the next one, not an event whose loss matters (unlike a detection or a lease
grant). `battery_soc` is never on this channel — §4 already assigns that field to
`Ros2WorldBackend` alone, so the bridge does not need to know battery parameters at all.

### 7. Runner plumbing (expected, not a Protocol violation)

`apps/runner/src/parkscan_runner/run.py` currently hardcodes `SimLite(SimConfig())`.
`RunnerConfig` needs a `backend: Literal["sim-lite", "ros2-gz"]` field and a `--backend` CLI
flag selecting which concrete class to construct — both already typed as `WorldBackend`. This
is the one change the coordinator/perception/agent layers are exempt from needing: they call
only Protocol methods and never inspect which backend they were given, so "unmodified" in
TASKS.md's verify step means *them*, not the runner's own bootstrap code.

## Scope — stated plainly, so it isn't discovered as a surprise later

- **Perception parity is explicitly out of scope.** `SimLite.potholes()` (ground-truth ready
  for a simulated detector) has no analogue in a Gazebo-backed world without a real camera
  pipeline, which is a materially larger effort than "same Protocol." `Ros2WorldBackend.sense()`
  returns `camera=None` always, exactly like `SimLite` does today — `apps/runner`'s
  `_perceive()` already degrades to zero detections gracefully for any non-`SimLite` backend
  (an `isinstance` check, not a crash), so this is a silent no-op today and will remain one.
  Phase S verification is telemetry/navigation/coordination parity, not perception parity.
- **Reproducibility parity is explicitly out of scope** beyond §5's initial-SOC seeding.
- **Dynamic robot discovery is out of scope.** Three robots, three namespaces, fixed at
  authoring time on both the bridge and `Ros2WorldBackend` sides.
- **Docker Compose:** additive only — a new `ros2` profile service (`docker compose --profile
  ros2 up`), never a change to the default `db`/`mqtt` services or to the documented rule that
  `make dev` works with Docker down entirely for the `sim-lite` path.

## Consequences

- ✅ `WorldBackend`'s six methods stay exactly as specified in `parkscan_core/world.py` — no
  Protocol change, no coordinator/agent/perception code change.
- ✅ The lockstep-via-control-service design (§3) means the *stepping* is genuinely wall-clock
  independent, satisfying the Protocol's own constraint, rather than merely being close to real
  time because RTF happens to measure near 1.0.
- ✅ Reusing `parkscan_sim.battery`'s pure functions keeps exactly one battery model in the
  system, matching the project's existing stated principle (`battery_policy.py`'s own docstring:
  *"there is exactly one battery model in the system"* — recommending it move to `parkscan_core`
  keeps that true structurally, not just by convention).
- ⚠️ `Ros2WorldBackend` is only partially seed-reproducible (§5). Any Phase S benchmark run must
  say so next to its numbers, the same way this project already states every other honest limit.
- ⚠️ A new `rclpy`-holding bridge process is a new component with its own failure modes (dropped
  MQTT messages, a stalled Gazebo world, a lidar plugin misconfigured relative to
  `SensorParams`) that `SimLite` simply does not have. None of this is covered by the existing
  `packages/parkscan_sim` test suite, and needs its own.
- ⚠️ This ADR does not resolve, and does not attempt to resolve, whether the lidar/camera sensor
  plugins available in `ros_gz_sim` for Jazzy actually support the exact ray count/FOV
  `SensorParams` specifies, or whether Gazebo's spawn/reset semantics behave exactly as the
  world-control service's documentation implies under `rmw_zenoh_cpp` specifically (vs. the
  default DDS the upstream docs are usually written against). Both are implementation-time
  discoveries, not design decisions — flagged here so they are looked for deliberately rather
  than found by surprise, the way the RTF spike's own command had two undocumented bugs
  (missing `setup.bash` sourcing, `-v 4` not actually reporting RTF) that only surfaced by
  running it.

## What this ADR does not do

It does not write the `.sdf` file, the bridge node, `Ros2WorldBackend`, the `ros_gz_bridge`
YAML, the Zenoh router config, or the Compose service. It is the design those should be built
against — matching the project's own pattern of a design review preceding implementation for
every phase complex enough to warrant one.

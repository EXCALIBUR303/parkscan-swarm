# ADR-0008 — `Ros2WorldBackend`: namespace scheme, lockstep timing, and the MQTT bridge

**Status:** Accepted and implemented. `Ros2WorldBackend`, the bridge node's MQTT relay, the SDF
generator, `GzControl`, the `ros2-gz` Dockerfile and compose service, and runner `--backend`
selection all exist and were run for real, end to end, through `apps/runner`'s own CLI against
the actual built `docker compose --profile ros2` stack — the coordinator assigned zones, every
robot began navigating, telemetry published — with zero code changes to `Coordinator`,
`RobotAgent`, or the allocator. Confirmed reliable for realistic, moderate-length runs (100
steps, clean). **Not fully reliable yet: a longer run (600 steps) failed** on a `gz-transport`
`Host unreachable` error, traced to the per-`step()`-call subprocess overhead in `GzControl` —
see the sixth finding below for what that means and why fixing it is a larger, deferred change.
**Also not done: `rmw_zenoh_cpp` is not configured** (verification used the base image's default
RMW; see the scope note below for why this specific design may not even need it) ·
**Extends:** ADR-0001, ADR-0002 · **Gated by:** the RTF spike in `docs/ROADMAP.md` (passed,
RTF ≈ 1.0)

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
carries three of the four topics `parkscan_core/world.py` names: `/<ns>/odom`
(`nav_msgs/Odometry`), `/<ns>/scan` (`sensor_msgs/LaserScan`), `/<ns>/cmd_vel`
(`geometry_msgs/Twist`). `/<ns>/camera/image_raw` is not wired at all, since no camera sensor is
in the SDF (Scope, below). The three that exist are wired via `ros_gz_bridge`'s
`parameter_bridge` — **not** my own rclpy bridge (§6). Two different bridges doing two different
jobs: this one translates Gazebo's own transport into genuine ROS 2 topics; mine (below) relays
those ROS 2 topics on to MQTT, since `rclpy` cannot run on the host. Verified directly, one topic
per direction:
`/robot_1/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry` (`[` = Gazebo → ROS) and
`/robot_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist` (`]` = ROS → Gazebo), repeated per
robot; confirmed odometry and lidar data actually flowing through to `ros2 topic echo` this way,
not just that the bridge process starts without error.

The lidar sensor plugin in the SDF **must** be configured for 37 rays over a 1.5π field of view
capped at 12.0 m — `parkscan_sim.sensors.SensorParams`'s exact defaults. `SensorFrame.ranges` is
documented as a *"fixed-count planar scan"*; nothing downstream tolerates a different count.

### 2. Coordinate frame: identity, not a transform

The SDF's world origin is `lot.yaml`'s `(0, 0)` with the ground plane in the X/Y plane, Z up.
Gazebo pose `(x, y, yaw)` in metres maps directly onto `Pose2D(x_m=x, y_m=y, theta_rad=yaw)` —
no rotation, no offset, no unit conversion. This is a deliberate simplicity choice, not an
oversight: introducing a transform here would be a second, silent place reproducibility could
drift between backends, on top of the reproducibility gap §4 already states plainly.

### 3. Timing: lockstep via the world control service — called by the bridge, relayed over MQTT

Gazebo is launched **paused** (no `-r`), with a fixed physics `step_size` in the SDF matching
`SimConfig.dt_s = 0.05` (e.g. `step_size: 0.001` with 50 iterations per call, or `step_size:
0.05` with 1 — either is fine as long as it is exact, not approximated). The world's
`/world/<name>/control` service (confirmed live and serving in the RTF spike's own log output —
`"Serving world controls on [/world/shapes/control]..."`, and notably printed by `gz sim` itself
with **zero ROS 2 nodes running** in that spike) supports a `multi_step` request to advance by a
specific iteration count while paused, and a reset field to snap physics and poses back to the
SDF's initial state.

**Correction to an earlier draft of this ADR, caught before implementation started:** that
service is native **Gazebo Transport** (`gz-transport`), not a ROS 2 service — a completely
separate middleware from `rmw`/Zenoh, with its own discovery and its own (undocumented, in this
repo) story for crossing the Docker Desktop VM boundary. It has exactly the same
host-unreachability problem §"Context" already established for `rclpy`: nothing on this Mac can
call it directly, whether or not `rclpy` is involved. So the bridge — the one process already
living inside the container with native `gz-transport` access — must own every `multi_step` and
reset call locally, and relay the result to `Ros2WorldBackend` over the same MQTT channel
already carrying `state`/`cmd`, rather than the backend calling the control service itself as an
earlier draft of this section assumed.

Two more request/response topics, added to §6's table: `.../control/step` (backend → bridge,
`{"request_id", "iterations"}`) and `.../control/step_ack` (bridge → backend, published only
once the bridge's local `multi_step` call has actually returned — i.e., once Gazebo has
genuinely advanced), plus the reset equivalents. `Ros2WorldBackend.step(dt)` publishes a step
request and **blocks on receiving the matching `step_ack`** (by `request_id`, guarding against a
stale or duplicate ack) before returning — synchronous and wall-clock-independent by
construction, exactly as intended, just relayed through one more hop than originally drafted.
`request_id` exists because MQTT is pub/sub, not RPC: without it there is no way to tell "the
ack for the step I just asked for" apart from "an ack for some earlier request," particularly
across a `reset()`.

These two topics use **QoS 1**, deliberately different from `state`/`cmd`'s QoS 0 (§6): a lost
step or reset ack would hang `Ros2WorldBackend` in a blocking wait forever, where a lost `state`
sample is just superseded by the next one a few milliseconds later.

**Confirmed empirically, before writing the bridge — and the answer to the question this
section originally left open.** Two facts, verified directly against a running `gz sim` inside
the container, neither of which is documented anywhere obvious:

1. `multi_step` alone does **not** step-then-repause. Sent by itself, the world advances a few
   iterations and then keeps running freely (observed: 474 iterations and 23.7 s of sim time,
   nearly a real *minute*'s worth, after a single `multi_step: 20` call with nothing else) —
   it behaves as an unpause, not a bounded step. The request must carry **both**
   `multi_step: N` *and* `pause: true` in the same `WorldControl` message. With both set, the
   world executes exactly `N` iterations and then genuinely returns to `paused: true` —
   verified by polling `stats` after the fact and seeing `iterations` land exactly on `N`.
2. The service's `gz.msgs.Boolean{data: true}` reply is an **accept, not a completion
   signal**. It returns almost immediately; the requested iterations are still being applied
   afterward, taking roughly `N * step_size` of real wall-clock time to actually finish
   (consistent with RTF ≈ 1.0). A bridge that treats the reply alone as "the step is done" and
   immediately reports `step_ack` would be racing ahead of physics.

**This changes the bridge's implementation, not the design's shape:** §6's `control/step`
handler must call `/world/<name>/control` with `{multi_step: N, pause: true}`, then poll (or
subscribe to) `/world/<name>/stats` until `iterations` has advanced by exactly `N` from where it
started, and only then publish `step_ack`. The MQTT-relay design and the QoS-1 choice (a lost
ack hangs forever) are unaffected — this just says precisely what "the bridge's local call has
actually returned" (§6's wording) must mean in code. Found by running it, the same way the RTF
spike's own two undocumented bugs were found — not by reading Gazebo's docs, which say none of
this.

### 4. Battery: reuse the pure proxy, own it in `Ros2WorldBackend`, propose relocating it

`parkscan_sim.battery`'s functions (`drain_soc`, `charge_soc`, `discharge_power_w`,
`BatteryParams`) take `(soc, v_mps, dt, params)` — no `SimLite` state, no world reference. They
are already exactly as backend-agnostic as `parkscan_core.lot`/`parkscan_core.types`.
**Done, not just recommended:** `battery.py` has been relocated from `parkscan_sim` to
`parkscan_core`, re-exported from `parkscan_sim` for the existing import sites — `parkscan_ros2`
depends only on `parkscan_core`, never on a peer backend.

`Ros2WorldBackend` (not the bridge) owns per-robot SOC as a plain Python float, updated every
`step(dt)` from the *measured* twist Gazebo reports back (the real, physics-simulated velocity —
matching `SimLite`'s own "measured twist, not commanded" discipline) and from proximity to a
charging bay (same `lot.charging_bays` positions and the same `charge_dock_radius_m` semantics
`SimLite` uses, so "docked" means the same thing on both backends). The bridge, by contrast,
relays pose/twist/ranges only — it has no battery concept and needs none.

**A third empirical finding, again confirmed against a real bridge before this line was
written:** `gz-sim-odometry-publisher-system`'s velocity estimate is unreliable for roughly the
first ten samples after the world starts or resets. Watched directly: ten consecutive real
odometry messages from a robot that had never been commanded to move and never left its spawn
position reported `linear.x/y` of `160, 80, 53.3, 40, 32, 26.7, 22.9, 20, 17.8, 16` — an exact
`160/n` decay — before correctly settling at `0.0` from the eleventh sample on. The pattern is
diagnostic of a finite-difference estimator dividing some tiny startup delta by elapsed time
since `t=0`; it is a real, reproducible warm-up artifact, not sensor noise to average away.
**Consequence for `Ros2WorldBackend`: clamp the reported `v_mps` to `[0, NavConfig.max_v_mps]`
before ever passing it to `discharge_power_w`.** A fleet that never legitimately exceeds 1.2 m/s
has no honest reading above that bound to trust, and an unclamped spike would otherwise drain a
fresh reset's battery by a real, if small, spurious amount (a single 160 m/s tick alone costs
~0.6% of a 20 Wh pack — `discharge_power_w(160, BatteryParams()) * 0.05 / 3600 / 20`). This is a
clamp on an already-known-bogus reading, not a general noise filter, and should not be read as
license to distrust the sensor more broadly than this one measured failure mode. **Reproduced
again, independently, once the full pipeline was wired end to end:** a live run through the real
bridge (Gazebo → `ros_gz_bridge` → this node → MQTT, no fakes anywhere) showed the exact same
signature — three robots reporting `160.0`, `160.0`, `50.0` m/s immediately after their first
`multi_step` — confirming this is a structural property of the plugin, not an artifact of the
narrower probe that first found it.

**A fourth empirical finding, found closing out this same end-to-end pass:**
`gz-sim-velocity-control-system`'s commanded velocity is measurably damped by DART's default
ground-contact friction. Commanded `0.8` m/s on `/robot_1/cmd_vel`, then measured the robot's
*actual* reported velocity a full 10 real seconds later (steady-state, confirmed flat across 200
further physics iterations, not a transient still settling): `~0.309` m/s — a real, repeatable
~61% loss. Setting `mu`/`mu2` to `0` on the robot's collision surface (`parkscan_ros2.sdf`)
restored tracking to `0.7999999999999983`, float precision of commanded. `SimLite` has no
friction model to lose motion against in the first place, so leaving Gazebo's default friction in
place would have made the two backends silently non-comparable — a robot told to drive at
`max_v_mps` would actually travel measurably slower under `ros2-gz` than under `sim-lite`, with
no error or warning anywhere. Zero friction is now the SDF's default for every robot.

**A fifth empirical finding, found only by actually running `apps/runner --backend ros2-gz`
end to end** — not by any of the piece-by-piece verification above, which is exactly why doing
this final integration run mattered: `reset: {all: true}` costs `ros_gz_bridge` noticeably more
than one physics iteration before it relays a fresh sample again. Verified with `rclpy` entirely
out of the picture — `gz service ... reset` then `gz service ... multi_step: 1` then `ros2 topic
echo /robot_1/odom --once` — the echo timed out. It is not permanently broken: enough further
stepping eventually produces a sample again, but "enough" is not a fixed, documented number.
`_handle_reset` now steps in 1-second chunks (`_RESET_WARMUP_CHUNK_ITERATIONS`), checking after
each chunk whether every robot has actually landed in this node's own cached state, up to a
bounded ceiling (`_RESET_WARMUP_MAX_CHUNKS` — 15 s of sim time) before giving up and never
acknowledging, rather than guessing a fixed warm-up count that might not hold on a different
host or lot. `Ros2WorldBackend.reset()`'s own wait for every robot's first fresh state (already
in the design, §3's ack-is-not-completion principle) is what makes this safe on the backend side
even before this fix: it never trusted the reset ack alone.

This is also the run that closed the design's own last open item —
**"Verify: identical coordinator/perception code runs unmodified on both backends"**
(`TASKS.md`, `docs/ROADMAP.md`) — for real: `apps/runner`'s actual CLI, talking to the actual
built `ros2-gz` Docker Compose service, with zero code changes to `Coordinator`, `RobotAgent`, or
the allocator. The coordinator assigned all three zones, every robot transitioned to
`RobotState.TRANSIT` and began navigating toward its assignment, and 150 telemetry messages
published cleanly over 100 steps with no errors on either side of the MQTT connection.

**A sixth empirical finding, and the one place this backend is not yet as reliable as
`SimLite` for anything beyond a moderate run:** a longer run (600 steps) failed partway through
with the world control service refusing a step request — `gz`'s own transport layer logged
`NodeShared::RecvSrvRequest() error sending response: Host unreachable` right before it. §6
already shells out to the `gz service` CLI once **per `step()` call**, each invocation spawning a
brand-new process that has to freshly complete `gz-transport`'s own discovery handshake before it
can even send its request; over enough consecutive calls in one run, that per-call overhead
appears to degrade into outright failures rather than just added latency. The 100-step run above
completed cleanly; the 600-step run did not, failing somewhere in between — the exact threshold
was not pinned down, and root-causing it further (accumulated file descriptors or ephemeral
ports inside the container's network namespace is one plausible mechanism, not a confirmed one)
would mean either a persistent `gz-transport` connection (its Python bindings are not installed
in this environment — the same gap §6 already named) or a different control channel entirely,
both larger changes than this pass's scope. **One real bug this did surface and fix regardless:**
`subprocess.run`'s own `timeout` argument raises `TimeoutExpired`, which `check=False` does not
suppress — that exception was escaping `GzControl`'s default runner uncaught and crashing the
bridge's MQTT callback outright. Fixed to convert a subprocess timeout into the same
`(non-zero code, message)` shape a refused service call already produces, so the *existing*
`GzControlError`-and-no-ack handling covers it too — confirmed by the second run failing
gracefully (a clean error log, node still alive) rather than crashing the process the way the
first run did. Stated plainly: this backend is verified working for realistic, moderate-length
runs; sustained long runs need either a higher tolerance for occasional step failures or the
architectural change above before they can be trusted the way `SimLite` already is.

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
| `parkscan/bridge/v1/control/step` | backend → bridge | `{"request_id", "iterations"}` | 1 |
| `parkscan/bridge/v1/control/step_ack` | bridge → backend | `{"request_id"}` — published only once the bridge's local `multi_step` call returns | 1 |
| `parkscan/bridge/v1/control/reset` | backend → bridge | `{"request_id", "seed"}` | 1 |
| `parkscan/bridge/v1/control/reset_ack` | bridge → backend | `{"request_id"}` | 1 |

`state`/`cmd` use QoS 0: this is a live-state relay sampled every physics step — a dropped or
superseded message is simply stale data waiting to be overwritten by the next one, not an event
whose loss matters (unlike a detection or a lease grant). `battery_soc` is never on this
channel — §4 already assigns that field to `Ros2WorldBackend` alone, so the bridge does not need
to know battery parameters at all. `control/*` uses QoS 1, per §3: a lost step or reset ack
hangs a blocking wait forever, unlike a superseded state sample.

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

## A scope note on `rmw_zenoh_cpp`, found while verifying the rest

All verification in this ADR (the bridge, the odometry warm-up quirk, the `multi_step` pairing)
was run via `docker exec` **inside a single container**, on whatever RMW ships default with
`ros:jazzy-ros-base` — `rmw_zenoh_cpp` was never installed, configured, or exercised. This
matters beyond "one more thing not yet tested": on reflection, the MQTT-relay design this whole
ADR describes may make ADR-0002's zenoh requirement **moot for this specific architecture**.
Zenoh-over-DDS-multicast was chosen because *something* needed to carry ROS 2 traffic across the
Docker Desktop VM boundary. But my own bridge (§6) exists precisely so that nothing does: the
rclpy bridge and `ros_gz_bridge` both live inside the same container, talking ROS 2 to each
other over a plain same-network-namespace connection no different from any other same-host ROS 2
setup, and everything that crosses the actual host/container boundary goes over MQTT, already
proven to cross it. Whichever RMW the container's ROS 2 nodes use for *purely internal*
same-container topics should not need zenoh's cross-boundary router at all.

This is not a reason to skip it. TASKS.md lists zenoh as a Phase S goal in its own right — proving
out ADR-0002's actual decision, not merely a means to an end — and this project's own principle
(stated plainly elsewhere: *"an adapter with one implementation is an unproven claim"*) argues
for actually configuring and exercising it rather than declaring it unnecessary from first
principles and never checking. Flagged here so whoever implements this either (a) wires zenoh
anyway and confirms it works exactly as ADR-0002 specified, or (b) makes a deliberate, stated
call to skip it because this design doesn't need it — never lets "verified against a real
Gazebo" quietly stand in for "verified with the RMW this project actually chose."

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
- ✅ Both things this bullet originally flagged as open have since been verified directly
  against a running `gz sim` in the container, before the bridge was written: the SDF's
  `gpu_lidar` sensor plugin (37 rays, 1.5π FOV, 12 m range — `SensorParams`'s exact defaults)
  loads and advertises cleanly per robot with zero errors, and `multi_step`'s true semantics
  are now pinned down in §3 (must pair with `pause: true`; the reply is an accept, not
  completion). Neither was documented anywhere; both were found by loading a real SDF and
  calling the real service, not by reading Gazebo's docs, the way the RTF spike's own two
  undocumented bugs (missing `setup.bash` sourcing, `-v 4` not actually reporting RTF) were
  found only by running it.

## What was actually built, and how it was verified

Updated after the fact, three times now: verifying this design against a real Gazebo turned into
writing real, tested code, not just checking assumptions; that code turned into a full
implementation; and the full implementation was then run for real, end to end, through
`apps/runner`'s own CLI against the actual built `docker compose --profile ros2` stack — not
stopped at "each piece works in isolation." Built, tested, and committed:

- `parkscan_ros2/sdf.py` — the world generator (unit-tested; the actual generated SDF loaded
  cleanly in a real Gazebo Sim Server v8.11.0, all three robots' plugins working, zero friction on
  every collision surface per the fourth finding below).
- `parkscan_ros2/bridge/gz_control.py` — the Gazebo lockstep control wrapper (unit-tested,
  including the subprocess-timeout-to-`GzControlError` conversion the sixth finding below needed;
  the real `multi_step`/`pause`/`reset` semantics it encodes were separately confirmed against a
  live `gz sim`).
- `parkscan_ros2/bridge/protocol.py` — the MQTT wire format (unit-tested standalone; confirmed
  byte-for-byte against real messages published by the real bridge over a real broker).
- `parkscan_ros2/bridge/geometry.py` — the quaternion-to-yaw conversion (unit-tested).
- `parkscan_ros2/bridge/node.py` — the rclpy bridge node. Cannot be unit-tested (no `rclpy` on
  this host — excluded from this project's mypy run for that exact reason) but was run for real,
  repeatedly, through the full `docker compose --profile ros2` stack: a `control/step` MQTT
  request produces a matching `step_ack` and fresh `state` messages for all three robots with
  correct pose and exactly 37 lidar ranges each; a `cmd` MQTT message reaches the robot and moves
  it; `_handle_reset`'s chunked warm-up (the fifth finding below) reliably produces fresh state
  after a reset before acknowledging it.
- `parkscan_ros2/backend.py` — `Ros2WorldBackend` (unit-tested against a fake, dependency-injected
  MQTT client covering the exact scenarios this ADR's findings describe — the velocity clamp, the
  docked-vs-draining branch, the ack race found and fixed the moment tests ran synchronously; then
  run for real, unmocked, as `apps/runner`'s actual backend against the actual compose stack).
- `infra/ros2/Dockerfile`, `infra/ros2/entrypoint.sh`, and the `docker-compose.yml` `ros2-gz`
  service (`--profile ros2`, additive only) — built, and the actual committed files were the ones
  built and run (`docker compose --profile ros2 build && up`), not a close approximation of them.
  The entrypoint's own `set -u` conflicted with `/opt/ros/jazzy/setup.bash` itself referencing an
  unbound variable — found by running the real container, fixed to `set -eo pipefail`.
- `apps/runner`'s `--backend {sim-lite,ros2-gz}` flag (ADR-0008 §7) — run for real, not just
  wired: `python -m parkscan_runner.run --backend ros2-gz` against the live stack assigned all
  three zones and moved all three robots with zero coordinator/agent code changes.

**Not done:** `rmw_zenoh_cpp` router configuration (the scope note above), and reliability on long
sustained runs (the sixth finding below — confirmed clean at 100 steps, failed at 600). Six
genuine, previously-undocumented bugs were found and fixed by actually running this system, not
by reading any manual: the per-robot topic-scoping collision, the `multi_step`/`pause` pairing
requirement, the odometry velocity warm-up artifact, DART's default friction damping commanded
velocity, the reset warm-up needing several chunks rather than one iteration, and
`subprocess.run`'s `timeout` argument raising uncaught past `check=False`.

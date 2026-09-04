# ParkScan Swarm — Architecture

**Status:** Phase 0 (design). No implementation yet.
**Date:** 2026-09-03. **Target machine:** Apple M5, macOS 27, arm64, 16 GB.
Every decision below traces to [`RESEARCH_LOG.md`](RESEARCH_LOG.md).

---

## 0. The one constraint that shapes everything

Three verified facts collide:

1. **ROS 2 does not support macOS on arm64 at any tier.** Native install is off the table.
2. **Docker Desktop on macOS gives containers no GPU.** A containerised Gazebo renders every
   camera through Mesa `llvmpipe` on the CPU. Gazebo's own docs recommend against this exact case.
3. **Perception cannot be accelerated in a container either** — PyTorch MPS and ONNX Runtime's
   CoreML EP are unreachable from a Linux guest. The detector must run natively on the host.

So the simulator wants to live in a Linux container, the detector must live on the macOS host, and
**DDS cannot reliably cross that boundary** (Docker Desktop host networking is layer-4 only). The
system is *forced* to have a process boundary with a plain-TCP bridge across it, no matter which
simulator we choose.

That is not a compromise to apologise for. It is the actual shape of the problem, and it dictates
the architecture: **put every piece of intellectual content behind a narrow, well-typed boundary,
and make the simulator a swappable backend.**

---

## 1. Decision: two simulation backends, one adapter

| | `sim-lite` **(default, MVP)** | `ros2-gz` *(optional, Phase 4+)* |
|---|---|---|
| What | Deterministic 2D kinematic simulator, pure Python | ROS 2 Jazzy + Gazebo Harmonic, headless, `linux/arm64` containers |
| Startup | < 60 s, no Docker | Docker Desktop + ~4 GB image pull |
| Speed | ~100× real time (headless, no rendering) | RTF likely 0.2–0.5 on llvmpipe |
| Determinism | Seeded, bit-reproducible, replayable | Not reproducible run-to-run |
| Cameras | Synthetic frames from ground-truth geometry | Real rendered frames at ~160×120 @ 2–5 Hz |
| Purpose | The demo, the tests, and the **30-seed metrics harness** | Proves the abstraction is real, not theoretical |

**Why `sim-lite` is the default and not a cop-out:** the single biggest credibility differentiator
in this project is `make bench` — 30 seeded runs × 3 allocation strategies, with paired bootstrap
confidence intervals, regenerating every number in the README from a committed CSV. At an RTF of
0.3 with software-rendered cameras, that experiment **cannot be run**. The fast deterministic
backend is what turns a demo into an experiment.

**Why `ros2-gz` still gets built:** an adapter with one implementation is an unproven claim. The
second backend is what makes the boundary honest. It is explicitly scoped as a stretch goal, and
the README will say so plainly rather than implying a ROS 2 system that was never run.

> **Honesty rule.** Until `ros2-gz` actually runs, the README says *"runs on a purpose-built
> deterministic simulator; a ROS 2 / Gazebo backend is in progress behind the same interface."*
> It never says "ROS 2 system". No screenshot of `sim-lite` is ever presented as a physics sim.

### The boundary (concrete, not generic)

The classic failure of "pluggable backends" is an abstraction so generic it carries no meaning.
This one is deliberately narrow and opinionated: **2D, metric, REP-103 axes (x forward, y left,
θ CCW), single flat ground plane, one tick = one fixed dt.**

```python
# parkscan_core/world.py
class WorldBackend(Protocol):
    def reset(self, seed: int, lot: LotConfig) -> None: ...
    def step(self, dt: float) -> None: ...                    # advance one tick
    def now_ns(self) -> int: ...                              # sim clock, not wall clock
    def robots(self) -> Sequence[RobotId]: ...
    def sense(self, robot: RobotId) -> SensorFrame: ...
    def actuate(self, robot: RobotId, cmd: TwistCmd) -> None: ...
```

```python
@dataclass(frozen=True, slots=True)
class SensorFrame:
    stamp_ns:  int
    pose:      Pose2D          # x_m, y_m, theta_rad  (REP-103)
    twist:     TwistCmd        # measured, not commanded
    battery:   float           # state of charge, 0.0-1.0
    ranges:    tuple[float, ...]   # fixed-count planar ranges, metres, inf = no return
    camera:    Frame | None    # None when the camera did not fire this tick
```

Everything above the adapter — the task state machine, the coordinator, the allocator, the
reservation manager, perception, geolocation, telemetry — imports **only** these types and never
knows which backend it is talking to. `sim-lite` implements `WorldBackend` directly. The ROS 2
adapter implements it as a thin node: subscribe `/<ns>/odom`, `/<ns>/scan`,
`/<ns>/camera/image_raw`; publish `/<ns>/cmd_vel`. **No autonomy logic lives inside ROS.**

---

## 2. Component map

```
                       macOS host (native, Apple Silicon)
  ┌──────────────────────────────────────────────────────────────────────┐
  │  parkscan-sim  (uv venv, Python 3.12)                                │
  │    WorldBackend = sim-lite        ← default                          │
  │    robot_1 robot_2 robot_3  · task state machine · local avoidance    │
  │    perception: Detector (SimulatedDetector | RfDetrDetector/MPS)      │
  │    geolocation: flat-ground back-projection                          │
  │  parkscan-coord  · zone allocation · leases+fencing · failure recovery│
  └───────────────────────────────┬──────────────────────────────────────┘
                                  │ MQTT (plain TCP, 1883)
  ┌───────────────────────────────▼──────────────────────────────────────┐
  │  mosquitto 2.1.2      (docker compose, or brew)                      │
  └───────────────────────────────┬──────────────────────────────────────┘
                                  │
  ┌───────────────────────────────▼──────────────────────────────────────┐
  │  FastAPI  (uvicorn, single worker)                                   │
  │    aiomqtt subscriber → validate → persist → fan out                 │
  │    REST /api/v1/*   ·   WebSocket /ws/live                           │
  └───────────────────────────────┬──────────────────────────────────────┘
                                  │ asyncpg
  ┌───────────────────────────────▼──────────────────────────────────────┐
  │  PostgreSQL 17 + PostGIS 3.5   (imresamu/postgis, host port 5433)    │
  └──────────────────────────────────────────────────────────────────────┘
                                  │ WebSocket
  ┌───────────────────────────────▼──────────────────────────────────────┐
  │  React 19 + TS 6 + Vite 8 + MapLibre GL 6      (npm, not pnpm)       │
  └──────────────────────────────────────────────────────────────────────┘

  Optional (Phase 4+):  docker compose --profile ros2
    ros:jazzy-ros-base (linux/arm64) + ros-jazzy-ros-gz + rmw_zenoh_cpp
    headless `gz sim -s -r --headless-rendering`, ogre2, one world, 3 namespaces
    → bridges to the same MQTT topics; nothing above the adapter changes.
```

---

## 3. Autonomy layer

**Robot state machine** (explicit enum, table-driven transitions, tested over *all* (state, event)
pairs including illegal ones):

`IDLE → ASSIGNED → TRANSIT → INSPECTING → RETURNING → CHARGING`, plus `WAITING_ON_LEASE`
and terminal `FAILED` / `OFFLINE`.

The coordinator keeps a **separate** judgement of each robot — `ONLINE` / `SUSPECT` / `OFFLINE` /
`STUCK` — deliberately not merged with the robot's self-reported state. That separation is the
point: a robot that believes it is `INSPECTING` while the coordinator has declared it `OFFLINE` is
exactly the case the recovery logic must handle.

**Navigation:** lane-graph shortest path between waypoints (Manhattan over the lane graph, *not*
Euclidean — a parking lot has lanes), with a reactive local avoidance layer (VFH-style steering
from the planar ranges) that can only slow or deflect, never re-plan.

**Return-to-charge trigger** is a *feasibility check*, not a fixed percentage:

```
soc < k · d_travel(pos, nearest_bay(pos)) · (1 + margin) + soc_reserve
```

evaluated **continuously during a sweep**, not only at zone boundaries — otherwise a robot commits
to a long zone and strands. A fixed 20% threshold is wasteful near the bays and fatal far from them.

---

## 4. Coordinator

**Honest classification.** Assigning 12 zones to 3 robots is **ST-SR-TA** (Gerkey & Matarić) —
scheduling class `R ∥ Σ w_j C_j`, strongly NP-hard. Because robots contend for shared lanes and
bays, utilities are not independent, so the iTax label is **XD [ST-SR-TA]** (Korsah et al. 2013).
The coordinator *deliberately decouples* that cross-schedule dependency into a separate reservation
layer, which is what lets the allocator stay ST-SR-IA-shaped. We implement **iterated ST-SR-IA**:
re-solve an instantaneous assignment whenever a robot frees up.

**Three interchangeable strategies over one shared cost function:**

| Strategy | What | Why it's there |
|---|---|---|
| `greedy` (default) | argmin over feasible (robot, zone) pairs, repeat | Simple, explainable, the baseline |
| `hungarian` | `scipy.optimize.linear_sum_assignment` (handles 3×12 rectangular) | **Optimal per-round reference.** Proves greedy's gap instead of asserting it |
| `ssi` | Sequential single-item auction, marginal-cost bids (BidSumPath) | ≤2× optimal on MiniSum — a real theorem, ~100 LOC reusing the same cost fn |

Cost: `c(i,z) = d_lane(pos_i, entry(z)) + sweep_len(z)`, with a battery-feasibility predicate
including the return leg to the nearest bay. `allocate(state) -> list[Assignment]` is a **pure
function** — no I/O — so the whole allocator is unit-testable without a simulator.

**CBBA is explicitly rejected**, and that rejection is a documented ADR rather than a silence: it
is 394–920 LOC of consensus logic solving *inconsistent situational awareness over a time-varying
comms graph*, and this system has one MQTT broker and a central coordinator. Its premise does not
hold. Implementing it and then claiming decentralised allocation would be an overclaim.

### Reservations — leases + fencing + ordering

Shared resources: single-occupancy lane segments and charging bays. **Bays are deliberately
scarcer than robots (2 bays, 3 robots)** so contention is real rather than fabricated.

- **Lease** (Gray & Cheriton, SOSP 1989): `acquire → (granted, token, expires_at)`, TTL ≈ 3× expected
  hold, renewed by the holder, reclaimed unilaterally on expiry. A crashed holder cannot block forever.
- **Fencing token** (Kleppmann): a stalled robot can wake *after* its lease expired still believing
  it holds the bay. Every action carries its token; the resource records the highest token ever
  accepted and **rejects anything lower**. ~15 lines, directly testable. This is the same mechanism
  VDA 5050 uses in industry via `orderId` / `orderUpdateId`.
- **Deadlock** is broken on two independent Coffman conditions. ⚠️ **Superseded by
  [ADR-0007](adr/0007-coordinator-concurrency.md):** the circular-wait half below does not survive the
  lane graph — routes acquire segments in traversal order, which is not id order, so the ordering rule
  would outlaw ordinary routes. Phase 4 breaks **hold-and-wait** instead, via atomic per-leg
  acquisition. The reasoning is kept here because why it fails is worth more than a tidier document.
  ~~**circular wait** by total resource ordering (never request an id lower than the highest currently
  held), and~~ **no-preemption** by lease expiry — which is retained, because neither ordering nor
  atomic acquisition does anything about a dead robot holding a bay.
- **Starvation** is prevented by a FIFO queue per resource with aging
  (`effective_priority = base + α · wait_time`), so a critically-low robot can jump the bay queue
  without unbounded waits for anyone else.

### Failure detection — three independent mechanisms

1. **MQTT Last Will** on hard disconnect — the broker does it, zero polling. (Must be a *real*
   broker; faking LWT at application level would void the claim.)
2. **Heartbeat deadline** — `OFFLINE` after k=3 missed 1 Hz beats. Catches the *hung* robot that
   holds its TCP session but stops progressing, which LWT will never fire for. Both are needed.
3. **Stuck detection** — over a 10 s window: displacement < ε **and** covered-cell count did not
   increase **and** state is not `WAITING_ON_LEASE`. That last exclusion is essential; without it
   the reservation system manufactures false stuck-detections every time a robot legitimately queues.

### Recovery hazards, each with a named prevention

| Hazard | Prevention |
|---|---|
| **Double-assignment** (revived robot + its replacement both work zone Z) | Per-zone monotonic **assignment epoch**; progress reports accepted only if `report.epoch == current`. Same fencing mechanism as the bay lease — one implementation, both uses |
| **Lost work** (half-swept zone discarded) | Reporting granularity finer than assignment granularity: robots report covered *cell ids*, so requeue is "zone Z, remaining cells {…}" |
| **Thrashing / oscillation** | Four stacked guards: hysteresis (`new < current − δ`), per-zone reassignment cooldown, **event-triggered** re-solve only (never a periodic loop), and a hard cap after which a zone is flagged `UNRECOVERABLE` on the dashboard rather than retried forever |
| **Lazarus** (robot declared dead returns) | Mandatory re-registration: fresh epoch + explicit abandon-current-order before it may act. Never a silent resume |

---

## 5. Perception

Everything sits behind one `Detector` Protocol, with three implementations sharing one contract test:

```python
@runtime_checkable
class Detector(Protocol):
    name:    str
    source:  DetSource            # SIMULATED | MODEL | GROUND_TRUTH
    classes: frozenset[DetClass]
    def warmup(self) -> None: ...
    def detect(self, frame: Frame, *, score_threshold: float = 0.25) -> Sequence[Detection]: ...
```

Three design decisions carry the whole thing:

1. **`detect()` is pure w.r.t. the frame.** All randomness lives in the constructor seed, derived
   per-frame from `hash(seed, frame_id)`, never a global RNG. That is what makes the simulated
   detector reproducible *and* makes offline evaluation meaningful for both stub and real model.
2. **The detector never receives robot pose and never emits world coordinates.** `Frame` carries
   only image / frame_id / stamp / camera_id. Geolocation is a separate, independently testable
   stage — a detector that knew the map could cheat.
3. **Every `Detection` carries a `source` enum that is persisted to Postgres** and drives the
   SIMULATED badge in the UI. This is the structural honesty guarantee, not a cosmetic label.

`assert_detector_contract(det, frame)` ships as a pytest every implementation must pass: determinism
across two calls, emitted classes are declared, scores respect the threshold, bboxes lie inside frame
bounds, `source` matches, and **raising the threshold never adds detections**.

**Real model: RF-DETR Nano (Apache-2.0)**, measured at **31.1 ms / frame** on this Mac.
**Ultralytics YOLO is banned** — it is AGPL-3.0, and because this project serves a web API,
§13 would force the *entire repo* under AGPL. A CI guard (`pip freeze | grep -Ei 'ultralytics|rfdetr_plus'`)
makes the trap impossible to reintroduce via a dependency bump.

### Geolocation

Flat-ground monocular back-projection from the bbox **bottom-centre** (the ground contact point):

```
d_opt   = [(u−cx)/fx, (v−cy)/fy, 1]
d_w     = Rz(θ) · Ry(−pitch) · R_c_opt · d_opt
reject if d_w[z] >= 0                       # at/above horizon — reject, never clamp
t       = −C_w[z] / d_w[z]
P_world = C_w + t · d_w
```

Verified numerically: round-trip error **0.000000000 m**. Clamping a near-horizon ray instead of
rejecting it produces a plausible-looking point hundreds of metres away that silently poisons the map.

**Error grows quadratically with range** — a 1° pitch error costs 0.028 m at 0.8 m but **5.24 m at
13.1 m**. Detections beyond ~6 m ground range are rejected, and the documented error ranking is:
pitch/attitude (dominant) → flat-ground assumption (a pothole is by definition *below* the plane) →
bbox bottom-edge localisation → pose drift → camera height → lens distortion → time sync.

---

## 6. Backend

- **FastAPI 0.141 / Pydantic 2.13 / SQLAlchemy 2.0.52 `[asyncio]` / asyncpg / GeoAlchemy2 / Alembic.**
  The `[asyncio]` extra is **mandatory** — bare `sqlalchemy` silently omits greenlet on macOS arm64.
- **`aiomqtt` 2.5.1**, wired through the modern `@asynccontextmanager lifespan` (never `@app.on_event`,
  which Starlette 1.6 removed). Pinned to 2.x — 3.0.0a1 is an alpha that swaps the protocol library.
- **WebSocket fan-out:** one in-process `ConnectionManager`; one MQTT subscription in lifespan feeding
  a **bounded** (maxsize 100, drop-oldest) `asyncio.Queue` per browser, each send in its own task with
  a timeout so one stalled tab cannot block the others. **No Redis** — at 3 robots that is architecture
  theatre. Single uvicorn worker, stated in the README as a deliberate MVP boundary.
- **The browser never talks to MQTT directly**, even though mosquitto's websockets listener would allow
  it. Bridging through FastAPI gives one place to validate, enrich, and stamp the simulated flag.
- **Geospatial:** store both. `local_x, local_y double precision` (the sim frame, source of truth) *and*
  `geog geography(Point,4326)` GIST-indexed for proximity. `geography`, not `geometry` — `ST_Distance`
  on `geometry(4326)` returns **degrees**. The lot is anchored at a plausible synthetic origin.
- **`data_source` is a NOT NULL enum on every telemetry and detection row.** Honesty is enforced by the
  schema, not by remembering to render a disclaimer.

---

## 7. Dashboard

**MapLibre GL 6 + react-map-gl 8** over Leaflet: Leaflet's stable release is from May 2023, its 2.0 has
been in alpha since May 2025 with an officially "unknown" release date, `react-leaflet` is pinned to
1.x, and `leaflet.heat`'s last release was **2015**. MapLibre shipped 5 releases in the last 3 weeks.
Cost accepted: ~341 kB gz vs ~111 kB.

- **No basemap.** A simulated lot does not exist in OpenStreetMap, so an OSM tile layer would place the
  robots on top of a real, wrong street — actively deceptive here. The lot is drawn from a checked-in
  `lot.geojson` (bays, aisles, charging bays, zone polygons) — the *same* data the coverage planner
  consumes, and diffable in git.
- **MapLibre is Web-Mercator-only**, so a ~30-line `src/lib/enu.ts` maps lot-metres ↔ WGS84 against a
  fixed origin, round-trip unit-tested to <1 mm. Measured projection distortion over the 150 m lot is
  **under 3 mm** — negligible.
- **Coverage is a choropleth, not a heatmap.** A heatmap is a kernel-density estimate whose appearance
  is set by `heatmap-radius` — an aesthetic knob, not data. It smears across zone boundaries and implies
  continuous measurement never taken. Zone coverage is a discrete countable quantity (cells swept / cells
  total) and a choropleth states it exactly, with a legend and numeric breaks.
- **Render split:** 3 robots as DOM `<Marker>` (focusable, real `<button>`, aria-labelled, CSS-transitioned);
  the 50–200 potholes/bays as GPU GeoJSON layers, using `promoteId` + `setFeatureState` for hover so
  source data is never rebuilt for a highlight.
- **Live updates:** raw WebSocket wrapped in `partysocket` (reconnect + backoff + send buffering), buffered
  into a ref and flushed to a **Zustand** store once per `requestAnimationFrame` so a burst becomes one
  commit. Never React Context at 10 Hz.
- **TypeScript pinned to 6.0.3** — TS 7 cannot install alongside `typescript-eslint` (reproduced ERESOLVE).
- **Accessibility (WCAG 2.2 AA, checkable not claimed):** a real `<table>` of robots/potholes/zones as the
  map's accessible equivalent; one *polite* live region for state **changes** only (never pose, or a screen
  reader is flooded at 10 Hz); Okabe-Ito severity colours (`#0072B2 / #E69F00 / #D55E00`, never red-green),
  each paired with a shape so meaning survives greyscale; `prefers-reduced-motion` honoured in marker CSS.

---

## 8. Infrastructure

| Service | Choice | Why |
|---|---|---|
| Postgres+PostGIS | **`imresamu/postgis:17-3.5-alpine`**, host port **5433** | The official `postgis/postgis` is **amd64-only** — it would run emulated here. 5432 is owned by the running Homebrew Postgres 17 |
| MQTT | `eclipse-mosquitto:2.1.2-alpine`, 1883 | Fully multi-arch. `allow_anonymous true` must be explicit or 2.x rejects every client |
| Fallback | `brew install postgis mosquitto` | **`make dev` must work with the Docker daemon down.** Documented, not hidden |
| Python | **3.12.12 via uv** | Empirically validated on this Mac with torch 2.14 + rfdetr + onnxruntime; already cached locally |
| Node | **npm, never pnpm** | pnpm ignores `NODE_OPTIONS`; this Mac needs `--no-network-family-autoselection --dns-result-order=ipv4first` or installs hang |
| ROS 2 *(optional)* | **Jazzy** + Gazebo Harmonic, `--platform=linux/arm64` explicit | Jazzy is LTS to **May 2029** (Kilted EOLs Dec 2026) and is the distro RoboStack supports on `osx-arm64`, preserving a native escape hatch if llvmpipe proves too slow. **Never `osrf/ros:*-simulation`** — amd64-only, silently emulated. Use `http://packages.ros.org` — HTTPS fails from this machine |

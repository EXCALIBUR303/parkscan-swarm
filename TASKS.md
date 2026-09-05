# ParkScan Swarm — Task Checklist

Single source of truth for progress. **Survives model switches** — update it as work lands,
and never tick a box that has not actually been verified.

Legend: `[ ]` todo · `[~]` in progress · `[x]` done & verified · `[!]` blocked

---

## Phase 0 — Discovery & architecture (Opus) — **COMPLETE**
- [x] Inspect workspace, OS, toolchain, existing project conventions
- [x] Verify ROS 2 macOS/arm64 support tier from primary source (REP-2000 + per-distro tables)
- [x] Verify Docker Hub arm64 manifests: `library/ros` ✅, `osrf/ros` ❌, `postgis/postgis` ❌, `imresamu/postgis` ✅, `eclipse-mosquitto` ✅
- [x] Verify PyPI arm64 wheel coverage for the full dependency set
- [x] Verify `packages.ros.org` HTTPS failure on this machine
- [x] Research: ROS2/sim, backend/data, frontend/map, multi-robot allocation, perception (5 fronts)
- [x] `docs/RESEARCH_LOG.md` — evidence base, every claim cited
- [x] `docs/ARCHITECTURE.md`
- [x] `docs/DATA_MODEL.md`
- [x] `docs/API_CONTRACT.md`
- [x] `docs/ROADMAP.md`
- [x] `docs/ACCEPTANCE.md`
- [x] ADRs 0001–0006
- [x] `TASKS.md` (this file)
- [x] **User decisions**: licence = MIT (default, changeable); Docker Desktop path chosen

## Phase 1 — Foundation (Sonnet) — **COMPLETE** (2026-09-03)
- [x] `git init`; `.gitignore`; LICENSE (MIT or Apache-2.0 — user's call)
- [x] Monorepo layout: `apps/api`, `apps/web`, `packages/parkscan_core`, `sim/lot/`, `docs/` (sim/coord/perception packages land in Phases 2–4)
- [x] `uv venv --python 3.12`; `pyproject.toml` with pinned deps (`sqlalchemy[asyncio]`, **not** bare)
- [x] `Makefile`: `bootstrap` / `up` / `down` / `dev` / `test` / `lint` / `demo` / `bench` / `eval` / `help`, exporting `NODE_OPTIONS`
- [x] `docker-compose.yml`: `imresamu/postgis` on **5433**, `eclipse-mosquitto` with explicit `allow_anonymous true`
- [~] Native fallback path documented; **not yet executed** (chose the Docker path this session)
- [x] Alembic migration 0001 (all tables + enums + GIST indexes + `data_source` NOT NULL)
- [x] FastAPI skeleton with `@asynccontextmanager` lifespan (**never** `@app.on_event`)
- [x] `/health` reporting db + mqtt status individually, green against real Postgres
- [x] `aiomqtt` ingest stub round-trips one message end to end
- [x] Vite + React + TS **6.0.3** shell rendering `lot.geojson` in MapLibre
- [x] `src/lib/enu.ts` + round-trip test < 1 mm
- [x] ruff, mypy, eslint, prettier  ·  [ ] pre-commit hooks (config not yet added)
- [~] GitHub Actions CI written (lint + test + AGPL/PML guard) — **never executed**, no remote
- [x] **Verify:** `make bootstrap && make test` green from a clean clone

## Phase 2 — Single-robot vertical slice (Sonnet) — **COMPLETE** (2026-09-03)
- [x] `parkscan_core.types` + `WorldBackend` Protocol
- [x] `sim-lite`: seeded kinematic sim, fixed dt, lane graph, planar ranges, battery model
- [x] Lot config + zone decomposition; test: zones tile the lot, zero gaps/overlap (C8)
- [x] Robot task state machine + **table-driven test over all (state, event) pairs** (C7)
- [x] Waypoint navigation on the lane graph (Manhattan, not Euclidean) + reactive avoidance
- [x] Feasibility-based return-to-charge, evaluated continuously during a sweep
- [x] Autonomous bay docking (A2)
- [x] Telemetry → MQTT → FastAPI → Postgres → WebSocket → moving marker (A3)
- [x] `make demo`
- [x] **Verify:** A1, A2, A3

## Phase 3 — Perception vertical slice (Sonnet) — **COMPLETE** (2026-09-04)
- [x] `Detector` Protocol + `DetSource` enum + `Detection`/`BBox` (`bottom_centre` property)
- [x] `SimulatedDetector` — per-frame seed from `hash(seed, frame_id)`, no global RNG
- [x] `assert_detector_contract` pytest (C12)
- [x] Geolocation: back-projection, horizon **rejection** (C11), 6 m range gate
- [x] `geoloc_valid` + `range_m` persisted; detections rendered as severity pins (Okabe-Ito)
- [~] Parking-space occupancy: schema, API and map layer exist; the *detector* emits potholes only — occupancy classification is Phase 4 work (A5 partial)
- [x] `make eval`: geolocation error **as a function of range**, not a single number
- [ ] `DATASETS.md` + download script — deferred: only needed when a REAL model is trained (ADR-0006). No datasets are used yet.
- [x] **Verify:** A4, A5

## Phase 4a — Swarm design review (**Opus**) — **COMPLETE** (2026-09-04) → [ADR-0007](docs/adr/0007-coordinator-concurrency.md)
- [x] Review allocator / lease / fencing / recovery design against the Phase 2–3 implementation
- [x] Confirm the deadlock argument holds against the real lane graph

## Phase 4b — Swarm implementation (Sonnet) — **COMPLETE** (2026-09-04)
- [x] 3 robots, namespaced; zone allocation, no duplicate work (A6)
- [x] `allocate()` as a pure function; `greedy` + `hungarian` + `ssi`
- [x] Property test: `greedy_total >= hungarian_total` over 1000 matrices (C9) + pinned 6.29× regression
- [x] Lease manager: TTL, renewal, reclaim, **fencing tokens** (C5) — implemented, unit-tested, and **integrated for charging bays** (`Coordinator._manage_bay_lease`); lane-segment leases remain unintegrated by design scope (Phase 6 finding closed for bays, see ADR-0007)
- [x] ~~Total resource ordering~~ **atomic per-leg acquisition** (ADR-0007 supersedes: routes acquire lanes out of id order, so the Havender rule would outlaw ordinary routes); FIFO queue + aging
- [x] Assignment epochs; stale-epoch reports dropped (C6)
- [x] Failure detection: MQTT LWT + heartbeat deadline + stuck detection (excluding `WAITING_ON_LEASE`)
- [x] Recovery: requeue with remaining cells; hysteresis + cooldown + event-triggered + hard cap
- [x] Lazarus re-registration
- [x] Failure injection: low-battery, offline, stuck, Lazarus — ≥20 events (A7)
- [x] `make bench`: 30 seeds × 3 strategies → `results.csv` + `results.json` (git SHA, config hash, versions)
- [x] Paired bootstrap 95% CIs + win/loss/tie
- [x] **Verify:** A6, A7, A11, C1–C4

## Phase 5 — Dashboard & polish (Sonnet) — **MOSTLY COMPLETE** (2026-09-04); alert feed, detection filtering and Playwright E2E not built
- [x] Fleet cards: battery, position, task, zone, health, state
- [x] Coverage choropleth + legend with numeric breaks
- [~] Detection list with severity/confidence/range — **filtering and the evidence viewer are not built** (the simulated detector emits no image evidence)
- [ ] Alert/task feed — **not built**; alerts exist in the schema and API but no UI
- [x] Persistent non-dismissable SIMULATED badge, rendered **from `data_source`** (H1, H2)
- [x] Empty / loading / error states everywhere
- [x] Accessible data-table equivalent of the map; polite live region for **changes only**
- [~] Keyboard nav, focus rings, `prefers-reduced-motion` done; **`@axe-core/playwright` not run** (Playwright binaries never verified on Darwin 27)
- [ ] Playwright E2E — **not built**; binaries were never verified on Darwin 27
- [x] README with real screenshots and only reproducible numbers
- [x] **Verify:** A9, A10

## Phase 6 — Final review (**Opus**)
- [ ] Deep review: incomplete paths, races, weak tests, security, setup correctness
- [ ] Audit every README claim against `results.csv` (A11, H3–H12)
- [ ] Resume-ready description built **only** from measured results
- [x] Bay-lease integration (the review's own finding): `Coordinator._manage_bay_lease`
  now arbitrates, releases only on physical clearance, redirects to a free bay before
  queueing, and gates redirect on charge feasibility — plus a pre-existing
  `battery_policy.soc_per_metre` miscalibration (>2x low vs. the simulator's real energy
  model) found and fixed along the way. See ADR-0007.

## Phase S — Stretch: `ros2-gz` backend (Sonnet)
- [x] **RTF spike first** (see ROADMAP) — measured RTF ≈ 1.0 (1.0 and 0.9998 across two
  samples, cross-checked from raw sim/real time deltas) on this host: Docker 29.7.2,
  native arm64, software-rendered (`LIBGL_ALWAYS_SOFTWARE=1`), headless. Well above the
  ~0.3 gate — Phase S is viable as specified, no RoboStack/pixi fallback needed. The
  actual implementation below is not started.
- [x] **Design review before implementation** (matching Phase 4a's pattern) → [ADR-0008](docs/adr/0008-ros2-backend-bridge-design.md):
  namespace scheme, the MQTT bridge wire protocol (didn't exist in any doc), lockstep timing
  via Gazebo's world control service (not wall-clock sleep — the Protocol forbids that), where
  battery tracking lives, and the seed-reproducibility gap vs. `SimLite` stated explicitly.
- [x] **Full implementation, built and run for real (2026-09-05).** `packages/parkscan_ros2`:
  `sdf.py` (world generator), `bridge/gz_control.py` (Gazebo lockstep control), `bridge/protocol.py`
  (the MQTT wire format), `bridge/geometry.py`, `bridge/node.py` (the rclpy bridge — the one
  module that imports `rclpy`, excluded from this project's mypy for that reason, verified by
  running it instead), and `backend.py` (`Ros2WorldBackend`). `battery.py` relocated to
  `parkscan_core` so a second backend doesn't depend on a peer one for it. 60+ tests across
  everything host-testable. Six real, previously-undocumented bugs found and fixed by actually
  running the system, not by reading a manual — see ADR-0008 for all six in full: a per-robot
  topic-scoping collision, `multi_step` needing `pause: true` paired with it, an odometry
  velocity warm-up artifact, DART's default friction damping commanded velocity by ~61%, a reset
  warm-up needing several stepped chunks rather than one iteration, and an uncaught
  `subprocess.TimeoutExpired` that was crashing the bridge's MQTT callback.
- [x] `ros:jazzy-ros-base` arm64 Dockerfile (`http://packages.ros.org`, `libgl1-mesa-dri`, ogre2)
  → `infra/ros2/Dockerfile` + `infra/ros2/entrypoint.sh`, multi-stage (a throwaway stage renders
  the SDF at build time so the runtime image never needs `parkscan_core`). Built and run for
  real via `docker compose --profile ros2 build && up`, not merely written.
- [ ] `rmw_zenoh_cpp` + router — **not done.** Verification used the base image's default RMW;
  ADR-0008's scope note explains why this specific MQTT-relay design may not even need it (no
  ROS 2 traffic crosses the host/container boundary zenoh was chosen to cross), but proving
  zenoh out was this bullet's own goal, independent of whether the design strictly requires it.
- [x] `Ros2WorldBackend` implementing the same Protocol; MQTT bridge — both built, both run for
  real against the actual compose stack (see below), not just unit-tested against fakes.
- [x] **Verify:** identical coordinator/perception code runs unmodified on both backends —
  confirmed for real: `python -m parkscan_runner.run --backend ros2-gz` against the live
  `docker compose --profile ros2` stack assigned all three zones and moved all three robots,
  zero changes to `Coordinator`, `RobotAgent`, or the allocator. Confirmed reliable for a
  realistic, moderate-length run (100 steps, clean). **Not yet reliable on long sustained runs**
  (600 steps failed on `gz-transport` overhead from spawning a subprocess per `step()` call —
  ADR-0008's sixth finding; a real, deferred throughput limitation, not a correctness bug in
  what does complete).

---

## Open decisions (need the user)
1. **Start Docker Desktop?** Needed for the compose path. The native brew fallback works without it.
2. **`brew install postgis mosquitto`?** ~1–2 GB and a long first build for PostGIS (gdal/geos/proj/sfcgal).
3. **Licence** for the repo — MIT or Apache-2.0?
4. **Public GitHub repo?** Not created without approval. Affects the licence/dataset story.
5. **Lovable** is connected and authenticated (workspace "Sidharth's Lovable"). Using it for the
   Phase 5 dashboard **consumes workspace credits** — approve before use, or the dashboard is built locally.
6. **Keep the project name "ParkScan Swarm"?** 3 centrally-coordinated robots are a coordinated
   fleet, not a swarm. Recommendation: keep the name, define the term honestly in the README's
   first paragraph (see ACCEPTANCE H11).
7. **Ever monetised?** Changes dataset choices (CC BY-NC becomes an outright blocker) and licence.

# ParkScan Swarm

**Multi-robot autonomous parking-lot inspection and pothole-mapping — simulation-first.**

> 🔬 **Everything here is simulated. There is no hardware.** Every telemetry and detection
> record carries a `data_source` / `det_source` column that is `NOT NULL` in PostgreSQL, and
> the dashboard renders its SIMULATED badge *from that value* rather than from a hardcoded
> string. The labelling is structural, so it cannot be forgotten.

Three coordinated robots sweep a 150 × 100 m parking lot: they divide it into zones without
duplicating work, navigate lane-by-lane, detect potholes from simulated camera geometry,
geolocate those detections onto a map, and return to a charging bay when the remaining charge
can no longer cover the trip home. A FastAPI backend ingests their MQTT telemetry into PostGIS
and pushes it live to a React dashboard.

**On the word "swarm":** these are three *centrally coordinated* robots, not a decentralised
swarm with emergent behaviour. The name is the project's; the architecture is a coordinated
fleet with a central allocator, and nothing here trades on the stronger term.

---

## The constraint that shaped the architecture

Three facts, each verified on the target machine and recorded in
[`docs/RESEARCH_LOG.md`](docs/RESEARCH_LOG.md):

1. **ROS 2 does not support macOS on arm64 at any tier.** REP-2000 and the per-distro platform
   tables list macOS as Tier 3, amd64-only; the arm64 cell is blank.
2. **Docker Desktop on macOS gives containers no GPU.** A containerised Gazebo renders every
   camera through Mesa `llvmpipe` on the CPU.
3. **Perception cannot be accelerated in a container either** — MPS and the CoreML EP are
   unreachable from a Linux guest.

So the simulator wants a container, the detector must stay on the host, and DDS cannot
reliably cross that boundary. The process split is *forced by the platform*, not chosen — so
everything of substance sits above a narrow `WorldBackend` adapter
([ADR-0001](docs/adr/0001-two-simulation-backends.md)), with a deterministic pure-Python
simulator as the default backend and ROS 2 + headless Gazebo as an optional second one.

**This is therefore not a ROS 2 system today,** and the ROS 2 backend has not been built. See
[Limitations](#limitations).

## Running it

Requires macOS or Linux, [uv](https://docs.astral.sh/uv/), Node 20+, and Docker.

```bash
make bootstrap     # uv sync + npm install, creates .env
make up            # PostGIS on host port 5433 + mosquitto on 1883
make migrate       # Alembic migrations
make smoke         # proves MQTT -> ingest -> PostGIS actually round-trips
make dev           # API on :8000, dashboard on :5173
make demo          # one robot sweeps a zone, as fast as the CPU allows
```

`make help` lists every target. **Port 5433 is not a typo** — a Homebrew PostgreSQL commonly
owns 5432, and silently connecting to the wrong database is a bad first-run experience.

| Target | What it does |
|---|---|
| `make demo` / `demo-watch` / `demo-dock` | single robot; real-time; near-flat battery so it must return and dock |
| `make bench` | 30 seeds × 3 allocation strategies → `apps/runner/results/` |
| `make eval` | geolocation accuracy vs known truth, binned by range → `apps/eval/results/` |
| `make test` · `make lint` | 787 Python + 74 web tests; ruff, mypy --strict, eslint, tsc |
| `make licence-guard` | fails if an AGPL/PML model dependency appears ([ADR-0006](docs/adr/0006-detector-licensing.md)) |
| `make up-ros2` | additionally builds and starts the `ros2-gz` backend ([ADR-0008](docs/adr/0008-ros2-backend-bridge-design.md)) — opt-in, not part of `make up` |

**Without Docker:** `brew install postgis mosquitto && make up-native`. The stack is designed
so the daemon is optional — though that path is documented rather than exercised.

## What it does, and how it is checked

| Capability | Evidence |
|---|---|
| 3 robots divide 12 zones with no duplication | `duplicate_visit_rate = 0.0` across all 90 benchmark runs |
| Complete coverage | `coverage_pct = 100.0`, minimum across all 90 runs |
| No collisions | `conflict_count = 0`, maximum across all 90 runs — lane-segment avoidance is sensor-based; bay-approach arbitration is the lease layer (ADR-0007) |
| No robot ever strands | `depletion_events = 0`, total across all 90 runs |
| Allocation does not thrash | `reassignment_churn = 0.0`, maximum across all 90 runs |
| An offline robot's work is taken over and **resumed**, not restarted | `apps/runner/tests/test_failure_injection.py` |
| A low-battery robot hands off and docks unaided | same |
| The fleet still finishes every zone after a robot is lost | same |

### Allocation strategies, compared honestly

30 seeds × 3 strategies, paired bootstrap 95% CIs over the same seeds. Regenerate with
`make bench`; every figure below is a row of the committed `apps/runner/results/results.csv`.

| Strategy | Median makespan | Median distance |
|---|---|---|
| greedy (baseline) | 649.5 s | 2036.8 m |
| Hungarian (`scipy.optimize.linear_sum_assignment`) | 628.4 s | 2036.7 m |
| SSI auction (marginal-cost / BidSumPath) | **581.0 s** | **1890.6 m** |

- **SSI beats greedy** by 68.6 s of makespan (29/30 wins) and 146.2 m of travel (30/30 wins).
  Both significant.
- **Hungarian does not beat greedy on makespan** — median difference 0.0 s, 95% CI
  [−10.6, +21.3], 10/30 wins. **Not significant**, and reported as such.

That null result is the interesting one. Solving each round optimally does not improve
makespan, because makespan is a property of the whole schedule and a per-round optimal
assignment does not optimise it. Its distance edge is real and useless: 4.8 cm. SSI wins
because marginal-cost bidding prices a zone against the bundle a robot already holds — the
only one of the three that reasons about the schedule at all.

**CBBA was evaluated and rejected**, and that rejection is
[an ADR](docs/adr/0005-allocation-strategies-cbba-rejected.md) rather than a silence: it is
394–920 LOC of consensus logic solving inconsistent situational awareness over a time-varying
comms graph, and this system has one broker and a central coordinator. Its premise does not
hold, and implementing it while claiming decentralised allocation would be an overclaim.

### Perception

Everything sits behind a `Detector` protocol with a shared contract test. `detect()` is pure
with respect to the frame, is never handed the robot pose, and never emits world coordinates —
geolocation is a separate stage, so a detection cannot read its own answer out of the map.

Geolocation **rejects** rays at or above the horizon rather than clamping them; a clamped ray
yields a plausible point hundreds of metres away that silently poisons the map. The API
refuses a valid fix without a point and an invalid fix with one, and the dashboard lists
rejected detections without ever plotting them.

Measured error, from `make eval` (see [`apps/eval/EVAL.md`](apps/eval/EVAL.md)):

| Ground range | Mean error | p95 |
|---|---|---|
| 0–2 m | 0.050 m | 0.106 m |
| 2–4 m | 0.176 m | 0.465 m |
| 4–6 m | 0.471 m | 1.094 m |
| 8–12 m | 1.914 m | 5.127 m |
| 16–20 m | 6.582 m | 21.722 m |

Error grows **quadratically** with range (`err/range²` stays ≈0.017–0.021), which is why
detections beyond a 6 m gate are rejected. Inside the gate: mean 0.315 m, p50 0.184 m.

> **Every number above is Tier 1 — pipeline measurement against known simulated truth.** It
> says nothing about real-world detection accuracy, and `EVAL.md` states in full what may and
> may not be claimed. mAP and precision/recall are **not measured here and cannot be.**

## Architecture

```
macOS host (native)                        containers
┌────────────────────────────────┐        ┌──────────────────────────┐
│ sim-lite  (deterministic 2D)   │  MQTT  │ mosquitto 2.1.2          │
│ robot_1..3  autonomy + FSM     ├───────►│                          │
│ coordinator  alloc/lease/heal  │        └──────────┬───────────────┘
│ perception  Detector + geoloc  │                   │
└────────────────────────────────┘        ┌──────────▼───────────────┐
                                          │ FastAPI  REST + WS       │
   React 19 + MapLibre 6  ◄──── WebSocket │ PostgreSQL 17 + PostGIS  │
                                          └──────────────────────────┘
```

| Doc | Contents |
|---|---|
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | The design and the platform constraint behind it |
| [`docs/RESEARCH_LOG.md`](docs/RESEARCH_LOG.md) | Every architectural claim and the evidence for it |
| [`docs/DATA_MODEL.md`](docs/DATA_MODEL.md) | PostGIS schema; fencing tokens, epochs, provenance |
| [`docs/API_CONTRACT.md`](docs/API_CONTRACT.md) | REST, WebSocket, MQTT topics |
| [`docs/ACCEPTANCE.md`](docs/ACCEPTANCE.md) | Functional, correctness **and honesty** criteria |
| [`docs/adr/`](docs/adr/) | Seven decision records, including what was rejected and why |
| [`apps/eval/EVAL.md`](apps/eval/EVAL.md) | What the perception numbers do and do not mean |
| [`TASKS.md`](TASKS.md) | Live progress checklist |

Two decisions worth reading: **[ADR-0007](docs/adr/0007-coordinator-concurrency.md)**, where a
design review found that the classic Havender resource-ordering rule cannot be used here —
measured, a normal route acquires lane segments out of id order, so the rule would outlaw
ordinary routes — and the design switched to atomic per-leg acquisition; and
**[ADR-0004](docs/adr/0004-maplibre-no-basemap.md)**, on why this map has no basemap.

## Limitations

Stated up front, because a portfolio project that hides these is worth less than one that names them.

- **It is now, sometimes, a ROS 2 system — but not yet a reliable one on long runs.** ROS 2 does
  not support macOS on arm64, so the default backend is a purpose-built deterministic simulator.
  The `ros2-gz` backend ([ADR-0001](docs/adr/0001-two-simulation-backends.md),
  [ADR-0002](docs/adr/0002-ros2-jazzy-over-lyrical.md), [ADR-0008](docs/adr/0008-ros2-backend-bridge-design.md))
  — a Gazebo world, an `rclpy` bridge, and `Ros2WorldBackend` talking to it over MQTT — is built
  and was run for real: `apps/runner --backend ros2-gz` against the actual built
  `docker compose --profile ros2` stack assigned zones and moved all three robots with zero
  changes to `Coordinator` or `RobotAgent`. Confirmed clean at a realistic 100-step run; a longer
  600-step run failed on transport overhead from the current per-tick subprocess control channel
  (ADR-0008 has the full finding). `rmw_zenoh_cpp` is also not yet configured. Until both are
  resolved, `ros2-gz` is a second working implementation, not yet an equally trusted one.
- **The reservation layer is wired in for charging bays, not for lane segments.**
  `parkscan_coord.leases` implements atomic acquisition, TTL leases, fencing tokens and an
  aging FIFO queue. `Coordinator._manage_bay_lease` now calls it every tick a robot is
  returning to, queued for, or occupying a bay, and a robot genuinely enters
  `WAITING_ON_LEASE` when every reachable bay is taken. The general case — a lease per lane
  segment a robot is about to drive — remains unintegrated: the agent, not the coordinator,
  owns its route, so the coordinator has no per-segment reservation to grant. Ordinary
  driving still avoids collisions the way it always did, by robots seeing each other in
  their range readings and steering around one another; only the final approach into a bay,
  where that avoidance is deliberately switched off, is arbitrated by the lease layer.
  [ADR-0007](docs/adr/0007-coordinator-concurrency.md) has the four fixes integration
  needed and what is still open (fencing tokens are not wired into the physical docking
  action; three simultaneous returns on two bays still leaves one robot queueing with no
  upper time bound).
- **No physics engine.** 2D kinematics on a flat plane. No dynamics, no wheel slip, no terrain.
- **The simulated detector does no vision.** It converts known ground-truth geometry into
  detection-shaped records so the pipeline is exercised end to end. It never reads a pixel.
  A real-model path (RF-DETR Nano, Apache-2.0) is specified but not trained or integrated.
- **Monocular flat-ground geolocation degrades quadratically with range**, and a pothole is by
  definition a depression *below* the assumed plane — an unmodelled bias.
- **Parking-space occupancy** has schema, API and map support, but the detector emits potholes
  only; occupancy classification is not implemented.
- **Local dev auth only.** No authentication; the API binds to localhost. Production auth is
  documented, not built.
- **Single uvicorn worker**, because WebSocket state is in-process. A deliberate MVP boundary.
- **`/alerts`, `/tasks` and `/reports/run/{id}` are specified but not implemented** — marked as such in the API contract.
- **CI is written but has never run** — there is no remote.
- The **no-Docker path** is documented but was not exercised.

## Verified on the development machine

Apple M5, macOS 27, arm64. Everything below was run, not asserted.

| Check | Result |
|---|---|
| `make test` | **767 Python + 74 web tests pass** |
| ruff · ruff format · mypy --strict | clean, incl. tests |
| eslint · tsc · vite build | clean |
| Alembic upgrade → downgrade → upgrade | round-trips on real PostGIS |
| `make smoke` | MQTT → ingest → PostGIS round-trip, geography populated |
| `make bench` | 90 runs, all correctness gates hold |
| `make eval` | byte-identical CSV for a fixed seed |
| Determinism | same seed ⇒ bit-identical 400-step trajectory |
| Live dashboard | 3 robots moving, detections pinned, coverage choropleth shading |

## Licence

MIT — see [LICENSE](LICENSE). Model and dataset licensing is governed by
[ADR-0006](docs/adr/0006-detector-licensing.md); Ultralytics YOLO is **banned** because
AGPL-3.0 §13 would relicense this entire repository, and `make licence-guard` enforces it.

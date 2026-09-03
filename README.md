# ParkScan Swarm

**Multi-robot autonomous parking-lot inspection and pothole-mapping — simulation-first.**

> ⚠️ **Status: Phase 0 (architecture only). No implementation yet.**
> Nothing in this repository runs. This README describes what is designed, not what exists.
> It will be replaced with real setup instructions, real screenshots and reproducible numbers as
> the phases land.

> 🔬 **All robots, sensors and detections are simulated.** There is no hardware. Every telemetry
> and detection record carries a `data_source` / `det_source` field that is NOT NULL at the database
> level and drives a persistent badge in the dashboard — the labelling is structural, not a disclaimer.

## What this is

Three coordinated robots sweep a simulated parking lot: they divide it into zones without duplicating
work, navigate lane-by-lane to inspection waypoints, detect potholes and parking-space occupancy from
simulated camera frames, geolocate those detections onto a map, and autonomously return to a charging
bay when the remaining charge can no longer cover the trip home. A FastAPI backend ingests their MQTT
telemetry into PostGIS and pushes it live to a React dashboard.

**On the word "swarm":** these are three *centrally coordinated* robots, not a decentralised swarm
with emergent behaviour. The name is kept because it is the project's name; the architecture is a
coordinated fleet with a central allocator, and this README will not trade on the stronger term.

## Documentation

| Doc | What's in it |
|---|---|
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | The design, and the platform constraint that shapes it |
| [`docs/RESEARCH_LOG.md`](docs/RESEARCH_LOG.md) | Every architectural claim, with the evidence it rests on |
| [`docs/DATA_MODEL.md`](docs/DATA_MODEL.md) | PostGIS schema; fencing tokens, epochs, `data_source` |
| [`docs/API_CONTRACT.md`](docs/API_CONTRACT.md) | REST, WebSocket, MQTT topics |
| [`docs/ROADMAP.md`](docs/ROADMAP.md) | Phased plan, and the one spike that gates ROS 2 |
| [`docs/ACCEPTANCE.md`](docs/ACCEPTANCE.md) | Functional + correctness + **honesty** criteria |
| [`docs/adr/`](docs/adr/) | Six decision records, including what was rejected and why |
| [`TASKS.md`](TASKS.md) | Live progress checklist |

## Known limitations (stated up front, by design)

- **This is not a ROS 2 system today.** ROS 2 does not support macOS on arm64 at any tier, so the
  default backend is a purpose-built deterministic simulator. A ROS 2 Jazzy + headless Gazebo
  backend is planned behind the same interface (ADR-0001) and will only be claimed once it runs.
- **No physics engine** in the default backend — 2D kinematics on a flat ground plane.
- **Simulated detections say nothing about real-world detection accuracy.** Pipeline metrics and
  model-accuracy metrics are reported separately and never blended (ACCEPTANCE H3, H4).
- **Monocular flat-ground geolocation degrades quadratically with range** — a 1° pitch error costs
  0.03 m at 0.8 m but 5.2 m at 13 m. Detections beyond ~6 m are rejected.
- **Local dev auth only** for the MVP; production auth is documented, not implemented.
- Single uvicorn worker (in-process WebSocket state) — a deliberate MVP boundary.

# Roadmap

Vertical slices. Each phase ends with something runnable and tested, not a layer.
Model recommendation per phase follows the project's model policy.

| Phase | Deliverable (definition of done) | Model |
|---|---|---|
| **0 — Discovery & architecture** ✅ | Evidence log, architecture, data model, API contract, acceptance criteria, ADRs, task checklist | Opus |
| **1 — Foundation** | Monorepo + `make bootstrap` / `make dev` / `make test`; docker-compose (postgis 5433 + mosquitto) **with a native fallback that works with the daemon down**; Alembic migration 0001; FastAPI `/health` green against real Postgres; MQTT ingest stub round-trips one message; Vite+React shell renders the lot from `lot.geojson`; ruff + mypy + eslint + GitHub Actions CI | Sonnet |
| **2 — Single-robot slice** | `sim-lite` backend; `robot_1` navigates ≥3 waypoints on the lane graph; telemetry → MQTT → FastAPI → Postgres → WebSocket → a moving marker; battery model + feasibility-triggered return-to-charge; autonomous bay parking; state-machine transition table test; `make demo` script | Sonnet |
| **3 — Perception slice** | `Detector` Protocol + `SimulatedDetector` + `assert_detector_contract`; geolocation with horizon rejection and the 6 m range gate; detections persisted with `det_source` and rendered as severity pins; parking-space occupancy; `make eval` reporting geolocation error **as a function of range** | Sonnet |
| **4a — Swarm design review** | Validate allocator/reservation/recovery design against the implementation before it hardens | **Opus** |
| **4b — Swarm implementation** | 3 robots, zone allocation (greedy + hungarian + ssi), leases with fencing tokens, resource ordering, FIFO+aging queue; failure injection: low-battery reassignment, offline robot, stuck robot, Lazarus return; `make bench` over 30 seeds → `results.csv` + paired bootstrap CIs | Sonnet |
| **5 — Dashboard & polish** | Full dashboard (fleet cards, choropleth coverage, detection filtering, alert feed, empty/loading/error states); WCAG 2.2 AA incl. the data-table equivalent + live region; Playwright E2E against a recorded WS fixture; README with real screenshots and only reproducible numbers | Sonnet |
| **6 — Final review** | Deep review: incomplete paths, race conditions, misleading claims, weak tests, security; resume-ready description built **only** from measured results | **Opus** |
| **S — Stretch: `ros2-gz`** | `ros:jazzy-ros-base` arm64 image + ros_gz + rmw_zenoh, headless Gazebo, 3 namespaces, MQTT bridge — behind the same `WorldBackend`. RTF spike below passed (≈1.0); implementation not started | Sonnet |

## The one spike that must happen before Phase S — **done, RTF ≈ 1.0, passed**

Before writing any ROS 2 code, run this and read the reported real-time factor. The `-v 4`
debug log does not print RTF anywhere; it has to be read off the `/world/<name>/stats`
topic directly, and `source`ing the ROS environment is required or `gz` is not on `PATH`
(the original form of this command below was missing that and fails with `gz: command
not found` — fixed here):

```bash
docker run --rm --platform=linux/arm64 ros:jazzy-ros-base bash -c \
  "sed -i 's|https://packages.ros.org|http://packages.ros.org|g' /etc/apt/sources.list.d/*.list* 2>/dev/null; \
   apt update && apt install -y ros-jazzy-ros-gz-sim libgl1-mesa-dri && \
   source /opt/ros/jazzy/setup.bash && \
   (DISPLAY= LIBGL_ALWAYS_SOFTWARE=1 gz sim -s -r --headless-rendering shapes.sdf &) && \
   sleep 12 && timeout 8 gz topic -e -t /world/shapes/stats -n 1 && \
   sleep 25 && timeout 8 gz topic -e -t /world/shapes/stats -n 1"
```

**Measured on this host** (Docker 29.7.2, native arm64, `--platform=linux/arm64`,
`LIBGL_ALWAYS_SOFTWARE=1` software rendering, dartsim physics, `--headless-rendering`):

| Sample | Wall time | `iterations` | `real_time_factor` |
|---|---|---|---|
| 1 | ~12 s in | 9,931 | 1.0 |
| 2 | ~37 s in | 35,027 | 0.9998 |

Cross-checked independently from the raw `sim_time`/`real_time` deltas between the two
samples: `(35.027 − 9.931) / (36.864 − 11.710) ≈ 0.998`. Both agree: **RTF ≈ 1.0**, more
than 3x above the ~0.3 gate below. **Decision: Phase S is viable on this host as specified
— software-rendered, headless, in Docker — with no RoboStack/pixi fallback needed.**

**That single number decides Phase S.** If RTF < ~0.3, the documented fallback is RoboStack/pixi
(`robostack-jazzy`, `osx-arm64`) for native Apple-Silicon rendering with a real GPU — or Phase S is
honestly reported as not completed. It is never reported as done when it is not.

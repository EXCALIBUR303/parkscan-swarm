# Research Log — Phase 0 Evidence Base

Every architectural decision in this project traces to a fact verified on **2026-09-02/03**, on
**this machine** (Apple M5, macOS 27.0, arm64, 16 GB), not to assumption. This file is the
evidence. If a claim here goes stale, the decision that rests on it should be revisited.

Method: 5 parallel research agents (864k tokens, 264 tool calls) plus first-hand verification
by the lead. Claims marked **[verified here]** were reproduced directly on this machine.

---

## 1. ROS 2 on this Mac

| Fact | Evidence |
|---|---|
| **ROS 2 does not support macOS on arm64 at any tier.** For Jazzy, Kilted and Lyrical the platform table lists macOS as **Tier 3, amd64 only**; the arm64 row's macOS cell is *blank*. Tier 3 = source build only (no `[d]` debs, no `[a]` archives). | **[verified here]** `rep-2000.rst` fetched from `raw.githubusercontent.com/ros-infrastructure/rep/master`; Jazzy + Kilted tables read directly. Agent additionally confirmed the same for Lyrical via `ros2_documentation/rolling/source/Releases/lyrical/supported-platforms.rst`, which adds: *"To use ROS Lyrical on any Tier 3 platform, you must build ROS Lyrical from source."* |
| Current LTS as of Sept 2026 is **Lyrical Luth** (rel. 2026-05-22, EOL May 2031). **Jazzy Jalisco** is LTS to May 2029. **Kilted Kaiju EOLs December 2026** — ~3 months out. | `ros2_documentation/rolling/source/Releases.rst` |
| REP-2000 is **stale** — no Lyrical section, last commit 2025-07-03. The maintained source is the per-distro `supported-platforms.rst`. | GitHub API commit history for `rep-2000.rst` |
| `docker.io/library/ros` **does** publish genuine `linux/arm64` manifests for humble, jazzy, kilted, lyrical, rolling (ros-core / ros-base / perception). | **[verified here]** Docker Hub v2 tags API, 100 tags enumerated with per-image `architecture` field |
| **`osrf/ros` — the family that ships Gazebo and RViz (`*-simulation`, `*-desktop-full`) — is amd64-ONLY. 100 of 100 tags, zero arm64.** Pulling it here silently runs under x86-64 emulation. This is the single easiest mistake to make, because most Gazebo tutorials tell you to pull exactly these. | Docker Hub v2 tags API over all 100 tags |
| The ROS 2 arm64 apt repo is near-complete: 2638 arm64 vs 2642 amd64 packages. `ros-*-ros-gz*`, 61 nav2 packages, `slam-toolbox`, `rmw-zenoh-cpp` all present as prebuilt arm64 debs. Only 4 packages are amd64-exclusive (`libtorch-vendor`, `open3d-vendor`, `torch-conversions`, `rti-connext-dds`) — none needed here. | `packages.ros.org/.../binary-{arm64,amd64}/Packages.gz` downloaded and diffed |
| **HTTPS to `packages.ros.org` FAILS from this machine** (curl exit 60, TLS handshake); plain HTTP returns 200. Any Dockerfile hardcoding `https://packages.ros.org` will fail to build here. | **[verified here]** `curl https://... ` → `code=000 exit=60`; `curl http://...` → `code=200 size=3797` |
| **Docker Desktop on macOS gives containers no GPU access at all** — Apple exposes no virtual-GPU API to Virtualization.framework guests. Every Gazebo camera sensor in a container renders on CPU via Mesa `llvmpipe`. | Gazebo docs (*"A GPU instance is recommended when sensors that require a render engine are used"*), NVIDIA Container Toolkit docs, corroborating reports |
| Gazebo *can* run server-only headless: `DISPLAY= gz sim -v 4 -s -r --headless-rendering world.sdf`. EGL headless requires the **ogre2** render engine, so the SDF must specify it. `libgl1-mesa-dri` must be installed explicitly — `gz-rendering-vendor` pulls `libegl1`/`libopengl0` but *not* the DRI driver that provides llvmpipe. | `gazebosim.org/api/sim/9/headless_rendering.html`; parsed `Depends:` of `ros-lyrical-gz-rendering-vendor` |
| Native Gazebo on Apple Silicon is unreliable: **gz-sim#2952 "Segmentation fault on running `gz sim -v 4 -s` on MacOS M1" is still OPEN**, hitting exactly the headless server mode this project would need. No Apple Silicon CI; macOS rendering tests disabled since 2022 (#1785). | gz-sim issue tracker |
| **Webots is unusable here**: no arm64 Linux build exists (*"Linux 64 (x86-64) systems only"*), so it cannot run in a native arm64 container; `webots-ros2-driver` is not released for Lyrical on any architecture; and the native macOS build officially supports only macOS 12/13/14 (this is macOS 27). | cyberbotics.com; ROS index |
| DDS multicast discovery is **not reliable across the container↔macOS-host boundary**. Docker Desktop host networking *"works on layer 4 … protocols below TCP or UDP are not supported"*, and there is no `docker0` on the macOS host. `rmw_zenoh_cpp` (Tier 1 from Kilted onward) with a router sidesteps this. | Docker docs; REP-2000 middleware table |
| ⚠️ **Not empirically tested**: the Docker daemon was stopped throughout, so no Docker/Gazebo claim above was executed on this Mac. All are from published manifests, package indexes and issue trackers. | — |

## 2. Backend / data

| Fact | Evidence |
|---|---|
| **`postgis/postgis` publishes no arm64 manifest** — amd64-only across every tag. (A high-ranking 2026 blog post claiming otherwise is wrong.) | **[verified here]** Docker Hub v2 API across 30 tags — all `amd64,unknown` |
| `imresamu/postgis` **does** publish arm64 (40+ arm64-capable tags, rebuilt 2026-08-31); maintained by the #2 committer on the official `postgis/docker-postgis`. | **[verified here]** Docker Hub v2 API |
| `brew install postgis` 3.6.4 is available with an `arm64_tahoe` bottle, but drags in gdal/geos/proj/sfcgal/boost (~1–2 GB). Running PG is 17.10 while `pg_config` is 17.11 — restart the service after installing PostGIS or `CREATE EXTENSION` may not find it. | **[verified here]** `brew info --json=v2 postgis`; `brew services list` |
| `eclipse-mosquitto` is fully multi-arch incl. arm64; 2.1.2 current. | **[verified here]** Docker Hub v2 API |
| **Bare `pip install sqlalchemy` omits greenlet on macOS arm64** — SQLAlchemy's marker tests for `aarch64`, macOS reports `arm64`. Async engines then fail at runtime with an opaque error. Must be `sqlalchemy[asyncio]`. | Agent resolved both forms on this host |
| **`httpx` `ASGITransport` does not run FastAPI lifespan.** Tests would exercise an app whose MQTT client never started. Needs `asgi-lifespan`'s `LifespanManager`. | httpx + asgi-lifespan docs |
| **Mosquitto 2.x defaults `allow_anonymous` to false** as soon as a config file with listeners is supplied — every client is silently rejected without it. | mosquitto.org docs |
| `aiomqtt` 3.x is **alpha only** (3.0.0a1, 2026-04-02) and swaps the underlying protocol library. Pin `aiomqtt==2.5.1`. `paho-mqtt` has not cut a release since 2024-04-29. | PyPI |
| `ST_Distance` on `geometry(4326)` returns **degrees**, not metres. Use `geography(Point,4326)`. | PostGIS docs |
| **SQLite/SpatiaLite is not a viable test fallback** — it would leave every spatial query (nearest bay, pothole clustering, zone containment) completely untested. | — |
| All required packages have arm64 wheels for CPython 3.12–3.14. `opencv-python-headless` ships `cp37-abi3-macosx_13_0_arm64` (works on any 3.7+). | **[verified here]** PyPI JSON API across 17 packages |

## 3. Frontend / map

| Fact | Evidence |
|---|---|
| **`typescript-eslint` cannot install alongside TypeScript 7** — peer range is `>=4.8.4 <6.1.0`, because TS 7.0 shipped without a stable programmatic API (due 7.1). Reproduced as `ERESOLVE`. Pin `typescript@6.0.3`. | Agent reproduced the install failure |
| MapLibre GL JS v6 requires **WebGL2 and is ESM-only** — no UMD/CJS. It **cannot render in jsdom/happy-dom**, so the map is untestable in standard Vitest unit env. MapLibre's own suite uses headless-gl, which is WebGL1 only (#2605). | package.json `exports`, dist inspection |
| MapLibre has **no non-geographic CRS** (Web Mercator only; #1228, #168 open) — no `L.CRS.Simple` equivalent. But measured distortion for a 150 m × 100 m lot is **0.0 mm at the equator, 1.82 mm at lat 37.77, 2.96 mm at lat 51.5**. Negligible: write a ~30-line ENU helper. | Agent computed the projection error directly |
| **Leaflet is a stalled bet**: stable 1.9.4 is from May 2023; 2.0 alpha since May 2025 with release date officially "unknown"; `react-leaflet` 5.0.0 (Dec 2024) is pinned to Leaflet 1.x. `leaflet.heat`'s last release was **2015-10-26**. MapLibre shipped 5 releases in the last 3 weeks. | npm registry, GitHub |
| Measured bundle cost: MapLibre ~341 kB gz vs Leaflet ~111 kB gz. `vite build` of a real map component took **217 ms** on this Mac. | Agent built both probes here |
| ⚠️ Playwright browser binaries are **not installed** and were not verified to launch on Darwin 27.0.0 (newer than published support matrices). Verify before committing to E2E. | — |

## 4. Multi-robot coordination

| Fact | Evidence |
|---|---|
| "Assign 12 zones to 3 robots" is **ST-SR-TA** in Gerkey & Matarić — time-extended, scheduling class `R ∥ Σ w_j C_j`, **strongly NP-hard**. The standard approximation is iterated **ST-SR-IA**, solvable exactly per-round by the Hungarian method. | Gerkey & Matarić (IJRR 2004), §5.2 |
| Because robots contend for shared lanes and bays, utilities are **not independent** → the honest iTax label is **XD [ST-SR-TA]**, not ND. | Korsah, Stentz & Dias (IJRR 2013) |
| **The "greedy is 2-competitive" claim is for MAXIMISING utility.** Under cost *minimisation* the ratio is **unbounded** — measured 6.288× on random 3×3 matrices and ~5×10⁵ on `[[1,1+ε],[1+ε,1e6]]`. Most portfolio write-ups get this wrong. | Agent computed both locally |
| Online greedy assignment is exactly **3-competitive, and 3 is provably optimal** for any online algorithm — a lower bound, not a bug. | Kalyanasundaram & Pruhs (1993) |
| `scipy.optimize.linear_sum_assignment` handles rectangular 3×12 natively. | **[verified here]** scipy 1.18.1 has cp312–cp315 wheels |
| **CBBA measured at 394–920 LOC** of consensus logic; its premise (inconsistent situational awareness over a time-varying comms graph) **does not hold** under a central MQTT coordinator. Rejected on those grounds, not on effort. | Agent cloned two reference implementations |
| SSI auction with marginal-cost bids (**BidSumPath**, Lagoudakis et al. RSS 2005) is ~80–120 LOC reusing the same cost function, and carries a real theorem: **≤2× optimal on MiniSum** (lower bound 1.5), the best of the six rules analysed. | Lagoudakis et al. (RSS 2005) |
| A lease with a timeout **alone is not correct** — a stalled holder can wake after expiry still believing it holds the resource. Requires a **fencing token** (monotonic counter, resource rejects any lower token). VDA 5050 does exactly this in industry with `orderId`/`orderUpdateId`. | Gray & Cheriton (SOSP 1989); Kleppmann; VDA 5050 |
| Deadlock needs all four Coffman conditions. Break **circular wait** by total resource ordering (Havender/Dijkstra) *and* **no-preemption** by lease expiry — ordering alone does nothing about a dead robot holding a bay. | Coffman et al. (1971) |
| Voronoi/Lloyd coverage control solves optimal **static sensor placement**, not exhaustive sweep. Using it and calling it "coverage" is the most common overclaim in portfolio swarm projects. | Cortés, Martínez, Karataş & Bullo (2004) |

## 5. Perception

| Fact | Evidence |
|---|---|
| **Ultralytics YOLO (v5/v8/v11/v12/YOLO26) is AGPL-3.0.** Because this project serves a FastAPI backend + React dashboard over a network, **AGPL §13 would force the entire repo** — ROS nodes, backend, frontend, MQTT glue, fine-tuned weights — under AGPL-3.0. Also rules out `from ultralytics import RTDETR` and any `yolov8*.pt` weights. | Ultralytics licensing; AGPL-3.0 §13 |
| **RF-DETR Nano is Apache-2.0** and runs here: `uv pip install rfdetr` succeeded, weights auto-downloaded with MD5 validation, **median 31.1 ms** on a 640×480 frame (~32 FPS). Dependency tree provably AGPL-clean — `ultralytics` absent across all 62 installed distributions. | **Agent executed this on this Mac** |
| ⚠️ **RF-DETR XL/2XL and `rfdetr_plus` are PML-1.0** — not OSI-approved, requires an active Roboflow agreement, revocable on non-payment. Only Nano/Small/Medium/Large may be used. | Roboflow licensing |
| **Perception cannot be accelerated inside Docker on this Mac.** Neither PyTorch MPS nor ONNX Runtime's CoreML EP is reachable from a Linux container — a containerised detector silently degrades to slow CPU. | Same no-GPU-passthrough constraint as §1 |
| Measured: accelerators are **not automatically faster**. CoreML EP 125.6 ms vs CPU EP 85.8 ms on a detector with in-graph NMS (unsupported ops force graph partitioning); MPS lost to CPU on ssdlite (67.4 vs 50.5 ms). **Always benchmark.** Export with NMS *outside* the graph. | Agent benchmarked on this Mac |
| Flat-ground monocular back-projection verified numerically: round-trip recovered a known ground point with **error 0.000000000 m**. Use bbox **bottom-centre**; reject (never clamp) rays with `d_w[z] >= 0` (at/above horizon). | Agent ran the verification script here |
| **Geolocation error grows ~quadratically with range.** A 1° pitch error costs 0.028 m at 0.8 m but **5.24 m at 13.1 m** (err/range² ≈ 0.025–0.031). Dominant error source is pitch/attitude, then bbox bottom edge, then camera height, then robot pose, then yaw. → document a valid range (~6 m) and reject beyond it. | Agent's measured error-budget sweep |
| Dataset licences: **PKLot** CC BY 4.0 (12,417 imgs, per-space rotated rects), **ACPDS** MIT (293 imgs, disjoint lots per split — the honest generalisation test), **RDD2022** CC BY-SA 4.0 (47k imgs), **Pothole Videos** CC BY 4.0. ⚠️ **Pothole Mix is CC BY-NC 3.0** — excluded. ⚠️ CNRPark+EXT boxes are at 2592×1944 but frames ship at 1000×750 — rescale or every box is wrong by ~2.6×. | Dataset pages, agent-verified |
| MMDetection (last push 2024-08-21) and YOLOX (2025-06-08) are Apache-2.0 but **effectively stale** and will fight torch 2.14 / numpy 2.5. | GitHub |

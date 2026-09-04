# Resume description

Built only from measured results. Every figure traces to a committed artifact:
`apps/runner/results/results.csv` (90 benchmark runs) or `apps/eval/results/results.csv`
(1,209 geolocation samples), both regenerable with `make bench` / `make eval`.

---

## One-line

**ParkScan Swarm** — a simulation-first multi-robot parking-lot inspection system: three
coordinated robots, a deterministic simulator, a PostGIS-backed FastAPI service and a live
React dashboard, built around an honest separation between what was measured and what was not.

## Two-line

Designed and built a simulation-first multi-robot inspection system (Python, FastAPI,
PostGIS, MQTT, React/TypeScript, MapLibre) in which three coordinated robots partition a
parking lot, detect and geolocate defects, and recover from robot failure. Compared three
task-allocation strategies over 30 seeded runs with paired bootstrap confidence intervals,
and reported the null result as null.

## Bullets

- **Chose the architecture from evidence, not habit.** Verified from primary sources that
  ROS 2 has no macOS/arm64 support at any tier and that Docker on macOS gives containers no
  GPU, then designed around a `WorldBackend` adapter so the autonomy is backend-agnostic —
  and shipped a deterministic simulator that runs ~130× real time, which is what made a
  30-seed experiment possible at all.
- **Ran a real experiment, not a demo.** 30 seeds × 3 allocation strategies (greedy,
  Hungarian, sequential-single-item auction) with paired bootstrap 95% CIs. The auction cut
  median makespan by 68.6 s (29/30 wins) and travel by 146.2 m (30/30). Solving each round
  optimally with the Hungarian method did **not** improve makespan — median difference 0.0 s,
  95% CI [−10.6, +21.3] — and that is reported as not significant.
- **Correctness as build gates, not charts.** Across all 90 runs: 100% coverage, zero
  collisions, zero duplicate visits, zero battery depletions, zero allocation churn. Failure
  injection shows an offline robot's zone taken over with its partial work *resumed* rather
  than restarted, and the fleet still completing every zone.
- **Found real bugs by taking the acceptance criteria seriously.** A zero-collision criterion
  exposed that robots were invisible to one another's sensors; a design review found that the
  textbook resource-ordering rule for deadlock could not be used, because measured routes
  acquire lane segments out of id order; a final audit found a fully-tested reservation layer
  that nothing called, and integrating it surfaced a second, unrelated bug — a battery model
  that had been underestimating real travel cost by more than 2x since Phase 2, silently
  passing every benchmark because natural runs never got close enough to the margin to expose
  it.
- **Made honesty structural.** Provenance is a `NOT NULL` column, so the dashboard's
  SIMULATED badge renders from data rather than a hardcoded string; geolocation *rejects*
  rays above the horizon rather than clamping them to a plausible wrong point; and the
  evaluation separates what a simulated detector can measure from what only a real model on a
  named benchmark could — reporting geolocation error binned by range (mean 0.315 m inside the
  6 m gate) because the error grows quadratically and a single number would hide it.

## What to say when asked what is *not* there

Volunteer these; each is documented in the README's Limitations section.

- **It is not a ROS 2 system, yet.** The second backend's gating spike (real-time factor) has
  passed on this host (~1.0); ADR-0008 designs the bridge it needs and that design has been
  verified against a real Gazebo Sim Server and `ros_gz_bridge` (a world SDF generator and a
  Gazebo control wrapper exist, unit-tested), but `Ros2WorldBackend` itself is not built — the
  adapter still has one running implementation.
- **The reservation layer is integrated for charging bays, not for lane segments.** Leases,
  fencing tokens and the aging queue are implemented, unit-tested, and now called by
  `Coordinator._manage_bay_lease` for bay arbitration — the general per-lane-segment case
  remains unintegrated by design scope (the agent, not the coordinator, owns its route).
  Ordinary driving still avoids collisions via sensor-based avoidance; only the final approach
  into a bay, where that avoidance is off, is arbitrated by the lease layer. ADR-0007 has the
  four fixes integration needed and what is still open.
- **The detector reads no pixels.** It turns known ground-truth geometry into
  detection-shaped records, so every perception number measures the pipeline and none of them
  says anything about real-world detection accuracy.
- No physics, no occupancy classification, no auth, no alert UI, no E2E tests, and CI that
  has never run.

## Scale

195 files · ~13k lines of Python and TypeScript · 787 Python + 74 web tests · ruff, mypy
`--strict` and TypeScript strict clean across the repo · 8 architecture decision records,
including two that record what was rejected and why.

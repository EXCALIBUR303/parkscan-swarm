# ADR-0001 — Two simulation backends behind one `WorldBackend` adapter

**Status:** Accepted (Phase 0) · **Supersedes:** nothing

## Context
ROS 2 is unsupported on macOS arm64 at any tier (REP-2000 / per-distro platform tables: macOS is
Tier 3, amd64 only; the arm64 cell is blank). Docker Desktop on macOS provides no GPU to containers,
so a containerised Gazebo renders cameras via Mesa `llvmpipe` on CPU. Perception cannot be
accelerated in a container either — MPS and the CoreML EP are unreachable from a Linux guest — so
the detector must run natively on the host. DDS does not reliably cross the Docker Desktop VM
boundary (host networking is layer-4 only).

## Decision
All logic sits above a narrow `WorldBackend` protocol. `sim-lite` (deterministic pure-Python 2D
kinematic sim) is the default and the MVP. `ros2-gz` (ROS 2 Jazzy + headless Gazebo Harmonic,
`linux/arm64`) is an optional Phase-S backend behind the same interface.

## Rationale
The process split is forced by the platform regardless of simulator choice, so the adapter is the
honest shape of the system rather than a workaround. Decisively: the project's strongest
credibility asset is `make bench` — 30 seeded runs × 3 strategies with bootstrap CIs. At an RTF of
0.2–0.5 with software-rendered cameras that experiment is impossible. The fast deterministic
backend is what turns a demo into an experiment.

## Consequences
- ✅ Runs in <60 s with no Docker; fully unit-testable; reproducible metrics.
- ⚠️ An adapter with one implementation is an unproven claim — `ros2-gz` must actually be attempted.
- ⚠️ Until it runs, the README must not describe this as a ROS 2 system (see ACCEPTANCE H12).
- The boundary is deliberately **not** generic: 2D, metric, REP-103, flat ground, fixed dt. That
  specificity is what stops it becoming a useless abstraction.

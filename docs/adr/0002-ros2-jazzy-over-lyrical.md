# ADR-0002 — ROS 2 Jazzy (not Lyrical, not Kilted) for the optional backend

**Status:** Accepted (Phase 0)

## Context
As of Sept 2026: Lyrical Luth is the newest LTS (EOL May 2031), Jazzy is LTS to May 2029, Kilted
EOLs **December 2026**. `docker.io/library/ros` ships genuine arm64 manifests for all of them, and
the arm64 apt repo has near-complete parity (2638 vs 2642 packages) including `ros_gz` and nav2.

## Decision
Target **Jazzy + Gazebo Harmonic**. Never `osrf/ros:*-simulation` (amd64-only, silently emulated).
Build our own image from `ros:jazzy-ros-base` with `--platform=linux/arm64` explicit so an
accidental amd64 pull fails loudly.

## Rationale
Kilted is excluded outright — shipping a portfolio project on a distro that EOLs in three months
reads as careless. Between Jazzy and Lyrical, Jazzy wins on two counts that matter here: the larger
tutorial/ecosystem surface, and — decisively — **RoboStack supports `robostack-jazzy` on
`osx-arm64`**. That preserves a native Apple-Silicon escape hatch with real GPU rendering if the
llvmpipe RTF proves too slow, which Lyrical does not currently offer.

## Consequences
- Dockerfiles must rewrite `https://packages.ros.org` → `http://` — HTTPS to that host fails from
  this machine (curl exit 60), verified directly.
- Must `apt install libgl1-mesa-dri` explicitly; `gz-rendering-vendor` pulls `libegl1`/`libopengl0`
  but not the DRI driver that provides llvmpipe. World SDF must specify `ogre2` for EGL headless.
- Use `rmw_zenoh_cpp` with a router rather than DDS multicast, which does not work across the VM boundary.

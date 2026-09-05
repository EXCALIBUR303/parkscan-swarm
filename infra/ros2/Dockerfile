# The ros2-gz backend's bridge container (ADR-0002, ADR-0008).
#
# Explicitly linux/arm64, never osrf/ros:*-simulation (amd64-only, silently emulated
# under Docker Desktop -- see ADR-0002). Built FROM ros:jazzy-ros-base rather than a
# pre-baked simulation image so the exact package set is visible here, not hidden
# upstream. packages.ros.org over HTTPS fails from this build environment (curl exit
# 60, verified directly); http works -- both stages that apt-install ROS packages
# rewrite it first.

# ---------------------------------------------------------------------- sdf-builder --
# The world SDF is generated at build time from the committed lot, not hand authored
# (parkscan_ros2.sdf, ADR-0008 §1). That module needs parkscan_core (LotSpec) to run,
# so it is invoked here, in a throwaway stage that never ships, rather than carrying
# parkscan_core and its dependencies into the runtime image just to render one file.
FROM python:3.12-slim AS sdf-builder
WORKDIR /build
RUN pip install --no-cache-dir numpy==2.5.2 'pydantic>=2' pyyaml
COPY packages/parkscan_core/ packages/parkscan_core/
COPY packages/parkscan_ros2/ packages/parkscan_ros2/
COPY sim/lot/lot.yaml sim/lot/lot.yaml
RUN pip install --no-cache-dir --no-deps -e packages/parkscan_core -e packages/parkscan_ros2 && \
    python3 -c "\
import numpy as np; \
from pathlib import Path; \
from parkscan_core.lot import load_lot; \
from parkscan_ros2.sdf import generate_world_sdf; \
lot = load_lot(Path('sim/lot/lot.yaml')); \
rng = np.random.default_rng(0); \
draw = rng.random(len(lot.parking_spaces)); \
occupied = [s.id for s, v in zip(lot.parking_spaces, draw, strict=True) if float(v) < 0.6]; \
poses = [('robot_1', 8.0, 17.5, 0.0), ('robot_2', 8.0, 67.5, 0.0), ('robot_3', 2.5, 17.5, 0.0)]; \
Path('/build/parkscan.sdf').write_text(generate_world_sdf(lot, occupied_space_ids=occupied, robot_poses=poses, dt_s=0.05))"

# ------------------------------------------------------------------------- runtime --
FROM --platform=linux/arm64 ros:jazzy-ros-base AS runtime
RUN sed -i 's|https://packages.ros.org|http://packages.ros.org|g' \
        /etc/apt/sources.list.d/*.list* 2>/dev/null; \
    apt-get update -qq && \
    apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-bridge \
        libgl1-mesa-dri \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Only paho-mqtt: the bridge (packages/parkscan_ros2/src/parkscan_ros2/bridge/) never
# imports parkscan_core or numpy, so this image does not install the full
# parkscan_ros2 package via its pyproject.toml -- that would pull in parkscan-core and
# everything it depends on for a container that never uses it. Only the bridge
# subpackage is copied in below, kept deliberately minimal.
RUN pip install --no-cache-dir --break-system-packages 'paho-mqtt>=2.1,<3'

WORKDIR /opt/parkscan
ENV PYTHONPATH=/opt/parkscan
COPY packages/parkscan_ros2/src/parkscan_ros2/__init__.py parkscan_ros2/__init__.py
COPY packages/parkscan_ros2/src/parkscan_ros2/bridge/ parkscan_ros2/bridge/
COPY --from=sdf-builder /build/parkscan.sdf /opt/parkscan/parkscan.sdf
COPY infra/ros2/entrypoint.sh /opt/parkscan/entrypoint.sh
RUN chmod +x /opt/parkscan/entrypoint.sh

ENTRYPOINT ["/opt/parkscan/entrypoint.sh"]

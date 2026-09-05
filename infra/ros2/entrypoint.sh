#!/usr/bin/env bash
# Starts the three processes this container's whole job is to run, in the order that
# matters: Gazebo (paused, per ADR-0008 §3 -- the backend steps it via GzControl, never
# real-time auto-run), then ros_gz_bridge (needs Gazebo's topics to already exist to
# discover their types), then the Python bridge node (needs both).
#
# Forwards SIGTERM to all three so `docker stop` shuts the container down cleanly
# rather than leaving orphaned children -- a container that hangs on stop is exactly
# the kind of thing this project's own history (a 3-hour hung dev container, found only
# by checking `docker ps`) says not to trust to luck.
#
# No `-u`: found the hard way -- /opt/ros/jazzy/setup.bash itself references
# AMENT_TRACE_SETUP_FILES without a default, so `set -u` makes sourcing it fail
# immediately with "unbound variable" before this script does anything at all. ROS 2's
# own setup scripts are not written to be `set -u`-safe; every variable this script
# reads from the environment already uses a `${VAR:-default}` fallback, so nothing
# here actually depends on unbound-variable detection to be caught.
set -eo pipefail

WORLD_NAME="${WORLD_NAME:-ParkScan_synthetic_lot_A}"
ROBOTS="${ROBOTS:-robot_1,robot_2,robot_3}"
MQTT_HOST="${MQTT_HOST:-mqtt}"
MQTT_PORT="${MQTT_PORT:-1883}"
SDF_PATH="${SDF_PATH:-/opt/parkscan/parkscan.sdf}"

source /opt/ros/jazzy/setup.bash

pids=()
cleanup() {
    trap - TERM INT
    for pid in "${pids[@]}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done
    wait
}
trap cleanup TERM INT

echo "[entrypoint] starting Gazebo (paused) on world '${WORLD_NAME}'"
DISPLAY= LIBGL_ALWAYS_SOFTWARE=1 gz sim -s --headless-rendering "${SDF_PATH}" &
pids+=("$!")

# Gazebo's transport services take a moment to come up; ros_gz_bridge fails fast and
# does not retry if started before they exist.
sleep 5

bridge_args=()
IFS=',' read -ra robot_list <<< "${ROBOTS}"
for robot_id in "${robot_list[@]}"; do
    bridge_args+=(
        "/${robot_id}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"
        "/${robot_id}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        "/${robot_id}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
    )
done

echo "[entrypoint] starting ros_gz_bridge for: ${ROBOTS}"
ros2 run ros_gz_bridge parameter_bridge "${bridge_args[@]}" &
pids+=("$!")

sleep 3

echo "[entrypoint] starting the parkscan bridge node"
python3 -m parkscan_ros2.bridge.node \
    --world-name "${WORLD_NAME}" \
    --robots "${ROBOTS}" \
    --mqtt-host "${MQTT_HOST}" \
    --mqtt-port "${MQTT_PORT}" &
pids+=("$!")

wait -n "${pids[@]}"
exit_code=$?
echo "[entrypoint] one process exited (code ${exit_code}); shutting down the rest"
cleanup
exit "${exit_code}"

"""The rclpy bridge node (ADR-0008 §6). Runs inside the ``ros2-gz`` container only --
this is the one module in the whole system that imports ``rclpy``.

Subscribes to each robot's bridged ROS 2 ``/<robot_id>/odom`` and ``/<robot_id>/scan``
(published by ``ros_gz_bridge``'s ``parameter_bridge``, not this node -- see ADR-0008 §1
for why those are two different bridges), publishes ``/<robot_id>/cmd_vel`` from inbound
MQTT commands, and owns the one :class:`~parkscan_ros2.bridge.gz_control.GzControl`
instance that talks to Gazebo's world control service on this side of the wire.

Kept as thin as this module's job allows: every piece of logic that does not need
``rclpy`` (the wire format, the Gazebo control semantics, the quaternion math) lives in
sibling modules that are unit-tested on the host. This file is the glue, and the one
place those pieces cannot be exercised together without a real ROS 2 + Gazebo stack.
"""

from __future__ import annotations

import argparse
import time
from collections.abc import Callable, Sequence
from typing import TYPE_CHECKING

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from parkscan_ros2.bridge.geometry import quaternion_to_yaw
from parkscan_ros2.bridge.gz_control import GzControl, GzControlError
from parkscan_ros2.bridge.protocol import RobotState as WireRobotState
from parkscan_ros2.bridge.protocol import (
    cmd_topic,
    control_ack_payload,
    control_reset_ack_topic,
    control_reset_topic,
    control_step_ack_topic,
    control_step_topic,
    parse_control_request,
    parse_twist_cmd,
    robot_state_payload,
    state_topic,
)

if TYPE_CHECKING:
    import paho.mqtt.client as mqtt

__all__ = ["BridgeNode", "main"]

_NS_PER_S = 1_000_000_000

_RESET_WARMUP_CHUNK_ITERATIONS = 20
"""Iterations per warm-up chunk after a reset -- 1 s of sim time at the SDF's 0.05 s
physics step. A single iteration is not enough: measured directly, a world reset takes
noticeably more than one physics step before `ros_gz_bridge` relays a fresh sample again
(see the module docstring's note on ``_handle_reset``); this chunk size is a compromise
between not overshooting the wait by much and not looping too many times to get there."""

_RESET_WARMUP_MAX_CHUNKS = 15
"""Bound on total warm-up stepping after a reset: 15 * 20 = 300 iterations, 15 s of sim
time. Generous, but never infinite -- an unresponsive reset should fail loudly (no ack
published, so the backend's own bounded wait raises) rather than step forever."""


class BridgeNode(Node):
    """One node, one world. Constructing it wires everything up; it does not spin
    itself -- :func:`main` owns the ``rclpy.spin`` call so this class stays testable in
    isolation from the executor, at least for whatever a real ROS 2 environment allows
    testing outside a running system (which, per ADR-0008's own honesty about this
    session's verification, was not exercised in this project's CI)."""

    def __init__(
        self,
        robot_ids: Sequence[str],
        world_name: str,
        mqtt_host: str,
        mqtt_port: int,
        *,
        mqtt_client_factory: Callable[[], mqtt.Client] | None = None,
    ) -> None:
        super().__init__("parkscan_bridge")
        self._robot_ids = tuple(robot_ids)
        self._latest_odom: dict[str, Odometry] = {}
        self._latest_scan: dict[str, LaserScan] = {}
        self._cmd_pubs: dict[str, object] = {}

        for robot_id in self._robot_ids:
            self.create_subscription(
                Odometry, f"/{robot_id}/odom", self._odom_callback(robot_id), 10
            )
            self.create_subscription(
                LaserScan, f"/{robot_id}/scan", self._scan_callback(robot_id), 10
            )
            self._cmd_pubs[robot_id] = self.create_publisher(Twist, f"/{robot_id}/cmd_vel", 10)

        self._gz = GzControl(world_name)
        self._gz.start()

        factory = mqtt_client_factory if mqtt_client_factory is not None else _default_client
        self._mqtt = factory()
        self._mqtt.on_message = self._on_mqtt_message  # type: ignore[assignment]
        self._mqtt.connect(mqtt_host, mqtt_port)
        for robot_id in self._robot_ids:
            self._mqtt.subscribe(cmd_topic(robot_id), qos=0)
        self._mqtt.subscribe(control_step_topic(), qos=1)
        self._mqtt.subscribe(control_reset_topic(), qos=1)
        self._mqtt.loop_start()

    # ------------------------------------------------------------------- ROS -> MQTT --
    def _odom_callback(self, robot_id: str) -> Callable[[Odometry], None]:
        def callback(msg: Odometry) -> None:
            self._latest_odom[robot_id] = msg
            self._publish_state(robot_id)

        return callback

    def _scan_callback(self, robot_id: str) -> Callable[[LaserScan], None]:
        def callback(msg: LaserScan) -> None:
            self._latest_scan[robot_id] = msg
            self._publish_state(robot_id)

        return callback

    def _publish_state(self, robot_id: str) -> None:
        odom = self._latest_odom.get(robot_id)
        if odom is None:
            return  # nothing to report yet -- pose is required, ranges are not
        scan = self._latest_scan.get(robot_id)
        ranges = tuple(float(r) for r in scan.ranges) if scan is not None else ()
        q = odom.pose.pose.orientation
        state = WireRobotState(
            stamp_ns=odom.header.stamp.sec * _NS_PER_S + odom.header.stamp.nanosec,
            x_m=odom.pose.pose.position.x,
            y_m=odom.pose.pose.position.y,
            theta_rad=quaternion_to_yaw(q.x, q.y, q.z, q.w),
            v_mps=odom.twist.twist.linear.x,
            omega_radps=odom.twist.twist.angular.z,
            ranges=ranges,
        )
        self._mqtt.publish(state_topic(robot_id), robot_state_payload(state), qos=0)

    # ------------------------------------------------------------------- MQTT -> ROS --
    def _on_mqtt_message(self, _client: object, _userdata: object, msg: object) -> None:
        topic = msg.topic  # type: ignore[attr-defined]
        payload = msg.payload.decode("utf-8")  # type: ignore[attr-defined]

        for robot_id in self._robot_ids:
            if topic == cmd_topic(robot_id):
                self._handle_cmd(robot_id, payload)
                return
        if topic == control_step_topic():
            self._handle_step(payload)
        elif topic == control_reset_topic():
            self._handle_reset(payload)

    def _handle_cmd(self, robot_id: str, payload: str) -> None:
        try:
            v_mps, omega_radps = parse_twist_cmd(payload)
        except ValueError:
            self.get_logger().warning(f"malformed cmd payload for {robot_id}: {payload!r}")
            return
        twist = Twist()
        twist.linear.x = v_mps
        twist.angular.z = omega_radps
        self._cmd_pubs[robot_id].publish(twist)  # type: ignore[attr-defined]

    def _handle_step(self, payload: str) -> None:
        try:
            req = parse_control_request(payload)
        except ValueError:
            self.get_logger().warning(f"malformed step request: {payload!r}")
            return
        if req.iterations is None:
            self.get_logger().warning(f"step request {req.request_id} carried no iterations")
            return
        try:
            self._gz.step(req.iterations)
        except GzControlError as exc:
            self.get_logger().error(f"step request {req.request_id} failed: {exc}")
            return  # no ack published -- the backend's own bounded wait will time out
        self._mqtt.publish(control_step_ack_topic(), control_ack_payload(req.request_id), qos=1)

    def _handle_reset(self, payload: str) -> None:
        """Reset physics, then step in chunks until every robot has actually reported
        fresh state, before acknowledging.

        Found the hard way, running the real system end to end, in two stages:

        1. The world is paused after reset (§3's lockstep design), and Gazebo's sensor
           plugins do not publish anything at all while paused -- so
           `Ros2WorldBackend.reset()`'s own wait for every robot's first fresh state
           (its docstring: an ack alone does not guarantee a pose has arrived) would
           otherwise deadlock forever, since nothing ever arrives until *some* step
           happens, and stepping is a separate call the backend only makes after
           reset() has already returned.
        2. A single warm-up iteration is not enough. Measured directly, with `rclpy`
           entirely out of the picture -- `gz service ... reset` then `gz service ...
           multi_step: 1` then `ros2 topic echo /robot_1/odom --once` -- the echo timed
           out. `reset: {all: true}` evidently costs `ros_gz_bridge` more than one
           physics tick to relay a fresh sample again; stepping in 1-second chunks
           until this node's own already-cached `_latest_odom`/`_latest_scan` actually
           show every robot, rather than guessing a fixed number up front, is what
           makes this robust to however long that genuinely takes.

        Never acknowledges reset if warm-up runs out its bounded budget without every
        robot reporting -- silence here is a clearer failure than an ack that lied
        about the state being fresh.
        """
        try:
            req = parse_control_request(payload)
        except ValueError:
            self.get_logger().warning(f"malformed reset request: {payload!r}")
            return
        try:
            self._gz.reset()
            for _ in range(_RESET_WARMUP_MAX_CHUNKS):
                self._gz.step(_RESET_WARMUP_CHUNK_ITERATIONS)
                time.sleep(0.1)  # let the just-stepped ROS callbacks actually arrive
                if all(r in self._latest_odom for r in self._robot_ids):
                    break
            else:
                self.get_logger().error(
                    f"reset request {req.request_id}: not every robot reported state "
                    f"within {_RESET_WARMUP_MAX_CHUNKS} warm-up chunks"
                )
                return
        except GzControlError as exc:
            self.get_logger().error(f"reset request {req.request_id} failed: {exc}")
            return
        self._mqtt.publish(control_reset_ack_topic(), control_ack_payload(req.request_id), qos=1)

    def close(self) -> None:
        self._mqtt.loop_stop()
        self._gz.stop()


def _default_client() -> mqtt.Client:
    import paho.mqtt.client as mqtt_client
    from paho.mqtt.enums import CallbackAPIVersion

    return mqtt_client.Client(CallbackAPIVersion.VERSION2)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="parkscan_bridge")
    parser.add_argument("--world-name", required=True)
    parser.add_argument("--robots", required=True, help="comma-separated robot ids")
    parser.add_argument("--mqtt-host", default="127.0.0.1")
    parser.add_argument("--mqtt-port", type=int, default=1883)
    args = parser.parse_args(argv)
    robot_ids = tuple(r for r in args.robots.split(",") if r)
    if not robot_ids:
        parser.error("--robots must name at least one robot id")

    rclpy.init()
    node = BridgeNode(robot_ids, args.world_name, args.mqtt_host, args.mqtt_port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

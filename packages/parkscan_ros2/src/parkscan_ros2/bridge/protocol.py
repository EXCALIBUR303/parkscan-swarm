"""The MQTT wire protocol between the bridge and :class:`~parkscan_ros2.backend.Ros2WorldBackend`.

Exactly ADR-0008 §6, as plain functions and dataclasses with no MQTT client, no ``rclpy``,
and no I/O -- so both sides of the wire (the bridge, which imports ``rclpy`` and cannot be
exercised on this host, and the backend, which cannot) can share the *same* encode/decode
code and be tested against each other's exact wire format without either one needing the
other's runtime dependencies.

Topics are internal, implementation-detail channels distinct from `docs/API_CONTRACT.md`'s
app-level contract -- nothing outside this backend should ever subscribe to them.
"""

from __future__ import annotations

import json
from dataclasses import dataclass

__all__ = [
    "BRIDGE_PREFIX",
    "ControlAck",
    "ControlRequest",
    "RobotState",
    "cmd_payload",
    "cmd_topic",
    "control_ack_payload",
    "control_reset_ack_topic",
    "control_reset_request_payload",
    "control_reset_topic",
    "control_step_ack_topic",
    "control_step_request_payload",
    "control_step_topic",
    "parse_control_ack",
    "parse_control_request",
    "parse_robot_state",
    "parse_twist_cmd",
    "robot_state_payload",
    "state_topic",
]

BRIDGE_PREFIX = "parkscan/bridge/v1"


def state_topic(robot_id: str) -> str:
    return f"{BRIDGE_PREFIX}/{robot_id}/state"


def cmd_topic(robot_id: str) -> str:
    return f"{BRIDGE_PREFIX}/{robot_id}/cmd"


def control_step_topic() -> str:
    return f"{BRIDGE_PREFIX}/control/step"


def control_step_ack_topic() -> str:
    return f"{BRIDGE_PREFIX}/control/step_ack"


def control_reset_topic() -> str:
    return f"{BRIDGE_PREFIX}/control/reset"


def control_reset_ack_topic() -> str:
    return f"{BRIDGE_PREFIX}/control/reset_ack"


@dataclass(frozen=True, slots=True)
class RobotState:
    """One robot's latest sensed state, as the bridge relays it. Battery is deliberately
    absent -- ADR-0008 §4 assigns state-of-charge tracking to the backend alone."""

    stamp_ns: int
    x_m: float
    y_m: float
    theta_rad: float
    v_mps: float
    omega_radps: float
    ranges: tuple[float, ...]


def robot_state_payload(state: RobotState) -> str:
    return json.dumps(
        {
            "stamp_ns": state.stamp_ns,
            "pose": {"x_m": state.x_m, "y_m": state.y_m, "theta_rad": state.theta_rad},
            "twist": {"v_mps": state.v_mps, "omega_radps": state.omega_radps},
            "ranges": list(state.ranges),
        }
    )


def parse_robot_state(payload: str | bytes) -> RobotState:
    """Raises:
    ValueError: ``payload`` is not valid JSON or is missing/mistyped a required field.
    """
    try:
        data = json.loads(payload)
        pose = data["pose"]
        twist = data["twist"]
        return RobotState(
            stamp_ns=int(data["stamp_ns"]),
            x_m=float(pose["x_m"]),
            y_m=float(pose["y_m"]),
            theta_rad=float(pose["theta_rad"]),
            v_mps=float(twist["v_mps"]),
            omega_radps=float(twist["omega_radps"]),
            ranges=tuple(float(r) for r in data["ranges"]),
        )
    except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
        msg = f"malformed robot state payload: {payload!r}"
        raise ValueError(msg) from exc


def cmd_payload(v_mps: float, omega_radps: float) -> str:
    return json.dumps({"v_mps": v_mps, "omega_radps": omega_radps})


def parse_twist_cmd(payload: str | bytes) -> tuple[float, float]:
    """Returns ``(v_mps, omega_radps)``.

    Raises:
        ValueError: ``payload`` is not valid JSON or is missing/mistyped a required field.
    """
    try:
        data = json.loads(payload)
        return float(data["v_mps"]), float(data["omega_radps"])
    except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
        msg = f"malformed cmd payload: {payload!r}"
        raise ValueError(msg) from exc


@dataclass(frozen=True, slots=True)
class ControlRequest:
    request_id: str
    iterations: int | None = None
    """Set for a step request, ``None`` for a reset request."""
    seed: int | None = None
    """Set for a reset request, ``None`` for a step request."""


def control_step_request_payload(request_id: str, iterations: int) -> str:
    return json.dumps({"request_id": request_id, "iterations": iterations})


def control_reset_request_payload(request_id: str, seed: int) -> str:
    return json.dumps({"request_id": request_id, "seed": seed})


def parse_control_request(payload: str | bytes) -> ControlRequest:
    """Raises:
    ValueError: ``payload`` is not valid JSON or is missing/mistyped ``request_id``.
    """
    try:
        data = json.loads(payload)
        return ControlRequest(
            request_id=str(data["request_id"]),
            iterations=int(data["iterations"]) if "iterations" in data else None,
            seed=int(data["seed"]) if "seed" in data else None,
        )
    except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
        msg = f"malformed control request payload: {payload!r}"
        raise ValueError(msg) from exc


@dataclass(frozen=True, slots=True)
class ControlAck:
    request_id: str


def control_ack_payload(request_id: str) -> str:
    return json.dumps({"request_id": request_id})


def parse_control_ack(payload: str | bytes) -> ControlAck:
    """Raises:
    ValueError: ``payload`` is not valid JSON or is missing ``request_id``.
    """
    try:
        data = json.loads(payload)
        return ControlAck(request_id=str(data["request_id"]))
    except (json.JSONDecodeError, KeyError, TypeError) as exc:
        msg = f"malformed control ack payload: {payload!r}"
        raise ValueError(msg) from exc

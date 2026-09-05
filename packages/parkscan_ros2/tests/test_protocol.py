"""The bridge<->backend MQTT wire protocol (ADR-0008 §6). Both the bridge (imports
``rclpy``, container-only) and :class:`~parkscan_ros2.backend.Ros2WorldBackend` (host-only)
must agree on this exact format without either one importing the other's runtime. These
tests are the contract they share: encode with one function, decode with its counterpart,
byte for byte."""

from __future__ import annotations

import pytest
from parkscan_ros2.bridge.protocol import (
    ControlAck,
    ControlRequest,
    RobotState,
    cmd_payload,
    cmd_topic,
    control_ack_payload,
    control_reset_ack_topic,
    control_reset_request_payload,
    control_reset_topic,
    control_step_ack_topic,
    control_step_request_payload,
    control_step_topic,
    parse_control_ack,
    parse_control_request,
    parse_robot_state,
    parse_twist_cmd,
    robot_state_payload,
    state_topic,
)


def test_topics_are_namespaced_away_from_the_app_level_contract() -> None:
    """Nothing outside this backend should ever recognise these as its own topics."""
    for topic in (
        state_topic("robot_1"),
        cmd_topic("robot_1"),
        control_step_topic(),
        control_step_ack_topic(),
        control_reset_topic(),
        control_reset_ack_topic(),
    ):
        assert topic.startswith("parkscan/bridge/v1/")
        assert "parkscan/v1/" not in topic  # the app-level contract's own prefix


def test_topics_are_unique_per_robot() -> None:
    assert state_topic("robot_1") != state_topic("robot_2")
    assert cmd_topic("robot_1") != cmd_topic("robot_2")


def test_robot_state_round_trips_exactly() -> None:
    original = RobotState(
        stamp_ns=1_234_500_000,
        x_m=8.0,
        y_m=17.5,
        theta_rad=3.14159,
        v_mps=0.9,
        omega_radps=-0.2,
        ranges=(1.0, 2.5, float("inf"), 0.05),
    )
    parsed = parse_robot_state(robot_state_payload(original))
    assert parsed == original


def test_robot_state_rejects_malformed_payloads() -> None:
    for bad in ("not json", "{}", '{"pose": {}}', '{"stamp_ns": "x"}'):
        with pytest.raises(ValueError, match="malformed robot state"):
            parse_robot_state(bad)


def test_twist_cmd_round_trips_exactly() -> None:
    v, omega = parse_twist_cmd(cmd_payload(0.75, -1.1))
    assert (v, omega) == (0.75, -1.1)


def test_twist_cmd_rejects_malformed_payloads() -> None:
    for bad in ("not json", "{}", '{"v_mps": "fast"}'):
        with pytest.raises(ValueError, match="malformed cmd"):
            parse_twist_cmd(bad)


def test_step_control_request_round_trips() -> None:
    req = parse_control_request(control_step_request_payload("req-1", 20))
    assert req == ControlRequest(request_id="req-1", iterations=20, seed=None)


def test_reset_control_request_round_trips() -> None:
    req = parse_control_request(control_reset_request_payload("req-2", 42))
    assert req == ControlRequest(request_id="req-2", iterations=None, seed=42)


def test_control_request_rejects_malformed_payloads() -> None:
    for bad in ("not json", "{}", '{"iterations": 5}'):
        with pytest.raises(ValueError, match="malformed control request"):
            parse_control_request(bad)


def test_control_ack_round_trips() -> None:
    ack = parse_control_ack(control_ack_payload("req-3"))
    assert ack == ControlAck(request_id="req-3")


def test_control_ack_rejects_malformed_payloads() -> None:
    for bad in ("not json", "{}"):
        with pytest.raises(ValueError, match="malformed control ack"):
            parse_control_ack(bad)

"""``Ros2WorldBackend`` against a fake in-process MQTT client -- no real broker, no
``rclpy``, no Gazebo. The fake plays the bridge's part: it can be told to answer control
requests instantly (the common case these tests exercise) or never (to exercise the
bounded-timeout paths), and to publish robot state on demand, exactly the shape a real
bridge would produce over the wire (``parkscan_ros2.bridge.protocol``)."""

from __future__ import annotations

import threading
from collections.abc import Callable
from pathlib import Path

import pytest
from parkscan_ros2.backend import (
    MAX_TRUSTED_V_MPS,
    Ros2BackendConfig,
    Ros2BackendError,
    Ros2WorldBackend,
)
from parkscan_ros2.bridge.protocol import RobotState as WireRobotState
from parkscan_ros2.bridge.protocol import (
    cmd_topic,
    control_ack_payload,
    control_reset_ack_topic,
    control_step_ack_topic,
    parse_control_request,
    robot_state_payload,
    state_topic,
)

from parkscan_core.lot import LotSpec, load_lot
from parkscan_core.world import WorldBackend

REPO_ROOT = Path(__file__).resolve().parents[3]
LOT_YAML = REPO_ROOT / "sim" / "lot" / "lot.yaml"

ROBOTS = ("robot_1", "robot_2")

#: The real committed lot's first charging bay -- sim/lot/lot.yaml, bay_1.
BAY_1_X_M, BAY_1_Y_M = 8.0, 17.5


class _FakeMqtt:
    """Stands in for the bridge. ``auto_ack`` (default True) answers every control
    request on the same thread, synchronously, as soon as it is published -- a
    deterministic stand-in for "the bridge replied instantly." Set it False to test the
    bounded-timeout paths, where nothing ever answers.

    ``reset()`` genuinely blocks for fresh per-robot state, not just the ack (real
    behaviour: an ack only confirms the physics reset, not that a sensor sample has
    arrived yet) -- so a fake that acks a reset without ever emitting state makes every
    caller's ``reset()`` legitimately time out. ``emit_state_on_reset`` (default True)
    makes the common case work out of the box; set it False for the one test that
    specifically wants that timeout path."""

    def __init__(self, *, auto_ack: bool = True, emit_state_on_reset: bool = True) -> None:
        self.on_message: Callable[[str, str], None] = lambda _t, _p: None
        self.published: list[tuple[str, str, int]] = []
        self.auto_ack = auto_ack
        self.emit_state_on_reset = emit_state_on_reset
        self.connected: tuple[str, int] | None = None
        self.subscriptions: list[str] = []

    def connect(self, host: str, port: int) -> None:
        self.connected = (host, port)

    def subscribe(self, topic: str, qos: int = 0) -> None:
        self.subscriptions.append(topic)

    def publish(self, topic: str, payload: str, qos: int = 0) -> None:
        self.published.append((topic, payload, qos))
        if not self.auto_ack:
            return
        if topic.endswith("/control/step"):
            req = parse_control_request(payload)
            self.on_message(control_step_ack_topic(), control_ack_payload(req.request_id))
        elif topic.endswith("/control/reset"):
            req = parse_control_request(payload)
            self.on_message(control_reset_ack_topic(), control_ack_payload(req.request_id))
            if self.emit_state_on_reset:
                for robot_id in ROBOTS:
                    self.emit_state(robot_id, _state())

    def loop_start(self) -> None:
        pass

    def loop_stop(self) -> None:
        pass

    def emit_state(self, robot_id: str, state: WireRobotState) -> None:
        """Simulate the bridge publishing a fresh state sample for ``robot_id``."""
        self.on_message(state_topic(robot_id), robot_state_payload(state))


def _lot() -> LotSpec:
    return load_lot(LOT_YAML)


def _backend(
    fake: _FakeMqtt, *, config: Ros2BackendConfig | None = None
) -> Ros2WorldBackend:
    return Ros2WorldBackend(ROBOTS, config=config, client_factory=lambda: fake)


def _state(x_m: float = 0.0, y_m: float = 0.0, v_mps: float = 0.0) -> WireRobotState:
    return WireRobotState(
        stamp_ns=0, x_m=x_m, y_m=y_m, theta_rad=0.0, v_mps=v_mps, omega_radps=0.0, ranges=()
    )


def test_satisfies_the_world_backend_protocol() -> None:
    assert isinstance(_backend(_FakeMqtt()), WorldBackend)


def test_connects_and_subscribes_to_every_robots_state_plus_control_acks() -> None:
    fake = _FakeMqtt()
    _backend(fake)
    assert fake.connected == ("127.0.0.1", 1883)
    for robot_id in ROBOTS:
        assert state_topic(robot_id) in fake.subscriptions
    assert control_step_ack_topic() in fake.subscriptions
    assert control_reset_ack_topic() in fake.subscriptions


def test_reset_publishes_a_reset_request_and_waits_for_fresh_state_per_robot() -> None:
    fake = _FakeMqtt()
    backend = _backend(fake)
    backend.reset(seed=0, lot=_lot())

    assert backend.now_ns() == 0
    for robot_id in ROBOTS:
        frame = backend.sense(robot_id)
        assert frame.pose.x_m == 0.0


def test_reset_seeds_initial_soc_reproducibly_from_the_seed() -> None:
    def make() -> tuple[_FakeMqtt, Ros2WorldBackend]:
        fake = _FakeMqtt()
        return fake, _backend(fake)

    _, backend_a = make()
    backend_a.reset(seed=7, lot=_lot())
    soc_a = {r: backend_a.sense(r).battery_soc for r in ROBOTS}

    _, backend_b = make()
    backend_b.reset(seed=7, lot=_lot())
    soc_b = {r: backend_b.sense(r).battery_soc for r in ROBOTS}

    assert soc_a == soc_b


def test_reset_raises_if_the_ack_never_arrives() -> None:
    fake = _FakeMqtt(auto_ack=False)
    backend = _backend(fake, config=Ros2BackendConfig(control_timeout_s=0.05))
    with pytest.raises(Ros2BackendError, match="never acknowledged"):
        backend.reset(seed=0, lot=_lot())


def test_reset_raises_if_acked_but_no_robot_ever_reports_state() -> None:
    fake = _FakeMqtt(emit_state_on_reset=False)  # acks the reset, but never emits state
    backend = _backend(fake, config=Ros2BackendConfig(control_timeout_s=0.05))
    with pytest.raises(Ros2BackendError, match="fresh state"):
        backend.reset(seed=0, lot=_lot())


def test_step_rejects_a_dt_that_does_not_match_the_configured_tick() -> None:
    backend = _backend(_FakeMqtt())
    with pytest.raises(ValueError, match="dt_s"):
        backend.step(0.1)


def test_step_sends_the_correct_iteration_count_and_advances_the_clock() -> None:
    fake = _FakeMqtt()
    backend = _backend(
        fake, config=Ros2BackendConfig(dt_s=0.05, world_step_size_s=0.01)
    )
    backend.step(0.05)

    step_requests = [p for t, p, _ in fake.published if t.endswith("/control/step")]
    assert len(step_requests) == 1
    req = parse_control_request(step_requests[0])
    assert req.iterations == 5  # 0.05 / 0.01
    assert backend.now_ns() == round(0.05 * 1_000_000_000)


def test_step_raises_if_never_acknowledged() -> None:
    fake = _FakeMqtt(auto_ack=False)
    backend = _backend(fake, config=Ros2BackendConfig(control_timeout_s=0.05))
    with pytest.raises(Ros2BackendError, match="never acknowledged"):
        backend.step(0.05)


def test_sense_raises_before_any_state_has_been_reported() -> None:
    backend = _backend(_FakeMqtt())
    with pytest.raises(Ros2BackendError, match="no state has been reported"):
        backend.sense("robot_1")


def test_actuate_publishes_the_commanded_twist_on_the_robots_cmd_topic() -> None:
    fake = _FakeMqtt()
    backend = _backend(fake)
    from parkscan_core.types import TwistCmd

    backend.actuate("robot_1", TwistCmd(v_mps=0.5, omega_radps=-0.1))

    cmds = [(t, p) for t, p, _ in fake.published if t == cmd_topic("robot_1")]
    assert len(cmds) == 1
    assert '"v_mps": 0.5' in cmds[0][1]
    assert '"omega_radps": -0.1' in cmds[0][1]


def test_a_garbage_startup_velocity_spike_is_clamped_before_draining_battery() -> None:
    """The exact quirk ADR-0008 §4 found: a stationary robot's odometry reported ~160
    m/s for its first several samples after a reset. An unclamped reading would drain
    the battery as if the robot were moving at 160 m/s; clamped, it drains as if moving
    at MAX_TRUSTED_V_MPS -- still conservative, but not wildly wrong."""
    fake = _FakeMqtt()
    backend = _backend(fake, config=Ros2BackendConfig(dt_s=0.05, world_step_size_s=0.05))
    backend.reset(seed=0, lot=_lot())
    soc_before = backend.sense("robot_1").battery_soc

    fake.emit_state("robot_1", _state(v_mps=160.0))
    fake.emit_state("robot_2", _state(v_mps=0.0))
    backend.step(0.05)

    from parkscan_core.battery import BatteryParams, drain_soc

    expected = drain_soc(soc_before, MAX_TRUSTED_V_MPS, 0.05, BatteryParams())
    assert backend.sense("robot_1").battery_soc == pytest.approx(expected)


def test_a_robot_at_a_charging_bay_charges_instead_of_draining() -> None:
    fake = _FakeMqtt()
    backend = _backend(fake, config=Ros2BackendConfig(dt_s=0.05, world_step_size_s=0.05))
    backend.reset(seed=0, lot=_lot())

    fake.emit_state("robot_1", _state(x_m=BAY_1_X_M, y_m=BAY_1_Y_M, v_mps=0.0))
    fake.emit_state("robot_2", _state(x_m=100.0, y_m=90.0, v_mps=0.0))
    soc_1_before = backend.sense("robot_1").battery_soc
    soc_2_before = backend.sense("robot_2").battery_soc
    backend.step(0.05)

    assert backend.sense("robot_1").battery_soc > soc_1_before  # docked: charges
    assert backend.sense("robot_2").battery_soc <= soc_2_before  # not docked: drains/flat


def test_config_rejects_a_dt_that_is_not_an_exact_multiple_of_world_step_size() -> None:
    with pytest.raises(ValueError, match="exact multiple"):
        Ros2BackendConfig(dt_s=0.05, world_step_size_s=0.03)


def test_robot_ids_must_be_non_empty() -> None:
    with pytest.raises(ValueError, match="non-empty"):
        Ros2WorldBackend((), client_factory=lambda: _FakeMqtt())


def test_concurrent_acks_are_matched_to_the_right_request_by_id() -> None:
    """Two step() calls in quick succession (from two threads, as a stress case) must
    each unblock on their own ack, never an unrelated one."""
    fake = _FakeMqtt()
    backend = _backend(fake)
    errors: list[Exception] = []

    def do_step() -> None:
        try:
            backend.step(0.05)
        except Exception as exc:
            errors.append(exc)

    threads = [threading.Thread(target=do_step) for _ in range(5)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=2.0)

    assert not errors
    assert backend.now_ns() == round(5 * 0.05 * 1_000_000_000)

"""``Ros2WorldBackend``: the host-side half of ADR-0008. Implements
:class:`~parkscan_core.world.WorldBackend` by talking MQTT to the bridge (`bridge/node.py`,
container-only) -- never ``rclpy``, which cannot run on this host at all.

Every design choice here traces to a specific ADR-0008 section:

* **Timing (§3).** ``step(dt)`` publishes a bounded control request and blocks for its ack
  before returning, exactly mirroring what :class:`~parkscan_ros2.bridge.gz_control.GzControl`
  does on the bridge's side of the same request -- wall-clock independent by construction on
  both ends of the wire, not just one.
* **Battery (§4).** This class, not the bridge, owns per-robot state of charge, using
  :mod:`parkscan_core.battery`'s pure proxy -- the same one ``SimLite`` uses, so there is
  exactly one battery model in the system regardless of which backend a run picks.
* **The velocity warm-up quirk (§4, the third empirical finding).** Reported ``v_mps`` is
  clamped to ``[0, MAX_TRUSTED_V_MPS]`` before it ever reaches ``drain_soc`` -- a stationary
  robot's odometry was watched reporting 160 m/s for its first several samples after a reset.
* **Reproducibility (§5).** ``reset(seed, lot)`` seeds initial state of charge only. Physical
  layout (occupied stalls, robot start poses) is fixed in the one committed SDF and does not
  vary by seed -- this backend is not, and does not claim to be, as reproducible as ``SimLite``.
"""

from __future__ import annotations

import math
import threading
import uuid
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Protocol

from parkscan_core.battery import BatteryParams, charge_soc, drain_soc
from parkscan_core.lot import LotSpec
from parkscan_core.types import Pose2D, RobotId, SensorFrame, TwistCmd
from parkscan_ros2.bridge.protocol import (
    cmd_payload,
    cmd_topic,
    control_reset_ack_topic,
    control_reset_request_payload,
    control_reset_topic,
    control_step_ack_topic,
    control_step_request_payload,
    control_step_topic,
    parse_control_ack,
    parse_robot_state,
    state_topic,
)

__all__ = [
    "MAX_TRUSTED_V_MPS",
    "MqttClient",
    "Ros2BackendConfig",
    "Ros2BackendError",
    "Ros2WorldBackend",
]

#: Mirrors ``parkscan_agent.navigation.NavConfig.max_v_mps`` exactly (duplicated as a plain
#: constant, not imported -- importing the agent layer from a backend would invert the
#: dependency ADR-0001 establishes: backends sit below autonomy, never the reverse).
MAX_TRUSTED_V_MPS: float = 1.2


class MqttClient(Protocol):
    """The narrow slice of an MQTT client this backend needs. Real implementations wrap
    ``paho-mqtt``; tests inject a fake satisfying this same shape -- neither a real broker
    nor ``rclpy`` is needed to exercise this class's logic."""

    on_message: Callable[[str, str], None]

    def connect(self, host: str, port: int) -> None: ...
    def subscribe(self, topic: str, qos: int = 0) -> None: ...
    def publish(self, topic: str, payload: str, qos: int = 0) -> None: ...
    def loop_start(self) -> None: ...
    def loop_stop(self) -> None: ...


class Ros2BackendError(RuntimeError):
    """A control request to the bridge was never acknowledged, or the bridge never reported
    a robot's state at all. Raised rather than hanging forever -- see
    :mod:`parkscan_ros2.bridge.gz_control` for why an unbounded wait here would repeat a
    mistake this project already paid for once."""


@dataclass(frozen=True, slots=True)
class Ros2BackendConfig:
    mqtt_host: str = "127.0.0.1"
    mqtt_port: int = 1883
    dt_s: float = 0.05
    """Must equal the fixed tick the caller advances by every :meth:`Ros2WorldBackend.step`
    call, the same discipline ``SimLite.step`` enforces."""
    world_step_size_s: float = 0.05
    """The Gazebo world's own physics ``max_step_size`` (see ``parkscan_ros2.sdf``). Must
    evenly divide ``dt_s``; ``dt_s / world_step_size_s`` is how many Gazebo iterations one
    :meth:`~Ros2WorldBackend.step` call requests."""
    battery: BatteryParams = field(default_factory=BatteryParams)
    charge_dock_radius_m: float = 1.0
    """Mirrors ``parkscan_sim.config.SimConfig.charge_dock_radius_m``'s default -- how close
    to a charging bay counts as docked, for charge-vs-drain purposes."""
    control_timeout_s: float = 10.0
    """Bound on every wait for a bridge ack or a fresh post-reset state report. Generous, but
    never infinite -- see :class:`Ros2BackendError`."""

    def __post_init__(self) -> None:
        if self.dt_s <= 0.0:
            msg = f"dt_s must be positive, got {self.dt_s}"
            raise ValueError(msg)
        if self.world_step_size_s <= 0.0:
            msg = f"world_step_size_s must be positive, got {self.world_step_size_s}"
            raise ValueError(msg)
        ratio = self.dt_s / self.world_step_size_s
        if not math.isclose(ratio, round(ratio), rel_tol=1e-9):
            msg = (
                f"dt_s ({self.dt_s}) must be an exact multiple of world_step_size_s "
                f"({self.world_step_size_s}), got ratio {ratio}"
            )
            raise ValueError(msg)
        if self.charge_dock_radius_m <= 0.0:
            msg = f"charge_dock_radius_m must be positive, got {self.charge_dock_radius_m}"
            raise ValueError(msg)
        if self.control_timeout_s <= 0.0:
            msg = f"control_timeout_s must be positive, got {self.control_timeout_s}"
            raise ValueError(msg)


class Ros2WorldBackend:
    """Satisfies :class:`~parkscan_core.world.WorldBackend` over MQTT. See the module
    docstring for how each method traces back to ADR-0008."""

    def __init__(
        self,
        robot_ids: Sequence[RobotId],
        *,
        config: Ros2BackendConfig | None = None,
        client_factory: Callable[[], MqttClient] | None = None,
    ) -> None:
        self._robot_ids: tuple[RobotId, ...] = tuple(robot_ids)
        if not self._robot_ids:
            msg = "robot_ids must be non-empty"
            raise ValueError(msg)
        self._cfg = config if config is not None else Ros2BackendConfig()
        self._iterations_per_step = round(self._cfg.dt_s / self._cfg.world_step_size_s)
        self._dt_ns = round(self._cfg.dt_s * 1_000_000_000)

        self._tick = 0
        self._lot: LotSpec | None = None
        self._soc: dict[RobotId, float] = dict.fromkeys(self._robot_ids, 1.0)
        self._latest_pose: dict[RobotId, Pose2D] = {}
        self._latest_twist: dict[RobotId, TwistCmd] = {}
        self._latest_ranges: dict[RobotId, tuple[float, ...]] = {}
        self._latest_stamp_ns: dict[RobotId, int] = {}

        self._lock = threading.Lock()
        self._pending_acks: dict[str, threading.Event] = {}
        self._fresh_state_event = threading.Event()

        factory = client_factory if client_factory is not None else _default_mqtt_client
        self._client = factory()
        self._client.on_message = self._on_message
        self._client.connect(self._cfg.mqtt_host, self._cfg.mqtt_port)
        for robot_id in self._robot_ids:
            self._client.subscribe(state_topic(robot_id), qos=0)
        self._client.subscribe(control_step_ack_topic(), qos=1)
        self._client.subscribe(control_reset_ack_topic(), qos=1)
        self._client.loop_start()

    # ---------------------------------------------------------------- MQTT plumbing --
    def _on_message(self, topic: str, payload: str) -> None:
        if topic in (control_step_ack_topic(), control_reset_ack_topic()):
            try:
                ack = parse_control_ack(payload)
            except ValueError:
                return
            with self._lock:
                event = self._pending_acks.get(ack.request_id)
            if event is not None:
                event.set()
            return

        for robot_id in self._robot_ids:
            if topic != state_topic(robot_id):
                continue
            try:
                state = parse_robot_state(payload)
            except ValueError:
                return
            v_mps = max(0.0, min(state.v_mps, MAX_TRUSTED_V_MPS))
            with self._lock:
                self._latest_pose[robot_id] = Pose2D(
                    x_m=state.x_m, y_m=state.y_m, theta_rad=state.theta_rad
                )
                self._latest_twist[robot_id] = TwistCmd(
                    v_mps=v_mps, omega_radps=state.omega_radps
                )
                self._latest_ranges[robot_id] = state.ranges
                self._latest_stamp_ns[robot_id] = state.stamp_ns
                if all(r in self._latest_pose for r in self._robot_ids):
                    self._fresh_state_event.set()
            return

    def _register_ack(self, request_id: str) -> threading.Event:
        """Register the wait *before* publishing the request it is for.

        Publishing first and registering after is a real race, not just a test
        artifact: if the bridge (or, in a test, a fake standing in for it) replies
        fast enough, ``_on_message`` can run and find no one listening yet, dropping
        the ack -- and then the wait that registers a moment too late blocks for the
        full timeout waiting for an ack that already came and went.
        """
        event = threading.Event()
        with self._lock:
            self._pending_acks[request_id] = event
        return event

    def _wait_for_ack(self, request_id: str, event: threading.Event, what: str) -> None:
        try:
            if not event.wait(self._cfg.control_timeout_s):
                msg = f"{what} (request {request_id}) was never acknowledged by the bridge"
                raise Ros2BackendError(msg)
        finally:
            with self._lock:
                self._pending_acks.pop(request_id, None)

    # --------------------------------------------------------------- WorldBackend API --
    def reset(self, seed: int, lot: LotSpec) -> None:
        """See the module docstring: only initial state of charge is seed-derived here.

        Blocks until the bridge acknowledges the physics reset *and* every robot has
        reported at least one fresh state sample -- an ack alone does not guarantee a
        pose has been received yet, since the bridge's sensor callbacks fire
        independently of the control acknowledgement.
        """
        import numpy as np  # local: this is the only place a seeded RNG is needed

        self._lot = lot
        rng = np.random.default_rng(seed)
        socs = rng.uniform(0.2, 1.0, size=len(self._robot_ids))

        with self._lock:
            self._latest_pose.clear()
            self._latest_twist.clear()
            self._latest_ranges.clear()
            self._latest_stamp_ns.clear()
            self._fresh_state_event.clear()
            self._soc = dict(zip(self._robot_ids, (float(s) for s in socs), strict=True))
        self._tick = 0

        request_id = str(uuid.uuid4())
        event = self._register_ack(request_id)
        self._client.publish(
            control_reset_topic(), control_reset_request_payload(request_id, seed), qos=1
        )
        self._wait_for_ack(request_id, event, "reset")

        if not self._fresh_state_event.wait(self._cfg.control_timeout_s):
            msg = (
                "reset acknowledged, but not every robot reported a fresh state within "
                f"{self._cfg.control_timeout_s}s"
            )
            raise Ros2BackendError(msg)

    def step(self, dt: float) -> None:
        if not math.isclose(dt, self._cfg.dt_s, rel_tol=1e-9):
            msg = f"step({dt}) but this backend is configured for dt_s={self._cfg.dt_s}"
            raise ValueError(msg)

        request_id = str(uuid.uuid4())
        event = self._register_ack(request_id)
        self._client.publish(
            control_step_topic(),
            control_step_request_payload(request_id, self._iterations_per_step),
            qos=1,
        )
        self._wait_for_ack(request_id, event, "step")
        self._tick += 1

        with self._lock:
            poses = dict(self._latest_pose)
            twists = dict(self._latest_twist)
        for robot_id in self._robot_ids:
            pose = poses.get(robot_id)
            twist = twists.get(robot_id)
            if pose is None or twist is None:
                continue  # no sample yet this run; soc simply does not move this tick
            docked = self._is_docked(pose)
            with self._lock:
                self._soc[robot_id] = (
                    charge_soc(self._soc[robot_id], dt, self._cfg.battery)
                    if docked
                    else drain_soc(self._soc[robot_id], twist.v_mps, dt, self._cfg.battery)
                )

    def _is_docked(self, pose: Pose2D) -> bool:
        if self._lot is None:
            return False
        return any(
            math.hypot(pose.x_m - bay.position[0], pose.y_m - bay.position[1])
            <= self._cfg.charge_dock_radius_m
            for bay in self._lot.charging_bays
        )

    def now_ns(self) -> int:
        return self._tick * self._dt_ns

    def robots(self) -> Sequence[RobotId]:
        return self._robot_ids

    def sense(self, robot: RobotId) -> SensorFrame:
        with self._lock:
            pose = self._latest_pose.get(robot)
            twist = self._latest_twist.get(robot)
            ranges = self._latest_ranges.get(robot, ())
            stamp_ns = self._latest_stamp_ns.get(robot, self.now_ns())
            soc = self._soc[robot]
        if pose is None or twist is None:
            msg = f"no state has been reported for {robot!r} yet -- was reset() called?"
            raise Ros2BackendError(msg)
        return SensorFrame(
            stamp_ns=stamp_ns,
            pose=pose,
            twist=twist,
            battery_soc=soc,
            ranges=ranges,
            camera=None,  # ADR-0008 Scope: camera parity is out of scope for this backend
        )

    def actuate(self, robot: RobotId, cmd: TwistCmd) -> None:
        self._client.publish(cmd_topic(robot), cmd_payload(cmd.v_mps, cmd.omega_radps), qos=0)

    def close(self) -> None:
        """Stop the MQTT client's network thread. Not part of ``WorldBackend`` -- callers
        that construct this backend directly (rather than through a context that already
        manages its lifetime) should call this when done with a run."""
        self._client.loop_stop()


def _default_mqtt_client() -> MqttClient:
    import paho.mqtt.client as mqtt
    from paho.mqtt.enums import CallbackAPIVersion

    client = mqtt.Client(CallbackAPIVersion.VERSION2)
    return _PahoAdapter(client)


class _PahoAdapter:
    """Adapts ``paho-mqtt``'s callback shape (``on_message(client, userdata, msg)``, a
    ``msg`` object with ``.topic``/``.payload``) to :class:`MqttClient`'s simpler
    ``(topic, payload) -> None``, so nothing above this module needs to know paho's API."""

    def __init__(self, client: object) -> None:
        self._client = client
        self.on_message: Callable[[str, str], None] = lambda _t, _p: None
        client.on_message = self._dispatch  # type: ignore[attr-defined]

    def _dispatch(self, _client: object, _userdata: object, msg: object) -> None:
        topic = msg.topic  # type: ignore[attr-defined]
        payload = msg.payload.decode("utf-8")  # type: ignore[attr-defined]
        self.on_message(topic, payload)

    def connect(self, host: str, port: int) -> None:
        self._client.connect(host, port)  # type: ignore[attr-defined]

    def subscribe(self, topic: str, qos: int = 0) -> None:
        self._client.subscribe(topic, qos=qos)  # type: ignore[attr-defined]

    def publish(self, topic: str, payload: str, qos: int = 0) -> None:
        self._client.publish(topic, payload, qos=qos)  # type: ignore[attr-defined]

    def loop_start(self) -> None:
        self._client.loop_start()  # type: ignore[attr-defined]

    def loop_stop(self) -> None:
        self._client.loop_stop()  # type: ignore[attr-defined]

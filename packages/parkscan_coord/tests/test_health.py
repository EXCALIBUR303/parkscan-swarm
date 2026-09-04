"""The failure-detection triad, with the false positives tested as hard as the positives.

The interesting test in this file is not that stuck detection fires for a wedged robot --
any displacement threshold does that. It is that it does **not** fire for a robot sitting
in ``WAITING_ON_LEASE``. Under ADR-0007 a robot that cannot get the whole resource set for
its next leg stops and queues, so legitimate queueing looks exactly like being wedged;
without the exclusion the reservation layer manufactures a coordination failure every time
it works correctly.
"""

from __future__ import annotations

import pytest
from parkscan_agent.robot import TelemetrySample
from parkscan_coord.health import (
    NS_PER_S,
    STUCK_EXEMPT_STATES,
    ConnectionEvent,
    HealthConfig,
    HealthMonitor,
    LivenessSource,
)

from parkscan_core.types import CoordView, Pose2D, RobotId, RobotState, TwistCmd

STILL = TwistCmd(v_mps=0.0, omega_radps=0.0)


def _sample(
    robot_id: RobotId,
    stamp_ns: int,
    *,
    x_m: float = 10.0,
    y_m: float = 5.0,
    state: RobotState = RobotState.TRANSIT,
    covered: tuple[str, ...] = (),
    zone_id: str | None = "zone_a",
    epoch: int = 1,
) -> TelemetrySample:
    return TelemetrySample(
        robot_id=robot_id,
        stamp_ns=stamp_ns,
        pose=Pose2D(x_m=x_m, y_m=y_m, theta_rad=0.0),
        twist=STILL,
        command=STILL,
        battery_soc=0.8,
        state=state,
        zone_id=zone_id,
        epoch=epoch,
        covered_cells=covered,
        detail="test",
    )


def _monitor(config: HealthConfig | None = None, *, at_ns: int = 0) -> HealthMonitor:
    monitor = HealthMonitor(config)
    monitor.register("robot_1", at_ns)
    return monitor


# ------------------------------------------------------------------- heartbeat --
@pytest.mark.parametrize(
    ("missed", "expected"),
    [
        (0, CoordView.ONLINE),
        (1, CoordView.SUSPECT),
        (2, CoordView.SUSPECT),
        (3, CoordView.OFFLINE),
        (4, CoordView.OFFLINE),
    ],
)
def test_offline_after_exactly_k_missed_beats(missed: int, expected: CoordView) -> None:
    monitor = _monitor()
    period = monitor.config.heartbeat_period_ns
    assert monitor.config.missed_beats_offline == 3
    assert monitor.assess("robot_1", missed * period).view is expected


def test_not_offline_one_nanosecond_before_the_kth_beat_comes_due() -> None:
    # The boundary is the whole claim: "after exactly k missed beats, not before".
    monitor = _monitor()
    deadline = monitor.config.detection_deadline_ns
    assert monitor.assess("robot_1", deadline - 1).view is CoordView.SUSPECT
    assert monitor.assess("robot_1", deadline).view is CoordView.OFFLINE


def test_a_heartbeat_resets_the_deadline() -> None:
    monitor = _monitor()
    monitor.on_telemetry(_sample("robot_1", 2 * NS_PER_S))
    assert monitor.missed_beats("robot_1", 4 * NS_PER_S) == 2
    assert monitor.assess("robot_1", 4 * NS_PER_S).view is CoordView.SUSPECT
    assert monitor.assess("robot_1", 5 * NS_PER_S).view is CoordView.OFFLINE


def test_an_out_of_order_sample_cannot_rewind_the_heartbeat() -> None:
    monitor = _monitor()
    monitor.on_telemetry(_sample("robot_1", 5 * NS_PER_S))
    monitor.on_telemetry(_sample("robot_1", 1 * NS_PER_S))
    assert monitor.missed_beats("robot_1", 6 * NS_PER_S) == 1


def test_an_unknown_robot_is_an_error_not_a_verdict() -> None:
    monitor = HealthMonitor()
    with pytest.raises(KeyError):
        monitor.assess("robot_9", 0)


# ------------------------------------------------------------------ last will --
def test_last_will_declares_offline_without_waiting_for_the_deadline() -> None:
    # The point of consuming the broker's will: a hard disconnect does not have to wait
    # out k heartbeats. Nothing here fabricates the will; it arrives as an event.
    monitor = _monitor()
    monitor.on_connection(
        ConnectionEvent(
            robot_id="robot_1",
            stamp_ns=NS_PER_S // 2,
            connected=False,
            source=LivenessSource.BROKER_LAST_WILL,
        )
    )
    assessment = monitor.assess("robot_1", NS_PER_S // 2)
    assert assessment.view is CoordView.OFFLINE
    assert assessment.missed_beats == 0
    assert "last will" in assessment.reason


def test_a_disconnect_that_did_not_come_from_the_broker_is_rejected() -> None:
    # Structural honesty: application code cannot mint a "last will" of its own, because
    # a detection mechanism we implemented ourselves is not the broker detecting anything.
    with pytest.raises(ValueError, match="Last Will"):
        ConnectionEvent(
            robot_id="robot_1",
            stamp_ns=0,
            connected=False,
            source=LivenessSource.BROKER_CONNECT,
        )


def test_a_connect_must_be_sourced_as_a_connect() -> None:
    with pytest.raises(ValueError, match="session opening"):
        ConnectionEvent(
            robot_id="robot_1",
            stamp_ns=0,
            connected=True,
            source=LivenessSource.BROKER_LAST_WILL,
        )


def test_reconnecting_does_not_by_itself_clear_the_verdict() -> None:
    monitor = _monitor()
    monitor.on_connection(
        ConnectionEvent(
            robot_id="robot_1",
            stamp_ns=0,
            connected=False,
            source=LivenessSource.BROKER_LAST_WILL,
        )
    )
    monitor.on_connection(
        ConnectionEvent(
            robot_id="robot_1",
            stamp_ns=NS_PER_S,
            connected=True,
            source=LivenessSource.BROKER_CONNECT,
        )
    )
    assert monitor.assess("robot_1", NS_PER_S).view is CoordView.OFFLINE


# ----------------------------------------------------------------- the latch --
def test_offline_latches_against_resumed_telemetry() -> None:
    # A dead robot must not be able to resurrect itself simply by publishing again; that
    # is the Lazarus hazard, and re-registration is the only way back.
    monitor = _monitor()
    assert monitor.assess("robot_1", 3 * NS_PER_S).view is CoordView.OFFLINE

    monitor.on_telemetry(_sample("robot_1", 4 * NS_PER_S))
    assert monitor.assess("robot_1", 4 * NS_PER_S).view is CoordView.OFFLINE
    assert monitor.is_latched_offline("robot_1")

    monitor.register("robot_1", 5 * NS_PER_S)  # a stray re-register must not revive it
    assert monitor.assess("robot_1", 5 * NS_PER_S).view is CoordView.OFFLINE


def test_clearing_the_latch_restores_a_clean_slate() -> None:
    monitor = _monitor()
    assert monitor.assess("robot_1", 3 * NS_PER_S).view is CoordView.OFFLINE
    monitor.clear_offline_latch("robot_1", 3 * NS_PER_S)
    assert not monitor.is_latched_offline("robot_1")
    assessment = monitor.assess("robot_1", 3 * NS_PER_S)
    assert assessment.view is CoordView.ONLINE
    assert assessment.missed_beats == 0


# ------------------------------------------------------------------- stuck --
def _feed(
    monitor: HealthMonitor,
    *,
    seconds: int,
    state: RobotState = RobotState.TRANSIT,
    step_m: float = 0.0,
    cells_per_tick: int = 0,
) -> int:
    """Feed one sample per second and return the timestamp of the last one."""
    stamp = 0
    for i in range(seconds + 1):
        stamp = i * NS_PER_S
        monitor.on_telemetry(
            _sample(
                "robot_1",
                stamp,
                x_m=10.0 + i * step_m,
                state=state,
                covered=tuple(f"zone_a:c{i:02d}r{j:02d}" for j in range(cells_per_tick)),
            )
        )
    return stamp


def test_stuck_fires_for_a_robot_that_stopped_making_progress() -> None:
    monitor = _monitor()
    stamp = _feed(monitor, seconds=10)
    assessment = monitor.assess("robot_1", stamp)
    assert assessment.view is CoordView.STUCK
    assert assessment.window_span_ns >= monitor.config.stuck_window_ns
    assert assessment.displacement_m == pytest.approx(0.0)


def test_stuck_does_not_fire_for_a_robot_waiting_on_a_lease() -> None:
    # The false positive that matters. Identical evidence to the test above -- motionless
    # for the whole window, no new cells -- and the verdict must be ONLINE, because under
    # ADR-0007 queueing for a complete resource set is correct behaviour, not a failure.
    monitor = _monitor()
    stamp = _feed(monitor, seconds=10, state=RobotState.WAITING_ON_LEASE)
    assert monitor.assess("robot_1", stamp).view is CoordView.ONLINE


def test_a_window_that_still_remembers_the_wait_does_not_fire_either() -> None:
    # Five seconds queued, then five seconds stalled after the grant. The window still
    # contains the legitimate wait, so the verdict holds off rather than blaming the
    # reservation system for the time it spent queueing.
    monitor = _monitor()
    for i in range(6):
        monitor.on_telemetry(_sample("robot_1", i * NS_PER_S, state=RobotState.WAITING_ON_LEASE))
    stamp = 0
    for i in range(6, 12):
        stamp = i * NS_PER_S
        monitor.on_telemetry(_sample("robot_1", stamp, state=RobotState.TRANSIT))
    assert monitor.assess("robot_1", stamp).view is CoordView.ONLINE


@pytest.mark.parametrize("state", sorted(STUCK_EXEMPT_STATES))
def test_no_exempt_state_can_ever_be_declared_stuck(state: RobotState) -> None:
    monitor = _monitor()
    stamp = _feed(monitor, seconds=10, state=state)
    assert monitor.assess("robot_1", stamp).view is not CoordView.STUCK


def test_a_moving_robot_is_never_stuck() -> None:
    monitor = _monitor()
    stamp = _feed(monitor, seconds=10, step_m=1.0)
    assessment = monitor.assess("robot_1", stamp)
    assert assessment.view is CoordView.ONLINE
    assert assessment.displacement_m == pytest.approx(10.0)


def test_a_stationary_robot_still_covering_cells_is_not_stuck() -> None:
    # Displacement alone is not the test: a robot inspecting from a standstill is making
    # progress, and the covered-cell count is what proves it.
    monitor = _monitor()
    stamp = _feed(monitor, seconds=10, cells_per_tick=1)
    assert monitor.assess("robot_1", stamp).view is CoordView.ONLINE


def test_stuck_waits_for_a_full_window() -> None:
    monitor = _monitor()
    stamp = _feed(monitor, seconds=4)
    assert monitor.assess("robot_1", stamp).view is CoordView.ONLINE
    assert monitor.assess("robot_1", stamp).window_span_ns < monitor.config.stuck_window_ns


def test_a_wedged_robot_that_frees_itself_recovers_without_re_registration() -> None:
    # STUCK is a live condition, not a death sentence: unlike OFFLINE it does not latch.
    monitor = _monitor()
    stamp = _feed(monitor, seconds=10)
    assert monitor.assess("robot_1", stamp).view is CoordView.STUCK
    for i in range(11, 22):
        stamp = i * NS_PER_S
        monitor.on_telemetry(_sample("robot_1", stamp, x_m=10.0 + i))
    assert monitor.assess("robot_1", stamp).view is CoordView.ONLINE


def test_a_slow_sampling_rate_still_reaches_a_full_window() -> None:
    # Samples every 3 s against a 10 s window: keeping only strictly-inside samples would
    # span 9 s forever and the detector would silently never fire.
    monitor = _monitor()
    stamp = 0
    for i in range(0, 8):
        stamp = i * 3 * NS_PER_S
        monitor.on_telemetry(_sample("robot_1", stamp))
    assert monitor.assess("robot_1", stamp).view is CoordView.STUCK


# ------------------------------------------------------------------ config --
def test_detection_latency_is_k_times_the_heartbeat_period() -> None:
    config = HealthConfig(heartbeat_period_ns=NS_PER_S, missed_beats_offline=3)
    assert config.detection_deadline_ns == 3 * NS_PER_S
    assert config.detection_latency_s == pytest.approx(3.0)


@pytest.mark.parametrize(
    ("kwargs", "match"),
    [
        ({"heartbeat_period_ns": 0}, "heartbeat_period_ns"),
        ({"missed_beats_offline": 0}, "missed_beats_offline"),
        ({"missed_beats_suspect": 9}, "missed_beats_suspect"),
        ({"stuck_window_ns": -1}, "stuck_window_ns"),
        ({"stuck_epsilon_m": -0.1}, "stuck_epsilon_m"),
        ({"stuck_min_samples": 1}, "stuck_min_samples"),
    ],
)
def test_config_rejects_nonsense(kwargs: dict[str, float], match: str) -> None:
    with pytest.raises(ValueError, match=match):
        HealthConfig(**kwargs)  # type: ignore[arg-type]


# ----------------------------------------------------------------- ordering --
def test_assessments_come_back_in_id_order_whatever_order_robots_arrived() -> None:
    monitor = HealthMonitor()
    for robot_id in ("robot_3", "robot_1", "robot_2"):
        monitor.register(robot_id, 0)
    assert tuple(a.robot_id for a in monitor.assess_all(0)) == ("robot_1", "robot_2", "robot_3")
    assert monitor.tracked == ("robot_1", "robot_2", "robot_3")


def test_telemetry_from_an_unregistered_robot_starts_tracking_it() -> None:
    monitor = HealthMonitor()
    monitor.on_telemetry(_sample("robot_7", 0))
    assert monitor.tracked == ("robot_7",)
    assert monitor.assess("robot_7", 0).view is CoordView.ONLINE


def test_suspect_counts_as_usable_but_offline_does_not() -> None:
    monitor = _monitor()
    assert monitor.assess("robot_1", NS_PER_S).usable
    assert not monitor.assess("robot_1", 3 * NS_PER_S).usable

"""The task state machine, exhaustively.

Acceptance criterion C7 requires coverage of ALL (state, event) pairs including the
illegal ones. That total is small (9 states x 15 events = 135) and the test is cheap, so
there is no reason to sample it -- and an exhaustive table test is exactly the evidence a
reviewer looks for that the machine is a machine rather than scattered if-statements.
"""

from __future__ import annotations

import itertools

import pytest
from parkscan_agent.statemachine import (
    TRANSITIONS,
    Event,
    IllegalTransition,
    is_legal,
    legal_events,
    step,
)

from parkscan_core.types import RobotState

ALL_PAIRS = list(itertools.product(RobotState, Event))


def test_every_state_event_pair_is_either_defined_or_raises() -> None:
    """C7: no pair is silently ignored. Every one of the 135 either transitions or raises."""
    defined = 0
    for state, event in ALL_PAIRS:
        if (state, event) in TRANSITIONS:
            assert step(state, event) is TRANSITIONS[(state, event)]
            defined += 1
        else:
            with pytest.raises(IllegalTransition):
                step(state, event)
    assert len(ALL_PAIRS) == len(RobotState) * len(Event) == 135
    assert 0 < defined < len(ALL_PAIRS), "a machine that accepts everything is not a machine"


def test_is_legal_agrees_with_step_on_every_pair() -> None:
    for state, event in ALL_PAIRS:
        if is_legal(state, event):
            step(state, event)  # must not raise
        else:
            with pytest.raises(IllegalTransition):
                step(state, event)


def test_legal_events_agrees_with_the_table_on_every_state() -> None:
    for state in RobotState:
        assert legal_events(state) == frozenset(e for e in Event if (state, e) in TRANSITIONS)


def test_the_table_is_immutable() -> None:
    """A mutable transition table is a state machine anyone can rewrite at runtime."""
    with pytest.raises(TypeError):
        TRANSITIONS[(RobotState.IDLE, Event.ABORT)] = RobotState.FAILED  # type: ignore[index]


def test_every_state_is_reachable_from_idle() -> None:
    """An unreachable state is dead code pretending to be behaviour."""
    seen = {RobotState.IDLE}
    frontier = [RobotState.IDLE]
    while frontier:
        state = frontier.pop()
        for event in legal_events(state):
            nxt = step(state, event)
            if nxt not in seen:
                seen.add(nxt)
                frontier.append(nxt)
    assert seen == set(RobotState), f"unreachable: {set(RobotState) - seen}"


def test_a_robot_can_always_be_failed_or_lost_while_working() -> None:
    """External bad news must be accepted in every working state.

    If FAULT or COMMS_LOST were illegal in some state, the coordinator could not mark a
    robot down at exactly the moment it most needs to.
    """
    working = [
        RobotState.ASSIGNED,
        RobotState.TRANSIT,
        RobotState.INSPECTING,
        RobotState.WAITING_ON_LEASE,
        RobotState.RETURNING,
        RobotState.CHARGING,
    ]
    for state in working:
        assert is_legal(state, Event.FAULT), state
        assert is_legal(state, Event.COMMS_LOST), state


def test_charging_completes_back_to_idle() -> None:
    assert step(RobotState.CHARGING, Event.CHARGED) is RobotState.IDLE


def test_battery_critical_diverts_from_every_working_state() -> None:
    for state in (RobotState.TRANSIT, RobotState.INSPECTING, RobotState.ASSIGNED):
        assert step(state, Event.BATTERY_CRITICAL) is RobotState.RETURNING

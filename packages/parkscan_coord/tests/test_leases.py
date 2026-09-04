"""Tests for ADR-0007: atomic acquisition, TTLs, fencing tokens and aging queues.

Four of these carry an argument rather than a behaviour, and they are the reason the
module is shaped the way it is:

* :func:`test_a_failed_acquisition_grants_nothing_at_all` -- there is no partial acquire.
* :func:`test_a_stale_token_is_rejected_by_the_resource_itself` -- fencing (acceptance
  C5), driven **directly** against the resource controller, never through the granter.
* :func:`test_no_robot_ever_holds_a_resource_while_waiting` -- the no-deadlock argument,
  executable.
* :func:`test_a_robot_that_keeps_losing_eventually_wins` -- ADR-0007 trades deadlock for
  starvation risk, so aging has to be measured, not asserted.

Time is a variable in this file. Every timestamp is written out explicitly and no test
reads a clock, which is what lets the whole suite be a pure function of its inputs.
"""

from __future__ import annotations

import ast
import inspect
import random
from collections import Counter
from itertools import pairwise
from pathlib import Path

import pytest
from parkscan_agent.lanegraph import route
from parkscan_coord import leases
from parkscan_coord.leases import (
    Grant,
    HoldAndWaitError,
    LeaseManager,
    QueueParams,
    ResourceController,
    StaleTokenError,
)

from parkscan_core.lot import load_lot

REPO_ROOT = Path(__file__).resolve().parents[3]
LOT_YAML = REPO_ROOT / "sim" / "lot" / "lot.yaml"

SECOND_NS = 1_000_000_000
TTL_NS = 3 * SECOND_NS

BAY_1 = "bay_1"
BAY_2 = "bay_2"
SPUR_1 = "ln_bay_1_spur"
WEST_1 = "ln_west_1"
WEST_2 = "ln_west_2"


def manager(*, alpha_per_s: float = 0.1) -> LeaseManager:
    return LeaseManager(queue=QueueParams(alpha_per_s=alpha_per_s))


def granted(grant: Grant | None) -> Grant:
    """Assert a grant happened and narrow the type for the rest of the test."""
    assert grant is not None
    return grant


def test_a_normal_route_acquires_lane_segments_out_of_id_order() -> None:
    """The measurement that killed resource ordering, re-taken here rather than cited.

    ``docs/ARCHITECTURE.md`` section 4 originally broke deadlock on circular wait with the
    Havender rule: never request an id lower than the highest you already hold. This test
    is the counter-example ADR-0007 was written around, recomputed from the committed lot
    so it cannot silently stop being true: an ordinary route from ``bay_1_node`` to the
    last zone's entry acquires its lane segments in an order that is **not** sorted, so
    that rule would make a perfectly normal drive illegal.

    Renaming the edges is not a fix -- the traversal order depends on where the robot is
    going, and reverses when it comes back -- which is why this package breaks
    hold-and-wait instead, and why :meth:`LeaseManager._normalise` sorts a request only
    for determinism and never compares a requested id against a held one.
    """
    lot = load_lot(LOT_YAML)
    edge_id = {}
    for edge in lot.lane_graph.edges:
        edge_id[(edge.a, edge.b)] = edge.id
        edge_id[(edge.b, edge.a)] = edge.id

    last_zone = lot.zones[-1]
    path = route(lot, "bay_1_node", last_zone.entry_waypoint)
    acquired = [edge_id[pair] for pair in pairwise(path)]

    assert acquired[:6] == [
        "ln_bay_1_spur",
        "ln_west_1",
        "ln_west_2",
        "ln_west_3",
        "ln_west_4",
        "ln_r3_h0",
    ]
    assert acquired != sorted(acquired), "the ordering rule would have been usable after all"

    # And no renaming scheme could fix it, because the return trip demands the opposite
    # order over the segments the two routes share. Both directions are 170.5 m here; the
    # router picks a different equal-cost path back, and the shared segments are traversed
    # in inverted relative order, so no single total order over ids can make both routes
    # legal. That is a contradiction, not a naming inconvenience.
    back = [edge_id[pair] for pair in pairwise(route(lot, last_zone.entry_waypoint, "bay_1_node"))]
    shared = sorted(set(acquired) & set(back))
    assert shared == ["ln_bay_1_spur", "ln_r3_h6", "ln_west_1"]
    forward_at = {edge: index for index, edge in enumerate(acquired)}
    back_at = {edge: index for index, edge in enumerate(back)}
    inverted = [
        (first, second)
        for first in shared
        for second in shared
        if forward_at[first] < forward_at[second] and back_at[first] > back_at[second]
    ]
    assert inverted, "the two directions agree on an order, so ordering might have worked"


# ---------------------------------------------------------------- atomic acquire --
def test_acquire_all_grants_the_whole_leg() -> None:
    leases_ = manager()
    grant = granted(leases_.acquire_all("robot_1", [WEST_2, SPUR_1, WEST_1], 0, TTL_NS))

    # Sorted internally: the attempt is deterministic and replayable. This is *not* the
    # Havender ordering rule (ADR-0007 rejects it); nothing compares a requested id
    # against a held one, because a requesting robot holds nothing.
    assert grant.resources == (SPUR_1, WEST_1, WEST_2)
    assert leases_.held_by("robot_1") == (SPUR_1, WEST_1, WEST_2)
    assert all(leases_.holder(r) == "robot_1" for r in grant.resources)
    assert grant.expires_at_ns == TTL_NS


def test_a_second_robot_asking_for_an_overlapping_set_is_refused_entirely() -> None:
    """Overlap on one segment refuses the whole leg -- there is no partial grant."""
    leases_ = manager()
    granted(leases_.acquire_all("robot_1", [WEST_1, WEST_2], 0, TTL_NS))

    refused = leases_.acquire_all("robot_2", [WEST_2, SPUR_1], 0, TTL_NS)

    assert refused is None
    assert leases_.held_by("robot_2") == ()
    assert leases_.holder(SPUR_1) is None, "the free half of the request must not be taken"
    assert leases_.is_waiting("robot_2")


def test_a_failed_acquisition_grants_nothing_at_all() -> None:
    """Atomicity, stated as "the world did not move".

    A partial acquire is what would put a robot in the position of holding one resource
    while waiting for another, which is the hold-and-wait condition ADR-0007 removes. So
    the assertion is over the *whole* resource table, not just the contended member.
    """
    leases_ = manager()
    resources = [BAY_1, BAY_2, SPUR_1, WEST_1, WEST_2]
    granted(leases_.acquire_all("robot_1", [BAY_1, WEST_1], 0, TTL_NS))
    before = {r: leases_.holder(r) for r in resources}

    assert leases_.acquire_all("robot_2", [WEST_1, WEST_2, BAY_2], SECOND_NS, TTL_NS) is None

    assert {r: leases_.holder(r) for r in resources} == before


def test_holding_one_set_and_asking_for_another_is_refused_loudly() -> None:
    """The structural break of hold-and-wait: release your leg before asking for the next.

    Tolerating this would be the one change that reintroduces the possibility of a
    wait-for cycle, so it raises rather than queueing quietly.
    """
    leases_ = manager()
    granted(leases_.acquire_all("robot_1", [WEST_1], 0, TTL_NS))

    with pytest.raises(HoldAndWaitError, match="Release the current leg first"):
        leases_.acquire_all("robot_1", [WEST_2], SECOND_NS, TTL_NS)

    assert leases_.release_all("robot_1") == (WEST_1,)
    assert granted(leases_.acquire_all("robot_1", [WEST_2], SECOND_NS, TTL_NS)).resources == (
        WEST_2,
    )


def test_re_requesting_the_set_you_hold_is_idempotent() -> None:
    """A retried request after a dropped acknowledgement must not be a fatal error."""
    leases_ = manager()
    first = granted(leases_.acquire_all("robot_1", [WEST_1, WEST_2], 0, TTL_NS))
    again = granted(leases_.acquire_all("robot_1", [WEST_2, WEST_1], SECOND_NS, TTL_NS))
    assert again == first


def test_an_empty_or_ill_formed_request_is_an_error() -> None:
    leases_ = manager()
    with pytest.raises(ValueError, match="at least one resource"):
        leases_.acquire_all("robot_1", [], 0, TTL_NS)
    with pytest.raises(ValueError, match="ttl_ns must be positive"):
        leases_.acquire_all("robot_1", [WEST_1], 0, 0)


# ------------------------------------------------------------------ ttl / expiry --
def test_expiry_reclaims_the_lease_unilaterally() -> None:
    """The no-preemption break: a robot that dies holding a bay cannot block it forever.

    Atomic acquisition does nothing about this case, which is exactly why ADR-0007 keeps
    lease expiry as a second, independent mechanism.
    """
    leases_ = manager()
    granted(leases_.acquire_all("robot_1", [BAY_1], 0, TTL_NS))

    assert leases_.acquire_all("robot_2", [BAY_1], TTL_NS - 1, TTL_NS) is None
    assert leases_.reap(TTL_NS - 1) == ()

    assert leases_.reap(TTL_NS) == (BAY_1,)
    assert leases_.holder(BAY_1) is None
    assert leases_.held_by("robot_1") == ()
    assert granted(leases_.acquire_all("robot_2", [BAY_1], TTL_NS, TTL_NS)).resources == (BAY_1,)


def test_renewal_extends_the_lease_and_keeps_the_token() -> None:
    """Renewing must not mint a new token: it is the same lease, not a new one.

    A bumped token would fence out the holder's own in-flight actions, which is a
    self-inflicted version of the failure fencing exists to prevent.
    """
    leases_ = manager()
    first = granted(leases_.acquire_all("robot_1", [BAY_1], 0, TTL_NS))

    renewed = granted(leases_.renew("robot_1", 2 * SECOND_NS, TTL_NS))

    assert renewed.token(BAY_1) == first.token(BAY_1)
    assert renewed.expires_at_ns == 2 * SECOND_NS + TTL_NS
    assert renewed.granted_at_ns == first.granted_at_ns
    assert leases_.reap(TTL_NS) == (), "the renewed lease must survive its original expiry"
    assert leases_.holder(BAY_1) == "robot_1"


def test_renewing_a_lease_that_already_expired_returns_none() -> None:
    """The honest answer to "may I keep it?" after it has already been reclaimed."""
    leases_ = manager()
    granted(leases_.acquire_all("robot_1", [BAY_1], 0, TTL_NS))
    assert leases_.renew("robot_1", TTL_NS, TTL_NS) is None
    assert leases_.holder(BAY_1) is None


# --------------------------------------------------------------------- fencing --
def test_a_stale_token_is_rejected_by_the_resource_itself() -> None:
    """Acceptance C5, driven **directly** against the resource controller.

    This test deliberately never asks :class:`LeaseManager` whether the old token is
    still good. In this simulation the granter and the resource live in one process, but
    in a real deployment they are separate machines and fencing only means anything when
    they are: a controller its granter could "correct" proves nothing, because the
    granter already knows the lease expired. ADR-0007 says so under "Known weakness,
    stated rather than hidden", and this is that test.

    So: the controller is a standalone object here, it is handed tokens the way a robot
    would hand them over, and the stale one is rejected by arithmetic alone -- the
    controller knows nothing about leases, TTLs or who is alive.
    """
    granter = LeaseManager()
    bay = ResourceController()

    stalled = granted(granter.acquire_all("robot_1", [BAY_1], 0, TTL_NS))
    old_token = stalled.token(BAY_1)
    bay.require_admission(BAY_1, "robot_1", old_token)
    assert bay.holder(BAY_1) == "robot_1"

    # robot_1 stalls. Its lease expires and robot_2 legitimately takes the bay.
    assert granter.reap(TTL_NS) == (BAY_1,)
    replacement = granted(granter.acquire_all("robot_2", [BAY_1], TTL_NS, TTL_NS))
    new_token = replacement.token(BAY_1)
    assert new_token > old_token
    bay.require_admission(BAY_1, "robot_2", new_token)

    # robot_1 wakes up still believing it holds the bay, and drives in.
    assert bay.admit(BAY_1, "robot_1", old_token) is False
    with pytest.raises(StaleTokenError, match="rejected"):
        bay.require_admission(BAY_1, "robot_1", old_token)

    assert bay.holder(BAY_1) == "robot_2"
    assert bay.highest_token(BAY_1) == new_token


def test_a_stale_holder_cannot_release_the_bay_from_under_the_new_holder() -> None:
    """Fencing in the other direction, which is easy to forget and just as damaging."""
    granter = LeaseManager()
    bay = ResourceController()
    old = granted(granter.acquire_all("robot_1", [BAY_1], 0, TTL_NS)).token(BAY_1)
    bay.require_admission(BAY_1, "robot_1", old)

    granter.reap(TTL_NS)
    new = granted(granter.acquire_all("robot_2", [BAY_1], TTL_NS, TTL_NS)).token(BAY_1)
    bay.require_admission(BAY_1, "robot_2", new)

    assert bay.release(BAY_1, "robot_1", old) is False
    assert bay.holder(BAY_1) == "robot_2"
    assert bay.release(BAY_1, "robot_2", new) is True
    assert bay.holder(BAY_1) is None


def test_tokens_are_monotonic_per_resource() -> None:
    """A monotonic counter per resource -- the same mechanism as the assignment epoch."""
    leases_ = manager()
    seen: list[int] = []
    for round_index in range(5):
        at = round_index * TTL_NS
        grant = granted(leases_.acquire_all(f"robot_{round_index}", [BAY_1, BAY_2], at, TTL_NS))
        seen.append(grant.token(BAY_1))
        assert grant.token(BAY_2) == grant.token(BAY_1), "counters are per resource, in step here"
        leases_.release_all(f"robot_{round_index}")
    assert seen == sorted(seen)
    assert len(set(seen)) == len(seen)


def test_a_controller_rejects_a_token_that_was_never_minted() -> None:
    bay = ResourceController()
    with pytest.raises(ValueError, match="tokens start at 1"):
        bay.admit(BAY_1, "robot_1", 0)


def test_a_grant_only_answers_for_its_own_resources() -> None:
    leases_ = manager()
    grant = granted(leases_.acquire_all("robot_1", [BAY_1], 0, TTL_NS))
    with pytest.raises(KeyError, match="not part of this grant"):
        grant.token(BAY_2)


# ------------------------------------------------------------- queue and aging --
def test_the_queue_is_fifo_among_equal_priorities() -> None:
    leases_ = manager()
    granted(leases_.acquire_all("holder", [BAY_1], 0, 10 * SECOND_NS))

    assert leases_.acquire_all("robot_1", [BAY_1], 1 * SECOND_NS, TTL_NS) is None
    assert leases_.acquire_all("robot_2", [BAY_1], 2 * SECOND_NS, TTL_NS) is None
    assert [w.robot_id for w in leases_.waiters_for(BAY_1, 3 * SECOND_NS)] == [
        "robot_1",
        "robot_2",
    ]

    leases_.release_all("holder")
    # The later arrival must not barge, even though the bay is free when it asks.
    assert leases_.acquire_all("robot_2", [BAY_1], 4 * SECOND_NS, TTL_NS) is None
    assert granted(leases_.acquire_all("robot_1", [BAY_1], 4 * SECOND_NS, TTL_NS))


def test_a_low_battery_robot_jumps_the_bay_queue() -> None:
    """Base priority is what lets urgency overtake seniority, in one direction only."""
    leases_ = manager()
    granted(leases_.acquire_all("holder", [BAY_1], 0, 10 * SECOND_NS))
    assert leases_.acquire_all("early", [BAY_1], 1 * SECOND_NS, TTL_NS) is None
    critical_at = 2 * SECOND_NS
    assert leases_.acquire_all("critical", [BAY_1], critical_at, TTL_NS, base_priority=1.0) is None

    ranked = leases_.waiters_for(BAY_1, 3 * SECOND_NS)
    assert [w.robot_id for w in ranked] == ["critical", "early"]

    leases_.release_all("holder")
    assert leases_.acquire_all("early", [BAY_1], 3 * SECOND_NS, TTL_NS) is None
    at = 3 * SECOND_NS
    assert granted(leases_.acquire_all("critical", [BAY_1], at, TTL_NS, base_priority=1.0))


@pytest.mark.parametrize(("alpha_per_s", "expected_win_round"), [(0.1, 11), (0.0, None)])
def test_a_robot_that_keeps_losing_eventually_wins(
    alpha_per_s: float, expected_win_round: int | None
) -> None:
    """Starvation is the hazard ADR-0007 trades deadlock for, so aging is load-bearing.

    ``urgent`` re-takes the bay every second with base priority 1.0; ``loser`` asks for it
    every second with base priority 0.0 and would never win on base priority alone. With
    ``alpha_per_s = 0.1`` its effective priority reaches 1.0 after 10 s of waiting, it
    outranks the fresh urgent request on the FIFO tie-break, and it wins on round 11 --
    exactly the ``(B - base) / alpha`` bound the module docstring claims.

    The ``alpha_per_s = 0.0`` row is the control: with aging switched off the same robot
    starves forever, which is what makes the first row evidence rather than a coincidence.
    """
    leases_ = manager(alpha_per_s=alpha_per_s)
    win_round: int | None = None
    waited_s = 0.0

    for round_index in range(1, 31):
        now = round_index * SECOND_NS
        if leases_.held_by("urgent"):
            leases_.release_all("urgent")
        leases_.acquire_all("urgent", [BAY_1], now, TTL_NS, base_priority=1.0)

        waiting = leases_.waiter("loser")
        pending = waiting.waited_s(now) if waiting is not None else 0.0
        if leases_.acquire_all("loser", [BAY_1], now, TTL_NS, base_priority=0.0) is not None:
            win_round, waited_s = round_index, pending
            break

    assert win_round == expected_win_round
    if expected_win_round is not None:
        assert waited_s == pytest.approx(10.0)
        assert leases_.holder(BAY_1) == "loser"


def test_cancelling_a_wait_unblocks_the_robots_behind_it() -> None:
    """The stated remedy for a waiter that died holding its place in the queue."""
    leases_ = manager()
    granted(leases_.acquire_all("holder", [BAY_1], 0, 10 * SECOND_NS))
    assert leases_.acquire_all("first", [BAY_1], SECOND_NS, TTL_NS) is None
    assert leases_.acquire_all("second", [BAY_1], 2 * SECOND_NS, TTL_NS) is None
    leases_.release_all("holder")

    assert leases_.acquire_all("second", [BAY_1], 3 * SECOND_NS, TTL_NS) is None
    assert leases_.cancel_wait("first") is True
    assert granted(leases_.acquire_all("second", [BAY_1], 3 * SECOND_NS, TTL_NS))


def test_an_abandoned_waiter_is_dropped_when_that_is_configured() -> None:
    leases_ = LeaseManager(queue=QueueParams(alpha_per_s=0.1, abandon_after_ns=5 * SECOND_NS))
    granted(leases_.acquire_all("holder", [BAY_1], 0, 60 * SECOND_NS))
    assert leases_.acquire_all("ghost", [BAY_1], SECOND_NS, TTL_NS) is None
    leases_.release_all("holder")

    # 'ghost' has not retried for more than abandon_after_ns, so it stops blocking.
    assert granted(leases_.acquire_all("later", [BAY_1], 10 * SECOND_NS, TTL_NS))
    assert leases_.is_waiting("ghost") is False


def test_queue_params_reject_nonsense() -> None:
    with pytest.raises(ValueError, match="alpha_per_s"):
        QueueParams(alpha_per_s=-1.0)
    with pytest.raises(ValueError, match="abandon_after_ns"):
        QueueParams(abandon_after_ns=-1)


# ------------------------------------------------------------------ no deadlock --
def test_no_robot_ever_holds_a_resource_while_waiting() -> None:
    """The ADR-0007 deadlock argument, executed rather than reasoned about.

    Five robots contend for overlapping random resource sets over 400 rounds of a
    controlled clock. After every single operation the invariant is checked: **a waiting
    robot holds nothing**. If that holds, the wait-for graph has no edges, so it has no
    cycle, so the system cannot deadlock -- and unlike the resource-ordering rule ADR-0007
    rejected, this argument does not care what order routes acquire lane segments in.

    Progress is asserted too, because "nobody holds anything, ever" would also satisfy the
    invariant. Every grant is checked to be the *whole* requested set, which is the
    property that would fail first if a partial-acquire path were ever added.
    """
    resources = [f"ln_seg_{i}" for i in range(6)]
    robots = [f"robot_{i}" for i in range(1, 6)]
    leases_ = manager(alpha_per_s=0.5)
    rng = random.Random(20260904)

    wanted: dict[str, tuple[str, ...]] = {}
    hold_until: dict[str, int] = {}
    grants: Counter[str] = Counter()
    now = 0

    for _round in range(400):
        now += SECOND_NS // 10
        for robot in robots:
            if leases_.held_by(robot):
                if now >= hold_until[robot]:
                    leases_.release_all(robot)
                    del wanted[robot]
                continue
            if robot not in wanted:
                wanted[robot] = tuple(rng.sample(resources, rng.randint(1, 3)))
            grant = leases_.acquire_all(robot, wanted[robot], now, TTL_NS)
            if grant is not None:
                assert grant.resources == tuple(sorted(wanted[robot])), "partial grant"
                grants[robot] += 1
                hold_until[robot] = now + rng.randint(1, 4) * (SECOND_NS // 10)
            leases_.check_no_hold_and_wait()

        holders = [(robot, r) for robot in robots for r in leases_.held_by(robot)]
        assert len({r for _, r in holders}) == len(holders), "two robots hold one resource"

    assert all(grants[robot] > 0 for robot in robots), f"a robot never got anything: {grants}"
    assert min(grants.values()) >= 20, f"progress is badly unfair: {grants}"


def test_the_lease_manager_never_reads_a_clock() -> None:
    """Determinism, checked structurally rather than hoped for.

    ``make bench`` replays 30 seeds and every README number has to be regenerable, so the
    coordinator must be a pure function of ``(state, events, now_ns)``. A single
    ``time.time()`` in here would make a run unreproducible in a way no assertion on
    behaviour would catch.
    """
    imported: set[str] = set()
    for node in ast.walk(ast.parse(inspect.getsource(leases))):
        if isinstance(node, ast.Import):
            imported |= {alias.name.split(".")[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module is not None:
            imported.add(node.module.split(".")[0])

    # The source is parsed rather than grepped: the module docstring talks about
    # ``time.time()`` at length, and a substring check would be testing the prose.
    assert imported.isdisjoint({"time", "datetime", "calendar"}), "a clock got imported"
    assert imported.isdisjoint({"random", "secrets", "uuid"}), "a global RNG got imported"

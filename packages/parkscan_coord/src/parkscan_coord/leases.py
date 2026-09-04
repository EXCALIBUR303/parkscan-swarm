"""Resource leases: atomic per-leg acquisition, TTLs, fencing tokens, aging queues.

This module implements ADR-0007 exactly, and the shape of it is the ADR's central
finding, so it is worth restating here rather than only in ``docs/``.

Why hold-and-wait, and not resource ordering
--------------------------------------------
``docs/ARCHITECTURE.md`` section 4 originally broke deadlock on **circular wait**, with
the Havender/Dijkstra rule: never request a resource id lower than the highest you
already hold. That rule does not survive the lane graph. A route acquires lane segments
in the order it **traverses** them, and a shortest path has no reason to visit them in id
order -- measured on the committed lot, the route from ``bay_1_node`` to the last zone's
entry acquires::

    ln_bay_1_spur, ln_west_1 ... ln_west_4, ln_r3_h0 ... ln_r3_h6

which is not sorted: ``ln_west_*`` precedes ``ln_r3_*`` in traversal but follows it
lexicographically. Enforcing the ordering rule would make that ordinary route illegal,
and renaming the edges cannot fix it because the traversal order depends on where the
robot is going, and reverses when it comes back.

So this implementation breaks **hold-and-wait** instead. A robot acquires the entire
resource set for its next leg **atomically or not at all** (:meth:`LeaseManager.acquire_all`).
The request is sorted by id internally, so the attempt is deterministic and replayable,
but a partial set is never granted and **there is no partial-acquire code path** -- adding
one would reintroduce the condition this design removes.

The deadlock argument is then a single line a reviewer can check:

    A waiting robot holds nothing, so it contributes no edge to the wait-for graph, so
    the wait-for graph has no edges, so it has no cycle, so there is no deadlock.

:meth:`LeaseManager.acquire_all` enforces the premise rather than assuming it: a robot
that still holds a resource is refused with :class:`HoldAndWaitError` and told to release
first. :meth:`LeaseManager.check_no_hold_and_wait` is the same invariant as a callable
assertion, and ``test_leases.py`` runs it after every operation of a randomised
multi-robot contention test.

The two things atomic acquisition does *not* cover
--------------------------------------------------
**A dead holder.** Atomicity says nothing about a robot that stops existing while holding
a bay, so leases keep a TTL (Gray & Cheriton, SOSP 1989): a grant expires at
``granted_at + ttl``, the holder renews it while it is still working, and expiry reclaims
the resource unilaterally. That is the second, independent Coffman break (no-preemption).

**A resurrected holder.** A robot can stall past its own expiry and wake up still
believing it holds the bay. Every action therefore carries a **fencing token**
(Kleppmann): a per-resource monotonic counter minted by the granter. The resource itself
-- :class:`ResourceController` -- records the highest token it has ever accepted and
rejects anything lower.

:class:`ResourceController` and :class:`LeaseManager` are deliberately **separate objects
with no reference to each other**. In this simulation both live in one process, but in a
real deployment the granter and the resource are different machines, and fencing only
means anything when they are: a controller that can be "corrected" by its granter proves
nothing. So the controller is driven by whoever acts on the resource, and the fencing
test in ``test_leases.py`` calls it directly with a stale token, never through the
granter. ADR-0007 says so in as many words, under "Known weakness, stated rather than
hidden".

Starvation is the hazard this design trades for
-----------------------------------------------
Atomic acquisition removes deadlock and leaves starvation, so the queue is load-bearing
rather than decorative. Each resource has a FIFO wait queue with **aging**::

    effective_priority = base_priority + alpha_per_s * seconds_waited

A robot that needs a bay urgently (low charge) enters with a higher base priority and
jumps the queue; but every waiter's priority grows with the time it has waited, so a
waiter behind a competitor of base priority ``B`` overtakes it after at most
``(B - base) / alpha_per_s`` seconds of waiting. Nobody waits forever, which is what
``test_leases.py::test_a_robot_that_keeps_losing_eventually_wins`` demonstrates.

Clock
-----
Every method takes ``now_ns`` from the caller. Nothing here reads ``time.time()``: the
coordinator runs on simulator time, ``make bench`` replays 30 seeds, and a lease manager
that consulted the wall clock would make those runs irreproducible.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from types import MappingProxyType

from parkscan_core.types import RobotId

__all__ = [
    "Grant",
    "HoldAndWaitError",
    "LeaseError",
    "LeaseManager",
    "QueueParams",
    "ResourceController",
    "StaleTokenError",
    "Waiter",
]

_NS_PER_S = 1_000_000_000.0


class LeaseError(Exception):
    """Base class for every failure this module raises."""


class HoldAndWaitError(LeaseError):
    """A robot tried to acquire a new resource set while still holding another.

    Raised rather than tolerated: permitting it would recreate the hold-and-wait
    condition that ADR-0007 removes, and the wait-for graph could then contain a cycle.
    The caller must :meth:`LeaseManager.release_all` its current leg first.
    """


class StaleTokenError(LeaseError):
    """A resource was handed a fencing token lower than one it has already accepted."""


@dataclass(frozen=True, slots=True)
class Grant:
    """An atomic, time-limited grant of a whole resource set to one robot."""

    robot_id: RobotId
    resources: tuple[str, ...]
    """Sorted by id. The set is indivisible: every member is granted, or none is."""
    tokens: Mapping[str, int]
    """Per-resource fencing token, to be presented to the :class:`ResourceController`."""
    granted_at_ns: int
    expires_at_ns: int

    def token(self, resource: str) -> int:
        """The fencing token for one resource of this grant."""
        try:
            return self.tokens[resource]
        except KeyError:
            msg = f"{resource!r} is not part of this grant {self.resources}"
            raise KeyError(msg) from None

    def is_expired(self, now_ns: int) -> bool:
        """A lease is dead **at** its expiry instant, not one nanosecond after."""
        return now_ns >= self.expires_at_ns

    def remaining_ns(self, now_ns: int) -> int:
        """Nanoseconds left before expiry; never negative."""
        return max(0, self.expires_at_ns - now_ns)


@dataclass(frozen=True, slots=True)
class Waiter:
    """One robot queued for a resource set it could not get atomically."""

    robot_id: RobotId
    resources: tuple[str, ...]
    since_ns: int
    """When this robot started waiting. Aging is measured from here, and retrying does
    not reset it -- otherwise a robot that polls would keep resetting its own age."""
    base_priority: float
    """Static urgency, higher wins. The coordinator sets it (e.g. from charge)."""
    seq: int
    """Arrival order, the FIFO half of the discipline and the tie-break between equals."""
    last_seen_ns: int
    """When this robot last retried. Only used by :attr:`QueueParams.abandon_after_ns`."""

    def waited_s(self, now_ns: int) -> float:
        return max(0, now_ns - self.since_ns) / _NS_PER_S

    def effective_priority(self, now_ns: int, alpha_per_s: float) -> float:
        """``base_priority + alpha_per_s * seconds_waited``."""
        return self.base_priority + alpha_per_s * self.waited_s(now_ns)


@dataclass(frozen=True, slots=True)
class QueueParams:
    """Queue policy. Frozen, so a run's policy is a value that can be logged."""

    alpha_per_s: float = 0.1
    """Priority gained per second of waiting.

    A policy choice, not a measurement. With base priorities in ``[0, 1]`` (the natural
    scale if the coordinator uses something like ``1 - soc``), 0.1/s means a waiter
    overtakes even a maximally urgent competitor after at most 10 s, which is the same
    order as the 3 s offline deadline and far below any run length.
    """

    abandon_after_ns: int = 0
    """Drop a waiter that has not retried for this long. ``0`` disables it.

    Disabled by default, and that is a real, stated hazard: the queue blocks lower-ranked
    robots on behalf of a waiter that may have died, so a coordinator that does not call
    :meth:`LeaseManager.cancel_wait` when its failure detector declares a robot OFFLINE
    should set this instead. Two remedies, both explicit; no silent one.
    """

    def __post_init__(self) -> None:
        if self.alpha_per_s < 0.0:
            msg = f"alpha_per_s must be non-negative, got {self.alpha_per_s}"
            raise ValueError(msg)
        if self.abandon_after_ns < 0:
            msg = f"abandon_after_ns must be non-negative, got {self.abandon_after_ns}"
            raise ValueError(msg)


class ResourceController:
    """The **resource** side of the fence: it accepts or rejects tokens.

    Deliberately independent of :class:`LeaseManager`. It holds no reference to the
    granter and the granter holds no reference to it, because a controller its granter
    could correct would prove nothing about fencing (ADR-0007). Whoever acts on the
    resource -- the robot, or the coordinator acting for it -- presents the token it was
    granted, and this object decides.

    The rule is one line: **remember the highest token ever accepted for a resource, and
    reject anything lower.** A robot that stalls past its lease expiry, loses the
    resource to someone else, then wakes up and acts on its old token is rejected by
    arithmetic, without the controller needing to know anything about leases, TTLs or who
    is alive.
    """

    def __init__(self) -> None:
        self._highest: dict[str, int] = {}
        self._holder: dict[str, RobotId] = {}

    def admit(self, resource: str, robot_id: RobotId, token: int) -> bool:
        """Try to act on ``resource`` with ``token``. ``False`` means fenced out.

        Equal tokens are accepted: a lease keeps its token across renewals, so the holder
        can act on the same resource repeatedly. Only a *lower* token is a stale one.
        """
        if token < 1:
            msg = f"fencing tokens start at 1, got {token}"
            raise ValueError(msg)
        if token < self._highest.get(resource, 0):
            return False
        self._highest[resource] = token
        self._holder[resource] = robot_id
        return True

    def require_admission(self, resource: str, robot_id: RobotId, token: int) -> None:
        """:meth:`admit`, raising :class:`StaleTokenError` instead of returning ``False``."""
        if not self.admit(resource, robot_id, token):
            msg = (
                f"{robot_id!r} presented token {token} for {resource!r}, which has "
                f"already accepted token {self._highest.get(resource, 0)}; rejected"
            )
            raise StaleTokenError(msg)

    def release(self, resource: str, robot_id: RobotId, token: int) -> bool:
        """Give the resource up. Fenced too: a stale holder cannot release it.

        Without the token check here, a robot waking up after its lease expired could
        release the resource out from under the robot that legitimately holds it now --
        the same failure fencing exists to prevent, in the other direction.
        """
        if token < self._highest.get(resource, 0):
            return False
        if self._holder.get(resource) != robot_id:
            return False
        self._highest[resource] = max(self._highest.get(resource, 0), token)
        del self._holder[resource]
        return True

    def highest_token(self, resource: str) -> int:
        """Highest token ever accepted for ``resource``; ``0`` if it never has been."""
        return self._highest.get(resource, 0)

    def holder(self, resource: str) -> RobotId | None:
        """Who this resource believes is acting on it, if anyone."""
        return self._holder.get(resource)


class LeaseManager:
    """The **granter**: atomic acquisition, TTLs, fencing tokens and aging queues.

    One instance owns a namespace of resource ids -- single-occupancy lane segments and
    charging bays, in this system. Ids are opaque strings; the manager never needs to
    know which is which.
    """

    def __init__(self, *, queue: QueueParams | None = None) -> None:
        self.queue = queue if queue is not None else QueueParams()
        self._grants: dict[RobotId, Grant] = {}
        self._holder: dict[str, RobotId] = {}
        self._next_token: dict[str, int] = {}
        self._waiters: dict[RobotId, Waiter] = {}
        self._next_seq = 0

    # ------------------------------------------------------------ acquisition --
    def acquire_all(
        self,
        robot_id: RobotId,
        resources: Iterable[str],
        now_ns: int,
        ttl_ns: int,
        *,
        base_priority: float = 0.0,
    ) -> Grant | None:
        """Acquire **every** resource in ``resources``, or none of them.

        Returns the :class:`Grant` on success. On failure returns ``None`` and enqueues
        the robot as a waiter on the set it asked for -- it holds nothing in the
        meantime, which is the property that makes a wait-for cycle impossible.

        A request fails when any member is held by someone else, **or** when another
        waiter for an overlapping set outranks this robot. That second condition is what
        stops a lucky late arrival from barging past a robot that has been waiting, and
        it is what makes the aging in :class:`QueueParams` effective rather than
        cosmetic.

        Re-requesting exactly the set you already hold is idempotent and returns the
        existing grant (use :meth:`renew` to extend it). Requesting anything else while
        holding raises.

        Raises:
            HoldAndWaitError: the robot still holds a different resource set.
            ValueError: empty request, empty id, or a non-positive TTL.
        """
        requested = self._normalise(resources)
        if ttl_ns <= 0:
            msg = f"ttl_ns must be positive, got {ttl_ns}"
            raise ValueError(msg)

        self.reap(now_ns)
        self._reap_waiters(now_ns)

        current = self._grants.get(robot_id)
        if current is not None:
            if current.resources == requested:
                return current
            msg = (
                f"{robot_id!r} holds {current.resources} and asked for {requested}. "
                "Release the current leg first: holding one resource while waiting for "
                "another is the condition ADR-0007 removes."
            )
            raise HoldAndWaitError(msg)

        blocked = tuple(r for r in requested if r in self._holder)
        if blocked or self._outranked(robot_id, requested, now_ns, base_priority):
            self._enqueue(robot_id, requested, now_ns, base_priority)
            return None

        grant = self._mint(robot_id, requested, now_ns, ttl_ns)
        self._waiters.pop(robot_id, None)
        return grant

    def renew(self, robot_id: RobotId, now_ns: int, ttl_ns: int) -> Grant | None:
        """Extend the robot's grant to ``now_ns + ttl_ns``, keeping its tokens.

        Returns the new grant, or ``None`` if the robot no longer holds one -- the honest
        answer when a lease has already expired and been reclaimed. Tokens are
        deliberately **not** re-minted: a renewal is the same lease, and bumping the token
        would fence out the holder's own in-flight actions.
        """
        if ttl_ns <= 0:
            msg = f"ttl_ns must be positive, got {ttl_ns}"
            raise ValueError(msg)
        self.reap(now_ns)
        current = self._grants.get(robot_id)
        if current is None:
            return None
        renewed = Grant(
            robot_id=current.robot_id,
            resources=current.resources,
            tokens=current.tokens,
            granted_at_ns=current.granted_at_ns,
            expires_at_ns=now_ns + ttl_ns,
        )
        self._grants[robot_id] = renewed
        return renewed

    def release_all(self, robot_id: RobotId) -> tuple[str, ...]:
        """Give up everything ``robot_id`` holds. Returns the resources released."""
        grant = self._grants.pop(robot_id, None)
        if grant is None:
            return ()
        for resource in grant.resources:
            if self._holder.get(resource) == robot_id:
                del self._holder[resource]
        return grant.resources

    def reap(self, now_ns: int) -> tuple[str, ...]:
        """Reclaim every expired lease. This is the no-preemption break.

        Called at the start of every acquisition, so a crashed holder never blocks the
        fleet for longer than its TTL, and callable directly by a coordinator that wants
        to reclaim on a timer.
        """
        expired = sorted(
            (g for g in self._grants.values() if g.is_expired(now_ns)),
            key=lambda g: g.robot_id,
        )
        released: list[str] = []
        for grant in expired:
            del self._grants[grant.robot_id]
            for resource in grant.resources:
                if self._holder.get(resource) == grant.robot_id:
                    del self._holder[resource]
                released.append(resource)
        return tuple(sorted(released))

    # ----------------------------------------------------------------- queries --
    def grant_of(self, robot_id: RobotId) -> Grant | None:
        """The robot's current grant, if it has one. No expiry check -- see :meth:`reap`."""
        return self._grants.get(robot_id)

    def held_by(self, robot_id: RobotId) -> tuple[str, ...]:
        """Resources ``robot_id`` currently holds, sorted."""
        grant = self._grants.get(robot_id)
        return grant.resources if grant is not None else ()

    def holder(self, resource: str) -> RobotId | None:
        """Which robot holds ``resource``, if any."""
        return self._holder.get(resource)

    def token_of(self, resource: str) -> int | None:
        """The fencing token of the current holder of ``resource``."""
        robot_id = self._holder.get(resource)
        if robot_id is None:
            return None
        return self._grants[robot_id].token(resource)

    def is_waiting(self, robot_id: RobotId) -> bool:
        return robot_id in self._waiters

    def waiter(self, robot_id: RobotId) -> Waiter | None:
        return self._waiters.get(robot_id)

    def waiters_for(self, resource: str, now_ns: int) -> tuple[Waiter, ...]:
        """Everyone queued for ``resource``, best claim first.

        Ordered by effective priority (aging included), then by arrival -- so equal
        priorities are strict FIFO.
        """
        queued = [w for w in self._waiters.values() if resource in w.resources]
        return tuple(sorted(queued, key=lambda w: self._rank(w, now_ns)))

    def cancel_wait(self, robot_id: RobotId) -> bool:
        """Remove a robot from every queue. Call this when you declare it OFFLINE."""
        return self._waiters.pop(robot_id, None) is not None

    def check_no_hold_and_wait(self) -> None:
        """The ADR-0007 deadlock argument, as an executable assertion.

        A waiting robot must hold nothing. If that holds for every robot then the
        wait-for graph has no edges at all, so it cannot contain a cycle, so the system
        cannot deadlock -- regardless of the order routes happen to acquire lanes in.

        Raises:
            HoldAndWaitError: some robot is both holding and waiting.
        """
        for robot_id in sorted(self._waiters):
            held = self.held_by(robot_id)
            if held:
                msg = (
                    f"{robot_id!r} holds {held} while waiting for "
                    f"{self._waiters[robot_id].resources}: the wait-for graph now has an "
                    "edge, and the no-deadlock argument no longer holds"
                )
                raise HoldAndWaitError(msg)

    # ---------------------------------------------------------------- internals --
    @staticmethod
    def _normalise(resources: Iterable[str]) -> tuple[str, ...]:
        """Sort and de-duplicate a request.

        Sorting is for determinism only -- it makes the attempt replayable -- and it is
        emphatically **not** the Havender ordering rule, which ADR-0007 rejects. Nothing
        here compares a requested id against a held one, because a requesting robot holds
        nothing.
        """
        requested = tuple(sorted(set(resources)))
        if not requested:
            msg = "acquire_all needs at least one resource; an empty set is not a leg"
            raise ValueError(msg)
        if any(not r for r in requested):
            msg = f"resource ids must be non-empty strings, got {requested}"
            raise ValueError(msg)
        return requested

    def _rank(self, waiter: Waiter, now_ns: int) -> tuple[float, int]:
        """Sort key: lower is a better claim. Priority first, then arrival order."""
        return (-waiter.effective_priority(now_ns, self.queue.alpha_per_s), waiter.seq)

    def _outranked(
        self,
        robot_id: RobotId,
        requested: tuple[str, ...],
        now_ns: int,
        base_priority: float,
    ) -> bool:
        """Whether some other waiter for an overlapping set has a better claim."""
        mine = self._waiters.get(robot_id)
        my_rank = (
            self._rank(mine, now_ns)
            if mine is not None
            # Not yet queued: this robot has its base priority but zero seconds of
            # ageing, and arrives last, so it loses every tie. That is what makes a first
            # request join the back of the queue instead of barging past robots that have
            # been waiting -- while still letting a genuinely urgent newcomer through.
            else (-base_priority, self._next_seq)
        )
        wanted = set(requested)
        for other_id, other in self._waiters.items():
            if other_id == robot_id or wanted.isdisjoint(other.resources):
                continue
            if self._rank(other, now_ns) < my_rank:
                return True
        return False

    def _enqueue(
        self, robot_id: RobotId, requested: tuple[str, ...], now_ns: int, base_priority: float
    ) -> None:
        existing = self._waiters.get(robot_id)
        if existing is None:
            self._waiters[robot_id] = Waiter(
                robot_id=robot_id,
                resources=requested,
                since_ns=now_ns,
                base_priority=base_priority,
                seq=self._next_seq,
                last_seen_ns=now_ns,
            )
            self._next_seq += 1
            return
        # A retry keeps its place and its age; only what it wants and how urgent it is
        # are refreshed. Resetting since_ns here would make polling reset a robot's
        # seniority, and aging would stop working.
        self._waiters[robot_id] = Waiter(
            robot_id=robot_id,
            resources=requested,
            since_ns=existing.since_ns,
            base_priority=base_priority,
            seq=existing.seq,
            last_seen_ns=now_ns,
        )

    def _reap_waiters(self, now_ns: int) -> None:
        if self.queue.abandon_after_ns <= 0:
            return
        cutoff = now_ns - self.queue.abandon_after_ns
        for robot_id in [w.robot_id for w in self._waiters.values() if w.last_seen_ns <= cutoff]:
            del self._waiters[robot_id]

    def _mint(
        self, robot_id: RobotId, requested: tuple[str, ...], now_ns: int, ttl_ns: int
    ) -> Grant:
        tokens: dict[str, int] = {}
        for resource in requested:
            token = self._next_token.get(resource, 1)
            self._next_token[resource] = token + 1
            tokens[resource] = token
            self._holder[resource] = robot_id
        grant = Grant(
            robot_id=robot_id,
            resources=requested,
            tokens=MappingProxyType(tokens),
            granted_at_ns=now_ns,
            expires_at_ns=now_ns + ttl_ns,
        )
        self._grants[robot_id] = grant
        return grant

# ADR-0007 — Coordinator concurrency: atomic leg acquisition, not resource ordering

**Status:** Accepted, **implemented but NOT INTEGRATED** (see below) · **Refines:** ADR-0005

> ⚠️ **Integration gap, found by the Phase 6 review.**
>
> `parkscan_coord.leases` implements everything below — atomic acquisition, TTL leases,
> fencing tokens, FIFO queues with aging — and it is covered by its own tests, including the
> executable no-deadlock invariant and the fencing check driven straight at the resource
> controller. **But nothing calls it.** `coordinator.py` does not import it, and no robot
> ever enters `WAITING_ON_LEASE` in a running system.
>
> Collisions are currently avoided by a different mechanism entirely: robots appear in each
> other's range readings and the autonomy's local avoidance layer steers around them. That
> is what drives `conflict_count = 0` across all 90 benchmark runs — **not** the reservation
> layer described here.
>
> **What integration needs.** Leases were specified assuming the coordinator brokers
> movement, but the agent owns its own route (`parkscan_agent.lanegraph`), so the coordinator
> never learns which lane segments a robot is about to traverse. Closing that means either
> the agent requesting a lease for its next leg before it moves — giving the agent a handle
> on the coordinator, which the current layering deliberately avoids — or the coordinator
> tracking each robot's planned route. The charging bays are the tractable subset: there are
> two for three robots, the coordinator already knows when a robot is returning, and bay
> contention needs no route knowledge at all.
>
> Until then, this ADR describes a design that is built and proven in isolation, not a
> property of the running fleet, and the README says so.

## Context

`docs/ARCHITECTURE.md` §4 specifies breaking deadlock on two independent Coffman
conditions: **circular wait**, via a total order on resources (the Havender/Dijkstra
rule — never request an id lower than the highest currently held), and **no-preemption**,
via lease expiry.

Reviewing that against the implementation before it hardens, the first half does not
survive contact with the lane graph.

A robot acquires lane segments **in the order its route traverses them**, and a shortest
path has no reason to visit them in id order. Measured on the committed lot, the route
from `bay_1_node` to the last zone's entry acquires:

```
ln_bay_1_spur, ln_west_1 … ln_west_4, ln_r3_h0 … ln_r3_h6
```

which is not sorted — `ln_west_*` precedes `ln_r3_*` in traversal but follows it
lexicographically. Enforcing "never request a lower id than one you hold" would make this
perfectly ordinary route **illegal**. Renaming edges to match traversal order is not a fix:
the order depends on origin and destination, and reversing the trip reverses it.

So the rule as written is either violated on normal routes or forces routes to be planned
around lock ids rather than distance. Both are worse than the problem.

## Decision

Break **hold-and-wait** instead of circular wait: a robot acquires the **entire resource
set for its next leg atomically, or acquires nothing and waits**. Internally the
coordinator sorts the requested set by id, so the acquisition attempt itself is
deterministic and replayable; but the robot never holds a partial set, so a wait-for cycle
cannot form regardless of traversal order.

Lease expiry stays as the second, independent break (no-preemption), because atomic
acquisition does nothing about a robot that dies holding a bay.

Retained from ARCHITECTURE §4 unchanged: leases with TTL and renewal, **fencing tokens**,
FIFO queues with aging, and per-zone assignment epochs.

## Rationale

At three robots the concurrency lost to all-or-nothing acquisition is negligible — the
research put this plainly: at this scale it is "the simplest thing that provably works".
The deadlock argument also becomes a one-liner a reviewer can check: *no robot ever holds
one resource while waiting for another, so there is no wait-for edge, so there is no
cycle.* That is stronger and shorter than an ordering invariant that normal routes break.

## Consequences

- The lease manager needs `acquire_all(robot, resources, ttl) -> granted | None`. There is
  no partial-acquire path, and adding one later would reintroduce hold-and-wait.
- **Starvation** becomes the live hazard rather than deadlock, so the FIFO queue with
  aging is now load-bearing rather than a nicety, and needs its own test: a robot that
  keeps losing must eventually win.
- Contention has to be made real or none of this is exercised. Most of the 52 lane edges
  can hold three robots without meeting, so the two **bay spurs** (`ln_bay_1_spur`,
  `ln_bay_2_spur`) plus a small number of named connector segments are declared
  single-occupancy in config. Both bays are already scarcer than the fleet by design.
- One fencing implementation serves both the bay lease and the zone assignment epoch:
  each is a monotonic counter whose holder must present the current value. They are the
  same mechanism, and VDA 5050 uses exactly this pairing (`orderId` / `orderUpdateId`).
- ARCHITECTURE §4's resource-ordering paragraph is superseded by this ADR. It is left in
  place with a pointer, rather than deleted, because the reasoning for *why* it does not
  work here is worth more than the tidier document.

## What this review did NOT change

The allocator (greedy / hungarian / SSI), the failure-detection triad (MQTT Last Will +
heartbeat deadline + stuck detection excluding `WAITING_ON_LEASE`), the four recovery
hazards and their preventions, and the rejection of CBBA all survived review unchanged and
are implemented as specified.

## Known weakness, stated rather than hidden

In simulation the coordinator is both the lease granter and the resource controller that
rejects a stale token. In a real deployment those are separate processes, and fencing only
means something when they are — a controller colocated with the granter can always be
"corrected" by it. The fencing test must therefore drive the resource controller
**directly** with a stale token, never through the granter, or it proves nothing. Noted in
the test itself.

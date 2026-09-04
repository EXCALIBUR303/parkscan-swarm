# ADR-0007 — Coordinator concurrency: atomic leg acquisition, not resource ordering

**Status:** Accepted, **implemented and integrated for charging bays** (see below) ·
**Refines:** ADR-0005

> ✅ **Integration gap, found by the Phase 6 review — now closed for bays.**
>
> `parkscan_coord.leases` implements atomic acquisition, TTL leases, fencing tokens, and
> FIFO queues with aging, and is covered by its own tests, including the executable
> no-deadlock invariant and the fencing check driven straight at the resource controller.
> The Phase 6 review found that none of it was wired up: `coordinator.py` did not import
> it, and no robot ever entered `WAITING_ON_LEASE` in a running system. Collisions were
> avoided by a different mechanism entirely — local avoidance steering robots around each
> other — which is not a substitute for arbitrating who gets a bay.
>
> **What integration needed**, and what it turned out to need in practice, differed from
> the plan below in one respect: lane-segment leases (the general case, agent-owned
> routing) remain unintegrated, exactly as scoped — closing that still means either the
> agent requesting a lease for its next leg before it moves, or the coordinator tracking
> each robot's planned route, and neither has been done. **Bay leases are integrated.**
> `Coordinator._manage_bay_lease` now calls `LeaseManager.acquire_all` every tick a robot
> is `RETURNING`, `WAITING_ON_LEASE`, or `CHARGING`, and a robot genuinely enters
> `WAITING_ON_LEASE` when every reachable bay is taken.
>
> Getting bay leases correctly wired up took four layered fixes, not one call site:
>
> 1. **Arbitration.** Two robots picking the same nearest bay independently used to
>    collide on final approach, where avoidance is deliberately off (docking is a
>    precision manoeuvre, not a swerve). `acquire_all` now grants the bay to exactly one
>    robot; every other robot calls `agent.hold(...)` and waits for
>    `Event.BAY_LEASE_GRANTED`.
> 2. **Physical handoff, not just logical release.** Releasing a bay's lease the instant
>    its holder reports `IDLE` let the next robot be granted a bay whose spur was still
>    physically blocked by the (logically vacated, physically stationary) previous
>    holder — it got wedged and timed out. `_clear_of_bay` now withholds release until the
>    former holder has put `AgentConfig.dock.approach_radius_m` behind it.
> 3. **Multi-bay fallback before queueing.** A robot denied its nearest bay used to queue
>    with no upper bound on wait time, even with a second, empty bay sitting idle — fatal,
>    since the robot's own return-to-charge decision budgets zero slack for waiting, and
>    idle draw keeps consuming charge while queued. `ranked_bays` plus
>    `RobotAgent.retarget_bay` now try every bay, nearest first, before ever queueing.
> 4. **Feasibility-gated redirect.** Fix 3 alone could redirect a robot to a farther bay
>    it could not actually reach on remaining charge — worse than queueing, since a doomed
>    drive burns exactly the charge that holding position would have conserved. Candidates
>    are now filtered to `battery_soc >= required_soc(bay.distance_m, ...)` before
>    `acquire_all` is even attempted; only if *no* bay is charge-feasible does the
>    fallback try the full ranked list anyway (refusing to act at all helps nothing
>    further at that point).
>
> Fix 4 surfaced a **separate, pre-existing bug** in
> `parkscan_agent.battery_policy.BatteryParams.soc_per_metre`: the constant (`1/2500`)
> underestimated real travel cost by more than 2x against the simulator's own
> `idle_w`/`move_w_per_mps`/`capacity_wh` at the actual cruise speed (`NavConfig.max_v_mps
> == 1.2`), so `required_soc` could clear a robot for a bay it would run flat before
> reaching — the 25% margin cannot absorb a 2x error. It predates this integration and
> affected every return-to-charge decision project-wide, not only bay redirects; it was
> only exposed here because redirect sends robots on longer, multi-turn routes that push
> closer to the model's edge. Fixed by deriving the constant from the simulator's real
> constants instead of an unmeasured guess (see the field's docstring in
> `battery_policy.py`).

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

Fencing tokens are also not wired into the physical docking action for bays: a "resurrected
holder" — one that stalls past its TTL, gets its lease reclaimed and reassigned, then
resumes — is not fenced off from the bay it no longer legitimately holds. This was already
a documented ADR-0007 gap before the bay-lease integration and the integration does not
close it; closing it needs a `ResourceController` in the docking path, which does not exist
yet.

A genuine three-way simultaneous return on a two-bay lot still leaves one robot with no
upper bound on wait time — fix 3 above (multi-bay fallback) has nowhere left to redirect to
once every bay is taken. This is accepted as a narrow, load-bearing limitation rather than
engineered against further: with 2 bays and 3 robots, "bay contention has to be real" (see
Consequences, above) means some contention is by design, not a bug to eliminate.

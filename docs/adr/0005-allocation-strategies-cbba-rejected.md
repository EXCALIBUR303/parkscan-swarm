# ADR-0005 — Greedy + Hungarian + SSI auction; CBBA rejected

**Status:** Accepted (Phase 0)

## Context
Assigning 12 zones to 3 robots is **ST-SR-TA** (Gerkey & Matarić), scheduling class `R ∥ Σ w_j C_j`,
strongly NP-hard. Lane and bay contention makes utilities interdependent, so the iTax label is
**XD [ST-SR-TA]** (Korsah et al. 2013).

## Decision
Approximate as **iterated ST-SR-IA**. Ship three strategies over one shared cost function:
`greedy` (default), `hungarian` (`scipy.optimize.linear_sum_assignment`, optimal per-round
reference), `ssi` (sequential single-item auction with marginal-cost / BidSumPath bids).
**CBBA is rejected.** Cross-schedule dependency is decoupled into a separate reservation layer.

## Rationale
The Hungarian baseline exists so greedy's gap is **proved rather than asserted** — a property test
over 1000 random matrices plus a pinned regression case where greedy is 6.29× worse. This also
corrects the most common error in portfolio write-ups: the famous "greedy is 2-competitive" result
is for **maximising utility**; under cost minimisation the ratio is **unbounded** (measured ~5×10⁵
on an adversarial matrix). SSI carries a real theorem (≤2× optimal on MiniSum) for ~100 LOC that
reuses the same cost function, making the comparison apples-to-apples.

CBBA is 394–920 LOC of consensus logic (two reference implementations measured) whose premise —
inconsistent situational awareness over a time-varying comms graph — **does not hold** in a system
with one MQTT broker and a central coordinator. Implementing it and then claiming decentralised
allocation would be precisely the overclaim this project forbids. CBS/optimal MAPF is rejected on
fleet size for the same style of reason.

## Consequences
- `allocate(state) -> list[Assignment]` is a pure function, unit-testable without a simulator.
- Comparison reports **makespan and total distance together**, over 30 fixed seeds, with paired
  bootstrap 95% CIs and win/loss/tie counts. If the difference is not significant at n=3, m=12,
  the README says so.

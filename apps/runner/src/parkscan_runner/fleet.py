"""Adapting the coordinator's :class:`AllocationRequest` to the allocator's signature.

Two shapes exist on purpose. The coordinator asks "who is available, what is outstanding,
and why am I being asked" (:class:`AllocationRequest`); the allocator answers "given these
robots and these zones, who takes what" (:class:`AllocatorState`). Keeping them apart is
what lets ``allocate`` stay a pure function with no notion of health, epochs or triggers —
and therefore what lets the 30-seed benchmark swap strategies without touching recovery.

This module is the ten lines that join them, and the only place that knows both.
"""

from __future__ import annotations

from collections.abc import Sequence

from parkscan_coord.allocation import AllocatorState, Assignment, Strategy, allocate
from parkscan_coord.coordinator import AllocationRequest


class StrategyAllocator:
    """An :class:`~parkscan_coord.coordinator.Allocator` bound to one strategy."""

    __slots__ = ("_strategy",)

    def __init__(self, strategy: Strategy = Strategy.GREEDY) -> None:
        self._strategy = strategy

    @property
    def strategy(self) -> Strategy:
        return self._strategy

    def __call__(self, request: AllocationRequest) -> Sequence[Assignment]:
        # Every demand is passed through with its state; `allocate` allocates only the
        # UNASSIGNED ones. Taking a zone off a robot that already holds it is a
        # discretionary swap, and the coordinator's hysteresis, cooldown and cap guards
        # own that decision — not the allocator.
        if not request.robots or not request.demands:
            return []
        state = AllocatorState(
            robots=tuple(request.robots),
            zones={demand.zone_id: demand.state for demand in request.demands},
        )
        return allocate(state, request.lot, request.params, strategy=self._strategy)

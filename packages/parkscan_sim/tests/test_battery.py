"""``parkscan_sim.battery`` is a re-export shim (ADR-0008: relocated to ``parkscan_core``
so ``parkscan_ros2`` can use it without depending on a peer backend). The real arithmetic
tests live at ``packages/parkscan_core/tests/test_battery.py``; this only pins that the
shim exports the *same objects*, not copies, so the two packages cannot silently drift
onto two different battery models."""

from __future__ import annotations

import parkscan_sim.battery as sim

import parkscan_core.battery as core


def test_the_shim_reexports_the_exact_same_objects_not_copies() -> None:
    for name in ("BatteryParams", "charge_soc", "discharge_power_w", "drain_soc", "endurance_s"):
        assert getattr(sim, name) is getattr(core, name), name

"""Re-exported from :mod:`parkscan_core.battery`.

Relocated there by ADR-0008: a second backend (``parkscan_ros2``) needs this same pure,
backend-agnostic proxy, and depending on a peer backend package just to reach it was the
wrong shape. This module exists only so every import site here (``parkscan_sim.world``,
``parkscan_sim.config``, this package's own tests) keeps working unchanged.
"""

from __future__ import annotations

from parkscan_core.battery import (
    BatteryParams,
    charge_soc,
    discharge_power_w,
    drain_soc,
    endurance_s,
)

__all__ = [
    "BatteryParams",
    "charge_soc",
    "discharge_power_w",
    "drain_soc",
    "endurance_s",
]

"""A linear battery proxy -- and a plain statement of what it is not.

**This does not model cell chemistry.** There is no open-circuit-voltage curve, no
internal resistance, no temperature term, no Peukert effect, no ageing and no
state-of-charge estimator. State of charge is a number in ``[0, 1]`` that falls linearly
with time and speed and rises linearly while docked. Nothing derived from it may be
described as a battery-life prediction.

It exists because the coordinator's return-to-charge rule is a *feasibility check*
(``docs/ARCHITECTURE.md`` section 3)::

    soc < k * d_travel(pos, nearest_bay(pos)) * (1 + margin) + soc_reserve

and that rule needs *something* honest to reason about. A linear model is the smallest
thing that makes the check meaningful: energy per metre is constant, so ``k`` is a real
constant of the model rather than a fudge factor, and "can I still reach a bay" has an
exact answer that a test can check. A more elaborate curve would add realism-flavoured
detail that no measurement in this project supports.

The model::

    P(v)     = idle_w + move_w_per_mps * |v|          [W]
    dE       = P(v) * dt / 3600                       [Wh]
    soc'     = clamp(soc - dE / capacity_wh, 0, 1)
    charging = clamp(soc + charge_w * dt / 3600 / capacity_wh, 0, 1)

The defaults are chosen for demo and benchmark *timescales*, not from any datasheet:
20 Wh with 8 W idle and 55 W per m/s gives roughly 16 minutes of driving at 1.2 m/s and a
10-minute recharge, so over a full-lot run a robot must dock two or three times and
bay contention between three robots and two bays is real rather than decorative.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = [
    "BatteryParams",
    "charge_soc",
    "discharge_power_w",
    "drain_soc",
    "endurance_s",
]

_SECONDS_PER_HOUR = 3600.0


@dataclass(frozen=True, slots=True)
class BatteryParams:
    """Coefficients of the linear proxy. See the module docstring for what they are not."""

    capacity_wh: float = 20.0
    idle_w: float = 8.0
    move_w_per_mps: float = 55.0
    charge_w: float = 120.0

    def __post_init__(self) -> None:
        for name in ("capacity_wh", "idle_w", "move_w_per_mps", "charge_w"):
            value: float = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                msg = f"{name} must be finite and positive, got {value}"
                raise ValueError(msg)


def _check_soc(soc: float) -> None:
    if not math.isfinite(soc) or not 0.0 <= soc <= 1.0:
        msg = f"soc must be a finite value in [0, 1], got {soc}"
        raise ValueError(msg)


def _check_dt(dt_s: float) -> None:
    if not math.isfinite(dt_s) or dt_s < 0.0:
        msg = f"dt_s must be finite and non-negative, got {dt_s}"
        raise ValueError(msg)


def discharge_power_w(v_mps: float, params: BatteryParams) -> float:
    """Instantaneous draw at body speed ``v_mps``. Direction does not matter, so ``abs``."""
    return params.idle_w + params.move_w_per_mps * abs(v_mps)


def drain_soc(soc: float, v_mps: float, dt_s: float, params: BatteryParams) -> float:
    """State of charge after ``dt_s`` at speed ``v_mps``. Clamped at ``0.0``, never below.

    Clamping rather than going negative is deliberate: a negative state of charge would
    propagate into the feasibility check as a *more* pessimistic number and quietly make
    the rule look conservative when in fact the robot is already flat. Depletion is
    surfaced by ``SimLite.is_depleted`` and counted by acceptance criterion C2 instead.
    """
    _check_soc(soc)
    _check_dt(dt_s)
    used_wh = discharge_power_w(v_mps, params) * dt_s / _SECONDS_PER_HOUR
    return max(0.0, soc - used_wh / params.capacity_wh)


def charge_soc(soc: float, dt_s: float, params: BatteryParams) -> float:
    """State of charge after ``dt_s`` docked at a bay. Clamped at ``1.0``.

    Constant-power charging all the way to full is the least realistic part of this
    model -- a real pack tapers -- and it is kept because the alternative is a taper
    curve nobody measured. It makes the sim optimistic about the *last* few percent of a
    recharge and that bias is stated here rather than buried.
    """
    _check_soc(soc)
    _check_dt(dt_s)
    gained_wh = params.charge_w * dt_s / _SECONDS_PER_HOUR
    return min(1.0, soc + gained_wh / params.capacity_wh)


def endurance_s(soc: float, v_mps: float, params: BatteryParams) -> float:
    """Seconds of driving left at a constant ``v_mps``, under this proxy.

    Provided here so the return-to-charge feasibility predicate has one place to get the
    number from, instead of every caller re-deriving the same division slightly wrong.
    """
    _check_soc(soc)
    return soc * params.capacity_wh * _SECONDS_PER_HOUR / discharge_power_w(v_mps, params)

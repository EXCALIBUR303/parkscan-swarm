"""The linear battery proxy. These tests check the arithmetic and the clamps -- they are
not, and must never be presented as, a validation of any real battery."""

from __future__ import annotations

import math

import pytest

from parkscan_core.battery import (
    BatteryParams,
    charge_soc,
    discharge_power_w,
    drain_soc,
    endurance_s,
)

P = BatteryParams()
DT = 0.05


def test_draw_is_idle_plus_a_linear_speed_term_and_ignores_direction() -> None:
    assert discharge_power_w(0.0, P) == P.idle_w
    assert discharge_power_w(1.0, P) == P.idle_w + P.move_w_per_mps
    assert discharge_power_w(-1.0, P) == discharge_power_w(1.0, P)


def test_drain_matches_the_documented_arithmetic() -> None:
    # One hour at rest costs idle_w watt-hours out of capacity_wh.
    params = BatteryParams(capacity_wh=20.0, idle_w=8.0, move_w_per_mps=55.0, charge_w=120.0)
    assert drain_soc(1.0, 0.0, 3600.0, params) == pytest.approx(1.0 - 8.0 / 20.0)
    # One hour at 1 m/s costs (8 + 55) Wh, which is more than the pack holds -> flat.
    assert drain_soc(1.0, 1.0, 3600.0, params) == 0.0


def test_state_of_charge_decreases_monotonically_while_moving() -> None:
    soc = 1.0
    previous = soc
    for _ in range(2000):
        soc = drain_soc(soc, 1.2, DT, P)
        assert soc <= previous
        previous = soc
    assert soc < 1.0


def test_state_of_charge_never_goes_below_zero_however_long_it_runs() -> None:
    soc = 0.02
    for _ in range(10_000):
        soc = drain_soc(soc, 1.2, DT, P)
        assert soc >= 0.0
    assert soc == 0.0


def test_moving_drains_strictly_faster_than_idling() -> None:
    moving = drain_soc(1.0, 1.2, DT, P)
    idling = drain_soc(1.0, 0.0, DT, P)
    assert moving < idling < 1.0


def test_charging_increases_monotonically_and_clamps_at_one() -> None:
    soc = 0.1
    previous = soc
    for _ in range(20_000):
        soc = charge_soc(soc, DT, P)
        assert soc >= previous
        assert soc <= 1.0
        previous = soc
    assert soc == 1.0


def test_charging_matches_the_documented_arithmetic() -> None:
    params = BatteryParams(capacity_wh=20.0, idle_w=8.0, move_w_per_mps=55.0, charge_w=120.0)
    # 120 W for 60 s is 2 Wh, which is a tenth of a 20 Wh pack.
    assert charge_soc(0.5, 60.0, params) == pytest.approx(0.6)


def test_a_zero_length_tick_changes_nothing() -> None:
    assert drain_soc(0.42, 1.2, 0.0, P) == 0.42
    assert charge_soc(0.42, 0.0, P) == 0.42


def test_endurance_is_consistent_with_the_drain_it_predicts() -> None:
    seconds = endurance_s(1.0, 1.2, P)
    assert drain_soc(1.0, 1.2, seconds, P) == pytest.approx(0.0, abs=1e-12)
    # And one tick short of that, the robot is still (just) alive.
    assert drain_soc(1.0, 1.2, seconds - DT, P) > 0.0


def test_default_parameters_give_the_documented_demo_timescales() -> None:
    """The defaults are a modelling choice for demo timescales, and this pins them."""
    assert endurance_s(1.0, 1.2, P) / 60.0 == pytest.approx(16.2, abs=0.2)
    assert (P.capacity_wh / P.charge_w) * 60.0 == pytest.approx(10.0, abs=0.1)


@pytest.mark.parametrize("bad", [-0.001, 1.001, math.nan, math.inf])
def test_functions_reject_an_out_of_range_state_of_charge(bad: float) -> None:
    with pytest.raises(ValueError, match="soc"):
        drain_soc(bad, 0.0, DT, P)
    with pytest.raises(ValueError, match="soc"):
        charge_soc(bad, DT, P)
    with pytest.raises(ValueError, match="soc"):
        endurance_s(bad, 1.0, P)


@pytest.mark.parametrize("bad", [-0.1, math.nan, math.inf])
def test_functions_reject_a_bad_tick(bad: float) -> None:
    with pytest.raises(ValueError, match="dt_s"):
        drain_soc(0.5, 0.0, bad, P)
    with pytest.raises(ValueError, match="dt_s"):
        charge_soc(0.5, bad, P)


@pytest.mark.parametrize("field", ["capacity_wh", "idle_w", "move_w_per_mps", "charge_w"])
@pytest.mark.parametrize("bad", [0.0, -1.0, math.nan, math.inf])
def test_battery_params_reject_nonsense(field: str, bad: float) -> None:
    with pytest.raises(ValueError, match=field):
        BatteryParams(**{field: bad})

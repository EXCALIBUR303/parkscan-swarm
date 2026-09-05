"""``quaternion_to_yaw`` against known orientations -- the identity quaternion, and each
axis-aligned quarter-turn, values anyone can check by hand."""

from __future__ import annotations

import math

import pytest
from parkscan_ros2.bridge.geometry import quaternion_to_yaw


def test_identity_quaternion_is_zero_yaw() -> None:
    assert quaternion_to_yaw(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0, abs=1e-12)


def test_a_quarter_turn_about_z_is_positive_half_pi() -> None:
    # (x, y, z, w) for a +90 deg rotation about Z: (0, 0, sin(45deg), cos(45deg))
    s = math.sin(math.pi / 4)
    c = math.cos(math.pi / 4)
    assert quaternion_to_yaw(0.0, 0.0, s, c) == pytest.approx(math.pi / 2, abs=1e-9)


def test_a_half_turn_about_z_is_pi() -> None:
    assert quaternion_to_yaw(0.0, 0.0, 1.0, 0.0) == pytest.approx(math.pi, abs=1e-9)


def test_a_negative_quarter_turn_is_negative_half_pi() -> None:
    s = math.sin(-math.pi / 4)
    c = math.cos(-math.pi / 4)
    assert quaternion_to_yaw(0.0, 0.0, s, c) == pytest.approx(-math.pi / 2, abs=1e-9)


def test_roll_and_pitch_only_no_yaw_is_still_zero() -> None:
    """A quaternion representing pure roll or pitch (no Z rotation) must yield ~0 yaw --
    this project's robots never leave the plane, but the formula should still degrade
    correctly if it ever received one."""
    # 90 degree roll about X: (sin(45deg), 0, 0, cos(45deg))
    s = math.sin(math.pi / 4)
    c = math.cos(math.pi / 4)
    assert quaternion_to_yaw(s, 0.0, 0.0, c) == pytest.approx(0.0, abs=1e-9)

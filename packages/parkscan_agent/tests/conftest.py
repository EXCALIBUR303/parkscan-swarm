"""Fixtures for the autonomy tests.

The backend used here is :class:`parkscan_core.testing.FakeBackend`, never
``parkscan_sim``. ADR-0001 requires the autonomy to run against any ``WorldBackend``, and
``test_robot_docking.py::test_autonomy_never_imports_the_simulator`` asserts that
boundary holds as the code grows.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from parkscan_core.lot import LotSpec, load_lot
from parkscan_core.testing import FakeBackend

REPO_ROOT = Path(__file__).resolve().parents[3]
LOT_YAML = REPO_ROOT / "sim" / "lot" / "lot.yaml"


@pytest.fixture(scope="session")
def lot() -> LotSpec:
    return load_lot(LOT_YAML)


@pytest.fixture
def backend_factory() -> type[FakeBackend]:
    """The double itself, so each test builds one with its own start poses."""
    return FakeBackend

"""Shared fixtures: the committed lot spec that every sim-lite run is built from."""

from __future__ import annotations

from pathlib import Path

import pytest

from parkscan_core.lot import LotSpec, load_lot

REPO_ROOT = Path(__file__).resolve().parents[3]
LOT_YAML = REPO_ROOT / "sim" / "lot" / "lot.yaml"


@pytest.fixture(scope="session")
def lot() -> LotSpec:
    return load_lot(LOT_YAML)

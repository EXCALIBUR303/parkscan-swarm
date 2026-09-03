"""Shared fixtures: the committed lot spec and its generated GeoJSON."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from parkscan_core.lot import LotSpec, load_lot

REPO_ROOT = Path(__file__).resolve().parents[3]
LOT_YAML = REPO_ROOT / "sim" / "lot" / "lot.yaml"
LOT_GEOJSON = REPO_ROOT / "sim" / "lot" / "lot.geojson"


@pytest.fixture(scope="session")
def lot() -> LotSpec:
    return load_lot(LOT_YAML)


@pytest.fixture(scope="session")
def lot_geojson() -> dict[str, Any]:
    with LOT_GEOJSON.open("r", encoding="utf-8") as fh:
        loaded: dict[str, Any] = json.load(fh)
    return loaded

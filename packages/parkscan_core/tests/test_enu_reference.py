"""Cross-language guard for the lot-metres <-> WGS84 projection.

The projection is implemented twice, independently: here in Python
(``parkscan_core.geo``) and in TypeScript (``apps/web/src/lib/enu.ts``). Nothing in
either codebase would fail if the two silently diverged -- robots and detections would
simply be drawn in the wrong place on the map, consistently, with every test green.

So both sides assert against one committed fixture: ``sim/lot/enu_reference.json``.
The mirror of this test is ``apps/web/src/lib/enu.reference.test.ts``. If you change the
projection, both tests fail until you regenerate the fixture deliberately.
"""

import json
from pathlib import Path
from typing import Any

import pytest

from parkscan_core.geo import Origin, lonlat_to_lot, lot_to_lonlat

REFERENCE = Path(__file__).resolve().parents[3] / "sim" / "lot" / "enu_reference.json"


@pytest.fixture(scope="module")
def reference() -> dict[str, Any]:
    assert REFERENCE.is_file(), f"missing cross-language fixture: {REFERENCE}"
    loaded: object = json.loads(REFERENCE.read_text())
    assert isinstance(loaded, dict)
    return loaded


def test_fixture_matches_python_implementation(reference: dict[str, Any]) -> None:
    origin = Origin(lat=reference["origin"]["lat"], lon=reference["origin"]["lon"])
    tol = reference["tolerance_deg"]

    for point in reference["points"]:
        lon, lat = lot_to_lonlat(point["x_m"], point["y_m"], origin)
        assert abs(lon - point["lon"]) <= tol, f"lon drift at ({point['x_m']}, {point['y_m']})"
        assert abs(lat - point["lat"]) <= tol, f"lat drift at ({point['x_m']}, {point['y_m']})"


def test_fixture_round_trips_back_to_lot_metres(reference: dict[str, Any]) -> None:
    origin = Origin(lat=reference["origin"]["lat"], lon=reference["origin"]["lon"])

    for point in reference["points"]:
        x, y = lonlat_to_lot(point["lon"], point["lat"], origin)
        assert abs(x - point["x_m"]) < 1e-6
        assert abs(y - point["y_m"]) < 1e-6


def test_fixture_origin_is_synthetic(reference: dict[str, Any]) -> None:
    """Null Island, not a real address.

    The fallback origin must never be a plausible land coordinate: a lot file with no
    declared origin would then render simulated robots on top of somewhere real, which
    is exactly what ADR-0004 forbids.
    """
    assert reference["origin"] == {"lat": 0.0, "lon": 0.0}

"""ENU <-> WGS84: acceptance criterion C10 (round trip within 1 mm)."""

from __future__ import annotations

import math

import pytest

from parkscan_core.geo import (
    WGS84_A,
    WGS84_E2,
    Origin,
    curvature_radii,
    lonlat_to_lot,
    lot_to_lonlat,
)

LOT_W, LOT_H = 150.0, 100.0
TOLERANCE_M = 1e-3  # C10: 1 mm

ORIGINS = [
    Origin(lat=0.0, lon=0.0),  # the lot's own origin: obviously synthetic, zero convergence
    Origin(lat=37.7749, lon=-122.4194),
    Origin(lat=-33.8688, lon=151.2093),
    Origin(lat=60.1699, lon=24.9384),
]


def _grid(step: float = 10.0) -> list[tuple[float, float]]:
    xs = [x * step for x in range(int(LOT_W / step) + 1)]
    ys = [y * step for y in range(int(LOT_H / step) + 1)]
    return [(x, y) for x in xs for y in ys]


@pytest.mark.parametrize("origin", ORIGINS, ids=lambda o: f"lat{o.lat}")
def test_round_trip_over_the_lot_grid_is_sub_millimetre(origin: Origin) -> None:
    """C10. Every lot point survives metres -> degrees -> metres."""
    worst = 0.0
    points = _grid()
    assert len(points) == 176
    for x, y in points:
        lon, lat = lot_to_lonlat(x, y, origin)
        x2, y2 = lonlat_to_lot(lon, lat, origin)
        worst = max(worst, math.dist((x, y), (x2, y2)))
    assert worst < TOLERANCE_M, f"worst round-trip error {worst:.3e} m at origin {origin}"


def test_round_trip_error_is_far_below_the_budget_at_the_lot_origin() -> None:
    """Not just under 1 mm -- under a nanometre, because the inverse is algebraic."""
    origin = Origin(lat=0.0, lon=0.0)
    worst = 0.0
    for x, y in _grid(step=5.0):
        lon, lat = lot_to_lonlat(x, y, origin)
        x2, y2 = lonlat_to_lot(lon, lat, origin)
        worst = max(worst, math.dist((x, y), (x2, y2)))
    assert worst < 1e-6


def test_reverse_round_trip_from_degrees() -> None:
    origin = Origin(lat=37.7749, lon=-122.4194)
    for dlon, dlat in ((1e-4, 5e-5), (-8e-4, 9e-4), (0.0, 0.0)):
        lon, lat = origin.lon + dlon, origin.lat + dlat
        x, y = lonlat_to_lot(lon, lat, origin)
        lon2, lat2 = lot_to_lonlat(x, y, origin)
        assert abs(lon2 - lon) < 1e-12
        assert abs(lat2 - lat) < 1e-12


def test_origin_maps_to_itself() -> None:
    for origin in ORIGINS:
        lon, lat = lot_to_lonlat(0.0, 0.0, origin)
        assert lon == pytest.approx(origin.lon, abs=1e-15)
        assert lat == pytest.approx(origin.lat, abs=1e-15)
        assert lonlat_to_lot(origin.lon, origin.lat, origin) == (0.0, 0.0)


def test_axes_point_east_and_north() -> None:
    origin = Origin(lat=0.0, lon=0.0)
    lon_e, lat_e = lot_to_lonlat(100.0, 0.0, origin)
    assert lon_e > origin.lon
    assert lat_e == pytest.approx(origin.lat, abs=1e-15)

    lon_n, lat_n = lot_to_lonlat(0.0, 100.0, origin)
    assert lat_n > origin.lat
    assert lon_n == pytest.approx(origin.lon, abs=1e-15)


def test_scale_matches_the_closed_form_at_the_equator() -> None:
    """1 m north at the equator is 1 / (a(1-e^2)) radians; 1 m east is 1 / a radians."""
    origin = Origin(lat=0.0, lon=0.0)
    lon, lat = lot_to_lonlat(1.0, 1.0, origin)
    assert math.radians(lat) == pytest.approx(1.0 / (WGS84_A * (1.0 - WGS84_E2)), rel=1e-12)
    assert math.radians(lon) == pytest.approx(1.0 / WGS84_A, rel=1e-12)


def test_east_scale_shrinks_with_latitude() -> None:
    """The same eastward metre spans more longitude the further from the equator."""
    dlons = []
    for lat in (0.0, 30.0, 60.0):
        origin = Origin(lat=lat, lon=0.0)
        lon, _ = lot_to_lonlat(100.0, 0.0, origin)
        dlons.append(lon)
    assert dlons[0] < dlons[1] < dlons[2]


def test_curvature_radii_are_the_wgs84_values() -> None:
    r_mer, r_norm = curvature_radii(0.0)
    assert r_norm == pytest.approx(WGS84_A, rel=1e-15)
    assert r_mer == pytest.approx(6335439.327, abs=1e-3)
    r_mer_pole, r_norm_pole = curvature_radii(90.0)
    assert r_norm_pole == pytest.approx(6399593.626, abs=1e-3)
    assert r_mer_pole == pytest.approx(r_norm_pole, rel=1e-12)


@pytest.mark.parametrize("lat", [90.0, -90.0, 89.5, -89.9])
def test_origin_rejects_latitudes_where_the_model_breaks_down(lat: float) -> None:
    with pytest.raises(ValueError, match="pole"):
        Origin(lat=lat, lon=0.0)


@pytest.mark.parametrize(("lat", "lon"), [(91.0, 0.0), (-91.0, 0.0), (0.0, 181.0), (0.0, -181.0)])
def test_origin_rejects_out_of_range_coordinates(lat: float, lon: float) -> None:
    with pytest.raises(ValueError, match="out of range"):
        Origin(lat=lat, lon=lon)

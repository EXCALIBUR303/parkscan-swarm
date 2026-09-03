"""Local tangent-plane (ENU) <-> WGS84 conversion for the lot frame.

This is the Python side of a two-language pair: ``apps/web/src/lib/enu.ts`` implements
exactly the same formula so that a point placed by the planner and a point drawn by
MapLibre land in the same place. If you change the maths here, change it there, and
re-run both round-trip tests.

Why a conversion exists at all
------------------------------
Everything above the simulator adapter works in **lot metres** (REP-103: ``x`` east,
``y`` north, flat ground). MapLibre GL is Web-Mercator-only and PostGIS proximity queries
want ``geography(Point, 4326)``, so metres have to become degrees somewhere. Doing it in
one ~40-line module, tested to sub-millimetre round-trip, is cheaper and far more
auditable than dragging ``pyproj`` in for a 150 m by 100 m rectangle.

The approximation
-----------------
A first-order local tangent plane at the configured origin::

    dlat = y_m / R_meridional(lat0)
    dlon = x_m / (R_normal(lat0) * cos(lat0))

with the WGS84 radii of curvature evaluated **once, at the origin latitude**::

    w            = sqrt(1 - e^2 sin^2(lat0))
    R_normal     = a / w                 (prime vertical)
    R_meridional = a (1 - e^2) / w^3

Holding the radii fixed is the entire approximation. The error it introduces is second
order in the offset: the curvature terms change by roughly ``(d / R)`` in relative terms,
so over a ``d = 150 m`` lot the north/east scale is wrong by about ``2e-5`` relative,
i.e. under 3 mm at the far corner. That is well below the width of a painted parking
stripe and far below the simulator's own pose noise, so it is not worth a projection
library. The approximation degrades as ``d^2 / R`` and should not be reused for anything
larger than a few kilometres.

The **round trip** is exact regardless of the approximation, because both directions use
the same two constants; :func:`lonlat_to_lot` is the algebraic inverse of
:func:`lot_to_lonlat`, so the only round-trip error is IEEE-754 rounding (order 1e-9 m
over this lot). Acceptance criterion C10 requires it to stay under 1 mm.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = [
    "WGS84_A",
    "WGS84_E2",
    "WGS84_F",
    "Origin",
    "curvature_radii",
    "lonlat_to_lot",
    "lot_to_lonlat",
]

WGS84_A: float = 6378137.0
"""WGS84 semi-major axis, metres."""

WGS84_F: float = 1.0 / 298.257223563
"""WGS84 flattening."""

WGS84_E2: float = WGS84_F * (2.0 - WGS84_F)
"""WGS84 first eccentricity squared."""

_MAX_ABS_LAT_DEG: float = 89.0
"""Beyond this the ``1 / cos(lat0)`` east scale blows up and the model is meaningless."""


@dataclass(frozen=True, slots=True)
class Origin:
    """The WGS84 anchor of the lot's local metric frame.

    ``lat``/``lon`` are degrees. Lot coordinates ``(0, 0)`` map to exactly this point,
    ``+x`` is east and ``+y`` is north.
    """

    lat: float
    lon: float

    def __post_init__(self) -> None:
        if not -90.0 <= self.lat <= 90.0:
            msg = f"origin latitude out of range: {self.lat}"
            raise ValueError(msg)
        if not -180.0 <= self.lon <= 180.0:
            msg = f"origin longitude out of range: {self.lon}"
            raise ValueError(msg)
        if abs(self.lat) > _MAX_ABS_LAT_DEG:
            msg = (
                f"origin latitude {self.lat} is inside {90 - _MAX_ABS_LAT_DEG} degrees of a "
                "pole; the local tangent-plane approximation is not usable there"
            )
            raise ValueError(msg)


def curvature_radii(lat_deg: float) -> tuple[float, float]:
    """Return ``(R_meridional, R_normal)`` in metres at ``lat_deg``.

    ``R_meridional`` scales north/south offsets, ``R_normal`` (the prime-vertical radius)
    scales east/west offsets before the ``cos(lat)`` convergence factor.
    """
    lat = math.radians(lat_deg)
    w = math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    r_normal = WGS84_A / w
    r_meridional = WGS84_A * (1.0 - WGS84_E2) / (w**3)
    return r_meridional, r_normal


def lot_to_lonlat(x_m: float, y_m: float, origin: Origin) -> tuple[float, float]:
    """Lot metres -> ``(lon_deg, lat_deg)``.

    Returned lon-first because that is GeoJSON's coordinate order (RFC 7946 §3.1.1) and
    this function's main consumer is :func:`parkscan_core.lot.to_geojson`.
    """
    r_meridional, r_normal = curvature_radii(origin.lat)
    lat0 = math.radians(origin.lat)
    dlat = y_m / r_meridional
    dlon = x_m / (r_normal * math.cos(lat0))
    return (origin.lon + math.degrees(dlon), origin.lat + math.degrees(dlat))


def lonlat_to_lot(lon_deg: float, lat_deg: float, origin: Origin) -> tuple[float, float]:
    """``(lon_deg, lat_deg)`` -> lot metres ``(x_m, y_m)``.

    The exact algebraic inverse of :func:`lot_to_lonlat`.
    """
    r_meridional, r_normal = curvature_radii(origin.lat)
    lat0 = math.radians(origin.lat)
    dlat = math.radians(lat_deg - origin.lat)
    dlon = math.radians(lon_deg - origin.lon)
    return (dlon * r_normal * math.cos(lat0), dlat * r_meridional)

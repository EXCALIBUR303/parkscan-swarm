"""The lot specification: metric ground truth in, WGS84 GeoJSON out.

``sim/lot/lot.yaml`` is the single authored description of the parking lot. It is written
in **lot metres** because that is the frame the simulator, the coverage planner and the
allocator all reason in; degrees appear only at the very edge of the system.

``sim/lot/lot.geojson`` is generated from it by::

    python -m parkscan_core.lot build sim/lot/lot.yaml sim/lot/lot.geojson

(the ``make lot`` target runs exactly that). Both files are committed, so a change to the
lot shows up as a reviewable diff instead of appearing at runtime.

The generated FeatureCollection is the **same data the coverage planner consumes** --
deliberately. The dashboard draws no basemap: a synthetic lot does not exist in
OpenStreetMap, and putting robots on top of a real, wrong street would be actively
deceptive. So the map is drawn from this file, and the map is therefore guaranteed to
show the geometry the planner actually used. Every feature carries a ``layer`` property,
one of ``zone`` | ``parking_space`` | ``charging_bay`` | ``lane`` | ``lot_outline``, which
is what the MapLibre style keys off.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Self

# PyYAML ships no inline types and `types-PyYAML` is not in this workspace's dev group;
# the untyped import is contained here and `raw` is immediately narrowed to `object`.
import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from parkscan_core.geo import Origin, lot_to_lonlat

__all__ = [
    "ChargingBay",
    "LaneEdge",
    "LaneGraph",
    "LaneNode",
    "LotOrigin",
    "LotSize",
    "LotSpec",
    "ParkingSpace",
    "Point2D",
    "Zone",
    "load_lot",
    "main",
    "to_geojson",
]

type Point2D = tuple[float, float]
"""A point in lot metres, ``(x_m, y_m)``."""

type Ring = tuple[Point2D, ...]
"""An open polygon ring in lot metres; the closing vertex is added on export."""

_COORD_DECIMALS = 9
"""Degrees rounded to 1e-9 (~0.1 mm at the equator) so the generated file is diffable."""


class _Model(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class LotOrigin(_Model):
    """WGS84 anchor of the lot's local frame."""

    lat: float = Field(ge=-90.0, le=90.0)
    lon: float = Field(ge=-180.0, le=180.0)

    def as_origin(self) -> Origin:
        """Convert to the plain :class:`~parkscan_core.geo.Origin` the maths uses."""
        return Origin(lat=self.lat, lon=self.lon)


class LotSize(_Model):
    """Lot extent in metres. The lot occupies ``[0, w] x [0, h]`` in the local frame."""

    w: float = Field(gt=0.0)
    h: float = Field(gt=0.0)


class Zone(_Model):
    """One inspection zone: the unit of allocation, coverage and reassignment.

    ``polygon`` is an open ring in lot metres. ``entry_waypoint`` and ``exit_waypoint``
    must be lane-graph nodes -- a robot enters a zone at one end of its aisle and leaves
    at the other, so the sweep does not have to double back to where it started.

    ``sweep_length_m`` is the in-zone path length of the back-and-forth sweep, and feeds
    the allocator's cost ``c(i, z) = d_lane(pos_i, entry(z)) + sweep_len(z)``. It is
    authored, not measured: on an obstacle-free rectangle a "cellular decomposition"
    degenerates to a lawnmower path, and pretending otherwise would be an overclaim
    (ACCEPTANCE H8).
    """

    id: str = Field(min_length=1)
    polygon: Ring
    entry_waypoint: str = Field(min_length=1)
    exit_waypoint: str = Field(min_length=1)
    sweep_length_m: float = Field(gt=0.0)

    @model_validator(mode="after")
    def _check_ring(self) -> Self:
        if len(self.polygon) < 3:
            msg = f"zone {self.id!r}: polygon needs at least 3 vertices"
            raise ValueError(msg)
        if self.polygon[0] == self.polygon[-1]:
            msg = (
                f"zone {self.id!r}: polygon ring must be open (the closing vertex is "
                "added on export)"
            )
            raise ValueError(msg)
        if self.entry_waypoint == self.exit_waypoint:
            msg = f"zone {self.id!r}: entry and exit waypoints must differ"
            raise ValueError(msg)
        return self


class ParkingSpace(_Model):
    """One marked parking stall, whose occupancy the fleet reports."""

    id: str = Field(min_length=1)
    zone_id: str = Field(min_length=1)
    polygon: Ring

    @model_validator(mode="after")
    def _check_ring(self) -> Self:
        if len(self.polygon) < 3:
            msg = f"parking space {self.id!r}: polygon needs at least 3 vertices"
            raise ValueError(msg)
        if self.polygon[0] == self.polygon[-1]:
            msg = f"parking space {self.id!r}: polygon ring must be open"
            raise ValueError(msg)
        return self


class ChargingBay(_Model):
    """A charging bay. Bays are scarcer than robots on purpose, so contention is real."""

    id: str = Field(min_length=1)
    node: str = Field(min_length=1)
    position: Point2D


class LaneNode(_Model):
    """A vertex of the drivable lane graph, in lot metres."""

    id: str = Field(min_length=1)
    x_m: float
    y_m: float


class LaneEdge(_Model):
    """An undirected, traversable lane segment between two :class:`LaneNode` ids.

    Segments are the unit of single-occupancy reservation, so ``id`` is stable and used
    as the resource key by the lease manager.
    """

    id: str = Field(min_length=1)
    a: str = Field(min_length=1)
    b: str = Field(min_length=1)

    @model_validator(mode="after")
    def _check_distinct(self) -> Self:
        if self.a == self.b:
            msg = f"lane edge {self.id!r} is a self-loop"
            raise ValueError(msg)
        return self


class LaneGraph(_Model):
    """The drivable network. Path cost is Manhattan-over-lanes, never Euclidean."""

    nodes: tuple[LaneNode, ...]
    edges: tuple[LaneEdge, ...]

    def node_ids(self) -> frozenset[str]:
        return frozenset(n.id for n in self.nodes)

    def node_map(self) -> dict[str, LaneNode]:
        return {n.id: n for n in self.nodes}

    @model_validator(mode="after")
    def _check_graph(self) -> Self:
        ids = [n.id for n in self.nodes]
        _reject_duplicates(ids, "lane node")
        _reject_duplicates([e.id for e in self.edges], "lane edge")
        known = set(ids)
        for edge in self.edges:
            missing = {edge.a, edge.b} - known
            if missing:
                msg = f"lane edge {edge.id!r} references unknown node(s) {sorted(missing)}"
                raise ValueError(msg)
        return self


class LotSpec(_Model):
    """The whole authored lot."""

    name: str = Field(min_length=1)
    origin: LotOrigin
    size_m: LotSize
    zones: tuple[Zone, ...]
    parking_spaces: tuple[ParkingSpace, ...]
    charging_bays: tuple[ChargingBay, ...]
    lane_graph: LaneGraph

    @model_validator(mode="after")
    def _check_references_and_bounds(self) -> Self:
        _reject_duplicates([z.id for z in self.zones], "zone")
        _reject_duplicates([s.id for s in self.parking_spaces], "parking space")
        _reject_duplicates([b.id for b in self.charging_bays], "charging bay")
        if not self.zones:
            msg = "a lot needs at least one zone"
            raise ValueError(msg)

        zone_ids = {z.id for z in self.zones}
        node_ids = self.lane_graph.node_ids()

        for zone in self.zones:
            for role, wp in (("entry", zone.entry_waypoint), ("exit", zone.exit_waypoint)):
                if wp not in node_ids:
                    msg = f"zone {zone.id!r}: {role}_waypoint {wp!r} is not a lane node"
                    raise ValueError(msg)
            self._check_in_bounds(zone.polygon, f"zone {zone.id!r}")

        for space in self.parking_spaces:
            if space.zone_id not in zone_ids:
                msg = f"parking space {space.id!r} references unknown zone {space.zone_id!r}"
                raise ValueError(msg)
            self._check_in_bounds(space.polygon, f"parking space {space.id!r}")

        nodes = self.lane_graph.node_map()
        for bay in self.charging_bays:
            if bay.node not in node_ids:
                msg = f"charging bay {bay.id!r}: node {bay.node!r} is not a lane node"
                raise ValueError(msg)
            node = nodes[bay.node]
            if not math.isclose(node.x_m, bay.position[0], abs_tol=1e-6) or not math.isclose(
                node.y_m, bay.position[1], abs_tol=1e-6
            ):
                msg = (
                    f"charging bay {bay.id!r} position {bay.position} disagrees with its "
                    f"lane node {bay.node!r} at ({node.x_m}, {node.y_m})"
                )
                raise ValueError(msg)
            self._check_in_bounds((bay.position,), f"charging bay {bay.id!r}")

        self._check_in_bounds(tuple((n.x_m, n.y_m) for n in self.lane_graph.nodes), "lane graph")
        return self

    def _check_in_bounds(self, points: Sequence[Point2D], what: str) -> None:
        for x, y in points:
            if not (0.0 <= x <= self.size_m.w and 0.0 <= y <= self.size_m.h):
                msg = (
                    f"{what}: point ({x}, {y}) falls outside the lot "
                    f"[0, {self.size_m.w}] x [0, {self.size_m.h}]"
                )
                raise ValueError(msg)


def _reject_duplicates(ids: Sequence[str], what: str) -> None:
    seen: set[str] = set()
    dupes: set[str] = set()
    for identifier in ids:
        if identifier in seen:
            dupes.add(identifier)
        seen.add(identifier)
    if dupes:
        msg = f"duplicate {what} id(s): {sorted(dupes)}"
        raise ValueError(msg)


# ------------------------------------------------------------------------ loading --
def load_lot(path: Path) -> LotSpec:
    """Parse and validate a lot spec from a YAML file."""
    with path.open("r", encoding="utf-8") as fh:
        raw: object = yaml.safe_load(fh)
    if not isinstance(raw, dict):
        msg = f"{path}: expected a YAML mapping at the top level, got {type(raw).__name__}"
        raise ValueError(msg)
    return LotSpec.model_validate(raw)


# ------------------------------------------------------------------------ export --
def _pt(x_m: float, y_m: float, origin: Origin) -> list[float]:
    lon, lat = lot_to_lonlat(x_m, y_m, origin)
    return [round(lon, _COORD_DECIMALS), round(lat, _COORD_DECIMALS)]


def _ring(points: Sequence[Point2D], origin: Origin) -> list[list[list[float]]]:
    ring = [_pt(x, y, origin) for x, y in points]
    ring.append(list(ring[0]))
    return [ring]


def _feature(geometry: dict[str, object], properties: dict[str, object]) -> dict[str, object]:
    return {"type": "Feature", "geometry": geometry, "properties": properties}


def to_geojson(spec: LotSpec) -> dict[str, object]:
    """Render the lot as a WGS84 ``FeatureCollection``.

    Every feature carries a ``layer`` property: ``lot_outline``, ``zone``,
    ``parking_space``, ``charging_bay`` or ``lane``. Metric coordinates are preserved in
    the properties where they are useful (lane node positions, zone bounds) so that a
    consumer never has to invert the projection to get back to the planner's frame.
    """
    origin = spec.origin.as_origin()
    w, h = spec.size_m.w, spec.size_m.h
    features: list[dict[str, object]] = [
        _feature(
            {
                "type": "Polygon",
                "coordinates": _ring(((0.0, 0.0), (w, 0.0), (w, h), (0.0, h)), origin),
            },
            {
                "layer": "lot_outline",
                "id": "lot_outline",
                "name": spec.name,
                "width_m": w,
                "height_m": h,
                "area_m2": round(w * h, 6),
            },
        )
    ]

    nodes = spec.lane_graph.node_map()

    for zone in spec.zones:
        xs = [p[0] for p in zone.polygon]
        ys = [p[1] for p in zone.polygon]
        features.append(
            _feature(
                {"type": "Polygon", "coordinates": _ring(zone.polygon, origin)},
                {
                    "layer": "zone",
                    "id": zone.id,
                    "entry_waypoint": zone.entry_waypoint,
                    "exit_waypoint": zone.exit_waypoint,
                    "sweep_length_m": zone.sweep_length_m,
                    "area_m2": round(_shoelace_area(zone.polygon), 6),
                    "min_x_m": min(xs),
                    "min_y_m": min(ys),
                    "max_x_m": max(xs),
                    "max_y_m": max(ys),
                },
            )
        )

    for space in spec.parking_spaces:
        features.append(
            _feature(
                {"type": "Polygon", "coordinates": _ring(space.polygon, origin)},
                {
                    "layer": "parking_space",
                    "id": space.id,
                    "zone_id": space.zone_id,
                    "area_m2": round(_shoelace_area(space.polygon), 6),
                },
            )
        )

    for bay in spec.charging_bays:
        features.append(
            _feature(
                {"type": "Point", "coordinates": _pt(bay.position[0], bay.position[1], origin)},
                {
                    "layer": "charging_bay",
                    "id": bay.id,
                    "node": bay.node,
                    "x_m": bay.position[0],
                    "y_m": bay.position[1],
                },
            )
        )

    for edge in spec.lane_graph.edges:
        a, b = nodes[edge.a], nodes[edge.b]
        features.append(
            _feature(
                {
                    "type": "LineString",
                    "coordinates": [
                        _pt(a.x_m, a.y_m, origin),
                        _pt(b.x_m, b.y_m, origin),
                    ],
                },
                {
                    "layer": "lane",
                    "id": edge.id,
                    "from": edge.a,
                    "to": edge.b,
                    "length_m": round(math.dist((a.x_m, a.y_m), (b.x_m, b.y_m)), 6),
                },
            )
        )

    return {
        "type": "FeatureCollection",
        "name": spec.name,
        "properties": {
            "origin": {"lat": spec.origin.lat, "lon": spec.origin.lon},
            "size_m": {"w": w, "h": h},
            "generator": "parkscan_core.lot",
            "note": (
                "Synthetic lot. Generated from sim/lot/lot.yaml; do not hand-edit. "
                "The origin is an obviously artificial location, not a real address."
            ),
        },
        "features": features,
    }


def _shoelace_area(ring: Sequence[Point2D]) -> float:
    """Absolute polygon area in square metres, for an open ring."""
    total = 0.0
    n = len(ring)
    for i in range(n):
        x1, y1 = ring[i]
        x2, y2 = ring[(i + 1) % n]
        total += x1 * y2 - x2 * y1
    return abs(total) / 2.0


# --------------------------------------------------------------------------- CLI --
def main(argv: Sequence[str] | None = None) -> int:
    """``python -m parkscan_core.lot build <in.yaml> <out.geojson>``."""
    parser = argparse.ArgumentParser(
        prog="python -m parkscan_core.lot",
        description="Lot spec tools. 'build' renders the metric YAML to WGS84 GeoJSON.",
    )
    sub = parser.add_subparsers(dest="command", required=True)
    build = sub.add_parser("build", help="render <in.yaml> to <out.geojson>")
    build.add_argument("input", type=Path, help="path to the metric lot spec (YAML)")
    build.add_argument("output", type=Path, help="path to write the GeoJSON to")

    args = parser.parse_args(argv)
    spec = load_lot(args.input)
    collection = to_geojson(spec)

    out: Path = args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8") as fh:
        json.dump(collection, fh, indent=2, sort_keys=False)
        fh.write("\n")

    features = collection["features"]
    n_features = len(features) if isinstance(features, list) else 0
    print(
        f"{out}: {n_features} features "
        f"({len(spec.zones)} zones, {len(spec.parking_spaces)} parking spaces, "
        f"{len(spec.charging_bays)} charging bays, {len(spec.lane_graph.edges)} lanes)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

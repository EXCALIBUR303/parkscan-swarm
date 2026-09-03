"""Lot spec, zone tiling (C8) and the generated GeoJSON.

**shapely is not installed** in this workspace and adding a dependency is out of scope,
so the tiling proof below is exact rectangle arithmetic rather than a geometry library:
every zone is asserted to be an axis-aligned rectangle, the union of all zone edges is
used to cut the lot into elementary cells, and each cell is required to be covered by
*exactly one* zone. On axis-aligned rectangles that is a complete decision procedure for
"zero gaps and zero overlap" -- stronger than an area comparison, which cannot tell a
gap from a compensating overlap.
"""

from __future__ import annotations

import json
import math
from collections import deque
from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Any

import pytest
from pydantic import ValidationError

from parkscan_core.geo import lonlat_to_lot
from parkscan_core.lot import LotSpec, Point2D, load_lot, main, to_geojson

REPO_ROOT = Path(__file__).resolve().parents[3]
LOT_YAML = REPO_ROOT / "sim" / "lot" / "lot.yaml"
LOT_GEOJSON = REPO_ROOT / "sim" / "lot" / "lot.geojson"

LAYERS = {"zone", "parking_space", "charging_bay", "lane", "lot_outline"}

Rect = tuple[float, float, float, float]


# --------------------------------------------------------------- rectangle helpers --
def as_rect(ring: Sequence[Point2D]) -> Rect:
    """Assert ``ring`` is an axis-aligned rectangle and return ``(x0, y0, x1, y1)``."""
    assert len(ring) == 4, f"expected 4 vertices, got {len(ring)}: {ring}"
    xs = sorted({p[0] for p in ring})
    ys = sorted({p[1] for p in ring})
    assert len(xs) == 2, f"ring is not axis-aligned in x: {ring}"
    assert len(ys) == 2, f"ring is not axis-aligned in y: {ring}"
    return (xs[0], ys[0], xs[1], ys[1])


def area(rect: Rect) -> float:
    return (rect[2] - rect[0]) * (rect[3] - rect[1])


def overlap_area(a: Rect, b: Rect) -> float:
    dx = min(a[2], b[2]) - max(a[0], b[0])
    dy = min(a[3], b[3]) - max(a[1], b[1])
    return max(dx, 0.0) * max(dy, 0.0)


def contains(rect: Rect, x: float, y: float) -> bool:
    return rect[0] <= x <= rect[2] and rect[1] <= y <= rect[3]


def elementary_cells(rects: Iterable[Rect]) -> list[tuple[float, float]]:
    """Cut the plane on every rectangle edge; return one interior point per cell."""
    rects = list(rects)
    xs = sorted({v for r in rects for v in (r[0], r[2])})
    ys = sorted({v for r in rects for v in (r[1], r[3])})
    return [
        ((xs[i] + xs[i + 1]) / 2.0, (ys[j] + ys[j + 1]) / 2.0)
        for i in range(len(xs) - 1)
        for j in range(len(ys) - 1)
    ]


# ------------------------------------------------------------------------- C8 --
def test_zones_tile_the_lot_with_zero_gaps_and_zero_overlap(lot: LotSpec) -> None:
    """Acceptance criterion C8."""
    rects = {z.id: as_rect(z.polygon) for z in lot.zones}
    assert len(rects) == 12

    lot_rect: Rect = (0.0, 0.0, lot.size_m.w, lot.size_m.h)

    # 1. Every zone lies inside the lot.
    for zid, r in rects.items():
        assert r[0] >= lot_rect[0] and r[1] >= lot_rect[1], zid
        assert r[2] <= lot_rect[2] and r[3] <= lot_rect[3], zid

    # 2. Zero overlap, pairwise and exactly.
    ids = sorted(rects)
    for i, a in enumerate(ids):
        for b in ids[i + 1 :]:
            assert overlap_area(rects[a], rects[b]) == 0.0, f"{a} overlaps {b}"

    # 3. Zero gaps: every elementary cell of the lot is covered exactly once.
    cells = elementary_cells([*rects.values(), lot_rect])
    assert cells, "degenerate decomposition"
    for cx, cy in cells:
        covering = [zid for zid, r in rects.items() if contains(r, cx, cy)]
        inside_lot = contains(lot_rect, cx, cy)
        assert inside_lot, f"cell ({cx}, {cy}) escapes the lot outline"
        assert len(covering) == 1, f"cell ({cx}, {cy}) covered by {covering or 'nothing'}"

    # 4. Total zone area equals -- and therefore never exceeds -- the lot area.
    total = sum(area(r) for r in rects.values())
    assert total == pytest.approx(area(lot_rect), rel=0.0, abs=1e-9)
    assert total <= area(lot_rect) + 1e-9


def test_parking_spaces_lie_inside_their_zone_and_do_not_overlap(lot: LotSpec) -> None:
    zone_rects = {z.id: as_rect(z.polygon) for z in lot.zones}
    space_rects = {s.id: as_rect(s.polygon) for s in lot.parking_spaces}
    assert len(space_rects) == 120

    for space in lot.parking_spaces:
        zr = zone_rects[space.zone_id]
        sr = space_rects[space.id]
        assert overlap_area(zr, sr) == pytest.approx(area(sr)), (
            f"{space.id} is not contained in {space.zone_id}"
        )
        assert area(sr) == pytest.approx(12.5), "stalls are 2.5 m x 5.0 m"

    ids = sorted(space_rects)
    for i, a in enumerate(ids):
        for b in ids[i + 1 :]:
            assert overlap_area(space_rects[a], space_rects[b]) == 0.0, f"{a} overlaps {b}"


# ------------------------------------------------------------------ lot contents --
def test_lot_dimensions_and_origin(lot: LotSpec) -> None:
    assert (lot.size_m.w, lot.size_m.h) == (150.0, 100.0)
    assert (lot.origin.lat, lot.origin.lon) == (0.0, 0.0)


def test_there_are_fewer_charging_bays_than_robots(lot: LotSpec) -> None:
    """2 bays for 3 robots: contention has to be real, not fabricated."""
    assert len(lot.charging_bays) == 2
    assert len({b.id for b in lot.charging_bays}) == 2


def test_every_zone_waypoint_is_a_distinct_lane_node(lot: LotSpec) -> None:
    node_ids = lot.lane_graph.node_ids()
    for zone in lot.zones:
        assert zone.entry_waypoint in node_ids
        assert zone.exit_waypoint in node_ids
        assert zone.entry_waypoint != zone.exit_waypoint


def test_sweep_length_matches_the_documented_formula(lot: LotSpec) -> None:
    """3 passes of 40 m at 2 m spacing = 124 m. Authored geometry, not an optimum."""
    for zone in lot.zones:
        assert zone.sweep_length_m == pytest.approx(3 * 40.0 + 2 * 2.0)


def test_lane_graph_is_connected_and_reaches_every_bay_and_zone(lot: LotSpec) -> None:
    adjacency: dict[str, set[str]] = {n.id: set() for n in lot.lane_graph.nodes}
    for edge in lot.lane_graph.edges:
        adjacency[edge.a].add(edge.b)
        adjacency[edge.b].add(edge.a)

    start = lot.charging_bays[0].node
    seen = {start}
    queue = deque([start])
    while queue:
        for nxt in adjacency[queue.popleft()]:
            if nxt not in seen:
                seen.add(nxt)
                queue.append(nxt)

    assert seen == lot.lane_graph.node_ids(), (
        f"unreachable from {start}: {sorted(lot.lane_graph.node_ids() - seen)}"
    )
    for zone in lot.zones:
        assert zone.entry_waypoint in seen
        assert zone.exit_waypoint in seen
    for bay in lot.charging_bays:
        assert bay.node in seen


def test_lane_edges_are_short_enough_to_be_real_segments(lot: LotSpec) -> None:
    nodes = lot.lane_graph.node_map()
    for edge in lot.lane_graph.edges:
        a, b = nodes[edge.a], nodes[edge.b]
        length = math.dist((a.x_m, a.y_m), (b.x_m, b.y_m))
        assert 0.0 < length <= 45.0, f"{edge.id} is {length} m long"


def test_lane_edges_are_axis_aligned(lot: LotSpec) -> None:
    """Lot lanes run along the aisles; a diagonal edge would cut through parked cars."""
    nodes = lot.lane_graph.node_map()
    for edge in lot.lane_graph.edges:
        a, b = nodes[edge.a], nodes[edge.b]
        assert a.x_m == b.x_m or a.y_m == b.y_m, edge.id


# ------------------------------------------------------------------ validation --
def _spec_dict() -> dict[str, Any]:
    return {
        "name": "tiny",
        "origin": {"lat": 0.0, "lon": 0.0},
        "size_m": {"w": 10.0, "h": 10.0},
        "zones": [
            {
                "id": "z1",
                "polygon": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]],
                "entry_waypoint": "n1",
                "exit_waypoint": "n2",
                "sweep_length_m": 10.0,
            }
        ],
        "parking_spaces": [
            {
                "id": "s1",
                "zone_id": "z1",
                "polygon": [[1.0, 1.0], [3.5, 1.0], [3.5, 6.0], [1.0, 6.0]],
            }
        ],
        "charging_bays": [{"id": "b1", "node": "n2", "position": [9.0, 1.0]}],
        "lane_graph": {
            "nodes": [
                {"id": "n1", "x_m": 1.0, "y_m": 1.0},
                {"id": "n2", "x_m": 9.0, "y_m": 1.0},
            ],
            "edges": [{"id": "e1", "a": "n1", "b": "n2"}],
        },
    }


def test_minimal_spec_validates() -> None:
    spec = LotSpec.model_validate(_spec_dict())
    assert spec.name == "tiny"
    assert spec.zones[0].polygon[0] == (0.0, 0.0)


def test_unknown_zone_waypoint_is_rejected() -> None:
    data = _spec_dict()
    data["zones"][0]["entry_waypoint"] = "nope"
    with pytest.raises(ValidationError, match="is not a lane node"):
        LotSpec.model_validate(data)


def test_parking_space_in_an_unknown_zone_is_rejected() -> None:
    data = _spec_dict()
    data["parking_spaces"][0]["zone_id"] = "ghost"
    with pytest.raises(ValidationError, match="unknown zone"):
        LotSpec.model_validate(data)


def test_lane_edge_to_an_unknown_node_is_rejected() -> None:
    data = _spec_dict()
    data["lane_graph"]["edges"][0]["b"] = "ghost"
    with pytest.raises(ValidationError, match="unknown node"):
        LotSpec.model_validate(data)


def test_duplicate_zone_ids_are_rejected() -> None:
    data = _spec_dict()
    data["zones"].append(dict(data["zones"][0]))
    with pytest.raises(ValidationError, match="duplicate zone id"):
        LotSpec.model_validate(data)


def test_geometry_outside_the_lot_is_rejected() -> None:
    data = _spec_dict()
    data["zones"][0]["polygon"] = [[0.0, 0.0], [11.0, 0.0], [11.0, 10.0], [0.0, 10.0]]
    with pytest.raises(ValidationError, match="outside the lot"):
        LotSpec.model_validate(data)


def test_closed_ring_is_rejected() -> None:
    data = _spec_dict()
    data["zones"][0]["polygon"] = [
        [0.0, 0.0],
        [10.0, 0.0],
        [10.0, 10.0],
        [0.0, 10.0],
        [0.0, 0.0],
    ]
    with pytest.raises(ValidationError, match="must be open"):
        LotSpec.model_validate(data)


def test_bay_position_must_agree_with_its_lane_node() -> None:
    data = _spec_dict()
    data["charging_bays"][0]["position"] = [2.0, 2.0]
    with pytest.raises(ValidationError, match="disagrees with its lane node"):
        LotSpec.model_validate(data)


def test_unknown_key_is_rejected() -> None:
    data = _spec_dict()
    data["colour"] = "blue"
    with pytest.raises(ValidationError):
        LotSpec.model_validate(data)


def test_load_lot_rejects_a_non_mapping(tmp_path: Path) -> None:
    bad = tmp_path / "bad.yaml"
    bad.write_text("- 1\n- 2\n", encoding="utf-8")
    with pytest.raises(ValueError, match="expected a YAML mapping"):
        load_lot(bad)


# -------------------------------------------------------------------- geojson --
def test_generated_file_is_in_sync_with_the_yaml(lot: LotSpec, lot_geojson: dict[str, Any]) -> None:
    """`make lot` output is committed; this fails if someone edited the YAML and forgot."""
    assert to_geojson(lot) == lot_geojson


def test_every_feature_declares_a_known_layer(lot_geojson: dict[str, Any]) -> None:
    assert lot_geojson["type"] == "FeatureCollection"
    features = lot_geojson["features"]
    assert len(features) == 1 + 12 + 120 + 2 + 52
    counts: dict[str, int] = {}
    for feature in features:
        layer = feature["properties"]["layer"]
        assert layer in LAYERS, layer
        assert feature["properties"]["id"]
        counts[layer] = counts.get(layer, 0) + 1
    assert counts == {
        "lot_outline": 1,
        "zone": 12,
        "parking_space": 120,
        "charging_bay": 2,
        "lane": 52,
    }


def _coords(geometry: dict[str, Any]) -> list[list[float]]:
    if geometry["type"] == "Point":
        return [geometry["coordinates"]]
    if geometry["type"] == "LineString":
        return list(geometry["coordinates"])
    return list(geometry["coordinates"][0])


def test_coordinates_are_plausible_and_invert_back_into_the_lot(
    lot: LotSpec, lot_geojson: dict[str, Any]
) -> None:
    origin = lot.origin.as_origin()
    max_lon = max_lat = -math.inf
    for feature in lot_geojson["features"]:
        for lon, lat in _coords(feature["geometry"]):
            assert -180.0 <= lon <= 180.0
            assert -90.0 <= lat <= 90.0
            x, y = lonlat_to_lot(lon, lat, origin)
            # 1e-3 m of slack for the 1e-9 degree rounding applied on export.
            assert -1e-3 <= x <= lot.size_m.w + 1e-3, (feature["properties"]["id"], x)
            assert -1e-3 <= y <= lot.size_m.h + 1e-3, (feature["properties"]["id"], y)
            max_lon, max_lat = max(max_lon, lon), max(max_lat, lat)
    # The lot is ~150 m x 100 m at the equator: ~1.35e-3 deg by ~9.0e-4 deg.
    assert max_lon == pytest.approx(1.3475e-3, rel=1e-3)
    assert max_lat == pytest.approx(9.0442e-4, rel=1e-3)


def test_polygon_rings_are_closed_as_geojson_requires(lot_geojson: dict[str, Any]) -> None:
    for feature in lot_geojson["features"]:
        if feature["geometry"]["type"] != "Polygon":
            continue
        ring = feature["geometry"]["coordinates"][0]
        assert len(ring) >= 4
        assert ring[0] == ring[-1], feature["properties"]["id"]


def test_collection_says_the_lot_is_synthetic(lot_geojson: dict[str, Any]) -> None:
    note = lot_geojson["properties"]["note"]
    assert "Synthetic" in note
    assert lot_geojson["properties"]["origin"] == {"lat": 0.0, "lon": 0.0}


# ------------------------------------------------------------------------ CLI --
def test_build_cli_round_trips(tmp_path: Path) -> None:
    out = tmp_path / "nested" / "out.geojson"
    assert main(["build", str(LOT_YAML), str(out)]) == 0
    assert out.exists()
    with out.open("r", encoding="utf-8") as fh:
        produced = json.load(fh)
    with LOT_GEOJSON.open("r", encoding="utf-8") as fh:
        committed = json.load(fh)
    assert produced == committed


def test_build_cli_rejects_a_missing_subcommand() -> None:
    with pytest.raises(SystemExit):
        main([])

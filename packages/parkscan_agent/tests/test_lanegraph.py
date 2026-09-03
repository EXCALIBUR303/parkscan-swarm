"""Routing on the lane graph.

The headline property: distance is LANE-GRAPH distance, never Euclidean. A parking lot
has lanes, and a cost function that flies straight through parked cars produces
allocations that look optimal and are not.
"""

from __future__ import annotations

import math

import pytest
from parkscan_agent.lanegraph import (
    NoRouteError,
    UnknownNodeError,
    lane_distance_m,
    nearest_node,
    node_xy,
    path_length_m,
    route,
    route_points,
)

from parkscan_core.lot import LaneEdge, LaneGraph, LaneNode, LotSpec


def test_route_from_a_node_to_itself_is_a_single_node(lot: LotSpec) -> None:
    assert route(lot, "bay_1_node", "bay_1_node") == ["bay_1_node"]
    assert path_length_m(lot, ["bay_1_node"]) == 0.0


def test_every_zone_entry_and_exit_is_reachable_from_every_bay(lot: LotSpec) -> None:
    """If a zone could not be reached from a bay, the fleet could not work it and return."""
    for bay in lot.charging_bays:
        for zone in lot.zones:
            for target in (zone.entry_waypoint, zone.exit_waypoint):
                path = route(lot, bay.node, target)
                assert path[0] == bay.node
                assert path[-1] == target
                assert path_length_m(lot, path) > 0.0


def test_lane_distance_is_never_shorter_than_straight_line(lot: LotSpec) -> None:
    """The triangle inequality, as a sanity check on the whole routing layer.

    A lane route that came out SHORTER than the Euclidean distance would mean the graph
    contains an edge that is not physically there.
    """
    for zone in lot.zones[:6]:
        a, b = zone.entry_waypoint, zone.exit_waypoint
        ax, ay = node_xy(lot, a)
        bx, by = node_xy(lot, b)
        straight = math.hypot(bx - ax, by - ay)
        assert lane_distance_m(lot, a, b) >= straight - 1e-9


def test_routing_is_deterministic_across_repeated_calls(lot: LotSpec) -> None:
    """Equal-cost paths must be broken the same way every run, or the sim is not seeded."""
    first = route(lot, "bay_2_node", lot.zones[-1].exit_waypoint)
    for _ in range(20):
        assert route(lot, "bay_2_node", lot.zones[-1].exit_waypoint) == first


def test_unknown_node_raises_rather_than_returning_empty(lot: LotSpec) -> None:
    with pytest.raises(UnknownNodeError):
        route(lot, "bay_1_node", "no_such_node")
    with pytest.raises(UnknownNodeError):
        route(lot, "no_such_node", "bay_1_node")


def test_a_disconnected_graph_raises_no_route() -> None:
    """Built by hand: two islands with no edge between them."""
    graph = LaneGraph(
        nodes=(
            LaneNode(id="a", x_m=0.0, y_m=0.0),
            LaneNode(id="b", x_m=1.0, y_m=0.0),
            LaneNode(id="island", x_m=50.0, y_m=50.0),
        ),
        edges=(LaneEdge(id="ab", a="a", b="b"),),
    )
    spec = _tiny_spec(graph)
    assert route(spec, "a", "b") == ["a", "b"]
    with pytest.raises(NoRouteError):
        route(spec, "a", "island")


def test_shortest_path_is_actually_shortest_on_a_known_graph() -> None:
    """A diamond where the direct edge is longer than the two-hop detour."""
    graph = LaneGraph(
        nodes=(
            LaneNode(id="a", x_m=0.0, y_m=0.0),
            LaneNode(id="mid", x_m=5.0, y_m=0.0),
            LaneNode(id="b", x_m=10.0, y_m=0.0),
            LaneNode(id="far", x_m=0.0, y_m=40.0),
        ),
        edges=(
            LaneEdge(id="a_mid", a="a", b="mid"),
            LaneEdge(id="mid_b", a="mid", b="b"),
            LaneEdge(id="a_far", a="a", b="far"),
            LaneEdge(id="far_b", a="far", b="b"),
        ),
    )
    spec = _tiny_spec(graph)
    assert route(spec, "a", "b") == ["a", "mid", "b"]
    assert lane_distance_m(spec, "a", "b") == pytest.approx(10.0)


def test_nearest_node_picks_the_closest_and_is_stable(lot: LotSpec) -> None:
    x, y = node_xy(lot, "bay_1_node")
    assert nearest_node(lot, x + 0.05, y - 0.05) == "bay_1_node"
    # A point equidistant from two nodes must resolve the same way every call.
    picks = {nearest_node(lot, 50.0, 50.0) for _ in range(20)}
    assert len(picks) == 1


def test_route_points_matches_the_node_coordinates(lot: LotSpec) -> None:
    path = route(lot, "bay_1_node", lot.zones[0].entry_waypoint)
    points = route_points(lot, path)
    assert len(points) == len(path)
    for node, point in zip(path, points, strict=True):
        assert point == pytest.approx(node_xy(lot, node))


def _tiny_spec(graph: LaneGraph) -> LotSpec:
    """A hand-built lot for graph tests.

    LotSpec insists on at least one zone (correctly -- a lot with nothing to inspect is a
    configuration error, not a valid input), so a single throwaway zone is attached to
    nodes 'a' and 'b'. Only the lane graph matters to these tests.
    """
    from parkscan_core.lot import LotOrigin, LotSize, Zone

    return LotSpec(
        name="test",
        origin=LotOrigin(lat=0.0, lon=0.0),
        size_m=LotSize(w=100.0, h=100.0),
        zones=(
            Zone(
                id="zone_test",
                polygon=((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)),
                entry_waypoint="a",
                exit_waypoint="b",
                sweep_length_m=10.0,
            ),
        ),
        parking_spaces=(),
        charging_bays=(),
        lane_graph=graph,
    )

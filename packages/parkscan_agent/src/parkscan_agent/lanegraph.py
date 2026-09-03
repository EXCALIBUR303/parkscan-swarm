"""Routing on the lot's lane graph.

Why this module exists at all
-----------------------------
A parking lot has lanes. The straight-line distance between a robot and a zone entry is
almost never the distance it has to drive: the kerbs, the stall rows and the landscaping
between the aisles are not drivable, and the west perimeter lane is the only way between
bands. Every cost in this project -- the allocator's ``c(i, z)``, the battery
feasibility check, the docking approach -- is therefore **lane-graph distance**, computed
by Dijkstra over :class:`~parkscan_core.lot.LaneGraph`, and never a Euclidean shortcut.

There is exactly one legitimate use of Euclidean distance here, and it is
:func:`nearest_node`: snapping a free-space pose onto the graph so that graph search has
somewhere to start. That is a *projection*, not a cost. It is a separate function with a
docstring saying so precisely because the temptation to reuse it as a cost is the mistake
this module exists to prevent.

Determinism
-----------
``make bench`` re-runs 30 seeds and every README number has to be regenerable, so a route
must not depend on dictionary iteration order or on floating-point tie-break luck. The
search is a uniform-cost search whose priority queue is ordered by
``(distance, path_so_far)``: when two paths cost the same, the lexicographically smaller
tuple of node ids wins. That is a total order over candidate paths, so the returned route
is a pure function of ``(graph, from_node, to_node)`` -- identical across runs, machines
and Python hash seeds.

The graph is tiny (44 nodes / 52 undirected edges for the shipped lot), so the adjacency
map is memoised on the frozen :class:`~parkscan_core.lot.LaneGraph` value and the search
itself is microseconds. Nothing here needs an A* heuristic; adding one would only
introduce a way for the result to become inconsistent with the cost function.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from functools import lru_cache
from heapq import heappop, heappush
from itertools import pairwise
from types import MappingProxyType

from parkscan_core.lot import LaneGraph, LotSpec

__all__ = [
    "NoRouteError",
    "RoutingError",
    "UnknownNodeError",
    "adjacency",
    "lane_distance_m",
    "nearest_node",
    "node_xy",
    "path_length_m",
    "route",
    "route_points",
]

type Adjacency = Mapping[str, tuple[tuple[str, float], ...]]
"""``node id -> ((neighbour id, edge length in metres), ...)``, neighbours sorted by id."""

type Point2D = tuple[float, float]


class RoutingError(Exception):
    """Base class for every failure this module raises."""


class UnknownNodeError(RoutingError):
    """A node id that is not in the lane graph was asked for."""


class NoRouteError(RoutingError):
    """Both endpoints exist but no drivable sequence of lanes connects them."""


# ------------------------------------------------------------------------ indexing --
@lru_cache(maxsize=8)
def _adjacency(graph: LaneGraph) -> Adjacency:
    """Memoised undirected adjacency for ``graph``.

    Keyed on the frozen :class:`~parkscan_core.lot.LaneGraph` value itself, so two specs
    describing the same graph share one index and a mutated spec can never hand back a
    stale one. The result is a read-only view: callers get a mapping they cannot corrupt.
    """
    nodes = graph.node_map()
    build: dict[str, list[tuple[str, float]]] = {node_id: [] for node_id in nodes}
    for edge in graph.edges:
        a, b = nodes[edge.a], nodes[edge.b]
        length = math.dist((a.x_m, a.y_m), (b.x_m, b.y_m))
        build[edge.a].append((edge.b, length))
        build[edge.b].append((edge.a, length))
    return MappingProxyType({k: tuple(sorted(v)) for k, v in sorted(build.items())})


def adjacency(spec: LotSpec) -> Adjacency:
    """Read-only adjacency of ``spec``'s lane graph, neighbours sorted by node id."""
    return _adjacency(spec.lane_graph)


@lru_cache(maxsize=8)
def _coords(graph: LaneGraph) -> Mapping[str, Point2D]:
    return MappingProxyType({n.id: (n.x_m, n.y_m) for n in graph.nodes})


def node_xy(spec: LotSpec, node_id: str) -> Point2D:
    """Position of a lane node in lot metres.

    Raises:
        UnknownNodeError: if ``node_id`` is not a node of ``spec``'s lane graph.
    """
    try:
        return _coords(spec.lane_graph)[node_id]
    except KeyError:
        raise UnknownNodeError(_unknown(spec, node_id)) from None


def _unknown(spec: LotSpec, node_id: str) -> str:
    known = sorted(spec.lane_graph.node_ids())
    preview = ", ".join(known[:6])
    return (
        f"{node_id!r} is not a lane node of lot {spec.name!r} "
        f"({len(known)} nodes, e.g. {preview}, ...)"
    )


# ------------------------------------------------------------------------ snapping --
def nearest_node(spec: LotSpec, x_m: float, y_m: float) -> str:
    """The lane node closest to ``(x_m, y_m)`` in a straight line.

    This is the **only** Euclidean measurement in the routing stack, and it is a
    projection rather than a cost: a robot standing in free space has to be attached to
    the graph before graph search can begin, and the nearest node is the honest way to do
    that on a lot whose aisles are the only drivable surface. Never use the distance to
    the returned node as a travel cost -- use :func:`lane_distance_m` from it.

    Ties (two nodes exactly equidistant) are broken by node id, so the result is stable.
    """
    coords = _coords(spec.lane_graph)
    if not coords:  # pragma: no cover - LaneGraph validation forbids an empty graph
        msg = f"lot {spec.name!r} has an empty lane graph"
        raise RoutingError(msg)
    best_id = ""
    best_d2 = math.inf
    for node_id in sorted(coords):
        nx, ny = coords[node_id]
        d2 = (nx - x_m) ** 2 + (ny - y_m) ** 2
        if d2 < best_d2:
            best_id, best_d2 = node_id, d2
    return best_id


# -------------------------------------------------------------------------- search --
@lru_cache(maxsize=4096)
def _shortest(graph: LaneGraph, from_node: str, to_node: str) -> tuple[tuple[str, ...], float]:
    """Uniform-cost search returning ``(path, length_m)``. Endpoints are pre-validated.

    Memoised because the battery feasibility check runs *every tick for every robot* and
    asks the same handful of questions ("how far to each bay from here?") over and over.
    The key space is bounded by the node count squared -- 1936 pairs for the shipped lot --
    so the cache is small, and because the search is deterministic the cached answer is the
    answer it would have recomputed.
    """
    adj = _adjacency(graph)
    if from_node == to_node:
        return ((from_node,), 0.0)

    # (cost, path) -- the second element makes the ordering total, which is what turns a
    # "some shortest path" guarantee into "the same shortest path, every run".
    frontier: list[tuple[float, tuple[str, ...]]] = [(0.0, (from_node,))]
    settled: set[str] = set()
    while frontier:
        cost, path = heappop(frontier)
        node = path[-1]
        if node in settled:
            continue
        settled.add(node)
        if node == to_node:
            return (path, cost)
        for neighbour, length in adj[node]:
            if neighbour not in settled:
                heappush(frontier, (cost + length, (*path, neighbour)))
    raise NoRouteError(_no_route(from_node, to_node))


def _no_route(from_node: str, to_node: str) -> str:
    return (
        f"no drivable route from {from_node!r} to {to_node!r}: they are in different "
        f"connected components of the lane graph"
    )


def route(spec: LotSpec, from_node: str, to_node: str) -> list[str]:
    """Shortest lane-graph path from ``from_node`` to ``to_node``, inclusive of both.

    Edge weight is the Euclidean length of that lane segment; the *path* cost is the sum
    of those weights, which is the distance the robot actually drives. Equal-cost paths
    are resolved by picking the lexicographically smallest sequence of node ids, so the
    output is byte-reproducible across runs.

    Returns:
        The node ids to visit in order. ``[from_node]`` when the endpoints are equal.

    Raises:
        UnknownNodeError: either endpoint is not a lane node.
        NoRouteError: the endpoints are in different connected components.
    """
    _require_nodes(spec, from_node, to_node)
    return list(_shortest(spec.lane_graph, from_node, to_node)[0])


def _require_nodes(spec: LotSpec, *nodes: str) -> None:
    known = _adjacency(spec.lane_graph)
    for node in nodes:
        if node not in known:
            raise UnknownNodeError(_unknown(spec, node))


def path_length_m(spec: LotSpec, path: Sequence[str]) -> float:
    """Total driven length of ``path`` in metres.

    ``path`` must be a real walk in the graph: every consecutive pair has to be joined by
    a lane edge. Validating that here is deliberate -- silently summing straight-line
    distances between non-adjacent nodes is exactly the Euclidean-shortcut bug this
    module exists to prevent, and it would under-report every cost it touched.

    Raises:
        UnknownNodeError: a node in ``path`` is not in the graph.
        NoRouteError: two consecutive nodes are not joined by a lane edge.
        ValueError: ``path`` is empty.
    """
    if not path:
        msg = "path_length_m needs at least one node; got an empty path"
        raise ValueError(msg)
    adj = _adjacency(spec.lane_graph)
    for node in path:
        if node not in adj:
            raise UnknownNodeError(_unknown(spec, node))
    total = 0.0
    for a, b in pairwise(path):
        for neighbour, length in adj[a]:
            if neighbour == b:
                total += length
                break
        else:
            msg = f"{a!r} and {b!r} are not joined by a lane edge, so this is not a drivable path"
            raise NoRouteError(msg)
    return total


def lane_distance_m(spec: LotSpec, from_node: str, to_node: str) -> float:
    """Driving distance along the lanes between two nodes, in metres.

    This is *the* distance function for costs and for the battery feasibility check. Never
    substitute ``math.dist`` on the node coordinates: on this lot a robot in the east
    columns is more than twice as far from a bay by road as by straight line, and the
    optimistic figure is what strands it.
    """
    _require_nodes(spec, from_node, to_node)
    return _shortest(spec.lane_graph, from_node, to_node)[1]


def route_points(spec: LotSpec, path: Sequence[str]) -> list[Point2D]:
    """The metric waypoints of ``path``, ready to hand to the navigation layer."""
    coords = _coords(spec.lane_graph)
    points: list[Point2D] = []
    for node in path:
        try:
            points.append(coords[node])
        except KeyError:
            raise UnknownNodeError(_unknown(spec, node)) from None
    return points

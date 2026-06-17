"""
Pathfinding algorithms for the Fly-in drone routing simulation.
Implements A* with zone-cost awareness and priority zone preference.
No graph libraries used — all logic is hand-implemented.
"""
from __future__ import annotations
import heapq
from models import Graph, Zone, ZoneType


def heuristic(zone: Zone, goal: Zone) -> float:
    """
    Manhattan distance heuristic for A*.

    Args:
        zone: Current zone.
        goal: Target zone.

    Returns:
        Estimated turns to reach goal.
    """
    return (zone.x - goal.x) ** 2 + (zone.y - goal.y) ** 2


def astar(
    graph: Graph,
    start: Zone,
    end: Zone,
    avoid_zones: set[str] | None = None
) -> list[Zone] | None:
    """
    A* pathfinding from start to end in the graph.
    Respects zone movement costs. Prioritizes priority zones.
    Blocked zones are never entered.

    Args:
        graph: The drone network graph.
        start: Starting zone.
        end: Destination zone.
        avoid_zones: Optional set of zone names to treat as blocked.

    Returns:
        Ordered list of zones from start to end (inclusive), or None if no path.
    """
    if avoid_zones is None:
        avoid_zones = set()

    # Priority queue: (f_score, g_score, zone_name)
    open_heap: list[tuple[float, float, str]] = []
    g: dict[str, float] = {start.name: 0.0}
    came_from: dict[str, Zone | None] = {start.name: None}

    f_start = heuristic(start, end)
    heapq.heappush(open_heap, (f_start, 0.0, start.name))
    in_open: set[str] = {start.name}

    while open_heap:
        _, g_current, current_name = heapq.heappop(open_heap)

        current = graph.zones[current_name]
        if current == end:
            return _reconstruct_path(came_from, end)

        if g_current > g.get(current_name, float('inf')):
            continue

        for neighbor, _conn in graph.get_neighbors(current):
            if neighbor.zone_type == ZoneType.BLOCKED:
                continue
            if neighbor.name in avoid_zones and neighbor != end:
                continue

            move_cost: float = float(neighbor.zone_type.cost())
            # Slight preference for priority zones: penalize non-priority paths
            if neighbor.zone_type == ZoneType.PRIORITY:
                move_cost = 0.9  # fractional to prefer without breaking costs
            elif neighbor.zone_type == ZoneType.RESTRICTED:
                move_cost = 2.0

            tentative_g = g[current_name] + move_cost

            if tentative_g < g.get(neighbor.name, float('inf')):
                g[neighbor.name] = tentative_g
                came_from[neighbor.name] = current
                f = tentative_g + heuristic(neighbor, end)
                heapq.heappush(open_heap, (f, tentative_g, neighbor.name))
                in_open.add(neighbor.name)

    return None


def _reconstruct_path(came_from: dict[str, Zone | None], end: Zone) -> list[Zone]:
    """
    Reconstruct the path from came_from map.

    Args:
        came_from: Map of zone to its predecessor.
        end: The destination zone.

    Returns:
        List of zones from start to end.
    """
    path: list[Zone] = []
    current: Zone | None = end
    while current is not None:
        path.append(current)
        current = came_from.get(current.name)
    path.reverse()
    return path


def find_k_shortest_paths(
    graph: Graph,
    start: Zone,
    end: Zone,
    k: int = 3
) -> list[list[Zone]]:
    """
    Find up to k distinct paths using Yen's algorithm concept.
    Uses repeated A* with increasing avoidance to find alternative routes.

    Args:
        graph: The drone network graph.
        start: Starting zone.
        end: Destination zone.
        k: Maximum number of paths to find.

    Returns:
        List of paths (each path is a list of zones), ordered by cost.
    """
    paths: list[list[Zone]] = []

    # Find first path
    first = astar(graph, start, end)
    if first is None:
        return []
    paths.append(first)

    for _ in range(k - 1):
        # Try to find alternative by avoiding zones from previous paths
        best_alt: list[Zone] | None = None
        best_cost = float('inf')

        # Try avoiding each intermediate zone from existing paths
        avoided_combos: list[set[str]] = []
        for existing_path in paths:
            for i in range(1, len(existing_path) - 1):
                avoid = {existing_path[i].name}
                if avoid not in avoided_combos:
                    avoided_combos.append(avoid)

        for avoid in avoided_combos:
            alt = astar(graph, start, end, avoid)
            if alt and alt not in paths:
                cost = path_cost(alt)
                if cost < best_cost:
                    best_cost = cost
                    best_alt = alt

        if best_alt is None:
            break
        paths.append(best_alt)

    # Sort by total cost
    paths.sort(key=lambda p: path_cost(p))
    return paths


def path_cost(path: list[Zone]) -> float:
    """
    Compute total turn cost of a path.

    Args:
        path: Ordered list of zones.

    Returns:
        Total integer turns required.
    """
    total = 0.0
    for zone in path[1:]:  # skip start zone
        total += zone.zone_type.cost()
    return total

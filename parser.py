"""
Parser for the Fly-in drone map file format.
Reads .txt map files and constructs a Graph object.
"""
from __future__ import annotations
import re
from models import Graph, Zone, ZoneType, Connection


class ParseError(Exception):
    """Raised when the map file has a syntax or semantic error."""
    def __init__(self, line_num: int, message: str) -> None:
        super().__init__(f"Line {line_num}: {message}")
        self.line_num = line_num


def _parse_metadata(raw: str, line_num: int) -> dict[str, str]:
    """
    Parse a metadata block like [zone=normal color=red max_drones=2].

    Args:
        raw: The bracket content (without brackets).
        line_num: Line number for error reporting.

    Returns:
        Dictionary of key-value metadata pairs.
    """
    result: dict[str, str] = {}
    tokens = raw.strip().split()
    for token in tokens:
        if '=' not in token:
            raise ParseError(line_num, f"Invalid metadata token: '{token}'")
        key, _, value = token.partition('=')
        if not key or not value:
            raise ParseError(line_num, f"Malformed metadata: '{token}'")
        result[key.strip()] = value.strip()
    return result


def _parse_zone_line(
    prefix: str,
    rest: str,
    line_num: int,
    is_start: bool,
    is_end: bool
) -> Zone:
    """
    Parse a hub, start_hub, or end_hub line into a Zone.

    Args:
        prefix: The line prefix (hub/start_hub/end_hub).
        rest: Everything after the colon.
        line_num: Line number for error reporting.
        is_start: Whether this is a start zone.
        is_end: Whether this is an end zone.

    Returns:
        A Zone instance.
    """
    # Extract optional metadata block
    meta_match = re.search(r'\[([^\]]*)\]', rest)
    metadata: dict[str, str] = {}
    if meta_match:
        metadata = _parse_metadata(meta_match.group(1), line_num)
        rest = rest[:meta_match.start()].strip()

    parts = rest.strip().split()
    if len(parts) < 3:
        raise ParseError(line_num, f"Expected: {prefix}: <name> <x> <y> [metadata]")

    name = parts[0]
    if '-' in name or ' ' in name:
        raise ParseError(line_num, f"Zone name '{name}' must not contain dashes or spaces")

    try:
        x = int(parts[1])
        y = int(parts[2])
    except ValueError:
        raise ParseError(line_num, "Zone coordinates must be integers")

    if x < 0 or y < 0:
        raise ParseError(line_num, "Zone coordinates must be positive integers")

    # Parse zone type
    zone_type_str = metadata.get('zone', 'normal')
    try:
        zone_type = ZoneType(zone_type_str)
    except ValueError:
        raise ParseError(
            line_num,
            f"Invalid zone type '{zone_type_str}'. "
            f"Must be one of: normal, blocked, restricted, priority"
        )

    # Parse max_drones
    max_drones = 1
    if 'max_drones' in metadata:
        try:
            max_drones = int(metadata['max_drones'])
            if max_drones <= 0:
                raise ValueError()
        except ValueError:
            raise ParseError(line_num, "max_drones must be a positive integer")

    color = metadata.get('color', None)

    return Zone(
        name=name,
        x=x,
        y=y,
        zone_type=zone_type,
        color=color,
        max_drones=max_drones,
        is_start=is_start,
        is_end=is_end
    )


def _parse_connection_line(rest: str, line_num: int, graph: Graph) -> Connection:
    """
    Parse a connection line into a Connection object.

    Args:
        rest: Everything after 'connection: '.
        line_num: Line number for error reporting.
        graph: The current graph (to look up zones).

    Returns:
        A Connection instance.
    """
    meta_match = re.search(r'\[([^\]]*)\]', rest)
    metadata: dict[str, str] = {}
    if meta_match:
        metadata = _parse_metadata(meta_match.group(1), line_num)
        rest = rest[:meta_match.start()].strip()

    rest = rest.strip()
    # Use a single dash as separator — zone names cannot contain dashes
    parts = rest.split('-')
    if len(parts) != 2:
        raise ParseError(
            line_num,
            f"Connection must be 'zone1-zone2', got: '{rest}'"
        )

    name_a, name_b = parts[0].strip(), parts[1].strip()

    zone_a = graph.get_zone(name_a)
    zone_b = graph.get_zone(name_b)

    if zone_a is None:
        raise ParseError(line_num, f"Unknown zone '{name_a}'")
    if zone_b is None:
        raise ParseError(line_num, f"Unknown zone '{name_b}'")

    # Parse max_link_capacity
    max_link_capacity = 1
    if 'max_link_capacity' in metadata:
        try:
            max_link_capacity = int(metadata['max_link_capacity'])
            if max_link_capacity <= 0:
                raise ValueError()
        except ValueError:
            raise ParseError(line_num, "max_link_capacity must be a positive integer")

    return Connection(
        zone_a=zone_a,
        zone_b=zone_b,
        max_link_capacity=max_link_capacity
    )


def parse_map_file(filepath: str) -> Graph:
    """
    Parse a drone map file and return a fully constructed Graph.

    Args:
        filepath: Path to the map file.

    Returns:
        A Graph object representing the drone network.

    Raises:
        ParseError: If any syntax or semantic error is found.
        FileNotFoundError: If the file does not exist.
    """
    graph = Graph()
    seen_connections: set[frozenset[str]] = set()
    start_count = 0
    end_count = 0
    nb_drones_set = False

    with open(filepath, 'r') as f:
        lines = f.readlines()

    for line_num, raw_line in enumerate(lines, start=1):
        line = raw_line.strip()

        # Skip empty lines and comments
        if not line or line.startswith('#'):
            continue

        # nb_drones
        if line.startswith('nb_drones:'):
            rest = line[len('nb_drones:'):].strip()
            try:
                nb = int(rest)
                if nb <= 0:
                    raise ValueError()
            except ValueError:
                raise ParseError(line_num, "nb_drones must be a positive integer")
            graph.nb_drones = nb
            nb_drones_set = True

        elif line.startswith('start_hub:'):
            if not nb_drones_set:
                raise ParseError(line_num, "nb_drones must be defined before zones")
            rest = line[len('start_hub:'):].strip()
            zone = _parse_zone_line('start_hub', rest, line_num, True, False)
            if zone.name in graph.zones:
                raise ParseError(line_num, f"Duplicate zone name '{zone.name}'")
            graph.zones[zone.name] = zone
            graph.start_zone = zone
            start_count += 1

        elif line.startswith('end_hub:'):
            if not nb_drones_set:
                raise ParseError(line_num, "nb_drones must be defined before zones")
            rest = line[len('end_hub:'):].strip()
            zone = _parse_zone_line('end_hub', rest, line_num, False, True)
            if zone.name in graph.zones:
                raise ParseError(line_num, f"Duplicate zone name '{zone.name}'")
            graph.zones[zone.name] = zone
            graph.end_zone = zone
            end_count += 1

        elif line.startswith('hub:'):
            if not nb_drones_set:
                raise ParseError(line_num, "nb_drones must be defined before zones")
            rest = line[len('hub:'):].strip()
            zone = _parse_zone_line('hub', rest, line_num, False, False)
            if zone.name in graph.zones:
                raise ParseError(line_num, f"Duplicate zone name '{zone.name}'")
            graph.zones[zone.name] = zone

        elif line.startswith('connection:'):
            rest = line[len('connection:'):].strip()
            conn = _parse_connection_line(rest, line_num, graph)

            # Check for duplicate connections
            key = frozenset([conn.zone_a.name, conn.zone_b.name])
            if key in seen_connections:
                raise ParseError(
                    line_num,
                    f"Duplicate connection '{conn.zone_a.name}-{conn.zone_b.name}'"
                )
            seen_connections.add(key)
            graph.connections.append(conn)

        else:
            raise ParseError(line_num, f"Unrecognized line format: '{line}'")

    # Validate final state
    if not nb_drones_set:
        raise ParseError(0, "Missing nb_drones definition")
    if start_count != 1:
        raise ParseError(0, f"Expected exactly 1 start_hub, found {start_count}")
    if end_count != 1:
        raise ParseError(0, f"Expected exactly 1 end_hub, found {end_count}")

    return graph

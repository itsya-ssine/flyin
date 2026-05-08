"""
Models for the Fly-in drone routing simulation.
Defines Zone, Connection, Drone, and Graph data structures.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class ZoneType(Enum):
    """Zone movement type with associated turn cost."""
    NORMAL = "normal"
    BLOCKED = "blocked"
    RESTRICTED = "restricted"
    PRIORITY = "priority"

    def cost(self) -> int:
        """Return movement cost in turns for this zone type."""
        if self == ZoneType.RESTRICTED:
            return 2
        return 1


@dataclass
class Zone:
    """Represents a zone (node) in the drone network graph."""
    name: str
    x: int
    y: int
    zone_type: ZoneType = ZoneType.NORMAL
    color: Optional[str] = None
    max_drones: int = 1
    is_start: bool = False
    is_end: bool = False

    def __hash__(self) -> int:
        """Hash based on name."""
        return hash(self.name)

    def __eq__(self, other: object) -> bool:
        """Equality based on name."""
        if not isinstance(other, Zone):
            return False
        return self.name == other.name

    def __repr__(self) -> str:
        """String representation."""
        return f"Zone({self.name})"


@dataclass
class Connection:
    """Represents a bidirectional edge between two zones."""
    zone_a: Zone
    zone_b: Zone
    max_link_capacity: int = 1

    def other(self, zone: Zone) -> Zone:
        """Return the zone on the other end of this connection."""
        if zone == self.zone_a:
            return self.zone_b
        return self.zone_a

    def involves(self, zone: Zone) -> bool:
        """Return True if this connection touches the given zone."""
        return zone == self.zone_a or zone == self.zone_b

    def __hash__(self) -> int:
        """Hash based on sorted zone names for bidirectionality."""
        names = tuple(sorted([self.zone_a.name, self.zone_b.name]))
        return hash(names)

    def __eq__(self, other: object) -> bool:
        """Equality ignoring direction."""
        if not isinstance(other, Connection):
            return False
        return (
            (self.zone_a == other.zone_a and self.zone_b == other.zone_b)
            or (self.zone_a == other.zone_b and self.zone_b == other.zone_a)
        )

    def __repr__(self) -> str:
        """String representation."""
        return f"Connection({self.zone_a.name}-{self.zone_b.name})"


@dataclass
class DroneState:
    """Represents the state of a drone at a given simulation turn."""
    drone_id: int
    current_zone: Optional[Zone] = None
    # If in transit to a restricted zone, store (connection, turns_remaining)
    in_transit: Optional[tuple[Connection, Zone, int]] = None
    arrived: bool = False
    path: list[Zone] = field(default_factory=list)
    path_index: int = 0

    @property
    def drone_name(self) -> str:
        """Return formatted drone name like D1."""
        return f"D{self.drone_id}"

    def is_in_transit(self) -> bool:
        """Return True if drone is currently traversing a restricted connection."""
        return self.in_transit is not None


@dataclass
class Graph:
    """Represents the full drone network graph."""
    zones: dict[str, Zone] = field(default_factory=dict)
    connections: list[Connection] = field(default_factory=list)
    start_zone: Optional[Zone] = None
    end_zone: Optional[Zone] = None
    nb_drones: int = 0

    def get_zone(self, name: str) -> Optional[Zone]:
        """Retrieve a zone by name."""
        return self.zones.get(name)

    def get_neighbors(self, zone: Zone) -> list[tuple[Zone, Connection]]:
        """Return all zones reachable from the given zone with their connections."""
        neighbors: list[tuple[Zone, Connection]] = []
        for conn in self.connections:
            if conn.involves(zone):
                other = conn.other(zone)
                if other.zone_type != ZoneType.BLOCKED:
                    neighbors.append((other, conn))
        return neighbors

    def get_connection(self, zone_a: Zone, zone_b: Zone) -> Optional[Connection]:
        """Return the connection between two zones if it exists."""
        for conn in self.connections:
            if conn.involves(zone_a) and conn.involves(zone_b):
                return conn
        return None

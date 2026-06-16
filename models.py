"""
Models for the Fly-in drone routing simulation.
Defines Zone, Connection, Drone, and Graph data structures.
"""
from __future__ import annotations
from enum import Enum
from typing import Optional, List, Tuple, Dict
from pydantic import BaseModel, Field, field_validator, model_validator


class ZoneType(str, Enum):
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


class Zone(BaseModel):
    """Represents a zone (node) in the drone network graph."""
    name: str = Field(..., description="Unique zone identifier")
    x: int = Field(..., description="X coordinate")
    y: int = Field(..., description="Y coordinate")
    zone_type: ZoneType = Field(default=ZoneType.NORMAL, description="Zone movement type")
    color: Optional[str] = Field(default=None, description="Optional visual color")
    max_drones: int = Field(default=1, ge=1, description="Maximum drones allowed")
    is_start: bool = Field(default=False, description="Is this the start zone?")
    is_end: bool = Field(default=False, description="Is this the end zone?")

    model_config = {
        "frozen": False,  # Allow mutations
        "extra": "forbid",  # Don't allow extra fields
    }

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

    @model_validator(mode="after")
    def validate_start_end(self) -> Zone:
        """Validate that start and end flags are mutually exclusive."""
        if self.is_start and self.is_end:
            raise ValueError("Zone cannot be both start and end")
        return self


class Connection(BaseModel):
    """Represents a bidirectional edge between two zones."""
    zone_a: Zone = Field(..., description="First endpoint zone")
    zone_b: Zone = Field(..., description="Second endpoint zone")
    max_link_capacity: int = Field(default=1, ge=1, description="Maximum drones on link")

    model_config = {
        "frozen": False,
        "extra": "forbid",
    }

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

    @model_validator(mode="after")
    def validate_zones(self) -> Connection:
        """Validate that zones are different."""
        if self.zone_a == self.zone_b:
            raise ValueError("Connection cannot connect a zone to itself")
        return self


class DroneState(BaseModel):
    """Represents the state of a drone at a given simulation turn."""
    drone_id: int = Field(..., ge=1, description="Unique drone identifier")
    current_zone: Optional[Zone] = Field(default=None, description="Current zone location")
    in_transit: Optional[Tuple[Connection, Zone, int]] = Field(
        default=None, 
        description="(connection, destination, turns_remaining) when traversing restricted zone"
    )
    arrived: bool = Field(default=False, description="Has drone reached the end zone?")
    path: List[Zone] = Field(default_factory=list, description="Planned route")
    path_index: int = Field(default=0, ge=0, description="Current position in path")

    model_config = {
        "frozen": False,
        "extra": "forbid",
    }

    @property
    def drone_name(self) -> str:
        """Return formatted drone name like D1."""
        return f"D{self.drone_id}"

    def is_in_transit(self) -> bool:
        """Return True if drone is currently traversing a restricted connection."""
        return self.in_transit is not None

    @field_validator("path_index")
    @classmethod
    def validate_path_index(cls, v: int, info) -> int:
        """Validate path_index is within path bounds."""
        if "path" in info.data and v >= len(info.data["path"]):
            raise ValueError(f"Path index {v} out of range for path of length {len(info.data['path'])}")
        return v


class Graph(BaseModel):
    """Represents the full drone network graph."""
    zones: Dict[str, Zone] = Field(default_factory=dict, description="Zone name to Zone mapping")
    connections: List[Connection] = Field(default_factory=list, description="All connections")
    start_zone: Optional[Zone] = Field(default=None, description="Designated start zone")
    end_zone: Optional[Zone] = Field(default=None, description="Designated end zone")
    nb_drones: int = Field(default=0, ge=0, description="Number of drones in simulation")

    model_config = {
        "frozen": False,
        "extra": "forbid",
    }

    def get_zone(self, name: str) -> Optional[Zone]:
        """Retrieve a zone by name."""
        return self.zones.get(name)

    def get_neighbors(self, zone: Zone) -> List[Tuple[Zone, Connection]]:
        """Return all zones reachable from the given zone with their connections."""
        neighbors: List[Tuple[Zone, Connection]] = []
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

    @model_validator(mode="after")
    def validate_start_end_exist(self) -> Graph:
        """Validate that start and end zones exist in the zones dict."""
        if self.start_zone and self.start_zone.name not in self.zones:
            raise ValueError(f"Start zone '{self.start_zone.name}' not found in zones")
        if self.end_zone and self.end_zone.name not in self.zones:
            raise ValueError(f"End zone '{self.end_zone.name}' not found in zones")
        return self

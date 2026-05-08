"""
Terminal visualizer for the Fly-in drone simulation.
Provides colored output showing drone movements and zone states.
"""
from __future__ import annotations
from models import Graph, Zone, ZoneType, DroneState
from simulation import TurnRecord

# ANSI color codes
RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"

# Zone type colors
ZONE_COLORS: dict[str, str] = {
    "normal": "\033[36m",       # Cyan
    "restricted": "\033[33m",   # Yellow
    "priority": "\033[32m",     # Green
    "blocked": "\033[31m",      # Red
    "start": "\033[92m",        # Bright green
    "end": "\033[93m",          # Bright yellow
}

DRONE_COLORS = [
    "\033[94m",  # Blue
    "\033[95m",  # Magenta
    "\033[96m",  # Cyan
    "\033[91m",  # Red
    "\033[92m",  # Green
    "\033[93m",  # Yellow
    "\033[97m",  # White
    "\033[35m",  # Dark magenta
]

COLOR_MAP: dict[str, str] = {
    "red": "\033[31m",
    "green": "\033[32m",
    "yellow": "\033[33m",
    "blue": "\033[34m",
    "magenta": "\033[35m",
    "cyan": "\033[36m",
    "white": "\033[37m",
    "gray": "\033[90m",
    "grey": "\033[90m",
}


def _zone_color(zone: Zone) -> str:
    """Return ANSI color for a zone based on its type or color attribute."""
    if zone.color and zone.color.lower() in COLOR_MAP:
        return COLOR_MAP[zone.color.lower()]
    if zone.is_start:
        return ZONE_COLORS["start"]
    if zone.is_end:
        return ZONE_COLORS["end"]
    return ZONE_COLORS.get(zone.zone_type.value, "\033[37m")


def _drone_color(drone_id: int) -> str:
    """Return ANSI color for a drone by ID."""
    return DRONE_COLORS[(drone_id - 1) % len(DRONE_COLORS)]


def print_header(graph: Graph) -> None:
    """
    Print simulation header with graph info.

    Args:
        graph: The drone network graph.
    """
    assert graph.start_zone and graph.end_zone
    width = 60
    print()
    print(BOLD + "=" * width + RESET)
    print(BOLD + "  ✈  FLY-IN DRONE ROUTING SIMULATION" + RESET)
    print("=" * width)
    print(f"  Drones:     {BOLD}{graph.nb_drones}{RESET}")
    print(f"  Zones:      {BOLD}{len(graph.zones)}{RESET}")
    print(f"  Connections:{BOLD}{len(graph.connections)}{RESET}")
    sc = _zone_color(graph.start_zone)
    ec = _zone_color(graph.end_zone)
    print(f"  Start:      {sc}{BOLD}{graph.start_zone.name}{RESET} "
          f"({graph.start_zone.x},{graph.start_zone.y})")
    print(f"  End:        {ec}{BOLD}{graph.end_zone.name}{RESET} "
          f"({graph.end_zone.x},{graph.end_zone.y})")
    print("=" * width)
    print()


def print_zone_legend(graph: Graph) -> None:
    """
    Print a legend of all zones with types and colors.

    Args:
        graph: The drone network graph.
    """
    print(BOLD + "  ZONES" + RESET)
    for name, zone in graph.zones.items():
        color = _zone_color(zone)
        tag = ""
        if zone.is_start:
            tag = " [START]"
        elif zone.is_end:
            tag = " [END]"
        elif zone.zone_type != ZoneType.NORMAL:
            tag = f" [{zone.zone_type.value.upper()}]"
        cap = f" cap={zone.max_drones}" if zone.max_drones > 1 else ""
        print(f"    {color}●{RESET} {BOLD}{name}{RESET}{tag}{cap}")
    print()


def print_turn(record: TurnRecord, drones: list[DroneState]) -> None:
    """
    Print a single simulation turn with drone movements colored.

    Args:
        record: The turn record with movements.
        drones: List of drone states (for color assignment).
    """
    drone_id_map: dict[str, int] = {f"D{d.drone_id}": d.drone_id for d in drones}
    line = f"  {DIM}Turn {record.turn_number:3d}{RESET}  "
    parts = []
    for drone_name, dest in record.movements:
        did = drone_id_map.get(drone_name, 1)
        dc = _drone_color(did)
        parts.append(f"{dc}{BOLD}{drone_name}{RESET}-{dest}")
    line += " ".join(parts)
    print(line)


def print_simulation(
    graph: Graph,
    records: list[TurnRecord],
    drones: list[DroneState]
) -> None:
    """
    Print the full simulation output with colors and summary.

    Args:
        graph: The drone network graph.
        records: All turn records from the simulation.
        drones: Final drone states.
    """
    print_header(graph)
    print_zone_legend(graph)

    print(BOLD + "  SIMULATION" + RESET)
    print()

    for record in records:
        print_turn(record, drones)

    print()
    print("=" * 60)
    total_turns = records[-1].turn_number if records else 0
    print(f"  {BOLD}✓ All drones delivered!{RESET}")
    print(f"  Total turns: {BOLD}{total_turns}{RESET}")

    # Secondary metrics
    total_moves = sum(len(r.movements) for r in records)
    avg_per_turn = total_moves / max(total_turns, 1)
    print(f"  Total drone movements: {BOLD}{total_moves}{RESET}")
    print(f"  Avg drones/turn: {BOLD}{avg_per_turn:.1f}{RESET}")
    print("=" * 60)
    print()


def print_raw_output(records: list[TurnRecord]) -> None:
    """
    Print the required machine-readable simulation output.
    Each turn on one line, space-separated drone movements.

    Args:
        records: All turn records from the simulation.
    """
    for record in records:
        line = record.format_output()
        if line:
            print(line)

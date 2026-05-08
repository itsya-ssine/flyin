# Fly-in: Drone Routing Simulation

*This project has been created as part of the 42 curriculum.*

## Description

Fly-in is a drone routing simulation system that navigates a fleet of drones through a
network of connected zones from a start hub to an end hub. The goal is to route all
drones in the **fewest possible simulation turns** while respecting capacity, zone type,
and movement constraints.

### Key features

- Custom A* pathfinding (no external graph libraries)
- Multi-path distribution using Yen's k-shortest paths concept
- Zone types: `normal`, `restricted` (2-turn cost), `priority` (preferred), `blocked`
- Per-zone and per-connection capacity constraints
- Turn-by-turn conflict resolution and deadlock avoidance
- Colored terminal output showing drone movements and zone states

## Instructions

### Install dependencies

```bash
make install
# or manually:
pip install flake8 mypy
```

### Run simulation

```bash
# Default example map
make run

# Custom map
make run MAP=maps/easy_linear.txt

# Raw output only (machine-readable format)
python main.py maps/example.txt --raw
```

### Lint

```bash
make lint         # flake8 + mypy standard
make lint-strict  # mypy --strict
```

### Clean

```bash
make clean
```

## Algorithm Design

### Pathfinding

The pathfinder uses **A\*** with a Manhattan distance heuristic. Zone movement costs are
reflected in the g-score:

- `normal` → 1 turn
- `priority` → 0.9 turns (fractional bias to prefer without altering integer semantics)
- `restricted` → 2 turns
- `blocked` → inaccessible

### Multi-path distribution

The `find_k_shortest_paths` function runs repeated A\* with avoided zones to discover
up to `k` alternative routes. Drones are distributed across these routes in round-robin
order, preferring cheaper paths first.

### Turn scheduling

Each simulation turn:
1. Drones in transit (heading to restricted zones) are advanced first.
2. Active drones are sorted by remaining path length (closer to end = higher priority).
3. Zone and connection capacity is tracked per-turn using arrival/departure counters.
4. Drones that cannot move wait in place and retry next turn.

### Conflict resolution

- Drones moving out of a zone free capacity for the same turn.
- Planned arrivals are tracked to prevent over-filling zones.
- Connection capacity is reserved per-turn before moves are committed.

## Visual Representation

The colored terminal output uses ANSI escape codes:

| Color | Meaning |
|-------|---------|
| Bright green | Start zone |
| Bright yellow | End zone |
| Yellow | Restricted zone |
| Green | Priority zone |
| Red | Blocked zone |
| Cyan | Normal zone |
| Per-drone colors | Individual drone movements |

Turn-by-turn output shows each drone's movement, making it easy to trace paths and
identify bottlenecks.

## Resources

- A* Search Algorithm: https://en.wikipedia.org/wiki/A*_search_algorithm
- Yen's k-shortest paths: https://en.wikipedia.org/wiki/Yen%27s_k-shortest_path_algorithm
- ANSI escape codes: https://en.wikipedia.org/wiki/ANSI_escape_code
- Python type hints: https://docs.python.org/3/library/typing.html
- PEP 257 Docstrings: https://peps.python.org/pep-0257/

### AI usage

Claude (Anthropic) was used to assist with:
- Structuring the project into OOP modules
- Writing docstrings and type hints
- Reviewing edge cases in the parser
- Drafting this README

All code was reviewed, understood, and tested before use.

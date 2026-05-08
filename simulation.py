"""
Simulation engine for the Fly-in drone routing system.
Handles turn-by-turn scheduling, capacity constraints, transit mechanics,
and conflict resolution.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional
from models import Graph, Zone, ZoneType, Connection, DroneState
from pathfinder import find_k_shortest_paths


@dataclass
class TurnRecord:
    """Records drone movements in a single simulation turn."""
    turn_number: int
    movements: list[tuple[str, str]] = field(default_factory=list)
    # (drone_name, destination_label)

    def add_movement(self, drone_name: str, dest_label: str) -> None:
        """Record a drone movement."""
        self.movements.append((drone_name, dest_label))

    def format_output(self) -> str:
        """Format this turn's movements in the required output format."""
        if not self.movements:
            return ""
        parts = [f"{dn}-{dl}" for dn, dl in sorted(self.movements)]
        return " ".join(parts)


class SimulationEngine:
    """
    Core simulation engine.
    Assigns paths to drones, resolves conflicts, and executes turns.
    """

    def __init__(self, graph: Graph) -> None:
        """
        Initialize the simulation engine.

        Args:
            graph: The parsed drone network graph.
        """
        self.graph = graph
        self.drones: list[DroneState] = []
        self.turn_records: list[TurnRecord] = []
        self._zone_occupancy: dict[str, int] = {}
        self._conn_usage: dict[tuple[str, str], int] = {}

    def _conn_key(self, conn: Connection) -> tuple[str, str]:
        """Return a canonical sorted key for a connection."""
        names = sorted([conn.zone_a.name, conn.zone_b.name])
        return (names[0], names[1])

    def setup(self) -> None:
        """Initialize drones and assign paths."""
        assert self.graph.start_zone is not None
        assert self.graph.end_zone is not None

        start = self.graph.start_zone
        end = self.graph.end_zone

        # Find multiple path options
        path_options = find_k_shortest_paths(self.graph, start, end, k=5)
        if not path_options:
            raise RuntimeError("No path found from start to end!")

        # Create drone states
        self.drones = [
            DroneState(drone_id=i + 1, current_zone=start)
            for i in range(self.graph.nb_drones)
        ]

        # Distribute drones across paths using round-robin
        # Prefer shorter/cheaper paths first, balance load
        self._assign_paths(path_options)

    def _assign_paths(self, path_options: list[list[Zone]]) -> None:
        """
        Assign paths to drones.
        Distributes drones across available paths to minimize max turns.

        Args:
            path_options: Available paths ordered by cost.
        """
        n_paths = len(path_options)
        for i, drone in enumerate(self.drones):
            # Assign path in round-robin, prioritizing shortest paths
            path_idx = i % n_paths
            drone.path = path_options[path_idx]
            drone.path_index = 0  # currently at start (index 0)

    def run(self) -> list[TurnRecord]:
        """
        Run the full simulation until all drones arrive.

        Returns:
            List of TurnRecords, one per simulation turn.
        """
        self.setup()
        max_turns = 1000  # safety limit

        for turn_num in range(1, max_turns + 1):
            record = TurnRecord(turn_number=turn_num)
            self._execute_turn(record)
            if record.movements:
                self.turn_records.append(record)

            # Check if all drones arrived
            if all(d.arrived for d in self.drones):
                break

        return self.turn_records

    def _get_zone_load(self, zone: Zone) -> int:
        """Return current drone count in a zone."""
        return self._zone_occupancy.get(zone.name, 0)

    def _get_conn_load(self, conn: Connection) -> int:
        """Return current drone count on a connection."""
        return self._conn_usage.get(self._conn_key(conn), 0)

    def _execute_turn(self, record: TurnRecord) -> None:
        """
        Execute one simulation turn.
        Handles transit drones first, then attempts moves for waiting drones.

        Args:
            record: TurnRecord to fill with movements.
        """
        assert self.graph.end_zone is not None

        # Track what moves will happen this turn
        # drones completing transit this turn
        transit_completions: list[DroneState] = []
        # drones starting transit this turn
        transit_starts: list[tuple[DroneState, Connection, Zone]] = []

        # Step 1: Handle drones in transit (restricted zones, turn 2)
        for drone in self.drones:
            if drone.arrived:
                continue
            if drone.in_transit is not None:
                conn, dest_zone, turns_left = drone.in_transit
                turns_left -= 1
                if turns_left == 0:
                    # Must arrive — check if destination has capacity
                    transit_completions.append(drone)
                else:
                    drone.in_transit = (conn, dest_zone, turns_left)

        # Step 2: Compute current zone occupancy (only non-transit, non-arrived drones)
        current_occupancy: dict[str, int] = {}
        for drone in self.drones:
            if drone.arrived or drone.in_transit is not None:
                continue
            if drone.current_zone:
                current_occupancy[drone.current_zone.name] = (
                    current_occupancy.get(drone.current_zone.name, 0) + 1
                )

        # Step 3: Plan moves for non-transit drones
        # Drones that will move out free up their zone; track planned moves
        planned_moves: list[tuple[DroneState, Zone, Optional[Connection]]] = []
        # Zone occupancy after planned departures
        post_depart_occ = dict(current_occupancy)
        # Zone occupancy after planned arrivals
        post_arrive_occ = dict(current_occupancy)
        # Connection usage planned
        planned_conn_use: dict[tuple[str, str], int] = {}

        # Transit completions are forced arrivals
        for drone in transit_completions:
            if drone.in_transit is None:
                continue
            _conn, dest_zone, _ = drone.in_transit
            post_arrive_occ[dest_zone.name] = (
                post_arrive_occ.get(dest_zone.name, 0) + 1
            )

        # Now plan regular moves
        # Sort drones: those closer to end get priority
        def drone_priority(d: DroneState) -> int:
            if d.path and d.path_index < len(d.path):
                remaining = len(d.path) - d.path_index
                return remaining
            return 999

        active_drones = [
            d for d in self.drones
            if not d.arrived and d.in_transit is None and d.current_zone is not None
        ]
        active_drones.sort(key=drone_priority)

        for drone in active_drones:
            assert drone.current_zone is not None
            moved = self._try_move_drone(
                drone,
                post_depart_occ,
                post_arrive_occ,
                planned_conn_use,
                transit_starts,
                planned_moves
            )
            if moved:
                # Free up current zone
                cname = drone.current_zone.name
                post_depart_occ[cname] = max(0, post_depart_occ.get(cname, 0) - 1)

        # Step 4: Apply all moves
        # Complete transits
        for drone in transit_completions:
            if drone.in_transit is None:
                continue
            conn, dest_zone, _ = drone.in_transit
            drone.in_transit = None
            drone.current_zone = dest_zone
            if dest_zone in drone.path:
                drone.path_index = drone.path.index(dest_zone)
            if dest_zone == self.graph.end_zone:
                drone.arrived = True
                drone.current_zone = dest_zone
            record.add_movement(drone.drone_name, dest_zone.name)

        # Apply regular moves
        for drone, dest_zone, conn_opt in planned_moves:
            if conn_opt is not None and dest_zone.zone_type == ZoneType.RESTRICTED:
                # Start transit
                drone.in_transit = (conn_opt, dest_zone, 1)  # 1 more turn
                # Label uses connection name for output
                conn_label = f"{conn_opt.zone_a.name}-{conn_opt.zone_b.name}"
                record.add_movement(drone.drone_name, conn_label)
            else:
                drone.current_zone = dest_zone
                # Update path index
                if dest_zone in drone.path:
                    new_idx = drone.path.index(dest_zone)
                    if new_idx > drone.path_index:
                        drone.path_index = new_idx
                if dest_zone == self.graph.end_zone:
                    drone.arrived = True
                record.add_movement(drone.drone_name, dest_zone.name)

        # Start transits
        for drone, conn, dest_zone in transit_starts:
            pass  # Already handled via planned_moves

    def _try_move_drone(
        self,
        drone: DroneState,
        post_depart_occ: dict[str, int],
        post_arrive_occ: dict[str, int],
        planned_conn_use: dict[tuple[str, str], int],
        transit_starts: list[tuple[DroneState, Connection, Zone]],
        planned_moves: list[tuple[DroneState, Zone, Optional[Connection]]]
    ) -> bool:
        """
        Attempt to move a drone one step along its path.

        Args:
            drone: The drone to move.
            post_depart_occ: Zone occupancy after planned departures.
            post_arrive_occ: Zone occupancy after planned arrivals.
            planned_conn_use: Connection usage for this turn.
            transit_starts: List to record new transit starts.
            planned_moves: List to record planned zone moves.

        Returns:
            True if a move was planned, False otherwise.
        """
        assert drone.current_zone is not None
        assert self.graph.end_zone is not None

        # Find next zone in path
        if drone.path_index + 1 >= len(drone.path):
            # At end of path or path exhausted — try direct A*
            from pathfinder import astar
            new_path = astar(self.graph, drone.current_zone, self.graph.end_zone)
            if new_path and len(new_path) > 1:
                drone.path = new_path
                drone.path_index = 0
            else:
                return False

        next_zone = drone.path[drone.path_index + 1]

        # Verify connection exists
        conn = self.graph.get_connection(drone.current_zone, next_zone)
        if conn is None:
            # Recalculate path from current zone
            from pathfinder import astar
            new_path = astar(self.graph, drone.current_zone, self.graph.end_zone)
            if not new_path or len(new_path) < 2:
                return False
            drone.path = new_path
            drone.path_index = 0
            next_zone = drone.path[1]
            conn = self.graph.get_connection(drone.current_zone, next_zone)
            if conn is None:
                return False

        if next_zone.zone_type == ZoneType.BLOCKED:
            return False

        # Check zone capacity
        is_end = next_zone == self.graph.end_zone
        if not is_end:
            current_in_zone = post_arrive_occ.get(next_zone.name, 0)
            if current_in_zone >= next_zone.max_drones:
                return False  # Zone full

        # Check connection capacity
        ckey = self._conn_key(conn)
        current_on_conn = planned_conn_use.get(ckey, 0)
        if current_on_conn >= conn.max_link_capacity:
            return False  # Connection full

        # Reserve slot
        planned_conn_use[ckey] = current_on_conn + 1
        post_arrive_occ[next_zone.name] = post_arrive_occ.get(next_zone.name, 0) + 1

        planned_moves.append((drone, next_zone, conn))
        drone.path_index += 1
        return True


def simulate(graph: Graph) -> tuple[list[TurnRecord], list[DroneState]]:
    """
    Run a complete drone routing simulation.

    Args:
        graph: The parsed drone network graph.

    Returns:
        Tuple of (turn_records, final_drone_states).
    """
    engine = SimulationEngine(graph)
    records = engine.run()
    return records, engine.drones

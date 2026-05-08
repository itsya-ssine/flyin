"""
Fly-in: Drone Routing Simulation
Main entry point. Parses a map file and runs the simulation.

Usage:
    python main.py <map_file> [--raw]

Arguments:
    map_file  Path to the drone network map file.
    --raw     Print only the machine-readable output format.
"""
from __future__ import annotations
import sys
import os
from parser import parse_map_file, ParseError
from simulation import simulate
from visualizer import print_simulation, print_raw_output


def main() -> int:
    """
    Main entry point.

    Returns:
        Exit code (0 for success, 1 for error).
    """
    if len(sys.argv) < 2:
        print("Usage: python main.py <map_file> [--raw]", file=sys.stderr)
        print("  --raw    Print only machine-readable output", file=sys.stderr)
        return 1

    map_file = sys.argv[1]
    raw_mode = '--raw' in sys.argv

    if not os.path.isfile(map_file):
        print(f"Error: File not found: '{map_file}'", file=sys.stderr)
        return 1

    try:
        graph = parse_map_file(map_file)
    except ParseError as e:
        print(f"Parse error: {e}", file=sys.stderr)
        return 1
    except FileNotFoundError:
        print(f"Error: Cannot read file '{map_file}'", file=sys.stderr)
        return 1

    try:
        records, drones = simulate(graph)
    except RuntimeError as e:
        print(f"Simulation error: {e}", file=sys.stderr)
        return 1

    if raw_mode:
        print_raw_output(records)
    else:
        print_simulation(graph, records, drones)
        print("\n--- Raw output format ---")
        print_raw_output(records)

    return 0


if __name__ == "__main__":
    sys.exit(main())

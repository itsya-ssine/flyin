#!/usr/bin/env python3
"""Generate a static PNG graph from a Fly-in map file."""

import sys
import matplotlib.pyplot as plt
import networkx as nx
from parser import parse_map_file, ParseError


def visualize(graph, output_file="map_graph.png"):
    """Create a static PNG graph from the parsed Graph object."""
    G = nx.Graph()

    # Add nodes with attributes
    for zone in graph.zones.values():
        color = {
            "start": "green",
            "end": "blue",
            "priority": "gold",
            "restricted": "red",
            "blocked": "black",
            "normal": "lightblue",
        }.get(zone.zone_type.value, "grey")
        G.add_node(
            zone.name,
            color=color,
            size=zone.max_drones * 200,   # scale node size by capacity
            pos=(zone.x, zone.y)          # use given coordinates
        )

    # Add edges
    for conn in graph.connections:
        G.add_edge(
            conn.zone_a.name,
            conn.zone_b.name,
            width=conn.max_link_capacity
        )

    # Extract attributes
    pos = nx.get_node_attributes(G, 'pos')
    node_colors = [G.nodes[n]['color'] for n in G.nodes()]
    node_sizes = [G.nodes[n]['size'] for n in G.nodes()]
    edge_widths = [G.edges[e]['width'] for e in G.edges()]

    # Draw
    plt.figure(figsize=(24, 16))
    nx.draw(
        G, pos,
        node_color=node_colors,
        node_size=node_sizes,
        edge_color='gray',
        width=edge_widths,
        with_labels=True,
        font_size=7,
        font_weight='bold',
        arrows=False
    )
    plt.title("Fly-in Map Graph", fontsize=16)
    plt.tight_layout()
    plt.savefig(output_file, dpi=150)
    plt.close()
    print(f"Graph saved to {output_file}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python vis_map.py <map_file>")
        sys.exit(1)

    map_file = sys.argv[1]
    try:
        graph = parse_map_file(map_file)
    except (ParseError, FileNotFoundError) as e:
        print(f"Error: {e}")
        sys.exit(1)

    visualize(graph)


if __name__ == "__main__":
    main()
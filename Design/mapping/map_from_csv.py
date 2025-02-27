from map_types import Node, Edge
from typing import Iterable

import csv
import os


def nodes_from_csv(file_path: str) -> Iterable[Node]:
    """Reads a CSV file with nodes and yields Node objects.

    the CSV file should have the following columns:
    - x_cm - the x coordinate of the node in centimeters
    - y_cm - the y coordinate of the node in centimeters
    """
    with open(file_path, mode='r', newline='') as csvfile:
        for row in csv.DictReader(csvfile):
            yield Node(float(row['x_cm'])/100.0, float(row['y_cm'])/100.0)


def edges_from_csv(file_path: str) -> Iterable[Edge]:
    """Reads a CSV file with edges and yields Edge objects.

    the CSV file should have the following columns:
    - src_node_id - the id of the source node
    - dest_node_id - the id of the destination node
    - weight_m - the weight of the edge in meters
    """
    with open(file_path, mode='r', newline='') as csvfile:
        for row in csv.DictReader(csvfile):
            yield Edge(int(row['src_node_id']),
                       int(row['dest_node_id']),
                       float(row['weight_m']))


if __name__ == '__main__':
    # Example usage
    nodes_csv = os.path.join(os.curdir, "map_nodes_from_sharepoint.csv")
    for node_id, node in enumerate(nodes_from_csv(nodes_csv)):
        print(node_id, node)

    edges_csv_file = os.path.join(
        os.curdir, "map_edges_from_sharepoint_nodes.csv")
    for edge in edges_from_csv(edges_csv_file):
        print(edge)

    from visualize_map import plot_map
    from map_types import RawMap, MapDimensions
    from rectangular_region import distance
    MAP_X_MIN = 0
    MAP_X_MAX = 6.096
    MAP_Y_MIN = 0
    MAP_Y_MAX = 4.8768
    dimensions = MapDimensions((MAP_X_MIN, MAP_X_MAX), (MAP_Y_MIN, MAP_Y_MAX))
    nodes = tuple(nodes_from_csv(nodes_csv))
    # Adjust the edge lengths to be at least the distance between the nodes. TODO: Remove this hack by fixing this in the actual map and updating the csv file.
    edges = tuple(edge if edge.length >= distance(nodes[edge.src], nodes[edge.dest]) else Edge(edge.src, edge.dest, distance(nodes[edge.src], nodes[edge.dest])+0.01)
                  for edge in edges_from_csv(edges_csv_file))

    map = RawMap(dimensions, nodes, edges)
    plot_map(map)

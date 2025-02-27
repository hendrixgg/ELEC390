import matplotlib.pyplot as plt
from map_types import RawMap, Node, Edge
from math import sqrt


def plot_map(map: RawMap):
    """Plots the nodes and edges on a 2D plane."""
    nodes = map.nodes
    edges = map.edges
    x_coords = [node.x for node in nodes]
    y_coords = [node.y for node in nodes]

    plt.scatter(x_coords, y_coords, label='Nodes')
    for node_id, node in enumerate(nodes):
        plt.text(node.x, node.y, f'{node_id}',
                 fontsize=12, color='orange', ha='right')
    # Add custom legend entry for node labels
    plt.scatter([], [], color='orange', label='Node ID')

    for edge in edges:
        src_node = nodes[edge.src]
        dest_node = nodes[edge.dest]
        plt.plot([src_node.x, dest_node.x], [src_node.y, dest_node.y],
                 'k-', lw=1, label='Edges' if edge == edges[0] else "")
        mid_x = (src_node.x + dest_node.x) / 2
        mid_y = (src_node.y + dest_node.y) / 2
        plt.text(mid_x, mid_y, f'{edge.length:.2f}',
                 fontsize=12, color='red', ha='center')
        # Calculate the direction vector and scale it
        dx = dest_node.x - src_node.x
        dy = dest_node.y - src_node.y
        length = sqrt(dx**2 + dy**2)
        direction_x = dx / length
        direction_y = dy / length
        # Adjust the length to avoid the head extending beyond the edge
        arrow_length = length - 0.1

        plt.arrow(src_node.x, src_node.y, direction_x * arrow_length, direction_y * arrow_length,
                  head_width=0.05, head_length=0.1, fc='k', ec='k')

    # Add custom legend entry for edge lengths
    plt.plot([], [], 'r-', lw=1, label='Edge Length(meters)')

    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('Map Nodes and Edges')
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    from map_types import MapDimensions, Node, Edge
    # Example usage
    dimensions = MapDimensions((0, 3), (0, 3))
    nodes = (Node(x=0, y=0), Node(x=1, y=1), Node(x=2, y=0))
    edges = (Edge(src=0, dest=1, length=10), Edge(src=1, dest=2, length=15))
    map = RawMap(dimensions, nodes, edges)
    plot_map(map)

from typing import Iterable, NamedTuple, Tuple
from rectangular_region import RectangularRegion, Point2D, distance, in_region
# MAP_X_MIN = 0
# MAP_X_MAX = 6.096
# MAP_Y_MIN = 0
# MAP_Y_MAX = 4.8768


MapDimensions = RectangularRegion
MapRegion = RectangularRegion
NodeId = int
EdgeId = int


class Node(Point2D):
    """Fields:
    - x: float - The x coordinate of the point.
    - y: float - The y coordinate of the point.

    A tuple of two floats representing a point in a 2D coordinate space with named access to the x and y coordinate."""

    def __repr__(self):
        return f"Node(x={self.x}, y={self.y})"


class Edge(NamedTuple):
    """Fields:
    - src: NodeId - The source node of the edge.
    - dest: NodeId - The destination node of the edge.
    - length: float - The length of the edge.

    A named tuple that represents an edge in terms of its source node (id), destination node (id), and length. Where the source and destination nodes are the indices of the nodes in the map."""
    src: NodeId
    dest: NodeId
    length: float


class RawMap(Tuple[MapDimensions, Tuple[Node, ...], Tuple[Edge, ...]]):
    """Fields:
    - dimensions: MapDimensions - The dimensions of the map.
    - nodes: tuple[Node] - The nodes in the map.
    - edges: tuple[MapEdge] - The edges in the map.

    It is assumed that the map is a strongly connected graph (there is a path between all pairs of nodes), and that all nodes are within the map dimensions.

    Each NodeId is an index in the `nodes` tuple. Each EdgeId is an index in the `edges` tuple."""

    def __new__(cls, dimensions: MapDimensions, raw_nodes: Iterable[Node], raw_edges: Iterable[Edge]):
        nodes = tuple(raw_nodes)
        edges = tuple(raw_edges)
        # assert that all nodes are within the dimensions
        error_nodes = [node for node in nodes if not in_region(
            dimensions, node)]
        if error_nodes:
            raise ValueError(
                f"Invalid nodes: {error_nodes=} not within {dimensions=}")
        # assert that all edges refer to valid in node indices
        error_edges = [edge for edge in raw_edges if not (0 <= edge.src < len(
            nodes)) or not (0 <= edge.dest < len(nodes))]
        if error_edges:
            raise ValueError(
                f"Invalid edges node ids: {error_edges=} not within the range of valid node ids [0, {len(nodes)-1}]")
        error_edges = [
            edge for edge in raw_edges if edge.length < distance(nodes[edge.src], nodes[edge.dest])]
        if error_edges:
            raise ValueError(
                f"Invalid edges: {error_edges=} must have length greater than or equal to the euclidean distance between the source and destination nodes")
        # TODO: would have to check that the graph is connected for full validation.
        return super().__new__(cls, (dimensions, nodes, edges))

    @property
    def dimensions(self) -> MapDimensions:
        return self[0]

    @property
    def nodes(self) -> Tuple[Node, ...]:
        return self[1]

    @property
    def edges(self) -> Tuple[Edge, ...]:
        return self[2]

    def __repr__(self):
        return f"RawMap(dimensions={self.dimensions}, nodes={self.nodes}, edges={self.edges})"

    def edge_id_valid(self, edge_id: EdgeId) -> bool:
        return 0 <= edge_id < len(self.edges)

    def node_id_valid(self, node_id: NodeId) -> bool:
        return 0 <= node_id < len(self.nodes)


class AdjListMap(Tuple[RawMap, Tuple[Tuple[EdgeId, ...], ...]]):
    """Fields:
    - raw: RawMap - The map that the adjacency list is based on.
    - adjs: tuple[tuple[EdgeId]] - The adjacency list representation of the map.

    A map with an adjacency list representation of the edges. Must be grounded in a RawMap."""

    def __new__(cls, raw_map: RawMap) -> 'AdjListMap':
        adjs = [[] for _ in range(len(raw_map.nodes))]
        for edge_id, edge in enumerate(raw_map.edges):
            adjs[edge.src].append(edge_id)
        return super().__new__(cls, (raw_map, tuple(tuple(a) for a in adjs)))

    @property
    def raw(self) -> RawMap:
        return self[0]

    @property
    def adjs(self) -> Tuple[Tuple[EdgeId, ...], ...]:
        return self[1]

    def __repr__(self):
        return f"AdjListMap(raw={self.raw}, adjs={self.adjs})"


# Test cases
if __name__ == "__main__":
    import traceback
    # test data validation for rectangular region
    # invalid regions
    try:
        region = RectangularRegion((1, 0), (1, 2))
    except ValueError as _:
        print(traceback.format_exc())

    try:
        region = RectangularRegion((0, 1), (2, 1))
    except ValueError as _:
        print(traceback.format_exc())

    # test data validation for edges (there is no validation for edges actually)
    edge = Edge(0, 20, 0.05)

    # test data validation for RawMap
    # invalid map dimensions
    try:
        raw_map = RawMap(MapDimensions((1, 0), (1, 2)), [], [])
    except ValueError as _:
        print(traceback.format_exc())

    # invalid map nodes in relation to map dimensions
    try:
        raw_map = RawMap(MapDimensions((0, 1), (0, 1)),
                         [Node(0, -1), Node(1, 2)], [])
    except ValueError as _:
        print(traceback.format_exc())

    # invalid map edges in relation to nodes
    try:
        raw_map = RawMap(MapDimensions((0, 1), (0, 1)), [Node(0.5, 0.5)], [
            Edge(1, 0, 0.15), Edge(0, -1, 0.15)])
    except ValueError as _:
        print(traceback.format_exc())

    # test data validation for AdjListMap
    # invalid map dimensions
    try:
        adj_map = AdjListMap(RawMap(
            MapDimensions((1, 0), (1, 2)), [], []))
    except ValueError as _:
        print(traceback.format_exc())

    # invalid map nodes in relation to map dimensions
    try:
        adj_map = AdjListMap(RawMap(MapDimensions((0, 1), (0, 1)),
                                    [Node(0, -1), Node(1, 2)], []))
    except ValueError as _:
        print(traceback.format_exc())

    # invalid map edges in relation to nodes
    try:
        adj_map = AdjListMap(RawMap(MapDimensions((0, 1), (0, 1)), [Node(0.5, 0.5)], [
            Edge(1, 0, 0.15), Edge(0, -1, 0.15)]))
    except ValueError as _:
        print(traceback.format_exc())

from dataclasses import dataclass
from typing import Callable

# MAP_X_MIN = 0
# MAP_X_MAX = 6.096
# MAP_Y_MIN = 0
# MAP_Y_MAX = 4.8768


@dataclass(frozen=True)
class RectangularRegion:
    x: tuple[float, float]
    y: tuple[float, float]

    def __post_init__(self):
        # assert x[0] <= x[1] and y[0] <= y[1]
        if not self.x[0] <= self.x[1]:
            raise ValueError(
                f"Invalid x range: {self.x}, must have x[0] <= x[1]")
        if not self.y[0] <= self.y[1]:
            raise ValueError(
                f"Invalid y range: {self.y}, must have y[0] <= y[1]")


MapDimensions = RectangularRegion
MapRegion = RectangularRegion


class MapPoint(tuple[float, float]):
    """This is a tuple of two floats representing a point in the map with named access to the x and y coordinate."""
    def __new__(cls, x: float, y: float):
        return super().__new__(cls, (x, y))

    def x(self) -> float:
        """Return the x coordinate of the point."""
        return self[0]

    def y(self) -> float:
        """Return the y coordinate of the point."""
        return self[1]

    def __repr__(self):
        return f"(x={self.x()}, y={self.y()})"


def in_region(region: MapRegion, point: MapPoint) -> bool:
    """Return True if the point is within the region."""
    return region.x[0] <= point.x() <= region.x[1] and region.y[0] <= point.y() <= region.y[1]


class DirectedSegment(tuple[MapPoint, MapPoint]):

    def __new__(cls, src: MapPoint, dest: MapPoint):
        return super().__new__(cls, (src, dest))

    def src(self):
        """Return the source node."""
        return self[0]

    def dest(self):
        """Return the destination node."""
        return self[1]

    def __repr__(self):
        return f"Segment({self.src()} -> {self.dest()})"


def distance_squared(src: MapPoint, dest: MapPoint) -> float:
    """Return the squared Euclidean distance between two points."""
    return (src.x() - dest.x())**2 + (src.y() - dest.y())**2


def distance(src: MapPoint, dest: MapPoint) -> float:
    """Return the Euclidean distance between two points."""
    return distance_squared(src, dest)**0.5


Node = MapPoint


class Edge(tuple[DirectedSegment, float]):
    def __new__(cls, src: Node, dest: Node, length: float):
        min_distance = distance(src, dest)
        if float(length) < min_distance:
            raise ValueError(
                f"Invalid Edge length: {length=} must be greater than or equal to {min_distance} (the euclidean distance between {src=} and {dest=})")
        return super().__new__(cls, (DirectedSegment(src, dest), float(length)))

    def src_dest(self) -> DirectedSegment:
        """Return the source and destination nodes DirectedSegment."""
        return self[0]

    def src(self):
        """Return the source node."""
        return self[0][0]

    def dest(self):
        """Return the destination node."""
        return self[0][1]

    def length(self) -> float:
        """Return the length of the edge."""
        return self[1]

    def __repr__(self):
        return f"Edge({self.src()} -{self.length()}> {self.dest()})"


NodeId = int
EdgeId = int


@dataclass(frozen=True)
class Map:
    """It is assumed that the map is a strongly connected graph (there is a path between all pairs of nodes), and that all nodes are within the map dimensions."""
    dimensions: MapDimensions
    nodes: list[Node]
    edges: list[Edge]

    def __post_init__(self):
        # assert all nodes are within the map
        error_nodes, error_edges = map_check(self)
        if error_nodes:
            raise ValueError(
                f"Invalid nodes: {error_nodes} not in {self.dimensions}")
        if error_edges:
            raise ValueError(
                f"Invalid edges: {error_edges} not within {self.nodes}")
        # TODO: would have to check that the graph is connected for full validation.

    def edge_id_valid(self, edge_id: EdgeId) -> bool:
        return 0 <= edge_id < len(self.edges)

    def node_id_valid(self, node_id: NodeId) -> bool:
        return 0 <= node_id < len(self.nodes)


def map_check(map: Map) -> tuple[list[Node], list[Edge]]:
    """Return a tuple of lists of (invalid nodes, invalid edges)."""
    # assert that all nodes are within the map
    error_nodes = [node for node in map.nodes if not in_region(
        map.dimensions, node)]
    # assert that all edges contain nodes in map.nodes
    error_edges = [
        edge for edge in map.edges if edge.src() not in map.nodes or edge.dest() not in map.nodes]
    return error_nodes, error_edges


TripRequest = DirectedSegment


class TripActual(DirectedSegment):
    """The actual start and end points of a trip. The map is just used for validation in initialization, and it is simply stored as a reference."""
    def __new__(cls, map: Map, src: Node, dest: Node):
        if src not in map.nodes:
            raise ValueError(f"Invalid src: {src} not in {map.nodes}")
        if dest not in map.nodes:
            raise ValueError(f"Invalid dest: {dest} not in {map.nodes}")
        instance = super().__new__(cls, src, dest)
        instance.map = map
        return instance

    def src(self) -> Node:
        """Return the source node."""
        return self[0]

    def dest(self) -> Node:
        """Return the destination node."""
        return self[1]

    def map(self) -> Map:
        """Return the map used to validate the source and destination locations of the Trip."""
        return self.map

    def __repr__(self):
        return f"TripActual(map={self.map}, {self.src()} -> {self.dest()})"


class TripRoute(list[EdgeId]):
    """A route is a list of edges that form a path from the start to the end of the trip. The map is just used for validation in initialization, and it is simply stored as a reference.

    The route is valid if all edges are in the map, and forall 0 <= i <= n-2. map.edges[i].dest = map.edges[i+1].src."""
    def __new__(cls, map: Map, edge_ids: list[EdgeId]):
        # assert that all edges are in the map
        error_edges = [edge_id for edge_id in edge_ids if not map.edge_id_valid(
            edge_id)]
        if error_edges:
            raise ValueError(
                f"Invalid edges: edge ids {error_edges} are out of bounds for map with {len(map.edges)} edges (must have 0 <= edge_id < len(map.edges)).")
        # assert that forall 0 <= i <= n-2. edge[i].dest = edge[i+1].src
        for id1, id2 in zip(edge_ids[:-1], edge_ids[1:]):
            if map.edges[id1].dest() != map.edges[id2].src():
                raise ValueError(
                    f"Invalid TripRoute edges: map.edges[{id1}].dest != map.edges[{id2}].src ({map.edges[id1].dest()} != {map.edge_ids[id2].src()})")
        instance = super().__new__(cls, edge_ids)
        instance.map = map
        return instance

    def __repr__(self):
        return f"TripRoute({self})"


def tripRequestToTripActual(map: Map, request: TripRequest) -> TripActual:
    # If the request is already in the map, no need to change the locations.
    if request.src() in map.nodes and request.dest() in map.nodes:
        return TripActual(map, request.src(), request.dest())
    else:
        # find the closest nodes to the requested points
        src = min(map.nodes, key=lambda node: distance(node, request.src()))
        dest = min(map.nodes, key=lambda node: distance(node, request.dest()))
        return TripActual(map, src, dest)


TripRouter = Callable[[Map, TripActual], TripRoute]


class TripPlan(tuple[TripRequest, TripActual, TripRoute]):
    """A TripPlan is a tuple of a TripRequest, TripActual, and TripRoute. The TripActual is the actual start and end points of the trip of the vehicle (as close to the request as possible), and the TripRoute is the shortest path from the actual start to the actual end.

    Additional information such as time of the trip and perhaps other vehicle locations"""
    def __new__(cls, map: Map, request: TripRequest, router: TripRouter):
        actual: TripActual = tripRequestToTripActual(map, request)
        # find the shortest path from the actual start to the actual end
        route: TripRoute = router(map, actual.src(), actual.dest())
        return super().__new__(cls, (request, actual, route))


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

    # test data validation for edges
    # invalid length
    try:
        edge = Edge(Node(0.5, 0.5), Node(0.5, 0.6), 0.05)
    except ValueError as _:
        print(traceback.format_exc())

    # test data validation for Map
    # invalid map dimensions
    try:
        map = Map(MapDimensions((1, 0), (1, 2)), [], [])
    except ValueError as _:
        print(traceback.format_exc())

    # invalid map nodes in relation to map dimensions
    try:
        map = Map(MapDimensions((0, 1), (0, 1)), [Node(0, -1), Node(1, 2)], [])
    except ValueError as _:
        print(traceback.format_exc())

    # invalid map edges in relation to nodes
    try:
        map = Map(MapDimensions((0, 1), (0, 1)), [Node(0.5, 0.5)], [
                  Edge(Node(0.5, 0.6), Node(0.5, 0.5), 0.15), Edge(Node(0.5, 0.5), Node(0.6, 0.5), 0.15)])
    except ValueError as _:
        print(traceback.format_exc())

    # test data validation of TripActual
    map = Map(MapDimensions((0, 1), (0, 1)), [Node(0.5, 0.5)], [])
    # invalid src
    try:
        trip = TripActual(map, Node(0.5, 0.6), Node(0.5, 0.5))
    except ValueError as _:
        print(traceback.format_exc())

    # invalid dest
    try:
        trip = TripActual(map, Node(0.5, 0.5), Node(0.5, 0.6))
    except ValueError as _:
        print(traceback.format_exc())

    # test data validation of TripRoute
    map = Map(MapDimensions((0, 1), (0, 1)), [Node(0.5, 0.5), Node(0.5, 0.6)], [
              Edge(Node(0.5, 0.5), Node(0.5, 0.6), 0.15), Edge(Node(0.5, 0.6), Node(0.5, 0.5), 0.25)])
    # invalid edges
    try:
        route = TripRoute(map, [0, 2, 3])
    except ValueError as _:
        print(traceback.format_exc())

    # valid tests
    map = Map(MapDimensions((0, 1), (0, 1)), [Node(0.5, 0.5), Node(0.5, 0.6), Node(0.6, 0.5)], [
              Edge(Node(0.5, 0.5), Node(0.5, 0.6), 0.15), Edge(Node(0.5, 0.5), Node(0.6, 0.5), 0.15)])
    print(TripActual(map, Node(0.5, 0.5), Node(0.5, 0.6)))  # valid trip actual

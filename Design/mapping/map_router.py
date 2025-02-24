from dataclasses import dataclass

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


@dataclass(frozen=True)
class Map:
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

    # test data validation for Map
    # invalid map dimensions
    try:
        map = Map(MapDimensions((1, 0), (1, 2)), [], [])
    except ValueError as _:
        print(traceback.format_exc())

    # invalid nodes
    try:
        map = Map(MapDimensions((0, 1), (0, 1)), [Node(0, -1), Node(1, 2)], [])
    except ValueError as _:
        print(traceback.format_exc())

    # invalid edges
    try:
        map = Map(MapDimensions((0, 1), (0, 1)), [Node(0.5, 0.5)], [
                  Edge(Node(0.5, 0.6), Node(0.5, 0.5), 0.15), Edge(Node(0.5, 0.5), Node(0.6, 0.5), 0.15)])
    except ValueError as _:
        print(traceback.format_exc())

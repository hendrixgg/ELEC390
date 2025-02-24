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
        if self.x[0] > self.x[1]:
            raise ValueError(f"Invalid x range: {self.x}")
        if self.y[0] > self.y[1]:
            raise ValueError(f"Invalid y range: {self.y}")


# type aliases
MapDimensions = RectangularRegion
MapRegion = RectangularRegion


class MapPoint(tuple[float, float]):

    def __new__(cls, x: float, y: float):
        return super().__new__(cls, (x, y))

    def x(self) -> float:
        return self[0]

    def y(self) -> float:
        return self[1]


def in_region(region: MapRegion, point: MapPoint) -> bool:
    return region.x[0] <= point.x() <= region.x[1] and region.y[0] <= point.y() <= region.y[1]


@dataclass(frozen=True)
class Node:
    id: int
    location: MapPoint


@dataclass(frozen=True)
class Edge:
    src: Node
    dest: Node
    length: float


@dataclass(frozen=True)
class Map:
    dimensions: MapDimensions
    nodes: list[Node]
    edges: list[Edge]

    def __post_init__(self):
        # assert that all nodes are within the map
        for node in self.nodes:
            assert in_region(
                self.dimensions, node.location), f"Invalid node: {node.location}"
        # assert that all edges contain nodes in self.nodes
        for edge in self.edges:
            assert edge.src in self.nodes, f"Invalid edge source node: {edge.src}"
            assert edge.dest in self.nodes, f"Invalid edge destination node: {edge.dest}"


@dataclass(frozen=True)
class TripRequest:
    src: MapPoint
    dest: MapPoint

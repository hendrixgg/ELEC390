from typing import Tuple, NamedTuple


class RectangularRegion(Tuple[Tuple[float, float], Tuple[float, float]]):
    """A tuple of two tuples of two floats representing a rectangular region in a plane.

    Fields:
    - x: Tuple[float, float] - The x range where x[0] <= x[1]
    - y: Tuple[float, float] - The y range where y[0] <= y[1]
    """

    def __new__(cls, x: Tuple[float, float], y: Tuple[float, float]) -> 'RectangularRegion':
        # assert x[0] <= x[1] and y[0] <= y[1]
        if not x[0] <= x[1]:
            raise ValueError(f"Invalid x range: {x}, must have x[0] <= x[1]")
        if not y[0] <= y[1]:
            raise ValueError(f"Invalid y range: {y}, must have y[0] <= y[1]")
        return super().__new__(cls, (x, y))

    @property
    def x(self) -> Tuple[float, float]:
        return self[0]

    @property
    def y(self) -> Tuple[float, float]:
        return self[1]

    def __repr__(self) -> str:
        return f"RectangularRegion(x={self.x}, y={self.y})"


# r = RectangularRegion((1, 2), (3, 4))
# print(r)
# print(RectangularRegion((1, 2), (3, 4)) ==
#       RectangularRegion((1, 2), (3, 4)))  # True
# print(RectangularRegion((0, 2), (3, 4)) ==
#       RectangularRegion((1, 2), (3, 4)))  # False


class Point2D(NamedTuple):
    """Fields:
    - x: float - The x coordinate of the point.
    - y: float - The y coordinate of the point.

    A tuple of two floats representing a point in a 2D coordinate space with named access to the x and y coordinate."""
    x: float
    y: float


class DirectedSegment(NamedTuple):
    """Field:
    - src: Point2D - The source point of the segment.
    - dest: Point2D - The destination point of the segment.

    A tuple of two points representing a directed segment in 2D space."""
    src: Point2D
    dest: Point2D

    def __repr__(self) -> str:
        return f"DirectedSegment({self.src} -> {self.dest})"


def in_region(region: Point2D, point: Point2D) -> bool:
    """Return True if the point is within the region."""
    return region.x[0] <= point.x <= region.x[1] and region.y[0] <= point.y <= region.y[1]


def distance_squared(src: Point2D, dest: Point2D) -> float:
    """Return the squared Euclidean distance between two points."""
    return (src.x - dest.x)**2 + (src.y - dest.y)**2


def distance(src: Point2D, dest: Point2D) -> float:
    """Return the Euclidean distance between two points."""
    return distance_squared(src, dest)**0.5

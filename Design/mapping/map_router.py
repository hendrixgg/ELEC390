from pydantic import BaseModel, Field, ValidationError
from pydantic.dataclasses import dataclass

# MAP_X_MIN = 0
# MAP_X_MAX = 6.096
# MAP_Y_MIN = 0
# MAP_Y_MAX = 4.8768


@dataclass
class MapDimensions(BaseModel):
    x: tuple[float, float] = Field(...,
                                   description="(xmin, xmax)", frozen=True)
    y: tuple[float, float] = Field(...,
                                   description="(ymin, ymax)", frozen=True)


class MapPoint(BaseModel):
    x: float = Field(...)
    y: float = Field(...)


class Node(BaseModel):
    id: int
    location: MapPoint


class Edge(BaseModel):
    src: Node = Field(...)
    dest: Node = Field(...)
    length: float = Field(...)

# MapRegion


class Map(BaseModel):
    dimensions: MapDimensions
    nodes: list[Node]
    edges: list[Edge]


class TripRequest(BaseModel):
    src: MapPoint
    dest: MapPoint

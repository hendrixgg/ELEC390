from map_types import AdjListMap, NodeId, EdgeId, Edge, distance
from rectangular_region import DirectedSegment
from typing import Tuple, Callable, Sequence, Any

TripRequest = DirectedSegment


class ActualTrip(Tuple[AdjListMap, NodeId, NodeId]):
    # MapTrip
    """Fields:
    - map: AdjListMap - The map that the trip is on. Just stored as a reference.
    - src_id: NodeId - The source node of the trip.
    - dest_id: NodeId - The destination node of the trip.

    The actual start and end points of a trip. The map is just used for validation in initialization, and it is simply stored as a reference."""

    def __new__(cls, map: AdjListMap, src_id: NodeId, dest_id: NodeId):
        if not map.raw.node_id_valid(src_id):
            raise ValueError(
                f"Invalid src_id: {src_id=} is not a valid NodeId in map with {len(map.raw.nodes)} node(s)")
        if not map.raw.node_id_valid(dest_id):
            raise ValueError(
                f"Invalid dest: {dest_id=} is not a valid NodeId in map with {len(map.raw.nodes)} node(s)")
        return super().__new__(cls, (map, src_id, dest_id))

    @property
    def map(self) -> AdjListMap:
        return self[0]

    @property
    def src_id(self) -> NodeId:
        return self[1]

    @property
    def dest_id(self) -> NodeId:
        return self[2]

    def __repr__(self):
        return f"ActualTrip(map={self.map}, {self.src_id} -> {self.dest_id})"


def argmin(iterable: Sequence, key: Callable[[Any], Any] = None):
    """Return the index of the minimum value of the Sequence."""
    if key is None:
        return min(range(len(iterable)), key=lambda i: iterable[i])
    return min(range(len(iterable)), key=lambda i: key(iterable[i]))


def tripRequestToActualTrip(map: AdjListMap, request: TripRequest) -> ActualTrip:
    """This will fail if the `len(map.nodes)==0` map has no nodes. Because on a map with no nodes, no TripActual is possible."""
    # If the request is already in the map, no need to change the locations.
    if len(map.raw.nodes) == 0:
        raise ValueError("Cannot create a TripActual on a map with no nodes.")
    # find the closest nodes to the requested points
    src_id = argmin(
        map.raw.nodes, key=lambda node: distance(request.src, node))
    dest_id = argmin(
        map.raw.nodes, key=lambda node: distance(request.dest, node))
    return ActualTrip(map, src_id, dest_id)


class TripRoute(Tuple[ActualTrip, Tuple[EdgeId, ...]]):
    # MapTrip_Route
    """Fields:
    - trip: TripActual - The actual trip that the route is for.
    - route: Tuple[EdgeId, ...] - The edge_ids that form the route.

    A route is a list of n edges that form a path from the source node to the destination node of the trip. The map is just used for validation in initialization, and it is simply stored as a reference.

    A route is a list of n>=0 edge_ids validated against a TripActual, trip. Where:
    - If n>0, then trip.src = map.edges[edge_id[0]].src and trip.dest = map.edges[edge_id[n-1]].dest
    - If n>1, then forall 0 <= i <= n-2. map.edges[edge_id[i]].dest = map.edges[edge_id[i+1]].src
    - If n=0, then trip.src = trip.dest"""

    def __new__(cls, trip: ActualTrip, route: Sequence[EdgeId]):
        # assert that all route edge_ids are in the map
        error_edge_ids = [id for id in route if not trip.map.raw.edge_id_valid(
            id)]
        if error_edge_ids:
            raise ValueError(
                f"Invalid edge_ids: {error_edge_ids} not in map with {len(trip.map.raw.edges)} edge(s)")
        # from the above edge_id check, if error_edge_ids is empty, then all edge_ids are valid.
        # assert that the edges form a valid path
        if len(route) > 0:
            if trip.map.raw.edges[route[0]].src != trip.src_id:
                raise ValueError(
                    f"Invalid route edge_id: first edge ({route[0]}) does not start at {trip.src_id=}")
            if trip.map.raw.edges[route[-1]].dest != trip.dest_id:
                raise ValueError(
                    f"Invalid route edge_id: last edge {route[-1]} does not end at {trip.dest_id=}")
            if len(route) > 1:
                for i in range(len(route)-1):
                    edge: Edge = trip.map.raw.edges[route[i]]
                    next_edge: Edge = trip.map.raw.edges[route[i+1]]
                    if edge.dest != next_edge.src:
                        raise ValueError(
                            f"Invalid route: {edge=} does not connect to {next_edge=}")
        else:  # len(edge_ids) == 0
            if trip.src_id != trip.dest_id:
                raise ValueError(
                    f"Invalid edge_ids: {route} must have length > 0 if {trip.src_id=} != {trip.dest_id=}")

        return super().__new__(cls, (trip, tuple(route)))

    @property
    def trip(self) -> ActualTrip:
        return self[0]

    @property
    def route(self) -> Tuple[EdgeId, ...]:
        return self[1]

    def __repr__(self):
        return f"TripRoute(trip={self.trip}, route={self.route})"


TripRouter = Callable[[ActualTrip], TripRoute]

# Ways to implement a TripRouter:
"""
Single-source shortest path algorithms
- Dijkstra's algorithm
- A* search
- Bellman-Ford algorithm

All-pairs shortest path algorithms (pre-compute for all nodes in the map, and then look up the shortest path for each trip)
- Floyd-Warshall algorithm to pre-compute the lengths of the shortest paths between all pairs of nodes in the map and predcessor nodes. Then reconstruct the path upon routing.
- repeated Dijkstra's algorithm (O(n^2 log n) for n nodes)
- repeated A* search (O(n^2 log n) for n nodes)
- Johnson's algorithm
"""


class TripPlan(Tuple[AdjListMap, TripRequest, Tuple[NodeId, NodeId], Tuple[EdgeId, ...]]):
    # MapTripPlan
    """Fields:
    - map: AdjListMap - The map that the trip is on.
    - request: TripRequest - The requested start and end points of the trip.
    - actual: Tuple[NodeId, NodeId] - The actual start and end points of the trip.
    - route: Tuple[EdgeId, ...] - The path from the actual start to the actual end computed by function `create_route`.

    A TripPlan is a tuple of a TripRequest, TripActual, and TripRoute. The TripActual is the actual start and end points of the trip of the vehicle (as close to the request as possible), and the TripRoute is the shortest path from the actual start to the actual end.

    It would also be good to incorporate orientation of the vehicle into the Nodes, perhaps primitively as the edge from which you are arriving at the node. Additional information such as time of the trip and perhaps other vehicle locations could be incorporated into the TripRouter for future iterations."""

    def __new__(cls, map: AdjListMap, request: TripRequest, actual: Tuple[NodeId, NodeId], route: Tuple[EdgeId, ...]):
        # validation, errors will be raised by the TripActual and TripRoute constructors
        TripRoute(ActualTrip(map, actual[0], actual[1]), route)
        return super().__new__(cls, (map, tuple(request), tuple(actual), tuple(route)))

    @classmethod
    def from_trip_request(cls, map: AdjListMap, request: TripRequest, create_route: TripRouter, requestToActual: Callable[[AdjListMap, TripRequest], ActualTrip] = tripRequestToActualTrip) -> 'TripPlan':
        actual: ActualTrip = requestToActual(map, request)
        route: TripRoute = create_route(actual)
        return cls(map, request, (actual.src_id, actual.dest_id), tuple(route.route))

    @property
    def map(self) -> AdjListMap:
        return self[0]

    @property
    def request(self) -> TripRequest:
        return self[1]

    @property
    def actual(self) -> Tuple[NodeId, NodeId]:
        return self[2]

    @property
    def route(self) -> Tuple[EdgeId, ...]:
        return self[3]

    def __repr__(self):
        return f"TripPlan(map={self.map}, request={self.request}, actual={self.actual}, route={self.route})"


if __name__ == "__main__":
    from map_types import RawMap, MapDimensions, Node, Edge
    import traceback
    # test data validation of TripActual
    map = AdjListMap.create_from_raw(
        MapDimensions((0, 1), (0, 1)), [Node(0.5, 0.5)], [])
    # invalid src
    try:
        trip = ActualTrip(map, 1, 0)
    except ValueError as _:
        print(traceback.format_exc())

    # invalid dest
    try:
        trip = ActualTrip(map, 0, 1)
    except ValueError as _:
        print(traceback.format_exc())

    # test tripRequestToValidTrip function
    raw_map = RawMap(MapDimensions((0, 1), (0, 1)), [Node(0.5, 0.5)], [])
    map = AdjListMap(raw_map)
    # invalid request
    try:
        trip = tripRequestToActualTrip(
            map, TripRequest(Node(0, 0), Node(0, 1)))
        print(trip)
    except ValueError as _:
        print(traceback.format_exc())

    # test data validation of TripRoute
    map = AdjListMap.create_from_raw(MapDimensions((0, 1), (0, 1)), [Node(0.5, 0.5), Node(0.5, 0.6)], [
        Edge(0, 1, 0.15), Edge(1, 0, 0.25)])
    # invalid edge_ids
    # edge_ids not in map
    try:
        route = TripRoute(ActualTrip(map, 0, 1), [0, 2, 3])
        print(route)
    except ValueError as _:
        print(traceback.format_exc())
    # edges do not start at src
    try:
        route = TripRoute(ActualTrip(map, 0, 1), [1])
        print(route)
    except ValueError as _:
        print(traceback.format_exc())

    # test data validation of TripPlan
    map = AdjListMap(RawMap(MapDimensions((0, 1), (0, 1)), [Node(0.5, 0.5), Node(0.5, 0.6), Node(0.6, 0.5)], [
        Edge(0, 1, 0.15), Edge(0, 2, 0.15)]))
    # valid trip plan
    print(TripPlan.from_trip_request(map, TripRequest(
        Node(0.5, 0.5), Node(0.5, 0.6)), lambda x: TripRoute(x, [0])))

    # invalid route for the trip plan
    try:
        plan = TripPlan(map, TripRequest(
            Node(0.5, 0.5), Node(0.5, 0.6)), (0, 1), (1,))
        print(plan)
    except ValueError as _:
        print(traceback.format_exc())

    # valid tests
    raw_map = RawMap(MapDimensions((0, 1), (0, 1)), [Node(0.5, 0.5), Node(0.5, 0.6), Node(0.6, 0.5)], [
        Edge(0, 1, 0.15), Edge(0, 2, 0.15)])
    map = AdjListMap(raw_map)
    print(ActualTrip(map, 0, 1))  # valid trip actual

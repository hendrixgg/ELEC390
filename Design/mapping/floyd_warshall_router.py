from map_types import RawMap, AdjListMap, NodeId, EdgeId, Edge
from trip_planning_model import ActualTrip, TripRouter, TripRoute, TripPlan
from typing import Sequence, Tuple, Callable, Literal

import math


def floyd_warshall_router(map: AdjListMap) -> Tuple[Callable[[ActualTrip], float], TripRouter]:
    """Returns functions that take ActualTrip as an argument, this actual trip must be based on the same AdjListMap passed into this function.

    Raises a ValueError if the map contains a negative cycle.

    Returns:
    - Callable[[ActualTrip], float] function that returns the distance of the routed trip (calculated using the stored distance), this will be float('inf') if there is no path from the source to the destination.
    - TripRouter that will raise ValueError if no path exists from trip source to destination."""
    # Pre-compute the shortest paths between all pairs of nodes in the map
    n = len(map.adjs)
    # dist[i][j] is to be the length of the shortest path from i to j
    dist = [[float('inf')]*n for _ in range(n)]
    # pred[i][j] is to be the predecessor edge_id of j on the shortest path from i to j
    pred = [[None]*n for _ in range(n)]
    # initialize paths of length 0
    for i in range(n):
        dist[i][i] = 0
    # initialize paths of length 1
    for src, adj in enumerate(map.adjs):
        for edge_id in adj:
            edge = map.raw.edges[edge_id]
            dest = edge.dest
            if dist[src][dest] > edge.length:
                dist[src][dest] = edge.length
                pred[src][dest] = edge_id
    # compute all other shortest paths
    for k in range(n):
        for i in range(n):
            for j in range(n):
                if dist[i][j] > dist[i][k] + dist[k][j]:
                    dist[i][j] = dist[i][k] + dist[k][j]
                    pred[i][j] = pred[k][j]
    # check for negative cycles
    for i in range(n):
        if dist[i][i] < 0:
            raise ValueError("map contains a negative cycle")

    # At this point, if there is no path from i to j, then dist[i][j] == float('inf') and pred[i][j] == None.
    # At this point, you would also want to check that the graph is strongly connected, but we are assuming that this is the case.

    def reconstruct_path(pred: Sequence[Sequence[EdgeId]], src: NodeId, dest: NodeId) -> Tuple[EdgeId, ...]:
        """Assumes that pred is a valid predecessor matrix (len(pred) == n and forall 0<=i<n. len(pred[i])==n, and entries make sense such that you won't be lead in a circle and build up some infinite path that will never finish and) and that src and dest are valid node ids.

        Also assumes that
        - `pred[i][j]` is the edge_id of the edge that is the predecessor of j on the shortest path from i to j.
        - `0 <= pred[i][j] < len(map.raw.edges)` if pred[i][j] is not None
        - `0 <= src, dest < len(map.raw.nodes)`

        If `pred[i][j]` is None, then there is no path from i to j and the function will raise a ValueError.

        To error out if pred produces a cycle, you could keep track of visited nodes and error out if you visit a node twice."""
        reverse_path = []
        while src != dest:
            edge_id = pred[src][dest]
            if edge_id is None:
                raise ValueError(f"No path from {src} to {dest}")
            reverse_path.append(edge_id)
            dest = map.raw.edges[edge_id].src
        return tuple(reversed(reverse_path))

    # return the TripRouter
    def router(trip: ActualTrip) -> TripRoute:
        if trip.map != map:
            raise ValueError(
                "router: map of trip does not match map of router")
        return TripRoute(trip, reconstruct_path(pred, trip.src_id, trip.dest_id))

    # return the distance function
    def min_distance(trip: ActualTrip) -> float:
        if trip.map != map:
            raise ValueError(
                "router: map of trip does not match map of router")
        return dist[trip.src_id][trip.dest_id]
    return min_distance, router

# Need a function that takes in two edges connected by a node (previous_edge, next_edge) and returns the direction between them, (either 'straight', 'left', or 'right').


def direction_between_edges(map: AdjListMap, previous_edge: EdgeId, next_edge: EdgeId) -> Literal["straight", "left", "right"]:
    """
    Determines the direction between two edges based on the angle between them.

    - If the angle between the two edges is greater than pi/4, then the direction is 'left'.
    - If the angle between the two edges is less than -pi/4, then the direction is 'right'.
    - Otherwise, the direction is 'straight'.
    """
    if map.raw.edges[previous_edge].dest != map.raw.edges[next_edge].src:
        raise ValueError(
            "direction_between_edges: edges are not connected by a node")
    # get the 3 nodes
    previous_node = map.raw.edges[previous_edge].src
    common_node = map.raw.edges[previous_edge].dest
    next_node = map.raw.edges[next_edge].dest
    # use the cross product to determine the direction
    prev_edge_vector = (map.raw.nodes[common_node].x - map.raw.nodes[previous_node].x,
                        map.raw.nodes[common_node].y - map.raw.nodes[previous_node].y)
    next_edge_vector = (map.raw.nodes[next_node].x - map.raw.nodes[common_node].x,
                        map.raw.nodes[next_node].y - map.raw.nodes[common_node].y)
    angle = math.atan2(next_edge_vector[1], next_edge_vector[0]) - \
        math.atan2(prev_edge_vector[1], prev_edge_vector[0])
    if angle > math.pi/4:
        return 'left'
    elif angle < -math.pi/4:
        return 'right'
    return 'straight'


def trip_plan_to_directions(trip_plan: TripPlan, prev_edge: EdgeId) -> Tuple[Literal["straight", "left", "right"], ...]:
    """Returns the directions for the given route.

    The first direction is the direction from the previous edge to the first edge in the route.
    """
    if not trip_plan.map.raw.edge_id_valid(prev_edge):
        raise ValueError("trip_route_to_directions: invalid previous edge")
    map = trip_plan.map
    route = trip_plan.route

    if map.raw.edges[prev_edge].dest != map.raw.edges[route[0]].src:
        raise ValueError(
            "trip_route_to_directions: previous_edge is not connected to the first edge in the route")
    directions = [direction_between_edges(map, prev_edge, route[0])]
    for i in range(len(route)-1):
        directions.append(
            direction_between_edges(map, route[i], route[i+1]))
    return tuple(directions)


# Test cases
if __name__ == "__main__":
    from map_types import RawMap, MapDimensions, Node, Edge
    from trip_planning_model import TripPlan, TripRequest
    import traceback
    # test data validation of TripPlan using floyd_warshall_router
    weak_map = AdjListMap(RawMap(MapDimensions((0, 1), (0, 1)), [Node(0.5, 0.5), Node(0.5, 0.6), Node(0.6, 0.5)], [
        Edge(0, 1, 0.15), Edge(0, 2, 0.15)]))
    # valid trip plan
    weak_router = floyd_warshall_router(weak_map)[1]
    print(TripPlan.from_trip_request(weak_map, TripRequest(
        Node(0.5, 0.5), Node(0.6, 0.5)), weak_router))

    # invalid trip plan due to the map not being strongly connected
    try:
        print(TripPlan.from_trip_request(weak_map, TripRequest(
            Node(0.5, 0.6), Node(0.5, 0.5)), weak_router))
    except ValueError as _:
        print(traceback.format_exc())

    strong_map = AdjListMap(RawMap(MapDimensions((0, 1), (0, 1)), [Node(0.5, 0.5), Node(0.5, 0.6), Node(0.6, 0.5)], [
        Edge(0, 1, 0.15), Edge(0, 2, 0.15), Edge(1, 0, 0.2), Edge(2, 0, 0.2)]))
    # valid trip plan, but using the wrong map.
    try:
        print(TripPlan.from_trip_request(strong_map, TripRequest(
            Node(0.5, 0.5), Node(0.6, 0.5)), weak_router))
    except ValueError as _:
        print(traceback.format_exc())

    strong_router = floyd_warshall_router(strong_map)[1]
    try:
        trip_plan = TripPlan.from_trip_request(strong_map, TripRequest(
            Node(0.5, 0.5), Node(0.6, 0.5)), strong_router)
        print(trip_plan)
        print(trip_plan_to_directions(trip_plan, 2))
    except ValueError as _:
        print(traceback.format_exc())

    import visualize_map
    visualize_map.plot_map(strong_map.raw)

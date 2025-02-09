# Road Map Representation
## Map Model
Map Dimensions: 
- sources: https://queensuca.sharepoint.com/teams/CCT-938446/SitePages/GPS-and-Fare-System.aspx, https://queensuca.sharepoint.com/teams/CCT-938446/SitePages/The-Town.aspx
- in feet: $(x,y)\in[0,20]\times[0,16]$ (float single precision)
- in meters: $(x,y)\in[0,6.096]\times[0,4.8768]$ (float single precision)
- origin is positioned in the bottom-left corner of the map

MapLocation:
```python
{
    "x": float, # x\in[0,6.096]
    "y": float, # y\in[0,4.8768]
}
```
MapRegion:
```python
# Could include some other properties about regions of map locations such as: risk zone, pedestrian zone, traffic circle, taxi depot, etc. 
```
Node (these are the actual points we will consider driving to on the map, intersections and road midpoints between intersections):
```python
{
    "id": int,
    "location": MapLocation,
    "type": "intersection" | "road", # perhaps this could be calculated using MapRegions, this may not be necessary immediately
}
```
```Haskell
distanceSqauared :: MapLocation -> MapLocation -> Float
distanceSquared (x1,y1) (x2, y2) = x*x + y*y
where
    x = x1-x2
    y = y1-y2
```
```Haskell
distance :: MapLocation -> MapLocation -> Float
distance = sqrt . distanceSquared
```
Edge (directed edge):
```python
{
    "id": int,
    "src": Node,
    "dest": Node,
    "length": float, # length in meters, some positive value >= straight line distance from src to dest. This is included because roads may be curved so distance can't be calculated by Node-Node straight line distance.
    # Could include some other properties about the edge such as: risk zone, pedestrian zone, traffic circle, taxi depot. 
}
```
MapNodes:
```python
[Node] # list of Node
```
MapEdges:
```python
[Edge] # list of Edge
```
Map:
```python
{
    "dimensions": {
        "xDim": {"xMin": float, "xMax": float}, # where xMin <= xMax
        "yDim": {"yMin": float, "yMax": float}, # where yMin <= yMax
    },
    "nodes": MapNodes, # where number of Nodes is n
    "edges": MapEdges, # where number of Edges is m
}
```
## Routing Model
(Ride)Fare:
```python
{
    "id": int,          # ID of the fare, used to claim it
    "modifiers": int    # Modifiers which apply to the fare. 0=Normal Fare, 1=Subsized Fare, 2=Senior Fare
    "src": {            # Ducky pickup location, in meters from map origin
      "x": float,
      "y": float
    },
    "dest": {           # Ducky dropoff location, in meters from map origin
      "x": float,
      "y": float,
    },
    "claimed": bool,    # True if the fare is already claimed
    "expiry": float,    # Fare expiry, cannot be claimed after this time. In UTC seconds, see python time.time()
    "pay": float,       # Fare payout
    "reputation": int   # Fare reputation gain, in %
}
```
TripRequest (a trip request from the perspective of the routing algorithm):
```python
{
    "src": MapLocation,
    "dest": MapLocation,
}
```
TripActual:
```python
{
    "src": Node,
    "dest": Node,
}
```
TripRoute:
```python
[Edge] # An ordered list of n edges where: forall 0 <= i <= n-2. edge[i].dest = edge[i+1].src 
```
TripPlan:
```python
{
    "request": TripRequest,
    "actual": TripActual,
    "route": TripRoute,
    # where: actual.src = NodeMin_i(distance(request.src, Node[i].location)) and actual.dest = NodeMin_i(distance(request.dest, Node[i].location))
    # since we are doing argmin(distance), we can equivalently do argmin(distanceSquared): actual.src = NodeMin_i(distanceSquared(request.src, Node[i].location)) and actual.dest = NodeMin_i(distanceSquared(request.dest, Node[i].location))
    # where: route is a list of n edges with route[0].src = actual.src and route[n-1].dest = actual.dest
}
```
```Haskell
-- Actually depends on the Map (more specifically, the list of nodes to choose from)
-- Also depends on the distanceSquared function.
closestNodeTo :: MapNodes -> MapLocation -> Node 
closestNodeTo nodes loc = nodes[i]
where
    i = argmin (distanceSquared loc . nodeLocation) nodes
```
```Haskell
-- Actually depends on the closestNodeTo function to satisfy the requirement of TripPlan.
tripRequestToActual :: TripRequest -> TripActual
tripRequestToActual {src, dest} = {"src": closestNodeTo src, "dest": closestNodeTo dest}
```
```Haskell
-- Actually depends on the Map also 
-- need to implement A* algorithm.
-- this uses a priority queue over nodes in the graph where priority p(n) = g(n) + h(n)
-- where g(n) = currently known cost of shortest path from start to n
-- where h = straight_line_distance_to_goal . nodeLocation
createTripRoute :: MapNodes -> TripActual -> TripRoute
-- see https://en.wikipedia.org/wiki/A*_search_algorithm for pseudocode
```
```Haskell
createTripPlan :: TripRequest -> TripPlan
createTripPlan request = { "request": request, "actual": actual, "route": route }
where
    actual = tripRequestToActual request -- this must satisfy the requirement specified in TripPlan
    route = createTripRoute actual -- this must satisfy the requirement specified in TripPlan
```


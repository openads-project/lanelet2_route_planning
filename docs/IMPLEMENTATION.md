# Implementation Details

## Route Message Format

See [planning_interfaces](https://github.com/ika-rwth-aachen/planning_interfaces?tab=readme-ov-file#overview-of-the-route_planning_msgs).

<img src="https://raw.githubusercontent.com/ika-rwth-aachen/planning_interfaces/refs/heads/main/assets/route_planning_msgs.png">

## Map Requirements

### Regulatory Elements

- Regulatory elements are attached to the lanelets they affect.
- `RightOfWay` lists all yielding lanelets as `yield` and all priority lanelets as `right_of_way`, including bicycle and
  pedestrian lanelets. Crosswalk markings or Areas alone do not establish priority.
- Explicit stop lines intersect the centerline of the affected lane; only their first and last points are used.
- Stop lines may lie beyond the end of the lanelet carrying the rule.
- `AllWayStop` uses the stop line assigned to each lanelet.
- Without a stop line, `RightOfWay`, `AllWayStop`, and `TrafficLight` use the lanelet end.

### Route-Dependent Yield Filtering (Optional)

- The `RightOfWay` references exactly one Area via role `intersection_area`. In OSM, this is a multipolygon relation
  with `subtype=intersection`, `outer` ways, and optional `inner` ways.
- The Area covers all crossing and merging movements of the junction, but no neighboring junction.
- The Area overlaps the first route lanelet after the yielding lanelet and each priority lanelet or its successor.
- `following` connections lead through the junction to its exits without lane changes.
- Approaches end and exits start at the Area boundary.

Yield is omitted only if no priority movement conflicts with the selected route:

- Priority movements are all lanelets reachable inside the Area and their first exits, for each legal direction of the
  vehicle, bicycle, and pedestrian routing graphs.
- A conflict is a shared lanelet or any contact of the 2D centerlines.
- Yield is retained for missing or invalid Area data or routing graphs, dead ends inside the Area, priority directions
  without exit, or search limits.
- Other rule types are not filtered.

#### Reciprocal Right of Way (Left Turns Yield to Oncoming Traffic)

Lanelet2 references a `RightOfWay` at the last lanelet before the intersection. A rule that only applies to left turns
(§9(3) StVO) is therefore mapped as two reciprocal `RightOfWay` relations of the same Area: approach A yields to the
opposite approach B, and B yields to A. Taken literally, every movement of A would yield to every movement of B.

- If the priority approach B yields to the yielding approach A in another `RightOfWay` of the same Area, and the
  selected route does not turn left across B, conflicts with movements of B that turn left across A are ignored.
- A movement turns left across oncoming traffic if, walking counterclockwise along the Area boundary from its approach,
  its exit lies completely beyond the oncoming entry (right-hand traffic). Exit and entry are the intervals spanned by
  the start and end points of the lanelet bounds. Turn angles are not used.
- If exit and oncoming entry overlap, e.g. on a shared bidirectional lanelet, the movement is ambiguous and treated
  conservatively: the route counts as turning left and the priority movement as not turning left, so Yield is retained.
- Left-turn routes are never exempted.

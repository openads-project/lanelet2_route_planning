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

Yield is omitted only if no priority movement conflicts with the selected route:

- Priority movements are all lanelets reachable inside the Area and their first exits, for each legal direction of the
  vehicle, bicycle, and pedestrian routing graphs.
- A conflict is a shared lanelet or any contact of the 2D centerlines.
- Yield is retained for missing or invalid Area data or routing graphs, dead ends inside the Area, priority directions
  without exit, or search limits.
- Other rule types are not filtered.

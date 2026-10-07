# Implementation Details

## Route Message Format

See [planning_interfaces](https://github.com/ika-rwth-aachen/planning_interfaces?tab=readme-ov-file#overview-of-the-route_planning_msgs).

<img src="https://raw.githubusercontent.com/ika-rwth-aachen/planning_interfaces/refs/heads/main/assets/route_planning_msgs.png">

## Map Requirements

- Attach regulatory elements to the affected lanelets. `RightOfWay` relations must identify yielding lanelets through
  `yield` and all priority approaches or crossings through `right_of_way`, including bicycle and pedestrian lanelets.
  A crosswalk marking or Area alone does not establish priority.
- Explicit stop/effect lines must intersect the affected lane's centerline; their first and last points define the
  segment used for assignment. `AllWayStop` uses the line assigned to each lanelet. Without a stop line, `RightOfWay`,
  `AllWayStop`, and `TrafficLight` use the lanelet end. Rules are assigned before the intersected route segment, even
  when the line lies beyond the lanelet carrying the rule; adjacent lanes use their own projected centerlines.
- To enable route-dependent Yield filtering, reference exactly one Lanelet2 Area from the `RightOfWay` relation via
  role `intersection_area`. In OSM, this is a relation with `type=multipolygon`, `subtype=intersection`, and boundary
  ways with role `outer` (optional holes use `inner`). Lanelets need no additional intersection tags.
- The Area must overlap the first route successor after the yielding approach and cover the junction's crossing and
  merging movements. Priority lanelets must overlap it themselves or have a successor that does. Model continuous
  `following` connections through the junction to its exits, without requiring lane changes, and plausible centerlines
  and participant permissions. Avoid including neighboring junctions in the same Area.

Yield is omitted only if every relevant priority traversal is complete and misses the selected route in 2D. Positive
lanelet-area overlap bounds traversal. Conflicts are intersections, endpoint contacts, or shared segments of complete
2D lanelet centerlines, including approaches and first exits; a shared lanelet always counts as a conflict.
Vehicle, bicycle, and pedestrian graphs supply legal priority directions; ego routing uses the vehicle graph.
Missing or invalid Area data, unavailable participant graphs, incomplete traversals, or search limits retain Yield.
The Area filter applies only to Yield; Stop and other rule types retain their usual behavior.

# lanelet2_route_planning

The `lanelet2_route_planning` node plans a route to a destination based on Lanelet2 map. It runs an action server that accepts a destination, plans a route, and continuously publishes feedback on the route progress to the action client. The `plan_route_action_client` node can be used alongside as a standard action client, e.g., by listening to goal poses published from RViz.

- [Container Images](#container-images)
- [lanelet2_route_planning](#lanelet2_route_planning)
- [plan_route_action_client](#plan_route_action_client)


### Container Images

| Description | Image:Tag | Default Command |
| --- | --- | -- |
| latest | `gitlab.ika.rwth-aachen.de:5050/fb-fi/its-modules/planning/lanelet2_route_planning:latest` | `ros2 launch lanelet2_route_planning lanelet2_route_planning_launch.py` |


## `lanelet2_route_planning`

The `lanelet2_route_planning` node computes a shortest path route from a current ego position to a destination based on a Lanelet2 map. Progress along the route is continuously tracked and published as action feedback via the integrated action server, which is accepting a destination in the first place. The route is published at a constant frequency, allowing downstream planning tasks to incorporate knowledge about the road topology. For this purpose, the published route not only contains a suggested reference line, but also information about adjacent lanes, regulatory elements, and drivable space. For the sake of computation and data efficiency, the route is only locally enriched with this additional information. More information on the outgoing `route_planning_msgs/msg/Route` format is found in [`planning_interfaces`](https://github.com/ika-rwth-aachen/planning_interfaces).

### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/ego_data` | `perception_msgs/msg/EgoData` | ego data |

### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/route` | `route_planning_msgs/msg/Route` | route |

### Actions

| Action | Type | Description |
| --- | --- | --- |
| `~/plan_route` | `route_planning_msgs/action/PlanRoute` | plan route to destination |

### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `ll2_map_server_name` | `string` | `ll2_map_server` | Name of lanelet2_map_server node |
| `publish_frequency` | `float` | `10.0` | Frequency of route publication [Hz] |
| `action_feedback_frequency` | `float` | `1.0` | Frequency of action feedback publication [Hz] |
| `sampling_distance` | `float` | `1.0` | Distance between resampled points along route [m] |
| `project_destination_to_reference_line` | `bool` | `true` | Whether to project destination to reference line |
| `destination_distance_threshold` | `float` | `1.0` | Distance to destination where destination is considered reached [m] |
| `enrich_route_ahead_ego_distance` | `float` | `100.0` | Distance ahead of ego position where global route is enriched with more information [m] (negative=unlimited) |
| `enrich_route_behind_ego_distance` | `float` | `10.0` | Distance behind ego position where global route is enriched with more information [m] (negative=unlimited) |
| `route_undershoot_distance` | `float` | `0.0` | Undershoot route by this distance before ego position [m] |
| `route_overshoot_distance` | `float` | `0.0` | Overshoot route by this distance behind destination [m] |
| `max_drivable_space_radius` | `float` | `50.0` | Maximum distance to left/right drivable space bounds, if not otherwise restricted [m] |


## `plan_route_action_client`

The `plan_route_action_client` node is an action client interacting with the `lanelet2_route_planning` action server, allowing to plan a route based on clicked RViz poses or other inputs. It primarily offers three different modes of route planning:
1. goal pose subscriber: plans a route to a `/goal_pose` published by RViz's goal pose plugin
1. waypoints: plans routes to pre-defined waypoints, one after the other
1. random: plans a route to a random destination on the map

### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | destination to navigate to |

### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `ll2_map_server_name` | `string` | `ll2_map_server` | Name of lanelet2_map_server node |
| `waypoints` | `string[]` | `[]` | List of WGS84 waypoints to endlessly follow (list of strings with comma-separated '<LATITUDE>,<LONGITUDE>') |
| `enable_random_destination` | `bool` | `false` | Whether to plan a route to a random destination |
| `enable_continuous_planning` | `bool` | `false` | Whether to continuously plan a new route (either to the next waypoint or to a random destination) |
| `cancel_route` | `bool` | `false` | Cancel active route planning action (to be set at runtime) |

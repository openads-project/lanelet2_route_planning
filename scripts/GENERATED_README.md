## `lanelet2_route_planning`

### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/ego_data` | `perception_msgs/msg/EgoData` | |

### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/route` | `route_planning_msgs/msg/Route` | |

### Actions

| Action | Type | Description |
| --- | --- | --- |
| `~/plan_route` | `route_planning_msgs/action/PlanRoute` | |

### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `ll2_map_server_name` | `string` | `lanelet2_map_server` | Name of lanelet2_map_server node |
| `publish_frequency` | `float` | `10.0` | Frequency of route publication [Hz] |
| `action_feedback_frequency` | `float` | `1.0` | Frequency of action feedback publication [Hz] |
| `sampling_distance` | `float` | `1.0` | Distance between resampled points along route [m] |
| `project_destination_to_reference_line` | `bool` | `true` | Whether to project destination to reference line |
| `destination_distance_threshold` | `float` | `1.0` | Distance to destination where destination is considered reached [m] |
| `required_traveled_distance_proportion` | `float` | `0.5` | Proportion of route length that must have been traveled before considering destination reached [0..1] |
| `enrich_route_ahead_ego_distance` | `float` | `100.0` | Distance ahead of ego position where global route is enriched with more information [m] (negative=unlimited) |
| `enrich_route_behind_ego_distance` | `float` | `10.0` | Distance behind ego position where global route is enriched with more information [m] (negative=unlimited) |
| `route_undershoot_distance` | `float` | `0.0` | Undershoot route by this distance before ego position [m] |
| `route_overshoot_distance` | `float` | `0.0` | Overshoot route by this distance behind destination [m] |
| `max_drivable_space_radius` | `float` | `50.0` | Maximum distance to left/right drivable space bounds, if not otherwise restricted [m] |
| `max_num_threads` | `int` | `0` | Maximum number of threads for parallel processing (0=max available) |
| `transform_timeout` | `float` | `0.02` | How long to wait for a transform to be available [s] |

## `plan_route_action_client`

### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | |

### Action Clients

| Action | Type | Description |
| --- | --- | --- |
| `/planning/lanelet2_route_planning/plan_route` | `route_planning_msgs/action/PlanRoute` | |

### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `ll2_map_server_name` | `string` | `ll2_map_server` | Name of lanelet2_map_server node |
| `waypoints` | `string[]` | `[]` | List of WGS84 waypoints to endlessly follow (list of strings with comma-separated '<LATITUDE>,<LONGITUDE>') |
| `enable_random_destination` | `bool` | `false` | Whether to plan a route to a random destination |
| `enable_continuous_planning` | `bool` | `false` | Whether to continuously plan a new route (either to the next waypoint or to a random destination) |
| `cancel_route` | `bool` | `false` | Cancel active route planning action (to be set at runtime) |

# Lanelet2 Route Planning


The [`lanelet2_route_planning`](.) contains two C++ nodes: [`global_planner_node`](lanelet2_route_planning/src/global_planner_node.cpp) and  [`global_maneuver_action_client`](global_maneuver_action_client/src/global_maneuver_action_client_node.cpp) for route planning based on the Lanelet2 framework. While the global_planner ist responsible for the planning task, the global_maneuver_action_client node, converts a goal_pose topic to a maneuver action request.

It has the following functionalities:

- [Lanelet2 Route Planning](#lanelet2-route-planning)
    - [lanelet2\_route\_planning/global\_planner\_node](#lanelet2_route_planningglobal_planner_node)
      - [Subscribed Topics](#subscribed-topics)
      - [Published Topics](#published-topics)
      - [Actions](#actions)
      - [Parameters](#parameters)
    - [global_maneuver_action_client/global_maneuver_action_client_node](#global_maneuver_action_clientglobal_maneuver_action_client)
      - [Subscribed Topics](#subscribed-topics)
      - [Published Topics](#published-topics)
      - [Actions](#actions)
      - [Parameters](#parameters)
  - [Usage of docker-ros Images](#usage-of-docker-ros-images)
    - [Available Images](#available-images)
    - [Default Command](#default-command)
    - [Launch Files](#launch-files)
    - [Configuration Files](#configuration-files)
    - [Additional Remarks](#additional-remarks)

### lanelet2_route_planning/global_planner_node

#### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/carla_its_converter/ego_vehicle/ego_data` | `perception_msgs/msg/EgoData` | EgoData-Message of the vehicle --> should be changed to `/carla_its_adapter/...` soon! |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/global/driveable_space` | `route_planning_msgs/msg/DriveableSpace` | Publish a `route_planning_msgs::msg::DriveableSpace` in the frame of the Lanelet2 map everytime a new global route is planned |
| `~/global/route` | `route_planning_msgs/msg/Route` | Publish a `route_planning_msgs::msg::Route` in the frame of the Lanelet2 map everytime a new global route is planned |
| `~/local/driveable_space` | `route_planning_msgs/msg/DriveableSpace` | Publish a `route_planning_msgs::msg::DriveableSpace` in the local vehicle frame (`base_link`) and environment with a frequency defined by the parameter `local_path_extraction_rate` |
| `~/local/route` | `route_planning_msgs/msg/Route` | Publish a `route_planning_msgs::msg::Route` in the local vehicle frame (`base_link`) and environment with a frequency defined by the parameter `local_path_extraction_rate` |

#### Actions

| Action | Type | Description |
| --- | --- | --- |
| `~/execute_global_maneuver` | `route_planning_msgs/action/GlobalManeuver` | Plan and execute a global maneuver |

#### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `ego_data_timeout` | `double` | Time-delta for which a EgoData message is classified as to old [s] |
| `map_server_name` | `string` | Name of the Lanelet2 Map Server Node |
| `route_sample_distance` | `double` | Sample distance of the generated route [m] |
| `route_smoothing_factor` | `double` | Parameter to influence the smoothing of the route |
| `lateral_driveable_space_width` | `double` | Maximum width of the generated driveable space [m] |
| `target_reached_thr` | `double` | Distance to the target-position to reach the destination [m] |
| `require_standstill` | `bool` |Bool indicating if it's necessary for the vehicle to stand still at the target-position |
| `vel_threshold_target` | `double` | Velocity threshold to define if the target is reached if `require_standstill` is true [m/s] |
| `local_path_extraction_rate` | `double` | Rate to extract and publish the local path / route / driveable space [Hz] |
| `look_ahead_time` | `double` | Look ahead time for extracting the local path [s] |
| `look_ahead_distance_min` | `double` | Minimum Look-Ahead distance for the local path extraction [m] |
| `look_behind_distance` | `double` | Look-Behind distance for the local path extraction [m] |

### global_maneuver_action_client/global_maneuver_action_client

#### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | PoseStamped-Message to define the goal pose. Could be used to trigger the route planning via RViz using the 2D-Goal-Pose-Tool without needing to trigger a specific action. |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| --- | --- | --- |

#### Actions

| Action | Type | Description |
| --- | --- | --- |
| `~/execute_global_maneuver` | `route_planning_msgs/action/GlobalManeuver` | Plan and execute a global maneuver |

#### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| --- | --- | --- |

## Usage of docker-ros Images

### Available Images

| Tag | Description |
| --- | --- |
| `latest` | latest version |
| `latest-dev` | latest dev version |

### Default Command

```bash
ros2 launch lanelet2_route_planning ll2_route_planning.launch.py
```

### Launch Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `lanelet2_route_planning` | `ll2_route_planning.launch.py` | `/docker-ros/ws/install/share/lanelet2_route_planning/launch` | Default launch file starting the global_planner_node |


### Configuration Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `lanelet2_route_planning` | `params.yml` | `/docker-ros/ws/install/share/lanelet2_route_planning/config` | Parameters for launch file |

### Additional Remarks

- ...

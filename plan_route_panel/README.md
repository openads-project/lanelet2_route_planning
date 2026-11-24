# `plan_route_panel`

RViz panel for controlling the Lanelet2 route planning action client

Start the route client and Lanelet2 route planning server before using the panel.

**Add the panel**

In RViz, select **Panels → Add New Panel → plan_route_panel/PlanRoutePanel**. The default client is `/plan_route_action_client`. For a client in the `/planning` namespace, set **Action Client Node** to `/planning/plan_route_action_client` in **Settings**.

**Planning modes**

Select one mode from the dropdown:

| Mode | Primary button | Behavior |
| --- | --- | --- |
| **Waypoints** | **Plan Route** | Uses `plan_route_action_client/srv/RequestRoute` with `MODE_ROUTE`, or `MODE_LOOP` when **Continuous planning** is enabled. |
| **Random Destination** | **Plan Route** | Uses the same `RequestRoute` service with `MODE_RANDOM`, or `MODE_RANDOM_CONTINUOUS` when **Continuous planning** is enabled. |

**Cancel Route** calls `~/cancel_route` to cancel all goals on the action server. If a route request is still pending, the panel waits for its response before sending the cancel request. This keeps the two requests in order. Automatic planning stops on acceptance. Rejected route or cancel requests appear in the detail field while the panel continues monitoring the current route.

**Saved routes**

Edit the installed `share/plan_route_panel/config/routes.yml` file to add named routes. Each route is an ordered list of WGS84 waypoint strings:

```yaml
routes:
  example_route:
    - "50.787524, 6.050095, -1"
    - "50.779794, 6.050634, -1"
    - "50.784935, 6.043669, -1"
    - "50.787801, 6.046555"
```

Use `latitude, longitude[, wait_time_s]` or `{latitude, longitude, wait_time_s}`. Each waypoint defaults to a wait time of `0` if omitted. Use `-1` for a pass-through waypoint. The final wait time must be nonnegative. **Plan Route** reloads route contents. Restart RViz after changing route names.

**Settings and ROS connections**

| Setting | Purpose |
| --- | --- |
| **LL2 Map Server Name** | Read-only display of the client's `ll2_map_server_name` parameter. |
| **Action Client Node** | Node whose `get_parameters`, `request_route`, and `cancel_route` services the panel uses. |
| **Status Action** | Base name of the `PlanRoute` action. The panel derives `/_action/status`, `/_action/feedback`, and `/_action/get_result` from it. |

RViz saves the endpoint settings. Service requests remain pending after a timeout to preserve their order. The action status does not identify the sending client, so the panel may display a goal from another client on the same server.

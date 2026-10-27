# `plan_route_panel`

RViz panel for configuring and monitoring the existing `plan_route_action_client`. It does not start that node or send `PlanRoute` action goals itself. The client and the Lanelet2 route planning server must already be running.

## Add the panel

In RViz, select **Panels → Add New Panel → plan_route_panel/PlanRoutePanel**. The default client node is `/plan_route_action_client`. Configure endpoint names in the collapsed **Settings** section at the bottom of the panel if your setup uses different names. RViz saves these fields in its configuration.

## Planning modes

Select one mode from the dropdown:

| Mode | Primary button | Behavior |
| --- | --- | --- |
| **Waypoints** | **Plan Route** | Sends the selected saved route through `RequestRoute`, or `ExtendedRequestRoute` with `MODE_LOOP` when **Continuous planning** is enabled. **Replan after fraction** sets `replanning_proportion` (0–1). |
| **Random Destination** | **Plan Route** | Uses `ExtendedRequestRoute` with `MODE_RANDOM`, or `MODE_RANDOM_CONTINUOUS` when **Continuous planning** is enabled. |
| **Destination (Click)** | **Set Destination** | Selects RViz's built-in Goal Pose tool. Click a destination in the RViz view; the tool publishes a `PoseStamped` to the configured goal pose topic and the client handles it. |

**Cancel Route** uses `CancelRoute` to disable automatic planning, clear route state, and asynchronously cancel only this client's goals. Pending service requests finish before cancellation is sent. The panel's status is based on the configured action's status, feedback, and result endpoints. It reports route progress when feedback arrives. The progress bar appears for pending or running Waypoints and Random Destination goals; Destination (Click) uses status text only. Rejected goals may not appear in action status.

**Destination (Click)** does not change the client's random destination, continuous planning, or waypoint parameters. If those modes are still active, the client can subsequently issue another goal. Clear the previous mode before using a clicked destination.

## Saved routes

Edit the installed `share/plan_route_panel/config/routes.yml` file to add named routes. Each route is an ordered list of the same WGS84 waypoint strings accepted by the client's `waypoints` parameter:

```yaml
routes:
  example_route:
    - "50.787524, 6.050095, -1"
    - "50.779794, 6.050634, -1"
    - "50.784935, 6.043669, -1"
    - "50.787801, 6.046555"
```

The format is `latitude, longitude[, wait_time_s]`. Use `-1` for an intermediate waypoint; the final waypoint must have a nonnegative wait time or omit it. The panel also accepts the older `{latitude, longitude, wait_time_s}` mapping format. Restart RViz after changing the route names, since the dropdown is populated when the panel is created. Route contents are read again when **Plan Route** is pressed and sent as a `RouteRequest`.

## Settings and ROS connections

| Setting | Purpose |
| --- | --- |
| **LL2 Map Server Name** | Display/configuration field only. The client reads `ll2_map_server_name` at startup; editing this field does not reconfigure it. |
| **Action Client Node** | Node whose `get_parameters`, `request_route`, `extended_request_route`, and `cancel_route` services the panel uses. |
| **Goal Pose Topic** | Topic assigned to RViz's Goal Pose tool in click mode; the client must subscribe to the same topic. |
| **Status Action** | Base name of the `PlanRoute` action. The panel derives `/_action/status`, `/_action/feedback`, and `/_action/get_result` from it. |

The panel reads client parameters when RViz initializes, when the client parameter service becomes available, or when the client node name changes. It does not continually poll parameter values. Planning and cancellation use asynchronous route services instead of parameter updates. Errors and timeouts are displayed; timed-out service requests remain pending to preserve request ordering. The action server's goal IDs do not identify the client that sent each goal, so goals from another client on the same action can appear as the panel's current goal.

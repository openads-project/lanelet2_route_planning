#pragma once

#include <vector>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <geometry_msgs/msg/point.hpp>
#include <new_lanelet2_route_planning_interfaces/msg/route.hpp>
#include <perception_msgs/msg/ego_data.hpp>

namespace new_lanelet2_route_planning {

namespace ll = lanelet;

ll::BasicPoint2d rosToLaneletPoint(const geometry_msgs::msg::Point& point);

ll::BasicPoint2d rosToLaneletPoint(const perception_msgs::msg::EgoData& ego_data);

geometry_msgs::msg::Point laneletToRosPoint(const ll::BasicPoint2d& point);

bool buildRoutingGraph(const ll::LaneletMapConstPtr& map, const ll::traffic_rules::TrafficRulesPtr& traffic_rules,
                       ll::routing::RoutingGraphUPtr& routing_graph);

bool findLaneletAtPoint(const ll::LaneletMapConstPtr& map, const ll::BasicPoint2d& point, ll::ConstLanelet& lanelet,
                        const std::optional<ll::traffic_rules::TrafficRulesPtr> traffic_rules = std::nullopt);

bool findLaneletAtPoint(const ll::LaneletMapConstPtr& map, const geometry_msgs::msg::Point& point,
                        ll::ConstLanelet& lanelet,
                        const std::optional<ll::traffic_rules::TrafficRulesPtr> traffic_rules = std::nullopt);

bool findLaneletAtEgoPosition(const ll::LaneletMapConstPtr& map, const std::string& map_frame_id,
                              const perception_msgs::msg::EgoData& ego_data, ll::ConstLanelet& lanelet,
                              const std::optional<ll::traffic_rules::TrafficRulesPtr> traffic_rules = std::nullopt);

ll::BasicPoint2d projectPointToCenterline(const ll::BasicPoint2d& position, const ll::ConstLanelet& lanelet);

ll::BasicPoint2d projectPointToCenterline(const geometry_msgs::msg::Point& position, const ll::ConstLanelet& lanelet);

ll::BasicPoint2d projectPointToCenterline(const perception_msgs::msg::EgoData& ego_data,
                                          const ll::ConstLanelet& lanelet);

ll::ConstLanelet followLanelet(const ll::routing::RoutingGraphUPtr& routing_graph, const ll::ConstLanelet& lanelet,
                               const ll::BasicPoint2d& position, const double distance);

new_lanelet2_route_planning_interfaces::msg::Route laneletToRosRoute(const ll::routing::Route& route);

std::vector<new_lanelet2_route_planning_interfaces::msg::RouteElement> laneletToRosRouteElements(
    const ll::ConstLanelet& shortest_path_lanelet, const ll::routing::Route& route);

}  // namespace new_lanelet2_route_planning

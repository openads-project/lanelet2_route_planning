#pragma once

#include <vector>

#include <Eigen/Core>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <geometry_msgs/msg/point.hpp>
#include <route_planning_msgs/msg/route.hpp>
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

// TODO: rewrite utilities for simpler datatypes? when to use constlinestring, when to use basic? Basic is probably better, just a typedef on vector<Eigen>
ll::BasicLineString2d resampleLineString(const ll::BasicLineString2d& line, const double delta_s, double& offset);

ll::BasicPoint2d projectPointToCenterline(const ll::BasicPoint2d& position, const ll::ConstLanelet& lanelet);

ll::BasicPoint2d projectPointToCenterline(const geometry_msgs::msg::Point& position, const ll::ConstLanelet& lanelet);

ll::BasicPoint2d projectPointToCenterline(const perception_msgs::msg::EgoData& ego_data,
                                          const ll::ConstLanelet& lanelet);

ll::BasicLineString2d projectLinePointsToOtherLine(const ll::BasicLineString2d& line, const ll::BasicLineString2d& other_line);

Eigen::Vector2d projectPointToLineAlongAxis(const Eigen::Vector2d& point, const Eigen::Vector2d& axis,
    const std::vector<Eigen::Vector2d>& line, bool& found_intersection_with_line_segment);

ll::ConstLanelet followLanelet(const ll::routing::RoutingGraphUPtr& routing_graph, const ll::ConstLanelet& lanelet,
                               const ll::BasicPoint2d& position, const double distance);

route_planning_msgs::msg::Route laneletToRosRoute(const ll::routing::Route& route, const std::string& frame_id);

std::vector<route_planning_msgs::msg::RouteElement> laneletToRosRouteElements(
    const ll::ConstLanelet& shortest_path_lanelet, const ll::routing::Route& route, double& resampling_offset);

}  // namespace new_lanelet2_route_planning

#pragma once

#include <vector>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <Eigen/Core>
#include <geometry_msgs/msg/point.hpp>
#include <perception_msgs/msg/ego_data.hpp>
#include <route_planning_msgs/msg/route.hpp>

namespace new_lanelet2_route_planning {

namespace ll = lanelet;

bool buildRoutingGraph(const ll::LaneletMapConstPtr& map, const ll::traffic_rules::TrafficRulesPtr& traffic_rules,
                       ll::routing::RoutingGraphUPtr& routing_graph);

bool findLaneletAtPoint(const ll::LaneletMapConstPtr& map, const Eigen::Vector2d& point, ll::ConstLanelet& lanelet,
                        const std::optional<ll::traffic_rules::TrafficRulesPtr> traffic_rules = std::nullopt);

bool findLaneletAtPoint(const ll::LaneletMapConstPtr& map, const geometry_msgs::msg::Point& point,
                        ll::ConstLanelet& lanelet,
                        const std::optional<ll::traffic_rules::TrafficRulesPtr> traffic_rules = std::nullopt);

bool findLaneletAtEgoPosition(const ll::LaneletMapConstPtr& map, const std::string& map_frame_id,
                              const perception_msgs::msg::EgoData& ego_data, ll::ConstLanelet& lanelet,
                              const std::optional<ll::traffic_rules::TrafficRulesPtr> traffic_rules = std::nullopt);

ll::ConstLanelet followLanelet(const ll::routing::RoutingGraphUPtr& routing_graph, const ll::ConstLanelet& lanelet,
                               const Eigen::Vector2d& position, const double distance);

ll::BasicLineString2d resampleCenterlinesAlongPath(const ll::routing::LaneletPath& path, const double delta_s,
                                                   bool monotonically, std::vector<size_t>& lanelet_idx_by_point);

std::vector<ll::ConstLanelet> adjacentLeftOrRightLanelets(const ll::ConstLanelet& lanelet,
                                                          const ll::routing::Route& route, bool left);

int computeFollowingLaneIdxOffset(const ll::ConstLanelet& lanelet, const ll::ConstLanelet& lanelet_of_next_point,
                                  const ll::routing::Route& route, const ll::routing::RoutingGraphUPtr& routing_graph);

uint8_t laneBoundaryType(const ll::ConstLineString2d& line);

}  // namespace new_lanelet2_route_planning

#pragma once

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <geometry_msgs/msg/point.hpp>
#include <perception_msgs/msg/ego_data.hpp>

namespace new_lanelet2_route_planning {

namespace ll = lanelet;

ll::BasicPoint2d rosToLaneletPoint(const geometry_msgs::msg::Point& point);

ll::BasicPoint2d rosToLaneletPoint(const perception_msgs::msg::EgoData& ego_data);

geometry_msgs::msg::Point laneletToRosPoint(const ll::BasicPoint2d& point);

ll::routing::RoutingGraphUPtr buildRoutingGraph(const ll::LaneletMapConstPtr& map,
                                                const ll::traffic_rules::TrafficRulesPtr& traffic_rules);

ll::ConstLanelet findLaneletAtPoint(
    const ll::LaneletMapConstPtr& map, const ll::BasicPoint2d& point,
    const std::optional<ll::traffic_rules::TrafficRulesPtr> traffic_rules = std::nullopt);

ll::ConstLanelet findLaneletAtPoint(
    const ll::LaneletMapConstPtr& map, const geometry_msgs::msg::Point& point,
    const std::optional<ll::traffic_rules::TrafficRulesPtr> traffic_rules = std::nullopt);

ll::ConstLanelet findLaneletAtEgoPosition(
    const ll::LaneletMapConstPtr& map, const std::string& map_frame_id, const perception_msgs::msg::EgoData& ego_data,
    const std::optional<ll::traffic_rules::TrafficRulesPtr> traffic_rules = std::nullopt);

}  // namespace new_lanelet2_route_planning

#include <tuple>

#include <lanelet2_routing/Route.h>
#include <lanelet2_utilities/lanelet2_utils.hpp>
#include <perception_msgs_utils/object_access.hpp>
#include <rclcpp/rclcpp.hpp>

#include "new_lanelet2_route_planning/utilities.hpp"

namespace new_lanelet2_route_planning {

ll::BasicPoint2d rosToLaneletPoint(const geometry_msgs::msg::Point& point) {
  return ll::BasicPoint2d(point.x, point.y);
}

ll::BasicPoint2d rosToLaneletPoint(const perception_msgs::msg::EgoData& ego_data) {
  return ll::BasicPoint2d(perception_msgs::object_access::getX(ego_data),
                          perception_msgs::object_access::getY(ego_data));
}

geometry_msgs::msg::Point laneletToRosPoint(const ll::BasicPoint2d& point) {
  geometry_msgs::msg::Point ros_point;
  ros_point.x = point.x();
  ros_point.y = point.y();
  return ros_point;
}

ll::routing::RoutingGraphUPtr buildRoutingGraph(const ll::LaneletMapConstPtr& map,
                                                const ll::traffic_rules::TrafficRulesPtr& traffic_rules) {
  ll::routing::RoutingGraphUPtr routing_graph = ll::routing::RoutingGraph::build(*map, *traffic_rules);
  ll::routing::Route::Errors error = routing_graph->checkValidity();
  if (error.size() > 0) {
    // TODO: good idea to have rclcpp here in includes?
    RCLCPP_ERROR(rclcpp::get_logger("new_lanelet2_route_planning"), "Routing graph is invalid");
    for (size_t i = 0; i < error.size(); ++i) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("new_lanelet2_route_planning"), error[i]);
    }
    // return false; // TODO: return?
  }
  return routing_graph;
}

ll::ConstLanelet findLaneletAtPoint(const ll::LaneletMapConstPtr& map, const ll::BasicPoint2d& point,
                                    const std::optional<ll::traffic_rules::TrafficRulesPtr> traffic_rules) {
  // find 5 nearest lanelets
  std::vector<std::pair<double, ll::ConstLanelet>> nearest_lanelets =
      ll::geometry::findNearest(map->laneletLayer, point, 5);

  // find best matching lanelet
  const double max_distance = 10.0;  // TODO: move to parameter or constant
  if (traffic_rules) {
    std::ignore = Lanelet2Utilities::laneletSorting(point, nearest_lanelets, {}, traffic_rules.value(), {});
    for (const auto& ll : nearest_lanelets) {
      if (ll.first <= max_distance && traffic_rules.value()->canPass(ll.second)) {
        return ll.second;
      }
    }
  } else if (!nearest_lanelets.empty()) {
    std::ignore = Lanelet2Utilities::laneletSorting(point, nearest_lanelets, {}, {}, {});
    return nearest_lanelets[0].second;
  }
  RCLCPP_ERROR(rclcpp::get_logger("new_lanelet2_route_planning"),
               "No passable lanelet in within %.3fm of given point (%.3f, %.3f)", max_distance, point.x(), point.y());
  return {};  // TODO: return bool instead? or std::optional?
}

ll::ConstLanelet findLaneletAtPoint(const ll::LaneletMapConstPtr& map, const geometry_msgs::msg::Point& point,
                                    const std::optional<ll::traffic_rules::TrafficRulesPtr> traffic_rules) {
  return findLaneletAtPoint(map, rosToLaneletPoint(point));
}

ll::ConstLanelet findLaneletAtEgoPosition(const ll::LaneletMapConstPtr& map, const std::string& map_frame_id,
                                          const perception_msgs::msg::EgoData& ego_data,
                                          const std::optional<ll::traffic_rules::TrafficRulesPtr> traffic_rules) {
  // check ego data timestamp
  const double timeout = 1.0;  // TODO: move to parameter or constant
  rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
  if ((now - ego_data.header.stamp).seconds() > timeout) {
    RCLCPP_WARN(rclcpp::get_logger("new_lanelet2_route_planning"),
                "Ego data is outdated by more than %.3fs while finding lanelet", timeout);
  }

  // check ego data frame
  if (ego_data.header.frame_id != map_frame_id) {
    RCLCPP_ERROR(rclcpp::get_logger("new_lanelet2_route_planning"),
                 "Ego data frame '%s' does not match map frame '%s' while finding lanelet",
                 ego_data.header.frame_id.c_str(), map_frame_id.c_str());
    return {};  // TODO : return bool instead ? or std::optional ?
  }

  return findLaneletAtPoint(map, rosToLaneletPoint(ego_data));
}

ll::BasicPoint2d projectPointToCenterline(const ll::BasicPoint2d& point, const ll::ConstLanelet& lanelet) {
  ll::BasicPoint3d point_3d = ll::BasicPoint3d(point.x(), point.y(), 0.0);
  ll::BasicPoint3d projected_point_3d = ll::geometry::project(lanelet.centerline(), point_3d);
  return ll::BasicPoint2d(projected_point_3d.x(), projected_point_3d.y());
}

ll::BasicPoint2d projectPointToCenterline(const geometry_msgs::msg::Point& point, const ll::ConstLanelet& lanelet) {
  return projectPointToCenterline(rosToLaneletPoint(point), lanelet);
}

ll::BasicPoint2d projectPointToCenterline(const perception_msgs::msg::EgoData& ego_data,
                                          const ll::ConstLanelet& lanelet) {
  return projectPointToCenterline(rosToLaneletPoint(ego_data), lanelet);
}

}  // namespace new_lanelet2_route_planning

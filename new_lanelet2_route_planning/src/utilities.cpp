#include <tuple>

#include <lanelet2_routing/Route.h>
#include <lanelet2_utilities/lanelet2_utils.hpp>
#include <perception_msgs_utils/object_access.hpp>
#include <rclcpp/rclcpp.hpp>

#include "new_lanelet2_route_planning/utilities.hpp"

namespace new_lanelet2_route_planning {

static const unsigned int k_nearest_lanelets = 5;
static const double k_timeout_ego_data = 1.0;
static const double k_max_distance_lanelet_matching = 5.0;

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

bool buildRoutingGraph(const ll::LaneletMapConstPtr& map, const ll::traffic_rules::TrafficRulesPtr& traffic_rules,
                       ll::routing::RoutingGraphUPtr& routing_graph) {
  routing_graph = ll::routing::RoutingGraph::build(*map, *traffic_rules);
  ll::routing::Route::Errors errors = routing_graph->checkValidity();
  if (errors.size() > 0) {
    // TODO: good idea to have rclcpp here in includes?
    RCLCPP_ERROR(rclcpp::get_logger("new_lanelet2_route_planning"), "Routing graph is invalid");
    for (size_t i = 0; i < errors.size(); ++i) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("new_lanelet2_route_planning"), errors[i]);
    }
    return false;
  }
  return true;
}

bool findLaneletAtPoint(const ll::LaneletMapConstPtr& map, const ll::BasicPoint2d& point, ll::ConstLanelet& lanelet,
                        const std::optional<ll::traffic_rules::TrafficRulesPtr> traffic_rules) {
  // find nearest lanelets
  std::vector<std::pair<double, ll::ConstLanelet>> nearest_lanelets =
      ll::geometry::findNearest(map->laneletLayer, point, k_nearest_lanelets);

  // find best matching lanelet
  if (traffic_rules) {
    std::ignore = Lanelet2Utilities::laneletSorting(point, nearest_lanelets, {}, traffic_rules.value(), {});
    for (const auto& ll : nearest_lanelets) {
      if (ll.first <= k_max_distance_lanelet_matching && traffic_rules.value()->canPass(ll.second)) {
        lanelet = ll.second;
        return true;
      }
    }
  } else if (!nearest_lanelets.empty()) {
    std::ignore = Lanelet2Utilities::laneletSorting(point, nearest_lanelets, {}, {}, {});
    lanelet = nearest_lanelets[0].second;
    return true;
  }
  RCLCPP_ERROR(rclcpp::get_logger("new_lanelet2_route_planning"),
               "No passable lanelet within %.3fm of given point (%.3f, %.3f)", point.x(), point.y());
  return false;
}

bool findLaneletAtPoint(const ll::LaneletMapConstPtr& map, const geometry_msgs::msg::Point& point,
                        ll::ConstLanelet& lanelet,
                        const std::optional<ll::traffic_rules::TrafficRulesPtr> traffic_rules) {
  return findLaneletAtPoint(map, rosToLaneletPoint(point), lanelet, traffic_rules);
}

bool findLaneletAtEgoPosition(const ll::LaneletMapConstPtr& map, const std::string& map_frame_id,
                              const perception_msgs::msg::EgoData& ego_data, ll::ConstLanelet& lanelet,
                              const std::optional<ll::traffic_rules::TrafficRulesPtr> traffic_rules) {
  // check ego data timestamp
  rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
  if ((now - ego_data.header.stamp).seconds() > k_timeout_ego_data) {
    RCLCPP_WARN(rclcpp::get_logger("new_lanelet2_route_planning"),
                "Ego data is outdated by more than %.3fs while finding lanelet", k_timeout_ego_data);
  }

  // check ego data frame
  if (ego_data.header.frame_id != map_frame_id) {
    RCLCPP_ERROR(rclcpp::get_logger("new_lanelet2_route_planning"),
                 "Ego data frame '%s' does not match map frame '%s' while finding lanelet",
                 ego_data.header.frame_id.c_str(), map_frame_id.c_str());
    return false;
  }

  return findLaneletAtPoint(map, rosToLaneletPoint(ego_data), lanelet, traffic_rules);
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

ll::ConstLanelet followLanelet(const ll::routing::RoutingGraphUPtr& routing_graph, const ll::ConstLanelet& lanelet,
                               const ll::BasicPoint2d& position, const double distance) {
  ll::ConstLanelet new_lanelet = lanelet;
  double remaining_length;
  if (distance > 0) {
    remaining_length = ll::geometry::length(lanelet.centerline2d()) -
                       ll::geometry::toArcCoordinates(lanelet.centerline2d(), position).length;
  } else {
    remaining_length = ll::geometry::toArcCoordinates(lanelet.centerline2d(), position).length;
  }
  double remaining_distance = std::abs(distance);
  while (remaining_distance > remaining_length) {
    lanelet::ConstLanelets next_lanelets;
    if (distance > 0) {
      next_lanelets = routing_graph->following(lanelet, false);
    } else {
      next_lanelets = routing_graph->previous(lanelet);
    }
    if (next_lanelets.empty()) {
      RCLCPP_WARN(rclcpp::get_logger("new_lanelet2_route_planning"), "No following lanelets found");
      break;
    }
    new_lanelet = next_lanelets.front();
    remaining_distance -= remaining_length;
    remaining_length = ll::geometry::length(new_lanelet.centerline2d());
  }

  return new_lanelet;
}

}  // namespace new_lanelet2_route_planning

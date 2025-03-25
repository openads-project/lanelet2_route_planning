#include <limits>

#include "new_lanelet2_route_planning/conversions.hpp"
#include "new_lanelet2_route_planning/utils.hpp"

namespace new_lanelet2_route_planning {

size_t indexOfLineStringPointClosestToPoint(const std::vector<Eigen::Vector2d>& line_string,
                                            const Eigen::Vector2d& point) {
  size_t idx_closest = 0;
  double min_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < line_string.size(); ++i) {
    double distance = (line_string[i] - point).norm();
    if (distance < min_distance) {
      min_distance = distance;
      idx_closest = i;
    }
  }
  return idx_closest;
}

route_planning_msgs::msg::RouteElement createMinimalRouteElement(const geometry_msgs::msg::Point& position,
                                                                 const geometry_msgs::msg::Quaternion& orientation,
                                                                 double s, bool will_change_suggested_lane,
                                                                 uint8_t speed_limit) {
  // create RouteElement
  route_planning_msgs::msg::RouteElement route_element_msg;
  // route_element_msg.left_boundary not set in global route
  // route_element_msg.right_boundary not set in global route
  // route_element_msg.regulatory_elements not set in global route
  route_element_msg.suggested_lane_idx = 0;
  route_element_msg.will_change_suggested_lane = will_change_suggested_lane;
  route_element_msg.s = s;

  // create LaneElement
  route_planning_msgs::msg::LaneElement lane_element_msg;
  lane_element_msg.reference_pose.position = position;
  lane_element_msg.reference_pose.orientation = orientation;
  // lane_element_msg.left_boundary not set in global route
  lane_element_msg.has_left_boundary = false;
  // lane_element_msg.right_boundary not set in global route
  lane_element_msg.has_right_boundary = false;
  lane_element_msg.speed_limit = speed_limit;
  // TODO: rename to regulatory_element_idcs?
  // TODO: has_regulatory_elements not needed?
  // lane_element_msg.regulatory_element_idx not set in global route
  lane_element_msg.following_lane_idx = 0;
  lane_element_msg.has_following_lane_idx = true;
  route_element_msg.lane_elements.push_back(lane_element_msg);

  return route_element_msg;
}

}  // namespace new_lanelet2_route_planning
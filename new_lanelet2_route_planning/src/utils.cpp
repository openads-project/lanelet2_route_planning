#include <limits>

#include <rclcpp/rclcpp.hpp>

#include "new_lanelet2_route_planning/conversions.hpp"
#include "new_lanelet2_route_planning/geometry.hpp"
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

bool changesLaneFromPointToPoint(const Eigen::Vector2d& point, const Eigen::Vector2d& next_point,
                                 const double sampling_distance) {
  const double epsilon = 1e-6;
  return ((next_point - point).norm() > (sampling_distance + epsilon));
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

std::vector<lanelet::ConstLanelet> adjacentLeftOrRightLanelets(const lanelet::ConstLanelet& lanelet,
                                                               const lanelet::routing::Route& route, bool left) {
  std::vector<lanelet::ConstLanelet> adjacent_lanelets;
  lanelet::routing::LaneletRelations relations = left ? route.leftRelations(lanelet) : route.rightRelations(lanelet);
  for (const auto& relation : relations) {
    if ((left && (relation.relationType == lanelet::routing::RelationType::Left ||
                  relation.relationType == lanelet::routing::RelationType::AdjacentLeft)) ||
        (!left && (relation.relationType == lanelet::routing::RelationType::Right ||
                   relation.relationType == lanelet::routing::RelationType::AdjacentRight))) {
      adjacent_lanelets.push_back(relation.lanelet);
    }
  }
  return adjacent_lanelets;
}

std::vector<ProjectedLaneletPoints> projectPointToLaneletLines(const Eigen::Vector2d& point,
                                                               const Eigen::Vector2d& prev_point,
                                                               const Eigen::Vector2d& next_point,
                                                               const std::vector<lanelet::ConstLanelet>& lanelets) {
  std::vector<ProjectedLaneletPoints> projected_points_per_lanelet;

  // loop over lanelets
  for (const auto& lanelet : lanelets) {
    ProjectedLaneletPoints projected_points;

    // project point to left bounds
    if (auto result = projectPointToLineStringAlongNormal(point, prev_point, next_point,
                                                          toEigen(lanelet.leftBound2d().basicLineString()))) {
      projected_points.left_bound_point = result->projected_point;
    } else {
      // TODO: how to handle logs here?
      RCLCPP_WARN(rclcpp::get_logger("new_lanelet2_route_planning"),
                  "Failed to project point (%.3f, %.3f) to left bounds of lanelet %ld", point.x(), point.y(),
                  lanelet.id());
    }

    // project point to centerline
    if (auto result = projectPointToLineStringAlongNormal(point, prev_point, next_point,
                                                          toEigen(lanelet.centerline2d().basicLineString()))) {
      projected_points.centerline_point = result->projected_point;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("new_lanelet2_route_planning"),
                  "Failed to project point (%.3f, %.3f) to centerline of lanelet %ld", point.x(), point.y(),
                  lanelet.id());
    }

    // project point to right bounds
    if (auto result = projectPointToLineStringAlongNormal(point, prev_point, next_point,
                                                          toEigen(lanelet.rightBound2d().basicLineString()))) {
      projected_points.right_bound_point = result->projected_point;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("new_lanelet2_route_planning"),
                  "Failed to project point (%.3f, %.3f) to right bounds of lanelet %ld", point.x(), point.y(),
                  lanelet.id());
    }

    projected_points_per_lanelet.push_back(projected_points);
  }

  return projected_points_per_lanelet;
}

}  // namespace new_lanelet2_route_planning
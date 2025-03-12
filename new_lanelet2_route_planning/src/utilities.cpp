#include <numeric>
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
               "No passable lanelet within %.3fm of given point (%.3f, %.3f)", k_max_distance_lanelet_matching, point.x(), point.y());
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

ll::BasicLineString2d resampleLineString(const ll::BasicLineString2d& line, const double delta_s, double& offset) {

  // TODO: use internal type for lines for easier processing? -> better understand what BasicLineString2d actually is, might save some lines here or there
  // copy to Eigen line
  std::vector<Eigen::Vector2d> line_eigen;
  for (const auto& point : line) {
    line_eigen.push_back(Eigen::Vector2d(point.x(), point.y()));
  }

  std::vector<Eigen::Vector2d> resampled_line_eigen;

  // set initial sampling distance
  double sampling_distance = delta_s;
  if (offset == 0.0) { // if no offset, start with first line point
    resampled_line_eigen.push_back(line_eigen.front());
  } else { // else sample first point with offset != delta_s
    sampling_distance = offset;
  }

  // loop over all line segments
  for (size_t i = 1; i < line_eigen.size(); ++i) {

    // determine segment length and unit direction
    double segment_length = (line_eigen[i] - line_eigen[i - 1]).norm();
    const Eigen::Vector2d segment_direction = (line_eigen[i] - line_eigen[i - 1]).normalized();

    // sample points along segment, increasing sampling_distance by delta_s
    while (segment_length >= sampling_distance) {
      const Eigen::Vector2d resampled_point = line_eigen[i - 1] + sampling_distance * segment_direction;
      resampled_line_eigen.push_back(resampled_point);
      sampling_distance += delta_s;
    }

    // reset sampling_distance for next segment, including overshoot
    sampling_distance = sampling_distance - segment_length;
  }

  // save overshoot in outgoing offset parameter
  offset = sampling_distance;

  // copy back to BasicLineString2d
  ll::BasicLineString2d resampled_line;
  for (const auto& point : resampled_line_eigen) {
    resampled_line.push_back(ll::BasicPoint2d(point.x(), point.y()));
  }

  return resampled_line;
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

ll::BasicLineString2d projectLinePointsToOtherLine(const ll::BasicLineString2d& line, const ll::BasicLineString2d& other_line) {

  ll::BasicLineString2d projected_line;
  for (size_t i = 0; i < line.size(); ++i) {

    const ll::BasicPoint2d point(line[i].x(), line[i].y());

    // if at end of line, just use ends of other line
    if (i == 0 || i == line.size() - 1) {
      const ll::BasicPoint2d other_point = (i == 0) ? ll::BasicPoint2d(other_line.front().x(), other_line.front().y())
                                                    : ll::BasicPoint2d(other_line.back().x(), other_line.back().y());
      projected_line.push_back(other_point);
      continue;
    }

    // find normal to tangent at point
    const Eigen::Vector2d current_point(line[i].x(), line[i].y());
    const Eigen::Vector2d prev_neighbor(line[i - 1].x(), line[i - 1].y());
    const Eigen::Vector2d next_neighbor(line[i + 1].x(), line[i + 1].y());
    const Eigen::Vector2d current_to_prev_unit = (prev_neighbor - current_point).normalized();
    const Eigen::Vector2d current_to_next_unit = (next_neighbor - current_point).normalized();
    const Eigen::Vector2d normal = (current_to_prev_unit + current_to_next_unit).normalized();

    // project current point to other line along normal to tangent
    std::vector<Eigen::Vector2d> other_line_eigen;
    for (const auto& other_point : other_line) {
      other_line_eigen.push_back(Eigen::Vector2d(other_point.x(), other_point.y()));
    }
    bool found_intersection_with_line_segment;
    const Eigen::Vector2d projected_to_other = projectPointToLineAlongAxis(current_point, normal, other_line_eigen, found_intersection_with_line_segment);
    if (found_intersection_with_line_segment) {
      projected_line.push_back(ll::BasicPoint2d(projected_to_other[0], projected_to_other[1]));
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("new_lanelet2_route_planning"), "Failed to project point to other line");
    }
  }

  return projected_line;
}

Eigen::Vector2d projectPointToLineAlongAxis(const Eigen::Vector2d& point, const Eigen::Vector2d& axis,
                                             const std::vector<Eigen::Vector2d>& line, bool& found_intersection_with_line_segment) {

  // TODO: use Eigen to compute intersection?
  // https://stackoverflow.com/a/50763846

  Eigen::Vector2d projected_point;
  found_intersection_with_line_segment = false;

  // define straight line (a1 x + b1 y = c1) along axis at point
  const double a1 = axis[1];
  const double b1 = -axis[0];
  const double c1 = a1 * point[0] + b1 * point[1];

  // loop over line segments
  for (size_t i = 0; i < line.size() - 1; ++i) {

    const auto& line_point1 = line[i];
    const auto& line_point2 = line[i + 1];

    // define straight line (a2 x + b2 y = c2) through line segment points
    const double a2 = line_point2[1] - line_point1[1];
    const double b2 = line_point1[0] - line_point2[0];
    const double c2 = line_point1[0] * line_point2[1] - line_point2[0] * line_point1[1];

    // find intersection of both lines by solving for x and y
    const double det = a1 * b2 - a2 * b1;
    if (det != 0) {
      const double x = (b2 * c1 - b1 * c2) / det;
      const double y = (a1 * c2 - a2 * c1) / det;

      // check if intersection point is within line segment
      if (x >= std::min(line_point1[0], line_point2[0]) && x <= std::max(line_point1[0], line_point2[0]) &&
          y >= std::min(line_point1[1], line_point2[1]) && y <= std::max(line_point1[1], line_point2[1])) {
        projected_point = Eigen::Vector2d(x, y);
        found_intersection_with_line_segment = true;
        break;
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("new_lanelet2_route_planning"), "Cannot find intersection of parallel lines");
      return projected_point;
    }
  }

  return projected_point;
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

route_planning_msgs::msg::Route laneletToRosRoute(const ll::routing::Route& route, const std::string& frame_id) {
  route_planning_msgs::msg::Route route_msg;
  // route_msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now(); // TODO: not correctly working with simtime
  route_msg.header.frame_id = frame_id;
  route_msg.destination = geometry_msgs::msg::Point();  // TODO
  route_msg.traveled_route = {};                        // TODO
  // route_msg.current_speed_limit = 0;                    // TODO: move to RouteElement or LaneElement?

  // get shortest path
  ll::routing::LaneletPath shortest_path = route.shortestPath();

  // loop over lanelets along shortest path to extract RouteElements
  double resampling_offset = 0.0;
  for (const auto& lanelet : shortest_path) {
    std::vector<route_planning_msgs::msg::RouteElement> route_element_msgs = laneletToRosRouteElements(lanelet, route, resampling_offset);
    route_msg.remaining_route.insert(route_msg.remaining_route.end(), route_element_msgs.begin(),
                                      route_element_msgs.end());
  }

  return route_msg;
}

std::vector<route_planning_msgs::msg::RouteElement> laneletToRosRouteElements(
    const ll::ConstLanelet& shortest_path_lanelet, const ll::routing::Route& route, double& resampling_offset) {
  std::vector<route_planning_msgs::msg::RouteElement> route_element_msgs;

  // TODO: refactor this function?

  // get all adjacent lanelets of route cross section along shortest path lanelet
  std::vector<ll::ConstLanelet> adjacent_left_lanelets, adjacent_right_lanelets;
  ll::routing::LaneletRelations left_relations = route.leftRelations(shortest_path_lanelet);
  ll::routing::LaneletRelations right_relations = route.rightRelations(shortest_path_lanelet);
  for (const auto& left_relation : left_relations) {
    if (left_relation.relationType == ll::routing::RelationType::Left) {
      adjacent_left_lanelets.push_back(left_relation.lanelet);
    }
  }
  for (const auto& right_relation : right_relations) {
    if (right_relation.relationType == ll::routing::RelationType::Right) {
      adjacent_right_lanelets.push_back(right_relation.lanelet);
    }
  }

  // walk along centerline of shortest path lanelet
  ll::BasicLineString2d centerline = shortest_path_lanelet.centerline2d().basicLineString();

  // resample centerline
  const double delta_s = 1.0; // TODO: move elsewhere
  ll::BasicLineString2d resampled_centerline = resampleLineString(centerline, delta_s, resampling_offset);
  if (resampled_centerline.empty()) {
    return route_element_msgs;
  }

  // project centerline points to bounds to ensure same number of points
  // ll::BasicLineString2d projected_left_bound = projectLinePointsToOtherLine(resampled_centerline, shortest_path_lanelet.leftBound2d().basicLineString());
  // ll::BasicLineString2d projected_right_bound = projectLinePointsToOtherLine(resampled_centerline, shortest_path_lanelet.rightBound2d().basicLineString());
  // if (projected_left_bound.size() != resampled_centerline.size() || projected_right_bound.size() != resampled_centerline.size()) {
  //   RCLCPP_ERROR(rclcpp::get_logger("new_lanelet2_route_planning"), "Failed to project centerline to bounds");
  //   return route_element_msgs;
  // }

  // project centerline points to bounds of adjacent lanelets
  // std::vector<ll::BasicLineString2d> adjacent_left_lanelets_left_bounds, adjacent_left_lanelets_right_bounds;
  // std::vector<ll::BasicLineString2d> adjacent_right_lanelets_left_bounds, adjacent_right_lanelets_right_bounds;
  // for (const auto& adjacent_left_lanelet : adjacent_left_lanelets) {
  //   ll::BasicLineString2d adjacent_left_bound = projectLinePointsToOtherLine(resampled_centerline, adjacent_left_lanelet.leftBound2d().basicLineString());
  //   ll::BasicLineString2d adjacent_right_bound = projectLinePointsToOtherLine(resampled_centerline, adjacent_left_lanelet.rightBound2d().basicLineString());
  //   if (adjacent_left_bound.size() != resampled_centerline.size() || adjacent_right_bound.size() != resampled_centerline.size()) {
  //     RCLCPP_ERROR(rclcpp::get_logger("new_lanelet2_route_planning"), "Failed to project centerline to bounds");
  //     return route_element_msgs;
  //   }
  //   adjacent_left_lanelets_left_bounds.push_back(adjacent_left_bound);
  //   adjacent_left_lanelets_right_bounds.push_back(adjacent_right_bound);
  // }
  // for (const auto& adjacent_right_lanelet : adjacent_right_lanelets) {
  //   ll::BasicLineString2d adjacent_left_bound = projectLinePointsToOtherLine(resampled_centerline, adjacent_right_lanelet.leftBound2d().basicLineString());
  //   ll::BasicLineString2d adjacent_right_bound = projectLinePointsToOtherLine(resampled_centerline, adjacent_right_lanelet.rightBound2d().basicLineString());
  //   if (adjacent_left_bound.size() != resampled_centerline.size() || adjacent_right_bound.size() != resampled_centerline.size()) {
  //     RCLCPP_ERROR(rclcpp::get_logger("new_lanelet2_route_planning"), "Failed to project centerline to bounds");
  //     return route_element_msgs;
  //   }
  //   adjacent_right_lanelets_left_bounds.push_back(adjacent_left_bound);
  //   adjacent_right_lanelets_right_bounds.push_back(adjacent_right_bound);
  // }

  // walk along centerline to extract route elements
  for (size_t i = 0; i < resampled_centerline.size(); ++i) {

    route_planning_msgs::msg::RouteElement route_element_msg;
    route_element_msg.domain_id = 0;                                       // TODO
    route_element_msg.current_lane_id = 0;                                 // TODO
    route_element_msg.current_s = 0;                                       // TODO: remove from msg?
    route_element_msg.lane_change = false;                                 // TODO
    // route_element_msg.drivable_space_left = laneletToRosPoint(left_bound[i]);    // TODO
    // route_element_msg.drivable_space_right = laneletToRosPoint(right_bound[i]);  // TODO

    // left lanes
    for (size_t j = 0; j < adjacent_left_lanelets.size(); ++j) {

      route_planning_msgs::msg::LaneElement lane_element_msg;
      lane_element_msg.reference_pose.position = laneletToRosPoint(resampled_centerline[i]); // TODO: use projected point!
      lane_element_msg.reference_pose.orientation = geometry_msgs::msg::Quaternion();  // TODO
      // lane_element_msg.lane_boundary_left = laneletToRosPoint(adjacent_left_lanelets_left_bounds[j][i]);
      // lane_element_msg.lane_boundary_right = laneletToRosPoint(adjacent_left_lanelets_right_bounds[j][i]);
      lane_element_msg.lane_separator_type_left = 0;   // TODO
      lane_element_msg.lane_separator_type_right = 0;  // TODO
      lane_element_msg.regulatory_elements = {};       // TODO
      route_element_msg.lane_elements.push_back(lane_element_msg);
    }

    // current lane
    route_planning_msgs::msg::LaneElement current_lane_element_msg;
    current_lane_element_msg.reference_pose.position = laneletToRosPoint(resampled_centerline[i]);
    current_lane_element_msg.reference_pose.orientation = geometry_msgs::msg::Quaternion();  // TODO
    // current_lane_element_msg.lane_boundary_left = laneletToRosPoint(projected_left_bound[i]);
    // current_lane_element_msg.lane_boundary_right = laneletToRosPoint(projected_right_bound[i]);
    current_lane_element_msg.lane_separator_type_left = 0;   // TODO
    current_lane_element_msg.lane_separator_type_right = 0;  // TODO
    current_lane_element_msg.regulatory_elements = {};       // TODO
    route_element_msg.lane_elements.push_back(current_lane_element_msg);
    route_element_msg.current_lane_id = adjacent_left_lanelets.size();

    // right lanes
    for (size_t j = 0; j < adjacent_right_lanelets.size(); ++j) {

      route_planning_msgs::msg::LaneElement lane_element_msg;
      lane_element_msg.reference_pose.position = laneletToRosPoint(resampled_centerline[i]); // TODO: use projected point!
      lane_element_msg.reference_pose.orientation = geometry_msgs::msg::Quaternion();  // TODO
      // lane_element_msg.lane_boundary_left = laneletToRosPoint(adjacent_right_lanelets_left_bounds[j][i]);
      // lane_element_msg.lane_boundary_right = laneletToRosPoint(adjacent_right_lanelets_right_bounds[j][i]);
      lane_element_msg.lane_separator_type_left = 0;   // TODO
      lane_element_msg.lane_separator_type_right = 0;  // TODO
      lane_element_msg.regulatory_elements = {};       // TODO
      route_element_msg.lane_elements.push_back(lane_element_msg);
    }

    route_element_msgs.push_back(route_element_msg);
  }

  return route_element_msgs;
}

}  // namespace new_lanelet2_route_planning

#include <cmath>
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

Eigen::Vector2d tangentOfPointAlongLineString(const Eigen::Vector2d& point, const Eigen::Vector2d& prev_point, const Eigen::Vector2d& next_point) {

  Eigen::Vector2d tangent;

  if (point == prev_point && point != next_point) { // single line segment
    tangent = (next_point - point).normalized();
  } else if (point != prev_point && point == next_point) { // single line segment
    tangent = (point - prev_point).normalized();
  } else if (point == prev_point && point == next_point) { // single point
    RCLCPP_WARN(rclcpp::get_logger("new_lanelet2_route_planning"), "Tangent of single point is undefined");
    tangent = Eigen::Vector2d(0.0, 0.0);
  } else { // proper two line segments with previous and next point
    const Eigen::Vector2d prev_to_point_unit = (point - prev_point).normalized();
    const Eigen::Vector2d point_to_next_unit = (next_point - point).normalized();
    tangent = (prev_to_point_unit + point_to_next_unit).normalized();
  }

  return tangent;
}

Eigen::Vector2d normalOfPointAlongLineString(const Eigen::Vector2d& point, const Eigen::Vector2d& prev_point, const Eigen::Vector2d& next_point) {

  Eigen::Vector2d tangent = tangentOfPointAlongLineString(point, prev_point, next_point);
  Eigen::Vector2d normal = Eigen::Vector2d(tangent.y(), -tangent.x());

  return normal;
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

Eigen::Vector2d projectPointToLineStringAlongNormal(const Eigen::Vector2d& point, const Eigen::Vector2d& prev_point, const Eigen::Vector2d& next_point, const ll::BasicLineString2d& line) {

  // find normal to tangent at point
  Eigen::Vector2d normal = normalOfPointAlongLineString(point, prev_point, next_point);

  // project current point to other line along normal to tangent
  bool found_intersection_with_line_segment;
  const Eigen::Vector2d projected_point = projectPointToLineAlongAxis(point, normal, line, found_intersection_with_line_segment);

  return projected_point;
}

Eigen::Vector2d projectPointToLineAlongAxis(const Eigen::Vector2d& point, const Eigen::Vector2d& axis,
                                             const ll::BasicLineString2d& line, bool& found_intersection_with_line_segment) {

  // TODO: use Eigen to compute intersection?
  // https://stackoverflow.com/a/50763846

  Eigen::Vector2d closest_projected_point;
  found_intersection_with_line_segment = false;
  double closest_distance_to_line_segment = std::numeric_limits<double>::max();
  bool found_at_least_one_intersection = false;

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
      Eigen::Vector2d projected_point = Eigen::Vector2d(x, y);
      found_at_least_one_intersection = true;

      // check if intersection point is within line segment
      if (x >= std::min(line_point1[0], line_point2[0]) && x <= std::max(line_point1[0], line_point2[0]) &&
          y >= std::min(line_point1[1], line_point2[1]) && y <= std::max(line_point1[1], line_point2[1])) {
        found_intersection_with_line_segment = true;
        closest_projected_point = projected_point;
        break;
      } else { // else still save as best projected point if closest to line segment
        double distance_to_line_segment = std::min((projected_point - line_point1).norm(), (projected_point - line_point2).norm());
        if (distance_to_line_segment < closest_distance_to_line_segment) {
          closest_distance_to_line_segment = distance_to_line_segment;
          closest_projected_point = projected_point;
        }
      }
    }
  }

  if (!found_at_least_one_intersection) {
    RCLCPP_ERROR(rclcpp::get_logger("new_lanelet2_route_planning"), "Could not project point to line along axis because axis is parallel to all line segments");
  }

  return closest_projected_point;
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

ll::BasicLineString2d resampleCenterlinesAlongPath(const ll::routing::LaneletPath& path, const double delta_s, bool monotonically, std::vector<size_t>& lanelet_idx_by_point) {

  // init variables
  ll::BasicLineString2d resampled_path_centerline;
  double resampling_offset = 0.0;
  lanelet_idx_by_point.clear();
  Eigen::Vector2d prev_sampled_point = Eigen::Vector2d(0.0, 0.0); // TODO: dangerous to init to 0?
  Eigen::Vector2d prev_sampled_point_orientation = Eigen::Vector2d(0.0, 0.0);

  // loop over lanelets in path
  for (size_t l = 0; l < path.size(); ++l) {

    // get centerline
    const ll::ConstLanelet& lanelet = path[l];
    ll::BasicLineString2d centerline = lanelet.centerline2d().basicLineString();

    // skip point if behind previous sampled point, e.g., if on adjacent lanelet in shortest path due to lane change
    if (monotonically && l > 0) {
      for (auto cit = centerline.begin(); cit != centerline.end();) {
        auto& centerline_point = *cit;
        if ((centerline_point - prev_sampled_point).dot(prev_sampled_point_orientation) < 0) { // angle > 90deg
          cit = centerline.erase(cit);
        } else {
          break;
        }
      }
    }

    // resample lanelet centerline
    ll::BasicLineString2d resampled_centerline = resampleLineString(centerline, delta_s, resampling_offset);
    resampled_path_centerline.insert(resampled_path_centerline.end(), resampled_centerline.begin(), resampled_centerline.end());
    lanelet_idx_by_point.insert(lanelet_idx_by_point.end(), resampled_centerline.size(), l);

    // update information for monotonicity check
    if (monotonically && !resampled_centerline.empty()) {
      if (resampled_centerline.size() > 1) {
        prev_sampled_point = resampled_centerline[resampled_centerline.size() - 2];
      }
      prev_sampled_point_orientation = tangentOfPointAlongLineString(resampled_centerline.back(), prev_sampled_point, resampled_centerline.back());
      prev_sampled_point = resampled_centerline.back();
    }
  }

  return resampled_path_centerline;
}

std::vector<ll::ConstLanelet> adjacentLeftOrRightLanelets(const ll::ConstLanelet& lanelet, const ll::routing::Route& route, bool left) {
  std::vector<ll::ConstLanelet> adjacent_lanelets;
  ll::routing::LaneletRelations relations = left ? route.leftRelations(lanelet) : route.rightRelations(lanelet);
  for (const auto& relation : relations) {
    if ((left && (relation.relationType == ll::routing::RelationType::Left || relation.relationType == ll::routing::RelationType::AdjacentLeft)) ||
        (!left && (relation.relationType == ll::routing::RelationType::Right || relation.relationType == ll::routing::RelationType::AdjacentRight))) {
      adjacent_lanelets.push_back(relation.lanelet);
    }
  }
  return adjacent_lanelets;
}

int computeFollowingLaneIdxOffset(const ll::ConstLanelet& lanelet, const ll::ConstLanelet& lanelet_of_next_point, const ll::routing::Route& route, const ll::routing::RoutingGraphUPtr& routing_graph) {

  int following_lane_idx_offset = 0;
  size_t n_adjacent_lanelets_of_next_lanelet;
  if (lanelet_of_next_point.id() != lanelet.id()) {

    // get adjacent lanelets of current lanelet
    std::vector<ll::ConstLanelet> adjacent_left_lanelets = adjacentLeftOrRightLanelets(lanelet, route, true);
    std::vector<ll::ConstLanelet> adjacent_right_lanelets = adjacentLeftOrRightLanelets(lanelet, route, false);
    int suggested_lane_idx = adjacent_left_lanelets.size();

    // get adjacent lanelets of next lanelet (lanelet of next point)
    std::vector<ll::ConstLanelet> adjacent_left_lanelets_of_next_lanelet = adjacentLeftOrRightLanelets(lanelet_of_next_point, route, true);
    std::vector<ll::ConstLanelet> adjacent_right_lanelets_of_next_lanelet = adjacentLeftOrRightLanelets(lanelet_of_next_point, route, false);
    std::vector<ll::ConstLanelet> adjacent_lanelets_of_next_lanelet = adjacent_left_lanelets_of_next_lanelet;
    adjacent_lanelets_of_next_lanelet.push_back(lanelet_of_next_point);
    adjacent_lanelets_of_next_lanelet.insert(adjacent_lanelets_of_next_lanelet.end(), adjacent_right_lanelets_of_next_lanelet.begin(), adjacent_right_lanelets_of_next_lanelet.end());
    n_adjacent_lanelets_of_next_lanelet = adjacent_lanelets_of_next_lanelet.size();

    // find following lanelet of current lanelet and adjacent lanelets in adjacent lanelets of next lanelet
    std::vector<std::vector<ll::ConstLanelet>> lanelet_groups = { {lanelet}, adjacent_left_lanelets, adjacent_right_lanelets };
    std::vector<int> lanelet_group_offset_factors = { 0, 1, -1 };
    following_lane_idx_offset = std::numeric_limits<int>::max();

    // first try to match following lanelet of current lanelet, then of adjacent left lanelets, then of adjacent right lanelets
    for (size_t group_idx = 0; group_idx < lanelet_groups.size(); ++group_idx) {
      const auto& lanelet_group = lanelet_groups[group_idx];
      int group_offset_factor = lanelet_group_offset_factors[group_idx];

      // loop over all lanelets in group
      for (size_t a = 0; a < lanelet_group.size(); ++a) {
        auto following_lanelets = routing_graph->following(lanelet_group[a], false);
        auto following_lanelet = following_lanelets.front();
        size_t following_lanelet_idx = 0;
        size_t follow_further_idx = 0; // counter for following lanelets more than once (e.g., if sampling skipped short lanelets)
        size_t max_follow_further_iterations = 3; // maximum number of following lanelets to check

        // loop until following lanelet is found in adjacent lanelets of next lanelet
        while (following_lane_idx_offset == std::numeric_limits<int>::max()) {
          // check all adjacent lanelets of next lanelet
          for (size_t l = 0; l < adjacent_lanelets_of_next_lanelet.size(); ++l) {
            if (following_lanelet.id() == adjacent_lanelets_of_next_lanelet[l].id()) {
              following_lane_idx_offset = l - suggested_lane_idx + group_offset_factor * (a + 1); // gottesformel
              break;
            }
          }

          // check abort conditions
          follow_further_idx++;
          if (follow_further_idx >= max_follow_further_iterations) {
            break;
          }

          // get next following lanelet
          following_lanelet_idx++;
          if (following_lanelet_idx < following_lanelets.size()) {
            following_lanelet = following_lanelets[following_lanelet_idx];
          } else {
            following_lanelets = routing_graph->following(following_lanelet, false);
            following_lanelet = following_lanelets.front();
            following_lanelet_idx = 0;
          }
        }

        if (following_lane_idx_offset != std::numeric_limits<int>::max()) {
          break;
        }
      }

      if (following_lane_idx_offset != std::numeric_limits<int>::max()) {
        break;
      }
    }

    if (following_lane_idx_offset == std::numeric_limits<int>::max()) {
      RCLCPP_ERROR(rclcpp::get_logger("new_lanelet2_route_planning"), "Could not match following lanelets to adjacent lanelets of lanelet of next point");
    }
  }

  return following_lane_idx_offset;
}

// TODO: move more functions to class for easy member access
route_planning_msgs::msg::Route laneletToRosRoute(const ll::routing::Route& route, const std::string& frame_id, const ll::routing::RoutingGraphUPtr& routing_graph) {

  // create Route
  route_planning_msgs::msg::Route route_msg;
  // route_msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now(); // TODO: not correctly working with simtime
  route_msg.header.frame_id = frame_id;
  route_msg.destination = geometry_msgs::msg::Point();  // TODO
  route_msg.traveled_route_elements = {};               // TODO

  // get shortest path
  ll::routing::LaneletPath shortest_path = route.shortestPath();

  // resample centerlines along shortest path to accumulate global centerline
  const double delta_s = 1.0; // TODO: move elsewhere
  bool monotonically = true;
  std::vector<size_t> lanelet_idx_by_point;
  ll::BasicLineString2d shortest_path_centerline = resampleCenterlinesAlongPath(shortest_path, delta_s, monotonically, lanelet_idx_by_point);

  // loop over global centerline
  for (size_t c = 0; c < shortest_path_centerline.size(); ++c) {

    // get current, previous and next centerline point
    const Eigen::Vector2d& point = shortest_path_centerline[c];
    const Eigen::Vector2d& prev_point = (c > 0) ? shortest_path_centerline[c - 1] : point;
    const Eigen::Vector2d& next_point = (c < shortest_path_centerline.size() - 1) ? shortest_path_centerline[c + 1] : point;

    // identify lane changes based on break in equidistant centerline
    bool changes_lane_from_prev_point = ((point - prev_point).norm() > delta_s + 0.001); // TODO: better handle epsilon for floating point comparison
    bool changes_lane_to_next_point = ((next_point - point).norm() > delta_s + 0.001);

    // determine neighboring points for projection
    Eigen::Vector2d prev_point_for_projection = changes_lane_from_prev_point ? point : prev_point;
    Eigen::Vector2d next_point_for_projection = changes_lane_to_next_point ? point : next_point;

    // compute orientation of centerline point
    Eigen::Vector2d orientation = tangentOfPointAlongLineString(point, prev_point_for_projection, next_point_for_projection);

    // get lanelet corresponding to centerline point
    const ll::ConstLanelet& lanelet = shortest_path[lanelet_idx_by_point[c]];

    // get adjacent lanelets
    // TODO: this is re-executed for every point on the same lanelet
    std::vector<ll::ConstLanelet> adjacent_left_lanelets = adjacentLeftOrRightLanelets(lanelet, route, true);
    std::vector<ll::ConstLanelet> adjacent_right_lanelets = adjacentLeftOrRightLanelets(lanelet, route, false);
    int suggested_lane_idx = adjacent_left_lanelets.size();

    // project centerline point to lanelet bounds
    ll::BasicPoint2d left_bounds_point = projectPointToLineStringAlongNormal(point, prev_point_for_projection, next_point_for_projection, lanelet.leftBound2d().basicLineString());
    ll::BasicPoint2d right_bounds_point = projectPointToLineStringAlongNormal(point, prev_point_for_projection, next_point_for_projection, lanelet.rightBound2d().basicLineString());

    // project centerline point to adjacent lanelet centerlines and bounds
    std::vector<ll::BasicPoint2d> adjacent_left_lanelets_centerline_points, adjacent_left_lanelets_left_bounds_points, adjacent_left_lanelets_right_bounds_points;
    std::vector<ll::BasicPoint2d> adjacent_right_lanelets_centerline_points, adjacent_right_lanelets_left_bounds_points, adjacent_right_lanelets_right_bounds_points;
    for (const auto& adjacent_lanelet : adjacent_left_lanelets) {
      ll::BasicPoint2d adjacent_lanelet_centerline_point = projectPointToLineStringAlongNormal(point, prev_point_for_projection, next_point_for_projection, adjacent_lanelet.centerline2d().basicLineString());
      ll::BasicPoint2d adjacent_lanelet_left_bounds_point = projectPointToLineStringAlongNormal(point, prev_point_for_projection, next_point_for_projection, adjacent_lanelet.leftBound2d().basicLineString());
      ll::BasicPoint2d adjacent_lanelet_right_bounds_point = projectPointToLineStringAlongNormal(point, prev_point_for_projection, next_point_for_projection, adjacent_lanelet.rightBound2d().basicLineString());
      adjacent_left_lanelets_centerline_points.push_back(adjacent_lanelet_centerline_point);
      adjacent_left_lanelets_left_bounds_points.push_back(adjacent_lanelet_left_bounds_point);
      adjacent_left_lanelets_right_bounds_points.push_back(adjacent_lanelet_right_bounds_point);
    }
    for (const auto& adjacent_lanelet : adjacent_right_lanelets) {
      ll::BasicPoint2d adjacent_lanelet_centerline_point = projectPointToLineStringAlongNormal(point, prev_point_for_projection, next_point_for_projection, adjacent_lanelet.centerline2d().basicLineString());
      ll::BasicPoint2d adjacent_lanelet_left_bounds_point = projectPointToLineStringAlongNormal(point, prev_point_for_projection, next_point_for_projection, adjacent_lanelet.leftBound2d().basicLineString());
      ll::BasicPoint2d adjacent_lanelet_right_bounds_point = projectPointToLineStringAlongNormal(point, prev_point_for_projection, next_point_for_projection, adjacent_lanelet.rightBound2d().basicLineString());
      adjacent_right_lanelets_centerline_points.push_back(adjacent_lanelet_centerline_point);
      adjacent_right_lanelets_left_bounds_points.push_back(adjacent_lanelet_left_bounds_point);
      adjacent_right_lanelets_right_bounds_points.push_back(adjacent_lanelet_right_bounds_point);
    }

    // compute offset of lane element indices from current to next route element
    const ll::ConstLanelet& lanelet_of_next_point = (c < shortest_path_centerline.size() - 1) ? shortest_path[lanelet_idx_by_point[c + 1]] : lanelet;
    int following_lane_idx_offset = computeFollowingLaneIdxOffset(lanelet, lanelet_of_next_point, route, routing_graph);

    // create RouteElement
    route_planning_msgs::msg::RouteElement route_element_msg;
    route_element_msg.suggested_lane_idx = suggested_lane_idx;
    route_element_msg.will_change_suggested_lane = changes_lane_to_next_point;
    // route_element_msg.left_boundary = 0;                                // TODO
    // route_element_msg.right_boundary = 0;                               // TODO
    route_element_msg.s = 0;                                               // TODO

    // create LaneElements for left adjacent lanes
    for (size_t a = 0; a < adjacent_left_lanelets.size(); ++a) {
      route_planning_msgs::msg::LaneElement lane_element_msg;
      lane_element_msg.reference_pose.position = laneletToRosPoint(adjacent_left_lanelets_centerline_points[a]);
      lane_element_msg.reference_pose.orientation = geometry_msgs::msg::Quaternion();  // TODO
      lane_element_msg.left_boundary.point = laneletToRosPoint(adjacent_left_lanelets_left_bounds_points[a]);
      lane_element_msg.left_boundary.type = route_planning_msgs::msg::LaneBoundary::UNKNOWN;   // TODO
      lane_element_msg.has_left_boundary = true;
      lane_element_msg.right_boundary.point = laneletToRosPoint(adjacent_left_lanelets_right_bounds_points[a]);
      lane_element_msg.right_boundary.type = route_planning_msgs::msg::LaneBoundary::UNKNOWN;   // TODO
      lane_element_msg.has_right_boundary = true;
      lane_element_msg.speed_limit = 0;          // TODO
      lane_element_msg.regulatory_elements = {};       // TODO
      lane_element_msg.following_lane_idx = route_element_msg.lane_elements.size() + following_lane_idx_offset;
      lane_element_msg.has_following_lane_idx = true;
      route_element_msg.lane_elements.push_back(lane_element_msg);
    }

    // create LaneElement for centerline lane
    route_planning_msgs::msg::LaneElement centerline_lane_element_msg;
    centerline_lane_element_msg.reference_pose.position = laneletToRosPoint(point);
    centerline_lane_element_msg.reference_pose.orientation = vectorToRosQuaternion(orientation);
    centerline_lane_element_msg.left_boundary.point = laneletToRosPoint(left_bounds_point);
    centerline_lane_element_msg.left_boundary.type = route_planning_msgs::msg::LaneBoundary::UNKNOWN;   // TODO
    centerline_lane_element_msg.has_left_boundary = true;
    centerline_lane_element_msg.right_boundary.point = laneletToRosPoint(right_bounds_point);
    centerline_lane_element_msg.right_boundary.type = route_planning_msgs::msg::LaneBoundary::UNKNOWN;   // TODO
    centerline_lane_element_msg.has_right_boundary = true;
    centerline_lane_element_msg.speed_limit = 0;          // TODO
    centerline_lane_element_msg.regulatory_elements = {};       // TODO
    centerline_lane_element_msg.following_lane_idx = route_element_msg.lane_elements.size() + following_lane_idx_offset;
    centerline_lane_element_msg.has_following_lane_idx = true;
    route_element_msg.lane_elements.push_back(centerline_lane_element_msg);

    // create LaneElements for right adjacent lanes
    for (size_t a = 0; a < adjacent_right_lanelets.size(); ++a) {
      route_planning_msgs::msg::LaneElement lane_element_msg;
      lane_element_msg.reference_pose.position = laneletToRosPoint(adjacent_right_lanelets_centerline_points[a]);
      lane_element_msg.reference_pose.orientation = geometry_msgs::msg::Quaternion();  // TODO
      lane_element_msg.left_boundary.point = laneletToRosPoint(adjacent_right_lanelets_left_bounds_points[a]);
      lane_element_msg.left_boundary.type = route_planning_msgs::msg::LaneBoundary::UNKNOWN;   // TODO
      lane_element_msg.has_left_boundary = true;
      lane_element_msg.right_boundary.point = laneletToRosPoint(adjacent_right_lanelets_right_bounds_points[a]);
      lane_element_msg.right_boundary.type = route_planning_msgs::msg::LaneBoundary::UNKNOWN;   // TODO
      lane_element_msg.has_right_boundary = true;
      lane_element_msg.speed_limit = 0;          // TODO
      lane_element_msg.regulatory_elements = {};       // TODO
      lane_element_msg.following_lane_idx = route_element_msg.lane_elements.size() + following_lane_idx_offset;
      lane_element_msg.has_following_lane_idx = true;
      route_element_msg.lane_elements.push_back(lane_element_msg);
    }

    route_msg.remaining_route_elements.push_back(route_element_msg);
  }

  return route_msg;
}

geometry_msgs::msg::Quaternion vectorToRosQuaternion(const Eigen::Vector2d& vector) {
  Eigen::Vector2d unit_vector = vector.normalized();
  double angle = std::atan2(unit_vector.y(), unit_vector.x());
  Eigen::Quaterniond quaternion(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
  geometry_msgs::msg::Quaternion ros_quaternion;
  ros_quaternion.x = quaternion.x();
  ros_quaternion.y = quaternion.y();
  ros_quaternion.z = quaternion.z();
  ros_quaternion.w = quaternion.w();
  return ros_quaternion;
}

}  // namespace new_lanelet2_route_planning

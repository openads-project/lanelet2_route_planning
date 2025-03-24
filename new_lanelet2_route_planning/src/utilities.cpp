#include <cmath>
#include <numeric>
#include <tuple>

#include <lanelet2_routing/Route.h>
#include <lanelet2_utilities/lanelet2_utils.hpp>
#include <perception_msgs_utils/object_access.hpp>
#include <rclcpp/rclcpp.hpp>

#include "new_lanelet2_route_planning/geometry.hpp"
#include "new_lanelet2_route_planning/utilities.hpp"

namespace new_lanelet2_route_planning {

static const unsigned int k_nearest_lanelets = 5;
static const double k_timeout_ego_data = 1.0;
static const double k_max_distance_lanelet_matching = 5.0;

std::vector<Eigen::Vector2d> lineStringAsEigen(const ll::BasicLineString2d& line_string) {
  return std::vector<Eigen::Vector2d>(line_string.begin(), line_string.end());
}

Eigen::Vector2d rosToLaneletPoint(const geometry_msgs::msg::Point& point) { return Eigen::Vector2d(point.x, point.y); }

Eigen::Vector2d rosToLaneletPoint(const perception_msgs::msg::EgoData& ego_data) {
  return Eigen::Vector2d(perception_msgs::object_access::getX(ego_data),
                         perception_msgs::object_access::getY(ego_data));
}

geometry_msgs::msg::Point laneletToRosPoint2d(const Eigen::Vector2d& point) {
  geometry_msgs::msg::Point ros_point;
  ros_point.x = point.x();
  ros_point.y = point.y();
  ros_point.z = 0.0;
  return ros_point;
}

geometry_msgs::msg::Point laneletToRosPoint(const Eigen::Vector3d& point) {
  geometry_msgs::msg::Point ros_point;
  ros_point.x = point.x();
  ros_point.y = point.y();
  ros_point.z = point.z();
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

bool findLaneletAtPoint(const ll::LaneletMapConstPtr& map, const Eigen::Vector2d& point, ll::ConstLanelet& lanelet,
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
               "No passable lanelet within %.3fm of given point (%.3f, %.3f)", k_max_distance_lanelet_matching,
               point.x(), point.y());
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

Eigen::Vector2d projectPointToCenterline(const Eigen::Vector2d& point, const ll::ConstLanelet& lanelet) {
  Eigen::Vector3d point_3d = Eigen::Vector3d(point.x(), point.y(), 0.0);
  Eigen::Vector3d projected_point_3d = ll::geometry::project(lanelet.centerline(), point_3d);
  return Eigen::Vector2d(projected_point_3d.x(), projected_point_3d.y());
}

Eigen::Vector2d projectPointToCenterline(const geometry_msgs::msg::Point& point, const ll::ConstLanelet& lanelet) {
  return projectPointToCenterline(rosToLaneletPoint(point), lanelet);
}

Eigen::Vector2d projectPointToCenterline(const perception_msgs::msg::EgoData& ego_data,
                                         const ll::ConstLanelet& lanelet) {
  return projectPointToCenterline(rosToLaneletPoint(ego_data), lanelet);
}

Eigen::Vector2d projectPointToLineStringAlongNormal(const Eigen::Vector2d& point, const Eigen::Vector2d& prev_point,
                                                    const Eigen::Vector2d& next_point,
                                                    const ll::BasicLineString2d& line) {
  // find normal to tangent at point
  Eigen::Vector2d normal = normalOfPointAlongLineString(point, prev_point, next_point);

  // project current point to other line along normal to tangent
  bool found_intersection_with_line_segment;
  const Eigen::Vector2d projected_point =
      projectPointToLineAlongAxis(point, normal, line, found_intersection_with_line_segment);

  return projected_point;
}

Eigen::Vector2d projectPointToLineAlongAxis(const Eigen::Vector2d& point, const Eigen::Vector2d& axis,
                      const ll::BasicLineString2d& line,
                      bool& found_intersection_with_line_segment) {

  Eigen::Vector2d closest_projected_point;
  found_intersection_with_line_segment = false;
  double closest_distance_to_line_segment = std::numeric_limits<double>::max();
  bool found_at_least_one_intersection = false;

  // define straight line along axis at point
  std::vector<Eigen::Vector2d> axis_line = {point, point + axis};

  // loop over line segments
  for (size_t i = 0; i < line.size() - 1; ++i) {
    std::vector<Eigen::Vector2d> line_segment = {line[i], line[i + 1]};

    // find intersection of axis line and line segment
    if (auto result = intersectionOfLines(axis_line, line_segment)) {
      const Eigen::Vector2d& intersection = result->intersection;
      const bool intersects_line_segment = result->intersects_line2;
      found_at_least_one_intersection = true;
      if (intersects_line_segment) {
        found_intersection_with_line_segment = true;
        closest_projected_point = intersection;
        break;
      } else {
        double distance_to_line_segment =
          std::min((intersection - line[i]).norm(), (intersection - line[i + 1]).norm());
        if (distance_to_line_segment < closest_distance_to_line_segment) {
          closest_distance_to_line_segment = distance_to_line_segment;
          closest_projected_point = intersection;
        }
      }
    }
  }

  if (!found_at_least_one_intersection) {
    RCLCPP_ERROR(rclcpp::get_logger("new_lanelet2_route_planning"),
          "Could not project point to line along axis because axis is parallel to all line segments");
  }

  return closest_projected_point;
}

ll::ConstLanelet followLanelet(const ll::routing::RoutingGraphUPtr& routing_graph, const ll::ConstLanelet& lanelet,
                               const Eigen::Vector2d& position, const double distance) {
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

ll::BasicLineString2d resampleCenterlinesAlongPath(const ll::routing::LaneletPath& path, const double delta_s,
                                                   bool monotonically, std::vector<size_t>& lanelet_idx_by_point) {
  // init variables
  ll::BasicLineString2d resampled_path_centerline;
  double resampling_offset = 0.0;
  lanelet_idx_by_point.clear();
  Eigen::Vector2d prev_sampled_point = Eigen::Vector2d(0.0, 0.0);  // TODO: dangerous to init to 0?
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
        if ((centerline_point - prev_sampled_point).dot(prev_sampled_point_orientation) < 0) {  // angle > 90deg
          cit = centerline.erase(cit);
        } else {
          break;
        }
      }
    }

    // resample lanelet centerline
    std::vector<Eigen::Vector2d> resampled_centerline = resampleLineString(lineStringAsEigen(centerline), delta_s, resampling_offset);
    resampled_path_centerline.insert(resampled_path_centerline.end(), resampled_centerline.begin(),
                                     resampled_centerline.end());
    lanelet_idx_by_point.insert(lanelet_idx_by_point.end(), resampled_centerline.size(), l);

    // update information for monotonicity check
    if (monotonically && !resampled_centerline.empty()) {
      if (resampled_centerline.size() > 1) {
        prev_sampled_point = resampled_centerline[resampled_centerline.size() - 2];
      }
      prev_sampled_point_orientation =
          tangentOfPointAlongLineString(resampled_centerline.back(), prev_sampled_point, resampled_centerline.back());
      prev_sampled_point = resampled_centerline.back();
    }
  }

  return resampled_path_centerline;
}

std::vector<ll::ConstLanelet> adjacentLeftOrRightLanelets(const ll::ConstLanelet& lanelet,
                                                          const ll::routing::Route& route, bool left) {
  std::vector<ll::ConstLanelet> adjacent_lanelets;
  ll::routing::LaneletRelations relations = left ? route.leftRelations(lanelet) : route.rightRelations(lanelet);
  for (const auto& relation : relations) {
    if ((left && (relation.relationType == ll::routing::RelationType::Left ||
                  relation.relationType == ll::routing::RelationType::AdjacentLeft)) ||
        (!left && (relation.relationType == ll::routing::RelationType::Right ||
                   relation.relationType == ll::routing::RelationType::AdjacentRight))) {
      adjacent_lanelets.push_back(relation.lanelet);
    }
  }
  return adjacent_lanelets;
}

int computeFollowingLaneIdxOffset(const ll::ConstLanelet& lanelet, const ll::ConstLanelet& lanelet_of_next_point,
                                  const ll::routing::Route& route, const ll::routing::RoutingGraphUPtr& routing_graph) {
  int following_lane_idx_offset = 0;
  size_t n_adjacent_lanelets_of_next_lanelet;
  if (lanelet_of_next_point.id() != lanelet.id()) {
    // get adjacent lanelets of current lanelet
    std::vector<ll::ConstLanelet> adjacent_left_lanelets = adjacentLeftOrRightLanelets(lanelet, route, true);
    std::vector<ll::ConstLanelet> adjacent_right_lanelets = adjacentLeftOrRightLanelets(lanelet, route, false);
    int suggested_lane_idx = adjacent_left_lanelets.size();

    // get adjacent lanelets of next lanelet (lanelet of next point)
    std::vector<ll::ConstLanelet> adjacent_left_lanelets_of_next_lanelet =
        adjacentLeftOrRightLanelets(lanelet_of_next_point, route, true);
    std::vector<ll::ConstLanelet> adjacent_right_lanelets_of_next_lanelet =
        adjacentLeftOrRightLanelets(lanelet_of_next_point, route, false);
    std::vector<ll::ConstLanelet> adjacent_lanelets_of_next_lanelet = adjacent_left_lanelets_of_next_lanelet;
    adjacent_lanelets_of_next_lanelet.push_back(lanelet_of_next_point);
    adjacent_lanelets_of_next_lanelet.insert(adjacent_lanelets_of_next_lanelet.end(),
                                             adjacent_right_lanelets_of_next_lanelet.begin(),
                                             adjacent_right_lanelets_of_next_lanelet.end());
    n_adjacent_lanelets_of_next_lanelet = adjacent_lanelets_of_next_lanelet.size();

    // find following lanelet of current lanelet and adjacent lanelets in adjacent lanelets of next lanelet
    std::vector<std::vector<ll::ConstLanelet>> lanelet_groups = {
        {lanelet}, adjacent_left_lanelets, adjacent_right_lanelets};
    std::vector<int> lanelet_group_offset_factors = {0, 1, -1};
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
        size_t follow_further_idx =
            0;  // counter for following lanelets more than once (e.g., if sampling skipped short lanelets)
        size_t max_follow_further_iterations = 3;  // maximum number of following lanelets to check

        // loop until following lanelet is found in adjacent lanelets of next lanelet
        while (following_lane_idx_offset == std::numeric_limits<int>::max()) {
          // check all adjacent lanelets of next lanelet
          for (size_t l = 0; l < adjacent_lanelets_of_next_lanelet.size(); ++l) {
            if (following_lanelet.id() == adjacent_lanelets_of_next_lanelet[l].id()) {
              following_lane_idx_offset = l - suggested_lane_idx + group_offset_factor * (a + 1);  // gottesformel
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
      RCLCPP_ERROR(rclcpp::get_logger("new_lanelet2_route_planning"),
                   "Could not match following lanelets to adjacent lanelets of lanelet of next point");
    }
  }

  return following_lane_idx_offset;
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

uint8_t laneBoundaryType(const ll::ConstLineString2d& line) {
  uint8_t lane_boundary_type = route_planning_msgs::msg::LaneBoundary::TYPE_UNKNOWN;

  // get type attribute
  ll::Attribute type;
  if (line.hasAttribute("type")) {
    type = line.attribute("type");
  } else {
    lane_boundary_type = route_planning_msgs::msg::LaneBoundary::TYPE_UNKNOWN;
    return lane_boundary_type;
  }

  // map lanelet type to lane boundary type
  if (type == "road_boarder" || type == "barrier") {
    lane_boundary_type = route_planning_msgs::msg::LaneBoundary::TYPE_CROSSING_RESTRICTED;
  } else if (type == "line_thin" || type == "line_thick") {
    lane_boundary_type = route_planning_msgs::msg::LaneBoundary::TYPE_UNKNOWN;
    if (line.hasAttribute("subtype")) {
      ll::Attribute subtype = line.attribute("subtype");
      if (subtype == "solid" || subtype == "solid_solid") {
        lane_boundary_type = route_planning_msgs::msg::LaneBoundary::TYPE_CROSSING_RESTRICTED;
      } else if (subtype == "dashed") {
        lane_boundary_type = route_planning_msgs::msg::LaneBoundary::TYPE_CROSSING_ALLOWED;
      } else if (subtype == "dashed_solid") {
        lane_boundary_type = route_planning_msgs::msg::LaneBoundary::TYPE_CROSSING_ALLOWED_FROM_LEFT;
      } else if (subtype == "solid_dashed") {
        lane_boundary_type = route_planning_msgs::msg::LaneBoundary::TYPE_CROSSING_ALLOWED_FROM_RIGHT;
      }
    }
  } else if (type == "virtual") {
    lane_boundary_type = route_planning_msgs::msg::LaneBoundary::TYPE_UNKNOWN;
  } else {
    lane_boundary_type = route_planning_msgs::msg::LaneBoundary::TYPE_UNKNOWN;
  }

  return lane_boundary_type;
}

}  // namespace new_lanelet2_route_planning

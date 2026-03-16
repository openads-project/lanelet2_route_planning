// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cmath>
#include <limits>
#include <regex>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_utilities/lanelet2_utils.hpp>
#include <route_planning_msgs_utils/route_access.hpp>

#include "lanelet2_route_planning/conversions.hpp"
#include "lanelet2_route_planning/geometry.hpp"
#include "lanelet2_route_planning/utils.hpp"

namespace lanelet2_route_planning {

std::optional<lanelet::routing::Route> getRoute(const lanelet::routing::RoutingGraphUPtr& routing_graph,
                                                const std::vector<lanelet::ConstLanelet>& route_lanelets) {
  if (route_lanelets.empty()) {
    return std::nullopt;
  }

  const lanelet::ConstLanelet& start_lanelet = route_lanelets.front();
  std::vector<lanelet::ConstLanelet> intermediate_lanelets(route_lanelets.begin() + 1, route_lanelets.end() - 1);
  const lanelet::ConstLanelet& destination_lanelet = route_lanelets.back();

  // compute default route
  const int routing_cost_id = 0;  // RoutingCostDistance
  const bool with_lane_changes = true;
  auto route = routing_graph->getRouteVia(start_lanelet, intermediate_lanelets, destination_lanelet, routing_cost_id,
                                          with_lane_changes);

  // compute route alternatives with inverted start/destination lanelets (useful, if they are bidirectional)
  auto route_alternative1 =
      routing_graph->getRouteVia(start_lanelet.invert(), intermediate_lanelets, destination_lanelet, routing_cost_id);
  auto route_alternative2 =
      routing_graph->getRouteVia(start_lanelet, intermediate_lanelets, destination_lanelet.invert(), routing_cost_id);
  auto route_alternative3 = routing_graph->getRouteVia(start_lanelet.invert(), intermediate_lanelets,
                                                       destination_lanelet.invert(), routing_cost_id);
  std::vector<lanelet::routing::Route> route_alternatives;
  if (route) route_alternatives.push_back(std::move(*route));
  if (route_alternative1) route_alternatives.push_back(std::move(*route_alternative1));
  if (route_alternative2) route_alternatives.push_back(std::move(*route_alternative2));
  if (route_alternative3) route_alternatives.push_back(std::move(*route_alternative3));

  // select shortest route
  auto shortest_route_ptr = std::min_element(
      route_alternatives.begin(), route_alternatives.end(),
      [](const lanelet::routing::Route& a, const lanelet::routing::Route& b) { return a.length2d() < b.length2d(); });

  if (shortest_route_ptr == route_alternatives.end()) {
    return std::nullopt;
  }
  auto shortest_route = std::optional<lanelet::routing::Route>(std::move(*shortest_route_ptr));
  return shortest_route;
}

size_t indexOfLineStringPointClosestToPoint(const std::vector<Eigen::Vector2d>& line_string,
                                            const Eigen::Vector2d& point, const bool consider_order,
                                            const bool behind) {
  // loop over all points in line string to find closest one to given point
  size_t idx_closest = 0;
  double min_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < line_string.size(); ++i) {
    double distance = (line_string[i] - point).norm();
    if (distance < min_distance) {
      min_distance = distance;
      idx_closest = i;
    }
  }

  // if considering order, make sure to return the point behind or ahead of the given point
  if (consider_order) {
    idx_closest = considerOrderForPointMatchedToLineString(line_string, point, idx_closest, behind);
  }

  return idx_closest;
}

size_t matchPointToLineString(const std::vector<Eigen::Vector2d>& line_string, const Eigen::Vector2d& point,
                              const size_t idx_indication, const bool consider_order, const bool behind) {
  // constants
  const double max_delta_s = 10.0;
  const double max_local_distance = 10.0;

  size_t idx_closest = 0;
  double min_distance = std::numeric_limits<double>::infinity();
  const size_t start_idx = std::min(idx_indication, line_string.size() - 1);

  // loop over points in front of and behind the given index
  for (int direction : {1, -1}) {
    double delta_s = 0.0;
    size_t idx = start_idx;
    // only check points within the local range of max_delta_s
    while (delta_s <= max_delta_s && idx >= 0 && idx < line_string.size()) {
      double distance = (line_string[idx] - point).norm();
      if (distance < min_distance) {
        min_distance = distance;
        idx_closest = idx;
      }
      delta_s += (direction == 1 ? (line_string[idx + 1] - line_string[idx]).norm()
                                 : (line_string[idx] - line_string[idx - 1]).norm());
      idx += direction;
    }
  }

  // check if closest local point is within max distance, else find globally closest point
  if (min_distance > max_local_distance) {
    idx_closest = indexOfLineStringPointClosestToPoint(line_string, point, consider_order, behind);
  } else if (consider_order) {
    // if considering order, make sure to return the point behind or ahead of the given point
    idx_closest = considerOrderForPointMatchedToLineString(line_string, point, idx_closest, behind);
  }

  return idx_closest;
}

size_t considerOrderForPointMatchedToLineString(const std::vector<Eigen::Vector2d>& line_string,
                                                const Eigen::Vector2d& point, const size_t idx_closest,
                                                const bool behind) {
  size_t new_idx_closest;
  Eigen::Vector2d closest_point_to_next;
  const Eigen::Vector2d closest_point_to_point = point - line_string[idx_closest];
  if (idx_closest + 1 < line_string.size()) {
    closest_point_to_next = line_string[idx_closest + 1] - line_string[idx_closest];
  } else if (idx_closest > 0) {
    closest_point_to_next = line_string[idx_closest] - line_string[idx_closest - 1];
  } else {
    return idx_closest;
  }

  // use angle to check if closest point is behind or ahead of the given point
  const double angle = angleBetweenVectors(closest_point_to_point, closest_point_to_next);
  if (behind && std::abs(angle) > M_PI_2) {
    new_idx_closest = idx_closest - 1;
  } else if (!behind && std::abs(angle) < M_PI_2) {
    new_idx_closest = idx_closest + 1;
  } else {
    new_idx_closest = idx_closest;
  }
  new_idx_closest = std::clamp(new_idx_closest, 0lu, line_string.size() - 1);

  return new_idx_closest;
}

bool changesLaneFromPointToPoint(const Eigen::Vector2d& point, const Eigen::Vector2d& next_point,
                                 const double sampling_distance) {
  const double epsilon = 1e-6;
  return ((next_point - point).norm() > (sampling_distance + epsilon));
}

std::vector<lanelet::ConstLanelet> adjacentLeftOrRightLanelets(const lanelet::ConstLanelet& lanelet,
                                                               const lanelet::routing::RoutingGraphUPtr& routing_graph,
                                                               bool left, bool sort_from_left) {
  std::vector<lanelet::ConstLanelet> adjacent_lanelets;
  const int routing_cost_id = 0;  // RoutingCostDistance
  lanelet::routing::LaneletRelations relations = left ? routing_graph->leftRelations(lanelet, routing_cost_id)
                                                      : routing_graph->rightRelations(lanelet, routing_cost_id);
  for (const auto& relation : relations) {
    if ((left && (relation.relationType == lanelet::routing::RelationType::Left ||
                  relation.relationType == lanelet::routing::RelationType::AdjacentLeft)) ||
        (!left && (relation.relationType == lanelet::routing::RelationType::Right ||
                   relation.relationType == lanelet::routing::RelationType::AdjacentRight))) {
      adjacent_lanelets.push_back(relation.lanelet);
    }
  }

  if ((left && sort_from_left) || (!left && !sort_from_left)) {
    std::reverse(adjacent_lanelets.begin(), adjacent_lanelets.end());
  }

  return adjacent_lanelets;
}

std::vector<ProjectedLaneletPoints> projectPointToLaneletLines(const Eigen::Vector2d& point,
                                                               const Eigen::Vector2d& prev_point,
                                                               const Eigen::Vector2d& next_point,
                                                               const std::vector<lanelet::ConstLanelet>& lanelets,
                                                               const rclcpp::Logger& logger) {
  std::vector<ProjectedLaneletPoints> projected_points_per_lanelet;

  // loop over lanelets
  for (const auto& lanelet : lanelets) {
    ProjectedLaneletPoints projected_points;

    // project point to left bounds
    if (auto result = projectPointToLineStringAlongNormal(point, prev_point, next_point,
                                                          toEigen(lanelet.leftBound2d().basicLineString()))) {
      projected_points.left_bound_point = result->projected_point;
    } else {
      RCLCPP_WARN(logger, "Failed to project point (%.3f, %.3f) to left bounds of lanelet %ld", point.x(), point.y(),
                  lanelet.id());
    }

    // project point to centerline
    if (auto result = projectPointToLineStringAlongNormal(point, prev_point, next_point,
                                                          toEigen(lanelet.centerline2d().basicLineString()))) {
      projected_points.centerline_point = result->projected_point;
    } else {
      RCLCPP_WARN(logger, "Failed to project point (%.3f, %.3f) to centerline of lanelet %ld", point.x(), point.y(),
                  lanelet.id());
    }

    // project point to right bounds
    if (auto result = projectPointToLineStringAlongNormal(point, prev_point, next_point,
                                                          toEigen(lanelet.rightBound2d().basicLineString()))) {
      projected_points.right_bound_point = result->projected_point;
    } else {
      RCLCPP_WARN(logger, "Failed to project point (%.3f, %.3f) to right bounds of lanelet %ld", point.x(), point.y(),
                  lanelet.id());
    }

    projected_points_per_lanelet.push_back(projected_points);
  }

  return projected_points_per_lanelet;
}

std::optional<int> computeFollowingLaneIdxOffset(const lanelet::ConstLanelet& lanelet,
                                                 const lanelet::ConstLanelet& lanelet_of_next_point,
                                                 const lanelet::routing::RoutingGraphUPtr& routing_graph) {
  int following_lane_idx_offset = 0;
  if (lanelet_of_next_point.id() != lanelet.id()) {
    // get adjacent lanelets of current lanelet
    std::vector<lanelet::ConstLanelet> adjacent_left_lanelets =
        adjacentLeftOrRightLanelets(lanelet, routing_graph, true);
    std::vector<lanelet::ConstLanelet> adjacent_right_lanelets =
        adjacentLeftOrRightLanelets(lanelet, routing_graph, false);
    int suggested_lane_idx = adjacent_left_lanelets.size();

    // get adjacent lanelets of next lanelet (lanelet of next point)
    std::vector<lanelet::ConstLanelet> adjacent_left_lanelets_of_next_lanelet =
        adjacentLeftOrRightLanelets(lanelet_of_next_point, routing_graph, true);
    std::vector<lanelet::ConstLanelet> adjacent_right_lanelets_of_next_lanelet =
        adjacentLeftOrRightLanelets(lanelet_of_next_point, routing_graph, false);
    std::vector<lanelet::ConstLanelet> adjacent_lanelets_of_next_lanelet = adjacent_left_lanelets_of_next_lanelet;
    adjacent_lanelets_of_next_lanelet.push_back(lanelet_of_next_point);
    adjacent_lanelets_of_next_lanelet.insert(adjacent_lanelets_of_next_lanelet.end(),
                                             adjacent_right_lanelets_of_next_lanelet.begin(),
                                             adjacent_right_lanelets_of_next_lanelet.end());

    // find following lanelet of current lanelet and adjacent lanelets in adjacent lanelets of next lanelet
    std::vector<std::vector<lanelet::ConstLanelet>> lanelet_groups = {
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
        if (following_lanelets.empty()) {
          continue;
        }
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
            if (following_lanelets.empty()) {
              continue;
            }
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
      // could not match following lanelets to adjacent lanelets of lanelet of next point
      return std::nullopt;
    }
  }

  return following_lane_idx_offset;
}

route_planning_msgs::msg::RouteElement createMinimalRouteElement(const geometry_msgs::msg::Point& position,
                                                                 const geometry_msgs::msg::Quaternion& orientation,
                                                                 double s, bool will_change_suggested_lane,
                                                                 uint8_t speed_limit) {
  // create RouteElement
  route_planning_msgs::msg::RouteElement route_element_msg;
  route_element_msg.suggested_lane_idx = 0;
  route_element_msg.will_change_suggested_lane = will_change_suggested_lane;
  route_element_msg.s = s;
  route_element_msg.is_enriched = false;
  // route_element_msg.left_boundary not set in global route
  // route_element_msg.right_boundary not set in global route
  // route_element_msg.regulatory_elements not set in global route

  // create LaneElement
  route_planning_msgs::msg::LaneElement lane_element_msg;
  lane_element_msg.reference_pose.position = position;
  lane_element_msg.reference_pose.orientation = orientation;
  // lane_element_msg.left_boundary not set in global route
  // lane_element_msg.right_boundary not set in global route
  lane_element_msg.speed_limit = speed_limit;
  // lane_element_msg.regulatory_element_idcs not set in global route
  lane_element_msg.following_lane_idx = 0;
  lane_element_msg.has_following_lane_idx = !will_change_suggested_lane;
  route_element_msg.lane_elements.push_back(lane_element_msg);

  return route_element_msg;
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> extractDrivableSpace(const lanelet::LineStringLayer& line_string_layer,
                                                                 const PointSequence& point_sequence,
                                                                 const double max_distance) {
  // find all line strings within max_distance
  const lanelet::BasicLineString2d line_to_search_around = {point_sequence.current, point_sequence.next};
  std::vector<std::pair<double, lanelet::ConstLineString3d>> line_strings_and_distances =
      lanelet::geometry::findWithin2d(line_string_layer, line_to_search_around, max_distance);

  // project current point to all line strings
  std::vector<std::pair<Eigen::Vector2d, size_t>> projected_points_and_line_string_idcs;
  for (size_t l = 0; l < line_strings_and_distances.size(); ++l) {
    const auto& line_string = line_strings_and_distances[l].second;
    auto result = projectPointToLineStringAlongNormal(point_sequence.current, point_sequence.prev, point_sequence.next,
                                                      to2d(toEigen(line_string.basicLineString())));
    if (result && result->found_intersection_with_line_segment) {
      projected_points_and_line_string_idcs.emplace_back(result->projected_point, l);
    }
  }

  // sort projected points by distance to current point
  std::sort(projected_points_and_line_string_idcs.begin(), projected_points_and_line_string_idcs.end(),
            [&point_sequence](const auto& a, const auto& b) {
              return (a.first - point_sequence.current).norm() < (b.first - point_sequence.current).norm();
            });

  // split projected points into those left and those right of current point
  const Eigen::Vector2d tangent =
      tangentOfPointAlongLineString(point_sequence.current, point_sequence.prev, point_sequence.next);
  std::vector<std::pair<Eigen::Vector2d, size_t>> left_projected_points_and_line_string_idcs,
      right_projected_points_and_line_string_idcs;
  for (const auto& projected_point_and_line_string_idx : projected_points_and_line_string_idcs) {
    // determine left/right based on angle between tangent and vector to projected point
    const Eigen::Vector2d point_to_projected_point = projected_point_and_line_string_idx.first - point_sequence.current;
    const double angle = angleBetweenVectors(tangent, point_to_projected_point);
    if (angle > 0) {
      left_projected_points_and_line_string_idcs.emplace_back(projected_point_and_line_string_idx);
    } else {
      right_projected_points_and_line_string_idcs.emplace_back(projected_point_and_line_string_idx);
    }
  }

  // find drivable space bounds by following projected points until corresponding line string is not passable anymore
  Eigen::Vector2d drivable_space_left, drivable_space_right;
  bool is_drivable_space_left_limited_by_line_strings = false;
  bool is_drivable_space_right_limited_by_line_strings = false;
  for (const auto& projected_point_and_line_string_idx : left_projected_points_and_line_string_idcs) {
    const auto& line_string = line_strings_and_distances[projected_point_and_line_string_idx.second].second;
    if (!isLineStringDrivable(line_string)) {
      drivable_space_left = projected_point_and_line_string_idx.first;
      is_drivable_space_left_limited_by_line_strings = true;
      break;
    }
  }
  for (const auto& projected_point_and_line_string_idx : right_projected_points_and_line_string_idcs) {
    const auto& line_string = line_strings_and_distances[projected_point_and_line_string_idx.second].second;
    if (!isLineStringDrivable(line_string)) {
      drivable_space_right = projected_point_and_line_string_idx.first;
      is_drivable_space_right_limited_by_line_strings = true;
      break;
    }
  }

  // if drivable space is not limited by line strings, use maximum distance
  const Eigen::Vector2d normal =
      normalOfPointAlongLineString(point_sequence.current, point_sequence.prev, point_sequence.next);
  if (!is_drivable_space_left_limited_by_line_strings) {
    drivable_space_left = point_sequence.current - normal * max_distance;
  }
  if (!is_drivable_space_right_limited_by_line_strings) {
    drivable_space_right = point_sequence.current + normal * max_distance;
  }

  return {drivable_space_left, drivable_space_right};
}

bool isLineStringDrivable(const lanelet::ConstLineString3d& line_string) {
  const std::unordered_set<std::string> drivable_types = {
      "arrow",         "bike_marking", "centerline",         "curbstone",    "lane_center",
      "line_thick",    "line_thin",    "pedestrian_marking", "roadpainting", "stop_line",
      "traffic_light", "virtual",      "zebra_marking"};
  if (line_string.hasAttribute("type")) {
    std::string type = line_string.attribute("type").value();
    if (drivable_types.count(type) > 0) {
      if (type == "curbstone") {
        if (!line_string.hasAttribute("subtype") || line_string.attribute("subtype").value() != "low") {
          return false;
        }
      }
      return true;
    } else {
      if (line_string.hasAttribute("HoldingLine")) {
        return true;
      } else {
        return false;
      }
    }
  } else {
    return false;
  }
}

ExtractRegulatoryElementsResult extractRegulatoryElements(
    const lanelet::ConstLanelet& lanelet, const std::vector<lanelet::ConstLanelet>& adjacent_left_lanelets,
    const std::vector<lanelet::ConstLanelet>& adjacent_right_lanelets, const PointSequence& point_sequence) {
  // init result
  ExtractRegulatoryElementsResult result;
  result.adjacent_left_regulatory_element_idcs.resize(adjacent_left_lanelets.size());
  result.adjacent_right_regulatory_element_idcs.resize(adjacent_right_lanelets.size());

  // gather lanelets in single vector (left adjacent, current, right adjacent)
  std::vector<lanelet::ConstLanelet> lanelets = adjacent_left_lanelets;
  lanelets.push_back(lanelet);
  lanelets.insert(lanelets.end(), adjacent_right_lanelets.begin(), adjacent_right_lanelets.end());

  // loop over lanelets
  std::unordered_map<size_t, size_t> regulatory_element_msg_idx_by_id;
  for (size_t l = 0; l < lanelets.size(); ++l) {
    const auto& current_lanelet = lanelets[l];

    // loop over regulatory elements of lanelet
    const auto regulatory_elements = current_lanelet.regulatoryElements();
    for (const auto& regulatory_element : regulatory_elements) {
      // create RegulatoryElement
      route_planning_msgs::msg::RegulatoryElement regulatory_element_msg;
      regulatory_element_msg.has_validity_stamp = false;
      regulatory_element_msg.validity_stamp = builtin_interfaces::msg::Time();

      // extract reference line
      if (auto reference_line = regulatoryElementReferenceLine(regulatory_element)) {
        regulatory_element_msg.reference_line = *reference_line;

        // only consider regulatory element if reference line intersects with point sequence
        std::vector<Eigen::Vector2d> reference_line_2d = {toEigen2d(reference_line->at(0)),
                                                          toEigen2d(reference_line->at(1))};
        std::vector<Eigen::Vector2d> line_to_next_point = {point_sequence.current, point_sequence.next};
        std::vector<Eigen::Vector2d> line_to_prev_point = {point_sequence.current, point_sequence.prev};
        if (auto result = intersectionOfLines(reference_line_2d, line_to_next_point)) {
          if (!result->intersects_line2) {
            if (auto inner_result = intersectionOfLines(reference_line_2d, line_to_prev_point)) {
              if (!inner_result->intersects_line2) {
                continue;
              }
            }
          }
        }
      } else {
        continue;
      }

      // extract sign positions and type
      regulatory_element_msg.positions = regulatoryElementPositions(regulatory_element);
      std::tie(regulatory_element_msg.type, regulatory_element_msg.meta_value) =
          regulatoryElementType(regulatory_element);

      // flatten regulatory element to z=0 (2D)
      for (auto& position : regulatory_element_msg.positions) {
        position.z -= (regulatory_element_msg.reference_line[0].z + regulatory_element_msg.reference_line[1].z) / 2.0;
      }
      regulatory_element_msg.reference_line[0].z = 0.0;
      regulatory_element_msg.reference_line[1].z = 0.0;

      // check if regulatory element has already been extracted (by another lanelet)
      size_t regulatory_element_msg_idx;
      if (regulatory_element_msg_idx_by_id.count(regulatory_element->id()) > 0) {
        regulatory_element_msg_idx = regulatory_element_msg_idx_by_id[regulatory_element->id()];
      } else {
        // add regulatory element to result
        regulatory_element_msg_idx = result.regulatory_element_msgs.size();
        regulatory_element_msg_idx_by_id[regulatory_element->id()] = regulatory_element_msg_idx;
        result.regulatory_element_msgs.push_back(regulatory_element_msg);
      }

      // assign regulatory element to respective lanelet in result
      if (l < adjacent_left_lanelets.size()) {
        result.adjacent_left_regulatory_element_idcs[l].push_back(regulatory_element_msg_idx);
      } else if (l < adjacent_left_lanelets.size() + 1) {
        result.regulatory_element_idcs.push_back(regulatory_element_msg_idx);
      } else {
        size_t l_right = l - adjacent_left_lanelets.size() - 1;
        result.adjacent_right_regulatory_element_idcs[l_right].push_back(regulatory_element_msg_idx);
      }
    }
  }

  return result;
}

std::optional<std::array<geometry_msgs::msg::Point, 2>> regulatoryElementReferenceLine(
    const std::shared_ptr<const lanelet::RegulatoryElement>& regulatory_element) {
  const std::vector<lanelet::ConstLineString3d> reference_lines =
      regulatory_element->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::RefLine);
  if (reference_lines.empty()) {
    return std::nullopt;
  }
  const std::vector<Eigen::Vector3d> reference_line = reference_lines.front().basicLineString();
  if (reference_line.size() < 2) {
    return std::nullopt;
  }
  std::array<geometry_msgs::msg::Point, 2> reference_line_ros = {toRos(reference_line.front()),
                                                                 toRos(reference_line.back())};
  return reference_line_ros;
}

std::vector<geometry_msgs::msg::Point> regulatoryElementPositions(
    const std::shared_ptr<const lanelet::RegulatoryElement>& regulatory_element) {
  std::vector<geometry_msgs::msg::Point> positions;
  const std::vector<lanelet::ConstLineString3d> sign_lines =
      regulatory_element->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::Refers);
  for (const auto& const_sign_line : sign_lines) {
    const std::vector<Eigen::Vector3d> sign_line = const_sign_line.basicLineString();
    if (!sign_line.empty()) {
      positions.push_back(toRos(sign_line.front()));
    }
  }
  return positions;
}

std::pair<uint8_t, uint8_t> regulatoryElementType(
    const std::shared_ptr<const lanelet::RegulatoryElement>& regulatory_element) {
  uint8_t type = route_planning_msgs::msg::RegulatoryElement::TYPE_UNKNOWN;
  uint8_t meta_value = 0;

  // https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/RegulatoryElementTagging.md
  if (regulatory_element->hasAttribute("subtype")) {
    std::string subtype = regulatory_element->attribute("subtype").value();
    if (subtype == "traffic_light") {
      type = route_planning_msgs::msg::RegulatoryElement::TYPE_TRAFFIC_LIGHT;
    } else if (subtype == "speed_limit") {
      type = route_planning_msgs::msg::RegulatoryElement::TYPE_SPEED_LIMIT;
      meta_value = regulatoryElementSpeedLimit(regulatory_element);
    } else if (subtype == "right_of_way") {
      type = route_planning_msgs::msg::RegulatoryElement::TYPE_YIELD;
    } else if (subtype == "all_way_stop") {
      type = route_planning_msgs::msg::RegulatoryElement::TYPE_STOP;
    }
  }
  return {type, meta_value};
}

uint8_t regulatoryElementSpeedLimit(const std::shared_ptr<const lanelet::RegulatoryElement>& regulatory_element) {
  uint8_t speed_limit = route_planning_msgs::msg::RegulatoryElement::META_VALUE_SPEED_UNKNOWN;

  // https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/RegulatoryElementTagging.md#speed-limit
  if (regulatory_element->hasAttribute("subtype")) {
    std::string subtype = regulatory_element->attribute("subtype").value();
    if (subtype == "speed_limit") {
      if (regulatory_element->hasAttribute("sign_type")) {
        std::string sign_type = regulatory_element->attribute("sign_type").value();
        std::smatch match;
        std::regex regex("(\\d+)\\s*(km/h|mph|mps)");
        if (std::regex_search(sign_type, match, regex)) {
          int speed = std::stoi(match.str(1));
          if (match.str(2) == "mph") {
            speed = static_cast<int>(speed * 1.60934);  // mph to km/h
          } else if (match.str(2) == "mps") {
            speed = static_cast<int>(speed * 3.6);  // mps to km/h
          }
          speed_limit = static_cast<uint8_t>(speed);
        } else {
          std::regex regex("(\\d+)");  // assume km/h if no unit is specified
          if (std::regex_search(sign_type, match, regex)) {
            speed_limit = static_cast<uint8_t>(std::stoi(match.str(1)));
          }
        }
      }
    }
  }

  uint8_t unlimited = route_planning_msgs::msg::RegulatoryElement::META_VALUE_SPEED_UNLIMITED;
  speed_limit = std::clamp(speed_limit, static_cast<uint8_t>(0), unlimited);

  return speed_limit;
}

uint8_t laneBoundaryType(const lanelet::ConstLineString2d& line) {
  uint8_t lane_boundary_type = route_planning_msgs::msg::LaneBoundary::TYPE_UNKNOWN;

  // get type attribute
  lanelet::Attribute type;
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
      lanelet::Attribute subtype = line.attribute("subtype");
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

uint8_t speedLimit(const lanelet::ConstLanelet& lanelet) {
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules = getTrafficRules();
  lanelet::traffic_rules::SpeedLimitInformation speed_limit_info = traffic_rules->speedLimit(lanelet);
  uint8_t unlimited = route_planning_msgs::msg::RegulatoryElement::META_VALUE_SPEED_UNLIMITED;
  if (speed_limit_info.isMandatory) {
    int speed_limit = std::round(lanelet::units::KmHQuantity(speed_limit_info.speedLimit).value());
    speed_limit = std::clamp(speed_limit, 0, static_cast<int>(unlimited));
    return static_cast<uint8_t>(speed_limit);
  } else {
    return unlimited;
  }
}

std::tuple<uint8_t, int> suggestedTurnSignal(const lanelet::ConstLanelet& lanelet, const rclcpp::Logger& logger) {
  uint8_t suggested_turn_signal = route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_NONE;
  int suggested_turn_signal_distance_ahead = 0;

  // parse suggested turn signal attribute
  if (lanelet.hasAttribute("suggested_turn_signal")) {
    std::string suggested_turn_signal_str = lanelet.attribute("suggested_turn_signal").value();
    if (suggested_turn_signal_str == "left") {
      suggested_turn_signal = route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_LEFT;
    } else if (suggested_turn_signal_str == "right") {
      suggested_turn_signal = route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_RIGHT;
    } else if (suggested_turn_signal_str == "hazard") {
      suggested_turn_signal = route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_HAZARD;
    } else {
      suggested_turn_signal = route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_NONE;
      RCLCPP_ERROR(logger, "Could not parse 'suggested_turn_signal' attribute value of lanelet '%ld': '%s'",
                   lanelet.id(), suggested_turn_signal_str.c_str());
      return std::make_tuple(suggested_turn_signal, suggested_turn_signal_distance_ahead);
    }

    // parse suggested turn signal distance ahead attribute
    if (lanelet.hasAttribute("suggested_turn_signal_distance_ahead")) {
      std::string suggested_turn_signal_distance_ahead_str =
          lanelet.attribute("suggested_turn_signal_distance_ahead").value();
      try {
        suggested_turn_signal_distance_ahead = std::stoi(suggested_turn_signal_distance_ahead_str);
      } catch (const std::exception&) {
        RCLCPP_ERROR(logger,
                     "Could not parse 'suggested_turn_signal_distance_ahead' attribute value of lanelet '%ld': '%s'",
                     lanelet.id(), suggested_turn_signal_distance_ahead_str.c_str());
        return std::make_tuple(suggested_turn_signal, suggested_turn_signal_distance_ahead);
      }
      if (suggested_turn_signal_distance_ahead < 0) {
        RCLCPP_ERROR(
            logger,
            "'suggested_turn_signal_distance_ahead' attribute value of lanelet '%ld' is negative, clamping to 0: %d",
            lanelet.id(), suggested_turn_signal_distance_ahead);
        suggested_turn_signal_distance_ahead = 0;
      }
    } else {
      RCLCPP_ERROR(
          logger,
          "Lanelet '%ld' has 'suggested_turn_signal' attribute but no 'suggested_turn_signal_distance_ahead', ignoring "
          "suggested turn signal",
          lanelet.id());
    }
  }
  return std::make_tuple(suggested_turn_signal, suggested_turn_signal_distance_ahead);
}

lanelet::traffic_rules::TrafficRulesPtr getTrafficRules() {
  auto location = lanelet::Locations::Germany;
  auto vehicle_type = std::string(lanelet::Participants::Vehicle);
  return lanelet::traffic_rules::TrafficRulesFactory::create(location, vehicle_type);
}

std::optional<lanelet::ConstLanelet> laneletAtPoint(
    const Eigen::Vector2d& point, const lanelet::LaneletMapConstPtr& map,
    const std::optional<lanelet::traffic_rules::TrafficRulesPtr> traffic_rules) {
  // parameters for lanelet matching
  const unsigned int k_nearest_lanelets = 5;
  const double max_distance_lanelet_matching = 5.0;

  // find nearest lanelets
  std::vector<std::pair<double, lanelet::ConstLanelet>> nearest_lanelets =
      lanelet::geometry::findNearest(map->laneletLayer, point, k_nearest_lanelets);

  // find best matching lanelet
  if (traffic_rules) {
    std::ignore = Lanelet2Utilities::laneletSorting(point, nearest_lanelets, {}, traffic_rules.value(), {});
    for (const auto& ll : nearest_lanelets) {
      if (ll.first <= max_distance_lanelet_matching && traffic_rules.value()->canPass(ll.second)) {
        return ll.second;
      }
    }
  } else if (!nearest_lanelets.empty()) {
    std::ignore = Lanelet2Utilities::laneletSorting(point, nearest_lanelets, {}, {}, {});
    if (nearest_lanelets[0].first <= max_distance_lanelet_matching) {
      return nearest_lanelets[0].second;
    }
  }

  return std::nullopt;
}

lanelet::ConstLanelet followLaneletsAlongRoutingGraph(const lanelet::routing::RoutingGraphUPtr& routing_graph,
                                                      const lanelet::ConstLanelet& lanelet,
                                                      const Eigen::Vector2d& position, const double distance) {
  lanelet::ConstLanelet followed_lanelet = lanelet;
  double remaining_length;
  if (distance > 0) {
    remaining_length = lanelet::geometry::length(lanelet.centerline2d()) -
                       lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), position).length;
  } else {
    remaining_length = lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), position).length;
  }
  double remaining_distance = std::abs(distance);

  while (remaining_distance > remaining_length) {
    lanelet::ConstLanelets next_lanelets;
    if (distance > 0) {
      next_lanelets = routing_graph->following(followed_lanelet, false);
    } else {
      next_lanelets = routing_graph->previous(followed_lanelet);
    }
    if (next_lanelets.empty()) {
      break;
    }
    followed_lanelet = next_lanelets.front();
    remaining_distance -= remaining_length;
    remaining_length = lanelet::geometry::length(followed_lanelet.centerline2d());
  }

  return followed_lanelet;
}

ResampleCenterlinesAlongPathResult resampleCenterlinesAlongPath(const lanelet::routing::LaneletPath& path,
                                                                const double delta_s, bool monotonically) {
  // init variables
  ResampleCenterlinesAlongPathResult result;
  double resampling_offset = 0.0;
  Eigen::Vector2d prev_sampled_point = Eigen::Vector2d(0.0, 0.0);
  Eigen::Vector2d prev_sampled_point_orientation = Eigen::Vector2d(0.0, 0.0);

  // loop over lanelets in path
  for (size_t l = 0; l < path.size(); ++l) {
    // get centerline
    const lanelet::ConstLanelet& lanelet = path[l];
    lanelet::BasicLineString2d centerline = lanelet.centerline2d().basicLineString();

    // skip point if behind previous sampled point, e.g., if on adjacent lanelet in shortest path due to lane change
    if (monotonically && !result.centerline.empty()) {
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
    std::vector<Eigen::Vector2d> resampled_centerline =
        resampleLineString(toEigen(centerline), delta_s, resampling_offset);
    result.centerline.insert(result.centerline.end(), resampled_centerline.begin(), resampled_centerline.end());
    result.lanelet_idx_by_point.insert(result.lanelet_idx_by_point.end(), resampled_centerline.size(), l);

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

  return result;
}

double distanceTraveled(const route_planning_msgs::msg::Route& route) {
  if (route.current_route_element_idx >= route.route_elements.size() ||
      route.starting_route_element_idx >= route.route_elements.size()) {
    return 0.0;
  }
  double to_current = route.route_elements[route.current_route_element_idx].s;
  double ahead_of_starting_point = route.route_elements[route.starting_route_element_idx].s;
  double traveled = to_current - ahead_of_starting_point;
  return traveled;
}

double distanceRemaining(const route_planning_msgs::msg::Route& route) {
  if (route.current_route_element_idx >= route.route_elements.size() ||
      route.destination_route_element_idx >= route.route_elements.size()) {
    return 0.0;
  }
  double to_current = route.route_elements[route.current_route_element_idx].s;
  double to_destination = route.route_elements[route.destination_route_element_idx].s;
  double remaining = to_destination - to_current;
  return remaining;
}

double estimateRemainingTime(const route_planning_msgs::msg::Route& route, const double reference_speed) {
  double remaining_time = 0.0;
  for (size_t r = route.current_route_element_idx; r < route.route_elements.size() - 1; ++r) {
    const auto& route_element = route.route_elements[r];
    const auto& next_route_element = route.route_elements[r + 1];
    double speed_limit = route_planning_msgs::route_access::getSuggestedLaneElement(route_element).speed_limit;
    if (speed_limit == 0) {
      speed_limit = reference_speed;
    }
    if (speed_limit > 0) {
      remaining_time += (next_route_element.s - route_element.s) / speed_limit;
    }
  }
  return remaining_time;
}

void postprocessRouteMessage(
    route_planning_msgs::msg::Route& route_msg,
    std::vector<std::vector<int>>& suggested_turn_signal_distance_ahead_by_route_element_by_lane_element) {
  // loop over route elements to set orientations and following lane indices
  std::vector<route_planning_msgs::msg::RouteElement>& route_elements = route_msg.route_elements;
  for (size_t r = 0; r < route_elements.size(); ++r) {
    // get current, previous and next route element
    auto& route_element = route_elements[r];
    auto& prev_route_element = (r > 0) ? route_elements[r - 1] : route_element;
    const auto& next_route_element = (r < route_elements.size() - 1) ? route_elements[r + 1] : route_element;

    // fix following lane index at the last non-enriched route element before route elements are enriched
    if (r > 0) {
      if (!prev_route_element.is_enriched) {
        prev_route_element.lane_elements[0].has_following_lane_idx = true;
        prev_route_element.lane_elements[0].following_lane_idx = route_element.suggested_lane_idx;
      }
    }

    // loop over lane elements of current route element
    for (size_t l = 0; l < route_element.lane_elements.size(); ++l) {
      // get current, previous and next lane element
      auto& lane_element = route_element.lane_elements[l];
      const auto prev_lane_element_opt =
          route_planning_msgs::route_access::getPrecedingLaneElement(l, prev_route_element);
      const auto next_lane_element_opt =
          route_planning_msgs::route_access::getFollowingLaneElement(lane_element, next_route_element);

      // find current, previous and next points to compute orientation
      const auto point = toEigen2d(lane_element.reference_pose.position);
      const bool changes_lane_from_prev_point = prev_route_element.will_change_suggested_lane;
      const bool changes_lane_to_next_point = next_route_element.will_change_suggested_lane;
      const auto prev_point_for_orientation = (changes_lane_from_prev_point || !prev_lane_element_opt)
                                                  ? point
                                                  : toEigen2d(prev_lane_element_opt->reference_pose.position);
      const auto next_point_for_orientation = (changes_lane_to_next_point || !next_lane_element_opt)
                                                  ? point
                                                  : toEigen2d(next_lane_element_opt->reference_pose.position);

      // compute orientation of current point
      const auto orientation =
          tangentOfPointAlongLineString(point, prev_point_for_orientation, next_point_for_orientation);
      lane_element.reference_pose.orientation = toRosQuaternion(orientation);
    }
  }

  // loop over route elements in reverse to propagate suggested turn signals backwards
  for (int r = route_elements.size() - 1; r >= 0; --r) {
    auto& route_element = route_elements[r];
    if (!route_element.is_enriched) continue;  // only handle turn signal information for enriched route elements

    // loop over lane elements of current route element
    for (size_t l = 0; l < route_element.lane_elements.size(); ++l) {
      auto& lane_element = route_element.lane_elements[l];

      if (suggested_turn_signal_distance_ahead_by_route_element_by_lane_element[r].empty()) continue;
      int suggested_turn_signal_distance_ahead =
          suggested_turn_signal_distance_ahead_by_route_element_by_lane_element[r][l];

      // get preceding lane element
      if (r <= 0) break;
      int curr_r = r;
      auto* curr_lane_element = &lane_element;
      auto* prev_route_element = &route_elements[r - 1];
      auto prev_lane_element_idx_opt =
          route_planning_msgs::route_access::getPrecedingLaneElementIdx(l, *prev_route_element);

      // iterate over preceding lane elements within suggested distance to set suggested turn signal
      while (suggested_turn_signal_distance_ahead > 0 && prev_lane_element_idx_opt) {
        auto& prev_lane_element = prev_route_element->lane_elements[*prev_lane_element_idx_opt];
        if (!prev_route_element->is_enriched) break;  // only propagate through enriched route elements

        // stop if preceding lane element has its own suggested turn signal information
        if (!suggested_turn_signal_distance_ahead_by_route_element_by_lane_element[curr_r - 1].empty()) {
          int prev_suggested_turn_signal_distance_ahead =
              suggested_turn_signal_distance_ahead_by_route_element_by_lane_element[curr_r - 1]
                                                                                   [*prev_lane_element_idx_opt];
          if (prev_suggested_turn_signal_distance_ahead >= 0) {
            break;
          }
        }

        // check distance to preceding lane element
        const auto point = toEigen2d(curr_lane_element->reference_pose.position);
        const auto prev_point = toEigen2d(prev_lane_element.reference_pose.position);
        const double distance_to_prev_point = (point - prev_point).norm();
        if (distance_to_prev_point > suggested_turn_signal_distance_ahead) {
          break;  // stop if distance to preceding point exceeds remaining distance ahead
        }

        // set suggested turn signal of preceding lane element
        prev_lane_element.suggested_turn_signal = lane_element.suggested_turn_signal;

        // update remaining distance and move to next preceding lane element
        suggested_turn_signal_distance_ahead -= static_cast<uint8_t>(std::round(distance_to_prev_point));
        curr_r -= 1;
        if (curr_r <= 0) break;
        curr_lane_element = &prev_lane_element;
        prev_route_element = &route_elements[curr_r - 1];
        prev_lane_element_idx_opt = route_planning_msgs::route_access::getPrecedingLaneElementIdx(
            *prev_lane_element_idx_opt, *prev_route_element);
      }
    }
  }
}

}  // namespace lanelet2_route_planning

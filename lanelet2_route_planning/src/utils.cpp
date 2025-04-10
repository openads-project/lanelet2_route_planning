#include <limits>

#include <lanelet2_core/utility/Units.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_utilities/lanelet2_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <route_planning_msgs_utils/route_access.hpp>

#include "lanelet2_route_planning/conversions.hpp"
#include "lanelet2_route_planning/geometry.hpp"
#include "lanelet2_route_planning/utils.hpp"

namespace lanelet2_route_planning {

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
      RCLCPP_WARN(rclcpp::get_logger("lanelet2_route_planning"),
                  "Failed to project point (%.3f, %.3f) to left bounds of lanelet %ld", point.x(), point.y(),
                  lanelet.id());
    }

    // project point to centerline
    if (auto result = projectPointToLineStringAlongNormal(point, prev_point, next_point,
                                                          toEigen(lanelet.centerline2d().basicLineString()))) {
      projected_points.centerline_point = result->projected_point;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("lanelet2_route_planning"),
                  "Failed to project point (%.3f, %.3f) to centerline of lanelet %ld", point.x(), point.y(),
                  lanelet.id());
    }

    // project point to right bounds
    if (auto result = projectPointToLineStringAlongNormal(point, prev_point, next_point,
                                                          toEigen(lanelet.rightBound2d().basicLineString()))) {
      projected_points.right_bound_point = result->projected_point;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("lanelet2_route_planning"),
                  "Failed to project point (%.3f, %.3f) to right bounds of lanelet %ld", point.x(), point.y(),
                  lanelet.id());
    }

    projected_points_per_lanelet.push_back(projected_points);
  }

  return projected_points_per_lanelet;
}

int computeFollowingLaneIdxOffset(const lanelet::ConstLanelet& lanelet,
                                  const lanelet::ConstLanelet& lanelet_of_next_point,
                                  const lanelet::routing::Route& route,
                                  const lanelet::routing::RoutingGraphUPtr& routing_graph) {
  int following_lane_idx_offset = 0;
  if (lanelet_of_next_point.id() != lanelet.id()) {
    // get adjacent lanelets of current lanelet
    std::vector<lanelet::ConstLanelet> adjacent_left_lanelets = adjacentLeftOrRightLanelets(lanelet, route, true);
    std::vector<lanelet::ConstLanelet> adjacent_right_lanelets = adjacentLeftOrRightLanelets(lanelet, route, false);
    int suggested_lane_idx = adjacent_left_lanelets.size();

    // get adjacent lanelets of next lanelet (lanelet of next point)
    std::vector<lanelet::ConstLanelet> adjacent_left_lanelets_of_next_lanelet =
        adjacentLeftOrRightLanelets(lanelet_of_next_point, route, true);
    std::vector<lanelet::ConstLanelet> adjacent_right_lanelets_of_next_lanelet =
        adjacentLeftOrRightLanelets(lanelet_of_next_point, route, false);
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
      RCLCPP_ERROR(rclcpp::get_logger("lanelet2_route_planning"),
                   "Could not match following lanelets to adjacent lanelets of lanelet of next point");
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
  // lane_element_msg.regulatory_element_idcs not set in global route
  lane_element_msg.following_lane_idx = 0;
  lane_element_msg.has_following_lane_idx = true;
  route_element_msg.lane_elements.push_back(lane_element_msg);

  return route_element_msg;
}

std::vector<route_planning_msgs::msg::RegulatoryElement> extractRegulatoryElements(const lanelet::ConstLanelet& lanelet,
                                                                                   const Eigen::Vector2d& point,
                                                                                   const Eigen::Vector2d& prev_point,
                                                                                   const Eigen::Vector2d& next_point) {
  std::vector<route_planning_msgs::msg::RegulatoryElement> regulatory_element_msgs;
  const auto regulatory_elements = lanelet.regulatoryElements();
  for (const auto& regulatory_element : regulatory_elements) {
    // create RegulatoryElement
    route_planning_msgs::msg::RegulatoryElement regulatory_element_msg;

    // extract effect line
    if (auto effect_line = regulatoryElementEffectLine(regulatory_element)) {
      regulatory_element_msg.effect_line = *effect_line;

      // only consider regulatory element if effect line intersects with centerline, else skip
      // TODO: check if we are actually filling the msg with 3d values, whenever possible, and only use 2d for checking stuff
      std::vector<Eigen::Vector2d> effect_line_2d = {toEigen2d(effect_line->at(0)), toEigen2d(effect_line->at(1))};
      std::vector<Eigen::Vector2d> line_to_next_point = {point, next_point};
      std::vector<Eigen::Vector2d> line_to_prev_point = {point, prev_point};
      if (auto result = intersectionOfLines(effect_line_2d, line_to_next_point)) {
        if (!result->intersects_line2) {
          if (auto inner_result = intersectionOfLines(effect_line_2d, line_to_prev_point)) {
            if (!inner_result->intersects_line2) {
              continue;
            }
          }
        }
      }
    } else {
      RCLCPP_WARN(rclcpp::get_logger("lanelet2_route_planning"),
                  "Failed to extract reference line of regulatory element '%ld' on lanelet '%ld', ignoring",
                  regulatory_element->id(), lanelet.id());
      continue;
    }

    // extract sign positions and type
    regulatory_element_msg.sign_positions = regulatoryElementSignPositions(regulatory_element);
    std::tie(regulatory_element_msg.type, regulatory_element_msg.meta_value) =
        regulatoryElementType(regulatory_element);

    regulatory_element_msgs.push_back(regulatory_element_msg);
  }

  return regulatory_element_msgs;
}

std::optional<std::array<geometry_msgs::msg::Point, 2>> regulatoryElementEffectLine(
    const std::shared_ptr<const lanelet::RegulatoryElement>& regulatory_element) {
  const std::vector<lanelet::ConstLineString3d> effect_lines =
      regulatory_element->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::RefLine);
  if (effect_lines.empty()) {
    return std::nullopt;
  }
  const std::vector<Eigen::Vector3d> effect_line = effect_lines.front().basicLineString();
  if (effect_line.size() < 2) {
    return std::nullopt;
  }
  std::array<geometry_msgs::msg::Point, 2> effect_line_ros = {toRos(effect_line.front()), toRos(effect_line.back())};
  return effect_line_ros;
}

std::vector<geometry_msgs::msg::Point> regulatoryElementSignPositions(
    const std::shared_ptr<const lanelet::RegulatoryElement>& regulatory_element) {
  std::vector<geometry_msgs::msg::Point> sign_positions;
  const std::vector<lanelet::ConstLineString3d> sign_lines =
      regulatory_element->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::Refers);
  for (const auto& const_sign_line : sign_lines) {
    const std::vector<Eigen::Vector3d> sign_line = const_sign_line.basicLineString();
    for (const auto& sign_point : sign_line) {
      sign_positions.push_back(toRos(sign_point));
    }
  }
  return sign_positions;
}

std::pair<uint8_t, uint8_t> regulatoryElementType(
    const std::shared_ptr<const lanelet::RegulatoryElement>& regulatory_element) {
  uint8_t type = route_planning_msgs::msg::RegulatoryElement::TYPE_UNKNOWN;
  uint8_t meta_value = 0;
  if (regulatory_element->hasAttribute("subtype")) {
    std::string subtype = regulatory_element->attribute("subtype").value();
    if (subtype == "traffic_light") {
      type = route_planning_msgs::msg::RegulatoryElement::TYPE_TRAFFIC_LIGHT;
    } else if (subtype == "speed_limit") {
      type = route_planning_msgs::msg::RegulatoryElement::TYPE_SPEED_LIMIT;
      // meta_value = // TODO
    } else if (subtype == "right_of_way") {
      type = route_planning_msgs::msg::RegulatoryElement::TYPE_YIELD;
    } else if (subtype == "all_way_stop") {
      type = route_planning_msgs::msg::RegulatoryElement::TYPE_STOP;
    }
  }
  return {type, meta_value};
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
  if (speed_limit_info.isMandatory) {  // TODO: include this fact in RouteMsg?
    return std::round(lanelet::units::KmHQuantity(speed_limit_info.speedLimit).value());
  } else {
    return 0;
  }
}

lanelet::traffic_rules::TrafficRulesPtr getTrafficRules() {
  // TODO: what is the ika postfix? move to constant?
  // TODO: make Germany a param?
  // TODO: make vehicle type a param?
  auto location = lanelet::Locations::Germany;
  std::string vehicle_type = std::string(lanelet::Participants::Vehicle) + ":ika";
  return lanelet::traffic_rules::TrafficRulesFactory::create(location, vehicle_type);
}

std::optional<lanelet::ConstLanelet> laneletAtPoint(
    const Eigen::Vector2d& point, const lanelet::LaneletMapConstPtr& map,
    const std::optional<lanelet::traffic_rules::TrafficRulesPtr> traffic_rules) {
  // parameters for lanelet matching
  const unsigned int k_nearest_lanelets = 5;
  const double max_distance_lanelet_matching = 10.0;

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

  RCLCPP_ERROR(rclcpp::get_logger("lanelet2_route_planning"),
               "No passable lanelet within %.3fm of given point (%.3f, %.3f)", max_distance_lanelet_matching, point.x(),
               point.y());
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
      next_lanelets = routing_graph->following(lanelet, false);
    } else {
      next_lanelets = routing_graph->previous(lanelet);
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

double estimateRemainingTime(const std::vector<route_planning_msgs::msg::RouteElement>& route_elements,
                             const double reference_speed) {
  double remaining_time = 0.0;
  for (size_t r = 0; r < route_elements.size() - 1; ++r) {
    const auto& route_element = route_elements[r];
    const auto& next_route_element = route_elements[r + 1];
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

}  // namespace lanelet2_route_planning
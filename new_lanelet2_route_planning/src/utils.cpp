#include <limits>

#include <lanelet2_core/utility/Units.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
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

std::vector<route_planning_msgs::msg::RegulatoryElement> extractRegulatoryElements(const lanelet::ConstLanelet& lanelet,
                                                                                   const Eigen::Vector2d& point,
                                                                                   const Eigen::Vector2d& prev_point,
                                                                                   const Eigen::Vector2d& next_point) {
  std::vector<route_planning_msgs::msg::RegulatoryElement> regulatory_element_msgs;
  // TODO: this returns super many RegElemens, e.g., 13 for lanelet 10001143
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
      RCLCPP_WARN(rclcpp::get_logger("new_lanelet2_route_planning"),
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
  // TODO: what is the ika postfix? move to constant?
  // TODO: make Germany a param?
  // TODO: make vehicle type a param?
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, std::string(lanelet::Participants::Vehicle) + ":ika");
  lanelet::traffic_rules::SpeedLimitInformation speed_limit_info = traffic_rules->speedLimit(lanelet);
  if (speed_limit_info.isMandatory) {  // TODO: include this fact in RouteMsg?
    return std::round(lanelet::units::KmHQuantity(speed_limit_info.speedLimit).value());
  } else {
    return 0;
  }
}

}  // namespace new_lanelet2_route_planning
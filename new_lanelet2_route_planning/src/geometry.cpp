#include <limits>

#include <lanelet2_core/geometry/LineString.h>
#include <rclcpp/rclcpp.hpp>

#include "new_lanelet2_route_planning/conversions.hpp"
#include "new_lanelet2_route_planning/geometry.hpp"

namespace new_lanelet2_route_planning {

std::optional<IntersectionOfLinesResult> intersectionOfLines(const std::vector<Eigen::Vector2d>& line1,
                                                             const std::vector<Eigen::Vector2d>& line2) {
  IntersectionOfLinesResult result;

  // check if lines are valid
  if (line1.size() < 2 || line2.size() < 2) {
    // TODO: how to handle logging in helper functions?
    RCLCPP_ERROR(rclcpp::get_logger("new_lanelet2_route_planning"),
                 "Lines must have no more or less than two points, found %lu and %lu points", line1.size(),
                 line2.size());
    return std::nullopt;
  }

  // define straight lines (a1 x + b1 y = c1) and (a2 x + b2 y = c2) through line points
  const double a1 = line1[1].y() - line1[0].y();
  const double b1 = line1[0].x() - line1[1].x();
  const double c1 = line1[0].x() * line1[1].y() - line1[1].x() * line1[0].y();
  const double a2 = line2[1].y() - line2[0].y();
  const double b2 = line2[0].x() - line2[1].x();
  const double c2 = line2[0].x() * line2[1].y() - line2[1].x() * line2[0].y();

  // find intersection of both lines by solving for x and y
  const double det = a1 * b2 - a2 * b1;
  if (det != 0) {
    const double x = (b2 * c1 - b1 * c2) / det;
    const double y = (a1 * c2 - a2 * c1) / det;
    result.intersection = Eigen::Vector2d(x, y);

    // check if intersection point is within line segments
    result.intersects_line1 = (x >= std::min(line1[0].x(), line1[1].x()) && x <= std::max(line1[0].x(), line1[1].x()) &&
                               y >= std::min(line1[0].y(), line1[1].y()) && y <= std::max(line1[0].y(), line1[1].y()));
    result.intersects_line2 = (x >= std::min(line2[0].x(), line2[1].x()) && x <= std::max(line2[0].x(), line2[1].x()) &&
                               y >= std::min(line2[0].y(), line2[1].y()) && y <= std::max(line2[0].y(), line2[1].y()));
  } else {
    return std::nullopt;
  }

  return result;
}

Eigen::Vector2d tangentOfPointAlongLineString(const Eigen::Vector2d& point, const Eigen::Vector2d& prev_point,
                                              const Eigen::Vector2d& next_point) {
  Eigen::Vector2d tangent;

  if (point == prev_point && point != next_point) {  // single line segment
    tangent = (next_point - point).normalized();
  } else if (point != prev_point && point == next_point) {  // single line segment
    tangent = (point - prev_point).normalized();
  } else if (point == prev_point && point == next_point) {  // single point
    RCLCPP_WARN(rclcpp::get_logger("new_lanelet2_route_planning"), "Tangent of single point is undefined");
    tangent = Eigen::Vector2d(0.0, 0.0);
  } else {  // proper two line segments with previous and next point
    const Eigen::Vector2d prev_to_point_unit = (point - prev_point).normalized();
    const Eigen::Vector2d point_to_next_unit = (next_point - point).normalized();
    tangent = (prev_to_point_unit + point_to_next_unit).normalized();
  }

  return tangent;
}

Eigen::Vector2d normalOfPointAlongLineString(const Eigen::Vector2d& point, const Eigen::Vector2d& prev_point,
                                             const Eigen::Vector2d& next_point) {
  Eigen::Vector2d tangent = tangentOfPointAlongLineString(point, prev_point, next_point);
  Eigen::Vector2d normal = Eigen::Vector2d(tangent.y(), -tangent.x());

  return normal;
}

std::vector<Eigen::Vector2d> resampleLineString(const std::vector<Eigen::Vector2d>& line_string, const double delta,
                                                double& offset) {
  std::vector<Eigen::Vector2d> resampled_line_string;

  // set initial sampling distance
  double sampling_distance = delta;
  if (offset == 0.0) {  // if no offset, start with first line point
    resampled_line_string.push_back(line_string.front());
  } else {  // else sample first point with offset != delta
    sampling_distance = offset;
  }

  // loop over all line segments
  for (size_t i = 1; i < line_string.size(); ++i) {
    // determine segment length and unit direction
    double segment_length = (line_string[i] - line_string[i - 1]).norm();
    const Eigen::Vector2d segment_direction = (line_string[i] - line_string[i - 1]).normalized();

    // sample points along segment, increasing sampling_distance by delta
    while (segment_length >= sampling_distance) {
      const Eigen::Vector2d resampled_point = line_string[i - 1] + sampling_distance * segment_direction;
      resampled_line_string.push_back(resampled_point);
      sampling_distance += delta;
    }

    // reset sampling_distance for next segment, including overshoot
    sampling_distance = sampling_distance - segment_length;
  }

  // save overshoot in outgoing offset parameter
  offset = sampling_distance;

  return resampled_line_string;
}

Eigen::Vector2d projectPointToLineString(const Eigen::Vector2d& point,
                                         const std::vector<Eigen::Vector2d>& line_string) {
  return lanelet::geometry::project(toLanelet(line_string), toLanelet(point));
}

std::optional<ProjectPointToLineStringAlongAxisResult> projectPointToLineStringAlongAxis(
    const Eigen::Vector2d& point, const Eigen::Vector2d& axis, const std::vector<Eigen::Vector2d>& line_string) {
  ProjectPointToLineStringAlongAxisResult result;
  result.found_intersection_with_line_segment = false;
  double closest_distance_to_line_segment = std::numeric_limits<double>::max();
  bool found_at_least_one_intersection = false;

  // define straight line along axis at point
  std::vector<Eigen::Vector2d> axis_line = {point, point + axis};

  // loop over line segments
  for (size_t i = 0; i < line_string.size() - 1; ++i) {
    std::vector<Eigen::Vector2d> line_segment = {line_string[i], line_string[i + 1]};

    // find intersection of axis line and line segment
    if (auto inner_result = intersectionOfLines(axis_line, line_segment)) {
      const Eigen::Vector2d& intersection = inner_result->intersection;
      const bool intersects_line_segment = inner_result->intersects_line2;
      found_at_least_one_intersection = true;
      if (intersects_line_segment) {
        result.found_intersection_with_line_segment = true;
        result.projected_point = intersection;
        break;
      } else {
        double distance_to_line_segment =
            std::min((intersection - line_string[i]).norm(), (intersection - line_string[i + 1]).norm());
        if (distance_to_line_segment < closest_distance_to_line_segment) {
          closest_distance_to_line_segment = distance_to_line_segment;
          result.projected_point = intersection;
        }
      }
    }
  }

  if (!found_at_least_one_intersection) {
    return std::nullopt;
  }

  return result;
}

std::optional<ProjectPointToLineStringAlongAxisResult> projectPointToLineStringAlongNormal(
    const Eigen::Vector2d& point, const Eigen::Vector2d& prev_point, const Eigen::Vector2d& next_point,
    const std::vector<Eigen::Vector2d>& line_string) {
  // find normal to tangent at point
  Eigen::Vector2d normal = normalOfPointAlongLineString(point, prev_point, next_point);
  if (normal == Eigen::Vector2d(0.0, 0.0)) {
    return std::nullopt;
  }

  // project current point to other line along normal to tangent
  if (auto result = projectPointToLineStringAlongAxis(point, normal, line_string)) {
    return result;
  } else {
    return std::nullopt;
  }
}

}  // namespace new_lanelet2_route_planning
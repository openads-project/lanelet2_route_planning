#pragma once

#include <optional>
#include <vector>

#include <Eigen/Core>

namespace lanelet2_route_planning {

/**
 * @brief Wraps an angle to the range [-π, π].
 *
 * @param[in] angle angle to wrap
 * @return wrapped angle [radians]
 */
double wrapAngle(const double angle);

/**
 * @brief Computes the angle between two 2D vectors.
 *
 * Angle is in the range [-π, π].
 *
 * @param[in] v1 vector 1
 * @param[in] v2 vector 2
 * @return angle between vectors [radians]
 */
double angleBetweenVectors(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2);

/**
 * @brief Return type of intersectionOfLines.
 */
struct IntersectionOfLinesResult {
  Eigen::Vector2d intersection;  ///< intersection point
  bool intersects_line1;         ///< whether intersection is on first line segment
  bool intersects_line2;         ///< whether intersection is on second line segment
};

/**
 * @brief Computes the intersection of two 2D lines.
 *
 * @param[in] line1 first line defined by two points
 * @param[in] line2 second line defined by two points
 * @return intersection point and whether intersection is on line segments
 */
std::optional<IntersectionOfLinesResult> intersectionOfLines(const std::vector<Eigen::Vector2d>& line1,
                                                             const std::vector<Eigen::Vector2d>& line2);

/**
 * @brief Computes a unit vector tangential to a point along a line string.
 *
 * The tangent is pointing in the average direction of previous to current and current to next point directions.
 *
 * @param[in] point point along line string
 * @param[in] prev_point previous point along line string
 * @param[in] next_point next point along line string
 * @return unit tangential vector
 */
Eigen::Vector2d tangentOfPointAlongLineString(const Eigen::Vector2d& point, const Eigen::Vector2d& prev_point,
                                              const Eigen::Vector2d& next_point);

/**
 * @brief Computes a unit vector normal to a point along a line string.
 *
 * The normal is normal to the tangent, see tangentOfPointAlongLineString.
 *
 * @param[in] point point along line string
 * @param[in] prev_point previous point along line string
 * @param[in] next_point next point along line string
 * @return unit normal vector
 */
Eigen::Vector2d normalOfPointAlongLineString(const Eigen::Vector2d& point, const Eigen::Vector2d& prev_point,
                                             const Eigen::Vector2d& next_point);

/**
 * @brief Resamples a line string with a constant sampling distance.
 *
 * @param[in] line line string
 * @param[in] delta sampling distance
 * @param[in,out] offset starts sampling at offset distance, returns overshoot distance
 * @return resampled line string
 */
std::vector<Eigen::Vector2d> resampleLineString(const std::vector<Eigen::Vector2d>& line_string, const double delta,
                                                double& offset);

/**
 * @brief Projects a point to the closest line segment of a line string.
 *
 * @param[in] point point
 * @param[in] line_string line string
 * @return projected point
 */
Eigen::Vector2d projectPointToLineString(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& line_string);

/**
 * @brief Return type of projectPointToLineStringAlongAxis.
 */
struct ProjectPointToLineStringAlongAxisResult {
  Eigen::Vector2d projected_point;            ///< intersection point
  bool found_intersection_with_line_segment;  ///< whether intersection is on line segment
};

/**
 * @brief Projects a point along an axis to the closest line segment of a line string.
 *
 * @param[in] point point
 * @param[in] axis axis direction
 * @param[in] line_string line string
 * @return projected point and whether intersection is on line segment
 */
std::optional<ProjectPointToLineStringAlongAxisResult> projectPointToLineStringAlongAxis(
    const Eigen::Vector2d& point, const Eigen::Vector2d& axis, const std::vector<Eigen::Vector2d>& line_string);

/**
 * @brief Projects a point to the closest line segment of a line string along the normal to the tangent at the point.
 *
 * @param[in] point point along line string
 * @param[in] prev_point previous point along line string
 * @param[in] next_point next point along line string
 * @param[in] line_string other line string to project to
 * @return projected point and whether intersection is on line segment
 */
std::optional<ProjectPointToLineStringAlongAxisResult> projectPointToLineStringAlongNormal(
    const Eigen::Vector2d& point, const Eigen::Vector2d& prev_point, const Eigen::Vector2d& next_point,
    const std::vector<Eigen::Vector2d>& line_string);

}  // namespace lanelet2_route_planning
#pragma once

#include <optional>
#include <vector>

#include <Eigen/Core>

namespace new_lanelet2_route_planning {

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

Eigen::Vector2d projectPointToLineString(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& line_string);

}  // namespace new_lanelet2_route_planning
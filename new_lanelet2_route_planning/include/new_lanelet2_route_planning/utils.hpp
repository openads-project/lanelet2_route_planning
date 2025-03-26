#pragma once

#include <cstdint>
#include <vector>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/Route.h>
#include <Eigen/Core>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <route_planning_msgs/msg/route_element.hpp>

namespace new_lanelet2_route_planning {

/**
 * @brief Find index of point in line string that is closest to another point.
 *
 * @param[in] line_string line string
 * @param[in] point other point
 * @return index of closest point
 */
size_t indexOfLineStringPointClosestToPoint(const std::vector<Eigen::Vector2d>& line_string,
                                            const Eigen::Vector2d& point);

/**
 * @brief Identifies a lane change based on the distance between two reference line points.
 *
 * Assumes that the two points are part of an equidistant reference line.
 * If the distance between the two points is greater than the sampling distance, a lane change is assumed.
 *
 * @param[in] point point on reference line
 * @param[in] next_point next point on reference line
 * @param[in] sampling_distance expected distance between points
 * @return whether a lane change is identified
 */
bool changesLaneFromPointToPoint(const Eigen::Vector2d& point, const Eigen::Vector2d& next_point,
                                 const double sampling_distance);

/**
 * @brief Create a minimal route element message.
 *
 * A minimal valid route element only has a single lane element with a reference pose.
 *
 * @param[in] position position
 * @param[in] orientation orientation
 * @param[in] s accumulated distance along route
 * @param[in] will_change_suggested_lane whether the lane will change to the next route element
 * @param[in] speed_limit speed limit
 * @return minimal route element
 */
route_planning_msgs::msg::RouteElement createMinimalRouteElement(const geometry_msgs::msg::Point& position,
                                                                 const geometry_msgs::msg::Quaternion& orientation,
                                                                 double s = 0.0,
                                                                 bool will_change_suggested_lane = false,
                                                                 uint8_t speed_limit = 0);

/**
 * @brief Finds lanelets adjacent to the left or right of a given lanelet.
 *
 * RelationType::Left and RelationType::AdjacentLeft are used for left adjacent lanelets, right vice versa.
 *
 * @param[in] lanelet lanelet
 * @param[in] route route
 * @param[in] left whether to find left or right adjacent lanelets
 * @return adjacent lanelets
 */
std::vector<lanelet::ConstLanelet> adjacentLeftOrRightLanelets(const lanelet::ConstLanelet& lanelet,
                                                               const lanelet::routing::Route& route, bool left);

/**
 * @brief Projected lanelet points.
 */
struct ProjectedLaneletPoints {
  Eigen::Vector2d left_bound_point;   ///< projected left bound point
  Eigen::Vector2d centerline_point;   ///< projected centerline point
  Eigen::Vector2d right_bound_point;  ///< projected right bound point
};

/**
 * @brief Projects a point to the centerline and bounds of a set of lanelets.
 *
 * For projection, projectPointToLineStringAlongNormal is used.
 *
 * @param[in] point point
 * @param[in] prev_point previous point
 * @param[in] next_point next point
 * @param[in] lanelets lanelets
 * @return projected points
 */
std::vector<ProjectedLaneletPoints> projectPointToLaneletLines(const Eigen::Vector2d& point,
                                                               const Eigen::Vector2d& prev_point,
                                                               const Eigen::Vector2d& next_point,
                                                               const std::vector<lanelet::ConstLanelet>& lanelets);

}  // namespace new_lanelet2_route_planning
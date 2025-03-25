#pragma once

#include <cstdint>
#include <vector>

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

}  // namespace new_lanelet2_route_planning
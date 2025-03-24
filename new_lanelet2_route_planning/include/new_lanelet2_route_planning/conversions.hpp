#pragma once

#include <vector>

#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <Eigen/Core>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <perception_msgs/msg/ego_data.hpp>

namespace new_lanelet2_route_planning {

/**
 * @brief Converts a 3D Eigen point to a 2D Eigen point.
 *
 * Removes the z-coordinate.
 *
 * @param[in] point point
 * @return converted point
 */
Eigen::Vector2d as2d(const Eigen::Vector3d& point);

/**
 * @brief Converts a 2D Eigen point to a 3D Eigen point.
 *
 * Adds a z-coordinate of 0.
 *
 * @param[in] point point
 * @return converted point
 */
Eigen::Vector3d as3d(const Eigen::Vector2d& point);

/**
 * @brief Converts a vector of 3D Eigen points to a vector of 2D Eigen points.
 *
 * Removes the z-coordinate.
 *
 * @param[in] points points
 * @return converted points
 */
std::vector<Eigen::Vector2d> as2d(const std::vector<Eigen::Vector3d>& points);

/**
 * @brief Converts a vector of 2D Eigen points to a vector of 3D Eigen points.
 *
 * Adds a z-coordinate of 0.
 *
 * @param[in] points points
 * @return converted points
 */
std::vector<Eigen::Vector3d> as3d(const std::vector<Eigen::Vector2d>& points);

/**
 * @brief Converts a 2D Eigen point to a ROS point.
 *
 * @param[in] point point
 * @return converted point
 */
geometry_msgs::msg::Point pointAsRos(const Eigen::Vector2d& point);

/**
 * @brief Converts a 3D Eigen point to a ROS point.
 *
 * @param[in] point point
 * @return converted point
 */
geometry_msgs::msg::Point pointAsRos(const Eigen::Vector3d& point);

/**
 * @brief Converts a 2D Lanelet point to a ROS point.
 *
 * @param[in] point point
 * @return converted point
 */
geometry_msgs::msg::Point pointAsRos(const lanelet::BasicPoint2d& point);

/**
 * @brief Converts a 2D Eigen point to a Lanelet point.
 *
 * @param[in] point point
 * @return converted point
 */
lanelet::BasicPoint2d pointAsLanelet(const Eigen::Vector2d& point);

/**
 * @brief Converts a 3D Eigen point to a Lanelet point.
 *
 * @param[in] point point
 * @return converted point
 */
lanelet::BasicPoint3d pointAsLanelet(const Eigen::Vector3d& point);

/**
 * @brief Converts a ROS point to a 2D Eigen point.
 *
 * @param[in] point point
 * @return converted point
 */
Eigen::Vector2d pointAsEigen2d(const geometry_msgs::msg::Point& point);

/**
 * @brief Converts a ROS point to a 3D Eigen point.
 *
 * @param[in] point point
 * @return converted point
 */
Eigen::Vector3d pointAsEigen(const geometry_msgs::msg::Point& point);

/**
 * @brief Extracts an EgoData position as a ROS point.
 *
 * @param[in] ego_data ego data
 * @return ego data position
 */
geometry_msgs::msg::Point position(const perception_msgs::msg::EgoData& ego_data);

/**
 * @brief Converts a 2D Lanelet line string to a vector of Eigen points.
 *
 * @param[in] line_string line string
 * @return converted line string
 */
std::vector<Eigen::Vector2d> lineStringAsEigen2d(const lanelet::BasicLineString2d& line_string);

/**
 * @brief Converts a 3D Lanelet line string to a vector of Eigen points.
 *
 * @param[in] line_string line string
 * @return converted line string
 */
std::vector<Eigen::Vector3d> lineStringAsEigen(const lanelet::BasicLineString3d& line_string);

/**
 * @brief Converts a vector of 2D Eigen points to a Lanelet line string.
 *
 * @param[in] line_string line string
 * @return converted line string
 */
lanelet::BasicLineString2d lineStringAsLanelet(const std::vector<Eigen::Vector2d>& line_string);

/**
 * @brief Converts a 2D Eigen vector pointing in a specific direction to a ROS quaternion.
 *
 * The vector is assumed to lie in the xy-plane.
 *
 * @param[in] vector vector
 * @return converted quaternion
 */
geometry_msgs::msg::Quaternion vectorAsRosQuaternion(const Eigen::Vector2d& vector);

}  // namespace new_lanelet2_route_planning
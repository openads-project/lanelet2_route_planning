#include <perception_msgs_utils/object_access.hpp>

#include "new_lanelet2_route_planning/conversions.hpp"

namespace new_lanelet2_route_planning {

geometry_msgs::msg::Point pointAsRos(const Eigen::Vector2d &point) {
  return pointAsRos(Eigen::Vector3d(point.x(), point.y(), 0.0));
}

geometry_msgs::msg::Point pointAsRos(const Eigen::Vector3d &point) {
  geometry_msgs::msg::Point ros_point;
  ros_point.x = point.x();
  ros_point.y = point.y();
  ros_point.z = point.z();
  return ros_point;
}

geometry_msgs::msg::Point pointAsRos(const lanelet::BasicPoint2d &point) {
  return pointAsRos(Eigen::Vector2d(point.x(), point.y()));
}

Eigen::Vector2d pointAsEigen2d(const geometry_msgs::msg::Point &point) { return Eigen::Vector2d(point.x, point.y); }

Eigen::Vector3d pointAsEigen(const geometry_msgs::msg::Point &point) {
  return Eigen::Vector3d(point.x, point.y, point.z);
}

geometry_msgs::msg::Point position(const perception_msgs::msg::EgoData &ego_data) {
  geometry_msgs::msg::Point position;
  position.x = perception_msgs::object_access::getX(ego_data);
  position.y = perception_msgs::object_access::getY(ego_data);
  position.z = perception_msgs::object_access::getZ(ego_data);
  return position;
}

std::vector<Eigen::Vector2d> lineStringAsEigen(const lanelet::BasicLineString2d &line_string) {
  return std::vector<Eigen::Vector2d>(line_string.begin(), line_string.end());
}

geometry_msgs::msg::Quaternion vectorAsRosQuaternion(const Eigen::Vector2d &vector) {
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

}  // namespace new_lanelet2_route_planning
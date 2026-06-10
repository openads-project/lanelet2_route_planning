#include <perception_msgs_utils/object_access.hpp>
#include <route_planning_msgs_utils/route_access.hpp>

#include "lanelet2_route_planning/conversions.hpp"

namespace lanelet2_route_planning {

Eigen::Vector2d to2d(const Eigen::Vector3d &point) { return point.head<2>(); }

Eigen::Vector3d to3d(const Eigen::Vector2d &point) { return Eigen::Vector3d(point.x(), point.y(), 0.0); }

Eigen::Vector3d to3d(const Eigen::Vector2d &point, double z) { return Eigen::Vector3d(point.x(), point.y(), z); }

std::vector<Eigen::Vector2d> to2d(const std::vector<Eigen::Vector3d> &points) {
  std::vector<Eigen::Vector2d> points_2d;
  for (const auto &point : points) {
    points_2d.push_back(to2d(point));
  }
  return points_2d;
}

std::vector<Eigen::Vector3d> to3d(const std::vector<Eigen::Vector2d> &points) {
  std::vector<Eigen::Vector3d> points_3d;
  for (const auto &point : points) {
    points_3d.push_back(to3d(point));
  }
  return points_3d;
}

geometry_msgs::msg::Point toRos(const Eigen::Vector2d &point) { return toRos(to3d(point)); }

geometry_msgs::msg::Point toRos(const Eigen::Vector3d &point) {
  geometry_msgs::msg::Point ros_point;
  ros_point.x = point.x();
  ros_point.y = point.y();
  ros_point.z = point.z();
  return ros_point;
}

geometry_msgs::msg::Point toRos(const lanelet::BasicPoint2d &point) {
  return toRos(Eigen::Vector2d(point.x(), point.y()));
}

lanelet::BasicPoint2d toLanelet(const Eigen::Vector2d &point) { return lanelet::BasicPoint2d(point.x(), point.y()); }

lanelet::BasicPoint3d toLanelet(const Eigen::Vector3d &point) { return point; }

Eigen::Vector2d toEigen2d(const geometry_msgs::msg::Point &point) { return Eigen::Vector2d(point.x, point.y); }

Eigen::Vector3d toEigen(const geometry_msgs::msg::Point &point) { return Eigen::Vector3d(point.x, point.y, point.z); }

geometry_msgs::msg::Point egoPosition(const perception_msgs::msg::EgoData &ego_data) {
  geometry_msgs::msg::Point position;
  position.x = perception_msgs::object_access::getX(ego_data);
  position.y = perception_msgs::object_access::getY(ego_data);
  position.z = perception_msgs::object_access::getZ(ego_data);
  return position;
}

std::vector<Eigen::Vector2d> toEigen(const lanelet::BasicLineString2d &line_string) {
  return std::vector<Eigen::Vector2d>(line_string.begin(), line_string.end());
}

std::vector<Eigen::Vector3d> toEigen(const lanelet::BasicLineString3d &line_string) {
  return std::vector<Eigen::Vector3d>(line_string.begin(), line_string.end());
}

lanelet::BasicLineString2d toLanelet(const std::vector<Eigen::Vector2d> &line_string) {
  lanelet::BasicLineString2d lanelet_line_string;
  for (const auto &point : line_string) {
    lanelet_line_string.push_back(toLanelet(point));
  }
  return lanelet_line_string;
}

lanelet::BasicLineString3d toLanelet(const std::vector<Eigen::Vector3d> &line_string) {
  lanelet::BasicLineString3d lanelet_line_string;
  for (const auto &point : line_string) {
    lanelet_line_string.push_back(toLanelet(point));
  }
  return lanelet_line_string;
}

geometry_msgs::msg::Quaternion toRosQuaternion(const Eigen::Vector2d &vector) {
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

std::vector<Eigen::Vector3d> suggestedReferenceLineToEigen(
    const std::vector<route_planning_msgs::msg::RouteElement> &route_elements) {
  std::vector<Eigen::Vector3d> reference_line;
  for (const auto &route_element : route_elements) {
    const auto &lane_element = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
    reference_line.push_back(toEigen(lane_element.reference_pose.position));
  }
  return reference_line;
}

}  // namespace lanelet2_route_planning
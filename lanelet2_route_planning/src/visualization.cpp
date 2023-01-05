#include "lanelet2_route_planning/global_planner_node.hpp"

visualization_msgs::msg::Marker GlobalPlanner::convertDestination2Marker(double target_x, double target_y, std::string frame_id)
{
  geometry_msgs::msg::Point point1;
  geometry_msgs::msg::Point point2;

  visualization_msgs::msg::Marker destination_marker;
  destination_marker.header.frame_id = frame_id;
  destination_marker.header.stamp = now();
  destination_marker.ns = "destination";
  destination_marker.type = visualization_msgs::msg::Marker::ARROW;
  destination_marker.action = visualization_msgs::msg::Marker::ADD;
  destination_marker.color.a = 1.0;
  destination_marker.color.r = 0.99;
  destination_marker.color.g = 0.2;
  destination_marker.color.b = 0.2;

  point1.x = target_x;
  point1.y = target_y;
  point1.z = 0.;
  point2.x = target_x;
  point2.y = target_y;
  point2.z = 3.0;

  destination_marker.points.push_back(point2);
  destination_marker.points.push_back(point1);

  destination_marker.scale.x = 0.8; // shaft diameter
  destination_marker.scale.y = 1.5; // head diameter

  return destination_marker;
}


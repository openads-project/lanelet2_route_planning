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

void GlobalPlanner::visualizeLinestring(std::vector<geometry_msgs::msg::Point>& line_string, const std::string& desc, visualization_msgs::msg::MarkerArray& marker_array, std::vector<float> colors)
{
  colors.resize(4);
  visualization_msgs::msg::Marker marker;
  colors[3] = 0.8;
  marker = Lanelet2Utilities::convertLinestring2VisuLineStrip(line_string, ll2if_->map_frame_id_, now(), desc, colors, 0.1);
  marker_array.markers.push_back(marker);
  colors[3] = 1.0;
  marker = Lanelet2Utilities::convertLinestring2VisuSphere(line_string, ll2if_->map_frame_id_, now(), desc + " points", colors);
  marker_array.markers.push_back(marker);
}


void GlobalPlanner::visualizeIndexMapping(visualization_msgs::msg::Marker& marker, visualization_msgs::msg::MarkerArray& marker_array, const lanelet::BasicLineString2d& bound, const std::string& left_right_string, const std::string& ns, const std::vector<int>& index_mapping)
{
  if (visualize_lvl_ > 1)
  {
    // Index mapping
    unsigned int index = 0;
    for (auto &p : bound)
    {
      marker.points.clear();

      marker.header.frame_id = ll2if_->map_frame_id_;
      marker.header.stamp = now();
      marker.ns = ns + "_" + left_right_string;
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.text = std::to_string(index);

      marker.scale.z = 0.1;

      marker.color.r = 1;
      marker.color.g = 1;
      marker.color.b = 1;

      marker.pose.position.x = p.x();
      marker.pose.position.y = p.y();
      marker.pose.position.z = 1.0;

      marker.id = index++;
      marker_array.markers.push_back(marker);
    }

    // Index mapping
    index = 0;
    for (auto &p : bound)
    {
      marker.points.clear();

      marker.header.frame_id = ll2if_->map_frame_id_;
      marker.header.stamp = now();
      marker.ns = ns + "_mapping_" + left_right_string;
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.text = std::to_string(index_mapping[index]);

      marker.scale.z = 0.1;

      marker.color.r = 0;
      marker.color.g = 1;
      marker.color.b = 0;

      marker.pose.position.x = p.x();
      marker.pose.position.y = p.y();
      marker.pose.position.z = 1.25;

      marker.id = index++;
      marker_array.markers.push_back(marker);
    }
  }
}



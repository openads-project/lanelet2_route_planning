#include "lanelet2_route_planning/global_planner_node.hpp"

// Convert to linestring, smooth and visualize
void GlobalPlanner::processLineString(lanelet::BasicLineString2d& line_string, const std::string& desc, visualization_msgs::msg::MarkerArray& marker_array, std::vector<float> colors, std::vector<float> colors_smoothed)
{
  colors.resize(4);
  colors_smoothed.resize(4);

  // Visualize
  if (false)//visualize_lvl_ > 1)
  {
    visualization_msgs::msg::Marker marker;
    colors[3] = 0.8;
    Lanelet2Utilities::convertLaneletLine2VisuLineStrip(line_string, marker, "map", now(), desc, colors, 0.1);
    marker_array.markers.push_back(marker);
    colors[3] = 1.0;
    Lanelet2Utilities::convertLaneletLine2VisuSphere(line_string, marker, "map", now(), desc + " points", colors);
    marker_array.markers.push_back(marker);

    // Index
    if (false)//visualize_lvl_ > 1)
    {
      colors_smoothed[3] = 1.0;
      Lanelet2Utilities::convertLaneletLine2VisuSphere(line_string, marker, "map", now(), desc + " points", colors_smoothed);
      marker_array.markers.push_back(marker);

      unsigned int index = 0;
      for (auto &p : line_string)
      {
          marker.points.clear();
          marker.header.frame_id = "map";
          marker.header.stamp = now();
          marker.ns = "lvl2_point_index_" + desc;
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
    }
  }

  // Smooth
  line_string = Lanelet2Utilities::smoothByQuadraticBezierCurve(line_string, smooth_factor_);

  // Visualize
  if (true)//visualize_lvl_)
  {
    visualization_msgs::msg::Marker marker;
    colors_smoothed[3] = 0.8;
    Lanelet2Utilities::convertLaneletLine2VisuLineStrip(line_string, marker, "map", now(), desc + " (smoothed)",  colors_smoothed, 0.1);
    marker_array.markers.push_back(marker);

    // Index
    if (false)//visualize_lvl_ > 1)
    {
      colors_smoothed[3] = 1.0;
      Lanelet2Utilities::convertLaneletLine2VisuSphere(line_string, marker, "map", now(), desc + " points (smoothed)", colors_smoothed);
      marker_array.markers.push_back(marker);

      unsigned int index = 0;
      for (auto &p : line_string)
      {
          marker.points.clear();
          marker.header.frame_id = "map";
          marker.header.stamp = now();
          marker.ns = "lvl2_point_index_" + desc + " (smoothed)";
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
    }
  }
}



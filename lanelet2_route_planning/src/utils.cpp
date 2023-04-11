#include "lanelet2_route_planning/global_planner_node.hpp"

// Convert to linestring, smooth and visualize
void GlobalPlanner::processLineString(lanelet::BasicLineString2d& line_string, const std::string& desc, visualization_msgs::msg::MarkerArray& marker_array, std::vector<float> colors, std::vector<float> colors_smoothed)
{
  // to Linestring
  std::vector<geometry_msgs::msg::Point> points = Lanelet2Utilities::convertLaneletLine2Linestring(line_string);
  // Visualize
  visualizeLinestring(points, desc, marker_array, colors);
  // Smooth
  line_string = Lanelet2Utilities::smoothByQuadraticBezierCurve(line_string, smooth_factor_);
  // Visualize
  visualizeLinestring(points, desc+" smoothed", marker_array, colors_smoothed);
  // to Linestring
  points = Lanelet2Utilities::convertLaneletLine2Linestring(line_string);
}

bool GlobalPlanner::handleInwardCorner(const lanelet::BasicPoint2d &base_p, lanelet::BasicPoint2d& best_point, const std::pair<lanelet::BasicLineString2d, size_t>*& last_intersection_free_test_line, lanelet::BasicLineString2d& previous_test_line,
                                       const uint& idx, std::deque<std::pair<lanelet::BasicLineString2d, size_t>>& last_test_lines, lanelet::BasicLineString2d& bound, std::vector<int>& index_mapping)
{
  const size_t max_queue_size = 30;

  lanelet::BasicLineString2d test_line_cut({base_p, best_point});
  if(last_intersection_free_test_line)
  {
    // Check if this line still intersects the last intersection free test line
    if(boost::geometry::intersects(test_line_cut, last_intersection_free_test_line->first))
    {
      // It does, don't add anything.
      previous_test_line = test_line_cut;
      return false;
    }
    else
    {
      // It does not; now check if the previous test line intersects any of the buffered ones (find start of curve)
      if(idx > 0)
      {
        for(auto& last_test_line : last_test_lines)
        {
          if(boost::geometry::intersects(previous_test_line, last_test_line.first)) // Check full length
          {
            // Oh yeah; this is the boundary sample of the start of the curve
            // Delete the samples we have added since then
            const int amount_to_delete = last_intersection_free_test_line->second - last_test_line.second + 1;
            if(amount_to_delete > 0 && static_cast<int>(bound.size()) >= amount_to_delete)
            {
              bound.resize(bound.size()-amount_to_delete);
              index_mapping.resize(index_mapping.size()-amount_to_delete);
            }
            break;
          }
        }
      }

      // Continue normally, this is the boundary sample at the end of the curve
      last_intersection_free_test_line = nullptr;
      last_test_lines.clear();
    }
  }
  else if(last_test_lines.size() > 0)
  {
    // Check if this test line is intersecting the last buffered one
    const auto& last_test_line = last_test_lines.back();
    if(boost::geometry::intersects(test_line_cut, last_test_line.first))
    {
      // It does; save last valid one and continue
      last_intersection_free_test_line = &last_test_line;
      previous_test_line = last_test_line.first;
      return false;
    }
  }

  // Valid test line, add to buffer
  last_test_lines.push_back(std::make_pair(test_line_cut, idx));
  if(last_test_lines.size() > max_queue_size) last_test_lines.pop_front();

  return true;
}

bool GlobalPlanner::checkLineDrivability(const lanelet::ConstLineString3d &lineToCheck)
{
  lanelet::Attribute type_str;
  lanelet::Attribute subtype_str;
  if (lineToCheck.hasAttribute("type") == false)
  {
    return false; // no type detectable, therefore for safety reasons don't look any further this direction
  }
  if (lineToCheck.hasAttribute("subtype") == false)
  {
    subtype_str = Attribute("high"); // no subtype detectable, therefore for safety reasons set to "high"
  }
  else
  {
    subtype_str = lineToCheck.attribute("subtype");
  }

  type_str = lineToCheck.attribute("type");

  if (type_str == "line_thin" ||
      type_str == "line_thick" ||
      type_str == "virtual" ||
      type_str == "zebra_marking" ||
      type_str == "bike_marking" ||
      type_str == "pedestrian_marking" ||
      type_str == "stop_line" ||
      type_str == "traffic_light" ||
      type_str == "roadpainting" || //Atlatec Maps
      type_str == "lane_center" || //Atlatec Maps
      (type_str == "curbstone" && subtype_str == "low") )
  {
    return true;
  }

  return false;
}

#include "lanelet2_route_planning/global_planner_node.hpp"

// Convert to linestring, smooth and visualize
std::vector<geometry_msgs::msg::Point> GlobalPlanner::processLineString(lanelet::BasicLineString2d& line_string, const std::string& desc, visualization_msgs::msg::MarkerArray& marker_array, std::vector<float> colors, std::vector<float> colors_smoothed)
{
  // to Linestring
  std::vector<geometry_msgs::msg::Point> points = Lanelet2Utilities::convertLaneletLine2Linestring(line_string);
  // Visualize
  visualizeLinestring(points, ll2if_->map_frame_id_, desc, marker_array, colors);
  // Smooth
  line_string = Lanelet2Utilities::smoothByQuadraticBezierCurve(line_string, smooth_factor_);
  // Visualize
  visualizeLinestring(points, ll2if_->map_frame_id_, desc+" smoothed", marker_array, colors_smoothed);
  // to Linestring
  points = Lanelet2Utilities::convertLaneletLine2Linestring(line_string);
  return points;
}

// lanelet::BasicLineString2d GlobalPlanner::sampleBoundaries(const lanelet::BasicLineString2d &centerline,
//                                                   const double test_dis,
//                                                   const bool &b_right,
//                                                   std::vector<int>& index_mapping,
//                                                   const lanelet::BasicLineString2d& lane_boundary,
//                                                   visualization_msgs::msg::MarkerArray& marker_array)
// {
//   double test_dis_left_right = test_dis;
//   double factor_left_right = 1.0;
//   std::string left_right_string = "left";
//   std::vector<float> left_right_colors = {0.95, 0.25, 0.25, 1.0};
//   if (b_right)
//   {
//     test_dis_left_right *= -1.;
//     factor_left_right *= -1.;
//     left_right_string = "right";
//     left_right_colors = {0.25, 0.25, 0.95, 1.0};
//   }

//   const size_t max_queue_size = 30;
//   std::deque<std::pair<lanelet::BasicLineString2d, size_t>> last_test_lines; // Test line till boundary sample, full length test line, index
//   lanelet::BasicLineString2d previous_test_line; // Full length
//   const std::pair<lanelet::BasicLineString2d, size_t>* last_intersection_free_test_line = nullptr;
//   lanelet::BasicLineString2d bound;

//   // For visualization
//   size_t ids_final_point = 0;
//   size_t ids_test_line = 0;
//   size_t ids_final_bound = 0;

//   // Process route
//   for (uint idx = 0; idx < centerline.size(); idx++)
//   {
//     const lanelet::BasicPoint2d &base_p = centerline.at(idx);
//     lanelet::BasicPoint2d best_point = geometry::internal::lateralShiftPointAtIndex(centerline, idx, test_dis_left_right);
//     const lanelet::BasicLineString2d test_line({base_p, best_point});

//     // Get all intersecting points
//     lanelet::BasicPoints2d interpoints;
//     boost::geometry::intersection(lane_boundary, test_line, interpoints);

//     // Sort according to distance
//     std::vector<std::pair<double, lanelet::BasicPoint2d> > all_interpoints;
//     for (const BasicPoint2d &poi : interpoints)
//     {
//       all_interpoints.emplace_back(geometry::distance(base_p, poi), poi);
//     }
//     std::sort(all_interpoints.begin(), all_interpoints.end(),
//               [](auto const &t1, auto const &t2) {
//                 return t1.first < t2.first;
//               });
//     if(interpoints.size())
//     {
//       best_point = all_interpoints.front().second;
//     }

//     // Special handling for inward corners
//     if(!handleInwardCorner(base_p, best_point, last_intersection_free_test_line, previous_test_line, idx, last_test_lines, bound, index_mapping))
//     {
//       continue;
//     }

//     // Add final point to boundary samples
//     bound.push_back(best_point);
//     index_mapping.push_back(idx);

//     // Visualize final point and test line
//     if (visualize_lvl_ > 1)
//     {
//       visualization_msgs::msg::Marker marker;

//       // Test line
//       left_right_colors[3] = 0.5;
//       Lanelet2Utilities::convertLaneletLine2VisuLineStrip(test_line, marker, ll2if_->map_frame_id_, now(), "lvl2_boundary_test_line_" + left_right_string, left_right_colors);
//       marker.id = ids_test_line++;
//       marker_array.markers.push_back(marker);

//       // Final point
//       lanelet::BasicLineString2d outerpoint;
//       outerpoint.push_back(best_point);
//       left_right_colors[3] = 1.0;
//       Lanelet2Utilities::convertLaneletLine2VisuSphere(outerpoint, marker, ll2if_->map_frame_id_, now(), "lvl2_boundary_points_" + left_right_string, left_right_colors);
//       marker.id = ids_final_point++;
//       marker_array.markers.push_back(marker);
//     }
//   }

//   // Visualize final boundary
//   if (visualize_lvl_ > 0)
//   {
//     visualization_msgs::msg::Marker marker;

//     if (visualize_lvl_ > 1)
//     {
//       left_right_colors[3] = 0.5;
//       Lanelet2Utilities::convertLaneletLine2VisuLineStrip(lane_boundary, marker, ll2if_->map_frame_id_, now(), "lvl1_boundary_raw_" + left_right_string, left_right_colors, 0.25);
//       marker.id = ids_final_bound++;
//       marker_array.markers.push_back(marker);
//     }

//     left_right_colors[3] = 1.0;
//     Lanelet2Utilities::convertLaneletLine2VisuLineStrip(bound, marker, ll2if_->map_frame_id_, now(), "lvl1_boundary_final_" + left_right_string, left_right_colors, 0.25);
//     marker.id = ids_final_bound++;
//     marker_array.markers.push_back(marker);

//     visualizeIndexMapping(marker, marker_array, bound, left_right_string, "lvl2_boundary_index", index_mapping);
//   }
//   return bound;
// }

lanelet2_route_planning_interfaces::msg::DriveableSpace GlobalPlanner::sampleDriveableSpace(
                                                        const lanelet::BasicLineString2d &centerline,
                                                        visualization_msgs::msg::MarkerArray& marker_array)
{
  lanelet2_route_planning_interfaces::msg::DriveableSpace driveable_space;
  driveable_space.header.frame_id = ll2if_->map_frame_id_;
  driveable_space.header.stamp = now();
  driveable_space.boundaries.left = sampleLinestring(centerline, 10.0, false);
  driveable_space.boundaries.right = sampleLinestring(centerline, 10.0, true);
  visualizeLinestring(driveable_space.boundaries.left, driveable_space.header.frame_id, "driveable space left", marker_array, {1.0, 0.53, 0.0, 1.0});
  visualizeLinestring(driveable_space.boundaries.right, driveable_space.header.frame_id, "driveable space right", marker_array, {1.0, 0.53, 0.0, 1.0});
  return driveable_space;
}

std::vector<geometry_msgs::msg::Point> GlobalPlanner::sampleLinestring(
                                          const lanelet::BasicLineString2d &centerline,
                                          const double test_dis,
                                          bool b_right)
{
  double test_dis_left_right = test_dis;
  double factor_left_right = 1.0;
  if (b_right)
  {
    test_dis_left_right *= -1.;
    factor_left_right *= -1.;

  }

  std::deque<std::pair<lanelet::BasicLineString2d, size_t>> last_test_lines; // Test line till drivable space sample, full length test line, index
  lanelet::BasicLineString2d previous_test_line; // Full length
  const std::pair<lanelet::BasicLineString2d, size_t>* last_intersection_free_test_line = nullptr;
  lanelet::BasicLineString2d ll_bound;

  // Process route
  for (uint idx = 0; idx < centerline.size(); idx++)
  {
    const lanelet::BasicPoint2d &base_p = centerline.at(idx);
    const lanelet::BasicPoint2d test_p = lanelet::geometry::internal::lateralShiftPointAtIndex(centerline, idx, test_dis_left_right);
    const lanelet::BasicLineString2d test_line({base_p, test_p});

    // Get all intersecting points
    std::vector<std::tuple<double, lanelet::BasicPoint2d, long>> all_interpoints; // signed distance, point, id of line
    std::vector<std::pair<double, lanelet::ConstLineString3d>> near_lines = lanelet::geometry::findWithin2d(llmap_->lineStringLayer, test_line, 5.0);
    for (const auto &line_to_test : near_lines)
    {
      lanelet::BasicPoints2d interpoints;
      boost::geometry::intersection(utils::to2D(line_to_test.second).basicLineString(), test_line, interpoints);

      for (const lanelet::BasicPoint2d &poi : interpoints)
      {
        all_interpoints.emplace_back(lanelet::geometry::distance(base_p, poi), poi, line_to_test.second.id());
      }
    }

    // Sort according to distance
    std::sort(all_interpoints.begin(), all_interpoints.end(),
              [](auto const &t1, auto const &t2) {
                return std::get<0>(t1) < std::get<0>(t2);
              });

    // Visualize these intersecting points and check for drivability
    lanelet::BasicPoint2d best_point = test_p;
    for (uint i = 0; i < all_interpoints.size(); i++)
    {
      // Intersection with non-drivable line?
      const lanelet::ConstLineString3d &line = llmap_->lineStringLayer.get(std::get<2>(all_interpoints.at(i)));
      if (checkLineDrivability(line) == false)
      {
        best_point = std::get<1>(all_interpoints.at(i));
        break;
      }
    }

    // Special handling for inward corners
    if(!handleInwardCorner(base_p, best_point, last_intersection_free_test_line, previous_test_line, idx, last_test_lines, ll_bound))
    {
      continue;
    }

    // Add final point to samples
    ll_bound.push_back(best_point);
  }
  // Convert to std::vector<geometry_msgs::msg::Point>
  std::vector<geometry_msgs::msg::Point> bound = Lanelet2Utilities::convertLaneletLine2Linestring(ll_bound);
  return bound; 
}

bool GlobalPlanner::handleInwardCorner(const lanelet::BasicPoint2d &base_p, lanelet::BasicPoint2d& best_point,
                                        const std::pair<lanelet::BasicLineString2d, size_t>*& last_intersection_free_test_line,
                                        lanelet::BasicLineString2d& previous_test_line, const uint& idx,
                                        std::deque<std::pair<lanelet::BasicLineString2d, size_t>>& last_test_lines, lanelet::BasicLineString2d& bound)
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

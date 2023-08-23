#include "lanelet2_route_planning/global_planner_node.hpp"

// Convert to linestring, smooth and visualize
std::vector<geometry_msgs::msg::Point> GlobalPlanner::processLineString(lanelet::BasicLineString2d& line_string)
{
  // to Linestring
  std::vector<geometry_msgs::msg::Point> points = Lanelet2Utilities::convertLaneletLine2Linestring(line_string);
  // Smooth
  line_string = Lanelet2Utilities::smoothByQuadraticBezierCurve(line_string, smooth_factor_);
  // to Linestring
  points = Lanelet2Utilities::convertLaneletLine2Linestring(line_string);
  return points;
}

route_planning_interfaces::msg::DriveableSpace GlobalPlanner::sampleDriveableSpace(const lanelet::BasicLineString2d &centerline)
{
  route_planning_interfaces::msg::DriveableSpace driveable_space;
  driveable_space.header.frame_id = ll2if_->map_frame_id_;
  driveable_space.header.stamp = now();
  driveable_space.boundaries.left = sampleLinestring(centerline, lateral_driv_space_width_/2.0, false);
  driveable_space.boundaries.right = sampleLinestring(centerline, lateral_driv_space_width_/2.0, true);
  return driveable_space;
}

std::vector<geometry_msgs::msg::Point> GlobalPlanner::sampleLinestring(const lanelet::BasicLineString2d &centerline,
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

void GlobalPlanner::sampleRouteBoundary(const lanelet::routing::Route &route,
                                        const lanelet::routing::LaneletPath &shortest_path,
                                        std::vector<geometry_msgs::msg::Point> &bound_left,
                                        std::vector<geometry_msgs::msg::Point> &bound_right)
{
  //for current and flowing lanelets
  for(int i=0; i<shortest_path.size(); i++)
  {
    
    lanelet::ConstLanelet cur_ll = shortest_path[i];
    lanelet::ConstLanelet outer_left_ll=cur_ll;
    lanelet::ConstLanelet outer_right_ll=cur_ll;
    
    bool left_is_present=true;
    bool right_is_present=true;

    //prove of routable lane on lefthandside
    while(left_is_present)
    {
      Optional<lanelet::ConstLanelet> left{routingGraph_->left(outer_left_ll)}; // Get routable left lanelet if it exists
      left_is_present=left.has_value();
      if(left_is_present)
      {
        outer_left_ll=left.get(); //get outer left lanelet
      }
    }

    //prove of routable lane on righthandside
    while(right_is_present) 
    {
      Optional<lanelet::ConstLanelet> right{routingGraph_->right(outer_right_ll)}; // Get routable right lanelet if it exists
      right_is_present=right.has_value();
      if(right_is_present)
      {
        outer_right_ll=right.get(); //get outer right lanelet
      }
    }

    //get boundaries of outer lanelet
    lanelet::ConstLineString2d outer_left_bound_ll = outer_left_ll.leftBound2d();  
    lanelet::ConstLineString2d outer_right_bound_ll = outer_right_ll.rightBound2d();

    //Convert to std::vector<geometry_msgs::msg::Point>
    std::vector<geometry_msgs::msg::Point> bound_left_ls = Lanelet2Utilities::convertLaneletLine2Linestring(outer_left_bound_ll.basicLineString());
    std::vector<geometry_msgs::msg::Point> bound_right_ls = Lanelet2Utilities::convertLaneletLine2Linestring(outer_right_bound_ll.basicLineString());

    //add boundaries to boundarielinestring
    bound_left.insert(bound_left.end(), bound_left_ls.begin(), bound_left_ls.end());     
    bound_right.insert(bound_right.end(), bound_right_ls.begin(), bound_right_ls.end());  
  }
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

route_planning_interfaces::msg::LaneSeparator GlobalPlanner::deriveLaneSeparator(const lanelet::ConstLineString3d &linestring)
{
  route_planning_interfaces::msg::LaneSeparator lane_sep;
  lanelet::Attribute type_str;
  if (!linestring.hasAttribute("type"))
  {
    // We're not considering linestrings without type --> line is empty
    lane_sep.type = route_planning_interfaces::msg::LaneSeparator::TYPE_UNKNOWN;
    return lane_sep;
  } 
  if(type_str == "road_boarder")
  {
    lane_sep.type = route_planning_interfaces::msg::LaneSeparator::TYPE_CROSSING_RESTRICTED;
    lane_sep.line = Lanelet2Utilities::convertLaneletLine2Linestring(linestring.basicLineString());
    return lane_sep;
  } 
  if(type_str == "virtual")
  {
    // We're not considering linestrings with type virtual --> line is empty
    lane_sep.type = route_planning_interfaces::msg::LaneSeparator::TYPE_UNKNOWN;
    return lane_sep;
  }
  if(type_str == "line_thin" || type_str == "line_thick")
  {
    lane_sep.line = Lanelet2Utilities::convertLaneletLine2Linestring(linestring.basicLineString());
    lane_sep.type = route_planning_interfaces::msg::LaneSeparator::TYPE_UNKNOWN;
    if (linestring.hasAttribute("subtype"))
    {
      lanelet::Attribute subtype_str = linestring.attribute("subtype");
      if(subtype_str == "solid" || subtype_str == "solid_solid") lane_sep.type = route_planning_interfaces::msg::LaneSeparator::TYPE_CROSSING_RESTRICTED;
      if(subtype_str == "dashed") lane_sep.type = route_planning_interfaces::msg::LaneSeparator::TYPE_CROSSING_ALLOWED;
      if(subtype_str == "dashed_solid") lane_sep.type = route_planning_interfaces::msg::LaneSeparator::TYPE_CROSSING_ALLOWED_FROM_LEFT;
      if(subtype_str == "solid_dashed") lane_sep.type = route_planning_interfaces::msg::LaneSeparator::TYPE_CROSSING_ALLOWED_FROM_RIGHT;
    }
    return lane_sep;
  }

  // Unknown type_str --> line keeps empty
  lane_sep.type = route_planning_interfaces::msg::LaneSeparator::TYPE_UNKNOWN; 
  return lane_sep;
}

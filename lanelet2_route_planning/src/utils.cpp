#include "lanelet2_route_planning/global_planner_node.hpp"

// Convert to linestring, smooth and visualize
std::vector<geometry_msgs::msg::Point> GlobalPlanner::processLineString(lanelet::BasicLineString2d &line_string) {
  // to Linestring
  std::vector<geometry_msgs::msg::Point> points = Lanelet2Utilities::convertLaneletLine2Linestring(line_string);
  // Smooth
  line_string = Lanelet2Utilities::smoothByQuadraticBezierCurve(line_string, smooth_factor_);
  // to Linestring
  points = Lanelet2Utilities::convertLaneletLine2Linestring(line_string);
  return points;
}

route_planning_msgs::msg::DriveableSpace GlobalPlanner::sampleDriveableSpace(
    const lanelet::BasicLineString2d &centerline) {
  route_planning_msgs::msg::DriveableSpace driveable_space;
  driveable_space.boundaries.left = sampleLinestring(centerline, lateral_driv_space_width_ / 2.0, false);
  driveable_space.boundaries.right = sampleLinestring(centerline, lateral_driv_space_width_ / 2.0, true);
  return driveable_space;
}

std::vector<geometry_msgs::msg::Point> GlobalPlanner::sampleLinestring(const lanelet::BasicLineString2d &centerline,
                                                                       const double test_dis, const bool b_right) {
  double test_dis_left_right = test_dis;
  double factor_left_right = 1.0;
  if (b_right) {
    test_dis_left_right *= -1.;
    factor_left_right *= -1.;
  }

  std::deque<std::pair<lanelet::BasicLineString2d, size_t>>
      last_test_lines;                            // Test line till drivable space sample, full length test line, index
  lanelet::BasicLineString2d previous_test_line;  // Full length
  const std::pair<lanelet::BasicLineString2d, size_t> *last_intersection_free_test_line = nullptr;
  lanelet::BasicLineString2d ll_bound;

  // Process route
  for (uint idx = 0; idx < centerline.size(); idx++) {
    const lanelet::BasicPoint2d &base_p = centerline.at(idx);
    const lanelet::BasicPoint2d test_p =
        lanelet::geometry::internal::lateralShiftPointAtIndex(centerline, idx, test_dis_left_right);
    const lanelet::BasicLineString2d test_line({base_p, test_p});

    // Get all intersecting points
    std::vector<std::tuple<double, lanelet::BasicPoint2d, long>> all_interpoints;  // signed distance, point, id of line
    lanelet::LaneletMapConstPtr llmap = ll2if_->getMapPtr();
    std::vector<std::pair<double, lanelet::ConstLineString3d>> near_lines =
        lanelet::geometry::findWithin2d(llmap->lineStringLayer, test_line, 5.0);
    for (const auto &line_to_test : near_lines) {
      lanelet::BasicPoints2d interpoints;
      boost::geometry::intersection(utils::to2D(line_to_test.second).basicLineString(), test_line, interpoints);

      for (const lanelet::BasicPoint2d &poi : interpoints) {
        all_interpoints.emplace_back(lanelet::geometry::distance(base_p, poi), poi, line_to_test.second.id());
      }
    }

    // Sort according to distance
    std::sort(all_interpoints.begin(), all_interpoints.end(),
              [](auto const &t1, auto const &t2) { return std::get<0>(t1) < std::get<0>(t2); });

    // check intersecting points for drivability
    lanelet::BasicPoint2d best_point = test_p;
    for (uint i = 0; i < all_interpoints.size(); i++) {
      // Intersection with non-drivable line?
      const lanelet::ConstLineString3d &line = llmap->lineStringLayer.get(std::get<2>(all_interpoints.at(i)));
      if (checkLineDrivability(line) == false) {
        best_point = std::get<1>(all_interpoints.at(i));
        break;
      }
    }

    // Special handling for inward corners
    if (!handleInwardCorner(base_p, best_point, last_intersection_free_test_line, previous_test_line, idx,
                            last_test_lines, ll_bound)) {
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
                                        std::vector<geometry_msgs::msg::Point> &bound_right) {
  //for current and flowing lanelets
  for (size_t i = 0; i < shortest_path.size(); i++) {
    lanelet::ConstLanelet cur_ll = shortest_path[i];
    lanelet::ConstLanelet outer_left_ll = cur_ll;
    lanelet::ConstLanelet outer_right_ll = cur_ll;

    bool left_is_present = true;
    bool right_is_present = true;
    lanelet::LaneletMapConstPtr llmap = ll2if_->getMapPtr();
    routing::RoutingGraphUPtr routingGraph = routing::RoutingGraph::build(*llmap, *trafficRules_);

    // check for routable lane on left-hand side
    while (left_is_present) {
      Optional<lanelet::ConstLanelet> left{
          routingGraph->left(outer_left_ll)};  // Get routable left lanelet if it exists
      left_is_present = left.has_value();
      if (left_is_present) {
        outer_left_ll = left.get();  //get outer left lanelet
      }
    }

    // check for routable lane on right-hand side
    while (right_is_present) {
      Optional<lanelet::ConstLanelet> right{
          routingGraph->right(outer_right_ll)};  // Get routable right lanelet if it exists
      right_is_present = right.has_value();
      if (right_is_present) {
        outer_right_ll = right.get();  //get outer right lanelet
      }
    }

    //get boundaries of outer lanelet
    lanelet::ConstLineString2d outer_left_bound_ll = outer_left_ll.leftBound2d();
    lanelet::ConstLineString2d outer_right_bound_ll = outer_right_ll.rightBound2d();

    //Convert to std::vector<geometry_msgs::msg::Point>
    std::vector<geometry_msgs::msg::Point> bound_left_ls =
        Lanelet2Utilities::convertLaneletLine2Linestring(outer_left_bound_ll.basicLineString());
    std::vector<geometry_msgs::msg::Point> bound_right_ls =
        Lanelet2Utilities::convertLaneletLine2Linestring(outer_right_bound_ll.basicLineString());

    //add boundaries to boundary linestring
    bound_left.insert(bound_left.end(), bound_left_ls.begin(), bound_left_ls.end());
    bound_right.insert(bound_right.end(), bound_right_ls.begin(), bound_right_ls.end());
  }
}

bool GlobalPlanner::handleInwardCorner(
    const lanelet::BasicPoint2d &base_p, lanelet::BasicPoint2d &best_point,
    const std::pair<lanelet::BasicLineString2d, size_t> *&last_intersection_free_test_line,
    lanelet::BasicLineString2d &previous_test_line, const uint &idx,
    std::deque<std::pair<lanelet::BasicLineString2d, size_t>> &last_test_lines, lanelet::BasicLineString2d &bound) {
  const size_t max_queue_size = 30;

  lanelet::BasicLineString2d test_line_cut({base_p, best_point});
  if (last_intersection_free_test_line) {
    // Check if this line still intersects the last intersection free test line
    if (boost::geometry::intersects(test_line_cut, last_intersection_free_test_line->first)) {
      // It does, don't add anything.
      previous_test_line = test_line_cut;
      return false;
    } else {
      // It does not; now check if the previous test line intersects any of the buffered ones (find start of curve)
      if (idx > 0) {
        for (auto &last_test_line : last_test_lines) {
          if (boost::geometry::intersects(previous_test_line, last_test_line.first))  // Check full length
          {
            // Oh yeah; this is the boundary sample of the start of the curve
            // Delete the samples we have added since then
            const int amount_to_delete = last_intersection_free_test_line->second - last_test_line.second + 1;
            if (amount_to_delete > 0 && static_cast<int>(bound.size()) >= amount_to_delete) {
              bound.resize(bound.size() - amount_to_delete);
            }
            break;
          }
        }
      }

      // Continue normally, this is the boundary sample at the end of the curve
      last_intersection_free_test_line = nullptr;
      last_test_lines.clear();
    }
  } else if (last_test_lines.size() > 0) {
    // Check if this test line is intersecting the last buffered one
    const auto &last_test_line = last_test_lines.back();
    if (boost::geometry::intersects(test_line_cut, last_test_line.first)) {
      // It does; save last valid one and continue
      last_intersection_free_test_line = &last_test_line;
      previous_test_line = last_test_line.first;
      return false;
    }
  }

  // Valid test line, add to buffer
  last_test_lines.push_back(std::make_pair(test_line_cut, idx));
  if (last_test_lines.size() > max_queue_size) last_test_lines.pop_front();

  return true;
}

bool GlobalPlanner::checkLineDrivability(const lanelet::ConstLineString3d &lineToCheck) {
  lanelet::Attribute type_str;
  lanelet::Attribute subtype_str;
  if (lineToCheck.hasAttribute("type") == false) {
    return false;  // no type detectable, therefore for safety reasons don't look any further this direction
  }
  if (lineToCheck.hasAttribute("subtype") == false) {
    subtype_str = Attribute("high");  // no subtype detectable, therefore for safety reasons set to "high"
  } else {
    subtype_str = lineToCheck.attribute("subtype");
  }

  type_str = lineToCheck.attribute("type");

  // the following types are considered drivable
  if (type_str == "line_thin" || type_str == "line_thick" || type_str == "virtual" || type_str == "zebra_marking" ||
      type_str == "bike_marking" || type_str == "pedestrian_marking" || type_str == "stop_line" ||
      type_str == "traffic_light" || type_str == "roadpainting" ||  //Atlatec Maps
      type_str == "lane_center" ||                                  //Atlatec Maps
      type_str == "centerline" ||                                   //Atlatec Maps
      (type_str == "curbstone" && subtype_str == "low")) {
    return true;
  }

  // the following keys are considered drivable
  if (lineToCheck.hasAttribute("HoldingLine")) {
    return true;
  }

  return false;
}

route_planning_msgs::msg::LaneSeparator GlobalPlanner::deriveLaneSeparator(
    const lanelet::ConstLineString3d &linestring) {
  route_planning_msgs::msg::LaneSeparator lane_sep;
  lanelet::Attribute type_str;
  if (!linestring.hasAttribute("type")) {
    RCLCPP_WARN_STREAM(get_logger(),
                       "Linestring with id=" << linestring.id() << " does not have the required attribute 'type'!");
    // We're not considering linestrings without type --> line is empty
    lane_sep.type = route_planning_msgs::msg::LaneSeparator::TYPE_UNKNOWN;
    return lane_sep;
  } else {
    type_str = linestring.attribute("type");
  }
  if (type_str == "road_boarder" || type_str == "barrier") {
    lane_sep.type = route_planning_msgs::msg::LaneSeparator::TYPE_CROSSING_RESTRICTED;
    lane_sep.line = Lanelet2Utilities::convertLaneletLine2Linestring(linestring.basicLineString());
    return lane_sep;
  }
  if (type_str == "virtual") {
    // We're not considering linestrings with type virtual --> line is empty
    lane_sep.type = route_planning_msgs::msg::LaneSeparator::TYPE_UNKNOWN;
    return lane_sep;
  }
  if (type_str == "line_thin" || type_str == "line_thick") {
    lane_sep.line = Lanelet2Utilities::convertLaneletLine2Linestring(linestring.basicLineString());
    lane_sep.type = route_planning_msgs::msg::LaneSeparator::TYPE_UNKNOWN;
    if (linestring.hasAttribute("subtype")) {
      lanelet::Attribute subtype_str = linestring.attribute("subtype");
      if (subtype_str == "solid" || subtype_str == "solid_solid")
        lane_sep.type = route_planning_msgs::msg::LaneSeparator::TYPE_CROSSING_RESTRICTED;
      if (subtype_str == "dashed") lane_sep.type = route_planning_msgs::msg::LaneSeparator::TYPE_CROSSING_ALLOWED;
      if (subtype_str == "dashed_solid")
        lane_sep.type = route_planning_msgs::msg::LaneSeparator::TYPE_CROSSING_ALLOWED_FROM_LEFT;
      if (subtype_str == "solid_dashed")
        lane_sep.type = route_planning_msgs::msg::LaneSeparator::TYPE_CROSSING_ALLOWED_FROM_RIGHT;
    } else {
      RCLCPP_WARN_STREAM(get_logger(), "Linestring with type 'line_thin' or 'line_thick' with id="
                                           << linestring.id() << " does not have the required attribute 'subtype'!");
    }
    return lane_sep;
  }

  // Unknown type_str --> line keeps empty
  lane_sep.type = route_planning_msgs::msg::LaneSeparator::TYPE_UNKNOWN;
  return lane_sep;
}

uint8_t GlobalPlanner::deriveValueForSpeedLimitType(const std::shared_ptr<const lanelet::RegulatoryElement> regelem,
                                                    const std::vector<lanelet::ConstLineString3d> refering_elems) {
  if (regelem->hasAttribute("sign_type")) {
    if (refering_elems.size() == 0) {
      RCLCPP_WARN(get_logger(), "No refering elements found for speed limit sign!");
      return route_planning_msgs::msg::RegulatoryElement::STATE_UNKNOWN;
    }
    std::string tsign_val = refering_elems[0].attribute("sign_type").value();
    boost::algorithm::erase_all(tsign_val, " ");
    boost::algorithm::to_lower(tsign_val);
    if (tsign_val.find("km/h") != std::string::npos) {
      boost::algorithm::erase_all(tsign_val, "km/h");
      int val = std::stoi(tsign_val);
      if (val == 30)
        return route_planning_msgs::msg::RegulatoryElement::SPEED_30;
      else if (val == 50)
        return route_planning_msgs::msg::RegulatoryElement::SPEED_50;
      else if (val == 70)
        return route_planning_msgs::msg::RegulatoryElement::SPEED_70;
      else {
        RCLCPP_WARN_STREAM(get_logger(), "Unknown sign type for Speed-Limit: " << val);
        return route_planning_msgs::msg::RegulatoryElement::STATE_UNKNOWN;
      }
    } else if (tsign_val.find("mps") != std::string::npos) {
      boost::algorithm::erase_all(tsign_val, "mps");
      int val = (int)std::round(std::stod(tsign_val) * 3.6);
      if (val < 33 && val > 27)
        return route_planning_msgs::msg::RegulatoryElement::SPEED_30;
      else if (val < 53 && val > 47)
        return route_planning_msgs::msg::RegulatoryElement::SPEED_50;
      else if (val < 73 && val > 67)
        return route_planning_msgs::msg::RegulatoryElement::SPEED_70;
      else {
        RCLCPP_WARN_STREAM(get_logger(), "Unknown sign type for Speed-Limit: " << val);
        return route_planning_msgs::msg::RegulatoryElement::STATE_UNKNOWN;
      }

    } else if (tsign_val.find("mph") != std::string::npos) {
      boost::algorithm::erase_all(tsign_val, "mph");
      int val = (int)std::round(std::stod(tsign_val) * 1.60934);
      if (val < 33 && val > 27)
        return route_planning_msgs::msg::RegulatoryElement::SPEED_30;
      else if (val < 53 && val > 47)
        return route_planning_msgs::msg::RegulatoryElement::SPEED_50;
      else if (val < 73 && val > 67)
        return route_planning_msgs::msg::RegulatoryElement::SPEED_70;
      else {
        RCLCPP_WARN_STREAM(get_logger(), "Unknown sign type for Speed-Limit: " << val);
        return route_planning_msgs::msg::RegulatoryElement::STATE_UNKNOWN;
      }
    } else {
      // Interpret as km/h according to documentation
      int val = std::stoi(tsign_val);
      if (val == 30)
        return route_planning_msgs::msg::RegulatoryElement::SPEED_30;
      else if (val == 50)
        return route_planning_msgs::msg::RegulatoryElement::SPEED_50;
      else if (val == 70)
        return route_planning_msgs::msg::RegulatoryElement::SPEED_70;
      else {
        RCLCPP_WARN_STREAM(get_logger(), "Unknown sign type for Speed-Limit: " << val);
        return route_planning_msgs::msg::RegulatoryElement::STATE_UNKNOWN;
      }
    }
  } else if (refering_elems.size() && refering_elems[0].hasAttribute("type") &&
             refering_elems[0].attribute("type").value() == "traffic_sign" &&
             refering_elems[0].hasAttribute("subtype")) {
    std::string tsign_code = refering_elems[0].attribute("subtype").value();
    return trafficSignCode2Type(tsign_code);
  }
}

uint8_t GlobalPlanner::trafficSignCode2Type(const std::string tsign_code) {
  if (tsign_code == "de274-30")
    return route_planning_msgs::msg::RegulatoryElement::SPEED_30;
  else if (tsign_code == "de274-50")
    return route_planning_msgs::msg::RegulatoryElement::SPEED_50;
  else if (tsign_code == "de274-70")
    return route_planning_msgs::msg::RegulatoryElement::SPEED_70;
  else {
    RCLCPP_WARN_STREAM(get_logger(), "Unknown sign code for Traffic-Sign: " << tsign_code);
    return route_planning_msgs::msg::RegulatoryElement::STATE_UNKNOWN;
  }
}

bool GlobalPlanner::calcIntersection(const geometry_msgs::msg::Point p1, const geometry_msgs::msg::Point p2,
                                         const geometry_msgs::msg::Point p3, const geometry_msgs::msg::Point p4,
                                         double& lambda) {
  // problem description:
  // Line 1: (x,y)=(x1,y1)+t1​⋅(x2−x1,y2−y1)
  // Line 2: (x,y)=(x3,y3)+t2​⋅(x4−x3,y4−y3)
  // solve for t1 and t2, where lines are equal --> (x1+t1​(x2−x1),y1+t1​(y2−y1))=(x3+t2​(x4−x3),y3+t2​(y4−y3)) 
  // two linear equations: x1+t1​(x2−x1)=x3+t2​(x4−x3) and y1+t1​(y2−y1)=y3+t2​(y4−y3)
  
  // direction vectors of the lines
  double d1x = p2.x - p1.x;
  double d1y = p2.y - p1.y;
  double d2x = p4.x - p3.x;
  double d2y = p4.y - p3.y;

  // determinant
  double det = d1x * d2y - d1y * d2x;
  if (det == 0) return false; // lines are parallel or collinear

  // Calculate parameters t1 and t2
  double t1 = ((p3.x - p1.x) * d2y - (p3.y - p1.y) * d2x) / det;
  double t2 = ((p3.x - p1.x) * d1y - (p3.y - p1.y) * d1x) / det;

  if (t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1) {
      // There is an intersection within the segments
      double x = p1.x + t1 * d1x;
      double y = p1.y + t1 * d1y;
      lambda = t1;
      return true;
  }

  return false;
}

void GlobalPlanner::setEffectLineS(route_planning_msgs::msg::Route& route) {
  for (size_t i = 0; i < route.remaining_route.size() - 1; ++i) {
    geometry_msgs::msg::Point p1 = route.remaining_route[i];
    geometry_msgs::msg::Point p2 = route.remaining_route[i + 1];
    for (size_t j = 0; j < route.regulatory_elements.size(); ++j) {
      geometry_msgs::msg::Point p3 = route.regulatory_elements[j].effect_line[0];
      geometry_msgs::msg::Point p4 = route.regulatory_elements[j].effect_line[1];
      double lambda;
      if (calcIntersection(p1, p2, p3, p4, lambda)) {
        double intersection_s = p1.z + lambda * (p2.z - p1.z);
        route.regulatory_elements[j].effect_line[0].z = intersection_s;
        route.regulatory_elements[j].effect_line[1].z = intersection_s;
      }
    }
  }
}

/**
 * @brief Accumulates the distance along a 2D line path, storing it as the z-coordinate of the points
 * 
 * @param path 2D path to accumulate distance along, z-coordinate will be overwritten with distance
 * @param initial_distance initial distance to start accumulating from
 */
void GlobalPlanner::accumulateDistanceAlong2DPath(std::vector<geometry_msgs::msg::Point> &path,
                                                  const double initial_distance) {
  if (path.empty()) return;
  double accumulated_distance = initial_distance;
  path[0].z = initial_distance;
  for (size_t i = 1; i < path.size(); i++) {
    accumulated_distance += this->distance(path[i - 1], path[i]);
    path[i].z = accumulated_distance;
  }
}
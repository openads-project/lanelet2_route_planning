#include "lanelet2_route_planning/global_planner_node.hpp"

GlobalPlanner::GlobalPlanner() : Node("global_planner")
{
  startup_timer_ = create_wall_timer(0.1s, std::bind(&GlobalPlanner::initializeGlobalPlanner, this));
  map_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>("/ego_vehicle/map_pose", 1, std::bind(&GlobalPlanner::mapPoseCallback, this, std::placeholders::_1));

}

void GlobalPlanner::initializeMapInterface()
{
  // To-Do load name as parameter
  std::string map_server_name = "ll2_map_server";
  // Important: shared_from_this() can not be called from within the constructor
  ll2if_ = new LL2MapInterface(shared_from_this(), map_server_name);
}

void GlobalPlanner::initializeGlobalPlanner()
{
  if(!ll2if_->map_loaded_)
  {
    RCLCPP_INFO(get_logger(), "Waiting for Lanelet2-Map-Interface to load map!");
    return;
  }
  else
  {
    // create an action server for handling action goal requests
    maneuver_action_server_ = rclcpp_action::create_server<lanelet2_route_planning_ifs::action::GlobalManeuver>(this, "~/execute_global_maneuver",
    std::bind(&GlobalPlanner::actionHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GlobalPlanner::actionHandleCancel, this, std::placeholders::_1),
    std::bind(&GlobalPlanner::actionHandleAccepted, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Created 'execute_global_maneuver' action-server!");

    viz_destination_pub_ = create_publisher<visualization_msgs::msg::Marker>("~/destination_marker",1);
    viz_route_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/route_marker",1);
    viz_boundary_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/boundary_marker",1);

    startup_timer_->cancel();
  }
}

bool GlobalPlanner::egoPositionSanityCheck()
{
  if((now() - ego_pose_.header.stamp).seconds()>0.2)
  {
    RCLCPP_ERROR(get_logger(), "Latest ego-pose message is depracted!");
    return false;
  }
  else
  {
    lanelet::LaneletMapConstPtr llmap = ll2if_->getMapPtr();
    // To-Do: Check if ego_pose_ frame equals lanelet2 map-frame
    //if(ego_pose_.header.frame_id != ll2if_)

    // Determine nearest lanelet
    std::vector<std::pair<double, lanelet::ConstLanelet>> nearestLanelets = lanelet::geometry::findNearest(llmap->laneletLayer, lanelet::BasicPoint2d(ego_pose_.pose.pose.position.x, ego_pose_.pose.pose.position.y), 5);
    if(nearestLanelets.size() < 1)
    {
      RCLCPP_ERROR(get_logger(), "No Lanelet at current ego-pose!");
      return false;
    }
    
    //Get actual Heading
    tf2::Quaternion quat;
    tf2::fromMsg(ego_pose_.pose.pose.orientation, quat);
    float yaw = tf2::impl::getYaw(quat);
    Lanelet2Utilities::laneletSorting(lanelet::BasicPoint2d(ego_pose_.pose.pose.position.x, ego_pose_.pose.pose.position.y), nearestLanelets, yaw, trafficRules_, {});
    start_ll_ = nearestLanelets.at(0).second; // most probable current Lanelet
    if (geometry::distance(start_ll_.polygon2d(), lanelet::BasicPoint2d(ego_pose_.pose.pose.position.x, ego_pose_.pose.pose.position.y)) > 0.5)
    {
      RCLCPP_ERROR(get_logger(), "Current ego-pose is not on the start lanelet!");
      return false;
    }
    else
    {
      return true;
    }
  }
}

bool GlobalPlanner::targetPositionSanityCheck(double target_x, double target_y)
{
  lanelet::LaneletMapConstPtr llmap = ll2if_->getMapPtr();
  std::vector<std::pair<double, lanelet::ConstLanelet>> nearestLaneletsTarget = lanelet::geometry::findNearest(llmap->laneletLayer, lanelet::BasicPoint2d(target_x, target_y), 5);
  if(nearestLaneletsTarget.size()<1)
  {
    RCLCPP_ERROR(get_logger(), "No Lanelet at given target-position!");
    return false;
  }
  Lanelet2Utilities::laneletSorting(lanelet::BasicPoint2d(target_x, target_y), nearestLaneletsTarget, {}, trafficRules_, {});
  bool b_found_target_ll = false;
  for (const auto &ll_target : nearestLaneletsTarget)
  {
    if (ll_target.first < 10. && trafficRules_->canPass(ll_target.second))
    {
      Optional<lanelet::routing::Route> temp_route = routingGraph_->getRoute(start_ll_, ll_target.second, 0); // 0 = routingCostId distance
      // Is route possible?
      if (!!temp_route)
      {
        target_ll_ = ll_target.second;
        b_found_target_ll = true;
        break;
      }
    }
  }
  if(!b_found_target_ll)
  {
    RCLCPP_ERROR(get_logger(), "Unable to plan a route to the given target position!");
  }
  return b_found_target_ll;
}

bool GlobalPlanner::planRoute(lanelet::ConstLanelet start_ll, lanelet::ConstLanelet target_ll)
{
  builtin_interfaces::msg::Time start_time = now();
  // Start and end positions
  lanelet::BasicPoint3d target;
  target.x()=maneuver_feedback_->destination_x;
  target.y()=maneuver_feedback_->destination_y;
  target.z()=0.0;
  target = lanelet::geometry::project(target_ll_.centerline(), target);
  maneuver_feedback_->destination_x = target.x();
  maneuver_feedback_->destination_y = target.y();
  lanelet::BasicPoint2d start_pos = lanelet::BasicPoint2d(ego_pose_.pose.pose.position.x, ego_pose_.pose.pose.position.y);
  lanelet::BasicPoint2d target_pos = lanelet::BasicPoint2d(target.x(), target.y());

  // Add small offset to start
  lanelet::ConstLanelet start_ll_offset = start_ll_;
  lanelet::ConstLanelet target_ll_offset = target_ll_;
  double remaining = lanelet::geometry::toArcCoordinates(start_ll_offset.centerline2d(), start_pos).length - 10.;
  while (remaining < 0.)
  {
    lanelet::ConstLanelets prevs = routingGraph_->previous(start_ll_offset);
    if(!prevs.size())
    {
      RCLCPP_WARN(get_logger(), "Current matched lanelet has no predecessors!");
      break;
    }
    start_ll_offset = prevs.at(0);
    remaining += lanelet::geometry::length(start_ll_offset.centerline2d()); // remaining is negative
  }
  start_pos = lanelet::geometry::interpolatedPointAtDistance(start_ll_offset.centerline2d(), remaining);

  // Add small offset to end (so drivable space does not end directly at target)
  double len_target_ll = lanelet::geometry::length(target_ll_offset.centerline2d());
  remaining = len_target_ll - (lanelet::geometry::toArcCoordinates(target_ll_offset.centerline2d(), target_pos).length + 20.);
  while (remaining < 0.)
  {
    lanelet::ConstLanelets following_lls = routingGraph_->following(target_ll_offset, false);
    if(!following_lls.size())
    {
      RCLCPP_WARN(get_logger(), "Current target lanelet has no followers!");
      break;
    }
    target_ll_offset = following_lls.at(0);
    len_target_ll = lanelet::geometry::length(target_ll_offset.centerline2d());
    remaining += len_target_ll;
  }
  target_pos = lanelet::geometry::interpolatedPointAtDistance(target_ll_offset.centerline2d(), len_target_ll - remaining);

  // Get route
  route_ = routingGraph_->getRoute(start_ll_offset, target_ll_offset, 0); // 0 = routingCostId distance
  RCLCPP_INFO_STREAM(get_logger(), "Time core route planning: " << (now() - start_time).seconds() << "s");
  // Is route-planning possible?
  if (!!route_)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Calculated new route! Start Lanelet: " << start_ll_.id() << " | Target Lanelet: " << target_ll_.id());
    
    // Extract shortest path and its boundaries
    lanelet::routing::LaneletPath shortestPath = route_->shortestPath(); // shortestPath = sorted Lanelets
    shortest_path_ll_ids_.clear();
    for(const auto& ll : shortestPath)
    {
      shortest_path_ll_ids_.push_back(ll.id());
    }
    std::pair<lanelet::BasicLineString2d, lanelet::BasicLineString2d> lane_boundaries;
    lanelet::BasicLineString2d shortest_path_centerline = Lanelet2Utilities::convertLLPath2LineString2dSBased(ConstLanelets(shortestPath.begin(), shortestPath.end()), start_pos, 10., 3., std::numeric_limits<double>::max(), ds_sample_, target_pos, lane_boundaries, *routingGraphBicycle_);
    visualization_msgs::msg::MarkerArray marker_array_route;
    processLineString(shortest_path_centerline, "shortest path", marker_array_route, {0.1, 0.35, 0.3}, {0.2, 0.5, 0.15});

    // Construct lane network
    start_time = now();
    constructLaneNetwork(shortestPath, marker_array_route);
    RCLCPP_INFO_STREAM(get_logger(), "Duration for lane network creation: " << (now() - start_time).seconds() << "s");

    // Process boundaries
    start_time = now();
    visualization_msgs::msg::MarkerArray marker_array_boundary;
    std::vector<int> shortest_path_bound_left_mapping;
    std::vector<int> shortest_path_bound_right_mapping;
    lanelet::BasicLineString2d shortest_path_bound_left  = sampleBoundaries(shortest_path_centerline, 10., false, shortest_path_bound_left_mapping, lane_boundaries.first, marker_array_boundary);
    lanelet::BasicLineString2d shortest_path_bound_right = sampleBoundaries(shortest_path_centerline, 10., true,  shortest_path_bound_right_mapping, lane_boundaries.second, marker_array_boundary);
    std::vector<int> drivable_space_left_mapping;
    std::vector<int> drivable_space_right_mapping;
    lanelet::BasicLineString2d drivable_space_left  = sampleDrivableSpace(shortest_path_centerline, 10., false, drivable_space_left_mapping, marker_array_boundary);
    lanelet::BasicLineString2d drivable_space_right = sampleDrivableSpace(shortest_path_centerline, 10., true,  drivable_space_right_mapping, marker_array_boundary);
    RCLCPP_INFO_STREAM(get_logger(), "Duration for calculation of boundaries: " << (now() - start_time).seconds() << "s");

    // Visualize
    viz_route_pub_->publish(marker_array_route);
    viz_boundary_pub_->publish(marker_array_boundary);

    return true;
  }
  else
  {
    RCLCPP_ERROR_STREAM(get_logger(), "No routing possibility from Lanelet " << start_ll_.id() << " to Lanelet " << target_ll_.id());
    return false;
  }
}


void GlobalPlanner::constructLaneNetwork(const lanelet::routing::LaneletPath &shortestPath, visualization_msgs::msg::MarkerArray &viz_marker_array)
{
  // Datastructures
  std::vector<std::pair<lanelet::ConstLanelets, size_t> > lanes_hierarchy(shortestPath.size()); // Contains the spatial hierarchy along the route (First entry of pair contains all neighboring lanelets per route section; second entry is the shortest path index in this list)
  std::vector<lanelet::ConstLanelets> lanes;                                                    // All dedicated lanes of the route. Their index in this vector is their lane id which is used to refer to them.
  std::vector<lanelet::BasicLineString2d> lanes_line;                                           // Extracted and smoothed linestring for earch lane.
  std::unordered_map<int64_t, int16_t> lane_id_mapping;                                         // Maps a lanelet id to its lane id
  std::unordered_map<int64_t, std::pair<size_t, size_t> > lane_hierarchy_mapping;               // Maps the lanelets of a lane to their spatial hierarchy entry.

  // Construct lane network. Work along shortest path.
  int lane_id = -1;
  size_t index_1 = 0;
  for (const lanelet::ConstLanelet &ll: shortestPath)
  {
    // Is this part of the shortest path a new lane?
    if(lane_id_mapping.find(ll.id()) == lane_id_mapping.end())
    {
      lane_id++;
      lanes.push_back(ConstLanelets());

      const lanelet::LaneletSequence rl = route_->remainingLane(ll);
      for (const lanelet::ConstLanelet &rl_ll: rl)
      {
        lane_id_mapping[rl_ll.id()] = lane_id;
        lanes[lane_id].push_back(rl_ll);
      }
    }

    // Right neighbors
    ConstLanelets right_neighbors;
    Optional<routing::LaneletRelation> neighbour_relation = route_->rightRelation(ll);
    while (!!neighbour_relation)
    {
      // Not yet visited?
      if(lane_id_mapping.find(neighbour_relation->lanelet.id()) == lane_id_mapping.end())
      {
        lane_id++;
        lanes.push_back(lanelet::ConstLanelets());

        const lanelet::LaneletSequence rl = route_->remainingLane(neighbour_relation->lanelet);
        for (const lanelet::ConstLanelet &rl_ll: rl)
        {
          lane_id_mapping[rl_ll.id()] = lane_id;
          lanes[lane_id].push_back(rl_ll);
        }
      }
      right_neighbors.push_back(neighbour_relation->lanelet);

      // Next right neighbor
      neighbour_relation = route_->rightRelation(neighbour_relation->lanelet);
    }

    // Left neighbors
    lanelet::ConstLanelets left_neighbors;
    neighbour_relation = route_->leftRelation(ll);
    while (!!neighbour_relation)
    {
      // Not yet visited?
      if(lane_id_mapping.find(neighbour_relation->lanelet.id()) == lane_id_mapping.end())
      {
        lane_id++;
        lanes.push_back(lanelet::ConstLanelets());

        const lanelet::LaneletSequence rl = route_->remainingLane(neighbour_relation->lanelet);
        for (const ConstLanelet &rl_ll: rl)
        {
          lane_id_mapping[rl_ll.id()] = lane_id;
          lanes[lane_id].push_back(rl_ll);
        }
      }
      left_neighbors.push_back(neighbour_relation->lanelet);

      // Next left neighbor
      neighbour_relation = route_->leftRelation(neighbour_relation->lanelet);
    }

    // Construct lanes hierarchy
    lanelet::ConstLanelets lane_hierarchy;
    lane_hierarchy.insert(lane_hierarchy.end(), right_neighbors.begin(), right_neighbors.end());
    lane_hierarchy.push_back(ll);
    lane_hierarchy.insert(lane_hierarchy.end(), left_neighbors.begin(), left_neighbors.end());
    for(size_t index_2=0; index_2<lane_hierarchy.size(); index_2++)
    {
      lane_hierarchy_mapping[lane_hierarchy[index_2].id()] = std::make_pair(index_1, index_2);
    }
    lanes_hierarchy[index_1++] = std::make_pair(lane_hierarchy, right_neighbors.size());
  }

  RCLCPP_INFO_STREAM(get_logger(), "Number of lanes: " << lanes.size());

  // Extract line string for lanes; smooth it and visualize
  lanes_line.resize(lanes.size());
  for(size_t i=0; i<lanes.size(); i++)
  {
    lanes_line[i] = Lanelet2Utilities::convertLLPath2LineString2dSBased(lanes[i], lanes[i].front().centerline2d().front(), 10., 3., std::numeric_limits<double>::max(), ds_sample_, {}, {}, {});
    processLineString(lanes_line[i], "lane " + std::to_string(i), viz_marker_array, {0.0f, 0.75f - i/(lanes.size()-1.0f) * 0.5f, 0.25f + i/(lanes.size()-1.0f) * 0.5f}, {0.75f - i/(lanes.size()-1.0f) * 0.5f, 0.0f, 0.25f + i/(lanes.size()-1.0f) * 0.5f});
  }

  // Fill message
  GlobalPlanner::LaneletLaneNetwork lane_network;
  lane_network.lane_hierarchy.resize(lanes_hierarchy.size());
  for(size_t i=0; i<lane_network.lane_hierarchy.size(); i++)
  {
    lane_network.lane_hierarchy[i].neighboring_lanelets.resize(lanes_hierarchy[i].first.size());

    for(size_t j=0; j<lanes_hierarchy[i].first.size(); j++)
    {
      GlobalPlanner::LaneletExtended lanelet_extended;
      lanelet_extended.lanelet_id = (lanes_hierarchy[i].first)[j].id();
      lanelet_extended.lane_id    = lane_id_mapping[lanelet_extended.lanelet_id];
      lanelet_extended.v_max      = lanelet::units::KmHQuantity(trafficRules_->speedLimit((lanes_hierarchy[i].first)[j]).speedLimit).value() / 3.6;
      lane_network.lane_hierarchy[i].neighboring_lanelets[j] = lanelet_extended;
    }
    lane_network.lane_hierarchy[i].shortest_path_index = lanes_hierarchy[i].second;
  }
  lane_network.lanes.resize(lanes.size());
  for(size_t i=0; i<lanes.size(); i++)
  {
    lane_network.lanes[i].lane_sections.resize(lanes[i].size());
    std::vector<std::pair<int64_t,bool>> ll_ids = Lanelet2Utilities::convertLLRoute2IdVec(lanes[i]);
    double s = 0;
    for(size_t j=0; j<lane_network.lanes[i].lane_sections.size(); j++)
    {
      lane_network.lanes[i].lane_sections[j].accumulated_s = s += geometry::length2d(lanes[i][j]);
      lane_network.lanes[i].lane_sections[j].route_index   = lane_hierarchy_mapping[ll_ids[j].first].first;
      lane_network.lanes[i].lane_sections[j].spatial_index = lane_hierarchy_mapping[ll_ids[j].first].second;

      if(lanes[i][j] == target_ll_)
      {
        target_lane_s_dest_ = lane_network.lanes[i].lane_sections[j].accumulated_s;
      }
    }
    lane_network.lanes[i].line = lanes_line[i];
  }

  start_lane_id_  = lane_id_mapping[start_ll_.id()];
  target_lane_id_ = lane_id_mapping[target_ll_.id()];
}

lanelet::BasicLineString2d GlobalPlanner::sampleBoundaries(const lanelet::BasicLineString2d &centerline,
                                                  const double test_dis,
                                                  const bool &b_right,
                                                  std::vector<int>& index_mapping,
                                                  const lanelet::BasicLineString2d& lane_boundary,
                                                  visualization_msgs::msg::MarkerArray& marker_array)
{
  double test_dis_left_right = test_dis;
  double factor_left_right = 1.0;
  std::string left_right_string = "left";
  std::vector<float> left_right_colors = {0.95, 0.25, 0.25, 1.0};
  if (b_right)
  {
    test_dis_left_right *= -1.;
    factor_left_right *= -1.;
    left_right_string = "right";
    left_right_colors = {0.25, 0.25, 0.95, 1.0};
  }

  const size_t max_queue_size = 30;
  std::deque<std::pair<lanelet::BasicLineString2d, size_t>> last_test_lines; // Test line till boundary sample, full length test line, index
  lanelet::BasicLineString2d previous_test_line; // Full length
  const std::pair<lanelet::BasicLineString2d, size_t>* last_intersection_free_test_line = nullptr;
  lanelet::BasicLineString2d bound;

  // For visualization
  size_t ids_final_point = 0;
  size_t ids_test_line = 0;
  size_t ids_final_bound = 0;

  // Process route
  for (uint idx = 0; idx < centerline.size(); idx++)
  {
    const lanelet::BasicPoint2d &base_p = centerline.at(idx);
    lanelet::BasicPoint2d best_point = geometry::internal::lateralShiftPointAtIndex(centerline, idx, test_dis_left_right);
    const lanelet::BasicLineString2d test_line({base_p, best_point});

    // Get all intersecting points
    lanelet::BasicPoints2d interpoints;
    boost::geometry::intersection(lane_boundary, test_line, interpoints);

    // Sort according to distance
    std::vector<std::pair<double, lanelet::BasicPoint2d> > all_interpoints;
    for (const BasicPoint2d &poi : interpoints)
    {
      all_interpoints.emplace_back(geometry::distance(base_p, poi), poi);
    }
    std::sort(all_interpoints.begin(), all_interpoints.end(),
              [](auto const &t1, auto const &t2) {
                return t1.first < t2.first;
              });
    if(interpoints.size())
    {
      best_point = all_interpoints.front().second;
    }

    // Special handling for inward corners
    if(!handleInwardCorner(base_p, best_point, last_intersection_free_test_line, previous_test_line, idx, last_test_lines, bound, index_mapping))
    {
      continue;
    }

    // Add final point to boundary samples
    bound.push_back(best_point);
    index_mapping.push_back(idx);

    // Visualize final point and test line
    if (visualize_lvl_ > 1)
    {
      visualization_msgs::msg::Marker marker;

      // Test line
      left_right_colors[3] = 0.5;
      Lanelet2Utilities::convertLaneletLine2VisuLineStrip(test_line, marker, ll2if_->map_frame_id_, now(), "lvl2_boundary_test_line_" + left_right_string, left_right_colors);
      marker.id = ids_test_line++;
      marker_array.markers.push_back(marker);

      // Final point
      lanelet::BasicLineString2d outerpoint;
      outerpoint.push_back(best_point);
      left_right_colors[3] = 1.0;
      Lanelet2Utilities::convertLaneletLine2VisuSphere(outerpoint, marker, ll2if_->map_frame_id_, now(), "lvl2_boundary_points_" + left_right_string, left_right_colors);
      marker.id = ids_final_point++;
      marker_array.markers.push_back(marker);
    }
  }

  // Visualize final boundary
  if (visualize_lvl_ > 0)
  {
    visualization_msgs::msg::Marker marker;

    if (visualize_lvl_ > 1)
    {
      left_right_colors[3] = 0.5;
      Lanelet2Utilities::convertLaneletLine2VisuLineStrip(lane_boundary, marker, ll2if_->map_frame_id_, now(), "lvl1_boundary_raw_" + left_right_string, left_right_colors, 0.25);
      marker.id = ids_final_bound++;
      marker_array.markers.push_back(marker);
    }

    left_right_colors[3] = 1.0;
    Lanelet2Utilities::convertLaneletLine2VisuLineStrip(bound, marker, ll2if_->map_frame_id_, now(), "lvl1_boundary_final_" + left_right_string, left_right_colors, 0.25);
    marker.id = ids_final_bound++;
    marker_array.markers.push_back(marker);

    visualizeIndexMapping(marker, marker_array, bound, left_right_string, "lvl2_boundary_index", index_mapping);
  }
  return bound;
}

lanelet::BasicLineString2d GlobalPlanner::sampleDrivableSpace(const lanelet::BasicLineString2d &centerline,
                                                              const double test_dis,
                                                              const bool &b_right,
                                                              std::vector<int>& index_mapping,
                                                              visualization_msgs::msg::MarkerArray& marker_array)
{
  double test_dis_left_right = test_dis;
  double factor_left_right = 1.0;
  std::string left_right_string = "left";
  std::vector<float> left_right_colors = {1.0, 0.53, 0.0, 1.0};
  if (b_right)
  {
    test_dis_left_right *= -1.;
    factor_left_right *= -1.;
    left_right_string = "right";
    left_right_colors = {0.19, 0.84, 0.78, 0.75};
  }

  std::deque<std::pair<lanelet::BasicLineString2d, size_t>> last_test_lines; // Test line till drivable space sample, full length test line, index
  lanelet::BasicLineString2d previous_test_line; // Full length
  const std::pair<lanelet::BasicLineString2d, size_t>* last_intersection_free_test_line = nullptr;
  lanelet::BasicLineString2d bound;

  // For visualization
  size_t ids_intersect = 0;
  size_t ids_final_point = 0;
  size_t ids_test_line = 0;
  size_t ids_final_bound = 0;

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
      if (visualize_lvl_ > 1)
      {
        // Intersection points - PINK vertical arrows
        visualization_msgs::msg::Marker line_strip;
        line_strip.id = ids_intersect++;
        geometry_msgs::msg::Point point;
        point.x = std::get<1>(all_interpoints.at(i)).x();
        point.y = std::get<1>(all_interpoints.at(i)).y();
        point.z = 3.0;
        line_strip.points.push_back(point);
        line_strip.header.frame_id = ll2if_->map_frame_id_;
        line_strip.header.stamp = now();
        line_strip.ns = "lvl2_drivable_space_intersection_points_" + left_right_string;
        line_strip.type = visualization_msgs::msg::Marker::ARROW;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.scale.x = 0.3;
        line_strip.scale.y = 0.5;
        line_strip.color.r = 0.7;
        line_strip.color.g = 0.4;
        line_strip.color.b = 0.7;
        line_strip.color.a = 1.0;
        point.z = 0.0;
        line_strip.points.push_back(point);
        marker_array.markers.push_back(line_strip);
      }

      // Intersection with non-drivable line?
      const lanelet::ConstLineString3d &line = llmap_->lineStringLayer.get(std::get<2>(all_interpoints.at(i)));
      if (checkLineDrivability(line) == false)
      {
        best_point = std::get<1>(all_interpoints.at(i));
        break;
      }
    }

    // Special handling for inward corners
    if(!handleInwardCorner(base_p, best_point, last_intersection_free_test_line, previous_test_line, idx, last_test_lines, bound, index_mapping))
    {
      continue;
    }

    // Add final point to samples
    bound.push_back(best_point);
    index_mapping.push_back(idx);

    // Visualize final point and test line
    if (visualize_lvl_ > 1)
    {
      visualization_msgs::msg::Marker marker;

      // Test line
      left_right_colors[3] = 0.5;
      Lanelet2Utilities::convertLaneletLine2VisuLineStrip(test_line, marker, ll2if_->map_frame_id_, now(), "lvl2_drivable_space_test_line_" + left_right_string, left_right_colors);
      marker.id = ids_test_line++;
      marker_array.markers.push_back(marker);

      // Final point
      BasicLineString2d outerpoint;
      outerpoint.push_back(best_point);
      left_right_colors[3] = 1.0;
      Lanelet2Utilities::convertLaneletLine2VisuSphere(outerpoint, marker, ll2if_->map_frame_id_, now(), "lvl2_drivable_space_points_" + left_right_string, left_right_colors);
      marker.id = ids_final_point++;
      marker_array.markers.push_back(marker);
    }
  }

  // Visualize final drivable space
  if (visualize_lvl_ > 0)
  {
    visualization_msgs::msg::Marker marker;

    left_right_colors[3] = 0.5;
    Lanelet2Utilities::convertLaneletLine2VisuLineStrip(bound, marker, ll2if_->map_frame_id_, now(), "lvl1_drivable_space_final_" + left_right_string, left_right_colors, 0.25);
    marker.id = ids_final_bound++;
    marker_array.markers.push_back(marker);

    visualizeIndexMapping(marker, marker_array, bound, left_right_string, "lvl2_drivable_space_index", index_mapping);
  }

  return bound;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto planner = std::make_shared<GlobalPlanner>();
  planner->initializeMapInterface();
  rclcpp::spin(planner);
  rclcpp::shutdown();
  return 0;
}

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
  Optional<lanelet::routing::Route> route = routingGraph_->getRoute(start_ll_offset, target_ll_offset, 0); // 0 = routingCostId distance
  RCLCPP_INFO_STREAM(get_logger(), "Time core route planning: " << (now() - start_time).seconds() << "s");
  // Is route-planning possible?
  if (!!route)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Calculated new route! Start Lanelet: " << start_ll_.id() << " | Target Lanelet: " << target_ll_.id());
    
    // Extract shortest path and its boundaries
    lanelet::routing::LaneletPath shortestPath = route->shortestPath(); // shortestPath = sorted Lanelets
    shortest_path_ll_ids_.clear();
    for(const auto& ll : shortestPath)
    {
      shortest_path_ll_ids_.push_back(ll.id());
    }
    std::pair<lanelet::BasicLineString2d, lanelet::BasicLineString2d> lane_boundaries;
    shortest_path_centerline_ = Lanelet2Utilities::convertLLPath2LineString2dSBased(ConstLanelets(shortestPath.begin(), shortestPath.end()), start_pos, 10., 3., std::numeric_limits<double>::max(), ds_sample_, target_pos, lane_boundaries, *routingGraphBicycle_);
    visualization_msgs::msg::MarkerArray marker_array_route;
    processLineString(shortest_path_centerline_, "shortest path", marker_array_route, {0.1, 0.35, 0.3}, {0.2, 0.5, 0.15});

    // Construct lane network
    start_time = now();
    //constructLaneNetwork(shortestPath);
    RCLCPP_INFO_STREAM(get_logger(), "Duration for lane network creation: " << (now() - start_time).seconds() << "s");

    // Process boundaries
    start_time = now();
    // shortest_path_bound_left_mapping_.clear();
    // shortest_path_bound_right_mapping_.clear();
    // shortest_path_bound_left_  = sampleBoundaries(shortest_path_centerline_, 10., false, shortest_path_bound_left_mapping_, lane_boundaries.first, marker_array_boundary_);
    // shortest_path_bound_right_ = sampleBoundaries(shortest_path_centerline_, 10., true,  shortest_path_bound_right_mapping_, lane_boundaries.second, marker_array_boundary_);
    // drivable_space_left_mapping_.clear();
    // drivable_space_right_mapping_.clear();
    // drivable_space_left_  = sampleDrivableSpace(shortest_path_centerline_, 10., false, drivable_space_left_mapping_, marker_array_boundary_);
    // drivable_space_right_ = sampleDrivableSpace(shortest_path_centerline_, 10., true,  drivable_space_right_mapping_, marker_array_boundary_);
    RCLCPP_INFO_STREAM(get_logger(), "Duration for calculation of boundaries: " << (now() - start_time).seconds() << "s");

    // Visualize
    //pub_visu_route_.publish(marker_array_route_);
    //pub_visu_boundary_.publish(marker_array_boundary_);

    return true;
  }
  else
  {
    RCLCPP_ERROR_STREAM(get_logger(), "No routing possibility from Lanelet " << start_ll_.id() << " to Lanelet " << target_ll_.id());
    return false;
  }
  
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // MutliThreadedExecutor is mandatory when using the lanelet2_map_interface
  rclcpp::executors::MultiThreadedExecutor executor;
  auto planner = std::make_shared<GlobalPlanner>();
  executor.add_node(planner);
  planner->initializeMapInterface();
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

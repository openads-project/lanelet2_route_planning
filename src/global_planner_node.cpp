#include "lanelet2_route_planning/global_planner_node.hpp"

GlobalPlanner::GlobalPlanner() : Node("global_planner")
{
  startup_timer_ = create_wall_timer(0.1s, std::bind(&GlobalPlanner::initializeGlobalPlanner, this));
  map_pose_sub_ = create_subscription<perception_interfaces::msg::EgoData>("/carla_its_converter/egoData", 1, std::bind(&GlobalPlanner::mapPoseCallback, this, std::placeholders::_1));
  goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1, std::bind(&GlobalPlanner::goalPoseCallback, this, std::placeholders::_1));
  maneuver_action_client_ = rclcpp_action::create_client<route_planning_interfaces::action::GlobalManeuver>(this, "~/execute_global_maneuver");

  // create a transform listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
    maneuver_action_server_ = rclcpp_action::create_server<route_planning_interfaces::action::GlobalManeuver>(this, "~/execute_global_maneuver",
    std::bind(&GlobalPlanner::actionHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GlobalPlanner::actionHandleCancel, this, std::placeholders::_1),
    std::bind(&GlobalPlanner::actionHandleAccepted, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Created 'execute_global_maneuver' action-server!");

    route_pub_ = create_publisher<route_planning_interfaces::msg::Route>("~/global/route",1);
    driveable_space_pub_ = create_publisher<route_planning_interfaces::msg::DriveableSpace>("~/global/driveable_space",1);

    // local path extraction
    local_route_pub_ = create_publisher<route_planning_interfaces::msg::Route>("~/local/route",1);
    local_driveable_space_pub_ = create_publisher<route_planning_interfaces::msg::DriveableSpace>("~/local/driveable_space",1);

    startup_timer_->cancel();
  }
}

bool GlobalPlanner::egoPositionSanityCheck()
{
  if((rclcpp::Clock{RCL_ROS_TIME}.now() - ego_data_.header.stamp).seconds()>2.0) // Change later to 0.2
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Latest ego-pose message is depracted (age: " << (rclcpp::Clock{RCL_ROS_TIME}.now() - ego_data_.header.stamp).seconds() << " s)!");
    return false;
  }
  else
  {
    lanelet::LaneletMapConstPtr llmap = ll2if_->getMapPtr();
    // Check if ego_data_ frame equals lanelet2 map-frame
    if(ego_data_.header.frame_id != ll2if_->map_frame_id_)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Ego-pose message (Frame: " << ego_data_.header.frame_id << ") is not given with respect to the frame of the lanelet2 map (Frame: " << ll2if_->map_frame_id_ << ")!");
      return false;
    }

    // Determine nearest lanelet
    std::vector<std::pair<double, lanelet::ConstLanelet>> nearestLanelets = lanelet::geometry::findNearest(llmap->laneletLayer, lanelet::BasicPoint2d(ego_pose_.pose.position.x, ego_pose_.pose.position.y), 5);
    if(nearestLanelets.size() < 1)
    {
      RCLCPP_ERROR(get_logger(), "No Lanelet at current ego-pose!");
      return false;
    }
    
    return true;
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
        
      // Determine start lanelet
      // Get actual Heading
      tf2::Quaternion quat;
      tf2::fromMsg(ego_pose_.pose.orientation, quat);
      float yaw = tf2::impl::getYaw(quat);
      std::vector<std::pair<double, lanelet::ConstLanelet>> nearestLanelets = lanelet::geometry::findNearest(llmap_->laneletLayer, lanelet::BasicPoint2d(ego_pose_.pose.position.x, ego_pose_.pose.position.y), 5);
      Lanelet2Utilities::laneletSorting(lanelet::BasicPoint2d(ego_pose_.pose.position.x, ego_pose_.pose.position.y), nearestLanelets, yaw, trafficRules_, {});
      start_ll_ = nearestLanelets.at(0).second; // most probable current Lanelet

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
  builtin_interfaces::msg::Time start_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  // Start and end positions
  lanelet::BasicPoint3d target;
  target.x()=maneuver_feedback_->destination_x;
  target.y()=maneuver_feedback_->destination_y;
  target.z()=0.0;
  target = lanelet::geometry::project(target_ll_.centerline(), target);
  maneuver_feedback_->destination_x = target.x();
  maneuver_feedback_->destination_y = target.y();
  lanelet::BasicPoint2d start_pos = lanelet::BasicPoint2d(ego_pose_.pose.position.x, ego_pose_.pose.position.y);
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
  RCLCPP_INFO_STREAM(get_logger(), "Time core route planning: " << (rclcpp::Clock{RCL_ROS_TIME}.now() - start_time).seconds() << "s");
  // Is route-planning possible?
  if (!!route_)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Calculated new route! Start Lanelet: " << start_ll_.id() << " | Target Lanelet: " << target_ll_.id());
    
    // Extract shortest path and its boundaries
    lanelet::routing::LaneletPath shortestPath = route_->shortestPath(); // shortestPath = sorted Lanelets

    std::pair<lanelet::BasicLineString2d, lanelet::BasicLineString2d> lane_boundaries;
    lanelet::BasicLineString2d shortest_path_centerline = Lanelet2Utilities::convertLLPath2LineString2dSBased(ConstLanelets(shortestPath.begin(), shortestPath.end()), start_pos, 10., 3., std::numeric_limits<double>::max(), ds_sample_, target_pos, lane_boundaries, *routingGraphBicycle_);
    //Start filling global route
    global_route_.header.frame_id = ll2if_->map_frame_id_;
    global_route_.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
    global_route_.target_position.x = target.x();
    global_route_.target_position.y = target.y();
    global_route_.target_position.y = target.y();
    global_route_.shortest_path = processLineString(shortest_path_centerline);

    // Process boundaries
    start_time = rclcpp::Clock{RCL_ROS_TIME}.now();
    global_driveable_space_ = sampleDriveableSpace(shortest_path_centerline);
    RCLCPP_INFO_STREAM(get_logger(), "Duration for calculation of driveable-space: " << (rclcpp::Clock{RCL_ROS_TIME}.now() - start_time).seconds() << "s");

    // Process route boundaries
    // lanelet::BasicLineString2d shortest_path_bound_left  = sampleBoundaries(shortest_path_centerline, 10., false, shortest_path_bound_left_mapping, lane_boundaries.first, marker_array_boundary);
    // lanelet::BasicLineString2d shortest_path_bound_right = sampleBoundaries(shortest_path_centerline, 10., true,  shortest_path_bound_right_mapping, lane_boundaries.second, marker_array_boundary);

    // Get regulatory elements along route

    // Process shortest path boundaries

    // Get regulatory elements along shortest path

    // Publish
    route_pub_->publish(global_route_);
    driveable_space_pub_->publish(global_driveable_space_);

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
  auto planner = std::make_shared<GlobalPlanner>();
  planner->initializeMapInterface();
  rclcpp::spin(planner);
  rclcpp::shutdown();
  return 0;
}

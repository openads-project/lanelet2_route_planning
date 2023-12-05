#include "lanelet2_route_planning/global_planner_node.hpp"

GlobalPlanner::GlobalPlanner() : Node("global_planner")
{
  startup_timer_ = create_wall_timer(0.1s, std::bind(&GlobalPlanner::initializeGlobalPlanner, this));
  map_pose_sub_ = create_subscription<perception_msgs::msg::EgoData>("/carla_its_adapter/ego_data", 1, std::bind(&GlobalPlanner::mapPoseCallback, this, std::placeholders::_1));
  goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 1, std::bind(&GlobalPlanner::goalPoseCallback, this, std::placeholders::_1));
  maneuver_action_client_ = rclcpp_action::create_client<route_planning_msgs::action::GlobalManeuver>(this, "~/execute_global_maneuver");

  // create a transform listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // load the parameters
  loadParameters();
}


void GlobalPlanner::loadParameters() {

  // General and Sanity Checks
  this->declare_parameter("ego_data_timeout", rclcpp::ParameterType::PARAMETER_DOUBLE);
  try {
    ego_data_timeout_ = this->get_parameter("ego_data_timeout").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Parameter \'ego_data_timeout\' set to: "+std::to_string(ego_data_timeout_));
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'ego_data_timeout\' is not set correctly, using default value: "+std::to_string(ego_data_timeout_));
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'ego_data_timeout\' is not set, using default value: "+std::to_string(ego_data_timeout_));
  }

  this->declare_parameter("map_server_name", rclcpp::ParameterType::PARAMETER_STRING);
  try {
    map_server_name_ = this->get_parameter("map_server_name").as_string();
    RCLCPP_INFO_STREAM(this->get_logger(), "Parameter \'map_server_name\' set to: "+map_server_name_);
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'map_server_name\' is not set correctly, using default value: "+map_server_name_);
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'map_server_name\' is not set, using default value: "+map_server_name_);
  }

  // Route Planning
  this->declare_parameter("route_sample_distance", rclcpp::ParameterType::PARAMETER_DOUBLE);
  try {
    ds_sample_ = this->get_parameter("route_sample_distance").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Parameter \'route_sample_distance\' set to: "+std::to_string(ds_sample_));
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'route_sample_distance\' is not set correctly, using default value: "+std::to_string(ds_sample_));
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'route_sample_distance\' is not set, using default value: "+std::to_string(ds_sample_));
  }

  this->declare_parameter("route_smoothing_factor", rclcpp::ParameterType::PARAMETER_DOUBLE);
  try {
    smooth_factor_ = this->get_parameter("route_smoothing_factor").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Parameter \'route_smoothing_factor\' set to: "+std::to_string(smooth_factor_));
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'route_smoothing_factor\' is not set correctly, using default value: "+std::to_string(smooth_factor_));
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'route_smoothing_factor\' is not set, using default value: "+std::to_string(smooth_factor_));
  }

  this->declare_parameter("lateral_driveable_space_width", rclcpp::ParameterType::PARAMETER_DOUBLE);
  try {
    lateral_driv_space_width_ = this->get_parameter("lateral_driveable_space_width").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Parameter \'lateral_driveable_space_width\' set to: "+std::to_string(lateral_driv_space_width_));
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'lateral_driveable_space_width\' is not set correctly, using default value: "+std::to_string(lateral_driv_space_width_));
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'lateral_driveable_space_width\' is not set, using default value: "+std::to_string(lateral_driv_space_width_));
  }

  this->declare_parameter("target_reached_thr", rclcpp::ParameterType::PARAMETER_DOUBLE);
  try {
    target_reached_thr_ = this->get_parameter("target_reached_thr").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Parameter \'target_reached_thr\' set to: "+std::to_string(target_reached_thr_));
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'target_reached_thr\' is not set correctly, using default value: "+std::to_string(target_reached_thr_));
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'target_reached_thr\' is not set, using default value: "+std::to_string(target_reached_thr_));
  }

  this->declare_parameter("require_standstill", rclcpp::ParameterType::PARAMETER_BOOL);
  try {
    require_standstill_ = this->get_parameter("require_standstill").as_bool();
    RCLCPP_INFO_STREAM(this->get_logger(), "Parameter \'require_standstill\' set to: "+std::to_string(require_standstill_));
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'require_standstill\' is not set correctly, using default value: "+std::to_string(require_standstill_));
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'require_standstill\' is not set, using default value: "+std::to_string(require_standstill_));
  }

  this->declare_parameter("vel_threshold_target", rclcpp::ParameterType::PARAMETER_DOUBLE);
  try {
    vel_threshold_target_ = this->get_parameter("vel_threshold_target").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Parameter \'vel_threshold_target\' set to: "+std::to_string(vel_threshold_target_));
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'vel_threshold_target\' is not set correctly, using default value: "+std::to_string(vel_threshold_target_));
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'vel_threshold_target\' is not set, using default value: "+std::to_string(vel_threshold_target_));
  }

  // Local Path Extraction
  this->declare_parameter("local_path_extraction_rate", rclcpp::ParameterType::PARAMETER_DOUBLE);
  try {
    path_extraction_rate_ = this->get_parameter("local_path_extraction_rate").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Parameter \'local_path_extraction_rate\' set to: "+std::to_string(path_extraction_rate_));
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'local_path_extraction_rate\' is not set correctly, using default value: "+std::to_string(path_extraction_rate_));
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'local_path_extraction_rate\' is not set, using default value: "+std::to_string(path_extraction_rate_));
  }

  this->declare_parameter("look_ahead_time", rclcpp::ParameterType::PARAMETER_DOUBLE);
  try {
    look_ahead_time_ = this->get_parameter("look_ahead_time").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Parameter \'look_ahead_time\' set to: "+std::to_string(look_ahead_time_));
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'look_ahead_time\' is not set correctly, using default value: "+std::to_string(look_ahead_time_));
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'look_ahead_time\' is not set, using default value: "+std::to_string(look_ahead_time_));
  }

  this->declare_parameter("look_ahead_distance_min", rclcpp::ParameterType::PARAMETER_DOUBLE);
  try {
    look_ahead_distance_min_ = this->get_parameter("look_ahead_distance_min").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Parameter \'look_ahead_distance_min\' set to: "+std::to_string(look_ahead_distance_min_));
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'look_ahead_distance_min\' is not set correctly, using default value: "+std::to_string(look_ahead_distance_min_));
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'look_ahead_distance_min\' is not set, using default value: "+std::to_string(look_ahead_distance_min_));
  }

  this->declare_parameter("look_behind_distance", rclcpp::ParameterType::PARAMETER_DOUBLE);
  try {
    look_behind_distance_ = this->get_parameter("look_behind_distance").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Parameter \'look_behind_distance\' set to: "+std::to_string(look_behind_distance_));
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'look_behind_distance\' is not set correctly, using default value: "+std::to_string(look_behind_distance_));
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'look_behind_distance\' is not set, using default value: "+std::to_string(look_behind_distance_));
  }
}


void GlobalPlanner::initializeMapInterface()
{
  // Important: shared_from_this() can not be called from within the constructor
  ll2if_ = new LL2MapInterface(shared_from_this(), map_server_name_);
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
    maneuver_action_server_ = rclcpp_action::create_server<route_planning_msgs::action::GlobalManeuver>(this, "~/execute_global_maneuver",
    std::bind(&GlobalPlanner::actionHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GlobalPlanner::actionHandleCancel, this, std::placeholders::_1),
    std::bind(&GlobalPlanner::actionHandleAccepted, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Created 'execute_global_maneuver' action-server!");

    route_pub_ = create_publisher<route_planning_msgs::msg::Route>("~/global/route",1);
    driveable_space_pub_ = create_publisher<route_planning_msgs::msg::DriveableSpace>("~/global/driveable_space",1);

    // local path extraction
    local_route_pub_ = create_publisher<route_planning_msgs::msg::Route>("~/local/route",1);
    local_driveable_space_pub_ = create_publisher<route_planning_msgs::msg::DriveableSpace>("~/local/driveable_space",1);

    startup_timer_->cancel();
  }
}

bool GlobalPlanner::egoPositionSanityCheck()
{
  if((now() - ego_data_.header.stamp).seconds()>ego_data_timeout_) // Change later to 0.2
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Latest ego-pose message is depracted (age: " << (now() - ego_data_.header.stamp).seconds() << " s)!");
    return false;
  }
  else
  {
    lanelet::LaneletMapConstPtr llmap = ll2if_->getMapPtr();

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
  builtin_interfaces::msg::Time start_time = now();
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
  RCLCPP_INFO_STREAM(get_logger(), "Time core route planning: " << (now() - start_time).seconds() << "s");
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
    global_route_.header.stamp = now();
    global_route_.target_position.x = target.x();
    global_route_.target_position.y = target.y();
    global_route_.target_position.y = target.y();
    global_route_.shortest_path = processLineString(shortest_path_centerline);

    // Process boundaries
    start_time = now();
    global_driveable_space_ = sampleDriveableSpace(shortest_path_centerline);
    RCLCPP_INFO_STREAM(get_logger(), "Duration for calculation of driveable-space: " << (now() - start_time).seconds() << "s");

    // Process route boundaries
    global_route_.boundaries.left.clear();
    global_route_.boundaries.right.clear();
    sampleRouteBoundary(route_.get(), shortestPath, global_route_.boundaries.left, global_route_.boundaries.right);

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

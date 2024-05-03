#include "lanelet2_route_planning/global_planner_node.hpp"

GlobalPlanner::GlobalPlanner() : Node("global_planner") {
  // create a transform listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // load the parameters
  loadParameters();

  ll2if_ = std::make_unique<LL2MapInterface>(*this, map_server_name_);
  startup_timer_ = create_wall_timer(0.1s, std::bind(&GlobalPlanner::initializeGlobalPlanner, this));
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

  this->declare_parameter("offset_behind_distance", rclcpp::ParameterType::PARAMETER_DOUBLE);
  try {
    offset_behind_distance_ = this->get_parameter("offset_behind_distance").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Parameter \'offset_behind_distance\' set to: "+std::to_string(offset_behind_distance_));
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'offset_behind_distance\' is not set correctly, using default value: "+std::to_string(offset_behind_distance_));
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'offset_behind_distance\' is not set, using default value: "+std::to_string(offset_behind_distance_));
  }

  this->declare_parameter("offset_ahead_distance", rclcpp::ParameterType::PARAMETER_DOUBLE);
  try {
    offset_ahead_distance_ = this->get_parameter("offset_ahead_distance").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Parameter \'offset_ahead_distance\' set to: "+std::to_string(offset_ahead_distance_));
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'offset_ahead_distance\' is not set correctly, using default value: "+std::to_string(offset_ahead_distance_));
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'offset_ahead_distance\' is not set, using default value: "+std::to_string(offset_ahead_distance_));
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

void GlobalPlanner::initializeGlobalPlanner()
{
  if(!ll2if_->map_loaded_)
  {
    RCLCPP_INFO(get_logger(), "Waiting for Lanelet2-Map-Interface to load map!");
    return;
  }
  else
  {
    // create subscription for ego-data
    map_pose_sub_ = create_subscription<perception_msgs::msg::EgoData>("~/ego_data", 1, std::bind(&GlobalPlanner::egoDataCallback, this, std::placeholders::_1));
    // create an action server for handling action goal requests
    maneuver_action_server_ = rclcpp_action::create_server<route_planning_msgs::action::GlobalManeuver>(this, "ll2_route_planning/execute_global_maneuver",
    std::bind(&GlobalPlanner::actionHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GlobalPlanner::actionHandleCancel, this, std::placeholders::_1),
    std::bind(&GlobalPlanner::actionHandleAccepted, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Created 'execute_global_maneuver' action-server!");

    // local path extraction
    route_pub_ = create_publisher<route_planning_msgs::msg::Route>("~/route",1);

    startup_timer_->cancel();
  }
}

void GlobalPlanner::egoDataCallback(perception_msgs::msg::EgoData::SharedPtr msg) {
  ego_data_ = *msg.get();
}

bool GlobalPlanner::deriveEgoLanelet(const perception_msgs::msg::EgoData ego_data, lanelet::ConstLanelet& start_lanelet) {
  if((now() - ego_data.header.stamp).seconds()>ego_data_timeout_) {
    RCLCPP_ERROR_STREAM(get_logger(), "Latest ego-pose message is depracted (age: " << (now() - ego_data.header.stamp).seconds() << " s)!");
    return false;
  }
  else {
    if(ego_data.header.frame_id != ll2if_->map_frame_id_) {
      RCLCPP_ERROR_STREAM(get_logger(), "Ego-pose message (Frame: " << ego_data.header.frame_id << ") is not given with respect to the frame of the lanelet2 map (Frame: " << ll2if_->map_frame_id_ << ")!");
      return false;
    }

    lanelet::LaneletMapConstPtr llmap = ll2if_->getMapPtr();
    // Determine nearest lanelet
    std::vector<std::pair<double, lanelet::ConstLanelet>> nearestLanelets = lanelet::geometry::findNearest(llmap->laneletLayer, lanelet::BasicPoint2d(perception_msgs::object_access::getX(ego_data), perception_msgs::object_access::getY(ego_data)), 5);
    if(nearestLanelets.size() < 1) {
      RCLCPP_ERROR(get_logger(), "No Lanelet at current ego-pose!");
      return false;
    }
    else {
      // Determine start lanelet
      Lanelet2Utilities::laneletSorting(lanelet::BasicPoint2d(perception_msgs::object_access::getX(ego_data), perception_msgs::object_access::getY(ego_data)), nearestLanelets, perception_msgs::object_access::getYaw(ego_data), trafficRules_, {});
      start_lanelet = nearestLanelets.at(0).second; // most probable current Lanelet
      return true;
    }
  }
}

bool GlobalPlanner::deriveDestinationLanelet(const geometry_msgs::msg::PointStamped destination, lanelet::ConstLanelet& destination_lanelet) {
  lanelet::LaneletMapConstPtr llmap = ll2if_->getMapPtr();
  std::vector<std::pair<double, lanelet::ConstLanelet>> nearestLaneletsTarget = lanelet::geometry::findNearest(llmap->laneletLayer, lanelet::BasicPoint2d(destination.point.x, destination.point.y), 5);
  if(nearestLaneletsTarget.size()<1)
  {
    RCLCPP_ERROR(get_logger(), "No Lanelet at given target-position!");
    return false;
  }
  Lanelet2Utilities::laneletSorting(lanelet::BasicPoint2d(destination.point.x, destination.point.y), nearestLaneletsTarget, {}, trafficRules_, {});
  bool b_found_target_ll = false;
  for (const auto &ll_target : nearestLaneletsTarget) {
    if (ll_target.first < 10. && trafficRules_->canPass(ll_target.second)) {
        destination_lanelet = ll_target.second;
        b_found_target_ll = true;
        return b_found_target_ll;
    }
  }
  if(!b_found_target_ll)
  {
    RCLCPP_ERROR(get_logger(), "Unable to plan a route to the given target position!");
    return b_found_target_ll;
  }
}

bool GlobalPlanner::planLaneletRoute(const perception_msgs::msg::EgoData ego_data, const geometry_msgs::msg::PointStamped destination, lanelet::routing::Route& lanelet_route, lanelet::BasicPoint3d& lanelet_destination_point) {
  lanelet::LaneletMapConstPtr llmap = ll2if_->getMapPtr();
  routing::RoutingGraphUPtr routingGraph = routing::RoutingGraph::build(*llmap, *trafficRules_);
  routing::RoutingGraphUPtr routingGraphBicycle = routing::RoutingGraph::build(*llmap, *trafficRulesBicycle_);
  std::vector<std::string> err = routingGraph->checkValidity();
  if(err.size()>0) {
    RCLCPP_ERROR(get_logger(), "Routing-Graph of given lanelet-map is invalid!");
    for(size_t i = 0; i<err.size(); ++i) {
      RCLCPP_ERROR_STREAM(get_logger(), err[i]);
    }
    return false;
  }

  // Check for global position of ego-vehicle
  lanelet::ConstLanelet start_lanelet;
  if(!deriveEgoLanelet(ego_data, start_lanelet)) return false;
  lanelet::ConstLanelet destination_lanelet;
  if(!deriveDestinationLanelet(destination, destination_lanelet)) return false;

  // Start position on centerline of start lanelet
  lanelet::BasicPoint3d start_point_on_centerline(perception_msgs::object_access::getX(ego_data), perception_msgs::object_access::getY(ego_data), 0.0);
  start_point_on_centerline = lanelet::geometry::project(start_lanelet.centerline(), start_point_on_centerline);
  lanelet::BasicPoint2d start_pos(start_point_on_centerline.x(), start_point_on_centerline.y());

  // End position on centerline of destination lanelet
  lanelet::BasicPoint3d target_point_on_centerline(destination.point.x, destination.point.y, 0.0);
  lanelet_destination_point = lanelet::geometry::project(destination_lanelet.centerline(), target_point_on_centerline);
  lanelet::BasicPoint2d target_pos(lanelet_destination_point.x(), lanelet_destination_point.y());

  // Add small offset to start
  lanelet::ConstLanelet start_ll_offset = start_lanelet;
  double remaining = lanelet::geometry::toArcCoordinates(start_ll_offset.centerline2d(), start_pos).length - offset_behind_distance_;
  while (remaining < 0.)
  {
    lanelet::ConstLanelets prevs = routingGraph->previous(start_ll_offset);
    if(!prevs.size())
    {
      RCLCPP_WARN(get_logger(), "Current matched lanelet has no predecessors!");
      break;
    }
    start_ll_offset = prevs.at(0);
    remaining += lanelet::geometry::length(start_ll_offset.centerline2d());
  }
  start_pos = lanelet::geometry::interpolatedPointAtDistance(start_ll_offset.centerline2d(), remaining);

  // Add small offset to end (so drivable space does not end directly at target)
  lanelet::ConstLanelet destination_ll_offset = destination_lanelet;
  double len_target_ll = lanelet::geometry::length(destination_ll_offset.centerline2d());
  remaining = len_target_ll - (lanelet::geometry::toArcCoordinates(destination_ll_offset.centerline2d(), target_pos).length + offset_ahead_distance_);
  while (remaining < 0.)
  {
    lanelet::ConstLanelets following_lls = routingGraph->following(destination_ll_offset, false);
    if(!following_lls.size())
    {
      RCLCPP_WARN(get_logger(), "Current target lanelet has no followers!");
      break;
    }
    destination_ll_offset = following_lls.at(0);
    len_target_ll = lanelet::geometry::length(destination_ll_offset.centerline2d());
    remaining += len_target_ll;
  }
  target_pos = lanelet::geometry::interpolatedPointAtDistance(destination_ll_offset.centerline2d(), len_target_ll - remaining);

  // Get route
  Optional<lanelet::routing::Route> llroute = routingGraph->getRoute(start_ll_offset, destination_ll_offset, 0); // 0 = routingCostId distance
  if(llroute) {
    lanelet_route = std::move(*llroute);
    return true;
  }
  else {
    RCLCPP_ERROR_STREAM(get_logger(), "Unable to plan a route from start-lanelet " << start_ll_offset.id() << " to destination-lanelet " << destination_ll_offset.id() << "!");
    return false;
  }
}

route_planning_msgs::msg::Route GlobalPlanner::processRoute(const lanelet::routing::Route ll_route, const lanelet::BasicPoint3d lanelet_destination_point) {
  rclcpp::Time start_time = now();
  // Extract shortest path and its boundaries
  lanelet::routing::LaneletPath shortestPath = ll_route.shortestPath(); // shortestPath = sorted Lanelets

  std::pair<lanelet::BasicLineString2d, lanelet::BasicLineString2d> lane_boundaries;
  lanelet::LaneletMapConstPtr llmap = ll2if_->getMapPtr();
  routing::RoutingGraphUPtr routingGraphBicycle = routing::RoutingGraph::build(*llmap, *trafficRulesBicycle_);
  lanelet::BasicLineString2d shortest_path_centerline = Lanelet2Utilities::llPath2llLineDistanceBased(ConstLanelets(shortestPath.begin(), shortestPath.end()), start_pos, 10., 3., std::numeric_limits<double>::max(), ds_sample_, target_pos, lane_boundaries, *routingGraphBicycle);
  
  //Start filling route
  route_planning_msgs::msg::Route route;
  route.header.frame_id = ll2if_->map_frame_id_;
  route.header.stamp = now();
  route.destination.x = lanelet_destination_point.x();
  route.destination.y = lanelet_destination_point.y();
  route.destination.z = lanelet_destination_point.z();
  route.traveled_route = {};
  route.remaining_route = processLineString(shortest_path_centerline);
  this->accumulateDistanceAlong2DPath(route.remaining_route);

  // Process boundaries
  route.driveable_space = sampleDriveableSpace(shortest_path_centerline);

  // Process route boundaries
  route.boundaries.left.clear();
  route.boundaries.right.clear();
  sampleRouteBoundary(ll_route, shortestPath, route.boundaries.left, route.boundaries.right);
  RCLCPP_INFO_STREAM(get_logger(), "Duration for calculation of driveable-space: " << (now() - start_time).seconds() << "s");
  return route;
}

void GlobalPlanner::publishEmptyRoute() {
  route_planning_msgs::msg::Route empty_route;
  empty_route.header.frame_id = local_vehicle_frame_id_;
  empty_route.header.stamp = now();
  route_pub_->publish(empty_route);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto planner = std::make_shared<GlobalPlanner>();
  rclcpp::spin(planner);
  rclcpp::shutdown();
  return 0;
}

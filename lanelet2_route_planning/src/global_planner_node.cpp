#include "lanelet2_route_planning/global_planner_node.hpp"

/**
 * @brief Declares and loads a ROS parameter
 *
 * @param name name
 * @param param parameter variable to load into
 * @param description description
 * @param add_to_auto_reconfigurable_params enable reconfiguration of parameter
 * @param is_required whether failure to load parameter will stop node
 * @param read_only set parameter to read-only
 * @param from_value parameter range minimum
 * @param to_value parameter range maximum
 * @param step_value parameter range step
 * @param additional_constraints additional constraints description
 */
template <typename T>
void GlobalPlanner::declareAndLoadParameter(const std::string& name,
                                            T& param,
                                            const std::string& description,
                                            const bool add_to_auto_reconfigurable_params,
                                            const bool is_required,
                                            const bool read_only,
                                            const std::optional<double>& from_value,
                                            const std::optional<double>& to_value,
                                            const std::optional<double>& step_value,
                                            const std::string& additional_constraints) {

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto type = rclcpp::ParameterValue(param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
    if constexpr(std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1);
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value())).set__step(step);
      param_desc.integer_range = {range};
    } else if constexpr(std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1.0);
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value())).set__step(step);
      param_desc.floating_point_range = {range};
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter type of parameter '%s' does not support specifying a range", name.c_str());
    }
  }

  this->declare_parameter(name, type, param_desc);

  try {
    param = this->get_parameter(name).get_value<T>();
    std::stringstream ss;
    ss << "Loaded parameter '" << name << "': ";
    if constexpr(is_vector_v<T>) {
      ss << "[";
      for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "]");
    } else {
      ss << param;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    if (is_required) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Missing required parameter '" << name << "', exiting");
      exit(EXIT_FAILURE);
    } else {
      std::stringstream ss;
      ss << "Missing parameter '" << name << "', using default value: ";
      if constexpr(is_vector_v<T>) {
        ss << "[";
        for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "]");
      } else {
        ss << param;
      }
      RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
      this->set_parameters({rclcpp::Parameter(name, rclcpp::ParameterValue(param))});
    }
  }

  if (add_to_auto_reconfigurable_params) {
    std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) {
      param = p.get_value<T>();
    };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}

/**
 * @brief Handles reconfiguration when a parameter value is changed
 *
 * @param parameters parameters
 * @return parameter change result
 */
rcl_interfaces::msg::SetParametersResult GlobalPlanner::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {

  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s'", param.get_name().c_str());
        break;
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}

GlobalPlanner::GlobalPlanner() : Node("global_planner") {
  // create a transform listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // load the parameters
  this->declareAndLoadParameter("vehicle_frame_id", vehicle_frame_id_, "Frame id of the vehicle (default: base_link)");
  this->declareAndLoadParameter("ego_data_timeout", ego_data_timeout_, "Timeout for ego data (default: 0.2)");
  this->declareAndLoadParameter("map_server_name", map_server_name_, "Name of the map server (default: ll2_map_server)");
  this->declareAndLoadParameter("route_sample_distance", ds_sample_, "Route sample distance (default: 0.5)");
  this->declareAndLoadParameter("route_smoothing_factor", smooth_factor_, "Route smoothing factor (default: 2.0)");
  this->declareAndLoadParameter("lateral_driveable_space_width", lateral_driv_space_width_, "Lateral driveable space width (default: 100.0)");
  this->declareAndLoadParameter("target_reached_thr", target_reached_thr_, "Target reached threshold (default: 1.0)");
  this->declareAndLoadParameter("require_standstill", require_standstill_, "Require standstill (default: false)");
  this->declareAndLoadParameter("vel_threshold_target", vel_threshold_target_, "Velocity threshold target (default: 1.0)");
  this->declareAndLoadParameter("offset_behind_distance", offset_behind_distance_, "Offset behind distance (default: 0.0)");
  this->declareAndLoadParameter("offset_ahead_distance", offset_ahead_distance_, "Offset ahead distance (default: 0.0)");
  this->declareAndLoadParameter("cancel_distance_ahead", cancel_distance_ahead_, "Cancel distance ahead (default: 30.0)");
  this->declareAndLoadParameter("local_path_extraction_rate", path_extraction_rate_, "Local path extraction rate (default: 10.0)");
  this->declareAndLoadParameter("look_ahead_time", look_ahead_time_, "Look ahead time (default: 10.0)");
  this->declareAndLoadParameter("look_ahead_distance_min", look_ahead_distance_min_, "Look ahead distance min (default: 50.0)");
  this->declareAndLoadParameter("look_behind_distance", look_behind_distance_, "Look behind distance (default: 20.0)");

  ll2if_ = std::make_unique<LL2MapInterface>(*this, map_server_name_);
  startup_timer_ = create_wall_timer(0.1s, std::bind(&GlobalPlanner::initializeGlobalPlanner, this));
}

void GlobalPlanner::initializeGlobalPlanner() {
  if (!ll2if_->map_loaded_) {
    RCLCPP_INFO(get_logger(), "Waiting for Lanelet2-Map-Interface to load map!");
    return;
  } else {
    // initially there is no update of lanelet2 map pending
    ll2if_->update_pending_ = false;

    // callback for dynamic parameter configuration
    parameters_callback_ = this->add_on_set_parameters_callback(std::bind(&GlobalPlanner::parametersCallback, this, std::placeholders::_1));

    // create subscription for ego-data
    map_pose_sub_ = create_subscription<perception_msgs::msg::EgoData>("~/ego_data", 1, std::bind(&GlobalPlanner::egoDataCallback, this, std::placeholders::_1));

    // create an action server for handling action goal requests
    action_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    maneuver_action_server_ = rclcpp_action::create_server<route_planning_msgs::action::GlobalManeuver>(
        this, "ll2_route_planning/execute_global_maneuver",
        std::bind(&GlobalPlanner::actionHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&GlobalPlanner::actionHandleCancel, this, std::placeholders::_1),
        std::bind(&GlobalPlanner::actionHandleAccepted, this, std::placeholders::_1),
        rcl_action_server_get_default_options(), action_callback_group_);

    // local path extraction
    route_pub_ = create_publisher<route_planning_msgs::msg::Route>("~/route", 1);

    startup_timer_->cancel();
  }
}

void GlobalPlanner::egoDataCallback(perception_msgs::msg::EgoData::SharedPtr msg) { ego_data_ = *msg.get(); }

bool GlobalPlanner::deriveEgoLanelet(const perception_msgs::msg::EgoData ego_data,
                                     lanelet::ConstLanelet& start_lanelet) {
  if ((now() - ego_data.header.stamp).seconds() > ego_data_timeout_) {
    RCLCPP_ERROR_STREAM(get_logger(), "Latest ego-pose message is too old (age: "
                                          << (now() - ego_data.header.stamp).seconds() << " s)!");
    return false;
  } else {
    if (ego_data.header.frame_id != ll2if_->map_frame_id_) {
      RCLCPP_ERROR_STREAM(get_logger(), "Ego-pose message (Frame: "
                                            << ego_data.header.frame_id
                                            << ") is not given with respect to the frame of the lanelet2 map (Frame: "
                                            << ll2if_->map_frame_id_ << ")!");
      return false;
    }

    lanelet::LaneletMapConstPtr llmap = ll2if_->getMapPtr();
    // Determine nearest lanelet
    std::vector<std::pair<double, lanelet::ConstLanelet>> nearestLanelets =
        lanelet::geometry::findNearest(llmap->laneletLayer,
                                       lanelet::BasicPoint2d(perception_msgs::object_access::getX(ego_data),
                                                             perception_msgs::object_access::getY(ego_data)),
                                       5);
    if (nearestLanelets.size() < 1) {
      RCLCPP_ERROR(get_logger(), "No Lanelet at current ego-pose!");
      return false;
    } else {
      // Determine ego lanelet
      // We're currently not using the yaw for determining the ego-lanelet, since we're also not considering the heading when matching a point to a point-list later on
      //float yaw = (float)perception_msgs::object_access::getYaw(ego_data);
      Lanelet2Utilities::laneletSorting(lanelet::BasicPoint2d(perception_msgs::object_access::getX(ego_data),
                                                              perception_msgs::object_access::getY(ego_data)),
                                        nearestLanelets, {}, trafficRules_, {});
      start_lanelet = nearestLanelets.at(0).second;  // most probable current Lanelet
      return true;
    }
  }
}

bool GlobalPlanner::deriveDestinationLanelet(const geometry_msgs::msg::PointStamped destination,
                                             lanelet::ConstLanelet& destination_lanelet) {
  lanelet::LaneletMapConstPtr llmap = ll2if_->getMapPtr();
  std::vector<std::pair<double, lanelet::ConstLanelet>> nearestLaneletsTarget = lanelet::geometry::findNearest(
      llmap->laneletLayer, lanelet::BasicPoint2d(destination.point.x, destination.point.y), 5);
  if (nearestLaneletsTarget.size() < 1) {
    RCLCPP_ERROR(get_logger(), "No Lanelet at given target-position!");
    return false;
  }
  Lanelet2Utilities::laneletSorting(lanelet::BasicPoint2d(destination.point.x, destination.point.y),
                                    nearestLaneletsTarget, {}, trafficRules_, {});
  bool b_found_target_ll = false;
  for (const auto& ll_target : nearestLaneletsTarget) {
    if (ll_target.first < 10. && trafficRules_->canPass(ll_target.second)) {
      destination_lanelet = ll_target.second;
      b_found_target_ll = true;
      return b_found_target_ll;
    }
  }
  if (!b_found_target_ll) {
    RCLCPP_ERROR(get_logger(), "Unable to plan a route to the given target position!");
    return b_found_target_ll;
  }
}

bool GlobalPlanner::egoIsOnRoute(const perception_msgs::msg::EgoData& ego_data,
                                 const lanelet::routing::Route& route) {

  lanelet::ConstLanelet ego_lanelet;
  if (!deriveEgoLanelet(ego_data, ego_lanelet)) return false;
  return route.contains(ego_lanelet);
}

bool GlobalPlanner::planLaneletRoute(const perception_msgs::msg::EgoData ego_data,
                                     const geometry_msgs::msg::PointStamped destination,
                                     lanelet::routing::Route& lanelet_route, lanelet::BasicPoint2d& start_offset_point,
                                     lanelet::BasicPoint3d& destination_on_centerline,
                                     lanelet::BasicPoint2d& destination_offset_point) {
  rclcpp::Clock wall_clock(RCL_SYSTEM_TIME);
  rclcpp::Time t0 = wall_clock.now();

  lanelet::LaneletMapConstPtr llmap = ll2if_->getMapPtr();
  routing::RoutingGraphUPtr routingGraph = routing::RoutingGraph::build(*llmap, *trafficRules_);
  routing::RoutingGraphUPtr routingGraphBicycle = routing::RoutingGraph::build(*llmap, *trafficRulesBicycle_);
  std::vector<std::string> err = routingGraph->checkValidity();
  if (err.size() > 0) {
    RCLCPP_ERROR(get_logger(), "Routing-Graph of given lanelet-map is invalid!");
    for (size_t i = 0; i < err.size(); ++i) {
      RCLCPP_ERROR_STREAM(get_logger(), err[i]);
    }
    return false;
  }

  // Check for global position of ego-vehicle
  lanelet::ConstLanelet start_lanelet;
  if (!deriveEgoLanelet(ego_data, start_lanelet)) return false;
  lanelet::ConstLanelet destination_lanelet;
  if (!deriveDestinationLanelet(destination, destination_lanelet)) return false;

  // Start position on centerline of start lanelet
  lanelet::BasicPoint3d start_point_on_centerline(perception_msgs::object_access::getX(ego_data),
                                                  perception_msgs::object_access::getY(ego_data), 0.0);
  start_point_on_centerline = lanelet::geometry::project(start_lanelet.centerline(), start_point_on_centerline);
  lanelet::BasicPoint2d start_pos(start_point_on_centerline.x(), start_point_on_centerline.y());

  // End position on centerline of destination lanelet
  lanelet::BasicPoint3d target_point_on_centerline(destination.point.x, destination.point.y, 0.0);
  destination_on_centerline = lanelet::geometry::project(destination_lanelet.centerline(), target_point_on_centerline);
  lanelet::BasicPoint2d dest_pos(destination_on_centerline.x(), destination_on_centerline.y());

  // Add small offset to start
  lanelet::ConstLanelet start_ll_offset = start_lanelet;
  double remaining =
      lanelet::geometry::toArcCoordinates(start_ll_offset.centerline2d(), start_pos).length - offset_behind_distance_;
  while (remaining < 0.) {
    lanelet::ConstLanelets prevs = routingGraph->previous(start_ll_offset);
    if (!prevs.size()) {
      RCLCPP_WARN(get_logger(), "Current matched lanelet has no predecessors!");
      break;
    }
    if (prevs.at(0).id() == destination_lanelet.id()) {
      RCLCPP_WARN(get_logger(), "Look behind distance projects onto the destination lanelet!");
      break;
    }
    start_ll_offset = prevs.at(0);
    remaining += lanelet::geometry::length(start_ll_offset.centerline2d());
  }
  start_offset_point = lanelet::geometry::interpolatedPointAtDistance(start_ll_offset.centerline2d(), remaining);

  // Add small offset to end (so drivable space does not end directly at target)
  lanelet::ConstLanelet destination_ll_offset = destination_lanelet;
  double len_target_ll = lanelet::geometry::length(destination_ll_offset.centerline2d());
  remaining =
      len_target_ll - (lanelet::geometry::toArcCoordinates(destination_ll_offset.centerline2d(), dest_pos).length +
                       offset_ahead_distance_);
  while (remaining < 0.) {
    lanelet::ConstLanelets following_lls = routingGraph->following(destination_ll_offset, false);
    if (!following_lls.size()) {
      RCLCPP_WARN(get_logger(), "Current target lanelet has no followers!");
      break;
    }
    if (following_lls.at(0).id() == start_lanelet.id()) {
      RCLCPP_WARN(get_logger(), "Look ahead distance projects onto the start lanelet!");
      break;
    }
    destination_ll_offset = following_lls.at(0);
    len_target_ll = lanelet::geometry::length(destination_ll_offset.centerline2d());
    remaining += len_target_ll;
  }
  destination_offset_point =
      lanelet::geometry::interpolatedPointAtDistance(destination_ll_offset.centerline2d(), len_target_ll - remaining);

  // Get route
  Optional<lanelet::routing::Route> llroute =
      routingGraph->getRoute(start_ll_offset, destination_ll_offset, 0);  // 0 = routingCostId distance
  if (llroute) {
    lanelet_route = std::move(*llroute);
    rclcpp::Duration duration = wall_clock.now() - t0;
    RCLCPP_INFO(this->get_logger(), "Planning of lanelet route took %.3f ms", duration.nanoseconds() / 1e6);
    return true;
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "Unable to plan a route from start-lanelet "
                                          << start_ll_offset.id() << " to destination-lanelet "
                                          << destination_ll_offset.id() << "!");
    return false;
  }
}

void GlobalPlanner::processRoute(const perception_msgs::msg::EgoData& ego_data, const lanelet::routing::Route& ll_route,
                                 const lanelet::BasicPoint2d& start_offset_point,
                                 const lanelet::BasicPoint3d& destination_on_centerline,
                                 const lanelet::BasicPoint2d& destination_offset_point,
                                 route_planning_msgs::msg::Route& route_out, int& initial_ego_pos_sample_cl_out,
                                 int& target_pos_sample_cl_out) {
  rclcpp::Clock wall_clock(RCL_SYSTEM_TIME);
  rclcpp::Time t0 = wall_clock.now();
  // Extract shortest path and its boundaries
  lanelet::routing::LaneletPath shortestPath = ll_route.shortestPath();  // shortestPath = sorted Lanelets

  // The function below is responsible to extract a 2D-Polyline describing the shortest-path from the current ego-position to the destination
  // The function handles lane-changes to adjacent lane-changes along the shortest-path.
  // Lane changes are modelled through sinus-curves sampled over a given distance definied by a lane-change velocity and time.
  // Inputs are:
  //   - the shortes-path of the route --> shortestPath
  //   - the current ego-position --> start_pos - offset_behind_distance_
  //   - the the length of a potential lane change maneuver is derived through multiplication of an velocity and a duration of the maneuver, in this case, 10 m/s for 3s lane change duration
  //   - the maximum accumulated length of the derived centerline is set to infinity
  //   - ds_sample_ indicates the step-width between two subsequent path-points
  //   - the dest_pos is the end position on the shortes path i.e. the last point of the resulting path, we extract the centerline with an additional offset offset_ahead_distance_
  // Output:
  //   - the sampled shortest-path given as lanelet::BasicLineString2d
  lanelet::BasicLineString2d offset_shortest_path_centerline = Lanelet2Utilities::llPath2llLineDistanceBased(
      ConstLanelets(shortestPath.begin(), shortestPath.end()), start_offset_point, 10.0, 3.0,
      std::numeric_limits<double>::max(), ds_sample_, destination_offset_point, {}, {});

  // Start filling route
  // this route contains the centerline including offset at start and destination!
  route_planning_msgs::msg::Route route;
  route.header.frame_id = ll2if_->map_frame_id_;
  route.header.stamp = now();
  route.destination.x = destination_on_centerline.x();
  route.destination.y = destination_on_centerline.y();
  route.destination.z = destination_on_centerline.z();
  route.traveled_route = {};
  route.remaining_route = processLineString(offset_shortest_path_centerline);
  // we calculate the accumulated distance of the whole path including the additional offsets (we will account for these offsets later)
  this->accumulateDistanceAlong2DPath(route.remaining_route);
  // identify the sample of the offset_shortest_path_centerline that equals the initial position of the ego-vehicle
  initial_ego_pos_sample_cl_out = 0;
  initial_ego_pos_sample_cl_out = findNearestSample(perception_msgs::object_access::getPosition(ego_data),
                                                    route.remaining_route, initial_ego_pos_sample_cl_out);
  // identify the sample of the offset_shortest_path_centerline that equals the target position
  target_pos_sample_cl_out = findNearestSampleReverse(route.destination, route.remaining_route);
  // Process boundaries
  route.driveable_space = sampleDriveableSpace(offset_shortest_path_centerline);

  // Process route boundaries
  sampleRouteBoundary(ll_route, shortestPath, route.boundaries.left, route.boundaries.right);

  route_out = route;

  rclcpp::Duration duration = wall_clock.now() - t0;
  RCLCPP_INFO(this->get_logger(), "Processing of lanelet route took %.3f ms", duration.nanoseconds() / 1e6);
}

void GlobalPlanner::publishEmptyRoute() {
  route_planning_msgs::msg::Route empty_route;
  empty_route.header.frame_id = vehicle_frame_id_;
  empty_route.header.stamp = now();
  route_pub_->publish(empty_route);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalPlanner>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

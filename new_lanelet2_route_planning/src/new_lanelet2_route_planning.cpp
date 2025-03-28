#include <functional>
#include <thread>
#include <utility>

#include <lanelet2_routing/RoutingGraph.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs_utils/object_access.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "new_lanelet2_route_planning/conversions.hpp"
#include "new_lanelet2_route_planning/geometry.hpp"
#include "new_lanelet2_route_planning/new_lanelet2_route_planning.hpp"
#include "new_lanelet2_route_planning/utils.hpp"

namespace new_lanelet2_route_planning {

/**
 * @brief Constructor
 *
 * @param options node options
 */
NewLanelet2RoutePlanning::NewLanelet2RoutePlanning() : Node("new_lanelet2_route_planning") {
  this->declareAndLoadParameter("ll2_map_server_name", ll2_map_server_name_, "Name of lanelet2_map_server node", false,
                                false, true);
  this->declareAndLoadParameter("publish_frequency", publish_frequency_, "Frequency of route publication [Hz]", true,
                                false, false, 0.1, 10.0, 0.1);
  this->declareAndLoadParameter("action_feedback_frequency", action_feedback_frequency_,
                                "Frequency of action feedback publication [Hz]", false, false, false, 0.1, 10.0, 0.1);
  this->declareAndLoadParameter("sampling_distance", sampling_distance_,
                                "Distance between resampled points along route [m]", true, false, false, 0.01, 10.0,
                                0.01);
  this->declareAndLoadParameter(
      "local_route_ahead_distance", local_route_ahead_distance_,
      "Distance ahead of ego position where global route is enriched with more information [m] (negative=unlimited)",
      true, false, false, -1.0, 1000.0, 0.1);
  this->declareAndLoadParameter(
      "local_route_behind_distance", local_route_behind_distance_,
      "Distance behind ego position where global route is enriched with more information [m] (negative=unlimited)",
      true, false, false, -1.0, 1000.0, 0.1);
  this->declareAndLoadParameter("route_undershoot_distance", route_undershoot_distance_,
                                "Undershoot route by this distance before ego position [m]", true, false, false);
  this->declareAndLoadParameter("route_overshoot_distance", route_overshoot_distance_,
                                "Overshoot route by this distance behind destination [m]", true, false, false);
  if (local_route_ahead_distance_ < 0.0) local_route_ahead_distance_ = std::numeric_limits<double>::infinity();
  if (local_route_behind_distance_ < 0.0) local_route_behind_distance_ = std::numeric_limits<double>::infinity();

  // initialize lanelet2 interface
  ll2_interface_ = std::make_unique<LL2MapInterface>(*this, ll2_map_server_name_);

  // delay setup to spin to allow to load the map
  delayed_setup_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&NewLanelet2RoutePlanning::setup, this));
}

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
void NewLanelet2RoutePlanning::declareAndLoadParameter(
    const std::string& name, T& param, const std::string& description, const bool add_to_auto_reconfigurable_params,
    const bool is_required, const bool read_only, const std::optional<double>& from_value,
    const std::optional<double>& to_value, const std::optional<double>& step_value,
    const std::string& additional_constraints) {
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto type = rclcpp::ParameterValue(param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
    if constexpr (std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1);
      range.set__from_value(static_cast<T>(from_value.value()))
          .set__to_value(static_cast<T>(to_value.value()))
          .set__step(step);
      param_desc.integer_range = {range};
    } else if constexpr (std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1.0);
      range.set__from_value(static_cast<T>(from_value.value()))
          .set__to_value(static_cast<T>(to_value.value()))
          .set__step(step);
      param_desc.floating_point_range = {range};
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter type of parameter '%s' does not support specifying a range",
                  name.c_str());
    }
  }

  this->declare_parameter(name, type, param_desc);

  try {
    param = this->get_parameter(name).get_value<T>();
    std::stringstream ss;
    ss << "Loaded parameter '" << name << "': ";
    if constexpr (is_vector_v<T>) {
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
      if constexpr (is_vector_v<T>) {
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
rcl_interfaces::msg::SetParametersResult NewLanelet2RoutePlanning::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s'", param.get_name().c_str());
        break;
      }
    }
  }

  // parameter-specific reconfigurations
  for (const auto& param : parameters) {
    if (param.get_name() == "publish_frequency") {
      publish_timer_->cancel();
      publish_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / publish_frequency_), std::bind(
          &NewLanelet2RoutePlanning::publishTimerCallback, this));
    }
  }
  if (local_route_ahead_distance_ < 0.0) local_route_ahead_distance_ = std::numeric_limits<double>::infinity();
  if (local_route_behind_distance_ < 0.0) local_route_behind_distance_ = std::numeric_limits<double>::infinity();

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}

/**
 * @brief Sets up subscribers, publishers, etc. to configure the node
 */
void NewLanelet2RoutePlanning::setup() {
  delayed_setup_timer_.reset();

  // build routing graph
  this->setupRoutingGraph();

  // callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(
      std::bind(&NewLanelet2RoutePlanning::parametersCallback, this, std::placeholders::_1));

  // tf transform listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // publishers
  publisher_route_ = this->create_publisher<route_planning_msgs::msg::Route>("~/route", 1);
  // TODO: start publishing only when route is available
  publish_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / publish_frequency_), std::bind(
      &NewLanelet2RoutePlanning::publishTimerCallback, this));
  is_publishing_route_ = false;

  // subscribers
  subscriber_ego_data_ = this->create_subscription<perception_msgs::msg::EgoData>(
      "~/ego_data", 1, std::bind(&NewLanelet2RoutePlanning::egoDataCallback, this, std::placeholders::_1));

  // action server for handling action goal requests
  // TODO: what is the callback group for?
  action_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_server_ = rclcpp_action::create_server<route_planning_msgs::action::GlobalManeuver>(
      this, "~/route",
      std::bind(&NewLanelet2RoutePlanning::actionHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NewLanelet2RoutePlanning::actionHandleCancel, this, std::placeholders::_1),
      std::bind(&NewLanelet2RoutePlanning::actionHandleAccepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), action_callback_group_);
}

void NewLanelet2RoutePlanning::setupRoutingGraph() {
  if (!ll2_interface_->map_loaded_) {
    RCLCPP_FATAL(this->get_logger(), "Cannot build routing graph, map not loaded by '%s'", ll2_map_server_name_.c_str());
    exit(EXIT_FAILURE);
  }

  // get map and traffic rules
  ll::LaneletMapConstPtr map = ll2_interface_->getMapPtr();
  ll::traffic_rules::TrafficRulesPtr traffic_rules = getTrafficRules();

  // build routing graph
  routing_graph_ = ll::routing::RoutingGraph::build(*map, *traffic_rules);
  ll::routing::Route::Errors errors = routing_graph_->checkValidity();
  if (errors.size() > 0) {
    RCLCPP_FATAL(this->get_logger(), "Failed to build valid routing graph");
    for (size_t i = 0; i < errors.size(); ++i) {
      RCLCPP_FATAL_STREAM(this->get_logger(), errors[i]);
    }
    exit(EXIT_FAILURE);
  }
}

void NewLanelet2RoutePlanning::egoDataCallback(const perception_msgs::msg::EgoData::SharedPtr msg) {
  // TODO: won't ego_data have to be transformed? are we ensuring same frames somewhere?
  latest_ego_data_ = *msg;

  // recompute local route
  if (is_publishing_route_) {
    bool success = this->laneletToLocalRosRoute();
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to compute local route");
    }
  }
}

// TODO: document all functions in header
void NewLanelet2RoutePlanning::publishTimerCallback() {
  if (is_publishing_route_) {
    publisher_route_->publish(latest_route_msg_);
  }
}

/**
 * @brief Processes action goal requests
 *
 * @param uuid unique goal identifier
 * @param goal action goal
 * @return goal response
 */
rclcpp_action::GoalResponse NewLanelet2RoutePlanning::actionHandleGoal(
    const rclcpp_action::GoalUUID& uuid, route_planning_msgs::action::GlobalManeuver::Goal::ConstSharedPtr goal) {
  (void)uuid;
  (void)goal;

  const geometry_msgs::msg::PointStamped& destination = goal->destination;
  RCLCPP_INFO(this->get_logger(), "Received request to plan route to destination (%.3f, %.3f, %.3f) in frame '%s'",
              destination.point.x, destination.point.y, destination.point.z, destination.header.frame_id.c_str());

  // plan route
  RCLCPP_INFO(this->get_logger(), "Planning route to destination ...");
  auto t0 = std::chrono::steady_clock::now();
  bool success = this->planRoute(destination);
  auto t1 = std::chrono::steady_clock::now();
  auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan route to destination, rejecting request");
    return rclcpp_action::GoalResponse::REJECT;
  } else {
    // TODO: change some prints to DEBUG; improve all logs in general
    RCLCPP_INFO(this->get_logger(), "Successfully planned route to destination (%.3fs)", dt);
  }

  // convert route to ROS message
  RCLCPP_INFO(this->get_logger(), "Converting route to ROS message ...");
  t0 = std::chrono::steady_clock::now();
  success = this->laneletToGlobalRosRoute();
  t1 = std::chrono::steady_clock::now();
  dt = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to convert route to ROS message, rejecting request");
    return rclcpp_action::GoalResponse::REJECT;
  } else {
    RCLCPP_INFO(this->get_logger(), "Successfully converted route to ROS message (%.3fs)", dt);
  }

  // abort current action if running
  if (action_goal_handle_ && action_goal_handle_->is_active()) {
    RCLCPP_WARN(this->get_logger(), "Existing action detected, aborting before accepting new goal");
    is_publishing_route_ = false; // stop publishing route
    action_goal_handle_->abort(action_result_);
  }

  // accept action goal request
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief Processes action cancel requests
 *
 * @param goal_handle action goal handle
 * @return cancel response
 */
rclcpp_action::CancelResponse NewLanelet2RoutePlanning::actionHandleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_msgs::action::GlobalManeuver>> goal_handle) {
  (void)goal_handle;

  RCLCPP_INFO(this->get_logger(), "Received request to cancel action goal");
  is_publishing_route_ = false; // stop publishing route

  return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief Processes accepted action goal requests
 *
 * @param goal_handle action goal handle
 */
void NewLanelet2RoutePlanning::actionHandleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_msgs::action::GlobalManeuver>> goal_handle) {
  action_goal_handle_ = goal_handle;

  // initialize feedback and result
  action_start_time_ = this->now();
  action_feedback_ = std::make_shared<route_planning_msgs::action::GlobalManeuver::Feedback>();
  action_feedback_->distance_traveled = 0.0;
  action_feedback_->distance_remaining = 0.0;
  if (!latest_route_msg_.remaining_route_elements.empty()) {
    action_feedback_->distance_remaining = latest_route_msg_.remaining_route_elements.back().s;
  }
  action_feedback_->time_traveled = rclcpp::Duration::from_seconds(0.0);
  action_feedback_->time_remaining = rclcpp::Duration::from_seconds(0.0);  // TODO: compute with speed limit along route
  action_result_ = std::make_shared<route_planning_msgs::action::GlobalManeuver::Result>();
  action_result_->distance_traveled = 0.0;
  action_result_->time_traveled = rclcpp::Duration::from_seconds(0.0);
  action_result_->destination_reached = false;

  // start publishing route
  is_publishing_route_ = true;

  // execute action in a separate thread to avoid blocking
  std::thread{std::bind(&NewLanelet2RoutePlanning::actionExecute, this, std::placeholders::_1), goal_handle}.detach();
}

/**
 * @brief Executes an action
 *
 * @param goal_handle action goal handle
 */
void NewLanelet2RoutePlanning::actionExecute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_msgs::action::GlobalManeuver>> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing action goal");

  rclcpp::Rate feedback_rate(action_feedback_frequency_);
  bool has_reached_destination = false;
  while (goal_handle->is_executing() && !goal_handle->is_canceling() && !has_reached_destination) {
    // check if destination reached
    // TODO: check by comparing ego-position to destination with threshold

    // update feedback and result
    // TODO: distance only accurate to RouteElements, but accurate enough?
    if (!latest_route_msg_.traveled_route_elements.empty()) {
      action_feedback_->distance_traveled = latest_route_msg_.traveled_route_elements.back().s;
    }
    if (!latest_route_msg_.remaining_route_elements.empty()) {
      action_feedback_->distance_remaining =
          latest_route_msg_.remaining_route_elements.back().s - action_feedback_->distance_traveled;
    }
    action_feedback_->time_traveled = this->now() - action_start_time_;
    action_feedback_->time_remaining =
        rclcpp::Duration::from_seconds(0.0);  // TODO: compute with speed limit along route
    action_result_->distance_traveled = action_feedback_->distance_traveled;
    action_result_->time_traveled = action_feedback_->time_traveled;

    // publish feedback
    goal_handle->publish_feedback(action_feedback_);
    feedback_rate.sleep();
  }

  // prepare result
  if (!latest_route_msg_.traveled_route_elements.empty()) {
    action_result_->distance_traveled = latest_route_msg_.traveled_route_elements.back().s;
  }
  action_result_->time_traveled = this->now() - action_start_time_;
  action_result_->destination_reached = has_reached_destination;

  // publish result
  if (goal_handle->is_canceling()) {
    // TODO: reroute to a few meters ahead if canceling? or rather handle the safe stop in simple_planner?
    goal_handle->canceled(action_result_);
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
  } else if (!goal_handle->is_executing()) {
    RCLCPP_INFO(this->get_logger(), "Goal aborted");
  } else if (rclcpp::ok()) {
    is_publishing_route_ = false; // stop publishing route
    goal_handle->succeed(action_result_);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

bool NewLanelet2RoutePlanning::planRoute(const geometry_msgs::msg::PointStamped& destination) {
  if (!ll2_interface_->map_loaded_) {
    RCLCPP_ERROR(this->get_logger(), "Cannot plan route, map not loaded by '%s'", ll2_map_server_name_.c_str());
    return false;
  }

  // transform destination to map frame
  geometry_msgs::msg::PointStamped destination_map_stamped;
  try {
    destination_map_stamped = tf_buffer_->transform(destination, ll2_interface_->map_frame_id_);
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform destination from frame '%s' to frame '%s': %s",
                 destination.header.frame_id.c_str(), ll2_interface_->map_frame_id_.c_str(), ex.what());
    return false;
  }
  geometry_msgs::msg::Point& destination_map = destination_map_stamped.point;

  // get map and traffic rules
  ll::LaneletMapConstPtr map = ll2_interface_->getMapPtr();
  ll::traffic_rules::TrafficRulesPtr traffic_rules = getTrafficRules();

  // check validity of ego data
  const double timeout_ego_data = 1.0;
  if ((this->now() - latest_ego_data_.header.stamp).seconds() > timeout_ego_data) {
    RCLCPP_WARN(this->get_logger(), "Ego data is outdated by %.3fs > %.3fs", (this->now() - latest_ego_data_.header.stamp).seconds(),
                timeout_ego_data);
  }
  if (latest_ego_data_.header.frame_id != ll2_interface_->map_frame_id_) {
    RCLCPP_ERROR(this->get_logger(), "Ego data frame '%s' does not match map frame '%s'", latest_ego_data_.header.frame_id.c_str(),
                 ll2_interface_->map_frame_id_.c_str());
    return false;
  }

  // project ego position to lanelet
  ll::ConstLanelet ego_ll;
  if (auto result = laneletAtPoint(toEigen2d(egoPosition(latest_ego_data_)), map)) {
    ego_ll = *result;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to find lanelet at ego position");
    return false;
  }
  Eigen::Vector2d ego_ll_position = projectPointToLineString(toEigen2d(egoPosition(latest_ego_data_)), toEigen(ego_ll.centerline2d().basicLineString()));

  // project destination to lanelet
  ll::ConstLanelet destination_ll;
  if (auto result = laneletAtPoint(toEigen2d(destination_map), map)) {
    destination_ll = *result;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to find lanelet at destination");
    return false;
  }
  Eigen::Vector2d destination_ll_position = projectPointToLineString(toEigen2d(destination_map), toEigen(destination_ll.centerline2d().basicLineString()));

  // undershoot/overshoot route endpoints
  // TODO: still needed?
  ll::ConstLanelet undershot_ego_ll = followLaneletsAlongRoutingGraph(routing_graph_, ego_ll, ego_ll_position, -std::abs(route_undershoot_distance_));
  ll::ConstLanelet overshot_destination_ll = followLaneletsAlongRoutingGraph(routing_graph_, destination_ll, destination_ll_position, route_overshoot_distance_);

  // TODO: check that start/end are not the same lanelet (?) (see L314 in global_planner_node.cpp)

  // plan route
  const int routing_cost_id = 0;  // RoutingCostDistance
  auto planned_route = routing_graph_->getRoute(undershot_ego_ll, overshot_destination_ll, routing_cost_id);
  if (planned_route) {
    destination_ = destination_map;
    latest_route_ = std::move(*planned_route);
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan route from lanelet %ld to lanelet %ld", ego_ll.id(),
                 destination_ll.id());
    return false;
  }
}

bool NewLanelet2RoutePlanning::laneletToGlobalRosRoute() {
  // create Route message
  route_planning_msgs::msg::Route route_msg;
  route_msg.header.stamp = this->now();
  route_msg.header.frame_id = ll2_interface_->map_frame_id_;
  route_msg.destination = destination_;
  route_msg.traveled_route_elements = {};
  route_msg.remaining_route_elements = {};

  // get shortest path
  ll::routing::LaneletPath shortest_path = latest_route_.shortestPath();

  // resample centerlines along shortest path to accumulate global reference line
  auto resampling_result = resampleCenterlinesAlongPath(
    shortest_path, sampling_distance_, true);
  std::vector<Eigen::Vector2d> shortest_path_centerline = resampling_result.centerline;
  latest_lanelet_idx_by_reference_line_point_idx_ = resampling_result.lanelet_idx_by_point;

  // fill route message with global reference line
  double accumulated_distance = 0;
  for (size_t c = 0; c < shortest_path_centerline.size(); ++c) {
    // get current, previous and next centerline point
    const Eigen::Vector2d& point = shortest_path_centerline[c];
    const Eigen::Vector2d& prev_point = (c > 0) ? shortest_path_centerline[c - 1] : point;
    const Eigen::Vector2d& next_point =
        (c < shortest_path_centerline.size() - 1) ? shortest_path_centerline[c + 1] : point;

    // get lanelet corresponding to centerline point
    const ll::ConstLanelet& lanelet = shortest_path[latest_lanelet_idx_by_reference_line_point_idx_[c]];

    // identify lane changes based on break in equidistant centerline
    bool changes_lane_from_prev_point = changesLaneFromPointToPoint(prev_point, point, sampling_distance_);
    bool changes_lane_to_next_point = changesLaneFromPointToPoint(point, next_point, sampling_distance_);

    // compute orientation of centerline point
    Eigen::Vector2d prev_point_for_orientation = changes_lane_from_prev_point ? point : prev_point;
    Eigen::Vector2d next_point_for_orientation = changes_lane_to_next_point ? point : next_point;
    Eigen::Vector2d orientation =
        tangentOfPointAlongLineString(point, prev_point_for_orientation, next_point_for_orientation);

    // determine speed limit
    uint8_t speed_limit = speedLimit(lanelet);

    // accumulate distance
    accumulated_distance += (point - prev_point).norm();

    // create RouteElement
    route_planning_msgs::msg::RouteElement route_element_msg = createMinimalRouteElement(toRos(point), toRosQuaternion(orientation), accumulated_distance, changes_lane_to_next_point, speed_limit);
    route_msg.remaining_route_elements.push_back(route_element_msg);
  }

  latest_route_msg_ = route_msg;
  return true;
}

// TODO: rename this function?
bool NewLanelet2RoutePlanning::laneletToLocalRosRoute() {
  // join traveled and remaining route elements
  route_planning_msgs::msg::Route route_msg = latest_route_msg_;
  std::vector<route_planning_msgs::msg::RouteElement> route_elements = route_msg.traveled_route_elements;
  route_elements.insert(route_elements.end(), route_msg.remaining_route_elements.begin(),
                        route_msg.remaining_route_elements.end());

  // find point of global reference line closest to ego position
  const Eigen::Vector2d ego_position = toEigen2d(egoPosition(latest_ego_data_));
  const std::vector<Eigen::Vector2d> reference_line = to2d(suggestedReferenceLineToEigen(route_msg));
  size_t c_min_distance_ego_to_route = indexOfLineStringPointClosestToPoint(reference_line, ego_position);

  // loop over global reference line
  for (size_t c = 0; c < route_elements.size(); ++c) {
    route_planning_msgs::msg::RouteElement& route_element_msg = route_elements[c];
    route_planning_msgs::msg::LaneElement& lane_element_msg =
        route_element_msg.lane_elements[route_element_msg.suggested_lane_idx];

    // only consider route elements within local_route_behind_distance_ and local_route_ahead_distance_ for local route
    double distance_ahead = route_element_msg.s - route_elements[c_min_distance_ego_to_route].s;
    if (distance_ahead > local_route_ahead_distance_ || distance_ahead < -local_route_behind_distance_) {
      // clear local route enriched information, if existing
      route_element_msg = createMinimalRouteElement(lane_element_msg.reference_pose.position, lane_element_msg.reference_pose.orientation, route_element_msg.s, route_element_msg.will_change_suggested_lane, lane_element_msg.speed_limit);
      continue;
    }

    // skip recomputation of local route if already enriched
    if (lane_element_msg.has_left_boundary) {
      continue;
    }

    // get current, previous and next centerline point
    // TODO: msg access function to get suggested lane element
    route_planning_msgs::msg::LaneElement& prev_lane_element_msg =
        (c > 0) ? route_elements[c - 1].lane_elements[route_elements[c - 1].suggested_lane_idx] : lane_element_msg;
    route_planning_msgs::msg::LaneElement& next_lane_element_msg =
        (c < route_elements.size() - 1) ? route_elements[c + 1].lane_elements[route_elements[c + 1].suggested_lane_idx]
                                        : lane_element_msg;
    const Eigen::Vector2d point = toEigen2d(lane_element_msg.reference_pose.position);
    const Eigen::Vector2d prev_point = toEigen2d(prev_lane_element_msg.reference_pose.position);
    const Eigen::Vector2d next_point = toEigen2d(next_lane_element_msg.reference_pose.position);

    // get lanelet corresponding to centerline point
    ll::routing::LaneletPath shortest_path = latest_route_.shortestPath();
    const ll::ConstLanelet& lanelet = shortest_path[latest_lanelet_idx_by_reference_line_point_idx_[c]];

    // identify lane changes
    bool changes_lane_from_prev_point = changesLaneFromPointToPoint(prev_point, point, sampling_distance_);
    bool changes_lane_to_next_point = changesLaneFromPointToPoint(point, next_point, sampling_distance_);

    // determine neighboring points for projection
    Eigen::Vector2d prev_point_for_projection = changes_lane_from_prev_point ? point : prev_point;
    Eigen::Vector2d next_point_for_projection = changes_lane_to_next_point ? point : next_point;

    // compute orientation of centerline point
    Eigen::Vector2d orientation =
        tangentOfPointAlongLineString(point, prev_point_for_projection, next_point_for_projection);

    // get adjacent lanelets
    // TODO: this is re-executed for every point on the same lanelet
    std::vector<ll::ConstLanelet> adjacent_left_lanelets = adjacentLeftOrRightLanelets(lanelet, latest_route_, true);
    std::vector<ll::ConstLanelet> adjacent_right_lanelets = adjacentLeftOrRightLanelets(lanelet, latest_route_, false);
    int suggested_lane_idx = adjacent_left_lanelets.size();

    // project centerline point to lanelet and adjacent lanelet centerlines and bounds
    auto lanelet_projected_points = projectPointToLaneletLines(point, prev_point_for_projection, next_point_for_projection, std::vector<lanelet::ConstLanelet>{lanelet})[0];
    auto adjacent_left_lanelets_projected_points = projectPointToLaneletLines(point, prev_point_for_projection, next_point_for_projection, adjacent_left_lanelets);
    auto adjacent_right_lanelets_projected_points = projectPointToLaneletLines(point, prev_point_for_projection, next_point_for_projection, adjacent_right_lanelets);

    // compute offset of lane element indices from current to next route element
    const ll::ConstLanelet& lanelet_of_next_point =
        (c < route_elements.size() - 1) ? shortest_path[latest_lanelet_idx_by_reference_line_point_idx_[c + 1]]
                                        : lanelet;
    int following_lane_idx_offset =
        computeFollowingLaneIdxOffset(lanelet, lanelet_of_next_point, latest_route_, routing_graph_);

    // extract drivable space
    // TODO: extract actual drivable space by shooting
    Eigen::Vector2d drivable_space_left = adjacent_left_lanelets_projected_points.empty()
                                           ? lanelet_projected_points.left_bound_point
                                           : adjacent_left_lanelets_projected_points.front().left_bound_point;
    Eigen::Vector2d drivable_space_right = adjacent_right_lanelets_projected_points.empty()
                                            ? lanelet_projected_points.right_bound_point
                                            : adjacent_right_lanelets_projected_points.back().right_bound_point;

    // extract regulatory elements
    // TODO: extract RegElems for adjacent lanes as well
    std::vector<route_planning_msgs::msg::RegulatoryElement> regulatory_elements = extractRegulatoryElements(lanelet, point, prev_point, next_point);

    // enrich RouteElement with local route information
    route_element_msg.lane_elements = {};
    route_element_msg.left_boundary = toRos(drivable_space_left);
    route_element_msg.right_boundary = toRos(drivable_space_right);
    route_element_msg.regulatory_elements = regulatory_elements;
    route_element_msg.suggested_lane_idx = suggested_lane_idx;
    route_element_msg.will_change_suggested_lane = changes_lane_to_next_point;
    // route_element_msg.s already set in global route

    // create LaneElements for left adjacent lanes
    for (size_t a = 0; a < adjacent_left_lanelets_projected_points.size(); ++a) {
      route_planning_msgs::msg::LaneElement lane_element_msg;
      lane_element_msg.reference_pose.position = toRos(adjacent_left_lanelets_projected_points[a].centerline_point);
      lane_element_msg.reference_pose.orientation = geometry_msgs::msg::Quaternion();  // TODO
      lane_element_msg.left_boundary.point = toRos(adjacent_left_lanelets_projected_points[a].left_bound_point);
      lane_element_msg.left_boundary.type = laneBoundaryType(adjacent_left_lanelets[a].leftBound2d());
      lane_element_msg.has_left_boundary = true;
      lane_element_msg.right_boundary.point = toRos(adjacent_left_lanelets_projected_points[a].right_bound_point);
      lane_element_msg.right_boundary.type = laneBoundaryType(adjacent_left_lanelets[a].rightBound2d());
      lane_element_msg.has_right_boundary = true;
      lane_element_msg.speed_limit = speedLimit(adjacent_left_lanelets[a]);
      lane_element_msg.regulatory_element_idx = {};  // TODO
      lane_element_msg.following_lane_idx = route_element_msg.lane_elements.size() + following_lane_idx_offset;
      lane_element_msg.has_following_lane_idx = true;
      route_element_msg.lane_elements.push_back(lane_element_msg);
    }

    // create LaneElement for centerline lane
    route_planning_msgs::msg::LaneElement centerline_lane_element_msg;
    centerline_lane_element_msg.reference_pose.position = toRos(point);
    centerline_lane_element_msg.reference_pose.orientation = toRosQuaternion(orientation);
    centerline_lane_element_msg.left_boundary.point = toRos(lanelet_projected_points.left_bound_point);
    centerline_lane_element_msg.left_boundary.type = laneBoundaryType(lanelet.leftBound2d());
    centerline_lane_element_msg.has_left_boundary = true;
    centerline_lane_element_msg.right_boundary.point = toRos(lanelet_projected_points.right_bound_point);
    centerline_lane_element_msg.right_boundary.type = laneBoundaryType(lanelet.rightBound2d());
    centerline_lane_element_msg.has_right_boundary = true;
    centerline_lane_element_msg.speed_limit = speedLimit(lanelet);
    centerline_lane_element_msg.regulatory_element_idx.resize(route_element_msg.regulatory_elements.size()); // TODO: dont assign all RegElems of RouteElem to CenterlineElem
    std::iota(centerline_lane_element_msg.regulatory_element_idx.begin(), centerline_lane_element_msg.regulatory_element_idx.end(), 0);
    centerline_lane_element_msg.following_lane_idx = route_element_msg.lane_elements.size() + following_lane_idx_offset;
    centerline_lane_element_msg.has_following_lane_idx = true;
    route_element_msg.lane_elements.push_back(centerline_lane_element_msg);

    // create LaneElements for right adjacent lanes
    for (size_t a = 0; a < adjacent_right_lanelets_projected_points.size(); ++a) {
      route_planning_msgs::msg::LaneElement lane_element_msg;
      lane_element_msg.reference_pose.position = toRos(adjacent_right_lanelets_projected_points[a].centerline_point);
      lane_element_msg.reference_pose.orientation = geometry_msgs::msg::Quaternion();  // TODO
      lane_element_msg.left_boundary.point = toRos(adjacent_right_lanelets_projected_points[a].left_bound_point);
      lane_element_msg.left_boundary.type = laneBoundaryType(adjacent_right_lanelets[a].leftBound2d());
      lane_element_msg.has_left_boundary = true;
      lane_element_msg.right_boundary.point = toRos(adjacent_right_lanelets_projected_points[a].right_bound_point);
      lane_element_msg.right_boundary.type = laneBoundaryType(adjacent_right_lanelets[a].rightBound2d());
      lane_element_msg.has_right_boundary = true;
      lane_element_msg.speed_limit = speedLimit(adjacent_right_lanelets[a]);
      lane_element_msg.regulatory_element_idx = {};  // TODO
      lane_element_msg.following_lane_idx = route_element_msg.lane_elements.size() + following_lane_idx_offset;
      lane_element_msg.has_following_lane_idx = true;
      route_element_msg.lane_elements.push_back(lane_element_msg);
    }
  }

  // split in traveled and remaining route elements
  route_msg.traveled_route_elements = {};
  route_msg.remaining_route_elements = {};
  route_msg.traveled_route_elements.insert(route_msg.traveled_route_elements.end(), route_elements.begin(),
                                           route_elements.begin() + c_min_distance_ego_to_route + 1);
  route_msg.remaining_route_elements.insert(route_msg.remaining_route_elements.end(),
                                            route_elements.begin() + c_min_distance_ego_to_route + 1,
                                            route_elements.end());
  route_msg.header.stamp = latest_ego_data_.header.stamp;

  // save as latest route message
  latest_route_msg_ = route_msg;
  return true;
}

}  // namespace new_lanelet2_route_planning

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<new_lanelet2_route_planning::NewLanelet2RoutePlanning>());
  rclcpp::shutdown();

  return 0;
}

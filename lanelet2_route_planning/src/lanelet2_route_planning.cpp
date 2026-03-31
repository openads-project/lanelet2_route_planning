// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <functional>
#include <thread>
#include <tuple>
#include <utility>

#include <lanelet2_routing/RoutingGraph.h>
#include <omp.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs_utils/object_access.hpp>
#include <route_planning_msgs_utils/route_access.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_perception_msgs/tf2_perception_msgs.hpp>

#include "lanelet2_route_planning/conversions.hpp"
#include "lanelet2_route_planning/geometry.hpp"
#include "lanelet2_route_planning/lanelet2_route_planning.hpp"
#include "lanelet2_route_planning/utils.hpp"

namespace lanelet2_route_planning {

Lanelet2RoutePlanning::Lanelet2RoutePlanning() : Node("lanelet2_route_planning") {
  this->declareAndLoadParameter("ll2_map_server_name", ll2_map_server_name_, "Name of lanelet2_map_server node", false, false,
                                true);
  this->declareAndLoadParameter("publish_frequency", publish_frequency_, "Frequency of route publication [Hz]", true, false,
                                false, 0.1, 20.0);
  this->declareAndLoadParameter("action_feedback_frequency", action_feedback_frequency_,
                                "Frequency of action feedback publication [Hz]", true, false, false, 0.1, 20.0);
  this->declareAndLoadParameter("sampling_distance", sampling_distance_, "Distance between resampled points along route [m]",
                                true, false, false, 0.1, 3.0);
  this->declareAndLoadParameter("project_destination_to_reference_line", project_destination_to_reference_line_,
                                "Whether to project destination to reference line", true, false, false);
  this->declareAndLoadParameter("destination_distance_threshold", destination_distance_threshold_,
                                "Distance to destination where destination is considered reached [m]", true, false, false, 0.1,
                                10.0);
  this->declareAndLoadParameter(
      "required_traveled_distance_proportion", required_traveled_distance_proportion_,
      "Proportion of route length that must have been traveled before considering destination reached [0..1]", true, false, false,
      0.0, 1.0);
  this->declareAndLoadParameter(
      "enrich_route_ahead_ego_distance", enrich_route_ahead_ego_distance_,
      "Distance ahead of ego position where global route is enriched with more information [m] (negative=unlimited)", true, false,
      false, -1.0, 1000.0);
  this->declareAndLoadParameter(
      "enrich_route_behind_ego_distance", enrich_route_behind_ego_distance_,
      "Distance behind ego position where global route is enriched with more information [m] (negative=unlimited)", true, false,
      false, -1.0, 1000.0);
  this->declareAndLoadParameter("route_undershoot_distance", route_undershoot_distance_,
                                "Undershoot route by this distance before ego position [m]", true, false, false, 0.0, 50.0);
  this->declareAndLoadParameter("route_overshoot_distance", route_overshoot_distance_,
                                "Overshoot route by this distance behind destination [m]", true, false, false, 0.0, 100.0);
  this->declareAndLoadParameter("max_drivable_space_radius", max_drivable_space_radius_,
                                "Maximum distance to left/right drivable space bounds, if not otherwise restricted [m]", true,
                                false, false, 3.0, 100.0);
  this->declareAndLoadParameter("max_num_threads", max_num_threads_,
                                "Maximum number of threads for parallel processing (0=max available)", true, false, false, 0,
                                omp_get_max_threads(), 1);
  this->declareAndLoadParameter("transform_timeout", transform_timeout_, "How long to wait for a transform to be available [s]",
                                true, false, false, 0.0, 1.0);
  if (enrich_route_ahead_ego_distance_ < 0.0) {
    enrich_route_ahead_ego_distance_ = std::numeric_limits<double>::infinity();
  }
  if (enrich_route_behind_ego_distance_ < 0.0) {
    enrich_route_behind_ego_distance_ = std::numeric_limits<double>::infinity();
  }
  if (max_num_threads_ <= 0) {
    max_num_threads_ = omp_get_max_threads();
    omp_set_num_threads(max_num_threads_);
  }

  this->setup();
}

template <typename T>
void Lanelet2RoutePlanning::declareAndLoadParameter(const std::string& name,
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
    if constexpr (std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value()));
      if (step_value.has_value()) range.set__step(static_cast<T>(step_value.value()));
      param_desc.integer_range = {range};
    } else if constexpr (std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value()));
      if (step_value.has_value()) range.set__step(static_cast<T>(step_value.value()));
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
    if constexpr (is_vector_v<T>) {
      ss << "[";
      for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
      ss << "]";
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
        for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
        ss << "]";
      } else {
        ss << param;
      }
      RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
      this->set_parameters({rclcpp::Parameter(name, rclcpp::ParameterValue(param))});
    }
  }

  if (add_to_auto_reconfigurable_params) {
    std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) { param = p.get_value<T>(); };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}

rcl_interfaces::msg::SetParametersResult Lanelet2RoutePlanning::parametersCallback(
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
      publish_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / publish_frequency_),
                                               std::bind(&Lanelet2RoutePlanning::publishTimerCallback, this));
    }
  }
  if (enrich_route_ahead_ego_distance_ < 0.0) {
    enrich_route_ahead_ego_distance_ = std::numeric_limits<double>::infinity();
  }
  if (enrich_route_behind_ego_distance_ < 0.0) {
    enrich_route_behind_ego_distance_ = std::numeric_limits<double>::infinity();
  }
  if (max_num_threads_ <= 0) {
    max_num_threads_ = omp_get_max_threads();
  }
  omp_set_num_threads(max_num_threads_);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}

bool Lanelet2RoutePlanning::checkMap(bool handle_update) {
  bool map_status = ll2_interface_->map_loaded_;
  // update routing graph on map update
  if (handle_update && ll2_interface_->update_pending_ && ll2_interface_->map_loaded_) {
    if (this->buildRoutingGraph()) {
      ll2_interface_->update_pending_ = false;
    }
    map_status = map_status && !ll2_interface_->update_pending_;
  }
  return map_status;
}

void Lanelet2RoutePlanning::setup() {
  // map interface
  ll2_interface_ = std::make_unique<LL2MapInterface>(*this, ll2_map_server_name_);

  // callback for dynamic parameter configuration
  parameters_callback_ =
      this->add_on_set_parameters_callback(std::bind(&Lanelet2RoutePlanning::parametersCallback, this, std::placeholders::_1));

  // tf transform listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // publishers
  publisher_route_ = this->create_publisher<route_planning_msgs::msg::Route>("~/route", 1);
  publish_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / publish_frequency_),
                                           std::bind(&Lanelet2RoutePlanning::publishTimerCallback, this));
  is_publishing_route_ = false;
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", publisher_route_->get_topic_name());

  // subscribers
  subscriber_ego_data_ = this->create_subscription<perception_msgs::msg::EgoData>(
      "~/ego_data", 1, std::bind(&Lanelet2RoutePlanning::egoDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", subscriber_ego_data_->get_topic_name());

  // action server for handling action goal requests
  action_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_server_ = rclcpp_action::create_server<route_planning_msgs::action::PlanRoute>(
      this, "~/plan_route",
      std::bind(&Lanelet2RoutePlanning::actionHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Lanelet2RoutePlanning::actionHandleCancel, this, std::placeholders::_1),
      std::bind(&Lanelet2RoutePlanning::actionHandleAccepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), action_callback_group_);
  RCLCPP_INFO(this->get_logger(), "Action server started");
}

bool Lanelet2RoutePlanning::buildRoutingGraph() {
  if (!this->checkMap(false)) {
    RCLCPP_ERROR(this->get_logger(), "Cannot build routing graph, map not loaded by '%s'", ll2_map_server_name_.c_str());
    return false;
  }

  // get map and traffic rules
  lanelet::LaneletMapConstPtr map = ll2_interface_->getMapPtr();
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules = getTrafficRules();

  // build routing graph
  routing_graph_ = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);
  lanelet::routing::Route::Errors errors = routing_graph_->checkValidity();
  if (errors.size() > 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to build valid routing graph");
    for (size_t i = 0; i < errors.size(); ++i) {
      RCLCPP_ERROR_STREAM(this->get_logger(), errors[i]);
    }
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Successfully built routing graph");
  return true;
}

void Lanelet2RoutePlanning::egoDataCallback(const perception_msgs::msg::EgoData::SharedPtr msg) {
  if (!this->checkMap(false)) {
    return;
  }

  // transform ego data to map frame
  if (msg->header.frame_id != ll2_interface_->map_frame_id_) {
    try {
      latest_ego_data_ = tf_buffer_->transform(*msg, ll2_interface_->map_frame_id_, tf2::durationFromSec(transform_timeout_));
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform ego data from frame '%s' to frame '%s': %s",
                   msg->header.frame_id.c_str(), ll2_interface_->map_frame_id_.c_str(), ex.what());
    }
  } else {
    latest_ego_data_ = *msg;
  }

  // recompute local route
  if (is_publishing_route_) {
    auto t0 = std::chrono::steady_clock::now();
    this->buildEnrichedRouteMessage();
    auto t1 = std::chrono::steady_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();
    RCLCPP_DEBUG(this->get_logger(), "Recomputed route (%.3fs)", dt);
  }
}

void Lanelet2RoutePlanning::publishTimerCallback() {
  if (is_publishing_route_ && has_enriched_route_) {
    publisher_route_->publish(latest_route_msg_);
  }
}

rclcpp_action::GoalResponse Lanelet2RoutePlanning::actionHandleGoal(
    const rclcpp_action::GoalUUID& uuid, route_planning_msgs::action::PlanRoute::Goal::ConstSharedPtr goal) {
  (void)uuid;
  (void)goal;

  const geometry_msgs::msg::PointStamped& destination = goal->destination;
  const std::vector<geometry_msgs::msg::PointStamped>& intermediate_destinations = goal->intermediate_destinations;
  RCLCPP_INFO(this->get_logger(), "Received request to plan route to destination (%.3f, %.3f, %.3f) in frame '%s'",
              destination.point.x, destination.point.y, destination.point.z, destination.header.frame_id.c_str());

  // check for and handle map updates
  if (!this->checkMap(true)) {
    RCLCPP_ERROR(this->get_logger(), "Cannot plan route, map not loaded by '%s'", ll2_map_server_name_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  // plan route
  auto t0 = std::chrono::steady_clock::now();
  bool success = this->planRoute(destination, intermediate_destinations);
  auto t1 = std::chrono::steady_clock::now();
  auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan route to destination, rejecting request");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // convert route to ROS message
  this->buildGlobalRouteMessage();
  RCLCPP_INFO(this->get_logger(), "Successfully planned route to destination (%.3fs)", dt);

  // abort current action if running
  if (action_goal_handle_ && action_goal_handle_->is_active()) {
    RCLCPP_WARN(this->get_logger(), "Existing action detected, aborting before accepting new goal");
    is_publishing_route_ = false;  // stop publishing route
    action_goal_handle_->abort(action_result_);
    action_goal_handle_.reset();
  }

  // accept action goal request
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Lanelet2RoutePlanning::actionHandleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_msgs::action::PlanRoute>> goal_handle) {
  (void)goal_handle;

  RCLCPP_INFO(this->get_logger(), "Received request to cancel action goal");
  is_publishing_route_ = false;  // stop publishing route

  return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief Processes accepted action goal requests
 *
 * @param goal_handle action goal handle
 */
void Lanelet2RoutePlanning::actionHandleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_msgs::action::PlanRoute>> goal_handle) {
  action_goal_handle_ = goal_handle;

  // initialize feedback and result
  action_start_time_ = this->now();
  action_feedback_ = std::make_shared<route_planning_msgs::action::PlanRoute::Feedback>();
  action_feedback_->distance_traveled = 0.0;
  action_feedback_->distance_remaining = 0.0;
  action_feedback_->distance_remaining = distanceRemaining(latest_route_msg_);
  action_feedback_->time_traveled = rclcpp::Duration::from_seconds(0.0);
  action_feedback_->time_remaining = rclcpp::Duration::from_seconds(estimateRemainingTime(latest_route_msg_));
  action_result_ = std::make_shared<route_planning_msgs::action::PlanRoute::Result>();
  action_result_->distance_traveled = 0.0;
  action_result_->time_traveled = rclcpp::Duration::from_seconds(0.0);
  action_result_->destination_reached = false;

  // start publishing route
  is_publishing_route_ = true;

  // execute action in a separate thread to avoid blocking
  std::thread{std::bind(&Lanelet2RoutePlanning::actionExecute, this, std::placeholders::_1), goal_handle}.detach();
}

void Lanelet2RoutePlanning::actionExecute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_msgs::action::PlanRoute>> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing action goal");

  rclcpp::Rate feedback_rate(action_feedback_frequency_);
  bool has_reached_destination = false;
  while (goal_handle->is_executing() && !goal_handle->is_canceling() && !has_reached_destination) {
    // update feedback and result
    action_feedback_->distance_traveled = distanceTraveled(latest_route_msg_);
    action_feedback_->distance_remaining = distanceRemaining(latest_route_msg_);
    action_feedback_->time_traveled = this->now() - action_start_time_;
    action_feedback_->time_remaining = rclcpp::Duration::from_seconds(estimateRemainingTime(latest_route_msg_));
    action_result_->distance_traveled = action_feedback_->distance_traveled;
    action_result_->time_traveled = action_feedback_->time_traveled;

    // check if destination reached (criteria: close to destination and some distance traveled)
    double distance_to_destination = (toEigen2d(egoPosition(latest_ego_data_)) - toEigen2d(destination_)).norm();
    double total_route_distance = action_feedback_->distance_traveled + action_feedback_->distance_remaining;
    bool is_close_to_destination = (distance_to_destination <= destination_distance_threshold_);
    bool has_traveled_sufficient_distance =
        (action_feedback_->distance_traveled >= required_traveled_distance_proportion_ * total_route_distance);
    has_reached_destination = (is_close_to_destination && has_traveled_sufficient_distance);

    // publish feedback
    goal_handle->publish_feedback(action_feedback_);
    if (!has_reached_destination) {
      feedback_rate.sleep();
    }
  }

  // prepare result
  action_result_->distance_traveled = action_feedback_->distance_traveled + action_feedback_->distance_remaining;
  action_result_->time_traveled = this->now() - action_start_time_;
  action_result_->destination_reached = has_reached_destination;

  // publish result
  if (goal_handle->is_canceling()) {
    goal_handle->canceled(action_result_);
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
  } else if (!goal_handle->is_executing()) {
    RCLCPP_INFO(this->get_logger(), "Goal aborted");
  } else if (rclcpp::ok()) {
    is_publishing_route_ = false;  // stop publishing route
    goal_handle->succeed(action_result_);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

bool Lanelet2RoutePlanning::planRoute(const geometry_msgs::msg::PointStamped& destination,
                                      const std::vector<geometry_msgs::msg::PointStamped>& intermediate_destinations) {
  if (!this->checkMap(false)) {
    RCLCPP_ERROR(this->get_logger(), "Cannot plan route, map not loaded by '%s'", ll2_map_server_name_.c_str());
    return false;
  }

  // transform destination to map frame
  geometry_msgs::msg::PointStamped destination_map_stamped;
  if (destination.header.frame_id != ll2_interface_->map_frame_id_) {
    try {
      destination_map_stamped =
          tf_buffer_->transform(destination, ll2_interface_->map_frame_id_, tf2::durationFromSec(transform_timeout_));
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform destination from frame '%s' to frame '%s': %s",
                   destination.header.frame_id.c_str(), ll2_interface_->map_frame_id_.c_str(), ex.what());
      return false;
    }
  } else {
    destination_map_stamped = destination;
  }
  geometry_msgs::msg::Point& destination_map = destination_map_stamped.point;

  // transform intermediate destinations to map frame
  std::vector<geometry_msgs::msg::Point> intermediate_destinations_map;
  for (const auto& intermediate : intermediate_destinations) {
    geometry_msgs::msg::PointStamped intermediate_map_stamped;
    if (intermediate.header.frame_id != ll2_interface_->map_frame_id_) {
      try {
        intermediate_map_stamped =
            tf_buffer_->transform(intermediate, ll2_interface_->map_frame_id_, tf2::durationFromSec(transform_timeout_));
      } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform intermediate from frame '%s' to frame '%s': %s",
                     intermediate.header.frame_id.c_str(), ll2_interface_->map_frame_id_.c_str(), ex.what());
        return false;
      }
      intermediate_destinations_map.push_back(intermediate_map_stamped.point);
    } else {
      intermediate_destinations_map.push_back(intermediate.point);
    }
  }

  // get map and traffic rules
  lanelet::LaneletMapConstPtr map = ll2_interface_->getMapPtr();
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules = getTrafficRules();

  // check validity of ego data
  const double timeout_ego_data = 1.0;
  if ((this->now() - latest_ego_data_.header.stamp).seconds() > timeout_ego_data) {
    RCLCPP_WARN(this->get_logger(), "Ego data is outdated by %.3fs > %.3fs",
                (this->now() - latest_ego_data_.header.stamp).seconds(), timeout_ego_data);
  }
  if (latest_ego_data_.header.frame_id != ll2_interface_->map_frame_id_) {
    RCLCPP_ERROR(this->get_logger(), "Ego data frame '%s' does not match map frame '%s'",
                 latest_ego_data_.header.frame_id.c_str(), ll2_interface_->map_frame_id_.c_str());
    return false;
  }

  // project ego position to lanelet
  lanelet::ConstLanelet ego_ll;
  if (auto result = laneletAtPoint(toEigen2d(egoPosition(latest_ego_data_)), map)) {
    ego_ll = *result;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to find lanelet at ego position");
    return false;
  }
  Eigen::Vector2d ego_ll_position =
      projectPointToLineString(toEigen2d(egoPosition(latest_ego_data_)), toEigen(ego_ll.centerline2d().basicLineString()));

  // project destination to lanelet
  lanelet::ConstLanelet destination_ll;
  if (auto result = laneletAtPoint(toEigen2d(destination_map), map)) {
    destination_ll = *result;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to find lanelet at destination");
    return false;
  }
  Eigen::Vector2d destination_ll_position =
      projectPointToLineString(toEigen2d(destination_map), toEigen(destination_ll.centerline2d().basicLineString()));

  // project intermediate destinations to lanelets
  std::vector<lanelet::ConstLanelet> intermediate_destination_lls;
  std::vector<geometry_msgs::msg::Point> intermediate_destinations_on_route;
  for (const auto& intermediate : intermediate_destinations_map) {
    lanelet::ConstLanelet intermediate_ll;
    if (auto result = laneletAtPoint(toEigen2d(intermediate), map)) {
      intermediate_destination_lls.push_back(*result);
      intermediate_destinations_on_route.push_back(intermediate);
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to find lanelet at intermediate point (%.3f, %.3f). Skipping...", intermediate.x,
                  intermediate.y);
      continue;  // skip this intermediate if no lanelet found
    }
  }

  // undershoot/overshoot route endpoints to enable context before start position and behind destination
  lanelet::ConstLanelet undershot_ego_ll =
      followLaneletsAlongRoutingGraph(routing_graph_, ego_ll, ego_ll_position, -std::abs(route_undershoot_distance_));
  lanelet::ConstLanelet overshot_destination_ll =
      followLaneletsAlongRoutingGraph(routing_graph_, destination_ll, destination_ll_position, route_overshoot_distance_);

  // compute route from start to destination along intermediate destinations
  std::vector<lanelet::ConstLanelet> route_lanelets = {undershot_ego_ll};
  route_lanelets.insert(route_lanelets.end(), intermediate_destination_lls.begin(), intermediate_destination_lls.end());
  route_lanelets.push_back(overshot_destination_ll);
  std::optional<lanelet::routing::Route> planned_route = getRoute(routing_graph_, route_lanelets);

  if (planned_route) {
    starting_point_ = egoPosition(latest_ego_data_);
    destination_ = destination_map;
    intermediate_destinations_ = intermediate_destinations_on_route;
    latest_route_ = std::move(*planned_route);
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan route from lanelet %ld to lanelet %ld", ego_ll.id(), destination_ll.id());
    return false;
  }
}

void Lanelet2RoutePlanning::buildGlobalRouteMessage() {
  // create Route message
  route_planning_msgs::msg::Route route_msg;
  route_msg.header.stamp = latest_ego_data_.header.stamp;
  route_msg.header.frame_id = ll2_interface_->map_frame_id_;
  route_msg.destination = destination_;
  route_msg.intermediate_destinations = intermediate_destinations_;
  route_msg.route_elements = {};

  // get shortest path
  lanelet::routing::LaneletPath shortest_path = latest_route_.shortestPath();

  // resample centerlines along shortest path to accumulate global reference line
  auto resampling_result = resampleCenterlinesAlongPath(shortest_path, sampling_distance_, true);
  std::vector<Eigen::Vector2d> shortest_path_centerline = resampling_result.centerline;
  latest_lanelet_idx_by_reference_line_point_idx_ = resampling_result.lanelet_idx_by_point;

  // fill route message with global reference line
  double accumulated_distance = 0;
  for (size_t c = 0; c < shortest_path_centerline.size(); ++c) {
    // get current, previous and next centerline point
    const Eigen::Vector2d& point = shortest_path_centerline[c];
    const Eigen::Vector2d& prev_point = (c > 0) ? shortest_path_centerline[c - 1] : point;
    const Eigen::Vector2d& next_point = (c < shortest_path_centerline.size() - 1) ? shortest_path_centerline[c + 1] : point;

    // get lanelet corresponding to centerline point
    const lanelet::ConstLanelet& lanelet = shortest_path[latest_lanelet_idx_by_reference_line_point_idx_[c]];

    // identify lane changes based on break in equidistant centerline
    bool changes_lane_from_prev_point = changesLaneFromPointToPoint(prev_point, point, sampling_distance_);
    bool changes_lane_to_next_point = changesLaneFromPointToPoint(point, next_point, sampling_distance_);

    // compute orientation of centerline point
    Eigen::Vector2d prev_point_for_orientation = changes_lane_from_prev_point ? point : prev_point;
    Eigen::Vector2d next_point_for_orientation = changes_lane_to_next_point ? point : next_point;
    Eigen::Vector2d orientation = tangentOfPointAlongLineString(point, prev_point_for_orientation, next_point_for_orientation);

    // determine speed limit
    uint8_t speed_limit = speedLimit(lanelet);

    // accumulate distance
    accumulated_distance += (point - prev_point).norm();

    // create RouteElement
    route_planning_msgs::msg::RouteElement route_element_msg = createMinimalRouteElement(
        toRos(point), toRosQuaternion(orientation), accumulated_distance, changes_lane_to_next_point, speed_limit);
    route_msg.route_elements.push_back(route_element_msg);
  }

  // project starting point and destinations to reference line, if enabled
  if (project_destination_to_reference_line_) {
    starting_point_ = toRos(projectPointToLineString(toEigen2d(starting_point_), shortest_path_centerline));
    for (auto& destination : intermediate_destinations_) {
      destination = toRos(projectPointToLineString(toEigen2d(destination), shortest_path_centerline));
    }
    destination_ = toRos(projectPointToLineString(toEigen2d(destination_), shortest_path_centerline));
    route_msg.destination = destination_;
    route_msg.intermediate_destinations = intermediate_destinations_;
  }

  // determine starting/current/destination indices in route elements
  route_msg.starting_route_element_idx =
      matchPointToLineString(shortest_path_centerline, toEigen2d(starting_point_), 0, true, true);
  route_msg.current_route_element_idx = matchPointToLineString(shortest_path_centerline, toEigen2d(egoPosition(latest_ego_data_)),
                                                               route_msg.starting_route_element_idx, true, true);
  route_msg.destination_route_element_idx =
      indexOfLineStringPointClosestToPoint(shortest_path_centerline, toEigen2d(destination_), true, false);

  has_enriched_route_ = false;
  latest_route_msg_ = route_msg;
}

void Lanelet2RoutePlanning::buildEnrichedRouteMessage() {
  route_planning_msgs::msg::Route route_msg = latest_route_msg_;
  std::vector<route_planning_msgs::msg::RouteElement>& route_elements = route_msg.route_elements;
  if (latest_suggested_turn_signal_distance_ahead_by_route_element_by_lane_element_.size() != route_elements.size()) {
    latest_suggested_turn_signal_distance_ahead_by_route_element_by_lane_element_ =
        std::vector<std::vector<int>>(route_elements.size());
  }

  // find point of global reference line closest to and behind of ego position
  const Eigen::Vector2d ego_position = toEigen2d(egoPosition(latest_ego_data_));
  const std::vector<Eigen::Vector2d> reference_line = to2d(suggestedReferenceLineToEigen(route_elements));
  size_t c_closest_point = matchPointToLineString(reference_line, ego_position, route_msg.current_route_element_idx, true, true);

// loop over global reference line in parallel
#pragma omp parallel for
  for (size_t c = 0; c < route_elements.size(); ++c) {
    route_planning_msgs::msg::RouteElement& route_element_msg = route_elements[c];
    route_planning_msgs::msg::LaneElement& lane_element_msg =
        route_element_msg.lane_elements[route_element_msg.suggested_lane_idx];

    // only consider route elements within enrich_route_behind_ego_distance_ and enrich_route_ahead_ego_distance_ for local route
    double distance_ahead = route_element_msg.s - route_elements[c_closest_point].s;
    if (distance_ahead > enrich_route_ahead_ego_distance_ || distance_ahead < -enrich_route_behind_ego_distance_) {
      // clear local route enriched information, if existing
      route_element_msg = createMinimalRouteElement(lane_element_msg.reference_pose.position,
                                                    lane_element_msg.reference_pose.orientation, route_element_msg.s,
                                                    route_element_msg.will_change_suggested_lane, lane_element_msg.speed_limit);
      latest_suggested_turn_signal_distance_ahead_by_route_element_by_lane_element_[c].clear();
      continue;
    }

    // skip recomputation of local route if already enriched
    if (route_element_msg.is_enriched) {
      continue;
    }

    // get current, previous and next centerline point
    route_planning_msgs::msg::LaneElement prev_lane_element_msg, next_lane_element_msg;
#pragma omp critical  // prevent race condition when accessing prev/next suggested lane element set by other threads
    {
      prev_lane_element_msg =
          (c > 0) ? route_planning_msgs::route_access::getSuggestedLaneElement(route_elements[c - 1]) : lane_element_msg;
      next_lane_element_msg = (c < route_elements.size() - 1)
                                  ? route_planning_msgs::route_access::getSuggestedLaneElement(route_elements[c + 1])
                                  : lane_element_msg;
    }
    const Eigen::Vector2d point = toEigen2d(lane_element_msg.reference_pose.position);
    const Eigen::Vector2d prev_point = toEigen2d(prev_lane_element_msg.reference_pose.position);
    const Eigen::Vector2d next_point = toEigen2d(next_lane_element_msg.reference_pose.position);

    // get lanelet corresponding to centerline point
    lanelet::routing::LaneletPath shortest_path = latest_route_.shortestPath();
    const lanelet::ConstLanelet& lanelet = shortest_path[latest_lanelet_idx_by_reference_line_point_idx_[c]];

    // identify lane changes
    bool changes_lane_from_prev_point = changesLaneFromPointToPoint(prev_point, point, sampling_distance_);
    bool changes_lane_to_next_point = changesLaneFromPointToPoint(point, next_point, sampling_distance_);

    // determine neighboring points for projection
    Eigen::Vector2d prev_point_for_projection = changes_lane_from_prev_point ? point : prev_point;
    Eigen::Vector2d next_point_for_projection = changes_lane_to_next_point ? point : next_point;

    // get adjacent lanelets
    std::vector<lanelet::ConstLanelet> adjacent_left_lanelets = adjacentLeftOrRightLanelets(lanelet, routing_graph_, true);
    std::vector<lanelet::ConstLanelet> adjacent_right_lanelets = adjacentLeftOrRightLanelets(lanelet, routing_graph_, false);
    int suggested_lane_idx = adjacent_left_lanelets.size();
    int n_lanes = adjacent_left_lanelets.size() + 1 + adjacent_right_lanelets.size();
    if (latest_suggested_turn_signal_distance_ahead_by_route_element_by_lane_element_[c].empty()) {
      latest_suggested_turn_signal_distance_ahead_by_route_element_by_lane_element_[c] = std::vector<int>(n_lanes, -1);
    }

    // project centerline point to lanelet and adjacent lanelet centerlines and bounds
    auto lanelet_projected_points =
        projectPointToLaneletLines(point, prev_point_for_projection, next_point_for_projection,
                                   std::vector<lanelet::ConstLanelet>{lanelet}, this->get_logger())[0];
    auto adjacent_left_lanelets_projected_points = projectPointToLaneletLines(
        point, prev_point_for_projection, next_point_for_projection, adjacent_left_lanelets, this->get_logger());
    auto adjacent_right_lanelets_projected_points = projectPointToLaneletLines(
        point, prev_point_for_projection, next_point_for_projection, adjacent_right_lanelets, this->get_logger());

    // compute offset of lane element indices from current to next route element
    const lanelet::ConstLanelet& lanelet_of_next_point =
        (c < route_elements.size() - 1) ? shortest_path[latest_lanelet_idx_by_reference_line_point_idx_[c + 1]] : lanelet;
    int following_lane_idx_offset;
    if (auto result = computeFollowingLaneIdxOffset(lanelet, lanelet_of_next_point, routing_graph_)) {
      following_lane_idx_offset = *result;
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to find following lane index offset for route element %ld on lanelet %ld, assuming no offset", c,
                   lanelet.id());
      following_lane_idx_offset = 0;
    }

    // extract drivable space
    Eigen::Vector2d drivable_space_left, drivable_space_right;
    std::tie(drivable_space_left, drivable_space_right) =
        extractDrivableSpace(ll2_interface_->getMapPtr()->lineStringLayer,
                             {prev_point_for_projection, point, next_point_for_projection}, max_drivable_space_radius_);

    // extract regulatory elements
    auto regulatory_element_extraction =
        extractRegulatoryElements(lanelet, adjacent_left_lanelets, adjacent_right_lanelets, {prev_point, point, next_point});

// enrich RouteElement with local route information
#pragma omp critical  // prevent race condition when accessing prev/next suggested lane element set by other threads
    {
      route_element_msg.lane_elements = {};
      route_element_msg.is_enriched = true;
      route_element_msg.left_boundary = toRos(drivable_space_left);
      route_element_msg.right_boundary = toRos(drivable_space_right);
      route_element_msg.regulatory_elements = regulatory_element_extraction.regulatory_element_msgs;
      route_element_msg.suggested_lane_idx = suggested_lane_idx;
      route_element_msg.will_change_suggested_lane = changes_lane_to_next_point;
      // route_element_msg.s already set in global route
      size_t lane_element_idx = route_element_msg.lane_elements.size();

      // create LaneElements for left adjacent lanes
      for (size_t a = 0; a < adjacent_left_lanelets_projected_points.size(); ++a) {
        route_planning_msgs::msg::LaneElement lane_element_msg;
        lane_element_msg.reference_pose.position = toRos(adjacent_left_lanelets_projected_points[a].centerline_point);
        // lane_element_msg.reference_pose.orientation computed in postprocessRouteMessage
        lane_element_msg.left_boundary.point = toRos(adjacent_left_lanelets_projected_points[a].left_bound_point);
        lane_element_msg.left_boundary.type = laneBoundaryType(adjacent_left_lanelets[a].leftBound2d());
        lane_element_msg.right_boundary.point = toRos(adjacent_left_lanelets_projected_points[a].right_bound_point);
        lane_element_msg.right_boundary.type = laneBoundaryType(adjacent_left_lanelets[a].rightBound2d());
        lane_element_msg.speed_limit = speedLimit(adjacent_left_lanelets[a]);
        lane_element_msg.regulatory_element_idcs = regulatory_element_extraction.adjacent_left_regulatory_element_idcs[a];
        int computed_following_lane_idx = route_element_msg.lane_elements.size() + following_lane_idx_offset;
        lane_element_msg.has_following_lane_idx = (computed_following_lane_idx >= 0 && computed_following_lane_idx < n_lanes);
        if (lane_element_msg.has_following_lane_idx) {
          lane_element_msg.following_lane_idx = computed_following_lane_idx;
        }
        std::tie(lane_element_msg.suggested_turn_signal,
                 latest_suggested_turn_signal_distance_ahead_by_route_element_by_lane_element_[c][lane_element_idx]) =
            suggestedTurnSignal(adjacent_left_lanelets[a], this->get_logger());
        route_element_msg.lane_elements.push_back(lane_element_msg);
        lane_element_idx = route_element_msg.lane_elements.size();
      }

      // create LaneElement for centerline lane
      route_planning_msgs::msg::LaneElement centerline_lane_element_msg;
      centerline_lane_element_msg.reference_pose.position = toRos(point);
      // centerline_lane_element_msg.reference_pose.orientation computed in postprocessRouteMessage
      centerline_lane_element_msg.left_boundary.point = toRos(lanelet_projected_points.left_bound_point);
      centerline_lane_element_msg.left_boundary.type = laneBoundaryType(lanelet.leftBound2d());
      centerline_lane_element_msg.right_boundary.point = toRos(lanelet_projected_points.right_bound_point);
      centerline_lane_element_msg.right_boundary.type = laneBoundaryType(lanelet.rightBound2d());
      centerline_lane_element_msg.speed_limit = speedLimit(lanelet);
      centerline_lane_element_msg.regulatory_element_idcs = regulatory_element_extraction.regulatory_element_idcs;
      int computed_following_lane_idx = route_element_msg.lane_elements.size() + following_lane_idx_offset;
      centerline_lane_element_msg.has_following_lane_idx =
          (computed_following_lane_idx >= 0 && computed_following_lane_idx < n_lanes);
      if (centerline_lane_element_msg.has_following_lane_idx) {
        centerline_lane_element_msg.following_lane_idx = computed_following_lane_idx;
      }
      std::tie(centerline_lane_element_msg.suggested_turn_signal,
               latest_suggested_turn_signal_distance_ahead_by_route_element_by_lane_element_[c][lane_element_idx]) =
          suggestedTurnSignal(lanelet, this->get_logger());
      route_element_msg.lane_elements.push_back(centerline_lane_element_msg);
      lane_element_idx = route_element_msg.lane_elements.size();

      // create LaneElements for right adjacent lanes
      for (size_t a = 0; a < adjacent_right_lanelets_projected_points.size(); ++a) {
        route_planning_msgs::msg::LaneElement lane_element_msg;
        lane_element_msg.reference_pose.position = toRos(adjacent_right_lanelets_projected_points[a].centerline_point);
        // lane_element_msg.reference_pose.orientation computed in postprocessRouteMessage
        lane_element_msg.left_boundary.point = toRos(adjacent_right_lanelets_projected_points[a].left_bound_point);
        lane_element_msg.left_boundary.type = laneBoundaryType(adjacent_right_lanelets[a].leftBound2d());
        lane_element_msg.right_boundary.point = toRos(adjacent_right_lanelets_projected_points[a].right_bound_point);
        lane_element_msg.right_boundary.type = laneBoundaryType(adjacent_right_lanelets[a].rightBound2d());
        lane_element_msg.speed_limit = speedLimit(adjacent_right_lanelets[a]);
        lane_element_msg.regulatory_element_idcs = regulatory_element_extraction.adjacent_right_regulatory_element_idcs[a];
        int computed_following_lane_idx = route_element_msg.lane_elements.size() + following_lane_idx_offset;
        lane_element_msg.has_following_lane_idx = (computed_following_lane_idx >= 0 && computed_following_lane_idx < n_lanes);
        if (lane_element_msg.has_following_lane_idx) {
          lane_element_msg.following_lane_idx = computed_following_lane_idx;
        }
        std::tie(lane_element_msg.suggested_turn_signal,
                 latest_suggested_turn_signal_distance_ahead_by_route_element_by_lane_element_[c][lane_element_idx]) =
            suggestedTurnSignal(adjacent_right_lanelets[a], this->get_logger());
        route_element_msg.lane_elements.push_back(lane_element_msg);
        lane_element_idx = route_element_msg.lane_elements.size();
      }
    }
  }

  // split in traveled and remaining route elements
  route_msg.current_route_element_idx = c_closest_point;
  route_msg.header.stamp = latest_ego_data_.header.stamp;

  // postprocess route message
  postprocessRouteMessage(route_msg, latest_suggested_turn_signal_distance_ahead_by_route_element_by_lane_element_);

  // save as latest route message
  latest_route_msg_ = route_msg;
  has_enriched_route_ = true;
}

}  // namespace lanelet2_route_planning

/**
 * @brief Starts the ROS node.
 *
 * @param[in] argc number of command-line arguments
 * @param[in] argv command-line arguments
 * @return process exit code
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lanelet2_route_planning::Lanelet2RoutePlanning>());
  rclcpp::shutdown();

  return 0;
}

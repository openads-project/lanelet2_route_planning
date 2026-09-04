// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <chrono>
#include <functional>
#include <iterator>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>

#include <lanelet2_core/geometry/LaneletMap.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "plan_route_action_client/plan_route_action_client.hpp"

namespace plan_route_action_client {

/**
 * @brief Parses WGS84 waypoints from "<LATITUDE>,<LONGITUDE>[,<WAIT_TIME_S>]" strings.
 *
 * @param[in] waypoints_param waypoint parameter values
 * @param[in,out] waypoint_wait_times wait time for each waypoint
 * @param[in] logger ROS logger
 * @return parsed latitude/longitude pairs, or `std::nullopt` if parsing fails
 */
std::optional<std::vector<std::pair<double, double>>> parseWaypoints(const std::vector<std::string>& waypoints_param,
                                                                     std::vector<double>& waypoint_wait_times,
                                                                     const rclcpp::Logger& logger) {
  std::vector<std::pair<double, double>> waypoints;
  std::vector<double> parsed_wait_times;
  for (const auto& waypoint : waypoints_param) {
    size_t comma_pos = waypoint.find(',');
    if (comma_pos != std::string::npos) {
      try {
        size_t wait_time_comma_pos = waypoint.find(',', comma_pos + 1);
        if (wait_time_comma_pos != std::string::npos && waypoint.find(',', wait_time_comma_pos + 1) != std::string::npos) {
          return std::nullopt;
        }
        double lat = std::stod(waypoint.substr(0, comma_pos));
        double lon = std::stod(waypoint.substr(comma_pos + 1, wait_time_comma_pos - comma_pos - 1));
        double wait_time_s = 0.0;
        if (wait_time_comma_pos != std::string::npos) {
          wait_time_s = std::stod(waypoint.substr(wait_time_comma_pos + 1));
        }
        waypoints.emplace_back(lat, lon);
        parsed_wait_times.push_back(wait_time_s);
      } catch (const std::invalid_argument& e) {
        return std::nullopt;
      } catch (const std::out_of_range& e) {
        return std::nullopt;
      }
    } else {
      return std::nullopt;
    }
  }

  if (!parsed_wait_times.empty() && parsed_wait_times.back() < 0.0) {
    RCLCPP_WARN(logger, "Last waypoint cannot be intermediate, treating it as stop with wait_time_s 0.0");
    parsed_wait_times.back() = 0.0;
  }

  waypoint_wait_times = parsed_wait_times;
  return waypoints;
}

PlanRouteActionClient::PlanRouteActionClient() : Node("plan_route_action_client") {
  this->declareAndLoadParameter("ll2_map_server_name", ll2_map_server_name_, "Name of lanelet2_map_server node", false);
  this->declareAndLoadParameter(
      "waypoints", waypoints_param_,
      "List of WGS84 waypoints to follow (list of strings with comma-separated '<LATITUDE>,<LONGITUDE>[,<WAIT_TIME_S>]', missing "
      "wait time defaults to 0s, negative wait time means intermediate destination)",
      true);
  this->declareAndLoadParameter("enable_random_destination", enable_random_destination_,
                                "Whether to plan a route to a random destination", true);
  this->declareAndLoadParameter("enable_continuous_planning", enable_continuous_planning_,
                                "Whether to continuously plan a new route (either looping waypoints or to a random destination)",
                                true);
  this->declareAndLoadParameter("continuous_planning_replanning_proportion", continuous_planning_replanning_proportion_,
                                "Traveled route proportion at which continuous "
                                "waypoint replanning starts",
                                true, false, false, 0.0, 1.0, 0.01);
  this->declareAndLoadParameter("cancel_route", cancel_route_, "Cancel active route planning action (to be set at runtime)",
                                true);
  this->setup();
}

template <typename T>
void PlanRouteActionClient::declareAndLoadParameter(const std::string& name,
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
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1);
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value())).set__step(step);
      param_desc.integer_range = {range};
    } else if constexpr (std::is_floating_point_v<T>) {
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
    std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) { param = p.get_value<T>(); };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}

rcl_interfaces::msg::SetParametersResult PlanRouteActionClient::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s'", param.get_name().c_str());
        break;
      }
    }

    // handle waypoints
    if (param.get_name() == "waypoints") {
      auto parsed_waypoints = parseWaypoints(waypoints_param_, waypoint_wait_times_, this->get_logger());
      if (parsed_waypoints) {
        waypoints_ = *parsed_waypoints;
      } else {
        std::stringstream ss;
        ss << "Failed to parse parameter 'waypoints': [";
        for (const auto& waypoint : waypoints_param_) {
          ss << waypoint << (&waypoint != &waypoints_param_.back() ? ", " : "]");
        }
        RCLCPP_ERROR(this->get_logger(), "%s", ss.str().c_str());
      }
    }

    // handle cancel_route
    if (param.get_name() == "cancel_route") {
      if (cancel_route_) {
        if (action_client_->wait_for_action_server(std::chrono::duration<double>(0.1))) {
          RCLCPP_INFO(this->get_logger(), "Cancelling route");
          action_client_->async_cancel_all_goals();
        } else {
          RCLCPP_WARN(this->get_logger(), "Action server not available, cannot cancel route");
        }
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}

void PlanRouteActionClient::setup() {
  // callback for dynamic parameter configuration
  parameters_callback_ =
      this->add_on_set_parameters_callback(std::bind(&PlanRouteActionClient::parametersCallback, this, std::placeholders::_1));

  // subscriber for goal pose
  goal_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/goal_pose", 10, std::bind(&PlanRouteActionClient::goalPoseCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", goal_pose_subscriber_->get_topic_name());
  global_route_subscriber_ = this->create_subscription<route_planning_msgs::msg::Route>(
      "~/global_route", 1, std::bind(&PlanRouteActionClient::globalRouteCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", global_route_subscriber_->get_topic_name());

  // action client
  action_client_ = rclcpp_action::create_client<PlanRoute>(this, "/planning/lanelet2_route_planning/plan_route");

  // ll2 map interface
  ll2_interface_ = std::make_unique<Lanelet2MapInterface>(*this, ll2_map_server_name_);

  // parse waypoints
  auto parsed_waypoints = parseWaypoints(waypoints_param_, waypoint_wait_times_, this->get_logger());
  if (parsed_waypoints) {
    waypoints_ = *parsed_waypoints;
  } else {
    std::stringstream ss;
    ss << "Failed to parse parameter 'waypoints': [";
    for (const auto& waypoint : waypoints_param_) {
      ss << waypoint << (&waypoint != &waypoints_param_.back() ? ", " : "]");
    }
    RCLCPP_ERROR(this->get_logger(), "%s", ss.str().c_str());
  }

  // set up auto-planning timer
  auto_planning_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                                 std::bind(&PlanRouteActionClient::autoPlanningTimerCallback, this));
}

void PlanRouteActionClient::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received goal pose (%.3f, %.3f, %.3f) in frame '%s'", msg->pose.position.x,
              msg->pose.position.y, msg->pose.position.z, msg->header.frame_id.c_str());
  has_active_waypoint_ = false;
  active_waypoint_wait_time_s_ = 0.0;
  active_route_waypoints_.clear();
  active_route_waypoint_indices_.clear();
  auto_planning_resume_time_s_ = 0.0;
  sendGoal(msg);
}

void PlanRouteActionClient::globalRouteCallback(const route_planning_msgs::msg::Route::SharedPtr msg) {
  latest_global_route_ = *msg;
}

void PlanRouteActionClient::autoPlanningTimerCallback() {
  const double now_s = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  if (now_s < auto_planning_resume_time_s_) {
    return;
  }

  if (enable_random_destination_ && (enable_continuous_planning_ || !has_completed_one_goal_)) {
    this->planToRandomDestination();
  } else if (!waypoints_.empty()) {
    if (next_waypoint_idx_ >= waypoints_.size() && enable_continuous_planning_) {
      next_waypoint_idx_ = 0;  // loop waypoints, if continuous planning is enabled
    }
    if (next_waypoint_idx_ < waypoints_.size()) {
      this->planToNextWaypoint();
    }
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Nothing to plan, waiting for waypoints or random destination");
  }
}

void PlanRouteActionClient::planToNextWaypoint() {
  // check if waypoint is valid
  if (next_waypoint_idx_ >= waypoints_.size()) {
    RCLCPP_ERROR(this->get_logger(), "Waypoint index %ld out of bounds (%ld), skipping", next_waypoint_idx_, waypoints_.size());
    return;
  }
  if (waypoint_wait_times_.size() != waypoints_.size()) {
    RCLCPP_ERROR(this->get_logger(), "Waypoint wait times do not match waypoints, skipping");
    return;
  }

  // check if map is loaded
  if (!ll2_interface_->map_loaded_) {
    RCLCPP_ERROR(this->get_logger(), "Map not loaded, cannot generate waypoint");
    return;
  }

  // generate goal pose from waypoint
  auto goal_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
  std::vector<geometry_msgs::msg::PointStamped> intermediate_destinations;
  std::vector<geometry_msgs::msg::PointStamped> route_waypoints;
  std::vector<size_t> route_waypoint_indices;
  auto ll2_projector = ll2_interface_->getProjectorPtr();
  if (!ll2_projector) {
    RCLCPP_ERROR(this->get_logger(), "Failed to generate waypoint goal pose");
    return;
  }

  size_t checked_waypoints = 0;
  while (checked_waypoints < waypoints_.size()) {
    const size_t waypoint_idx = next_waypoint_idx_;
    const auto& waypoint = waypoints_[next_waypoint_idx_];
    const double wait_time_s = waypoint_wait_times_[next_waypoint_idx_];
    lanelet::GPSPoint gps_waypoint;
    gps_waypoint.lat = waypoint.first;
    gps_waypoint.lon = waypoint.second;
    lanelet::BasicPoint3d map_waypoint = ll2_projector->forward(gps_waypoint);

    geometry_msgs::msg::PointStamped waypoint_point;
    waypoint_point.header.frame_id = ll2_interface_->map_frame_id_;
    waypoint_point.header.stamp = this->now();
    waypoint_point.point.x = map_waypoint.x();
    waypoint_point.point.y = map_waypoint.y();
    waypoint_point.point.z = 0.0;

    if (wait_time_s < 0.0) {
      RCLCPP_INFO(this->get_logger(), "Adding intermediate waypoint (%.6f, %.6f)", waypoint.first, waypoint.second);
      intermediate_destinations.push_back(waypoint_point);
      route_waypoints.push_back(waypoint_point);
      route_waypoint_indices.push_back(waypoint_idx);
      next_waypoint_idx_++;
      if (next_waypoint_idx_ >= waypoints_.size()) {
        if (enable_continuous_planning_) {
          next_waypoint_idx_ = 0;
        } else {
          RCLCPP_ERROR(this->get_logger(), "No destination waypoint found after intermediate waypoints");
          return;
        }
      }
      checked_waypoints++;
      continue;
    }

    RCLCPP_INFO(this->get_logger(), "Planning route to next waypoint (%.6f, %.6f)", waypoint.first, waypoint.second);
    goal_pose->pose.position = waypoint_point.point;
    goal_pose->header = waypoint_point.header;
    active_waypoint_wait_time_s_ = wait_time_s;
    route_waypoints.push_back(waypoint_point);
    route_waypoint_indices.push_back(waypoint_idx);
    next_waypoint_idx_++;
    break;
  }

  // send goal
  if (!goal_pose->header.frame_id.empty()) {
    RCLCPP_INFO(this->get_logger(), "Generated waypoint goal pose (%.3f, %.3f, %.3f) in frame '%s'", goal_pose->pose.position.x,
                goal_pose->pose.position.y, goal_pose->pose.position.z, goal_pose->header.frame_id.c_str());
    auto_planning_timer_->cancel();  // cancel auto-planning timer until goal completion
    has_active_waypoint_ = true;
    active_route_waypoints_ = route_waypoints;
    active_route_waypoint_indices_ = route_waypoint_indices;
    this->sendGoal(goal_pose, intermediate_destinations);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to generate waypoint goal pose");
  }
}

void PlanRouteActionClient::planToRandomDestination() {
  RCLCPP_INFO(this->get_logger(), "Planning route to random destination");

  // check if map is loaded
  if (!ll2_interface_->map_loaded_) {
    RCLCPP_ERROR(this->get_logger(), "Map not loaded, cannot generate a random destination");
    return;
  }

  // generate random goal pose by sampling a random lanelet
  auto goal_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
  lanelet::LaneletMapConstPtr map = ll2_interface_->getMapPtr();
  if (!map->laneletLayer.empty()) {
    const auto lanelet_count = static_cast<int>(map->laneletLayer.size());
    const auto random_lanelet_idx =
        static_cast<std::iterator_traits<decltype(map->laneletLayer.begin())>::difference_type>(std::rand() % lanelet_count);
    auto random_lanelet = *std::next(map->laneletLayer.begin(), random_lanelet_idx);
    auto centerline = random_lanelet.centerline();
    if (!centerline.empty()) {
      auto point = centerline.back();
      goal_pose->pose.position.x = point.x();
      goal_pose->pose.position.y = point.y();
      goal_pose->pose.position.z = point.z();
      if (centerline.size() > 1) {
        auto heading =
            std::atan2(point.y() - centerline[centerline.size() - 2].y(), point.x() - centerline[centerline.size() - 2].x());
        tf2::Quaternion q;
        q.setRPY(0, 0, heading);
        goal_pose->pose.orientation = tf2::toMsg(q);
      }
      goal_pose->header.frame_id = ll2_interface_->map_frame_id_;
      goal_pose->header.stamp = this->now();
    }
  }

  // send goal
  if (!goal_pose->header.frame_id.empty()) {
    RCLCPP_INFO(this->get_logger(), "Generated random goal pose (%.3f, %.3f, %.3f) in frame '%s'", goal_pose->pose.position.x,
                goal_pose->pose.position.y, goal_pose->pose.position.z, goal_pose->header.frame_id.c_str());
    auto_planning_timer_->cancel();  // cancel auto-planning timer until goal completion
    has_active_waypoint_ = false;
    active_waypoint_wait_time_s_ = 0.0;
    active_route_waypoints_.clear();
    active_route_waypoint_indices_.clear();
    this->sendGoal(goal_pose);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to generate random goal pose");
  }
}

void PlanRouteActionClient::sendGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                                     const std::vector<geometry_msgs::msg::PointStamped>& intermediate_destinations) {
  RCLCPP_INFO(this->get_logger(), "Requesting to plan route to destination (%.3f, %.3f, %.3f) in frame '%s'",
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->header.frame_id.c_str());

  // check if action server is available
  if (!action_client_->wait_for_action_server(std::chrono::duration<double>(0.1))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available, aborting");
    if (continuous_replanning_pending_) {
      pending_route_waypoints_.clear();
      pending_route_waypoint_indices_.clear();
      continuous_replanning_pending_ = false;
      replaced_goal_id_.reset();
      return;
    }
    has_active_waypoint_ = false;
    active_route_waypoints_.clear();
    active_route_waypoint_indices_.clear();
    active_goal_id_.reset();
    auto_planning_timer_->reset();  // restart auto-planning timer
    return;
  }

  // build goal
  auto goal = PlanRoute::Goal();
  goal.destination = geometry_msgs::msg::PointStamped();
  goal.destination.header = msg->header;
  goal.destination.point = msg->pose.position;
  goal.intermediate_destinations = intermediate_destinations;

  // send goal
  auto send_goal_options = rclcpp_action::Client<PlanRoute>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&PlanRouteActionClient::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
      std::bind(&PlanRouteActionClient::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&PlanRouteActionClient::resultCallback, this, std::placeholders::_1);
  goal_handle_future_ = action_client_->async_send_goal(goal, send_goal_options);
  RCLCPP_INFO(this->get_logger(), "Goal sent");
}

void PlanRouteActionClient::goalResponseCallback(const GoalHandlePlanRoute::SharedPtr& goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal rejected by action server");
    if (continuous_replanning_pending_) {
      pending_route_waypoints_.clear();
      pending_route_waypoint_indices_.clear();
      continuous_replanning_pending_ = false;
      replaced_goal_id_.reset();
      return;
    }
    has_active_waypoint_ = false;
    active_route_waypoints_.clear();
    active_route_waypoint_indices_.clear();
    active_goal_id_.reset();
    auto_planning_timer_->reset();  // restart auto-planning timer
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by action server");
    active_goal_id_ = goal_handle->get_goal_id();
    if (continuous_replanning_pending_) {
      active_route_waypoints_ = std::move(pending_route_waypoints_);
      active_route_waypoint_indices_ = std::move(pending_route_waypoint_indices_);
      next_waypoint_idx_ = pending_next_waypoint_idx_;
      active_waypoint_wait_time_s_ = pending_waypoint_wait_time_s_;
      continuous_replanning_pending_ = false;
    }
  }
}

void PlanRouteActionClient::feedbackCallback(GoalHandlePlanRoute::SharedPtr goal_handle,
                                             const std::shared_ptr<const PlanRoute::Feedback> feedback) {
  const double distance_traveled = feedback->distance_traveled;
  const double distance_total = feedback->distance_remaining + feedback->distance_traveled;
  rclcpp::Duration time_traveled(feedback->time_traveled.sec, feedback->time_traveled.nanosec);
  rclcpp::Duration time_remaining(feedback->time_remaining.sec, feedback->time_remaining.nanosec);
  rclcpp::Duration time_total = time_traveled + time_remaining;
  RCLCPP_INFO(this->get_logger(), "Route progress: %.2f / %.2f m, %.1f / %.1f s", distance_traveled, distance_total,
              time_traveled.seconds(), time_total.seconds());

  if (!active_goal_id_.has_value() || goal_handle->get_goal_id() != active_goal_id_.value()) {
    return;
  }

  if (!enable_continuous_planning_ || !has_active_waypoint_ || active_waypoint_wait_time_s_ > 0.0 ||
      continuous_replanning_pending_ || distance_total <= 0.0 ||
      distance_traveled / distance_total < continuous_planning_replanning_proportion_ || waypoints_.empty() ||
      waypoint_wait_times_.size() != waypoints_.size() ||
      active_route_waypoints_.size() != active_route_waypoint_indices_.size() || latest_global_route_.route_elements.empty() ||
      latest_global_route_.intermediate_destinations.size() + 1 != active_route_waypoints_.size() ||
      latest_global_route_.current_route_element_idx >= latest_global_route_.route_elements.size()) {
    return;
  }

  // Match the ordered active waypoints to monotonically increasing route
  // element indices.
  std::vector<size_t> waypoint_route_indices;
  size_t search_start = 0;
  for (const auto& waypoint : active_route_waypoints_) {
    double min_squared_distance = std::numeric_limits<double>::max();
    size_t closest_idx = search_start;
    for (size_t idx = search_start; idx < latest_global_route_.route_elements.size(); ++idx) {
      const auto& route_element = latest_global_route_.route_elements[idx];
      if (route_element.lane_elements.empty()) {
        continue;
      }
      const size_t lane_idx =
          route_element.suggested_lane_idx < route_element.lane_elements.size() ? route_element.suggested_lane_idx : 0;
      const auto& position = route_element.lane_elements[lane_idx].reference_pose.position;
      const double dx = position.x - waypoint.point.x;
      const double dy = position.y - waypoint.point.y;
      const double squared_distance = dx * dx + dy * dy;
      if (squared_distance < min_squared_distance) {
        min_squared_distance = squared_distance;
        closest_idx = idx;
      }
    }
    if (min_squared_distance == std::numeric_limits<double>::max()) {
      return;
    }
    waypoint_route_indices.push_back(closest_idx);
    search_start = closest_idx;
  }

  const auto first_unpassed = std::find_if(waypoint_route_indices.begin(), waypoint_route_indices.end(),
                                           [this](size_t idx) { return idx >= latest_global_route_.current_route_element_idx; });
  const size_t first_unpassed_idx = static_cast<size_t>(std::distance(waypoint_route_indices.begin(), first_unpassed));
  if (first_unpassed == waypoint_route_indices.end()) {
    return;
  }

  auto ll2_projector = ll2_interface_->getProjectorPtr();
  if (!ll2_projector) {
    return;
  }

  std::vector<geometry_msgs::msg::PointStamped> rotated_waypoints;
  std::vector<size_t> rotated_waypoint_indices;
  const size_t first_waypoint_idx = active_route_waypoint_indices_[first_unpassed_idx];
  if (first_waypoint_idx >= waypoints_.size()) {
    return;
  }
  for (size_t offset = 0; offset < waypoints_.size(); ++offset) {
    const size_t waypoint_idx = (first_waypoint_idx + offset) % waypoints_.size();
    lanelet::GPSPoint gps_waypoint;
    gps_waypoint.lat = waypoints_[waypoint_idx].first;
    gps_waypoint.lon = waypoints_[waypoint_idx].second;
    const lanelet::BasicPoint3d map_waypoint = ll2_projector->forward(gps_waypoint);

    geometry_msgs::msg::PointStamped waypoint_point;
    waypoint_point.header.frame_id = ll2_interface_->map_frame_id_;
    waypoint_point.header.stamp = this->now();
    waypoint_point.point.x = map_waypoint.x();
    waypoint_point.point.y = map_waypoint.y();
    waypoint_point.point.z = 0.0;
    rotated_waypoints.push_back(waypoint_point);
    rotated_waypoint_indices.push_back(waypoint_idx);

    if (waypoint_wait_times_[waypoint_idx] > 0.0) {
      break;
    }
  }

  auto goal_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
  goal_pose->header = rotated_waypoints.back().header;
  goal_pose->pose.position = rotated_waypoints.back().point;
  std::vector<geometry_msgs::msg::PointStamped> intermediate_destinations(rotated_waypoints.begin(), rotated_waypoints.end() - 1);

  RCLCPP_INFO(this->get_logger(),
              "Route progress reached %.0f%%, rotating waypoint route after "
              "%ld passed waypoint(s)",
              100.0 * continuous_planning_replanning_proportion_, first_unpassed_idx);
  pending_route_waypoints_ = rotated_waypoints;
  pending_route_waypoint_indices_ = rotated_waypoint_indices;
  pending_next_waypoint_idx_ = rotated_waypoint_indices.back() + 1;
  pending_waypoint_wait_time_s_ = std::max(0.0, waypoint_wait_times_[rotated_waypoint_indices.back()]);
  continuous_replanning_pending_ = true;
  replaced_goal_id_ = goal_handle->get_goal_id();
  this->sendGoal(goal_pose, intermediate_destinations);
}

void PlanRouteActionClient::resultCallback(const GoalHandlePlanRoute::WrappedResult& result) {
  if (replaced_goal_id_.has_value() && result.goal_id == replaced_goal_id_.value() &&
      result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_INFO(this->get_logger(), "Previous waypoint goal aborted after continuous replanning");
    replaced_goal_id_.reset();
    return;
  }
  if (active_goal_id_.has_value() && result.goal_id != active_goal_id_.value()) {
    return;
  }

  const double distance_traveled = result.result->distance_traveled;
  const builtin_interfaces::msg::Duration& time_traveled = result.result->time_traveled;
  const double wait_time_s = has_active_waypoint_ ? active_waypoint_wait_time_s_ : 0.0;

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    if (result.result->destination_reached) {
      RCLCPP_INFO(this->get_logger(), "Goal succeeded: destination reached after %.2fm and %ds", distance_traveled,
                  time_traveled.sec);
    } else {
      RCLCPP_WARN(this->get_logger(), "Goal succeeded, but destination not reached after %.2fm and %ds", distance_traveled,
                  time_traveled.sec);
    }
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(this->get_logger(), "Goal canceled: traveled %.2fm and %ds", distance_traveled, time_traveled.sec);
  } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_ERROR(this->get_logger(), "Goal aborted: traveled %.2fm and %ds", distance_traveled, time_traveled.sec);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Goal finished with unknown result code: %d", static_cast<int>(result.code));
  }

  has_active_waypoint_ = false;
  active_route_waypoints_.clear();
  active_route_waypoint_indices_.clear();
  active_goal_id_.reset();
  has_completed_one_goal_ = true;
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED && wait_time_s > 0.0) {
    RCLCPP_INFO(this->get_logger(), "Waiting %.2fs before planning next waypoint", wait_time_s);
    const double now_s = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    auto_planning_resume_time_s_ = now_s + wait_time_s;
  }
  auto_planning_timer_->reset();  // restart auto-planning timer
}

}  // namespace plan_route_action_client

/**
 * @brief Starts the ROS node.
 *
 * @param[in] argc number of command-line arguments
 * @param[in] argv command-line arguments
 * @return process exit code
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plan_route_action_client::PlanRouteActionClient>());
  rclcpp::shutdown();

  return 0;
}

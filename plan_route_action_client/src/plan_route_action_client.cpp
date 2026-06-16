// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <functional>
#include <optional>

#include <lanelet2_core/geometry/LaneletMap.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "plan_route_action_client/plan_route_action_client.hpp"

namespace plan_route_action_client {

/**
 * @brief Parses WGS84 waypoints from "<LATITUDE>,<LONGITUDE>" strings.
 *
 * @param[in] waypoints_param waypoint parameter values
 * @return parsed latitude/longitude pairs, or `std::nullopt` if parsing fails
 */
std::optional<std::vector<std::pair<double, double>>> parseWaypoints(const std::vector<std::string>& waypoints_param) {
  std::vector<std::pair<double, double>> waypoints;
  for (const auto& waypoint : waypoints_param) {
    size_t comma_pos = waypoint.find(',');
    if (comma_pos != std::string::npos) {
      try {
        double lat = std::stod(waypoint.substr(0, comma_pos));
        double lon = std::stod(waypoint.substr(comma_pos + 1));
        waypoints.emplace_back(lat, lon);
      } catch (const std::invalid_argument& e) {
        return std::nullopt;
      } catch (const std::out_of_range& e) {
        return std::nullopt;
      }
    } else {
      return std::nullopt;
    }
  }
  return waypoints;
}

PlanRouteActionClient::PlanRouteActionClient() : Node("plan_route_action_client") {
  this->declareAndLoadParameter("ll2_map_server_name", ll2_map_server_name_, "Name of lanelet2_map_server node", false);
  this->declareAndLoadParameter(
      "waypoints", waypoints_param_,
      "List of WGS84 waypoints to follow (list of strings with comma-separated '<LATITUDE>,<LONGITUDE>')", true);
  this->declareAndLoadParameter("enable_random_destination", enable_random_destination_,
                                "Whether to plan a route to a random destination", true);
  this->declareAndLoadParameter(
      "enable_continuous_planning", enable_continuous_planning_,
      "Whether to continuously plan a new route (either looping waypoints or to a random destination)", true);
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
      auto parsed_waypoints = parseWaypoints(waypoints_param_);
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

  // action client
  action_client_ = rclcpp_action::create_client<PlanRoute>(this, "/planning/lanelet2_route_planning/plan_route");

  // ll2 map interface
  ll2_interface_ = std::make_unique<Lanelet2MapInterface>(*this, ll2_map_server_name_);

  // parse waypoints
  auto parsed_waypoints = parseWaypoints(waypoints_param_);
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
  sendGoal(msg);
}

void PlanRouteActionClient::autoPlanningTimerCallback() {
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
  RCLCPP_INFO(this->get_logger(), "Planning route to next waypoint (%.6f, %.6f)", waypoints_[next_waypoint_idx_].first,
              waypoints_[next_waypoint_idx_].second);

  // check if map is loaded
  if (!ll2_interface_->map_loaded_) {
    RCLCPP_ERROR(this->get_logger(), "Map not loaded, cannot generate waypoint");
    return;
  }

  // generate goal pose from waypoint
  auto goal_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
  auto ll2_projector = ll2_interface_->getProjectorPtr();
  if (ll2_projector) {
    lanelet::GPSPoint gps_waypoint;
    gps_waypoint.lat = waypoints_[next_waypoint_idx_].first;
    gps_waypoint.lon = waypoints_[next_waypoint_idx_].second;
    lanelet::BasicPoint3d map_waypoint = ll2_projector->forward(gps_waypoint);
    goal_pose->pose.position.x = map_waypoint.x();
    goal_pose->pose.position.y = map_waypoint.y();
    goal_pose->pose.position.z = 0.0;
    goal_pose->header.frame_id = ll2_interface_->map_frame_id_;
    goal_pose->header.stamp = this->now();
  }

  // send goal
  if (!goal_pose->header.frame_id.empty()) {
    RCLCPP_INFO(this->get_logger(), "Generated waypoint goal pose (%.3f, %.3f, %.3f) in frame '%s'", goal_pose->pose.position.x,
                goal_pose->pose.position.y, goal_pose->pose.position.z, goal_pose->header.frame_id.c_str());
    auto_planning_timer_->cancel();  // cancel auto-planning timer until goal completion
    next_waypoint_idx_++;
    this->sendGoal(goal_pose);
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
    auto random_lanelet = *std::next(map->laneletLayer.begin(), std::rand() % map->laneletLayer.size());
    auto centerline = random_lanelet.centerline();
    if (centerline.size() > 0) {
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
    this->sendGoal(goal_pose);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to generate random goal pose");
  }
}

void PlanRouteActionClient::sendGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Requesting to plan route to destination (%.3f, %.3f, %.3f) in frame '%s'",
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->header.frame_id.c_str());

  // check if action server is available
  if (!action_client_->wait_for_action_server(std::chrono::duration<double>(0.1))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available, aborting");
    auto_planning_timer_->reset();  // restart auto-planning timer
    return;
  }

  // build goal
  auto goal = PlanRoute::Goal();
  goal.destination = geometry_msgs::msg::PointStamped();
  goal.destination.header = msg->header;
  goal.destination.point = msg->pose.position;

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
    auto_planning_timer_->reset();  // restart auto-planning timer
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by action server");
  }
}

void PlanRouteActionClient::feedbackCallback(GoalHandlePlanRoute::SharedPtr goal_handle,
                                             const std::shared_ptr<const PlanRoute::Feedback> feedback) {
  (void)goal_handle;

  const double distance_traveled = feedback->distance_traveled;
  const double distance_total = feedback->distance_remaining + feedback->distance_traveled;
  rclcpp::Duration time_traveled(feedback->time_traveled.sec, feedback->time_traveled.nanosec);
  rclcpp::Duration time_remaining(feedback->time_remaining.sec, feedback->time_remaining.nanosec);
  rclcpp::Duration time_total = time_traveled + time_remaining;
  RCLCPP_INFO(this->get_logger(), "Route progress: %.2f / %.2f m, %.1f / %.1f s", distance_traveled, distance_total,
              time_traveled.seconds(), time_total.seconds());
}

void PlanRouteActionClient::resultCallback(const GoalHandlePlanRoute::WrappedResult& result) {
  const double distance_traveled = result.result->distance_traveled;
  const builtin_interfaces::msg::Duration& time_traveled = result.result->time_traveled;
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

  auto_planning_timer_->reset();  // restart auto-planning timer
  has_completed_one_goal_ = true;
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

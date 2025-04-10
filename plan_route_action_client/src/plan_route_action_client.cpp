#include <chrono>
#include <functional>

#include <plan_route_action_client/plan_route_action_client.hpp>

namespace plan_route_action_client {

PlanRouteActionClient::PlanRouteActionClient() : Node("plan_route_action_client") {
  this->declareAndLoadParameter("cancel_route", cancel_route_,
                                "Cancel active route planning action (to be set at runtime)", false);
  this->setup();
}

template <typename T>
void PlanRouteActionClient::declareAndLoadParameter(const std::string& name, T& param, const std::string& description,
                                                    const bool add_to_auto_reconfigurable_params,
                                                    const bool is_required, const bool read_only,
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

    // handle cancel_route parameter
    if (param.get_name() == "cancel_route") {
      cancel_route_ = param.as_bool();
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
  parameters_callback_ = this->add_on_set_parameters_callback(
      std::bind(&PlanRouteActionClient::parametersCallback, this, std::placeholders::_1));

  // subscriber for goal pose (from rviz)
  goal_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&PlanRouteActionClient::goalPoseCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", goal_pose_subscriber_->get_topic_name());

  // action client
  action_client_ = rclcpp_action::create_client<GlobalManeuver>(this, "/lanelet2_route_planning/plan_route");
}

void PlanRouteActionClient::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received goal pose (%.3f, %.3f, %.3f) in frame '%s'", msg->pose.position.x,
              msg->pose.position.y, msg->pose.position.z, msg->header.frame_id.c_str());
  sendGoal(msg);
}

void PlanRouteActionClient::sendGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Requesting to plan route to destination (%.3f, %.3f, %.3f) in frame '%s'",
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->header.frame_id.c_str());

  // check if action server is available
  if (!action_client_->wait_for_action_server(std::chrono::duration<double>(0.1))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available, aborting");
    return;
  }

  // build goal
  auto goal = GlobalManeuver::Goal();
  goal.destination = geometry_msgs::msg::PointStamped();
  goal.destination.header = msg->header;
  goal.destination.point = msg->pose.position;

  // send goal
  auto send_goal_options = rclcpp_action::Client<GlobalManeuver>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&PlanRouteActionClient::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
      std::bind(&PlanRouteActionClient::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&PlanRouteActionClient::resultCallback, this, std::placeholders::_1);
  goal_handle_future_ = action_client_->async_send_goal(goal, send_goal_options);
  RCLCPP_INFO(this->get_logger(), "Goal sent");
}

void PlanRouteActionClient::goalResponseCallback(const GoalHandleGlobalManeuver::SharedPtr& goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal rejected by action server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by action server");
  }
}

void PlanRouteActionClient::feedbackCallback(GoalHandleGlobalManeuver::SharedPtr goal_handle,
                                             const std::shared_ptr<const GlobalManeuver::Feedback> feedback) {
  (void)goal_handle;

  const double distance_traveled = feedback->distance_traveled;
  const double distance_total = feedback->distance_remaining + feedback->distance_traveled;
  rclcpp::Duration time_traveled(feedback->time_traveled.sec, feedback->time_traveled.nanosec);
  rclcpp::Duration time_remaining(feedback->time_remaining.sec, feedback->time_remaining.nanosec);
  rclcpp::Duration time_total = time_traveled + time_remaining;
  RCLCPP_INFO(this->get_logger(), "Route progress: %.2f / %.2f m, %.1f / %.1f s", distance_traveled, distance_total,
              time_traveled.seconds(), time_total.seconds());
}

void PlanRouteActionClient::resultCallback(const GoalHandleGlobalManeuver::WrappedResult& result) {
  const double distance_traveled = result.result->distance_traveled;
  const builtin_interfaces::msg::Duration& time_traveled = result.result->time_traveled;
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    if (result.result->destination_reached) {
      RCLCPP_INFO(this->get_logger(), "Goal succeeded: destination reached after %.2fm and %ds", distance_traveled,
                  time_traveled.sec);
    } else {
      RCLCPP_WARN(this->get_logger(), "Goal succeeded, but destination not reached after %.2fm and %ds",
                  distance_traveled, time_traveled.sec);
    }
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(this->get_logger(), "Goal canceled: traveled %.2fm and %ds", distance_traveled, time_traveled.sec);
  } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_ERROR(this->get_logger(), "Goal aborted: traveled %.2fm and %ds", distance_traveled, time_traveled.sec);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Goal finished with unknown result code: %d", static_cast<int>(result.code));
  }
}

}  // namespace plan_route_action_client

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plan_route_action_client::PlanRouteActionClient>());
  rclcpp::shutdown();

  return 0;
}

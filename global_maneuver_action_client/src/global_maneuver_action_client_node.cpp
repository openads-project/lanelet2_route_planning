#include "global_maneuver_action_client/global_maneuver_action_client_node.hpp"

//lanelet2
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>

//tf2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace global_maneuver_action_client {

GlobalManeuverActionClient::GlobalManeuverActionClient() : Node("global_maneuver_action_client") {
  /// declare and load node parameters
  this->declareAndLoadParameter("random_planning", random_planning_, "Bool indicating whether to plan random routes");
  this->declareAndLoadParameter("map_server_name", map_server_name_, "Name of the lanelet2 map server");

  // subscriber and setup action client and
  this->setup();
}

template <typename T>
void GlobalManeuverActionClient::declareAndLoadParameter(
    const std::string& name, T& member_param, const std::string& description,
    const bool add_to_auto_reconfigurable_params, const bool is_required, const bool read_only,
    const std::optional<T>& from_value, const std::optional<T>& to_value, const std::optional<T>& step_value,
    const std::string& additional_constraints) {
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto param_type = rclcpp::ParameterValue(member_param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
    if constexpr (std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      T step = step_value.has_value() ? step_value.value() : 0;
      range.set__from_value(from_value.value()).set__to_value(to_value.value()).set__step(step);
      param_desc.integer_range = {range};
    } else if constexpr (std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      T step = step_value.has_value() ? step_value.value() : 0.0;
      range.set__from_value(from_value.value()).set__to_value(to_value.value()).set__step(step);
      param_desc.floating_point_range = {range};
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter type does not support range.");
    }
  }

  this->declare_parameter(name, param_type, param_desc);

  try {
    member_param = this->get_parameter(name).get_value<T>();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    if (is_required) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Parameter '" << name << "' not set but required. Exiting.");
      exit(EXIT_FAILURE);
    } else {
      std::stringstream ss;
      ss << "Parameter '" << name << "' not set. Using default value: ";
      if constexpr (is_vector_v<T>) {
        ss << "[";
        for (const auto& element : member_param) ss << element << (&element != &member_param.back() ? ", " : "]");
      } else {
        ss << member_param;
      }
      RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
    }
  }

  if (add_to_auto_reconfigurable_params) {
    std::function<void(const rclcpp::Parameter&)> setter = [&member_param](const rclcpp::Parameter& param) {
      member_param = param.get_value<T>();
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
rcl_interfaces::msg::SetParametersResult GlobalManeuverActionClient::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
      }
    }
  }

  // mark parameter change successful
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}

void GlobalManeuverActionClient::setup() {
  // goal-pose subscriber
  subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      kGoalPoseTopic, 1, std::bind(&GlobalManeuverActionClient::goalPoseCallback, this, std::placeholders::_1));

  // random-planning timer
  timer_ = create_wall_timer(5s, std::bind(&GlobalManeuverActionClient::generateRandomGoal, this));

  // action client
  action_client_ = rclcpp_action::create_client<GlobalManeuver>(this, "ll2_route_planning/execute_global_maneuver");

  // lanelet2 map interface
  ll2if_ = std::make_unique<LL2MapInterface>(*this, map_server_name_);
}

void GlobalManeuverActionClient::goalPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!random_planning_)
    sendGoal(msg);
  else
    RCLCPP_WARN(this->get_logger(), "Ignoring goal pose subscribed on topic '%s', random planning is enabled",
                kGoalPoseTopic.c_str());
}

void GlobalManeuverActionClient::sendGoal(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Triggering global maneuver to destination (%.3f, %.3f, %.3f) in frame '%s'",
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->header.frame_id.c_str());

  // check for action server
  if (!action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available, returning");
    return;
  }

  // build goal
  auto goal = GlobalManeuver::Goal();
  goal.destination = geometry_msgs::msg::PointStamped();
  goal.destination.header = msg->header;
  goal.destination.point = msg->pose.position;

  // send goal
  RCLCPP_INFO(this->get_logger(), "Sending goal");
  auto send_goal_options = rclcpp_action::Client<GlobalManeuver>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&GlobalManeuverActionClient::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
      std::bind(&GlobalManeuverActionClient::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
      std::bind(&GlobalManeuverActionClient::resultCallback, this, std::placeholders::_1);
  goal_handle_future_ = action_client_->async_send_goal(goal, send_goal_options);
}

void GlobalManeuverActionClient::goalResponseCallback(const GoalHandleGlobalManeuver::SharedPtr& goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
  }
}

void GlobalManeuverActionClient::generateRandomGoal() {
  if (random_planning_) {
    // check if action is running
    if (goal_handle_future_.valid() && goal_handle_future_.wait_for(1s) == std::future_status::timeout) {
      RCLCPP_DEBUG(this->get_logger(), "Not generating a random goal while action is running");
      return;
    } else {
      if (!ll2if_->map_loaded_) {
        RCLCPP_WARN(get_logger(), "Lanelet2 map not loaded, cannot generate random goal");
        return;
      } else {
        auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
        lanelet::LaneletMapConstPtr llmap = ll2if_->getMapPtr();
        if (!llmap->laneletLayer.empty()) {
          // get random iterator of laneletLayer
          auto random_lanelet = *std::next(llmap->laneletLayer.begin(), std::rand() % llmap->laneletLayer.size());
          // get centerline of random lanelet
          auto centerline = random_lanelet.centerline();
          if (centerline.size() > 0) {
            // get last point of lanelet centerline
            auto last_point = centerline.back();
            msg->pose.position.x = last_point.x();
            msg->pose.position.y = last_point.y();
            msg->pose.position.z = last_point.z();
            if (centerline.size() > 1) {
              // get heading from last and previous centerline point
              auto heading = atan2(last_point.y() - centerline[centerline.size() - 2].y(),
                                   last_point.x() - centerline[centerline.size() - 2].x());
              // add heading to pose
              tf2::Quaternion q;
              q.setRPY(0, 0, heading);
              msg->pose.orientation = tf2::toMsg(q);
            }
            msg->header.frame_id = ll2if_->map_frame_id_;
            msg->header.stamp = rclcpp::Clock().now();
            sendGoal(msg);
            RCLCPP_INFO(get_logger(), "Generated random goal at (%.3f, %.3f, %.3f) in frame '%s'", msg->pose.position.x,
                        msg->pose.position.y, msg->pose.position.z, msg->header.frame_id.c_str());
          } else {
            RCLCPP_DEBUG(get_logger(), "Random lanelet has no centerline points");
            return;
          }
        } else {
          RCLCPP_DEBUG(get_logger(), "No lanelets in map");
          return;
        }
      }
    }
  }
}

void GlobalManeuverActionClient::feedbackCallback(GoalHandleGlobalManeuver::SharedPtr goal_handle,
                                                  const std::shared_ptr<const GlobalManeuver::Feedback> feedback) {
  (void)goal_handle;

  double distance_traveled = feedback->distance_traveled;
  double distance_total = feedback->distance_remaining + feedback->distance_traveled;
  rclcpp::Duration rcl_time_traveled(feedback->time_traveled.sec, feedback->time_traveled.nanosec);
  rclcpp::Duration rcl_time_remaining(feedback->time_remaining.sec, feedback->time_remaining.nanosec);
  rclcpp::Duration rcl_time_total = rcl_time_traveled + rcl_time_remaining;
  RCLCPP_INFO(this->get_logger(), "Progress towards destination: %.2f / %.2f m, %.1f / %.1f s", distance_traveled,
              distance_total, rcl_time_traveled.seconds(), rcl_time_total.seconds());
}

void GlobalManeuverActionClient::resultCallback(const GoalHandleGlobalManeuver::WrappedResult& result) {
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    double distance_traveled = result.result->distance_traveled;
    builtin_interfaces::msg::Duration time_traveled = result.result->time_traveled;
    if (result.result->destination_reached) {
      RCLCPP_INFO(this->get_logger(), "Goal succeeded: destination reached after %.2fm and %ds", distance_traveled,
                  time_traveled.sec);
    } else {
      RCLCPP_WARN(this->get_logger(), "Goal succeeded, but destination not reached after %.2fm and %ds",
                  distance_traveled, time_traveled.sec);
    }
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(this->get_logger(), "Goal was canceled");
  } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Goal finished with unknown result code");
  }
}

}  // namespace global_maneuver_action_client

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<global_maneuver_action_client::GlobalManeuverActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

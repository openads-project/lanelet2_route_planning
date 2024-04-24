#include "global_maneuver_action_client/global_action_client_node.hpp"

namespace global_maneuver_action_client {

GlobalManeuverActionClient::GlobalManeuverActionClient() : Node("global_maneuver_action_client")
{
  subscriber_ = create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 1, std::bind(&GlobalManeuverActionClient::sendGoal, this, std::placeholders::_1));
  action_client_ = rclcpp_action::create_client<GlobalManeuver>(this, "ll2_route_planning/execute_global_maneuver");
}

void GlobalManeuverActionClient::sendGoal(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Triggering global maneuver to destination in frame '%s' at position (%.3f, %.3f, %.3f)", msg->header.frame_id.c_str(), msg->point.x, msg->point.y, msg->point.z);

  // check for action server
  if(!this->action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available, returning");
    return;
  }

  // check for running actions
  if(goal_handle_future_.valid())
  {
    auto goal_handle = goal_handle_future_.get();
    RCLCPP_WARN(this->get_logger(), "Existing global maneuver detected, cancelling before sending new goal");
    auto cancel_future = action_client_->async_cancel_goals_before(this->now());
  }

  // build goal
  auto goal = GlobalManeuver::Goal();
  goal.destination = msg;

  // send goal
  RCLCPP_INFO(this->get_logger(), "Sending goal");
  auto send_goal_options = rclcpp_action::Client<GlobalManeuver>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&GlobalManeuverActionClient::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&GlobalManeuverActionClient::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&GlobalManeuverActionClient::resultCallback, this, std::placeholders::_1);
  goal_handle_future_ = this->action_client_->async_send_goal(goal, send_goal_options);
}

void GlobalManeuverActionClient::goal_response_callback(std::shared_future<GoalHandleGlobalManeuver>::SharedPtr> future) {

  auto goal_handle = future->get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
  }
}

void GlobalManeuverActionClient::feedback_callback(GoalHandleGlobalManeuver::SharedPtr goal_handle, const std::shared_ptr<const GlobalManeuver::Feedback> feedback) {

  double distance_remaining = feedback->distance_remaining;
  double distance_total = feedback->distance_traveled + distance_remaining;
  builtin_interfaces::msg::Duration time_remaining = feedback->time_remaining;
  builtin_interfaces::msg::Duration time_total = feedback->time_traveled + time_remaining;
  RCLCPP_INFO(this->get_logger(), "Progress towards destination: %.2f / %.2f m, %d / %d s", distance_remaining, distance_total, time_remaining.sec, time_total.sec);
}

void GlobalManeuverActionClient::result_callback(const GoalHandleGlobalManeuver::WrappedResult& result) {

  double distance_traveled = result.result->distance_traveled;
  builtin_interfaces::msg::Duration time_traveled = result.result->time_traveled;
  if (feedback->destination_reached) {
    RCLCPP_INFO(this->get_logger(), "Destination reached after %.2fm and %.2fs", distance_traveled, time_traveled.sec);
  } else {
    RCLCPP_WARN(this->get_logger(), "Action finished, but destination not reached after %.2fm and %ds", distance_traveled, time_traveled.sec);
  }
}

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto planner = std::make_shared<GlobalManeuverActionClient>();
  rclcpp::spin(planner);
  rclcpp::shutdown();
  return 0;
}

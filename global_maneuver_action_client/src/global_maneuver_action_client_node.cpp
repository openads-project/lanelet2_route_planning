#include "global_maneuver_action_client/global_maneuver_action_client_node.hpp"

namespace global_maneuver_action_client {

GlobalManeuverActionClient::GlobalManeuverActionClient() : Node("global_maneuver_action_client")
{
  subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 1, std::bind(&GlobalManeuverActionClient::sendGoal, this, std::placeholders::_1));
  action_client_ = rclcpp_action::create_client<GlobalManeuver>(this, "ll2_route_planning/execute_global_maneuver");
}

void GlobalManeuverActionClient::sendGoal(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Triggering global maneuver to destination (%.3f, %.3f, %.3f) in frame '%s'", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->header.frame_id.c_str());

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
  goal.destination = geometry_msgs::msg::PointStamped();
  goal.destination.header = msg->header;
  goal.destination.point = msg->pose.position;

  // send goal
  RCLCPP_INFO(this->get_logger(), "Sending goal");
  auto send_goal_options = rclcpp_action::Client<GlobalManeuver>::SendGoalOptions();
//  send_goal_options.goal_response_callback = std::bind(&GlobalManeuverActionClient::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&GlobalManeuverActionClient::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&GlobalManeuverActionClient::resultCallback, this, std::placeholders::_1);
  goal_handle_future_ = this->action_client_->async_send_goal(goal, send_goal_options);
}

void GlobalManeuverActionClient::goalResponseCallback(const GoalHandleGlobalManeuver::SharedPtr& goal_handle) {

  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
  }
}

void GlobalManeuverActionClient::feedbackCallback(GoalHandleGlobalManeuver::SharedPtr goal_handle, const std::shared_ptr<const GlobalManeuver::Feedback> feedback) {

  (void)goal_handle;

  double distance_traveled = feedback->distance_traveled;
  double distance_total = feedback->distance_remaining + feedback->distance_traveled;
  rclcpp::Duration rcl_time_traveled(feedback->time_traveled.sec, feedback->time_traveled.nanosec);
  rclcpp::Duration rcl_time_remaining(feedback->time_remaining.sec, feedback->time_remaining.nanosec);
  rclcpp::Duration rcl_time_total = rcl_time_traveled + rcl_time_remaining;
  RCLCPP_INFO(this->get_logger(), "Progress towards destination: %.2f / %.2f m, %.1f / %.1f s", distance_traveled, distance_total, rcl_time_traveled.seconds(), rcl_time_total.seconds());
}

void GlobalManeuverActionClient::resultCallback(const GoalHandleGlobalManeuver::WrappedResult& result) {

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    double distance_traveled = result.result->distance_traveled;
    builtin_interfaces::msg::Duration time_traveled = result.result->time_traveled;
    if (result.result->destination_reached) {
      RCLCPP_INFO(this->get_logger(), "Goal succeeded: destination reached after %.2fm and %ds", distance_traveled, time_traveled.sec);
    } else {
      RCLCPP_WARN(this->get_logger(), "Goal succeeded, but destination not reached after %.2fm and %ds", distance_traveled, time_traveled.sec);
    }
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(this->get_logger(), "Goal was canceled");
  } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Goal finished with unknown result code");
  }
}

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<global_maneuver_action_client::GlobalManeuverActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

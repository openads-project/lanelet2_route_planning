#include "global_maneuver_action_client/global_action_client_node.hpp"

GlobalManeuverActionClient::GlobalManeuverActionClient() : Node("global_maneuver_action_client")
{
  subscriber_ = create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 1, std::bind(&GlobalManeuverActionClient::pointCallback, this, std::placeholders::_1));
  action_client_ = rclcpp_action::create_client<route_planning_msgs::action::GlobalManeuver>(this, "ll2_route_planning/execute_global_maneuver");
}

void GlobalManeuverActionClient::pointCallback(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Triggering global maneuver to destination in frame '%s' at position (%.3f, %.3f, %.3f)", msg->header.frame_id.c_str(), msg->point.x, msg->point.y, msg->point.z);

  // check for action server
  if(!this->action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available! Could not create a global maneuver action!");
    return;
  }

  // check for running actions
  if(goal_handle_future_.valid())
  {
    auto goal_handle = goal_handle_future_.get();
    RCLCPP_WARN(this->get_logger(), "There is a running action that will be canceled now!");
    auto cancel_future = action_client_->async_cancel_goals_before(this->now());
  }

  // send goal
  auto goal = route_planning_msgs::action::GlobalManeuver::Goal();
  goal.destination = msg;
  auto send_goal_options = rclcpp_action::Client<route_planning_msgs::action::GlobalManeuver>::SendGoalOptions();
  goal_handle_future_ = this->action_client_->async_send_goal(goal, send_goal_options);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto planner = std::make_shared<GlobalManeuverActionClient>();
  rclcpp::spin(planner);
  rclcpp::shutdown();
  return 0;
}

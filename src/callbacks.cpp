#include "lanelet2_route_planning/global_planner_node.hpp"

void GlobalPlanner::mapPoseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  ego_pose_ = *msg.get();
}

void GlobalPlanner::goalPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received a new goal-pose!");
  // Check for running actions
  if(goal_handle_future_.valid())
  {
    auto goal_handle = goal_handle_future_.get();
    auto goal_status = goal_handle->get_status();
    if(goal_status == rclcpp_action::GoalStatus::STATUS_EXECUTING || goal_status == rclcpp_action::GoalStatus::STATUS_ACCEPTED)
    {
      RCLCPP_WARN(this->get_logger(), "There is a running action that will be canceled now!");
      auto cancel_future = maneuver_action_client_->async_cancel_goal(goal_handle);
    }
  }

  if(!this->maneuver_action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available! Could not create a global maneuver action!");
    return;
  }

  auto action_goal = route_planning_interfaces::action::GlobalManeuver::Goal();
  action_goal.target_pos_x = msg->pose.position.x;
  action_goal.target_pos_y = msg->pose.position.y;

  RCLCPP_INFO(this->get_logger(), "Sending a new action goal to plan a maneuver to the desired goal-pose!");
  auto send_goal_options = rclcpp_action::Client<route_planning_interfaces::action::GlobalManeuver>::SendGoalOptions();
  goal_handle_future_ = this->maneuver_action_client_->async_send_goal(action_goal, send_goal_options);
}


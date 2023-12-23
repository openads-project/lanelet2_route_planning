#include "lanelet2_route_planning/global_planner_node.hpp"

void GlobalPlanner::mapPoseCallback(perception_msgs::msg::EgoData::SharedPtr msg)
{
  ego_data_ = *msg.get();
  ego_pose_ = perception_msgs::object_access::getPoseWithCovariance(ego_data_);
}

void GlobalPlanner::goalPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received a new goal-pose!");
  // Check for running actions
  if(goal_handle_future_.valid())
  {
    auto goal_handle = goal_handle_future_.get();
    RCLCPP_WARN(this->get_logger(), "There is a running action that will be canceled now!");
    auto cancel_future = maneuver_action_client_->async_cancel_goals_before(this->now());
  }

  if(!this->maneuver_action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available! Could not create a global maneuver action!");
    return;
  }

  auto action_goal = route_planning_msgs::action::GlobalManeuver::Goal();
  action_goal.target_pos_x = msg->pose.position.x;
  action_goal.target_pos_y = msg->pose.position.y;

  RCLCPP_INFO(this->get_logger(), "Sending a new action goal to plan a maneuver to the desired goal-pose!");
  auto send_goal_options = rclcpp_action::Client<route_planning_msgs::action::GlobalManeuver>::SendGoalOptions();
  goal_handle_future_ = this->maneuver_action_client_->async_send_goal(action_goal, send_goal_options);
}


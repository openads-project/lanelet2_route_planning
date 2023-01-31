#include "lanelet2_route_planning/global_planner_node.hpp"

void GlobalPlanner::mapPoseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  ego_pose_ = *msg.get();
}

void GlobalPlanner::goalPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  maneuver_action_client_ = rclcpp_action::create_client<lanelet2_route_planning_ifs::action::GlobalManeuver>(
      this,
      "~/execute_global_maneuver");

  if(!this->maneuver_action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available! Could not create a global maneuver action!");
    return;
  }

  auto action_goal = lanelet2_route_planning_ifs::action::GlobalManeuver::Goal();
  action_goal.target_pos_x = msg->pose.position.x;
  action_goal.target_pos_y = msg->pose.position.y;

  RCLCPP_INFO(this->get_logger(), "Sending goal");
  auto send_goal_options = rclcpp_action::Client<lanelet2_route_planning_ifs::action::GlobalManeuver>::SendGoalOptions();
  this->maneuver_action_client_->async_send_goal(action_goal, send_goal_options);
  RCLCPP_INFO(this->get_logger(), "async_send_goal finshed");
}


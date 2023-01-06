#include "lanelet2_route_planning/global_planner_node.hpp"

rclcpp_action::GoalResponse GlobalPlanner::actionHandleGoal(
  const rclcpp_action::GoalUUID& uuid,
  std::shared_ptr<const lanelet2_route_planning_ifs::action::GlobalManeuver::Goal> goal)
{
  maneuver_result_->destination_reached = false;
  RCLCPP_INFO(get_logger(), "Received a global maneuver request!");
  RCLCPP_INFO_STREAM(get_logger(), "Target Position Latitude: " << goal->target_pos_lat);
  RCLCPP_INFO_STREAM(get_logger(), "Target Position Longitude: " << goal->target_pos_lon);
  lanelet::GPSPoint glob_target;
  glob_target.lat = goal->target_pos_lat;
  glob_target.lon = goal->target_pos_lon;
  std::shared_ptr<lanelet::Projector> proj = ll2if_->getProjectorPtr();
  llmap_ = ll2if_->getMapPtr();
  lanelet::BasicPoint3d map_target = proj->forward(glob_target);
  maneuver_feedback_->destination_x = map_target.x();
  maneuver_feedback_->destination_y = map_target.y();

  routingGraph_ = routing::RoutingGraph::build(*llmap_, *trafficRules_);
  routingGraphBicycle_ = routing::RoutingGraph::build(*llmap_, *trafficRulesBicycle_);
  std::vector<std::string> err = routingGraph_->checkValidity();
  if(err.size()>0)
  {
    RCLCPP_ERROR(get_logger(), "Routing-Graph of given lanelet-map is invalid!");
    for(int i = 0; i<err.size(); i++)
    {
      RCLCPP_ERROR_STREAM(get_logger(), err[i]);
    }
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Check for global position of ego-vehicle
  if(!egoPositionSanityCheck() || !targetPositionSanityCheck(map_target.x(), map_target.y()))
  {
    RCLCPP_ERROR(get_logger(), "Unable to plan a global maneuver!");
    return rclcpp_action::GoalResponse::REJECT;
  }

  visualization_msgs::msg::Marker marker = convertDestination2Marker(map_target.x(), map_target.y(), ll2if_->map_frame_id_);
  viz_destination_pub_->publish(marker);

  //planRoute(start_ll_, target_ll_);

  // accept action goal request
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalPlanner::actionHandleCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<lanelet2_route_planning_ifs::action::GlobalManeuver>> goal_handle)
{
  // this callback is invoked when a running action is requested to cancel
  RCLCPP_INFO(get_logger(), "Received request to cancel action goal");

  // accept action cancel request
  return rclcpp_action::CancelResponse::ACCEPT;

}

void GlobalPlanner::actionHandleAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<lanelet2_route_planning_ifs::action::GlobalManeuver>> goal_handle)
{
  // this callback is invoked when an action goal request is accepted
  // execute the action in a separate thread to avoid blocking
  std::thread{std::bind(&GlobalPlanner::actionExecute, this, std::placeholders::_1), goal_handle}.detach();
}

void GlobalPlanner::actionExecute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<lanelet2_route_planning_ifs::action::GlobalManeuver>> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing action goal");

  // define a sleeping rate between computing individual Fibonacci numbers
  rclcpp::Rate loop_rate(0.1);
  // create handy accessors for the action goal
  const auto goal = goal_handle->get_goal();

  bool destination_reached = false;

  while(!destination_reached)
  {
    // cancel, if requested
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(maneuver_result_);
      RCLCPP_INFO(get_logger(), "Action goal canceled");
      return;
    }

    // publish the current sequence as action feedback
    goal_handle->publish_feedback(maneuver_feedback_);
    RCLCPP_INFO(get_logger(), "Publishing action feedback");

    // sleep
    loop_rate.sleep();
  }
}
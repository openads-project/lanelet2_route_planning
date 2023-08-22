#include "lanelet2_route_planning/global_planner_node.hpp"

rclcpp_action::GoalResponse GlobalPlanner::actionHandleGoal(
  const rclcpp_action::GoalUUID& uuid,
  std::shared_ptr<const route_planning_interfaces::action::GlobalManeuver::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Received a global maneuver request!");
  RCLCPP_INFO_STREAM(get_logger(), "Target Position X: " << goal->target_pos_x);
  RCLCPP_INFO_STREAM(get_logger(), "Target Position Y: " << goal->target_pos_y);

  llmap_ = ll2if_->getMapPtr();
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
  if(!egoPositionSanityCheck() || !targetPositionSanityCheck(goal->target_pos_x, goal->target_pos_y))
  {
    RCLCPP_ERROR(get_logger(), "Unable to plan a global maneuver!");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (geometry::distance(start_ll_.polygon2d(), lanelet::BasicPoint2d(ego_pose_.pose.position.x, ego_pose_.pose.position.y)) > 0.5)
  {
    RCLCPP_ERROR(get_logger(), "Current ego-pose is not on the start lanelet anymore!");
    return rclcpp_action::GoalResponse::REJECT;
  }

  maneuver_result_ = std::make_shared<route_planning_interfaces::action::GlobalManeuver::Result>();
  maneuver_feedback_ = std::make_shared<route_planning_interfaces::action::GlobalManeuver::Feedback>();

  maneuver_result_->destination_reached = false;
  maneuver_feedback_->destination_x = goal->target_pos_x;
  maneuver_feedback_->destination_y = goal->target_pos_y;

  planRoute(start_ll_, target_ll_);

  // accept action goal request
  maneuver_start_time_ = now();
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalPlanner::actionHandleCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_interfaces::action::GlobalManeuver>> goal_handle)
{
  // this callback is invoked when a running action is requested to cancel
  RCLCPP_INFO(get_logger(), "Received request to cancel action goal");

  // accept action cancel request
  return rclcpp_action::CancelResponse::ACCEPT;

}

void GlobalPlanner::actionHandleAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_interfaces::action::GlobalManeuver>> goal_handle)
{
  // this callback is invoked when an action goal request is accepted
  // execute the action in a separate thread to avoid blocking
  std::thread{std::bind(&GlobalPlanner::actionExecute, this, std::placeholders::_1), goal_handle}.detach();
}

void GlobalPlanner::actionExecute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<route_planning_interfaces::action::GlobalManeuver>> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing action goal");
  // Reset / Initialize
  initializeLocalPathExtraction(global_route_);

  // define a sleeping rate
  rclcpp::Rate loop_rate(path_extraction_rate_);
  // create handy accessors for the action goal
  const auto goal = goal_handle->get_goal();
  while(!maneuver_result_->destination_reached)
  {
    // cancel, if requested
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(maneuver_result_);
      RCLCPP_INFO(get_logger(), "Action goal canceled");
      return;
    }

    // map update pending
    if(ll2if_->update_pending_)
    {
      //To-Do --> Safe Cancel Action?
    }

    // Check for destination reached
    // To-Do: add subscriber for actual vehicle velocity and check for standstill if require_standstill parameter is set!
    if(egoPositionSanityCheck())
    {
      double velocity = perception_interfaces::object_access::getVelLon(ego_data_);
      if (geometry::distance(lanelet::BasicPoint2d(goal->target_pos_x, goal->target_pos_y), lanelet::BasicPoint2d(ego_pose_.pose.position.x, ego_pose_.pose.position.y)) < target_reached_thr_ && (std::fabs(velocity) < vel_threshold_target_ || !require_standstill_))
      {
        RCLCPP_INFO(get_logger(),"Destination reached!");
        maneuver_result_->destination_reached = true;
        rclcpp::Duration diff = now()-maneuver_start_time_;
        maneuver_result_->duration.sec = diff.seconds();
        maneuver_result_->duration.nanosec = diff.nanoseconds();
        // Check if goal is done
        if (rclcpp::ok()) {
          goal_handle->succeed(maneuver_result_);
        }
      }
      // Extract local section of driveable space and route
      route_planning_interfaces::msg::DriveableSpace driveable_space_local;
      route_planning_interfaces::msg::Route route_local;

      rclcpp::Time start = now();
      extractLocalMapInfo(ego_pose_, global_driveable_space_, driveable_space_local, global_route_, route_local);
      rclcpp::Duration diff = now()-start;
      RCLCPP_INFO_STREAM(get_logger(), "Duration to extract the local map information: " << std::setprecision(10) << (diff.seconds() + (double)diff.nanoseconds() / 1e9));
      // publish the current sequence as action feedback
      goal_handle->publish_feedback(maneuver_feedback_);
      RCLCPP_INFO(get_logger(), "Publishing action feedback");
    }
    // sleep
    loop_rate.sleep();
    continue;
  }
}
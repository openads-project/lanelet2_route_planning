#include "lanelet2_route_planning/global_planner_node.hpp"

rclcpp_action::GoalResponse GlobalPlanner::actionHandleGoal(
  const rclcpp_action::GoalUUID& uuid,
  std::shared_ptr<const lanelet2_route_planning_interfaces::action::GlobalManeuver::Goal> goal)
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

  if (geometry::distance(start_ll_.polygon2d(), lanelet::BasicPoint2d(ego_pose_.pose.pose.position.x, ego_pose_.pose.pose.position.y)) > 0.5)
  {
    RCLCPP_ERROR(get_logger(), "Current ego-pose is not on the start lanelet anymore!");
    return rclcpp_action::GoalResponse::REJECT;
  }

  visualization_msgs::msg::Marker marker = convertDestination2Marker(goal->target_pos_x, goal->target_pos_y, ll2if_->map_frame_id_);
  viz_destination_pub_->publish(marker);

  maneuver_result_ = std::make_shared<lanelet2_route_planning_interfaces::action::GlobalManeuver::Result>();
  maneuver_feedback_ = std::make_shared<lanelet2_route_planning_interfaces::action::GlobalManeuver::Feedback>();

  maneuver_result_->destination_reached = false;
  maneuver_feedback_->destination_x = goal->target_pos_x;
  maneuver_feedback_->destination_y = goal->target_pos_y;

  planRoute(start_ll_, target_ll_);

  // accept action goal request
  maneuver_start_time_ = now();
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalPlanner::actionHandleCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<lanelet2_route_planning_interfaces::action::GlobalManeuver>> goal_handle)
{
  // this callback is invoked when a running action is requested to cancel
  RCLCPP_INFO(get_logger(), "Received request to cancel action goal");

  // accept action cancel request
  return rclcpp_action::CancelResponse::ACCEPT;

}

void GlobalPlanner::actionHandleAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<lanelet2_route_planning_interfaces::action::GlobalManeuver>> goal_handle)
{
  // this callback is invoked when an action goal request is accepted
  // execute the action in a separate thread to avoid blocking
  std::thread{std::bind(&GlobalPlanner::actionExecute, this, std::placeholders::_1), goal_handle}.detach();
}

void GlobalPlanner::actionExecute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<lanelet2_route_planning_interfaces::action::GlobalManeuver>> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing action goal");

  // define a sleeping rate
  rclcpp::Rate loop_rate(10.0);
  // create handy accessors for the action goal
  const auto goal = goal_handle->get_goal();

  while(!maneuver_result_->destination_reached)
  {

    // Check for destination reached
    // To-Do: add subscriber for actual vehicle velocity and check for standstill if require_standstill parameter is set!
    bool require_standstill = false;
    double velocity = 0.0;
    // if (geometry::distance(target_point_, cur_pos) < 5.5 && (std::fabs(velocity) < 0.05 || !require_standstill))
    // {
    //   RCLCPP_INFO(get_logger(),"Destination reached!");
    //   maneuver_result_->destination_reached = true;
    //   maneuver_result_->duration = (now()-maneuver_start_time_).seconds();
    //   // Check if goal is done
    //   if (rclcpp::ok()) {
    //     goal_handle->succeed(maneuver_result_);
    //   }
    // }

    // cancel, if requested
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(maneuver_result_);
      RCLCPP_INFO(get_logger(), "Action goal canceled");
      return;
    }

    if(ll2if_->update_pending_)
    {
      //To-Do --> Safe Cancel Action?
    }

    if(egoPositionSanityCheck())
    {
      lanelet::BasicPoint2d cur_pos;
      lanelet::ConstLanelet current_ll;
      unsigned int lane_network_route_index;
      unsigned int lane_network_spatial_index;
      unsigned int current_lane;
      if(false)//locateInLaneNetwork(lane_network_, cur_pos, current_ll, lane_network_route_index, lane_network_spatial_index, current_lane))
      {
        // // Roadtype
        // std::string current_ll_location = current_ll.hasAttribute("location") ? current_ll.attribute("location").value() : "";
        // std::string current_ll_subtype  = current_ll.hasAttribute("subtype")  ? current_ll.attribute("subtype").value() : "";

        // // Arc coordinates w.r.t. current lanelet
        // lanelet::ArcCoordinates arc = lanelet::geometry::toArcCoordinates(current_ll.centerline2d(), cur_pos);
        // double current_ll_s = arc.length;
        // double current_ll_d = arc.distance;

        // // Arc coordinates w.r.t. current lane
        // arc                          = lanelet::geometry::toArcCoordinates(lane_network_.lanes[current_lane], cur_pos);
        // double current_lane_s = arc.length;
        // double current_lane_d = arc.distance;

        // // Arc coordinates w.r.t. shortest path in route
        // const lanelet::ArcCoordinates arc_center_route = geometry::toArcCoordinates(shortest_path_centerline_, cur_pos);
        // double shortest_path_s = arc_center_route.length;
        // double shortest_path_d = arc_center_route.distance;
      }
      else
      {
        loop_rate.sleep(); // sleep
        continue;
      }
    }
    else
    {
      loop_rate.sleep(); // sleep
      continue;
    }

    // publish the current sequence as action feedback
    goal_handle->publish_feedback(maneuver_feedback_);
    RCLCPP_INFO(get_logger(), "Publishing action feedback");

    // sleep
    loop_rate.sleep();
  }
}
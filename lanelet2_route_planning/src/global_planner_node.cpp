#include "lanelet2_route_planning/global_planner_node.hpp"

GlobalPlanner::GlobalPlanner() : Node("global_planner")
{
  startup_timer_ = this->create_wall_timer(0.1s, std::bind(&GlobalPlanner::initializeGlobalPlanner, this));
}

void GlobalPlanner::initializeMapInterface()
{
  // To-Do load name as parameter
  std::string map_server_name = "ll2_map_server";
  // Important: shared_from_this() can not be called from within the constructor
  ll2if_ = new LL2MapInterface(shared_from_this(), map_server_name);
}

void GlobalPlanner::initializeGlobalPlanner()
{
  if(!ll2if_->map_loaded_)
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for Lanelet2-Map-Interface to load map!");
    return;
  }
  else
  {
    // create an action server for handling action goal requests
    maneuver_action_server_ = rclcpp_action::create_server<lanelet2_route_planning_ifs::action::GlobalManeuver>(this, "~/execute_global_maneuver",
    std::bind(&GlobalPlanner::actionHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GlobalPlanner::actionHandleCancel, this, std::placeholders::_1),
    std::bind(&GlobalPlanner::actionHandleAccepted, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Created 'execute_global_maneuver' action-server!");
    startup_timer_->cancel();
  }
}

rclcpp_action::GoalResponse GlobalPlanner::actionHandleGoal(
  const rclcpp_action::GoalUUID& uuid,
  std::shared_ptr<const lanelet2_route_planning_ifs::action::GlobalManeuver::Goal> goal)
{
  maneuver_result_->destination_reached = false;
  RCLCPP_INFO(this->get_logger(), "Received a global maneuver request!");
  RCLCPP_INFO_STREAM(this->get_logger(), "Target Position Latitude: " << goal->target_pos_lat);
  RCLCPP_INFO_STREAM(this->get_logger(), "Target Position Longitude: " << goal->target_pos_lon);
  lanelet::GPSPoint glob_target;
  glob_target.lat = goal->target_pos_lat;
  glob_target.lon = goal->target_pos_lon;
  std::shared_ptr<lanelet::Projector> proj = ll2if_->getProjectorPtr();
  lanelet::LaneletMapConstPtr llmap = ll2if_->getMapPtr();
  lanelet::BasicPoint3d map_target = proj->forward(glob_target);
  maneuver_feedback_->destination_x = map_target.x();
  maneuver_feedback_->destination_y = map_target.y();
  // Check if there are any lanelets at the destination
  std::vector<std::pair<double, lanelet::ConstLanelet>> nearestLaneletsTarget = lanelet::geometry::findNearest(llmap->laneletLayer, lanelet::BasicPoint2d(map_target.x(), map_target.y()), 5);
  //Lanelet2Utilities::laneletSorting(BasicPoint2d(map_target.x(), map_target.y()), nearestLaneletsTarget, {}, trafficRules_, {});

  // accept action goal request
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalPlanner::actionHandleCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<lanelet2_route_planning_ifs::action::GlobalManeuver>> goal_handle)
{
  // this callback is invoked when a running action is requested to cancel
  RCLCPP_INFO(this->get_logger(), "Received request to cancel action goal");

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
  RCLCPP_INFO(this->get_logger(), "Executing action goal");

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
      RCLCPP_INFO(this->get_logger(), "Action goal canceled");
      return;
    }

    // publish the current sequence as action feedback
    goal_handle->publish_feedback(maneuver_feedback_);
    RCLCPP_INFO(this->get_logger(), "Publishing action feedback");

    // sleep before computing the next Fibonacci number
    loop_rate.sleep();
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // MutliThreadedExecutor is mandatory when using the lanelet2_map_interface
  rclcpp::executors::MultiThreadedExecutor executor;
  auto planner = std::make_shared<GlobalPlanner>();
  executor.add_node(planner);
  planner->initializeMapInterface();
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

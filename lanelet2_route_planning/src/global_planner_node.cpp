#include "lanelet2_route_planning/global_planner_node.hpp"

GlobalPlanner::GlobalPlanner() : Node("global_planner")
{
  startup_timer_ = this->create_wall_timer(0.1s, std::bind(&GlobalPlanner::initializeGlobalPlanner, this));
  map_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/ego_vehicle/map_pose", 1, std::bind(&GlobalPlanner::mapPoseCallback, this, std::placeholders::_1));
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
    RCLCPP_INFO(get_logger(), "Waiting for Lanelet2-Map-Interface to load map!");
    return;
  }
  else
  {
    // create an action server for handling action goal requests
    maneuver_action_server_ = rclcpp_action::create_server<lanelet2_route_planning_ifs::action::GlobalManeuver>(this, "~/execute_global_maneuver",
    std::bind(&GlobalPlanner::actionHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GlobalPlanner::actionHandleCancel, this, std::placeholders::_1),
    std::bind(&GlobalPlanner::actionHandleAccepted, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Created 'execute_global_maneuver' action-server!");
    startup_timer_->cancel();
  }
}

bool GlobalPlanner::egoPositionSanityCheck()
{
  return true;
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

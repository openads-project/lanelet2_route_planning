#include "lanelet2_route_planning/global_planner_node.hpp"

GlobalPlanner::GlobalPlanner() : Node("global_planner")
{

}

void GlobalPlanner::initializeMapInterface()
{
  // To-Do load name as parameter
  std::string map_server_name = "ll2_map_server";
  // Important: shared_from_this() can not be called from within the constructor
  ll2if_ = new LL2MapInterface(shared_from_this(), map_server_name);
}

int main(int argc, char ** argv)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Start");
  rclcpp::init(argc, argv);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Init Ready");
  auto planner = std::make_shared<GlobalPlanner>();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Created Node Pointer");

  while(rclcpp::ok())
  {
    rclcpp::spin_some(planner);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Spinned once!");
    if(!planner->map_if_initialized_)
    {
      planner->initializeMapInterface();
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Initialized Map Interface");
    }
  }
  
  rclcpp::shutdown();
  return 0;
}

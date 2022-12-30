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

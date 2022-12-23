#include "lanelet2_route_planning/global_planner_node.hpp"

GlobalPlanner::GlobalPlanner() : Node("global_planner")
{
  timer_ = this->create_wall_timer(500ms, std::bind(&GlobalPlanner::timer_callback, this));
}

void GlobalPlanner::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "Hello World!");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlanner>());
  rclcpp::shutdown();
  return 0;
}

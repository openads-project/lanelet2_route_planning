#include "lanelet2_route_planning/global_planner_node.hpp"

void GlobalPlanner::mapPoseCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  ego_pose_ = *msg.get();
}


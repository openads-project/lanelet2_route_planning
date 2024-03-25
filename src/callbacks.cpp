#include "lanelet2_route_planning/global_planner_node.hpp"

void GlobalPlanner::mapPoseCallback(perception_msgs::msg::EgoData::SharedPtr msg)
{
  ego_data_ = *msg.get();
  ego_pose_ = perception_msgs::object_access::getPoseWithCovariance(ego_data_);
}
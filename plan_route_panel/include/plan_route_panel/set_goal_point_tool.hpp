#pragma once

#include <functional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>

namespace plan_route_panel {

class SetGoalPointTool : public rviz_common::Tool {
public:
  void onInitialize() override;
  void activate() override;
  void deactivate() override {}
  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;
  void setClientName(const std::string& client_name);
  void setGoalSentCallback(std::function<void(bool)> callback);

private:
  rviz_rendering::ViewportProjectionFinder projection_finder_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr public_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr client_publisher_;
  std::function<void(bool)> goal_sent_callback_;
};

}  // namespace plan_route_panel

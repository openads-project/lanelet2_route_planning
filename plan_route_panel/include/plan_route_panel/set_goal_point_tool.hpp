#pragma once

#include <functional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>

namespace rviz_common::properties {
class StringProperty;
}

namespace plan_route_panel {

class SetGoalPointTool : public rviz_common::Tool {
  Q_OBJECT

public:
  SetGoalPointTool();
  void onInitialize() override;
  void activate() override;
  void deactivate() override {}
  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;
  void setGoalSentCallback(std::function<void(bool)> callback);
  std::string clientName() const;
  std::string actionName() const;
  void setMapServerName(const std::string& name);
  void setConfigChangedCallback(std::function<void()> callback);

private Q_SLOTS:
  void updateClientName();
  void updateGoalTopic();
  void updateActionName();

private:
  rviz_rendering::ViewportProjectionFinder projection_finder_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr public_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rviz_common::properties::StringProperty* client_name_property_;
  rviz_common::properties::StringProperty* goal_topic_property_;
  rviz_common::properties::StringProperty* action_name_property_;
  rviz_common::properties::StringProperty* status_topic_property_;
  rviz_common::properties::StringProperty* feedback_topic_property_;
  rviz_common::properties::StringProperty* result_service_property_;
  rviz_common::properties::StringProperty* map_server_property_;
  std::string previous_client_name_ = "/plan_route_action_client";
  std::function<void(bool)> goal_sent_callback_;
  std::function<void()> config_changed_callback_;
};

}  // namespace plan_route_panel

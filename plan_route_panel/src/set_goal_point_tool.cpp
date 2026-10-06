#include "plan_route_panel/set_goal_point_tool.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/render_window.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace plan_route_panel {

void SetGoalPointTool::onInitialize() {
  setName("Set destination");
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  clock_ = node_->get_clock();
  public_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", rclcpp::QoS(10));
}

void SetGoalPointTool::setClientName(const std::string& client_name) {
  client_publisher_.reset();
  if (!client_name.empty()) {
    client_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        client_name + "/goal_pose", rclcpp::QoS(10));
  }
}

void SetGoalPointTool::activate() {
  setStatus("Click once to set a destination.");
}

int SetGoalPointTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
  if (!event.leftDown()) return 0;

  const auto projection = projection_finder_.getViewportPointProjectionOnXYPlane(
      event.panel->getRenderWindow(), event.x, event.y);
  if (!projection.first) return 0;

  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = clock_->now();
  goal.header.frame_id = context_->getFixedFrame().toStdString();
  goal.pose.position.x = projection.second.x;
  goal.pose.position.y = projection.second.y;
  goal.pose.orientation.w = 1.0;
  auto publisher = client_publisher_ ? client_publisher_ : public_publisher_;
  if (publisher->get_subscription_count() == 0 && public_publisher_->get_subscription_count() > 0) {
    publisher = public_publisher_;
  }
  publisher->publish(goal);

  setStatus(QString("Destination sent to %1").arg(QString::fromUtf8(publisher->get_topic_name())));
  return Finished | Render;
}

}  // namespace plan_route_panel

PLUGINLIB_EXPORT_CLASS(plan_route_panel::SetGoalPointTool, rviz_common::Tool)

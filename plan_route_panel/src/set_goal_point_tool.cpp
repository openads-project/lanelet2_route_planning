#include "plan_route_panel/set_goal_point_tool.hpp"

#include <utility>

#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/render_window.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace plan_route_panel {

SetGoalPointTool::SetGoalPointTool() {
  client_name_property_ = new rviz_common::properties::StringProperty(
      "Action Client Node", "/plan_route_action_client", "Node whose parameters control route planning.",
      getPropertyContainer(), SLOT(updateClientName()), this);
  goal_topic_property_ = new rviz_common::properties::StringProperty(
      "Goal Pose Topic", "/plan_route_action_client/goal_pose", "Destination topic; falls back to /goal_pose if only that has a subscriber.",
      getPropertyContainer(), SLOT(updateGoalTopic()), this);
  action_name_property_ = new rviz_common::properties::StringProperty(
      "Status Action", "/planning/lanelet2_route_planning/plan_route",
      "Action whose status and feedback the panel displays; the client's action target is set at startup.",
      getPropertyContainer(), SLOT(updateActionName()), this);
  status_topic_property_ = new rviz_common::properties::StringProperty(
      "Status Topic", "", "Action goal states used by the panel.", getPropertyContainer());
  feedback_topic_property_ = new rviz_common::properties::StringProperty(
      "Feedback Topic", "", "Route distance and time updates used by the panel.", getPropertyContainer());
  result_service_property_ = new rviz_common::properties::StringProperty(
      "Result Service", "", "Final destination and travel result used by the panel.", getPropertyContainer());
  status_topic_property_->setReadOnly(true);
  feedback_topic_property_->setReadOnly(true);
  result_service_property_->setReadOnly(true);
  map_server_property_ = new rviz_common::properties::StringProperty(
      "LL2 Map Server Name", "Not connected", "Read-only client parameter; set it when starting the client.",
      getPropertyContainer());
  map_server_property_->setReadOnly(true);
  updateActionName();
}

void SetGoalPointTool::onInitialize() {
  setName("Set Destination");
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  clock_ = node_->get_clock();
  public_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", rclcpp::QoS(10));
  updateGoalTopic();
}

void SetGoalPointTool::updateClientName() {
  const auto name = clientName();
  if (goal_topic_property_->getStdString() == previous_client_name_ + "/goal_pose") {
    goal_topic_property_->setStdString(name + "/goal_pose");
  }
  previous_client_name_ = name;
  if (config_changed_callback_) config_changed_callback_();
}

void SetGoalPointTool::updateGoalTopic() {
  goal_publisher_.reset();
  if (node_ && !goal_topic_property_->getStdString().empty()) {
    goal_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        goal_topic_property_->getStdString(), rclcpp::QoS(10));
  }
  if (config_changed_callback_) config_changed_callback_();
}

void SetGoalPointTool::updateActionName() {
  const auto action = actionName();
  status_topic_property_->setStdString(action.empty() ? "" : action + "/_action/status");
  feedback_topic_property_->setStdString(action.empty() ? "" : action + "/_action/feedback");
  result_service_property_->setStdString(action.empty() ? "" : action + "/_action/get_result");
  if (config_changed_callback_) config_changed_callback_();
}

std::string SetGoalPointTool::clientName() const {
  auto name = client_name_property_->getString().trimmed().toStdString();
  if (!name.empty() && name.front() != '/') name.insert(name.begin(), '/');
  while (name.size() > 1 && name.back() == '/') name.pop_back();
  return name;
}

std::string SetGoalPointTool::actionName() const {
  auto name = action_name_property_->getString().trimmed().toStdString();
  if (!name.empty() && name.front() != '/') name.insert(name.begin(), '/');
  while (name.size() > 1 && name.back() == '/') name.pop_back();
  return name;
}

void SetGoalPointTool::setMapServerName(const std::string& name) {
  map_server_property_->setStdString(name);
}

void SetGoalPointTool::setGoalSentCallback(std::function<void(bool)> callback) {
  goal_sent_callback_ = std::move(callback);
}

void SetGoalPointTool::setConfigChangedCallback(std::function<void()> callback) {
  config_changed_callback_ = std::move(callback);
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
  auto publisher = goal_publisher_ ? goal_publisher_ : public_publisher_;
  if (!publisher) {
    if (goal_sent_callback_) goal_sent_callback_(false);
    setStatus("Set a goal pose topic in Tool Properties.");
    return Finished;
  }
  if (publisher->get_subscription_count() == 0 && public_publisher_->get_subscription_count() > 0) {
    publisher = public_publisher_;
  }
  if (goal_sent_callback_) goal_sent_callback_(publisher->get_subscription_count() > 0);
  publisher->publish(goal);

  setStatus(QString("Destination sent to %1").arg(QString::fromUtf8(publisher->get_topic_name())));
  return Finished | Render;
}

}  // namespace plan_route_panel

PLUGINLIB_EXPORT_CLASS(plan_route_panel::SetGoalPointTool, rviz_common::Tool)

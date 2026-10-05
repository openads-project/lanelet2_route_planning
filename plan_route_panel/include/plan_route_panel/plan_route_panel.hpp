#pragma once

#include <functional>
#include <deque>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <QWidget>
#include <QString>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <route_planning_msgs/action/plan_route.hpp>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QProgressBar;
class QPushButton;

namespace plan_route_panel {

struct CallbackBridge;

class PlanRoutePanel : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit PlanRoutePanel(QWidget* parent = nullptr);
  ~PlanRoutePanel() override;
  void onInitialize() override;

private:
  void connectToClient();
  void refreshParameters();
  void sendParameters(const std::vector<rclcpp::Parameter>& parameters,
                      std::function<void(bool, const QString&)> done);
  void planRoute();
  void cancelRoute();
  void loadPresets();
  void showStatus(const QString& status, const QString& detail = {});
  void updateGoalStatus(const action_msgs::msg::GoalStatusArray& msg);
  void updateFeedback(const std::string& goal_id, const route_planning_msgs::action::PlanRoute::Feedback& feedback);
  bool rememberGoal(const std::string& goal_id);
  void requestResult(const std::string& goal_id);
  std::string clientName() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr set_client_;
  rclcpp::Client<route_planning_msgs::action::PlanRoute::Impl::GetResultService>::SharedPtr result_client_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::Subscription<route_planning_msgs::action::PlanRoute::Impl::FeedbackMessage>::SharedPtr feedback_sub_;
  std::shared_ptr<CallbackBridge> callback_bridge_;

  QLineEdit* client_name_;
  QCheckBox* random_destination_;
  QCheckBox* continuous_planning_;
  QDoubleSpinBox* replanning_proportion_;
  QComboBox* presets_;
  QLabel* status_;
  QLabel* detail_;
  QProgressBar* progress_;
  QPushButton* plan_button_;
  QPushButton* cancel_button_;

  std::set<std::string> seen_goal_ids_;
  std::deque<std::string> seen_goal_order_;
  std::string tracked_goal_id_;
  std::string result_goal_id_;
  std::string failed_destination_goal_id_;
  bool awaiting_goal_ = false;
  bool cancel_requested_ = false;
  bool parameter_update_pending_ = false;
  bool continuous_run_ = false;
  int remaining_goals_ = 0;
  std::string connected_client_name_;
};

}  // namespace plan_route_panel

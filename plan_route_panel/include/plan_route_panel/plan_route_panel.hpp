// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <functional>
#include <deque>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <QWidget>
#include <QPointer>
#include <QString>
#include <rviz_common/panel.hpp>
#include <rviz_common/tool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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

/**
 * @brief RViz panel that configures the running route action client and observes its goals.
 *
 * Saved routes are sent as client parameters. Clicked destinations use RViz's
 * built-in Goal Pose tool and the client's goal pose subscription.
 */
class PlanRoutePanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the panel controls and loads saved route names
   *
   * @param[in] parent parent widget
   */
  explicit PlanRoutePanel(QWidget* parent = nullptr);

  /**
   * @brief Invalidates callbacks queued from ROS threads
   */
  ~PlanRoutePanel() override;

  /**
   * @brief Connects to the RViz ROS node and the configured client
   */
  void onInitialize() override;

  /**
   * @brief Restores endpoint settings from the RViz configuration
   *
   * @param[in] config RViz configuration
   */
  void load(const rviz_common::Config& config) override;

  /**
   * @brief Saves endpoint settings to the RViz configuration
   *
   * @param[in,out] config RViz configuration
   */
  void save(rviz_common::Config config) const override;

 private:
  /**
   * @brief Creates parameter service clients for the configured node
   */
  void connectToClient();

  /**
   * @brief Subscribes to action status and feedback and creates a result client
   */
  void connectToAction();

  /**
   * @brief Finds or adds RViz's Goal Pose tool
   *
   * @return Goal Pose tool, or nullptr if the tool manager is unavailable
   */
  rviz_common::Tool* ensureGoalTool();

  /**
   * @brief Applies the configured goal pose topic to the RViz tool
   */
  void updateGoalTopic();

  /**
   * @brief Reads the current planning mode from the client
   */
  void refreshParameters();

  /**
   * @brief Sends an atomic parameter update and reports completion on the Qt thread
   *
   * @param[in] parameters parameters to send to the client
   * @param[in] done callback receiving success and an optional error message
   */
  void sendParameters(const std::vector<rclcpp::Parameter>& parameters,
                      std::function<void(bool, const QString&)> done);

  /**
   * @brief Starts a saved or random route through the existing client
   */
  void planRoute();

  /**
   * @brief Disables automatic planning and requests cancellation
   */
  void cancelRoute();

  /**
   * @brief Loads route names from the installed routes.yml file
   */
  void loadPresets();

  /**
   * @brief Enables the primary button when its current mode is usable
   */
  void updatePlanButton();

  /**
   * @brief Updates the status text and progress indicator
   *
   * @param[in] status short status label
   * @param[in] detail optional detail text
   */
  void showStatus(const QString& status, const QString& detail = {});

  /**
   * @brief Tracks action state changes for the current goal
   *
   * @param[in] msg action goal statuses
   */
  void updateGoalStatus(const action_msgs::msg::GoalStatusArray& msg);

  /**
   * @brief Displays feedback for the current goal
   *
   * @param[in] goal_id action goal identifier
   * @param[in] feedback action feedback
   */
  void updateFeedback(const std::string& goal_id, const route_planning_msgs::action::PlanRoute::Feedback& feedback);

  /**
   * @brief Records a goal ID once, with bounded history
   *
   * @param[in] goal_id action goal identifier
   * @return true if the goal ID was not seen before
   */
  bool rememberGoal(const std::string& goal_id);

  /**
   * @brief Requests the terminal result of a tracked goal
   *
   * @param[in] goal_id action goal identifier
   */
  void requestResult(const std::string& goal_id);

  /**
   * @brief Returns the normalized client node name
   *
   * @return absolute client node name, or an empty string if unset
   */
  std::string clientName() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr set_client_;
  rclcpp::Client<route_planning_msgs::action::PlanRoute::Impl::GetResultService>::SharedPtr result_client_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::Subscription<route_planning_msgs::action::PlanRoute::Impl::FeedbackMessage>::SharedPtr feedback_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  std::shared_ptr<CallbackBridge> callback_bridge_;
  QPointer<rviz_common::Tool> goal_tool_;

  QComboBox* mode_;
  QCheckBox* continuous_planning_;
  QDoubleSpinBox* replanning_proportion_;
  QComboBox* presets_;
  QLabel* status_;
  QLabel* detail_;
  QProgressBar* progress_;
  QPushButton* plan_button_;
  QPushButton* cancel_button_;
  QLineEdit* map_server_name_;
  QLineEdit* client_name_;
  QLineEdit* goal_topic_;
  QLineEdit* action_name_;
  bool selecting_destination_ = false;

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
  std::string connected_action_name_;
  bool client_ready_ = false;
};

}  // namespace plan_route_panel

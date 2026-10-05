#include "plan_route_panel/plan_route_panel.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <future>
#include <iomanip>
#include <locale>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QTableWidget>
#include <QVBoxLayout>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <yaml-cpp/yaml.h>

namespace plan_route_panel {

struct CallbackBridge {
  std::mutex mutex;
  PlanRoutePanel* panel = nullptr;
};

namespace {

constexpr char kActionName[] = "/planning/lanelet2_route_planning/plan_route";

struct Waypoint {
  double latitude;
  double longitude;
  double wait_time_s;
};

template <typename F>
void postToPanel(const std::weak_ptr<CallbackBridge>& weak_bridge, F&& callback) {
  auto bridge = weak_bridge.lock();
  if (!bridge) return;
  std::lock_guard<std::mutex> lock(bridge->mutex);
  if (!bridge->panel) return;
  QMetaObject::invokeMethod(bridge->panel, [weak_bridge, callback = std::forward<F>(callback)]() mutable {
    auto current = weak_bridge.lock();
    if (!current) return;
    std::lock_guard<std::mutex> guard(current->mutex);
    if (current->panel) callback(current->panel);
  }, Qt::QueuedConnection);
}

std::string goalId(const std::array<uint8_t, 16>& uuid) {
  return std::string(reinterpret_cast<const char*>(uuid.data()), uuid.size());
}

QDoubleSpinBox* coordinateSpin(QWidget* parent, double min, double max, double value) {
  auto* spin = new QDoubleSpinBox(parent);
  spin->setRange(min, max);
  spin->setDecimals(9);
  spin->setSingleStep(0.00001);
  spin->setValue(value);
  return spin;
}

QDoubleSpinBox* cellSpin(QTableWidget* table, int row, int column) {
  return qobject_cast<QDoubleSpinBox*>(table->cellWidget(row, column));
}

Waypoint rowValue(QTableWidget* table, int row) {
  return {cellSpin(table, row, 0)->value(), cellSpin(table, row, 1)->value(), cellSpin(table, row, 2)->value()};
}

void setRowValue(QTableWidget* table, int row, const Waypoint& waypoint) {
  cellSpin(table, row, 0)->setValue(waypoint.latitude);
  cellSpin(table, row, 1)->setValue(waypoint.longitude);
  cellSpin(table, row, 2)->setValue(waypoint.wait_time_s);
}

bool parseWaypoint(const std::string& value, Waypoint& waypoint) {
  const auto fields = QString::fromStdString(value).split(',');
  if (fields.size() < 2 || fields.size() > 3) return false;
  bool lat_ok = false, lon_ok = false, wait_ok = true;
  waypoint.latitude = fields[0].trimmed().toDouble(&lat_ok);
  waypoint.longitude = fields[1].trimmed().toDouble(&lon_ok);
  waypoint.wait_time_s = fields.size() == 3 ? fields[2].trimmed().toDouble(&wait_ok) : 0.0;
  return lat_ok && lon_ok && wait_ok && std::isfinite(waypoint.latitude) && std::isfinite(waypoint.longitude) &&
         std::isfinite(waypoint.wait_time_s) && std::abs(waypoint.latitude) <= 90.0 &&
         std::abs(waypoint.longitude) <= 180.0;
}

std::string serializeWaypoint(const Waypoint& waypoint) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(12) << waypoint.latitude << ',' << waypoint.longitude << ',' << waypoint.wait_time_s;
  return out.str();
}

}  // namespace

PlanRoutePanel::PlanRoutePanel(QWidget* parent) : rviz_common::Panel(parent), callback_bridge_(std::make_shared<CallbackBridge>()) {
  callback_bridge_->panel = this;
  auto* outer = new QVBoxLayout(this);
  auto* scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  auto* content = new QWidget(scroll);
  auto* layout = new QVBoxLayout(content);
  scroll->setWidget(content);
  outer->addWidget(scroll);

  auto* parameters = new QGroupBox(tr("Route Parameters"), content);
  auto* form = new QFormLayout(parameters);
  client_name_ = new QLineEdit("/plan_route_action_client", parameters);
  auto* refresh = new QPushButton(tr("Refresh"), parameters);
  auto* client_row = new QHBoxLayout;
  client_row->addWidget(client_name_);
  client_row->addWidget(refresh);
  form->addRow(tr("Action client node"), client_row);
  map_server_name_ = new QLineEdit("ll2_map_server", parameters);
  map_server_name_->setToolTip(tr("The running client only reads this setting at startup. Restart it after changing the name."));
  form->addRow(tr("Map server name"), map_server_name_);
  form->addRow(new QLabel(tr("Map server changes require an action client restart."), parameters));
  random_destination_ = new QCheckBox(tr("Random destination"), parameters);
  continuous_planning_ = new QCheckBox(tr("Continuous planning"), parameters);
  form->addRow(random_destination_);
  form->addRow(continuous_planning_);
  replanning_proportion_ = new QDoubleSpinBox(parameters);
  replanning_proportion_->setRange(0.0, 1.0);
  replanning_proportion_->setSingleStep(0.01);
  replanning_proportion_->setDecimals(2);
  replanning_proportion_->setValue(0.6);
  form->addRow(tr("Replan after fraction"), replanning_proportion_);
  layout->addWidget(parameters);

  auto* waypoint_group = new QGroupBox(tr("Waypoints (WGS84, in route order)"), content);
  auto* waypoint_layout = new QVBoxLayout(waypoint_group);
  waypoints_ = new QTableWidget(0, 3, waypoint_group);
  waypoints_->setHorizontalHeaderLabels({tr("Latitude"), tr("Longitude"), tr("Wait (s)")});
  waypoints_->setSelectionBehavior(QAbstractItemView::SelectRows);
  waypoints_->setSelectionMode(QAbstractItemView::SingleSelection);
  waypoint_layout->addWidget(waypoints_);
  waypoint_layout->addWidget(new QLabel(tr("Negative wait = intermediate destination; last waypoint must be a stop."), waypoint_group));
  auto* waypoint_buttons = new QHBoxLayout;
  auto* add = new QPushButton(tr("Add"), waypoint_group);
  auto* remove = new QPushButton(tr("Remove"), waypoint_group);
  auto* up = new QPushButton(tr("Up"), waypoint_group);
  auto* down = new QPushButton(tr("Down"), waypoint_group);
  for (auto* button : {add, remove, up, down}) waypoint_buttons->addWidget(button);
  waypoint_layout->addLayout(waypoint_buttons);
  layout->addWidget(waypoint_group);

  auto* presets_group = new QGroupBox(tr("Saved Routes"), content);
  auto* presets_layout = new QHBoxLayout(presets_group);
  presets_ = new QComboBox(presets_group);
  auto* reload = new QPushButton(tr("Reload"), presets_group);
  auto* apply = new QPushButton(tr("Use Route"), presets_group);
  presets_layout->addWidget(presets_, 1);
  presets_layout->addWidget(reload);
  presets_layout->addWidget(apply);
  layout->addWidget(presets_group);

  auto* action_buttons = new QHBoxLayout;
  plan_button_ = new QPushButton(tr("Plan Route"), content);
  cancel_button_ = new QPushButton(tr("Cancel"), content);
  plan_button_->setEnabled(false);
  cancel_button_->setEnabled(false);
  action_buttons->addWidget(plan_button_);
  action_buttons->addWidget(cancel_button_);
  layout->addLayout(action_buttons);
  status_ = new QLabel(tr("Status: Idle"), content);
  detail_ = new QLabel(content);
  detail_->setWordWrap(true);
  progress_ = new QProgressBar(content);
  progress_->setRange(0, 1);
  progress_->setValue(0);
  progress_->setTextVisible(false);
  layout->addWidget(status_);
  layout->addWidget(detail_);
  layout->addWidget(progress_);
  layout->addStretch();

  connect(refresh, &QPushButton::clicked, this, [this] { connectToClient(); refreshParameters(); });
  connect(add, &QPushButton::clicked, this, [this] { addWaypoint(); });
  connect(remove, &QPushButton::clicked, this, [this] {
    if (waypoints_->currentRow() >= 0) waypoints_->removeRow(waypoints_->currentRow());
  });
  connect(up, &QPushButton::clicked, this, [this] {
    const int row = waypoints_->currentRow();
    if (row <= 0) return;
    const auto a = rowValue(waypoints_, row);
    setRowValue(waypoints_, row, rowValue(waypoints_, row - 1));
    setRowValue(waypoints_, row - 1, a);
    waypoints_->selectRow(row - 1);
  });
  connect(down, &QPushButton::clicked, this, [this] {
    const int row = waypoints_->currentRow();
    if (row < 0 || row >= waypoints_->rowCount() - 1) return;
    const auto a = rowValue(waypoints_, row);
    setRowValue(waypoints_, row, rowValue(waypoints_, row + 1));
    setRowValue(waypoints_, row + 1, a);
    waypoints_->selectRow(row + 1);
  });
  connect(reload, &QPushButton::clicked, this, [this] { loadPresets(); });
  connect(apply, &QPushButton::clicked, this, [this] { applyPreset(); });
  connect(plan_button_, &QPushButton::clicked, this, [this] { planRoute(); });
  connect(cancel_button_, &QPushButton::clicked, this, [this] { cancelRoute(); });
  loadPresets();
}

PlanRoutePanel::~PlanRoutePanel() {
  std::lock_guard<std::mutex> lock(callback_bridge_->mutex);
  callback_bridge_->panel = nullptr;
}

void PlanRoutePanel::onInitialize() {
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    showStatus(tr("Failed"), tr("RViz ROS node is unavailable."));
    return;
  }
  node_ = abstraction->get_raw_node();
  const std::weak_ptr<CallbackBridge> weak_bridge = callback_bridge_;
  status_sub_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      std::string(kActionName) + "/_action/status", rclcpp::QoS(10).reliable().transient_local(),
      [weak_bridge](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
        postToPanel(weak_bridge, [msg](PlanRoutePanel* panel) { panel->updateGoalStatus(*msg); });
      });
  feedback_sub_ = node_->create_subscription<route_planning_msgs::action::PlanRoute::Impl::FeedbackMessage>(
      std::string(kActionName) + "/_action/feedback", rclcpp::QoS(10),
      [weak_bridge](route_planning_msgs::action::PlanRoute::Impl::FeedbackMessage::ConstSharedPtr msg) {
        const auto id = goalId(msg->goal_id.uuid);
        const auto feedback = msg->feedback;
        postToPanel(weak_bridge, [id, feedback](PlanRoutePanel* panel) { panel->updateFeedback(id, feedback); });
      });
  result_client_ = node_->create_client<route_planning_msgs::action::PlanRoute::Impl::GetResultService>(
      std::string(kActionName) + "/_action/get_result");
  connectToClient();
  refreshParameters();
  plan_button_->setEnabled(true);
  cancel_button_->setEnabled(true);
}

std::string PlanRoutePanel::clientName() const {
  auto name = client_name_->text().trimmed().toStdString();
  if (name.empty()) return {};
  if (name.front() != '/') name.insert(name.begin(), '/');
  while (name.size() > 1 && name.back() == '/') name.pop_back();
  return name;
}

void PlanRoutePanel::connectToClient() {
  if (!node_) return;
  const auto name = clientName();
  if (name.empty()) {
    get_client_.reset();
    set_client_.reset();
    connected_client_name_.clear();
    showStatus(tr("Failed"), tr("Enter the action client node name."));
    return;
  }
  if (name == connected_client_name_ && get_client_ && set_client_) return;
  get_client_ = node_->create_client<rcl_interfaces::srv::GetParameters>(name + "/get_parameters");
  set_client_ = node_->create_client<rcl_interfaces::srv::SetParametersAtomically>(name + "/set_parameters_atomically");
  connected_client_name_ = name;
}

void PlanRoutePanel::refreshParameters() {
  if (!get_client_ || !get_client_->service_is_ready()) {
    showStatus(tr("Idle"), tr("Action client parameter service is not available. Press Refresh when it starts."));
    return;
  }
  const auto requested_client = clientName();
  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = {"ll2_map_server_name", "waypoints", "enable_random_destination", "enable_continuous_planning",
                    "continuous_planning_replanning_proportion"};
  const std::weak_ptr<CallbackBridge> weak_bridge = callback_bridge_;
  get_client_->async_send_request(
      request, [weak_bridge, requested_client](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) {
    try {
      const auto values = future.get()->values;
      postToPanel(weak_bridge, [values, requested_client](PlanRoutePanel* panel) {
        if (panel->clientName() != requested_client || values.size() != 5) return;
        if (values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
          panel->loaded_map_server_name_ = values[0].string_value;
          panel->map_server_name_->setText(QString::fromStdString(values[0].string_value));
        }
        if (values[1].type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY) {
          std::vector<Waypoint> parsed;
          for (const auto& value : values[1].string_array_value) {
            Waypoint waypoint;
            if (!parseWaypoint(value, waypoint)) {
              panel->showStatus(QObject::tr("Failed"), QObject::tr("Client contains an invalid waypoint."));
              return;
            }
            parsed.push_back(waypoint);
          }
          panel->waypoints_->setRowCount(0);
          for (const auto& waypoint : parsed) panel->addWaypoint(waypoint.latitude, waypoint.longitude, waypoint.wait_time_s);
        }
        if (values[2].type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
          panel->random_destination_->setChecked(values[2].bool_value);
        }
        if (values[3].type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
          panel->continuous_planning_->setChecked(values[3].bool_value);
        }
        if (values[4].type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
          panel->replanning_proportion_->setValue(values[4].double_value);
        }
        if (!panel->awaiting_goal_ && panel->tracked_goal_id_.empty()) {
          panel->showStatus(QObject::tr("Idle"), QObject::tr("Loaded parameters from action client."));
        }
      });
    } catch (const std::exception& e) {
      const QString error = QString::fromUtf8(e.what());
      postToPanel(weak_bridge, [error](PlanRoutePanel* panel) { panel->showStatus(QObject::tr("Failed"), error); });
    }
  });
}

void PlanRoutePanel::sendParameters(const std::vector<rclcpp::Parameter>& parameters,
                                    std::function<void(bool, const QString&)> done) {
  if (!set_client_ || !set_client_->service_is_ready()) {
    done(false, tr("Action client parameter service is unavailable."));
    return;
  }
  auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
  for (const auto& parameter : parameters) request->parameters.push_back(parameter.to_parameter_msg());
  const std::weak_ptr<CallbackBridge> weak_bridge = callback_bridge_;
  set_client_->async_send_request(
      request, [weak_bridge, done = std::move(done)](
                   rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture future) {
    try {
      const auto response = future.get();
      const bool success = response->result.successful;
      const QString reason = QString::fromStdString(response->result.reason);
      postToPanel(weak_bridge, [done, success, reason](PlanRoutePanel*) { done(success, reason); });
    } catch (const std::exception& e) {
      const QString error = QString::fromUtf8(e.what());
      postToPanel(weak_bridge, [done, error](PlanRoutePanel*) { done(false, error); });
    }
  });
}

void PlanRoutePanel::planRoute() {
  connectToClient();
  if (!random_destination_->isChecked() && waypoints_->rowCount() == 0) {
    showStatus(tr("Failed"), tr("Add at least one waypoint or enable random destination."));
    return;
  }
  if (!random_destination_->isChecked() && rowValue(waypoints_, waypoints_->rowCount() - 1).wait_time_s < 0.0) {
    showStatus(tr("Failed"), tr("The final waypoint must have a nonnegative wait time."));
    return;
  }
  std::vector<std::string> waypoints;
  remaining_goals_ = random_destination_->isChecked() ? 1 : 0;
  for (int row = 0; row < waypoints_->rowCount(); ++row) {
    const auto waypoint = rowValue(waypoints_, row);
    waypoints.push_back(serializeWaypoint(waypoint));
    if (!random_destination_->isChecked() && waypoint.wait_time_s >= 0.0) ++remaining_goals_;
  }
  const auto map_name = map_server_name_->text().trimmed().toStdString();
  if (map_name.empty()) {
    showStatus(tr("Failed"), tr("Enter a map server name."));
    return;
  }
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("cancel_route", false);
  parameters.emplace_back("ll2_map_server_name", map_name);
  parameters.emplace_back("enable_random_destination", random_destination_->isChecked());
  parameters.emplace_back("enable_continuous_planning", continuous_planning_->isChecked());
  parameters.emplace_back("continuous_planning_replanning_proportion", replanning_proportion_->value());
  parameters.emplace_back("waypoints", waypoints);
  awaiting_goal_ = true;
  cancel_requested_ = false;
  continuous_run_ = continuous_planning_->isChecked();
  tracked_goal_id_.clear();
  result_goal_id_.clear();
  failed_destination_goal_id_.clear();
  showStatus(tr("Sending"), tr("Updating the existing action client; waiting for its goal."));
  progress_->setRange(0, 0);
  plan_button_->setEnabled(false);
  sendParameters(parameters, [this, map_name](bool success, const QString& reason) {
    plan_button_->setEnabled(true);
    if (!success) {
      awaiting_goal_ = false;
      tracked_goal_id_.clear();
      progress_->setRange(0, 1);
      progress_->setValue(0);
      showStatus(tr("Failed"), reason.isEmpty() ? tr("Parameter update rejected.") : reason);
      return;
    }
    if (map_name != loaded_map_server_name_) {
      showStatus(tracked_goal_id_.empty() ? tr("Sending") : tr("Running"),
                 tr("Parameters updated. Restart the action client to apply the new map server name."));
    }
  });
}

void PlanRoutePanel::cancelRoute() {
  connectToClient();
  cancel_requested_ = true;
  awaiting_goal_ = false;
  continuous_run_ = false;
  remaining_goals_ = 0;
  // Disable automatic replanning before asking the client to cancel its active goal.
  sendParameters({rclcpp::Parameter("enable_random_destination", false),
                  rclcpp::Parameter("enable_continuous_planning", false),
                  rclcpp::Parameter("waypoints", std::vector<std::string>{}),
                  rclcpp::Parameter("cancel_route", true)},
                 [this](bool success, const QString& reason) {
                   if (!success) {
                     cancel_requested_ = false;
                     showStatus(tr("Failed"), reason.isEmpty() ? tr("Cancel request rejected.") : reason);
                     return;
                   }
                   random_destination_->setChecked(false);
                   continuous_planning_->setChecked(false);
                   showStatus(tr("Canceling"), tr("Cancel requested from the action client."));
                   sendParameters({rclcpp::Parameter("cancel_route", false)}, [](bool, const QString&) {});
                 });
}

void PlanRoutePanel::addWaypoint(double latitude, double longitude, double wait_time_s) {
  const int row = waypoints_->rowCount();
  waypoints_->insertRow(row);
  waypoints_->setCellWidget(row, 0, coordinateSpin(waypoints_, -90.0, 90.0, latitude));
  waypoints_->setCellWidget(row, 1, coordinateSpin(waypoints_, -180.0, 180.0, longitude));
  auto* wait = coordinateSpin(waypoints_, -1000000.0, 1000000.0, wait_time_s);
  wait->setDecimals(3);
  wait->setSingleStep(1.0);
  waypoints_->setCellWidget(row, 2, wait);
  waypoints_->selectRow(row);
}

void PlanRoutePanel::loadPresets() {
  presets_->clear();
  try {
    const auto path = ament_index_cpp::get_package_share_directory("plan_route_panel") + "/config/routes.yml";
    const auto root = YAML::LoadFile(path);
    const auto routes = root["routes"];
    if (!routes || !routes.IsMap()) throw std::runtime_error("'routes' must be a map");
    for (const auto& route : routes) presets_->addItem(QString::fromStdString(route.first.as<std::string>()));
    if (presets_->count() == 0) presets_->addItem(tr("No saved routes"));
  } catch (const std::exception& e) {
    presets_->addItem(tr("Could not load routes"));
    showStatus(tr("Failed"), QString::fromUtf8(e.what()));
  }
}

void PlanRoutePanel::applyPreset() {
  try {
    const auto path = ament_index_cpp::get_package_share_directory("plan_route_panel") + "/config/routes.yml";
    const auto root = YAML::LoadFile(path);
    const auto route = root["routes"][presets_->currentText().toStdString()];
    if (!route || !route.IsSequence()) throw std::runtime_error("Selected route is not a waypoint list");
    std::vector<Waypoint> parsed;
    for (const auto& entry : route) {
      Waypoint waypoint;
      if (entry.IsScalar()) {
        if (!parseWaypoint(entry.as<std::string>(), waypoint)) {
          throw std::runtime_error("Route contains an invalid waypoint string");
        }
      } else if (entry.IsMap()) {
        waypoint = {entry["latitude"].as<double>(), entry["longitude"].as<double>(),
                    entry["wait_time_s"] ? entry["wait_time_s"].as<double>() : 0.0};
      } else {
        throw std::runtime_error("Route waypoints must be strings or coordinate maps");
      }
      if (!std::isfinite(waypoint.latitude) || !std::isfinite(waypoint.longitude) || !std::isfinite(waypoint.wait_time_s) ||
          std::abs(waypoint.latitude) > 90.0 || std::abs(waypoint.longitude) > 180.0) {
        throw std::runtime_error("Route contains an invalid waypoint");
      }
      parsed.push_back(waypoint);
    }
    if (parsed.empty()) throw std::runtime_error("Selected route is empty");
    if (parsed.back().wait_time_s < 0.0) throw std::runtime_error("Last waypoint must be a stop");
    waypoints_->setRowCount(0);
    for (const auto& waypoint : parsed) addWaypoint(waypoint.latitude, waypoint.longitude, waypoint.wait_time_s);
    showStatus(tr("Idle"), tr("Saved route loaded into the waypoint table."));
  } catch (const std::exception& e) {
    showStatus(tr("Failed"), QString::fromUtf8(e.what()));
  }
}

void PlanRoutePanel::showStatus(const QString& status, const QString& detail) {
  status_->setText(tr("Status: %1").arg(status));
  detail_->setText(detail);
}

bool PlanRoutePanel::rememberGoal(const std::string& goal_id) {
  if (!seen_goal_ids_.insert(goal_id).second) return false;
  seen_goal_order_.push_back(goal_id);
  if (seen_goal_order_.size() > 1024) {
    seen_goal_ids_.erase(seen_goal_order_.front());
    seen_goal_order_.pop_front();
  }
  return true;
}

void PlanRoutePanel::requestResult(const std::string& goal_id) {
  result_goal_id_ = goal_id;
  if (!result_client_ || !result_client_->service_is_ready()) return;
  auto request = std::make_shared<route_planning_msgs::action::PlanRoute::Impl::GetResultService::Request>();
  std::copy(goal_id.begin(), goal_id.end(), request->goal_id.uuid.begin());
  const std::weak_ptr<CallbackBridge> weak_bridge = callback_bridge_;
  result_client_->async_send_request(
      request, [weak_bridge, goal_id](
                   rclcpp::Client<route_planning_msgs::action::PlanRoute::Impl::GetResultService>::SharedFuture future) {
    try {
      const auto response = future.get();
      postToPanel(weak_bridge, [goal_id, response](PlanRoutePanel* panel) {
        if (panel->result_goal_id_ != goal_id) return;
        const auto& result = response->result;
        const auto detail = QObject::tr("Traveled %1 m in %2 s.")
                                .arg(result.distance_traveled, 0, 'f', 1)
                                .arg(result.time_traveled.sec + result.time_traveled.nanosec * 1e-9, 0, 'f', 1);
        if (response->status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
          if (!result.destination_reached) {
            panel->failed_destination_goal_id_ = goal_id;
            panel->progress_->setRange(0, 1);
            panel->progress_->setValue(0);
          }
          panel->showStatus(result.destination_reached ? QObject::tr("Success") : QObject::tr("Failed"),
                            (result.destination_reached ? QObject::tr("Destination reached. ")
                                                        : QObject::tr("Destination not reached. ")) + detail);
        } else if (response->status == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
          panel->showStatus(QObject::tr("Aborted"), detail);
        } else if (response->status == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
          panel->showStatus(QObject::tr("Canceled"), detail);
        }
      });
    } catch (const std::exception&) {
      // The status subscription still reports terminal action states.
    }
  });
}

void PlanRoutePanel::updateGoalStatus(const action_msgs::msg::GoalStatusArray& msg) {
  for (const auto& goal : msg.status_list) {
    const auto id = goalId(goal.goal_info.goal_id.uuid);
    const bool is_new = rememberGoal(id);
    if (awaiting_goal_ && tracked_goal_id_.empty() && is_new &&
        goal.status != action_msgs::msg::GoalStatus::STATUS_UNKNOWN) {
      tracked_goal_id_ = id;
      awaiting_goal_ = false;
      requestResult(id);
    }
    if (id != tracked_goal_id_) continue;
    switch (goal.status) {
      case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
      case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
        if (!cancel_requested_) showStatus(tr("Running"), detail_->text());
        break;
      case action_msgs::msg::GoalStatus::STATUS_CANCELING:
        showStatus(tr("Canceling"), detail_->text());
        break;
      case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
        if (remaining_goals_ > 0) --remaining_goals_;
        if (continuous_run_ || remaining_goals_ > 0) {
          awaiting_goal_ = true;
          progress_->setRange(0, 0);
          showStatus(tr("Sending"), tr("Waiting for the client's next route goal."));
        } else {
          progress_->setRange(0, 1);
          progress_->setValue(id == failed_destination_goal_id_ ? 0 : 1);
          showStatus(id == failed_destination_goal_id_ ? tr("Failed") : tr("Success"), detail_->text());
        }
        tracked_goal_id_.clear();
        break;
      case action_msgs::msg::GoalStatus::STATUS_ABORTED:
        if (remaining_goals_ > 0) --remaining_goals_;
        awaiting_goal_ = continuous_run_ || remaining_goals_ > 0;
        progress_->setRange(0, 1);
        progress_->setValue(0);
        showStatus(tr("Aborted"), detail_->text());
        tracked_goal_id_.clear();
        break;
      case action_msgs::msg::GoalStatus::STATUS_CANCELED:
        awaiting_goal_ = false;
        continuous_run_ = false;
        remaining_goals_ = 0;
        progress_->setRange(0, 1);
        progress_->setValue(0);
        showStatus(tr("Canceled"), detail_->text());
        tracked_goal_id_.clear();
        cancel_requested_ = false;
        break;
      default:
        break;
    }
  }
}

void PlanRoutePanel::updateFeedback(const std::string& goal_id,
                                    const route_planning_msgs::action::PlanRoute::Feedback& feedback) {
  if (tracked_goal_id_.empty() && awaiting_goal_ && !seen_goal_ids_.count(goal_id)) {
    tracked_goal_id_ = goal_id;
    awaiting_goal_ = false;
    rememberGoal(goal_id);
    requestResult(goal_id);
  }
  if (tracked_goal_id_ != goal_id) return;
  const double remaining_s = feedback.time_remaining.sec + feedback.time_remaining.nanosec * 1e-9;
  showStatus(cancel_requested_ ? tr("Canceling") : tr("Running"),
             tr("Distance: %1 m traveled, %2 m remaining · Time remaining: %3 s")
                 .arg(feedback.distance_traveled, 0, 'f', 1)
                 .arg(feedback.distance_remaining, 0, 'f', 1)
                 .arg(remaining_s, 0, 'f', 1));
}

}  // namespace plan_route_panel

PLUGINLIB_EXPORT_CLASS(plan_route_panel::PlanRoutePanel, rviz_common::Panel)

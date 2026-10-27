// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include "plan_route_panel/plan_route_panel.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <utility>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/tool_manager.hpp>
#include <yaml-cpp/yaml.h>

namespace plan_route_panel {

struct CallbackBridge {
  std::mutex mutex;
  PlanRoutePanel* panel = nullptr;
};

namespace {

constexpr char kActionName[] = "/planning/lanelet2_route_planning/plan_route";
constexpr int kWaypointsMode = 0;
constexpr int kRandomMode = 1;
constexpr int kDestinationMode = 2;

struct Waypoint {
  double latitude;
  double longitude;
  double wait_time_s;
};

template <typename F>
void postToPanel(const std::weak_ptr<CallbackBridge>& weak_bridge, F&& callback) {
  // ROS callbacks must only touch widgets on the Qt thread, and never after panel destruction.
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

bool parseWaypoint(const std::string& value, Waypoint& waypoint) {
  const auto fields = QString::fromStdString(value).split(',');
  if (fields.size() < 2 || fields.size() > 3) return false;
  bool lat_ok = false, lon_ok = false, wait_ok = true;
  waypoint.latitude = fields[0].trimmed().toDouble(&lat_ok);
  waypoint.longitude = fields[1].trimmed().toDouble(&lon_ok);
  waypoint.wait_time_s = fields.size() == 3 ? fields[2].trimmed().toDouble(&wait_ok) : 0.0;
  // Coordinate ranges and finiteness are checked for both YAML formats in readRoute().
  return lat_ok && lon_ok && wait_ok;
}

std::string routesFilePath() {
  return ament_index_cpp::get_package_share_directory("plan_route_panel") + "/config/routes.yml";
}

std::vector<Waypoint> readRoute(const std::string& name) {
  const auto root = YAML::LoadFile(routesFilePath());
  const auto routes = root["routes"];
  if (!routes || !routes.IsMap()) throw std::runtime_error("'routes' must be a map");
  const auto route = routes[name];
  if (!route || !route.IsSequence()) throw std::runtime_error("Selected route is not a waypoint list");

  std::vector<Waypoint> waypoints;
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
        std::abs(waypoint.latitude) > 90.0 || std::abs(waypoint.longitude) > 180.0 ||
        (waypoint.wait_time_s < 0.0 && waypoint.wait_time_s != -1.0) ||
        !std::isfinite(static_cast<float>(waypoint.wait_time_s))) {
      throw std::runtime_error("Route contains an invalid waypoint");
    }
    waypoints.push_back(waypoint);
  }
  if (waypoints.empty()) throw std::runtime_error("Selected route is empty");
  if (waypoints.back().wait_time_s < 0.0) throw std::runtime_error("Last waypoint must be a stop");
  return waypoints;
}

}  // namespace

PlanRoutePanel::PlanRoutePanel(QWidget* parent) : rviz_common::Panel(parent), callback_bridge_(std::make_shared<CallbackBridge>()) {
  callback_bridge_->panel = this;
  auto* outer = new QVBoxLayout(this);
  auto* scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  auto* content = new QWidget(scroll);
  auto* layout = new QVBoxLayout(content);
  scroll->setWidget(content);
  outer->addWidget(scroll, 1);

  auto* parameters = new QWidget(content);
  auto* mode_row = new QVBoxLayout(parameters);
  mode_row->setContentsMargins(0, 0, 0, 0);
  mode_row->setSpacing(4);
  mode_ = new QComboBox(parameters);
  mode_->addItems({tr("Waypoints"), tr("Random Destination"), tr("Destination (Click)")});
  mode_row->addWidget(mode_);
  auto* waypoint_options = new QWidget(parameters);
  auto* form = new QFormLayout(waypoint_options);
  form->setContentsMargins(22, 0, 0, 2);
  form->setVerticalSpacing(4);
  continuous_planning_ = new QCheckBox(tr("Continuous planning"), waypoint_options);
  form->addRow(continuous_planning_);
  replanning_proportion_ = new QDoubleSpinBox(waypoint_options);
  replanning_proportion_->setRange(0.0, 1.0);
  replanning_proportion_->setSingleStep(0.01);
  replanning_proportion_->setDecimals(2);
  replanning_proportion_->setValue(0.6);
  form->addRow(tr("Replan after fraction"), replanning_proportion_);
  auto* replanning_label = form->labelForField(replanning_proportion_);
  presets_ = new QComboBox(waypoint_options);
  auto* saved_routes_label = new QLabel(tr("Saved Routes"), waypoint_options);
  form->addRow(saved_routes_label, presets_);
  const int label_width = std::max(replanning_label->sizeHint().width(), saved_routes_label->sizeHint().width());
  saved_routes_label->setMinimumWidth(label_width);
  mode_row->addWidget(waypoint_options);
  layout->addWidget(parameters);

  auto* action_buttons = new QHBoxLayout;
  plan_button_ = new QPushButton(tr("Plan Route"), content);
  cancel_button_ = new QPushButton(tr("Cancel Route"), content);
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
  progress_->hide();
  layout->addWidget(status_);
  layout->addWidget(detail_);
  layout->addWidget(progress_);

  auto* settings_toggle = new QToolButton(content);
  settings_toggle->setText(tr("Settings"));
  settings_toggle->setCheckable(true);
  settings_toggle->setArrowType(Qt::RightArrow);
  settings_toggle->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  settings_toggle->setAutoRaise(true);
  layout->addWidget(settings_toggle, 0, Qt::AlignLeft);
  auto* settings = new QWidget(content);
  auto* settings_form = new QFormLayout(settings);
  settings_form->setContentsMargins(0, 0, 0, 0);
  map_server_name_ = new QLineEdit("ll2_map_server", settings);
  client_name_ = new QLineEdit("/plan_route_action_client", settings);
  goal_topic_ = new QLineEdit("/plan_route_action_client/goal_pose", settings);
  action_name_ = new QLineEdit(kActionName, settings);
  map_server_name_->setToolTip(tr("Client startup parameter; changing this field does not reconfigure a running client."));
  settings_form->addRow(tr("LL2 Map Server Name"), map_server_name_);
  settings_form->addRow(tr("Action Client Node"), client_name_);
  settings_form->addRow(tr("Goal Pose Topic"), goal_topic_);
  settings_form->addRow(tr("Status Action"), action_name_);
  settings->hide();
  layout->addWidget(settings);
  connect(settings_toggle, &QToolButton::toggled, this, [settings_toggle, settings](bool open) {
    settings_toggle->setArrowType(open ? Qt::DownArrow : Qt::RightArrow);
    settings->setVisible(open);
  });
  layout->addStretch();

  auto update_mode = [this, waypoint_options, replanning_label, saved_routes_label] {
    const bool waypoints = mode_->currentIndex() == kWaypointsMode;
    plan_button_->setText(mode_->currentIndex() == kDestinationMode ? tr("Set Destination") : tr("Plan Route"));
    waypoint_options->setVisible(mode_->currentIndex() != kDestinationMode);
    saved_routes_label->setVisible(waypoints);
    presets_->setVisible(waypoints);
    const bool show_replanning = waypoints && continuous_planning_->isChecked();
    replanning_label->setVisible(show_replanning);
    replanning_proportion_->setVisible(show_replanning);
    if (mode_->currentIndex() == kDestinationMode) progress_->hide();
    updatePlanButton();
  };
  connect(mode_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this, update_mode](int mode) {
    if (mode != kDestinationMode && selecting_destination_) {
      selecting_destination_ = false;
      goal_pose_sub_.reset();
      pending_destination_status_.reset();
      pending_destination_goal_id_.clear();
      auto* manager = getDisplayContext()->getToolManager();
      if (manager && manager->getCurrentTool() == goal_tool_) manager->setCurrentTool(manager->getDefaultTool());
    }
    update_mode();
  });
  connect(continuous_planning_, &QCheckBox::toggled, this, [update_mode](bool) { update_mode(); });
  connect(plan_button_, &QPushButton::clicked, this, [this] {
    if (mode_->currentIndex() != kDestinationMode) {
      planRoute();
      return;
    }
    auto* manager = getDisplayContext()->getToolManager();
    if (!manager) {
      showStatus(tr("Failed"), tr("RViz tool manager is unavailable."));
      return;
    }
    auto* goal_tool = ensureGoalTool();
    if (!goal_tool) {
      showStatus(tr("Failed"), tr("RViz Goal Pose tool could not be loaded."));
      return;
    }
    const auto topic = goal_topic_->text().trimmed().toStdString();
    if (topic.empty()) {
      showStatus(tr("Failed"), tr("Enter a goal pose topic in Settings."));
      return;
    }
    goal_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic, rclcpp::QoS(10), [weak_bridge = std::weak_ptr<CallbackBridge>(callback_bridge_)](
                                  geometry_msgs::msg::PoseStamped::ConstSharedPtr) {
          postToPanel(weak_bridge, [](PlanRoutePanel* panel) {
            if (!panel->selecting_destination_) return;
            panel->selecting_destination_ = false;
            panel->goal_pose_sub_.reset();
            panel->awaiting_goal_ = true;
            panel->terminal_status_received_ = false;
            panel->cancel_requested_ = false;
            panel->continuous_run_ = false;
            panel->remaining_goals_ = 1;
            panel->tracked_goal_id_.clear();
            panel->result_goal_id_.clear();
            panel->failed_destination_goal_id_.clear();
            // Reuse a new goal observed while the pose callback was waiting for Qt.
            if (!panel->pending_destination_goal_id_.empty()) {
              panel->tracked_goal_id_ = std::move(panel->pending_destination_goal_id_);
              panel->awaiting_goal_ = false;
              panel->requestResult(panel->tracked_goal_id_);
              panel->showStatus(QObject::tr("Running"), QObject::tr("Route goal received."));
            }
            if (panel->pending_destination_status_) {
              const auto pending = std::move(*panel->pending_destination_status_);
              panel->pending_destination_status_.reset();
              if (!panel->tracked_goal_id_.empty()) panel->updateGoalStatus(pending);
            }
            if (panel->awaiting_goal_ && panel->tracked_goal_id_.empty()) {
              panel->progress_->setRange(0, 0);
              panel->showStatus(QObject::tr("Sending"), QObject::tr("Destination sent; waiting for route goal."));
            }
          });
        });
    pending_destination_status_.reset();
    pending_destination_goal_id_.clear();
    selecting_destination_ = true;
    manager->setCurrentTool(goal_tool);
    showStatus(tr("Selecting"), tr("Set a goal pose in RViz."));
  });
  connect(cancel_button_, &QPushButton::clicked, this, [this] { cancelRoute(); });
  connect(client_name_, &QLineEdit::editingFinished, this, [this] {
    connectToClient();
    refreshParameters();
    Q_EMIT configChanged();
  });
  connect(goal_topic_, &QLineEdit::editingFinished, this, [this] {
    updateGoalTopic();
    Q_EMIT configChanged();
  });
  connect(action_name_, &QLineEdit::editingFinished, this, [this] {
    connectToAction();
    Q_EMIT configChanged();
  });
  connect(map_server_name_, &QLineEdit::editingFinished, this, [this] { Q_EMIT configChanged(); });
  loadPresets();
  update_mode();
}

PlanRoutePanel::~PlanRoutePanel() {
  std::lock_guard<std::mutex> lock(callback_bridge_->mutex);
  callback_bridge_->panel = nullptr;
}

void PlanRoutePanel::load(const rviz_common::Config& config) {
  rviz_common::Panel::load(config);
  const std::pair<const char*, QLineEdit*> fields[] = {
      {"LL2 Map Server Name", map_server_name_}, {"Action Client Node", client_name_},
      {"Goal Pose Topic", goal_topic_}, {"Status Action", action_name_}};
  for (const auto& [key, field] : fields) {
    QString value;
    if (config.mapGetString(key, &value)) field->setText(value);
  }
  if (node_) {
    updateGoalTopic();
    connectToAction();
    connectToClient();
    refreshParameters();
  }
}

void PlanRoutePanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  const std::pair<const char*, QLineEdit*> fields[] = {
      {"LL2 Map Server Name", map_server_name_}, {"Action Client Node", client_name_},
      {"Goal Pose Topic", goal_topic_}, {"Status Action", action_name_}};
  for (const auto& [key, field] : fields) config.mapSetValue(key, field->text());
}

void PlanRoutePanel::onInitialize() {
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    showStatus(tr("Failed"), tr("RViz ROS node is unavailable."));
    return;
  }
  node_ = abstraction->get_raw_node();
  ensureGoalTool();
  connectToAction();
  connectToClient();
  refreshParameters();
  auto* retry_timer = new QTimer(this);
  connect(retry_timer, &QTimer::timeout, this, [this] {
    if (!get_client_) return;
    if (!get_client_->service_is_ready()) {
      client_ready_ = false;
    } else if (!client_ready_) {
      refreshParameters();
    }
  });
  retry_timer->start(1000);
  updatePlanButton();
  cancel_button_->setEnabled(true);
}

rviz_common::Tool* PlanRoutePanel::ensureGoalTool() {
  if (goal_tool_) {
    updateGoalTopic();
    return goal_tool_.data();
  }
  auto* manager = getDisplayContext()->getToolManager();
  if (!manager) return nullptr;
  for (int index = 0; index < manager->numTools(); ++index) {
    auto* candidate = manager->getTool(index);
    if (candidate->getClassId() == "rviz_default_plugins/SetGoal") {
      goal_tool_ = candidate;
      break;
    }
  }
  if (!goal_tool_) {
    goal_tool_ = manager->addTool("rviz_default_plugins/SetGoal");
  }
  updateGoalTopic();
  return goal_tool_.data();
}

void PlanRoutePanel::updateGoalTopic() {
  if (!goal_tool_) return;
  auto* topic = goal_tool_->getPropertyContainer()->subProp("Topic");
  if (topic && !goal_topic_->text().trimmed().isEmpty()) topic->setValue(goal_topic_->text().trimmed());
}

void PlanRoutePanel::connectToAction() {
  if (!node_) return;
  const auto action = action_name_->text().trimmed().toStdString();
  if (action == connected_action_name_ && status_sub_ && feedback_sub_ && result_client_) return;
  status_sub_.reset();
  feedback_sub_.reset();
  result_client_.reset();
  connected_action_name_.clear();
  tracked_goal_id_.clear();
  result_goal_id_.clear();
  awaiting_goal_ = false;
  terminal_status_received_ = false;
  if (action.empty()) return;
  const std::weak_ptr<CallbackBridge> weak_bridge = callback_bridge_;
  status_sub_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      action + "/_action/status", rclcpp::QoS(10).reliable().transient_local(),
      [weak_bridge, action](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
        postToPanel(weak_bridge, [msg, action](PlanRoutePanel* panel) {
          if (panel->connected_action_name_ == action) panel->updateGoalStatus(*msg);
        });
      });
  feedback_sub_ = node_->create_subscription<route_planning_msgs::action::PlanRoute::Impl::FeedbackMessage>(
      action + "/_action/feedback", rclcpp::QoS(10),
      [weak_bridge, action](route_planning_msgs::action::PlanRoute::Impl::FeedbackMessage::ConstSharedPtr msg) {
        const auto id = goalId(msg->goal_id.uuid);
        const auto feedback = msg->feedback;
        postToPanel(weak_bridge, [id, feedback, action](PlanRoutePanel* panel) {
          if (panel->connected_action_name_ == action) panel->updateFeedback(id, feedback);
        });
      });
  result_client_ = node_->create_client<route_planning_msgs::action::PlanRoute::Impl::GetResultService>(
      action + "/_action/get_result");
  connected_action_name_ = action;
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
    request_client_.reset();
    extended_request_client_.reset();
    cancel_client_.reset();
    ++request_generation_;
    service_request_pending_ = false;
    cancel_after_request_ = false;
    updatePlanButton();
    connected_client_name_.clear();
    client_ready_ = false;
    showStatus(tr("Failed"), tr("Enter the action client node name."));
    return;
  }
  if (name == connected_client_name_ && get_client_ && request_client_ && extended_request_client_ && cancel_client_) return;
  ++request_generation_;
  service_request_pending_ = false;
  cancel_after_request_ = false;
  updatePlanButton();
  client_ready_ = false;
  get_client_ = node_->create_client<rcl_interfaces::srv::GetParameters>(name + "/get_parameters");
  request_client_ = node_->create_client<route_planning_msgs::srv::RequestRoute>(name + "/request_route");
  extended_request_client_ = node_->create_client<route_planning_msgs::srv::ExtendedRequestRoute>(name + "/extended_request_route");
  cancel_client_ = node_->create_client<route_planning_msgs::srv::CancelRoute>(name + "/cancel_route");
  connected_client_name_ = name;
}

void PlanRoutePanel::refreshParameters() {
  if (!get_client_ || !get_client_->service_is_ready()) {
    client_ready_ = false;
    showStatus(tr("Idle"), tr("Action client parameter service is not available."));
    return;
  }
  client_ready_ = true;
  const auto requested_client = clientName();
  const auto generation = request_generation_;
  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = {"enable_random_destination", "enable_continuous_planning",
                    "continuous_planning_replanning_proportion"};
  const std::weak_ptr<CallbackBridge> weak_bridge = callback_bridge_;
  get_client_->async_send_request(
      request, [weak_bridge, requested_client, generation](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) {
    try {
      const auto values = future.get()->values;
      postToPanel(weak_bridge, [values, requested_client, generation](PlanRoutePanel* panel) {
        // Ignore replies from a client that was selected before the request completed.
        if (panel->clientName() != requested_client || panel->request_generation_ != generation || values.size() != 3) return;
        const bool random = values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL
                                ? values[0].bool_value
                                : panel->mode_->currentIndex() == kRandomMode;
        if (values[1].type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
          panel->continuous_planning_->setChecked(values[1].bool_value);
        }
        if (panel->mode_->currentIndex() != kDestinationMode) {
          panel->mode_->setCurrentIndex(random ? kRandomMode : kWaypointsMode);
        }
        if (values[2].type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
          panel->replanning_proportion_->setValue(values[2].double_value);
        }
        if (!panel->awaiting_goal_ && panel->tracked_goal_id_.empty()) {
          panel->showStatus(QObject::tr("Idle"), QObject::tr("Loaded parameters from action client."));
        }
      });
    } catch (const std::exception& e) {
      const QString error = QString::fromUtf8(e.what());
      postToPanel(weak_bridge, [error, requested_client, generation](PlanRoutePanel* panel) {
        if (panel->clientName() != requested_client || panel->request_generation_ != generation) return;
        panel->client_ready_ = false;
        panel->showStatus(QObject::tr("Failed"), error);
      });
    }
  });
}

template <typename Service>
void PlanRoutePanel::sendRequest(const typename rclcpp::Client<Service>::SharedPtr& client,
                                  const std::shared_ptr<typename Service::Request>& request,
                                  std::function<void(bool, const QString&)> done) {
  if (!client || !client->service_is_ready()) {
    done(false, tr("Route service is unavailable."));
    return;
  }
  const auto generation = request_generation_;
  const std::weak_ptr<CallbackBridge> weak_bridge = callback_bridge_;
  // This flag is accessed only on the Qt thread, including queued ROS replies.
  const auto completed = std::make_shared<bool>(false);
  try {
    client->async_send_request(
        request, [weak_bridge, generation, completed, done](typename rclcpp::Client<Service>::SharedFuture future) {
      bool accepted = false;
      QString message;
      try {
        const auto response = future.get();
        accepted = response->accepted;
        message = QString::fromStdString(response->message);
      } catch (const std::exception& e) {
        message = QString::fromUtf8(e.what());
      }
      postToPanel(weak_bridge, [generation, completed, done, accepted, message](PlanRoutePanel* panel) {
        if (*completed) return;
        *completed = true;
        if (panel->request_generation_ == generation) done(accepted, message);
      });
    });
    QTimer::singleShot(10000, this, [this, generation, completed] {
      if (*completed || request_generation_ != generation || terminal_status_received_) return;
      // Keep the response callback: a timeout does not withdraw the remote request.
      showStatus(tr("Waiting"), cancel_after_request_
          ? tr("Route service response timed out. Cancellation will be sent after the pending request completes.")
          : tr("Route service response timed out; the request is still pending. "
               "New requests wait for its response to preserve their order."));
    });
  } catch (const std::exception& e) {
    *completed = true;
    done(false, QString::fromUtf8(e.what()));
  }
}

void PlanRoutePanel::planRoute() {
  connectToClient();
  if (service_request_pending_) return;
  const bool random = mode_->currentIndex() == kRandomMode;
  const bool continuous = continuous_planning_->isChecked();
  if (!random && presets_->currentIndex() < 0) {
    showStatus(tr("Failed"), tr("Select a saved route."));
    return;
  }
  std::vector<Waypoint> route;
  if (!random) {
    try {
      route = readRoute(presets_->currentText().toStdString());
    } catch (const std::exception& e) {
      showStatus(tr("Failed"), QString::fromUtf8(e.what()));
      return;
    }
  }
  route_planning_msgs::msg::RouteRequest route_request;
  remaining_goals_ = random ? 1 : 0;
  for (const auto& waypoint : route) {
    geographic_msgs::msg::GeoPoint point;
    point.latitude = waypoint.latitude;
    point.longitude = waypoint.longitude;
    route_request.waypoints.push_back(point);
    route_request.wait_times.push_back(static_cast<float>(waypoint.wait_time_s));
    if (waypoint.wait_time_s >= 0.0) ++remaining_goals_;
  }
  ++request_generation_;
  pending_destination_status_.reset();
  pending_destination_goal_id_.clear();
  awaiting_goal_ = true;
  terminal_status_received_ = false;
  cancel_requested_ = false;
  continuous_run_ = continuous;
  tracked_goal_id_.clear();
  result_goal_id_.clear();
  failed_destination_goal_id_.clear();
  showStatus(tr("Sending"), tr("Requesting a route from the action client; waiting for its goal."));
  progress_->setRange(0, 0);
  service_request_pending_ = true;
  updatePlanButton();
  auto done = [this](bool success, const QString& reason) {
    service_request_pending_ = false;
    if (cancel_after_request_) {
      cancel_after_request_ = false;
      cancelRoute();
      return;
    }
    updatePlanButton();
    if (!success) {
      awaiting_goal_ = false;
      continuous_run_ = false;
      remaining_goals_ = 0;
      tracked_goal_id_.clear();
      result_goal_id_.clear();
      progress_->setRange(0, 1);
      progress_->setValue(0);
      showStatus(tr("Failed"), reason.isEmpty() ? tr("Route request rejected.") : reason);
      return;
    }
    const auto generation = request_generation_;
    QTimer::singleShot(10000, this, [this, generation] {
      if (request_generation_ != generation || !awaiting_goal_ || !result_goal_id_.empty()) return;
      // Keep observing: a slow planner may still accept the goal after this message.
      showStatus(tr("Waiting"),
                 tr("Route request accepted, but no new action goal has been observed. "
                    "Check the planner logs for rejected goals and verify Status Action in Settings. "
                    "You can retry or cancel the route."));
    });
  };
  if (!random && !continuous) {
    auto request = std::make_shared<route_planning_msgs::srv::RequestRoute::Request>();
    request->route_request = std::move(route_request);
    sendRequest<route_planning_msgs::srv::RequestRoute>(request_client_, request, std::move(done));
  } else {
    using Service = route_planning_msgs::srv::ExtendedRequestRoute;
    auto request = std::make_shared<Service::Request>();
    request->mode = random ? (continuous ? Service::Request::MODE_RANDOM_CONTINUOUS : Service::Request::MODE_RANDOM)
                           : Service::Request::MODE_LOOP;
    request->route_request = std::move(route_request);
    if (!random) request->replanning_proportion = static_cast<float>(replanning_proportion_->value());
    sendRequest<Service>(extended_request_client_, request, std::move(done));
  }
}

void PlanRoutePanel::cancelRoute() {
  const bool was_selecting = selecting_destination_;
  if (was_selecting) {
    selecting_destination_ = false;
    goal_pose_sub_.reset();
    pending_destination_status_.reset();
    pending_destination_goal_id_.clear();
    auto* manager = getDisplayContext()->getToolManager();
    if (manager && manager->getCurrentTool() == goal_tool_) manager->setCurrentTool(manager->getDefaultTool());
  }
  connectToClient();
  cancel_requested_ = true;
  awaiting_goal_ = false;
  continuous_run_ = false;
  remaining_goals_ = 0;
  if (service_request_pending_) {
    // Keep the route response valid and send cancellation only after it arrives.
    cancel_after_request_ = true;
    if (!terminal_status_received_) {
      showStatus(tr("Canceling"), tr("Waiting for the pending service request before sending cancellation."));
    }
    return;
  }
  ++request_generation_;
  service_request_pending_ = true;
  updatePlanButton();
  auto request = std::make_shared<route_planning_msgs::srv::CancelRoute::Request>();
  sendRequest<route_planning_msgs::srv::CancelRoute>(cancel_client_, request,
      [this, was_selecting](bool success, const QString& reason) {
        service_request_pending_ = false;
        if (cancel_after_request_) {
          cancel_after_request_ = false;
          cancelRoute();
          return;
        }
        updatePlanButton();
        if (!success) {
          cancel_requested_ = false;
          showStatus(tr("Failed"), reason.isEmpty() ? tr("Cancel request rejected.") : reason);
          return;
        }
        if (mode_->currentIndex() == kRandomMode) mode_->setCurrentIndex(kWaypointsMode);
        continuous_planning_->setChecked(false);
        // A terminal status/result may have arrived before this service response.
        if (terminal_status_received_) return;
        showStatus(tracked_goal_id_.empty() ? (was_selecting ? tr("Canceled") : tr("Cancel requested")) : tr("Canceling"),
                   was_selecting && tracked_goal_id_.empty() ? tr("Destination selection canceled.")
                       : tr("Cancel requested from the action client."));
      });
}

void PlanRoutePanel::loadPresets() {
  presets_->clear();
  try {
    const auto root = YAML::LoadFile(routesFilePath());
    const auto routes = root["routes"];
    if (!routes || !routes.IsMap()) throw std::runtime_error("'routes' must be a map");
    for (const auto& route : routes) presets_->addItem(QString::fromStdString(route.first.as<std::string>()));
    if (presets_->count() == 0) showStatus(tr("Failed"), tr("No saved routes configured."));
  } catch (const std::exception& e) {
    showStatus(tr("Failed"), QString::fromUtf8(e.what()));
  }
}

void PlanRoutePanel::updatePlanButton() {
  plan_button_->setEnabled(node_ && !service_request_pending_ &&
                           (mode_->currentIndex() != kWaypointsMode || presets_->count() > 0));
}

void PlanRoutePanel::showStatus(const QString& status, const QString& detail) {
  status_->setText(tr("Status: %1").arg(status));
  detail_->setText(detail);
  progress_->setVisible(mode_->currentIndex() != kDestinationMode &&
                        (status == tr("Sending") || status == tr("Running") ||
                         (status == tr("Canceling") && !tracked_goal_id_.empty())));
}

bool PlanRoutePanel::rememberGoal(const std::string& goal_id) {
  if (!seen_goal_ids_.insert(goal_id).second) return false;
  seen_goal_order_.push_back(goal_id);
  // Action status can retain old goals; keep only a bounded history for deduplication.
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
  const auto action = connected_action_name_;
  result_client_->async_send_request(
      request, [weak_bridge, goal_id, action](
                   rclcpp::Client<route_planning_msgs::action::PlanRoute::Impl::GetResultService>::SharedFuture future) {
    try {
      const auto response = future.get();
      postToPanel(weak_bridge, [goal_id, response, action](PlanRoutePanel* panel) {
        if (panel->connected_action_name_ != action || panel->result_goal_id_ != goal_id) return;
        if (response->status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED ||
            response->status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
            response->status == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
          panel->terminal_status_received_ = true;
        }
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
  if (selecting_destination_) pending_destination_status_ = msg;
  for (const auto& goal : msg.status_list) {
    const auto id = goalId(goal.goal_info.goal_id.uuid);
    const bool is_new = rememberGoal(id);
    if (selecting_destination_ && is_new && goal.status != action_msgs::msg::GoalStatus::STATUS_UNKNOWN) {
      pending_destination_goal_id_ = id;
    }
    // Action status has no sender ID, so a new goal is only a best-effort match.
    if (awaiting_goal_ && tracked_goal_id_.empty() && is_new &&
        goal.status != action_msgs::msg::GoalStatus::STATUS_UNKNOWN) {
      tracked_goal_id_ = id;
      terminal_status_received_ = false;
      awaiting_goal_ = false;
      requestResult(id);
    }
    if (id != tracked_goal_id_) continue;
    switch (goal.status) {
      case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
      case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
        if (!cancel_requested_ && !terminal_status_received_) showStatus(tr("Running"), detail_->text());
        break;
      case action_msgs::msg::GoalStatus::STATUS_CANCELING:
        if (!terminal_status_received_) showStatus(tr("Canceling"), detail_->text());
        break;
      case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
        terminal_status_received_ = true;
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
        terminal_status_received_ = true;
        if (remaining_goals_ > 0) --remaining_goals_;
        awaiting_goal_ = continuous_run_ || remaining_goals_ > 0;
        progress_->setRange(0, 1);
        progress_->setValue(0);
        showStatus(tr("Aborted"), detail_->text());
        tracked_goal_id_.clear();
        break;
      case action_msgs::msg::GoalStatus::STATUS_CANCELED:
        terminal_status_received_ = true;
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
  if (selecting_destination_ && !seen_goal_ids_.count(goal_id)) pending_destination_goal_id_ = goal_id;
  // Feedback may arrive before the corresponding status update.
  if (tracked_goal_id_.empty() && awaiting_goal_ && !seen_goal_ids_.count(goal_id)) {
    tracked_goal_id_ = goal_id;
    terminal_status_received_ = false;
    awaiting_goal_ = false;
    rememberGoal(goal_id);
    requestResult(goal_id);
  }
  if (tracked_goal_id_ != goal_id || terminal_status_received_) return;
  const double remaining_s = feedback.time_remaining.sec + feedback.time_remaining.nanosec * 1e-9;
  showStatus(cancel_requested_ ? tr("Canceling") : tr("Running"),
             tr("Distance: %1 m traveled, %2 m remaining · Time remaining: %3 s")
                 .arg(feedback.distance_traveled, 0, 'f', 1)
                 .arg(feedback.distance_remaining, 0, 'f', 1)
                 .arg(remaining_s, 0, 'f', 1));
}

}  // namespace plan_route_panel

PLUGINLIB_EXPORT_CLASS(plan_route_panel::PlanRoutePanel, rviz_common::Panel)
